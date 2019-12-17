import time
from picamera import PiCamera
import numpy as np
import json
import os
from flask import Flask, render_template, request, jsonify
import threading
import subprocess
from PIL import Image,ImageFilter
from PyV4L2Camera.camera import Camera
from PyV4L2Camera.controls import ControlIDs

width = 64 * 2
height = 48 * 2
ratio = 8
sthres = 1
thres = ratio * ratio * 255 / 30
size = width * height

motion_area_config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'motion_area_config')
motion_threshold_config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'motion_threshold_config')
lux_area_config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'lux_area_config')
gauss_filter_config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'gauss_filter_config')
image_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'static/ss.jpg')
print('motion area config path: {}'.format(motion_area_config_path))
print('motion threshold config path: {}'.format(motion_threshold_config_path))
print('lux area config path: {}'.format(lux_area_config_path))
print('gauss filter config path: {}'.format(gauss_filter_config_path))
print('image path: {}'.format(image_path))

motion_targets = np.empty(shape=(4,4), dtype=int)
motion_thresholds = np.empty(shape=4)
lux_targets = np.empty(shape=(4,4))
motion_rects = np.empty(shape=(4,4))
lux_rects = np.empty(shape=(4,4))

gauss_radius=1

snapping = False
gaussSnapping = False

# camera
camera=Camera('/dev/video0', 320, 240)
# controls=camera.get_controls()
# Camera warm-up time
time.sleep(2)

last_trigger_times=[0,0,0,0]

# flask
app = Flask(__name__)

def loadMotionConfigData():
    global motion_targets
    global motion_thresholds

    try:
        motion_targets = np.loadtxt(motion_area_config_path, dtype=int)
    except:
        motion_targets = np.array([[0,0,320,240],[0,0,1,1],[0,0,1,1],[0,0,1,1]])

    try:
        motion_thresholds = np.loadtxt(motion_threshold_config_path, dtype=int)
    except:
        motion_thresholds = np.array([1,1,1,1])

def loadLuxConfigData():
    global lux_targets
    try:
        lux_targets = np.loadtxt(lux_area_config_path, dtype=int)
    except:
        lux_targets = np.array([[0,0,320,240],[0,0,1,1],[0,0,1,1],[0,0,1,1]])

def loadGaussFilterConfigData():
    global gauss_radius
    try:
        file=open(gauss_filter_config_path)
        gauss_radius=int(file.readline())
        if(gauss_radius<=0):
            gauss_radius=5
        if((gauss_radius%2)==0):
            gauss_radius=5;
        file.close()
    except:
        gauss_radius=5

def dataInit():
    global motion_rects
    global lux_rects
    loadMotionConfigData()
    loadLuxConfigData()
    loadGaussFilterConfigData()
    motion_rects = np.divide(motion_targets, ratio * 320 / width).astype(int)
    lux_rects = np.divide(lux_targets, ratio * 320 / width).astype(int)

@app.route('/')
def index():
    return render_template('index.html')


@app.route('/snap')
def post_snap():
    snap()
    return ''

@app.route('/reboot')
def reboot():
    subprocess.call(['sudo', 'reboot'])
    return ''

@app.route('/restart_app')
def restart_app():
    subprocess.call(['/home/pi/motion/bin/stop_all.sh'])
    return ''


@app.route('/motion_config', methods=['GET', 'POST'])
def config_motion():
    if request.method == 'POST':
        set_motion_config(request.json)
        return ''
    else:
        return get_motion_config()

@app.route('/lux_config', methods=['GET', 'POST'])
def config_lux():
    if request.method == 'POST':
        set_lux_config(request.json)
        return ''
    else:
        return get_lux_config()

@app.route('/gauss_filter_config', methods=['GET', 'POST'])
def config_gauss_filter():
    if request.method == 'POST':
        set_gauss_filter_config(request.json)
        return ''
    else:
        return get_gauss_filter_config()

def set_motion_config(config):
    if len(config['areas']) == 0:
        return
    if len(config['thresholds']) == 0:
        return
    with open(motion_area_config_path, 'w+', encoding='utf-8') as file:
        for line in config['areas']:
            file.write(' '.join(str(x) for x in line) + '\n')
    with open(motion_threshold_config_path, 'w+', encoding='utf-8') as file:
        file.write(' '.join(str(x) for x in config['thresholds']))
    loadMotionConfigData()

def set_lux_config(config):
    if len(config) == 0:
        return
    with open(lux_area_config_path, 'w', encoding='utf-8') as file:
        for line in config:
            file.write(' '.join(str(x) for x in line) + '\n')
    loadLuxConfigData()

def set_gauss_filter_config(config):
    with open(gauss_filter_config_path, 'w', encoding='utf-8') as file:
        file.write(str(config['radius'])) 
    loadGaussFilterConfigData()

def get_motion_config():
    return jsonify({'areas': motion_targets.tolist(), 'thresholds': motion_thresholds.tolist()})

def get_lux_config():
    return jsonify(lux_targets.tolist())

def get_gauss_filter_config():
    return jsonify({'radius': gauss_radius})

def start_app():
    # run app
    app.run(host='0.0.0.0')


def main():
    dataInit()

    t = threading.Thread(name='web', target=start_app)
    t.daemon = True
    t.start()

    # Create an in-memory stream
    last_data = None
    # count = 0
    while True:
        if snapping:
            time.sleep(1)
            continue
        data = np.empty((width, height), dtype=np.uint8)
        try:
            frame=camera.get_frame()
            img=Image.frombytes('RGB', (camera.width, camera.height), frame, 'raw', 'RGB')
            img=img.filter(ImageFilter.GaussianBlur(radius=gauss_radius))
            img_yuv=img.convert('L')
            data=np.array(img_yuv)
        except IOError as e:
            print(str(e))
            pass
        # for i in range(height):
        #     print(' '.join('{:3d}'.format(x) for x in data[i * width: (i+1) * width]))
        # print('\n\n')

        if last_data is not None:
            judge_motion(last_data, data)
        judge_light(data)
        last_data = data

        # count = count + 1
        # if count % 10 == 0:
        #     snap()

        try:
            time.sleep(0.1)
        except KeyboardInterrupt:
            break
    print("main loop finished")



def snap():
    global snapping
    global gaussSnapping
    snapping = True
    time.sleep(2)
    frame=camera.get_frame()
    img=Image.frombytes('RGB', (camera.width, camera.height), frame, 'raw', 'RGB')
    if gaussSnapping:
        img=img.filter(ImageFilter.GaussianBlur(radius=gauss_radius))
        gaussSnapping=False
    else:
        gaussSnapping=True
    img.save(image_path)
    snapping = False


def judge_light(current):

    res = {}
    ih = int(height / ratio)
    iw = int(width / ratio)
    current = np.sum(current.reshape(ih, ratio, iw, -1), axis=(1,3))
    current = np.divide(current, ratio * ratio).astype(int)
    for i in range(len(lux_rects)):
        rect = lux_rects[i]
        submatrix = current[rect[1]:rect[3], rect[0]:rect[2]]
        try:
            res[i] = int(np.average(submatrix))
        except:
            res[i]=0

    with open('/home/pi/motion/fifo', 'w', encoding='utf-8') as file:
        out = json.dumps({'light': res}) + '\n'
        print('write to serial: {}'.format(out))
        file.write(out)


def judge_motion(last, current):
    global motion_thresholds
    # if len(last) != size or len(current) != size:
    #     print('invalid input for judge motion')
    #     return
    start = time.clock()

    # 差值
    ih = int(height / ratio)
    iw = int(width / ratio)
    idiff = np.abs(np.subtract(last.astype(int), current.astype(int)))
    idiff[idiff < sthres] = 0
    diff = np.sum(idiff.reshape(ih, ratio, iw, -1), axis=(1,3))
    # 二值化
    # print(diff)
    diff[diff < thres] = 0
    diff[diff >= thres] = 1
    print(diff)

    res = []
    for i in range(len(motion_rects)):
        rect = motion_rects[i]
        submatrix = diff[rect[1]:rect[3], rect[0]:rect[2]]
        # print(submatrix)
        current_trigger_time=time.perf_counter()
        triggerSum=np.sum(submatrix)
        print("TRIGGER SUM = "+str(triggerSum)+", TRIGGER THRES SUM = "+str(motion_thresholds[i]))
        if (triggerSum > motion_thresholds[i]) and (current_trigger_time-last_trigger_times[i]>=3):
            last_trigger_times[i]=current_trigger_time
            res.append(i)

    if len(res) > 0:
        with open('/home/pi/motion/fifo', 'w', encoding='utf-8') as file:
            out = json.dumps({'motion': res}) + '\n'
            print('write to serial: {}'.format(out))
            file.write(out)

    print('judge motion used: {}'.format(time.clock() - start))


if __name__ == '__main__':
    main()
    # judge_motion(np.ones(size), np.zeros(size))
