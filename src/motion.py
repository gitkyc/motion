import time
from picamera import PiCamera
import numpy as np
import json
import os
from flask import Flask, render_template, request, jsonify
import threading
import subprocess
from PIL import Image
from PyV4L2Camera.camera import Camera
from PyV4L2Camera.controls import ControlIDs

width = 64 * 2
height = 48 * 2
ratio = 8
sthres = 1
thres = ratio * ratio * 255 / 30
size = width * height
motion_config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'motion_config')
lux_config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'lux_config')
image_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'static/ss.jpg')
print('motion config path: {}, image path: {}'.format(motion_config_path, image_path))
print('lux config path: {}, image path: {}'.format(lux_config_path, image_path))
motion_targets = np.loadtxt(motion_config_path, dtype=int)
lux_targets = np.loadtxt(lux_config_path, dtype=int)
motion_rects = np.divide(motion_targets, ratio * 320 / width).astype(int)
lux_rects = np.divide(lux_targets, ratio * 320 / width).astype(int)

snapping = False

# camera
camera=Camera('/dev/video0', 320, 240)
# controls=camera.get_controls()
# Camera warm-up time
time.sleep(2)

# flask
app = Flask(__name__)


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

def set_motion_config(config):
    if len(config) == 0:
        return
    with open(motion_config_path, 'w', encoding='utf-8') as file:
        for line in config:
            file.write(' '.join(str(x) for x in line) + '\n')
    global motion_targets
    motion_targets = np.loadtxt(motion_config_path, dtype=int)

def set_lux_config(config):
    if len(config) == 0:
        return
    with open(lux_config_path, 'w', encoding='utf-8') as file:
        for line in config:
            file.write(' '.join(str(x) for x in line) + '\n')
    global lux_targets
    lux_targets = np.loadtxt(lux_config_path, dtype=int)

def get_motion_config():
    return jsonify(motion_targets.tolist())

def get_lux_config():
    return jsonify(lux_targets.tolist())

def start_app():
    # run app
    app.run(host='0.0.0.0')


def main():
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
        print(type(data))
        try:
            frame=camera.get_frame()
            img=Image.frombytes('RGB', (camera.width, camera.height), frame, 'raw', 'RGB').resize((width,height))
            img_yuv=img.convert('YCbCr')
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
            time.sleep(0.2)
        except KeyboardInterrupt:
            break
    print("main loop finished")



def snap():
    global snapping
    snapping = True
    time.sleep(2)
    frame=camera.get_frame()
    img=Image.frombytes('RGB', (camera.width, camera.height), frame, 'raw', 'RGB').resize((320,240))
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
        res[i] = int(np.average(submatrix))

    with open('/home/pi/motion/fifo', 'w', encoding='utf-8') as file:
        out = json.dumps({'light': res}) + '\n'
        print('write to serial: {}'.format(out))
        file.write(out)


def judge_motion(last, current):
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
        print(submatrix)
        if np.sum(submatrix) > 0:
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
