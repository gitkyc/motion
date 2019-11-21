import queue
import serial
import threading
import time
import json
import subprocess


input_path = '/home/pi/motion/fifo'
serial_dev = '/dev/ttyS0'
#input_path = '/tmp/fifo'
#serial_dev = '/dev/tty.usbserial-1410'

P_TURN_ON_AP = b'\xfa\x0a\x00\x00\x00\x00\x00\x00\x00\x00\xd1'
P_TURN_OFF_AP = b'\xfa\x0b\x00\x00\x00\x00\x00\x00\x00\x00\xa8'

HOSTAPD_STATUS_QUERY_MAX_INTERVAL_SEC = 10

hostapd_is_opened=False
hostapd_status_query_hold_sec = 0

def main():
    global hostapd_status_query_hold_sec
    s = serial.Serial(serial_dev, 115200)
    q = queue.Queue()
    t = threading.Thread(name='read fifo', target=read, args=[input_path, q])
    t.daemon = True
    t.start()
    t2 = threading.Thread(name='read serial', target=read_serial, args=[s])
    t2.daemon = True
    t2.start()
    t3 = threading.Thread(name='read hostapd status', target=read_hostapd_status)
    t3.daemon = True
    t3.start()
    while True:
        if q.empty():
            hostapd_status_query_hold_sec = hostapd_status_query_hold_sec+1
            if hostapd_status_query_hold_sec >= HOSTAPD_STATUS_QUERY_MAX_INTERVAL_SEC:
                # send hostapd status force when queue is empty for a long time
                send_hostapd_status_data(s)
                hostapd_status_query_hold_sec=0
            try:
                time.sleep(1)
                continue
            except KeyboardInterrupt:
                break
        data = q.get()
        send_data(data, s)
        hostapd_status_query_hold_sec=0

    print('main loop finished')

def read_hostapd_status():
    global hostapd_is_opened
    while True:
        out=subprocess.getstatusoutput("service hostapd status | grep 'Active:'");
        if len(out) < 2:
            hostapd_is_opened = False
            return;
        result=out[1]
        print(result)
        if "running" in result:
            hostapd_is_opened = True
        else:
            hostapd_is_opened = False
        time.sleep(3)

def read_serial(s):
    while True:
        rb = s.read(size=11)
        handle_serial_in(rb)


def handle_serial_in(proto):
    try:
        if proto == P_TURN_ON_AP:
            print("try to start ap")
            subprocess.call(['sudo', 'service', 'hostapd', 'start'])
        elif proto == P_TURN_OFF_AP:
            print("try to stop ap")
            subprocess.call(['sudo', 'service', 'hostapd', 'stop'])
    except Exception as e:
        print("execute service command failed. " + e.msg)


def send_data(data, s):
    # data like { "motion": [ 1, 2, 4], "light": [2, 6] }
    global hostapd_is_opened
    print("send data: {}".format(json.dumps(data)))
    motion = 0
    if "motion" in data:
        motion = array_to_bits(data["motion"])
    out = bytearray([0xfa, 0x01, motion, 0x0f, 0, 0, 0, 0, 0, 0])
    if "light" in data:
        for k, v in data['light'].items():
            out[4+int(k)] = (v&0xFF)
    if hostapd_is_opened:
        out[8]=0x0a
    else:
        out[8]=0x0b
    crc = crc_bytes(0x07, out)
    out.append(crc)
    print(', '.join('0x{:02x}'.format(x) for x in out))
    s.write(out)
    s.flush()

def send_hostapd_status_data(s):
    global hostapd_is_opened
    out = bytearray([0xfa, 0x01, 0, 0, 0, 0, 0, 0, 0, 0])
    if hostapd_is_opened:
        out[8]=0x0a
    else:
        out[8]=0x0b
    crc = crc_bytes(0x07, out)
    out.append(crc)
    print(', '.join('0x{:02x}'.format(x) for x in out))
    s.write(out)
    s.flush()

def array_to_bits(arr):
    out = 0
    for i in range(0, len(arr)):
        out = out | (0x01 << arr[i])
    return out


def crc_bytes(poly, byte_arr):
    crc = 0
    for i in range(0, len(byte_arr)):
        crc = crc_byte(poly, crc ^ byte_arr[i])
    return crc


def crc_byte(poly, data):
    crc = data
    for bit in range(0, 8):
        if crc & 0x80:
            crc = (crc << 1) ^ poly
        else:
            crc = (crc << 1)
    return crc & 0xff


def read(filename, q):
    print('start reading from fifo')
    last_time = time.perf_counter()
    data = {}
    with open(filename, 'r', encoding='utf-8') as ifile:
        while True:
            line = ifile.readline()
            if line == '':
                # print('warn: read empty input')
                time.sleep(0.01)
            else:
                print('read data: ' + line)
                try:
                    input = json.loads(line)
                    if "motion" in input:
                        if 'motion' in data:
                            data['motion'] = data['motion'] + input['motion']
                        else:
                            data['motion'] = input['motion']
                    if "light" in input:
                        data['light'] = input['light']
                except Exception as e:
                    print("can't parse data: {}".format(e))

            if time.perf_counter() - last_time > 0.5:
                if len(data) > 0 and q.qsize() < 10:
                    print('put data: ' + json.dumps(data))
                    q.put(data)
                last_time = time.perf_counter()
                data = {}


if __name__ == '__main__':
    main()
    # send_data('{ "motion": [ 1, 2, 4], "light": [2, 6] }', None)
    # print(hex(crc_bytes(0x07, [0xfa, 0x01, 0x0f, 0x0f, 0, 0, 0, 0, 0, 0])))

