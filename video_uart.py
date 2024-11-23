import cv2
import numpy as np
import serial
import time  # 导入time模块用于等待
import struct
import os

class PLAY_METHOD():
    ONCE = 1,
    LOOP = 2,
    LIST = 3


COM = 'COM4'
bauds = 3000000
# VIDEO_H = [32, 40, ]
# VIDEO_W = [32, 40, ]
# VIDEO_FILE = ['./down_up', './granny_spin', ]
# VIDEO_EX = ['gif', 'gif', ]
VIDEO_H = [67, 40, ]
VIDEO_W = [120, 40, ]
VIDEO_FILE = ['./video_material', './granny_spin', ]
VIDEO_EX = ['mp4', 'gif', ]
play_method = PLAY_METHOD.LIST
REP_TIME = 10

def read_serial(ser: serial.Serial, n = 2):
    raw_data = struct.unpack(f'<{n}B', ser.read(n))
    print('recv: ', raw_data)
    return raw_data

def send_frame(ser: serial.Serial, frame_data: np.ndarray):
    ser.write(frame_data.astype(np.uint16).tobytes())

def send_number(ser: serial.Serial, num, bts = 2):
    ser.write(num.to_bytes(bts, byteorder='little'))

def send_meta(ser: serial.Serial, H, W, f, t):
    send_number(ser, 0x10, 1)
    read_serial(ser, 1)
    send_number(ser, H, 2)
    send_number(ser, W, 2)
    send_number(ser, f, 2)
    send_number(ser, t, 2)

def get_frame_data(vf, vw, vh):
    saved_filename = f'{vf}_{vw}_{vh}.npy'

    if os.path.exists(saved_filename):
        all_frame_data = np.load(saved_filename)
    else:
        print('converting video...')
        all_frame_data = []
        frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        while True:
            ret, frame = cap.read()
            if frame_count % 200 == 0:
                print(f'{frame_count}     \r', end='')
                frame_count -= 1
            if not ret:
                break

            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            resized_frame = cv2.resize(frame_rgb, (vw, vh), interpolation=cv2.INTER_AREA)
            resized_frame = np.array(resized_frame, dtype=np.uint32)
            height, width = resized_frame.shape[:2]
            frame_16bit = np.zeros((height, width), dtype=np.uint16)
            r = (((resized_frame[:, :, 0] >> 3)) << 11).astype(np.uint16)
            g = (((resized_frame[:, :, 1] >> 2)) << 5).astype(np.uint16)
            b = ((resized_frame[:, :, 2] >> 3) & 0x1F).astype(np.uint16)
            frame_16bit = r | g | b

            flat_array = frame_16bit.flatten()
            all_frame_data.append(flat_array)

        all_frame_data = np.asarray(all_frame_data)
        np.save(saved_filename, all_frame_data)
        cap.release()
    return all_frame_data

if __name__ == '__main__':
    ser = serial.Serial(COM, bauds, 8, "N", 1, timeout=2, dsrdtr=False, rtscts=False)
    for vf, ve, vh, vw in zip(VIDEO_FILE, VIDEO_EX, VIDEO_H, VIDEO_W):
        video_path = f'{vf}.{ve}'
        print(video_path)
        cap = cv2.VideoCapture(video_path)

        if not cap.isOpened():
            print("Error: Could not open video.")
            exit()

        fps = cap.get(cv2.CAP_PROP_FPS)
        print(fps)

        all_frame_data = get_frame_data(vf, vw, vh)
        print("video prepare done")
        
        # print(all_frame_data[0])
        all_frame_list = []
        for i in range(len(all_frame_data)):
            all_frame_list.append(all_frame_data[i].astype(np.uint16).tobytes())
        
        rp = REP_TIME
        # time.sleep(0.1)
        send_meta(ser, vw, vh, int(fps), len(all_frame_data))
        raw = read_serial(ser, 1)
        # time.sleep(0.1)
        
        send_number(ser, 0x30, 1)
        raw = read_serial(ser, 1)
        is_stop = True if raw[0] != 0 else False
        while True:
            frame_index = 0
            frame_forward = 1
            frame_speed = 1
            frame_skip = 1.0
            while frame_index < len(all_frame_data):
                if is_stop:
                    frame_forward = 0
                else:
                    frame_forward = frame_speed * frame_skip

                send_number(ser, 0, 1)
                raw = read_serial(ser, 1)
                if raw[0] == 0x20:
                    is_stop = not is_stop
                if raw[0] == 0x21:
                    frame_speed = frame_speed * 2
                if raw[0] == 0x22:
                    frame_speed = frame_speed / 2
                if raw[0] == 0xff:
                    exit()
                if raw[0] != 0x00:
                    print(f'recv {raw[0]}')
                tt_start = time.time()
                # send_frame(ser, all_frame_data[int(frame_index)])
                ser.write(all_frame_list[int(frame_index)])
                send_number(ser, int(frame_index), 2)
                # time.sleep(0.05)
                print("send: ", time.time() - tt_start)
                raw = read_serial(ser, 1)
                
                if raw[0] == 0x10:
                    send_meta(ser, vw, vh, int(fps), len(all_frame_data))
                    raw = read_serial(ser, 1)
                if raw[0] == 0x20:
                    is_stop = not is_stop
                if raw[0] == 0x21:
                    frame_speed = frame_speed * 2
                if raw[0] == 0x22:
                    frame_speed = frame_speed / 2
                if raw[0] == 0xff:
                    exit()
                if raw[0] != 0x00:
                    print(f'recv {raw[0]}')

                if not is_stop:
                    tt_interval = time.time() - tt_start
                    true_interval = frame_forward/fps/frame_speed
                    if tt_interval < true_interval:
                        time.sleep(true_interval - tt_interval)
                    
                    if true_interval >= 0.000001:
                        frame_skip = max(frame_skip * tt_interval / true_interval, 1.0/frame_speed)

                    frame_index += frame_forward

                print(frame_index, frame_speed, frame_skip, is_stop, 'fps=', 1.0/tt_interval)
                # input()
            if play_method == PLAY_METHOD.LOOP:
                continue
            elif play_method == PLAY_METHOD.LIST:
                rp -= 1
                if rp > 0:
                    continue
            break

        
        if play_method == PLAY_METHOD.LIST:
            continue
        break

    ser.close()
    print("所有帧已发送完毕。")