import can
import time
import threading
import numpy as np
from pynput import keyboard
from pyproj import Proj, Transformer

# ========== 설정값 ==========
WHEELBASE = 2.7        # 차량 축간 거리 (m)
LOOKAHEAD_DIST = 4.0   # Pure Pursuit lookahead 거리 (m)
GNSS_OFFSET = 0.87     # GNSS → 뒷바퀴 중심까지 거리 (m)

# ========== 전역 상태 ==========
steering_angle = 0
throttle_percent = 0
brake_percent = 0
target_steering_angle = 0

key_states = {'up': False, 'down': False}

running = True
alive_count = 0

# ========== 키보드 제어 ==========
def control_loop():
    global throttle_percent, brake_percent, steering_angle, target_steering_angle
    accel_ramp = 0
    brake_ramp = 0

    while running:
        if key_states['up']:
            accel_ramp = min(300, accel_ramp + 10)
        else:
            accel_ramp = 0

        if key_states['down']:
            brake_ramp = min(700, brake_ramp + 10)
        else:
            brake_ramp = 0

        throttle_percent = accel_ramp
        brake_percent = brake_ramp

        # Pure Pursuit로 계산된 조향각 사용
        steering_angle = target_steering_angle

        print(f"[제어값] 핸들: {steering_angle}, 엑셀: {throttle_percent}, 브레이크: {brake_percent}")
        time.sleep(0.01)

def on_press(key):
    try:
        if key == keyboard.Key.up:
            key_states['up'] = True
        elif key == keyboard.Key.down:
            key_states['down'] = True
    except:
        pass

def on_release(key):
    try:
        if key == keyboard.Key.up:
            key_states['up'] = False
        elif key == keyboard.Key.down:
            key_states['down'] = False
    except:
        pass

def start_keyboard_listener():
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

# ========== CAN 전송 ==========
def clamp(val, min_val, max_val):
    return max(min_val, min(val, max_val))

def send_0x156():
    global alive_count
    data = bytearray([0x01, 0xFF, 0x01, 0x00, 0x00, 0x00, 0x00, alive_count])
    msg = can.Message(arbitration_id=0x156, data=data, is_extended_id=False)
    bus.send(msg)

def send_0x157():
    steer = clamp(steering_angle, -3600, 3600)
    accel = clamp(throttle_percent, 0, 1000)
    brake = clamp(brake_percent, 0, 1000)

    data = bytearray(8)
    data[0:2] = int(steer).to_bytes(2, 'little', signed=True)
    data[2] = 0x00
    data[3] = accel & 0xff
    data[4] = (accel >> 8) & 0xff
    data[5] = 0x00
    data[6] = brake & 0xff
    data[7] = (brake >> 8) & 0xff

    msg = can.Message(arbitration_id=0x157, data=data, is_extended_id=False)
    bus.send(msg)

def send_loop():
    global alive_count
    while running:
        send_0x156()
        send_0x157()
        alive_count = (alive_count + 1) % 256
        time.sleep(0.01)

# ========== Pure Pursuit 알고리즘 ==========
def pure_pursuit_control(path_enu, current_pos, current_yaw):
    x_gnss, y_gnss = current_pos
    x_rear = x_gnss - GNSS_OFFSET * np.cos(current_yaw)
    y_rear = y_gnss - GNSS_OFFSET * np.sin(current_yaw)

    target = None
    for px, py in path_enu:
        dx, dy = px - x_rear, py - y_rear
        dist = np.hypot(dx, dy)
        if dist >= LOOKAHEAD_DIST:
            target = (px, py)
            break
    if not target:
        target = path_enu[-1]

    dx, dy = target[0] - x_rear, target[1] - y_rear
    alpha = np.arctan2(dy, dx) - current_yaw
    delta = np.arctan2(2 * WHEELBASE * np.sin(alpha), LOOKAHEAD_DIST)

    return int(np.degrees(delta) * 20)  # 보정 계수 필요시 수정

# ========== 좌표 변환 도우미 ==========
class ENUConverter:
    def __init__(self, ref_lat, ref_lon):
        zone = int((ref_lon + 180) / 6) + 1
        self.transformer = Transformer.from_crs("epsg:4326", f"+proj=utm +zone={zone} +ellps=WGS84")
        self.ref_e, self.ref_n = self.transformer.transform(ref_lon, ref_lat)

    def to_enu(self, lat, lon):
        e, n = self.transformer.transform(lon, lat)
        return e - self.ref_e, n - self.ref_n

# ========== 실행 진입점 ==========
can_interface = 'can0'
bus = can.interface.Bus(channel=can_interface, interface='socketcan')

if __name__ == '__main__':
    try:
        # 1. 기준 경로 불러오기 및 ENU 변환
        path_csv = 'Path.csv'
        with open(path_csv, 'r') as f:
            import csv
            reader = list(csv.DictReader(f))
            ref_lat = float(reader[0]['latitude'])
            ref_lon = float(reader[0]['longitude'])
            enu = ENUConverter(ref_lat, ref_lon)
            path_enu = [enu.to_enu(float(r['latitude']), float(r['longitude'])) for r in reader]

        # 2. 스레드 시작
        start_keyboard_listener()
        threading.Thread(target=control_loop).start()
        threading.Thread(target=send_loop).start()

        print("[시작] 자율조향 + 수동엑셀/브레이크 제어 중...")

        while True:
            # GNSS + yaw 실시간 데이터 받기 (예: 외부 센서 수신 또는 다른 모듈 연동 필요)
            current_lat = ...
            current_lon = ...
            current_yaw_rad = ...

            x_gnss, y_gnss = enu.to_enu(current_lat, current_lon)
            target_steering_angle = pure_pursuit_control(path_enu, (x_gnss, y_gnss), current_yaw_rad)

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("종료 중...")
        running = False
        bus.shutdown()
