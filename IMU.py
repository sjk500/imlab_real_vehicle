import serial

def parse_pashr(sentence: str):
    """$PASHR 메시지에서 yaw(heading) 추출"""
    try:
        parts = sentence.split(',')
        if len(parts) < 3:
            return None
        yaw = float(parts[2])
        return yaw
    except (ValueError, IndexError):
        return None

def parse_avr(sentence: str):
    """$PTNL,AVR 메시지에서 yaw(heading) 추출"""
    try:
        parts = sentence.split(',')
        for i in range(len(parts)):
            if "Yaw" in parts[i]:
                yaw = float(parts[i + 1].replace('+', ''))
                return yaw
        return None
    except (ValueError, IndexError):
        return None

def get_imu_yaw(port="/dev/ttyUSB0", baudrate=9600, timeout=1.0):
    """GNSS 장비로부터 실시간 IMU 메시지를 읽고 yaw 추출"""
    try:
        ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
        print(f"[IMU] 연결됨: {port}")
    except serial.SerialException as e:
        print(f"[IMU] 포트 연결 실패: {e}")
        return None

    while True:
        try:
            line = ser.readline().decode('ascii', errors='ignore').strip()
            if line.startswith('$PASHR'):
                yaw = parse_pashr(line)
                if yaw is not None:
                    print(f"[IMU] PASHR → Yaw: {yaw}")
                    return yaw
            elif line.startswith('$PTNL,AVR'):
                yaw = parse_avr(line)
                if yaw is not None:
                    print(f"[IMU] AVR → Yaw: {yaw}")
                    return yaw
        except KeyboardInterrupt:
            print("[IMU] 중단됨")
            break
        except Exception as e:
            print(f"[IMU] 오류: {e}")
            continue

    ser.close()
