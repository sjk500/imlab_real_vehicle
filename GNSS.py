import serial

def dmm_to_dd(dmm: str, direction: str) -> float:
    if not dmm:
        return None
    degrees = int(float(dmm) / 100)
    minutes = float(dmm) - degrees * 100
    dd = degrees + minutes / 60
    if direction in ['S', 'W']:
        dd *= -1
    return dd

class GNSSReader:
    def __init__(self, port="/dev/ttyUSB0", baudrate=115200, timeout=1.0):
        try:
            self.ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
            print(f"[GNSS] 연결됨: {port}")
        except serial.SerialException as e:
            print(f"[GNSS] 포트 연결 실패: {e}")
            self.ser = None

    def read(self):
        if not self.ser:
            return None
        try:
            line = self.ser.readline().decode('ascii', errors='ignore').strip()
            if line.startswith('$GPGGA'):
                parts = line.split(',')
                if len(parts) < 6:
                    return None
                try:
                    timestamp = parts[1]
                    lat = dmm_to_dd(parts[2], parts[3])
                    lon = dmm_to_dd(parts[4], parts[5])
                    return timestamp, lat, lon
                except (ValueError, IndexError):
                    return None
        except Exception as e:
            print(f"[GNSS] 읽기 오류: {e}")
            return None

    def close(self):
        if self.ser:
            self.ser.close()
