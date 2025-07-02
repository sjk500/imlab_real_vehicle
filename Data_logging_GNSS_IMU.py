import serial
import os

def get_next_filename(base_dir="./logs", base_name="Data", ext=".txt"):
    os.makedirs(base_dir, exist_ok=True)
    index = 1
    while True:
        filename = os.path.join(base_dir, f"{base_name}{index}{ext}")
        if not os.path.exists(filename):
            return filename
        index += 1

def log_gnss_raw_data(port="/dev/ttyUSB0", baudrate=115200, output_dir="/home/i/Vehicle_control/GNSS_Logging/"):
    filename = get_next_filename(output_dir)
    try:
        with serial.Serial(port, baudrate, timeout=1) as ser, open(filename, 'w') as file:
            print(f"[INFO] Logging GNSS raw data to: {filename}")
            while True:
                line = ser.readline()
                if line:
                    try:
                        decoded_line = line.decode('utf-8', errors='ignore').strip()
                        print(decoded_line)
                        file.write(decoded_line + "\n")
                    except Exception as e:
                        print(f"[ERROR] Decode failed: {e}")
    except serial.SerialException as e:
        print(f"[ERROR] Serial port error: {e}")
    except KeyboardInterrupt:
        print("\n[INFO] Logging stopped by user.")

if __name__ == "__main__":
    log_gnss_raw_data(port="/dev/ttyUSB0", baudrate=115200)
