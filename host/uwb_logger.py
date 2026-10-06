import serial
import threading
import datetime
import os

LOG_DIR = os.path.expanduser("~/uwb_logs")
os.makedirs(LOG_DIR, exist_ok=True)

def log_port(port, name):
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{LOG_DIR}/{name}_{timestamp}.txt"
    
    try:
        ser = serial.Serial(port, 115200, timeout=1)
        print(f"[{name}] 접속 성공: {port} → {filename}")
        
        with open(filename, 'w') as f:
            while True:
                line = ser.readline().decode('utf-8', errors='ignore')
                if line:
                    ts = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
                    entry = f"[{ts}] {line}"
                    print(entry, end='')
                    f.write(entry)
                    f.flush()
    except Exception as e:
        print(f"[{name}] 에러: {e}")

t1 = threading.Thread(target=log_port, args=('/dev/ttyACM0', 'listener1'))
t2 = threading.Thread(target=log_port, args=('/dev/ttyACM1', 'listener2'))

t1.start()
t2.start()

t1.join()
t2.join()
