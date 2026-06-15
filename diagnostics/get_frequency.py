import os
import sys
import time
import serial

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from update_config import load_config

config = load_config()

SERIAL_PORT = config["serial_port"]
BAUD_RATE = config["baud_rate"]


def find_loop_frequency():
    prev_time = time.time()
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

    while True:

        line = ser.readline().decode("utf-8", errors="ignore").strip()

        if not line:
            continue

        current_time = time.time()
        dt = current_time - prev_time
        prev_time = current_time

        print(dt)

if __name__ == "__main__":
    find_loop_frequency()