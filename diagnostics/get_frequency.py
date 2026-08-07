import os
import sys
import time
import serial
from collections import deque

window_average = deque()


sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from update_config import load_config

config = load_config()

SERIAL_PORT = config["serial_port"]
BAUD_RATE = config["baud_rate"]
WINDOW_LENGTH = 10

def find_loop_frequency():
    prev_time = time.time()
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

    while True:

        line = ser.readline().decode("utf-8", errors="ignore").strip()

        if not line:
            continue

        current_time = time.time()
        dt = current_time - prev_time
        window_average.append(dt)

        if len(window_average) >= WINDOW_LENGTH:
            window_average.popleft()

        average_dt = get_average()

        average_frequency = 1/average_dt

        prev_time = current_time

        print(f"Frequency {average_frequency}, change in time:  {dt}")


def get_average():
    sum_of_window = 0
    for i in range(len(window_average)):
        sum_of_window += window_average[i]

    return sum_of_window/len(window_average)
        

if __name__ == "__main__":
    find_loop_frequency()