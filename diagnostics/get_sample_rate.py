"""
Author: Michael Boucouvalas
Date: 2026, Sep 2
Version: 2.0
Description: Find the frequency of output data either from the serial port or csv file
"""

import os
import sys
import csv
import time
import serial
import argparse
from pathlib import Path
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

        average_frequency = 1 / average_dt

        prev_time = current_time

        print(f"Frequency {average_frequency}, change in time:  {dt}")


def get_average():
    sum_of_window = 0
    for i in range(len(window_average)):
        sum_of_window += window_average[i]

    return sum_of_window / len(window_average)


def get_sample_rate_from_csv(file, time_index=0):
    """
    Assuming time is a given index in a csv file
    """
    try:
        with open(
            file,
            mode="r",
            newline="",
            encoding="utf-8",
            
        ) as file:
            reader = csv.reader(file)
            next(reader)
            max_diff = 0
            min_diff = float("inf")
            previous_time = None
            time_sum = 0
            num_samples = 0
            for row in reader:

                arduino_time = int(row[time_index])

                # Should only be for first value
                if previous_time is None:
                    previous_time = arduino_time
                    continue

                time_diff = arduino_time - previous_time
                num_samples += 1
                time_sum += time_diff

                max_diff = max(time_diff, max_diff)

                min_diff = min(time_diff, min_diff)

                previous_time = arduino_time

            average_sample_time = time_sum / num_samples
            print(
                f"Average sample time: {average_sample_time:.2f}, max: {max_diff}, min: {min_diff}"
            )

    except Exception as e:
        print(f"Failed to read sensor data from csv: {e}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Find the output sample rate over the serial port or from a file"
    )

    parser.add_argument(
        "-f",
        "--name_file",
        type=Path,
        required=False,
        default=None,
        help="Path to the name to key file",
    )

    parser.add_argument(
        "--time_index",
        type=int,
        required=False,
        default=0,
        help="Index of time in csv",
    )

    args = parser.parse_args()

    if args.name_file is None:
        find_loop_frequency()

    else:
        get_sample_rate_from_csv(args.name_file, args.time_index)
