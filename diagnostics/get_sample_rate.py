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

serial_window = deque()
camera_window = deque()


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
        serial_window.append(dt)

        if len(serial_window) >= WINDOW_LENGTH:
            serial_window.popleft()

        average_dt = get_average()

        average_frequency = 1 / average_dt

        prev_time = current_time

        print(f"Frequency {average_frequency}, change in time:  {dt}")


def get_average():
    sum_of_window = 0
    for i in range(len(serial_window)):
        sum_of_window += serial_window[i]

    return sum_of_window / len(serial_window)


def get_sample_rate_from_csv(file, serial_time_index=0, camera_time_index=1):
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
            serial_max_diff, camera_max_diff, camera_serial_max_diff = 0, 0, 0
            serial_min_diff, camera_min_diff, camera_serial_min_diff = (
                float("inf"),
                float("inf"),
                float("inf"),
            )
            previous_serial_time, previous_camera_time = None, None
            init_serial_time, init_camera_time = None, None
            serial_time_sum, camera_time_sum, camera_serial_diff_sum = 0, 0, 0

            num_samples = 0
            for row in reader:
                serial_time = int(row[serial_time_index])
                camera_time = float(row[camera_time_index]) * 1000  # convert to ms

                # Should only be for first value
                if previous_serial_time is None and previous_camera_time is None:
                    previous_serial_time, init_serial_time = serial_time, serial_time
                    previous_camera_time, init_camera_time = camera_time, camera_time
                    continue

                serial_time_diff = serial_time - previous_serial_time
                camera_time_diff = camera_time - previous_camera_time

                camera_serial_time_diff = (camera_time - init_camera_time) - (
                    serial_time - init_serial_time
                )

                # Get totals to compute averages
                num_samples += 1
                serial_time_sum += serial_time_diff
                camera_time_sum += camera_time_diff
                camera_serial_diff_sum += camera_serial_time_diff

                # Get max values
                if serial_time_diff > serial_max_diff:
                    serial_max_diff = serial_time_diff

                if camera_time_diff > camera_max_diff:
                    camera_max_diff = camera_time_diff

                if camera_serial_time_diff > camera_serial_max_diff:
                    print(serial_time)
                    camera_serial_max_diff = camera_serial_time_diff

                # Get min values
                if serial_time_diff < serial_min_diff:
                    serial_min_diff = serial_time_diff

                if camera_time_diff < camera_min_diff:
                    camera_min_diff = camera_time_diff

                if camera_serial_time_diff < camera_serial_min_diff:
                    camera_serial_min_diff = camera_serial_time_diff

                previous_serial_time = serial_time
                previous_camera_time = camera_time

            average_serial_time = serial_time_sum / num_samples
            average_camera_time = camera_time_sum / num_samples
            average_camera_serial_time_diff = camera_serial_diff_sum / num_samples
            print(
                f"Average serial time (ms): {average_serial_time:.2f}, max: {serial_max_diff:.2f}, min: {serial_min_diff:.2f}"
            )
            print(
                f"Average camera time (ms): {average_camera_time:.2f}, max: {camera_max_diff:.2f}, min: {camera_min_diff:.2f}"
            )
            print(
                f"Average difference between camera and serial times (ms): {average_camera_serial_time_diff:.2f}, max: {camera_serial_max_diff:.2f}, min: {camera_serial_min_diff:.2f}"
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
        "--serial_time_index",
        type=int,
        required=False,
        default=0,
        help="Index of time in csv",
    )

    args = parser.parse_args()

    if args.name_file is None:
        find_loop_frequency()

    else:
        get_sample_rate_from_csv(args.name_file, args.serial_time_index)
