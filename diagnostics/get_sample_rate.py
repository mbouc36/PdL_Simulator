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
import numpy as np
import pandas as pd
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
FPS = 30


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
            serial_max_diff, camera_max_diff = 0, 0
            serial_min_diff, camera_min_diff = (
                float("inf"),
                float("inf"),
            )
            previous_serial_time, previous_camera_time = None, None
            serial_time_sum, camera_time_sum = 0, 0

            num_samples = 0
            for row in reader:
                serial_time = int(row[serial_time_index])
                camera_time = float(row[camera_time_index]) * 1000  # convert to ms

                # Should only be for first value
                if previous_serial_time is None and previous_camera_time is None:
                    previous_serial_time = serial_time
                    previous_camera_time = camera_time
                    continue

                serial_time_diff = serial_time - previous_serial_time
                camera_time_diff = camera_time - previous_camera_time

                # Get totals to compute averages
                num_samples += 1
                serial_time_sum += serial_time_diff
                camera_time_sum += camera_time_diff

                # Get max values
                if serial_time_diff > serial_max_diff:
                    serial_max_diff = serial_time_diff

                if camera_time_diff > camera_max_diff:
                    camera_max_diff = camera_time_diff

                # Get min values
                if serial_time_diff < serial_min_diff:
                    serial_min_diff = serial_time_diff

                if camera_time_diff < camera_min_diff:
                    camera_min_diff = camera_time_diff

                previous_serial_time = serial_time
                previous_camera_time = camera_time

            average_serial_time = serial_time_sum / num_samples
            average_camera_time = camera_time_sum / num_samples
            print(
                f"Average serial time (ms): {average_serial_time:.2f}, max: {serial_max_diff:.2f}, min: {serial_min_diff:.2f}"
            )
            print(
                f"Average camera time (ms): {average_camera_time:.2f}, max: {camera_max_diff:.2f}, min: {camera_min_diff:.2f}"
            )

            return average_serial_time, average_camera_time

    except Exception as e:
        print(f"Failed to read sensor data from csv: {e}")



def is_likely_clock_drift(
    file,
    min_r_squared=0.80,
    max_residual_ms=100,
    max_p95_residual_ms=70,
):
    """
    Measure difference between arduino time and camera time and detemermine if the
    difference can be measured as a linear offset

    Returns true if the data is validated
    """
    df = pd.read_csv(file)

    arduino = df["Arduino Time"].to_numpy() / 1000
    camera = df["Camera Time"].to_numpy()

    # Start both clocks at zero
    arduino = arduino - arduino[0]
    camera = camera - camera[0]

    # Difference between the clocks
    error = camera - arduino

    # Fit linear clock drift
    slope, intercept = np.polyfit(arduino, error, 1)
    predicted_error = slope * arduino + intercept

    # Timing difference NOT explained by linear drift
    residual = error - predicted_error
    residual_ms = np.abs(residual) * 1000

    # How well does linear drift explain the error?
    ss_res = np.sum((error - predicted_error) ** 2)
    ss_tot = np.sum((error - np.mean(error)) ** 2)

    r_squared = 1 - (ss_res / ss_tot)

    # Residual statistics
    p95_residual = np.percentile(residual_ms, 95)
    max_residual = np.max(residual_ms)
    rmse = np.sqrt(np.mean(residual ** 2)) * 1000

    print(f"Drift rate:       {slope * 1000:.2f} ms/s")
    print(f"R²:               {r_squared:.4f}")
    print(f"Residual RMSE:    {rmse:.2f} ms")
    print(f"95% residual:     {p95_residual:.2f} ms")
    print(f"Maximum residual: {max_residual:.2f} ms")

    # Drift must be sufficiently linear
    linear_drift = r_squared >= min_r_squared

    # There must not be excessive unexplained timing error
    residual_valid = (
        p95_residual <= max_p95_residual_ms
        and max_residual <= max_residual_ms
    )

    if linear_drift and residual_valid:
        print("PASS: Difference is consistent with linear clock drift.")
        return True

    print("FAIL: Difference cannot be explained sufficiently by clock drift.")

    if not linear_drift:
        print("  - Drift is not sufficiently linear.")

    if not residual_valid:
        print("  - Excessive timing error remains after removing drift.")

    return False

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Find the output sample rate over the serial port or from a file"
    )

    parser.add_argument(
        "-f",
        "--csv_file",
        type=Path,
        required=False,
        default=None,
        help="Path to data output csv file",
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
        is_likely_clock_drift(args.name_file)
