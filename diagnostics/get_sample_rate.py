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


def get_time_stats_from_csv(file_path, column_index=0, scale=1.0, has_header=True):
    """Calculate statistics for consecutive time differences in a CSV column."""
    if column_index < 0:
        raise ValueError("column_index must be non-negative")

    previous_time = None
    total_diff = 0.0
    min_diff = float("inf")
    max_diff = float("-inf")
    interval_count = 0

    with open(file_path, mode="r", newline="", encoding="utf-8") as file:
        reader = csv.reader(file)

        if has_header:
            next(reader, None)

        for row in reader:
            if not row:
                continue

            try:
                current_time = float(row[column_index]) * scale
            except (IndexError, ValueError) as error:
                raise ValueError(
                    f"Invalid value at CSV line {reader.line_num}, "
                    f"column {column_index}"
                ) from error

            if previous_time is not None:
                time_diff = current_time - previous_time
                total_diff += time_diff
                min_diff = min(min_diff, time_diff)
                max_diff = max(max_diff, time_diff)
                interval_count += 1

            previous_time = current_time

    if interval_count == 0:
        raise ValueError("At least two time values are required")

    average = total_diff / interval_count

    min_value = min_diff

    max_value = max_diff

    count = interval_count

    print(f"{file_path}:")
    print(f"Average: {average:.3f}")
    print(f"Min: {min_value:.3f}")
    print(f"Max: {max_value:.3f}")
    print(f"Interval Count: {count}")

    return average


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
    rmse = np.sqrt(np.mean(residual**2)) * 1000

    print(f"Drift rate:       {slope * 1000:.2f} ms/s")
    print(f"R²:               {r_squared:.4f}")
    print(f"Residual RMSE:    {rmse:.2f} ms")
    print(f"95% residual:     {p95_residual:.2f} ms")
    print(f"Maximum residual: {max_residual:.2f} ms")

    # Drift must be sufficiently linear
    linear_drift = r_squared >= min_r_squared

    # There must not be excessive unexplained timing error
    residual_valid = (
        p95_residual <= max_p95_residual_ms and max_residual <= max_residual_ms
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

    if args.csv_file is None:
        find_loop_frequency()

    else:
        get_time_stats_from_csv(args.csv_file, args.serial_time_index)
        is_likely_clock_drift(args.csv_file)