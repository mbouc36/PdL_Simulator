# Calibration script for left/right TOF sensor
# Collects multiple samples at known distances, averages them,
# fits a polynomial calibration function, and saves the coefficients.

import argparse
import csv
import os
import serial
import sys
import numpy as np

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), "../.."))
from update_config import load_config

config = load_config()

PORT = config["serial_port"]
BAUD = config["baud_rate"]

SERIAL_INPUT_LENGTH = 23

# Index of each TOF value in the comma-separated serial data
LEFT_TOF_INDEX = 3
RIGHT_TOF_INDEX = 4

POLY_DEGREE = 5

# Known calibration distances in mm
LEFT_KNOWN_DISTANCES = [54, 68, 80, 95, 106, 120, 127]

RIGHT_KNOWN_DISTANCES = [0, 55, 70, 82, 97, 109, 121, 130]

PARENT_DIR = os.path.dirname(os.path.abspath(__file__))


def parse_args():
    parser = argparse.ArgumentParser(
        description="Calibrate either the left or right TOF sensor."
    )

    parser.add_argument(
        "tof",
        choices=["left", "right"],
        help="TOF sensor to calibrate.",
    )

    parser.add_argument(
        "-n",
        "--samples",
        type=int,
        default=100,
        help="Number of samples to average at each calibration point.",
    )

    return parser.parse_args()


def read_tof_sample(ser, sensor_index):
    """
    Waits for a valid serial line and returns one TOF measurement.
    """

    while True:
        line = ser.readline().decode(errors="ignore").strip()

        if not line:
            continue

        raw_sensor_data = line.split(",")

        if len(raw_sensor_data) != SERIAL_INPUT_LENGTH:
            continue

        try:
            value = float(raw_sensor_data[sensor_index].strip())
            return value

        except ValueError:
            continue


def capture_average(ser, sensor_index, num_samples):
    """
    Captures num_samples TOF measurements and returns their average.
    """

    samples = []

    while len(samples) < num_samples:
        value = read_tof_sample(ser, sensor_index)

        samples.append(value)

        print(
            f"\rCollecting samples: {len(samples)}/{num_samples}",
            end="",
            flush=True,
        )

    print()

    samples = np.array(samples, dtype=float)

    return np.mean(samples)


# ---------------------------------------------------------
# Main
# ---------------------------------------------------------


def main():

    args = parse_args()

    sensor_name = args.tof
    num_samples = args.samples
    known_distances = []

    if sensor_name == "left":
        sensor_index = LEFT_TOF_INDEX
        known_distances = LEFT_KNOWN_DISTANCES
    else:
        sensor_index = RIGHT_TOF_INDEX
        known_distances = RIGHT_KNOWN_DISTANCES

    output_file = os.path.join(
        PARENT_DIR,
        f"coeff.txt",
    )

    raw_data_file = os.path.join(
        PARENT_DIR,
        f"{sensor_name}_tof_calibration_data.csv",
    )

    if len(known_distances) < POLY_DEGREE + 1:
        raise ValueError(
            f"At least {POLY_DEGREE + 1} calibration points are required "
            f"for a {POLY_DEGREE}th-order polynomial."
        )

    print(f"\nCalibrating {sensor_name.upper()} TOF")
    print(f"Samples per position: {num_samples}")
    print(f"Polynomial degree: {POLY_DEGREE}\n")

    ser = serial.Serial(PORT, BAUD, timeout=1)

    # Clear any old serial data
    ser.reset_input_buffer()

    measured_distances = []

    try:

        for known_distance in known_distances:

            print("\n----------------------------------------")
            print(f"Measure at known point: {known_distance} mm")
            input("Press ENTER when the sensor is positioned...")

            # Throw away serial data collected while positioning sensor
            ser.reset_input_buffer()

            average = capture_average(
                ser,
                sensor_index,
                num_samples,
            )

            measured_distances.append(average)

            print(f"Known distance:    {known_distance:.2f} mm")
            print(f"Average measured: {average:.2f} mm")
            print(f"Error:             {known_distance - average:.2f} mm")

    finally:
        ser.close()

    actual = np.array(known_distances, dtype=float)
    measured = np.array(measured_distances, dtype=float)

    coefficients = np.polyfit(
        measured,
        actual,
        POLY_DEGREE,
    )

    with open(raw_data_file, "w", newline="") as file:

        writer = csv.writer(file)

        writer.writerow(
            [
                "actual_mm",
                "average_measured_mm",
                "error_mm",
            ]
        )

        for actual_distance, measured_distance in zip(actual, measured):

            writer.writerow(
                [
                    f"{actual_distance:.6f}",
                    f"{measured_distance:.6f}",
                    f"{actual_distance - measured_distance:.6f}",
                ]
            )

    section_name = f"{sensor_name.upper()}_SENSOR_COEFFICIENTS"

    # Build new coefficient section
    new_section = section_name + "\n"

    for power, coefficient in zip(
        range(POLY_DEGREE, -1, -1),
        coefficients,
    ):
        new_section += f"a{power}={coefficient:.12e}\n"

    # Read existing file if it exists
    existing_content = ""

    if os.path.exists(output_file):
        with open(output_file, "r") as file:
            existing_content = file.read()

    sections = {}

    # Parse existing LEFT/RIGHT sections
    current_section = None

    for line in existing_content.splitlines():

        if line in (
            "LEFT_SENSOR_COEFFICIENTS",
            "RIGHT_SENSOR_COEFFICIENTS",
        ):
            current_section = line
            sections[current_section] = []

        elif current_section is not None:
            sections[current_section].append(line)

    # Update only the sensor that was just calibrated
    sections[section_name] = new_section.splitlines()[1:]

    # Write both sensors into the same file
    with open(output_file, "w") as file:

        for name in (
            "LEFT_SENSOR_COEFFICIENTS",
            "RIGHT_SENSOR_COEFFICIENTS",
        ):
            if name in sections:
                file.write(f"{name}\n")

                for line in sections[name]:
                    if line:
                        file.write(f"{line}\n")

                file.write("\n")

    print("\n========================================")
    print("Calibration complete")
    print("========================================")

    print("\nCalibration data saved to:")
    print(raw_data_file)

    print("\nPolynomial coefficients saved to:")
    print(output_file)

    print("\nCoefficients:")
    print(coefficients)

    print("\nPolynomial:")
    print(np.poly1d(coefficients))


if __name__ == "__main__":
    main()
