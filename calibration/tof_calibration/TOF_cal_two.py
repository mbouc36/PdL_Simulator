# Calibration python script for 2 TOF sensors
# Applies 5th order calibration/offset function and saves coefficients

import os
import serial
import numpy as np
import sys

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), "../.."))
from update_config import load_config

config = load_config()

PORT = config["serial_port"]
BAUD = config["baud_rate"]

PARENT_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_FILE = os.path.join(PARENT_DIR, "coeff.txt")
RAW_DATA_FILE = os.path.join(PARENT_DIR, "dual_tof_calibration_data.csv")

POLY_DEGREE = 5

actual = []
left_sensor_measured = []
right_sensor_measured = []

ser = serial.Serial(PORT, BAUD, timeout=1)

print("Python serial monitor started.")
print("Listening for Arduino calibration data...\n")

saving = False

while True:
    line = ser.readline().decode(errors="ignore").strip()

    if not line:
        continue

    print(line)

    if line == "CALIBRATION_DATA_START":
        saving = True
        continue

    if line == "CALIBRATION_DATA_END":
        saving = False
        break

    if saving and line.startswith("CAL_DATA,"):
        parts = line.split(",")

        if len(parts) != 4:
            print("Malformed CAL_DATA line ignored:", line)
            continue

        actual_mm = float(parts[1])
        left_sensor_mm = float(parts[2])
        right_sensor_mm = float(parts[3])

        actual.append(actual_mm)
        left_sensor_measured.append(left_sensor_mm)
        right_sensor_measured.append(right_sensor_mm)

ser.close()

actual = np.array(actual, dtype=float)
left_sensor_measured = np.array(left_sensor_measured, dtype=float)
right_sensor_measured = np.array(right_sensor_measured, dtype=float)

if len(actual) < POLY_DEGREE + 1:
    raise ValueError("Not enough calibration points for a 5th-order polynomial fit.")

# Fit: actual_distance = f(measured_distance)
left_coeff = np.polyfit(left_sensor_measured, actual, POLY_DEGREE)
right_coeff = np.polyfit(right_sensor_measured, actual, POLY_DEGREE)

# np.polyfit returns highest power first:
# [a5, a4, a3, a2, a1, a0]
a5_left, a4_left, a3_left, a2_left, a1_left, a0_left = left_coeff
a5_right, a4_right, a3_right, a2_right, a1_right, a0_right = right_coeff

with open(RAW_DATA_FILE, "w") as f:
    f.write("actual_mm,left_measured_mm,right_measured_mm\n")
    for a, s_left, s_right in zip(actual, left_sensor_measured, right_sensor_measured):
        f.write(f"{a:.6f},{s_left:.6f},{s_right:.6f}\n")

with open(OUTPUT_FILE, "w") as f:
    f.write("LEFT_SENSOR_COEFFICIENTS\n")
    f.write(f"a5={a5_left:.12e}\n")
    f.write(f"a4={a4_left:.12e}\n")
    f.write(f"a3={a3_left:.12e}\n")
    f.write(f"a2={a2_left:.12e}\n")
    f.write(f"a1={a1_left:.12e}\n")
    f.write(f"a0={a0_left:.12e}\n\n")

    f.write("RIGHT_SENSOR_COEFFICIENTS\n")
    f.write(f"a5={a5_right:.12e}\n")
    f.write(f"a4={a4_right:.12e}\n")
    f.write(f"a3={a3_right:.12e}\n")
    f.write(f"a2={a2_right:.12e}\n")
    f.write(f"a1={a1_right:.12e}\n")
    f.write(f"a0={a0_right:.12e}\n")

print("\nCalibration complete.")
print("Raw calibration data saved to:")
print(RAW_DATA_FILE)

print("\nPolynomial coefficients saved to:")
print(OUTPUT_FILE)

print("\nLeft sensor coefficients:")
print(left_coeff)

print("\nRight sensor coefficients:")
print(right_coeff)
