#Calibration python script for 2 TOF sensors 
# Applies 5th order calibration/offset function and saves coefficients 

import serial
import numpy as np
from pathlib import Path

PORT = "COM4"
BAUD = 9600

OUTPUT_FILE = Path(r"C:\Users\SYSC4907\Documents\Arduino\coeff.txt")
RAW_DATA_FILE = Path(r"C:\Users\SYSC4907\Documents\Arduino\dual_tof_calibration_data.csv")

POLY_DEGREE = 5

actual = []
sensor1_measured = []
sensor2_measured = []

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
        sensor1_mm = float(parts[2])
        sensor2_mm = float(parts[3])

        actual.append(actual_mm)
        sensor1_measured.append(sensor1_mm)
        sensor2_measured.append(sensor2_mm)

ser.close()

actual = np.array(actual, dtype=float)
sensor1_measured = np.array(sensor1_measured, dtype=float)
sensor2_measured = np.array(sensor2_measured, dtype=float)

if len(actual) < POLY_DEGREE + 1:
    raise ValueError("Not enough calibration points for a 5th-order polynomial fit.")

# Fit: actual_distance = f(measured_distance)
coeff1 = np.polyfit(sensor1_measured, actual, POLY_DEGREE)
coeff2 = np.polyfit(sensor2_measured, actual, POLY_DEGREE)

# np.polyfit returns highest power first:
# [a5, a4, a3, a2, a1, a0]
a5_1, a4_1, a3_1, a2_1, a1_1, a0_1 = coeff1
a5_2, a4_2, a3_2, a2_2, a1_2, a0_2 = coeff2

with open(RAW_DATA_FILE, "w") as f:
    f.write("actual_mm,sensor1_measured_mm,sensor2_measured_mm\n")
    for a, s1, s2 in zip(actual, sensor1_measured, sensor2_measured):
        f.write(f"{a:.6f},{s1:.6f},{s2:.6f}\n")

with open(OUTPUT_FILE, "w") as f:
    f.write("SENSOR_1_COEFFICIENTS\n")
    f.write(f"a5={a5_1:.12e}\n")
    f.write(f"a4={a4_1:.12e}\n")
    f.write(f"a3={a3_1:.12e}\n")
    f.write(f"a2={a2_1:.12e}\n")
    f.write(f"a1={a1_1:.12e}\n")
    f.write(f"a0={a0_1:.12e}\n\n")

    f.write("SENSOR_2_COEFFICIENTS\n")
    f.write(f"a5={a5_2:.12e}\n")
    f.write(f"a4={a4_2:.12e}\n")
    f.write(f"a3={a3_2:.12e}\n")
    f.write(f"a2={a2_2:.12e}\n")
    f.write(f"a1={a1_2:.12e}\n")
    f.write(f"a0={a0_2:.12e}\n")

print("\nCalibration complete.")
print("Raw calibration data saved to:")
print(RAW_DATA_FILE)

print("\nPolynomial coefficients saved to:")
print(OUTPUT_FILE)

print("\nSensor 1 coefficients:")
print(coeff1)

print("\nSensor 2 coefficients:")
print(coeff2)