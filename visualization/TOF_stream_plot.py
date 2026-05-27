import serial
from pathlib import Path
from collections import deque
import time
import matplotlib.pyplot as plt
import os
import sys

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), "../.."))
from update_config import load_config

config = load_config()

PORT = config["serial_port"]
BAUD = config["baud_rate"]

COEFF_FILE = Path(r"C:\Users\SYSC4907\Documents\Arduino\coeff.txt")
WINDOW_SIZE = 20
PLOT_DURATION_SECONDS = 30


def load_coefficients(path):
    coeffs = {"SENSOR_1": {}, "SENSOR_2": {}}

    current_sensor = None

    with open(path, "r") as f:
        for line in f:
            line = line.strip()

            if not line:
                continue

            if line == "SENSOR_1_COEFFICIENTS":
                current_sensor = "SENSOR_1"
                continue

            if line == "SENSOR_2_COEFFICIENTS":
                current_sensor = "SENSOR_2"
                continue

            if "=" in line and current_sensor is not None:
                key, value = line.split("=")
                coeffs[current_sensor][key.strip()] = float(value.strip())

    required = ["a5", "a4", "a3", "a2", "a1", "a0"]

    for sensor_name in ["SENSOR_1", "SENSOR_2"]:
        for key in required:
            if key not in coeffs[sensor_name]:
                raise ValueError(f"Missing {key} for {sensor_name}")

    return coeffs


def correct_distance(raw, c):
    return (
        c["a5"] * raw**5
        + c["a4"] * raw**4
        + c["a3"] * raw**3
        + c["a2"] * raw**2
        + c["a1"] * raw
        + c["a0"]
    )


def parse_tof_line(line):
    if not line.startswith("TOF1:"):
        return None, None

    try:
        parts = line.split(",")

        raw1 = float(parts[0].replace("TOF1:", "").strip())
        raw2 = float(parts[1].replace("TOF2:", "").strip())

        return raw1, raw2

    except Exception:
        return None, None


coeffs = load_coefficients(COEFF_FILE)

window1 = deque(maxlen=WINDOW_SIZE)
window2 = deque(maxlen=WINDOW_SIZE)

time_points = []
sensor1_points = []
sensor2_points = []

ser = serial.Serial(PORT, BAUD, timeout=1)

print("Testing dual VL53L0X coefficients")
print("Corrected1_mm, Corrected2_mm")
print(f"Collecting {PLOT_DURATION_SECONDS} seconds of data for plotting...")

start_time = time.time()

while True:
    line = ser.readline().decode(errors="ignore").strip()

    if not line:
        continue

    if line == "READY":
        print("Arduino ready.")
        continue

    raw1, raw2 = parse_tof_line(line)

    if raw1 is None or raw2 is None:
        print(line)
        continue

    corrected1 = None
    corrected2 = None

    if raw1 >= 0:
        corrected1 = correct_distance(raw1, coeffs["SENSOR_1"])
        window1.append(corrected1)

    if raw2 >= 0:
        corrected2 = correct_distance(raw2, coeffs["SENSOR_2"])
        window2.append(corrected2)

    corrected1_str = f"{corrected1:.2f}" if corrected1 is not None else "OUT_OF_RANGE"
    corrected2_str = f"{corrected2:.2f}" if corrected2 is not None else "OUT_OF_RANGE"

    print(f"{corrected1_str}, {corrected2_str}")

    elapsed_time = time.time() - start_time

    if corrected1 is not None and corrected2 is not None:
        time_points.append(elapsed_time)
        sensor1_points.append(corrected1)
        sensor2_points.append(corrected2)

    if elapsed_time >= PLOT_DURATION_SECONDS:
        break

ser.close()

plt.figure()
plt.plot(time_points, sensor1_points, label="TOF Sensor 1")
plt.plot(time_points, sensor2_points, label="TOF Sensor 2")
plt.xlabel("Time (s)")
plt.ylabel("Distance (mm)")
plt.title("Corrected TOF Distance Over First 30 Seconds")
plt.legend()
plt.grid(True)
plt.show()
