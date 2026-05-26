import serial
from pathlib import Path
from collections import deque

PORT = "COM4"
BAUD = 9600

COEFF_FILE = Path(r"C:\Users\SYSC4907\Documents\Arduino\coeff.txt")
WINDOW_SIZE = 20


def load_coefficients(path):
    coeffs = {
        "SENSOR_1": {},
        "SENSOR_2": {}
    }

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
        c["a5"] * raw**5 +
        c["a4"] * raw**4 +
        c["a3"] * raw**3 +
        c["a2"] * raw**2 +
        c["a1"] * raw +
        c["a0"]
    )


def parse_tof_line(line):
    """
    Expected Arduino format:
    TOF1:152,TOF2:149
    """

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

ser = serial.Serial(PORT, BAUD, timeout=1)

print("Testing dual VL53L0X coefficients")
print("Corrected1_mm,  Corrected2_mm")

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
    avg1 = None
    avg2 = None

    if raw1 >= 0:
        corrected1 = correct_distance(raw1, coeffs["SENSOR_1"])
        window1.append(corrected1)
        avg1 = sum(window1) / len(window1)

    if raw2 >= 0:
        corrected2 = correct_distance(raw2, coeffs["SENSOR_2"])
        window2.append(corrected2)
        avg2 = sum(window2) / len(window2)

    #raw1_str = f"{raw1:.2f}" if raw1 >= 0 else "OUT_OF_RANGE"
    #raw2_str = f"{raw2:.2f}" if raw2 >= 0 else "OUT_OF_RANGE"

    corrected1_str = f"{corrected1:.2f}" if corrected1 is not None else "OUT_OF_RANGE"
    corrected2_str = f"{corrected2:.2f}" if corrected2 is not None else "OUT_OF_RANGE"

    #avg1_str = f"{avg1:.2f}" if avg1 is not None else "OUT_OF_RANGE"
    #avg2_str = f"{avg2:.2f}" if avg2 is not None else "OUT_OF_RANGE"

    print(
        f" {corrected1_str}"
        f" {corrected2_str}"
    )