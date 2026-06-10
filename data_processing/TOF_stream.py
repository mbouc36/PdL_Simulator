import os
import serial
from collections import deque
import sys

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from update_config import load_config

config = load_config()

PORT = config["serial_port"]
BAUD = config["baud_rate"]

PARENT_DIR = os.path.dirname(os.path.abspath(__file__))
COEFF_FILE = os.path.join(PARENT_DIR, "../calibration/tof_calibration/coeff.txt")
WINDOW_SIZE = 20


class TOFManager:
    """
    This class manages two ToF sensors
    """

    def __init__(self, path=COEFF_FILE):
        self.coeff = self.load_coefficients(path)

    def load_coefficients(self, path):
        """
        Load offset coeffecients from a specified file
        """
        coeffs = {"LEFT_SENSOR": {}, "RIGHT_SENSOR": {}}

        current_sensor = None

        with open(path, "r") as f:
            for line in f:
                line = line.strip()

                if not line:
                    continue

                if line == "LEFT_SENSOR_COEFFICIENTS":
                    current_sensor = "LEFT_SENSOR"
                    continue

                if line == "RIGHT_SENSOR_COEFFICIENTS":
                    current_sensor = "RIGHT_SENSOR"
                    continue

                if "=" in line and current_sensor is not None:
                    key, value = line.split("=")
                    coeffs[current_sensor][key.strip()] = float(value.strip())

        required = ["a5", "a4", "a3", "a2", "a1", "a0"]

        for sensor_name in ["LEFT_SENSOR", "RIGHT_SENSOR"]:
            for key in required:
                if key not in coeffs[sensor_name]:
                    raise ValueError(f"Missing {key} for {sensor_name}")

        return coeffs

    def correct_distance(self, raw_distance, coeff):
        """
        Pass a raw distance through coeffecients to yeild a
        corrected distance
        """
        if raw_distance <= 0:
            return None

        distance = (
            coeff["a5"] * raw_distance**5
            + coeff["a4"] * raw_distance**4
            + coeff["a3"] * raw_distance**3
            + coeff["a2"] * raw_distance**2
            + coeff["a1"] * raw_distance
            + coeff["a0"]
        )

        if distance <= 0:
            return None
        else:
            return distance

    def parse_tof_line(self, line):
        """
        Expected Arduino format:
        TOF1:152,TOF2:149 (TOF1 being left, and TOF2 being right)
        152, 149 (first value indicates left distance, second value indicates right)

        """

        try:
            parts = line.split(",")
            if line.startswith("TOF1:"):
                raw_left = float(parts[0].replace("TOF1:", "").strip())
                raw_right = float(parts[1].replace("TOF2:", "").strip())
            else:
                raw_left = float(parts[0].strip())
                raw_right = float(parts[1].strip())

            return raw_left, raw_right

        except Exception:
            return None, None

    def get_distances(self, line):
        """
        Get a valid distance value from a line
        """

        raw_left, raw_right = self.parse_tof_line(line)

        corrected_left = self.correct_distance(raw_left, self.coeff["LEFT_SENSOR"])
        corrected_right = self.correct_distance(raw_right, self.coeff["RIGHT_SENSOR"])

        return corrected_left, corrected_right


if __name__ == "__main__":
    tof_manager = TOFManager()
    ser = serial.Serial(PORT, BAUD, timeout=1)

    while True:
        line = ser.readline().decode(errors="ignore").strip()

        if not line:
            continue

        raw_left, raw_right = tof_manager.parse_tof_line(line)

        if raw_left is None or raw_right is None:
            print(line)
            continue

        corrected_left, corrected_right = tof_manager.get_distances()

        corrected_left_str = (
            f"{corrected_left:.2f}" if corrected_left is not None else "OUT_OF_RANGE"
        )
        corrected_right_str = (
            f"{corrected_right:.2f}" if corrected_right is not None else "OUT_OF_RANGE"
        )

        print(f" {corrected_left_str}" f" {corrected_right_str}")
