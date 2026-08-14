import serial
import json
import math
import numpy as np
from ahrs.filters import Madgwick
from ahrs.common.quaternion import Quaternion
from ahrs.common.orientation import q2euler

import os
import sys

GAUSS_TO_MILLI_TESLA_CONVERSION = 10
MILLISECOND_TO_SECOND_CONVERSION = 1000
CONFIG_FILENAME = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "../calibration/imu_calibration/cal_data.json",
)

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from update_config import load_config

config = load_config()

SERIAL_PORT = config["serial_port"]
BAUD_RATE = config["baud_rate"]
GAIN = 0.041


class BodyRotationTracker:
    def __init__(self, name="left", config_file=CONFIG_FILENAME):
        self.filter = Madgwick(gain=GAIN)
        self.q_prev = np.array([1.0, 0.0, 0.0, 0.0])

        # Three completely independent accumulated body-frame angles
        self.body_x_deg = 0.0
        self.body_y_deg = 0.0
        self.body_z_deg = 0.0

        self.accOffset = None
        self.accScale = None
        self.gOffset = None
        self.magOffset = None
        self.magScale = None
        self.name = name
        self.load_calibration_data(config_file)
        self.last_time = 0

    def load_calibration_data(self, file):
        """
        Read data from filename for IMU offset and scale data
        """
        with open(file, "r", encoding="utf-8") as f:
            data = json.load(f)

        try:
            data = data[self.name]
        except:
            print(f"Failed to find name: {self.name} in {file}")
            exit(1)

        self.accOffset = data["accOffset"]
        self.accScale = data["accScale"]
        self.gOffset = data["gOffset"]
        self.magOffset = data["magOffset"]
        self.magScale = data["magScale"]

    def get_imu_data(self, values):
        """
        Read Raw Sensor Data and use calibrated values
        """
        try:
            time, ax, ay, az, gx, gy, gz, mx, my, mz = map(float, values)
        except ValueError as e:
            # TODO convert to warning
            print(f"Failed to convert to float with error: {e}")

        axCal = (ax - self.accOffset["x"]) * self.accScale["x"]
        ayCal = (ay - self.accOffset["y"]) * self.accScale["y"]
        azCal = (az - self.accOffset["z"]) * self.accScale["z"]

        gxCal = math.radians(gx - self.gOffset["x"])
        gyCal = math.radians(gy - self.gOffset["y"])
        gzCal = math.radians(gz - self.gOffset["z"])

        mxCal = (
            (mx - self.magOffset["x"]) * self.magScale["x"]
        ) / GAUSS_TO_MILLI_TESLA_CONVERSION
        myCal = (
            (my - self.magOffset["y"]) * self.magScale["y"]
        ) / GAUSS_TO_MILLI_TESLA_CONVERSION
        mzCal = (
            (mz - self.magOffset["z"]) * self.magScale["z"]
        ) / GAUSS_TO_MILLI_TESLA_CONVERSION

        gyro_data = np.array([gxCal, gyCal, gzCal])
        acc_data = np.array([axCal, ayCal, azCal])
        mag_data = np.array([mxCal, myCal, mzCal])

        dt = (time - self.last_time) / MILLISECOND_TO_SECOND_CONVERSION
        self.last_time = time

        return dt, gyro_data, acc_data, mag_data

    def update(self, dt, gyro, accel, mag):
        # 1. Get the global, drift-corrected quaternion from Madgwick
        q_curr = self.filter.updateMARG(
            self.q_prev, gyr=gyro, acc=accel, mag=mag, dt=dt
        )

        # Save state for the next frame
        self.q_prev = q_curr

        return q_curr

    def get_quaternion(self, line):
        if type(line) == str:
            values = line.split(",")
        else:
            values = line

        if len(values) != 10:
            print("Incorrect number of variables passed")
            return

        # 1. Grab the fresh data from the IMU
        dt, gyro, accel, mag = self.get_imu_data(values)

        # 2. Feed the vectors into the filter and get the delta-calculated Z angle
        q = self.update(dt, gyro, accel, mag)

        # 3. Print or use your perfectly drift-corrected body-frame rotation
        return q


def poll_serial_port():
    """
    Function which reads serial port and prints angles
    """
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    except Exception:
        print(f"Failed to connect to port: {SERIAL_PORT}")
        exit(1)

    # Initialize the tracker
    tracker = BodyRotationTracker()

    try:
        while True:
            try:
                line = ser.readline().decode("utf-8").strip()
            except Exception as e:
                print(f"Failed to read line: {e}")
                continue

            angles = tracker.get_quaternion(line)
            if angles is None:
                print("Failed to retrived angles")
                continue

            msg = f"{angles[0].item():.2f}, {angles[1].item():.2f}, {angles[2].item():.2f}"
            print(msg)

    except KeyboardInterrupt:
        print("\nTracking stopped.")

if __name__ == "__main__":
    poll_serial_port()
