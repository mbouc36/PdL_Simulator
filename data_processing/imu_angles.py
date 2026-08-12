import serial
import time
import json
import math
import socket
import numpy as np
from ahrs.filters import Madgwick
from ahrs.common.quaternion import Quaternion
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
FREQUENCY = 180.0
GAIN = 0.2



class BodyRotationTracker:
    def __init__(self, name="left", config_file=CONFIG_FILENAME):
        self.filter = Madgwick( gain=GAIN)
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
        time, ax, ay, az, gx, gy, gz, mx, my, mz = map(float, values)

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

        dt = (time - self.last_time)/MILLISECOND_TO_SECOND_CONVERSION
        self.last_time = time

        return dt, gyro_data, acc_data, mag_data

    def update(self, dt, gyro, accel, mag):
        # 1. Get the global, drift-corrected quaternion from Madgwick
        q_curr = self.filter.updateMARG(self.q_prev, gyr=gyro, acc=accel, mag=mag, dt=dt)

        # 2. Isolate the movement strictly to the local body frame
        q_prev_inv = np.array(
            [self.q_prev[0], -self.q_prev[1], -self.q_prev[2], -self.q_prev[3]]
        )
        delta_q_body = Quaternion(q_prev_inv) * (Quaternion(q_curr))

        w, x, y, z = delta_q_body

        # 3. Extract independent local rotations (Infinitesimal approximation)
        # Because the time step is 0.01s, these are perfectly decoupled.
        delta_x = 2.0 * np.arctan2(x, w)
        delta_y = 2.0 * np.arctan2(y, w)
        delta_z = 2.0 * np.arctan2(z, w)

        # 4. Accumulate into your independent registers
        self.body_x_deg += np.degrees(delta_x)
        self.body_y_deg += np.degrees(delta_y)
        self.body_z_deg += np.degrees(delta_z)

        # Save state for the next frame
        self.q_prev = q_curr

        return round(self.body_x_deg, 4) , round(self.body_y_deg, 4), round(self.body_z_deg, 4)

    def get_angles(self, line):
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
        angles = self.update(dt, gyro, accel, mag)

        # 3. Print or use your perfectly drift-corrected body-frame rotation
        return angles


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

            angles = tracker.get_angles(line)
            if angles is None:
                print("Failed to retrived angles")
                continue
            
            msg = f"{angles[0].item():.2f}, {angles[1].item():.2f}, {angles[2].item():.2f}"
            print(msg)

    except KeyboardInterrupt:
        print("\nTracking stopped.")


if __name__ == "__main__":
    poll_serial_port()
