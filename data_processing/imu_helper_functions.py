import math
import numpy as np
import json
import serial
import time
import os


GAUSS_TO_MILLI_TESLA_CONVERSION = 10
FILENAME = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../calibration/cal_data.json")
SERIAL_PORT = "/dev/cu.wchusbserial1330"
BAUD_RATE = 115200


def get_calibration_data(filename=FILENAME):
    """
    Read data from filename for IMU offset and scale data
    """
    with open(filename, "r", encoding="utf-8") as f:
        data = json.load(f)

    accOffset = data["accOffset"]
    accScale = data["accScale"]

    gOffset = data["gOffset"]

    magOffset = data["magOffset"]
    magScale = data["magScale"]

    return accOffset, accScale, gOffset, magOffset, magScale


def get_imu_data(values, calibration_data):
    """
    Read Raw Sensor Data and use calibrated values
    """
    accOffset, accScale, gOffset, magOffset, magScale = map(
        dict, calibration_data
    )
    ax, ay, az, gx, gy, gz, mx, my, mz = map(float, values)

    axCal = (ax - accOffset["x"]) * accScale["x"]
    ayCal = (ay - accOffset["y"]) * accScale["y"]
    azCal = (az - accOffset["z"]) * accScale["z"]

    gxCal = math.radians(gx - gOffset["x"])
    gyCal = math.radians(gy - gOffset["y"])
    gzCal = math.radians(gz - gOffset["z"])

    mxCal = ((mx - magOffset["x"]) * magScale["x"]) / GAUSS_TO_MILLI_TESLA_CONVERSION
    myCal = ((my - magOffset["y"]) * magScale["y"]) / GAUSS_TO_MILLI_TESLA_CONVERSION
    mzCal = ((mz - magOffset["z"]) * magScale["z"]) / GAUSS_TO_MILLI_TESLA_CONVERSION

    gyro_data = np.array([gxCal, gyCal, gzCal])
    acc_data = np.array([axCal, ayCal, azCal])
    mag_data = np.array([mxCal, myCal, mzCal])

    return gyro_data, acc_data, mag_data


def find_loop_frequency():
    prev_time = time.time()
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

    while True:

        line = ser.readline().decode("utf-8", errors="ignore").strip()

        if not line:
            continue

        current_time = time.time()
        dt = current_time - prev_time
        prev_time = current_time

        print(dt)