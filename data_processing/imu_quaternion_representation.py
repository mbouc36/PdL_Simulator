import serial
import numpy as np
import math
from ahrs.filters.mahony import Mahony
from scipy.spatial.transform import Rotation as R
from PyQt5 import QtWidgets, QtCore
import os
import sys
from time import sleep
from threading import Thread

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from visualization.triangular_roll_pitch_yaw import AttitudeVisualizer

# Offsets and Scale constants
accOffset = {}
accScale = {}
magOffset = {}
magScale = {}

accOffset["x"] = 125.13895887416402
accOffset["y"] = -335.94039488277485
accOffset["z"] = 270.7634645447579
accScale["x"] = 6.013120690784818e-05
accScale["y"] = 6.013120690784818e-05
accScale["z"] = 6.013120690784818e-05
gxOffset = -0.29179999999999995
gyOffset = 0.10055
gzOffset = -0.3958
magOffset["x"] = -4024.256778670425
magOffset["y"] = 1972.9403410143923
magOffset["z"] = 801.4448991733041
magScale["x"] = 0.0002805069537402399
magScale["y"] = 0.0002458879135635146
magScale["z"] = 0.0002821867086059957

SERIAL_PORT = "/dev/cu.wchusbserial1330"
BAUD_RATE = 115200
FREQUENCY = 200.0
K_P = 1.0
K_I = 0.01
GAUSS_TO_MILLI_TESLA_CONVERSION = 10  
PRINT_FREQUENCY = 100
roll = 0
pitch = 0
yaw = 0 
# Shared attitude state
attitude = {
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
}


def imu_loop(r_p_y = False):
    global attitude

    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    except Exception:
        print(f"Failed to connect to port: {SERIAL_PORT}")
        return

    mahony_filter = Mahony(frequency=FREQUENCY, k_P=K_P, k_I=K_I)
    q = np.array([1.0, 0.0, 0.0, 0.0])

    # Set the shaft to the x axis of the IMU
    shaft_axis_local = np.array([1.0, 0.0, 0.0])
    q_ref = q.copy()
    print_count = 0

    while True:
        try:
            line = ser.readline().decode("utf-8").strip()
            values = line.split(",")

            if len(values) != 9:
                continue

            ax, ay, az, gx, gy, gz, mx, my, mz = map(float, values)

            axCal = (ax - accOffset["x"]) * accScale["x"]
            ayCal = (ay - accOffset["y"]) * accScale["y"]
            azCal = (az - accOffset["z"]) * accScale["z"]

            gxCal = math.radians(gx - gxOffset)
            gyCal = math.radians(gy - gyOffset)
            gzCal = math.radians(gz - gzOffset)

            mxCal = ((mx - magOffset["x"]) * magScale["x"]) / GAUSS_TO_MILLI_TESLA_CONVERSION
            myCal = ((my - magOffset["y"]) * magScale["y"]) / GAUSS_TO_MILLI_TESLA_CONVERSION
            mzCal = ((mz - magOffset["z"]) * magScale["z"]) / GAUSS_TO_MILLI_TESLA_CONVERSION

            gyro_data = np.array([gxCal, gyCal, gzCal])
            acc_data = np.array([axCal, ayCal, azCal])
            mag_data = np.array([mxCal, myCal, mzCal])

            q = mahony_filter.updateMARG(q, gyr=gyro_data, acc=acc_data, mag=mag_data)
            q = q / np.linalg.norm(q)

            r = R.from_quat([q[1], q[2], q[3], q[0]])
            if (r_p_y):
                roll, pitch, yaw = r.as_euler("xyz", degrees=True)

                attitude["roll"] = roll
                attitude["pitch"] = pitch
                attitude["yaw"] = yaw

                print(roll, pitch, yaw)
            else:
                print_count += 1
                if print_count == PRINT_FREQUENCY:
                    shaft_angle = shaft_twist_angle(q_ref, q, shaft_axis_local)
                    print("Shaft twist:", shaft_angle)
                    print_count = 0


        except Exception as e:
            print(f"Invalid raw data: {e}")


def shaft_twist_angle(q_ref, q_cur, shaft_axis_local):
    r_ref = R.from_quat([q_ref[1], q_ref[2], q_ref[3], q_ref[0]])
    r_cur = R.from_quat([q_cur[1], q_cur[2], q_cur[3], q_cur[0]])

    # Rotation from reference orientation to current orientation
    r_delta = r_ref.inv() * r_cur

    # Express the shaft axis in the reference/global frame
    shaft_axis_global = r_ref.apply(shaft_axis_local)
    shaft_axis_global = shaft_axis_global / np.linalg.norm(shaft_axis_global)

    # Delta rotation vector in global/reference frame
    rotvec = r_delta.as_rotvec()

    # Project rotation onto shaft axis
    twist_rad = np.dot(rotvec, shaft_axis_global)

    return math.degrees(twist_rad)

def visualize_roll_pitch_yaw():
    app = QtWidgets.QApplication(sys.argv)

    visualizer = AttitudeVisualizer()
    visualizer.show()

    def update_data():
        visualizer.update_data(
            0,
            attitude["roll"],
            attitude["pitch"],
            attitude["yaw"],
        )

    timer = QtCore.QTimer()
    timer.timeout.connect(update_data)
    timer.start(20)

    sys.exit(app.exec_())


def visualize_quaternion():
    pass


def main():
    imu_thread = Thread(target=imu_loop, daemon=True)
    imu_thread.start()

    visualize_roll_pitch_yaw()


if __name__ == "__main__":
    main()