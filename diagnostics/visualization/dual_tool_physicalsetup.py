"""
Author: Michael Boucouvalas
Date: 2026, Aug 26th
Version: 3.1

Description:
Visualize the local +Z direction of two IMUs in one 3D plot,
along with the RCMs, camera lens, and calibration positions.
All coordinates are relative to the Left RCM.
"""

import os
import sys
import numpy as np
import serial
import pyqtgraph as pg
import pyqtgraph.opengl as gl

from scipy.spatial.transform import Rotation
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout
from PyQt5.QtCore import QThread, pyqtSignal

sys.path.append(
    os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "../.."
    )
)

from update_config import load_config
from tools.imu_orientation import IMUQuaternionTracker
from tools.tof_manager import TOFManager


# ============================================================
# CONFIGURATION
# ============================================================

config = load_config()

SERIAL_PORT = config["serial_port"]
BAUD_RATE = config["baud_rate"]

NORTH_OFFSET = 180
COUNT_TILL_SETTLED = 400

VECTOR_LENGTH = 45.0
VECTOR_WIDTH = 4

# Set to "wxyz" or "xyzw" depending on your tracker output.
QUATERNION_ORDER = "wxyz"


# ============================================================
# FIXED POSITIONS RELATIVE TO LEFT RCM
# ============================================================

LEFT_RCM = np.array([0.000, 0.000, 0.000])
RIGHT_RCM = np.array([-1.880, -93.263, -0.869])
CAMERA_LENS = np.array([13.355, -45.835, 2.959])

CALIBRATION_POINTS = {
    "R1C1": (53.518, -9.291, -87.031),
    "R1C7": (53.518, -83.791, -87.031),
    "R2C5": (57.268, -63.041, -70.031),
    "R3C2": (62.268, -12.541, -70.031),
    "R3C3": (62.268, -32.541, -70.031),
    "R4C4": (72.268, -50.541, -70.031),
    "R4C6": (72.268, -75.541, -70.031),
    "R5C2": (87.268, -12.541, -70.031),
    "R5C3": (87.268, -32.541, -70.031),
    "R6C4": (102.268, -50.541, -70.031),
    "R6C6": (102.268, -75.541, -70.031),
    "R7C2": (112.268, -12.541, -70.031),
    "R7C3": (112.268, -32.541, -70.031),
    "R8C5": (117.268, -63.041, -70.031),
}


# ============================================================
# VISUALIZATION
# ============================================================

class VisualizeDualIMU(QWidget):

    def __init__(self):
        super().__init__()

        self.setWindowTitle("Dual IMU Z-Axis Visualization")
        self.resize(1300, 900)

        layout = QVBoxLayout(self)

        # Single shared 3D plot
        self.view = gl.GLViewWidget()
        layout.addWidget(self.view)

        # FIX: Use pg.Vector instead of gl.Vector
        self.view.setCameraPosition(
            pos=pg.Vector(55, -45, -40),
            distance=270,
            elevation=25,
            azimuth=45
        )

        # ----------------------------------------------------
        # GRID
        # ----------------------------------------------------

        grid = gl.GLGridItem()
        grid.setSize(160, 160)
        grid.setSpacing(20, 20)
        grid.translate(55, -45, -90)

        self.view.addItem(grid)

        # ----------------------------------------------------
        # FIXED REFERENCE POINTS
        # ----------------------------------------------------

        self.add_point(
            LEFT_RCM,
            color=(0, 1, 0, 1),
            size=12
        )

        self.add_point(
            RIGHT_RCM,
            color=(1, 0, 0, 1),
            size=12
        )

        self.add_point(
            CAMERA_LENS,
            color=(1, 1, 0, 1),
            size=12
        )

        self.add_label(LEFT_RCM, "Left RCM")
        self.add_label(RIGHT_RCM, "Right RCM")
        self.add_label(CAMERA_LENS, "Camera Lens")

        # ----------------------------------------------------
        # CALIBRATION POINTS
        # ----------------------------------------------------

        for name, position in CALIBRATION_POINTS.items():

            point = np.array(position, dtype=float)

            if name.startswith("R1"):
                # R1 calibration points: orange
                color = (1, 0.5, 0, 1)
                size = 11
            else:
                # Other calibration points: blue
                color = (0, 0.7, 1, 1)
                size = 8

            self.add_point(point, color, size)
            self.add_label(point, name)

        # ----------------------------------------------------
        # DYNAMIC IMU +Z VECTORS
        # ----------------------------------------------------

        self.left_vector = gl.GLLinePlotItem(
            pos=np.array([
                LEFT_RCM,
                LEFT_RCM + np.array([0, 0, VECTOR_LENGTH])
            ]),
            color=(0, 1, 0, 1),
            width=VECTOR_WIDTH,
            antialias=True
        )

        self.right_vector = gl.GLLinePlotItem(
            pos=np.array([
                RIGHT_RCM,
                RIGHT_RCM + np.array([0, 0, VECTOR_LENGTH])
            ]),
            color=(1, 0, 0, 1),
            width=VECTOR_WIDTH,
            antialias=True
        )

        self.view.addItem(self.left_vector)
        self.view.addItem(self.right_vector)

        # ----------------------------------------------------
        # SERIAL DATA THREAD
        # ----------------------------------------------------

        self.data_thread = DualIMUData()
        self.data_thread.quaternion_data.connect(self.load_data)
        self.data_thread.start()

    # ========================================================
    # ADD FIXED POINT
    # ========================================================

    def add_point(self, position, color, size):

        scatter = gl.GLScatterPlotItem(
            pos=np.array([position], dtype=float),
            color=color,
            size=size,
            pxMode=True
        )

        self.view.addItem(scatter)

    # ========================================================
    # ADD TEXT LABEL
    # ========================================================

    def add_label(self, position, text):

        label = gl.GLTextItem(
            pos=np.asarray(position, dtype=float),
            text=text,
            color=(255, 255, 255, 255)
        )

        self.view.addItem(label)

    # ========================================================
    # EXTRACT IMU LOCAL +Z DIRECTION
    # ========================================================

    @staticmethod
    def quaternion_to_z_axis(q):

        q = np.asarray(q, dtype=float).flatten()

        if q.size != 4:
            raise ValueError(
                f"Expected 4 quaternion components, got {q.size}"
            )

        if QUATERNION_ORDER == "wxyz":
            q_scipy = [q[1], q[2], q[3], q[0]]
        elif QUATERNION_ORDER == "xyzw":
            q_scipy = q
        else:
            raise ValueError(
                "QUATERNION_ORDER must be 'wxyz' or 'xyzw'"
            )

        rotation = Rotation.from_quat(q_scipy)

        # Local IMU +Z expressed in world coordinates
        z_axis = rotation.apply([0, 0, 1])

        return z_axis / np.linalg.norm(z_axis)

    # ========================================================
    # UPDATE BOTH IMU VECTORS
    # ========================================================

    def load_data(self, data):

        left_q, left_distance, right_q, right_distance = data

        try:
            left_z = self.quaternion_to_z_axis(left_q)
            right_z = self.quaternion_to_z_axis(right_q)

        except (ValueError, TypeError) as e:
            print(f"Quaternion conversion error: {e}")
            return

        # Vectors originate at their respective RCMs
        left_endpoint = LEFT_RCM + VECTOR_LENGTH * left_z
        right_endpoint = RIGHT_RCM + VECTOR_LENGTH * right_z

        self.left_vector.setData(
            pos=np.array([
                LEFT_RCM,
                left_endpoint
            ])
        )

        self.right_vector.setData(
            pos=np.array([
                RIGHT_RCM,
                right_endpoint
            ])
        )

    # ========================================================
    # CLEAN SHUTDOWN
    # ========================================================

    def closeEvent(self, event):

        self.data_thread.stop()
        self.data_thread.wait()

        super().closeEvent(event)


# ============================================================
# SERIAL DATA ACQUISITION
# ============================================================

class DualIMUData(QThread):

    quaternion_data = pyqtSignal(list)

    def __init__(self):
        super().__init__()
        self.running = True

    def stop(self):
        self.running = False

    def run(self):

        try:
            ser = serial.Serial(
                SERIAL_PORT,
                BAUD_RATE,
                timeout=1
            )

        except Exception as e:
            print(
                f"Failed to connect to port {SERIAL_PORT}: {e}"
            )
            return

        left_tracker = IMUQuaternionTracker("left")
        right_tracker = IMUQuaternionTracker("right")

        tof_manager = TOFManager()

        iter_counter = 0

        try:
            while self.running:

                try:
                    raw_line = ser.readline().decode(
                        "utf-8"
                    ).strip()

                    if not raw_line:
                        continue

                    line = raw_line.split(",")

                    # Timestamp + fields through both IMUs
                    if len(line) < 23:
                        continue

                except Exception as e:
                    print(f"Failed to read line: {e}")
                    continue

                iter_counter += 1

                if iter_counter == COUNT_TILL_SETTLED:

                    print("Gain updated")

                    left_tracker.set_gain(0.2)
                    right_tracker.set_gain(0.2)

                arduino_time = line[0]

                left_imu_values = (
                    [arduino_time] + line[5:14]
                )

                right_imu_values = (
                    [arduino_time] + line[14:]
                )

                tof_values = line[3:5]

                # Left IMU quaternion
                left_q = left_tracker.get_quaternion(
                    left_imu_values
                )

                if left_q is None:
                    print("Failed to retrieve left quaternion")
                    continue

                # Right IMU quaternion
                right_q = right_tracker.get_quaternion(
                    right_imu_values
                )

                if right_q is None:
                    print("Failed to retrieve right quaternion")
                    continue

                # ToF distances
                try:
                    distances = list(
                        tof_manager.get_distances(tof_values)
                    )

                    if len(distances) < 2:
                        continue

                except Exception as e:
                    print(f"ToF error: {e}")
                    continue

                data = [
                    left_q,
                    distances[0],
                    right_q,
                    distances[1]
                ]

                self.quaternion_data.emit(data)

        finally:
            ser.close()
            print("Serial connection closed.")


# ============================================================
# MAIN
# ============================================================

if __name__ == "__main__":

    app = QApplication(sys.argv)

    window = VisualizeDualIMU()
    window.show()

    sys.exit(app.exec())