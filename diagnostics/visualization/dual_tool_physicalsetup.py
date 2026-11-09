
"""
Author: Joya
Version: 4.0

Description:
Live 3D visualization of two tools using IMU quaternions
and ToF distance measurements.

Each tool displays:
- Fixed RCM
- ToF sensor position
- Handle wall position
- Full tool shaft (wall to tip)
- Calculated tool tip
- Live ToF distance

All positions are in millimeters relative to Left RCM.
"""

import os
import sys
import numpy as np
import serial
import pyqtgraph as pg
import pyqtgraph.opengl as gl

from scipy.spatial.transform import Rotation
from PyQt5.QtWidgets import (
    QApplication,
    QWidget,
    QVBoxLayout,
    QLabel,
)
from PyQt5.QtCore import QThread, pyqtSignal, Qt

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

COUNT_TILL_SETTLED = 400

# Quaternion format returned by IMUQuaternionTracker
QUATERNION_ORDER = "wxyz"

# Set True if the quaternion represents world-to-sensor
# rather than sensor-to-world rotation.
INVERT_QUATERNION = False

# Mechanical geometry (mm)
TOOL_LENGTH = 285.5

LEFT_TOF_OFFSET = 37.743
RIGHT_TOF_OFFSET = 36.214

# Set to 1.0 if TOFManager returns millimeters.
# Set to 1000.0 if it returns meters.
TOF_TO_MM = 1.0

# Visualization
SHAFT_WIDTH = 5
TOF_LINE_WIDTH = 2
AXIS_VECTOR_LENGTH = 35.0


# ============================================================
# FIXED POSITIONS (LEFT RCM COORDINATE SYSTEM)
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
# QUATERNION AND GEOMETRY
# ============================================================

def quaternion_to_z_axis(q):
    """
    Convert IMU quaternion to local +Z direction
    expressed in the world coordinate system.
    """

    q = np.asarray(q, dtype=float).flatten()

    if q.size != 4:
        raise ValueError("Quaternion must have 4 elements")

    if QUATERNION_ORDER == "wxyz":
        q_scipy = [q[1], q[2], q[3], q[0]]

    elif QUATERNION_ORDER == "xyzw":
        q_scipy = q

    else:
        raise ValueError("Invalid quaternion order")

    rotation = Rotation.from_quat(q_scipy)

    if INVERT_QUATERNION:
        rotation = rotation.inv()

    z_axis = rotation.apply([0, 0, 1])

    return z_axis / np.linalg.norm(z_axis)


def calculate_tool_geometry(
    rcm,
    z_axis,
    tof_distance,
    tof_offset
):
    """
    Geometry:

    RCM -> ToF: local -Z, fixed offset
    ToF -> Wall: local -Z, measured distance
    Wall -> Tip: local +Z, 285.5 mm

    All positions returned in world coordinates.
    """

    rcm = np.asarray(rcm, dtype=float)
    z_axis = np.asarray(z_axis, dtype=float)

    # ToF sensor position
    tof_position = (
        rcm - tof_offset * z_axis
    )

    # Handle wall position
    wall_position = (
        tof_position - tof_distance * z_axis
    )

    # Tool tip position
    tip_position = (
        wall_position + TOOL_LENGTH * z_axis
    )

    return {
        "rcm": rcm,
        "tof": tof_position,
        "wall": wall_position,
        "tip": tip_position,
        "z_axis": z_axis,
        "distance": tof_distance,
        "tip_rcm_distance": (
            TOOL_LENGTH - tof_offset - tof_distance
        ),
    }


# ============================================================
# VISUALIZATION
# ============================================================

class VisualizeDualIMU(QWidget):

    def __init__(self):
        super().__init__()

        self.setWindowTitle(
            "Dual Tool Geometry - IMU + ToF"
        )

        self.resize(1400, 950)

        layout = QVBoxLayout(self)

        # ----------------------------------------------------
        # STATUS TEXT
        # ----------------------------------------------------

        self.status = QLabel(
            "Waiting for IMU and ToF data..."
        )

        self.status.setAlignment(Qt.AlignCenter)

        self.status.setStyleSheet(
            "font-size: 14px; padding: 8px;"
        )

        layout.addWidget(self.status)

        # ----------------------------------------------------
        # 3D VIEW
        # ----------------------------------------------------

        self.view = gl.GLViewWidget()
        layout.addWidget(self.view)

        self.view.setCameraPosition(
            pos=pg.Vector(55, -45, -40),
            distance=450,
            elevation=25,
            azimuth=45
        )

        # Grid
        grid = gl.GLGridItem()

        grid.setSize(300, 300)
        grid.setSpacing(25, 25)
        grid.translate(50, -45, -90)

        self.view.addItem(grid)

        # ----------------------------------------------------
        # REFERENCE POINTS
        # ----------------------------------------------------

        self.add_static_point(
            LEFT_RCM,
            (0, 1, 0, 1),
            13,
            "Left RCM"
        )

        self.add_static_point(
            RIGHT_RCM,
            (1, 0, 0, 1),
            13,
            "Right RCM"
        )

        self.add_static_point(
            CAMERA_LENS,
            (1, 1, 0, 1),
            12,
            "Camera Lens"
        )

        # ----------------------------------------------------
        # CALIBRATION POINTS
        # ----------------------------------------------------

        for name, position in CALIBRATION_POINTS.items():

            point = np.array(position, dtype=float)

            if name.startswith("R1"):
                color = (1, 0.5, 0, 1)
                size = 11
            else:
                color = (0, 0.7, 1, 1)
                size = 8

            self.add_static_point(
                point,
                color,
                size,
                name
            )

        # ----------------------------------------------------
        # CREATE DYNAMIC TOOL VISUALIZATIONS
        # ----------------------------------------------------

        self.left_tool = self.create_tool(
            color=(0, 1, 0, 1)
        )

        self.right_tool = self.create_tool(
            color=(1, 0, 0, 1)
        )

        # ----------------------------------------------------
        # SERIAL THREAD
        # ----------------------------------------------------

        self.data_thread = DualIMUData()

        self.data_thread.quaternion_data.connect(
            self.load_data
        )

        self.data_thread.start()

    # ========================================================
    # STATIC POINTS
    # ========================================================

    def add_static_point(
        self,
        position,
        color,
        size,
        label=None
    ):

        position = np.asarray(position, dtype=float)

        scatter = gl.GLScatterPlotItem(
            pos=position.reshape(1, 3),
            color=color,
            size=size,
            pxMode=True
        )

        self.view.addItem(scatter)

        if label is not None:

            text = gl.GLTextItem(
                pos=position,
                text=label,
                color=(255, 255, 255, 255)
            )

            self.view.addItem(text)

    # ========================================================
    # CREATE DYNAMIC TOOL
    # ========================================================

    def create_tool(self, color):

        zero_line = np.zeros((2, 3))
        zero_point = np.zeros((1, 3))

        # Full shaft: wall -> tip
        shaft = gl.GLLinePlotItem(
            pos=zero_line.copy(),
            color=color,
            width=SHAFT_WIDTH,
            antialias=True
        )

        # ToF beam: sensor -> wall
        tof_beam = gl.GLLinePlotItem(
            pos=zero_line.copy(),
            color=(1, 1, 0, 1),
            width=TOF_LINE_WIDTH,
            antialias=True
        )

        # +Z direction from RCM
        z_vector = gl.GLLinePlotItem(
            pos=zero_line.copy(),
            color=color,
            width=2,
            antialias=True
        )

        # Tool tip
        tip_marker = gl.GLScatterPlotItem(
            pos=zero_point.copy(),
            color=(1, 1, 1, 1),
            size=13,
            pxMode=True
        )

        # ToF sensor
        tof_marker = gl.GLScatterPlotItem(
            pos=zero_point.copy(),
            color=(1, 1, 0, 1),
            size=11,
            pxMode=True
        )

        # Handle wall
        wall_marker = gl.GLScatterPlotItem(
            pos=zero_point.copy(),
            color=(1, 0.5, 0, 1),
            size=12,
            pxMode=True
        )

        # Add to 3D view
        for item in [
            shaft,
            tof_beam,
            z_vector,
            tip_marker,
            tof_marker,
            wall_marker
        ]:
            self.view.addItem(item)

        return {
            "shaft": shaft,
            "tof_beam": tof_beam,
            "z_vector": z_vector,
            "tip_marker": tip_marker,
            "tof_marker": tof_marker,
            "wall_marker": wall_marker,
        }

    # ========================================================
    # UPDATE TOOL GEOMETRY
    # ========================================================

    def update_tool(self, tool, geometry):

        rcm = geometry["rcm"]
        tof = geometry["tof"]
        wall = geometry["wall"]
        tip = geometry["tip"]
        z_axis = geometry["z_axis"]

        # Full physical shaft
        tool["shaft"].setData(
            pos=np.vstack([wall, tip])
        )

        # ToF measurement beam
        tool["tof_beam"].setData(
            pos=np.vstack([tof, wall])
        )

        # Local +Z direction
        tool["z_vector"].setData(
            pos=np.vstack([
                rcm,
                rcm + AXIS_VECTOR_LENGTH * z_axis
            ])
        )

        # Markers
        tool["tip_marker"].setData(
            pos=tip.reshape(1, 3)
        )

        tool["tof_marker"].setData(
            pos=tof.reshape(1, 3)
        )

        tool["wall_marker"].setData(
            pos=wall.reshape(1, 3)
        )

    # ========================================================
    # LIVE DATA UPDATE
    # ========================================================

    def load_data(self, data):

        left_q, left_distance, right_q, right_distance = data

        try:

            left_distance = (
                float(left_distance) * TOF_TO_MM
            )

            right_distance = (
                float(right_distance) * TOF_TO_MM
            )

            if (
                not np.isfinite(left_distance)
                or not np.isfinite(right_distance)
                or left_distance < 0
                or right_distance < 0
            ):
                return

            # Quaternion -> +Z direction
            left_z = quaternion_to_z_axis(left_q)
            right_z = quaternion_to_z_axis(right_q)

            # Left tool geometry
            left_geometry = calculate_tool_geometry(
                rcm=LEFT_RCM,
                z_axis=left_z,
                tof_distance=left_distance,
                tof_offset=LEFT_TOF_OFFSET
            )

            # Right tool geometry
            right_geometry = calculate_tool_geometry(
                rcm=RIGHT_RCM,
                z_axis=right_z,
                tof_distance=right_distance,
                tof_offset=RIGHT_TOF_OFFSET
            )

            # Update both tools
            self.update_tool(
                self.left_tool,
                left_geometry
            )

            self.update_tool(
                self.right_tool,
                right_geometry
            )

            # Live status
            self.status.setText(
                f"LEFT  |  ToF: {left_distance:.1f} mm"
                f"  |  Tip-RCM axial: "
                f"{left_geometry['tip_rcm_distance']:.1f} mm"
                f"  |  Tip: "
                f"{np.round(left_geometry['tip'], 1)}"
                "\n"
                f"RIGHT |  ToF: {right_distance:.1f} mm"
                f"  |  Tip-RCM axial: "
                f"{right_geometry['tip_rcm_distance']:.1f} mm"
                f"  |  Tip: "
                f"{np.round(right_geometry['tip'], 1)}"
            )

        except (ValueError, TypeError) as e:
            print(f"Visualization update error: {e}")

    # ========================================================
    # CLEAN SHUTDOWN
    # ========================================================

    def closeEvent(self, event):

        self.data_thread.stop()
        self.data_thread.wait()

        super().closeEvent(event)


# ============================================================
# SERIAL DATA THREAD
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
                f"Failed to connect to {SERIAL_PORT}: {e}"
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

                    if len(line) < 23:
                        continue

                except Exception as e:

                    print(f"Serial error: {e}")
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

                # Left quaternion
                left_q = left_tracker.get_quaternion(
                    left_imu_values
                )

                if left_q is None:
                    continue

                # Right quaternion
                right_q = right_tracker.get_quaternion(
                    right_imu_values
                )

                if right_q is None:
                    continue

                # ToF measurements
                try:

                    distances = list(
                        tof_manager.get_distances(
                            tof_values
                        )
                    )

                    if len(distances) < 2:
                        continue

                except Exception as e:

                    print(f"ToF error: {e}")
                    continue

                self.quaternion_data.emit([
                    left_q,
                    distances[0],
                    right_q,
                    distances[1]
                ])

        finally:

            ser.close()
            print("Serial connection closed")


# ============================================================
# MAIN
# ============================================================

if __name__ == "__main__":

    app = QApplication(sys.argv)

    window = VisualizeDualIMU()
    window.show()

    sys.exit(app.exec())
