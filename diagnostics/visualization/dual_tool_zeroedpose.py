
"""
Author: Joya
Version: 4.4 — compact live data + 3D plot

Description:
Live dual-tool 3D visualization using IMU quaternions
and ToF distance measurements.

Version 4.4:
- GUI contains only live data/status and the 3D plot (no control panel)
- Inverse relative rotation by default to test reversed movement
- Press I to toggle relative-rotation inversion
- Press M to switch world/local relative-rotation composition
- Left tool tip shortened by 15.6 mm
- Right tool tip shortened by 21.2 mm
- Corrections applied along each tool's local -Z axis
- Press Z to align tool directions with R1C1 and R1C7
- No data is saved
"""

import os
import sys
import numpy as np
import serial
import pyqtgraph as pg
import pyqtgraph.opengl as gl

from scipy.spatial.transform import Rotation
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel
from PyQt5.QtCore import QThread, pyqtSignal, Qt
import time

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

QUATERNION_ORDER = "wxyz"
INVERT_QUATERNION = False
MAX_ZERO_SAMPLE_AGE_S = 0.5

# Original geometry (mm)
TOOL_LENGTH = 285.5

LEFT_TOF_OFFSET = 37.743
RIGHT_TOF_OFFSET = 36.214

# NEW: tip corrections (mm)
LEFT_TIP_CORRECTION = 15.6
RIGHT_TIP_CORRECTION = 21.2

# Effective lengths
LEFT_TOOL_LENGTH = TOOL_LENGTH - LEFT_TIP_CORRECTION
RIGHT_TOOL_LENGTH = TOOL_LENGTH - RIGHT_TIP_CORRECTION

TOF_TO_MM = 1.0

SHAFT_WIDTH = 5
TOF_LINE_WIDTH = 2
AXIS_VECTOR_LENGTH = 35.0


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
    "R6C4": (72.268 + 30, -50.541, -70.031),
    "R6C6": (72.268 + 30, -75.541, -70.031),
    "R7C2": (112.268, -12.541, -70.031),
    "R7C3": (112.268, -32.541, -70.031),
    "R8C5": (117.268, -63.041, -70.031),
}


# ============================================================
# QUATERNION AND TOOL GEOMETRY
# ============================================================

def as_rotation(q):
    """Return sensor-to-world Rotation, using the existing quaternion convention."""
    q = np.asarray(q, dtype=float).flatten()
    if q.size != 4 or not np.all(np.isfinite(q)) or np.linalg.norm(q) < 1e-10:
        raise ValueError("Invalid quaternion")
    q = q / np.linalg.norm(q)
    if QUATERNION_ORDER == "wxyz":
        q = [q[1], q[2], q[3], q[0]]
    elif QUATERNION_ORDER != "xyzw":
        raise ValueError("Invalid quaternion order")
    rot = Rotation.from_quat(q)
    return rot.inv() if INVERT_QUATERNION else rot


def target_rotation(rcm, target):
    """Minimum-angle rotation taking local +Z to RCM -> target."""
    direction = np.asarray(target, dtype=float) - np.asarray(rcm, dtype=float)
    direction /= np.linalg.norm(direction)
    z = np.array([0., 0., 1.])
    dot = np.clip(np.dot(z, direction), -1., 1.)
    if dot > 1.0 - 1e-12:
        return Rotation.identity()
    if dot < -1.0 + 1e-12:
        return Rotation.from_rotvec(np.pi * np.array([1., 0., 0.]))
    axis = np.cross(z, direction)
    axis /= np.linalg.norm(axis)
    return Rotation.from_rotvec(np.arccos(dot) * axis)


def calibrated_z_axis(raw_q, zero_rotation, reference_rotation, inverse_delta=True, frame="world"):
    """Map the Z-press orientation to the known target and apply live changes.

    The inverse-delta default tests the observed opposite movement. It does
    not establish the full IMU-to-pegboard frame registration. Toggle with I.
    Frame mode can be changed with M to test world versus local composition.
    """
    raw_rotation = as_rotation(raw_q)
    if frame == "world":
        delta = raw_rotation * zero_rotation.inv()
        if inverse_delta:
            delta = delta.inv()
        calibrated = delta * reference_rotation
    elif frame == "local":
        delta = zero_rotation.inv() * raw_rotation
        if inverse_delta:
            delta = delta.inv()
        calibrated = reference_rotation * delta
    else:
        raise ValueError("Unknown rotation frame")
    z_axis = calibrated.apply([0., 0., 1.])
    return z_axis / np.linalg.norm(z_axis)


def quaternion_to_z_axis(q):

    q = np.asarray(q, dtype=float).flatten()

    if q.size != 4:
        raise ValueError("Quaternion must contain 4 elements")

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
    tof_offset,
    tip_correction
):
    """
    RCM -> ToF: -Z
    ToF -> Wall: -Z
    Wall -> Tip: +Z

    Tip correction shortens wall-to-tip length.
    """

    rcm = np.asarray(rcm, dtype=float)
    z_axis = np.asarray(z_axis, dtype=float)

    tof_position = rcm - tof_offset * z_axis

    wall_position = (
        tof_position - tof_distance * z_axis
    )

    effective_length = TOOL_LENGTH - tip_correction

    tip_position = (
        wall_position + effective_length * z_axis
    )

    return {
        "rcm": rcm,
        "tof": tof_position,
        "wall": wall_position,
        "tip": tip_position,
        "z_axis": z_axis,
        "distance": tof_distance,
        "effective_length": effective_length,
        "tip_rcm_distance": (
            effective_length - tof_offset - tof_distance
        ),
    }


# ============================================================
# VISUALIZATION
# ============================================================

class VisualizeDualIMU(QWidget):

    def __init__(self):
        super().__init__()

        self.setWindowTitle(
            "Dual Tool Geometry - Z-Calibrated IMU + ToF (v4.4)"
        )

        self.resize(1400, 950)

        layout = QVBoxLayout(self)

        self.status = QLabel(
            "Waiting for IMU and ToF data..."
        )

        self.status.setAlignment(Qt.AlignCenter)
        self.status.setStyleSheet(
            "font-size: 14px; padding: 8px;"
        )

        layout.addWidget(self.status)

        # Keyboard shortcuts remain active; no separate controls panel.
        self.setFocusPolicy(Qt.StrongFocus)
        self.setFocus()

        self.latest_sample = None
        self.zero_rotations = None
        self.inverse_delta = True
        self.rotation_frame = "world"
        self.reference_rotations = {
            "left": target_rotation(LEFT_RCM, CALIBRATION_POINTS["R1C1"]),
            "right": target_rotation(RIGHT_RCM, CALIBRATION_POINTS["R1C7"]),
        }

        # Shared 3D view
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

        # RCM and camera
        self.add_static_point(
            LEFT_RCM, (0, 1, 0, 1), 13, "Left RCM"
        )

        self.add_static_point(
            RIGHT_RCM, (1, 0, 0, 1), 13, "Right RCM"
        )

        self.add_static_point(
            CAMERA_LENS, (1, 1, 0, 1), 12, "Camera Lens"
        )

        # Calibration points
        for name, position in CALIBRATION_POINTS.items():

            point = np.array(position, dtype=float)

            if name.startswith("R1"):
                color = (1, 0.5, 0, 1)
                size = 11
            else:
                color = (0, 0.7, 1, 1)
                size = 8

            self.add_static_point(
                point, color, size, name
            )

        # Dynamic tools
        self.left_tool = self.create_tool(
            color=(0, 1, 0, 1)
        )

        self.right_tool = self.create_tool(
            color=(1, 0, 0, 1)
        )

        # Serial thread
        self.data_thread = DualIMUData()
        self.data_thread.quaternion_data.connect(
            self.load_data
        )
        self.data_thread.start()
        self.status.setText("LIVE | Point LEFT at R1C1 and RIGHT at R1C7, then press Z.  I: invert motion  |  M: frame")

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Z and not event.isAutoRepeat():
            self.zero_now()
            event.accept()
        elif event.key() == Qt.Key_I and not event.isAutoRepeat():
            self.inverse_delta = not self.inverse_delta
            self.refresh_latest()
            event.accept()
        elif event.key() == Qt.Key_M and not event.isAutoRepeat():
            self.rotation_frame = "local" if self.rotation_frame == "world" else "world"
            self.refresh_latest()
            event.accept()
        else:
            super().keyPressEvent(event)

    def refresh_latest(self):
        if self.zero_rotations is not None and self.latest_sample is not None:
            _, lq, ld, rq, rd = self.latest_sample
            self.update_calibrated_tools(lq, ld, rq, rd)

    def zero_now(self):
        if self.latest_sample is None:
            self.status.setText("No valid IMU/ToF sample yet; cannot zero.")
            return
        stamp, left_q, left_distance, right_q, right_distance = self.latest_sample
        age = time.monotonic() - stamp
        if age > MAX_ZERO_SAMPLE_AGE_S:
            self.status.setText(f"Sample stale ({age:.2f} s); zero cancelled.")
            return
        try:
            left_zero = as_rotation(left_q)
            right_zero = as_rotation(right_q)
        except (ValueError, TypeError) as exc:
            self.status.setText(f"Invalid zero sample: {exc}")
            return
        self.zero_rotations = {"left": left_zero, "right": right_zero}
        self.update_calibrated_tools(left_q, left_distance, right_q, right_distance)

    # ========================================================
    # STATIC POINT
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
    # CREATE TOOL
    # ========================================================

    def create_tool(self, color):

        # Hide dynamic geometry until Z is pressed.
        zero_line = np.full((2, 3), np.nan)
        zero_point = np.full((1, 3), np.nan)

        shaft = gl.GLLinePlotItem(
            pos=zero_line.copy(),
            color=color,
            width=SHAFT_WIDTH,
            antialias=True
        )

        tof_beam = gl.GLLinePlotItem(
            pos=zero_line.copy(),
            color=(1, 1, 0, 1),
            width=TOF_LINE_WIDTH,
            antialias=True
        )

        z_vector = gl.GLLinePlotItem(
            pos=zero_line.copy(),
            color=color,
            width=2,
            antialias=True
        )

        tip_marker = gl.GLScatterPlotItem(
            pos=zero_point.copy(),
            color=(1, 1, 1, 1),
            size=13,
            pxMode=True
        )

        tof_marker = gl.GLScatterPlotItem(
            pos=zero_point.copy(),
            color=(1, 1, 0, 1),
            size=11,
            pxMode=True
        )

        wall_marker = gl.GLScatterPlotItem(
            pos=zero_point.copy(),
            color=(1, 0.5, 0, 1),
            size=12,
            pxMode=True
        )

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
    # UPDATE TOOL
    # ========================================================

    def update_tool(self, tool, geometry):

        rcm = geometry["rcm"]
        tof = geometry["tof"]
        wall = geometry["wall"]
        tip = geometry["tip"]
        z_axis = geometry["z_axis"]

        tool["shaft"].setData(
            pos=np.vstack([wall, tip])
        )

        tool["tof_beam"].setData(
            pos=np.vstack([tof, wall])
        )

        tool["z_vector"].setData(
            pos=np.vstack([
                rcm,
                rcm + AXIS_VECTOR_LENGTH * z_axis
            ])
        )

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
    # LIVE UPDATE
    # ========================================================

    def load_data(self, data):
        try:
            left_q, left_distance, right_q, right_distance = data
            left_distance = float(left_distance) * TOF_TO_MM
            right_distance = float(right_distance) * TOF_TO_MM
            if (not np.isfinite([left_distance, right_distance]).all()
                    or left_distance < 0 or right_distance < 0):
                return
            # Validate rotations before accepting the sample for calibration.
            as_rotation(left_q)
            as_rotation(right_q)
            self.latest_sample = (
                time.monotonic(), np.asarray(left_q, dtype=float).copy(),
                left_distance, np.asarray(right_q, dtype=float).copy(), right_distance
            )
            if self.zero_rotations is None:
                self.status.setText(
                    f"LIVE (not zeroed) | LEFT ToF: {left_distance:.1f} mm  |  "
                    f"RIGHT ToF: {right_distance:.1f} mm\n"
                    "Point LEFT at R1C1 and RIGHT at R1C7; press Z to start plotting."
                )
                return  # Fixed reference points remain visible; tools wait for Z.
            self.update_calibrated_tools(left_q, left_distance, right_q, right_distance)
        except (ValueError, TypeError) as exc:
            print(f"Visualization error: {exc}")

    def update_calibrated_tools(self, left_q, left_distance, right_q, right_distance):
        left_z = calibrated_z_axis(
            left_q, self.zero_rotations["left"], self.reference_rotations["left"],
            self.inverse_delta, self.rotation_frame
        )
        right_z = calibrated_z_axis(
            right_q, self.zero_rotations["right"], self.reference_rotations["right"],
            self.inverse_delta, self.rotation_frame
        )
        left_geometry = calculate_tool_geometry(
            rcm=LEFT_RCM, z_axis=left_z, tof_distance=left_distance,
            tof_offset=LEFT_TOF_OFFSET, tip_correction=LEFT_TIP_CORRECTION
        )
        right_geometry = calculate_tool_geometry(
            rcm=RIGHT_RCM, z_axis=right_z, tof_distance=right_distance,
            tof_offset=RIGHT_TOF_OFFSET, tip_correction=RIGHT_TIP_CORRECTION
        )
        self.update_tool(self.left_tool, left_geometry)
        self.update_tool(self.right_tool, right_geometry)
        self.status.setText(
            f"LIVE / ZEROED [{self.rotation_frame}, inverse={'ON' if self.inverse_delta else 'OFF'}] "
            f"| LEFT: ToF {left_distance:.1f} mm | length {LEFT_TOOL_LENGTH:.1f} mm "
            f"| tip {np.round(left_geometry['tip'], 1)}\n"
            f"ZEROED | RIGHT: ToF {right_distance:.1f} mm | length {RIGHT_TOOL_LENGTH:.1f} mm "
            f"| tip {np.round(right_geometry['tip'], 1)}"
        )

    # ========================================================
    # SHUTDOWN
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
            print(f"Failed to connect to {SERIAL_PORT}: {e}")
            return

        left_tracker = IMUQuaternionTracker("left", use_offset=False)
        right_tracker = IMUQuaternionTracker("right", use_offset=False)

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

                left_q = left_tracker.get_quaternion(
                    left_imu_values
                )

                if left_q is None:
                    continue

                right_q = right_tracker.get_quaternion(
                    right_imu_values
                )

                if right_q is None:
                    continue

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
