"""Dual IMU pegboard quaternion / ToF verification (v1.1: fixed zero-first).

WRF: origin at left RCM; +X forward, +Y left, +Z up.
Fusion input: +X right, +Y forward, +Z up.

At zero: local IMU +Z is RCM->hole, +X is projected world-up
(tick mark down), +Y = Z cross X. This follows v4.2i.

Expected quaternion is the IMU-to-WRF reference quaternion, NOT a
prediction of the uncalibrated AHRS quaternion. Press Z with LEFT at R1C1 and RIGHT at R1C2, tick marks down.
The reference holes are FIXED; dropdowns only select verification holes.
Changing dropdowns never changes the stored zero orientation.
"""
import os
import sys
import time
import numpy as np
import serial
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from scipy.spatial.transform import Rotation
from PyQt5.QtCore import Qt, QThread, pyqtSignal
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout,
                             QLabel, QPushButton, QComboBox, QGroupBox, QGridLayout)

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../..'))
from update_config import load_config
from tools.imu_orientation import IMUQuaternionTracker
from tools.tof_manager import TOFManager

config = load_config()
SERIAL_PORT = config['serial_port']
BAUD_RATE = config['baud_rate']
COUNT_TILL_SETTLED = 400
QUATERNION_ORDER = 'wxyz'
INVERT_QUATERNION = False
MAX_ZERO_SAMPLE_AGE_S = 0.5
TOOL_LENGTH_MM = 285.5  # wall to tip
LEFT_TOF_OFFSET_MM = 37.743  # RCM to ToF, opposite +shaft
RIGHT_TOF_OFFSET_MM = 36.214
TOF_TO_MM = 1.0
LEFT_X_SIGN = 1
RIGHT_X_SIGN = 1
IMU_AXIS_LENGTH_MM = 22.0
IMU_FROM_TOF_MM = 0.0  # visualization proxy, not measured IMU position
LEFT_ZERO_HOLE = 'R1C1'
RIGHT_ZERO_HOLE = 'R1C2'

# Fusion XYZ: right, forward, up. New WRF XYZ: forward, left, up.
FUSION_LEFT_RCM = (-45.835, -52.692, -43.735)
FUSION_RIGHT_RCM = (47.428, -54.572, -44.604)
FUSION_CAMERA = (0.0, -39.337, -40.776)
FUSION_HOLES = {
    'R1C1': (-15.273, 4.798, -142.628),
    'R1C2': (16.727, 4.798, -142.628),
    'R2C1': (-15.273, 22.798, -142.628),
    'R2C2': (16.727, 22.798, -142.628),
    'R3C1': (-15.273, 40.798, -142.628),
    'R3C2': (16.727, 40.798, -142.628),
    'R4C1': (-15.273, 58.798, -142.628),
    'R4C2': (16.727, 58.798, -142.628),
    'R5C1': (-15.273, 76.798, -142.628),
    'R5C2': (16.727, 76.798, -142.628),
    'R6C1': (-15.273, 94.798, -142.628),
    'R6C2': (16.727, 94.798, -142.628),
}


def fusion_to_wrf(point):
    delta = np.asarray(point, dtype=float) - np.asarray(FUSION_LEFT_RCM, dtype=float)
    return np.array([delta[1], -delta[0], delta[2]], dtype=float)


LEFT_RCM = fusion_to_wrf(FUSION_LEFT_RCM)
RIGHT_RCM = fusion_to_wrf(FUSION_RIGHT_RCM)
CAMERA_LENS = fusion_to_wrf(FUSION_CAMERA)
HOLES = {name: fusion_to_wrf(p) for name, p in FUSION_HOLES.items()}


def as_rotation(q):
    q = np.asarray(q, dtype=float).reshape(-1)
    if q.size != 4 or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-10:
        raise ValueError('Invalid quaternion')
    q = q / np.linalg.norm(q)
    if QUATERNION_ORDER == 'wxyz':
        q = [q[1], q[2], q[3], q[0]]
    elif QUATERNION_ORDER != 'xyzw':
        raise ValueError('QUATERNION_ORDER must be wxyz or xyzw')
    rot = Rotation.from_quat(q)
    return rot.inv() if INVERT_QUATERNION else rot


def quat_wxyz(rotation):
    x, y, z, w = rotation.as_quat()
    return np.array([w, x, y, z])


def quat_text(rotation):
    return np.array2string(quat_wxyz(rotation), precision=4, suppress_small=True)


def raw_quat_text(q):
    return np.array2string(np.asarray(q), precision=4, suppress_small=True)


def expected_rotation(rcm, hole, x_sign=1):
    """IMU-to-WRF: local Z along shaft; local X as upward as possible."""
    if x_sign not in (-1, 1):
        raise ValueError('x_sign must be +/-1')
    z = np.asarray(hole) - np.asarray(rcm)
    length = np.linalg.norm(z)
    if length < 1e-10:
        raise ValueError('Hole coincides with RCM')
    z /= length
    up = np.array([0.0, 0.0, 1.0])
    x = up - np.dot(up, z) * z
    x_len = np.linalg.norm(x)
    if x_len < 1e-8:
        raise ValueError('Shaft vertical: tick-down cannot determine roll')
    x = x_sign * x / x_len
    y = np.cross(z, x)
    y /= np.linalg.norm(y)
    return Rotation.from_matrix(np.column_stack((x, y, z)))


def expected_tof(rcm, hole, tof_offset):
    """tip = RCM + (tool_length - tof_offset - tof) * unit_shaft."""
    tip_from_rcm = np.linalg.norm(np.asarray(hole) - np.asarray(rcm))
    return TOOL_LENGTH_MM - tof_offset - tip_from_rcm


def geometry(rcm, rotation, tof, offset):
    z = rotation.apply([0, 0, 1])
    tof_position = rcm - offset * z
    wall = tof_position - tof * z
    tip = wall + TOOL_LENGTH_MM * z
    imu = tof_position + IMU_FROM_TOF_MM * z
    return {'rcm': rcm, 'tof': tof_position, 'wall': wall, 'tip': tip, 'imu': imu}


class SerialData(QThread):
    sample = pyqtSignal(object)
    error = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.running = True

    def stop(self):
        self.running = False

    def run(self):
        try:
            ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.5)
        except Exception as exc:
            self.error.emit(f'Serial connection failed: {exc}')
            return
        try:
            left_tracker = IMUQuaternionTracker('left', use_offset=False)
            right_tracker = IMUQuaternionTracker('right', use_offset=False)
            tof_manager = TOFManager()
            counter = 0
            while self.running:
                try:
                    parts = ser.readline().decode('utf-8').strip().split(',')
                    if len(parts) < 23:
                        continue
                    counter += 1
                    if counter == COUNT_TILL_SETTLED:
                        left_tracker.set_gain(0.2)
                        right_tracker.set_gain(0.2)
                    arduino_time = parts[0]
                    left_q = left_tracker.get_quaternion([arduino_time] + parts[5:14])
                    right_q = right_tracker.get_quaternion([arduino_time] + parts[14:])
                    if left_q is None or right_q is None:
                        continue
                    d = list(tof_manager.get_distances(parts[3:5]))
                    if len(d) < 2:
                        continue
                    self.sample.emit((time.monotonic(), np.asarray(left_q).copy(),
                                      float(d[0]) * TOF_TO_MM, np.asarray(right_q).copy(),
                                      float(d[1]) * TOF_TO_MM))
                except Exception as exc:
                    self.error.emit(f'Data error: {exc}')
        finally:
            ser.close()


class VerificationGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Dual IMU — Pegboard Quaternion and ToF Verification')
        self.resize(1450, 1000)
        self.latest = None
        self.zero = None
        self.zero_reference = None
        self.setFocusPolicy(Qt.StrongFocus)
        root = QVBoxLayout(self)
        top = QGroupBox('Selected holes, expected measurements and live sensor measurements')
        controls = QGridLayout(top)
        self.left_select = QComboBox()
        self.right_select = QComboBox()
        self.left_select.addItems(HOLES.keys())
        self.right_select.addItems(HOLES.keys())
        self.left_select.setCurrentText(LEFT_ZERO_HOLE)
        self.right_select.setCurrentText(RIGHT_ZERO_HOLE)
        controls.addWidget(QLabel('LEFT tool hole:'), 0, 0)
        controls.addWidget(self.left_select, 0, 1)
        controls.addWidget(QLabel('RIGHT tool hole:'), 0, 2)
        controls.addWidget(self.right_select, 0, 3)
        self.zero_button = QPushButton('STEP 1 — Zero: LEFT R1C1, RIGHT R1C2, tick marks DOWN (Z)')
        self.zero_button.clicked.connect(self.zero_now)
        controls.addWidget(self.zero_button, 1, 0, 1, 4)
        self.instructions = QLabel('FIRST: Put LEFT tip at R1C1 and RIGHT tip at R1C2; align BOTH cylinder tick marks down and press Z. '
                                   'THEN: Move tools and select their verification holes. The zero references stay fixed. '
                                   'For full quaternion comparison, keep tick marks down at each verification hole.')
        self.instructions.setWordWrap(True)
        controls.addWidget(self.instructions, 2, 0, 1, 4)
        self.left_info = QLabel()
        self.right_info = QLabel()
        for info in (self.left_info, self.right_info):
            info.setTextInteractionFlags(Qt.TextSelectableByMouse)
            info.setStyleSheet('font-family: monospace; font-size: 12px; padding: 7px;')
            info.setWordWrap(True)
        controls.addWidget(self.left_info, 3, 0, 1, 2)
        controls.addWidget(self.right_info, 3, 2, 1, 2)
        self.status = QLabel('NOT ZEROED — place LEFT at R1C1 and RIGHT at R1C2; tick marks down; press Z. Expected poses are shown below.')
        self.status.setWordWrap(True)
        controls.addWidget(self.status, 4, 0, 1, 4)
        root.addWidget(top)

        self.view = gl.GLViewWidget()
        root.addWidget(self.view, stretch=1)
        self.view.setCameraPosition(pos=pg.Vector(95, -45, -45), distance=410,
                                    elevation=25, azimuth=45)
        grid = gl.GLGridItem()
        grid.setSize(300, 300)
        grid.setSpacing(25, 25)
        grid.translate(95, -45, -100)
        self.view.addItem(grid)
        self.static_point(LEFT_RCM, (0, 1, 0, 1), 13, 'Left RCM')
        self.static_point(RIGHT_RCM, (1, 0, 0, 1), 13, 'Right RCM')
        self.static_point(CAMERA_LENS, (1, 1, 0, 1), 12, 'Camera')
        for name, point in HOLES.items():
            self.static_point(point, (0.2, 0.7, 1, 1), 9, name)
        # Expected poses: cyan/magenta. Measured poses: green/red.
        self.expected_left = self.make_tool((0, 1, 1, 1), dashed=False)
        self.expected_right = self.make_tool((1, 0.3, 1, 1), dashed=False)
        self.measured_left = self.make_tool((0, 1, 0, 1), dashed=False)
        self.measured_right = self.make_tool((1, 0.1, 0.1, 1), dashed=False)
        self.left_select.currentTextChanged.connect(self.refresh)
        self.right_select.currentTextChanged.connect(self.refresh)
        self.thread = SerialData()
        self.thread.sample.connect(self.receive)
        self.thread.error.connect(self.serial_error)
        self.thread.start()
        self.refresh()

    def static_point(self, position, color, size, name):
        self.view.addItem(gl.GLScatterPlotItem(pos=np.asarray(position).reshape(1, 3),
                                               color=color, size=size, pxMode=True))
        self.view.addItem(gl.GLTextItem(pos=np.asarray(position), text=name,
                                       color=(255, 255, 255, 255)))

    def make_tool(self, color, dashed=False):
        hidden = np.full((2, 3), np.nan)
        tip_hidden = np.full((1, 3), np.nan)
        parts = {}
        for name, c, width in [('shaft', color, 4), ('beam', (1, 1, 0, 0.8), 2),
                               ('x', (1, 0.3, 0.3, 1), 3), ('y', (0.2, 0.5, 1, 1), 3)]:
            parts[name] = gl.GLLinePlotItem(pos=hidden.copy(), color=c, width=width,
                                            antialias=True)
            self.view.addItem(parts[name])
        parts['tip'] = gl.GLScatterPlotItem(pos=tip_hidden, color=color, size=12, pxMode=True)
        self.view.addItem(parts['tip'])
        return parts

    def draw_tool(self, parts, rcm, rot, tof, offset):
        g = geometry(rcm, rot, tof, offset)
        parts['shaft'].setData(pos=np.vstack((g['wall'], g['tip'])))
        parts['beam'].setData(pos=np.vstack((g['tof'], g['wall'])))
        parts['x'].setData(pos=np.vstack((g['imu'], g['imu'] +
                                         IMU_AXIS_LENGTH_MM * rot.apply([1, 0, 0]))))
        parts['y'].setData(pos=np.vstack((g['imu'], g['imu'] +
                                         IMU_AXIS_LENGTH_MM * rot.apply([0, 1, 0]))))
        parts['tip'].setData(pos=g['tip'].reshape(1, 3))
        return g

    def receive(self, sample):
        self.latest = sample
        self.refresh()

    def serial_error(self, message):
        self.status.setText(message)

    def zero_now(self):
        if self.latest is None:
            self.status.setText('Cannot zero: no sensor sample yet.')
            return
        stamp, left_q, left_tof, right_q, right_tof = self.latest
        if time.monotonic() - stamp > MAX_ZERO_SAMPLE_AGE_S:
            self.status.setText('Cannot zero: latest sensor sample is stale.')
            return
        try:
            new_zero = {'left': as_rotation(left_q), 'right': as_rotation(right_q)}
            new_reference = {
                'left': expected_rotation(LEFT_RCM, HOLES[LEFT_ZERO_HOLE], LEFT_X_SIGN),
                'right': expected_rotation(RIGHT_RCM, HOLES[RIGHT_ZERO_HOLE], RIGHT_X_SIGN),
            }
        except (ValueError, TypeError) as exc:
            self.status.setText(f'Zero failed: {exc}')
            return
        self.zero = new_zero
        self.zero_reference = new_reference
        self.status.setText('ZEROED — LEFT R1C1 / RIGHT R1C2. Select verification holes above. Tick marks must remain down for full orientation comparison.')
        self.refresh()

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Z and not event.isAutoRepeat():
            self.zero_now()
            event.accept()
        else:
            super().keyPressEvent(event)

    def refresh(self, *args):
        for side, selector, rcm, offset, sign, info, expected_item, measured_item in [
            ('left', self.left_select, LEFT_RCM, LEFT_TOF_OFFSET_MM, LEFT_X_SIGN,
             self.left_info, self.expected_left, self.measured_left),
            ('right', self.right_select, RIGHT_RCM, RIGHT_TOF_OFFSET_MM, RIGHT_X_SIGN,
             self.right_info, self.expected_right, self.measured_right),
        ]:
            name = selector.currentText()
            hole = HOLES[name]
            exp_rot = expected_rotation(rcm, hole, sign)
            exp_tof = expected_tof(rcm, hole, offset)
            # Drawing expected geometry with the expected ToF places its tip at the hole.
            self.draw_tool(expected_item, rcm, exp_rot, exp_tof, offset)
            lines = [f'{side.upper()} — verifying {name} (zero: {LEFT_ZERO_HOLE if side == "left" else RIGHT_ZERO_HOLE})',
                     f'Expected ToF (mm): {exp_tof:.3f}',
                     f'Expected WRF quaternion [w,x,y,z]: {quat_text(exp_rot)}',
                     f'Expected tip WRF (mm): {np.round(hole, 3)}']
            if self.latest is not None:
                stamp, left_q, left_tof, right_q, right_tof = self.latest
                q, tof = (left_q, left_tof) if side == 'left' else (right_q, right_tof)
                lines += [f'Live ToF (mm): {tof:.3f}  |  error: {tof - exp_tof:+.3f} mm',
                          f'Raw AHRS quaternion [w,x,y,z]: {raw_quat_text(q)}']
                if self.zero is not None:
                    try:
                        measured_rot = (self.zero_reference[side] *
                                        (self.zero[side].inv() * as_rotation(q)))
                        angular_error = np.degrees((exp_rot.inv() * measured_rot).magnitude())
                        expected_z = exp_rot.apply([0, 0, 1])
                        measured_z = measured_rot.apply([0, 0, 1])
                        shaft_error = np.degrees(np.arccos(np.clip(np.dot(expected_z, measured_z), -1., 1.)))
                        measured_geometry = self.draw_tool(measured_item, rcm, measured_rot,
                                                           tof, offset)
                        tip_error = np.linalg.norm(measured_geometry['tip'] - hole)
                        lines += [f'Measured WRF quaternion [w,x,y,z]: {quat_text(measured_rot)}',
                                  f'Full orientation error: {angular_error:.2f} deg',
                                  f'Shaft direction error: {shaft_error:.2f} deg',
                                  f'Tip-to-selected-hole error: {tip_error:.2f} mm']
                    except (ValueError, TypeError) as exc:
                        lines.append(f'Measured orientation error: {exc}')
                else:
                    lines.append('Measured WRF quaternion: NOT AVAILABLE until fixed-hole Z calibration')
            else:
                lines.append('Waiting for live quaternion and ToF...')
            if exp_tof < 0:
                lines.append('WARNING: expected ToF is negative; check geometry/offsets')
            info.setText('\n'.join(lines))

    def closeEvent(self, event):
        self.thread.stop()
        self.thread.wait()
        super().closeEvent(event)


if __name__ == '__main__':
    print('New WRF coordinates (mm):')
    print('LEFT RCM:', LEFT_RCM)
    print('RIGHT RCM:', RIGHT_RCM)
    print('CAMERA:', CAMERA_LENS)
    for key, point in HOLES.items():
        print(f'{key}: {point}')
    app = QApplication(sys.argv)
    gui = VerificationGUI()
    gui.show()
    sys.exit(app.exec())
