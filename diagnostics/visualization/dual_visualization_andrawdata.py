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
from collections import deque
import numpy as np
import serial
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from scipy.spatial.transform import Rotation
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout,
                             QLabel, QPushButton, QComboBox, QGroupBox, QGridLayout)

# Locate the project root even when this script is moved one directory higher.
from pathlib import Path
_here = Path(__file__).resolve().parent
_project_root = next((p for p in (_here, *_here.parents)
                      if (p / 'update_config.py').is_file()
                      and (p / 'tools' / 'imu_orientation.py').is_file()), None)
if _project_root is None:
    raise ImportError('Cannot find project root containing update_config.py and tools/imu_orientation.py')
sys.path.insert(0, str(_project_root))
from update_config import load_config
from tools.imu_orientation import IMUQuaternionTracker
from tools.tof_manager import TOFManager

config = load_config()
SERIAL_PORT = config['serial_port']
BAUD_RATE = config['baud_rate']
MADGWICK_GAIN = 0.1  # Constant throughout initialization AND normal operation.
CONVERGENCE_THRESHOLD_DEG = 1.0
CONVERGENCE_WINDOW_S = 1.0
STABILITY_DURATION_S = 2.0
MAX_CONVERGENCE_TIME_S = 120.0  # Exit program if both IMUs have not converged.
PRINT_INTERVAL_S = 1
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
FUSION_LEFT_RCM = (-39.413, -52.663, -45.256)
FUSION_RIGHT_RCM = (41.444, -53.383, -47.122)
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


NEW_FUSION_HOLES = {
    'R1C1': (-36.544, 0.826, -143.317),
    'R1C7': (37.956, 0.826, -143.317),
    'R2C5': (17.206, 4.576, -126.317),
    'R3C2': (-33.294, 9.576, -126.317),
    'R3C3': (-13.294, 9.576, -126.317),
    'R4C4': (4.706, 19.576, -126.317),
    'R4C6': (29.706, 19.576, -126.317),
    'R5C2': (-33.294, 34.576, -126.317),
    'R5C3': (-13.294, 34.576, -126.317),
    'R6C4': (4.706, 49.576, -126.317),
    'R6C6': (29.706, 49.576, -126.317),
    'R7C2': (-33.294, 59.576, -126.317),
    'R7C3': (-13.294, 59.576, -126.317),
    'R8C5': (17.206, 64.576, -126.317),
}
BOARD_CONFIGS = {
    'Original 6x2': {'holes': FUSION_HOLES, 'left_zero': 'R1C1', 'right_zero': 'R1C2'},
    'New 14-hole': {'holes': NEW_FUSION_HOLES, 'left_zero': 'R1C1', 'right_zero': 'R1C7'},
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


class AngularStabilityMonitor:
    """Detect a stable quaternion over a full time window, sustained in time.

    This measures relative rotation magnitude, NOT absolute pose accuracy.
    """
    def __init__(self):
        self.history = deque()
        self.stable_since = None
        self.converged = False
        self.samples = 0
        self.window_angle_deg = None

    def update(self, q, now):
        if self.converged:
            return True
        self.samples += 1
        self.history.append((now, as_rotation(q)))
        while len(self.history) > 1 and now - self.history[1][0] >= CONVERGENCE_WINDOW_S:
            self.history.popleft()
        oldest_time, oldest_rot = self.history[0]
        if now - oldest_time < CONVERGENCE_WINDOW_S:
            self.stable_since = None
            return False
        self.window_angle_deg = max(
            np.degrees((oldest_rot.inv() * rot).magnitude())
            for _, rot in self.history
        )
        if self.window_angle_deg < CONVERGENCE_THRESHOLD_DEG:
            if self.stable_since is None:
                self.stable_since = now
            if now - self.stable_since >= STABILITY_DURATION_S:
                self.converged = True
        else:
            self.stable_since = None
        return self.converged


class SerialData(QThread):
    sample = pyqtSignal(object)
    error = pyqtSignal(str)
    converged = pyqtSignal(object)
    progress = pyqtSignal(str)
    failed = pyqtSignal(str)

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
            left_tracker.set_gain(MADGWICK_GAIN)
            right_tracker.set_gain(MADGWICK_GAIN)
            tof_manager = TOFManager()
            left_monitor = AngularStabilityMonitor()
            right_monitor = AngularStabilityMonitor()
            ready = False
            first_timestamp = None
            start = time.monotonic()  # Timeout includes waiting for first valid data.
            last_print = 0.0
            previous_diag_time = None
            while self.running:
                now = time.monotonic()
                if not ready and now - start >= MAX_CONVERGENCE_TIME_S:
                    message = (f'CONVERGENCE TIMEOUT after {MAX_CONVERGENCE_TIME_S:.1f}s: '
                               f'LEFT={left_monitor.converged} '
                               f'({left_monitor.samples} samples), '
                               f'RIGHT={right_monitor.converged} '
                               f'({right_monitor.samples} samples). Exiting.')
                    print(message, flush=True)
                    self.failed.emit(message)
                    break
                try:
                    parts = ser.readline().decode('utf-8').strip().split(',')
                    if len(parts) < 23:
                        continue
                    arduino_time = parts[0]
                    if first_timestamp is None:
                        first_timestamp = float(arduino_time)
                        left_tracker.previous_time = first_timestamp
                        right_tracker.previous_time = first_timestamp
                        continue
                    left_q = left_tracker.get_quaternion([arduino_time] + parts[5:14])
                    right_q = right_tracker.get_quaternion([arduino_time] + parts[14:23])
                    if left_q is None or right_q is None:
                        continue
                    left_q = np.asarray(left_q, dtype=float)
                    right_q = np.asarray(right_q, dtype=float)
                    if not np.isfinite(left_q).all() or not np.isfinite(right_q).all():
                        continue
                    now = time.monotonic()
                    if not ready:
                        left_monitor.update(left_q, now)
                        right_monitor.update(right_q, now)
                        if now - last_print >= PRINT_INTERVAL_S:
                            def detail(m):
                                angle = ('waiting' if m.window_angle_deg is None
                                         else f'{m.window_angle_deg:.3f} deg')
                                return f'{angle}, {m.samples} samples, converged={m.converged}'
                            msg = (f'{now-start:.1f}s | LEFT: {detail(left_monitor)} '
                                   f'| RIGHT: {detail(right_monitor)}')
                            print(msg, flush=True)
                            self.progress.emit(msg)
                            last_print = now
                        if left_monitor.converged and right_monitor.converged:
                            ready = True
                            result = {'elapsed_s': now-start,
                                      'left_samples': left_monitor.samples,
                                      'right_samples': right_monitor.samples}
                            print(f'BOTH IMUs CONVERGED in {now-start:.2f}s; GUI opening. '
                                  f'Gain remains {MADGWICK_GAIN}.', flush=True)
                            self.converged.emit(result)
                    # Diagnostic measurements from the same packet used for AHRS.
                    # Gyro: bias-corrected rad/s. Magnetometer: offset/scale corrected
                    # in the same (unspecified) units as the calibration file.
                    try:
                        packet_time = float(arduino_time)
                        dt_diag = (packet_time - previous_diag_time) / 1000.0 if previous_diag_time is not None else float('nan')
                        previous_diag_time = packet_time
                        diagnostics = {'dt_s': dt_diag}
                        for side, tracker, first in (('left', left_tracker, 5), ('right', right_tracker, 14)):
                            raw = np.array([float(v) for v in parts[first:first+9]], dtype=float)
                            if not np.isfinite(raw).all():
                                raise ValueError('Non-finite raw IMU measurement')
                            gyro = np.radians(raw[3:6] - np.array([tracker.gOffset[a] for a in 'xyz']))
                            mag = (raw[6:9] - np.array([tracker.magOffset[a] for a in 'xyz'])) * np.array([tracker.magScale[a] for a in 'xyz'])
                            diagnostics[side] = {
                                'gyro_raw_dps': raw[3:6].copy(),
                                'gyro_dps': np.degrees(gyro),
                                'mag_raw': raw[6:9].copy(),
                                'mag_cal': mag.copy(),
                                'mag_norm': float(np.linalg.norm(mag)),
                            }
                    except (ValueError, TypeError, IndexError) as exc:
                        self.error.emit(f'Diagnostic data error: {exc}')
                        continue
                    # Keep updating quaternions with constant gain after convergence.
                    d = list(tof_manager.get_distances(parts[3:5]))
                    if len(d) < 2:
                        continue
                    self.sample.emit((now, left_q.copy(), float(d[0]) * TOF_TO_MM,
                                      right_q.copy(), float(d[1]) * TOF_TO_MM,
                                      diagnostics))
                except Exception as exc:
                    self.error.emit(f'Data error: {exc}')
        finally:
            ser.close()


class VerificationGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Dual IMU — Switchable Pegboard Verification')
        self.resize(1450, 1000)
        self.latest = None
        self.zero = None
        self.zero_reference = None
        self.board_name = 'Original 6x2'
        self.holes = dict(HOLES)
        self.left_zero_hole = LEFT_ZERO_HOLE
        self.right_zero_hole = RIGHT_ZERO_HOLE
        self.setFocusPolicy(Qt.StrongFocus)
        root = QVBoxLayout(self)
        top = QGroupBox('Selected holes, expected measurements and live sensor measurements')
        controls = QGridLayout(top)
        self.board_select = QComboBox()
        self.board_select.addItems(BOARD_CONFIGS.keys())
        controls.addWidget(QLabel('Board:'), 0, 4)
        controls.addWidget(self.board_select, 0, 5)
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
        # Matching sections in each instrument column make side-by-side diagnosis easier.
        self.info_labels = {}
        for column, side in ((0, 'left'), (2, 'right')):
            panel = QGroupBox(f'{side.upper()} IMU')
            panel_layout = QVBoxLayout(panel)
            for section, heading in (
                ('tof', 'ToF / Distance'),
                ('orientation', 'Orientation / Tool Pose'),
                ('diagnostics', 'Sensor Diagnostics (IMU frame)'),
            ):
                group = QGroupBox(heading)
                group_layout = QVBoxLayout(group)
                info = QLabel()
                info.setTextInteractionFlags(Qt.TextSelectableByMouse)
                info.setStyleSheet('font-family: monospace; font-size: 11px; padding: 3px;')
                info.setWordWrap(True)
                group_layout.addWidget(info)
                panel_layout.addWidget(group)
                self.info_labels[(side, section)] = info
            controls.addWidget(panel, 3, column, 1, 2)
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
        self.hole_items = []
        self.rebuild_hole_points()
        # Expected poses: cyan/magenta. Measured poses: green/red.
        self.expected_left = self.make_tool((0, 1, 1, 1), dashed=False)
        self.expected_right = self.make_tool((1, 0.3, 1, 1), dashed=False)
        self.measured_left = self.make_tool((0, 1, 0, 1), dashed=False)
        self.measured_right = self.make_tool((1, 0.1, 0.1, 1), dashed=False)
        self.board_select.currentTextChanged.connect(self.switch_board)
        self.left_select.currentTextChanged.connect(self.refresh)
        self.right_select.currentTextChanged.connect(self.refresh)
        self.thread = SerialData()
        self.thread.sample.connect(self.receive)
        self.thread.error.connect(self.serial_error)
        self.thread.converged.connect(self.on_converged)
        self.thread.progress.connect(self.on_progress)
        self.thread.failed.connect(self.on_failed)
        self.thread.start()
        self.refresh()

    def rebuild_hole_points(self):
        for item in self.hole_items:
            self.view.removeItem(item)
        self.hole_items.clear()
        for name, point in self.holes.items():
            self.hole_items.extend(self.static_point(point, (0.2, 0.7, 1, 1), 9, name))
            # On the new board only, draw a 15 mm white peg line in -Z.
            # Exclude the two starting holes used for zero calibration.
            if self.board_name == 'New 14-hole' and name not in ('R1C1', 'R1C7'):
                start = np.asarray(point, dtype=float)
                end = start + np.array([0.0, 0.0, -15.0])
                peg_line = gl.GLLinePlotItem(
                    pos=np.vstack((start, end)),
                    color=(1.0, 1.0, 1.0, 1.0),
                    width=2,
                    antialias=True,
                )
                self.view.addItem(peg_line)
                self.hole_items.append(peg_line)

    def switch_board(self, board_name):
        config = BOARD_CONFIGS[board_name]
        self.board_name = board_name
        self.holes = {name: fusion_to_wrf(p) for name, p in config['holes'].items()}
        self.left_zero_hole = config['left_zero']
        self.right_zero_hole = config['right_zero']
        # A different board changes the known reference orientations.
        self.zero = None
        self.zero_reference = None
        self.left_select.blockSignals(True)
        self.right_select.blockSignals(True)
        for selector, zero_hole in ((self.left_select, self.left_zero_hole),
                                    (self.right_select, self.right_zero_hole)):
            selector.clear()
            selector.addItems(self.holes.keys())
            selector.setCurrentText(zero_hole)
        self.left_select.blockSignals(False)
        self.right_select.blockSignals(False)
        self.rebuild_hole_points()
        self.zero_button.setText(
            f'STEP 1 — Zero: LEFT {self.left_zero_hole}, RIGHT {self.right_zero_hole}, tick marks DOWN (Z)')
        self.instructions.setText(
            f'FIRST: Put LEFT tip at {self.left_zero_hole} and RIGHT tip at {self.right_zero_hole}; '
            'align BOTH cylinder tick marks down and press Z. THEN select verification holes. '
            'Changing verification holes does not alter calibration.')
        self.status.setText(
            f'{board_name} selected — NOT ZEROED. LEFT {self.left_zero_hole}, '
            f'RIGHT {self.right_zero_hole}; tick marks down; press Z.')
        # Hide stale measured geometry until the new board is zeroed.
        for parts in (self.measured_left, self.measured_right):
            for name in ('shaft', 'beam', 'x', 'y'):
                parts[name].setData(pos=np.full((2, 3), np.nan))
            parts['tip'].setData(pos=np.full((1, 3), np.nan))
        self.refresh()

    def on_converged(self, result):
        self.status.setText(
            f"Both IMUs stable after {result['elapsed_s']:.2f} s "
            f"(left {result['left_samples']} samples; right {result['right_samples']} samples). "
            f"Gain stays at {MADGWICK_GAIN}. Now place tools at selected board zero holes and press Z."
        )
        self.show()
        self.raise_()
        self.activateWindow()

    def on_failed(self, message):
        print(message, flush=True)
        self.thread.stop()
        QApplication.instance().exit(1)

    def on_progress(self, message):
        # GUI remains hidden until both filters satisfy the stability test.
        self.status.setText(message)

    def static_point(self, position, color, size, name):
        dot = gl.GLScatterPlotItem(pos=np.asarray(position).reshape(1, 3),
                                   color=color, size=size, pxMode=True)
        label = gl.GLTextItem(pos=np.asarray(position), text=name,
                              color=(255, 255, 255, 255))
        self.view.addItem(dot)
        self.view.addItem(label)
        return dot, label

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
        stamp, left_q, left_tof, right_q, right_tof = self.latest[:5]
        if time.monotonic() - stamp > MAX_ZERO_SAMPLE_AGE_S:
            self.status.setText('Cannot zero: latest sensor sample is stale.')
            return
        try:
            new_zero = {'left': as_rotation(left_q), 'right': as_rotation(right_q)}
            new_reference = {
                'left': expected_rotation(LEFT_RCM, self.holes[self.left_zero_hole], LEFT_X_SIGN),
                'right': expected_rotation(RIGHT_RCM, self.holes[self.right_zero_hole], RIGHT_X_SIGN),
            }
        except (ValueError, TypeError) as exc:
            self.status.setText(f'Zero failed: {exc}')
            return
        self.zero = new_zero
        self.zero_reference = new_reference
        self.status.setText(f'ZEROED — LEFT {self.left_zero_hole} / RIGHT {self.right_zero_hole}. Select verification holes above.')
        self.refresh()

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Z and not event.isAutoRepeat():
            self.zero_now()
            event.accept()
        else:
            super().keyPressEvent(event)

    def refresh(self, *args):
        for side, selector, rcm, offset, sign, expected_item, measured_item in [
            ('left', self.left_select, LEFT_RCM, LEFT_TOF_OFFSET_MM, LEFT_X_SIGN,
             self.expected_left, self.measured_left),
            ('right', self.right_select, RIGHT_RCM, RIGHT_TOF_OFFSET_MM, RIGHT_X_SIGN,
             self.expected_right, self.measured_right),
        ]:
            name = selector.currentText()
            if name not in self.holes:
                continue
            hole = self.holes[name]
            exp_rot = expected_rotation(rcm, hole, sign)
            exp_tof = expected_tof(rcm, hole, offset)
            self.draw_tool(expected_item, rcm, exp_rot, exp_tof, offset)

            tof_lines = [f'Expected:  {exp_tof:9.3f} mm']
            orientation_lines = [
                f'Verifying: {name}  |  Zero: {self.left_zero_hole if side == "left" else self.right_zero_hole}',
                f'Expected WRF q [w,x,y,z]: {quat_text(exp_rot)}',
                f'Expected tip WRF (mm): {np.round(hole, 3)}',
            ]
            diagnostic_lines = ['Waiting for sensor data...']
            if self.latest is not None:
                stamp, left_q, left_tof, right_q, right_tof = self.latest[:5]
                diagnostics = self.latest[5] if len(self.latest) > 5 else {}
                q, tof = (left_q, left_tof) if side == 'left' else (right_q, right_tof)
                tof_lines += [f'Actual:    {tof:9.3f} mm',
                              f'Error:     {tof - exp_tof:+9.3f} mm']
                orientation_lines.append(f'Raw AHRS q [w,x,y,z]: {raw_quat_text(q)}')
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
                        # Heading of the shaft projection in the WRF XY plane.
                        expected_heading = np.degrees(np.arctan2(expected_z[1], expected_z[0]))
                        measured_heading = np.degrees(np.arctan2(measured_z[1], measured_z[0]))
                        heading_error = (measured_heading - expected_heading + 180.0) % 360.0 - 180.0
                        orientation_lines += [
                            f'Measured WRF q [w,x,y,z]: {quat_text(measured_rot)}',
                            f'World-Z shaft heading error: {heading_error:+.2f} deg',
                            f'Full orientation error: {angular_error:.2f} deg',
                            f'Shaft direction error: {shaft_error:.2f} deg',
                            f'Tip-to-hole error: {tip_error:.2f} mm',
                        ]
                    except (ValueError, TypeError) as exc:
                        orientation_lines.append(f'Measured orientation error: {exc}')
                else:
                    orientation_lines.append('Measured WRF q: NOT AVAILABLE until Z calibration')
                d = diagnostics.get(side)
                if d is not None:
                    def vec(v):
                        return '[' + ', '.join(f'{float(x):+.3f}' for x in v) + ']'
                    diagnostic_lines = [
                        f'Gyro raw XYZ (deg/s): {vec(d["gyro_raw_dps"])}',
                        f'Gyro calibrated XYZ (deg/s): {vec(d["gyro_dps"])}',
                        f'Mag raw XYZ: {vec(d["mag_raw"])}',
                        f'Mag calibrated XYZ: {vec(d["mag_cal"])}',
                        f'Mag calibrated magnitude: {d["mag_norm"]:.3f}',
                        f'Packet dt: {diagnostics.get("dt_s", float("nan"))*1000:.2f} ms',
                    ]
            else:
                tof_lines.append('Actual: waiting...')
                orientation_lines.append('Waiting for live quaternion...')
            if exp_tof < 0:
                tof_lines.append('WARNING: expected ToF is negative; check geometry/offsets')
            self.info_labels[(side, 'tof')].setText('\n'.join(tof_lines))
            self.info_labels[(side, 'orientation')].setText('\n'.join(orientation_lines))
            self.info_labels[(side, 'diagnostics')].setText('\n'.join(diagnostic_lines))

    def closeEvent(self, event):
        self.thread.stop()
        self.thread.wait()
        super().closeEvent(event)
        QApplication.instance().quit()


if __name__ == '__main__':
    print('New WRF coordinates (mm):')
    print('LEFT RCM:', LEFT_RCM)
    print('RIGHT RCM:', RIGHT_RCM)
    print('CAMERA:', CAMERA_LENS)
    for key, point in HOLES.items():
        print(f'{key}: {point}')
    app = QApplication(sys.argv)
    gui = VerificationGUI()
    print(f'Waiting for BOTH IMUs to stabilize at constant Madgwick gain {MADGWICK_GAIN}.')
    print('GUI will open only after convergence. Keep both IMUs stationary.')
    print('If both IMUs are not converged within 60 seconds, the program exits.')
    # Keep the hidden window and its worker alive until the worker emits converged.
    app.setQuitOnLastWindowClosed(False)
    sys.exit(app.exec())
