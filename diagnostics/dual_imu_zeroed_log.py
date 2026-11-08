"""Dual-IMU zeroed surge/orientation logger.

Dependencies: numpy scipy PyQt5
Run demo: python dual_imu_zero_logger.py --demo
Integrate: create DualIMULogger, call update_sample(...) on each synchronized
left/right acquisition, and keep its window focused when pressing Z.

Quaternions are [w,x,y,z]. World tool +Z points toward the tip.
"""
import argparse
import csv
from collections import deque
from datetime import datetime, timezone
from pathlib import Path
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QPushButton

RCMS = {
    'Left': np.array([0.000, 0.000, 0.000]),
    'Right': np.array([-1.880, -93.263, -0.869]),
}
TARGETS = {
    'Left': np.array([53.518, -9.291, -87.031]),
    'Right': np.array([53.518, -83.791, -87.031]),
}
TOF_OFFSET_MM = {'Left': 37.743, 'Right': 36.214}
HANDLE_TO_TIP_MM = 285.5


def as_rotation(q):
    q = np.asarray(q, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-10:
        raise ValueError('Quaternion must be finite, nonzero [w,x,y,z]')
    q = q / np.linalg.norm(q)
    return Rotation.from_quat(q, scalar_first=True)


def target_rotation(name):
    """Minimum-angle rotation taking IMU local +Z to RCM->target direction.

    Roll about the tool axis is unobservable from the two points and is chosen
    by the minimum-angle convention.
    """
    direction = TARGETS[name] - RCMS[name]
    direction /= np.linalg.norm(direction)
    z = np.array([0., 0., 1.])
    dot = np.clip(np.dot(z, direction), -1., 1.)
    if dot > 1 - 1e-12:
        return Rotation.identity()
    if dot < -1 + 1e-12:
        return Rotation.from_rotvec(np.pi * np.array([1., 0., 0.]))
    axis = np.cross(z, direction)
    axis /= np.linalg.norm(axis)
    return Rotation.from_rotvec(np.arccos(dot) * axis)


def quat_values(rotation):
    q = rotation.as_quat(scalar_first=True)
    return [float(x) for x in q]


class DualIMULogger(QWidget):
    """Call update_sample(left_q, right_q, left_surge_mm, right_surge_mm,
    timestamp_monotonic=None) once per synchronized acquisition.

    Z selects the sample with minimum |acquisition_time - keypress_time|
    from a short buffer. Samples arriving after keypress cannot be selected.
    """
    def __init__(self, output='processed_data.txt', parent=None):
        super().__init__(parent)
        self.output = Path(output)
        self.samples = deque(maxlen=500)
        self.zero = None
        self.zero_rotations = None
        self.file = None
        self.writer = None
        self.last_written_time = float('-inf')
        self.setWindowTitle('Dual IMU — Z to zero and record')
        self.resize(600, 200)
        layout = QVBoxLayout(self)
        self.status = QLabel('Waiting for synchronized samples. Press Z to zero and begin recording.')
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        button = QPushButton('Zero now (Z)')
        button.clicked.connect(self.zero_now)
        layout.addWidget(button)
        self.setFocusPolicy(Qt.StrongFocus)
        self.setFocus()

    def update_sample(self, left_q, right_q, left_surge_mm, right_surge_mm,
                      timestamp_monotonic=None):
        stamp = time.monotonic() if timestamp_monotonic is None else float(timestamp_monotonic)
        left_rot = as_rotation(left_q)
        right_rot = as_rotation(right_q)
        left_surge, right_surge = float(left_surge_mm), float(right_surge_mm)
        if not all(np.isfinite([stamp, left_surge, right_surge])):
            raise ValueError('Timestamp and surge values must be finite')
        sample = (stamp, left_rot, right_rot, left_surge, right_surge)
        self.samples.append(sample)
        if self.zero is not None and stamp > self.zero[0] and stamp > self.last_written_time:
            self.write_sample(sample, 'DATA')

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Z:
            self.zero_now()
        else:
            super().keyPressEvent(event)

    def zero_now(self):
        if not self.samples:
            self.status.setText('No samples received; cannot zero.')
            return
        key_time = time.monotonic()
        # Nearest already-received sample; reject stale data.
        eligible = [s for s in self.samples if s[0] <= key_time]
        if not eligible:
            self.status.setText('No eligible sample received.')
            return
        sample = min(eligible, key=lambda s: abs(s[0] - key_time))
        age = key_time - sample[0]
        if age > 0.5:
            self.status.setText(f'Latest sample is stale ({age:.3f} s); zero cancelled.')
            return
        self.stop_file()
        self.zero = sample
        self.zero_rotations = {'Left': sample[1], 'Right': sample[2]}
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.file = self.output.open('w', newline='', encoding='utf-8')
        self.file.write('# Zeroed dual-IMU log. Surge = handle-to-TOF distance (mm).\n')
        self.file.write('# Calibrated orientation = target_rotation * inverse(zero_raw_rotation) * raw_rotation.\n')
        self.file.write('# Quaternion order w,x,y,z; Euler intrinsic XYZ degrees.\n')
        self.file.write('# First row ZERO is the acquired reference sample.\n')
        self.file.write('# Calibration direction: RCM -> target is local +Z; roll chosen by minimum rotation.\n')
        self.file.write(f'# Handle-to-tip mm: {HANDLE_TO_TIP_MM}; Left offset mm: {TOF_OFFSET_MM["Left"]}; Right offset mm: {TOF_OFFSET_MM["Right"]}\n')
        self.file.write(f'# Zero UTC: {datetime.now(timezone.utc).isoformat()}\n')
        fields = ['record_type', 'elapsed_s', 'left_surge_mm', 'right_surge_mm']
        for side in ('left', 'right'):
            fields += [f'{side}_q_{v}' for v in 'wxyz']
            fields += [f'{side}_euler_{v}_deg' for v in 'xyz']
            fields += [f'{side}_relative_q_{v}' for v in 'wxyz']
            fields += [f'{side}_relative_euler_{v}_deg' for v in 'xyz']
        self.writer = csv.writer(self.file, delimiter='\t')
        self.writer.writerow(fields)
        self.last_written_time = float('-inf')
        self.write_sample(sample, 'ZERO')
        self.status.setText(f'ZERO captured ({age*1000:.0f} ms before keypress). Recording to {self.output}')

    def write_sample(self, sample, kind):
        stamp, left, right, left_surge, right_surge = sample
        row = [kind, f'{stamp-self.zero[0]:.6f}', f'{left_surge:.6f}', f'{right_surge:.6f}']
        for side, raw in [('Left', left), ('Right', right)]:
            relative = self.zero_rotations[side].inv() * raw
            calibrated = target_rotation(side) * relative
            for rot in (calibrated, relative):
                row.extend(f'{v:.9f}' for v in quat_values(rot))
                row.extend(f'{v:.6f}' for v in rot.as_euler('XYZ', degrees=True))
        self.writer.writerow(row)
        self.file.flush()
        self.last_written_time = stamp

    def stop_file(self):
        if self.file is not None:
            self.file.flush()
            self.file.close()
            self.file = None
            self.writer = None

    def closeEvent(self, event):
        self.stop_file()
        super().closeEvent(event)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', default='processed_data.txt')
    parser.add_argument('--demo', action='store_true', help='Generate simulated samples (not hardware)')
    args = parser.parse_args()
    app = QApplication(sys.argv)
    window = DualIMULogger(args.output)
    window.show()
    if args.demo:
        start = time.monotonic()
        timer = QTimer(window)
        def generate():
            t = time.monotonic() - start
            ql = Rotation.from_euler('xyz', [5*np.sin(t), 10*np.cos(t*.5), 0], degrees=True)
            qr = Rotation.from_euler('xyz', [3*np.sin(t*.7), -8*np.cos(t*.4), 0], degrees=True)
            window.update_sample(quat_values(ql), quat_values(qr),
                                 129.526 + 2*np.sin(t), 125.216 + 2*np.cos(t))
        timer.timeout.connect(generate)
        timer.start(20)
    else:
        window.status.setText('Awaiting hardware samples. Connect your acquisition callback to update_sample(...). Press Z to zero.')
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
