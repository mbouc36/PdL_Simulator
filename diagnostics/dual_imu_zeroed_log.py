"""Dual-IMU zeroed surge/orientation logger.

Real hardware uses DualIMUData defined in this file, which invokes
IMUQuaternionTracker and TOFManager. Quaternions use [w, x, y, z].

Run:
    python dual_imu_zero_logger.py
    python dual_imu_zero_logger.py --demo
    python dual_imu_zero_logger.py --output recordings/session1.txt

IMPORTANT: DualIMUData emits [left_q, left_distance, right_q, right_distance].
This script records those distances as surge without further conversion.
Confirm that this matches the physical meaning of surge in your setup.
"""
import argparse
import csv
from collections import deque
from datetime import datetime, timezone
from pathlib import Path
import sys
import time
import warnings
import os

# Logger is in diagnostics/; project root is one directory above.
sys.path.append(
    os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        ".."
    )
)

import numpy as np
from scipy.spatial.transform import Rotation
from PyQt5.QtCore import Qt, QTimer, QThread, pyqtSignal
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
    return Rotation.from_quat(q / np.linalg.norm(q), scalar_first=True)


def target_rotation(name):
    """Minimum-angle rotation taking local +Z to the RCM-to-target direction."""
    direction = (TARGETS[name] - RCMS[name]).astype(float)
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
    return [float(x) for x in rotation.as_quat(scalar_first=True)]


class DualIMUData(QThread):
    """Read synchronized IMU/ToF samples and emit [left_q, left_dist, right_q, right_dist]."""
    quaternion_data = pyqtSignal(list)
    acquisition_error = pyqtSignal(str)

    def run(self):
        try:
            import serial
            from update_config import load_config
            from tools.imu_orientation import IMUQuaternionTracker
            from tools.tof_manager import TOFManager

            config = load_config()
            ser = serial.Serial(config['serial_port'], config['baud_rate'], timeout=1)
        except Exception as exc:
            self.acquisition_error.emit(f'Cannot initialize hardware: {exc}')
            return

        try:
            # Disable the tracker's fixed heading offset; Z applies calibration.
            left_tracker = IMUQuaternionTracker('left', use_offset=False)
            right_tracker = IMUQuaternionTracker('right', use_offset=False)
            tof_manager = TOFManager()
            iter_counter = 0

            while not self.isInterruptionRequested():
                try:
                    raw = ser.readline()
                    if not raw:
                        continue
                    line = raw.decode('utf-8').strip().split(',')
                    # time + 4 other fields + 9 left IMU + 9 right IMU
                    if len(line) != 23:
                        continue
                    arduino_time = line[0]
                    left_values = [arduino_time] + line[5:14]
                    right_values = [arduino_time] + line[14:23]
                    tof_values = line[3:5]

                    left_q = left_tracker.get_quaternion(left_values)
                    right_q = right_tracker.get_quaternion(right_values)
                    if left_q is None or right_q is None:
                        continue
                    distances = list(tof_manager.get_distances(tof_values))
                    if len(distances) != 2:
                        continue
                    self.quaternion_data.emit([left_q, distances[0], right_q, distances[1]])
                    iter_counter += 1
                    if iter_counter == 400:
                        left_tracker.set_gain(0.2)
                        right_tracker.set_gain(0.2)
                except (ValueError, TypeError, UnicodeDecodeError, IndexError) as exc:
                    # A malformed serial frame should not terminate acquisition.
                    continue
                except Exception as exc:
                    self.acquisition_error.emit(f'Acquisition stopped: {exc}')
                    break
        except Exception as exc:
            self.acquisition_error.emit(f'IMU setup failed: {exc}')
        finally:
            ser.close()


class DualIMULogger(QWidget):
    """Collect synchronized dual-IMU samples and zero/record with Z.

    Only samples already received at the keypress are eligible for zeroing.
    The acquisition thread is started only in hardware mode.
    """
    def __init__(self, output='processed_data.txt', parent=None, use_hardware=True):
        super().__init__(parent)
        self.output = Path(output)
        self.samples = deque(maxlen=500)
        self.zero = None
        self.zero_rotations = None
        self.file = None
        self.writer = None
        self.last_written_time = float('-inf')
        self.data_thread = None
        self._closing = False

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

        if use_hardware:
            self.data_thread = DualIMUData()
            self.data_thread.quaternion_data.connect(self.receive_imu_data)
            self.data_thread.acquisition_error.connect(self.status.setText)
            self.data_thread.start()
            self.status.setText('Starting hardware acquisition. Press Z after valid samples arrive.')

    def receive_imu_data(self, data):
        """Handle DualIMUData signal: [left_q, left_distance, right_q, right_distance]."""
        if self._closing:
            return
        try:
            if len(data) != 4:
                raise ValueError(f'Expected four signal fields, got {len(data)}')
            left_q, left_distance, right_q, right_distance = data
            # Distances are passed through as surge. Confirm geometry before use.
            self.update_sample(left_q, right_q, left_distance, right_distance)
        except (TypeError, ValueError) as exc:
            self.status.setText(f'Invalid sensor sample: {exc}')

    def update_sample(self, left_q, right_q, left_surge_mm, right_surge_mm,
                      timestamp_monotonic=None):
        stamp = time.monotonic() if timestamp_monotonic is None else float(timestamp_monotonic)
        left_rot = as_rotation(left_q)
        right_rot = as_rotation(right_q)
        left_surge, right_surge = float(left_surge_mm), float(right_surge_mm)
        if not np.isfinite([stamp, left_surge, right_surge]).all():
            raise ValueError('Timestamp and surge values must be finite')
        sample = (stamp, left_rot, right_rot, left_surge, right_surge)
        self.samples.append(sample)
        if self.zero is not None and stamp > self.zero[0] and stamp > self.last_written_time:
            self.write_sample(sample, 'DATA')

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Z:
            self.zero_now()
            event.accept()
        else:
            super().keyPressEvent(event)

    def zero_now(self):
        if not self.samples:
            self.status.setText('No samples received; cannot zero.')
            return
        key_time = time.monotonic()
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
        self.file.write('# Zeroed dual-IMU log. Incoming TOFManager distance is recorded as surge (mm).\n')
        self.file.write('# NOTE: Incoming DualIMUData distances are used as surge without conversion.\n')
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
                with warnings.catch_warnings():
                    warnings.filterwarnings('ignore', message='Gimbal lock detected')
                    euler = rot.as_euler('XYZ', degrees=True)
                row.extend(f'{v:.6f}' for v in euler)
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
        self._closing = True
        if self.data_thread is not None and self.data_thread.isRunning():
            self.data_thread.requestInterruption()
            if not self.data_thread.wait(2500):
                self.status.setText('Acquisition thread did not stop; close cancelled.')
                self._closing = False
                event.ignore()
                return
        self.stop_file()
        super().closeEvent(event)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', default='processed_data.txt')
    parser.add_argument('--demo', action='store_true', help='Generate simulated samples (not hardware)')
    args = parser.parse_args()
    app = QApplication(sys.argv)
    window = DualIMULogger(args.output, use_hardware=not args.demo)
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
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
