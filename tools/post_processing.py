import os
import sys
import csv
import numpy as np

from PyQt5.QtCore import QThread, pyqtSignal

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from GUI.data_thread import IMU_INITIALIZATION_SAMPLES
from tools.imu_orientation import IMUQuaternionTracker
from tools.tof_manager import TOFManager
from tools.get_files_from_folder import (
    get_raw_data_file,
    get_camera_data_file,
    get_processed_data_file,
)
from diagnostics.validate_data import is_output_data_valid

PROCESSED_CSV_COLUMNS = [
    "Frame",
    "Arduino Time",
    "Camera Time",
    "Front Weight",
    "Back Weight",
    "Left Surge",
    "Right Surge",
    "Left Quaternion",
    "Right Quaternion",
]


class PostProcessingThread(QThread):
    progress = pyqtSignal(int)
    processing_complete = pyqtSignal(str)
    processing_error = pyqtSignal(str)
    data_valid = pyqtSignal(bool)

    def __init__(
        self,
        output_folder,
    ):
        super().__init__()

        self.raw_sensor_csv = get_raw_data_file(output_folder)
        self.camera_timestamp_csv = get_camera_data_file(output_folder)
        self.processed_data_csv = get_processed_data_file(output_folder)
        self.output_folder = output_folder

        self.running = False

    def run(self):

        self.running = True

        try:

            sensor_data = self._load_sensor_data()
            camera_data = self._load_camera_data()

            if len(sensor_data) < 2:
                raise ValueError("Not enough Arduino samples for interpolation.")

            if len(camera_data) == 0:
                raise ValueError("No camera timestamps found.")

            left_imu = IMUQuaternionTracker(name="left")
            right_imu = IMUQuaternionTracker(name="right")
            tof_manager = TOFManager()

            sensor_idx = 0

            while sensor_idx < IMU_INITIALIZATION_SAMPLES:
                line = sensor_data[sensor_idx]
                arduino_time = line[0]
                left_imu_values = [arduino_time] + line[6:15]
                right_imu_values = [arduino_time] + line[15:]
                sensor_idx += 1

                left_imu.get_quaternion(left_imu_values)
                right_imu.get_quaternion(right_imu_values)

            left_imu.set_gain()
            right_imu.set_gain()


            self.clock_a, self.clock_b = self.calculate_clock_mapping(sensor_data)

            print(
                f"Clock mapping: "
                f"PC = {self.clock_a:.9f} * Arduino + "
                f"{self.clock_b:.9f}"
            )

            if self.clock_a == 0:
                raise ValueError("Invalid clock mapping: slope is zero.")

            total_frames = len(camera_data)

            with open(
                self.processed_data_csv,
                mode="w",
                newline="",
                encoding="utf-8",
            ) as file:

                writer = csv.writer(file)

                writer.writerow(PROCESSED_CSV_COLUMNS)

                for camera_idx, (
                    frame_number,
                    camera_time,
                ) in enumerate(camera_data):

                    if not self.running:
                        return

                    target_arduino_time = self._camera_to_arduino_time(camera_time)

                    while (
                        sensor_idx + 1 < len(sensor_data)
                        and sensor_data[sensor_idx + 1][0] < target_arduino_time
                    ):
                        sensor_idx += 1

                    # No later Arduino sample exists
                    if sensor_idx + 1 >= len(sensor_data):
                        break

                    before = sensor_data[sensor_idx]
                    after = sensor_data[sensor_idx + 1]

                    t1 = before[0]
                    t2 = after[0]

                    # Camera frame lies outside available
                    # Arduino samples
                    if not (t1 <= target_arduino_time <= t2):
                        continue

                    interpolated = self._interpolate_sample(
                        target_arduino_time,
                        before,
                        after,
                    )

                    load_cell_values = [
                        round(interpolated[1], 2),
                        round(interpolated[2], 2),
                    ]
                    tof_values = interpolated[3:5]
                    left_imu_values = [target_arduino_time] + interpolated[5:14]
                    right_imu_values = [target_arduino_time] + interpolated[14:]

                    distances = list(tof_manager.get_distances(tof_values))
                    left_quaternion = [left_imu.get_quaternion(left_imu_values)]
                    right_quaternion = [right_imu.get_quaternion(right_imu_values)]

                    processed_data = (
                        [
                            frame_number,
                            target_arduino_time,
                            camera_time,
                        ]
                        + list(load_cell_values)
                        + distances
                        + left_quaternion
                        + right_quaternion
                    )

                    writer.writerow(processed_data)
                    percent = int(((camera_idx + 1) / total_frames) * 100)

                    self.progress.emit(percent)

            if self.running:
                self.data_valid.emit(is_output_data_valid(self.output_folder))
                self.progress.emit(100)
                self.processing_complete.emit(self.processed_data_csv)

        except Exception as e:
            self.processing_error.emit(str(e))

        finally:
            self.running = False
            valid_output = is_output_data_valid(self.output_folder)
            self.data_valid.emit(valid_output)

    def _load_sensor_data(self):

        data = []

        with open(
            self.raw_sensor_csv,
            mode="r",
            newline="",
            encoding="utf-8",
        ) as file:

            reader = csv.reader(file)

            # Skip column names
            next(reader, None)

            for row in reader:

                if not row:
                    continue

                try:
                    values = [float(value) for value in row]

                    data.append(values)

                except ValueError:
                    print(
                        "Invalid sensor row:",
                        row,
                    )

        # Ensure Arduino samples are chronological
        data.sort(key=lambda sample: sample[0])

        return data

    def _load_camera_data(self):

        data = []

        with open(
            self.camera_timestamp_csv,
            mode="r",
            newline="",
            encoding="utf-8",
        ) as file:

            reader = csv.reader(file)

            # Skip column names
            next(reader, None)

            for row in reader:

                if not row:
                    continue

                try:
                    frame_number = int(row[0])
                    camera_time = float(row[1])

                    data.append(
                        (
                            frame_number,
                            camera_time,
                        )
                    )

                except (ValueError, IndexError):

                    print(
                        "Invalid camera row:",
                        row,
                    )

        # Ensure frames are chronological
        data.sort(key=lambda sample: sample[0])

        return data

    def _interpolate_sample(
        self,
        target_time,
        before,
        after,
    ):
        """
        Interpolate raw sensor values at target Arduino time.

        Raw CSV format:

            [0] Arduino Time
            [1] PC Receive Time
            [2:] Sensor values

        Returned format:

            [0] Arduino Time
            [1:] Interpolated sensor values

        PC Receive Time is NOT interpolated because it is used
        only to synchronize the Arduino and PC clocks.
        """

        t1 = before[0]
        t2 = after[0]

        # Prevent division by zero
        if t2 == t1:
            return [target_time] + before[2:].copy()

        alpha = (target_time - t1) / (t2 - t1)

        interpolated = [target_time]

        # Start at index 2 because index 1 is PC Receive Time
        for i in range(
            2,
            len(before),
        ):

            x1 = before[i]
            x2 = after[i]

            value = x1 + alpha * (x2 - x1)

            interpolated.append(value)

        return interpolated

    def calculate_clock_mapping(
        self,
        sensor_data,
    ):
        """
        Calculate the relationship between the Arduino clock
        and the PC perf_counter clock.

        Model:

            PC_time = a * Arduino_time_seconds + b

        Arduino timestamps are assumed to be milliseconds.
        """

        arduino_time = np.array(
            [row[0] for row in sensor_data],
            dtype=float,
        )

        pc_receive_time = np.array(
            [row[1] for row in sensor_data],
            dtype=float,
        )

        # Arduino milliseconds -> seconds
        arduino_time_seconds = arduino_time / 1000.0

        a, b = np.polyfit(
            arduino_time_seconds,
            pc_receive_time,
            1,
        )

        return a, b

    def _camera_to_arduino_time(
        self,
        camera_time,
    ):
        """
        Convert a camera perf_counter timestamp into the
        Arduino time domain.

        Clock relationship:

            PC = a * Arduino + b

        Therefore:

            Arduino = (PC - b) / a
        """

        arduino_time_seconds = (camera_time - self.clock_b) / self.clock_a

        # Convert seconds -> Arduino milliseconds
        return arduino_time_seconds * 1000.0

    def stop(self):

        self.running = False

        self.wait()
