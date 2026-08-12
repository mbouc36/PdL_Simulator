import sys
import os
import cv2
import csv
import serial
from pathlib import Path

from PyQt5.QtCore import QThread, pyqtSignal


sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from update_config import load_config
from data_processing.imu_angles import BodyRotationTracker
from data_processing.TOF_stream import TOFManager

config = load_config()

SERIAL_PORT = config["serial_port"]
BAUD_RATE = config["baud_rate"]


OUTPUT_DATA_FOLDER = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "../output_data"
)

VIDEO_FILENAME = "video.mp4"
RAW_SENSOR_CSV = "raw_sensor_data.csv"
RAW_SENSOR_CSV_COLUMNS = [
    "Time",
    "Front Weight",
    "Back Weight",
    "Raw Left Surge",
    "Raw Right Surge",
    "Left IMU Acc X",
    "Left IMU Acc Y",
    "Left IMU Acc Z",
    "Left IMU Gyro X",
    "Left IMU Gyro Y",
    "Left IMU Gyro Z",
    "Left IMU Mag X",
    "Left IMU Mag Y",
    "Left IMU Mag Z",
    "Right IMU Acc X",
    "Right IMU Acc Y",
    "Right IMU Acc Z",
    "Right IMU Gyro X",
    "Right IMU Gyro Y",
    "Right IMU Gyro Z",
    "Right IMU Mag X",
    "Right IMU Mag Y",
    "Right IMU Mag Z"
]
PROCESSED_DATA_CSV = "processed_data.csv"
PROCESSED_CSV_COLUMNS = [
    "Time",
    "Front Weight",
    "Back Weight",
    "Left Surge",
    "Right Surge",
    "Left Roll",
    "Left Pitch",
    "Left Yaw",
    "Right Roll",
    "Right Pitch",
    "Right Yaw"

]

# CSV File Data
NAME_COLUMN = "Name"
KEY_COLUMN = "Key"

class DataThread(QThread):
    frame_ready = pyqtSignal(object)
    sensor_data = pyqtSignal(object)

    def __init__(self, folder_name, visualize):
        super().__init__()
        self.running = False
        self.output_folder = os.path.join(OUTPUT_DATA_FOLDER, folder_name)
        # Define the folder path
        folder_path = Path(self.output_folder)

        # Create the folder safely
        folder_path.mkdir(parents=True, exist_ok=True)

        self.video_output_path = os.path.join(self.output_folder, VIDEO_FILENAME)
        self.raw_data_csv = os.path.join(self.output_folder, RAW_SENSOR_CSV)
        self.processed_data_csv = os.path.join(self.output_folder, PROCESSED_DATA_CSV)
        self.write_to_csv(self.raw_data_csv, RAW_SENSOR_CSV_COLUMNS)
        self.write_to_csv(self.processed_data_csv, PROCESSED_CSV_COLUMNS)

        self.frame_idx = 0
        self.visualize = visualize
        

    def write_to_csv(self, filen_path, values):
        try:
            with open(filen_path, mode="a", newline="", encoding="utf-8",) as file:
                writer = csv.writer(file)
                writer.writerow(values)
        except Exception as e:
            print(f"Failed to write sensor data to csv: {e}")

    def run(self):
        self.running = True
        cap = cv2.VideoCapture(0)
        frame_width = 1920
        frame_height = 1080
        fps = 30.0  # Set a default FPS


        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)

        # Define codec and VideoWriter object (uses 'mp4v' for MP4)
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        video_output = cv2.VideoWriter(
            self.video_output_path, fourcc, fps, (frame_width, frame_height)
        )

        # Initialize the tracker
        left_imu = BodyRotationTracker(name="left")
        right_imu = BodyRotationTracker(name="right")

        # Initialize tof manager
        tof_manager = TOFManager()

        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        while self.running:
            # Synchronize everything with serial prints
            try:
                line = ser.readline().decode("utf-8").strip()  # wait till new line
            except Exception as e:
                print(e)
                continue

            if not line:
                continue

            ret, frame = cap.read()
            if not ret:
                print("Error capturing frame")
                continue

            self.frame_ready.emit(frame)

            # Write to video file
            video_output.write(frame)

            raw_sensor_data = line.split(",")
            if len(raw_sensor_data) != len(RAW_SENSOR_CSV_COLUMNS):
                print("Invalid line")
                continue

            arduino_time = raw_sensor_data[0]
            load_cell_values = raw_sensor_data[1:3]
            tof_values = raw_sensor_data[3:5]
            left_imu_values = [arduino_time] + raw_sensor_data[5:14]
            right_imu_values = [arduino_time] + raw_sensor_data[14:]

            distances = list(tof_manager.get_distances(tof_values))
            left_angles = list(left_imu.get_angles(left_imu_values))
            right_angles = list(right_imu.get_angles(right_imu_values))

            # Ensure all values are the same format
            processed_data = (
                [arduino_time]
                + list(load_cell_values)
                + distances
                + left_angles
                + right_angles
            )

            # load to csv
            self.write_to_csv(self.processed_data_csv, processed_data)
            self.write_to_csv(self.raw_data_csv, raw_sensor_data)

            # visualize
            if self.visualize:
                self.sensor_data.emit(processed_data)

        video_output.release()
        cap.release()

    def stop(self):
        self.running = False
        self.wait()