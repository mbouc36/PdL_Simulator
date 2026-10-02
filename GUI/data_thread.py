"""
Author: Michael Boucouvalas
Date: 2026, Aug 14th
Version: 2.0
Description: Get raw data over serial port and camera, process it and save it in csv or video file
"""

import os
import sys
import cv2
import csv
import time
import queue
import serial
from pathlib import Path

from PyQt5.QtCore import QThread, pyqtSignal

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from update_config import load_config
from tools.imu_orientation import IMUQuaternionTracker
from tools.tof_manager import TOFManager

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
    "Right IMU Mag Z",
]
PROCESSED_DATA_CSV = "processed_data.csv"
PROCESSED_CSV_COLUMNS = [
    "Arduino Time",
    "Camera Time",
    "Front Weight",
    "Back Weight",
    "Left Surge",
    "Right Surge",
    "Left Quaternion",
    "Right Quaternion",
]

# CSV File Data
NAME_COLUMN = "Name"
KEY_COLUMN = "Key"

# Sensor Data
IMU_INITIALIZATION_TIME = 15  # s




class DataThread(QThread):
    frame_ready = pyqtSignal(object)
    sensors_ready = pyqtSignal(object)
    sensor_data = pyqtSignal(object)

    def __init__(self, folder_name, visualize):
        super().__init__()
        self.running = False
        self.update_output_files(folder_name)

        self.frame_idx = 0
        self.visualize = visualize

        self.serial_queue = queue.Queue()
        self.serial_thread = SerialThread(self.serial_queue )

    def write_to_csv(self, filen_path, values):
        try:
            with open(
                filen_path,
                mode="a",
                newline="",
                encoding="utf-8",
            ) as file:
                writer = csv.writer(file)
                writer.writerow(values)
        except Exception as e:
            print(f"Failed to write sensor data to csv: {e}")

    def run(self):
        cap, video_output = self.create_camera_object()
        self.running = True

        # Initialize the tracker
        left_imu = IMUQuaternionTracker(name="left")
        right_imu = IMUQuaternionTracker(name="right")

        # Initialize tof manager
        tof_manager = TOFManager()
        self.serial_thread.start()
        self.serial_thread.start_initialization()
        init_start_time = time.monotonic()

        while time.monotonic() - init_start_time < IMU_INITIALIZATION_TIME:
            try:
                raw_sensor_data = self.serial_queue.get(timeout=0.1)
            except queue.Empty:
                print("No serial sample available")
                continue

            arduino_time = raw_sensor_data[0]
            left_imu_values = [arduino_time] + raw_sensor_data[5:14]
            right_imu_values = [arduino_time] + raw_sensor_data[14:]
            left_quaternions = [left_imu.get_quaternion(left_imu_values)]
            right_quaternions = [right_imu.get_quaternion(right_imu_values)]

        left_imu.set_gain()
        right_imu.set_gain()
        self.sensors_ready.emit(True)
        # Camera warm-up
        for _ in range(10):

            cap.read()
            
        self.serial_thread.clear_queue()
        self.serial_thread.start_recording()
        while self.running:

            ret, frame = cap.read()
            if not ret:
                print("Error capturing frame")
                continue

            camera_time = time.perf_counter()

            try:
                raw_sensor_data = self.serial_queue.get(timeout=0.1)
            except queue.Empty:
                print("No serial sample available")
                continue

            self.frame_ready.emit(frame)

            # Write to video file
            video_output.write(frame)

            arduino_time = raw_sensor_data[0]
            load_cell_values = raw_sensor_data[1:3]
            tof_values = raw_sensor_data[3:5]
            left_imu_values = [arduino_time] + raw_sensor_data[5:14]
            right_imu_values = [arduino_time] + raw_sensor_data[14:]

            distances = list(tof_manager.get_distances(tof_values))
            left_quaternions = [left_imu.get_quaternion(left_imu_values)]
            right_quaternions = [right_imu.get_quaternion(right_imu_values)]

            # Ensure all values are the same format
            processed_data = (
                [arduino_time]
                + [camera_time]
                + list(load_cell_values)
                + distances
                + left_quaternions
                + right_quaternions
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
        self.serial_thread.running = False
        self.sensors_ready.emit(False)
        self.wait()

    def update_output_files(self, output_folder):
        self.output_folder = os.path.join(OUTPUT_DATA_FOLDER, output_folder)
        output_folder_path = Path(self.output_folder)

        output_folder_path.mkdir(parents=True, exist_ok=True)

        self.video_output_path = os.path.join(self.output_folder, VIDEO_FILENAME)
        self.raw_data_csv = os.path.join(self.output_folder, RAW_SENSOR_CSV)
        self.processed_data_csv = os.path.join(self.output_folder, PROCESSED_DATA_CSV)
        self.write_to_csv(self.raw_data_csv, RAW_SENSOR_CSV_COLUMNS)
        self.write_to_csv(self.processed_data_csv, PROCESSED_CSV_COLUMNS)

    def create_camera_object(self):
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

        return cap, video_output


class SerialThread(QThread):

    def __init__(self, serial_queue):
        super().__init__()

        self.running = False
        self.serial_queue = serial_queue

        self.ser = serial.Serial(
            SERIAL_PORT,
            BAUD_RATE,
            timeout=1
        )

    def start_initialization(self):
        self.ser.reset_input_buffer()
        self.ser.write(b"INITIALIZATION\n")
        self.ser.flush()

    def start_recording(self):
        self.ser.reset_input_buffer()
        self.ser.write(b"START\n")
        self.ser.flush()

    def run(self):
        self.running = True

        while self.running:
            try:
                line = self.ser.readline().decode("utf-8").strip()

            except Exception as e:
                print(e)
                continue

            if not line:
                continue

            raw_sensor_data = line.split(",")

            if len(raw_sensor_data) != len(RAW_SENSOR_CSV_COLUMNS):
                print("Invalid line")
                continue

            self.serial_queue.put(raw_sensor_data)

    def clear_queue(self):
        while True:

            try:
                self.serial_queue.get_nowait()
            except queue.Empty:

                break
