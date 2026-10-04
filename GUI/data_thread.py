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
import serial
from pathlib import Path

from PyQt5.QtCore import QThread, pyqtSignal

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from update_config import load_config

config = load_config()

SERIAL_PORT = config["serial_port"]
BAUD_RATE = config["baud_rate"]


OUTPUT_DATA_FOLDER = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "../output_data"
)

VIDEO_FILENAME = "video.mp4"
RAW_SENSOR_CSV = "raw_sensor_data.csv"
RAW_SENSOR_CSV_COLUMNS = [
    "Arduino Time",
    "PC Receive Time",
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
CAMERA_TIMESTAMP_CSV = "camera_timestamps.csv"
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


CAMERA_TIMESTAMP_CSV_COLUMNS = [
    "Frame",
    "Camera Time",
]


# Sensor Data
IMU_INITIALIZATION_SAMPLES = 300


class DataThread(QThread):
 
    frame_ready = pyqtSignal(object)

    def __init__(self, folder_name):
        super().__init__()

        self.running = False

        self.output_folder = os.path.join(
            OUTPUT_DATA_FOLDER,
            folder_name,
        )

        folder_path = Path(self.output_folder)
        folder_path.mkdir(parents=True, exist_ok=True)

        self.video_output_path = os.path.join(
            self.output_folder,
            VIDEO_FILENAME,
        )

        self.raw_data_csv = os.path.join(
            self.output_folder,
            RAW_SENSOR_CSV,
        )

        self.camera_timestamp_csv = os.path.join(
            self.output_folder,
            CAMERA_TIMESTAMP_CSV,
        )

        self.initialize_csv(
            self.raw_data_csv,
            RAW_SENSOR_CSV_COLUMNS,
        )

        self.initialize_csv(
            self.camera_timestamp_csv,
            CAMERA_TIMESTAMP_CSV_COLUMNS,
        )

        self.serial_thread = SerialThread(self.raw_data_csv)
        self.serial_thread.init_complete.connect(self.set_sensor_ready)

        self.frame_idx = 0

    def initialize_csv(self, file_path, columns):

        try:
            with open(
                file_path,
                mode="w",
                newline="",
                encoding="utf-8",
            ) as file:

                writer = csv.writer(file)
                writer.writerow(columns)

        except Exception as e:
            print(f"Failed to initialize CSV {file_path}: {e}")

    def run(self):

        self.running = True

        self.serial_thread.start()

        self.cap = cv2.VideoCapture(0)

        frame_width = 1920
        frame_height = 1080
        fps = 30.0

        self.cap.set(
            cv2.CAP_PROP_BUFFERSIZE,
            1,
        )

        self.cap.set(
            cv2.CAP_PROP_FRAME_WIDTH,
            frame_width,
        )

        self.cap.set(
            cv2.CAP_PROP_FRAME_HEIGHT,
            frame_height,
        )

        fourcc = cv2.VideoWriter_fourcc(*"mp4v")

        self.video_output = cv2.VideoWriter(
            self.video_output_path,
            fourcc,
            fps,
            (frame_width, frame_height),
        )

        # ensure sensor values are ready before continuing
        while not self.sensors_ready: 
            time.sleep(0.01)

        self.load_camera_data()

    def set_sensor_ready(self, value):
        self.sensors_ready = value

    def load_camera_data(self):
        try:

            with open(
                self.camera_timestamp_csv,
                mode="a",
                newline="",
                encoding="utf-8",
            ) as camera_csv:

                camera_writer = csv.writer(camera_csv)

                while self.running:

                    ret, frame = self.cap.read()

                    # Timestamp immediately after capture
                    camera_time = time.perf_counter()

                    if not ret:
                        print("Error capturing frame")
                        continue

                    self.video_output.write(frame)
                    camera_writer.writerow(
                        [
                            self.frame_idx,
                            camera_time,
                        ]
                    )

                    
                    self.frame_ready.emit(frame)
                    self.frame_idx += 1

        except Exception as e:
            print(f"Camera acquisition error: {e}")

        finally:
            self.video_output.release()
            self.cap.release()
            self.serial_thread.stop()
            self.sensors_ready = False

    def stop(self):
        self.running = False
        self.serial_thread.stop()
        self.wait()


class SerialThread(QThread):
    init_complete = pyqtSignal(object)

    def __init__(self, raw_data_csv):
        super().__init__()

        self.running = False
        self.init_samples = 0
        self.raw_data_csv = raw_data_csv

    def run(self):

        self.running = True

        try:
            ser = serial.Serial(
                SERIAL_PORT,
                BAUD_RATE,
                timeout=1,
            )

        except Exception as e:
            print(f"Failed to open serial port: {e}")
            return

        try:

            with open(
                self.raw_data_csv,
                mode="a",
                newline="",
                encoding="utf-8",
            ) as raw_csv:

                writer = csv.writer(raw_csv)

                while self.running:

                    try:
                        # Wait for complete Arduino packet
                        line = ser.readline()

                        # Timestamp packet arrival on PC
                        pc_receive_time = time.perf_counter()

                        # Decode packet
                        line = line.decode("utf-8").strip()

                    except Exception as e:
                        print(f"Serial read error: {e}")
                        continue

                    if not line:
                        continue

                    raw_sensor_data = line.split(",")

                    # Arduino sends 23 values
                    if len(raw_sensor_data) != len(RAW_SENSOR_CSV_COLUMNS) - 1:
                        print("Invalid Arduino line:", line)
                        continue

                    # Add PC timestamp to the row
                    row = [
                        raw_sensor_data[0],  # Arduino Time
                        pc_receive_time,  # PC Receive Time
                    ] + raw_sensor_data[1:]

                    writer.writerow(row)

                    if self.init_samples < IMU_INITIALIZATION_SAMPLES:
                        self.init_samples += 1
                        if self.init_samples == IMU_INITIALIZATION_SAMPLES:
                            self.init_complete.emit(True)


        finally:
            ser.close()

    def stop(self):
        self.running = False
        self.wait()
