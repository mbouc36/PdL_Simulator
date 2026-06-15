import sys
import os
import cv2
import csv
import time
import serial
import numpy as np

from PyQt5.QtCore import Qt, QThread, pyqtSignal
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import (
    QApplication,
    QWidget,
    QPushButton,
    QLabel,
    QVBoxLayout,
    QStackedWidget,
)

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from update_config import load_config
from data_processing.imu_angles import BodyRotationTracker
from data_processing.TOF_stream import TOFManager

config = load_config()

SERIAL_PORT = config["serial_port"]
BAUD_RATE = config["baud_rate"]


FRAME_OUTPUT_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "../output_data/frame_data"
)
FRAME_FILENAME = "frames.txt"
VIDEO_OUTPUT_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "../output_data/video"
)

SENSOR_OUTPUT_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "../output_data/sensor_data"
)

frame_output_path = os.path.join(FRAME_OUTPUT_DIR, FRAME_FILENAME)
video_output_path = os.path.join(VIDEO_OUTPUT_DIR, "output.mp4")



class DataThread(QThread):
    frame_ready = pyqtSignal(object)

    def __init__(self, filename):
        super().__init__()
        self.running = False
        self.filename = os.path.join(SENSOR_OUTPUT_DIR, filename + ".csv")
        self.frame_idx = 0

        # init serial
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        except Exception:
            print(f"Failed to connect to port: {SERIAL_PORT}")
            exit(1)

    def write_frames_txt(self, frame, time):
        try:
            with open(frame_output_path, "a", encoding="utf-8") as file:
                # store simple image features, not full pixels
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                mean_intensity = np.mean(gray)
                std_intensity = np.std(gray)
                row = [self.frame_idx, time, mean_intensity, std_intensity]
                file.write(" ".join(map(str, row)) + "\n")
                self.frame_idx += 1
        except Exception as e:
            print(f"Failed to write frame to txt: {e}")


    def write_to_csv(self, serial_values):
        try:
            with open(self.filename, mode='a', newline='', encoding='utf-8') as file:
                writer = csv.writer(file)
                writer.writerow(serial_values)
        except Exception as e:
            print(f"Failed to writ sensr data to csv: {e}")

    def run(self):
        self.running = True
        cap = cv2.VideoCapture(0)
        frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = 30.0  # Set a default FPS

        # Define codec and VideoWriter object (uses 'mp4v' for MP4)
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        video_output = cv2.VideoWriter(
            video_output_path, fourcc, fps, (frame_width, frame_height)
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
                line = ser.readline().decode("utf-8").strip() # wait till new line
            except Exception as e:
                print(e)
                continue

            if not line:
                continue

            ret, frame = cap.read()
            if not ret:
                continue

            self.frame_ready.emit(frame)

            # Write to video file
            video_output.write(frame)

            serial_values = line.split(",")
            if len(serial_values) != 23:
                print("Invalid line")
                continue

            arduino_time = serial_values[0]
            load_cell_values = serial_values[1:3]
            tof_values = serial_values[3:5] 
            left_imu_values = serial_values[5:14]
            right_imu_values = serial_values[14:] 

            # Write to txt file
            self.write_frames_txt(frame, arduino_time)

            distances = list(tof_manager.get_distances(tof_values))
            print(distances)
            left_angles = list(left_imu.get_angles(left_imu_values))
            right_angles = list(right_imu.get_angles(right_imu_values))
            # Ensure all values are the same format
            sensor_values = [arduino_time] + list(load_cell_values) + distances + left_angles + right_angles

            # load to csv
            self.write_to_csv(sensor_values)

        video_output.release()
        cap.release()


    def stop(self):
        self.running = False
        self.wait()


class MainWindow(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("GUI with Livestream")
        self.resize(900, 600)

        self.video_thread = None
        self.pages = QStackedWidget()

        self.create_start_page()
        self.create_video_page()

        main_layout = QVBoxLayout()
        main_layout.addWidget(self.pages)
        self.setLayout(main_layout)

    def create_start_page(self):
        start_page = QWidget()
        layout = QVBoxLayout()

        # title = QLabel("Welcome")
        # title.setAlignment(Qt.AlignCenter)
        # title.setStyleSheet("font-size: 32px;")

        start_btn = QPushButton("Start")
        settings_btn = QPushButton("Settings")

        for btn in [start_btn, settings_btn]:
            btn.setFixedHeight(50)

        start_btn.clicked.connect(self.show_video_page)

        layout.addStretch()
        # layout.addWidget(title)
        layout.addWidget(start_btn)
        layout.addWidget(settings_btn)
        layout.addStretch()

        start_page.setLayout(layout)
        self.pages.addWidget(start_page)

    def create_video_page(self):
        video_page = QWidget()
        layout = QVBoxLayout()

        self.video_label = QLabel("Video not started")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setFixedSize(640, 480)
        self.video_label.setStyleSheet("background-color: black; color: white;")

        exit_btn = QPushButton("Exit")
        exit_btn.setFixedHeight(50)
        exit_btn.clicked.connect(self.close)

        layout.addStretch()
        layout.addWidget(self.video_label, alignment=Qt.AlignCenter)
        layout.addWidget(exit_btn)
        layout.addStretch()

        video_page.setLayout(layout)
        self.pages.addWidget(video_page)

    def show_video_page(self):
        self.pages.setCurrentIndex(1)
        self.start_video()

    def start_video(self):
        if self.video_thread is not None:
            return

        self.video_thread = DataThread("frames")
        self.video_thread.frame_ready.connect(self.update_video_frame)
        self.video_thread.start()

    def update_video_frame(self, frame):
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        h, w, ch = frame.shape
        bytes_per_line = ch * w

        image = QImage(frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(image)

        self.video_label.setPixmap(
            pixmap.scaled(
                self.video_label.width(),
                self.video_label.height(),
                Qt.KeepAspectRatio,
            )
        )

    def closeEvent(self, event):
        if self.video_thread is not None:
            self.video_thread.stop()

        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
