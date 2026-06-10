import sys
import os
import cv2
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
    QHBoxLayout,
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

frame_output_path = os.path.join(FRAME_OUTPUT_DIR, FRAME_FILENAME)
video_output_path = os.path.join(VIDEO_OUTPUT_DIR, "output.mp4")


class DataThread(QThread):
    frame_ready = pyqtSignal(object)

    def __init__(self):
        super().__init__()
        self.running = False

        # init serial
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        except Exception:
            print(f"Failed to connect to port: {SERIAL_PORT}")
            exit(1)

    def write_to_csv():
        pass

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

            # video
            ret, frame = cap.read()
            if not ret:
                continue

            self.frame_ready.emit(frame)

            # Write to txt file
            with open(frame_output_path, "a", encoding="utf-8") as file:
                file.write(np.array_str(frame))

            # Write to video file
            video_output.write(frame)

            # get sensor data
            try:
                line = ser.readline().decode("utf-8").strip()
            except Exception as e:
                print(e)
                continue

            values = line.split(",")

            load_cell_values = values[:2] # First 2 values
            tof_values = values[2:4] 
            left_imu_values = values[5:14]
            right_imu_values = values[14:] 

            distances = tof_manager.get_distances(tof_values)
            left_angles = left_imu.get_angles(left_imu_values)
            right_angles = right_imu.get_angles(right_imu_values)
            sensor_values = tuple(load_cell_values) + distances + left_angles + right_angles

            # load to csv



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

        self.video_thread = DataThread()
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
