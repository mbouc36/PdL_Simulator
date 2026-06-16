import sys
import os
import cv2
import csv
import serial

import argparse
from PyQt5.QtCore import Qt, QThread, pyqtSignal
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import (
    QApplication,
    QWidget,
    QPushButton,
    QLabel,
    QVBoxLayout,
    QStackedWidget,
    QStackedLayout,
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
        print("hello")
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
            left_imu_values = arduino_time + serial_values[5:14]
            right_imu_values = arduino_time + serial_values[14:] 

            # Write to txt file
            self.write_frames_txt(frame, arduino_time)

            distances = list(tof_manager.get_distances(tof_values))
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
    def __init__(self, test_mode=False):
        super().__init__()

        self.setWindowTitle("GUI with Livestream")
        self.resize(900, 600)

        self.video_thread = None
        self.pages = QStackedWidget()

        self.create_start_page()
        self.create_video_page()

        main_layout = QVBoxLayout()
        main_layout.addWidget(self.pages)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)
        self.setLayout(main_layout)
        self.test_mode = test_mode
        self.setStyleSheet("""
            QWidget {
                background-color: #FFFFF0;
            }
        """)

    def create_start_page(self):
        start_page = QWidget()
        layout = QVBoxLayout()

        title = QLabel("Laparoscopic Simulator")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("""
            font-size: 48px;
            font-weight: bold;
            color: black;
            font-family:  "Times New Roman", Times, serif;
            margin-top: 48px;
            margin-bottom: 48px;
        """)

        start_btn = QPushButton("Start")
        settings_btn = QPushButton("Settings")

        for btn in [start_btn, settings_btn]:
            btn.setFixedHeight(50)
            btn.setMinimumWidth(250)
            btn.setMaximumWidth(400)
            btn.setStyleSheet("""
                QPushButton {
                    background-color: #FFFFF0;
                    color: black;
                    border: 1px solid #D8D0C0;   /* Thin warm gray outline */
                    border-radius: 12px;
                    padding: 10px;
                    font-size: 16px;
                    font-family:  "Times New Roman", Times, serif;
                }

                QPushButton:hover {
                    background-color: #FAF0E6;
                    border: 1px solid #C8C0B0;
                }

                QPushButton:pressed {
                    background-color: #FDF6E3;
                    border: 1px solid #B8B0A0;
                }
            """)

        start_btn.clicked.connect(self.show_video_page)

        layout.addStretch()
        layout.addWidget(title)
        layout.addWidget(start_btn, alignment=Qt.AlignCenter)
        layout.addWidget(settings_btn, alignment=Qt.AlignCenter)
        layout.addStretch()

        start_page.setLayout(layout)
        self.pages.addWidget(start_page)

    def create_video_page(self):
        video_page = QWidget()
        video_page.setStyleSheet("background-color: black;")

        stack_layout = QStackedLayout(video_page)
        stack_layout.setContentsMargins(0, 0, 0, 0)
        stack_layout.setSpacing(0)
        stack_layout.setStackingMode(QStackedLayout.StackAll)

        self.video_label = QLabel("Video not started")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setStyleSheet("background-color: black; color: white;")
        self.video_label.setScaledContents(True)

        overlay = QWidget()
        overlay.setAttribute(Qt.WA_TransparentForMouseEvents, False)
        overlay.setStyleSheet("background: transparent;")

        overlay_layout = QVBoxLayout(overlay)
        overlay_layout.setContentsMargins(20, 20, 20, 20)
        overlay_layout.setSpacing(0)

        exit_btn = QPushButton("Exit")
        exit_btn.setFixedSize(120, 50)
        exit_btn.clicked.connect(self.close)

        exit_btn.setStyleSheet("""
            QPushButton {
                background-color: #FFFFF0;
                color: black;
                border: 1px solid #D8D0C0;
                border-radius: 12px;
                font-size: 16px;
                font-family: "Times New Roman";
            }
        """)

        overlay_layout.addWidget(exit_btn, alignment=Qt.AlignTop | Qt.AlignRight)
        overlay_layout.addStretch()

        stack_layout.addWidget(self.video_label)
        stack_layout.addWidget(overlay)

        # Important: make overlay the top/current widget
        stack_layout.setCurrentWidget(overlay)

        self.pages.addWidget(video_page)

    def show_video_page(self):
        self.pages.setCurrentIndex(1)
        if not self.test_mode:
            self.start_video()
        else:
            pass

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
    parser = argparse.ArgumentParser(description="A script which loads the gui and parses data from the camera and arduino sensors")
    parser.add_argument('--test', action='store_true', help='Run GUI without sensors and camera')

    args = parser.parse_args()

    app = QApplication(sys.argv)
    window = MainWindow(test_mode=args.test)
    window.show()
    sys.exit(app.exec())
