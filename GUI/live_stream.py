import sys
import os
import cv2
import csv
import serial
from pathlib import Path

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
NUM_SENSOR_OUTPUT_VALUE = 23


OUTPUT_DATA_FOLDER =  os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "../output_data"
)

VIDEO_FILENAME = "video.mp4"
RAW_SENSOR_CSV = "raw_sensor_data.csv"
PROCESSED_DATA_CSV = "processed_data.csv"



class DataThread(QThread):
    frame_ready = pyqtSignal(object)

    def __init__(self, folder_name):
        super().__init__()
        self.running = False
        self.output_folder = os.path.join(OUTPUT_DATA_FOLDER, folder_name)

        # Define the folder path
        folder_path = Path(self.output_folder)

        # Create the folder safely
        folder_path.mkdir(parents=True, exist_ok=True)

        self.video_output_path = os.path.join(self.output_folder,VIDEO_FILENAME)
        self.raw_data_csv = os.path.join(self.output_folder, RAW_SENSOR_CSV)
        self.processed_data_csv = os.path.join(self.output_folder, PROCESSED_DATA_CSV)
        self.frame_idx = 0

        # init serial
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        except Exception:
            print(f"Failed to connect to port: {SERIAL_PORT}")
            exit(1)


    def write_to_csv(self, filen_path, values):
        try:
            with open(filen_path, mode='a', newline='', encoding='utf-8') as file:
                writer = csv.writer(file)
                writer.writerow(values)
        except Exception as e:
            print(f"Failed to write sensor data to csv: {e}")

    def run(self):
        self.running = True
        cap = cv2.VideoCapture(0)
        frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = 30.0  # Set a default FPS

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

            raw_sensor_data = line.split(",")
            if len(raw_sensor_data) != NUM_SENSOR_OUTPUT_VALUE:
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
            processed_data = [arduino_time] + list(load_cell_values) + distances + left_angles + right_angles

            # load to csv
            self.write_to_csv(self.processed_data_csv, processed_data)
            self.write_to_csv(self.raw_data_csv, raw_sensor_data)

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

        self.video_thread = DataThread("test")
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
