"""
Author: Michael Boucouvalas
Date: 2026, Sep 9
Version: 1.0
Description: Using data stored in csv, replay all IMU and ToF data
"""

import os
import sys
import csv
import cv2
import argparse
import numpy as np

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import (
    QApplication,
    QWidget,
    QHBoxLayout,
    QLabel,
    QStackedLayout,
)

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from diagnostics.visualization.tool_visualization import ToolVisualization

NORTH_OFFSET = 0

# Change these indices to match your CSV.
# Example assumes each visualization receives a quaternion [w, x, y, z].
LEFT_Q_INDEX = 6
LEFT_TOF_INDEX = 4

RIGHT_Q_INDEX = 7
RIGHT_TOF_INDEX = 5


class ReplayKinematicData(QWidget):
    def __init__(self, video_path, csv_path):
        super().__init__()

        self.video_path = video_path
        self.csv_path = csv_path

        self.current_index = 0
        self.data = []
        # -------------------------
        # Visualization widgets
        # -------------------------
        self.left_viz = ToolVisualization(NORTH_OFFSET)
        self.right_viz = ToolVisualization(NORTH_OFFSET)
        self.left_viz.setFixedSize(400, 300)
        self.right_viz.setFixedSize(400, 300)

        box_layout = QHBoxLayout()
        box_layout.setContentsMargins(0, 0, 0, 0)
        box_layout.addWidget(self.left_viz, alignment=Qt.AlignBottom | Qt.AlignLeft)
        box_layout.addWidget(self.right_viz, alignment=Qt.AlignBottom | Qt.AlignRight)

        self.visualization_widget = QWidget()
        self.visualization_widget.setLayout(box_layout)

        # Important for overlay
        self.visualization_widget.setAttribute(Qt.WA_TranslucentBackground)
        self.visualization_widget.setStyleSheet("background: transparent;")

        # -------------------------
        # Video
        # -------------------------
        self.video_label = QLabel("Video not started")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setStyleSheet("background-color: black; color: white;")

        # -------------------------
        # Overlay layout
        # -------------------------
        stack_layout = QStackedLayout(self)
        stack_layout.setContentsMargins(0, 0, 0, 0)
        stack_layout.setSpacing(0)
        stack_layout.setStackingMode(QStackedLayout.StackAll)

        stack_layout.addWidget(self.video_label)
        stack_layout.addWidget(self.visualization_widget)

        self.visualization_widget.raise_()
        # -------------------------
        # Replay timer
        # -------------------------
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.next_frame)
        self.capture = None

        self.replay_kinematic_data()

    def update_video_frame(self, frame):
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        h, w, ch = frame.shape
        bytes_per_line = ch * w

        image = QImage(
            frame.data,
            w,
            h,
            bytes_per_line,
            QImage.Format_RGB888,
        ).copy()

        pixmap = QPixmap.fromImage(image)

        self.video_label.setPixmap(
            pixmap.scaled(
                self.video_label.width(),
                self.video_label.height(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation,
            )
        )

    def replay_kinematic_data(self):
        """
        Using kinematic data visualization, replay kinematic data
        side by side with video.

        Keep playing new line / next frame of video as long as
        Enter is held down.

        Arrow keys:
            <- and down: previous frame
            -> and up:   next frame
        """

        # -------------------------
        # Fetch data from CSV
        # -------------------------
        with open(self.csv_path, "r", newline="") as file:
            reader = csv.reader(file)

            # Skip header
            next(reader, None)

            for row in reader:
                try:
                    self.data.append([value for value in row])
                except ValueError:
                    print(f"Skipping invalid CSV row: {row}")

        if not self.data:
            raise ValueError("No valid data found in CSV.")

        # -------------------------
        # Open video
        # -------------------------
        self.capture = cv2.VideoCapture(self.video_path)

        if not self.capture.isOpened():
            raise ValueError(f"Could not open video: {self.video_path}")

        fps = self.capture.get(cv2.CAP_PROP_FPS)

        if fps <= 0:
            fps = 30

        self.frame_interval_ms = max(1, int(1000 / fps))

        # Load first frame/data point
        self.load_index(0)

    def load_index(self, index):
        """
        Load a specific video frame and corresponding CSV row.
        """

        if index < 0:
            return

        if index >= len(self.data):
            return

        video_frame_count = int(self.capture.get(cv2.CAP_PROP_FRAME_COUNT))

        if index >= video_frame_count:
            return

        self.current_index = index

        # -------------------------
        # Load CSV data
        # -------------------------
        row = self.data[index]

        left_q = [float(x) for x in row[LEFT_Q_INDEX].strip("[]").split(",")]
        left_distance = float(row[LEFT_TOF_INDEX])

        right_q = [float(x) for x in row[RIGHT_Q_INDEX].strip("[]").split(",")]
        right_distance = float(row[RIGHT_TOF_INDEX])

        self.left_viz.load_latest_data(left_q, 0)
        self.right_viz.load_latest_data(right_q, 0)

        # -------------------------
        # Get corresponding frame
        # -------------------------
        self.capture.set(
            cv2.CAP_PROP_POS_FRAMES,
            self.current_index,
        )

        success, frame = self.capture.read()

        if success:
            self.update_video_frame(frame)

    def next_frame(self):
        """
        Move forward by one frame.
        """
        next_index = self.current_index + 1

        if next_index >= len(self.data):
            self.timer.stop()
            return

        self.load_index(next_index)

    def previous_frame(self):
        """
        Move backward by one frame.
        """
        previous_index = self.current_index - 1

        if previous_index >= 0:
            self.load_index(previous_index)

    def keyPressEvent(self, event):
        """
        Enter:
            Continuously play while held.

        Right / Up:
            Advance one frame.

        Left / Down:
            Go back one frame.
        """

        if event.key() in (Qt.Key_Return, Qt.Key_Enter):
            if not event.isAutoRepeat() and not self.timer.isActive():
                self.timer.start(self.frame_interval_ms)

        elif event.key() in (Qt.Key_Right, Qt.Key_Up):
            self.next_frame()

        elif event.key() in (Qt.Key_Left, Qt.Key_Down):
            self.previous_frame()

        else:
            super().keyPressEvent(event)

    def keyReleaseEvent(self, event):
        """
        Stop playback when Enter is released.
        """

        if event.key() in (Qt.Key_Return, Qt.Key_Enter):
            if not event.isAutoRepeat():
                self.timer.stop()

        else:
            super().keyReleaseEvent(event)

    def closeEvent(self, event):
        if self.capture is not None:
            self.capture.release()

        event.accept()


if __name__ == "__main__":
    """
    Parse args to load CSV and video path.
    """

    parser = argparse.ArgumentParser(
        description="Replay kinematic CSV data alongside video."
    )

    parser.add_argument(
        "-v",
        "--video",
        help="Path to video file",
    )

    parser.add_argument(
        "-c",
        "--csv",
        help="Path to kinematic CSV file",
    )

    args = parser.parse_args()

    app = QApplication(sys.argv)

    window = ReplayKinematicData(
        video_path=args.video,
        csv_path=args.csv,
    )

    window.resize(1200, 700)
    window.show()

    sys.exit(app.exec())
