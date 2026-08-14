from PyQt5.QtWidgets import QWidget, QVBoxLayout
from PyQt5.QtWidgets import QApplication
import sys
from matplotlib.figure import Figure
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg

import numpy as np
from time import sleep


class SensorVisualization(QWidget):

    def __init__(self, rotations=None, time_interval=10):
        super().__init__()

        # -------------------------
        # Matplotlib setup
        # -------------------------

        self.figure = Figure()
        self.canvas = FigureCanvasQTAgg(self.figure)

        self.ax = self.figure.add_subplot(111, projection="3d")

        layout = QVBoxLayout()
        layout.addWidget(self.canvas)
        self.setLayout(layout)
        self.setStyleSheet("background-color: white;")

        # -------------------------
        # Define IMU box
        # -------------------------

        self.vertices = np.array(
            [
                [-1.5, -0.8, -0.2],
                [1.5, -0.8, -0.2],
                [1.5, 0.8, -0.2],
                [-1.5, 0.8, -0.2],
                [-1.5, -0.8, 0.2],
                [1.5, -0.8, 0.2],
                [1.5, 0.8, 0.2],
                [-1.5, 0.8, 0.2],
            ]
        )

        self.edges = [
            (0, 1),
            (1, 2),
            (2, 3),
            (3, 0),
            (4, 5),
            (5, 6),
            (6, 7),
            (7, 4),
            (0, 4),
            (1, 5),
            (2, 6),
            (3, 7),
        ]

        # create circle to mark screw hole
        self.front_marker = np.array([
            1.2,   # towards +X
            0.6,   # on the +Y face
            0.2    # towards +Z
        ])

        self.marker, = self.ax.plot(
            [],
            [],
            [],
            "ro",      # red circle
            markersize=8
        )

        # Create line objects ONCE
        self.lines = []

        for start, end in self.edges:

            (line,) = self.ax.plot([], [], [])

            self.lines.append(line)

        # -------------------------
        # World frame
        # -------------------------

        self.ax.set_xlim(-2.5, 2.5)
        self.ax.set_ylim(-2.5, 2.5)
        self.ax.set_zlim(-2.5, 2.5)

        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_zlabel("Z")

        self.ax.set_box_aspect((1, 1, 1))

        self.ax.set_title("IMU Orientation")

        # Initial orientation
        self.update_orientation(0, 0, 0)

        if rotations is not None:
            self.rotation_sequence(rotations, time_interval)

    def rotation_matrix(self, roll, pitch, yaw):

        # Degrees -> radians
        roll = np.radians(roll)
        pitch = np.radians(pitch)
        yaw = np.radians(yaw)

        # Roll: X axis
        Rx = np.array(
            [
                [1, 0, 0],
                [0, np.cos(roll), -np.sin(roll)],
                [0, np.sin(roll), np.cos(roll)],
            ]
        )

        # Pitch: Y axis
        Ry = np.array(
            [
                [np.cos(pitch), 0, np.sin(pitch)],
                [0, 1, 0],
                [-np.sin(pitch), 0, np.cos(pitch)],
            ]
        )

        # Yaw: Z axis
        Rz = np.array(
            [[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]]
        )

        # Rotation order
        return Rz @ Ry @ Rx

    def update_orientation(self, roll, pitch, yaw):
        print(roll, pitch, yaw)

        R = self.rotation_matrix(roll, pitch, yaw)

        # Rotate original vertices
        rotated = self.vertices @ R.T

        # Update each edge
        for line, (start, end) in zip(self.lines, self.edges):

            p1 = rotated[start]
            p2 = rotated[end]

            line.set_data([p1[0], p2[0]], [p1[1], p2[1]])

            line.set_3d_properties([p1[2], p2[2]])

        # Rotate Marker
        marker = R @ self.front_marker

        self.marker.set_data(
            [marker[0]],
            [marker[1]]
        )

        self.marker.set_3d_properties(
            [marker[2]]
)

        self.canvas.draw_idle()