"""
Author: Michael Boucouvalas
Date: 2026, Aug 14th
Version: 2.0
Description: Class which is used to visualize the quaternions of an IMU
"""

from PyQt5.QtWidgets import QWidget, QVBoxLayout
from matplotlib.figure import Figure
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg

import numpy as np


class RotationVisualization(QWidget):

    def __init__(self):
        super().__init__()

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

        # Marker on IMU face
        self.front_marker = np.array(
            [
                1.2,
                0.6,
                0.2,
            ]
        )

        (self.marker,) = self.ax.plot(
            [],
            [],
            [],
            "ro",
            markersize=8,
        )

        # Create line objects once
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

        # Identity quaternion:
        # w = 1, x = y = z = 0
        self.update_orientation([1.0, 0.0, 0.0, 0.0])

    def quaternion_to_matrix(self, q):
        """
        q = [w, x, y, z]
        """

        q = np.asarray(q, dtype=float)

        # Normalize quaternion
        q = q / np.linalg.norm(q)

        w, x, y, z = q

        R = np.array(
            [
                [
                    1 - 2 * (y * y + z * z),
                    2 * (x * y - z * w),
                    2 * (x * z + y * w),
                ],
                [
                    2 * (x * y + z * w),
                    1 - 2 * (x * x + z * z),
                    2 * (y * z - x * w),
                ],
                [
                    2 * (x * z - y * w),
                    2 * (y * z + x * w),
                    1 - 2 * (x * x + y * y),
                ],
            ]
        )

        return R

    def update_orientation(self, quaternion):

        R = self.quaternion_to_matrix(quaternion)

        # Rotate original vertices
        rotated = self.vertices @ R.T

        # Update edges
        for line, (start, end) in zip(self.lines, self.edges):

            p1 = rotated[start]
            p2 = rotated[end]

            line.set_data(
                [p1[0], p2[0]],
                [p1[1], p2[1]],
            )

            line.set_3d_properties([p1[2], p2[2]])

        # Rotate marker
        marker = R @ self.front_marker

        self.marker.set_data(
            [marker[0]],
            [marker[1]],
        )

        self.marker.set_3d_properties([marker[2]])

        self.canvas.draw_idle()
