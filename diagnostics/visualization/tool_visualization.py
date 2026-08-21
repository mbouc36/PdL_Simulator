"""
Author: Michael Boucouvalas
Date: 2026, Aug 14th
Version: 1.0
Description: Class which is used to visualize the quaternions of an IMU and offset from ToF
"""


from PyQt5.QtWidgets import QWidget, QVBoxLayout
from matplotlib.figure import Figure
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg

import numpy as np


class ToolVisualization(QWidget):

    def __init__(self, north_offset=0):
        super().__init__()

        self.figure = Figure()
        self.canvas = FigureCanvasQTAgg(self.figure)

        self.ax = self.figure.add_subplot(111, projection="3d")
        self.set_north_offset(north_offset)

        layout = QVBoxLayout()
        layout.addWidget(self.canvas)
        self.setLayout(layout)
        self.setStyleSheet("background-color: white;")

        # -------------------------
        # Define IMU box
        # -------------------------

        self.base_vertices = np.array(
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

        self.base_front_marker = np.array(
            [
                1.2,
                0.6,
                0.2,
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

        (self.marker,) = self.ax.plot(
            [],
            [],
            [],
            "ro",
            markersize=8,
        )

        # -------------------------
        # Origin marker
        # -------------------------

        (self.origin_marker,) = self.ax.plot(
            [0],
            [0],
            [0],
            "ko",
            markersize=6,
        )

        # -------------------------
        # Distance line
        # -------------------------

        (self.distance_line,) = self.ax.plot(
            [],
            [],
            [],
            "k--",
            linewidth=1.5,
        )

        # Distance text
        self.distance_text = self.ax.text(
            0,
            0,
            0,
            "",
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
        self.ax.set_title("Tool Orientation")

        # Identity quaternion:
        # w = 1, x = y = z = 0
        self.update_orientation([1.0, 0.0, 0.0, 0.0], 0.0)

    def set_north_offset(self, north_offset):
        self.north_offset = north_offset % 360

        self.ax.view_init(
            elev=25,
            azim=self.north_offset #- 180
        )

        self.canvas.draw_idle()

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

        
    def update_orientation(self, rotation_quaternion, distance):

        R = self.quaternion_to_matrix(rotation_quaternion)

        # ---------------------------------
        # Local +Z direction
        # ---------------------------------

        z_direction = R @ np.array([0.0, 0.0, 1.0])

        # ---------------------------------
        # Visual scaling
        # ---------------------------------

        # Minimum scale keeps the tool its normal size nearby.
        # As distance increases, scale the box proportionally.
        visual_scale = max(1.0, abs(distance) / 5.0)

        vertices = self.base_vertices * visual_scale
        front_marker = self.base_front_marker * visual_scale

        # Thickness of box in the local +Z direction
        half_thickness = 0.2 * visual_scale

        # ---------------------------------
        # Position the tool
        # ---------------------------------

        # Keep the +Z face exactly 'distance' away from origin
        center = (distance - half_thickness) * z_direction

        # ---------------------------------
        # Rotate + translate vertices
        # ---------------------------------

        rotated = vertices @ R.T
        translated = rotated + center

        # Update edges
        for line, (start, end) in zip(self.lines, self.edges):

            p1 = translated[start]
            p2 = translated[end]

            line.set_data(
                [p1[0], p2[0]],
                [p1[1], p2[1]],
            )

            line.set_3d_properties(
                [p1[2], p2[2]]
            )

        # ---------------------------------
        # Rotate + translate marker
        # ---------------------------------

        marker = R @ front_marker
        marker = marker + center

        self.marker.set_data(
            [marker[0]],
            [marker[1]],
        )

        self.marker.set_3d_properties(
            [marker[2]]
        )

        # ---------------------------------
        # Distance line
        # ---------------------------------

        # +Z face center
        face_center = center + half_thickness * z_direction

        self.distance_line.set_data(
            [0, face_center[0]],
            [0, face_center[1]],
        )

        self.distance_line.set_3d_properties(
            [0, face_center[2]]
        )

        # Distance label
        midpoint = face_center / 2

        self.distance_text.set_position(
            (midpoint[0], midpoint[1])
        )

        self.distance_text.set_3d_properties(
            midpoint[2]
        )

        self.distance_text.set_text(
            f"{distance:.2f}"
        )

        # ---------------------------------
        # Dynamically scale background
        # ---------------------------------

        points = np.vstack(
            [
                translated,
                np.array([[0.0, 0.0, 0.0]])
            ]
        )

        min_vals = points.min(axis=0)
        max_vals = points.max(axis=0)

        # Padding proportional to current view size
        padding = max(0.5, abs(distance) * 0.1)

        min_vals -= padding
        max_vals += padding

        ranges = max_vals - min_vals
        max_range = max(ranges)

        midpoints = (max_vals + min_vals) / 2
        half_range = max_range / 2

        self.ax.set_xlim(
            midpoints[0] - half_range,
            midpoints[0] + half_range
        )

        self.ax.set_ylim(
            midpoints[1] - half_range,
            midpoints[1] + half_range
        )

        self.ax.set_zlim(
            midpoints[2] - half_range,
            midpoints[2] + half_range
        )

        self.canvas.draw_idle()