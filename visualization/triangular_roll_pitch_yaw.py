import math
from collections import deque
from dataclasses import dataclass

import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from PyQt5 import QtGui, QtWidgets
from pyqtgraph.opengl import GLViewWidget


TRAIL_MAX_POINTS = 100
EPSILON = 1e-6


@dataclass
class AttitudeData:
    timestamp: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0


class AttitudeVisualizer(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()

        self.data = AttitudeData()
        self.items = {}
        self.trail_buffer = deque(maxlen=TRAIL_MAX_POINTS)

        self.setWindowTitle("3D Attitude Visualizer")
        self.setMinimumSize(900, 800)

        self.layout = QtWidgets.QGridLayout(self)

        self._setup_3d_view()
        self._setup_attitude_plots()

    # =========================
    # Public API
    # =========================
    def update_data(self, timestamp: float, roll: float, pitch: float, yaw: float):
        """
        Call this from another class whenever new attitude data is available.
        """
        self.data = AttitudeData(timestamp, roll, pitch, yaw)
        self.update_visuals()

    # =========================
    # Setup
    # =========================
    def _setup_3d_view(self):
        self.view_3d = GLViewWidget()
        self.view_3d.setCameraPosition(distance=3, elevation=30, azimuth=45)
        self.layout.addWidget(self.view_3d, 0, 0)

        self._add_reference_sphere()
        self._add_xy_circle()
        self._add_axes()

        self.vector_arrow = self._make_line((1, 0, 0, 1), 3)
        self.xy_projection = self._make_line((0, 1, 1, 1), 2)
        self.vertical_component = self._make_line((1, 0, 1, 1), 2)
        self.vector_trail = self._make_line((1, 1, 0, 0.6), 1)

    def _add_reference_sphere(self):
        mesh = gl.MeshData.sphere(rows=20, cols=20, radius=1.0)
        sphere = gl.GLMeshItem(
            meshdata=mesh,
            smooth=True,
            color=(0.3, 0.3, 0.3, 0.2),
            shader="shaded",
            drawFaces=True,
        )
        self.view_3d.addItem(sphere)

    def _add_xy_circle(self):
        mesh = gl.MeshData.cylinder(
            rows=1,
            cols=50,
            radius=[1.0, 1.0],
            length=0.001,
        )
        circle = gl.GLMeshItem(
            meshdata=mesh,
            smooth=True,
            color=(1, 0.5, 0, 0.4),
            shader="shaded",
            glOptions="additive",
        )
        circle.rotate(90, 1, 0, 0)
        circle.setDepthValue(-5)
        self.view_3d.addItem(circle)

    def _add_axes(self):
        axes = [
            ((1.2, 0, 0), (1, 0, 0, 1)),
            ((0, 1.2, 0), (0, 1, 0, 1)),
            ((0, 0, 1.2), (0, 0, 1, 1)),
        ]

        for endpoint, color in axes:
            line = gl.GLLinePlotItem(
                pos=np.array([[0, 0, 0], endpoint]),
                color=color,
                width=2,
                antialias=True,
            )
            self.view_3d.addItem(line)

    def _make_line(self, color, width):
        line = gl.GLLinePlotItem(
            pos=np.array([[0, 0, 0], [0, 0, 0]]),
            color=color,
            width=width,
            antialias=True,
        )
        line.setDepthValue(10)
        self.view_3d.addItem(line)
        return line

    def _setup_attitude_plots(self):
        self.roll_plot = self._create_roll_plot()
        self.pitch_plot = self._create_pitch_plot()
        self.yaw_plot = self._create_yaw_plot()

        self.layout.addWidget(self.roll_plot, 0, 1)
        self.layout.addWidget(self.pitch_plot, 1, 0)
        self.layout.addWidget(self.yaw_plot, 1, 1)

    def _make_attitude_plot(self, title):
        plot = pg.PlotWidget(title=title)
        plot.setMinimumSize(250, 250)
        plot.setRange(xRange=(-1.5, 1.5), yRange=(-1.5, 1.5))
        plot.setAspectLocked(True)
        plot.hideAxis("bottom")
        plot.hideAxis("left")
        return plot

    def _add_readout(self, plot, key, y=-1.4):
        readout = pg.TextItem("0.0°", anchor=(0.5, 0.5), color=(0, 255, 0))
        readout.setFont(QtGui.QFont("Courier New", 14, QtGui.QFont.Bold))
        readout.setPos(0, y)
        readout.setZValue(15)
        plot.addItem(readout)
        self.items[key] = readout

    def _add_sky_ground(self, plot):
        sky = pg.PlotDataItem(
            x=[-1.5, 1.5, 1.5, -1.5],
            y=[0, 0, 1.5, 1.5],
            pen=None,
            fillLevel=1.5,
            brush=pg.mkBrush(0, 191, 255, 255),
        )

        ground = pg.PlotDataItem(
            x=[-1.5, 1.5, 1.5, -1.5],
            y=[-1.5, -1.5, 0, 0],
            pen=None,
            fillLevel=-1.5,
            brush=pg.mkBrush(139, 69, 19, 255),
        )

        plot.addItem(sky)
        plot.addItem(ground)

    def _create_roll_plot(self):
        plot = self._make_attitude_plot("Roll (°)")
        self._add_sky_ground(plot)
        self._add_readout(plot, "roll_readout")

        horizon = pg.PlotDataItem(
            x=[-1, 1],
            y=[0, 0],
            pen=pg.mkPen(color=(0, 100, 0), width=3),
        )
        plot.addItem(horizon)
        self.items["roll_horizon"] = horizon

        return plot

    def _create_pitch_plot(self):
        plot = self._make_attitude_plot("Pitch (°)")
        self._add_sky_ground(plot)
        self._add_readout(plot, "pitch_readout")

        self.pitch_angles = [-90, -60, -30, -10, 0, 10, 30, 60, 90]
        self.items["pitch_ladder"] = []

        for angle in self.pitch_angles:
            line = self._make_pitch_ladder_line(angle)
            plot.addItem(line)
            self.items["pitch_ladder"].append(line)

        return plot

    def _make_pitch_ladder_line(self, angle):
        y = angle / 90 * 1.2

        if angle == 0:
            x_values = [-1, 1]
            y_values = [y, y]
            pen = pg.mkPen(color=(0, 100, 0), width=3)
        else:
            x_values = [-0.5, -0.2, -0.2, 0.2, 0.2, 0.5]
            y_values = [y, y, y + 0.05, y + 0.05, y, y]
            pen = pg.mkPen(color=(0, 100, 0), width=2)

        return pg.PlotDataItem(x=x_values, y=y_values, pen=pen)

    def _create_yaw_plot(self):
        plot = self._make_attitude_plot("Yaw (°)")
        self._add_readout(plot, "yaw_readout", y=-1.5)

        theta = np.linspace(0, 2 * np.pi, 100)
        circle = pg.PlotDataItem(
            x=1.2 * np.sin(theta),
            y=1.2 * np.cos(theta),
            pen=pg.mkPen("w", width=2),
        )
        plot.addItem(circle)

        needle = pg.PlotDataItem(
            x=[0, 0],
            y=[0, 1],
            pen=pg.mkPen("r", width=3),
        )
        plot.addItem(needle)
        self.items["yaw_needle"] = needle

        return plot

    # =========================
    # Updates
    # =========================
    def update_visuals(self):
        roll = self.data.roll
        pitch = self.data.pitch
        yaw = self.data.yaw

        self.items["roll_readout"].setText(f"Roll: {roll:+.1f}°")
        self.items["pitch_readout"].setText(f"Pitch: {pitch:+.1f}°")
        self.items["yaw_readout"].setText(f"Yaw: {yaw:.1f}°")

        self._update_3d_attitude_vector(pitch, yaw)
        self._update_roll_horizon(roll)
        self._update_pitch_ladder(pitch)
        self._update_yaw_needle(yaw)

    def _update_3d_attitude_vector(self, pitch, yaw):
        pitch_rad = math.radians(pitch)
        yaw_rad = math.radians(yaw)

        vector = np.array(
            [
                math.cos(pitch_rad) * math.sin(yaw_rad),
                math.cos(pitch_rad) * math.cos(yaw_rad),
                math.sin(pitch_rad),
            ]
        )

        norm = np.linalg.norm(vector)
        if norm < EPSILON:
            return

        unit_vector = vector / norm
        xy_projection = np.array([unit_vector[0], unit_vector[1], 0])

        self.vector_arrow.setData(pos=np.array([[0, 0, 0], unit_vector]))
        self.xy_projection.setData(pos=np.array([[0, 0, 0], xy_projection]))
        self.vertical_component.setData(pos=np.array([xy_projection, unit_vector]))

        self.trail_buffer.append(unit_vector)
        self.vector_trail.setData(pos=np.array(self.trail_buffer))

    def _update_roll_horizon(self, roll):
        roll_rad = math.radians(roll)

        x1 = -math.cos(roll_rad)
        y1 = math.sin(roll_rad)
        x2 = math.cos(roll_rad)
        y2 = -math.sin(roll_rad)

        self.items["roll_horizon"].setData(x=[x1, x2], y=[y1, y2])

    def _update_pitch_ladder(self, pitch):
        pitch_offset = -pitch / 90 * 1.2

        for index, angle in enumerate(self.pitch_angles):
            y_base = angle / 90 * 1.2
            y = y_base + pitch_offset
            line = self.items["pitch_ladder"][index]

            if angle == 0:
                line.setData(x=[-1, 1], y=[y, y])
            else:
                line.setData(
                    x=[-0.5, -0.2, -0.2, 0.2, 0.2, 0.5],
                    y=[y, y, y + 0.05, y + 0.05, y, y],
                )

    def _update_yaw_needle(self, yaw):
        yaw_rad = math.radians(yaw)

        needle_x = math.sin(yaw_rad)
        needle_y = math.cos(yaw_rad)

        self.items["yaw_needle"].setData(x=[0, needle_x], y=[0, needle_y])