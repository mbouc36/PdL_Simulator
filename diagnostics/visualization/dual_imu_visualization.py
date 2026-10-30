"""
Dual-IMU 3D Visualization — PyQt5 Widget
=========================================
Box dimensions: 18 (length/X) × 10 (width/Y) × 9 cm (height/Z)
World convention: ENU  (X=East, Y=North, Z=Up)

IMUs are mounted on the two upper-width-edge vertices of the box:
  IMU A → front-left  top vertex  = ( 9, -5, 4.5) cm
  IMU B → front-right top vertex  = ( 9,  5, 4.5) cm

Embed in any PyQt5 layout:
    widget = DualIMUVisualization(parent=self)
    layout.addWidget(widget)

Update from any thread-safe call:
    widget.update_imu_data(q1, q2, dist1_mm, dist2_mm)
"""

import numpy as np

from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QMainWindow
from PyQt5.QtCore import Qt, QTimer

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure
from matplotlib.lines import Line2D

TIMEOUT_MS = 200   # render interval in milliseconds (5 Hz)

# ---------------------------------------------------------------------------
# Box geometry (cm)
# ---------------------------------------------------------------------------
BOX_L = 18.0
BOX_W = 10.0
BOX_H = 9.0

VERTEX_A = np.array([ BOX_L / 2, -BOX_W / 2,  BOX_H / 2])   # front-left-top
VERTEX_B = np.array([ BOX_L / 2,  BOX_W / 2,  BOX_H / 2])   # front-right-top

IMU_HALF = np.array([1.5, 0.8, 0.2])   # half-extents of the IMU box visual (cm)

COLORS = {
    "A":    "#E04040",
    "B":    "#2060CC",
    "box":  "#888888",
    "beam": {"A": "#FF8888", "B": "#88AAFF"},
}

BOX_EDGES = [
    (0,1),(1,2),(2,3),(3,0),
    (4,5),(5,6),(6,7),(7,4),
    (0,4),(1,5),(2,6),(3,7),
]

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _imu_box_vertices(half):
    hx, hy, hz = half
    return np.array([
        [-hx,-hy,-hz],[ hx,-hy,-hz],[ hx, hy,-hz],[-hx, hy,-hz],
        [-hx,-hy, hz],[ hx,-hy, hz],[ hx, hy, hz],[-hx, hy, hz],
    ])

def quaternion_to_matrix(q):
    """[w, x, y, z] → 3×3 rotation matrix."""
    q = np.asarray(q, dtype=float)
    q = q / np.linalg.norm(q)
    w, x, y, z = q
    return np.array([
        [1-2*(y*y+z*z),  2*(x*y-z*w),   2*(x*z+y*w)  ],
        [2*(x*y+z*w),    1-2*(x*x+z*z), 2*(y*z-x*w)  ],
        [2*(x*z-y*w),    2*(y*z+x*w),   1-2*(x*x+y*y)],
    ])

# ---------------------------------------------------------------------------
# Main widget
# ---------------------------------------------------------------------------

class DualIMUVisualization(QWidget):
    """
    PyQt5 widget that renders two IMU tool beams on a shared 3-D axes.

    Parameters
    ----------
    parent : QWidget, optional
        Parent widget for Qt ownership.
    north_offset : float
        Initial azimuth of the 3-D camera in degrees.
    """

    def __init__(self, parent=None, north_offset=0):
        super().__init__(parent)

        # ---- Matplotlib figure embedded in Qt ----
        self.figure = Figure()
        self.canvas = FigureCanvasQTAgg(self.figure)
        self.canvas.setParent(self)

        self.ax = self.figure.add_subplot(111, projection="3d")
        self.ax.set_title("Dual-IMU Tool Visualization", fontsize=13, fontweight="bold")

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.canvas)

        # ---- Build scene ----
        self._init_static_box()
        self._init_imu_artists()
        self._configure_axes(north_offset)

        # ---- Internal state ----
        self._imu_arrows = {"A": None, "B": None}

        # Latest sensor data — written by load_latest_data(), read by _redraw()
        self._q1          = [1.0, 0.0, 0.0, 0.0]
        self._q2          = [1.0, 0.0, 0.0, 0.0]
        self._dist1_mm    = 0.0
        self._dist2_mm    = 0.0

        # ---- Initial draw ----
        self._redraw()

        # ---- Timer-driven rendering (mirrors ToolVisualization pattern) ----
        self._timer = QTimer(self)
        self._timer.setTimerType(Qt.TimerType.PreciseTimer)
        self._timer.timeout.connect(self._redraw)
        self._timer.start(TIMEOUT_MS)

    # ------------------------------------------------------------------
    # Camera
    # ------------------------------------------------------------------

    def set_north_offset(self, north_offset):
        """Rotate the camera azimuth without redrawing the data."""
        self.ax.view_init(elev=25, azim=north_offset % 360)
        self.canvas.draw_idle()

    def load_latest_data(self, q1, q2, dist1_mm, dist2_mm):
        """
        Store the latest sensor readings.  Safe to call from any thread or
        callback — does not touch the plot directly.  The timer will pick up
        the values on its next tick.

        Parameters
        ----------
        q1, q2       : array-like [w, x, y, z]
        dist1_mm     : float  distance for IMU A (mm)
        dist2_mm     : float  distance for IMU B (mm)
        """
        self._q1       = q1
        self._q2       = q2
        self._dist1_mm = dist1_mm
        self._dist2_mm = dist2_mm

    # ------------------------------------------------------------------
    # Static reference box
    # ------------------------------------------------------------------

    def _init_static_box(self):
        hx, hy, hz = BOX_L/2, BOX_W/2, BOX_H/2
        v = np.array([
            [-hx,-hy,-hz],[ hx,-hy,-hz],[ hx, hy,-hz],[-hx, hy,-hz],
            [-hx,-hy, hz],[ hx,-hy, hz],[ hx, hy, hz],[-hx, hy, hz],
        ])
        for i, j in BOX_EDGES:
            self.ax.plot(
                [v[i,0],v[j,0]], [v[i,1],v[j,1]], [v[i,2],v[j,2]],
                color=COLORS["box"], linewidth=1.2, alpha=0.5,
            )
        for label, vertex, color in [("A", VERTEX_A, COLORS["A"]),
                                      ("B", VERTEX_B, COLORS["B"])]:
            self.ax.scatter(*vertex, color=color, s=60, zorder=5)
            self.ax.text(vertex[0]+0.3, vertex[1]+0.3, vertex[2]+0.5,
                         f"IMU {label}", color=color, fontsize=9, fontweight="bold")

    # ------------------------------------------------------------------
    # Dynamic artists (created once, updated every frame)
    # ------------------------------------------------------------------

    def _init_imu_artists(self):
        self.imu_lines      = {}
        self.imu_markers    = {}
        self.imu_beams      = {}
        self.imu_beam_texts = {}

        for label, color in [("A", COLORS["A"]), ("B", COLORS["B"])]:
            lines = []
            for _ in BOX_EDGES:
                (ln,) = self.ax.plot([], [], [], color=color, linewidth=1.8)
                lines.append(ln)
            self.imu_lines[label] = lines

            (mk,) = self.ax.plot([], [], [], "o", color=color, markersize=7, zorder=6)
            self.imu_markers[label] = mk

            (bm,) = self.ax.plot([], [], [], "--",
                                  color=COLORS["beam"][label], linewidth=1.5)
            self.imu_beams[label] = bm

            self.imu_beam_texts[label] = self.ax.text(
                0, 0, 0, "", color=color, fontsize=8
            )

    # ------------------------------------------------------------------
    # Axes
    # ------------------------------------------------------------------

    def _configure_axes(self, north_offset=0):
        self.ax.set_xlabel("X (East)  cm", labelpad=6)
        self.ax.set_ylabel("Y (North) cm", labelpad=6)
        self.ax.set_zlabel("Z (Up)    cm", labelpad=6)
        self.ax.set_box_aspect((1, 1, 1))
        self.ax.view_init(elev=25, azim=north_offset % 360)
        self.ax.legend(handles=[
            Line2D([0],[0], color=COLORS["A"],   lw=2,              label="IMU A (front-left)"),
            Line2D([0],[0], color=COLORS["B"],   lw=2,              label="IMU B (front-right)"),
            Line2D([0],[0], color=COLORS["box"], lw=1.2, alpha=0.5, label="Reference box"),
        ], loc="upper left", fontsize=8)

    # ------------------------------------------------------------------
    # Convenience wrapper — store + redraw immediately (optional)
    # ------------------------------------------------------------------

    def update_imu_data(self, q1, q2, dist1_mm, dist2_mm):
        """
        Convenience: store data and force an immediate redraw.
        For timer-driven use, prefer load_latest_data() instead.
        """
        self.load_latest_data(q1, q2, dist1_mm, dist2_mm)
        self._redraw()

    # ------------------------------------------------------------------
    # Timer callback — reads stored state and repaints
    # ------------------------------------------------------------------

    def _redraw(self):
        beam_ends = [VERTEX_A, VERTEX_B]
        for label, q, dist_mm, vertex in [
            ("A", self._q1, self._dist1_mm, VERTEX_A),
            ("B", self._q2, self._dist2_mm, VERTEX_B),
        ]:
            beam_end = self._update_single_imu(label, q, dist_mm, vertex)
            beam_ends.append(beam_end)

        self._auto_scale(beam_ends)
        self.canvas.draw_idle()

    # ------------------------------------------------------------------
    # Per-IMU update
    # ------------------------------------------------------------------

    def _update_single_imu(self, label, q, dist_mm, vertex):
        R        = quaternion_to_matrix(q)
        dist_cm  = dist_mm / 10.0
        color    = COLORS[label]
        z_dir    = R @ np.array([0.0, 0.0, 1.0])

        # Beam end: point along +z_dir from vertex by dist_cm.
        # The IMU's local +Z face is its "front" and must point toward the
        # vertex, so the IMU sits beyond beam_end with its -Z face flush there.
        # Box centre is therefore one half-thickness further out.
        beam_end   = vertex + dist_cm * z_dir
        box_centre = beam_end + IMU_HALF[2] * z_dir   # push box past beam tip

        # ---- IMU box centred at box_centre ----
        translated_v = _imu_box_vertices(IMU_HALF) @ R.T + box_centre
        for line, (i, j) in zip(self.imu_lines[label], BOX_EDGES):
            p1, p2 = translated_v[i], translated_v[j]
            line.set_data([p1[0], p2[0]], [p1[1], p2[1]])
            line.set_3d_properties([p1[2], p2[2]])

        # ---- Front-face marker: local +Z face corner, facing vertex ----
        # Local +Z face is at -hz in our convention (flipped to face vertex),
        # so the marker sits on the -Z face of the box (toward the vertex).
        mk_world = R @ np.array([IMU_HALF[0]*0.6, IMU_HALF[1]*0.6, -IMU_HALF[2]]) + box_centre
        self.imu_markers[label].set_data([mk_world[0]], [mk_world[1]])
        self.imu_markers[label].set_3d_properties([mk_world[2]])

        # ---- Dashed beam: vertex → beam_end (face of IMU toward vertex) ----
        self.imu_beams[label].set_data([vertex[0], beam_end[0]],
                                        [vertex[1], beam_end[1]])
        self.imu_beams[label].set_3d_properties([vertex[2], beam_end[2]])

        # ---- Distance label at beam midpoint ----
        mid = (vertex + beam_end) / 2
        self.imu_beam_texts[label].set_position((mid[0], mid[1]))
        self.imu_beam_texts[label].set_3d_properties(mid[2])
        self.imu_beam_texts[label].set_text(f"{dist_mm:.1f} mm")

        # ---- Arrow points from box back toward vertex (along -z_dir) ----
        if self._imu_arrows[label] is not None:
            self._imu_arrows[label].remove()
        arrow_len = max(1.5, abs(dist_cm) * 0.15 + 1.5)
        self._imu_arrows[label] = self.ax.quiver(
            box_centre[0], box_centre[1], box_centre[2],
            -z_dir[0], -z_dir[1], -z_dir[2],
            length=arrow_len, normalize=True,
            arrow_length_ratio=0.25, linewidth=2,
            color=color,
        )

        return box_centre

    # ------------------------------------------------------------------
    # Auto-scale axes
    # ------------------------------------------------------------------

    def _auto_scale(self, extra_pts=None):
        all_pts = list(extra_pts) if extra_pts else [VERTEX_A, VERTEX_B]
        pts = np.array(all_pts)
        lo = np.minimum(pts.min(axis=0), [-BOX_L/2-2, -BOX_W/2-2, -BOX_H/2-2])
        hi = np.maximum(pts.max(axis=0), [ BOX_L/2+2,  BOX_W/2+2,  BOX_H/2+2])
        mid  = (lo + hi) / 2
        half = max((hi - lo).max() / 2, 8)
        self.ax.set_xlim(mid[0]-half, mid[0]+half)
        self.ax.set_ylim(mid[1]-half, mid[1]+half)
        self.ax.set_zlim(mid[2]-half, mid[2]+half)


# ---------------------------------------------------------------------------
# Minimal demo window
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import sys

    app = QApplication(sys.argv)

    window = QMainWindow()
    window.setWindowTitle("Dual-IMU Visualization Demo")
    window.resize(900, 650)

    imu_widget = DualIMUVisualization(parent=window)
    window.setCentralWidget(imu_widget)
    window.show()

    # Simulate a fast data-producer (e.g. serial read) running independently
    # of the widget's own render timer.  load_latest_data() just stores values;
    # the widget redraws at TIMEOUT_MS regardless of how often data arrives.
    t = 0.0

    def _produce_data():
        global t
        angle_a = np.deg2rad(30 * np.sin(t * 0.5))
        q1 = [np.cos(angle_a/2), np.sin(angle_a/2), 0.0, 0.0]

        angle_b = np.deg2rad(20 * np.sin(t * 0.4 + 1.0))
        q2 = [np.cos(angle_b/2), 0.0, np.sin(angle_b/2), 0.0]

        imu_widget.load_latest_data(
            q1, q2,
            80 + 40 * np.sin(t * 0.3),
            60 + 30 * np.cos(t * 0.35),
        )
        t += 0.05

    data_timer = QTimer()
    data_timer.timeout.connect(_produce_data)
    data_timer.start(50)   # 20 Hz data feed; widget renders at TIMEOUT_MS

    sys.exit(app.exec_())
