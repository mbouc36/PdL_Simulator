"""
Dual-IMU 3D Visualization
=========================
Box dimensions: 18 (length/X) × 10 (width/Y) × 9 cm (height/Z)
IMU coordinate convention: local X→right, Y→up, Z→out-of-board
World convention: ENU  (X=East, Y=North, Z=Up)

The two IMUs are mounted on the two upper-width-edge vertices of the box:
  IMU A → front-left  top vertex  (+X/2, -Y/2, +Z_top)  = ( 9, -5, 9) cm
  IMU B → front-right top vertex  (+X/2, +Y/2, +Z_top)  = ( 9,  5, 9) cm

call update_imu_data(q1, q2, dist1_mm, dist2_mm) to refresh the plot.
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# ---------------------------------------------------------------------------
# Box geometry (cm)
# ---------------------------------------------------------------------------
BOX_L = 18.0   # X
BOX_W = 10.0   # Y
BOX_H = 9.0    # Z

# Upper-width-edge vertices  (front face = +X side)
VERTEX_A = np.array([ BOX_L / 2, -BOX_W / 2,  BOX_H / 2])   # front-left-top
VERTEX_B = np.array([ BOX_L / 2,  BOX_W / 2,  BOX_H / 2])   # front-right-top

# IMU tool geometry (cm, scaled around each vertex origin)
IMU_HALF = np.array([1.5, 0.8, 0.2])   # half-extents of the IMU box visual

COLORS = {
    "A":    "#E04040",   # IMU A → red
    "B":    "#2060CC",   # IMU B → blue
    "box":  "#888888",
    "beam": {"A": "#FF8888", "B": "#88AAFF"},
}

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _box_vertices(half):
    """Return 8 corners of an axis-aligned box with given half-extents."""
    hx, hy, hz = half
    return np.array([
        [-hx, -hy, -hz], [ hx, -hy, -hz], [ hx,  hy, -hz], [-hx,  hy, -hz],
        [-hx, -hy,  hz], [ hx, -hy,  hz], [ hx,  hy,  hz], [-hx,  hy,  hz],
    ])

BOX_EDGES = [
    (0,1),(1,2),(2,3),(3,0),
    (4,5),(5,6),(6,7),(7,4),
    (0,4),(1,5),(2,6),(3,7),
]

def quaternion_to_matrix(q):
    """q = [w, x, y, z] → 3×3 rotation matrix."""
    q = np.asarray(q, dtype=float)
    q = q / np.linalg.norm(q)
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z),  2*(x*y - z*w),      2*(x*z + y*w)     ],
        [2*(x*y + z*w),      1 - 2*(x*x + z*z),  2*(y*z - x*w)     ],
        [2*(x*z - y*w),      2*(y*z + x*w),      1 - 2*(x*x + y*y) ],
    ])


# ---------------------------------------------------------------------------
# Main scene class
# ---------------------------------------------------------------------------

class DualIMUVisualization:
    """
    Renders a static reference box and two animated IMU tool beams.

    Usage
    -----
    vis = DualIMUVisualization()
    vis.update_imu_data(q1, q2, dist1_mm, dist2_mm)
    plt.pause(0.001)   # or plt.show() for blocking
    """

    def __init__(self):
        self.fig = plt.figure(figsize=(12, 8))
        self.ax = self.fig.add_subplot(111, projection="3d")
        self.ax.set_title("Dual-IMU Tool Visualization", fontsize=13, fontweight="bold")

        self._init_static_box()
        self._init_imu_artists()
        self._configure_axes()

        # Default state: identity quaternion, zero distance
        q0 = [1.0, 0.0, 0.0, 0.0]
        self.update_imu_data(q0, q0, 0.0, 0.0)

    # ------------------------------------------------------------------
    # Static reference box
    # ------------------------------------------------------------------

    def _init_static_box(self):
        """Draw the reference box outline."""
        hx, hy, hz = BOX_L / 2, BOX_W / 2, BOX_H / 2
        v = np.array([
            [-hx, -hy, -hz], [ hx, -hy, -hz], [ hx,  hy, -hz], [-hx,  hy, -hz],
            [-hx, -hy,  hz], [ hx, -hy,  hz], [ hx,  hy,  hz], [-hx,  hy,  hz],
        ])
        for i, j in BOX_EDGES:
            self.ax.plot(
                [v[i,0], v[j,0]], [v[i,1], v[j,1]], [v[i,2], v[j,2]],
                color=COLORS["box"], linewidth=1.2, alpha=0.5, zorder=1,
            )

        # Vertex markers
        for label, vertex, color in [
            ("A", VERTEX_A, COLORS["A"]),
            ("B", VERTEX_B, COLORS["B"]),
        ]:
            self.ax.scatter(*vertex, color=color, s=60, zorder=5)
            self.ax.text(vertex[0] + 0.3, vertex[1] + 0.3, vertex[2] + 0.5,
                         f"IMU {label}", color=color, fontsize=9, fontweight="bold")

    # ------------------------------------------------------------------
    # Per-IMU dynamic artists
    # ------------------------------------------------------------------

    def _init_imu_artists(self):
        """
        Create placeholder line objects for both IMUs.
        We store them in dicts keyed by label.
        """
        self.imu_lines = {}          # edge lines for the IMU box
        self.imu_markers = {}        # front-face red/blue dot
        self.imu_beams = {}          # dashed distance line
        self.imu_beam_texts = {}     # distance label
        self.imu_arrows = {}         # +Z quiver arrow (removed/re-added each frame)

        for label, color in [("A", COLORS["A"]), ("B", COLORS["B"])]:
            lines = []
            for _ in BOX_EDGES:
                (ln,) = self.ax.plot([], [], [], color=color, linewidth=1.8, zorder=3)
                lines.append(ln)
            self.imu_lines[label] = lines

            (mk,) = self.ax.plot([], [], [], "o", color=color, markersize=7, zorder=6)
            self.imu_markers[label] = mk

            (bm,) = self.ax.plot([], [], [], "--",
                                 color=COLORS["beam"][label], linewidth=1.5, zorder=2)
            self.imu_beams[label] = bm

            self.imu_beam_texts[label] = self.ax.text(0, 0, 0, "",
                                                       color=color, fontsize=8)
            self.imu_arrows[label] = None

    # ------------------------------------------------------------------
    # Axes styling
    # ------------------------------------------------------------------

    def _configure_axes(self):
        margin = 14
        self.ax.set_xlim(-margin, margin)
        self.ax.set_ylim(-margin, margin)
        self.ax.set_zlim(-margin, margin)
        self.ax.set_xlabel("X (East)  cm", labelpad=6)
        self.ax.set_ylabel("Y (North) cm", labelpad=6)
        self.ax.set_zlabel("Z (Up)    cm", labelpad=6)
        self.ax.set_box_aspect((1, 1, 1))
        self.ax.view_init(elev=25, azim=-60)

        # Legend proxies
        from matplotlib.lines import Line2D
        legend_elements = [
            Line2D([0], [0], color=COLORS["A"], lw=2, label="IMU A (front-left)"),
            Line2D([0], [0], color=COLORS["B"], lw=2, label="IMU B (front-right)"),
            Line2D([0], [0], color=COLORS["box"], lw=1.2, alpha=0.5, label="Reference box"),
        ]
        self.ax.legend(handles=legend_elements, loc="upper left", fontsize=8)

    # ------------------------------------------------------------------
    # Public update method
    # ------------------------------------------------------------------

    def update_imu_data(self, q1, q2, dist1_mm, dist2_mm):
        """
        Parameters
        ----------
        q1, q2       : array-like [w, x, y, z]  quaternions for IMU A and B
        dist1_mm     : float   distance measurement for IMU A (mm)
        dist2_mm     : float   distance measurement for IMU B (mm)
        """
        configs = [
            ("A", q1, dist1_mm, VERTEX_A),
            ("B", q2, dist2_mm, VERTEX_B),
        ]

        beam_ends = [VERTEX_A, VERTEX_B]
        for label, q, dist_mm, vertex in configs:
            beam_end = self._update_single_imu(label, q, dist_mm, vertex)
            beam_ends.append(beam_end)

        self._auto_scale(beam_ends)
        try:
            self.fig.canvas.draw_idle()
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Per-IMU update
    # ------------------------------------------------------------------

    def _update_single_imu(self, label, q, dist_mm, vertex):
        R = quaternion_to_matrix(q)
        dist_cm = dist_mm / 10.0   # convert to cm

        # Local +Z in world frame
        z_dir = R @ np.array([0.0, 0.0, 1.0])

        # Scale the IMU box visually (fixed physical scale for clarity)
        imu_half = IMU_HALF.copy()

        # ---- IMU box ----
        raw_v = _box_vertices(imu_half)
        rotated_v = raw_v @ R.T
        # Translate: origin is the vertex, tool tip is dist_cm along -z_dir from vertex
        # (the sensor face sits at vertex, beam goes outward from there)
        center_offset = (imu_half[2]) * z_dir   # push IMU body back behind vertex
        translated_v = rotated_v + vertex - center_offset

        for line, (i, j) in zip(self.imu_lines[label], BOX_EDGES):
            p1, p2 = translated_v[i], translated_v[j]
            line.set_data([p1[0], p2[0]], [p1[1], p2[1]])
            line.set_3d_properties([p1[2], p2[2]])

        # ---- Front marker (local +Z face corner) ----
        mk_local = np.array([imu_half[0] * 0.6, imu_half[1] * 0.6, imu_half[2]])
        mk_world = R @ mk_local + vertex - center_offset
        self.imu_markers[label].set_data([mk_world[0]], [mk_world[1]])
        self.imu_markers[label].set_3d_properties([mk_world[2]])

        # ---- Distance beam ----
        # Beam from vertex outward along z_dir for dist_cm
        beam_end = vertex + dist_cm * z_dir
        self.imu_beams[label].set_data([vertex[0], beam_end[0]],
                                        [vertex[1], beam_end[1]])
        self.imu_beams[label].set_3d_properties([vertex[2], beam_end[2]])

        # Distance label at midpoint
        mid = (vertex + beam_end) / 2
        self.imu_beam_texts[label].set_position((mid[0], mid[1]))
        self.imu_beam_texts[label].set_3d_properties(mid[2])
        self.imu_beam_texts[label].set_text(f"{dist_mm:.1f} mm")

        return beam_end

        # ---- +Z arrow ----
        if self.imu_arrows[label] is not None:
            self.imu_arrows[label].remove()
        arrow_len = max(1.5, abs(dist_cm) * 0.15 + 1.5)
        color = COLORS["A"] if label == "A" else COLORS["B"]
        self.imu_arrows[label] = self.ax.quiver(
            vertex[0], vertex[1], vertex[2],
            z_dir[0], z_dir[1], z_dir[2],
            length=arrow_len, normalize=True,
            arrow_length_ratio=0.25, linewidth=2,
            color=color,
        )

    # ------------------------------------------------------------------
    # Auto-scale axes to keep everything visible
    # ------------------------------------------------------------------

    def _auto_scale(self, extra_pts=None):
        all_pts = list(extra_pts) if extra_pts else [VERTEX_A, VERTEX_B]

        pts = np.array(all_pts)
        lo = pts.min(axis=0)
        hi = pts.max(axis=0)

        # Include box extents
        lo = np.minimum(lo, [-BOX_L/2 - 2, -BOX_W/2 - 2, -BOX_H/2 - 2])
        hi = np.maximum(hi, [ BOX_L/2 + 2,  BOX_W/2 + 2,  BOX_H/2 + 2])

        mid = (lo + hi) / 2
        half = max((hi - lo).max() / 2, 8)

        self.ax.set_xlim(mid[0] - half, mid[0] + half)
        self.ax.set_ylim(mid[1] - half, mid[1] + half)
        self.ax.set_zlim(mid[2] - half, mid[2] + half)


# ---------------------------------------------------------------------------
# Demo / entry-point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import time

    vis = DualIMUVisualization()
    plt.ion()
    plt.show()

    print("Demo: animating two IMUs with slowly changing quaternions …")
    print("Close the window to exit.")

    t = 0.0
    while plt.fignum_exists(vis.fig.number):
        # Simulate slow rotation for IMU A (tilting forward)
        angle_a = np.deg2rad(30 * np.sin(t * 0.5))
        q1 = [np.cos(angle_a / 2), np.sin(angle_a / 2), 0.0, 0.0]

        # Simulate slow rotation for IMU B (tilting sideways)
        angle_b = np.deg2rad(20 * np.sin(t * 0.4 + 1.0))
        q2 = [np.cos(angle_b / 2), 0.0, np.sin(angle_b / 2), 0.0]

        dist1 = 80 + 40 * np.sin(t * 0.3)     # mm
        dist2 = 60 + 30 * np.cos(t * 0.35)    # mm

        vis.update_imu_data(q1, q2, dist1, dist2)
        plt.pause(0.05)
        t += 0.05

    print("Done.")
