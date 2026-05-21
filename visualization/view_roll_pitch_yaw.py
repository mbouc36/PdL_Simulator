import serial
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

SERIAL_PORT = "/dev/cu.wchusbserial1330"  
BAUD_RATE = 115200

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

def rotation_matrix(roll, pitch, yaw):
    r = np.radians(roll)
    p = np.radians(pitch)
    y = np.radians(yaw)

    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(r), -np.sin(r)],
        [0, np.sin(r), np.cos(r)]
    ])

    Ry = np.array([
        [np.cos(p), 0, np.sin(p)],
        [0, 1, 0],
        [-np.sin(p), 0, np.cos(p)]
    ])

    Rz = np.array([
        [np.cos(y), -np.sin(y), 0],
        [np.sin(y), np.cos(y), 0],
        [0, 0, 1]
    ])

    return Rz @ Ry @ Rx


# Flat rectangular plane
plane = np.array([
    [-1, -0.6, 0],
    [ 1, -0.6, 0],
    [ 1,  0.6, 0],
    [-1,  0.6, 0]
])

plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

while True:
    try:
        line = ser.readline().decode(errors="ignore").strip()

        if not line:
            continue

        time, roll, pitch, yaw = map(float, line.split(","))

        R = rotation_matrix(roll, pitch, yaw)
        rotated_plane = plane @ R.T

        ax.clear()

        ax.add_collection3d(
            Poly3DCollection(
                [rotated_plane],
                alpha=0.5,
                edgecolor="black"
            )
        )

        # Draw local axes
        origin = np.array([0, 0, 0])
        x_axis = R @ np.array([1.5, 0, 0])
        y_axis = R @ np.array([0, 1.5, 0])
        z_axis = R @ np.array([0, 0, 1.5])

        ax.quiver(*origin, *x_axis)
        ax.quiver(*origin, *y_axis)
        ax.quiver(*origin, *z_axis)

        ax.set_xlim([-2, 2])
        ax.set_ylim([-2, 2])
        ax.set_zlim([-2, 2])

        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")

        ax.set_title(f"Roll={roll:.1f}, Pitch={pitch:.1f}, Yaw={yaw:.1f}")

        plt.pause(0.01)

    except ValueError:
        print("Bad line:", line)

    except KeyboardInterrupt:
        break

ser.close()