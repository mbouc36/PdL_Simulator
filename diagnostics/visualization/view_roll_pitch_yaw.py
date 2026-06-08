import socket
import time
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

UDP_IP = "127.0.0.1"
UDP_PORT = 5005
MAX_POINTS = 600

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(False)

times = deque(maxlen=MAX_POINTS)
rolls = deque(maxlen=MAX_POINTS)
pitches = deque(maxlen=MAX_POINTS)
yaws = deque(maxlen=MAX_POINTS)

fig, ax = plt.subplots()
roll_line, = ax.plot([], [], label="Roll")
pitch_line, = ax.plot([], [], label="Pitch")
yaw_line, = ax.plot([], [], label="Yaw")

ax.set_title("Live IMU Orientation")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Angle (degrees)")
ax.legend()
ax.grid(True)

start_time = time.time()

def update(frame):
    while True:
        try:
            data, _ = sock.recvfrom(1024)
            t, roll, pitch, yaw = map(float, data.decode().split(","))

            times.append(t - start_time)
            rolls.append(roll)
            pitches.append(pitch)
            yaws.append(yaw)

        except BlockingIOError:
            break

    if len(times) > 0:
        roll_line.set_data(times, rolls)
        pitch_line.set_data(times, pitches)
        yaw_line.set_data(times, yaws)

        # Dynamic x-axis: show last 3 seconds
        xmin = max(0, times[-1] - 3)
        xmax = times[-1] + 0.1
        ax.set_xlim(xmin, xmax)

        # Dynamic y-axis: include roll, pitch, and yaw
        all_values = list(rolls) + list(pitches) + list(yaws)

        ymin = min(all_values)
        ymax = max(all_values)

        padding = max(10, 0.1 * (ymax - ymin))

        ax.set_ylim(ymin - padding, ymax + padding)

    return roll_line, pitch_line, yaw_line


ani = FuncAnimation(fig, update, interval=30)
plt.show()