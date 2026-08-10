import time
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

MAX_POINTS = 600

class SensorVisualization:
    def __init__(self, max_points=MAX_POINTS):

        self.times = deque(maxlen=max_points)
        self.rolls = deque(maxlen=max_points)
        self.pitches = deque(maxlen=max_points)
        self.yaws = deque(maxlen=max_points)

        fig, self.ax = plt.subplots()
        self.roll_line, = self.ax.plot([], [], label="Roll")
        self.pitch_line, = self.ax.plot([], [], label="Pitch")
        self.yaw_line, = self.ax.plot([], [], label="Yaw")

        self.ax.set_title("Live IMU Orientation")
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Angle (degrees)")
        self.ax.legend()
        self.ax.grid(True)

        self.start_time = time.time()

        self.ani = FuncAnimation(fig, self.update, interval=30)
        plt.show()


    def update(self, data):
        while True:
            try:
                # Data on port should be of form time, roll, pitch, yaw
                t, roll, pitch, yaw = map(float, data.decode().split(","))

                self.times.append(t - self.start_time)
                self.rolls.append(roll)
                self.pitches.append(pitch)
                self.yaws.append(yaw)

            except BlockingIOError:
                break

        if len(self.times) > 0:
            self.roll_line.set_data(self.times, self.rolls)
            self.pitch_line.set_data(self.times, self.pitches)
            self.yaw_line.set_data(self.times, self.yaws)

            # Dynamic x-axis: show last 3 seconds
            xmin = max(0, self.times[-1] - 3)
            xmax = self.times[-1] + 0.1
            self.ax.set_xlim(xmin, xmax)

            # Dynamic y-axis: include roll, pitch, and yaw
            all_values = list(self.rolls) + list(self.pitches) + list(self.yaws)

            ymin = min(all_values)
            ymax = max(all_values)

            padding = max(10, 0.1 * (ymax - ymin))

            self.ax.set_ylim(ymin - padding, ymax + padding)

        return self.roll_line, self.pitch_line, self.yaw_line
