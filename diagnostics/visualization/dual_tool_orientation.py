"""
Author: Michael Boucouvalas
Date: 2026, Aug 26th
Version: 2.0
Description: Use IMU and ToF distance to get orientation in a quaternion visualization
"""

import sys
import os
import serial
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QThread, pyqtSignal

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), "../.."))
from update_config import load_config
from data_processing.imu_orientation import IMUQuaternionTracker
from diagnostics.visualization.tool_visualization import ToolVisualization

config = load_config()

SERIAL_PORT = config["serial_port"]
BAUD_RATE = config["baud_rate"]

NORTH_OFFSET = 180


class VisualizeSingleIMU:
    def __init__(self):

        self.left_viz = ToolVisualization(NORTH_OFFSET)
        self.right_viz = ToolVisualization(NORTH_OFFSET)
        self.data_thread = DualIMUData()
        self.data_thread.quaternion_data.connect(self.load_data)
        self.data_thread.start()

    def load_data(self, data):
        self.left_viz.load_latest_data(data[0], data[1])
        self.right_viz.load_latest_data(data[2], data[3])


class DualIMUData(QThread):
    quaternion_data = pyqtSignal(list)

    def __init__(self):
        super().__init__()
        # self.visualize_from_serial()

    def run(self):
        """
        Visualize the data from the serial port
        """
        try:
            ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        except Exception:
            print(f"Failed to connect to port: {SERIAL_PORT}")
            exit(1)

        # Initialize the left_tracker
        left_tracker = IMUQuaternionTracker("left")
        right_tracker = IMUQuaternionTracker("right")

        try:
            while True:
                try:
                    line = ser.readline().decode("utf-8").strip()
                    line = line.split(",")
                except Exception as e:
                    print(f"Failed to read line: {e}")
                    continue

                if not line:
                    continue

                arduino_time = line[0]
                left_imu_values = [arduino_time] + line[5:14]
                right_imu_values = [arduino_time] + line[14:]

                left_q = left_tracker.get_quaternion(left_imu_values)
                if left_q is None:
                    print("Failed to retrived left quaternion")
                    continue
                

                right_q = right_tracker.get_quaternion(right_imu_values)
                if right_q is None:
                    print("Failed to retrived right quaternion")
                    continue

                data = [left_q, 0, right_q, 0]
                self.quaternion_data.emit(data)

        except KeyboardInterrupt:
            print("\nTracking stopped.")


if __name__ == "__main__":

    app = QApplication(sys.argv)
    window = VisualizeSingleIMU()
    window.show()
    sys.exit(app.exec())
