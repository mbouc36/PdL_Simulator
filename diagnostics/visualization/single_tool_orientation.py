"""
Author: Michael Boucouvalas
Date: 2026, Aug 14th
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


class VisualizeSingleIMU(ToolVisualization):
    def __init__(self):
        super().__init__(180)

        self.data_thread = SingleIMUData()
        self.data_thread.quaternion_data.connect(self.load_latest_data)
        self.data_thread.start()


class SingleIMUData(QThread):
    quaternion_data = pyqtSignal(list, int)

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

        # Initialize the tracker
        tracker = IMUQuaternionTracker("left_test")

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

                q = tracker.get_quaternion(left_imu_values)
                if q is None:
                    print("Failed to retrived quaternion")
                    continue
                self.quaternion_data.emit(q, 0)

        except KeyboardInterrupt:
            print("\nTracking stopped.")


if __name__ == "__main__":

    app = QApplication(sys.argv)
    window = VisualizeSingleIMU()
    window.show()
    sys.exit(app.exec())
