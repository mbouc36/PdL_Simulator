import sys
import os
import serial
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QThread, pyqtSignal
from numpy import ndarray
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), "../.."))
from update_config import load_config
from data_processing.imu_angles import BodyRotationTracker
from view_roll_pitch_yaw import SensorVisualization

config = load_config()

SERIAL_PORT = config["serial_port"]
BAUD_RATE = config["baud_rate"]


class VisualizeSingleIMU(SensorVisualization):
    def __init__(self):
        super().__init__()

        self.data_thread = SingleIMUData()
        self.data_thread.angle_data.connect(self.update_orientation)
        self.data_thread.start()


class SingleIMUData(QThread):
    angle_data = pyqtSignal(ndarray)

    def __init__(self):
        super().__init__()
        #self.visualize_from_serial()

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
        tracker = BodyRotationTracker()

        try:
            while True:
                try:
                    line = ser.readline().decode("utf-8").strip()
                except Exception as e:
                    print(f"Failed to read line: {e}")
                    continue

                q = tracker.get_quaternion(line)
                if q is None:
                    print("Failed to retrived angles")
                    continue

                self.angle_data.emit(q)

        except KeyboardInterrupt:
            print("\nTracking stopped.")


if __name__ == "__main__":

    app = QApplication(sys.argv)
    window = VisualizeSingleIMU()
    window.show()
    sys.exit(app.exec())
