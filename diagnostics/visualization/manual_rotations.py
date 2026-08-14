from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QThread, pyqtSignal
from time import sleep
import sys

from view_roll_pitch_yaw import SensorVisualization

class DisplayThread(SensorVisualization):
    def __init__(self, rotations, time_interval=5):
        super().__init__()
        self.data_thread = RotationThread(rotations, time_interval)
        self.data_thread.angle_data.connect(self.update_orientation)
        self.data_thread.start()


class RotationThread(QThread):
    angle_data = pyqtSignal(float, float, float)

    def __init__(self, rotations, time_interval):
        super().__init__()
        self.rotations = rotations
        self.time_interval = time_interval

    def run(self):
        """
        Perform a list of rotations sperated by a time interval

        Used to test if visualization is working as expected
        """
        sleep(self.time_interval)
        for rotation in self.rotations:
            print(rotation)
            self.angle_data.emit(rotation[0], rotation[1], rotation[2])
            print(f"Performing rotation: {rotation}")

            sleep(self.time_interval)

        print("Completed rotation sequence")



if __name__ == "__main__":
    # Update list as needed to view rotations
    rotations = [[88, 0, 0], [88, 45, 0], [88, 0, 0], [88, 0, 45]]

    app = QApplication(sys.argv)
    window = DisplayThread(rotations=rotations)
    window.show()
    sys.exit(app.exec())
