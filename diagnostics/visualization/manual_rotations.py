"""
Author: Michael Boucouvalas
Date: 2026, Aug 14th
Version: 1.0
Description: Class which is used to manually manipulate visualizations of a quaternion
"""

from PyQt5.QtWidgets import QApplication 
from PyQt5.QtCore import QThread, pyqtSignal
from time import sleep
import sys

from rotation_visualization import RotationVisualization


class DisplayThread(RotationVisualization):
    def __init__(self, rotations, time_interval=5):
        super().__init__()
        self.data_thread = RotationThread(rotations, time_interval)
        self.data_thread.quaternion_data.connect(self.update_orientation)
        self.data_thread.start()


class RotationThread(QThread):
    quaternion_data = pyqtSignal(list)

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
            self.quaternion_data.emit(rotation)
            print(f"Performing rotation: {rotation}")

            sleep(self.time_interval)

        print("Completed rotation sequence")


if __name__ == "__main__":
    # Update list as needed to view rotations
    rotations = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]

    app = QApplication(sys.argv)
    window = DisplayThread(rotations=rotations)
    window.show()
    sys.exit(app.exec())
