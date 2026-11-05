"""
Author: Michael Boucouvalas
Date: 2026, Oct 11
Version: 1.0
Description: Place the tools at a certain position and measure a quaternion, print the value 
"""

import os
import sys
from time import sleep
from scipy.spatial.transform import Rotation

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from tools.serial_parser import SerialParser
from tools.imu_orientation import IMUQuaternionTracker
from tools.tof_manager import TOFManager

CONFIG_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "../calibration/imu_calibration/cal_data.json"
)

CONIFG_ELEMENT_NAME = "heading_offset"

EULEUR_ANGLE_CONVENTION = "xyz"
class OrientationCheck():
    def __init__(self, first_imu_name="left", right_imu_name="right"):
        self.serial_parser = SerialParser()
        self.tof_manager = TOFManager()
        self.left_imu = IMUQuaternionTracker(name=first_imu_name, use_offset=True)
        self.right_imu = IMUQuaternionTracker(name=right_imu_name, use_offset=True)
        self.converge_immus()

    def converge_immus(self, num_init_samples=400):
        print("Hold tools still")
        sleep(5)
        init_samples = 0
        while init_samples < num_init_samples:
            serial_line = self.serial_parser.get_serial_line()

            if serial_line == None:
                continue

            serial_time = serial_line[SerialParser.TIME_INDEX]
            self.left_imu.get_quaternion(
                [serial_time] + serial_line[SerialParser.LEFT_IMU_INDICES]
            )
            self.right_imu.get_quaternion(
                [serial_time] + serial_line[SerialParser.RIGHT_IMU_INDICES]
            )
            init_samples += 1

            print(
                f"\rInitializing IMUs: {init_samples}/{num_init_samples}",
                end="",
                flush=True,
            )
        print()

        self.left_imu.set_gain()
        self.right_imu.set_gain()

    def get_orientation(self, num_samples=100):
        """
        Get x samples of quaternions from both left and right IMUs, 
        """
        self.serial_parser.reset_input_buffer()
        left_q_samples = []
        right_q_samples = []

        left_distances = []
        right_distances = []
        
        self.serial_parser.reset_input_buffer()

        while len(left_q_samples) < num_samples and len(right_q_samples) < num_samples:

            serial_line = self.serial_parser.get_serial_line()

            if serial_line == None:
                continue

            serial_time = serial_line[SerialParser.TIME_INDEX]
            left_imu_data = [serial_time] + serial_line[SerialParser.LEFT_IMU_INDICES]
            right_imu_data = [serial_time] + serial_line[SerialParser.RIGHT_IMU_INDICES]
            tof_data = [serial_line[SerialParser.LEFT_DISTANCE_INDEX], serial_line[SerialParser.RIGHT_DISTANCE_INDEX]]
            left_q = self.left_imu.get_quaternion(left_imu_data)
            right_q = self.right_imu.get_quaternion(right_imu_data)
            distances = self.tof_manager.get_distances(tof_data)

            if left_q is None or right_q is None:
                continue

            left_q_samples.append(left_q)
            right_q_samples.append(right_q)

            left_distances.append(distances[0])
            right_distances.append(distances[1])

            print(
                f"\rCollecting samples: {len(left_q_samples)}/{num_samples}",
                end="",
                flush=True,
            )

        print()

        return left_q_samples, right_q_samples, left_distances, right_distances


    def compute_average_quaternion(self, samples: list[tuple[list[float]]]) -> Rotation:
        """
        Get average quaternion from both left and right imus over the sample size
        """

        # convert Quaternion into roation object
        rotations = Rotation.from_quat(samples, scalar_first=True)
        return rotations.mean()


    def compute_offset_quaternions(self):
        while (True):
            user_input = input("Press ENTER when tools are in position (or type 'exit'): ")
            if user_input.strip().lower() == "exit":
                print("Exiting")
                break 
            else:
                left_q_samples, right_q_samples, left_distances, right_distances = self.get_orientation()

                left_distance = sum(left_distances)
                right_distance = sum(right_distances)

                left_average_rotation = self.compute_average_quaternion(left_q_samples)
                right_average_rotation = self.compute_average_quaternion(right_q_samples)
                left_average_distance = round(left_distance / len(left_distances),2)
                right_average_distance = round(right_distance / len(right_distances), 2)

                print("Left Quaternion", left_average_rotation.as_euler(EULEUR_ANGLE_CONVENTION,degrees=True), "Left Distance", left_average_distance)
                print("Right Quaternion", right_average_rotation.as_euler(EULEUR_ANGLE_CONVENTION,degrees=True), "Right Distance", right_average_distance)


if __name__ == "__main__":
    orientation_check = OrientationCheck()
    orientation_check.compute_offset_quaternions()
