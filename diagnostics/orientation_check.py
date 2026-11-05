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


def get_orientation(
    first_imu_name, right_imu_name, num_samples=100, num_init_samples=400
):
    """
    Get x samples of quaternions from both left and right IMUs, calculate the average
    to get the offset quaternion
    """

    serial_parser = SerialParser()
    tof_manager = TOFManager()
    left_imu = IMUQuaternionTracker(name=first_imu_name, use_offset=True)
    right_imu = IMUQuaternionTracker(name=right_imu_name, use_offset=True)
    input("Press ENTER when tools are in position")
    serial_parser.reset_input_buffer()
    left_q_samples = []
    right_q_samples = []

    left_distances = []
    right_distances = []
    

    sleep(5)
    init_samples = 0
    while init_samples < num_init_samples:
        serial_line = serial_parser.get_serial_line()

        if serial_line == None:
            continue

        serial_time = serial_line[SerialParser.TIME_INDEX]
        left_imu.get_quaternion(
            [serial_time] + serial_line[SerialParser.LEFT_IMU_INDICES]
        )
        right_imu.get_quaternion(
            [serial_time] + serial_line[SerialParser.RIGHT_IMU_INDICES]
        )
        init_samples += 1

        print(
            f"\rInitializing IMUs: {init_samples}/{num_init_samples}",
            end="",
            flush=True,
        )
    print()

    left_imu.set_gain()
    right_imu.set_gain()
    serial_parser.reset_input_buffer()

    while len(left_q_samples) < num_samples and len(right_q_samples) < num_samples:

        serial_line = serial_parser.get_serial_line()

        if serial_line == None:
            continue

        serial_time = serial_line[SerialParser.TIME_INDEX]
        left_imu_data = [serial_time] + serial_line[SerialParser.LEFT_IMU_INDICES]
        right_imu_data = [serial_time] + serial_line[SerialParser.RIGHT_IMU_INDICES]
        tof_data = [serial_line[SerialParser.LEFT_DISTANCE_INDEX], serial_line[SerialParser.RIGHT_DISTANCE_INDEX]]
        left_q = left_imu.get_quaternion(left_imu_data)
        right_q = right_imu.get_quaternion(right_imu_data)
        distances = tof_manager.get_distances(tof_data)

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


def compute_average_quaternion(samples: list[tuple[list[float]]]) -> Rotation:
    """
    Get average quaternion from both left and right imus over the sample size
    """

    # convert Quaternion into roation object
    rotations = Rotation.from_quat(samples, scalar_first=True)
    return rotations.mean()


def compute_offset_quaternions(first_imu_name="left", second_imu_name="right"):

    left_q_samples, right_q_samples, left_distances, right_distances = get_orientation(
        first_imu_name, second_imu_name
    )

    left_distance = sum(left_distances)
    right_distance = sum(right_distances)

    left_average_rotation = compute_average_quaternion(left_q_samples)
    right_average_rotation = compute_average_quaternion(right_q_samples)
    left_average_distance = left_distance / len(left_distances)
    right_average_distance = right_distance / len(right_distances)

    print("Left Quaternion", left_average_rotation.as_euler(EULEUR_ANGLE_CONVENTION,degrees=True), "Left Distance", left_average_distance)
    print("Right Quaternion", right_average_rotation.as_euler(EULEUR_ANGLE_CONVENTION,degrees=True), "Right Distance", right_average_distance)


if __name__ == "__main__":
    compute_offset_quaternions()
