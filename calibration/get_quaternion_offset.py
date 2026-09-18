"""
Author: Michael Boucouvalas
Date: 2026, Sep 6th
Version: 1.0
Description: Script which calculates the direction the simulator is facing and saves the output quaternion
"""

import os
import sys
import json
from time import sleep
from scipy.spatial.transform import Rotation

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from tools.serial_parser import SerialParser
from tools.imu_orientation import IMUQuaternionTracker

CONFIG_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "imu_calibration/cal_data.json"
)

CONIFG_ELEMENT_NAME = "yaw_offset"


def get_sample_quaternions(
    first_imu_name, right_imu_name, num_samples=2000, num_init_samples=400
):
    """
    Get x samples of quaternions from both left and right IMUs, calculate the average
    to get the offset quaternion
    """

    serial_parser = SerialParser()
    left_imu = IMUQuaternionTracker(name=first_imu_name, use_offset=False)
    right_imu = IMUQuaternionTracker(name=right_imu_name, use_offset=False)
    input("Press ENTER when the sensor is positioned in the grooves")
    serial_parser.reset_input_buffer()
    left_samples = []
    right_samples = []

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

    while len(left_samples) < num_samples and len(right_samples) < num_samples:

        serial_line = serial_parser.get_serial_line()

        if serial_line == None:
            continue

        serial_time = serial_line[SerialParser.TIME_INDEX]
        left_imu_data = [serial_time] + serial_line[SerialParser.LEFT_IMU_INDICES]
        right_imu_data = [serial_time] + serial_line[SerialParser.RIGHT_IMU_INDICES]
        left_q = left_imu.get_quaternion(left_imu_data)
        right_q = right_imu.get_quaternion(right_imu_data)

        if left_q is None or right_q is None:
            continue

        left_samples.append(left_q)
        right_samples.append(right_q)

        print(
            f"\rCollecting samples: {len(left_samples)}/{num_samples}",
            end="",
            flush=True,
        )

    print()

    return left_samples, right_samples


def compute_average_quaternion(samples: list[tuple[list[float]]]) -> Rotation:
    """
    Get average quaternion from both left and right imus over the sample size
    """

    # convert Quaternion into roation object
    rotations = Rotation.from_quat(samples)
    return rotations.mean()


def approx_equal_angle(value_1, value_2, degree_tolerance=5):
    diff = abs((value_1 - value_2 + 180) % 360 - 180)
    print(f"Difference between L and R yaw: {diff:.2f}")
    return diff <= degree_tolerance


def write_to_config_file(imu_name, offset_yaw, config_file=CONFIG_PATH):
    """
    Find imu_name in json file, write offset yaw
    """
    with open(config_file, "r", encoding="utf-8") as f:
        data = json.load(f)

    data[imu_name][CONIFG_ELEMENT_NAME] = offset_yaw

    with open(CONFIG_PATH, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=4)


def compute_offset_quaternions(first_imu_name="left", second_imu_name="right"):

    left_samples, right_samples = get_sample_quaternions(
        first_imu_name, second_imu_name
    )
    left_average_rotation = compute_average_quaternion(left_samples)
    right_average_rotation = compute_average_quaternion(right_samples)

    left_offset = round(left_average_rotation.as_euler("xyz", degrees=True)[2], 2)
    right_offset = round(right_average_rotation.as_euler("xyz", degrees=True)[2], 2)
    print(f"Left Offest {left_offset % 360}, Right Offset: {right_offset % 360}")
    write_to_config_file(first_imu_name, left_offset)
    write_to_config_file(second_imu_name, right_offset)


if __name__ == "__main__":
    compute_offset_quaternions()
