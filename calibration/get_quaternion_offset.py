"""
Author: Michael Boucouvalas
Date: 2026, Sep 6th
Version: 1.0
Description: Script which calculates the direction the simulator is facing and saves the output quaternion
"""
import os
import sys
import math
from scipy.spatial.transform import Rotation


sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from tools.imu_orientation import IMUQuaternionTracker
from tools.serial_parser import SerialParser

def get_sample_quaternions(num_samples=300, num_init_samples=200):
    """
    Get x samples of quaternions from both left and right IMUs, calculate the average 
    to get the offset quaternion
    """

    serial_parser = SerialParser()
    left_imu = IMUQuaternionTracker(name="left")
    right_imu = IMUQuaternionTracker(name="right")
    input("Press ENTER when the sensor is positioned in the grooves")
    serial_parser.reset_input_buffer()
    left_samples = []
    right_samples = []

    init_samples = 0
    while init_samples < num_init_samples:
        serial_line = serial_parser.get_serial_line()

        if serial_line == None:
            continue

        left_imu.get_quaternion(serial_line[SerialParser.LEFT_IMU_INDICES])
        right_imu.get_quaternion(serial_line[SerialParser.RIGHT_IMU_INDICES])

    left_imu.set_gain()
    right_imu.set_gain()
    serial_parser.reset_input_buffer()

    while len(left_samples) < num_samples and len(right_samples) < num_samples:

        serial_line = serial_parser.get_serial_line()

        if serial_line == None:
            continue

        left_q = left_imu.get_quaternion(serial_line[SerialParser.LEFT_IMU_INDICES])
        right_q = right_imu.get_quaternion(serial_line[SerialParser.RIGHT_IMU_INDICES])
        left_samples.append(left_q)
        right_samples.append(right_q)

        print(
            f"\rCollecting samples: {len(left_samples)}/{num_samples}",
            end="",
            flush=True,
        )

    print()

    return left_samples, right_samples


def compute_average_quaternion(samples: list[tuple[list[float]]] ) -> Rotation:
    """
    Get average quaternion from both left and right imus over the sample size
    """

    # convert Quaternion into roation object
    rotations = Rotation.from_quat(samples)
    return rotations.mean().as_quat()


def compute_offset_quaternion(degree_tolerance=15):


    left_samples, right_samples = get_sample_quaternions()
    left_average_rotation = compute_average_quaternion(left_samples)
    right_average_rotation = compute_average_quaternion(right_samples)

    similar_rotations = left_average_rotation.approx_equal(other=right_average_rotation, atol=math.radians(degree_tolerance))
    print(similar_rotations)
    if similar_rotations[3] == True:
        average_rotation = Rotation.concatenate([left_average_rotation, right_average_rotation])
        return average_rotation
    else:
        print(f"Rotations invalid ")


if __name__ == "__main__":
    compute_average_quaternion()