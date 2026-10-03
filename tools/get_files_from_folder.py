"""
Author: Michael Boucouvalas
Date: 2026, Oct 3rd
Version: 1.0
Description: Get output files from task folder
"""

import os
import sys

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from GUI.data_thread import (
    VIDEO_FILENAME,
    RAW_SENSOR_CSV,
    PROCESSED_DATA_CSV,
    CAMERA_TIMESTAMP_CSV,
)


def check_path_valid(path):

    if os.path.exists(path):
        return True
    else:
        print(f"Path:{path} does not exist")
        return False


def get_file_from_folder(folder, file):
    if not check_path_valid(folder):
        return None

    path = os.path.join(folder, file)

    return path


def get_video_file(task_folder):
    get_file_from_folder(task_folder, VIDEO_FILENAME)


def get_raw_data_file(task_folder):
    get_file_from_folder(task_folder, RAW_SENSOR_CSV)


def get_camera_data_file(task_folder):
    get_file_from_folder(task_folder, CAMERA_TIMESTAMP_CSV)


def get_processed_data_file(task_folder):
    get_file_from_folder(task_folder, PROCESSED_DATA_CSV)
