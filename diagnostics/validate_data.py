"""
Author: Michael Boucouvalas
Date: 2026, Sep 27th
Version: 2.0
Description: Validates Sensor and video output data
"""
import os
import cv2 
import sys
import argparse
import pandas as pd
from pathlib import Path

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from diagnostics.get_sample_rate import is_likely_clock_drift, get_time_stats_from_csv
from tools.get_files_from_folder import get_raw_data_file, get_camera_data_file, get_processed_data_file, get_video_file

SAMPLE_RATE_THRESHOLD = 35
MAX_DIFF_SAMPLE_RATE = .5


def frames_equal_csv_rows(video_path, csv_path):
    # Get number of frames in video
    video = cv2.VideoCapture(video_path)

    if not video.isOpened():
        raise ValueError(f"Could not open video: {video_path}")

    frame_count = int(video.get(cv2.CAP_PROP_FRAME_COUNT))
    video.release()

    # Get number of rows in CSV
    df = pd.read_csv(csv_path)
    row_count = len(df)

    print(f"Video frames: {frame_count}")
    print(f"CSV rows:     {row_count}")

    return frame_count == row_count


def is_output_data_valid(output_folder) -> bool:
    video_file = get_video_file(output_folder)
    raw_data = get_raw_data_file(output_folder)
    camera_data = get_camera_data_file(output_folder)
    output_data = get_processed_data_file(output_folder)

    serial_sample_rate = get_time_stats_from_csv(raw_data, has_header=True)
    camera_sample_rate = get_time_stats_from_csv(camera_data, column_index=1, has_header=True) * 1000 #ms

    if serial_sample_rate > SAMPLE_RATE_THRESHOLD or camera_sample_rate > SAMPLE_RATE_THRESHOLD:
        print(f"Sample rates greater than threshold of: {SAMPLE_RATE_THRESHOLD}")
        print(f"Serial: {serial_sample_rate}, Camera: {camera_sample_rate}")
        return False

    if not frames_equal_csv_rows(csv_path=output_data, video_path=video_file):
        return False


    return is_likely_clock_drift(output_data)



if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Validate sensor and video output data"
    )

    parser.add_argument(
        "-c",
        "--csv_file",
        type=Path,
        required=False,
        default=None,
        help="Path to the ouput csv file",
    )

    parser.add_argument(
        "-v",
        "--video_file",
        type=Path,
        required=False,
        default=None,
        help="Path to video output file",
    )

    args = parser.parse_args()
    if not is_output_data_valid(args.csv_file, args.video_file):
        print("Output Invalid")
