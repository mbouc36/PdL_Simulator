"""
Author: Michael Boucouvalas
Date: 2026, Sep 9th
Version: 1.0
Description: Class for parsing serial data
"""

import os
import sys
import serial

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from update_config import load_config

config = load_config()

SERIAL_PORT = config["serial_port"]
BAUD_RATE = config["baud_rate"]


class SerialParser:
    EXPECTED_NUM_OUTPUT = 23
    TIME_INDEX = 0
    LEFT_WEIGHT_INDEX = 1
    RIGHT_WEIGHT_INDEX = 2
    LEFT_DISTANCE_INDEX = 3
    RIGHT_DISTANCE_INDEX = 4
    LEFT_IMU_INDICES = slice(5, 14)
    RIGHT_IMU_INDICES = slice(14, EXPECTED_NUM_OUTPUT + 1)

    def __init__(self):
        self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

    def get_serial_line(self):
        try:
            line = self.ser.readline().decode("utf-8").strip()
        except Exception as e:
            print(e)
            return None

        raw_sensor_data = line.split(",")
        if not line or len(raw_sensor_data) != self.EXPECTED_NUM_OUTPUT:
            print("Invalid line")
            return None

        return raw_sensor_data

    def reset_input_buffer(self):
        self.reset_input_buffer()

    def close(self):
        self.close()
