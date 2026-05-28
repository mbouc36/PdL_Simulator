"""
Script ran to create a config.json
Includes functions which load data from the config to all scripts
"""

import json
import os

CONFIG_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "local_config.json"
)


def load_config():
    """
    Loads config from local_config.json
    """
    try:
        with open(CONFIG_PATH, "r", encoding="utf-8") as f:
            config = json.load(f)

    except FileNotFoundError as e:
        print(f"Ensure local_config.json is created by running {__name__}.py")
        exit(1)

    return config


def update_config():
    """
    Update local_config.json
    """
    serial_port = input("What is your serial port: ")
    print(f"Serial port is: {serial_port}")

    baud_rate = input("What is your baud rate (default 115200): ")

    if baud_rate == "":
        baud_rate = 115200
    elif baud_rate != 115200:
        print("Arduino script will need to be updated manually")

    data = {"serial_port": serial_port, "baud_rate": baud_rate}

    # Create file if doesn't exist else update
    with open(CONFIG_PATH, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=4)

    print(f"\n{CONFIG_PATH} Updated")


if __name__ == "__main__":
    update_config()
