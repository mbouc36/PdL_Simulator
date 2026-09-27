"""
Author: Michael Boucouvalas
Date: 2026, Sep 14th
Version: 1.0
Description: Manages creating files and keys and organizing output folders
"""

import os
import csv
import string
import secrets
import pandas as pd
from pathlib import Path

OUTPUT_DATA_FOLDER = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "../output_data"
)

# CSV File Data
NAME_COLUMN = "Name"
KEY_COLUMN = "Key"


class FileManager:
    PEG_TRANSFER = "PT"
    PERCISION_CUTTING = "PC"
    LITIGATION_LOOP = "LL"
    INTRACORPOREAL_SUTURING = "IS"
    EXTRACORPOREAL_SUTURING = "ES"
    WARM_UP = "WU"

    def __init__(self, name_to_key_file):
        self.key = None
        self.create_name_to_key_file(name_to_key_file)
        self.tasks = [
            self.PEG_TRANSFER,
            self.PERCISION_CUTTING,
            self.LITIGATION_LOOP,
            self.INTRACORPOREAL_SUTURING,
            self.EXTRACORPOREAL_SUTURING,
            self.WARM_UP,
        ]
        self.current_task = None
        self.destination_folder = None

    def create_task_folder(self, task):
        if task not in self.tasks:
            print("Failed to create task")
            return None

        if self.user_folder is None:
            self.create_new_user_folder()
        self.current_task = Path(os.path.join(self.user_folder, task))

        self.current_task.mkdir(parents=True, exist_ok=True)

    def create_trial_number(self):
        if self.current_task is None:
            print("No task folder created")
            return None

        children = list(self.current_task.iterdir())

        if len(children) == 0:
            current_trial = Path(os.path.join(self.current_task, "001"))
            current_trial.mkdir(parents=True, exist_ok=True)

        else:
            children.sort(key=int)
            last_task = int(children[-1])
            current_trial = Path(os.path.join(current_trial, str(last_task)))
            current_trial.mkdir(parents=True, exist_ok=True)

        self.destination_folder = current_trial

    def create_new_user_folder(self):
        """
        Create folders for a new user

        The format will be as follows

        output_data
            [Participant ID]
                [Task ID]
                    [Trial Number]
                    ...

        return: the path of the trial for the specfied user on todays date
        """

        self.key = self.create_key()
        self.user_folder = Path(os.path.join(OUTPUT_DATA_FOLDER, self.key))

        # Continue to create key if key already exists
        while not self.is_new_key_vald(self.key):
            self.key = self.create_key()
            self.user_folder = Path(os.path.join(OUTPUT_DATA_FOLDER, self.key))

        print(self.user_folder)

        self.user_folder.mkdir(parents=True, exist_ok=True)

    def update_name_to_key_file(self, name):

        # update name to key 
        if self.is_name_valid(self.name):
            print("Name is invalid")
            exit(1)

        with open(
            self.name_to_key_file, mode="a", newline="", encoding="utf-8"
        ) as file:
            writer = csv.writer(file)
            new_row = [name, self.key]
            writer.writerow(new_row)

    def is_name_valid(self, name) -> bool:
        if name is None:
            return False

        df = pd.read_csv(self.name_to_key_file)
        names = df[NAME_COLUMN]

        if name in names.values:
            return False

        return True

    def is_new_key_vald(self, key):
        if key is None:
            return False

        df = pd.read_csv(self.name_to_key_file)
        keys = df[KEY_COLUMN]

        if key in keys.values:
            return False

        dir_path = Path(os.join(OUTPUT_DATA_FOLDER, key))

        return not dir_path.exists()

    def create_key(self, length=4):

        characters = string.ascii_letters
        nums = string.digits
        first_char = "".join(secrets.choice(characters))

        return first_char.join(secrets.choices(nums) for _ in range(length))

    def create_name_to_key_file(self, file_path):
        
        if file_path is None:
            print("name_to_key_file is not a valid path")
            exit(1)

        self.create_name_to_key_file = Path(file_path)
        if os.path.exists(file_path):
            return

        columns = [NAME_COLUMN, KEY_COLUMN]

        df = pd.DataFrame(columns=columns)

        df.to_csv(file_path, index=False)

    def update_destination_folder(self, task):
        self.create_task_folder(task)
        self.create_task_folder()
        
