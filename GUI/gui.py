"""
Author: Michael Boucouvalas
Date: 2026, Aug 17th
Version: 2.0
Description: Script which contains class used to manage gui
"""

import sys
import os
import cv2
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import (
    QWidget,
    QPushButton,
    QLabel,
    QVBoxLayout,
    QStackedWidget,
    QStackedLayout,
    QLineEdit,
    QHBoxLayout,
)

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from diagnostics.visualization.tool_visualization import ToolVisualization
from data_thread import DataThread
from file_manager import FileManager

class GUI(QWidget):
    def __init__(self, name_to_key_file, test_mode=False, visualize=False):
        super().__init__()

        self.setWindowTitle("GUI with Livestream")
        self.resize(900, 600)

        self.name = ""
        self.key = ""
        self.task_type = None
        self.create_name_to_key_file(name_to_key_file)
        self.name_to_key_file = name_to_key_file
        self.visualize = visualize

        self.data_thread = None
        self.pages = QStackedWidget()

        self.btn_style = """
        QPushButton {
            background-color: #FFFFF0;
            color: black;
            border: 1px solid #D8D0C0;   /* Thin warm gray outline */
            border-radius: 12px;
            padding: 10px;
            font-size: 16px;
            font-family:  "Times New Roman", Times, serif;
        }

        QPushButton:hover {
            background-color: #FAF0E6;
            border: 1px solid #C8C0B0;
        }

        QPushButton:pressed {
            background-color: #FDF6E3;
            border: 1px solid #B8B0A0;
        }
        """

        # Create User Pages
        self.login_page = self.create_login_page()
        self.video_page = self.create_video_page()
        self.new_user_page = self.create_new_user_page()
        self.existing_user_page = self.create_existing_user_page()
        self.task_menu_page = self.create_task_menu()
        self.post_task_page = self.create_post_task_menu()

        main_layout = QVBoxLayout()
        main_layout.addWidget(self.pages)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)
        self.setLayout(main_layout)
        self.test_mode = test_mode
        self.setStyleSheet("""
            QWidget {
                background-color: #FFFFF0;
            }
        """)

        self.file_manager = FileManager(name_to_key_file=name_to_key_file)

    def create_login_page(self):
        start_page = QWidget()
        layout = QVBoxLayout()

        title = QLabel("Laparoscopic Simulator")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("""
            font-size: 48px;
            font-weight: bold;
            color: black;
            font-family:  "Times New Roman", Times, serif;
            margin-top: 48px;
            margin-bottom: 24px;
        """)

        new_user_btn = QPushButton("New User")
        existing_user_btn = QPushButton("Existing User")

        for btn in [new_user_btn, existing_user_btn]:
            btn.setFixedHeight(50)
            btn.setMinimumWidth(250)
            btn.setMaximumWidth(400)
            btn.setStyleSheet(self.btn_style)

        new_user_btn.clicked.connect(
            lambda: self.pages.setCurrentWidget(self.new_user_page)
        )

        existing_user_btn.clicked.connect(
            lambda: self.pages.setCurrentWidget(self.existing_user_page)
        )

        layout.addStretch()
        layout.addWidget(title)
        layout.addWidget(new_user_btn, alignment=Qt.AlignCenter)
        layout.addWidget(existing_user_btn, alignment=Qt.AlignCenter)
        layout.addStretch()

        start_page.setLayout(layout)
        self.pages.addWidget(start_page)

        return start_page

    def create_video_page(self):
        video_page = QWidget()
        video_page.setStyleSheet("background-color: black;")

        stack_layout = QStackedLayout(video_page)
        stack_layout.setContentsMargins(0, 0, 0, 0)
        stack_layout.setSpacing(0)
        stack_layout.setStackingMode(QStackedLayout.StackAll)

        self.video_label = QLabel("Video not started")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setStyleSheet("background-color: black; color: white;")
        self.video_label.setScaledContents(True)

        overlay = QWidget()
        overlay.setAttribute(Qt.WA_TransparentForMouseEvents, False)
        overlay.setStyleSheet("background: transparent;")

        overlay_layout = QVBoxLayout(overlay)
        overlay_layout.setContentsMargins(20, 20, 20, 20)
        overlay_layout.setSpacing(0)

        complete_btn = QPushButton("Complete Task")
        complete_btn.setFixedSize(120, 50)

        def complete_task():
            self.data_thread.stop()
            self.pages.setCurrentWidget(self.post_task_page)

        complete_btn.clicked.connect(lambda checked=False: complete_task())

        complete_btn.setStyleSheet("""
            QPushButton {
                background-color: #FFFFF0;
                color: black;
                border: 1px solid #D8D0C0;
                border-radius: 12px;
                font-size: 16px;
                font-family: "Times New Roman";
            }
        """)

        overlay_layout.addWidget(complete_btn, alignment=Qt.AlignTop | Qt.AlignRight)
        overlay_layout.addStretch()
        if self.visualize:
            self.left_imu_visualization = ToolVisualization(180)
            self.right_imu_visualization = ToolVisualization(180)
            self.left_imu_visualization.setFixedSize(400, 300)
            self.right_imu_visualization.setFixedSize(400, 300)
            visualization_box = QHBoxLayout()

            visualization_box.addWidget(
                self.left_imu_visualization, alignment=Qt.AlignBottom | Qt.AlignLeft
            )
            visualization_box.addWidget(
                self.right_imu_visualization, alignment=Qt.AlignBottom | Qt.AlignRight
            )
            overlay_layout.addLayout(visualization_box)

        stack_layout.addWidget(self.video_label)
        stack_layout.addWidget(overlay)

        # Important: make overlay the top/current widget
        stack_layout.setCurrentWidget(overlay)

        self.pages.addWidget(video_page)

        return video_page

    def show_video_page(self):
        self.pages.setCurrentIndex(1)
        if not self.test_mode:
            self.start_video()
        else:
            pass

    def start_video(self):

        # If in test mode don't create data thread
        if self.test_mode:
            return

        if self.data_thread is None:
            self.data_thread = DataThread(self.file_manager.destination_folder, self.visualize)
            self.data_thread.frame_ready.connect(self.update_video_frame)
            if self.visualize:
                self.data_thread.sensor_data.connect(self.update_visulization)

        self.data_thread.start()

    def update_video_frame(self, frame):
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        h, w, ch = frame.shape
        bytes_per_line = ch * w

        image = QImage(frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(image)

        self.video_label.setPixmap(
            pixmap.scaled(
                self.video_label.width(),
                self.video_label.height(),
                Qt.KeepAspectRatio,
            )
        )

    def update_visulization(self, data):
        self.left_imu_visualization.load_latest_data(data[5], 0)
        self.right_imu_visualization.load_latest_data(data[6], 0)

    def create_new_user_page(self):
        """
        GUI page which prompts the user for a username
        """

        new_user_page = QWidget()
        layout = QVBoxLayout()

        title = QLabel("Create New User")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("""
            font-size: 48px;
            font-weight: bold;
            color: black;
            font-family:  "Times New Roman", Times, serif;
            margin-top: 180px;
            margin-bottom: 12px;
        """)

        name_label = QLabel("Name:")
        name_label.setStyleSheet("""
            font-size: 24px;
            font-family: "Times New Roman";
            color: black;
        """)

        name_box = QLineEdit()
        name_box.setFixedHeight(45)

        name_box.setStyleSheet("""
            QLineEdit {
                font-size: 24px;
                font-family: "Times New Roman";
                color: black;
                background-color: transparent;
                border: 1px solid #D8D0C0;
                padding: 4px 8px;
            }
        """)
        name_box.setPlaceholderText("Enter your name")
        name_box.setFocus()

        text_box_row = QHBoxLayout()
        text_box_row.addStretch()
        text_box_row.addWidget(name_label)
        text_box_row.addWidget(name_box)
        text_box_row.addStretch()

        next_btn = QPushButton("Next")
        back_btn = QPushButton("Back")

        for btn in [next_btn, back_btn]:
            btn.setFixedHeight(50)
            btn.setMinimumWidth(250)
            btn.setMaximumWidth(400)
            btn.setStyleSheet(self.btn_style)

        # Bottom button
        bottom_row = QHBoxLayout()
        bottom_row.addWidget(back_btn, alignment=Qt.AlignLeft)
        bottom_row.addStretch()
        bottom_row.addWidget(next_btn, alignment=Qt.AlignRight)

        back_btn.clicked.connect(lambda: self.pages.setCurrentWidget(self.login_page))

        error_label = QLabel("")
        error_label.setAlignment(Qt.AlignCenter)
        error_label.setStyleSheet("""
            color: red;
            font-size: 18px;
            font-family: "Times New Roman";
        """)

        layout.addWidget(title)
        layout.addSpacing(10)
        layout.addLayout(text_box_row)
        layout.addWidget(error_label)
        layout.addStretch()
        layout.addLayout(bottom_row)

        new_user_page.setLayout(layout)
        self.pages.addWidget(new_user_page)

        def validate_username():
            name = name_box.text().strip()

            if not name or not self.is_name_valid(name):
                error_label.setText("Please enter a valid name.")
                return

            # Clear any previous error
            error_label.setText("")
            name_box.clear()
            self.name = name

            self.pages.setCurrentWidget(self.task_menu_page)
            return

        next_btn.clicked.connect(lambda checked=False: validate_username())

        return new_user_page

    def create_existing_user_page(self):
        """
        GUI page which promts the user for a key

        If a matching key isn't found notify the user and allow retries
        """
        existing_user_page = QWidget()
        layout = QVBoxLayout()

        title = QLabel("Existing User")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("""
            font-size: 48px;
            font-weight: bold;
            color: black;
            font-family:  "Times New Roman", Times, serif;
            margin-top: 180px;
            margin-bottom: 12px;
        """)

        key_label = QLabel("Name:")
        key_label.setStyleSheet("""
            font-size: 24px;
            font-family: "Times New Roman";
            color: black;
        """)
        key_box = QLineEdit()
        key_box.setFixedHeight(45)

        key_box.setStyleSheet("""
            QLineEdit {
                font-size: 24px;
                font-family: "Times New Roman";
                color: black;
                background-color: transparent;
                border: 1px solid #D8D0C0;
                padding: 4px 8px;
            }
        """)
        key_box.setPlaceholderText("Enter your key")

        text_box_row = QHBoxLayout()
        text_box_row.addStretch()
        text_box_row.addWidget(key_label)
        text_box_row.addWidget(key_box)
        text_box_row.addStretch()

        next_btn = QPushButton("Next")
        back_btn = QPushButton("Back")

        for btn in [next_btn, back_btn]:
            btn.setFixedHeight(50)
            btn.setMinimumWidth(250)
            btn.setMaximumWidth(400)
            btn.setStyleSheet(self.btn_style)

        # Bottom butt21
        bottom_row = QHBoxLayout()
        bottom_row.addWidget(back_btn, alignment=Qt.AlignLeft)
        bottom_row.addStretch()
        bottom_row.addWidget(next_btn, alignment=Qt.AlignRight)

        back_btn.clicked.connect(lambda: self.pages.setCurrentWidget(self.login_page))

        existing_user_page.setLayout(layout)
        self.pages.addWidget(existing_user_page)

        error_label = QLabel("")
        error_label.setAlignment(Qt.AlignCenter)
        error_label.setStyleSheet("""
            color: red;
            font-size: 18px;
            font-family: "Times New Roman";
        """)

        layout.addWidget(title)
        layout.addSpacing(10)
        layout.addLayout(text_box_row)
        layout.addWidget(error_label)
        layout.addStretch()
        layout.addLayout(bottom_row)

        def validate_key():
            key = key_box.text().strip()

            if not key and not self.is_key_vald(key):
                error_label.setText("Please enter a valid key.")
                return

            # Clear any previous error
            error_label.setText("")
            self.key = key

            self.pages.setCurrentWidget(self.task_menu_page)
            return

        next_btn.clicked.connect(lambda checked=False: validate_key())

        return existing_user_page

    def create_task_menu(self):
        """
        A menu which lists out tasks
        """
        task_menu_page = QWidget()
        layout = QVBoxLayout()

        title = QLabel("Task Menu")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("""
            font-size: 48px;
            font-weight: bold;
            color: black;
            font-family:  "Times New Roman", Times, serif;
            margin-top: 24px;
            margin-bottom: 12px;
        """)

        peg_transfer_btn = QPushButton("Peg Transfer")
        in_suturing_btn = QPushButton("Intracorporeal Suturing")

        for btn in [peg_transfer_btn, in_suturing_btn]:
            btn.setFixedHeight(50)
            btn.setMinimumWidth(250)
            btn.setMaximumWidth(400)
            btn.setStyleSheet(self.btn_style)

        layout.addStretch()
        layout.addWidget(title)
        layout.addWidget(peg_transfer_btn, alignment=Qt.AlignCenter)
        layout.addWidget(in_suturing_btn, alignment=Qt.AlignCenter)
        layout.addStretch()

        task_menu_page.setLayout(layout)
        self.pages.addWidget(task_menu_page)

        def set_peg_transfer_task():
            self.file_manager.update_destination_folder(FileManager.PEG_TRANSFER)
            self.start_video()
            self.pages.setCurrentWidget(self.video_page)

        def set_in_suturing_task():
            self.file_manager.update_destination_folder(FileManager.INTRACORPOREAL_SUTURING)
            self.start_video()
            self.pages.setCurrentWidget(self.video_page)

        peg_transfer_btn.clicked.connect(lambda checked=False: set_peg_transfer_task())

        in_suturing_btn.clicked.connect(lambda checked=False: set_in_suturing_task())

        return task_menu_page

    def create_post_task_menu(self):
        """
        A menu which pops up after the task is complete

        Button options:
            - New Task
            - Logout
        """
        post_task_page = QWidget()
        layout = QVBoxLayout()

        title = QLabel("Task Complete")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("""
            font-size: 48px;
            font-weight: bold;
            color: black;
            font-family:  "Times New Roman", Times, serif;
            margin-top: 24px;
            margin-bottom: 12px;
        """)

        new_task_btn = QPushButton("New Task")
        logout_btn = QPushButton("Logout")

        for btn in [new_task_btn, logout_btn]:
            btn.setFixedHeight(50)
            btn.setMinimumWidth(250)
            btn.setMaximumWidth(400)
            btn.setStyleSheet(self.btn_style)

        new_task_btn.clicked.connect(
            lambda: self.pages.setCurrentWidget(self.task_menu_page)
        )

        def logout_user():
            self.key = ""
            self.name = ""
            self.pages.setCurrentWidget(self.login_page)

        logout_btn.clicked.connect(lambda checked=False: logout_user())

        layout.addStretch()
        layout.addWidget(title)
        layout.addWidget(new_task_btn, alignment=Qt.AlignCenter)
        layout.addWidget(logout_btn, alignment=Qt.AlignCenter)
        layout.addStretch()

        post_task_page.setLayout(layout)
        self.pages.addWidget(post_task_page)

        return post_task_page

    def closeEvent(self, event):
        if self.data_thread is not None:
            self.data_thread.stop()

        event.accept()
