import sys
import argparse
from pathlib import Path
from PyQt5.QtWidgets import QApplication
from gui import GUI

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="A script which loads the gui and parses data from the camera and arduino sensors"
    )
    parser.add_argument(
        "--test", action="store_true", help="Run GUI without sensors and camera"
    )

    parser.add_argument(
        "-f",
        "--name_file",
        type=Path,
        required=True,
        help="Path to the name to key file",
    )

    parser.add_argument(
        "-v",
        "--visualize",
        action="store_true",
        help="Visualize data processed from sensors",
    )

    args = parser.parse_args()

    app = QApplication(sys.argv)
    window = GUI(
        name_to_key_file=args.name_file, test_mode=args.test, visualize=args.visualize
    )
    window.show()
    sys.exit(app.exec())