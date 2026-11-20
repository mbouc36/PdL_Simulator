
"""
Author:
Date: 2026
Version: 1.0

Description:
    Dual IMU Madgwick convergence test.

    Uses the existing IMUQuaternionTracker class.
    Sets Madgwick gain to 0.1 immediately.

    Detects convergence using quaternion angular
    variation over a sliding time window.

    When converged, displays:
        - Convergence time
        - Maximum quaternion angular variation
        - Euler X, Y, Z differences (xyz convention)
        - Final Euler X, Y, Z orientation

    Keep both IMUs stationary during testing.
"""

import os
import sys
import time
import warnings
from collections import deque

import numpy as np
import serial
from scipy.spatial.transform import Rotation


sys.path.append(
    os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        ".."
    )
)

from update_config import load_config
from tools.imu_orientation import IMUQuaternionTracker


# --------------------------------------------------
# Configuration
# --------------------------------------------------

config = load_config()

SERIAL_PORT = config["serial_port"]
BAUD_RATE = config["baud_rate"]

MADGWICK_GAIN = 0.1

CONVERGENCE_THRESHOLD_DEG = 0.2
CONVERGENCE_WINDOW_S = 1.0
STABILITY_DURATION_S = 2.0

MAX_CONVERGENCE_TIME_S = 60.0

PRINT_INTERVAL_S = 0.5

EULER_CONVENTION = "xyz"


# --------------------------------------------------
# Quaternion utilities
# --------------------------------------------------

def as_rotation(q):
    """
    Convert scalar-first quaternion [w,x,y,z]
    to a scipy Rotation object.
    """

    q = np.asarray(q, dtype=float)

    if q.shape != (4,):
        raise ValueError("Quaternion must contain 4 values")

    if not np.isfinite(q).all():
        raise ValueError("Quaternion contains invalid values")

    norm = np.linalg.norm(q)

    if norm < 1e-10:
        raise ValueError("Quaternion has zero magnitude")

    return Rotation.from_quat(
        q / norm,
        scalar_first=True
    )


def quaternion_difference_deg(q1, q2):
    """
    Shortest 3D angular difference between two
    quaternions, in degrees.
    """

    r1 = as_rotation(q1)
    r2 = as_rotation(q2)

    relative_rotation = r1.inv() * r2

    return np.degrees(relative_rotation.magnitude())


def quaternion_euler_difference_deg(q1, q2):
    """
    Euler angles of the relative rotation q1 -> q2.

    Uses intrinsic xyz Euler convention.

    Returns:
        [X, Y, Z] in degrees

    These are relative Euler angles, not differences
    obtained by subtracting absolute Euler angles.
    """

    r1 = as_rotation(q1)
    r2 = as_rotation(q2)

    relative_rotation = r1.inv() * r2

    return relative_rotation.as_euler(
        EULER_CONVENTION,
        degrees=True
    )


def quaternion_to_euler_deg(q):
    """
    Convert quaternion into absolute Euler angles
    using intrinsic xyz convention.
    """

    return as_rotation(q).as_euler(
        EULER_CONVENTION,
        degrees=True
    )


# --------------------------------------------------
# Convergence monitor
# --------------------------------------------------

class ConvergenceMonitor:

    def __init__(self, name):

        self.name = name

        self.history = deque()

        self.stable_since = None

        self.converged = False
        self.convergence_time = None

        self.angle_change = None

        self.euler_difference = None
        self.final_euler = None

    def update(self, q, current_time, start_time):
        """
        Check orientation stability over a sliding
        time window.

        Convergence requires:
            1. Full window of valid samples
            2. Maximum angular variation below threshold
            3. Stability sustained for required duration
        """

        if self.converged:
            return

        q = np.asarray(q, dtype=float)

        self.history.append(
            (current_time, q.copy())
        )

        # Keep approximately one second of history,
        # retaining a sample at least one window old.
        while (
            len(self.history) > 1
            and current_time - self.history[1][0]
                >= CONVERGENCE_WINDOW_S
        ):
            self.history.popleft()

        oldest_time, oldest_q = self.history[0]

        window_duration = (
            current_time - oldest_time
        )

        # Wait for a complete comparison window.
        if window_duration < CONVERGENCE_WINDOW_S:
            return

        # Maximum orientation difference relative
        # to the oldest quaternion in the window.
        self.angle_change = max(
            quaternion_difference_deg(
                oldest_q,
                sample_q
            )
            for _, sample_q in self.history
        )

        stable = (
            self.angle_change
            < CONVERGENCE_THRESHOLD_DEG
        )

        if stable:

            if self.stable_since is None:
                self.stable_since = current_time

            stable_duration = (
                current_time - self.stable_since
            )

            if stable_duration >= STABILITY_DURATION_S:

                self.converged = True

                self.convergence_time = (
                    current_time - start_time
                )

                # Euler difference between the
                # oldest and newest quaternions in
                # the final convergence window.
                self.euler_difference = (
                    quaternion_euler_difference_deg(
                        oldest_q,
                        q
                    )
                )

                # Absolute Euler orientation
                # when convergence is detected.
                self.final_euler = (
                    quaternion_to_euler_deg(q)
                )

                self.print_results()

        else:
            # Stability must be continuous.
            self.stable_since = None

    def print_results(self):

        error_x, error_y, error_z = (
            self.euler_difference
        )

        final_x, final_y, final_z = (
            self.final_euler
        )

        print()
        print("=" * 50)
        print(f"{self.name.upper()} IMU CONVERGED")
        print("=" * 50)

        print(
            f"Convergence time: "
            f"{self.convergence_time:.3f} s"
        )

        print(
            f"Maximum window angular variation: "
            f"{self.angle_change:.4f} deg"
        )

        print()
        print(
            f"Euler angle differences "
            f"({EULER_CONVENTION}):"
        )

        print(f"  X: {error_x:+.4f} deg")
        print(f"  Y: {error_y:+.4f} deg")
        print(f"  Z: {error_z:+.4f} deg")

        print()
        print(
            f"Final Euler orientation "
            f"({EULER_CONVENTION}):"
        )

        print(f"  X: {final_x:+.4f} deg")
        print(f"  Y: {final_y:+.4f} deg")
        print(f"  Z: {final_z:+.4f} deg")

        print("=" * 50)
        print()


# --------------------------------------------------
# Serial processing
# --------------------------------------------------

def main():

    print()
    print("=" * 50)
    print("DUAL IMU MADGWICK CONVERGENCE TEST")
    print("=" * 50)

    print(f"Serial port: {SERIAL_PORT}")
    print(f"Baud rate: {BAUD_RATE}")

    print(f"Madgwick gain: {MADGWICK_GAIN}")

    print(
        f"Convergence threshold: "
        f"{CONVERGENCE_THRESHOLD_DEG} deg"
    )

    print(
        f"Comparison window: "
        f"{CONVERGENCE_WINDOW_S} s"
    )

    print(
        f"Required stable duration: "
        f"{STABILITY_DURATION_S} s"
    )

    print(
        f"Euler convention: "
        f"{EULER_CONVENTION}"
    )

    print()
    print("Keep both IMUs stationary.")
    print()

    # ----------------------------------------------
    # Initialize IMU trackers
    # ----------------------------------------------

    left_tracker = IMUQuaternionTracker(
        "left",
        use_offset=False
    )

    right_tracker = IMUQuaternionTracker(
        "right",
        use_offset=False
    )

    # Immediately override the starting gain.
    left_tracker.set_gain(MADGWICK_GAIN)
    right_tracker.set_gain(MADGWICK_GAIN)

    # ----------------------------------------------
    # Initialize convergence monitors
    # ----------------------------------------------

    left_monitor = ConvergenceMonitor("left")
    right_monitor = ConvergenceMonitor("right")

    start_time = None
    last_print = time.monotonic()

    # ----------------------------------------------
    # Open serial connection
    # ----------------------------------------------

    try:

        with serial.Serial(
            SERIAL_PORT,
            BAUD_RATE,
            timeout=0.5
        ) as ser:

            print("Waiting for valid IMU data...")

            # Ignore first sample to prevent an
            # excessively large initial dt.
            first_sample = True

            while True:

                try:

                    line = ser.readline().decode(
                        "utf-8"
                    ).strip()

                    parts = line.split(",")

                    if len(parts) < 23:
                        continue

                    arduino_time = parts[0]

                    left_values = (
                        [arduino_time]
                        + parts[5:14]
                    )

                    right_values = (
                        [arduino_time]
                        + parts[14:23]
                    )

                    # ----------------------------------
                    # Initialize timestamp
                    # ----------------------------------

                    if first_sample:

                        initial_time = float(
                            arduino_time
                        )

                        left_tracker.previous_time = (
                            initial_time
                        )

                        right_tracker.previous_time = (
                            initial_time
                        )

                        first_sample = False

                        continue

                    # ----------------------------------
                    # Update quaternions
                    # ----------------------------------

                    left_q = (
                        left_tracker.get_quaternion(
                            left_values
                        )
                    )

                    right_q = (
                        right_tracker.get_quaternion(
                            right_values
                        )
                    )

                    if left_q is None or right_q is None:
                        continue

                    left_q = np.asarray(
                        left_q,
                        dtype=float
                    )

                    right_q = np.asarray(
                        right_q,
                        dtype=float
                    )

                    if not (
                        np.isfinite(left_q).all()
                        and np.isfinite(right_q).all()
                    ):
                        continue

                    current_time = time.monotonic()

                    if start_time is None:

                        start_time = current_time

                        print()
                        print(
                            "Receiving valid quaternions."
                        )
                        print(
                            "Monitoring convergence..."
                        )
                        print()

                    # ----------------------------------
                    # Check convergence
                    # ----------------------------------

                    left_monitor.update(
                        left_q,
                        current_time,
                        start_time
                    )

                    right_monitor.update(
                        right_q,
                        current_time,
                        start_time
                    )

                    # ----------------------------------
                    # Periodic progress
                    # ----------------------------------

                    if (
                        current_time - last_print
                        >= PRINT_INTERVAL_S
                    ):

                        elapsed = (
                            current_time - start_time
                        )

                        left_change = (
                            left_monitor.angle_change
                        )

                        right_change = (
                            right_monitor.angle_change
                        )

                        left_text = (
                            f"{left_change:.4f} deg"
                            if left_change is not None
                            else "waiting"
                        )

                        right_text = (
                            f"{right_change:.4f} deg"
                            if right_change is not None
                            else "waiting"
                        )

                        print(
                            f"Time: {elapsed:6.2f} s | "
                            f"Left: {left_text:>12} | "
                            f"Right: {right_text:>12}"
                        )

                        last_print = current_time

                    # ----------------------------------
                    # Both IMUs converged
                    # ----------------------------------

                    if (
                        left_monitor.converged
                        and right_monitor.converged
                    ):

                        print()
                        print("=" * 50)
                        print("BOTH IMUs CONVERGED")
                        print("=" * 50)

                        print(
                            f"Left convergence time: "
                            f"{left_monitor.convergence_time:.3f} s"
                        )

                        print(
                            f"Right convergence time: "
                            f"{right_monitor.convergence_time:.3f} s"
                        )

                        print(
                            f"Total elapsed time: "
                            f"{current_time - start_time:.3f} s"
                        )

                        print("=" * 50)

                        break

                    # ----------------------------------
                    # Timeout
                    # ----------------------------------

                    if (
                        current_time - start_time
                        >= MAX_CONVERGENCE_TIME_S
                    ):

                        print()
                        print("=" * 50)
                        print("CONVERGENCE TIMEOUT")
                        print("=" * 50)

                        for monitor in (
                            left_monitor,
                            right_monitor
                        ):

                            if monitor.converged:

                                print(
                                    f"{monitor.name.upper()}: "
                                    f"CONVERGED in "
                                    f"{monitor.convergence_time:.3f} s"
                                )

                            else:

                                print(
                                    f"{monitor.name.upper()}: "
                                    f"NOT CONVERGED"
                                )

                        break

                except ValueError as exc:

                    print(
                        f"Data conversion error: {exc}"
                    )

                except KeyboardInterrupt:

                    print()
                    print("Stopped by user.")

                    break

    except serial.SerialException as exc:

        print(
            f"Serial connection failed: {exc}"
        )


if __name__ == "__main__":

    with warnings.catch_warnings():
        warnings.simplefilter(
            "default",
            UserWarning
        )
        main()
