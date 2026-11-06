"""
Author: Michael Boucouvalas
Date: 2026, Oct 11
Version: 1.0
Description: Place the tools at a certain position and measure a quaternion, print the value 
"""
import os
import sys
import threading
import time
from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from tools.serial_parser import SerialParser
from tools.imu_orientation import IMUQuaternionTracker
from tools.tof_manager import TOFManager

CONFIG_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "../calibration/imu_calibration/cal_data.json"
)
CONFIG_ELEMENT_NAME = "heading_offset"
EULER_ANGLE_CONVENTION = "xyz"


@dataclass(frozen=True)
class Sample:
    """One processed reading, published by the producer."""
    seq: int
    serial_time: float
    left_q: np.ndarray       # scalar-first quaternion
    right_q: np.ndarray      # scalar-first quaternion
    left_distance: float
    right_distance: float


# --------------------------------------------------------------------------- #
# Producer
# --------------------------------------------------------------------------- #
class SensorProducer(threading.Thread):
    """
    Continuously reads the serial port, runs the IMU and ToF trackers and
    publishes the latest processed sample into shared state.

    The IMU trackers and serial parser are only ever touched from this thread,
    so they need no locking. Only the published Sample is shared.
    """

    def __init__(self, left_imu_name="left", right_imu_name="right",
                 num_init_samples=400, settle_time=5.0):
        super().__init__(name="SensorProducer", daemon=True)

        self.serial_parser = SerialParser()
        self.tof_manager = TOFManager()
        self.left_imu = IMUQuaternionTracker(name=left_imu_name, use_offset=True)
        self.right_imu = IMUQuaternionTracker(name=right_imu_name, use_offset=True)

        self.num_init_samples = num_init_samples
        self.settle_time = settle_time

        # Shared state (guarded by _cond)
        self._cond = threading.Condition()
        self._latest = None
        self._seq = 0

        # Convergence progress (read-only for other threads, informational)
        self.settling = True
        self.init_progress = 0

        self.converged = threading.Event()
        self._stop_event = threading.Event()
        self.error = None

    # ---- lifecycle -------------------------------------------------------- #
    def stop(self):
        self._stop_event.set()
        with self._cond:
            self._cond.notify_all()

    @property
    def is_running(self):
        return not self._stop_event.is_set()

    def run(self):
        try:
            self._loop()
        except Exception as exc:  # surface errors to the consumer
            self.error = exc
        finally:
            self._stop_event.set()
            with self._cond:
                self._cond.notify_all()

    # ---- main loop -------------------------------------------------------- #
    def _loop(self):
        start = time.monotonic()

        while not self._stop_event.is_set():
            serial_line = self.serial_parser.get_serial_line()
            if serial_line is None:
                continue

            serial_time = serial_line[SerialParser.TIME_INDEX]
            left_q = self.left_imu.get_quaternion(
                [serial_time] + serial_line[SerialParser.LEFT_IMU_INDICES]
            )
            right_q = self.right_imu.get_quaternion(
                [serial_time] + serial_line[SerialParser.RIGHT_IMU_INDICES]
            )
            distances = self.tof_manager.get_distances(
                [serial_line[SerialParser.LEFT_DISTANCE_INDEX],
                 serial_line[SerialParser.RIGHT_DISTANCE_INDEX]]
            )

            # Convergence phase: keep feeding the filters, but don't publish.
            if not self.converged.is_set():
                if time.monotonic() - start < self.settle_time:
                    continue
                self.settling = False
                self.init_progress += 1
                if self.init_progress >= self.num_init_samples:
                    self.left_imu.set_gain()
                    self.right_imu.set_gain()
                    self.converged.set()
                continue

            if left_q is None or right_q is None:
                continue

            with self._cond:
                self._seq += 1
                self._latest = Sample(
                    seq=self._seq,
                    serial_time=serial_time,
                    left_q=np.array(left_q, dtype=float),
                    right_q=np.array(right_q, dtype=float),
                    left_distance=distances[0],
                    right_distance=distances[1],
                )
                self._cond.notify_all()

    # ---- consumer-facing API --------------------------------------------- #
    @property
    def current_seq(self):
        with self._cond:
            return self._seq

    def wait_for_new_sample(self, last_seq, timeout=1.0):
        """Block until a sample newer than last_seq exists. Returns None on timeout/stop."""
        with self._cond:
            self._cond.wait_for(
                lambda: self._seq > last_seq or self._stop_event.is_set(),
                timeout=timeout,
            )
            if self._latest is not None and self._latest.seq > last_seq:
                return self._latest
            return None


# --------------------------------------------------------------------------- #
# Consumer
# --------------------------------------------------------------------------- #
class OrientationConsumer:
    def __init__(self, producer: SensorProducer, num_samples=400, convergence_timeout=60.0):
        self.producer = producer
        self.num_samples = num_samples
        self.convergence_timeout = convergence_timeout

    def _check_producer(self):
        if not self.producer.is_running:
            raise RuntimeError("Sensor producer stopped") from self.producer.error

    def wait_for_convergence(self):
        print("Hold tools still")
        deadline = time.monotonic() + self.convergence_timeout

        while not self.producer.converged.wait(timeout=0.1):
            self._check_producer()
            if time.monotonic() > deadline:
                raise TimeoutError("IMUs did not converge in time")

            if self.producer.settling:
                msg = "Settling..."
            else:
                msg = f"Initializing IMUs: {self.producer.init_progress}/{self.producer.num_init_samples}"
            print(f"\r{msg:<40}", end="", flush=True)

        print("\rIMUs converged." + " " * 30)

    def collect_samples(self):
        """Collect num_samples fresh samples produced after this call starts."""
        samples = []
        last_seq = self.producer.current_seq  # ignore anything produced before ENTER

        while len(samples) < self.num_samples:
            sample = self.producer.wait_for_new_sample(last_seq, timeout=1.0)
            if sample is None:
                self._check_producer()
                continue

            samples.append(sample)
            last_seq = sample.seq
            print(f"\rCollecting samples: {len(samples)}/{self.num_samples}", end="", flush=True)

        print()
        return samples

    @staticmethod
    def compute_average_quaternion(quats):
        rotations = Rotation.from_quat(np.asarray(quats), scalar_first=True)
        average_rotation = rotations.mean()

        differences = np.degrees((average_rotation.inv() * rotations).magnitude())
        print(f"Mean deviation: {np.mean(differences):.2f}°")
        print(f"Max deviation: {np.max(differences):.2f}°")
        print(f"Standard deviation: {np.std(differences):.2f}°")

        return average_rotation

    @staticmethod
    def quaternion_difference(r1: Rotation, r2: Rotation):
        """Angular difference in degrees (0° to 180°)."""
        return np.degrees((r1.inv() * r2).magnitude())

    def run(self):
        old_left, old_right = None, None

        while True:
            user_input = input("Press ENTER when tools are in position (or type 'exit'): ")
            if user_input.strip().lower() == "exit":
                print("Exiting")
                break

            samples = self.collect_samples()

            left_rot = self.compute_average_quaternion([s.left_q for s in samples])
            right_rot = self.compute_average_quaternion([s.right_q for s in samples])
            left_dist = round(float(np.mean([s.left_distance for s in samples])), 2)
            right_dist = round(float(np.mean([s.right_distance for s in samples])), 2)

            print("Left Quaternion", left_rot.as_quat(scalar_first=True), "Left Distance", left_dist)
            print("Right Quaternion", right_rot.as_quat(scalar_first=True), "Right Distance", right_dist)

            if old_left is not None and old_right is not None:
                print("Left Quat diff (deg)", self.quaternion_difference(old_left, left_rot))
                print("Right Quat diff (deg)", self.quaternion_difference(old_right, right_rot))

            print("Left Euler", left_rot.as_euler(EULER_ANGLE_CONVENTION, degrees=True), "Left Distance", left_dist)
            print("Right Euler", right_rot.as_euler(EULER_ANGLE_CONVENTION, degrees=True), "Right Distance", right_dist)

            old_left, old_right = left_rot, right_rot


if __name__ == "__main__":
    producer = SensorProducer()
    producer.start()
    try:
        consumer = OrientationConsumer(producer)
        consumer.wait_for_convergence()
        consumer.run()
    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        producer.stop()
        producer.join(timeout=2.0)