import argparse
import csv
import os
import sys
import threading
import time
from dataclasses import dataclass
from datetime import datetime

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from tools.serial_parser import SerialParser
from tools.imu_orientation import IMUQuaternionTracker
from tools.tof_manager import TOFManager


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
    """

    def __init__(self, gain, left_imu_name="left", right_imu_name="right",
                 num_init_samples=400, settle_time=5.0):
        super().__init__(name="SensorProducer", daemon=True)

        self.serial_parser = SerialParser()
        self.tof_manager = TOFManager()
        self.left_imu = IMUQuaternionTracker(name=left_imu_name, use_offset=True)
        self.right_imu = IMUQuaternionTracker(name=right_imu_name, use_offset=True)

        self.gain = gain
        self.num_init_samples = num_init_samples
        self.settle_time = settle_time

        self._cond = threading.Condition()
        self._latest = None
        self._seq = 0

        self.settling = True
        self.init_progress = 0

        self.converged = threading.Event()
        self._stop_event = threading.Event()
        self.error = None

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
        except Exception as exc:
            self.error = exc
        finally:
            self._stop_event.set()
            with self._cond:
                self._cond.notify_all()

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

            # Convergence phase: feed the filters but don't publish.
            if not self.converged.is_set():
                if time.monotonic() - start < self.settle_time:
                    continue
                self.settling = False
                self.init_progress += 1
                if self.init_progress >= self.num_init_samples:
                    self.left_imu.set_gain(self.gain)
                    self.right_imu.set_gain(self.gain)
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
CSV_HEADER = [
    "time_s",
    "left_qw", "left_qx", "left_qy", "left_qz",
    "right_qw", "right_qx", "right_qy", "right_qz",
    "num_samples",
    "gain",
]


class QuaternionLogger(threading.Thread):
    """
    Waits for the producer to converge, then every `interval_s` seconds averages
    all samples received during that interval and writes one CSV row.
    Time 0 is the moment logging starts (right after convergence).
    """

    def __init__(self, producer: SensorProducer, output_path, duration_s, interval_s,
                 convergence_timeout=60.0):
        super().__init__(name="QuaternionLogger")
        self.producer = producer
        self.gain = producer.gain
        self.output_path = output_path
        self.duration_s = duration_s
        self.interval_s = interval_s
        self.convergence_timeout = convergence_timeout

        self._stop_event = threading.Event()
        self.error = None
        self.rows_written = 0

    def stop(self):
        self._stop_event.set()

    def _check_producer(self):
        if not self.producer.is_running:
            raise RuntimeError("Sensor producer stopped") from self.producer.error

    def run(self):
        try:
            self._wait_for_convergence()
            if not self._stop_event.is_set():
                self._log()
        except Exception as exc:
            self.error = exc

    def _wait_for_convergence(self):
        print("Hold tools still")
        deadline = time.monotonic() + self.convergence_timeout

        while not self.producer.converged.wait(timeout=0.1):
            if self._stop_event.is_set():
                return
            self._check_producer()
            if time.monotonic() > deadline:
                raise TimeoutError("IMUs did not converge in time")

            if self.producer.settling:
                msg = "Settling..."
            else:
                msg = f"Initializing IMUs: {self.producer.init_progress}/{self.producer.num_init_samples}"
            print(f"\r{msg:<40}", end="", flush=True)

        print("\rIMUs converged." + " " * 30)

    @staticmethod
    def _average(quats):
        return Rotation.from_quat(np.asarray(quats), scalar_first=True).mean().as_quat(scalar_first=True)

    def _log(self):
        start = time.monotonic()
        end = start + self.duration_s
        next_capture = start + self.interval_s
        last_seq = self.producer.current_seq
        left_window, right_window = [], []

        with open(self.output_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(CSV_HEADER)

            print(f"Logging to {self.output_path}")

            while not self._stop_event.is_set() and next_capture <= end + 1e-9:
                # Gather samples until the next capture time
                remaining = next_capture - time.monotonic()
                if remaining > 0:
                    sample = self.producer.wait_for_new_sample(last_seq, timeout=min(remaining, 1.0))
                    if sample is None:
                        self._check_producer()
                    else:
                        left_window.append(sample.left_q)
                        right_window.append(sample.right_q)
                        last_seq = sample.seq
                    continue

                # Capture time reached
                elapsed = next_capture - start
                if left_window:
                    left_avg = self._average(left_window)
                    right_avg = self._average(right_window)
                    writer.writerow(
                        [f"{elapsed:.3f}", *left_avg, *right_avg, len(left_window), self.gain]
                    )
                    f.flush()
                    self.rows_written += 1
                else:
                    print(f"\nWarning: no samples received for t={elapsed:.3f}s, row skipped")

                left_window.clear()
                right_window.clear()
                next_capture += self.interval_s  # fixed schedule, no drift

                print(
                    f"\rElapsed: {elapsed:8.1f}s / {self.duration_s:.0f}s   rows: {self.rows_written}",
                    end="", flush=True,
                )

        print()


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #
def positive_float(value):
    v = float(value)
    if v <= 0:
        raise argparse.ArgumentTypeError(f"must be > 0, got {value}")
    return v


def parse_args():
    parser = argparse.ArgumentParser(description="Log averaged IMU quaternions to a CSV file.")
    parser.add_argument("-d", "--duration", type=positive_float, required=True,
                        help="How long to log for, in minutes (after convergence).")
    parser.add_argument("-i", "--interval", type=positive_float, required=True,
                        help="How often to capture a quaternion, in seconds.")
    parser.add_argument("-g", "--gain", type=float, default=IMUQuaternionTracker.SETTLED_GAIN,
                        help="Filter gain to set after the convergence period.")
    parser.add_argument("-o", "--output-folder", default="logs",
                        help="Folder to write the CSV into (created if missing). Default: current folder.")
    parser.add_argument("-n", "--name", default=None,
                        help="Output CSV file name. Default: quaternion_samples_<date>_<start_time>.csv")
    args = parser.parse_args()

    if args.interval > args.duration * 60:
        parser.error("--interval cannot be longer than --duration")
    return args


def main():
    args = parse_args()

    start_stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    name = args.name or f"quaternion_samples_{start_stamp}.csv"
    if not name.lower().endswith(".csv"):
        name += ".csv"

    os.makedirs(args.output_folder, exist_ok=True)
    output_path = os.path.join(args.output_folder, name)

    producer = SensorProducer(gain=args.gain)
    logger = QuaternionLogger(
        producer,
        output_path=output_path,
        duration_s=args.duration * 60,
        interval_s=args.interval,
    )

    producer.start()
    logger.start()

    try:
        while logger.is_alive():
            logger.join(timeout=0.5)
    except KeyboardInterrupt:
        print("\nInterrupted, finishing up...")
        logger.stop()
        logger.join()
    finally:
        producer.stop()
        producer.join(timeout=2.0)

    if logger.error:
        print(f"Error: {logger.error!r}")
        sys.exit(1)
    print(f"Done. {logger.rows_written} rows written to {output_path}")


if __name__ == "__main__":
    main()