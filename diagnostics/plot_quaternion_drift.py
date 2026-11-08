import argparse
import csv
import os
import re
import sys

import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation

SIDES = ("left", "right")


def load_csv(path):
    """Return (times, {side: Rotation}, gain or None) from a quaternion_logger CSV."""
    times = []
    quats = {side: [] for side in SIDES}
    gain = None

    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            times.append(float(row["time_s"]))
            for side in SIDES:
                quats[side].append([float(row[f"{side}_q{c}"]) for c in "wxyz"])
            if gain is None and row.get("gain") not in (None, ""):
                gain = row["gain"]

    if not times:
        raise ValueError(f"{path} contains no data rows")

    rotations = {side: Rotation.from_quat(np.array(q), scalar_first=True)
                 for side, q in quats.items()}
    return np.array(times), rotations, gain


def gain_from_filename(path):
    match = re.search(r"gain[_-]?([0-9]*\.?[0-9]+)", os.path.basename(path), re.IGNORECASE)
    return match.group(1) if match else None


def angular_difference(rotations: Rotation, mode):
    """
    Angle in degrees between orientations.
      initial:     each sample vs. the first sample (drift over time)
      consecutive: each sample vs. the previous one (first point is 0)
    """
    if mode == "initial":
        rel = rotations[0].inv() * rotations
        return np.degrees(rel.magnitude())

    rel = rotations[:-1].inv() * rotations[1:]
    return np.concatenate([[0.0], np.degrees(rel.magnitude())])


def parse_args():
    parser = argparse.ArgumentParser(
        description="Plot quaternion angular difference over time from quaternion_logger CSVs."
    )
    parser.add_argument("csvs", nargs="+", help="One or more CSV files to plot.")
    parser.add_argument("-m", "--mode", choices=["initial", "consecutive"], default="initial",
                        help="Difference vs. the first sample (default) or vs. the previous sample.")
    parser.add_argument("-g", "--gains", nargs="+", default=None,
                        help="Override gain labels, one per CSV, in the same order.")
    parser.add_argument("-o", "--output", default=None,
                        help="Save the figure to this path (e.g. plot.png).")
    parser.add_argument("--no-show", action="store_true", help="Don't open a plot window.")
    args = parser.parse_args()

    if args.gains and len(args.gains) != len(args.csvs):
        parser.error("--gains needs exactly one value per CSV")
    return args


def main():
    args = parse_args()

    fig, axes = plt.subplots(2, 1, sharex=True, figsize=(11, 7))
    plotted = 0

    for i, path in enumerate(args.csvs):
        try:
            times, rotations, gain = load_csv(path)
        except (OSError, KeyError, ValueError) as exc:
            print(f"Skipping {path}: {exc!r}", file=sys.stderr)
            continue

        if args.gains:
            gain = args.gains[i]
        elif gain is None:
            gain = gain_from_filename(path)

        label = f"gain = {gain}" if gain is not None else os.path.basename(path)

        for ax, side in zip(axes, SIDES):
            diff = angular_difference(rotations[side], args.mode)
            ax.plot(times, diff, label=label)
            print(f"{os.path.basename(path)} [{side}] {label}: "
                  f"mean {diff.mean():.3f}°, max {diff.max():.3f}°, final {diff[-1]:.3f}°")
        plotted += 1

    if plotted == 0:
        sys.exit("No CSVs could be plotted.")

    ref = "first sample" if args.mode == "initial" else "previous sample"
    for ax, side in zip(axes, SIDES):
        ax.set_title(f"{side.capitalize()} IMU")
        ax.set_ylabel(f"Angle vs {ref} (°)")
        ax.grid(True, alpha=0.3)
        ax.legend()
    axes[-1].set_xlabel("Time (s)")
    fig.suptitle("Quaternion difference over time")
    fig.tight_layout()

    if args.output:
        fig.savefig(args.output, dpi=150)
        print(f"Saved figure to {args.output}")
    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()