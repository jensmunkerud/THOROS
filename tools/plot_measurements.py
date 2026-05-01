#!/usr/bin/env python3
"""Plot pitch, roll, and yaw measurements over time from a THOROS CSV log."""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd

from autotune_pid import AXES, load_axis_data


def parse_args() -> argparse.Namespace:
	parser = argparse.ArgumentParser(description="Plot pitch/roll/yaw measurements from a flight log CSV.")
	parser.add_argument("--csv", required=True, help="Path to CSV file.")
	parser.add_argument("--time-col", default="timestamp_s", help="Timestamp column in seconds.")

	for axis in AXES:
		parser.add_argument(f"--{axis}-setpoint-col", default=None)
		parser.add_argument(f"--{axis}-measurement-col", default=None)
		parser.add_argument(f"--{axis}-control-col", default=None)

	return parser.parse_args()


def main() -> None:
	args = parse_args()

	csv_path = Path(args.csv)
	if not csv_path.exists():
		raise FileNotFoundError(f"CSV not found: {csv_path}")

	df = pd.read_csv(csv_path)

	fig, axes = plt.subplots(len(AXES), 1, sharex=True, figsize=(12, 9))
	fig.suptitle(f"THOROS attitude measurements over time\n{csv_path.name}")

	for axis_name, ax in zip(AXES, axes):
		axis_data = load_axis_data(
			df=df,
			axis=axis_name,
			time_col=args.time_col,
			setpoint_col=getattr(args, f"{axis_name}_setpoint_col"),
			measurement_col=getattr(args, f"{axis_name}_measurement_col"),
			control_col=getattr(args, f"{axis_name}_control_col"),
		)

		ax.plot(axis_data.t, axis_data.y, label=f"{axis_name} measurement", linewidth=1.6)
		ax.plot(axis_data.t, axis_data.r, label=f"{axis_name} setpoint", linewidth=1.1, alpha=0.8)
		ax.plot(axis_data.t, axis_data.u, label=f"{axis_name} command", linewidth=1.1, alpha=0.8)
		ax.set_ylabel(f"{axis_name.capitalize()}\n(deg)")
		ax.grid(True, alpha=0.25)
		ax.legend(loc="upper right")

	axes[-1].set_xlabel("Time (s)")
	fig.tight_layout(rect=[0, 0, 1, 0.96])
	plt.show()


if __name__ == "__main__":
	main()