#!/usr/bin/env python3
"""Plot pitch, roll, yaw, and throttle measurements over time from a THOROS CSV log."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import pandas as pd

from autotune_pid import AXES, load_axis_data


def _resolve_throttle_col(df: pd.DataFrame, preferred: str | None) -> str | None:
	for candidate in (preferred, "throttle", "throttle_command", "flight_throttle"):
		if candidate and candidate in df.columns:
			return candidate
	return None


def parse_args() -> argparse.Namespace:
	parser = argparse.ArgumentParser(description="Plot pitch/roll/yaw measurements from a flight log CSV.")
	parser.add_argument("--csv", required=True, help="Path to CSV file.")
	parser.add_argument("--time-col", default="timestamp_s", help="Timestamp column in seconds.")
	parser.add_argument("--throttle-col", default=None, help="Throttle column name. Defaults to throttle if present.")

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
	throttle_col = _resolve_throttle_col(df, args.throttle_col)

	axis_payloads = []
	for axis_name in AXES:
		try:
			axis_data = load_axis_data(
				df=df,
				axis=axis_name,
				time_col=args.time_col,
				setpoint_col=getattr(args, f"{axis_name}_setpoint_col"),
				measurement_col=getattr(args, f"{axis_name}_measurement_col"),
				control_col=getattr(args, f"{axis_name}_control_col"),
			)
		except ValueError as exc:
			print(f"Skipping {axis_name}: {exc}", file=sys.stderr)
			continue

		axis_payloads.append((axis_name, axis_data))

	if not axis_payloads and not throttle_col:
		raise ValueError("No plottable series found in CSV.")

	nrows = len(axis_payloads) + (1 if throttle_col else 0)
	fig, axes = plt.subplots(nrows, 1, sharex=True, figsize=(8, 2 * nrows))
	if nrows == 1:
		axes = [axes]
	fig.suptitle(f"THOROS attitude measurements over time\n{csv_path.name}")

	for (axis_name, axis_data), ax in zip(axis_payloads, axes[: len(axis_payloads)]):

		ax.plot(axis_data.t, axis_data.y, label=f"{axis_name} measurement", linewidth=1.6)
		ax.plot(axis_data.t, axis_data.r, label=f"{axis_name} setpoint", linewidth=1.1, alpha=0.8)
		ax.plot(axis_data.t, axis_data.u, label=f"{axis_name} command", linewidth=1.1, alpha=0.8)
		ax.set_ylabel(f"{axis_name.capitalize()}\n(deg)")
		ax.set_title(axis_name.capitalize())
		ax.grid(True, alpha=0.25)
		ax.legend(loc="upper right")

	if throttle_col:
		throttle_ax = axes[len(axis_payloads)]
		throttle_time = pd.to_numeric(df[args.time_col], errors="coerce")
		throttle_data = pd.to_numeric(df[throttle_col], errors="coerce")
		mask = throttle_time.notna() & throttle_data.notna()
		throttle_ax.plot(throttle_time[mask], throttle_data[mask], label="throttle", linewidth=1.6)
		throttle_ax.set_ylabel("Throttle\n(raw)")
		throttle_ax.set_title("Throttle")
		throttle_ax.grid(True, alpha=0.25)
		throttle_ax.legend(loc="upper right")

	axes[-1].set_xlabel("Time (s)")
	fig.tight_layout(rect=[0, 0, 1, 0.96])
	plt.show()


if __name__ == "__main__":
	main()