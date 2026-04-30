# PID Autotune Tool

This folder contains an offline PID recommendation tool for THOROS logs.

## What it does

- Reads a CSV log containing setpoint/measurement time-series.
- Identifies a first-order model per axis (pitch/roll/yaw).
- Optimizes PID gains against your real trajectory.
- Produces:
  - Markdown report with recommended and safe-scaled gains.
  - JSON file ready for scripting and automation.
  - A firmware payload string in the report with format:
    `pitchP/pitchI/pitchD/rollP/rollI/rollD/yawP/yawI/yawD`

## Install

```bash
python -m venv .venv
.venv\Scripts\activate
pip install -r tools/pid_autotune/requirements.txt
```

## Required CSV columns (default names)

- `timestamp_s`
- `pitch_setpoint`, `pitch_measurement`, `pitch_control` (optional)
- `roll_setpoint`, `roll_measurement`, `roll_control` (optional)
- `yaw_setpoint`, `yaw_measurement`, `yaw_control` (optional)

If control columns are missing, the tool falls back to setpoint-based identification and marks confidence lower.

## Run

```bash
python tools/pid_autotune/autotune_pid.py --csv path/to/flight_log.csv
```

To plot the attitude measurements from the same CSV shape:

```bash
python tools/pid_autotune/plot_measurements.py --csv path/to/flight_log.csv
```

Outputs are written to `tools/pid_autotune/out/` by default.

## Column override example

```bash
python tools/pid_autotune/autotune_pid.py \
  --csv logs/hover_test.csv \
  --time-col time_s \
  --pitch-setpoint-col target_pitch_deg \
  --pitch-measurement-col imu_pitch_deg \
  --pitch-control-col pitch_pid_cmd
```

## Recommended workflow

1. Log stable hover and small step maneuvers per axis.
2. Run this tool and use the safe-scaled gains first.
3. Fly short verification hops and re-log.
4. Repeat until fit quality and response settle.
