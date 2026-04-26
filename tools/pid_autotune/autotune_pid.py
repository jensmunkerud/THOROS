#!/usr/bin/env python3
"""
Offline PID autotuner for THOROS flight logs.

Given time-series logs, this tool:
1) Identifies a first-order plant model per axis.
2) Optimizes PID gains against the measured setpoint trajectory.
3) Generates safe-first-flight scaled gains and a markdown report.

Expected default columns:
- timestamp_s
- pitch_setpoint, pitch_measurement, pitch_control (optional)
- roll_setpoint, roll_measurement, roll_control (optional)
- yaw_setpoint, yaw_measurement, yaw_control (optional)
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import pandas as pd
from scipy.optimize import minimize

AXES = ("pitch", "roll", "yaw")


@dataclass
class AxisData:
    t: np.ndarray
    r: np.ndarray
    y: np.ndarray
    u: np.ndarray
    dt: float
    has_control_column: bool


@dataclass
class PlantModel:
    gain: float
    tau: float
    delay_steps: int
    fit_r2: float


@dataclass
class GainResult:
    kp: float
    ki: float
    kd: float
    cost: float
    safe_kp: float
    safe_ki: float
    safe_kd: float


@dataclass
class AxisResult:
    axis: str
    model: PlantModel
    gain_result: GainResult
    notes: List[str]


def _resolve_col(df: pd.DataFrame, preferred: str, fallback: Optional[str]) -> Optional[str]:
    if preferred in df.columns:
        return preferred
    if fallback and fallback in df.columns:
        return fallback
    return None


def _to_numeric_clean(series: pd.Series) -> np.ndarray:
    return pd.to_numeric(series, errors="coerce").to_numpy(dtype=float)


def _median_dt_seconds(t: np.ndarray) -> float:
    dt = np.diff(t)
    dt = dt[np.isfinite(dt)]
    dt = dt[dt > 0]
    if dt.size == 0:
        raise ValueError("Could not infer dt from timestamp column.")
    return float(np.median(dt))


def _finite_mask(*arrs: np.ndarray) -> np.ndarray:
    mask = np.ones_like(arrs[0], dtype=bool)
    for arr in arrs:
        mask &= np.isfinite(arr)
    return mask


def load_axis_data(
    df: pd.DataFrame,
    axis: str,
    time_col: str,
    setpoint_col: Optional[str],
    measurement_col: Optional[str],
    control_col: Optional[str],
) -> AxisData:
    if time_col not in df.columns:
        raise ValueError(f"Missing time column '{time_col}'.")

    sp = setpoint_col or f"{axis}_setpoint"
    ms = measurement_col or f"{axis}_measurement"
    ct = control_col or f"{axis}_control"

    sp_col = _resolve_col(df, sp, None)
    ms_col = _resolve_col(df, ms, None)

    if sp_col is None or ms_col is None:
        raise ValueError(
            f"Axis '{axis}' requires columns '{sp}' and '{ms}'. "
            "Use CLI overrides if your CSV uses different names."
        )

    t = _to_numeric_clean(df[time_col])
    r = _to_numeric_clean(df[sp_col])
    y = _to_numeric_clean(df[ms_col])

    has_control_column = ct in df.columns
    if has_control_column:
        u = _to_numeric_clean(df[ct])
    else:
        # Fallback only when control is absent: identify an approximate model from setpoint.
        u = r.copy()

    mask = _finite_mask(t, r, y, u)
    t, r, y, u = t[mask], r[mask], y[mask], u[mask]

    if t.size < 100:
        raise ValueError(f"Axis '{axis}' has too few valid samples ({t.size}).")

    order = np.argsort(t)
    t, r, y, u = t[order], r[order], y[order], u[order]

    dt = _median_dt_seconds(t)
    return AxisData(t=t, r=r, y=y, u=u, dt=dt, has_control_column=has_control_column)


def estimate_delay_steps(u: np.ndarray, y: np.ndarray, max_steps: int = 25) -> int:
    if len(u) < (max_steps + 2):
        return 0

    u0 = u - np.mean(u)
    y0 = y - np.mean(y)
    best_lag = 0
    best_corr = -np.inf

    for lag in range(0, max_steps + 1):
        left = u0[:-lag] if lag > 0 else u0
        right = y0[lag:] if lag > 0 else y0
        if left.size < 10 or right.size < 10:
            continue
        corr = float(np.corrcoef(left, right)[0, 1])
        if np.isfinite(corr) and corr > best_corr:
            best_corr = corr
            best_lag = lag

    return max(0, int(best_lag))


def identify_first_order_model(axis_data: AxisData) -> PlantModel:
    t = axis_data.t
    y = axis_data.y
    u = axis_data.u

    delay_steps = estimate_delay_steps(u, y)
    if delay_steps > 0:
        u_eff = np.concatenate([np.full(delay_steps, u[0]), u[:-delay_steps]])
    else:
        u_eff = u

    dt = axis_data.dt
    yk = y[:-1]
    yk1 = y[1:]
    uk = u_eff[:-1]

    # Fit y[k+1] = a*y[k] + b*u[k] + c using least squares.
    A = np.column_stack([yk, uk, np.ones_like(yk)])
    sol, *_ = np.linalg.lstsq(A, yk1, rcond=None)
    a, b, c = [float(v) for v in sol]

    # Convert to first-order continuous-ish params.
    a_clamped = float(np.clip(a, 1e-4, 0.9999))
    tau = -dt / math.log(a_clamped)
    gain = b / (1.0 - a_clamped)

    yhat = A @ sol
    ss_res = float(np.sum((yk1 - yhat) ** 2))
    ss_tot = float(np.sum((yk1 - np.mean(yk1)) ** 2))
    fit_r2 = 1.0 - ss_res / ss_tot if ss_tot > 1e-12 else 0.0

    # Fold static offset term into gain-based behavior around operating point.
    _ = c
    return PlantModel(gain=gain, tau=max(tau, 1e-3), delay_steps=delay_steps, fit_r2=fit_r2)


def simulate_closed_loop(
    model: PlantModel,
    reference: np.ndarray,
    dt: float,
    gains: Tuple[float, float, float],
    control_limit: float,
) -> Tuple[np.ndarray, np.ndarray]:
    kp, ki, kd = gains
    n = len(reference)

    y = np.zeros(n, dtype=float)
    u = np.zeros(n, dtype=float)

    integ = 0.0
    prev_err = 0.0

    delay = max(0, model.delay_steps)
    u_buffer = np.zeros(delay + 1, dtype=float)

    for k in range(1, n):
        err = reference[k - 1] - y[k - 1]
        deriv = (err - prev_err) / dt

        unsat = kp * err + ki * integ + kd * deriv
        sat = float(np.clip(unsat, -control_limit, control_limit))

        # Conditional integration anti-windup.
        if abs(unsat - sat) < 1e-8 or (sat > 0 and err < 0) or (sat < 0 and err > 0):
            integ += err * dt

        u_buffer = np.roll(u_buffer, 1)
        u_buffer[0] = sat
        u_delayed = u_buffer[-1]

        dy = (dt / model.tau) * (-y[k - 1] + model.gain * u_delayed)
        y[k] = y[k - 1] + dy
        u[k] = sat
        prev_err = err

    return y, u


def objective_for_gains(
    gains: np.ndarray,
    model: PlantModel,
    axis_data: AxisData,
    control_limit: float,
) -> float:
    kp, ki, kd = [float(g) for g in gains]
    y_sim, u_sim = simulate_closed_loop(model, axis_data.r, axis_data.dt, (kp, ki, kd), control_limit)

    tracking_mse = float(np.mean((axis_data.y - y_sim) ** 2))
    jerk_penalty = float(np.mean(np.diff(u_sim) ** 2)) if len(u_sim) > 2 else 0.0
    control_penalty = float(np.mean(np.abs(u_sim)))

    return tracking_mse + 0.002 * jerk_penalty + 0.0005 * control_penalty


def tune_axis(
    axis: str,
    axis_data: AxisData,
    initial_gains: Tuple[float, float, float],
    safe_scale: float,
) -> AxisResult:
    notes: List[str] = []

    model = identify_first_order_model(axis_data)

    if not axis_data.has_control_column:
        notes.append("No control column found; model was identified from setpoint fallback, confidence reduced.")

    if model.fit_r2 < 0.5:
        notes.append("Plant fit quality is low (R^2 < 0.5); recommendations may be noisy.")

    u_abs = np.abs(axis_data.u)
    observed_limit = float(np.percentile(u_abs, 99.5)) if u_abs.size else 200.0
    control_limit = max(25.0, observed_limit)

    bounds = [(0.0, 200.0), (0.0, 200.0), (0.0, 200.0)]

    def _obj(x: np.ndarray) -> float:
        return objective_for_gains(x, model, axis_data, control_limit)

    x0 = np.array(initial_gains, dtype=float)
    x0 = np.clip(x0, [b[0] for b in bounds], [b[1] for b in bounds])

    result = minimize(
        _obj,
        x0,
        method="L-BFGS-B",
        bounds=bounds,
        options={"maxiter": 300},
    )

    if not result.success:
        notes.append(f"Optimizer warning: {result.message}")

    kp, ki, kd = [float(v) for v in result.x]
    gain_result = GainResult(
        kp=kp,
        ki=ki,
        kd=kd,
        cost=float(result.fun),
        safe_kp=kp * safe_scale,
        safe_ki=ki * safe_scale,
        safe_kd=kd * safe_scale,
    )

    return AxisResult(axis=axis, model=model, gain_result=gain_result, notes=notes)


def parse_gains(text: str, axis: str) -> Tuple[float, float, float]:
    try:
        parts = [float(x.strip()) for x in text.split(",")]
    except ValueError as exc:
        raise ValueError(f"Invalid {axis} gains format '{text}'. Expected 'Kp,Ki,Kd'.") from exc

    if len(parts) != 3:
        raise ValueError(f"Invalid {axis} gains format '{text}'. Expected exactly 3 values.")

    return parts[0], parts[1], parts[2]


def build_markdown_report(
    csv_path: Path,
    results: List[AxisResult],
    safe_scale: float,
) -> str:
    lines: List[str] = []
    lines.append("# PID Autotune Report")
    lines.append("")
    lines.append(f"Input log: {csv_path}")
    lines.append(f"Safe scale factor: {safe_scale:.2f}")
    lines.append("")

    lines.append("## Suggested Gains")
    lines.append("")
    lines.append("| Axis | Kp | Ki | Kd | Safe Kp | Safe Ki | Safe Kd | Fit R^2 | Cost |")
    lines.append("|---|---:|---:|---:|---:|---:|---:|---:|---:|")
    for r in results:
        g = r.gain_result
        m = r.model
        lines.append(
            f"| {r.axis} | {g.kp:.4f} | {g.ki:.4f} | {g.kd:.4f} "
            f"| {g.safe_kp:.4f} | {g.safe_ki:.4f} | {g.safe_kd:.4f} "
            f"| {m.fit_r2:.3f} | {g.cost:.5f} |"
        )

    lines.append("")
    lines.append("## Identified Plant")
    lines.append("")
    lines.append("| Axis | Gain | Tau (s) | Delay (samples) |")
    lines.append("|---|---:|---:|---:|")
    for r in results:
        m = r.model
        lines.append(f"| {r.axis} | {m.gain:.5f} | {m.tau:.5f} | {m.delay_steps} |")

    lines.append("")
    lines.append("## Notes")
    lines.append("")
    any_notes = False
    for r in results:
        for n in r.notes:
            any_notes = True
            lines.append(f"- [{r.axis}] {n}")
    if not any_notes:
        lines.append("- No warnings.")

    lines.append("")
    lines.append("## Firmware Payload")
    lines.append("")
    # pitch/roll/yaw order expected by your parser: P/I/D for each axis.
    payload_values: List[str] = []
    for axis in ("pitch", "roll", "yaw"):
        found = next(r for r in results if r.axis == axis)
        payload_values.extend([
            f"{found.gain_result.safe_kp:.4f}",
            f"{found.gain_result.safe_ki:.4f}",
            f"{found.gain_result.safe_kd:.4f}",
        ])
    lines.append("Safe first-flight payload string:")
    lines.append("")
    lines.append(f"`{'/'.join(payload_values)}`")

    return "\n".join(lines)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Offline PID autotune from flight log CSV.")
    parser.add_argument("--csv", required=True, help="Path to CSV file.")
    parser.add_argument("--time-col", default="timestamp_s", help="Timestamp column in seconds.")
    parser.add_argument("--output-dir", default="tools/pid_autotune/out", help="Output folder for report and JSON.")
    parser.add_argument("--safe-scale", type=float, default=0.65, help="Scale applied to suggested gains for first flight.")

    parser.add_argument("--pitch-initial", default="6,0.2,0.7", help="Initial Kp,Ki,Kd for pitch.")
    parser.add_argument("--roll-initial", default="3,0.2,0.6", help="Initial Kp,Ki,Kd for roll.")
    parser.add_argument("--yaw-initial", default="2,4,1", help="Initial Kp,Ki,Kd for yaw.")

    # Optional column overrides.
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

    initial = {
        "pitch": parse_gains(args.pitch_initial, "pitch"),
        "roll": parse_gains(args.roll_initial, "roll"),
        "yaw": parse_gains(args.yaw_initial, "yaw"),
    }

    results: List[AxisResult] = []

    for axis in AXES:
        axis_data = load_axis_data(
            df=df,
            axis=axis,
            time_col=args.time_col,
            setpoint_col=getattr(args, f"{axis}_setpoint_col"),
            measurement_col=getattr(args, f"{axis}_measurement_col"),
            control_col=getattr(args, f"{axis}_control_col"),
        )

        tuned = tune_axis(axis, axis_data, initial[axis], args.safe_scale)
        results.append(tuned)

    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    report_md = build_markdown_report(csv_path, results, args.safe_scale)
    report_path = out_dir / "pid_autotune_report.md"
    report_path.write_text(report_md, encoding="utf-8")

    gains_payload = {
        r.axis: {
            "recommended": {
                "kp": r.gain_result.kp,
                "ki": r.gain_result.ki,
                "kd": r.gain_result.kd,
            },
            "safe_first_flight": {
                "kp": r.gain_result.safe_kp,
                "ki": r.gain_result.safe_ki,
                "kd": r.gain_result.safe_kd,
            },
            "plant": {
                "gain": r.model.gain,
                "tau_s": r.model.tau,
                "delay_steps": r.model.delay_steps,
                "fit_r2": r.model.fit_r2,
            },
            "notes": r.notes,
        }
        for r in results
    }

    json_path = out_dir / "pid_autotune_gains.json"
    json_path.write_text(json.dumps(gains_payload, indent=2), encoding="utf-8")

    print(f"Wrote report: {report_path}")
    print(f"Wrote gains:  {json_path}")


if __name__ == "__main__":
    main()
