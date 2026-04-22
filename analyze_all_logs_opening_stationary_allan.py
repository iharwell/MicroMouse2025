#!/usr/bin/env python3
"""
Batch Allan-deviation analysis for the opening stationary section of *all* CSV logs in a directory tree.

What it does:
- walks one file or a directory of CSV files
- detects the opening stationary prefix of each log
- computes overlapping Allan deviation from the selected gyro column
- estimates:
  * direct stationary sample noise stats
  * white-noise density
  * bias instability
  * rate-random-walk coefficient
  * derived Q_bgz from RRW, when estimable
- writes:
  * one JSON summary per log
  * one Allan-curve CSV per log
  * one aggregate CSV across all logs
  * one aggregate JSON with run metadata and success/failure counts

No external Allan-deviation package is required.
"""

from __future__ import annotations

import argparse
import json
import math
import traceback
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable, Optional

import numpy as np
import pandas as pd


DEFAULT_GYRO_COL_CANDIDATES = [
    "gyro_raw_radps",
    "gyro_radps",
    "measured_angular_speed_radps",
    "imu_gyro_z",
    "gyro_z_radps",
]
DEFAULT_TIME_COL_CANDIDATES = [
    "imu_timestamp_us",
    "master_time_us",
    "encoder_timestamp_us",
    "time_us",
    "timestamp_us",
    "time_s",
    "timestamp_s",
]
DEFAULT_CMD_LINEAR_COLS = ["cmd_linear_mps", "target_linear_mps", "linear_cmd_mps"]
DEFAULT_CMD_ANGULAR_COLS = ["cmd_angular_radps", "target_angular_radps", "angular_cmd_radps"]
DEFAULT_LEFT_VEL_COLS = [
    "left_encoder_velocity_mps",
    "left_wheel_velocity_mps",
    "left_velocity_mps",
    "left_bank_velocity_mps",
]
DEFAULT_RIGHT_VEL_COLS = [
    "right_encoder_velocity_mps",
    "right_wheel_velocity_mps",
    "right_velocity_mps",
    "right_bank_velocity_mps",
]


@dataclass
class AllanSummary:
    file: str
    rows_total: int
    rows_stationary: int
    stationary_start_index: int
    stationary_end_index: int
    stationary_start_time_s: float
    stationary_end_time_s: float
    stationary_duration_s: float
    time_column: str
    gyro_column: str
    dt_s_median: float
    sample_rate_hz: float
    sample_mean_radps: float
    sample_std_radps: float
    sample_variance_radps2: float
    white_noise_density_radps_per_sqrt_hz: Optional[float]
    white_noise_density_mdps_per_sqrt_hz: Optional[float]
    bias_instability_radps: Optional[float]
    bias_instability_dps: Optional[float]
    bias_instability_tau_s: Optional[float]
    rate_random_walk_radps_per_sqrt_s: Optional[float]
    derived_Q_bgz_from_rrw_radps2_per_sample: Optional[float]
    recommended_Rg_radps2: float
    q_bgz_policy_moving_radps2_per_sample: float
    warnings: list[str]


def choose_existing(columns: Iterable[str], candidates: list[str]) -> Optional[str]:
    s = set(columns)
    for c in candidates:
        if c in s:
            return c
    return None


def load_csv(csv_path: Path, max_rows: Optional[int]) -> pd.DataFrame:
    return pd.read_csv(csv_path, low_memory=False, nrows=max_rows)


def normalize_time_column(series: pd.Series, name: str) -> np.ndarray:
    x = pd.to_numeric(series, errors="coerce").to_numpy(dtype=float)
    if not np.isfinite(x).any():
        raise ValueError(f"time column {name!r} contains no finite values")
    if name.endswith("_us") or np.nanmax(np.abs(x)) > 1e6:
        x = x * 1e-6
    return x


def detect_opening_stationary_prefix(
    df: pd.DataFrame,
    cmd_linear_col: Optional[str],
    cmd_angular_col: Optional[str],
    left_vel_col: Optional[str],
    right_vel_col: Optional[str],
    *,
    cmd_linear_thresh: float,
    cmd_angular_thresh: float,
    encoder_vel_thresh: float,
    break_consecutive: int,
    min_prefix_samples: int,
) -> tuple[int, int]:
    n = len(df)
    if n == 0:
        return 0, -1

    stationary = np.ones(n, dtype=bool)

    def bounded(col: Optional[str], thresh: float):
        nonlocal stationary
        if col and col in df:
            vals = pd.to_numeric(df[col], errors="coerce").fillna(0.0).to_numpy(dtype=float)
            stationary &= np.abs(vals) <= thresh

    bounded(cmd_linear_col, cmd_linear_thresh)
    bounded(cmd_angular_col, cmd_angular_thresh)
    bounded(left_vel_col, encoder_vel_thresh)
    bounded(right_vel_col, encoder_vel_thresh)

    nonstat = ~stationary
    run = 0
    for i in range(n):
        if nonstat[i]:
            run += 1
        else:
            run = 0
        if i + 1 >= max(min_prefix_samples, break_consecutive) and run >= break_consecutive:
            end_idx = max(0, i - break_consecutive)
            return 0, end_idx

    hard_breaks = np.flatnonzero(nonstat)
    if hard_breaks.size > 0 and hard_breaks[0] >= min_prefix_samples:
        return 0, int(hard_breaks[0] - 1)
    return 0, n - 1


def unique_log_spaced_ints(start: int, stop: int, count: int) -> np.ndarray:
    if stop < start:
        return np.array([], dtype=int)
    vals = np.logspace(np.log10(start), np.log10(stop), count)
    vals = np.unique(np.clip(np.round(vals), start, stop).astype(int))
    return vals


def overlapping_allan_deviation_rate(y: np.ndarray, dt: float, max_cluster: Optional[int] = None) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    y = np.asarray(y, dtype=float)
    n = y.size
    if n < 3 or not np.isfinite(y).all():
        return np.array([]), np.array([]), np.array([])
    if max_cluster is None:
        max_cluster = max(1, n // 10)
    max_cluster = min(max_cluster, (n - 1) // 2)
    if max_cluster < 1:
        return np.array([]), np.array([]), np.array([])

    ms = unique_log_spaced_ints(1, max_cluster, 60)
    csum = np.concatenate(([0.0], np.cumsum(y)))
    taus, adev, ns = [], [], []
    for m in ms:
        avgs = (csum[m:] - csum[:-m]) / m
        if avgs.size < m + 1:
            continue
        diff = avgs[m:] - avgs[:-m]
        if diff.size < 2:
            continue
        avar = 0.5 * np.mean(diff * diff)
        if avar <= 0.0 or not np.isfinite(avar):
            continue
        taus.append(m * dt)
        adev.append(math.sqrt(avar))
        ns.append(diff.size)
    return np.asarray(taus), np.asarray(adev), np.asarray(ns)


def adjacent_log_slopes(x: np.ndarray, y: np.ndarray) -> np.ndarray:
    if len(x) < 2:
        return np.array([])
    return np.diff(np.log10(y)) / np.diff(np.log10(x))


def fit_white_noise_density(taus: np.ndarray, adev: np.ndarray) -> Optional[float]:
    if len(taus) < 2:
        return None
    slopes = adjacent_log_slopes(taus, adev)
    mask = (slopes > -0.7) & (slopes < -0.3)
    point_mask = np.zeros(len(taus), dtype=bool)
    point_mask[:-1] |= mask
    point_mask[1:] |= mask
    point_mask &= taus <= np.percentile(taus, 40)
    if not np.any(point_mask):
        point_mask[: min(5, len(taus))] = True
    vals = adev[point_mask] * np.sqrt(taus[point_mask])
    vals = vals[np.isfinite(vals)]
    return float(np.median(vals)) if vals.size else None


def fit_bias_instability(taus: np.ndarray, adev: np.ndarray) -> tuple[Optional[float], Optional[float]]:
    if len(taus) == 0:
        return None, None
    idx = int(np.nanargmin(adev))
    sigma_min = float(adev[idx])
    tau_min = float(taus[idx])
    bias_instability = sigma_min / 0.664  # common Allan-dev approximation for flicker floor
    return bias_instability, tau_min


def fit_rate_random_walk(taus: np.ndarray, adev: np.ndarray) -> Optional[float]:
    if len(taus) < 3:
        return None
    slopes = adjacent_log_slopes(taus, adev)
    mask = (slopes > 0.3) & (slopes < 0.7)
    point_mask = np.zeros(len(taus), dtype=bool)
    point_mask[:-1] |= mask
    point_mask[1:] |= mask
    point_mask &= taus >= np.percentile(taus, 60)
    if not np.any(point_mask):
        return None
    vals = adev[point_mask] / np.sqrt(taus[point_mask] / 3.0)  # sigma(τ)=K*sqrt(τ/3)
    vals = vals[np.isfinite(vals)]
    return float(np.median(vals)) if vals.size else None


def analyze_log(
    csv_path: Path,
    output_dir: Path,
    *,
    gyro_col: Optional[str],
    time_col: Optional[str],
    cmd_linear_thresh: float,
    cmd_angular_thresh: float,
    encoder_vel_thresh: float,
    break_consecutive: int,
    min_prefix_samples: int,
    max_rows: Optional[int],
) -> AllanSummary:
    df = load_csv(csv_path, max_rows=max_rows)
    if df.empty:
        raise ValueError("empty CSV")

    cols = list(df.columns)
    gyro_name = gyro_col or choose_existing(cols, DEFAULT_GYRO_COL_CANDIDATES)
    if gyro_name is None:
        raise ValueError(f"could not detect gyro column in {csv_path.name}")
    time_name = time_col or choose_existing(cols, DEFAULT_TIME_COL_CANDIDATES)
    if time_name is None:
        raise ValueError(f"could not detect time column in {csv_path.name}")

    cmd_linear_col = choose_existing(cols, DEFAULT_CMD_LINEAR_COLS)
    cmd_angular_col = choose_existing(cols, DEFAULT_CMD_ANGULAR_COLS)
    left_vel_col = choose_existing(cols, DEFAULT_LEFT_VEL_COLS)
    right_vel_col = choose_existing(cols, DEFAULT_RIGHT_VEL_COLS)

    start_idx, end_idx = detect_opening_stationary_prefix(
        df,
        cmd_linear_col=cmd_linear_col,
        cmd_angular_col=cmd_angular_col,
        left_vel_col=left_vel_col,
        right_vel_col=right_vel_col,
        cmd_linear_thresh=cmd_linear_thresh,
        cmd_angular_thresh=cmd_angular_thresh,
        encoder_vel_thresh=encoder_vel_thresh,
        break_consecutive=break_consecutive,
        min_prefix_samples=min_prefix_samples,
    )
    if end_idx < start_idx:
        raise ValueError("no stationary prefix found")

    t = normalize_time_column(df[time_name], time_name)
    gyro = pd.to_numeric(df[gyro_name], errors="coerce").to_numpy(dtype=float)

    t = t[start_idx : end_idx + 1]
    gyro = gyro[start_idx : end_idx + 1]

    valid = np.isfinite(t) & np.isfinite(gyro)
    t = t[valid]
    gyro = gyro[valid]
    if gyro.size < max(16, min_prefix_samples):
        raise ValueError(f"stationary prefix too short after filtering: {gyro.size} samples")

    dt = np.diff(t)
    dt = dt[np.isfinite(dt) & (dt > 0)]
    if dt.size == 0:
        raise ValueError("could not estimate dt")
    dt_median = float(np.median(dt))
    fs = float(1.0 / dt_median)

    centered = gyro - np.mean(gyro)
    taus, adev, ns = overlapping_allan_deviation_rate(centered, dt_median)

    white = fit_white_noise_density(taus, adev)
    bias_instability, bias_tau = fit_bias_instability(taus, adev)
    rrw = fit_rate_random_walk(taus, adev)
    derived_q = (rrw ** 2) * dt_median if rrw is not None else None

    # Stationary direct sample variance is a practical in-situ R_g estimate.
    sample_mean = float(np.mean(gyro))
    sample_std = float(np.std(gyro, ddof=1)) if gyro.size > 1 else 0.0
    sample_var = float(sample_std * sample_std)

    warnings: list[str] = []
    if white is None:
        warnings.append("Could not robustly fit white-noise density from Allan curve.")
    if rrw is None:
        warnings.append("Could not robustly fit rate-random-walk coefficient from Allan curve.")
    if taus.size < 8:
        warnings.append("Allan curve is sparse; results are lower confidence.")
    if t[-1] - t[0] < 10.0:
        warnings.append("Opening stationary section is short; long-timescale terms are low confidence.")

    rel = csv_path.name
    stem = csv_path.stem
    per_log_dir = output_dir / stem
    per_log_dir.mkdir(parents=True, exist_ok=True)

    allan_df = pd.DataFrame({
        "tau_s": taus,
        "allan_dev_radps": adev,
        "num_diffs": ns,
    })
    allan_csv = per_log_dir / f"{stem}.allan.csv"
    allan_df.to_csv(allan_csv, index=False)

    summary = AllanSummary(
        file=str(csv_path),
        rows_total=int(len(df)),
        rows_stationary=int(gyro.size),
        stationary_start_index=int(start_idx),
        stationary_end_index=int(end_idx),
        stationary_start_time_s=float(t[0]),
        stationary_end_time_s=float(t[-1]),
        stationary_duration_s=float(t[-1] - t[0]),
        time_column=time_name,
        gyro_column=gyro_name,
        dt_s_median=dt_median,
        sample_rate_hz=fs,
        sample_mean_radps=sample_mean,
        sample_std_radps=sample_std,
        sample_variance_radps2=sample_var,
        white_noise_density_radps_per_sqrt_hz=white,
        white_noise_density_mdps_per_sqrt_hz=(white * 1e3 * 180.0 / math.pi) if white is not None else None,
        bias_instability_radps=bias_instability,
        bias_instability_dps=(bias_instability * 180.0 / math.pi) if bias_instability is not None else None,
        bias_instability_tau_s=bias_tau,
        rate_random_walk_radps_per_sqrt_s=rrw,
        derived_Q_bgz_from_rrw_radps2_per_sample=derived_q,
        recommended_Rg_radps2=sample_var,
        q_bgz_policy_moving_radps2_per_sample=0.0,
        warnings=warnings,
    )

    summary_json = per_log_dir / f"{stem}.summary.json"
    with summary_json.open("w", encoding="utf-8") as f:
        json.dump(asdict(summary), f, indent=2)

    return summary


def iter_csv_files(inputs: list[Path], recursive: bool, include_patterns: list[str], exclude_patterns: list[str]) -> list[Path]:
    files: list[Path] = []
    seen: set[Path] = set()
    for p in inputs:
        if p.is_file() and p.suffix.lower() == ".csv":
            cand = p.resolve()
            if cand not in seen:
                files.append(cand)
                seen.add(cand)
        elif p.is_dir():
            it = p.rglob("*.csv") if recursive else p.glob("*.csv")
            for cand in it:
                rc = cand.resolve()
                if rc in seen:
                    continue
                s = str(rc)
                if include_patterns and not any(pat in s for pat in include_patterns):
                    continue
                if exclude_patterns and any(pat in s for pat in exclude_patterns):
                    continue
                files.append(rc)
                seen.add(rc)
    return sorted(files)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("inputs", nargs="+", help="CSV file(s) or director(ies) to scan.")
    ap.add_argument("--recursive", action="store_true", help="Recurse into subdirectories.")
    ap.add_argument("--gyro-col", default=None, help="Explicit gyro column name.")
    ap.add_argument("--time-col", default=None, help="Explicit timestamp column name.")
    ap.add_argument("--output-dir", default="allan_batch_output", help="Directory for outputs.")
    ap.add_argument("--cmd-linear-thresh", type=float, default=0.01)
    ap.add_argument("--cmd-angular-thresh", type=float, default=0.10)
    ap.add_argument("--encoder-vel-thresh", type=float, default=0.02)
    ap.add_argument("--break-consecutive", type=int, default=25)
    ap.add_argument("--min-prefix-samples", type=int, default=100)
    ap.add_argument("--max-rows", type=int, default=None, help="Optional row limit per CSV for testing.")
    ap.add_argument("--include", action="append", default=[], help="Only process paths containing this substring. Can repeat.")
    ap.add_argument("--exclude", action="append", default=[], help="Skip paths containing this substring. Can repeat.")
    args = ap.parse_args()

    inputs = [Path(x) for x in args.inputs]
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    csvs = iter_csv_files(inputs, recursive=args.recursive, include_patterns=args.include, exclude_patterns=args.exclude)
    if not csvs:
        print("No CSV files found.", flush=True)
        return 2

    summaries: list[AllanSummary] = []
    failures: list[dict[str, str]] = []

    for csv_path in csvs:
        try:
            summary = analyze_log(
                csv_path,
                output_dir,
                gyro_col=args.gyro_col,
                time_col=args.time_col,
                cmd_linear_thresh=args.cmd_linear_thresh,
                cmd_angular_thresh=args.cmd_angular_thresh,
                encoder_vel_thresh=args.encoder_vel_thresh,
                break_consecutive=args.break_consecutive,
                min_prefix_samples=args.min_prefix_samples,
                max_rows=args.max_rows,
            )
            summaries.append(summary)
            print(f"[OK] {csv_path}")
        except Exception as e:
            failures.append({
                "file": str(csv_path),
                "error": f"{type(e).__name__}: {e}",
                "traceback": traceback.format_exc(limit=3),
            })
            print(f"[FAIL] {csv_path}: {type(e).__name__}: {e}")

    if summaries:
        agg_df = pd.DataFrame([asdict(x) for x in summaries])
        agg_csv = output_dir / "allan_stationary_batch_summary.csv"
        agg_df.to_csv(agg_csv, index=False)

    agg_json = output_dir / "allan_stationary_batch_report.json"
    report = {
        "files_seen": len(csvs),
        "files_succeeded": len(summaries),
        "files_failed": len(failures),
        "output_dir": str(output_dir.resolve()),
        "aggregate_csv": str((output_dir / "allan_stationary_batch_summary.csv").resolve()) if summaries else None,
        "failures": failures,
    }
    with agg_json.open("w", encoding="utf-8") as f:
        json.dump(report, f, indent=2)

    print()
    print(f"Succeeded: {len(summaries)} / {len(csvs)}")
    print(f"Failed:    {len(failures)} / {len(csvs)}")
    print(f"Report:    {agg_json}")
    if summaries:
        print(f"Summary:   {output_dir / 'allan_stationary_batch_summary.csv'}")

    return 0 if summaries else 1


if __name__ == "__main__":
    raise SystemExit(main())
