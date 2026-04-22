#!/usr/bin/env python3
"""
Analyze the opening stationary section of one or more CSV logs and estimate gyro noise
terms using overlapping Allan deviation.

Outputs, per log:
  * stationary section boundaries and duration
  * sample-rate estimate
  * direct sample noise stats
  * Allan deviation curve CSV
  * JSON summary with white-noise, bias-instability, and rate-random-walk estimates

No external Allan-deviation package is required.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Iterable, Optional

import numpy as np
import pandas as pd

try:
    import matplotlib.pyplot as plt  # optional
    HAVE_MPL = True
except Exception:
    HAVE_MPL = False


DEFAULT_GYRO_COL_CANDIDATES = [
    "gyro_raw_radps",
    "gyro_radps",
    "imu_gyro_z",
    "measured_angular_speed_radps",
]
DEFAULT_TIME_COL_CANDIDATES = [
    "imu_timestamp_us",
    "master_time_us",
    "encoder_timestamp_us",
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
    recommended_Rg_radps2: float
    bias_instability_radps: Optional[float]
    bias_instability_dps: Optional[float]
    bias_instability_tau_s: Optional[float]
    rate_random_walk_radps_per_sqrt_s: Optional[float]
    derived_Q_bgz_from_rrw_radps2_per_sample: Optional[float]
    q_bgz_policy_moving_radps2_per_sample: float


def choose_existing(columns: Iterable[str], candidates: list[str]) -> Optional[str]:
    s = set(columns)
    for c in candidates:
        if c in s:
            return c
    return None


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

    if cmd_linear_col and cmd_linear_col in df:
        stationary &= np.abs(pd.to_numeric(df[cmd_linear_col], errors="coerce").fillna(0.0).to_numpy()) <= cmd_linear_thresh
    if cmd_angular_col and cmd_angular_col in df:
        stationary &= np.abs(pd.to_numeric(df[cmd_angular_col], errors="coerce").fillna(0.0).to_numpy()) <= cmd_angular_thresh
    if left_vel_col and left_vel_col in df:
        stationary &= np.abs(pd.to_numeric(df[left_vel_col], errors="coerce").fillna(0.0).to_numpy()) <= encoder_vel_thresh
    if right_vel_col and right_vel_col in df:
        stationary &= np.abs(pd.to_numeric(df[right_vel_col], errors="coerce").fillna(0.0).to_numpy()) <= encoder_vel_thresh

    # Find the first sustained run of non-stationary samples after a minimum opening prefix.
    nonstat = ~stationary
    run = 0
    for i in range(n):
        if nonstat[i]:
            run += 1
        else:
            run = 0
        if i + 1 >= max(min_prefix_samples, break_consecutive) and run >= break_consecutive:
            end_idx = i - break_consecutive
            end_idx = max(0, end_idx)
            return 0, end_idx

    # If no sustained break is found, use the longest opening prefix until the first hard violation,
    # else the whole file.
    hard_breaks = np.flatnonzero(nonstat)
    if hard_breaks.size > 0 and hard_breaks[0] >= min_prefix_samples:
        return 0, int(hard_breaks[0] - 1)
    return 0, n - 1


def unique_log_spaced_ints(start: int, stop: int, count: int) -> np.ndarray:
    if stop < start:
        return np.array([], dtype=int)
    vals = np.unique(np.clip(np.round(np.logspace(np.log10(start), np.log10(stop), count)), start, stop).astype(int))
    return vals


def overlapping_allan_deviation_rate(y: np.ndarray, dt: float, max_cluster: Optional[int] = None) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Compute overlapping Allan deviation for rate data y sampled every dt seconds.

    Returns (taus, adev, ns) where ns is the number of overlapping differences used.
    """
    y = np.asarray(y, dtype=float)
    n = y.size
    if n < 3:
        return np.array([]), np.array([]), np.array([])
    if max_cluster is None:
        max_cluster = max(1, n // 10)
    max_cluster = min(max_cluster, (n - 1) // 2)
    if max_cluster < 1:
        return np.array([]), np.array([]), np.array([])

    ms = unique_log_spaced_ints(1, max_cluster, 60)
    csum = np.concatenate(([0.0], np.cumsum(y)))
    taus = []
    adev = []
    ns = []
    for m in ms:
        # Moving averages over m-sample windows.
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
    lx = np.log10(x)
    ly = np.log10(y)
    return np.diff(ly) / np.diff(lx)


def fit_white_noise_density(taus: np.ndarray, adev: np.ndarray) -> Optional[float]:
    if len(taus) < 2:
        return None
    slopes = adjacent_log_slopes(taus, adev)
    mid_mask = (slopes > -0.7) & (slopes < -0.3)
    # Map slope mask to point mask.
    point_mask = np.zeros(len(taus), dtype=bool)
    point_mask[:-1] |= mid_mask
    point_mask[1:] |= mid_mask
    # Prefer small taus.
    point_mask &= taus <= np.percentile(taus, 40)
    if not np.any(point_mask):
        point_mask = np.zeros(len(taus), dtype=bool)
        point_mask[:min(5, len(taus))] = True
    vals = adev[point_mask] * np.sqrt(taus[point_mask])
    vals = vals[np.isfinite(vals)]
    if vals.size == 0:
        return None
    return float(np.median(vals))


def fit_bias_instability(taus: np.ndarray, adev: np.ndarray) -> tuple[Optional[float], Optional[float]]:
    if len(taus) == 0:
        return None, None
    idx = int(np.argmin(adev))
    sigma_min = float(adev[idx])
    tau_min = float(taus[idx])
    # Approximation for flicker-noise coefficient from Allan deviation minimum.
    bias_instability = sigma_min / 0.664
    return bias_instability, tau_min


def fit_rate_random_walk(taus: np.ndarray, adev: np.ndarray) -> Optional[float]:
    if len(taus) < 3:
        return None
    slopes = adjacent_log_slopes(taus, adev)
    cand = (slopes > 0.3) & (slopes < 0.7)
    point_mask = np.zeros(len(taus), dtype=bool)
    point_mask[:-1] |= cand
    point_mask[1:] |= cand
    # Prefer larger taus.
    point_mask &= taus >= np.percentile(taus, 50)
    if not np.any(point_mask):
        return None
    vals = adev[point_mask] * np.sqrt(3.0 / taus[point_mask])
    vals = vals[np.isfinite(vals)]
    if vals.size == 0:
        return None
    return float(np.median(vals))


def estimate_summary(
    file: Path,
    df: pd.DataFrame,
    time_col: str,
    gyro_col: str,
    start_idx: int,
    end_idx: int,
) -> tuple[AllanSummary, pd.DataFrame]:
    seg = df.iloc[start_idx:end_idx + 1].copy()
    t_raw = pd.to_numeric(seg[time_col], errors="coerce").to_numpy(dtype=float)
    y = pd.to_numeric(seg[gyro_col], errors="coerce").to_numpy(dtype=float)
    valid = np.isfinite(t_raw) & np.isfinite(y)
    seg = seg.loc[valid].copy()
    t_raw = t_raw[valid]
    y = y[valid]

    # Convert timestamp units. Prefer explicit column suffixes, then fall back to magnitude.
    dt_raw = np.diff(t_raw)
    dt_raw = dt_raw[np.isfinite(dt_raw) & (dt_raw > 0)]
    if dt_raw.size == 0:
        raise ValueError(f"No positive time deltas in stationary segment for {file}")
    median_raw = float(np.median(dt_raw))
    time_col_lower = time_col.lower()
    if time_col_lower.endswith("_us") or "micro" in time_col_lower:
        time_scale = 1e-6
    elif time_col_lower.endswith("_ms") or "milli" in time_col_lower:
        time_scale = 1e-3
    elif median_raw >= 1e3:
        time_scale = 1e-6  # likely microseconds
    elif median_raw >= 1.0:
        time_scale = 1e-3  # likely milliseconds
    else:
        time_scale = 1.0   # seconds
    t_s = t_raw * time_scale
    dt_s = float(np.median(np.diff(t_s)))
    fs = 1.0 / dt_s

    # Remove constant offset for Allan processing; Allan itself is insensitive to DC,
    # but centering makes other derived stats more readable.
    y_centered = y - np.mean(y)
    sample_std = float(np.std(y_centered, ddof=1)) if len(y_centered) > 1 else 0.0
    sample_var = sample_std ** 2

    taus, adev, ns = overlapping_allan_deviation_rate(y_centered, dt_s)
    allan_df = pd.DataFrame({
        "tau_s": taus,
        "adev_radps": adev,
        "num_differences": ns,
    })

    white = fit_white_noise_density(taus, adev)
    bias_instab, bias_tau = fit_bias_instability(taus, adev)
    rrw = fit_rate_random_walk(taus, adev)

    # Recommended measurement variance from direct stationary sample variance.
    recommended_Rg = sample_var

    derived_q = None
    if rrw is not None:
        # Continuous bias random-walk intensity q_b = K^2, discrete Q = q_b * dt.
        derived_q = float((rrw ** 2) * dt_s)

    summary = AllanSummary(
        file=str(file),
        rows_total=len(df),
        rows_stationary=len(seg),
        stationary_start_index=int(start_idx),
        stationary_end_index=int(end_idx),
        stationary_start_time_s=float(t_s[0]),
        stationary_end_time_s=float(t_s[-1]),
        stationary_duration_s=float(t_s[-1] - t_s[0]),
        time_column=time_col,
        gyro_column=gyro_col,
        dt_s_median=dt_s,
        sample_rate_hz=fs,
        sample_mean_radps=float(np.mean(y)),
        sample_std_radps=sample_std,
        sample_variance_radps2=sample_var,
        white_noise_density_radps_per_sqrt_hz=white,
        white_noise_density_mdps_per_sqrt_hz=(white * 180.0 / math.pi * 1e3) if white is not None else None,
        recommended_Rg_radps2=recommended_Rg,
        bias_instability_radps=bias_instab,
        bias_instability_dps=(bias_instab * 180.0 / math.pi) if bias_instab is not None else None,
        bias_instability_tau_s=bias_tau,
        rate_random_walk_radps_per_sqrt_s=rrw,
        derived_Q_bgz_from_rrw_radps2_per_sample=derived_q,
        q_bgz_policy_moving_radps2_per_sample=0.0,
    )
    return summary, allan_df


def collect_inputs(paths: list[str], recursive: bool) -> list[Path]:
    out: list[Path] = []
    for p_str in paths:
        p = Path(p_str)
        if p.is_file() and p.suffix.lower() == ".csv":
            out.append(p)
        elif p.is_dir():
            pattern = "**/*.csv" if recursive else "*.csv"
            out.extend(sorted(p.glob(pattern)))
        else:
            # Allow glob-like paths expanded by shell or literal non-existing path.
            matches = list(Path().glob(p_str))
            out.extend([m for m in matches if m.is_file() and m.suffix.lower() == ".csv"])
    # Deduplicate while preserving order.
    seen = set()
    uniq = []
    for p in out:
        rp = str(p.resolve())
        if rp not in seen:
            seen.add(rp)
            uniq.append(p)
    return uniq


def save_plot(path: Path, allan_df: pd.DataFrame, summary: AllanSummary) -> None:
    if not HAVE_MPL or allan_df.empty:
        return
    fig, ax = plt.subplots(figsize=(7, 4.5))
    ax.loglog(allan_df["tau_s"], allan_df["adev_radps"], marker="o", linewidth=1)
    ax.set_xlabel("Tau [s]")
    ax.set_ylabel("Allan deviation [rad/s]")
    ax.set_title(Path(summary.file).name)
    ax.grid(True, which="both", alpha=0.3)
    # Reference lines
    if summary.white_noise_density_radps_per_sqrt_hz is not None:
        taus = allan_df["tau_s"].to_numpy()
        ref = summary.white_noise_density_radps_per_sqrt_hz / np.sqrt(taus)
        ax.loglog(taus, ref, "--", linewidth=1, label="white-noise fit")
    if summary.bias_instability_radps is not None and summary.bias_instability_tau_s is not None:
        ax.scatter([summary.bias_instability_tau_s], [0.664 * summary.bias_instability_radps], s=30, label="bias-instability min")
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def main(argv: Optional[list[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("inputs", nargs="+", help="CSV file(s), directories, or glob patterns")
    ap.add_argument("--recursive", action="store_true", help="Recurse into directories")
    ap.add_argument("--output-dir", default="allan_output", help="Directory for JSON/CSV/PNG outputs")
    ap.add_argument("--gyro-col", default=None, help="Gyro rate column in rad/s (default: auto-detect)")
    ap.add_argument("--time-col", default=None, help="Timestamp column (default: auto-detect)")
    ap.add_argument("--cmd-linear-col", default="cmd_linear_mps")
    ap.add_argument("--cmd-angular-col", default="cmd_angular_radps")
    ap.add_argument("--left-enc-vel-col", default="left_encoder_velocity_mps")
    ap.add_argument("--right-enc-vel-col", default="right_encoder_velocity_mps")
    ap.add_argument("--cmd-linear-thresh", type=float, default=0.01, help="Opening stationary threshold for |cmd_linear_mps|")
    ap.add_argument("--cmd-angular-thresh", type=float, default=0.10, help="Opening stationary threshold for |cmd_angular_radps|")
    ap.add_argument("--encoder-vel-thresh", type=float, default=0.02, help="Opening stationary threshold for encoder velocity magnitude")
    ap.add_argument("--break-consecutive", type=int, default=25, help="Consecutive nonstationary samples required to terminate the opening stationary prefix")
    ap.add_argument("--min-prefix-samples", type=int, default=100, help="Require at least this many opening samples before ending the stationary prefix")
    ap.add_argument("--no-plot", action="store_true", help="Do not emit PNG Allan plots")
    ns = ap.parse_args(argv)

    files = collect_inputs(ns.inputs, recursive=ns.recursive)
    if not files:
        print("No CSV files found.", file=sys.stderr)
        return 2

    outdir = Path(ns.output_dir)
    outdir.mkdir(parents=True, exist_ok=True)

    summaries: list[AllanSummary] = []
    for file in files:
        try:
            df = pd.read_csv(file)
            time_col = ns.time_col or choose_existing(df.columns, DEFAULT_TIME_COL_CANDIDATES)
            gyro_col = ns.gyro_col or choose_existing(df.columns, DEFAULT_GYRO_COL_CANDIDATES)
            if time_col is None:
                raise ValueError("Could not auto-detect time column")
            if gyro_col is None:
                raise ValueError("Could not auto-detect gyro column")

            start_idx, end_idx = detect_opening_stationary_prefix(
                df,
                ns.cmd_linear_col if ns.cmd_linear_col in df.columns else None,
                ns.cmd_angular_col if ns.cmd_angular_col in df.columns else None,
                ns.left_enc_vel_col if ns.left_enc_vel_col in df.columns else None,
                ns.right_enc_vel_col if ns.right_enc_vel_col in df.columns else None,
                cmd_linear_thresh=ns.cmd_linear_thresh,
                cmd_angular_thresh=ns.cmd_angular_thresh,
                encoder_vel_thresh=ns.encoder_vel_thresh,
                break_consecutive=ns.break_consecutive,
                min_prefix_samples=ns.min_prefix_samples,
            )

            summary, allan_df = estimate_summary(file, df, time_col, gyro_col, start_idx, end_idx)
            summaries.append(summary)

            stem = file.stem
            (outdir / f"{stem}.allan.csv").write_text(allan_df.to_csv(index=False))
            (outdir / f"{stem}.summary.json").write_text(json.dumps(asdict(summary), indent=2))
            if not ns.no_plot:
                save_plot(outdir / f"{stem}.allan.png", allan_df, summary)

            print(f"[{file.name}]")
            print(f"  stationary rows : {summary.rows_stationary} / {summary.rows_total}")
            print(f"  stationary time : {summary.stationary_duration_s:.3f} s")
            print(f"  sample rate     : {summary.sample_rate_hz:.3f} Hz")
            print(f"  sample std      : {summary.sample_std_radps:.6g} rad/s")
            print(f"  R_g (sample var): {summary.recommended_Rg_radps2:.6g} (rad/s)^2")
            if summary.white_noise_density_radps_per_sqrt_hz is not None:
                print(f"  white noise dens: {summary.white_noise_density_radps_per_sqrt_hz:.6g} rad/s/√Hz")
                print(f"                   {summary.white_noise_density_mdps_per_sqrt_hz:.3f} mdps/√Hz")
            if summary.bias_instability_radps is not None:
                print(f"  bias instability: {summary.bias_instability_radps:.6g} rad/s @ tau≈{summary.bias_instability_tau_s:.3g} s")
            if summary.rate_random_walk_radps_per_sqrt_s is not None:
                print(f"  rate random walk: {summary.rate_random_walk_radps_per_sqrt_s:.6g} rad/s/√s")
                print(f"  Q_bgz from RRW  : {summary.derived_Q_bgz_from_rrw_radps2_per_sample:.6g} (rad/s)^2/sample")
            print(f"  moving Q_bgz policy: {summary.q_bgz_policy_moving_radps2_per_sample:.6g} (rad/s)^2/sample")
            print()
        except Exception as exc:
            print(f"[{file}] ERROR: {exc}", file=sys.stderr)

    if summaries:
        (outdir / "index.json").write_text(json.dumps([asdict(s) for s in summaries], indent=2))
        return 0
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
