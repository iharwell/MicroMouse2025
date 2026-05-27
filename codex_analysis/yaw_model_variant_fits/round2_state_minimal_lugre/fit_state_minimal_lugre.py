#!/usr/bin/env python3
"""Round 2 yaw-contact LuGre/Dahl-style residual model evaluation.

Analysis-only tooling. This script reads the existing contact-continuum feature
artifacts and writes outputs only beside itself. It intentionally does not edit
production code, build metadata, or tests.
"""

from __future__ import annotations

import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np
import pandas as pd


ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent
PRIMARY = (
    ROOT
    / "codex_analysis"
    / "contact_continuum_yaw_identification"
    / "ablation"
    / "phase_classified_feature_sample.csv"
)
CONSTANTS = (
    ROOT
    / "codex_analysis"
    / "contact_continuum_yaw_identification"
    / "features"
    / "plant_mirror_constants.csv"
)
B_COEFFS = ROOT / "codex_analysis" / "yaw_model_variant_fits" / "stribeck_scrub" / "stribeck_coefficients.csv"
C_SPLIT = ROOT / "codex_analysis" / "yaw_model_variant_fits" / "combined_slip_surface" / "split_metrics.csv"
C_SELECTED = ROOT / "codex_analysis" / "yaw_model_variant_fits" / "combined_slip_surface" / "selected_log_metrics.csv"
C_COEFFS = ROOT / "codex_analysis" / "yaw_model_variant_fits" / "combined_slip_surface" / "model_coefficients.csv"

SELECTED_RUNS = [
    "2026-05-04_20-35-47",
    "2026-05-04_16-57-53",
    "2026-04-22_12-10-34",
    "2026-04-22_01-06-32",
    "2026-04-21_05-32-06",
    "2026-04-21_00-16-10",
    "2026-04-20_12-10-58",
    "2026-04-20_08-38-39",
    "diag003",
]

INPUT_COLUMNS = [
    "run_id",
    "family",
    "schema",
    "recommendation",
    "dataset_split",
    "row_index",
    "time_us",
    "physics_phase",
    "physics_active",
    "phase_progress",
    "forward_velocity_mps",
    "yaw_rate_radps",
    "measured_yaw_accel_radps2",
    "vbar_rel_mps",
    "vbar_lat_mps",
    "vbar_yaw_mps",
    "max_force_preprojection_utilization",
    "max_force_limiter_activity",
    "hardware_saturation_evidence",
    "gyro_derivative_spike",
    "residual_additive_yaw_torque_nm",
    "residual_opposes_yaw_nm",
    "patch_yaw_req_basis_nm",
    "patch_yaw_force_basis_nm",
    "patch_yaw_velocity_basis_m2ps",
    "patch_yaw_abs_velocity_basis_m2ps",
]

SPLIT_ORDER = [
    "primary_open_floor_fit_authoritative",
    "open_floor_fit_downweighted",
    "open_floor_validation_only",
    "diag_validation_only",
    "aux_downweighted_validation",
    "validation_non_authoritative",
]


@dataclass(frozen=True)
class LugreParams:
    req_activation_nm: float
    stribeck_speed_mps: float
    speed_fade_mps: float
    rel_weight: float
    tau_fill_s: float
    bristle_slip_distance_m: float


@dataclass
class FitResult:
    model: str
    params: LugreParams
    beta: np.ndarray
    feature_names: list[str]
    fixedpoint_extra_nm: float
    beta_scale_to_gate: float
    weighted_train_rmse_nm: float
    primary_rmse_nm: float
    validation_rmse_nm: float
    selected_score_nm: float


def read_constants() -> dict[str, float]:
    with CONSTANTS.open(newline="", encoding="utf-8") as fh:
        return {row["name"]: float(row["value"]) for row in csv.DictReader(fh)}


def read_key_value_csv(path: Path) -> dict[str, float]:
    with path.open(newline="", encoding="utf-8") as fh:
        return {row["parameter"]: float(row["value"]) for row in csv.DictReader(fh)}


def read_coeff_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as fh:
        return list(csv.DictReader(fh))


def write_csv(path: Path, rows: Iterable[dict[str, object]]) -> None:
    rows = list(rows)
    fields: list[str] = []
    for row in rows:
        for key in row:
            if key not in fields:
                fields.append(key)
    with path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def sign_array(values: np.ndarray, eps: float = 1.0e-6) -> np.ndarray:
    out = np.zeros(len(values), dtype=float)
    out[values > eps] = 1.0
    out[values < -eps] = -1.0
    return out


def sign(value: float, eps: float = 1.0e-6) -> float:
    return float((value > eps) - (value < -eps))


def smooth_positive(values: np.ndarray | float, epsilon: float = 1.0e-6) -> np.ndarray | float:
    return 0.5 * (values + np.sqrt(values * values + epsilon * epsilon))


def smooth_gate(value: float, scale: float) -> float:
    ratio = abs(value) / scale if scale > 0.0 else 0.0
    return 1.0 / (1.0 + ratio * ratio)


def fmt(value: object, digits: int = 6) -> str:
    try:
        x = float(value)
    except (TypeError, ValueError):
        return str(value)
    if not math.isfinite(x):
        return ""
    if abs(x) >= 1000.0 or (0.0 < abs(x) < 1.0e-4):
        return f"{x:.{digits}g}"
    return f"{x:.{digits}f}".rstrip("0").rstrip(".")


def load_frame() -> pd.DataFrame:
    frame = pd.read_csv(PRIMARY, usecols=INPUT_COLUMNS)
    numeric_columns = [
        col
        for col in INPUT_COLUMNS
        if col
        not in {
            "run_id",
            "family",
            "schema",
            "recommendation",
            "dataset_split",
            "physics_phase",
        }
    ]
    for column in numeric_columns:
        frame[column] = pd.to_numeric(frame[column], errors="coerce")
    frame = frame.replace([np.inf, -np.inf], np.nan)
    frame = frame.dropna(
        subset=[
            "run_id",
            "dataset_split",
            "row_index",
            "time_us",
            "forward_velocity_mps",
            "yaw_rate_radps",
            "vbar_rel_mps",
            "vbar_yaw_mps",
            "residual_additive_yaw_torque_nm",
            "residual_opposes_yaw_nm",
            "patch_yaw_req_basis_nm",
        ]
    )
    frame = frame[frame["dataset_split"] != "excluded_or_unclassified"].copy()
    frame = frame.sort_values(["run_id", "time_us", "row_index"], kind="mergesort").reset_index(drop=True)
    frame["abs_forward_velocity_mps"] = frame["forward_velocity_mps"].abs()
    frame["abs_yaw_rate_radps"] = frame["yaw_rate_radps"].abs()
    yaw_sign = sign_array(frame["yaw_rate_radps"].to_numpy())
    req_sign = sign_array(frame["patch_yaw_req_basis_nm"].to_numpy())
    yaw_sign[yaw_sign == 0.0] = req_sign[yaw_sign == 0.0]
    yaw_sign[yaw_sign == 0.0] = 1.0
    frame["yaw_sign"] = yaw_sign
    frame["positive_patch_yaw_req_basis_nm"] = smooth_positive(
        frame["patch_yaw_req_basis_nm"].to_numpy(), epsilon=1.0e-6
    )
    frame["dt_s"] = frame.groupby("run_id")["time_us"].diff().fillna(1000.0) / 1.0e6
    frame.loc[(frame["dt_s"] <= 0.0) | (frame["dt_s"] > 0.25), "dt_s"] = 0.001
    return frame


def training_weights(frame: pd.DataFrame) -> np.ndarray:
    weights = np.zeros(len(frame), dtype=float)
    split = frame["dataset_split"].to_numpy()
    recommendation = frame["recommendation"].to_numpy()
    family = frame["family"].to_numpy()
    weights[split == "primary_open_floor_fit_authoritative"] = 1.0
    downweighted = (
        (split == "open_floor_fit_downweighted")
        & (recommendation == "fit_downweighted")
        & (family == "open_floor")
    )
    weights[downweighted] = 0.25

    limiter = np.clip(frame["max_force_limiter_activity"].fillna(0.0).to_numpy(), 0.0, 1.0)
    saturation = np.clip(frame["hardware_saturation_evidence"].fillna(0.0).to_numpy(), 0.0, 1.0)
    spike = np.clip(frame["gyro_derivative_spike"].fillna(0.0).to_numpy(), 0.0, 1.0)
    quality = (1.0 / (1.0 + 4.0 * limiter)) * (1.0 - 0.75 * saturation) * (1.0 - 0.75 * spike)
    quality = np.clip(quality, 0.02, 1.0)
    weights *= quality

    fit_counts = frame.loc[weights > 0.0, "run_id"].value_counts()
    if not fit_counts.empty:
        run_scale = frame["run_id"].map({run: 1.0 / math.sqrt(count) for run, count in fit_counts.items()}).fillna(0.0)
        weights *= run_scale.to_numpy()
        positive = weights > 0.0
        weights[positive] *= positive.sum() / weights[positive].sum()
    return weights


def weighted_nnls(features: np.ndarray, target: np.ndarray, weights: np.ndarray) -> np.ndarray:
    sqrt_w = np.sqrt(np.clip(weights, 0.0, None))
    xw = features * sqrt_w[:, None]
    yw = target * sqrt_w
    beta = np.zeros(features.shape[1], dtype=float)
    residual = yw.copy()
    col_norm = np.sum(xw * xw, axis=0)
    col_norm = np.where(col_norm > 1.0e-18, col_norm, 1.0)
    for _ in range(120):
        max_delta = 0.0
        for col in range(features.shape[1]):
            old = beta[col]
            residual += xw[:, col] * old
            new = np.dot(xw[:, col], residual) / col_norm[col]
            new = max(0.0, new)
            beta[col] = new
            residual -= xw[:, col] * new
            max_delta = max(max_delta, abs(new - old))
        if max_delta < 1.0e-11:
            break
    return beta


def lugre_components(
    frame: pd.DataFrame,
    params: LugreParams,
    model: str,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    abs_vf = frame["abs_forward_velocity_mps"].to_numpy()
    transition = np.sqrt(np.square(params.rel_weight * frame["vbar_rel_mps"].to_numpy()) + np.square(abs_vf))
    activation = 1.0 - np.exp(-np.square(frame["positive_patch_yaw_req_basis_nm"].to_numpy() / params.req_activation_nm))
    stribeck = np.exp(-np.square(transition / params.stribeck_speed_mps))
    relief = 1.0 / (1.0 + np.square(transition / params.speed_fade_mps))
    denom = 1.0 + params.tau_fill_s * transition / params.bristle_slip_distance_m
    q_eq = np.clip(activation / denom, 0.0, 1.0)
    if model == "memoryless_equilibrium":
        return q_eq, activation, transition, stribeck, relief
    if model != "state_minimal_lugre":
        raise ValueError(f"unknown model: {model}")

    q = np.zeros(len(frame), dtype=float)
    previous_run = None
    previous_direction = 0.0
    previous_q = 0.0
    runs = frame["run_id"].to_numpy()
    direction = frame["yaw_sign"].to_numpy()
    dt = frame["dt_s"].to_numpy()
    for idx in range(len(frame)):
        run = runs[idx]
        new_run = run != previous_run
        flipped = (
            not new_run
            and previous_direction != 0.0
            and direction[idx] != 0.0
            and direction[idx] != previous_direction
        )
        if new_run:
            previous_q = q_eq[idx]
        elif flipped:
            previous_q = 0.0

        rate = (1.0 / params.tau_fill_s) + (transition[idx] / params.bristle_slip_distance_m)
        dt_eff = min(max(float(dt[idx]), 0.001), 0.050)
        current_q = q_eq[idx] + (previous_q - q_eq[idx]) * math.exp(-rate * dt_eff)
        q[idx] = min(max(current_q, 0.0), 1.0)
        previous_q = q[idx]
        previous_run = run
        if direction[idx] != 0.0:
            previous_direction = direction[idx]
    return q, activation, transition, stribeck, relief


def lugre_features(frame: pd.DataFrame, params: LugreParams, model: str) -> tuple[np.ndarray, list[str]]:
    q, _activation, _transition, stribeck, relief = lugre_components(frame, params, model)
    vbar_yaw = frame["vbar_yaw_mps"].to_numpy()
    features = np.column_stack(
        [
            q * relief * stribeck,
            q * relief,
            q * relief * vbar_yaw,
        ]
    )
    names = [
        "bristle_stribeck_static_nm",
        "bristle_sliding_nm",
        "bristle_viscous_nm_per_mps",
    ]
    return features, names


def residuals_after(frame: pd.DataFrame, pred_opposes: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    baseline = frame["residual_additive_yaw_torque_nm"].to_numpy()
    pred_additive = -frame["yaw_sign"].to_numpy() * pred_opposes
    corrected = baseline - pred_additive
    return baseline, corrected


def metric_values(frame: pd.DataFrame, pred_opposes: np.ndarray) -> dict[str, float]:
    if len(frame) == 0:
        return {
            "count": 0,
            "run_count": 0,
            "baseline_rmse_nm": math.nan,
            "corrected_rmse_nm": math.nan,
            "baseline_mae_nm": math.nan,
            "corrected_mae_nm": math.nan,
            "baseline_median_abs_nm": math.nan,
            "corrected_median_abs_nm": math.nan,
            "baseline_signed_median_nm": math.nan,
            "corrected_signed_median_nm": math.nan,
        }
    baseline, corrected = residuals_after(frame, pred_opposes)
    return {
        "count": int(len(frame)),
        "run_count": int(frame["run_id"].nunique()),
        "baseline_rmse_nm": float(np.sqrt(np.mean(np.square(baseline)))),
        "corrected_rmse_nm": float(np.sqrt(np.mean(np.square(corrected)))),
        "baseline_mae_nm": float(np.mean(np.abs(baseline))),
        "corrected_mae_nm": float(np.mean(np.abs(corrected))),
        "baseline_median_abs_nm": float(np.median(np.abs(baseline))),
        "corrected_median_abs_nm": float(np.median(np.abs(corrected))),
        "baseline_signed_median_nm": float(np.median(baseline)),
        "corrected_signed_median_nm": float(np.median(corrected)),
    }


def group_metrics(frame: pd.DataFrame, pred_opposes: np.ndarray, model_name: str) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    index = frame.index.to_numpy()
    for split in SPLIT_ORDER[:-1]:
        mask = frame["dataset_split"].to_numpy() == split
        if not np.any(mask):
            continue
        metric = metric_values(frame.loc[mask], pred_opposes[np.isin(index, frame.loc[mask].index.to_numpy())])
        metric["group"] = split
        metric["model"] = model_name
        rows.append(metric)
    validation_mask = frame["dataset_split"].isin(
        [
            "open_floor_fit_downweighted",
            "open_floor_validation_only",
            "diag_validation_only",
            "aux_downweighted_validation",
        ]
    ).to_numpy()
    metric = metric_values(frame.loc[validation_mask], pred_opposes[np.isin(index, frame.loc[validation_mask].index.to_numpy())])
    metric["group"] = "validation_non_authoritative"
    metric["model"] = model_name
    rows.append(metric)
    return rows


def selected_metrics(frame: pd.DataFrame, pred_opposes: np.ndarray, model_name: str) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    index = frame.index.to_numpy()
    for run_id in SELECTED_RUNS:
        mask = frame["run_id"].to_numpy() == run_id
        if not np.any(mask):
            rows.append(
                {
                    "model": model_name,
                    "run_id": run_id,
                    "present": False,
                    "dataset_split": "",
                    "count": 0,
                }
            )
            continue
        subset_index = frame.loc[mask].index.to_numpy()
        metric = metric_values(frame.loc[mask], pred_opposes[np.isin(index, subset_index)])
        metric["model"] = model_name
        metric["run_id"] = run_id
        metric["present"] = True
        metric["dataset_split"] = str(frame.loc[mask, "dataset_split"].iloc[0])
        rows.append(metric)
    return rows


def fit_training_target(frame: pd.DataFrame, features: np.ndarray, weights: np.ndarray, anchor: tuple[np.ndarray, float]) -> np.ndarray:
    target = frame["residual_opposes_yaw_nm"].to_numpy()
    anchor_x, anchor_y = anchor
    x = np.vstack([features, anchor_x.reshape(1, -1)])
    y = np.concatenate([target, np.array([anchor_y])])
    w = np.concatenate([weights, np.array([4000.0])])
    return weighted_nnls(x, y, w)


def baseline_opposing_yaw_torque(constants: dict[str, float], yaw_rate: float) -> float:
    longitudinal = constants["drive_wheel_longitudinal_offset_m"]
    front_gain = constants["front_right_contact_force_gain_n_per_mps"]
    rear_gain = constants["rear_right_contact_force_gain_n_per_mps"]
    front_right_velocity = -longitudinal * yaw_rate
    rear_right_velocity = longitudinal * yaw_rate
    front_right_force_total = 2.0 * front_gain * front_right_velocity
    rear_right_force_total = 2.0 * rear_gain * rear_right_velocity
    yaw_moment = longitudinal * (front_right_force_total - rear_right_force_total)
    return -yaw_moment


def b_extra_fixedpoint(base_opposing: float, constants: dict[str, float], vf_mps: float, yaw_rate: float) -> float:
    coeff = read_key_value_csv(B_COEFFS)
    longitudinal = constants["drive_wheel_longitudinal_offset_m"]
    vbar_rel = longitudinal * abs(yaw_rate)
    transition = math.hypot(coeff["rel_weight"] * vbar_rel, abs(vf_mps))
    stribeck = math.exp(-((transition / coeff["stribeck_speed_mps"]) ** 2))
    relief = 1.0 / (1.0 + (transition / coeff["speed_fade_mps"]) ** 2)
    extra = 0.0
    for _ in range(60):
        requested = base_opposing + extra
        activation = 1.0 - math.exp(-((smooth_positive(requested) / coeff["req_activation_nm"]) ** 2))
        next_extra = activation * relief * (coeff["static_extra_nm"] * stribeck + coeff["sliding_nm"])
        if abs(next_extra - extra) < 1.0e-13:
            return float(next_extra)
        extra = float(next_extra)
    return float(extra)


def b_prediction(frame: pd.DataFrame) -> np.ndarray:
    coeff = read_key_value_csv(B_COEFFS)
    transition = np.sqrt(
        np.square(coeff["rel_weight"] * frame["vbar_rel_mps"].to_numpy())
        + np.square(frame["abs_forward_velocity_mps"].to_numpy())
    )
    activation = 1.0 - np.exp(
        -np.square(frame["positive_patch_yaw_req_basis_nm"].to_numpy() / coeff["req_activation_nm"])
    )
    stribeck = np.exp(-np.square(transition / coeff["stribeck_speed_mps"]))
    relief = 1.0 / (1.0 + np.square(transition / coeff["speed_fade_mps"]))
    return activation * relief * (coeff["static_extra_nm"] * stribeck + coeff["sliding_nm"])


def lugre_feature_for_point(
    params: LugreParams,
    beta: np.ndarray,
    constants: dict[str, float],
    vf_mps: float,
    yaw_rate: float,
    total_requested_opposing_nm: float,
    cold_start_s: float | None = None,
) -> float:
    longitudinal = constants["drive_wheel_longitudinal_offset_m"]
    transition = math.hypot(params.rel_weight * longitudinal * abs(yaw_rate), abs(vf_mps))
    activation = 1.0 - math.exp(-((smooth_positive(total_requested_opposing_nm) / params.req_activation_nm) ** 2))
    stribeck = math.exp(-((transition / params.stribeck_speed_mps) ** 2))
    relief = 1.0 / (1.0 + (transition / params.speed_fade_mps) ** 2)
    q_eq = activation / (1.0 + params.tau_fill_s * transition / params.bristle_slip_distance_m)
    if cold_start_s is None:
        q = q_eq
    else:
        rate = (1.0 / params.tau_fill_s) + (transition / params.bristle_slip_distance_m)
        q = q_eq * (1.0 - math.exp(-rate * max(cold_start_s, 0.0)))
    features = np.array([q * relief * stribeck, q * relief, q * relief * longitudinal * abs(yaw_rate)])
    return float(max(0.0, np.dot(beta, features)))


def lugre_extra_fixedpoint(
    params: LugreParams,
    beta: np.ndarray,
    constants: dict[str, float],
    vf_mps: float,
    yaw_rate: float,
    cold_start_s: float | None = None,
) -> float:
    base = baseline_opposing_yaw_torque(constants, yaw_rate)
    extra = 0.0
    for _ in range(80):
        next_extra = lugre_feature_for_point(
            params,
            beta,
            constants,
            vf_mps,
            yaw_rate,
            base + extra,
            cold_start_s=cold_start_s,
        )
        if abs(next_extra - extra) < 1.0e-13:
            return next_extra
        extra = next_extra
    return extra


def beta_scaled_to_gate(
    params: LugreParams,
    beta: np.ndarray,
    constants: dict[str, float],
    target_extra_nm: float,
) -> tuple[np.ndarray, float, float]:
    current = lugre_extra_fixedpoint(params, beta, constants, 0.0, 1.0)
    if current <= 1.0e-12:
        return beta, 1.0, current
    lo = 0.0
    hi = max(1.0, target_extra_nm / current * 2.0)
    for _ in range(80):
        if lugre_extra_fixedpoint(params, beta * hi, constants, 0.0, 1.0) >= target_extra_nm:
            break
        hi *= 2.0
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        value = lugre_extra_fixedpoint(params, beta * mid, constants, 0.0, 1.0)
        if value < target_extra_nm:
            lo = mid
        else:
            hi = mid
    scaled = beta * hi
    return scaled, hi, lugre_extra_fixedpoint(params, scaled, constants, 0.0, 1.0)


def candidate_grid() -> list[LugreParams]:
    params: list[LugreParams] = []
    for tau_fill_s in [0.005, 0.020, 0.050, 0.120]:
        for slip_distance in [0.010, 0.030, 0.080]:
            for req_knee in [0.035, 0.050]:
                for stribeck_speed in [0.100, 0.140]:
                    params.append(
                        LugreParams(
                            req_activation_nm=req_knee,
                            stribeck_speed_mps=stribeck_speed,
                            speed_fade_mps=0.640,
                            rel_weight=0.750,
                            tau_fill_s=tau_fill_s,
                            bristle_slip_distance_m=slip_distance,
                        )
                    )
    return params


def score_fit(
    frame: pd.DataFrame,
    pred: np.ndarray,
    weights: np.ndarray,
) -> tuple[float, float, float, float]:
    baseline, corrected = residuals_after(frame, pred)
    weighted_train_rmse = math.sqrt(float(np.sum(weights * np.square(corrected)) / max(np.sum(weights), 1.0e-12)))
    primary = frame["dataset_split"].to_numpy() == "primary_open_floor_fit_authoritative"
    validation = frame["dataset_split"].isin(
        [
            "open_floor_fit_downweighted",
            "open_floor_validation_only",
            "diag_validation_only",
            "aux_downweighted_validation",
        ]
    ).to_numpy()
    primary_rmse = float(np.sqrt(np.mean(np.square(corrected[primary]))))
    validation_rmse = float(np.sqrt(np.mean(np.square(corrected[validation]))))
    selected_mask = frame["run_id"].isin(SELECTED_RUNS).to_numpy()
    selected_rmse = float(np.sqrt(np.mean(np.square(corrected[selected_mask]))))
    _ = baseline
    score = 0.65 * weighted_train_rmse + 0.25 * validation_rmse + 0.10 * selected_rmse
    return weighted_train_rmse, primary_rmse, validation_rmse, score


def fit_model_family(
    frame: pd.DataFrame,
    weights: np.ndarray,
    constants: dict[str, float],
    model: str,
    gate_extra_nm: float,
) -> tuple[FitResult, list[dict[str, object]], np.ndarray]:
    best: FitResult | None = None
    best_pred: np.ndarray | None = None
    tuning_rows: list[dict[str, object]] = []
    for params in candidate_grid():
        features, names = lugre_features(frame, params, model)
        base_1rad = baseline_opposing_yaw_torque(constants, 1.0)
        anchor_extra = gate_extra_nm
        anchor_total = base_1rad + anchor_extra
        anchor = np.array(
            [
                lugre_feature_for_point(params, np.array([1.0, 0.0, 0.0]), constants, 0.0, 1.0, anchor_total),
                lugre_feature_for_point(params, np.array([0.0, 1.0, 0.0]), constants, 0.0, 1.0, anchor_total),
                lugre_feature_for_point(params, np.array([0.0, 0.0, 1.0]), constants, 0.0, 1.0, anchor_total),
            ]
        )
        beta = fit_training_target(frame, features, weights, (anchor, anchor_extra))
        beta, scale, fixedpoint = beta_scaled_to_gate(params, beta, constants, gate_extra_nm)
        pred = np.maximum(0.0, features @ beta)
        train_rmse, primary_rmse, validation_rmse, selected_score = score_fit(frame, pred, weights)
        row = {
            "model": model,
            "req_activation_nm": params.req_activation_nm,
            "stribeck_speed_mps": params.stribeck_speed_mps,
            "speed_fade_mps": params.speed_fade_mps,
            "rel_weight": params.rel_weight,
            "tau_fill_s": params.tau_fill_s,
            "bristle_slip_distance_m": params.bristle_slip_distance_m,
            "beta_bristle_static_nm": beta[0],
            "beta_bristle_sliding_nm": beta[1],
            "beta_bristle_viscous_nm_per_mps": beta[2],
            "beta_scale_to_gate": scale,
            "fixedpoint_extra_nm": fixedpoint,
            "weighted_train_rmse_nm": train_rmse,
            "primary_corrected_rmse_nm": primary_rmse,
            "validation_corrected_rmse_nm": validation_rmse,
            "objective_score_nm": selected_score,
        }
        tuning_rows.append(row)
        result = FitResult(
            model=model,
            params=params,
            beta=beta,
            feature_names=names,
            fixedpoint_extra_nm=fixedpoint,
            beta_scale_to_gate=scale,
            weighted_train_rmse_nm=train_rmse,
            primary_rmse_nm=primary_rmse,
            validation_rmse_nm=validation_rmse,
            selected_score_nm=selected_score,
        )
        if best is None or selected_score < best.selected_score_nm:
            best = result
            best_pred = pred
    if best is None or best_pred is None:
        raise RuntimeError(f"no {model} fit produced")
    tuning_rows.sort(key=lambda row: float(row["objective_score_nm"]))
    return best, tuning_rows, best_pred


def torque_from_command(command: float, wheel_speed_radps: float, constants: dict[str, float]) -> float:
    resistance = constants["drive_resistance_ohms"]
    speed_constant = constants["speed_constant_radps_per_volt"]
    torque_constant = constants["torque_constant_nm_per_a"]
    gear_ratio = constants["gear_ratio"]
    battery = constants["drive_voltage_v"]
    no_load = constants["no_load_current_a"]
    current = (command * battery / resistance) - ((wheel_speed_radps * (gear_ratio / speed_constant)) / resistance)
    armature_sign = sign(current)
    wheel_sign = sign(wheel_speed_radps)
    no_load_sign = armature_sign if armature_sign else wheel_sign
    load_current = current - no_load_sign * no_load
    if no_load_sign > 0.0 and load_current < 0.0:
        load_current = 0.0
    elif no_load_sign < 0.0 and load_current > 0.0:
        load_current = 0.0
    return torque_constant * gear_ratio * load_current


def command_from_torque(command_torque: float, wheel_speed_radps: float, constants: dict[str, float]) -> float:
    resistance = constants["drive_resistance_ohms"]
    speed_constant = constants["speed_constant_radps_per_volt"]
    torque_constant = constants["torque_constant_nm_per_a"]
    gear_ratio = constants["gear_ratio"]
    battery = constants["drive_voltage_v"]
    no_load = constants["no_load_current_a"]
    motor_torque = command_torque / gear_ratio
    torque_sign = sign(motor_torque)
    wheel_sign = sign(wheel_speed_radps)
    no_load_sign = torque_sign if torque_sign else wheel_sign
    current = (motor_torque / torque_constant) + no_load_sign * no_load
    back_emf = wheel_speed_radps * (gear_ratio / speed_constant)
    return ((current * resistance) + back_emf) / battery


def static_launch_torque(constants: dict[str, float]) -> float:
    return max(0.0, torque_from_command(constants["static_launch_command"], 0.0, constants))


def signed_direction(preferred: float, fallback: float) -> float:
    preferred_sign = sign(preferred)
    return preferred_sign if preferred_sign != 0.0 else sign(fallback)


def wheel_speeds(vf_mps: float, yaw_rate: float, constants: dict[str, float]) -> tuple[float, float, float, float]:
    half_track = 0.5 * constants["track_width_m"]
    radius = constants["wheel_radius_m"]
    left_surface = vf_mps + half_track * yaw_rate
    right_surface = vf_mps - half_track * yaw_rate
    return left_surface, right_surface, left_surface / radius, right_surface / radius


def command_torque_for_applied(applied_torque: float, wheel_speed_radps: float, constants: dict[str, float]) -> tuple[float, float]:
    surface_speed = constants["wheel_radius_m"] * wheel_speed_radps
    ratio = abs(surface_speed) / constants["static_friction_max_speed_mps"]
    launch = static_launch_torque(constants) * math.exp(-(ratio * ratio))
    launch_dir = signed_direction(applied_torque, wheel_speed_radps)
    loss_dir = signed_direction(wheel_speed_radps, applied_torque)
    rolling = constants["rolling_friction_torque_nm"] * loss_dir
    command_torque = applied_torque
    if launch_dir != 0.0:
        command_torque += launch_dir * launch + rolling
    return command_torque, launch


def motor_commands_for_opposing_torque(
    opposing_yaw_torque: float,
    constants: dict[str, float],
    vf_mps: float,
    yaw_rate: float,
) -> dict[str, float]:
    radius = constants["wheel_radius_m"]
    track = constants["track_width_m"]
    left_surface, right_surface, left_speed, right_speed = wheel_speeds(vf_mps, yaw_rate, constants)
    applied_bank = opposing_yaw_torque * radius / track
    left_torque, left_launch = command_torque_for_applied(applied_bank, left_speed, constants)
    right_torque, right_launch = command_torque_for_applied(-applied_bank, right_speed, constants)
    left_cmd = command_from_torque(left_torque, left_speed, constants)
    right_cmd = command_from_torque(right_torque, right_speed, constants)
    return {
        "required_applied_bank_torque_nm": applied_bank,
        "left_command": left_cmd,
        "right_command": right_cmd,
        "lr_delta_command": left_cmd - right_cmd,
        "left_surface_mps": left_surface,
        "right_surface_mps": right_surface,
        "left_command_torque_nm": left_torque,
        "right_command_torque_nm": right_torque,
        "left_launch_torque_nm": left_launch,
        "right_launch_torque_nm": right_launch,
    }


def contact_utilization(constants: dict[str, float], yaw_rate: float) -> float:
    longitudinal = constants["drive_wheel_longitudinal_offset_m"]
    front_force = abs(constants["front_right_contact_force_gain_n_per_mps"] * longitudinal * yaw_rate)
    rear_force = abs(constants["rear_right_contact_force_gain_n_per_mps"] * longitudinal * yaw_rate)
    force_limit = constants["mass_kg"] * constants["sustained_lateral_accel_mps2"] / 4.0
    return max(front_force, rear_force) / force_limit


def limiter_activity(constants: dict[str, float], yaw_rate: float) -> float:
    return max(0.0, contact_utilization(constants, yaw_rate) - 1.0)


def c_extra_fixedpoint(base_opposing: float, constants: dict[str, float], vf_mps: float, yaw_rate: float) -> float:
    rows = read_coeff_rows(C_COEFFS)
    coeff_rows = [row for row in rows if row["candidate"] == "saturation_aware_surface"]
    coeffs = {row["feature"]: float(row["standardized_coefficient_nm"]) for row in coeff_rows}
    scales = {row["feature"]: float(row["feature_scale"]) for row in coeff_rows}
    vrel_knee = float(coeff_rows[0]["vrel_knee_mps"])
    fwd_knee = float(coeff_rows[0]["fwd_knee_mps"])
    longitudinal = constants["drive_wheel_longitudinal_offset_m"]
    v_rel = longitudinal * abs(yaw_rate)
    util = min(max(contact_utilization(constants, yaw_rate), 0.0), 5.0)
    limiter = min(max(limiter_activity(constants, yaw_rate), 0.0), 5.0)
    util_smooth = util / (1.0 + util)
    limiter_smooth = limiter / (1.0 + limiter)
    low_rel = smooth_gate(v_rel, vrel_knee)
    high_forward = 1.0 - smooth_gate(vf_mps, fwd_knee)
    right_front = 2.0 * (longitudinal * longitudinal) * abs(yaw_rate)
    right_rear = 2.0 * (longitudinal * longitudinal) * abs(yaw_rate)

    def feature_value(feature: str, extra: float) -> float:
        total_req = base_opposing + extra
        force_moment = extra
        values = {
            "gain_front_right_basis": right_front,
            "gain_rear_right_basis": right_rear,
            "gain_left_long_basis": 0.0,
            "gain_right_long_basis": 0.0,
            "req_moment_opposes_yaw_nm": -total_req,
            "force_moment_opposes_yaw_nm": -force_moment,
            "force_gap_opposes_yaw_nm": -base_opposing,
            "req_abs_contact_moment_nm": abs(total_req),
            "force_abs_contact_moment_nm": abs(force_moment),
        }
        base, suffix = feature.split("__", 1)
        x = values.get(base, 0.0)
        if suffix == "base":
            return x
        if suffix == "low_rel":
            return x * low_rel
        if suffix == "high_forward":
            return x * high_forward
        if suffix == "util":
            return x * util_smooth
        if suffix == "limiter":
            return x * limiter_smooth
        if suffix == "limiter_signed":
            return x * limiter_smooth
        if suffix == "load_delta":
            return 0.0
        return 0.0

    extra = 0.0
    for _ in range(60):
        predicted = 0.0
        for feature, beta in coeffs.items():
            scale = max(scales.get(feature, 1.0), 1.0e-12)
            value = feature_value(feature, extra)
            value = min(max(value, -8.0 * scale), 8.0 * scale)
            predicted += beta * (value / scale)
        if abs(predicted - extra) < 1.0e-13:
            return float(predicted)
        extra = float(predicted)
    return float(extra)


def vf_grid() -> list[float]:
    return [round(i * 0.15 / 5.0, 9) for i in range(6)]


def yaw_grid() -> list[float]:
    return [round(0.2 + i * (6.0 - 0.2) / 9.0, 9) for i in range(10)]


def make_grid_rows(constants: dict[str, float], state_fit: FitResult, memoryless_fit: FitResult) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for vf_mps in vf_grid():
        for yaw_rate in yaw_grid():
            base = baseline_opposing_yaw_torque(constants, yaw_rate)
            variants = [
                ("Baseline", 0.0, "current PlantModel right-contact scrub approximation"),
                ("B_stribeck", b_extra_fixedpoint(base, constants, vf_mps, yaw_rate), "prior B request-activated Stribeck scrub"),
                ("C_combined_slip", c_extra_fixedpoint(base, constants, vf_mps, yaw_rate), "prior C combined-slip surface"),
                (
                    "LuGre_state_steady",
                    lugre_extra_fixedpoint(state_fit.params, state_fit.beta, constants, vf_mps, yaw_rate),
                    "round2 one-state steady bristle equilibrium",
                ),
                (
                    "LuGre_state_cold20ms",
                    lugre_extra_fixedpoint(state_fit.params, state_fit.beta, constants, vf_mps, yaw_rate, cold_start_s=0.020),
                    "round2 one-state response after 20 ms from zero bristle state",
                ),
                (
                    "LuGre_memoryless_eq",
                    lugre_extra_fixedpoint(memoryless_fit.params, memoryless_fit.beta, constants, vf_mps, yaw_rate),
                    "round2 algebraic equilibrium approximation",
                ),
            ]
            util = contact_utilization(constants, yaw_rate)
            limiter = limiter_activity(constants, yaw_rate)
            for name, extra, caveat in variants:
                total = base + extra
                cmd = motor_commands_for_opposing_torque(total, constants, vf_mps, yaw_rate)
                max_abs_cmd = max(abs(cmd["left_command"]), abs(cmd["right_command"]))
                rows.append(
                    {
                        "vf_mps": vf_mps,
                        "yaw_rate_radps": yaw_rate,
                        "variant": name,
                        "baseline_opposing_yaw_torque_nm": base,
                        "extra_opposing_yaw_torque_nm": extra,
                        "total_opposing_yaw_torque_nm": total,
                        **cmd,
                        "max_abs_command": max_abs_cmd,
                        "command_outside_unit": max_abs_cmd > 1.0,
                        "contact_utilization_raw": util,
                        "limiter_activity_proxy": limiter,
                        "contact_projection_sensitive": util > 1.0,
                        "low_speed_launch_sensitive": min(abs(cmd["left_surface_mps"]), abs(cmd["right_surface_mps"]))
                        < 2.0 * constants["static_friction_max_speed_mps"],
                        "caveat": caveat,
                    }
                )
    return rows


def grid_pivot_lines(rows: list[dict[str, object]]) -> list[str]:
    variants = [
        "Baseline",
        "B_stribeck",
        "C_combined_slip",
        "LuGre_state_steady",
        "LuGre_state_cold20ms",
        "LuGre_memoryless_eq",
    ]
    by_key = {(row["variant"], row["vf_mps"], row["yaw_rate_radps"]): row for row in rows}
    lines = [
        "# Round2 L/R Command Delta Grid",
        "",
        "Values are `left_command - right_command` for positive clockwise yaw. Yaw-rate columns are rad/s; `Vf` rows are m/s.",
        "",
    ]
    for variant in variants:
        lines.append(f"## {variant}")
        lines.append("")
        lines.append("| Vf \\ yaw | " + " | ".join(f"{yaw:.3g}" for yaw in yaw_grid()) + " |")
        lines.append("| ---: | " + " | ".join("---:" for _ in yaw_grid()) + " |")
        for vf in vf_grid():
            values = [f"{float(by_key[(variant, vf, yaw)]['lr_delta_command']):.3f}" for yaw in yaw_grid()]
            lines.append(f"| {vf:.3f} | " + " | ".join(values) + " |")
        lines.append("")
    lines.append("## Flags")
    lines.append("")
    lines.append(f"- Rows with `|cmd| > 1`: {sum(1 for row in rows if row['command_outside_unit'])} of {len(rows)}.")
    lines.append(
        f"- Rows where raw contact utilization exceeds 1.0: {sum(1 for row in rows if row['contact_projection_sensitive'])} of {len(rows)}."
    )
    lines.append(
        f"- Rows near a wheel zero-crossing where launch friction matters: {sum(1 for row in rows if row['low_speed_launch_sensitive'])} of {len(rows)}."
    )
    lines.append("")
    lines.append("Full per-point commands, torques, and flags are in `lr_delta_grid.csv`.")
    return lines


def read_c_split_metrics() -> dict[str, float]:
    if not C_SPLIT.exists():
        return {}
    table = pd.read_csv(C_SPLIT)
    return {str(row["group"]): float(row["corrected_rmse_nm"]) for _, row in table.iterrows()}


def read_c_selected_metrics() -> dict[str, float]:
    if not C_SELECTED.exists():
        return {}
    table = pd.read_csv(C_SELECTED)
    return {str(row["run_id"]): float(row["corrected_rmse_nm"]) for _, row in table.iterrows() if int(row.get("present_in_primary_feature_input", 1)) != 0}


def comparison_split_table(
    frame: pd.DataFrame,
    b_pred: np.ndarray,
    state_pred: np.ndarray,
    memoryless_pred: np.ndarray,
) -> list[dict[str, object]]:
    b_rows = {row["group"]: row for row in group_metrics(frame, b_pred, "B_stribeck")}
    state_rows = {row["group"]: row for row in group_metrics(frame, state_pred, "LuGre_state")}
    mem_rows = {row["group"]: row for row in group_metrics(frame, memoryless_pred, "LuGre_memoryless")}
    c_rows = read_c_split_metrics()
    rows: list[dict[str, object]] = []
    for group in SPLIT_ORDER:
        if group not in state_rows:
            continue
        state_rmse = float(state_rows[group]["corrected_rmse_nm"])
        mem_rmse = float(mem_rows[group]["corrected_rmse_nm"])
        b_rmse = float(b_rows[group]["corrected_rmse_nm"]) if group in b_rows else math.nan
        c_rmse = c_rows.get(group, math.nan)
        baseline = float(state_rows[group]["baseline_rmse_nm"])
        rows.append(
            {
                "group": group,
                "count": state_rows[group]["count"],
                "baseline_rmse_nm": baseline,
                "b_stribeck_rmse_nm": b_rmse,
                "c_combined_slip_rmse_nm": c_rmse,
                "lugre_state_rmse_nm": state_rmse,
                "lugre_memoryless_rmse_nm": mem_rmse,
                "lugre_state_delta_vs_b_pct": 100.0 * (state_rmse - b_rmse) / b_rmse if b_rmse > 0.0 else math.nan,
                "lugre_state_delta_vs_c_pct": 100.0 * (state_rmse - c_rmse) / c_rmse if c_rmse > 0.0 else math.nan,
                "lugre_memoryless_delta_vs_b_pct": 100.0 * (mem_rmse - b_rmse) / b_rmse if b_rmse > 0.0 else math.nan,
                "lugre_memoryless_delta_vs_c_pct": 100.0 * (mem_rmse - c_rmse) / c_rmse if c_rmse > 0.0 else math.nan,
            }
        )
    return rows


def comparison_selected_table(
    frame: pd.DataFrame,
    b_pred: np.ndarray,
    state_pred: np.ndarray,
    memoryless_pred: np.ndarray,
) -> list[dict[str, object]]:
    b_rows = {row["run_id"]: row for row in selected_metrics(frame, b_pred, "B_stribeck")}
    state_rows = {row["run_id"]: row for row in selected_metrics(frame, state_pred, "LuGre_state")}
    mem_rows = {row["run_id"]: row for row in selected_metrics(frame, memoryless_pred, "LuGre_memoryless")}
    c_rows = read_c_selected_metrics()
    rows: list[dict[str, object]] = []
    for run_id in SELECTED_RUNS:
        state = state_rows[run_id]
        if not state.get("present", False):
            rows.append({"run_id": run_id, "present": False})
            continue
        b_rmse = float(b_rows[run_id]["corrected_rmse_nm"])
        c_rmse = c_rows.get(run_id, math.nan)
        state_rmse = float(state["corrected_rmse_nm"])
        mem_rmse = float(mem_rows[run_id]["corrected_rmse_nm"])
        rows.append(
            {
                "run_id": run_id,
                "present": True,
                "dataset_split": state["dataset_split"],
                "count": state["count"],
                "baseline_rmse_nm": state["baseline_rmse_nm"],
                "b_stribeck_rmse_nm": b_rmse,
                "c_combined_slip_rmse_nm": c_rmse,
                "lugre_state_rmse_nm": state_rmse,
                "lugre_memoryless_rmse_nm": mem_rmse,
                "lugre_state_delta_vs_b_pct": 100.0 * (state_rmse - b_rmse) / b_rmse if b_rmse > 0.0 else math.nan,
                "lugre_state_delta_vs_c_pct": 100.0 * (state_rmse - c_rmse) / c_rmse if c_rmse > 0.0 else math.nan,
                "lugre_memoryless_delta_vs_b_pct": 100.0 * (mem_rmse - b_rmse) / b_rmse if b_rmse > 0.0 else math.nan,
                "lugre_memoryless_delta_vs_c_pct": 100.0 * (mem_rmse - c_rmse) / c_rmse if c_rmse > 0.0 else math.nan,
            }
        )
    return rows


def risk_metrics(frame: pd.DataFrame, state_pred: np.ndarray, memoryless_pred: np.ndarray) -> list[dict[str, object]]:
    groups = {
        "straightish_abs_yaw_lt_0p05": frame["abs_yaw_rate_radps"] < 0.05,
        "straightish_forward_abs_yaw_lt_0p05_vf_ge_0p05": (frame["abs_yaw_rate_radps"] < 0.05)
        & (frame["abs_forward_velocity_mps"] >= 0.05),
        "low_speed_yaw_vf_lt_0p05_yaw_ge_0p2": (frame["abs_forward_velocity_mps"] < 0.05)
        & (frame["abs_yaw_rate_radps"] >= 0.2),
        "high_forward_vf_ge_0p5": frame["abs_forward_velocity_mps"] >= 0.5,
        "limiter_active": frame["max_force_limiter_activity"].fillna(0.0) > 0.0,
    }
    rows: list[dict[str, object]] = []
    index = frame.index.to_numpy()
    for name, mask_series in groups.items():
        mask = mask_series.to_numpy()
        if not np.any(mask):
            continue
        subset_index = frame.loc[mask].index.to_numpy()
        state = metric_values(frame.loc[mask], state_pred[np.isin(index, subset_index)])
        mem = metric_values(frame.loc[mask], memoryless_pred[np.isin(index, subset_index)])
        rows.append(
            {
                "group": name,
                "count": state["count"],
                "baseline_rmse_nm": state["baseline_rmse_nm"],
                "lugre_state_rmse_nm": state["corrected_rmse_nm"],
                "lugre_memoryless_rmse_nm": mem["corrected_rmse_nm"],
                "baseline_median_abs_nm": state["baseline_median_abs_nm"],
                "lugre_state_median_abs_nm": state["corrected_median_abs_nm"],
                "lugre_memoryless_median_abs_nm": mem["corrected_median_abs_nm"],
            }
        )
    return rows


def coefficient_rows(result: FitResult) -> list[dict[str, object]]:
    rows = [
        {"model": result.model, "parameter": "req_activation_nm", "value": result.params.req_activation_nm, "unit": "Nm"},
        {"model": result.model, "parameter": "stribeck_speed_mps", "value": result.params.stribeck_speed_mps, "unit": "m/s"},
        {"model": result.model, "parameter": "speed_fade_mps", "value": result.params.speed_fade_mps, "unit": "m/s"},
        {"model": result.model, "parameter": "rel_weight", "value": result.params.rel_weight, "unit": "dimensionless"},
        {"model": result.model, "parameter": "tau_fill_s", "value": result.params.tau_fill_s, "unit": "s"},
        {
            "model": result.model,
            "parameter": "bristle_slip_distance_m",
            "value": result.params.bristle_slip_distance_m,
            "unit": "m",
        },
    ]
    for name, beta in zip(result.feature_names, result.beta):
        unit = "Nm"
        if "viscous" in name:
            unit = "Nm per (m/s)"
        rows.append({"model": result.model, "parameter": name, "value": float(beta), "unit": unit})
    rows.extend(
        [
            {
                "model": result.model,
                "parameter": "fixedpoint_extra_at_1radps_in_place_nm",
                "value": result.fixedpoint_extra_nm,
                "unit": "Nm",
            },
            {"model": result.model, "parameter": "beta_scale_to_gate", "value": result.beta_scale_to_gate, "unit": "dimensionless"},
            {"model": result.model, "parameter": "weighted_train_rmse_nm", "value": result.weighted_train_rmse_nm, "unit": "Nm"},
            {"model": result.model, "parameter": "primary_rmse_nm", "value": result.primary_rmse_nm, "unit": "Nm"},
            {"model": result.model, "parameter": "validation_rmse_nm", "value": result.validation_rmse_nm, "unit": "Nm"},
        ]
    )
    return rows


def report_table(rows: list[dict[str, object]], columns: list[str], limit: int | None = None) -> list[str]:
    shown = rows[:limit] if limit else rows
    out = ["| " + " | ".join(columns) + " |", "| " + " | ".join("---" for _ in columns) + " |"]
    for row in shown:
        out.append("| " + " | ".join(fmt(row.get(col, "")) for col in columns) + " |")
    return out


def write_report(
    constants: dict[str, float],
    state_fit: FitResult,
    memoryless_fit: FitResult,
    split_comparison: list[dict[str, object]],
    selected_comparison: list[dict[str, object]],
    in_place_rows: list[dict[str, object]],
    grid_rows: list[dict[str, object]],
    risks: list[dict[str, object]],
) -> None:
    state_cmd = next(row for row in in_place_rows if row["variant"] == "LuGre_state_steady")
    mem_cmd = next(row for row in in_place_rows if row["variant"] == "LuGre_memoryless_eq")
    state_gate_pass = bool(state_cmd["passes_abs_command_gate"])
    mem_gate_pass = bool(mem_cmd["passes_abs_command_gate"])
    lines: list[str] = [
        "# Round2 State-Minimal LuGre Yaw Residual Fit",
        "",
        "Analysis-only output. Production code, build metadata, and tests were not edited.",
        "",
        "## Model Family",
        "",
        "The dynamic candidate uses one scalar bristle-fill state `q` per yaw/contact aggregate:",
        "",
        "`v_c = sqrt((w_rel * vbar_rel)^2 + |Vf|^2)`",
        "",
        "`A = 1 - exp(-(positive(M_req) / M_act)^2)`",
        "",
        "`q_eq = A / (1 + tau_fill * v_c / x_slip)`",
        "",
        "`dq/dt = (A - q) / tau_fill - (v_c / x_slip) * q`",
        "",
        "`R = 1 / (1 + (v_c / v_fade)^2)`",
        "",
        "`S = exp(-(v_c / v_s)^2)`",
        "",
        "`M_extra = q * R * (K_static * S + K_slide + K_visc * vbar_yaw)`",
        "",
        "The additive yaw-torque correction applied to the residual sign convention is `M_add = -sign(yaw) * M_extra`.",
        "",
        "The memoryless approximation replaces the replayed state with the algebraic equilibrium `q = q_eq`.",
        "",
        "## New Mechanism",
        "",
        "This is not another low-order A/C/D-style residual surface with a yaw-launch constraint. The new mechanism is the "
        "single bristle-fill state `q`: it is integrated through time, fills under requested yaw moment, is depleted by "
        "moving contact slip, and resets on yaw direction reversal. Two rows with identical instantaneous `Vf`, yaw rate, "
        "and contact features can therefore predict different breakaway resistance if their recent bristle history differs.",
        "",
        "The fitted coefficients only scale the static/Stribeck, sliding, and viscous terms after that state evolution. "
        "A ridge/surface fit has no equivalent stored pre-sliding deflection and cannot represent launch hysteresis or "
        "direction-reversal breakaway without adding this state or reducing back to the algebraic approximation.",
        "",
        "## Command-Invariance Reassessment",
        "",
        "New hard rule: traction/resistance must not differ for the same physical contact state and tire/contact forces "
        "merely because command values differ. Command may determine actuator torque input, but it must not be an "
        "independent traction-model selector or hidden state machine.",
        "",
        "Under that rule, the fitted activation `A = 1 - exp(-(positive(M_req) / M_act)^2)` is production-rejected when "
        "`M_req` is a requested/pre-projection command-derived yaw moment rather than an actual physical contact-force "
        "state. The hard-gated fit remains useful as a diagnostic magnitude target, but the fitted `q` fill law and the "
        "memoryless `q_eq` approximation are not production-eligible in their current request-driven form.",
        "",
        "Production revision: drive bristle state only from contact slip and actual tire/contact force history. A "
        "per-contact form is preferable:",
        "",
        "`dot(z_i) = v_t_i - (|v_t_i| / g_i(|v_t_i|, N_i)) * z_i`",
        "",
        "`g_i = (F_c_i + (F_s_i - F_c_i) * exp(-( |v_t_i| / v_s )^2)) / sigma0_i`",
        "",
        "`F_t_i = clamp(sigma0_i * z_i + sigma1_i * dot(z_i) + sigma2_i * v_t_i, -mu_i * N_i, +mu_i * N_i)`",
        "",
        "`M_yaw = sum_i cross(r_i, F_t_i)`",
        "",
        "In that revision, command affects the motor torque and therefore the solved physical contact forces, but the "
        "friction law sees only `v_t_i`, `N_i`, `z_i`, and actual tangential contact force/load state. For a memoryless "
        "fallback, use the steady sliding/Stribeck force from `v_t_i` and `N_i`, or a static-capacity projection based "
        "on actual tangential force demand, never on requested command labels or pre-projection command moments.",
        "",
        "## Hard Gate",
        "",
        "Reference condition: `Vf=0`, `Vr=0`, `yaw_rate=+1 rad/s`. Acceptance threshold: `|left_command| >= 0.6` "
        "and `|right_command| >= 0.6`; measured/calculated reference is approximately `+0.646/-0.646`.",
        "",
        f"Prior B extra opposing torque at +1 rad/s in-place is `{state_fit.fixedpoint_extra_nm:.6f} Nm` by construction. "
        f"The state model command is left `{float(state_cmd['left_command']):.6f}`, right `{float(state_cmd['right_command']):.6f}`. "
        f"The memoryless command is left `{float(mem_cmd['left_command']):.6f}`, right `{float(mem_cmd['right_command']):.6f}`.",
        "",
        f"Gate result: state model `{'PASS' if state_gate_pass else 'FAIL'}`; memoryless approximation "
        f"`{'PASS' if mem_gate_pass else 'FAIL'}`.",
        "",
        "Production eligibility result under the command-invariance rule: `FAIL` for the fitted request-driven state and "
        "memoryless forms; `PASS` only as a diagnostic magnitude check. A production candidate must be refit/replayed "
        "with the force/slip-driven law above.",
        "",
        "## Coefficients",
        "",
    ]
    coeff_rows = coefficient_rows(state_fit) + coefficient_rows(memoryless_fit)
    lines.extend(report_table(coeff_rows, ["model", "parameter", "value", "unit"]))
    lines.extend(
        [
            "",
            "## +1 Rad/s In-Place Command",
            "",
        ]
    )
    lines.extend(
        report_table(
            in_place_rows,
            [
                "variant",
                "extra_opposing_yaw_torque_nm",
                "total_opposing_yaw_torque_nm",
                "required_applied_bank_torque_nm",
                "left_command",
                "right_command",
                "lr_delta_command",
                "passes_abs_command_gate",
            ],
        )
    )
    lines.extend(
        [
            "",
            "## Split RMSE Versus B/C",
            "",
        ]
    )
    lines.extend(
        report_table(
            split_comparison,
            [
                "group",
                "baseline_rmse_nm",
                "b_stribeck_rmse_nm",
                "c_combined_slip_rmse_nm",
                "lugre_state_rmse_nm",
                "lugre_memoryless_rmse_nm",
                "lugre_state_delta_vs_b_pct",
                "lugre_state_delta_vs_c_pct",
            ],
        )
    )
    lines.extend(
        [
            "",
            "## Selected-Log RMSE Versus B/C",
            "",
        ]
    )
    lines.extend(
        report_table(
            selected_comparison,
            [
                "run_id",
                "dataset_split",
                "baseline_rmse_nm",
                "b_stribeck_rmse_nm",
                "c_combined_slip_rmse_nm",
                "lugre_state_rmse_nm",
                "lugre_memoryless_rmse_nm",
                "lugre_state_delta_vs_b_pct",
                "lugre_state_delta_vs_c_pct",
            ],
        )
    )
    lines.extend(
        [
            "",
            "## 6x10 Vf/Yaw Grid Summary",
            "",
            "The full grid is in `lr_delta_grid.csv`; the pivot view is in `lr_delta_pivot.md`. Summary:",
            "",
        ]
    )
    variants = sorted({str(row["variant"]) for row in grid_rows})
    summary_rows = []
    for variant in variants:
        subset = [row for row in grid_rows if row["variant"] == variant]
        deltas = [float(row["lr_delta_command"]) for row in subset]
        summary_rows.append(
            {
                "variant": variant,
                "cells": len(subset),
                "min_lr_delta_command": min(deltas),
                "max_lr_delta_command": max(deltas),
                "cells_abs_cmd_gt_1": sum(1 for row in subset if row["command_outside_unit"]),
                "cells_contact_util_gt_1": sum(1 for row in subset if row["contact_projection_sensitive"]),
            }
        )
    lines.extend(
        report_table(
            summary_rows,
            [
                "variant",
                "cells",
                "min_lr_delta_command",
                "max_lr_delta_command",
                "cells_abs_cmd_gt_1",
                "cells_contact_util_gt_1",
            ],
        )
    )
    lines.extend(
        [
            "",
            "## Risk Checks",
            "",
        ]
    )
    lines.extend(
        report_table(
            risks,
            [
                "group",
                "count",
                "baseline_rmse_nm",
                "lugre_state_rmse_nm",
                "lugre_memoryless_rmse_nm",
                "baseline_median_abs_nm",
                "lugre_state_median_abs_nm",
                "lugre_memoryless_median_abs_nm",
            ],
        )
    )
    lines.extend(
        [
            "",
            "## Production Implications",
            "",
            "- The request-driven fitted `q` law is rejected for production because it can change traction/resistance for identical physical contact state and tire/contact forces when only command/request changes.",
            "- A true production internal state still belongs in `PlantModel`, but it must be driven by contact slip, normal load, and actual tangential force history. If estimator prediction uses it, the estimator needs the same state or a deterministic mirror advanced with the same physical inputs.",
            "- The state must be reset or strongly decayed on yaw direction reversal, physical lift/discontinuous vehicle state, and any `LoopController` session boundary where continuity is not guaranteed.",
            "- The memoryless `q_eq` approximation from this fit is also production-rejected because its fill term is request-driven. A production memoryless fallback may use only steady sliding/Stribeck force from contact slip/load or a static-capacity projection based on actual tangential contact force demand.",
            "- The B-scale `+0.6427/-0.6427` command remains the minimum magnitude target for any force/slip-driven refit; broad RMSE must not be used to accept a lower-command C-like model.",
            "",
            "## Output Files",
            "",
            "- `fit_state_minimal_lugre.py`",
            "- `round2_state_minimal_lugre_report.md`",
            "- `candidate_tuning_scores.csv`",
            "- `lugre_coefficients.csv`",
            "- `lugre_split_metrics.csv`",
            "- `lugre_selected_log_metrics.csv`",
            "- `split_rmse_comparison.csv`",
            "- `selected_log_rmse_comparison.csv`",
            "- `in_place_1radps_command.csv`",
            "- `lr_delta_grid.csv`",
            "- `lr_delta_pivot.md`",
            "- `risk_metrics.csv`",
            "- `production_reassessment.csv`",
            "- `prediction_sample.csv`",
            "- `commands_run.txt`",
            "",
            "## Constants Read",
            "",
            f"- track width: {constants['track_width_m']:.12g} m",
            f"- drive wheel longitudinal offset: {constants['drive_wheel_longitudinal_offset_m']:.12g} m",
            f"- wheel radius: {constants['wheel_radius_m']:.12g} m",
        ]
    )
    (OUT / "round2_state_minimal_lugre_report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def in_place_command_rows(constants: dict[str, float], state_fit: FitResult, memoryless_fit: FitResult) -> list[dict[str, object]]:
    yaw_rate = 1.0
    vf_mps = 0.0
    base = baseline_opposing_yaw_torque(constants, yaw_rate)
    variants = [
        ("Current baseline", 0.0, "current PlantModel right-contact scrub approximation"),
        ("Variant B Stribeck scrub", b_extra_fixedpoint(base, constants, vf_mps, yaw_rate), "prior B hard-gate reference"),
        ("Variant C combined slip", c_extra_fixedpoint(base, constants, vf_mps, yaw_rate), "prior C lower-command reference"),
        (
            "LuGre_state_steady",
            lugre_extra_fixedpoint(state_fit.params, state_fit.beta, constants, vf_mps, yaw_rate),
            "round2 one-state steady bristle equilibrium",
        ),
        (
            "LuGre_state_cold20ms",
            lugre_extra_fixedpoint(state_fit.params, state_fit.beta, constants, vf_mps, yaw_rate, cold_start_s=0.020),
            "round2 one-state response after 20 ms from zero bristle state",
        ),
        (
            "LuGre_memoryless_eq",
            lugre_extra_fixedpoint(memoryless_fit.params, memoryless_fit.beta, constants, vf_mps, yaw_rate),
            "round2 algebraic equilibrium approximation",
        ),
    ]
    rows: list[dict[str, object]] = []
    gate_threshold = 0.6
    for variant, extra, caveat in variants:
        total = base + extra
        cmd = motor_commands_for_opposing_torque(total, constants, vf_mps, yaw_rate)
        gate_pass = abs(cmd["left_command"]) >= gate_threshold and abs(cmd["right_command"]) >= gate_threshold
        if variant.startswith("LuGre"):
            command_invariance = False
            production_status = "rejected_current_request_driven_form"
        elif variant == "Variant B Stribeck scrub":
            command_invariance = False
            production_status = "diagnostic_reference_only_if_request_activated"
        elif variant == "Variant C combined slip":
            command_invariance = None
            production_status = "rejected_low_command_gate"
        elif variant == "Current baseline":
            command_invariance = True
            production_status = "rejected_low_command_gate"
        else:
            command_invariance = None
            production_status = "not_assessed"
        rows.append(
            {
                "variant": variant,
                "extra_opposing_yaw_torque_nm": extra,
                "total_opposing_yaw_torque_nm": total,
                **cmd,
                "gate_threshold_abs_command": gate_threshold,
                "passes_abs_command_gate": gate_pass,
                "passes_command_invariance_rule": command_invariance,
                "production_status": production_status,
                "caveat": caveat,
            }
        )
    return rows


def production_reassessment_rows(in_place_rows: list[dict[str, object]]) -> list[dict[str, object]]:
    rows = []
    for row in in_place_rows:
        rows.append(
            {
                "variant": row["variant"],
                "passes_abs_command_gate": row["passes_abs_command_gate"],
                "passes_command_invariance_rule": row["passes_command_invariance_rule"],
                "production_status": row["production_status"],
                "reason": row["caveat"],
            }
        )
    rows.append(
        {
            "variant": "Revised force/slip-driven LuGre",
            "passes_abs_command_gate": "",
            "passes_command_invariance_rule": True,
            "production_status": "recommended_next_fit_not_yet_scored",
            "reason": "Drive bristle state from contact slip, normal load, and actual tangential contact force history; refit must still meet +0.6/-0.6 gate.",
        }
    )
    return rows


def prediction_sample(frame: pd.DataFrame, b_pred: np.ndarray, state_pred: np.ndarray, mem_pred: np.ndarray) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    counts: dict[str, int] = {}
    selected = set(SELECTED_RUNS)
    for idx, row in frame.iterrows():
        run_id = str(row["run_id"])
        if run_id not in selected:
            continue
        count = counts.get(run_id, 0)
        if count >= 20:
            continue
        baseline = float(row["residual_additive_yaw_torque_nm"])
        yaw_sign = float(row["yaw_sign"])
        rows.append(
            {
                "run_id": run_id,
                "dataset_split": row["dataset_split"],
                "physics_phase": row["physics_phase"],
                "row_index": int(row["row_index"]),
                "time_us": int(row["time_us"]),
                "forward_velocity_mps": row["forward_velocity_mps"],
                "yaw_rate_radps": row["yaw_rate_radps"],
                "vbar_rel_mps": row["vbar_rel_mps"],
                "patch_yaw_req_basis_nm": row["patch_yaw_req_basis_nm"],
                "residual_additive_yaw_torque_nm": baseline,
                "b_predicted_additive_yaw_torque_nm": -yaw_sign * float(b_pred[idx]),
                "lugre_state_predicted_additive_yaw_torque_nm": -yaw_sign * float(state_pred[idx]),
                "lugre_memoryless_predicted_additive_yaw_torque_nm": -yaw_sign * float(mem_pred[idx]),
            }
        )
        counts[run_id] = count + 1
    return rows


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    constants = read_constants()
    frame = load_frame()
    weights = training_weights(frame)
    base_1rad = baseline_opposing_yaw_torque(constants, 1.0)
    gate_extra = b_extra_fixedpoint(base_1rad, constants, 0.0, 1.0)

    state_fit, state_tuning, state_pred = fit_model_family(
        frame,
        weights,
        constants,
        "state_minimal_lugre",
        gate_extra,
    )
    memoryless_fit, memoryless_tuning, memoryless_pred = fit_model_family(
        frame,
        weights,
        constants,
        "memoryless_equilibrium",
        gate_extra,
    )
    b_pred = b_prediction(frame)

    tuning_rows = state_tuning + memoryless_tuning
    tuning_rows.sort(key=lambda row: (str(row["model"]), float(row["objective_score_nm"])))
    write_csv(OUT / "candidate_tuning_scores.csv", tuning_rows)
    write_csv(OUT / "lugre_coefficients.csv", coefficient_rows(state_fit) + coefficient_rows(memoryless_fit))

    lugre_metric_rows = []
    lugre_metric_rows.extend(group_metrics(frame, state_pred, "LuGre_state"))
    lugre_metric_rows.extend(group_metrics(frame, memoryless_pred, "LuGre_memoryless"))
    write_csv(OUT / "lugre_split_metrics.csv", lugre_metric_rows)
    selected_rows = []
    selected_rows.extend(selected_metrics(frame, state_pred, "LuGre_state"))
    selected_rows.extend(selected_metrics(frame, memoryless_pred, "LuGre_memoryless"))
    write_csv(OUT / "lugre_selected_log_metrics.csv", selected_rows)

    split_comparison = comparison_split_table(frame, b_pred, state_pred, memoryless_pred)
    selected_comparison = comparison_selected_table(frame, b_pred, state_pred, memoryless_pred)
    write_csv(OUT / "split_rmse_comparison.csv", split_comparison)
    write_csv(OUT / "selected_log_rmse_comparison.csv", selected_comparison)

    command_rows = in_place_command_rows(constants, state_fit, memoryless_fit)
    write_csv(OUT / "in_place_1radps_command.csv", command_rows)
    write_csv(OUT / "production_reassessment.csv", production_reassessment_rows(command_rows))

    grid_rows = make_grid_rows(constants, state_fit, memoryless_fit)
    write_csv(OUT / "lr_delta_grid.csv", grid_rows)
    (OUT / "lr_delta_pivot.md").write_text("\n".join(grid_pivot_lines(grid_rows)) + "\n", encoding="utf-8")

    risks = risk_metrics(frame, state_pred, memoryless_pred)
    write_csv(OUT / "risk_metrics.csv", risks)
    write_csv(OUT / "prediction_sample.csv", prediction_sample(frame, b_pred, state_pred, memoryless_pred))
    (OUT / "commands_run.txt").write_text(
        "& 'C:\\Users\\thene\\.cache\\codex-runtimes\\codex-primary-runtime\\dependencies\\python\\python.exe' "
        "codex_analysis\\yaw_model_variant_fits\\round2_state_minimal_lugre\\fit_state_minimal_lugre.py\n",
        encoding="utf-8",
    )
    write_report(constants, state_fit, memoryless_fit, split_comparison, selected_comparison, command_rows, grid_rows, risks)

    print(f"gate_extra_nm={gate_extra:.9f}")
    print(f"state_left_command={next(row for row in command_rows if row['variant'] == 'LuGre_state_steady')['left_command']:.9f}")
    print(f"memoryless_left_command={next(row for row in command_rows if row['variant'] == 'LuGre_memoryless_eq')['left_command']:.9f}")
    print((OUT / "round2_state_minimal_lugre_report.md").as_posix())


if __name__ == "__main__":
    main()
