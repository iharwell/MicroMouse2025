#!/usr/bin/env python3
"""Temporary four-contact-patch traction evaluation for Agent C.

Reads existing project analysis CSVs and writes temporary reports beside this
script. No production source or build metadata is modified.
"""

from __future__ import annotations

import csv
import json
import math
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np
import pandas as pd


ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent

if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from codex_analysis.yaw_model_variant_fits.regime_weighting import add_forward_accel_columns_to_frame


PRIMARY = ROOT / "codex_analysis" / "contact_continuum_yaw_identification" / "ablation" / "phase_classified_feature_sample.csv"
FEATURES = ROOT / "codex_analysis" / "contact_continuum_yaw_identification" / "features" / "contact_continuum_feature_sample.csv"
CONSTANTS = ROOT / "codex_analysis" / "contact_continuum_yaw_identification" / "features" / "plant_mirror_constants.csv"
DISCOVERED_LOGS = ROOT / "codex_analysis" / "yaw_torque_expanded_validation" / "discovered_logs.csv"

CONTACTS = ("fl", "fr", "rl", "rr")
VALID_SPLITS = {
    "primary_open_floor_fit_authoritative",
    "open_floor_fit_downweighted",
    "open_floor_validation_only",
    "diag_validation_only",
    "aux_downweighted_validation",
}


def read_constants() -> dict[str, float]:
    table = pd.read_csv(CONSTANTS)
    return {str(row["name"]): float(row["value"]) for _, row in table.iterrows()}


def load_frame() -> pd.DataFrame:
    primary_cols = [
        "run_id",
        "row_index",
        "dataset_split",
        "recommendation",
        "physics_phase",
        "gyro_derivative_spike",
        "hardware_saturation_evidence",
    ]
    feature_cols = [
        "run_id",
        "row_index",
        "time_us",
        "left_command",
        "right_command",
        "left_drive_force_n",
        "right_drive_force_n",
        "forward_velocity_mps",
        "yaw_rate_radps",
        "measured_yaw_accel_radps2",
        "observed_yaw_moment_nm",
        "model_yaw_moment_nm",
        "total_normal_load_n",
        "max_force_limiter_activity",
    ]
    for c in CONTACTS:
        feature_cols.extend(
            [
                f"{c}_v_rel_f_mps",
                f"{c}_v_rel_r_mps",
                f"{c}_force_f_n",
                f"{c}_force_r_n",
                f"{c}_normal_n",
            ]
        )
    primary = pd.read_csv(PRIMARY, usecols=primary_cols)
    features = pd.read_csv(FEATURES, usecols=feature_cols)
    frame = primary.merge(features, on=["run_id", "row_index"], how="inner")
    if DISCOVERED_LOGS.exists():
        inventory = pd.read_csv(DISCOVERED_LOGS)
        inventory["status"] = inventory["status"].astype(str)
        inventory["kind"] = inventory["kind"].astype(str)
        allowed_runs = set(
            inventory[
                inventory["status"].str.startswith("included")
                & inventory["kind"].ne("competition_fwc")
            ]["run_id"].astype(str)
        )
        frame = frame[frame["run_id"].astype(str).isin(allowed_runs)].copy()
    frame = frame[frame["dataset_split"].isin(VALID_SPLITS)].copy()
    frame = frame.replace([np.inf, -np.inf], np.nan)
    numeric_cols = [c for c in frame.columns if c not in {"run_id", "dataset_split", "recommendation", "physics_phase"}]
    for col in numeric_cols:
        frame[col] = pd.to_numeric(frame[col], errors="coerce")
    required = [
        "time_us",
        "forward_velocity_mps",
        "yaw_rate_radps",
        "measured_yaw_accel_radps2",
        "observed_yaw_moment_nm",
        "model_yaw_moment_nm",
        "left_drive_force_n",
        "right_drive_force_n",
    ]
    for c in CONTACTS:
        required.extend([f"{c}_v_rel_f_mps", f"{c}_v_rel_r_mps", f"{c}_normal_n"])
    frame = frame.dropna(subset=required).reset_index(drop=True)
    frame = add_forward_accel_columns_to_frame(frame)
    return frame.reset_index(drop=True)


def quantile_edges(values: np.ndarray, bins: int = 5) -> np.ndarray:
    finite = values[np.isfinite(values)]
    if len(finite) == 0:
        return np.array([-np.inf, np.inf], dtype=float)
    edges = np.quantile(finite, np.linspace(0.0, 1.0, bins + 1))
    edges[0] = -np.inf
    edges[-1] = np.inf
    unique = [edges[0]]
    for value in edges[1:]:
        if value > unique[-1]:
            unique.append(value)
    if len(unique) < 2:
        return np.array([-np.inf, np.inf], dtype=float)
    return np.array(unique, dtype=float)


def bucket_ids(frame: pd.DataFrame, edge_map: dict[str, np.ndarray]) -> np.ndarray:
    columns = [
        "forward_velocity_mps",
        "yaw_rate_radps",
        "derived_forward_accel_mps2",
        "measured_yaw_accel_radps2",
    ]
    ids = []
    multipliers = []
    scale = 1
    for col in columns:
        edges = edge_map[col]
        b = np.searchsorted(edges, frame[col].to_numpy(dtype=float), side="right") - 1
        b = np.clip(b, 0, len(edges) - 2)
        ids.append(b.astype(np.int64))
        multipliers.append(scale)
        scale *= max(len(edges) - 1, 1)
    out = np.zeros(len(frame), dtype=np.int64)
    for b, mult in zip(ids, multipliers):
        out += b * mult
    return out


def bucket_weights(ids: np.ndarray) -> np.ndarray:
    groups: dict[int, list[int]] = defaultdict(list)
    for i, value in enumerate(ids):
        groups[int(value)].append(i)
    weights = np.zeros(len(ids), dtype=float)
    if not groups:
        return weights
    cell_mass = 1.0 / len(groups)
    for indices in groups.values():
        weights[indices] = cell_mass / len(indices)
    return weights


def rmse(err: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.square(err)))) if len(err) else math.nan


def weighted_rmse(err: np.ndarray, weights: np.ndarray) -> float:
    mask = weights > 0
    return float(np.sqrt(np.average(np.square(err[mask]), weights=weights[mask]))) if np.any(mask) else math.nan


def mean_bucket_rmse(err: np.ndarray, ids: np.ndarray) -> float:
    groups: dict[int, list[int]] = defaultdict(list)
    for i, value in enumerate(ids):
        groups[int(value)].append(i)
    if not groups:
        return math.nan
    return float(np.mean([rmse(err[indices]) for indices in groups.values()]))


def contact_arrays(frame: pd.DataFrame) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    vf = np.column_stack([frame[f"{c}_v_rel_f_mps"].to_numpy(dtype=float) for c in CONTACTS])
    vr = np.column_stack([frame[f"{c}_v_rel_r_mps"].to_numpy(dtype=float) for c in CONTACTS])
    normal = np.column_stack([frame[f"{c}_normal_n"].to_numpy(dtype=float) for c in CONTACTS])
    current_ff = np.column_stack([frame[f"{c}_force_f_n"].to_numpy(dtype=float) for c in CONTACTS])
    current_fr = np.column_stack([frame[f"{c}_force_r_n"].to_numpy(dtype=float) for c in CONTACTS])
    return vf, vr, normal, current_ff, current_fr


def predict_candidate(
    frame: pd.DataFrame,
    constants: dict[str, float],
    params: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    cf, cr, mu_f, mu_r, drive_scale = params
    vf, vr, normal, _, _ = contact_arrays(frame)
    left_drive = frame["left_drive_force_n"].to_numpy(dtype=float)
    right_drive = frame["right_drive_force_n"].to_numpy(dtype=float)
    drive = np.column_stack(
        [
            0.5 * left_drive,
            0.5 * right_drive,
            0.5 * left_drive,
            0.5 * right_drive,
        ]
    )
    # PlantModel stores relative forward velocity as surface minus body and
    # relative right velocity as zero-surface minus body. Positive gain on the
    # logged relative velocity therefore opposes physical body/contact slip.
    raw_f = drive_scale * drive + cf * vf
    raw_r = cr * vr
    max_f = np.maximum(mu_f * normal, 1.0e-9)
    max_r = np.maximum(mu_r * normal, 1.0e-9)
    utilization = np.sqrt(np.square(raw_f / max_f) + np.square(raw_r / max_r))
    scale = np.where(utilization > 1.0, 1.0 / utilization, 1.0)
    ff = raw_f * scale
    fr = raw_r * scale
    return forces_to_accel(frame, constants, ff, fr)


def forces_to_accel(
    frame: pd.DataFrame,
    constants: dict[str, float],
    ff: np.ndarray,
    fr: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    mass = constants["mass_kg"]
    yaw_den = constants["yaw_denominator_including_wheel_spinup_kg_m2"]
    half_track = 0.5 * constants["track_width_m"]
    forward_offset = constants["drive_wheel_longitudinal_offset_m"]
    r = np.array([-half_track, half_track, -half_track, half_track], dtype=float)
    f = np.array([forward_offset, forward_offset, -forward_offset, -forward_offset], dtype=float)
    total_forward = np.sum(ff, axis=1)
    total_right = np.sum(fr, axis=1)
    yaw_moment = np.sum(f * fr - r * ff, axis=1)
    return total_forward / mass, total_right / mass, yaw_moment / yaw_den, yaw_moment


def current_plant_reference(frame: pd.DataFrame, constants: dict[str, float]) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    _, _, _, ff, fr = contact_arrays(frame)
    return forces_to_accel(frame, constants, ff, fr)


def make_edges(frame: pd.DataFrame) -> dict[str, np.ndarray]:
    return {
        "forward_velocity_mps": quantile_edges(frame["forward_velocity_mps"].to_numpy(dtype=float), 5),
        "yaw_rate_radps": quantile_edges(frame["yaw_rate_radps"].to_numpy(dtype=float), 5),
        "derived_forward_accel_mps2": quantile_edges(frame["derived_forward_accel_mps2"].to_numpy(dtype=float), 5),
        "measured_yaw_accel_radps2": quantile_edges(frame["measured_yaw_accel_radps2"].to_numpy(dtype=float), 5),
    }


def score_params(
    frame: pd.DataFrame,
    constants: dict[str, float],
    params: np.ndarray,
    weights: np.ndarray,
    af_scale: float,
    yaw_scale: float,
) -> float:
    af, _, yaw_accel, _ = predict_candidate(frame, constants, params)
    af_err = (af - frame["derived_forward_accel_mps2"].to_numpy(dtype=float)) / af_scale
    yaw_err = (yaw_accel - frame["measured_yaw_accel_radps2"].to_numpy(dtype=float)) / yaw_scale
    err2 = np.square(af_err) + np.square(yaw_err)
    mask = weights > 0
    return float(np.average(err2[mask], weights=weights[mask]))


def fit_candidate(frame: pd.DataFrame, constants: dict[str, float], edges: dict[str, np.ndarray]) -> np.ndarray:
    fit_mask = frame["dataset_split"].isin(
        ["primary_open_floor_fit_authoritative", "open_floor_fit_downweighted"]
    ).to_numpy()
    fit = frame[fit_mask].reset_index(drop=True)
    ids = bucket_ids(fit, edges)
    weights = bucket_weights(ids)
    af_scale = max(float(np.std(fit["derived_forward_accel_mps2"])), 0.25)
    yaw_scale = max(float(np.std(fit["measured_yaw_accel_radps2"])), 10.0)
    rng = np.random.default_rng(20260603)
    seeds = [
        np.array([4.12, 18.0, 1.65, 1.65, 1.0], dtype=float),
        np.array([8.0, 18.0, 1.36, 1.36, 1.0], dtype=float),
        np.array([2.0, 8.0, 1.0, 1.0, 1.0], dtype=float),
    ]
    for _ in range(900):
        cf = 10 ** rng.uniform(-0.3, 1.7)
        cr = 10 ** rng.uniform(-0.3, 1.9)
        mu_f = rng.uniform(0.35, 2.6)
        mu_r = rng.uniform(0.35, 2.6)
        drive_scale = rng.uniform(0.1, 1.6)
        seeds.append(np.array([cf, cr, mu_f, mu_r, drive_scale], dtype=float))
    best = None
    best_score = math.inf
    for params in seeds:
        score = score_params(fit, constants, params, weights, af_scale, yaw_scale)
        if score < best_score:
            best = params.copy()
            best_score = score
    assert best is not None
    bounds = np.array([[0.05, 100.0], [0.05, 150.0], [0.20, 3.0], [0.20, 3.0], [0.0, 2.0]], dtype=float)
    step_sets = [
        np.array([0.50, 0.50, 0.30, 0.30, 0.25]),
        np.array([0.25, 0.25, 0.15, 0.15, 0.12]),
        np.array([0.12, 0.12, 0.08, 0.08, 0.06]),
        np.array([0.06, 0.06, 0.04, 0.04, 0.03]),
    ]
    for steps in step_sets:
        improved = True
        while improved:
            improved = False
            for j in range(len(best)):
                for direction in (-1.0, 1.0):
                    trial = best.copy()
                    if j < 2:
                        trial[j] *= math.exp(direction * steps[j])
                    else:
                        trial[j] += direction * steps[j]
                    trial[j] = min(max(trial[j], bounds[j, 0]), bounds[j, 1])
                    score = score_params(fit, constants, trial, weights, af_scale, yaw_scale)
                    if score < best_score:
                        best = trial
                        best_score = score
                        improved = True
    return best


def metric_row(
    name: str,
    frame: pd.DataFrame,
    edges: dict[str, np.ndarray],
    af_pred: np.ndarray,
    yaw_accel_pred: np.ndarray,
    yaw_moment_pred: np.ndarray,
    constants: dict[str, float],
) -> dict[str, object]:
    ids = bucket_ids(frame, edges)
    af_target = frame["derived_forward_accel_mps2"].to_numpy(dtype=float)
    yaw_accel_target = frame["measured_yaw_accel_radps2"].to_numpy(dtype=float)
    yaw_moment_target = frame["observed_yaw_moment_nm"].to_numpy(dtype=float)
    force_err = constants["mass_kg"] * (af_pred - af_target)
    moment_err = yaw_moment_pred - yaw_moment_target
    af_err = af_pred - af_target
    yaw_accel_err = yaw_accel_pred - yaw_accel_target
    return {
        "scope": name,
        "rows": len(frame),
        "runs": int(frame["run_id"].nunique()) if len(frame) else 0,
        "occupied_buckets": int(len(set(ids.tolist()))) if len(frame) else 0,
        "forward_accel_rmse_mps2": rmse(af_err),
        "forward_accel_mean_bucket_rmse_mps2": mean_bucket_rmse(af_err, ids),
        "forward_force_rmse_n": rmse(force_err),
        "forward_force_mean_bucket_rmse_n": mean_bucket_rmse(force_err, ids),
        "yaw_accel_rmse_radps2": rmse(yaw_accel_err),
        "yaw_accel_mean_bucket_rmse_radps2": mean_bucket_rmse(yaw_accel_err, ids),
        "yaw_moment_rmse_nm": rmse(moment_err),
        "yaw_moment_mean_bucket_rmse_nm": mean_bucket_rmse(moment_err, ids),
    }


def command_from_torque(command_torque: float, wheel_speed_radps: float, constants: dict[str, float]) -> float:
    resistance = constants["drive_resistance_ohms"]
    speed_constant = constants["speed_constant_radps_per_volt"]
    torque_constant = constants["torque_constant_nm_per_a"]
    gear_ratio = constants["gear_ratio"]
    battery = constants["drive_voltage_v"]
    no_load = constants["no_load_current_a"]
    motor_torque = command_torque / gear_ratio
    torque_sign = (motor_torque > 1.0e-6) - (motor_torque < -1.0e-6)
    wheel_sign = (wheel_speed_radps > 1.0e-6) - (wheel_speed_radps < -1.0e-6)
    no_load_sign = torque_sign if torque_sign else wheel_sign
    current = (motor_torque / torque_constant) + no_load_sign * no_load
    back_emf = wheel_speed_radps * (gear_ratio / speed_constant)
    return ((current * resistance) + back_emf) / battery


def in_place_projection(constants: dict[str, float], params: np.ndarray, target_yaw_rate: float = 1.0) -> dict[str, float | bool]:
    cf, cr, mu_f, mu_r, drive_scale = params
    mass = constants["mass_kg"]
    yaw_den = constants["yaw_denominator_including_wheel_spinup_kg_m2"]
    track = constants["track_width_m"]
    half_track = 0.5 * track
    forward_offset = constants["drive_wheel_longitudinal_offset_m"]
    wheel_radius = constants["wheel_radius_m"]
    total_load = mass * 9.80665 + constants["fan_downforce_full_duty_n"]
    normal = np.full((1, 4), 0.25 * total_load, dtype=float)
    vf = np.zeros((1, 4), dtype=float)
    vr = np.array(
        [[-forward_offset * target_yaw_rate, -forward_offset * target_yaw_rate,
          forward_offset * target_yaw_rate, forward_offset * target_yaw_rate]],
        dtype=float,
    )
    left_speed = half_track * target_yaw_rate / wheel_radius
    right_speed = -left_speed

    def yaw_accel_for_command(abs_command: float) -> float:
        left_command = abs_command
        right_command = -abs_command
        left_torque = command_torque(left_command, left_speed, constants)
        right_torque = command_torque(right_command, right_speed, constants)
        left_drive_force = left_torque / wheel_radius
        right_drive_force = right_torque / wheel_radius
        drive = np.array([[0.5 * left_drive_force, 0.5 * right_drive_force, 0.5 * left_drive_force, 0.5 * right_drive_force]])
        raw_f = drive_scale * drive + cf * vf
        raw_r = cr * vr
        max_f = np.maximum(mu_f * normal, 1.0e-9)
        max_r = np.maximum(mu_r * normal, 1.0e-9)
        utilization = np.sqrt(np.square(raw_f / max_f) + np.square(raw_r / max_r))
        scale = np.where(utilization > 1.0, 1.0 / utilization, 1.0)
        ff = raw_f * scale
        fr = raw_r * scale
        r = np.array([-half_track, half_track, -half_track, half_track], dtype=float)
        f = np.array([forward_offset, forward_offset, -forward_offset, -forward_offset], dtype=float)
        moment = float(np.sum(f * fr - r * ff))
        return moment / yaw_den

    low, high = 0.0, 1.0
    for _ in range(50):
        mid = 0.5 * (low + high)
        if yaw_accel_for_command(mid) >= 0.0:
            high = mid
        else:
            low = mid
    # The acceptance phrasing is "at least command to sustain"; use a 1 rad/s
    # sustaining reference from the prior yaw analyses: command required for
    # zero yaw acceleration against modeled scrub at r=1 rad/s.
    threshold = high
    projected_at_054 = yaw_accel_for_command(0.54)
    projected_at_threshold = yaw_accel_for_command(threshold)
    return {
        "target_yaw_rate_radps": target_yaw_rate,
        "left_command_test": 0.54,
        "right_command_test": -0.54,
        "yaw_accel_at_abs_0p54_radps2": projected_at_054,
        "estimated_abs_command_for_zero_yaw_accel": threshold,
        "left_command_threshold": threshold,
        "right_command_threshold": -threshold,
        "yaw_accel_at_threshold_radps2": projected_at_threshold,
        "passes_requires_at_least_0p54": threshold >= 0.54,
    }


def command_torque(command: float, wheel_speed_radps: float, constants: dict[str, float]) -> float:
    resistance = constants["drive_resistance_ohms"]
    speed_constant = constants["speed_constant_radps_per_volt"]
    torque_constant = constants["torque_constant_nm_per_a"]
    gear_ratio = constants["gear_ratio"]
    battery = constants["drive_voltage_v"]
    no_load = constants["no_load_current_a"]
    applied_voltage = command * battery
    current = (applied_voltage / resistance) - ((wheel_speed_radps * (gear_ratio / speed_constant)) / resistance)
    armature_sign = (current > 1.0e-6) - (current < -1.0e-6)
    wheel_sign = (wheel_speed_radps > 1.0e-6) - (wheel_speed_radps < -1.0e-6)
    no_load_sign = armature_sign if armature_sign else wheel_sign
    load_current = current - no_load_sign * no_load
    if no_load_sign > 0.0 and load_current < 0.0:
        load_current = 0.0
    elif no_load_sign < 0.0 and load_current > 0.0:
        load_current = 0.0
    return torque_constant * gear_ratio * load_current


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    constants = read_constants()
    frame = load_frame()
    edges = make_edges(frame)
    params = fit_candidate(frame, constants, edges)
    acceptance_params = params.copy()
    unconstrained_in_place = in_place_projection(constants, params)
    if not bool(unconstrained_in_place["passes_requires_at_least_0p54"]):
        lo = 0.0
        hi = float(params[4])
        for _ in range(40):
            mid = 0.5 * (lo + hi)
            trial = params.copy()
            trial[4] = mid
            if bool(in_place_projection(constants, trial)["passes_requires_at_least_0p54"]):
                lo = mid
            else:
                hi = mid
        acceptance_params[4] = lo

    cand_af, cand_right, cand_yaw_accel, cand_yaw_moment = predict_candidate(frame, constants, params)
    acc_af, acc_right, acc_yaw_accel, acc_yaw_moment = predict_candidate(frame, constants, acceptance_params)
    plant_af, plant_right, plant_yaw_accel, plant_yaw_moment = current_plant_reference(frame, constants)
    scopes = {
        "all": np.ones(len(frame), dtype=bool),
        "fit_primary": frame["dataset_split"].eq("primary_open_floor_fit_authoritative").to_numpy(),
        "validation_non_primary": frame["dataset_split"].ne("primary_open_floor_fit_authoritative").to_numpy(),
    }
    rows: list[dict[str, object]] = []
    for model_name, af, yaw_accel, yaw_moment in [
        ("candidate_saturated_four_patch", cand_af, cand_yaw_accel, cand_yaw_moment),
        ("candidate_acceptance_adjusted_four_patch", acc_af, acc_yaw_accel, acc_yaw_moment),
        ("current_plantmodel_logged_reference", plant_af, plant_yaw_accel, plant_yaw_moment),
    ]:
        for scope_name, mask in scopes.items():
            sub = frame[mask].reset_index(drop=True)
            rows.append(
                {
                    "model": model_name,
                    **metric_row(scope_name, sub, edges, af[mask], yaw_accel[mask], yaw_moment[mask], constants),
                }
            )
    write_csv(OUT / "rmse_summary.csv", rows)

    param_names = ["cf_n_per_mps", "cr_n_per_mps", "mu_forward", "mu_right", "drive_force_scale"]
    params_row = {"model": "candidate_saturated_four_patch", **{name: float(value) for name, value in zip(param_names, params)}}
    acceptance_params_row = {
        "model": "candidate_acceptance_adjusted_four_patch",
        **{name: float(value) for name, value in zip(param_names, acceptance_params)},
    }
    write_csv(OUT / "fitted_parameters.csv", [params_row, acceptance_params_row])

    split_rows = []
    for split in sorted(VALID_SPLITS):
        mask = frame["dataset_split"].eq(split).to_numpy()
        sub = frame[mask].reset_index(drop=True)
        split_rows.append(
            {
                "model": "candidate_saturated_four_patch",
                **metric_row(split, sub, edges, cand_af[mask], cand_yaw_accel[mask], cand_yaw_moment[mask], constants),
            }
        )
    write_csv(OUT / "candidate_split_rmse.csv", split_rows)

    bucket_design = {
        key: ["-inf" if x == -math.inf else "inf" if x == math.inf else float(x) for x in value]
        for key, value in edges.items()
    }
    ids = bucket_ids(frame, edges)
    bucket_design["occupied_4d_buckets"] = int(len(set(ids.tolist())))
    bucket_design["nominal_bins_per_axis"] = 5
    bucket_design["aggregation"] = "unweighted mean of occupied-bucket RMSEs; rows inside a bucket are equally weighted"
    (OUT / "bucket_design.json").write_text(json.dumps(bucket_design, indent=2), encoding="utf-8")

    in_place = {
        "candidate_saturated_four_patch": unconstrained_in_place,
        "candidate_acceptance_adjusted_four_patch": in_place_projection(constants, acceptance_params),
    }
    (OUT / "in_place_projection.json").write_text(json.dumps(in_place, indent=2), encoding="utf-8")

    provenance = {
        "primary_metadata_csv": str(PRIMARY),
        "feature_csv": str(FEATURES),
        "constants_csv": str(CONSTANTS),
        "discovered_logs_csv": str(DISCOVERED_LOGS),
        "log_inventory_filter": "status starts with included and kind != competition_fwc",
        "rows": int(len(frame)),
        "runs": int(frame["run_id"].nunique()),
        "splits": {str(k): int(v) for k, v in frame["dataset_split"].value_counts().sort_index().items()},
        "fields_used": [
            "run_id",
            "row_index",
            "time_us",
            "dataset_split",
            "left_drive_force_n",
            "right_drive_force_n",
            "forward_velocity_mps",
            "yaw_rate_radps",
            "derived_forward_accel_mps2",
            "measured_yaw_accel_radps2",
            "observed_yaw_moment_nm",
            "model_yaw_moment_nm",
            "*_v_rel_f_mps",
            "*_v_rel_r_mps",
            "*_normal_n",
            "*_force_f_n",
            "*_force_r_n",
        ],
    }
    (OUT / "provenance.json").write_text(json.dumps(provenance, indent=2), encoding="utf-8")
    print(f"wrote {OUT}")
    print(json.dumps({"params": [params_row, acceptance_params_row], "in_place": in_place, "rows": len(frame)}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
