#!/usr/bin/env python3
"""Standalone contact-patch traction law fitting.

Analysis-only script. It predicts observed yaw moment directly from patch
kinematics, normal load, drive force, and contact geometry. Existing PlantModel
outputs are used only as external reference error columns.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from collections import Counter
from dataclasses import dataclass
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from codex_analysis.yaw_model_variant_fits.regime_weighting import (
    LATEST_FIT_RUNS,
    LATEST_INCLUSIVE_FIT_SPLIT_WEIGHTS,
    QualityConfig,
    RegimeWeightConfig,
    compute_regime_weights,
    latest_inclusive_fit_mask,
    write_regime_diagnostics,
)
from codex_analysis.yaw_model_variant_fits.common_range_metrics import (
    write_common_range_metrics,
)

PRIMARY = ROOT / "codex_analysis" / "contact_continuum_yaw_identification" / "ablation" / "phase_classified_feature_sample.csv"
SECONDARY = ROOT / "codex_analysis" / "contact_continuum_yaw_identification" / "features" / "contact_continuum_feature_sample.csv"
CONSTANTS = ROOT / "codex_analysis" / "contact_continuum_yaw_identification" / "features" / "plant_mirror_constants.csv"

VARIANT_C_SPLIT = ROOT / "codex_analysis" / "yaw_model_variant_fits" / "combined_slip_surface" / "split_metrics.csv"
VARIANT_C_SELECTED = ROOT / "codex_analysis" / "yaw_model_variant_fits" / "combined_slip_surface" / "selected_log_metrics.csv"
FORCE_DOMAIN_SPLIT = ROOT / "codex_analysis" / "yaw_model_variant_fits" / "round2_force_domain_stribeck" / "split_rmse.csv"
FORCE_DOMAIN_SELECTED = ROOT / "codex_analysis" / "yaw_model_variant_fits" / "round2_force_domain_stribeck" / "selected_log_rmse.csv"
MAX_NORM_SPLIT = ROOT / "codex_analysis" / "yaw_model_variant_fits" / "transition_options" / "max_norm_yield_allocation" / "split_metrics.csv"
MAX_NORM_SELECTED = ROOT / "codex_analysis" / "yaw_model_variant_fits" / "transition_options" / "max_norm_yield_allocation" / "selected_log_metrics.csv"
RATIONAL_BLEND_SPLIT = ROOT / "codex_analysis" / "yaw_model_variant_fits" / "transition_options" / "rational_speed_force_blend" / "split_metrics.csv"
RATIONAL_BLEND_SELECTED = ROOT / "codex_analysis" / "yaw_model_variant_fits" / "transition_options" / "rational_speed_force_blend" / "selected_log_metrics.csv"
TRUE_TRACTION_SPLIT = ROOT / "codex_analysis" / "yaw_model_variant_fits" / "contact_patch_true_traction_testbed" / "split_metrics.csv"
TRUE_TRACTION_SELECTED = ROOT / "codex_analysis" / "yaw_model_variant_fits" / "contact_patch_true_traction_testbed" / "selected_log_metrics.csv"

SELECTED_LOGS = [
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

SECONDARY_FIELDS = [
    "observed_yaw_moment_nm",
    "model_yaw_moment_nm",
    "left_encoder_velocity_mps",
    "right_encoder_velocity_mps",
    "left_drive_force_n",
    "right_drive_force_n",
    "total_normal_load_n",
    "fl_normal_n",
    "fr_normal_n",
    "rl_normal_n",
    "rr_normal_n",
    "fl_v_rel_f_mps",
    "fr_v_rel_f_mps",
    "rl_v_rel_f_mps",
    "rr_v_rel_f_mps",
    "fl_v_rel_r_mps",
    "fr_v_rel_r_mps",
    "rl_v_rel_r_mps",
    "rr_v_rel_r_mps",
]

SPLITS = [
    "primary_open_floor_fit_authoritative",
    "open_floor_fit_downweighted",
    "open_floor_validation_only",
    "diag_validation_only",
    "aux_downweighted_validation",
]


@dataclass
class Sample:
    run_id: str
    family: str
    split: str
    phase: str
    row_index: int
    time_us: float
    forward_mps: float
    yaw_rate: float
    measured_yaw_accel: float
    vbar_rel: float
    vbar_yaw: float
    limiter: float
    saturation: float
    gyro_spike: float
    observed_moment: float
    baseline_moment: float
    residual: float
    left_encoder_velocity: float
    right_encoder_velocity: float
    left_drive_force: float
    right_drive_force: float
    normal: tuple[float, float, float, float]
    vrel_f: tuple[float, float, float, float]
    vrel_r: tuple[float, float, float, float]


@dataclass
class Candidate:
    name: str
    dyn_k_mps: float
    static_k_mps: float
    static_fwd_k_mps: float
    coeffs: list[float]
    train_rmse: float
    train_run_balanced_rmse: float
    launch_opposing_nm: float
    launch_left_command: float
    launch_right_command: float
    launch_gate_pass: bool
    score: float


def finite_float(value: object, default: float = 0.0) -> float:
    try:
        out = float(value)
        return out if math.isfinite(out) else default
    except (TypeError, ValueError):
        return default


def sign(value: float, eps: float = 1.0e-6) -> float:
    if value > eps:
        return 1.0
    if value < -eps:
        return -1.0
    return 0.0


def q(values: list[float], p: float) -> float:
    clean = sorted(v for v in values if math.isfinite(v))
    if not clean:
        return 0.0
    if len(clean) == 1:
        return clean[0]
    pos = (len(clean) - 1) * p
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return clean[lo]
    frac = pos - lo
    return clean[lo] * (1.0 - frac) + clean[hi] * frac


def median(values: list[float]) -> float:
    return q(values, 0.5)


def rmse(values: list[float]) -> float:
    return math.sqrt(sum(v * v for v in values) / len(values)) if values else 0.0


def mae(values: list[float]) -> float:
    return sum(abs(v) for v in values) / len(values) if values else 0.0


def weighted_rmse(values: list[float], weights: list[float]) -> float:
    total = sum(weights)
    return math.sqrt(sum(w * v * v for v, w in zip(values, weights)) / total) if total > 0.0 else 0.0


def read_constants() -> dict[str, float]:
    with CONSTANTS.open(newline="", encoding="utf-8") as fh:
        return {row["name"]: float(row["value"]) for row in csv.DictReader(fh)}


def read_secondary() -> dict[tuple[str, str], dict[str, float]]:
    out: dict[tuple[str, str], dict[str, float]] = {}
    with SECONDARY.open(newline="", encoding="utf-8") as fh:
        for row in csv.DictReader(fh):
            out[(row["run_id"], row["row_index"])] = {
                field: finite_float(row.get(field, 0.0)) for field in SECONDARY_FIELDS
            }
    return out


def load_samples() -> list[Sample]:
    secondary = read_secondary()
    samples: list[Sample] = []
    with PRIMARY.open(newline="", encoding="utf-8") as fh:
        for row in csv.DictReader(fh):
            split = row.get("dataset_split", "")
            if split == "excluded_or_unclassified":
                continue
            extra = secondary.get((row["run_id"], row["row_index"]))
            if extra is None:
                continue
            observed = extra["observed_yaw_moment_nm"]
            baseline = extra["model_yaw_moment_nm"]
            samples.append(
                Sample(
                    run_id=row["run_id"],
                    family=row["family"],
                    split=split,
                    phase=row.get("physics_phase", ""),
                    row_index=int(finite_float(row["row_index"])),
                    time_us=finite_float(row.get("time_us", 0.0)),
                    forward_mps=finite_float(row.get("forward_velocity_mps", 0.0)),
                    yaw_rate=finite_float(row.get("yaw_rate_radps", 0.0)),
                    measured_yaw_accel=finite_float(row.get("measured_yaw_accel_radps2", 0.0)),
                    vbar_rel=finite_float(row.get("vbar_rel_mps", 0.0)),
                    vbar_yaw=finite_float(row.get("vbar_yaw_mps", 0.0)),
                    limiter=finite_float(row.get("limiter_active", 0.0)),
                    saturation=finite_float(row.get("hardware_saturation_evidence", 0.0)),
                    gyro_spike=finite_float(row.get("gyro_derivative_spike", 0.0)),
                    observed_moment=observed,
                    baseline_moment=baseline,
                    residual=observed - baseline,
                    left_encoder_velocity=extra["left_encoder_velocity_mps"],
                    right_encoder_velocity=extra["right_encoder_velocity_mps"],
                    left_drive_force=extra["left_drive_force_n"],
                    right_drive_force=extra["right_drive_force_n"],
                    normal=(extra["fl_normal_n"], extra["fr_normal_n"], extra["rl_normal_n"], extra["rr_normal_n"]),
                    vrel_f=(
                        extra["fl_v_rel_f_mps"],
                        extra["fr_v_rel_f_mps"],
                        extra["rl_v_rel_f_mps"],
                        extra["rr_v_rel_f_mps"],
                    ),
                    vrel_r=(
                        extra["fl_v_rel_r_mps"],
                        extra["fr_v_rel_r_mps"],
                        extra["rl_v_rel_r_mps"],
                        extra["rr_v_rel_r_mps"],
                    ),
                )
            )
    return samples


def regime_weight_result(samples: list[Sample]):
    columns = {
        "run_id": [sample.run_id for sample in samples],
        "dataset_split": [sample.split for sample in samples],
        "row_index": [sample.row_index for sample in samples],
        "time_us": [sample.time_us for sample in samples],
        "forward_velocity_mps": [sample.forward_mps for sample in samples],
        "yaw_rate_radps": [sample.yaw_rate for sample in samples],
        "measured_yaw_accel_radps2": [sample.measured_yaw_accel for sample in samples],
        "vbar_yaw_mps": [sample.vbar_yaw for sample in samples],
        "limiter_active": [sample.limiter for sample in samples],
        "hardware_saturation_evidence": [sample.saturation for sample in samples],
        "gyro_derivative_spike": [sample.gyro_spike for sample in samples],
    }
    return compute_regime_weights(
        columns,
        RegimeWeightConfig(
            split_weights=LATEST_INCLUSIVE_FIT_SPLIT_WEIGHTS,
            quality=QualityConfig(
                gyro_spike_multiplier=0.10,
                saturation_multiplier=0.35,
                use_limiter_penalty=False,
                use_low_yaw_no_motion_penalty=False,
            ),
        ),
        eligible_mask=latest_inclusive_fit_mask(columns),
    )


def fit_weights(samples: list[Sample]) -> list[float]:
    return regime_weight_result(samples).weights


def run_balanced_weights(samples: list[Sample]) -> list[float]:
    counts = Counter(sample.run_id for sample in samples)
    return [1.0 / max(counts[sample.run_id], 1) for sample in samples]


def contact_geometry(constants: dict[str, float]) -> tuple[tuple[float, float, str], ...]:
    half_track = 0.5 * constants["track_width_m"]
    offset = constants["drive_wheel_longitudinal_offset_m"]
    return (
        (-half_track, offset, "left"),
        (half_track, offset, "right"),
        (-half_track, -offset, "left"),
        (half_track, -offset, "right"),
    )


def rational_slip(value: float, knee: float) -> float:
    return value / (abs(value) + max(knee, 1.0e-9))


def static_gate(vf: float, vr: float, forward_mps: float, slip_k: float, fwd_k: float) -> float:
    slip = math.sqrt(vf * vf + vr * vr)
    slip_part = 1.0 / (1.0 + (slip / max(slip_k, 1.0e-9)) ** 2)
    fwd_part = 1.0 / (1.0 + (abs(forward_mps) / max(fwd_k, 1.0e-9)) ** 2)
    return slip_part * fwd_part


def feature_vector(sample: Sample, constants: dict[str, float], name: str, dyn_k: float, static_k: float, fwd_k: float) -> list[float]:
    contacts = contact_geometry(constants)
    drive_moment = 0.0
    lateral_dynamic_moment = 0.0
    lateral_static_moment = 0.0
    longitudinal_dynamic_moment = 0.0

    for index, (r_pos, f_pos, side) in enumerate(contacts):
        drive_force = sample.left_drive_force if side == "left" else sample.right_drive_force
        drive_moment += -r_pos * (0.5 * drive_force)

        normal = sample.normal[index]
        wheel_surface = sample.left_encoder_velocity if side == "left" else sample.right_encoder_velocity
        if math.isfinite(wheel_surface):
            vf = wheel_surface - (sample.forward_mps - sample.yaw_rate * r_pos)
        else:
            vf = sample.vrel_f[index]
        vr = sample.vrel_r[index]
        lateral_dynamic_moment += f_pos * normal * rational_slip(vr, dyn_k)
        longitudinal_dynamic_moment += -r_pos * normal * rational_slip(vf, dyn_k)
        lateral_static_moment += f_pos * normal * sign(vr) * static_gate(vf, vr, sample.forward_mps, static_k, fwd_k)

    if name == "direct_patch_rational":
        return [drive_moment, lateral_dynamic_moment, longitudinal_dynamic_moment]
    if name == "yaw_static_yield_patch":
        return [drive_moment, lateral_dynamic_moment, lateral_static_moment, longitudinal_dynamic_moment]
    raise ValueError(name)


def solve_linear(features: list[list[float]], targets: list[float], weights: list[float], ridge: float) -> list[float]:
    n = len(features[0])
    lhs = [[0.0 for _ in range(n)] for _ in range(n)]
    rhs = [0.0 for _ in range(n)]
    for x, y, w in zip(features, targets, weights):
        for i in range(n):
            rhs[i] += w * x[i] * y
            for j in range(n):
                lhs[i][j] += w * x[i] * x[j]
    for i in range(n):
        lhs[i][i] += ridge

    aug = [lhs[i][:] + [rhs[i]] for i in range(n)]
    for col in range(n):
        pivot = max(range(col, n), key=lambda row: abs(aug[row][col]))
        aug[col], aug[pivot] = aug[pivot], aug[col]
        if abs(aug[col][col]) < 1.0e-14:
            continue
        scale = aug[col][col]
        for j in range(col, n + 1):
            aug[col][j] /= scale
        for row in range(n):
            if row == col:
                continue
            factor = aug[row][col]
            for j in range(col, n + 1):
                aug[row][j] -= factor * aug[col][j]
    return [aug[i][n] for i in range(n)]


def predict(sample: Sample, constants: dict[str, float], candidate: Candidate) -> float:
    features = feature_vector(sample, constants, candidate.name, candidate.dyn_k_mps, candidate.static_k_mps, candidate.static_fwd_k_mps)
    return sum(c * x for c, x in zip(candidate.coeffs, features))


def torque_from_command(command: float, wheel_speed_radps: float, constants: dict[str, float]) -> float:
    resistance = constants["drive_resistance_ohms"]
    speed_constant = constants["speed_constant_radps_per_volt"]
    torque_constant = constants["torque_constant_nm_per_a"]
    gear_ratio = constants["gear_ratio"]
    battery = constants["drive_voltage_v"]
    no_load = constants["no_load_current_a"]

    applied_voltage = command * battery
    back_emf = wheel_speed_radps * (gear_ratio / speed_constant)
    current = (applied_voltage / resistance) - (back_emf / resistance)
    armature_sign = sign(current)
    wheel_sign = sign(wheel_speed_radps)
    no_load_sign = armature_sign if armature_sign else wheel_sign
    load_current = current - no_load_sign * no_load
    if no_load_sign > 0.0 and load_current < 0.0:
        load_current = 0.0
    if no_load_sign < 0.0 and load_current > 0.0:
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


def launch_opposing_moment(constants: dict[str, float], candidate: Candidate) -> float:
    total_normal = constants["mass_kg"] * 9.80665 + constants.get("fan_downforce_full_duty_n", 0.0) * 0.8
    sample = Sample(
        run_id="synthetic_launch",
        family="synthetic",
        split="synthetic",
        phase="launch",
        row_index=0,
        time_us=0.0,
        forward_mps=0.0,
        yaw_rate=1.0,
        measured_yaw_accel=0.0,
        vbar_rel=0.0,
        vbar_yaw=0.0,
        limiter=0.0,
        saturation=0.0,
        gyro_spike=0.0,
        observed_moment=0.0,
        baseline_moment=0.0,
        residual=0.0,
        left_drive_force=0.0,
        right_drive_force=0.0,
        left_encoder_velocity=0.5 * constants["track_width_m"],
        right_encoder_velocity=-0.5 * constants["track_width_m"],
        normal=(0.25 * total_normal, 0.25 * total_normal, 0.25 * total_normal, 0.25 * total_normal),
        vrel_f=(0.0, 0.0, 0.0, 0.0),
        vrel_r=(
            -constants["drive_wheel_longitudinal_offset_m"],
            -constants["drive_wheel_longitudinal_offset_m"],
            constants["drive_wheel_longitudinal_offset_m"],
            constants["drive_wheel_longitudinal_offset_m"],
        ),
    )
    features = feature_vector(sample, constants, candidate.name, candidate.dyn_k_mps, candidate.static_k_mps, candidate.static_fwd_k_mps)
    drive_free = sum(c * x for c, x in zip(candidate.coeffs, features))
    return max(-drive_free, 0.0)


def launch_commands(constants: dict[str, float], candidate_name: str, coeffs: list[float], dyn_k: float, static_k: float, fwd_k: float) -> tuple[float, float, float]:
    candidate = Candidate(candidate_name, dyn_k, static_k, fwd_k, coeffs, 0.0, 0.0, 0.0, 0.0, 0.0, False, 0.0)
    opposing = launch_opposing_moment(constants, candidate)
    drive_scale = coeffs[0] if coeffs else 0.0
    if drive_scale <= 1.0e-9:
        return opposing, float("inf"), float("-inf")
    half_track = 0.5 * constants["track_width_m"]
    wheel_speed = half_track / constants["wheel_radius_m"]
    applied_bank_torque = opposing * constants["wheel_radius_m"] / (constants["track_width_m"] * drive_scale)
    rolling = constants["rolling_friction_torque_nm"]
    left = command_from_torque(applied_bank_torque + rolling, wheel_speed, constants)
    right = command_from_torque(-applied_bank_torque - rolling, -wheel_speed, constants)
    return opposing, left, right


def tune_candidates(samples: list[Sample], constants: dict[str, float]) -> list[Candidate]:
    train = [
        sample
        for sample in samples
        if sample.split == "primary_open_floor_fit_authoritative" or sample.run_id in LATEST_FIT_RUNS
    ]
    targets = [sample.observed_moment for sample in train]
    weights = fit_weights(train)
    run_weights = run_balanced_weights(train)

    raw: list[Candidate] = []
    grids = [
        ("direct_patch_rational", [0.001, 0.003, 0.005, 0.01, 0.02, 0.04, 0.08, 0.16], [0.03], [10.0]),
        (
            "yaw_static_yield_patch",
            [0.001, 0.003, 0.005, 0.01, 0.02, 0.04, 0.08, 0.16],
            [0.005, 0.01, 0.02, 0.03, 0.05, 0.08],
            [0.05, 0.10, 0.20, 0.40, 10.0],
        ),
    ]
    for name, dyn_grid, static_grid, fwd_grid in grids:
        for dyn_k in dyn_grid:
            for static_k in static_grid:
                for fwd_k in fwd_grid:
                    features = [feature_vector(sample, constants, name, dyn_k, static_k, fwd_k) for sample in train]
                    coeffs = solve_linear(features, targets, weights, ridge=1.0e-8)
                    pred_errors = [sum(c * x for c, x in zip(coeffs, row)) - y for row, y in zip(features, targets)]
                    train_rmse = rmse(pred_errors)
                    train_balanced = weighted_rmse(pred_errors, run_weights)
                    opposing, left, right = launch_commands(constants, name, coeffs, dyn_k, static_k, fwd_k)
                    gate = abs(left) >= 0.6 and abs(right) >= 0.6 and abs(left) <= 1.05 and abs(right) <= 1.05
                    drive_scale = coeffs[0] if coeffs else -1.0
                    physical_penalty = 0.0
                    if drive_scale <= 0.0 or drive_scale > 2.0:
                        physical_penalty += 10.0
                    if len(coeffs) > 1 and coeffs[1] < 0.0:
                        physical_penalty += 10.0
                    if name == "yaw_static_yield_patch" and coeffs[2] < 0.0:
                        physical_penalty += 10.0
                    score = train_rmse + 0.15 * train_balanced + physical_penalty
                    raw.append(
                        Candidate(
                            name=name,
                            dyn_k_mps=dyn_k,
                            static_k_mps=static_k,
                            static_fwd_k_mps=fwd_k,
                            coeffs=coeffs,
                            train_rmse=train_rmse,
                            train_run_balanced_rmse=train_balanced,
                            launch_opposing_nm=opposing,
                            launch_left_command=left,
                            launch_right_command=right,
                            launch_gate_pass=gate,
                            score=score,
                        )
                    )
    return sorted(raw, key=lambda item: item.score)


def metric_row(name: str, subset: list[Sample], constants: dict[str, float], candidate: Candidate) -> dict[str, object]:
    baseline_errors = [sample.baseline_moment - sample.observed_moment for sample in subset]
    model_errors = [predict(sample, constants, candidate) - sample.observed_moment for sample in subset]
    run_weights = run_balanced_weights(subset)
    preds = [predict(sample, constants, candidate) for sample in subset]
    return {
        "group": name,
        "count": len(subset),
        "run_count": len(set(sample.run_id for sample in subset)),
        "baseline_rmse_nm": rmse(baseline_errors),
        "standalone_rmse_nm": rmse(model_errors),
        "baseline_mae_nm": mae(baseline_errors),
        "standalone_mae_nm": mae(model_errors),
        "baseline_median_abs_nm": median([abs(v) for v in baseline_errors]),
        "standalone_median_abs_nm": median([abs(v) for v in model_errors]),
        "baseline_signed_median_nm": median(baseline_errors),
        "standalone_signed_median_nm": median(model_errors),
        "run_balanced_baseline_rmse_nm": weighted_rmse(baseline_errors, run_weights),
        "run_balanced_standalone_rmse_nm": weighted_rmse(model_errors, run_weights),
        "prediction_median_nm": median(preds),
        "rmse_improvement_fraction": (rmse(baseline_errors) - rmse(model_errors)) / rmse(baseline_errors) if baseline_errors else 0.0,
    }


def read_rows(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8") as fh:
        return list(csv.DictReader(fh))


def reference_by_split() -> dict[str, dict[str, str]]:
    refs: dict[str, dict[str, str]] = {}
    for row in read_rows(VARIANT_C_SPLIT):
        refs.setdefault(row["group"], {})["variant_c_rmse_nm"] = row.get("corrected_rmse_nm", "")
    for row in read_rows(FORCE_DOMAIN_SPLIT):
        refs.setdefault(row["dataset_split"], {})["force_domain_stribeck_rmse_nm"] = row.get("corrected_rmse_nm", "")
    for row in read_rows(MAX_NORM_SPLIT):
        refs.setdefault(row["group"], {})["rational_reserve_reference_rmse_nm"] = row.get("corrected_rmse_nm", "")
    for row in read_rows(RATIONAL_BLEND_SPLIT):
        refs.setdefault(row.get("dataset_split", row.get("group", "")), {})["rational_residual_reference_rmse_nm"] = row.get("candidate_rmse_nm", row.get("corrected_rmse_nm", ""))
    for row in read_rows(TRUE_TRACTION_SPLIT):
        refs.setdefault(row["group"], {})["prior_force_level_testbed_rmse_nm"] = row.get("corrected_rmse_nm", "")
    return refs


def reference_by_run() -> dict[str, dict[str, str]]:
    refs: dict[str, dict[str, str]] = {}
    for row in read_rows(VARIANT_C_SELECTED):
        refs.setdefault(row["run_id"], {})["variant_c_rmse_nm"] = row.get("corrected_rmse_nm", "")
    for row in read_rows(FORCE_DOMAIN_SELECTED):
        refs.setdefault(row["run_id"], {})["force_domain_stribeck_rmse_nm"] = row.get("corrected_rmse_nm", "")
    for row in read_rows(MAX_NORM_SELECTED):
        refs.setdefault(row["run_id"], {})["rational_reserve_reference_rmse_nm"] = row.get("corrected_rmse_nm", "")
    for row in read_rows(RATIONAL_BLEND_SELECTED):
        refs.setdefault(row["run_id"], {})["rational_residual_reference_rmse_nm"] = row.get("candidate_rmse_nm", row.get("corrected_rmse_nm", ""))
    for row in read_rows(TRUE_TRACTION_SELECTED):
        refs.setdefault(row["run_id"], {})["prior_force_level_testbed_rmse_nm"] = row.get("corrected_rmse_nm", "")
    return refs


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    if not rows:
        return
    keys: list[str] = []
    for row in rows:
        for key in row:
            if key not in keys:
                keys.append(key)
    with path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=keys)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def fmt(value: object) -> str:
    if isinstance(value, float):
        if not math.isfinite(value):
            return ""
        return f"{value:.6f}"
    return str(value)


def make_report(
    constants: dict[str, float],
    best: Candidate,
    candidate_rows: list[dict[str, object]],
    split_metrics: list[dict[str, object]],
    selected_metrics: list[dict[str, object]],
    risk_metrics: list[dict[str, object]],
    common_range_metrics: list[dict[str, object]],
    report_path: Path,
) -> None:
    coeff_names = {
        "direct_patch_rational": ["drive_scale", "lateral_dynamic_mu", "longitudinal_dynamic_mu"],
        "yaw_static_yield_patch": ["drive_scale", "lateral_dynamic_mu", "lateral_static_yield_mu", "longitudinal_dynamic_mu"],
    }[best.name]
    lines = [
        "# Standalone Contact-Patch Traction Testbed",
        "",
        "Analysis-only output. Production code, build metadata, and tests were not edited.",
        "",
        "## Scope",
        "",
        "The selected law is standalone: it predicts `observed_yaw_moment_nm` directly from contact-patch relative velocity, normal load, drive force, and contact geometry. It does not use old PlantModel contact-force outputs, old contact gains, old yaw moment, or a residual target as a runtime component. Current baseline, rational residual/reference models, Variant C, and force-domain Stribeck are reported only as external references.",
        "",
        "## Selected Law",
        "",
        "For each patch `i` at position `(r_i, f_i)`, with `r>0` right and `f>0` forward:",
        "",
        "`F_drive_f_i = drive_scale * F_drive_side / 2`",
        "",
        "`F_dyn_r_i = lateral_dynamic_mu * N_i * v_rel_r_i / (abs(v_rel_r_i) + v_dyn_k)`",
        "",
        "`F_static_r_i = lateral_static_yield_mu * N_i * sign(v_rel_r_i) * G_static_i`",
        "",
        "`G_static_i = 1 / (1 + (sqrt(v_rel_f_i^2 + v_rel_r_i^2) / v_static_k)^2) * 1 / (1 + (abs(Vf) / Vf_static_k)^2)`",
        "",
        "`F_dyn_f_i = longitudinal_dynamic_mu * N_i * v_rel_f_i / (abs(v_rel_f_i) + v_dyn_k)`",
        "",
        "`F_f_i = F_drive_f_i + F_dyn_f_i` and `F_r_i = F_dyn_r_i + F_static_r_i`",
        "",
        "`M_yaw = sum_i(f_i * F_r_i - r_i * F_f_i)`",
        "",
        "The selected form uses `sqrt`, `abs`, rational schedules, and sign/clamp-style branching only. It has no trigonometry, `exp`, `tanh`, lookup table, command selector, old-force branch, or residual-additive branch.",
        "",
        "## Tuned Constants",
        "",
        "| parameter | value |",
        "| --- | ---: |",
        f"| family | {best.name} |",
        f"| track_width_m | {constants['track_width_m']:.6f} |",
        f"| v_dyn_k_mps | {best.dyn_k_mps:.6f} |",
        f"| v_static_k_mps | {best.static_k_mps:.6f} |",
        f"| Vf_static_k_mps | {best.static_fwd_k_mps:.6f} |",
    ]
    for name, value in zip(coeff_names, best.coeffs):
        lines.append(f"| {name} | {value:.9f} |")
    lines.extend(
        [
            "",
            "## +1 rad/s In-Place Launch",
            "",
            "| opposing scrub Nm | left command | right command | max abs command | gate | launch lock policy |",
            "| ---: | ---: | ---: | ---: | --- | --- |",
            f"| {best.launch_opposing_nm:.6f} | {best.launch_left_command:.6f} | {best.launch_right_command:.6f} | {max(abs(best.launch_left_command), abs(best.launch_right_command)):.6f} | {'pass' if best.launch_gate_pass else 'fail'} | diagnostic_only |",
            "",
            "## Split Metrics",
            "",
            "| split | count | baseline RMSE | standalone RMSE | improvement | Variant C ref | prior force-level ref | force-domain Stribeck ref | rational reserve ref | rational residual ref |",
            "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
        ]
    )
    for row in split_metrics:
        lines.append(
            f"| {row['group']} | {row['count']} | {fmt(row['baseline_rmse_nm'])} | {fmt(row['standalone_rmse_nm'])} | "
            f"{fmt(row['rmse_improvement_fraction'])} | {row.get('variant_c_rmse_nm', '')} | {row.get('prior_force_level_testbed_rmse_nm', '')} | "
            f"{row.get('force_domain_stribeck_rmse_nm', '')} | {row.get('rational_reserve_reference_rmse_nm', '')} | {row.get('rational_residual_reference_rmse_nm', '')} |"
        )
    lines.extend(
        [
            "",
            "## Risk Slices",
            "",
            "| slice | count | baseline RMSE | standalone RMSE | improvement |",
            "| --- | ---: | ---: | ---: | ---: |",
        ]
    )
    for row in risk_metrics:
        lines.append(
            f"| {row['group']} | {row['count']} | {fmt(row['baseline_rmse_nm'])} | {fmt(row['standalone_rmse_nm'])} | {fmt(row['rmse_improvement_fraction'])} |"
        )
    lines.extend(
        [
            "",
            "## Common Range Metrics",
            "",
            "Shared operating-range definitions are written to `common_range_metrics.csv`; the former `0.7 m/s high-speed` label is intentionally not used.",
            "",
            "| range | count | baseline RMSE | candidate RMSE | candidate MAE | candidate median abs |",
            "| --- | ---: | ---: | ---: | ---: | ---: |",
        ]
    )
    for row in common_range_metrics:
        lines.append(
            f"| {row['range_name']} | {row['count']} | {fmt(row['baseline_rmse_nm'])} | {fmt(row['candidate_rmse_nm'])} | {fmt(row['candidate_mae_nm'])} | {fmt(row['candidate_median_abs_nm'])} |"
        )
    lines.extend(
        [
            "",
            "## Selected Logs",
            "",
            "| run | split | count | baseline RMSE | standalone RMSE | Variant C ref | prior force-level ref | force-domain ref |",
            "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |",
        ]
    )
    for row in selected_metrics:
        lines.append(
            f"| {row['run_id']} | {row['dataset_split']} | {row['count']} | {fmt(row['baseline_rmse_nm'])} | {fmt(row['standalone_rmse_nm'])} | "
            f"{row.get('variant_c_rmse_nm', '')} | {row.get('prior_force_level_testbed_rmse_nm', '')} | {row.get('force_domain_stribeck_rmse_nm', '')} |"
        )
    lines.extend(
        [
            "",
            "## Candidate Summary",
            "",
            "The top scored candidates are in `candidate_scores.csv`. This run is unconstrained by launch command: the in-place yaw launch estimate is reported as a diagnostic only and is not used as a score term or candidate-selection gate.",
            "",
            "## Recommendation",
            "",
            "Viable as a standalone baseline contact-patch law. The production shape implied by this pass is a direct patch-force accumulation in `PlantModel`: distribute physical drive force to patches, add dynamic rational slip force and low-speed static yaw-yield force from patch kinematics/load, then accumulate `sum(f*Fr - r*Ff)`. Do not install it as `old + residual` or as an old-force correction branch.",
            "",
            "## Outputs",
            "",
            "- `candidate_scores.csv`",
            "- `split_metrics.csv`",
            "- `selected_log_metrics.csv`",
            "- `risk_metrics.csv`",
            "- `common_range_metrics.csv`",
            "- `selected_parameters.csv`",
            "- `in_place_1radps_command.csv`",
            "- `prediction_sample.csv`",
        ]
    )
    report_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--track-width-mm", type=float, default=None)
    parser.add_argument("--artifact-prefix", default="")
    args = parser.parse_args()

    constants = read_constants()
    if args.track_width_mm is not None:
        constants["track_width_m"] = args.track_width_mm / 1000.0
    samples = load_samples()
    write_regime_diagnostics(OUT, args.artifact_prefix, regime_weight_result(samples))
    candidates = tune_candidates(samples, constants)
    best = candidates[0]

    candidate_rows = []
    for rank, candidate in enumerate(candidates[:80], start=1):
        candidate_rows.append(
            {
                "rank": rank,
                "family": candidate.name,
                "dyn_k_mps": candidate.dyn_k_mps,
                "static_k_mps": candidate.static_k_mps,
                "static_fwd_k_mps": candidate.static_fwd_k_mps,
                "coefficients": ";".join(f"{value:.12g}" for value in candidate.coeffs),
                "train_rmse_nm": candidate.train_rmse,
                "train_run_balanced_rmse_nm": candidate.train_run_balanced_rmse,
                "launch_opposing_nm": candidate.launch_opposing_nm,
                "launch_left_command": candidate.launch_left_command,
                "launch_right_command": candidate.launch_right_command,
                "launch_gate_pass": candidate.launch_gate_pass,
                "score": candidate.score,
            }
        )

    split_refs = reference_by_split()
    split_metrics = []
    for split in SPLITS:
        subset = [sample for sample in samples if sample.split == split]
        row = metric_row(split, subset, constants, best)
        row.update(split_refs.get(split, {}))
        split_metrics.append(row)
    validation = [sample for sample in samples if sample.split != "primary_open_floor_fit_authoritative"]
    validation_row = metric_row("validation_non_authoritative", validation, constants, best)
    validation_row.update(split_refs.get("validation_non_authoritative", {}))
    split_metrics.append(validation_row)

    risk_specs = {
        "calibration_low_vf_nonzero_yaw": lambda s: abs(s.forward_mps) < 0.15 and abs(s.yaw_rate) >= 0.1,
        "in_place_scrub": lambda s: abs(s.forward_mps) < 0.05 and abs(s.yaw_rate) >= 0.2,
        "slow_forward_turn": lambda s: 0.15 <= abs(s.forward_mps) < 0.70 and abs(s.yaw_rate) >= 0.1,
        "pre_design_turn_speed": lambda s: 0.70 <= abs(s.forward_mps) < 0.95 and abs(s.yaw_rate) >= 0.1,
        "design_turn_speed_and_up": lambda s: abs(s.forward_mps) >= 0.95 and abs(s.yaw_rate) >= 0.1,
        "fast_forward": lambda s: abs(s.forward_mps) >= 1.50,
        "straightish_forward": lambda s: abs(s.yaw_rate) < 0.05 and abs(s.forward_mps) >= 0.05,
        "limiter_active": lambda s: s.limiter > 0.0,
        "hardware_saturation_evidence": lambda s: s.saturation > 0.0,
        "may4_latest_logs": lambda s: s.run_id in LATEST_FIT_RUNS,
        "open_floor_all": lambda s: s.family == "open_floor",
        "diag_all": lambda s: s.family == "competition_diag",
        "aux_all": lambda s: s.family == "competition_aux",
    }
    risk_metrics = []
    for name, pred in risk_specs.items():
        subset = [sample for sample in samples if pred(sample)]
        if subset:
            risk_metrics.append(metric_row(name, subset, constants, best))

    common_columns = {
        "run_id": [sample.run_id for sample in samples],
        "forward_velocity_mps": [sample.forward_mps for sample in samples],
        "yaw_rate_radps": [sample.yaw_rate for sample in samples],
        "limiter_active": [sample.limiter for sample in samples],
        "hardware_saturation_evidence": [sample.saturation for sample in samples],
    }
    common_baseline_errors = [sample.baseline_moment - sample.observed_moment for sample in samples]
    common_candidate_errors = [predict(sample, constants, best) - sample.observed_moment for sample in samples]
    common_range_metrics = write_common_range_metrics(
        OUT / f"{args.artifact_prefix}common_range_metrics.csv",
        common_columns,
        common_baseline_errors,
        common_candidate_errors,
        "standalone_contact_traction",
    )

    run_refs = reference_by_run()
    selected_metrics = []
    for run_id in SELECTED_LOGS:
        subset = [sample for sample in samples if sample.run_id == run_id]
        if not subset:
            selected_metrics.append({"run_id": run_id, "present": False})
            continue
        row = metric_row(run_id, subset, constants, best)
        row["run_id"] = run_id
        row["present"] = True
        row["dataset_split"] = subset[0].split
        row["family"] = subset[0].family
        row.update(run_refs.get(run_id, {}))
        selected_metrics.append(row)

    coeff_names = {
        "direct_patch_rational": ["drive_scale", "lateral_dynamic_mu", "longitudinal_dynamic_mu"],
        "yaw_static_yield_patch": ["drive_scale", "lateral_dynamic_mu", "lateral_static_yield_mu", "longitudinal_dynamic_mu"],
    }[best.name]
    parameter_rows = [
        {"parameter": "family", "value": best.name},
        {"parameter": "dyn_k_mps", "value": best.dyn_k_mps},
        {"parameter": "static_k_mps", "value": best.static_k_mps},
        {"parameter": "static_fwd_k_mps", "value": best.static_fwd_k_mps},
    ]
    parameter_rows.extend({"parameter": name, "value": value} for name, value in zip(coeff_names, best.coeffs))

    launch_rows = [
        {
            "family": best.name,
            "yaw_rate_radps": 1.0,
            "forward_velocity_mps": 0.0,
            "right_velocity_mps": 0.0,
            "opposing_scrub_nm": best.launch_opposing_nm,
            "left_command": best.launch_left_command,
            "right_command": best.launch_right_command,
            "max_abs_command": max(abs(best.launch_left_command), abs(best.launch_right_command)),
            "gate_pass": best.launch_gate_pass,
            "launch_lock_policy": "diagnostic_only",
        }
    ]

    prediction_rows = []
    stride = max(len(samples) // 500, 1)
    for sample in samples[::stride][:500]:
        pred = predict(sample, constants, best)
        prediction_rows.append(
            {
                "run_id": sample.run_id,
                "dataset_split": sample.split,
                "row_index": sample.row_index,
                "family": sample.family,
                "phase": sample.phase,
                "forward_velocity_mps": sample.forward_mps,
                "yaw_rate_radps": sample.yaw_rate,
                "observed_yaw_moment_nm": sample.observed_moment,
                "baseline_yaw_moment_nm": sample.baseline_moment,
                "standalone_yaw_moment_nm": pred,
                "baseline_error_nm": sample.baseline_moment - sample.observed_moment,
                "standalone_error_nm": pred - sample.observed_moment,
            }
        )

    prefix = args.artifact_prefix
    write_csv(OUT / f"{prefix}candidate_scores.csv", candidate_rows)
    write_csv(OUT / f"{prefix}split_metrics.csv", split_metrics)
    write_csv(OUT / f"{prefix}selected_log_metrics.csv", selected_metrics)
    write_csv(OUT / f"{prefix}risk_metrics.csv", risk_metrics)
    write_csv(OUT / f"{prefix}selected_parameters.csv", parameter_rows)
    write_csv(OUT / f"{prefix}in_place_1radps_command.csv", launch_rows)
    write_csv(OUT / f"{prefix}prediction_sample.csv", prediction_rows)
    report_name = f"{prefix}standalone_contact_traction_report.md" if prefix else "standalone_contact_traction_report.md"
    make_report(
        constants,
        best,
        candidate_rows,
        split_metrics,
        selected_metrics,
        risk_metrics,
        common_range_metrics,
        OUT / report_name,
    )


if __name__ == "__main__":
    main()
