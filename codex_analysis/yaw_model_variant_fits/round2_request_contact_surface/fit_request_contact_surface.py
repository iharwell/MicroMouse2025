#!/usr/bin/env python3
"""Round-2 requested-yaw/contact-velocity surface fit.

Analysis-only tooling. This script reads existing analysis artifacts and writes
outputs only beside this file. It does not modify production code, build
metadata, or tests.
"""

from __future__ import annotations

import csv
import importlib.util
import json
import math
import sys
from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent
COMBINED_SCRIPT = (
    ROOT
    / "codex_analysis"
    / "yaw_model_variant_fits"
    / "combined_slip_surface"
    / "fit_combined_slip_surface.py"
)
GRID_SCRIPT = (
    ROOT
    / "codex_analysis"
    / "yaw_model_variant_fits"
    / "lr_delta_grid"
    / "estimate_lr_delta_grid.py"
)
B_COEFFS = (
    ROOT
    / "codex_analysis"
    / "yaw_model_variant_fits"
    / "stribeck_scrub"
    / "stribeck_coefficients.csv"
)
B_SPLITS = (
    ROOT
    / "codex_analysis"
    / "yaw_model_variant_fits"
    / "stribeck_scrub"
    / "metrics_by_split.csv"
)
B_SELECTED = (
    ROOT
    / "codex_analysis"
    / "yaw_model_variant_fits"
    / "stribeck_scrub"
    / "metrics_by_selected_run.csv"
)
C_SPLITS = (
    ROOT
    / "codex_analysis"
    / "yaw_model_variant_fits"
    / "combined_slip_surface"
    / "split_metrics.csv"
)
C_SELECTED = (
    ROOT
    / "codex_analysis"
    / "yaw_model_variant_fits"
    / "combined_slip_surface"
    / "selected_log_metrics.csv"
)

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


def load_module(name: str, path: Path) -> Any:
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


cc = load_module("combined_slip_surface_reference", COMBINED_SCRIPT)
grid_ref = load_module("lr_delta_grid_reference", GRID_SCRIPT)


def read_key_value_csv(path: Path) -> dict[str, float]:
    with path.open(newline="", encoding="utf-8") as fh:
        return {row["parameter"]: float(row["value"]) for row in csv.DictReader(fh)}


def read_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as fh:
        return list(csv.DictReader(fh))


B = read_key_value_csv(B_COEFFS)
MECHANISM_CONSTANTS = cc.load_constants()
HALF_TRACK = 0.5 * MECHANISM_CONSTANTS.get("track_width_m", 0.084635)
FRONT_F = MECHANISM_CONSTANTS.get("drive_wheel_longitudinal_offset_m", 0.01475)
CONTACTS = {
    "fl": (-HALF_TRACK, FRONT_F),
    "fr": (HALF_TRACK, FRONT_F),
    "rl": (-HALF_TRACK, -FRONT_F),
    "rr": (HALF_TRACK, -FRONT_F),
}
HARD_GATE_COMMAND = 0.60


def f(row: dict[str, object], key: str, default: float = 0.0) -> float:
    return cc.f(row, key, default)


def sign(value: float, eps: float = 1.0e-6) -> float:
    if value > eps:
        return 1.0
    if value < -eps:
        return -1.0
    return 0.0


def smooth_positive(value: float, epsilon: float = 1.0e-6) -> float:
    return 0.5 * (value + math.sqrt(value * value + epsilon * epsilon))


def signed_contact_yaw_moment(row: dict[str, object], stem: str) -> float:
    moment = 0.0
    for name, (r_pos, f_pos) in CONTACTS.items():
        moment += f_pos * f(row, f"{name}_{stem}_r_n") - r_pos * f(row, f"{name}_{stem}_f_n")
    return moment


def force_activation(row: dict[str, object]) -> float:
    force_authority = f(row, "applied_yaw_force_authority_nm")
    scale = max(B["req_activation_nm"], 1.0e-9)
    return 1.0 - math.exp(-((force_authority / scale) ** 2))


def launch_core_opposes(row: dict[str, object]) -> float:
    transition = math.hypot(
        B["rel_weight"] * f(row, "vbar_rel_mps"),
        f(row, "abs_forward_velocity_mps"),
    )
    stribeck = math.exp(-((transition / B["stribeck_speed_mps"]) ** 2))
    relief = 1.0 / (1.0 + (transition / B["speed_fade_mps"]) ** 2)
    return force_activation(row) * relief * (
        B["static_extra_nm"] * stribeck + B["sliding_nm"]
    )


def target_opposes(row: dict[str, object]) -> float:
    return -f(row, "sign_yaw") * f(row, "residual_additive_yaw_torque_nm")


def contact_creep_moment_opposes(row: dict[str, object], velocity_knee_mps: float) -> float:
    """Load-weighted bounded contact-friction moment.

    This is the new moving-contact mechanism: each contact gets a normalized
    traction proxy `N_i * tanh(v_rel / v_k)` opposing local slip, then those
    local tractions are projected into yaw moment.
    """

    yaw_moment = 0.0
    knee = max(velocity_knee_mps, 1.0e-9)
    for name, (r_pos, f_pos) in CONTACTS.items():
        normal = max(f(row, f"{name}_normal_n"), 0.0)
        slip_f = f(row, f"{name}_v_rel_f_mps")
        slip_r = f(row, f"{name}_v_rel_r_mps")
        force_f = -normal * math.tanh(slip_f / knee)
        force_r = -normal * math.tanh(slip_r / knee)
        yaw_moment += f_pos * force_r - r_pos * force_f
    return -f(row, "sign_yaw") * yaw_moment


def prepare_rows(rows: list[dict[str, object]]) -> None:
    for row in rows:
        yaw_sign = sign(f(row, "yaw_rate_radps"))
        if yaw_sign == 0.0:
            yaw_sign = sign(f(row, "patch_yaw_req_basis_nm"))
        if yaw_sign == 0.0:
            yaw_sign = 1.0
        row["sign_yaw"] = yaw_sign
        row["abs_forward_velocity_mps"] = abs(f(row, "forward_velocity_mps"))
        row["abs_yaw_rate_radps"] = abs(f(row, "yaw_rate_radps"))
        applied_moment = signed_contact_yaw_moment(row, "req")
        actual_moment = signed_contact_yaw_moment(row, "force")
        row["applied_yaw_force_moment_nm"] = applied_moment
        row["applied_yaw_force_authority_nm"] = abs(applied_moment)
        row["actual_contact_yaw_moment_nm"] = actual_moment
        row["actual_contact_yaw_authority_nm"] = abs(actual_moment)
        row["force_gap_yaw_authority_nm"] = abs(applied_moment - actual_moment)
        row["contact_creep_tight_nm"] = contact_creep_moment_opposes(row, 0.035)
        row["contact_creep_wide_nm"] = contact_creep_moment_opposes(row, 0.120)
        row["launch_core_opposes_nm"] = launch_core_opposes(row)
        row["target_opposes_nm"] = target_opposes(row)


def fit_weight_rows(rows: list[dict[str, object]]) -> list[float]:
    base: list[float] = []
    for row in rows:
        split = str(row.get("dataset_split", ""))
        rec = str(row.get("recommendation", ""))
        family = str(row.get("family", ""))
        weight = 0.0
        if split == "primary_open_floor_fit_authoritative":
            weight = 1.0
        elif (
            split == "open_floor_fit_downweighted"
            and rec == "fit_downweighted"
            and family == "open_floor"
        ):
            weight = 0.25

        limiter = min(max(f(row, "max_force_limiter_activity"), 0.0), 1.0)
        saturation = min(max(f(row, "hardware_saturation_evidence"), 0.0), 1.0)
        spike = min(max(f(row, "gyro_derivative_spike"), 0.0), 1.0)
        quality = (1.0 / (1.0 + 4.0 * limiter)) * (1.0 - 0.75 * saturation) * (
            1.0 - 0.75 * spike
        )
        if f(row, "abs_yaw_rate_radps") < 0.02 and f(row, "vbar_yaw_mps") < 0.002:
            quality *= 0.25
        base.append(weight * max(min(quality, 1.0), 0.02))

    counts = Counter(str(row["run_id"]) for row, weight in zip(rows, base) if weight > 0.0)
    weights: list[float] = []
    for row, weight in zip(rows, base):
        if weight <= 0.0:
            weights.append(0.0)
        else:
            weights.append(weight / math.sqrt(max(counts[str(row["run_id"])], 1)))
    total = sum(weights)
    positive = sum(1 for weight in weights if weight > 0.0)
    if total > 0.0:
        weights = [weight * positive / total for weight in weights]
    return weights


def schedule_values(
    row: dict[str, object],
    vrel_knee: float,
    fwd_knee: float,
) -> dict[str, float]:
    vrel = max(f(row, "vbar_rel_mps"), f(row, "load_weighted_rel_mps"), 0.0)
    vf = f(row, "abs_forward_velocity_mps")
    low_rel = 1.0 / (1.0 + (vrel / max(vrel_knee, 1.0e-9)) ** 2)
    low_forward = 1.0 / (1.0 + (vf / max(fwd_knee, 1.0e-9)) ** 2)
    req_act = force_activation(row)
    nonlaunch = max(0.0, 1.0 - low_rel * low_forward)
    gate = req_act * nonlaunch
    util = min(max(f(row, "max_force_preprojection_utilization"), 0.0), 5.0)
    limiter = min(max(f(row, "max_force_limiter_activity"), 0.0), 5.0)
    return {
        "gate": gate,
        "gate_low_rel": gate * low_rel,
        "gate_high_rel": gate * (1.0 - low_rel),
        "gate_low_forward": gate * low_forward,
        "gate_high_forward": gate * (1.0 - low_forward),
        "gate_util": gate * (util / (1.0 + util)),
        "gate_limiter": gate * (limiter / (1.0 + limiter)),
        "gate_yaw": gate * math.tanh(f(row, "abs_yaw_rate_radps") / 1.0),
    }


def feature_names() -> list[str]:
    return [
        "contact_creep_tight_nm__gate",
        "contact_creep_tight_nm__gate_util",
        "contact_creep_tight_nm__gate_high_forward",
        "contact_creep_wide_nm__gate",
        "contact_creep_wide_nm__gate_util",
        "applied_yaw_force_authority_nm__gate",
        "applied_yaw_force_authority_nm__gate_util",
        "applied_yaw_force_authority_nm__gate_high_forward",
        "actual_contact_yaw_authority_nm__gate",
        "actual_contact_yaw_authority_nm__gate_util",
        "force_gap_yaw_authority_nm__gate_limiter",
        "actual_contact_yaw_moment_nm__gate_high_forward",
        "vbar_rel_mps__gate",
        "abs_yaw_rate_radps__gate_high_forward",
        "launch_core_opposes_nm__gate_high_forward",
    ]


def feature_value(
    row: dict[str, object],
    name: str,
    vrel_knee: float,
    fwd_knee: float,
) -> float:
    base, suffix = name.split("__", 1)
    schedules = schedule_values(row, vrel_knee, fwd_knee)
    if base == "surface_bias":
        raw = 1.0
    else:
        raw = f(row, base)
    return raw * schedules[suffix]


def solve_linear(a: list[list[float]], b: list[float]) -> list[float]:
    return cc.solve_linear(a, b)


@dataclass
class RequestContactModel:
    vrel_knee: float
    fwd_knee: float
    ridge: float
    names: list[str]
    scales: list[float]
    beta: list[float]
    clip_sigma: float = 8.0

    def surface_delta_opposes(self, row: dict[str, object]) -> float:
        total = 0.0
        for name, scale, beta in zip(self.names, self.scales, self.beta):
            value = feature_value(row, name, self.vrel_knee, self.fwd_knee)
            limit = self.clip_sigma * scale
            if value > limit:
                value = limit
            elif value < -limit:
                value = -limit
            total += beta * (value / scale)
        return total

    def predict_opposes(self, row: dict[str, object]) -> float:
        return launch_core_opposes(row) + self.surface_delta_opposes(row)


def fit_model(
    rows: list[dict[str, object]],
    weights: list[float],
    vrel_knee: float,
    fwd_knee: float,
    ridge: float,
) -> RequestContactModel:
    names = feature_names()
    train_pairs = [(row, weight) for row, weight in zip(rows, weights) if weight > 0.0]
    scales: list[float] = []
    train_vectors: list[list[float]] = []
    for row, _ in train_pairs:
        train_vectors.append([feature_value(row, name, vrel_knee, fwd_knee) for name in names])

    for idx in range(len(names)):
        values = [abs(vector[idx]) for vector in train_vectors]
        scale = cc.q(values, 0.80)
        if scale < 1.0e-12:
            scale = math.sqrt(sum(value * value for value in values) / len(values)) if values else 1.0
        scales.append(scale if scale > 1.0e-12 else 1.0)

    p = len(names)
    xtx = [[0.0 for _ in range(p)] for _ in range(p)]
    xty = [0.0 for _ in range(p)]
    for (row, weight), raw_x in zip(train_pairs, train_vectors):
        y = f(row, "target_opposes_nm") - launch_core_opposes(row)
        x: list[float] = []
        for value, scale in zip(raw_x, scales):
            limit = 8.0 * scale
            if value > limit:
                value = limit
            elif value < -limit:
                value = -limit
            x.append(value / scale)
        for i, xi in enumerate(x):
            weighted_xi = weight * xi
            xty[i] += weighted_xi * y
            for j, xj in enumerate(x):
                xtx[i][j] += weighted_xi * xj
    for i in range(p):
        xtx[i][i] += ridge
    beta = solve_linear(xtx, xty)
    return RequestContactModel(vrel_knee, fwd_knee, ridge, names, scales, beta)


def residual_lists(
    rows: list[dict[str, object]],
    model: RequestContactModel | None,
) -> tuple[list[float], list[float], list[float]]:
    baseline: list[float] = []
    corrected: list[float] = []
    predicted_raw: list[float] = []
    for row in rows:
        raw = f(row, "residual_additive_yaw_torque_nm")
        pred_opposes = model.predict_opposes(row) if model else 0.0
        pred_raw = -f(row, "sign_yaw") * pred_opposes
        baseline.append(raw)
        predicted_raw.append(pred_raw)
        corrected.append(raw - pred_raw)
    return baseline, corrected, predicted_raw


def metric_row(
    label: str,
    rows: list[dict[str, object]],
    model: RequestContactModel | None,
) -> dict[str, object]:
    baseline, corrected, predicted_raw = residual_lists(rows, model)
    weights = cc.run_balanced_weights(rows)
    baseline_rmse = cc.rmse(baseline)
    corrected_rmse = cc.rmse(corrected)
    baseline_mae = cc.mae(baseline)
    corrected_mae = cc.mae(corrected)
    return {
        "group": label,
        "count": len(rows),
        "run_count": len({row["run_id"] for row in rows}),
        "baseline_rmse_nm": baseline_rmse,
        "round2_corrected_rmse_nm": corrected_rmse,
        "baseline_mae_nm": baseline_mae,
        "round2_corrected_mae_nm": corrected_mae,
        "baseline_median_abs_nm": cc.median([abs(value) for value in baseline]),
        "round2_median_abs_nm": cc.median([abs(value) for value in corrected]),
        "baseline_signed_median_nm": cc.median(baseline),
        "round2_signed_median_nm": cc.median(corrected),
        "run_balanced_baseline_rmse_nm": cc.weighted_rmse(baseline, weights),
        "run_balanced_round2_rmse_nm": cc.weighted_rmse(corrected, weights),
        "prediction_rmse_nm": cc.rmse(predicted_raw),
        "rmse_improvement_pct": 100.0 * (baseline_rmse - corrected_rmse) / baseline_rmse
        if baseline_rmse > 0.0
        else 0.0,
    }


def rows_for_split(rows: list[dict[str, object]], split: str) -> list[dict[str, object]]:
    return [row for row in rows if str(row.get("dataset_split", "")) == split]


def rows_for_run(rows: list[dict[str, object]], run_id: str) -> list[dict[str, object]]:
    return [row for row in rows if str(row.get("run_id", "")) == run_id]


def model_objective(
    model: RequestContactModel,
    rows: list[dict[str, object]],
) -> dict[str, float]:
    validation = [row for row in rows if row.get("dataset_split") != "primary_open_floor_fit_authoritative"]
    straight = [
        row
        for row in rows
        if f(row, "abs_yaw_rate_radps") < 0.05 and f(row, "abs_forward_velocity_mps") >= 0.05
    ]
    low_speed_yaw = [
        row
        for row in rows
        if f(row, "abs_forward_velocity_mps") < 0.05 and f(row, "abs_yaw_rate_radps") >= 0.2
    ]
    val_metric = metric_row("validation_non_authoritative", validation, model)
    straight_metric = metric_row("straight_forward", straight, model)
    low_metric = metric_row("low_speed_yaw", low_speed_yaw, model)
    gate = in_place_command_row(model)
    gate_abs_command = min(abs(float(gate["left_command"])), abs(float(gate["right_command"])))
    hard_gate_pass = gate_abs_command >= HARD_GATE_COMMAND
    hard_gate_penalty = 0.0 if hard_gate_pass else 1.0 + (HARD_GATE_COMMAND - gate_abs_command)
    if gate_abs_command > 0.75:
        hard_gate_penalty += (gate_abs_command - 0.75) * 0.05
    objective = (
        float(val_metric["run_balanced_round2_rmse_nm"])
        + 0.35 * float(straight_metric["run_balanced_round2_rmse_nm"])
        + 0.20 * float(low_metric["run_balanced_round2_rmse_nm"])
        + hard_gate_penalty
    )
    return {
        "objective_score": objective,
        "validation_rb_rmse_nm": float(val_metric["run_balanced_round2_rmse_nm"]),
        "straight_rb_rmse_nm": float(straight_metric["run_balanced_round2_rmse_nm"]),
        "low_speed_yaw_rb_rmse_nm": float(low_metric["run_balanced_round2_rmse_nm"]),
        "in_place_left_command": float(gate["left_command"]),
        "in_place_right_command": float(gate["right_command"]),
        "hard_gate_min_abs_command": gate_abs_command,
        "hard_gate_pass": hard_gate_pass,
    }


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    fields: list[str] = []
    for row in rows:
        for key in row:
            if key not in fields:
                fields.append(key)
    with path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def reference_by_key(path: Path, key_column: str) -> dict[str, dict[str, str]]:
    return {row[key_column]: row for row in read_rows(path)}


def float_field(row: dict[str, str] | None, key: str) -> float:
    if not row:
        return math.nan
    try:
        return float(row.get(key, "nan"))
    except ValueError:
        return math.nan


def split_comparison_rows(
    rows: list[dict[str, object]],
    model: RequestContactModel,
) -> list[dict[str, object]]:
    b_by_split = reference_by_key(B_SPLITS, "dataset_split")
    c_by_split = reference_by_key(C_SPLITS, "group")
    splits = [
        "primary_open_floor_fit_authoritative",
        "open_floor_fit_downweighted",
        "open_floor_validation_only",
        "diag_validation_only",
        "aux_downweighted_validation",
    ]
    out: list[dict[str, object]] = []
    for split in splits:
        subset = rows_for_split(rows, split)
        metric = metric_row(split, subset, model)
        b = b_by_split.get(split)
        c = c_by_split.get(split)
        out.append(
            {
                "dataset_split": split,
                "count": metric["count"],
                "run_count": metric["run_count"],
                "baseline_rmse_nm": metric["baseline_rmse_nm"],
                "b_corrected_rmse_nm": float_field(b, "corrected_rmse_nm"),
                "c_corrected_rmse_nm": float_field(c, "corrected_rmse_nm"),
                "round2_corrected_rmse_nm": metric["round2_corrected_rmse_nm"],
                "baseline_mae_nm": metric["baseline_mae_nm"],
                "b_corrected_mae_nm": float_field(b, "corrected_mae_nm"),
                "c_corrected_mae_nm": float_field(c, "corrected_mae_nm"),
                "round2_corrected_mae_nm": metric["round2_corrected_mae_nm"],
                "round2_median_abs_nm": metric["round2_median_abs_nm"],
                "round2_signed_median_nm": metric["round2_signed_median_nm"],
                "round2_rmse_improvement_pct": metric["rmse_improvement_pct"],
            }
        )
    validation = [row for row in rows if row.get("dataset_split") != "primary_open_floor_fit_authoritative"]
    metric = metric_row("validation_non_authoritative", validation, model)
    c_val = c_by_split.get("validation_non_authoritative")
    out.append(
        {
            "dataset_split": "validation_non_authoritative",
            "count": metric["count"],
            "run_count": metric["run_count"],
            "baseline_rmse_nm": metric["baseline_rmse_nm"],
            "b_corrected_rmse_nm": math.nan,
            "c_corrected_rmse_nm": float_field(c_val, "corrected_rmse_nm"),
            "round2_corrected_rmse_nm": metric["round2_corrected_rmse_nm"],
            "baseline_mae_nm": metric["baseline_mae_nm"],
            "b_corrected_mae_nm": math.nan,
            "c_corrected_mae_nm": float_field(c_val, "corrected_mae_nm"),
            "round2_corrected_mae_nm": metric["round2_corrected_mae_nm"],
            "round2_median_abs_nm": metric["round2_median_abs_nm"],
            "round2_signed_median_nm": metric["round2_signed_median_nm"],
            "round2_rmse_improvement_pct": metric["rmse_improvement_pct"],
        }
    )
    return out


def selected_comparison_rows(
    rows: list[dict[str, object]],
    model: RequestContactModel,
) -> list[dict[str, object]]:
    b_by_run = reference_by_key(B_SELECTED, "run_id")
    c_by_run = reference_by_key(C_SELECTED, "run_id")
    out: list[dict[str, object]] = []
    for run_id in SELECTED_LOGS:
        subset = rows_for_run(rows, run_id)
        present = bool(subset)
        metric = metric_row(run_id, subset, model) if present else {}
        b = b_by_run.get(run_id)
        c = c_by_run.get(run_id)
        out.append(
            {
                "run_id": run_id,
                "present": present,
                "dataset_split": subset[0].get("dataset_split", "") if present else "",
                "count": metric.get("count", 0),
                "baseline_rmse_nm": metric.get("baseline_rmse_nm", math.nan),
                "b_corrected_rmse_nm": float_field(b, "corrected_rmse_nm"),
                "c_corrected_rmse_nm": float_field(c, "corrected_rmse_nm"),
                "round2_corrected_rmse_nm": metric.get("round2_corrected_rmse_nm", math.nan),
                "baseline_signed_median_nm": metric.get("baseline_signed_median_nm", math.nan),
                "b_signed_median_after_nm": float_field(b, "corrected_signed_median_nm"),
                "c_signed_median_after_nm": float_field(c, "corrected_signed_median_nm"),
                "round2_signed_median_nm": metric.get("round2_signed_median_nm", math.nan),
                "round2_median_abs_nm": metric.get("round2_median_abs_nm", math.nan),
            }
        )
    return out


def constants() -> dict[str, float]:
    return grid_ref.read_constants()


def synthetic_row(
    constants_map: dict[str, float],
    vf_mps: float,
    yaw_rate: float,
    extra_opposes: float,
) -> dict[str, object]:
    base = grid_ref.baseline_opposing_yaw_torque(constants_map, yaw_rate)
    total_req = base + extra_opposes
    longitudinal = constants_map["drive_wheel_longitudinal_offset_m"]
    v_rel = longitudinal * abs(yaw_rate)
    util = grid_ref.contact_utilization(constants_map, yaw_rate)
    limiter = grid_ref.limiter_activity(constants_map, yaw_rate)
    right_basis = 2.0 * longitudinal * longitudinal * abs(yaw_rate)
    cmd = grid_ref.motor_commands_for_opposing_torque(base + extra_opposes, constants_map, vf_mps, yaw_rate)
    row = {
        "run_id": "synthetic_grid",
        "dataset_split": "synthetic",
        "forward_velocity_mps": vf_mps,
        "abs_forward_velocity_mps": abs(vf_mps),
        "yaw_rate_radps": yaw_rate,
        "abs_yaw_rate_radps": abs(yaw_rate),
        "sign_yaw": sign(yaw_rate) or 1.0,
        "vbar_rel_mps": v_rel,
        "vbar_yaw_mps": v_rel,
        "load_weighted_rel_mps": v_rel,
        "max_force_preprojection_utilization": util,
        "max_force_limiter_activity": limiter,
        "patch_yaw_req_basis_nm": total_req,
        "launch_core_opposes_nm": 0.0,
        "target_opposes_nm": 0.0,
        "gain_front_right_basis": right_basis,
        "gain_rear_right_basis": right_basis,
        "gain_left_long_basis": 0.0,
        "gain_right_long_basis": 0.0,
        "req_moment_opposes_yaw_nm": -total_req,
        "force_moment_opposes_yaw_nm": -extra_opposes,
        "force_gap_opposes_yaw_nm": -base,
        "req_abs_contact_moment_nm": abs(total_req),
        "force_abs_contact_moment_nm": abs(extra_opposes),
        "left_command": cmd["left_command"],
        "right_command": cmd["right_command"],
        "left_applied_torque_nm": cmd["left_command_torque_nm"],
        "right_applied_torque_nm": cmd["right_command_torque_nm"],
    }
    req_contact_force = total_req / (2.0 * constants_map["track_width_m"])
    force_contact_force = extra_opposes / (2.0 * constants_map["track_width_m"])
    for name, (r_pos, f_pos) in CONTACTS.items():
        row[f"{name}_normal_n"] = 0.25 * 1.932931008
        side = 1.0 if r_pos < 0.0 else -1.0
        row[f"{name}_req_f_n"] = side * req_contact_force
        row[f"{name}_req_r_n"] = 0.0
        row[f"{name}_force_f_n"] = side * force_contact_force
        row[f"{name}_force_r_n"] = 0.0
        row[f"{name}_v_rel_f_mps"] = -yaw_rate * r_pos
        row[f"{name}_v_rel_r_mps"] = -yaw_rate * f_pos
    applied_moment = signed_contact_yaw_moment(row, "req")
    actual_moment = signed_contact_yaw_moment(row, "force")
    row["applied_yaw_force_moment_nm"] = applied_moment
    row["applied_yaw_force_authority_nm"] = abs(applied_moment)
    row["actual_contact_yaw_moment_nm"] = actual_moment
    row["actual_contact_yaw_authority_nm"] = abs(actual_moment)
    row["force_gap_yaw_authority_nm"] = abs(applied_moment - actual_moment)
    row["contact_creep_tight_nm"] = contact_creep_moment_opposes(row, 0.035)
    row["contact_creep_wide_nm"] = contact_creep_moment_opposes(row, 0.120)
    row["launch_core_opposes_nm"] = launch_core_opposes(row)
    return row


def round2_extra(
    model: RequestContactModel,
    constants_map: dict[str, float],
    vf_mps: float,
    yaw_rate: float,
) -> float:
    extra = 0.0
    for _ in range(60):
        row = synthetic_row(constants_map, vf_mps, yaw_rate, extra)
        next_extra = model.predict_opposes(row)
        if abs(next_extra - extra) < 1.0e-12:
            return next_extra
        extra = 0.65 * extra + 0.35 * next_extra
    return extra


def command_row_for_variant(
    constants_map: dict[str, float],
    variant: str,
    vf_mps: float,
    yaw_rate: float,
    extra: float,
) -> dict[str, object]:
    base = grid_ref.baseline_opposing_yaw_torque(constants_map, yaw_rate)
    total = base + extra
    cmd = grid_ref.motor_commands_for_opposing_torque(total, constants_map, vf_mps, yaw_rate)
    min_abs_command = min(abs(cmd["left_command"]), abs(cmd["right_command"]))
    return {
        "vf_mps": vf_mps,
        "yaw_rate_radps": yaw_rate,
        "variant": variant,
        "baseline_opposing_yaw_torque_nm": base,
        "extra_opposing_yaw_torque_nm": extra,
        "total_opposing_yaw_torque_nm": total,
        "required_applied_bank_torque_nm": cmd["applied_bank_torque_nm"],
        "left_command": cmd["left_command"],
        "right_command": cmd["right_command"],
        "lr_delta_command": cmd["lr_delta_command"],
        "max_abs_command": max(abs(cmd["left_command"]), abs(cmd["right_command"])),
        "hard_gate_min_abs_command": min_abs_command,
        "hard_gate_pass": min_abs_command >= HARD_GATE_COMMAND,
        "command_outside_unit": max(abs(cmd["left_command"]), abs(cmd["right_command"])) > 1.0,
        "contact_utilization_raw": grid_ref.contact_utilization(constants_map, yaw_rate),
        "limiter_activity_proxy": grid_ref.limiter_activity(constants_map, yaw_rate),
    }


def in_place_command_row(model: RequestContactModel) -> dict[str, object]:
    constants_map = constants()
    yaw_rate = 1.0
    vf_mps = 0.0
    extra = round2_extra(model, constants_map, vf_mps, yaw_rate)
    return command_row_for_variant(constants_map, "Round2_force_contact_surface", vf_mps, yaw_rate, extra)


def write_in_place(model: RequestContactModel) -> None:
    constants_map = constants()
    yaw_rate = 1.0
    vf_mps = 0.0
    base = grid_ref.baseline_opposing_yaw_torque(constants_map, yaw_rate)
    rows = [
        command_row_for_variant(constants_map, "Current baseline", vf_mps, yaw_rate, 0.0),
        command_row_for_variant(
            constants_map,
            "Variant B Stribeck scrub",
            vf_mps,
            yaw_rate,
            grid_ref.variant_b_extra(base, constants_map, vf_mps, yaw_rate),
        ),
        command_row_for_variant(
            constants_map,
            "Variant C combined slip",
            vf_mps,
            yaw_rate,
            grid_ref.variant_c_extra(base, constants_map, vf_mps, yaw_rate),
        ),
        in_place_command_row(model),
    ]
    write_csv(OUT / "in_place_1radps_command_estimate.csv", rows)


def vf_grid() -> list[float]:
    return grid_ref.vf_grid()


def yaw_grid() -> list[float]:
    return grid_ref.yaw_grid()


def write_lr_grid(model: RequestContactModel) -> None:
    constants_map = constants()
    rows: list[dict[str, object]] = []
    for vf_mps in vf_grid():
        for yaw_rate in yaw_grid():
            base = grid_ref.baseline_opposing_yaw_torque(constants_map, yaw_rate)
            rows.append(command_row_for_variant(constants_map, "Baseline", vf_mps, yaw_rate, 0.0))
            rows.append(
                command_row_for_variant(
                    constants_map,
                    "B_stribeck",
                    vf_mps,
                    yaw_rate,
                    grid_ref.variant_b_extra(base, constants_map, vf_mps, yaw_rate),
                )
            )
            rows.append(
                command_row_for_variant(
                    constants_map,
                    "C_combined_slip",
                    vf_mps,
                    yaw_rate,
                    grid_ref.variant_c_extra(base, constants_map, vf_mps, yaw_rate),
                )
            )
            rows.append(
                command_row_for_variant(
                    constants_map,
                    "Round2_force_contact_surface",
                    vf_mps,
                    yaw_rate,
                    round2_extra(model, constants_map, vf_mps, yaw_rate),
                )
            )
    write_csv(OUT / "lr_delta_grid.csv", rows)
    (OUT / "lr_delta_pivot.md").write_text("\n".join(pivot_lines(rows)) + "\n", encoding="utf-8")


def pivot_lines(rows: list[dict[str, object]]) -> list[str]:
    variants = ["Baseline", "B_stribeck", "C_combined_slip", "Round2_force_contact_surface"]
    yaws = yaw_grid()
    by_key = {(row["variant"], row["vf_mps"], row["yaw_rate_radps"]): row for row in rows}
    lines = [
        "# Round-2 L/R Command Delta Grid",
        "",
        "Values are `left_cmd - right_cmd` for positive clockwise yaw. Yaw columns are rad/s; `Vf` rows are m/s.",
        "",
    ]
    for variant in variants:
        lines.append(f"## {variant}")
        lines.append("")
        lines.append("| Vf \\ yaw | " + " | ".join(f"{yaw:.3g}" for yaw in yaws) + " |")
        lines.append("| ---: | " + " | ".join("---:" for _ in yaws) + " |")
        for vf_mps in vf_grid():
            values = [
                f"{float(by_key[(variant, vf_mps, yaw)]['lr_delta_command']):.3f}"
                for yaw in yaws
            ]
            lines.append(f"| {vf_mps:.3f} | " + " | ".join(values) + " |")
        lines.append("")
    saturated = sum(1 for row in rows if row["command_outside_unit"])
    projection = sum(1 for row in rows if float(row["contact_utilization_raw"]) > 1.0)
    lines.extend(
        [
            "## Flags",
            "",
            f"- Rows with `|cmd| > 1`: {saturated} of {len(rows)}.",
            f"- Rows where raw contact utilization exceeds 1.0: {projection} of {len(rows)}.",
            "",
            "Full per-point commands and torque quantities are in `lr_delta_grid.csv`.",
        ]
    )
    return lines


def write_coefficients(model: RequestContactModel) -> None:
    rows: list[dict[str, object]] = [
        {
            "component": "fixed_launch_core",
            "feature": "force_activation_knee_nm",
            "value": B["req_activation_nm"],
            "unit": "Nm",
        },
        {
            "component": "fixed_launch_core",
            "feature": "stribeck_speed_mps",
            "value": B["stribeck_speed_mps"],
            "unit": "m/s",
        },
        {
            "component": "fixed_launch_core",
            "feature": "speed_fade_mps",
            "value": B["speed_fade_mps"],
            "unit": "m/s",
        },
        {
            "component": "fixed_launch_core",
            "feature": "rel_weight",
            "value": B["rel_weight"],
            "unit": "dimensionless",
        },
        {
            "component": "fixed_launch_core",
            "feature": "static_extra_nm",
            "value": B["static_extra_nm"],
            "unit": "Nm",
        },
        {
            "component": "fixed_launch_core",
            "feature": "sliding_nm",
            "value": B["sliding_nm"],
            "unit": "Nm",
        },
        {
            "component": "surface_hyperparameter",
            "feature": "vrel_knee_mps",
            "value": model.vrel_knee,
            "unit": "m/s",
        },
        {
            "component": "surface_hyperparameter",
            "feature": "fwd_knee_mps",
            "value": model.fwd_knee,
            "unit": "m/s",
        },
        {
            "component": "surface_hyperparameter",
            "feature": "ridge",
            "value": model.ridge,
            "unit": "dimensionless",
        },
    ]
    for name, scale, beta in sorted(
        zip(model.names, model.scales, model.beta),
        key=lambda item: abs(item[2]),
        reverse=True,
    ):
        rows.append(
            {
                "component": "fitted_surface",
                "feature": name,
                "value": beta,
                "unit": "Nm standardized coefficient",
                "feature_scale": scale,
                "raw_coefficient_nm_per_feature": beta / scale,
                "abs_standardized_coefficient_nm": abs(beta),
            }
        )
    write_csv(OUT / "request_contact_surface_coefficients.csv", rows)


def write_tuning(rows: list[dict[str, object]]) -> None:
    write_csv(OUT / "candidate_tuning_scores.csv", rows)


def fmt(value: object, digits: int = 6) -> str:
    if isinstance(value, bool):
        return str(value)
    try:
        x = float(value)
    except (TypeError, ValueError):
        return str(value)
    if not math.isfinite(x):
        return "nan"
    return f"{x:.{digits}f}"


def markdown_table(rows: list[dict[str, object]], columns: list[str], limit: int | None = None) -> list[str]:
    selected = rows if limit is None else rows[:limit]
    lines = ["| " + " | ".join(columns) + " |"]
    lines.append("| " + " | ".join("---" for _ in columns) + " |")
    for row in selected:
        lines.append("| " + " | ".join(fmt(row.get(col, "")) for col in columns) + " |")
    return lines


def write_failure_modes() -> None:
    lines = [
        "# Round-2 Force/Contact Surface Failure Modes",
        "",
        "- The fixed launch core preserves the B-scale in-place breakaway point, but the fitted surface is still a residual correction. It should not be treated as a final physical tire law without force-replay validation.",
        f"- The model is rejected as a viable candidate if the +1 rad/s in-place command is below `{HARD_GATE_COMMAND:.2f}` in absolute left/right command, regardless of broad RMSE.",
        "- The moving-contact term is materially different from the first-round low-order ridge/surface attempts because it projects bounded per-contact `N*tanh(v_rel/v_k)` traction proxies into yaw moment before fitting branch gains. If future work collapses this back to arbitrary low-order features, it should be treated as a different and weaker model.",
        "- Synthetic grid evaluation uses the same approximate contact replay as the prior L/R grid: no full PlantModel projection is replayed after the residual correction changes applied tire-force demand.",
        "- Lateral body velocity is unavailable in the source logs; right-relative contact velocity assumes `Vr=0`, so lateral scrub terms are reconstruction features.",
        "- Rows with limiter or hardware-saturation evidence are downweighted, not excluded. The surface may still learn some projection artifacts.",
        "- The contact surface is launch-gated to avoid erasing the B breakaway authority. That means it may underfit real low-speed sliding regimes that are neither static launch nor forward-speed-dominated.",
        "- The target is differentiated gyro yaw acceleration, so single-row residuals contain timing jitter and derivative noise. Aggregate split/run metrics are more meaningful than individual samples.",
        "- The model uses force activation and continuous schedules, not command values or maneuver labels. It can still be biased by the open-floor coverage distribution because selected maneuvers dominate the available data.",
    ]
    (OUT / "failure_modes.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_report(
    rows: list[dict[str, object]],
    model: RequestContactModel,
    tuning_rows: list[dict[str, object]],
    split_rows_out: list[dict[str, object]],
    selected_rows_out: list[dict[str, object]],
) -> None:
    in_place = read_rows(OUT / "in_place_1radps_command_estimate.csv")
    round2_in_place = [row for row in in_place if row["variant"] == "Round2_force_contact_surface"][0]
    coeffs = read_rows(OUT / "request_contact_surface_coefficients.csv")
    fitted = [row for row in coeffs if row["component"] == "fitted_surface"]
    fitted.sort(key=lambda row: abs(float(row["value"])), reverse=True)

    lines = [
        "# Round-2 Force-Coupled Contact-Velocity Surface",
        "",
        "Analysis-only output. Production code, build metadata, and tests were not modified.",
        "",
        "## Model Family",
        "",
        "The selected family is not another low-order residual ridge over A/C/D-style features. The new mechanism is a two-branch yaw-friction law:",
        "",
        "1. a force-activated presliding/static reserve that is live before yaw motion develops, and",
        "2. a moving-contact branch that projects bounded per-contact slip tractions into yaw moment.",
        "",
        "The moving-contact branch uses each contact's normal load and relative velocity through `N_i * tanh(v_rel_i / v_k)`, so it behaves like a saturated contact-friction/creep mechanism rather than an unbounded polynomial or residual table. The fit tunes branch gains and schedule knees; it does not introduce maneuver labels or a grid lookup.",
        "",
        "The model equations are:",
        "",
        "`R_total = R_launch + DeltaR_surface`",
        "",
        "`R_launch = A_force * F_speed * (K_slide + K_static * exp(-(v_transition / v_s)^2))`",
        "",
        "`A_force = 1 - exp(-(abs(M_applied_contact_force) / M_force_knee)^2)`",
        "",
        "`v_transition = sqrt((rel_weight * vbar_rel)^2 + abs(Vf)^2)`",
        "",
        "`F_speed = 1 / (1 + (v_transition / v_fade)^2)`",
        "",
        "`F_contact_i = -N_i * [tanh(v_rel_f_i / v_f_k), tanh(v_rel_r_i / v_r_k)]`",
        "",
        "`M_contact_opp = -sign(yaw) * sum_i (f_i * F_contact_r_i - r_i * F_contact_f_i)`",
        "",
        "`DeltaR_surface = G_move * (beta_c * M_contact_opp + beta_f * abs(M_applied_contact_force) + beta_gap * abs(M_force_gap) + beta_v * vbar_rel + ...)`",
        "",
        "`G_move = A_force * (1 - low_rel * low_forward)`",
        "",
        "The fitted terms are branch gains on contact-creep, applied contact-force moment, force-gap, yaw-rate, forward-speed, and utilization components. Command values are not features and do not select a traction mode; if two rows have the same contact state, relative velocities, loads, and contact-force inputs, the model returns the same resistance.",
        "",
        "Rejected design note: a command/request-gated surface would violate the current rule because identical contact state and tire/contact forces could produce different resistance solely from command metadata. This report excludes that design; motor command is used only after prediction to estimate whether the +1 rad/s command gate is satisfied.",
        "",
        "The additive yaw-torque correction applied to the residual convention is `M_corr = -sign(yaw_rate_or_applied_force) * R_total`.",
        "",
        "## Fit Basis",
        "",
        f"- Rows evaluated: {len(rows)}",
        f"- Training rows with nonzero fit weight: {sum(1 for row in rows if row.get('fit_weight', 0.0) > 0.0)}",
        "- Primary fit rows carry full weight; open-floor `fit_downweighted` rows carry 0.25 base weight so May 4 yaw-launch evidence remains visible.",
        "- No maneuver labels are used as features.",
        "",
        "## Selected Hyperparameters",
        "",
        f"- `vrel_knee_mps`: {model.vrel_knee:.6f}",
        f"- `fwd_knee_mps`: {model.fwd_knee:.6f}",
        f"- `ridge`: {model.ridge:g}",
        f"- fixed launch `K_static`: {B['static_extra_nm']:.9f} Nm",
        f"- fixed launch `K_slide`: {B['sliding_nm']:.9f} Nm",
        "",
        "## +1 rad/s In-Place Command",
        "",
        "Hard gate result for positive clockwise +1 rad/s, zero forward speed:",
        "",
        "| Variant | Extra opposing Nm | Total opposing Nm | Left cmd | Right cmd |",
        "| --- | ---: | ---: | ---: | ---: |",
    ]
    for row in in_place:
        lines.append(
            f"| {row['variant']} | {fmt(row['extra_opposing_yaw_torque_nm'])} | "
            f"{fmt(row['total_opposing_yaw_torque_nm'])} | {fmt(row['left_command'], 3)} | "
            f"{fmt(row['right_command'], 3)} |"
        )
    lines.extend(
        [
            "",
            f"Acceptance gate: `min(abs(left_cmd), abs(right_cmd)) >= {HARD_GATE_COMMAND:.3f}` at `Vf=0`, `Vr=0`, `yaw_rate=+1 rad/s`.",
            f"Round-2 maps +1 rad/s in-place to `{float(round2_in_place['left_command']):.3f}/{float(round2_in_place['right_command']):.3f}`; pass/fail = `{round2_in_place['hard_gate_pass']}`.",
            "",
            "## Split RMSE Versus B/C",
            "",
        ]
    )
    lines.extend(
        markdown_table(
            split_rows_out,
            [
                "dataset_split",
                "baseline_rmse_nm",
                "b_corrected_rmse_nm",
                "c_corrected_rmse_nm",
                "round2_corrected_rmse_nm",
                "round2_rmse_improvement_pct",
            ],
        )
    )
    lines.extend(["", "## Selected-Log RMSE Versus B/C", ""])
    lines.extend(
        markdown_table(
            selected_rows_out,
            [
                "run_id",
                "dataset_split",
                "baseline_rmse_nm",
                "b_corrected_rmse_nm",
                "c_corrected_rmse_nm",
                "round2_corrected_rmse_nm",
                "round2_signed_median_nm",
            ],
        )
    )
    lines.extend(["", "## Dominant Fitted Surface Coefficients", ""])
    lines.extend(
        markdown_table(
            fitted,
            [
                "feature",
                "value",
                "feature_scale",
                "raw_coefficient_nm_per_feature",
                "abs_standardized_coefficient_nm",
            ],
            limit=15,
        )
    )
    lines.extend(["", "## 6x10 Vf/Yaw L-R Delta Grid Summary", ""])
    lines.append("The full grid is in `lr_delta_grid.csv`; the pivot summary is in `lr_delta_pivot.md`. Key observation: the +1 rad/s in-place neighborhood stays near B, while the surface increasingly diverges from B as forward speed, contact-relative speed, yaw rate, and utilization rise.")
    lines.extend(["", "## Tuning Candidates", ""])
    lines.extend(
        markdown_table(
            tuning_rows,
            [
                "vrel_knee_mps",
                "fwd_knee_mps",
                "ridge",
                "objective_score",
                "validation_rb_rmse_nm",
                "straight_rb_rmse_nm",
                "low_speed_yaw_rb_rmse_nm",
                "in_place_left_command",
                "hard_gate_pass",
            ],
        )
    )
    lines.extend(
        [
            "",
            "## Failure Modes",
            "",
            "See `failure_modes.md` for the detailed list. The most important caveat is that this is still a residual fit over reconstructed contact features; production eligibility requires full force replay and targeted yaw-launch/low-speed-turn validation.",
            "",
            "## Output Files",
            "",
            "- `fit_request_contact_surface.py`",
            "- `request_contact_surface_report.md`",
            "- `request_contact_surface_coefficients.csv`",
            "- `candidate_tuning_scores.csv`",
            "- `split_rmse_vs_b_c.csv`",
            "- `selected_log_rmse_vs_b_c.csv`",
            "- `in_place_1radps_command_estimate.csv`",
            "- `lr_delta_grid.csv`",
            "- `lr_delta_pivot.md`",
            "- `failure_modes.md`",
            "- `commands_run.txt`",
        ]
    )
    (OUT / "request_contact_surface_report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    rows = cc.load_rows()
    prepare_rows(rows)
    weights = fit_weight_rows(rows)
    for row, weight in zip(rows, weights):
        row["fit_weight"] = weight

    candidates = [
        (0.08, 0.35, 0.01),
        (0.10, 0.35, 0.01),
        (0.10, 0.70, 0.01),
        (0.10, 0.35, 0.03),
    ]
    tuning_rows: list[dict[str, object]] = []
    best_model: RequestContactModel | None = None
    best_score = float("inf")
    for vrel_knee, fwd_knee, ridge in candidates:
        model = fit_model(rows, weights, vrel_knee, fwd_knee, ridge)
        objective = model_objective(model, rows)
        row = {
            "vrel_knee_mps": vrel_knee,
            "fwd_knee_mps": fwd_knee,
            "ridge": ridge,
            "feature_count": len(model.names),
            **objective,
        }
        tuning_rows.append(row)
        if objective["objective_score"] < best_score:
            best_score = objective["objective_score"]
            best_model = model
    assert best_model is not None
    tuning_rows.sort(key=lambda row: float(row["objective_score"]))

    write_coefficients(best_model)
    write_tuning(tuning_rows)
    split_rows_out = split_comparison_rows(rows, best_model)
    selected_rows_out = selected_comparison_rows(rows, best_model)
    write_csv(OUT / "split_rmse_vs_b_c.csv", split_rows_out)
    write_csv(OUT / "selected_log_rmse_vs_b_c.csv", selected_rows_out)
    write_in_place(best_model)
    write_lr_grid(best_model)
    write_failure_modes()
    write_report(rows, best_model, tuning_rows, split_rows_out, selected_rows_out)
    (OUT / "commands_run.txt").write_text(
        "python codex_analysis\\yaw_model_variant_fits\\round2_request_contact_surface\\fit_request_contact_surface.py\n",
        encoding="utf-8",
    )
    print((OUT / "request_contact_surface_report.md").as_posix())


if __name__ == "__main__":
    main()
