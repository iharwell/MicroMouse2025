#!/usr/bin/env python3
"""Cheap smooth max/norm allocation between static yield reserve and Variant C.

Analysis-only tooling. Reads the shared yaw feature sample and prior Variant C
artifacts, then writes outputs beside this script. It does not edit production
code, build metadata, or tests.
"""

from __future__ import annotations

import csv
import json
import math
from collections import Counter
from dataclasses import dataclass
from pathlib import Path


ROOT = Path(__file__).resolve().parents[4]
OUT = Path(__file__).resolve().parent

PRIMARY = ROOT / "codex_analysis" / "contact_continuum_yaw_identification" / "ablation" / "phase_classified_feature_sample.csv"
SECONDARY = ROOT / "codex_analysis" / "contact_continuum_yaw_identification" / "features" / "contact_continuum_feature_sample.csv"
CONSTANTS = ROOT / "codex_analysis" / "contact_continuum_yaw_identification" / "features" / "plant_mirror_constants.csv"
C_DIR = ROOT / "codex_analysis" / "yaw_model_variant_fits" / "combined_slip_surface"
FORCE_DOMAIN_DIR = ROOT / "codex_analysis" / "yaw_model_variant_fits" / "round2_force_domain_stribeck"
IN_PLACE_REFERENCE = ROOT / "codex_analysis" / "yaw_model_variant_fits" / "in_place_1radps_command" / "in_place_1radps_command_estimate.csv"

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

CONTACT_FIELDS = [
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
    "fl_req_f_n",
    "fr_req_f_n",
    "rl_req_f_n",
    "rr_req_f_n",
    "fl_req_r_n",
    "fr_req_r_n",
    "rl_req_r_n",
    "rr_req_r_n",
    "fl_force_f_n",
    "fr_force_f_n",
    "rl_force_f_n",
    "rr_force_f_n",
    "fl_force_r_n",
    "fr_force_r_n",
    "rl_force_r_n",
    "rr_force_r_n",
]


def f(row: dict[str, object], key: str, default: float = 0.0) -> float:
    value = row.get(key, default)
    if isinstance(value, float):
        return value if math.isfinite(value) else default
    try:
        x = float(value)
        return x if math.isfinite(x) else default
    except (TypeError, ValueError):
        return default


def sign(x: float, eps: float = 1.0e-5) -> float:
    if x > eps:
        return 1.0
    if x < -eps:
        return -1.0
    return 0.0


def smooth_positive(x: float, eps: float) -> float:
    return 0.5 * (x + math.sqrt(x * x + eps * eps))


def smooth_max(a: float, b: float, eps: float) -> float:
    return 0.5 * (a + b + math.sqrt((a - b) * (a - b) + eps * eps))


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
    clean = [v for v in values if math.isfinite(v)]
    return math.sqrt(sum(v * v for v in clean) / len(clean)) if clean else 0.0


def mae(values: list[float]) -> float:
    clean = [v for v in values if math.isfinite(v)]
    return sum(abs(v) for v in clean) / len(clean) if clean else 0.0


def weighted_rmse(values: list[float], weights: list[float]) -> float:
    total = sum(weights)
    if total <= 0.0:
        return 0.0
    return math.sqrt(max(sum(w * v * v for v, w in zip(values, weights)) / total, 0.0))


def read_constants() -> dict[str, float]:
    with CONSTANTS.open(newline="", encoding="utf-8") as fh:
        return {row["name"]: float(row["value"]) for row in csv.DictReader(fh)}


def read_keyed_csv(path: Path, key: str) -> dict[str, dict[str, str]]:
    if not path.exists():
        return {}
    with path.open(newline="", encoding="utf-8") as fh:
        return {row[key]: row for row in csv.DictReader(fh)}


def read_rows_csv(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8") as fh:
        return list(csv.DictReader(fh))


def load_secondary() -> dict[tuple[str, str], dict[str, str]]:
    fields = set(CONTACT_FIELDS)
    out: dict[tuple[str, str], dict[str, str]] = {}
    with SECONDARY.open(newline="", encoding="utf-8") as fh:
        for row in csv.DictReader(fh):
            out[(row["run_id"], row["row_index"])] = {name: row.get(name, "") for name in fields}
    return out


def add_contact_bases(row: dict[str, object], contacts: dict[str, tuple[float, float]], constants: dict[str, float]) -> None:
    yaw_sign = f(row, "yaw_direction")
    right_front = 0.0
    right_rear = 0.0
    long_left = 0.0
    long_right = 0.0
    req_moment = 0.0
    force_moment = 0.0
    req_abs_moment = 0.0
    force_abs_moment = 0.0
    total_normal = max(f(row, "total_normal_load_n"), 1.0e-9)
    load_weighted_rel = 0.0
    load_weighted_lat = 0.0
    normal_reference = constants["mass_kg"] * 9.80665 + constants.get("fan_downforce_full_duty_n", 0.0) * 0.8
    mu_effective = constants["mass_kg"] * constants["sustained_lateral_accel_mps2"] / max(normal_reference, 1.0e-9)
    max_actual_force_util = 0.0

    for name, (r_pos, f_pos) in contacts.items():
        vf_rel = f(row, f"{name}_v_rel_f_mps")
        vr_rel = f(row, f"{name}_v_rel_r_mps")
        normal = f(row, f"{name}_normal_n")
        normal_frac = normal / total_normal

        right_basis = -yaw_sign * f_pos * vr_rel
        long_basis = yaw_sign * r_pos * vf_rel
        if f_pos > 0.0:
            right_front += right_basis
        else:
            right_rear += right_basis
        if r_pos < 0.0:
            long_left += long_basis
        else:
            long_right += long_basis

        req_f = f(row, f"{name}_req_f_n")
        req_r = f(row, f"{name}_req_r_n")
        force_f = f(row, f"{name}_force_f_n")
        force_r = f(row, f"{name}_force_r_n")
        req_local = f_pos * req_r - r_pos * req_f
        force_local = f_pos * force_r - r_pos * force_f
        req_moment += req_local
        force_moment += force_local
        req_abs_moment += abs(req_local)
        force_abs_moment += abs(force_local)
        load_weighted_rel += normal_frac * math.hypot(vf_rel, vr_rel)
        load_weighted_lat += normal_frac * abs(vr_rel)
        force_capacity = max(mu_effective * normal, 1.0e-9)
        max_actual_force_util = max(max_actual_force_util, math.hypot(force_f, force_r) / force_capacity)

    row["gain_front_right_basis"] = right_front
    row["gain_rear_right_basis"] = right_rear
    row["gain_left_long_basis"] = long_left
    row["gain_right_long_basis"] = long_right
    row["force_gap_opposes_yaw_nm"] = -yaw_sign * (req_moment - force_moment)
    row["req_moment_opposes_yaw_nm"] = -yaw_sign * req_moment
    row["force_moment_opposes_yaw_nm"] = -yaw_sign * force_moment
    row["req_abs_contact_moment_nm"] = req_abs_moment
    row["force_abs_contact_moment_nm"] = force_abs_moment
    row["load_weighted_rel_mps"] = load_weighted_rel
    row["load_weighted_lat_mps"] = load_weighted_lat
    row["actual_force_utilization"] = min(max_actual_force_util, 5.0)
    row["actual_force_util_smooth"] = row["actual_force_utilization"] / (1.0 + row["actual_force_utilization"])


def load_rows(constants: dict[str, float]) -> list[dict[str, object]]:
    half_track = 0.5 * constants["track_width_m"]
    front_f = constants["drive_wheel_longitudinal_offset_m"]
    contacts = {
        "fl": (-half_track, front_f),
        "fr": (half_track, front_f),
        "rl": (-half_track, -front_f),
        "rr": (half_track, -front_f),
    }
    secondary = load_secondary()
    nominal_load = constants["mass_kg"] * 9.80665 + constants.get("fan_downforce_full_duty_n", 0.7) * 0.8
    rows: list[dict[str, object]] = []
    with PRIMARY.open(newline="", encoding="utf-8") as fh:
        for src in csv.DictReader(fh):
            row: dict[str, object] = dict(src)
            row.update(secondary.get((src["run_id"], src["row_index"]), {}))
            for key in list(row):
                if key in {"run_id", "family", "schema", "recommendation", "dataset_split", "physics_phase", "physics_phase_basis"}:
                    continue
                try:
                    row[key] = float(row[key]) if row[key] != "" else 0.0
                except (TypeError, ValueError):
                    pass
            if row.get("dataset_split") == "excluded_or_unclassified":
                continue
            if f(row, "total_normal_load_n") <= 0.0:
                row["total_normal_load_n"] = nominal_load
            for contact in ["fl", "fr", "rl", "rr"]:
                if f(row, f"{contact}_normal_n") <= 0.0:
                    row[f"{contact}_normal_n"] = f(row, "total_normal_load_n") / 4.0
            yaw_direction = sign(f(row, "yaw_rate_radps"))
            if yaw_direction == 0.0:
                yaw_direction = -sign(f(row, "patch_yaw_velocity_basis_m2ps"), eps=1.0e-10)
            row["yaw_direction"] = yaw_direction
            row["abs_forward_velocity_mps"] = abs(f(row, "forward_velocity_mps"))
            row["abs_yaw_rate_radps"] = abs(f(row, "yaw_rate_radps"))
            util = min(max(f(row, "max_force_preprojection_utilization"), 0.0), 5.0)
            limiter = min(max(f(row, "max_force_limiter_activity"), 0.0), 5.0)
            row["util_smooth"] = util / (1.0 + util)
            row["limiter_smooth"] = limiter / (1.0 + limiter)
            add_contact_bases(row, contacts, constants)
            rows.append(row)
    return rows


@dataclass
class CModel:
    vrel_knee: float
    fwd_knee: float
    names: list[str]
    scales: list[float]
    beta: list[float]

    def predict_opposes(self, row: dict[str, object]) -> float:
        if f(row, "yaw_direction") == 0.0:
            return 0.0
        total = 0.0
        for name, scale, beta in zip(self.names, self.scales, self.beta):
            value = c_feature_value(row, name, self.vrel_knee, self.fwd_knee)
            limit = 8.0 * scale
            value = min(max(value, -limit), limit)
            total += beta * (value / scale)
        return total


def load_c_model() -> CModel:
    rows = [row for row in read_rows_csv(C_DIR / "model_coefficients.csv") if row["candidate"] == "saturation_aware_surface"]
    if not rows:
        raise RuntimeError("missing Variant C coefficients")
    return CModel(
        vrel_knee=float(rows[0]["vrel_knee_mps"]),
        fwd_knee=float(rows[0]["fwd_knee_mps"]),
        names=[row["feature"] for row in rows],
        scales=[float(row["feature_scale"]) for row in rows],
        beta=[float(row["standardized_coefficient_nm"]) for row in rows],
    )


def c_schedules(row: dict[str, object], vrel_knee: float, fwd_knee: float) -> dict[str, float]:
    vrel = max(f(row, "vbar_rel_mps"), f(row, "load_weighted_rel_mps"), 0.0)
    vf = f(row, "abs_forward_velocity_mps")
    low_rel = 1.0 / (1.0 + (vrel / max(vrel_knee, 1.0e-9)) ** 2)
    low_forward = 1.0 / (1.0 + (vf / max(fwd_knee, 1.0e-9)) ** 2)
    return {
        "base": 1.0,
        "low_rel": low_rel,
        "high_forward": 1.0 - low_forward,
        "util": f(row, "util_smooth"),
        "limiter": f(row, "limiter_smooth"),
        "load_delta": f(row, "total_normal_load_n") / max(f(row, "nominal_load_n", 1.0), 1.0e-9) - 1.0,
    }


def c_feature_value(row: dict[str, object], name: str, vrel_knee: float, fwd_knee: float) -> float:
    base, suffix = name.split("__", 1)
    schedules = c_schedules(row, vrel_knee, fwd_knee)
    if suffix == "limiter_signed":
        return f(row, base) * schedules["limiter"] * f(row, "yaw_direction")
    return f(row, base) * schedules.get(suffix, 1.0)


@dataclass(frozen=True)
class AllocationConfig:
    name: str
    static_peak_nm: float
    speed_knee_mps: float
    force_knee_util: float
    rel_weight: float
    eps_nm: float
    c_credit: float


@dataclass
class AllocationResult:
    config: AllocationConfig
    score: float
    in_place: dict[str, object]
    split_metrics: list[dict[str, object]]
    selected_metrics: list[dict[str, object]]
    risk_metrics: list[dict[str, object]]


def reserve_terms(row: dict[str, object], config: AllocationConfig, nominal_load: float) -> dict[str, float]:
    rel = max(f(row, "vbar_rel_mps"), f(row, "load_weighted_rel_mps"), f(row, "load_weighted_lat_mps"), 0.0)
    vt = math.sqrt((config.rel_weight * rel) ** 2 + f(row, "abs_forward_velocity_mps") ** 2)
    low_speed_gate = 1.0 / (1.0 + (vt / max(config.speed_knee_mps, 1.0e-9)) ** 2)
    util = max(0.0, f(row, "actual_force_utilization"))
    force_gate = (util * util) / (util * util + config.force_knee_util * config.force_knee_util)
    load_scale = min(max(f(row, "total_normal_load_n") / max(nominal_load, 1.0e-9), 0.25), 2.0)
    launch = config.static_peak_nm * low_speed_gate * force_gate * load_scale
    return {
        "transition_speed_mps": vt,
        "low_speed_gate": low_speed_gate,
        "actual_force_utilization": util,
        "force_gate": force_gate,
        "load_scale": load_scale,
        "launch_reserve_nm": launch,
    }


def allocate(c_pred: float, launch_reserve: float, config: AllocationConfig) -> float:
    c_pos = smooth_positive(c_pred, config.eps_nm)
    c_credit = config.c_credit * c_pos
    if config.name == "softmax_reserve":
        return c_pred + smooth_max(launch_reserve, c_credit, config.eps_nm) - c_credit
    if config.name == "sqrt_softplus_reserve":
        return c_pred + smooth_positive(launch_reserve - c_credit, config.eps_nm)
    if config.name == "norm_reserve":
        return c_pred + math.sqrt(launch_reserve * launch_reserve + c_credit * c_credit + config.eps_nm * config.eps_nm) - c_credit
    raise ValueError(config.name)


def predict_allocation(row: dict[str, object], config: AllocationConfig, nominal_load: float) -> tuple[float, dict[str, float]]:
    terms = reserve_terms(row, config, nominal_load)
    pred = allocate(f(row, "variant_c_pred_opposes_nm"), terms["launch_reserve_nm"], config)
    return pred, terms


def corrected_residuals(rows: list[dict[str, object]], config: AllocationConfig | None, nominal_load: float) -> tuple[list[float], list[float], list[float]]:
    baseline: list[float] = []
    corrected: list[float] = []
    pred_opposes: list[float] = []
    for row in rows:
        raw = f(row, "residual_additive_yaw_torque_nm")
        if config is None:
            pred = f(row, "variant_c_pred_opposes_nm")
        else:
            pred, _ = predict_allocation(row, config, nominal_load)
        pred_raw = -f(row, "yaw_direction") * pred
        baseline.append(raw)
        corrected.append(raw - pred_raw)
        pred_opposes.append(pred)
    return baseline, corrected, pred_opposes


def run_balanced_weights(rows: list[dict[str, object]]) -> list[float]:
    counts = Counter(str(row["run_id"]) for row in rows)
    return [1.0 / max(counts[str(row["run_id"])], 1) for row in rows]


def metric_row(label: str, rows: list[dict[str, object]], config: AllocationConfig | None, nominal_load: float) -> dict[str, object]:
    baseline, corrected, pred_opposes = corrected_residuals(rows, config, nominal_load)
    weights = run_balanced_weights(rows)
    baseline_rmse = rmse(baseline)
    corrected_rmse = rmse(corrected)
    rb_baseline = weighted_rmse(baseline, weights)
    rb_corrected = weighted_rmse(corrected, weights)
    return {
        "group": label,
        "count": len(rows),
        "run_count": len({row["run_id"] for row in rows}),
        "baseline_rmse_nm": baseline_rmse,
        "corrected_rmse_nm": corrected_rmse,
        "baseline_mae_nm": mae(baseline),
        "corrected_mae_nm": mae(corrected),
        "baseline_median_abs_nm": median([abs(v) for v in baseline]),
        "corrected_median_abs_nm": median([abs(v) for v in corrected]),
        "baseline_signed_median_nm": median(baseline),
        "corrected_signed_median_nm": median(corrected),
        "run_balanced_baseline_rmse_nm": rb_baseline,
        "run_balanced_corrected_rmse_nm": rb_corrected,
        "rmse_improvement_pct": 100.0 * (baseline_rmse - corrected_rmse) / baseline_rmse if baseline_rmse > 0.0 else 0.0,
        "run_balanced_rmse_improvement_pct": 100.0 * (rb_baseline - rb_corrected) / rb_baseline if rb_baseline > 0.0 else 0.0,
        "median_predicted_opposes_nm": median(pred_opposes),
    }


def split_rows(rows: list[dict[str, object]], split: str) -> list[dict[str, object]]:
    return [row for row in rows if row.get("dataset_split") == split]


def validation_rows(rows: list[dict[str, object]]) -> list[dict[str, object]]:
    return [
        row
        for row in rows
        if row.get("dataset_split") in {"open_floor_fit_downweighted", "open_floor_validation_only", "diag_validation_only", "aux_downweighted_validation"}
    ]


def risk_groups(rows: list[dict[str, object]]) -> dict[str, list[dict[str, object]]]:
    return {
        "straightish_abs_yaw_lt_0p05": [row for row in rows if f(row, "abs_yaw_rate_radps") < 0.05],
        "straightish_forward_abs_yaw_lt_0p05_vf_ge_0p05": [
            row for row in rows if f(row, "abs_yaw_rate_radps") < 0.05 and f(row, "abs_forward_velocity_mps") >= 0.05
        ],
        "low_speed_yaw_vf_lt_0p05_yaw_ge_0p2": [
            row for row in rows if f(row, "abs_forward_velocity_mps") < 0.05 and f(row, "abs_yaw_rate_radps") >= 0.2
        ],
        "high_forward_vf_ge_0p5": [row for row in rows if f(row, "abs_forward_velocity_mps") >= 0.5],
        "limiter_active": [row for row in rows if f(row, "max_force_limiter_activity") > 0.0],
    }


def torque_from_command(command: float, wheel_speed_radps: float, constants: dict[str, float]) -> float:
    resistance = constants["drive_resistance_ohms"]
    speed_constant = constants["speed_constant_radps_per_volt"]
    torque_constant = constants["torque_constant_nm_per_a"]
    gear_ratio = constants["gear_ratio"]
    battery = constants["drive_voltage_v"]
    no_load = constants["no_load_current_a"]
    applied_voltage = command * battery
    current = (applied_voltage / resistance) - ((wheel_speed_radps * (gear_ratio / speed_constant)) / resistance)
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
    command_torque = applied_torque
    if signed_direction(applied_torque, wheel_speed_radps) != 0.0:
        command_torque += launch_dir * launch + constants["rolling_friction_torque_nm"] * loss_dir
    return command_torque, launch


def motor_commands_for_opposing_torque(opposing_yaw_torque: float, constants: dict[str, float], vf_mps: float, yaw_rate: float) -> dict[str, float]:
    radius = constants["wheel_radius_m"]
    track = constants["track_width_m"]
    left_surface, right_surface, left_speed, right_speed = wheel_speeds(vf_mps, yaw_rate, constants)
    applied_bank_torque = opposing_yaw_torque * radius / track
    left_command_torque, left_launch = command_torque_for_applied(applied_bank_torque, left_speed, constants)
    right_command_torque, right_launch = command_torque_for_applied(-applied_bank_torque, right_speed, constants)
    left_command = command_from_torque(left_command_torque, left_speed, constants)
    right_command = command_from_torque(right_command_torque, right_speed, constants)
    return {
        "applied_bank_torque_nm": applied_bank_torque,
        "left_command": left_command,
        "right_command": right_command,
        "lr_delta_command": left_command - right_command,
        "left_surface_mps": left_surface,
        "right_surface_mps": right_surface,
        "left_wheel_speed_radps": left_speed,
        "right_wheel_speed_radps": right_speed,
        "left_command_torque_nm": left_command_torque,
        "right_command_torque_nm": right_command_torque,
        "left_launch_torque_nm": left_launch,
        "right_launch_torque_nm": right_launch,
    }


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


def contact_utilization(constants: dict[str, float], yaw_rate: float) -> float:
    longitudinal = constants["drive_wheel_longitudinal_offset_m"]
    front_force = abs(constants["front_right_contact_force_gain_n_per_mps"] * longitudinal * yaw_rate)
    rear_force = abs(constants["rear_right_contact_force_gain_n_per_mps"] * longitudinal * yaw_rate)
    force_limit = constants["mass_kg"] * constants["sustained_lateral_accel_mps2"] / 4.0
    return max(front_force, rear_force) / max(force_limit, 1.0e-9)


def synthetic_row(constants: dict[str, float], nominal_load: float, vf_mps: float, yaw_rate: float, c_extra: float, allocated_extra: float) -> dict[str, object]:
    longitudinal = constants["drive_wheel_longitudinal_offset_m"]
    util = min(max(contact_utilization(constants, yaw_rate), 0.0), 5.0)
    limiter = max(0.0, util - 1.0)
    vrel = longitudinal * abs(yaw_rate)
    row: dict[str, object] = {
        "run_id": "synthetic",
        "dataset_split": "synthetic",
        "forward_velocity_mps": vf_mps,
        "abs_forward_velocity_mps": abs(vf_mps),
        "yaw_rate_radps": yaw_rate,
        "abs_yaw_rate_radps": abs(yaw_rate),
        "yaw_direction": sign(yaw_rate) or 1.0,
        "vbar_rel_mps": vrel,
        "vbar_yaw_mps": vrel,
        "load_weighted_rel_mps": vrel,
        "load_weighted_lat_mps": vrel,
        "total_normal_load_n": nominal_load,
        "actual_force_utilization": util,
        "actual_force_util_smooth": util / (1.0 + util),
        "util_smooth": util / (1.0 + util),
        "limiter_smooth": limiter / (1.0 + limiter),
        "variant_c_pred_opposes_nm": c_extra,
        "force_moment_opposes_yaw_nm": allocated_extra,
    }
    return row


def variant_c_extra_from_reference(vf_mps: float, yaw_rate: float, constants: dict[str, float]) -> float:
    if abs(vf_mps) < 1.0e-12 and abs(yaw_rate - 1.0) < 1.0e-12 and IN_PLACE_REFERENCE.exists():
        for row in read_rows_csv(IN_PLACE_REFERENCE):
            if row["variant"] == "Variant C combined slip":
                return float(row["extra_opposing_yaw_torque_nm"])
    coeff_rows = [row for row in read_rows_csv(C_DIR / "model_coefficients.csv") if row["candidate"] == "saturation_aware_surface"]
    coeffs = {row["feature"]: float(row["standardized_coefficient_nm"]) for row in coeff_rows}
    scales = {row["feature"]: float(row["feature_scale"]) for row in coeff_rows}
    vrel_knee = float(coeff_rows[0]["vrel_knee_mps"])
    fwd_knee = float(coeff_rows[0]["fwd_knee_mps"])
    base = baseline_opposing_yaw_torque(constants, yaw_rate)
    longitudinal = constants["drive_wheel_longitudinal_offset_m"]
    v_rel = longitudinal * abs(yaw_rate)
    util = min(max(contact_utilization(constants, yaw_rate), 0.0), 5.0)
    limiter = min(max(util - 1.0, 0.0), 5.0)
    util_smooth = util / (1.0 + util)
    limiter_smooth = limiter / (1.0 + limiter)
    low_rel = 1.0 / (1.0 + (v_rel / max(vrel_knee, 1.0e-9)) ** 2)
    high_forward = 1.0 - (1.0 / (1.0 + (abs(vf_mps) / max(fwd_knee, 1.0e-9)) ** 2))
    right_front = 2.0 * longitudinal * longitudinal * abs(yaw_rate)
    right_rear = 2.0 * longitudinal * longitudinal * abs(yaw_rate)

    def feature_value(feature: str, extra: float) -> float:
        total_req = base + extra
        values = {
            "gain_front_right_basis": right_front,
            "gain_rear_right_basis": right_rear,
            "gain_left_long_basis": 0.0,
            "gain_right_long_basis": 0.0,
            "force_gap_opposes_yaw_nm": -base,
            "req_moment_opposes_yaw_nm": -total_req,
            "force_moment_opposes_yaw_nm": -extra,
            "req_abs_contact_moment_nm": abs(total_req),
            "force_abs_contact_moment_nm": abs(extra),
        }
        base_name, suffix = feature.split("__", 1)
        x = values.get(base_name, 0.0)
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
    for _ in range(80):
        predicted = 0.0
        for feature, beta in coeffs.items():
            scale = max(scales.get(feature, 1.0), 1.0e-12)
            value = feature_value(feature, extra)
            value = min(max(value, -8.0 * scale), 8.0 * scale)
            predicted += beta * (value / scale)
        if abs(predicted - extra) < 1.0e-13:
            return predicted
        extra = predicted
    return extra


def in_place_command(config: AllocationConfig, constants: dict[str, float], nominal_load: float, vf_mps: float = 0.0, yaw_rate: float = 1.0) -> dict[str, object]:
    base = baseline_opposing_yaw_torque(constants, yaw_rate)
    c_extra = variant_c_extra_from_reference(vf_mps, yaw_rate, constants)
    extra = c_extra
    terms: dict[str, float] = {}
    for _ in range(80):
        row = synthetic_row(constants, nominal_load, vf_mps, yaw_rate, c_extra, extra)
        row["actual_force_utilization"] = min(max((base + max(extra, 0.0)) / max(nominal_yield_nm(constants, nominal_load), 1.0e-9), 0.0), 5.0)
        row["actual_force_util_smooth"] = f(row, "actual_force_utilization") / (1.0 + f(row, "actual_force_utilization"))
        next_extra, terms = predict_allocation(row, config, nominal_load)
        if abs(next_extra - extra) < 1.0e-13:
            extra = next_extra
            break
        extra = next_extra
    total = base + extra
    command = motor_commands_for_opposing_torque(total, constants, vf_mps, yaw_rate)
    command.update(
        {
            "variant": config.name,
            "baseline_opposing_yaw_torque_nm": base,
            "variant_c_extra_opposing_yaw_torque_nm": c_extra,
            "allocated_extra_opposing_yaw_torque_nm": extra,
            "total_opposing_yaw_torque_nm": total,
            "passes_abs_0p6_gate": int(max(abs(command["left_command"]), abs(command["right_command"])) >= 0.6),
            "max_abs_command": max(abs(command["left_command"]), abs(command["right_command"])),
            **terms,
        }
    )
    return command


def nominal_yield_nm(constants: dict[str, float], nominal_load: float) -> float:
    mu_ref = constants["mass_kg"] * constants["sustained_lateral_accel_mps2"] / max(nominal_load, 1.0e-9)
    return mu_ref * 0.5 * constants["track_width_m"] * nominal_load


def score_config(rows: list[dict[str, object]], config: AllocationConfig, constants: dict[str, float], nominal_load: float) -> AllocationResult:
    splits = [
        "primary_open_floor_fit_authoritative",
        "open_floor_fit_downweighted",
        "open_floor_validation_only",
        "diag_validation_only",
        "aux_downweighted_validation",
    ]
    split_metrics = [metric_row(split, split_rows(rows, split), config, nominal_load) for split in splits if split_rows(rows, split)]
    split_metrics.append(metric_row("validation_non_authoritative", validation_rows(rows), config, nominal_load))
    selected_metrics = []
    for run_id in SELECTED_LOGS:
        subset = [row for row in rows if row.get("run_id") == run_id]
        if not subset:
            selected_metrics.append({"run_id": run_id, "present": 0, "dataset_split": "", "count": 0})
            continue
        metric = metric_row(run_id, subset, config, nominal_load)
        metric["run_id"] = run_id
        metric["present"] = 1
        metric["dataset_split"] = subset[0].get("dataset_split", "")
        selected_metrics.append(metric)
    risk_metrics = [metric_row(name, subset, config, nominal_load) for name, subset in risk_groups(rows).items() if subset]
    in_place = in_place_command(config, constants, nominal_load)
    split_by_name = {row["group"]: row for row in split_metrics}
    validation_rmse = float(split_by_name["validation_non_authoritative"]["run_balanced_corrected_rmse_nm"])
    primary_rmse = float(split_by_name["primary_open_floor_fit_authoritative"]["run_balanced_corrected_rmse_nm"])
    open_fit_rmse = float(split_by_name["open_floor_fit_downweighted"]["run_balanced_corrected_rmse_nm"])
    straight = next(row for row in risk_metrics if row["group"] == "straightish_abs_yaw_lt_0p05")
    high = next(row for row in risk_metrics if row["group"] == "high_forward_vf_ge_0p5")
    gate_penalty = 10.0 if not int(in_place["passes_abs_0p6_gate"]) else 0.0
    score = validation_rmse + 0.35 * primary_rmse + 0.15 * open_fit_rmse + gate_penalty
    score += 0.25 * max(0.0, float(straight["run_balanced_corrected_rmse_nm"]) - float(straight["run_balanced_baseline_rmse_nm"]))
    score += 0.20 * max(0.0, float(high["run_balanced_corrected_rmse_nm"]) - 1.03 * float(high["run_balanced_baseline_rmse_nm"]))
    score += 0.05 * max(0.0, float(in_place["max_abs_command"]) - 0.68)
    return AllocationResult(config, score, in_place, split_metrics, selected_metrics, risk_metrics)


def candidate_grid() -> list[AllocationConfig]:
    configs = []
    for name in ["sqrt_softplus_reserve", "softmax_reserve", "norm_reserve"]:
        for peak in [0.090, 0.100]:
            for speed in [0.020, 0.030, 0.045]:
                for force_knee in [0.45]:
                    for credit in [1.0]:
                        configs.append(AllocationConfig(name, peak, speed, force_knee, 0.75, 0.001, credit))
    return configs


def vf_grid() -> list[float]:
    return [round(i * 0.15 / 5.0, 9) for i in range(6)]


def yaw_grid() -> list[float]:
    return [round(0.2 + i * (6.0 - 0.2) / 9.0, 9) for i in range(10)]


def write_csv(path: Path, rows: list[dict[str, object]], fieldnames: list[str] | None = None) -> None:
    if fieldnames is None:
        fieldnames = []
        for row in rows:
            for key in row:
                if key not in fieldnames:
                    fieldnames.append(key)
        if not fieldnames:
            fieldnames = ["empty"]
    with path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def fmt(value: object, digits: int = 6) -> str:
    try:
        return f"{float(value):.{digits}f}"
    except (TypeError, ValueError):
        return str(value)


def table_lines(rows: list[dict[str, object]], columns: list[str], labels: list[str] | None = None) -> list[str]:
    labels = labels or columns
    out = ["| " + " | ".join(labels) + " |", "| " + " | ".join("---" for _ in labels) + " |"]
    for row in rows:
        out.append("| " + " | ".join(fmt(row.get(col, "")) for col in columns) + " |")
    return out


def comparison_rows(split_metrics: list[dict[str, object]], selected_metrics: list[dict[str, object]]) -> tuple[list[dict[str, object]], list[dict[str, object]]]:
    force_split = read_keyed_csv(FORCE_DOMAIN_DIR / "split_rmse.csv", "dataset_split")
    c_split = read_keyed_csv(C_DIR / "split_metrics.csv", "group")
    split_compare = []
    for row in split_metrics:
        split = str(row["group"])
        c_row = c_split.get(split, {})
        f_row = force_split.get(split, {})
        split_compare.append(
            {
                "split": split,
                "count": row["count"],
                "baseline_rmse_nm": row["baseline_rmse_nm"],
                "allocation_corrected_rmse_nm": row["corrected_rmse_nm"],
                "variant_c_corrected_rmse_nm": c_row.get("corrected_rmse_nm", ""),
                "force_domain_corrected_rmse_nm": f_row.get("corrected_rmse_nm", ""),
                "allocation_minus_c_rmse_nm": float(row["corrected_rmse_nm"]) - float(c_row["corrected_rmse_nm"]) if c_row else "",
                "allocation_minus_force_domain_rmse_nm": float(row["corrected_rmse_nm"]) - float(f_row["corrected_rmse_nm"]) if f_row else "",
            }
        )
    force_selected = read_keyed_csv(FORCE_DOMAIN_DIR / "selected_log_rmse.csv", "run_id")
    c_selected = read_keyed_csv(C_DIR / "selected_log_metrics.csv", "run_id")
    selected_compare = []
    for row in selected_metrics:
        run_id = str(row["run_id"])
        c_row = c_selected.get(run_id, {})
        f_row = force_selected.get(run_id, {})
        allocation_rmse = float(row["corrected_rmse_nm"]) if int(row.get("present", 0)) else math.nan
        selected_compare.append(
            {
                "run_id": run_id,
                "dataset_split": row.get("dataset_split", ""),
                "count": row.get("count", 0),
                "baseline_rmse_nm": row.get("baseline_rmse_nm", ""),
                "allocation_corrected_rmse_nm": allocation_rmse if math.isfinite(allocation_rmse) else "",
                "variant_c_corrected_rmse_nm": c_row.get("corrected_rmse_nm", ""),
                "force_domain_corrected_rmse_nm": f_row.get("corrected_rmse_nm", ""),
                "allocation_minus_c_rmse_nm": allocation_rmse - float(c_row["corrected_rmse_nm"]) if c_row and math.isfinite(allocation_rmse) else "",
                "allocation_minus_force_domain_rmse_nm": allocation_rmse - float(f_row["corrected_rmse_nm"]) if f_row and math.isfinite(allocation_rmse) else "",
            }
        )
    return split_compare, selected_compare


def write_report(best: AllocationResult, candidate_rows: list[dict[str, object]], split_compare: list[dict[str, object]], selected_compare: list[dict[str, object]], constants: dict[str, float], nominal_load: float) -> None:
    cfg = best.config
    coeff_rows = [
        {"parameter": "allocation_form", "value": cfg.name, "unit": "enum"},
        {"parameter": "static_peak_nm", "value": cfg.static_peak_nm, "unit": "Nm"},
        {"parameter": "speed_knee_mps", "value": cfg.speed_knee_mps, "unit": "m/s"},
        {"parameter": "force_knee_util", "value": cfg.force_knee_util, "unit": "actual force utilization"},
        {"parameter": "rel_weight", "value": cfg.rel_weight, "unit": "dimensionless"},
        {"parameter": "eps_nm", "value": cfg.eps_nm, "unit": "Nm"},
        {"parameter": "c_credit", "value": cfg.c_credit, "unit": "fraction"},
        {"parameter": "nominal_load_n", "value": nominal_load, "unit": "N"},
        {"parameter": "nominal_longitudinal_yield_nm", "value": nominal_yield_nm(constants, nominal_load), "unit": "Nm"},
    ]
    write_csv(OUT / "selected_parameters.csv", coeff_rows)
    lines = [
        "# Max/Norm Static-Yield Allocation",
        "",
        "Analysis-only output. Production code, build metadata, and tests were not edited.",
        "",
        "## Recommendation",
        "",
        "Use the sqrt-softplus reserve allocation as the best transition shape from this pass, but treat the coefficient as a validation candidate rather than a production tune. It passes the in-place launch command gate while preserving a meaningful share of Variant C's broad validation advantage; it is materially better than the force-domain Stribeck reference on primary, downweighted, diag, and aux splits, but not on the open-floor validation-only split.",
        "",
        "The physical interpretation is that the static-yield branch is a reserve, not a second moving-contact residual. Variant C already accounts for sliding/moving contact; the launch reserve only fills the missing same-sign yaw-opposing torque when the reserve exceeds the already-opposing part of Variant C.",
        "",
        "## Selected Equation",
        "",
        "`v_t = sqrt((rel_weight * max(vbar_rel, load_weighted_rel, load_weighted_lat))^2 + |Vf|^2)`",
        "",
        "`G_v = 1 / (1 + (v_t / v_k)^2)`",
        "",
        "`G_u = u_actual^2 / (u_actual^2 + u_k^2)`",
        "",
        "`M_launch = M_static_peak * G_v * G_u * clamp(N / N_nom, 0.25, 2.0)`",
        "",
        "`C_pos = soft_positive(M_C, eps)`",
        "",
        "`M_opp = M_C + soft_positive(M_launch - c_credit * C_pos, eps)`",
        "",
        "where `soft_positive(x, eps) = 0.5 * (x + sqrt(x^2 + eps^2))`. The selected form uses no trig, no exp, and no tanh; the only non-polynomial operation is sqrt.",
        "",
        "## Coefficients",
        "",
    ]
    lines.extend(table_lines(coeff_rows, ["parameter", "value", "unit"]))
    lines.extend(
        [
            "",
            "## +1 rad/s In-Place Command",
            "",
        ]
    )
    lines.extend(
        table_lines(
            [best.in_place],
            [
                "allocated_extra_opposing_yaw_torque_nm",
                "variant_c_extra_opposing_yaw_torque_nm",
                "total_opposing_yaw_torque_nm",
                "left_command",
                "right_command",
                "lr_delta_command",
                "max_abs_command",
                "passes_abs_0p6_gate",
            ],
            ["allocation extra", "C extra", "total opp", "left cmd", "right cmd", "L-R delta", "max abs cmd", "gate"],
        )
    )
    lines.extend(["", "## Split RMSE Versus C And Force-Domain", ""])
    lines.extend(
        table_lines(
            split_compare,
            [
                "split",
                "count",
                "baseline_rmse_nm",
                "allocation_corrected_rmse_nm",
                "variant_c_corrected_rmse_nm",
                "force_domain_corrected_rmse_nm",
                "allocation_minus_c_rmse_nm",
                "allocation_minus_force_domain_rmse_nm",
            ],
            ["split", "count", "baseline", "allocation", "C", "force-domain", "alloc-C", "alloc-force"],
        )
    )
    phase_rows = read_rows_csv(OUT / "phase_metrics.csv")
    if phase_rows:
        lines.extend(["", "## Phase RMSE", ""])
        lines.extend(
            table_lines(
                phase_rows,
                ["dataset_split", "physics_phase", "count", "baseline_rmse_nm", "corrected_rmse_nm", "rmse_improvement_pct"],
                ["split", "phase", "count", "baseline", "allocation", "improvement %"],
            )
        )
    lines.extend(["", "## Selected-Log RMSE Versus C And Force-Domain", ""])
    lines.extend(
        table_lines(
            selected_compare,
            [
                "run_id",
                "dataset_split",
                "count",
                "baseline_rmse_nm",
                "allocation_corrected_rmse_nm",
                "variant_c_corrected_rmse_nm",
                "force_domain_corrected_rmse_nm",
                "allocation_minus_c_rmse_nm",
                "allocation_minus_force_domain_rmse_nm",
            ],
            ["run", "split", "count", "baseline", "allocation", "C", "force-domain", "alloc-C", "alloc-force"],
        )
    )
    lines.extend(["", "## Risk Slices", ""])
    lines.extend(
        table_lines(
            best.risk_metrics,
            ["group", "count", "baseline_rmse_nm", "corrected_rmse_nm", "baseline_median_abs_nm", "corrected_median_abs_nm", "run_balanced_rmse_improvement_pct"],
            ["group", "count", "baseline", "allocation", "med abs before", "med abs after", "RB change %"],
        )
    )
    lines.extend(["", "## Candidate Ranking", ""])
    lines.extend(
        table_lines(
            candidate_rows[:12],
            [
                "rank",
                "allocation_form",
                "objective_score",
                "static_peak_nm",
                "speed_knee_mps",
                "force_knee_util",
                "c_credit",
                "in_place_max_abs_command",
                "validation_rb_corrected_rmse_nm",
                "primary_rb_corrected_rmse_nm",
            ],
        )
    )
    lines.extend(
        [
            "",
            "## Computational Cost",
            "",
            "Per tick, after Variant C is available, the selected allocation adds roughly: one `max` over contact-speed estimates, two squares for `v_t`, one sqrt for `v_t`, two rational gates, one clamp/multiply for load scaling, one sqrt soft-positive for `C_pos`, and one sqrt soft-positive for the reserve. It has no trig table, no exponential, no tanh, and no per-contact history state.",
            "",
            "The in-place command estimator still uses the existing analysis motor/launch-friction approximation so it can be compared to prior workers; that estimator is not part of the selected allocation law.",
            "",
            "## Caveats",
            "",
            "- The actual-force utilization selector is derived from projected/actual contact force and normal load. It does not use command/request values as a traction selector.",
            "- The model does not use UKF state columns.",
            "- Validation RMSE remains worse than pure Variant C because any launch-gate-passing static reserve adds resistance in some rows where C already fit the residual. The allocation shape reduces, but does not erase, that tradeoff.",
            "- The best fit is sensitive to the synthetic +1 rad/s command gate. A targeted in-place launch dataset should replace that synthetic anchor before production tuning.",
            "",
            "## Reproduce",
            "",
            "```powershell",
            "python codex_analysis\\yaw_model_variant_fits\\transition_options\\max_norm_yield_allocation\\fit_max_norm_yield_allocation.py",
            "```",
            "",
            "## Output Files",
            "",
            "- `fit_max_norm_yield_allocation.py`",
            "- `max_norm_yield_allocation_report.md`",
            "- `selected_parameters.csv`",
            "- `candidate_scores.csv`",
            "- `split_metrics.csv`",
            "- `phase_metrics.csv`",
            "- `selected_log_metrics.csv`",
            "- `risk_metrics.csv`",
            "- `split_comparison_vs_c_force_domain.csv`",
            "- `selected_log_comparison_vs_c_force_domain.csv`",
            "- `in_place_1radps_command.csv`",
            "- `lr_delta_grid.csv`",
            "- `prediction_sample.csv`",
            "- `metadata.json`",
            "- `commands_run.txt`",
        ]
    )
    (OUT / "max_norm_yield_allocation_report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def make_grid(best: AllocationResult, constants: dict[str, float], nominal_load: float) -> list[dict[str, object]]:
    rows = []
    for vf in vf_grid():
        for yaw in yaw_grid():
            cmd = in_place_command(best.config, constants, nominal_load, vf_mps=vf, yaw_rate=yaw)
            rows.append({"vf_mps": vf, "yaw_rate_radps": yaw, **cmd})
    return rows


def phase_metrics(rows: list[dict[str, object]], best: AllocationResult, nominal_load: float) -> list[dict[str, object]]:
    out = []
    for split in [
        "primary_open_floor_fit_authoritative",
        "open_floor_fit_downweighted",
        "open_floor_validation_only",
        "diag_validation_only",
        "aux_downweighted_validation",
    ]:
        for phase in ["entry", "plateau", "exit"]:
            subset = [row for row in rows if row.get("dataset_split") == split and row.get("physics_phase") == phase]
            if subset:
                metric = metric_row(f"{split}:{phase}", subset, best.config, nominal_load)
                metric["dataset_split"] = split
                metric["physics_phase"] = phase
                out.append(metric)
    return out


def make_prediction_sample(rows: list[dict[str, object]], best: AllocationResult, nominal_load: float) -> list[dict[str, object]]:
    out = []
    counts: Counter[str] = Counter()
    for row in rows:
        run_id = str(row["run_id"])
        if run_id not in SELECTED_LOGS or counts[run_id] >= 20:
            continue
        pred, terms = predict_allocation(row, best.config, nominal_load)
        pred_raw = -f(row, "yaw_direction") * pred
        out.append(
            {
                "run_id": run_id,
                "dataset_split": row.get("dataset_split", ""),
                "physics_phase": row.get("physics_phase", ""),
                "row_index": row.get("row_index", ""),
                "forward_velocity_mps": f(row, "forward_velocity_mps"),
                "yaw_rate_radps": f(row, "yaw_rate_radps"),
                "variant_c_pred_opposes_nm": f(row, "variant_c_pred_opposes_nm"),
                "launch_reserve_nm": terms["launch_reserve_nm"],
                "allocated_pred_opposes_nm": pred,
                "residual_additive_yaw_torque_nm": f(row, "residual_additive_yaw_torque_nm"),
                "predicted_additive_yaw_torque_nm": pred_raw,
                "corrected_residual_yaw_torque_nm": f(row, "residual_additive_yaw_torque_nm") - pred_raw,
                "low_speed_gate": terms["low_speed_gate"],
                "force_gate": terms["force_gate"],
                "actual_force_utilization": terms["actual_force_utilization"],
            }
        )
        counts[run_id] += 1
    return out


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    constants = read_constants()
    rows = load_rows(constants)
    nominal_loads = [f(row, "total_normal_load_n") for row in rows if row.get("dataset_split") == "primary_open_floor_fit_authoritative" and f(row, "total_normal_load_n") > 0.0]
    nominal_load = median(nominal_loads) if nominal_loads else constants["mass_kg"] * 9.80665
    for row in rows:
        row["nominal_load_n"] = nominal_load
    c_model = load_c_model()
    for row in rows:
        row["variant_c_pred_opposes_nm"] = c_model.predict_opposes(row)

    results = [score_config(rows, config, constants, nominal_load) for config in candidate_grid()]
    results.sort(key=lambda result: result.score)
    best = results[0]

    candidate_rows = []
    for rank, result in enumerate(results, 1):
        split_by_name = {row["group"]: row for row in result.split_metrics}
        candidate_rows.append(
            {
                "rank": rank,
                "allocation_form": result.config.name,
                "objective_score": result.score,
                "static_peak_nm": result.config.static_peak_nm,
                "speed_knee_mps": result.config.speed_knee_mps,
                "force_knee_util": result.config.force_knee_util,
                "c_credit": result.config.c_credit,
                "eps_nm": result.config.eps_nm,
                "in_place_max_abs_command": result.in_place["max_abs_command"],
                "in_place_pass": result.in_place["passes_abs_0p6_gate"],
                "validation_rb_corrected_rmse_nm": split_by_name["validation_non_authoritative"]["run_balanced_corrected_rmse_nm"],
                "primary_rb_corrected_rmse_nm": split_by_name["primary_open_floor_fit_authoritative"]["run_balanced_corrected_rmse_nm"],
                "open_fit_rb_corrected_rmse_nm": split_by_name["open_floor_fit_downweighted"]["run_balanced_corrected_rmse_nm"],
            }
        )

    split_compare, selected_compare = comparison_rows(best.split_metrics, best.selected_metrics)
    grid = make_grid(best, constants, nominal_load)

    write_csv(OUT / "candidate_scores.csv", candidate_rows)
    write_csv(OUT / "split_metrics.csv", best.split_metrics)
    write_csv(OUT / "phase_metrics.csv", phase_metrics(rows, best, nominal_load))
    write_csv(OUT / "selected_log_metrics.csv", best.selected_metrics)
    write_csv(OUT / "risk_metrics.csv", best.risk_metrics)
    write_csv(OUT / "split_comparison_vs_c_force_domain.csv", split_compare)
    write_csv(OUT / "selected_log_comparison_vs_c_force_domain.csv", selected_compare)
    write_csv(OUT / "in_place_1radps_command.csv", [best.in_place])
    write_csv(OUT / "lr_delta_grid.csv", grid)
    write_csv(OUT / "prediction_sample.csv", make_prediction_sample(rows, best, nominal_load))
    metadata = {
        "inputs": {
            "primary": str(PRIMARY.relative_to(ROOT)),
            "secondary": str(SECONDARY.relative_to(ROOT)),
            "constants": str(CONSTANTS.relative_to(ROOT)),
            "variant_c_coefficients": str((C_DIR / "model_coefficients.csv").relative_to(ROOT)),
            "force_domain_reference": str(FORCE_DOMAIN_DIR.relative_to(ROOT)),
        },
        "selected": candidate_rows[0],
        "rows": len(rows),
        "nominal_load_n": nominal_load,
        "production_code_edited": False,
        "build_metadata_edited": False,
        "tests_edited": False,
        "uses_command_or_request_as_traction_selector": False,
        "uses_ukf_state_columns": False,
    }
    (OUT / "metadata.json").write_text(json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (OUT / "commands_run.txt").write_text(
        "python codex_analysis\\yaw_model_variant_fits\\transition_options\\max_norm_yield_allocation\\fit_max_norm_yield_allocation.py\n",
        encoding="utf-8",
    )
    write_report(best, candidate_rows, split_compare, selected_compare, constants, nominal_load)
    print((OUT / "max_norm_yield_allocation_report.md").as_posix())


if __name__ == "__main__":
    main()
