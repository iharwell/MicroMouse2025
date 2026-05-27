#!/usr/bin/env python3
"""Pade/rational static-yield + moving combined-slip yaw residual fit.

Analysis-only tooling. Reads existing contact-continuum feature artifacts and
writes outputs only beside this script. Uses only the Python standard library.
"""

from __future__ import annotations

import csv
import json
import math
from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path


ROOT = Path(__file__).resolve().parents[4]
OUT = Path(__file__).resolve().parent
PRIMARY = ROOT / "codex_analysis" / "contact_continuum_yaw_identification" / "ablation" / "phase_classified_feature_sample.csv"
SECONDARY = ROOT / "codex_analysis" / "contact_continuum_yaw_identification" / "features" / "contact_continuum_feature_sample.csv"
CONSTANTS = ROOT / "codex_analysis" / "contact_continuum_yaw_identification" / "features" / "plant_mirror_constants.csv"
FORCE_DOMAIN_COEFF = ROOT / "codex_analysis" / "yaw_model_variant_fits" / "round2_force_domain_stribeck" / "force_domain_coefficients.csv"
C_COEFF = ROOT / "codex_analysis" / "yaw_model_variant_fits" / "combined_slip_surface" / "model_coefficients.csv"
FORCE_DOMAIN_SPLIT = ROOT / "codex_analysis" / "yaw_model_variant_fits" / "round2_force_domain_stribeck" / "split_rmse.csv"
C_SPLIT = ROOT / "codex_analysis" / "yaw_model_variant_fits" / "combined_slip_surface" / "split_metrics.csv"
FORCE_DOMAIN_SELECTED = ROOT / "codex_analysis" / "yaw_model_variant_fits" / "round2_force_domain_stribeck" / "selected_log_rmse.csv"
C_SELECTED = ROOT / "codex_analysis" / "yaw_model_variant_fits" / "combined_slip_surface" / "selected_log_metrics.csv"

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

PRIMARY_COLUMNS = {
    "run_id",
    "family",
    "schema",
    "recommendation",
    "dataset_split",
    "row_index",
    "time_us",
    "physics_phase",
    "forward_velocity_mps",
    "yaw_rate_radps",
    "vbar_rel_mps",
    "vbar_yaw_mps",
    "vbar_lat_mps",
    "max_force_preprojection_utilization",
    "max_force_limiter_activity",
    "hardware_saturation_evidence",
    "gyro_derivative_spike",
    "residual_additive_yaw_torque_nm",
    "residual_opposes_yaw_nm",
    "patch_yaw_req_basis_nm",
    "patch_yaw_force_basis_nm",
}

SURFACE_FEATURES = [
    "gain_front_right_basis__moving",
    "gain_rear_right_basis__moving",
    "gain_left_long_basis__moving",
    "gain_right_long_basis__moving",
    "force_moment_opposes_yaw_nm__moving",
    "force_moment_opposes_yaw_nm__util_moving",
    "force_moment_opposes_yaw_nm__high_forward",
    "force_abs_contact_moment_nm__limiter_signed_moving",
]

HARD_GATE_ABS_COMMAND_MIN = 0.60
HARD_GATE_ABS_COMMAND_MAX = 0.72
HARD_GATE_ABS_COMMAND_TARGET = 0.646
SURFACE_CAP_NM = 0.040


@dataclass(frozen=True)
class YieldParams:
    yaw_activation_mps: float
    force_activation_util: float
    static_speed_mps: float
    speed_fade_mps: float
    rel_weight: float
    util_k: float
    slide_ratio: float
    static_quartic: float
    load_exp: float = 1.0


@dataclass
class SurfaceModel:
    names: list[str]
    scales: list[float]
    beta: list[float]
    ridge: float


@dataclass
class StaticYieldContactModel:
    params: YieldParams
    static_peak_nm: float
    surface_rel_knee_mps: float
    surface_fwd_knee_mps: float
    nominal_load_n: float
    surface: SurfaceModel
    surface_cap_nm: float
    hard_gate_left_command: float
    hard_gate_right_command: float
    hard_gate_lr_delta: float
    hard_gate_extra_opposing_nm: float
    hard_gate_total_opposing_nm: float
    hard_gate_pass: bool

    @property
    def sliding_yield_nm(self) -> float:
        return self.static_peak_nm * self.params.slide_ratio


def f(row: dict[str, object], key: str, default: float = 0.0) -> float:
    value = row.get(key, default)
    if isinstance(value, float):
        return value if math.isfinite(value) else default
    try:
        x = float(value)
        return x if math.isfinite(x) else default
    except (TypeError, ValueError):
        return default


def sign(value: float, eps: float = 1.0e-6) -> float:
    if value > eps:
        return 1.0
    if value < -eps:
        return -1.0
    return 0.0


def smooth_positive(value: float, epsilon: float = 1.0e-6) -> float:
    return 0.5 * (value + math.sqrt(value * value + epsilon * epsilon))


def rational_fade(value: float, knee: float, quartic: float = 0.0) -> float:
    x = value / max(knee, 1.0e-9)
    x2 = x * x
    return 1.0 / (1.0 + x2 + max(quartic, 0.0) * x2 * x2)


def rational_rise(value: float, knee: float) -> float:
    x = value / max(knee, 1.0e-9)
    x2 = x * x
    return x2 / (1.0 + x2)


def soft_cap(value: float, cap: float) -> float:
    limit = max(cap, 1.0e-9)
    z = value / limit
    return limit * z / math.sqrt(1.0 + z * z)


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
    if not clean:
        return 0.0
    return math.sqrt(sum(v * v for v in clean) / len(clean))


def mae(values: list[float]) -> float:
    clean = [v for v in values if math.isfinite(v)]
    if not clean:
        return 0.0
    return sum(abs(v) for v in clean) / len(clean)


def weighted_rmse(values: list[float], weights: list[float]) -> float:
    total = sum(weights)
    if total <= 0.0:
        return 0.0
    return math.sqrt(max(sum(w * v * v for v, w in zip(values, weights)) / total, 0.0))


def weighted_mae(values: list[float], weights: list[float]) -> float:
    total = sum(weights)
    if total <= 0.0:
        return 0.0
    return sum(w * abs(v) for v, w in zip(values, weights)) / total


def read_constants() -> dict[str, float]:
    with CONSTANTS.open(newline="", encoding="utf-8") as fh:
        return {row["name"]: float(row["value"]) for row in csv.DictReader(fh)}


def read_key_value_csv(path: Path) -> dict[str, float]:
    with path.open(newline="", encoding="utf-8") as fh:
        out: dict[str, float] = {}
        for row in csv.DictReader(fh):
            try:
                out[row["parameter"]] = float(row["value"])
            except (TypeError, ValueError):
                continue
        return out


def read_coeff_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as fh:
        return list(csv.DictReader(fh))


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


def load_secondary_contact_fields(keys: set[tuple[str, str]]) -> dict[tuple[str, str], dict[str, str]]:
    fields = set(CONTACT_FIELDS)
    out: dict[tuple[str, str], dict[str, str]] = {}
    with SECONDARY.open(newline="", encoding="utf-8") as fh:
        reader = csv.DictReader(fh)
        for row in reader:
            key = (row["run_id"], row["row_index"])
            if key in keys:
                out[key] = {name: row.get(name, "") for name in fields}
    return out


def load_rows(constants: dict[str, float]) -> list[dict[str, object]]:
    nominal_load = constants["mass_kg"] * 9.80665 + constants.get("fan_downforce_full_duty_n", 0.7) * 0.8
    primary_rows: list[dict[str, str]] = []
    primary_keys: set[tuple[str, str]] = set()
    with PRIMARY.open(newline="", encoding="utf-8") as fh:
        reader = csv.DictReader(fh)
        for src in reader:
            if src.get("dataset_split") == "excluded_or_unclassified":
                continue
            primary_rows.append(src)
            primary_keys.add((src["run_id"], src["row_index"]))
    secondary = load_secondary_contact_fields(primary_keys)
    rows: list[dict[str, object]] = []
    for src in primary_rows:
            row: dict[str, object] = {key: src.get(key, "") for key in PRIMARY_COLUMNS}
            row.update(secondary.get((src["run_id"], src["row_index"]), {}))
            for key in list(row):
                if key in {"run_id", "family", "schema", "recommendation", "dataset_split", "physics_phase", "row_index"}:
                    continue
                try:
                    row[key] = float(row[key]) if row[key] != "" else 0.0
                except (TypeError, ValueError):
                    row[key] = 0.0
            if f(row, "total_normal_load_n") <= 0.0:
                row["total_normal_load_n"] = nominal_load
            for name in ["fl", "fr", "rl", "rr"]:
                if f(row, f"{name}_normal_n") <= 0.0:
                    row[f"{name}_normal_n"] = f(row, "total_normal_load_n") / 4.0
            yaw_sign = sign(f(row, "yaw_rate_radps"))
            if yaw_sign == 0.0:
                yaw_sign = sign(f(row, "patch_yaw_force_basis_nm"))
            if yaw_sign == 0.0:
                yaw_sign = 1.0
            row["yaw_sign"] = yaw_sign
            row["abs_forward_velocity_mps"] = abs(f(row, "forward_velocity_mps"))
            row["abs_yaw_rate_radps"] = abs(f(row, "yaw_rate_radps"))
            rows.append(row)

    half_track = 0.5 * constants["track_width_m"]
    front_f = constants["drive_wheel_longitudinal_offset_m"]
    contacts = {
        "fl": (-half_track, front_f),
        "fr": (half_track, front_f),
        "rl": (-half_track, -front_f),
        "rr": (half_track, -front_f),
    }
    force_limit = constants["mass_kg"] * constants["sustained_lateral_accel_mps2"] / 4.0
    for idx, row in enumerate(rows):
        row["_idx"] = idx
        add_contact_bases(row, contacts, force_limit)
    return rows


def add_contact_bases(row: dict[str, object], contacts: dict[str, tuple[float, float]], force_limit: float) -> None:
    yaw_sign = f(row, "yaw_sign")
    total_normal = max(f(row, "total_normal_load_n"), 1.0e-9)
    right_front = 0.0
    right_rear = 0.0
    long_left = 0.0
    long_right = 0.0
    req_moment = 0.0
    force_moment = 0.0
    req_abs_moment = 0.0
    force_abs_moment = 0.0
    load_weighted_rel = 0.0
    load_weighted_lat = 0.0
    max_actual_force_util = 0.0
    for name, (r_pos, f_pos) in contacts.items():
        vf_rel = f(row, f"{name}_v_rel_f_mps")
        vr_rel = f(row, f"{name}_v_rel_r_mps")
        normal_frac = f(row, f"{name}_normal_n") / total_normal
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
        req_local = f_pos * f(row, f"{name}_req_r_n") - r_pos * f(row, f"{name}_req_f_n")
        force_local = f_pos * f(row, f"{name}_force_r_n") - r_pos * f(row, f"{name}_force_f_n")
        force_mag = math.hypot(f(row, f"{name}_force_f_n"), f(row, f"{name}_force_r_n"))
        max_actual_force_util = max(max_actual_force_util, force_mag / max(force_limit, 1.0e-9))
        req_moment += req_local
        force_moment += force_local
        req_abs_moment += abs(req_local)
        force_abs_moment += abs(force_local)
        load_weighted_rel += normal_frac * math.hypot(vf_rel, vr_rel)
        load_weighted_lat += normal_frac * abs(vr_rel)

    row["gain_front_right_basis"] = right_front
    row["gain_rear_right_basis"] = right_rear
    row["gain_left_long_basis"] = long_left
    row["gain_right_long_basis"] = long_right
    row["req_moment_opposes_yaw_nm"] = -yaw_sign * req_moment
    row["force_moment_opposes_yaw_nm"] = -yaw_sign * force_moment
    row["force_gap_opposes_yaw_nm"] = -yaw_sign * (req_moment - force_moment)
    row["req_abs_contact_moment_nm"] = req_abs_moment
    row["force_abs_contact_moment_nm"] = force_abs_moment
    row["load_weighted_rel_mps"] = load_weighted_rel
    row["load_weighted_lat_mps"] = load_weighted_lat
    row["actual_force_utilization"] = max_actual_force_util
    row["actual_force_limiter_activity"] = max(0.0, max_actual_force_util - 1.0)


def selected_nominal_load(rows: list[dict[str, object]]) -> float:
    train_loads = [
        f(row, "total_normal_load_n")
        for row in rows
        if row.get("dataset_split") == "primary_open_floor_fit_authoritative" and f(row, "total_normal_load_n") > 0.0
    ]
    med = median(train_loads)
    return med if med > 0.0 else 1.0


def fit_weights(rows: list[dict[str, object]]) -> list[float]:
    weights: list[float] = []
    for row in rows:
        w = 0.0
        if row.get("dataset_split") == "primary_open_floor_fit_authoritative":
            w = 1.0
        elif (
            row.get("dataset_split") == "open_floor_fit_downweighted"
            and row.get("recommendation") == "fit_downweighted"
            and row.get("family") == "open_floor"
        ):
            w = 0.25
        limiter = min(max(f(row, "actual_force_limiter_activity"), 0.0), 1.0)
        saturation = min(max(f(row, "hardware_saturation_evidence"), 0.0), 1.0)
        spike = min(max(f(row, "gyro_derivative_spike"), 0.0), 1.0)
        quality = (1.0 / (1.0 + 4.0 * limiter)) * (1.0 - 0.75 * saturation) * (1.0 - 0.75 * spike)
        weights.append(w * min(max(quality, 0.02), 1.0))
    counts = Counter(str(row["run_id"]) for row, w in zip(rows, weights) if w > 0.0)
    for idx, row in enumerate(rows):
        if weights[idx] > 0.0:
            weights[idx] *= 1.0 / math.sqrt(max(counts[str(row["run_id"])], 1))
    total = sum(w for w in weights if w > 0.0)
    positive = sum(1 for w in weights if w > 0.0)
    if total > 0.0:
        weights = [w * positive / total if w > 0.0 else 0.0 for w in weights]
    return weights


def run_balanced_weights(rows: list[dict[str, object]]) -> list[float]:
    counts = Counter(str(row["run_id"]) for row in rows)
    return [1.0 / max(counts[str(row["run_id"])], 1) for row in rows]


def transition_terms(row: dict[str, object], params: YieldParams, nominal_load: float) -> dict[str, float]:
    rel = max(f(row, "vbar_rel_mps"), f(row, "load_weighted_rel_mps"), 0.0)
    vf = f(row, "abs_forward_velocity_mps")
    yaw_contact = max(f(row, "vbar_yaw_mps"), f(row, "load_weighted_lat_mps"), rel, 0.0)
    vt = math.hypot(params.rel_weight * rel, vf)
    static_frac = rational_fade(vt, params.static_speed_mps, params.static_quartic)
    util = min(max(f(row, "actual_force_utilization"), 0.0), 5.0)
    limiter = min(max(f(row, "actual_force_limiter_activity"), 0.0), 5.0)
    load_ratio = min(max(f(row, "total_normal_load_n") / max(nominal_load, 1.0e-9), 0.25), 2.0)
    return {
        "rel": rel,
        "vf": vf,
        "yaw_contact": yaw_contact,
        "vt": vt,
        "static_frac": static_frac,
        "slide": 1.0 - static_frac,
        "speed_relief": rational_fade(vt, params.speed_fade_mps),
        "util": util,
        "util_smooth": util / (1.0 + util),
        "limiter_smooth": limiter / (1.0 + limiter),
        "load_scale": load_ratio**params.load_exp,
        "load_delta": load_ratio - 1.0,
    }


def yield_basis(row: dict[str, object], params: YieldParams, nominal_load: float) -> float:
    terms = transition_terms(row, params, nominal_load)
    velocity_activation = rational_rise(terms["yaw_contact"], params.yaw_activation_mps)
    force_activation = rational_rise(terms["util"], params.force_activation_util)
    activation = 1.0 - (1.0 - velocity_activation) * (1.0 - force_activation)
    util_gate = terms["util"] / (terms["util"] + params.util_k)
    mixed = params.slide_ratio + (1.0 - params.slide_ratio) * terms["static_frac"]
    return activation * terms["speed_relief"] * util_gate * terms["load_scale"] * mixed


def surface_schedule(row: dict[str, object], suffix: str, model: StaticYieldContactModel) -> float:
    terms = transition_terms(row, model.params, model.nominal_load_n)
    low_rel = 1.0 / (1.0 + (terms["rel"] / model.surface_rel_knee_mps) ** 2)
    low_forward = 1.0 / (1.0 + (terms["vf"] / model.surface_fwd_knee_mps) ** 2)
    moving = terms["slide"] * terms["slide"]
    if suffix == "moving":
        return moving
    if suffix == "low_rel_moving":
        return low_rel * moving
    if suffix == "util_moving":
        return terms["util_smooth"] * moving
    if suffix == "limiter_moving":
        return terms["limiter_smooth"] * moving
    if suffix == "limiter_signed_moving":
        return terms["limiter_smooth"] * moving * f(row, "yaw_sign")
    if suffix == "high_forward":
        return (1.0 - low_forward) * moving
    if suffix == "load_moving":
        return terms["load_delta"] * moving
    return 0.0


def surface_feature_value(row: dict[str, object], name: str, model: StaticYieldContactModel) -> float:
    base, suffix = name.split("__", 1)
    return f(row, base) * surface_schedule(row, suffix, model)


def solve_linear(a: list[list[float]], b: list[float]) -> list[float]:
    n = len(b)
    aug = [row[:] + [rhs] for row, rhs in zip(a, b)]
    for col in range(n):
        pivot = max(range(col, n), key=lambda r: abs(aug[r][col]))
        if abs(aug[pivot][col]) < 1.0e-14:
            continue
        if pivot != col:
            aug[col], aug[pivot] = aug[pivot], aug[col]
        div = aug[col][col]
        aug[col] = [x / div for x in aug[col]]
        for r in range(n):
            if r == col:
                continue
            factor = aug[r][col]
            if factor:
                aug[r] = [x - factor * y for x, y in zip(aug[r], aug[col])]
    return [aug[i][-1] for i in range(n)]


def predict_surface(row: dict[str, object], model: StaticYieldContactModel) -> float:
    total = 0.0
    for name, scale, beta in zip(model.surface.names, model.surface.scales, model.surface.beta):
        value = surface_feature_value(row, name, model)
        limit = 8.0 * scale
        value = min(max(value, -limit), limit)
        total += beta * (value / scale)
    return soft_cap(total, model.surface_cap_nm)


def predict_opposes(row: dict[str, object], model: StaticYieldContactModel) -> float:
    return model.static_peak_nm * yield_basis(row, model.params, model.nominal_load_n) + predict_surface(row, model)


def fit_surface(
    rows: list[dict[str, object]],
    weights: list[float],
    residual_target: list[float],
    model: StaticYieldContactModel,
    ridge: float,
) -> SurfaceModel:
    names = SURFACE_FEATURES[:]
    train_rows = [row for row, w in zip(rows, weights) if w > 0.0]
    scales: list[float] = []
    for name in names:
        vals = [abs(surface_feature_value(row, name, model)) for row in train_rows]
        scale = q(vals, 0.80)
        if scale < 1.0e-12:
            rms = math.sqrt(sum(v * v for v in vals) / len(vals)) if vals else 0.0
            scale = rms
        scales.append(scale if scale > 1.0e-12 else 1.0)

    p = len(names)
    xtx = [[0.0 for _ in range(p)] for _ in range(p)]
    xty = [0.0 for _ in range(p)]
    for idx, (row, w) in enumerate(zip(rows, weights)):
        if w <= 0.0:
            continue
        x: list[float] = []
        for name, scale in zip(names, scales):
            value = surface_feature_value(row, name, model)
            limit = 8.0 * scale
            value = min(max(value, -limit), limit)
            x.append(value / scale)
        y = residual_target[idx]
        for i in range(p):
            xi = x[i]
            xty[i] += w * xi * y
            for j in range(p):
                xtx[i][j] += w * xi * x[j]
    for i in range(p):
        xtx[i][i] += ridge
    return SurfaceModel(names=names, scales=scales, beta=solve_linear(xtx, xty), ridge=ridge)


def residuals_after(rows: list[dict[str, object]], predictions: list[float]) -> tuple[list[float], list[float], list[float]]:
    baseline: list[float] = []
    corrected: list[float] = []
    predicted_raw: list[float] = []
    for row, pred_opposes in zip(rows, predictions):
        raw = f(row, "residual_additive_yaw_torque_nm")
        pred_raw = -f(row, "yaw_sign") * pred_opposes
        baseline.append(raw)
        predicted_raw.append(pred_raw)
        corrected.append(raw - pred_raw)
    return baseline, corrected, predicted_raw


def metric_row(label: str, rows: list[dict[str, object]], predictions: list[float]) -> dict[str, object]:
    baseline, corrected, predicted_raw = residuals_after(rows, predictions)
    weights = run_balanced_weights(rows)
    baseline_rmse = rmse(baseline)
    corrected_rmse = rmse(corrected)
    rb_base = weighted_rmse(baseline, weights)
    rb_corr = weighted_rmse(corrected, weights)
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
        "run_balanced_baseline_rmse_nm": rb_base,
        "run_balanced_corrected_rmse_nm": rb_corr,
        "run_balanced_baseline_mae_nm": weighted_mae(baseline, weights),
        "run_balanced_corrected_mae_nm": weighted_mae(corrected, weights),
        "prediction_rmse_nm": rmse(predicted_raw),
        "improvement_fraction_rmse": 1.0 - corrected_rmse / max(baseline_rmse, 1.0e-12),
        "run_balanced_improvement_fraction_rmse": 1.0 - rb_corr / max(rb_base, 1.0e-12),
    }


def subset_predictions(all_predictions: list[float], subset: list[dict[str, object]]) -> list[float]:
    return [all_predictions[int(row["_idx"])] for row in subset]


def split_rows(rows: list[dict[str, object]], split: str) -> list[dict[str, object]]:
    return [row for row in rows if row.get("dataset_split") == split]


def validation_rows(rows: list[dict[str, object]]) -> list[dict[str, object]]:
    splits = {"open_floor_fit_downweighted", "open_floor_validation_only", "diag_validation_only", "aux_downweighted_validation"}
    return [row for row in rows if row.get("dataset_split") in splits]


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
    current = applied_voltage / resistance - (wheel_speed_radps * (gear_ratio / speed_constant)) / resistance
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
    current = motor_torque / torque_constant + no_load_sign * no_load
    back_emf = wheel_speed_radps * (gear_ratio / speed_constant)
    return ((current * resistance) + back_emf) / battery


def static_launch_torque(constants: dict[str, float]) -> float:
    return max(0.0, torque_from_command(constants["static_launch_command"], 0.0, constants))


def signed_direction(preferred: float, fallback: float) -> float:
    preferred_sign = sign(preferred)
    return preferred_sign if preferred_sign else sign(fallback)


def wheel_speeds(vf_mps: float, yaw_rate: float, constants: dict[str, float]) -> tuple[float, float, float, float]:
    half_track = 0.5 * constants["track_width_m"]
    radius = constants["wheel_radius_m"]
    left_surface = vf_mps + half_track * yaw_rate
    right_surface = vf_mps - half_track * yaw_rate
    return left_surface, right_surface, left_surface / radius, right_surface / radius


def command_torque_for_applied(applied_torque: float, wheel_speed_radps: float, constants: dict[str, float]) -> tuple[float, float]:
    surface_speed = constants["wheel_radius_m"] * wheel_speed_radps
    launch = static_launch_torque(constants) * math.exp(-((abs(surface_speed) / constants["static_friction_max_speed_mps"]) ** 2))
    launch_dir = signed_direction(applied_torque, wheel_speed_radps)
    loss_dir = signed_direction(wheel_speed_radps, applied_torque)
    rolling = constants["rolling_friction_torque_nm"] * loss_dir
    command_torque = applied_torque
    if signed_direction(applied_torque, wheel_speed_radps) != 0.0:
        command_torque += launch_dir * launch + rolling
    return command_torque, launch


def motor_commands_for_opposing_torque(opposing_yaw_torque: float, constants: dict[str, float], vf_mps: float, yaw_rate: float) -> dict[str, float]:
    radius = constants["wheel_radius_m"]
    track = constants["track_width_m"]
    left_surface, right_surface, left_speed, right_speed = wheel_speeds(vf_mps, yaw_rate, constants)
    applied_bank_torque = opposing_yaw_torque * radius / track
    left_torque, left_launch = command_torque_for_applied(applied_bank_torque, left_speed, constants)
    right_torque, right_launch = command_torque_for_applied(-applied_bank_torque, right_speed, constants)
    left_command = command_from_torque(left_torque, left_speed, constants)
    right_command = command_from_torque(right_torque, right_speed, constants)
    return {
        "applied_bank_torque_nm": applied_bank_torque,
        "left_command_torque_nm": left_torque,
        "right_command_torque_nm": right_torque,
        "left_command": left_command,
        "right_command": right_command,
        "lr_delta_command": left_command - right_command,
        "left_surface_mps": left_surface,
        "right_surface_mps": right_surface,
        "left_wheel_speed_radps": left_speed,
        "right_wheel_speed_radps": right_speed,
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


def baseline_opposing_yaw_torque(constants: dict[str, float], yaw_rate: float) -> float:
    longitudinal = constants["drive_wheel_longitudinal_offset_m"]
    front_right_velocity = -longitudinal * yaw_rate
    rear_right_velocity = longitudinal * yaw_rate
    front_right_force_total = 2.0 * constants["front_right_contact_force_gain_n_per_mps"] * front_right_velocity
    rear_right_force_total = 2.0 * constants["rear_right_contact_force_gain_n_per_mps"] * rear_right_velocity
    yaw_moment = longitudinal * (front_right_force_total - rear_right_force_total)
    return -yaw_moment


def force_domain_stribeck_extra(base_opposing: float, constants: dict[str, float], vf_mps: float, yaw_rate: float) -> float:
    coeff = read_key_value_csv(FORCE_DOMAIN_COEFF)
    longitudinal = constants["drive_wheel_longitudinal_offset_m"]
    vbar_rel = longitudinal * abs(yaw_rate)
    transition_speed = math.hypot(coeff["rel_weight"] * vbar_rel, abs(vf_mps))
    stribeck = math.exp(-((transition_speed / coeff["stribeck_speed_mps"]) ** 2))
    speed_relief = 1.0 / (1.0 + (transition_speed / coeff["speed_fade_mps"]) ** 2)
    yield_nm = max(coeff["nominal_longitudinal_yield_nm"], 1.0e-9)
    extra = 0.0
    for _ in range(50):
        utilization = smooth_positive(base_opposing + extra) / yield_nm
        activation = 1.0 - math.exp(-((utilization / coeff["utilization_activation"]) ** 2))
        next_extra = activation * speed_relief * (coeff["static_extra_nm"] * stribeck + coeff["sliding_nm"])
        if abs(next_extra - extra) < 1.0e-12:
            return next_extra
        extra = next_extra
    return extra


def variant_c_extra(base_opposing: float, constants: dict[str, float], vf_mps: float, yaw_rate: float) -> float:
    coeff_rows = [row for row in read_coeff_rows(C_COEFF) if row["candidate"] == "saturation_aware_surface"]
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
    low_rel = 1.0 / (1.0 + (v_rel / vrel_knee) ** 2)
    high_forward = 1.0 - 1.0 / (1.0 + (abs(vf_mps) / fwd_knee) ** 2)
    right_front = 2.0 * (longitudinal * longitudinal) * abs(yaw_rate)
    right_rear = right_front

    def feature_value(feature: str, extra: float) -> float:
        total_req = base_opposing + extra
        values = {
            "gain_front_right_basis": right_front,
            "gain_rear_right_basis": right_rear,
            "gain_left_long_basis": 0.0,
            "gain_right_long_basis": 0.0,
            "force_gap_opposes_yaw_nm": -base_opposing,
            "req_moment_opposes_yaw_nm": -total_req,
            "force_moment_opposes_yaw_nm": -extra,
            "req_abs_contact_moment_nm": abs(total_req),
            "force_abs_contact_moment_nm": abs(extra),
        }
        base, suffix = feature.split("__", 1)
        value = values.get(base, 0.0)
        if suffix == "base":
            return value
        if suffix == "low_rel":
            return value * low_rel
        if suffix == "high_forward":
            return value * high_forward
        if suffix == "util":
            return value * util_smooth
        if suffix == "limiter":
            return value * limiter_smooth
        if suffix == "limiter_signed":
            return value * limiter_smooth
        return 0.0

    extra = 0.0
    for _ in range(50):
        predicted = 0.0
        for feature, beta in coeffs.items():
            scale = max(scales.get(feature, 1.0), 1.0e-12)
            value = min(max(feature_value(feature, extra), -8.0 * scale), 8.0 * scale)
            predicted += beta * (value / scale)
        if abs(predicted - extra) < 1.0e-12:
            return predicted
        extra = predicted
    return extra


def synthetic_yield_basis(base_plus_extra: float, constants: dict[str, float], params: YieldParams, nominal_load: float, vf_mps: float, yaw_rate: float) -> float:
    longitudinal = constants["drive_wheel_longitudinal_offset_m"]
    rel = longitudinal * abs(yaw_rate)
    vt = math.hypot(params.rel_weight * rel, abs(vf_mps))
    static_frac = rational_fade(vt, params.static_speed_mps, params.static_quartic)
    speed_relief = rational_fade(vt, params.speed_fade_mps)
    util = min(max(contact_utilization(constants, yaw_rate), 0.0), 5.0)
    util_gate = util / (util + params.util_k)
    velocity_activation = rational_rise(rel, params.yaw_activation_mps)
    force_activation = rational_rise(util, params.force_activation_util)
    activation = 1.0 - (1.0 - velocity_activation) * (1.0 - force_activation)
    mixed = params.slide_ratio + (1.0 - params.slide_ratio) * static_frac
    return activation * speed_relief * util_gate * mixed * (nominal_load / max(nominal_load, 1.0e-9)) ** params.load_exp


def synthetic_surface_feature_value(constants: dict[str, float], model: StaticYieldContactModel, name: str, vf_mps: float, yaw_rate: float, extra: float) -> float:
    longitudinal = constants["drive_wheel_longitudinal_offset_m"]
    rel = longitudinal * abs(yaw_rate)
    vt = math.hypot(model.params.rel_weight * rel, abs(vf_mps))
    static_frac = rational_fade(vt, model.params.static_speed_mps, model.params.static_quartic)
    moving = (1.0 - static_frac) * (1.0 - static_frac)
    low_rel = 1.0 / (1.0 + (rel / model.surface_rel_knee_mps) ** 2)
    low_forward = 1.0 / (1.0 + (abs(vf_mps) / model.surface_fwd_knee_mps) ** 2)
    util = min(max(contact_utilization(constants, yaw_rate), 0.0), 5.0)
    limiter = min(max(limiter_activity(constants, yaw_rate), 0.0), 5.0)
    base_opposes = baseline_opposing_yaw_torque(constants, yaw_rate)
    bases = {
        "gain_front_right_basis": 2.0 * longitudinal * longitudinal * abs(yaw_rate),
        "gain_rear_right_basis": 2.0 * longitudinal * longitudinal * abs(yaw_rate),
        "gain_left_long_basis": 0.0,
        "gain_right_long_basis": 0.0,
        "force_moment_opposes_yaw_nm": base_opposes + extra,
        "force_abs_contact_moment_nm": abs(extra),
    }
    base, suffix = name.split("__", 1)
    schedule = 0.0
    if suffix == "moving":
        schedule = moving
    elif suffix == "low_rel_moving":
        schedule = low_rel * moving
    elif suffix == "util_moving":
        schedule = util / (1.0 + util) * moving
    elif suffix == "limiter_moving":
        schedule = limiter / (1.0 + limiter) * moving
    elif suffix == "limiter_signed_moving":
        schedule = limiter / (1.0 + limiter) * moving
    elif suffix == "high_forward":
        schedule = (1.0 - low_forward) * moving
    return bases.get(base, 0.0) * schedule


def synthetic_model_extra(constants: dict[str, float], model: StaticYieldContactModel, vf_mps: float, yaw_rate: float) -> float:
    base = baseline_opposing_yaw_torque(constants, yaw_rate)
    extra = 0.0
    for _ in range(50):
        yield_part = model.static_peak_nm * synthetic_yield_basis(base + extra, constants, model.params, model.nominal_load_n, vf_mps, yaw_rate)
        surface_part = 0.0
        for name, scale, beta in zip(model.surface.names, model.surface.scales, model.surface.beta):
            value = synthetic_surface_feature_value(constants, model, name, vf_mps, yaw_rate, extra)
            value = min(max(value, -8.0 * scale), 8.0 * scale)
            surface_part += beta * (value / scale)
        surface_part = soft_cap(surface_part, model.surface_cap_nm)
        next_extra = yield_part + surface_part
        if abs(next_extra - extra) < 1.0e-12:
            return next_extra
        extra = next_extra
    return extra


def command_for_static_peak(constants: dict[str, float], params: YieldParams, nominal_load: float, static_peak: float, vf_mps: float, yaw_rate: float) -> dict[str, float]:
    base = baseline_opposing_yaw_torque(constants, yaw_rate)
    extra = 0.0
    for _ in range(50):
        next_extra = static_peak * synthetic_yield_basis(base + extra, constants, params, nominal_load, vf_mps, yaw_rate)
        if abs(next_extra - extra) < 1.0e-12:
            break
        extra = next_extra
    cmd = motor_commands_for_opposing_torque(base + extra, constants, vf_mps, yaw_rate)
    cmd["extra_opposing_yaw_torque_nm"] = extra
    cmd["total_opposing_yaw_torque_nm"] = base + extra
    return cmd


def command_for_model(constants: dict[str, float], model: StaticYieldContactModel, vf_mps: float, yaw_rate: float) -> dict[str, float]:
    base = baseline_opposing_yaw_torque(constants, yaw_rate)
    extra = synthetic_model_extra(constants, model, vf_mps, yaw_rate)
    cmd = motor_commands_for_opposing_torque(base + extra, constants, vf_mps, yaw_rate)
    cmd["baseline_opposing_yaw_torque_nm"] = base
    cmd["extra_opposing_yaw_torque_nm"] = extra
    cmd["total_opposing_yaw_torque_nm"] = base + extra
    return cmd


def gate_pass(left_command: float, right_command: float) -> bool:
    return (
        abs(left_command) >= HARD_GATE_ABS_COMMAND_MIN
        and abs(right_command) >= HARD_GATE_ABS_COMMAND_MIN
    )


def min_static_peak_for_gate(constants: dict[str, float], params: YieldParams, nominal_load: float) -> float:
    lo = 0.0
    hi = 0.15
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        cmd = command_for_static_peak(constants, params, nominal_load, mid, 0.0, 1.0)
        if abs(cmd["left_command"]) >= HARD_GATE_ABS_COMMAND_MIN and abs(cmd["right_command"]) >= HARD_GATE_ABS_COMMAND_MIN:
            hi = mid
        else:
            lo = mid
    return hi


def static_peak_for_abs_command(constants: dict[str, float], params: YieldParams, nominal_load: float, abs_command: float) -> float:
    lo = 0.0
    hi = 0.15
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        cmd = command_for_static_peak(constants, params, nominal_load, mid, 0.0, 1.0)
        if abs(cmd["left_command"]) >= abs_command and abs(cmd["right_command"]) >= abs_command:
            hi = mid
        else:
            lo = mid
    return hi


def objective_for_predictions(rows: list[dict[str, object]], predictions: list[float]) -> tuple[float, dict[str, object]]:
    val = validation_rows(rows)
    val_metric = metric_row("validation_non_authoritative", val, subset_predictions(predictions, val))
    risks = risk_groups(rows)
    straight = metric_row("straightish_abs_yaw_lt_0p05", risks["straightish_abs_yaw_lt_0p05"], subset_predictions(predictions, risks["straightish_abs_yaw_lt_0p05"]))
    high = metric_row("high_forward_vf_ge_0p5", risks["high_forward_vf_ge_0p5"], subset_predictions(predictions, risks["high_forward_vf_ge_0p5"]))
    score = float(val_metric["run_balanced_corrected_rmse_nm"])
    if float(straight["run_balanced_corrected_rmse_nm"]) > 1.03 * max(float(straight["run_balanced_baseline_rmse_nm"]), 1.0e-12):
        score += 0.25 * (
            float(straight["run_balanced_corrected_rmse_nm"]) - float(straight["run_balanced_baseline_rmse_nm"])
        )
    if high["count"] and float(high["run_balanced_corrected_rmse_nm"]) > 1.05 * max(float(high["run_balanced_baseline_rmse_nm"]), 1.0e-12):
        score += 0.25 * (
            float(high["run_balanced_corrected_rmse_nm"]) - float(high["run_balanced_baseline_rmse_nm"])
        )
    return score, {
        "objective_score": score,
        "validation_rb_corrected_rmse_nm": val_metric["run_balanced_corrected_rmse_nm"],
        "validation_rb_baseline_rmse_nm": val_metric["run_balanced_baseline_rmse_nm"],
        "straight_rb_corrected_rmse_nm": straight["run_balanced_corrected_rmse_nm"],
        "straight_rb_baseline_rmse_nm": straight["run_balanced_baseline_rmse_nm"],
        "high_forward_rb_corrected_rmse_nm": high["run_balanced_corrected_rmse_nm"],
        "high_forward_rb_baseline_rmse_nm": high["run_balanced_baseline_rmse_nm"],
    }


def evaluate_model(rows: list[dict[str, object]], constants: dict[str, float], model: StaticYieldContactModel) -> tuple[list[float], dict[str, object], float]:
    gate = command_for_model(constants, model, 0.0, 1.0)
    model.hard_gate_left_command = gate["left_command"]
    model.hard_gate_right_command = gate["right_command"]
    model.hard_gate_lr_delta = gate["lr_delta_command"]
    model.hard_gate_extra_opposing_nm = gate["extra_opposing_yaw_torque_nm"]
    model.hard_gate_total_opposing_nm = gate["total_opposing_yaw_torque_nm"]
    model.hard_gate_pass = gate_pass(gate["left_command"], gate["right_command"])
    predictions = [predict_opposes(row, model) for row in rows]
    score, objective = objective_for_predictions(rows, predictions)
    score += 0.003 * abs(abs(gate["left_command"]) - HARD_GATE_ABS_COMMAND_TARGET) / 0.046
    return predictions, objective, score


def fit_models(rows: list[dict[str, object]], constants: dict[str, float]) -> tuple[StaticYieldContactModel, list[float], list[dict[str, object]]]:
    weights = fit_weights(rows)
    nominal_load = selected_nominal_load(rows)
    target = [f(row, "residual_opposes_yaw_nm") for row in rows]
    candidates: list[dict[str, object]] = []
    best: StaticYieldContactModel | None = None
    best_predictions: list[float] = []
    best_score = float("inf")

    for yaw_activation in [0.008]:
        for force_activation in [0.20]:
            for static_speed in [0.025, 0.035]:
                for speed_fade in [0.64]:
                    for util_k in [0.10]:
                        for slide_ratio in [0.00]:
                            for static_quartic in [0.0, 0.20]:
                                params = YieldParams(yaw_activation, force_activation, static_speed, speed_fade, 0.75, util_k, slide_ratio, static_quartic)
                                basis = [yield_basis(row, params, nominal_load) for row in rows]
                                denom = sum(w * b * b for w, b in zip(weights, basis))
                                fitted_peak = (
                                    sum(w * b * y for w, b, y in zip(weights, basis, target)) / denom
                                    if denom > 1.0e-12
                                    else 0.0
                                )
                                min_peak = min_static_peak_for_gate(constants, params, nominal_load)
                                target_peak = static_peak_for_abs_command(constants, params, nominal_load, HARD_GATE_ABS_COMMAND_TARGET)
                                static_peak = min(max(fitted_peak, target_peak), 0.180)
                                if min_peak > 0.180:
                                    candidates.append(
                                        {
                                            "accepted": 0,
                                            "reject_reason": "min_static_peak_exceeds_cap",
                                            "yaw_activation_mps": yaw_activation,
                                            "force_activation_util": force_activation,
                                            "static_speed_mps": static_speed,
                                            "speed_fade_mps": speed_fade,
                                            "util_k": util_k,
                                            "slide_ratio": slide_ratio,
                                            "static_quartic": static_quartic,
                                            "fitted_static_peak_nm": fitted_peak,
                                            "static_peak_nm": static_peak,
                                            "min_static_peak_nm": min_peak,
                                            "target_static_peak_nm": target_peak,
                                        }
                                    )
                                    continue

                                empty_surface = SurfaceModel(SURFACE_FEATURES[:], [1.0] * len(SURFACE_FEATURES), [0.0] * len(SURFACE_FEATURES), 0.0)
                                base_model = StaticYieldContactModel(
                                params=params,
                                static_peak_nm=static_peak,
                                surface_rel_knee_mps=0.060,
                                surface_fwd_knee_mps=0.700,
                                nominal_load_n=nominal_load,
                                surface=empty_surface,
                                surface_cap_nm=SURFACE_CAP_NM,
                                hard_gate_left_command=0.0,
                                hard_gate_right_command=0.0,
                                hard_gate_lr_delta=0.0,
                                hard_gate_extra_opposing_nm=0.0,
                                hard_gate_total_opposing_nm=0.0,
                                hard_gate_pass=False,
                                )

                                for ridge in [1.0e-3]:
                                    base_model.static_peak_nm = static_peak
                                    residual_target = [
                                        y - static_peak * b
                                        for y, b in zip(target, basis)
                                    ]
                                    base_model.surface = fit_surface(rows, weights, residual_target, base_model, ridge)

                                    gate = command_for_model(constants, base_model, 0.0, 1.0)
                                    if not gate_pass(gate["left_command"], gate["right_command"]):
                                        lo = static_peak
                                        hi = 0.180
                                        for _ in range(50):
                                            mid = 0.5 * (lo + hi)
                                            base_model.static_peak_nm = mid
                                            gate_mid = command_for_model(constants, base_model, 0.0, 1.0)
                                            if abs(gate_mid["left_command"]) >= HARD_GATE_ABS_COMMAND_MIN and abs(gate_mid["right_command"]) >= HARD_GATE_ABS_COMMAND_MIN:
                                                hi = mid
                                            else:
                                                lo = mid
                                        base_model.static_peak_nm = max(hi, target_peak)
                                        residual_target = [
                                            y - base_model.static_peak_nm * b for y, b in zip(target, basis)
                                        ]
                                        base_model.surface = fit_surface(rows, weights, residual_target, base_model, ridge)

                                    predictions, objective, score = evaluate_model(rows, constants, base_model)
                                    row = {
                                        "accepted": int(base_model.hard_gate_pass),
                                        "reject_reason": "" if base_model.hard_gate_pass else "outside_abs_0p6_1radps_gate",
                                        "yaw_activation_mps": yaw_activation,
                                        "force_activation_util": force_activation,
                                        "static_speed_mps": static_speed,
                                        "speed_fade_mps": speed_fade,
                                        "rel_weight": params.rel_weight,
                                        "util_k": util_k,
                                        "slide_ratio": slide_ratio,
                                        "static_quartic": static_quartic,
                                        "surface_ridge": ridge,
                                        "fitted_static_peak_nm": fitted_peak,
                                        "static_peak_nm": base_model.static_peak_nm,
                                        "sliding_yield_nm": base_model.sliding_yield_nm,
                                        "min_static_peak_nm": min_peak,
                                        "target_static_peak_nm": target_peak,
                                        "hard_gate_left_command": base_model.hard_gate_left_command,
                                        "hard_gate_right_command": base_model.hard_gate_right_command,
                                        "hard_gate_lr_delta": base_model.hard_gate_lr_delta,
                                        "hard_gate_extra_opposing_nm": base_model.hard_gate_extra_opposing_nm,
                                        **objective,
                                    }
                                    candidates.append(row)
                                    if base_model.hard_gate_pass and score < best_score:
                                        best_score = score
                                        best = StaticYieldContactModel(
                                            params=params,
                                            static_peak_nm=base_model.static_peak_nm,
                                            surface_rel_knee_mps=base_model.surface_rel_knee_mps,
                                            surface_fwd_knee_mps=base_model.surface_fwd_knee_mps,
                                            nominal_load_n=nominal_load,
                                            surface=SurfaceModel(base_model.surface.names[:], base_model.surface.scales[:], base_model.surface.beta[:], ridge),
                                            surface_cap_nm=SURFACE_CAP_NM,
                                            hard_gate_left_command=base_model.hard_gate_left_command,
                                            hard_gate_right_command=base_model.hard_gate_right_command,
                                            hard_gate_lr_delta=base_model.hard_gate_lr_delta,
                                            hard_gate_extra_opposing_nm=base_model.hard_gate_extra_opposing_nm,
                                            hard_gate_total_opposing_nm=base_model.hard_gate_total_opposing_nm,
                                            hard_gate_pass=base_model.hard_gate_pass,
                                        )
                                        best_predictions = predictions[:]

    if best is None:
        raise RuntimeError("No model satisfied explicit |cmd| >= 0.6 at +1 rad/s in-place")
    candidates.sort(key=lambda row: (int(row.get("accepted", 0)) == 0, float(row.get("objective_score", 999.0))))
    return best, best_predictions, candidates


def metric_tables(rows: list[dict[str, object]], predictions: list[float]) -> tuple[list[dict[str, object]], list[dict[str, object]], list[dict[str, object]], list[dict[str, object]]]:
    split_metrics: list[dict[str, object]] = []
    for split in [
        "primary_open_floor_fit_authoritative",
        "open_floor_fit_downweighted",
        "open_floor_validation_only",
        "diag_validation_only",
        "aux_downweighted_validation",
    ]:
        subset = split_rows(rows, split)
        if subset:
            split_metrics.append(metric_row(split, subset, subset_predictions(predictions, subset)))
    val = validation_rows(rows)
    split_metrics.append(metric_row("validation_non_authoritative", val, subset_predictions(predictions, val)))

    phase_metrics: list[dict[str, object]] = []
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
                out = metric_row(f"{split}:{phase}", subset, subset_predictions(predictions, subset))
                out["dataset_split"] = split
                out["physics_phase"] = phase
                phase_metrics.append(out)

    selected_metrics: list[dict[str, object]] = []
    for run_id in SELECTED_RUNS:
        subset = [row for row in rows if row.get("run_id") == run_id]
        if subset:
            out = metric_row(run_id, subset, subset_predictions(predictions, subset))
            out["run_id"] = run_id
            out["present"] = 1
            out["dataset_split"] = subset[0].get("dataset_split", "")
            selected_metrics.append(out)
        else:
            selected_metrics.append({"run_id": run_id, "present": 0, "dataset_split": "", "count": 0})

    risk_metrics: list[dict[str, object]] = []
    for group, subset in risk_groups(rows).items():
        if subset:
            risk_metrics.append(metric_row(group, subset, subset_predictions(predictions, subset)))
    return split_metrics, phase_metrics, selected_metrics, risk_metrics


def compare_split_metrics(ours: list[dict[str, object]]) -> list[dict[str, object]]:
    with FORCE_DOMAIN_SPLIT.open(newline="", encoding="utf-8") as fh:
        b = {row["dataset_split"]: row for row in csv.DictReader(fh)}
    validation_parts = ["open_floor_fit_downweighted", "open_floor_validation_only", "diag_validation_only", "aux_downweighted_validation"]
    if "validation_non_authoritative" not in b:
        count = 0
        corrected_sum = 0.0
        mae_sum = 0.0
        for split in validation_parts:
            part = b.get(split)
            if not part:
                continue
            n = int(float(part["count"]))
            count += n
            corrected_sum += n * float(part["corrected_rmse_nm"]) ** 2
            mae_sum += n * float(part["corrected_mae_nm"])
        if count:
            b["validation_non_authoritative"] = {
                "corrected_rmse_nm": math.sqrt(corrected_sum / count),
                "corrected_mae_nm": mae_sum / count,
            }
    with C_SPLIT.open(newline="", encoding="utf-8") as fh:
        c = {row["group"]: row for row in csv.DictReader(fh)}
    out: list[dict[str, object]] = []
    for row in ours:
        split = str(row["group"])
        out.append(
            {
                "split": split,
                "count": row["count"],
                "baseline_rmse_nm": row["baseline_rmse_nm"],
                "force_domain_stribeck_corrected_rmse_nm": b.get(split, {}).get("corrected_rmse_nm", ""),
                "c_combined_slip_corrected_rmse_nm": c.get(split, {}).get("corrected_rmse_nm", ""),
                "pade_static_yield_plus_c_corrected_rmse_nm": row["corrected_rmse_nm"],
                "force_domain_stribeck_corrected_mae_nm": b.get(split, {}).get("corrected_mae_nm", ""),
                "c_combined_slip_corrected_mae_nm": c.get(split, {}).get("corrected_mae_nm", ""),
                "pade_static_yield_plus_c_corrected_mae_nm": row["corrected_mae_nm"],
            }
        )
    return out


def compare_selected_metrics(ours: list[dict[str, object]]) -> list[dict[str, object]]:
    with FORCE_DOMAIN_SELECTED.open(newline="", encoding="utf-8") as fh:
        b = {row["run_id"]: row for row in csv.DictReader(fh)}
    with C_SELECTED.open(newline="", encoding="utf-8") as fh:
        c = {row["run_id"]: row for row in csv.DictReader(fh)}
    ours_by_run = {str(row["run_id"]): row for row in ours if "run_id" in row}
    out: list[dict[str, object]] = []
    for run_id in SELECTED_RUNS:
        row = ours_by_run.get(run_id, {"present": 0})
        if int(row.get("present", 0)) == 0:
            out.append({"run_id": run_id, "present": 0})
            continue
        out.append(
            {
                "run_id": run_id,
                "present": 1,
                "dataset_split": row["dataset_split"],
                "count": row["count"],
                "baseline_rmse_nm": row["baseline_rmse_nm"],
                "force_domain_stribeck_corrected_rmse_nm": b.get(run_id, {}).get("corrected_rmse_nm", ""),
                "c_combined_slip_corrected_rmse_nm": c.get(run_id, {}).get("corrected_rmse_nm", ""),
                "pade_static_yield_plus_c_corrected_rmse_nm": row["corrected_rmse_nm"],
                "baseline_signed_median_nm": row["baseline_signed_median_nm"],
                "pade_signed_median_nm": row["corrected_signed_median_nm"],
            }
        )
    return out


def vf_grid() -> list[float]:
    return [round(i * 0.15 / 5.0, 9) for i in range(6)]


def yaw_grid() -> list[float]:
    return [round(0.2 + i * (6.0 - 0.2) / 9.0, 9) for i in range(10)]


def one_rad_summary(constants: dict[str, float], model: StaticYieldContactModel) -> list[dict[str, object]]:
    base = baseline_opposing_yaw_torque(constants, 1.0)
    variants = [
        ("Baseline", 0.0),
        ("ForceDomainStribeck", force_domain_stribeck_extra(base, constants, 0.0, 1.0)),
        ("C_combined_slip", variant_c_extra(base, constants, 0.0, 1.0)),
        ("Pade_static_yield_plus_C", synthetic_model_extra(constants, model, 0.0, 1.0)),
    ]
    rows = []
    for variant, extra in variants:
        cmd = motor_commands_for_opposing_torque(base + extra, constants, 0.0, 1.0)
        rows.append(
            {
                "variant": variant,
                "baseline_opposing_yaw_torque_nm": base,
                "extra_opposing_yaw_torque_nm": extra,
                "total_opposing_yaw_torque_nm": base + extra,
                "left_command": cmd["left_command"],
                "right_command": cmd["right_command"],
                "lr_delta_command": cmd["lr_delta_command"],
                "passes_abs_0p6_gate": int(abs(cmd["left_command"]) >= HARD_GATE_ABS_COMMAND_MIN and abs(cmd["right_command"]) >= HARD_GATE_ABS_COMMAND_MIN),
            }
        )
    return rows


def grid_rows(constants: dict[str, float], model: StaticYieldContactModel) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for vf in vf_grid():
        for yaw in yaw_grid():
            base = baseline_opposing_yaw_torque(constants, yaw)
            variants = [
                ("Baseline", 0.0, "current raw right-contact scrub approximation"),
                ("ForceDomainStribeck", force_domain_stribeck_extra(base, constants, vf, yaw), "projected-force utilization Stribeck rewrite"),
                ("C_combined_slip", variant_c_extra(base, constants, vf, yaw), "prior combined-slip surface"),
                ("Pade_static_yield_plus_C", synthetic_model_extra(constants, model, vf, yaw), "Pade static yield plus rational-gated moving combined-slip residual"),
            ]
            util = contact_utilization(constants, yaw)
            limiter = limiter_activity(constants, yaw)
            for variant, extra, caveat in variants:
                total = base + extra
                cmd = motor_commands_for_opposing_torque(total, constants, vf, yaw)
                rows.append(
                    {
                        "vf_mps": vf,
                        "yaw_rate_radps": yaw,
                        "variant": variant,
                        "baseline_opposing_yaw_torque_nm": base,
                        "extra_opposing_yaw_torque_nm": extra,
                        "total_opposing_yaw_torque_nm": total,
                        "left_command": cmd["left_command"],
                        "right_command": cmd["right_command"],
                        "lr_delta_command": cmd["lr_delta_command"],
                        "left_surface_mps": cmd["left_surface_mps"],
                        "right_surface_mps": cmd["right_surface_mps"],
                        "left_command_torque_nm": cmd["left_command_torque_nm"],
                        "right_command_torque_nm": cmd["right_command_torque_nm"],
                        "max_abs_command": max(abs(cmd["left_command"]), abs(cmd["right_command"])),
                        "command_outside_unit": max(abs(cmd["left_command"]), abs(cmd["right_command"])) > 1.0,
                        "contact_utilization_raw": util,
                        "limiter_activity_proxy": limiter,
                        "contact_projection_sensitive": util > 1.0,
                        "low_speed_launch_sensitive": min(abs(cmd["left_surface_mps"]), abs(cmd["right_surface_mps"]))
                        < 2.0 * constants["static_friction_max_speed_mps"],
                        "caveat": caveat,
                    }
                )
    return rows


def pivot_lines(rows: list[dict[str, object]]) -> list[str]:
    variants = ["Baseline", "ForceDomainStribeck", "C_combined_slip", "Pade_static_yield_plus_C"]
    by_key = {(row["variant"], row["vf_mps"], row["yaw_rate_radps"]): row for row in rows}
    lines = [
        "# Round 2 L/R Command Delta Grid",
        "",
        "Values are `left_cmd - right_cmd` for positive clockwise yaw. Commands use the same motor inverse and launch/rolling-friction approximation as the prior grid worker.",
        "",
        "Yaw-rate columns are rad/s; `Vf` rows are m/s.",
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
    saturated = sum(1 for row in rows if row["command_outside_unit"])
    projection = sum(1 for row in rows if row["contact_projection_sensitive"])
    launch = sum(1 for row in rows if row["low_speed_launch_sensitive"])
    lines.extend(
        [
            "## Flags",
            "",
            f"- Rows with `|cmd| > 1`: {saturated} of {len(rows)}.",
            f"- Rows where raw contact utilization exceeds 1.0: {projection} of {len(rows)}.",
            f"- Rows near a wheel zero-crossing where launch friction matters: {launch} of {len(rows)}.",
            "",
            "Full per-point commands, torques, contact-utilization flags, and caveats are in `lr_delta_grid.csv`.",
        ]
    )
    return lines


def coefficient_rows(model: StaticYieldContactModel) -> list[dict[str, object]]:
    rows = []
    for name, scale, beta in zip(model.surface.names, model.surface.scales, model.surface.beta):
        rows.append(
            {
                "feature": name,
                "standardized_coefficient_nm": beta,
                "feature_scale": scale,
                "raw_coefficient_nm_per_feature": beta / scale,
                "abs_standardized_coefficient_nm": abs(beta),
            }
        )
    rows.sort(key=lambda row: float(row["abs_standardized_coefficient_nm"]), reverse=True)
    return rows


def parameter_rows(model: StaticYieldContactModel) -> list[dict[str, object]]:
    return [
        {"parameter": "yaw_activation_mps", "value": model.params.yaw_activation_mps, "unit": "m/s"},
        {"parameter": "force_activation_util", "value": model.params.force_activation_util, "unit": "actual force utilization"},
        {"parameter": "static_speed_mps", "value": model.params.static_speed_mps, "unit": "m/s"},
        {"parameter": "speed_fade_mps", "value": model.params.speed_fade_mps, "unit": "m/s"},
        {"parameter": "rel_weight", "value": model.params.rel_weight, "unit": "dimensionless"},
        {"parameter": "util_k", "value": model.params.util_k, "unit": "dimensionless utilization"},
        {"parameter": "slide_ratio", "value": model.params.slide_ratio, "unit": "sliding/static ratio"},
        {"parameter": "static_quartic", "value": model.params.static_quartic, "unit": "Pade x^4 denominator coefficient"},
        {"parameter": "load_exp", "value": model.params.load_exp, "unit": "dimensionless"},
        {"parameter": "static_peak_nm", "value": model.static_peak_nm, "unit": "Nm"},
        {"parameter": "sliding_yield_nm", "value": model.sliding_yield_nm, "unit": "Nm"},
        {"parameter": "surface_rel_knee_mps", "value": model.surface_rel_knee_mps, "unit": "m/s"},
        {"parameter": "surface_fwd_knee_mps", "value": model.surface_fwd_knee_mps, "unit": "m/s"},
        {"parameter": "surface_ridge", "value": model.surface.ridge, "unit": "ridge"},
        {"parameter": "surface_cap_nm", "value": model.surface_cap_nm, "unit": "Nm"},
        {"parameter": "nominal_load_n", "value": model.nominal_load_n, "unit": "N"},
        {"parameter": "hard_gate_abs_command_min", "value": HARD_GATE_ABS_COMMAND_MIN, "unit": "command"},
        {"parameter": "hard_gate_left_command_1radps", "value": model.hard_gate_left_command, "unit": "command"},
        {"parameter": "hard_gate_right_command_1radps", "value": model.hard_gate_right_command, "unit": "command"},
        {"parameter": "hard_gate_lr_delta_1radps", "value": model.hard_gate_lr_delta, "unit": "command delta"},
        {"parameter": "hard_gate_extra_opposing_nm", "value": model.hard_gate_extra_opposing_nm, "unit": "Nm"},
        {"parameter": "hard_gate_pass", "value": int(model.hard_gate_pass), "unit": "boolean"},
    ]


def make_prediction_sample(rows: list[dict[str, object]], predictions: list[float], model: StaticYieldContactModel) -> list[dict[str, object]]:
    out: list[dict[str, object]] = []
    counts: defaultdict[str, int] = defaultdict(int)
    for row in rows:
        run_id = str(row["run_id"])
        if run_id not in SELECTED_RUNS or counts[run_id] >= 25:
            continue
        pred_opposes = predictions[int(row["_idx"])]
        yield_part = model.static_peak_nm * yield_basis(row, model.params, model.nominal_load_n)
        pred_additive = -f(row, "yaw_sign") * pred_opposes
        out.append(
            {
                "run_id": run_id,
                "dataset_split": row.get("dataset_split", ""),
                "physics_phase": row.get("physics_phase", ""),
                "row_index": row.get("row_index", ""),
                "forward_velocity_mps": f(row, "forward_velocity_mps"),
                "yaw_rate_radps": f(row, "yaw_rate_radps"),
                "vbar_rel_mps": f(row, "vbar_rel_mps"),
                "max_force_preprojection_utilization": f(row, "max_force_preprojection_utilization"),
                "yield_predicted_opposes_nm": yield_part,
                "surface_predicted_opposes_nm": pred_opposes - yield_part,
                "total_predicted_opposes_nm": pred_opposes,
                "residual_additive_yaw_torque_nm": f(row, "residual_additive_yaw_torque_nm"),
                "predicted_additive_yaw_torque_nm": pred_additive,
                "corrected_residual_yaw_torque_nm": f(row, "residual_additive_yaw_torque_nm") - pred_additive,
            }
        )
        counts[run_id] += 1
    return out


def fmt(value: object, digits: int = 6) -> str:
    try:
        return f"{float(value):.{digits}f}"
    except (TypeError, ValueError):
        return str(value)


def markdown_table(rows: list[dict[str, object]], columns: list[str], digits: int = 6) -> list[str]:
    lines = ["| " + " | ".join(columns) + " |", "| " + " | ".join("---" for _ in columns) + " |"]
    for row in rows:
        lines.append("| " + " | ".join(fmt(row.get(col, ""), digits) for col in columns) + " |")
    return lines


def make_report(
    rows: list[dict[str, object]],
    model: StaticYieldContactModel,
    split_metrics: list[dict[str, object]],
    selected_metrics: list[dict[str, object]],
    risk_metrics: list[dict[str, object]],
    split_comparison: list[dict[str, object]],
    selected_comparison: list[dict[str, object]],
    one_rad: list[dict[str, object]],
    coeff_rows: list[dict[str, object]],
    candidate_rows: list[dict[str, object]],
) -> None:
    accepted = [row for row in candidate_rows if int(row.get("accepted", 0)) == 1]
    one_rad_round2 = [row for row in one_rad if row["variant"] == "Pade_static_yield_plus_C"][0]
    pass_label = "PASS" if int(one_rad_round2["passes_abs_0p6_gate"]) else "FAIL"
    best_split = {row["group"]: row for row in split_metrics}
    train = best_split["primary_open_floor_fit_authoritative"]
    validation = best_split["validation_non_authoritative"]
    lines: list[str] = [
        "# Pade Static-Yield Plus C Moving-Contact Yaw Model",
        "",
        "Analysis-only output. Production code, build metadata, and tests were not edited.",
        "",
        "## Model Form",
        "",
        "The model predicts yaw-opposing residual torque and converts it back to additive yaw torque with `M_add = -sign(yaw) * M_opp`.",
        "",
        "`M_opp = M_yield + M_move_surface`",
        "",
        "`M_yield = A_state(v_yaw_contact, u_force) * R_fade(v_t) * R_force(u_force) * L(N) * M0 * [r_slide + (1-r_slide) * S(v_t)]`",
        "",
        "where `A_state = 1-(1-v_y^2/(v_y^2+k_y^2))*(1-u^2/(u^2+k_u^2))`, `v_t = sqrt((rel_weight*v_rel)^2 + Vf^2)`, `S(v)=1/(1+x^2+b*x^4)`, `R_fade(v)=1/(1+(v/v_fade)^2)`, `R_force(u)=u/(u+u_k)`, and `L(N)=(N/N_nom)^load_exp`.",
        "",
        "`M_move_surface` is a C-style ridge surface over per-contact velocity bases and actual projected-force bases. It is gated in by the rational moving schedule `(1-S(v_t))^2` and bounded by `M_cap * z/sqrt(1+z^2)`, where `z=M_raw/M_cap`. It has no command/request selectors, maneuver-label inputs, trig, tanh, or exp in the selected law.",
        "",
        "Command/request values are not inputs to the traction/resistance prediction. They are used only downstream in the motor inverse that asks how much command would be needed to supply the predicted physical opposing torque. The selected moving branch excludes the request/gap features used by some prior diagnostic C-style surfaces.",
        "",
        "The fit rejects candidates outside the hard gate before scoring RMSE: at `Vf=0`, `Vr=0`, `yaw=+1 rad/s`, both left and right command magnitudes must be at least `0.6`. The reference condition is approximately `+0.646/-0.646`.",
        "",
        "## New Mechanism Versus Prior Fits",
        "",
        "The mechanism tested here is the hardware-cheap transition: a rational static/pre-sliding yield envelope hands off to a rational-gated moving-contact surface. The static branch is fitted with a hard lower bound from the +1 rad/s in-place command before the moving surface is considered.",
        "",
        "The moving surface is only a residual corrector on `target - M_yield`; it is multiplied by `(1-S)^2`, so it is not allowed to be the source of the launch-scale breakaway torque. This keeps the launch command floor from the static branch while trying to recover C-like validation performance once the contact is sliding.",
        "",
        "## Fitted Parameters",
        "",
    ]
    lines.extend(markdown_table(parameter_rows(model), ["parameter", "value", "unit"]))
    lines.extend(["", "## 1 rad/s In-Place Command", ""])
    lines.append(f"Pade hard-gate result: **{pass_label}** (`|cmd| >= 0.6`; predicted left/right `{fmt(one_rad_round2['left_command'])}/{fmt(one_rad_round2['right_command'])}`).")
    lines.append("")
    lines.extend(
        markdown_table(
            one_rad,
            [
                "variant",
                "extra_opposing_yaw_torque_nm",
                "total_opposing_yaw_torque_nm",
                "left_command",
                "right_command",
                "lr_delta_command",
                "passes_abs_0p6_gate",
            ],
        )
    )
    lines.extend(["", "## Split RMSE Versus B/C", ""])
    lines.extend(
        markdown_table(
            split_comparison,
            [
                "split",
                "baseline_rmse_nm",
                "force_domain_stribeck_corrected_rmse_nm",
                "c_combined_slip_corrected_rmse_nm",
                "pade_static_yield_plus_c_corrected_rmse_nm",
            ],
        )
    )
    lines.extend(["", "## Selected Log RMSE Versus B/C", ""])
    lines.extend(
        markdown_table(
            selected_comparison,
            [
                "run_id",
                "dataset_split",
                "baseline_rmse_nm",
                "force_domain_stribeck_corrected_rmse_nm",
                "c_combined_slip_corrected_rmse_nm",
                "pade_static_yield_plus_c_corrected_rmse_nm",
                "pade_signed_median_nm",
            ],
        )
    )
    lines.extend(["", "## Risk Slices", ""])
    lines.extend(
        markdown_table(
            risk_metrics,
            ["group", "count", "baseline_rmse_nm", "corrected_rmse_nm", "baseline_median_abs_nm", "corrected_median_abs_nm"],
        )
    )
    lines.extend(["", "## Dominant Moving-Surface Coefficients", ""])
    lines.extend(markdown_table(coeff_rows[:14], ["feature", "standardized_coefficient_nm", "feature_scale", "raw_coefficient_nm_per_feature"]))
    lines.extend(
        [
            "",
            "## Evaluation Notes",
            "",
            f"- Fit-authoritative rows: {len(split_rows(rows, 'primary_open_floor_fit_authoritative'))}; non-authoritative validation rows: {len(validation_rows(rows))}.",
            f"- Accepted hard-gate candidates: {len(accepted)} of {len(candidate_rows)}.",
            f"- Fit-authoritative corrected RMSE: {fmt(train['corrected_rmse_nm'])} Nm versus baseline {fmt(train['baseline_rmse_nm'])} Nm.",
            f"- Non-authoritative validation corrected RMSE: {fmt(validation['corrected_rmse_nm'])} Nm versus baseline {fmt(validation['baseline_rmse_nm'])} Nm.",
            "- Verdict: this family passes the yaw-launch price of entry. The selected no-exp/no-tanh rational transition is still not production-ready as an overall replacement unless the split tables show it closes enough of Variant C's validation gap for the intended use.",
            "- Cost estimate: selected runtime needs contact-speed square roots, rational divides for static fade, activation, force/load schedules and moving gate, plus one sqrt soft-cap. It uses no trig, atan, sin/cos, tanh, or exp.",
            "",
            "## Failure Modes",
            "",
            "- The static-yield branch is deliberately constrained by the +1 rad/s in-place gate; it can over-add resistance in logs or cells where the current plant already over-resists yaw.",
            "- The model is memoryless. It does not represent pre-sliding displacement history, hysteresis, surface heating, or stick duration.",
            "- The synthetic command grid replays an approximate contact projection, not the full production force limiter. High-utilization cells are flagged and should not be treated as exact command requirements.",
            "- Near-zero yaw still depends on a continuous yaw/contact-force sign convention. A production implementation would need a continuity-preserving deadband or signed contact-yaw velocity input.",
            "- Rare limiter-scheduled force features have near-zero scales in this dataset; even though the sqrt soft-cap bounds output, those coefficients are weak evidence and should not be promoted directly.",
            "- Load scheduling is fitted mostly around the logged fan/load range; extrapolation to very different fan duty or normal load is weak evidence.",
            "",
            "## Output Files",
            "",
            "- `fit_pade_static_yield_plus_c.py`",
            "- `pade_static_yield_plus_c_report.md`",
            "- `pade_static_yield_plus_c_parameters.csv`",
            "- `pade_static_yield_plus_c_surface_coefficients.csv`",
            "- `candidate_scores.csv`",
            "- `split_metrics.csv`",
            "- `phase_metrics.csv`",
            "- `selected_log_metrics.csv`",
            "- `risk_metrics.csv`",
            "- `split_comparison_vs_b_c.csv`",
            "- `selected_log_comparison_vs_b_c.csv`",
            "- `one_rad_in_place_command.csv`",
            "- `lr_delta_grid.csv`",
            "- `lr_delta_pivot.md`",
            "- `prediction_sample.csv`",
            "- `commands_run.txt`",
        ]
    )
    (OUT / "pade_static_yield_plus_c_report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    constants = read_constants()
    rows = load_rows(constants)
    model, predictions, candidate_rows = fit_models(rows, constants)
    split_metrics, phase_metrics, selected_metrics, risk_metrics = metric_tables(rows, predictions)
    split_comparison = compare_split_metrics(split_metrics)
    selected_comparison = compare_selected_metrics(selected_metrics)
    one_rad = one_rad_summary(constants, model)
    grid = grid_rows(constants, model)
    coeff_rows = coefficient_rows(model)

    write_csv(OUT / "pade_static_yield_plus_c_parameters.csv", parameter_rows(model))
    write_csv(OUT / "pade_static_yield_plus_c_surface_coefficients.csv", coeff_rows)
    write_csv(OUT / "candidate_scores.csv", candidate_rows)
    write_csv(OUT / "split_metrics.csv", split_metrics)
    write_csv(OUT / "phase_metrics.csv", phase_metrics)
    write_csv(OUT / "selected_log_metrics.csv", selected_metrics)
    write_csv(OUT / "risk_metrics.csv", risk_metrics)
    write_csv(OUT / "split_comparison_vs_b_c.csv", split_comparison)
    write_csv(OUT / "selected_log_comparison_vs_b_c.csv", selected_comparison)
    write_csv(OUT / "one_rad_in_place_command.csv", one_rad)
    write_csv(OUT / "lr_delta_grid.csv", grid)
    (OUT / "lr_delta_pivot.md").write_text("\n".join(pivot_lines(grid)) + "\n", encoding="utf-8")
    write_csv(OUT / "prediction_sample.csv", make_prediction_sample(rows, predictions, model))
    make_report(
        rows,
        model,
        split_metrics,
        selected_metrics,
        risk_metrics,
        split_comparison,
        selected_comparison,
        one_rad,
        coeff_rows,
        candidate_rows,
    )
    assumptions = {
        "hard_gate_abs_command_min": HARD_GATE_ABS_COMMAND_MIN,
        "hard_gate_abs_command_max": HARD_GATE_ABS_COMMAND_MAX,
        "hard_gate_abs_command_reference": HARD_GATE_ABS_COMMAND_TARGET,
        "hard_gate_condition": "Vf=0, Vr=0, yaw=+1 rad/s",
        "vf_grid_mps": vf_grid(),
        "yaw_grid_radps": yaw_grid(),
        "positive_yaw": "clockwise",
        "model_input_rule": "continuous contact primitives only; no command/request selectors, no maneuver labels in prediction",
        "selected_runtime_math": "sqrt, abs, clamp/min/max, rational divides; no trig/tanh/exp",
    }
    (OUT / "assumptions.json").write_text(json.dumps(assumptions, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (OUT / "commands_run.txt").write_text(
        "python codex_analysis\\yaw_model_variant_fits\\transition_options\\pade_static_yield_plus_c\\fit_pade_static_yield_plus_c.py\n",
        encoding="utf-8",
    )
    print((OUT / "pade_static_yield_plus_c_report.md").as_posix())


if __name__ == "__main__":
    main()
