#!/usr/bin/env python3
"""Variant C combined-slip contact surface fit.

Analysis-only tooling. Reads the shared contact-continuum feature sample and
writes all outputs beside this script. Production code, build metadata, and
tests are intentionally untouched.
"""

from __future__ import annotations

import csv
import math
from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent
PRIMARY = ROOT / "codex_analysis" / "contact_continuum_yaw_identification" / "ablation" / "phase_classified_feature_sample.csv"
SECONDARY = ROOT / "codex_analysis" / "contact_continuum_yaw_identification" / "features" / "contact_continuum_feature_sample.csv"
CONSTANTS = ROOT / "codex_analysis" / "contact_continuum_yaw_identification" / "features" / "plant_mirror_constants.csv"

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

PARENT_EXTRA_SANITY_LOGS = [
    "2026-04-16_02-43-36",
]

TARGET = "residual_opposes_yaw_nm"
RAW_RESIDUAL = "residual_additive_yaw_torque_nm"

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


def finite(values: list[float]) -> list[float]:
    return [v for v in values if math.isfinite(v)]


def rmse(values: list[float]) -> float:
    clean = finite(values)
    if not clean:
        return 0.0
    return math.sqrt(sum(v * v for v in clean) / len(clean))


def mae(values: list[float]) -> float:
    clean = finite(values)
    if not clean:
        return 0.0
    return sum(abs(v) for v in clean) / len(clean)


def weighted_mean(values: list[float], weights: list[float]) -> float:
    total_w = sum(weights)
    if total_w <= 0.0:
        return 0.0
    return sum(v * w for v, w in zip(values, weights)) / total_w


def weighted_rmse(values: list[float], weights: list[float]) -> float:
    return math.sqrt(max(weighted_mean([v * v for v in values], weights), 0.0))


def weighted_mae(values: list[float], weights: list[float]) -> float:
    return weighted_mean([abs(v) for v in values], weights)


def run_balanced_weights(rows: list[dict[str, object]]) -> list[float]:
    counts = Counter(str(row["run_id"]) for row in rows)
    return [1.0 / max(counts[str(row["run_id"])], 1) for row in rows]


def fit_weights(rows: list[dict[str, object]]) -> list[float]:
    counts = Counter(str(row["run_id"]) for row in rows)
    weights: list[float] = []
    for row in rows:
        w = 1.0 / max(counts[str(row["run_id"])], 1)
        if f(row, "gyro_derivative_spike") > 0.0:
            w *= 0.10
        if f(row, "hardware_saturation_evidence") > 0.0:
            w *= 0.35
        if abs(f(row, "yaw_rate_radps")) < 0.02 and f(row, "vbar_yaw_mps") < 0.002:
            w *= 0.25
        weights.append(w)
    return weights


def load_constants() -> dict[str, float]:
    with CONSTANTS.open(newline="", encoding="utf-8") as fh:
        return {row["name"]: float(row["value"]) for row in csv.DictReader(fh)}


def load_secondary_contact_fields() -> dict[tuple[str, str], dict[str, str]]:
    fields = set(CONTACT_FIELDS)
    out: dict[tuple[str, str], dict[str, str]] = {}
    with SECONDARY.open(newline="", encoding="utf-8") as fh:
        reader = csv.DictReader(fh)
        for row in reader:
            key = (row["run_id"], row["row_index"])
            out[key] = {name: row.get(name, "") for name in fields}
    return out


def row_key(row: dict[str, object]) -> tuple[str, str]:
    return str(row["run_id"]), str(row["row_index"])


def add_contact_bases(row: dict[str, object], contacts: dict[str, tuple[float, float]]) -> None:
    yaw_sign = f(row, "sign_yaw")
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

    row["gain_front_right_basis"] = right_front
    row["gain_rear_right_basis"] = right_rear
    row["gain_left_long_basis"] = long_left
    row["gain_right_long_basis"] = long_right
    row["gain_right_total_basis"] = right_front + right_rear
    row["gain_long_total_basis"] = long_left + long_right
    row["req_moment_opposes_yaw_nm"] = -yaw_sign * req_moment
    row["force_moment_opposes_yaw_nm"] = -yaw_sign * force_moment
    row["force_gap_opposes_yaw_nm"] = -yaw_sign * (req_moment - force_moment)
    row["req_abs_contact_moment_nm"] = req_abs_moment
    row["force_abs_contact_moment_nm"] = force_abs_moment
    row["load_weighted_rel_mps"] = load_weighted_rel
    row["load_weighted_lat_mps"] = load_weighted_lat


def load_rows() -> list[dict[str, object]]:
    constants = load_constants()
    half_track = 0.5 * constants.get("track_width_m", 0.084635)
    front_f = constants.get("drive_wheel_longitudinal_offset_m", 0.01475)
    contacts = {
        "fl": (-half_track, front_f),
        "fr": (half_track, front_f),
        "rl": (-half_track, -front_f),
        "rr": (half_track, -front_f),
    }
    secondary = load_secondary_contact_fields()
    rows: list[dict[str, object]] = []
    with PRIMARY.open(newline="", encoding="utf-8") as fh:
        reader = csv.DictReader(fh)
        for src in reader:
            row: dict[str, object] = dict(src)
            extra = secondary.get(row_key(row), {})
            row.update(extra)
            for key in list(row):
                if key in {
                    "run_id",
                    "family",
                    "schema",
                    "recommendation",
                    "dataset_split",
                    "physics_phase",
                    "physics_phase_basis",
                }:
                    continue
                try:
                    row[key] = float(row[key]) if row[key] != "" else 0.0
                except (TypeError, ValueError):
                    pass
            row["sign_yaw"] = sign(f(row, "yaw_rate_radps"))
            row["abs_forward_velocity_mps"] = abs(f(row, "forward_velocity_mps"))
            row["abs_yaw_rate_radps"] = abs(f(row, "yaw_rate_radps"))
            util = min(max(f(row, "max_force_preprojection_utilization"), 0.0), 5.0)
            limiter = min(max(f(row, "max_force_limiter_activity"), 0.0), 5.0)
            row["util_smooth"] = util / (1.0 + util)
            row["limiter_smooth"] = limiter / (1.0 + limiter)
            add_contact_bases(row, contacts)
            if str(row.get("dataset_split", "")) != "excluded_or_unclassified":
                rows.append(row)
    return rows


def selected_nominal_load(rows: list[dict[str, object]]) -> float:
    train = [
        f(row, "total_normal_load_n")
        for row in rows
        if row.get("dataset_split") == "primary_open_floor_fit_authoritative"
        and f(row, "total_normal_load_n") > 0.0
    ]
    med = median(train)
    return med if med > 0.0 else 1.0


def feature_names(candidate: str) -> list[str]:
    if candidate == "compact_gain_surface":
        groups = ["gain_right_total_basis", "gain_long_total_basis"]
        schedules = ["base", "low_rel", "high_forward", "util", "load_delta"]
        return [f"{group}__{sched}" for group in groups for sched in schedules]

    groups = [
        "gain_front_right_basis",
        "gain_rear_right_basis",
        "gain_left_long_basis",
        "gain_right_long_basis",
    ]
    schedules = ["base", "low_rel", "high_forward", "util", "load_delta"]
    names = [f"{group}__{sched}" for group in groups for sched in schedules]
    if candidate == "saturation_aware_surface":
        names.extend(
            [
                "force_gap_opposes_yaw_nm__base",
                "force_gap_opposes_yaw_nm__low_rel",
                "force_gap_opposes_yaw_nm__util",
                "force_gap_opposes_yaw_nm__limiter",
                "force_moment_opposes_yaw_nm__high_forward",
                "req_moment_opposes_yaw_nm__util",
                "req_abs_contact_moment_nm__limiter_signed",
                "force_abs_contact_moment_nm__limiter_signed",
            ]
        )
    return names


def schedules(row: dict[str, object], vrel_knee: float, fwd_knee: float, nominal_load: float) -> dict[str, float]:
    vrel = max(f(row, "vbar_rel_mps"), f(row, "load_weighted_rel_mps"), 0.0)
    vf = f(row, "abs_forward_velocity_mps")
    low_rel = 1.0 / (1.0 + (vrel / max(vrel_knee, 1.0e-6)) ** 2)
    low_forward = 1.0 / (1.0 + (vf / max(fwd_knee, 1.0e-6)) ** 2)
    load_delta = f(row, "total_normal_load_n") / max(nominal_load, 1.0e-9) - 1.0
    return {
        "base": 1.0,
        "low_rel": low_rel,
        "high_rel": 1.0 - low_rel,
        "low_forward": low_forward,
        "high_forward": 1.0 - low_forward,
        "util": f(row, "util_smooth"),
        "limiter": f(row, "limiter_smooth"),
        "load_delta": load_delta,
    }


def feature_value(row: dict[str, object], name: str, vrel_knee: float, fwd_knee: float, nominal_load: float) -> float:
    sched = schedules(row, vrel_knee, fwd_knee, nominal_load)
    base, suffix = name.split("__", 1)
    if suffix == "limiter_signed":
        return f(row, base) * sched["limiter"] * f(row, "sign_yaw")
    return f(row, base) * sched.get(suffix, 1.0)


def feature_vector(
    row: dict[str, object],
    names: list[str],
    vrel_knee: float,
    fwd_knee: float,
    nominal_load: float,
) -> list[float]:
    sched = schedules(row, vrel_knee, fwd_knee, nominal_load)
    sign_yaw = f(row, "sign_yaw")
    values: list[float] = []
    for name in names:
        base, suffix = name.split("__", 1)
        if suffix == "limiter_signed":
            values.append(f(row, base) * sched["limiter"] * sign_yaw)
        else:
            values.append(f(row, base) * sched.get(suffix, 1.0))
    return values


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


@dataclass
class Model:
    candidate: str
    vrel_knee: float
    fwd_knee: float
    ridge: float
    nominal_load: float
    names: list[str]
    scales: list[float]
    beta: list[float]
    clip_sigma: float = 8.0

    def predict_opposes(self, row: dict[str, object]) -> float:
        if f(row, "sign_yaw") == 0.0:
            return 0.0
        total = 0.0
        values = feature_vector(row, self.names, self.vrel_knee, self.fwd_knee, self.nominal_load)
        for value, scale, beta in zip(values, self.scales, self.beta):
            limit = self.clip_sigma * scale
            if value > limit:
                value = limit
            elif value < -limit:
                value = -limit
            total += beta * (value / scale)
        return total


def fit_model(
    rows: list[dict[str, object]],
    candidate: str,
    vrel_knee: float,
    fwd_knee: float,
    ridge: float,
    nominal_load: float,
) -> Model:
    names = feature_names(candidate)
    scales: list[float] = []
    train_vectors = [feature_vector(row, names, vrel_knee, fwd_knee, nominal_load) for row in rows]
    for idx, name in enumerate(names):
        vals = [abs(vec[idx]) for vec in train_vectors]
        scale = q(vals, 0.80)
        if scale < 1.0e-12:
            scale = math.sqrt(sum(v * v for v in vals) / len(vals)) if vals else 1.0
        scales.append(scale if scale > 1.0e-12 else 1.0)

    p = len(names)
    xtx = [[0.0 for _ in range(p)] for _ in range(p)]
    xty = [0.0 for _ in range(p)]
    weights = fit_weights(rows)
    for row, weight, raw_x in zip(rows, weights, train_vectors):
        if f(row, "sign_yaw") == 0.0:
            continue
        y = f(row, TARGET)
        x: list[float] = []
        for value, scale in zip(raw_x, scales):
            limit = 8.0 * scale
            if value > limit:
                value = limit
            elif value < -limit:
                value = -limit
            x.append(value / scale)
        for i in range(p):
            xty[i] += weight * x[i] * y
            xi = x[i]
            for j in range(p):
                xtx[i][j] += weight * xi * x[j]
    for i in range(p):
        xtx[i][i] += ridge
    beta = solve_linear(xtx, xty)
    return Model(candidate, vrel_knee, fwd_knee, ridge, nominal_load, names, scales, beta)


def residuals_after(rows: list[dict[str, object]], model: Model | None) -> tuple[list[float], list[float], list[float]]:
    baseline: list[float] = []
    corrected: list[float] = []
    predicted_raw: list[float] = []
    for row in rows:
        raw = f(row, RAW_RESIDUAL)
        pred_opposes = model.predict_opposes(row) if model else 0.0
        pred_raw = -f(row, "sign_yaw") * pred_opposes
        baseline.append(raw)
        predicted_raw.append(pred_raw)
        corrected.append(raw - pred_raw)
    return baseline, corrected, predicted_raw


def metric_row(label: str, rows: list[dict[str, object]], model: Model | None) -> dict[str, object]:
    baseline, corrected, predicted_raw = residuals_after(rows, model)
    weights = run_balanced_weights(rows)
    return {
        "group": label,
        "count": len(rows),
        "run_count": len({row["run_id"] for row in rows}),
        "baseline_rmse_nm": rmse(baseline),
        "corrected_rmse_nm": rmse(corrected),
        "baseline_mae_nm": mae(baseline),
        "corrected_mae_nm": mae(corrected),
        "baseline_median_abs_nm": median([abs(v) for v in baseline]),
        "corrected_median_abs_nm": median([abs(v) for v in corrected]),
        "baseline_signed_median_nm": median(baseline),
        "corrected_signed_median_nm": median(corrected),
        "run_balanced_baseline_rmse_nm": weighted_rmse(baseline, weights),
        "run_balanced_corrected_rmse_nm": weighted_rmse(corrected, weights),
        "run_balanced_baseline_mae_nm": weighted_mae(baseline, weights),
        "run_balanced_corrected_mae_nm": weighted_mae(corrected, weights),
        "prediction_rmse_nm": rmse(predicted_raw),
        "improvement_fraction_rmse": 1.0 - (rmse(corrected) / max(rmse(baseline), 1.0e-12)),
        "run_balanced_improvement_fraction_rmse": 1.0
        - (weighted_rmse(corrected, weights) / max(weighted_rmse(baseline, weights), 1.0e-12)),
    }


def split_rows(rows: list[dict[str, object]], split: str) -> list[dict[str, object]]:
    return [row for row in rows if row.get("dataset_split") == split]


def phase_rows(rows: list[dict[str, object]], split: str, phase: str) -> list[dict[str, object]]:
    return [row for row in rows if row.get("dataset_split") == split and row.get("physics_phase") == phase]


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


def objective_score(model: Model, validation_rows: list[dict[str, object]], all_rows: list[dict[str, object]]) -> tuple[float, dict[str, object]]:
    val = metric_row("validation_objective", validation_rows, model)
    straight = metric_row("straightish_abs_yaw_lt_0p05", risk_groups(all_rows)["straightish_abs_yaw_lt_0p05"], model)
    high = metric_row("high_forward_vf_ge_0p5", risk_groups(all_rows)["high_forward_vf_ge_0p5"], model)
    score = float(val["run_balanced_corrected_rmse_nm"])
    if float(straight["run_balanced_corrected_rmse_nm"]) > 1.03 * max(float(straight["run_balanced_baseline_rmse_nm"]), 1.0e-12):
        score += 0.25 * (
            float(straight["run_balanced_corrected_rmse_nm"])
            - float(straight["run_balanced_baseline_rmse_nm"])
        )
    if high["count"] and float(high["run_balanced_corrected_rmse_nm"]) > 1.05 * max(float(high["run_balanced_baseline_rmse_nm"]), 1.0e-12):
        score += 0.25 * (
            float(high["run_balanced_corrected_rmse_nm"])
            - float(high["run_balanced_baseline_rmse_nm"])
        )
    return score, {
        "objective_score": score,
        "validation_rb_corrected_rmse_nm": val["run_balanced_corrected_rmse_nm"],
        "validation_rb_baseline_rmse_nm": val["run_balanced_baseline_rmse_nm"],
        "straight_rb_corrected_rmse_nm": straight["run_balanced_corrected_rmse_nm"],
        "straight_rb_baseline_rmse_nm": straight["run_balanced_baseline_rmse_nm"],
        "high_forward_rb_corrected_rmse_nm": high["run_balanced_corrected_rmse_nm"],
        "high_forward_rb_baseline_rmse_nm": high["run_balanced_baseline_rmse_nm"],
    }


def write_csv(path: Path, rows: list[dict[str, object]], fieldnames: list[str] | None = None) -> None:
    if not fieldnames:
        fieldnames = list(rows[0].keys()) if rows else ["empty"]
    with path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def fmt(x: object, digits: int = 6) -> str:
    try:
        return f"{float(x):.{digits}f}"
    except (TypeError, ValueError):
        return str(x)


def top_coefficients(model: Model, n: int = 10) -> list[dict[str, object]]:
    rows = []
    for name, scale, beta in zip(model.names, model.scales, model.beta):
        rows.append(
            {
                "feature": name,
                "standardized_coefficient_nm": beta,
                "feature_scale": scale,
                "raw_coefficient_nm_per_feature": beta / scale,
                "abs_standardized_coefficient_nm": abs(beta),
            }
        )
    rows.sort(key=lambda r: float(r["abs_standardized_coefficient_nm"]), reverse=True)
    return rows[:n]


def make_report(
    best: Model,
    tuning_rows: list[dict[str, object]],
    split_metrics: list[dict[str, object]],
    selected_metrics: list[dict[str, object]],
    extra_metrics: list[dict[str, object]],
    risk_metrics: list[dict[str, object]],
    coefficient_rows: list[dict[str, object]],
    rows: list[dict[str, object]],
) -> None:
    train_count = len(split_rows(rows, "primary_open_floor_fit_authoritative"))
    validation_count = sum(
        1
        for row in rows
        if row.get("dataset_split")
        in {"open_floor_fit_downweighted", "open_floor_validation_only", "diag_validation_only", "aux_downweighted_validation"}
    )
    selected_present = [r for r in selected_metrics if int(r.get("present_in_primary_feature_input", 0)) == 1]
    selected_absent = [r["run_id"] for r in selected_metrics if int(r.get("present_in_primary_feature_input", 0)) == 0]
    best_split = {r["group"]: r for r in split_metrics}
    validation_summary = best_split.get("validation_non_authoritative", {})
    train_summary = best_split.get("primary_open_floor_fit_authoritative", {})

    lines: list[str] = []
    lines.append("# Variant C Combined-Slip Surface Fit")
    lines.append("")
    lines.append("Analysis-only output. Production code, build metadata, and tests were not edited.")
    lines.append("")
    lines.append("## Reproduce")
    lines.append("")
    lines.append("```powershell")
    lines.append("python codex_analysis\\yaw_model_variant_fits\\combined_slip_surface\\fit_combined_slip_surface.py")
    lines.append("```")
    lines.append("")
    lines.append("## Input Contract Alignment")
    lines.append("")
    lines.append(f"- Primary input: `{PRIMARY.relative_to(ROOT)}`")
    lines.append(f"- Secondary contact/load merge: `{SECONDARY.relative_to(ROOT)}`")
    lines.append(f"- Fit-authoritative training rows: {train_count}")
    lines.append(f"- Non-authoritative validation rows: {validation_count}")
    if selected_absent:
        lines.append(f"- Selected logs absent from primary feature input: {', '.join(selected_absent)}")
    else:
        lines.append("- All contract-selected logs were present in the primary feature input.")
    lines.append("")
    lines.append("## Model Form")
    lines.append("")
    lines.append(
        "The selected form predicts yaw-aligned residual torque `residual_opposes_yaw_nm`, then converts it "
        "back to a raw additive yaw-moment correction with `predicted_raw = -sign(yaw_rate) * predicted_opposes`. "
        "The corrected residual is `residual_additive_yaw_torque_nm - predicted_raw`."
    )
    lines.append("")
    lines.append(
        "Per-contact bases are derived from contact-relative velocities and wheel coordinates: right-force gain "
        "terms use `-sign(yaw) * f_i * v_rel_r_i`; longitudinal gain terms use "
        "`sign(yaw) * r_i * v_rel_f_i`. The saturation-aware candidate also uses the yaw-opposing difference "
        "between requested and projected contact moment. Schedules are continuous scalar multipliers: "
        "`low_rel = 1/(1+(vbar_rel/k_rel)^2)`, `high_forward = 1 - 1/(1+(|Vf|/k_fwd)^2)`, smooth force "
        "utilization, smooth limiter activity, and total-load delta."
    )
    lines.append("")
    lines.append(
        f"Selected model: `{best.candidate}` with `k_rel={best.vrel_knee:.3f} m/s`, "
        f"`k_fwd={best.fwd_knee:.3f} m/s`, ridge `{best.ridge:g}`, nominal load `{best.nominal_load:.6f} N`."
    )
    lines.append("")
    lines.append("## Tuning Summary")
    lines.append("")
    lines.append(
        "This is a narrow tune: the pass compares the three Variant C candidate surfaces at the common "
        "`k_rel=0.060 m/s`, `k_fwd=0.700 m/s` schedule point with ridge `0.001`. A wider pure-Python grid "
        "was too slow for the shared dataset in this workstream."
    )
    lines.append("")
    lines.append("| Rank | Candidate | k_rel | k_fwd | Ridge | Objective | Validation corrected RMSE | Straight corrected RMSE |")
    lines.append("| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: |")
    for idx, row in enumerate(tuning_rows[:8], 1):
        lines.append(
            f"| {idx} | {row['candidate']} | {fmt(row['vrel_knee_mps'], 3)} | {fmt(row['fwd_knee_mps'], 3)} | "
            f"{row['ridge']} | {fmt(row['objective_score'])} | "
            f"{fmt(row['validation_rb_corrected_rmse_nm'])} | {fmt(row['straight_rb_corrected_rmse_nm'])} |"
        )
    lines.append("")
    lines.append("## Split Metrics")
    lines.append("")
    lines.append("| Split | Count | Baseline RMSE | Corrected RMSE | Baseline MAE | Corrected MAE | Median abs before | Median abs after | RB RMSE change |")
    lines.append("| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |")
    for row in split_metrics:
        lines.append(
            f"| {row['group']} | {row['count']} | {fmt(row['baseline_rmse_nm'])} | {fmt(row['corrected_rmse_nm'])} | "
            f"{fmt(row['baseline_mae_nm'])} | {fmt(row['corrected_mae_nm'])} | "
            f"{fmt(row['baseline_median_abs_nm'])} | {fmt(row['corrected_median_abs_nm'])} | "
            f"{float(row['run_balanced_improvement_fraction_rmse']):.1%} |"
        )
    lines.append("")
    lines.append("## Selected Log Metrics")
    lines.append("")
    lines.append("| Run | Split | Count | Baseline RMSE | Corrected RMSE | Median signed before | Median signed after | Median abs before | Median abs after |")
    lines.append("| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |")
    for row in selected_metrics:
        if int(row.get("present_in_primary_feature_input", 0)) == 0:
            lines.append(f"| {row['run_id']} | absent | 0 |  |  |  |  |  |  |")
            continue
        lines.append(
            f"| {row['run_id']} | {row['dataset_split']} | {row['count']} | {fmt(row['baseline_rmse_nm'])} | "
            f"{fmt(row['corrected_rmse_nm'])} | {fmt(row['baseline_signed_median_nm'])} | "
            f"{fmt(row['corrected_signed_median_nm'])} | {fmt(row['baseline_median_abs_nm'])} | "
            f"{fmt(row['corrected_median_abs_nm'])} |"
        )
    lines.append("")
    if extra_metrics:
        lines.append("## Parent-Context Extra Sanity Logs")
        lines.append("")
        lines.append("| Run | Split | Count | Baseline RMSE | Corrected RMSE | Median abs before | Median abs after |")
        lines.append("| --- | --- | ---: | ---: | ---: | ---: | ---: |")
        for row in extra_metrics:
            lines.append(
                f"| {row['run_id']} | {row['dataset_split']} | {row['count']} | {fmt(row['baseline_rmse_nm'])} | "
                f"{fmt(row['corrected_rmse_nm'])} | {fmt(row['baseline_median_abs_nm'])} | "
                f"{fmt(row['corrected_median_abs_nm'])} |"
            )
        lines.append("")
    lines.append("## Straight-Line And High-Speed Risk")
    lines.append("")
    lines.append("| Group | Count | Baseline RMSE | Corrected RMSE | Median abs before | Median abs after | RB RMSE change |")
    lines.append("| --- | ---: | ---: | ---: | ---: | ---: | ---: |")
    for row in risk_metrics:
        lines.append(
            f"| {row['group']} | {row['count']} | {fmt(row['baseline_rmse_nm'])} | {fmt(row['corrected_rmse_nm'])} | "
            f"{fmt(row['baseline_median_abs_nm'])} | {fmt(row['corrected_median_abs_nm'])} | "
            f"{float(row['run_balanced_improvement_fraction_rmse']):.1%} |"
        )
    lines.append("")
    lines.append("## Dominant Coefficients")
    lines.append("")
    lines.append("| Feature | Std coeff Nm | Feature scale | Raw coeff |")
    lines.append("| --- | ---: | ---: | ---: |")
    for row in coefficient_rows[:12]:
        lines.append(
            f"| {row['feature']} | {fmt(row['standardized_coefficient_nm'])} | "
            f"{fmt(row['feature_scale'])} | {fmt(row['raw_coefficient_nm_per_feature'])} |"
        )
    lines.append("")
    lines.append("## Assessment")
    lines.append("")
    train_change = float(train_summary.get("run_balanced_improvement_fraction_rmse", 0.0))
    val_change = float(validation_summary.get("run_balanced_improvement_fraction_rmse", 0.0))
    lines.append(
        f"Fit-authoritative run-balanced RMSE changed by {train_change:.1%}; non-authoritative validation changed by {val_change:.1%}. "
        "Because the form is continuous, contact-primitive based, and zeroes out when yaw sign is zero, it is a plausible "
        "production-shape candidate. The validation result should still be treated as analysis evidence, not a production tune."
    )
    lines.append("")
    lines.append(
        "Straight-line risk is bounded mainly by the odd-in-yaw correction convention and by the zero-intercept fit. "
        "Near-zero yaw rows can still receive a correction from noisy yaw sign, so production use would need an explicit "
        "continuity-preserving deadband or direct dependence on signed contact yaw velocity rather than a raw gyro sign gate."
    )
    lines.append("")
    lines.append("## Output Files")
    lines.append("")
    for name in [
        "fit_combined_slip_surface.py",
        "variant_c_combined_slip_surface_report.md",
        "candidate_tuning_scores.csv",
        "model_coefficients.csv",
        "split_metrics.csv",
        "phase_metrics.csv",
        "selected_log_metrics.csv",
        "risk_metrics.csv",
        "prediction_sample.csv",
        "extra_parent_log_metrics.csv",
        "commands_run.txt",
    ]:
        lines.append(f"- `{name}`")
    lines.append("")
    (OUT / "variant_c_combined_slip_surface_report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    rows = load_rows()
    nominal_load = selected_nominal_load(rows)
    train_rows = split_rows(rows, "primary_open_floor_fit_authoritative")
    validation_rows = [
        row
        for row in rows
        if row.get("dataset_split")
        in {"open_floor_fit_downweighted", "open_floor_validation_only", "diag_validation_only", "aux_downweighted_validation"}
    ]

    candidates = ["compact_gain_surface", "front_rear_gain_surface", "saturation_aware_surface"]
    vrel_knees = [0.06]
    fwd_knees = [0.70]
    ridges = [1.0e-3]

    tuning_rows: list[dict[str, object]] = []
    best_model: Model | None = None
    best_score = float("inf")
    for candidate in candidates:
        for vrel_knee in vrel_knees:
            for fwd_knee in fwd_knees:
                for ridge in ridges:
                    model = fit_model(train_rows, candidate, vrel_knee, fwd_knee, ridge, nominal_load)
                    score, objective = objective_score(model, validation_rows, rows)
                    train_metric = metric_row("train", train_rows, model)
                    row = {
                        "candidate": candidate,
                        "vrel_knee_mps": vrel_knee,
                        "fwd_knee_mps": fwd_knee,
                        "ridge": ridge,
                        "feature_count": len(model.names),
                        "train_rb_baseline_rmse_nm": train_metric["run_balanced_baseline_rmse_nm"],
                        "train_rb_corrected_rmse_nm": train_metric["run_balanced_corrected_rmse_nm"],
                        **objective,
                    }
                    tuning_rows.append(row)
                    if score < best_score:
                        best_score = score
                        best_model = model

    if best_model is None:
        raise RuntimeError("no model fit")

    tuning_rows.sort(key=lambda r: float(r["objective_score"]))
    split_metrics = []
    for split in [
        "primary_open_floor_fit_authoritative",
        "open_floor_fit_downweighted",
        "open_floor_validation_only",
        "diag_validation_only",
        "aux_downweighted_validation",
    ]:
        subset = split_rows(rows, split)
        if subset:
            split_metrics.append(metric_row(split, subset, best_model))
    validation_metric = metric_row("validation_non_authoritative", validation_rows, best_model)
    split_metrics.append(validation_metric)

    phase_metrics = []
    for split in [
        "primary_open_floor_fit_authoritative",
        "open_floor_fit_downweighted",
        "open_floor_validation_only",
        "diag_validation_only",
        "aux_downweighted_validation",
    ]:
        for phase in ["entry", "plateau", "exit"]:
            subset = phase_rows(rows, split, phase)
            if subset:
                row = metric_row(f"{split}:{phase}", subset, best_model)
                row["dataset_split"] = split
                row["physics_phase"] = phase
                phase_metrics.append(row)

    selected_metrics: list[dict[str, object]] = []
    for run_id in SELECTED_LOGS:
        subset = [row for row in rows if row.get("run_id") == run_id]
        if not subset:
            selected_metrics.append(
                {
                    "run_id": run_id,
                    "present_in_primary_feature_input": 0,
                    "dataset_split": "",
                    "count": 0,
                }
            )
            continue
        row = metric_row(run_id, subset, best_model)
        row["run_id"] = run_id
        row["present_in_primary_feature_input"] = 1
        row["dataset_split"] = subset[0].get("dataset_split", "")
        selected_metrics.append(row)

    extra_metrics: list[dict[str, object]] = []
    for run_id in PARENT_EXTRA_SANITY_LOGS:
        subset = [row for row in rows if row.get("run_id") == run_id]
        if subset:
            row = metric_row(run_id, subset, best_model)
            row["run_id"] = run_id
            row["dataset_split"] = subset[0].get("dataset_split", "")
            extra_metrics.append(row)

    risk_metrics = []
    for group, subset in risk_groups(rows).items():
        if subset:
            risk_metrics.append(metric_row(group, subset, best_model))

    coefficient_rows = []
    for name, scale, beta in zip(best_model.names, best_model.scales, best_model.beta):
        coefficient_rows.append(
            {
                "candidate": best_model.candidate,
                "vrel_knee_mps": best_model.vrel_knee,
                "fwd_knee_mps": best_model.fwd_knee,
                "ridge": best_model.ridge,
                "feature": name,
                "standardized_coefficient_nm": beta,
                "feature_scale": scale,
                "raw_coefficient_nm_per_feature": beta / scale,
                "abs_standardized_coefficient_nm": abs(beta),
            }
        )
    coefficient_rows.sort(key=lambda r: float(r["abs_standardized_coefficient_nm"]), reverse=True)

    prediction_sample = []
    sample_by_run: defaultdict[str, int] = defaultdict(int)
    for row in rows:
        run_id = str(row["run_id"])
        if run_id not in SELECTED_LOGS and run_id not in PARENT_EXTRA_SANITY_LOGS:
            continue
        if sample_by_run[run_id] >= 25:
            continue
        baseline, corrected, predicted = residuals_after([row], best_model)
        prediction_sample.append(
            {
                "run_id": run_id,
                "dataset_split": row.get("dataset_split", ""),
                "physics_phase": row.get("physics_phase", ""),
                "row_index": row.get("row_index", ""),
                "forward_velocity_mps": f(row, "forward_velocity_mps"),
                "yaw_rate_radps": f(row, "yaw_rate_radps"),
                "vbar_rel_mps": f(row, "vbar_rel_mps"),
                "max_force_preprojection_utilization": f(row, "max_force_preprojection_utilization"),
                "residual_additive_yaw_torque_nm": baseline[0],
                "predicted_additive_yaw_torque_nm": predicted[0],
                "corrected_residual_yaw_torque_nm": corrected[0],
            }
        )
        sample_by_run[run_id] += 1

    write_csv(OUT / "candidate_tuning_scores.csv", tuning_rows)
    write_csv(OUT / "model_coefficients.csv", coefficient_rows)
    write_csv(OUT / "split_metrics.csv", split_metrics)
    write_csv(OUT / "phase_metrics.csv", phase_metrics)
    write_csv(OUT / "selected_log_metrics.csv", selected_metrics)
    write_csv(OUT / "extra_parent_log_metrics.csv", extra_metrics if extra_metrics else [{"empty": 1}])
    write_csv(OUT / "risk_metrics.csv", risk_metrics)
    write_csv(OUT / "prediction_sample.csv", prediction_sample)
    make_report(best_model, tuning_rows, split_metrics, selected_metrics, extra_metrics, risk_metrics, coefficient_rows, rows)
    (OUT / "commands_run.txt").write_text(
        "python codex_analysis\\yaw_model_variant_fits\\combined_slip_surface\\fit_combined_slip_surface.py\n",
        encoding="utf-8",
    )


if __name__ == "__main__":
    main()
