#!/usr/bin/env python3
"""Phase split and ablation for contact-continuum yaw identification.

Analysis-only script. It reads feature extraction and data-quality outputs,
classifies samples from sensor/command/contact-continuum dynamics, and writes
all artifacts beside this script.
"""

from __future__ import annotations

import csv
import math
from collections import defaultdict
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
ANALYSIS = ROOT / "codex_analysis" / "contact_continuum_yaw_identification"
FEATURES = ANALYSIS / "features"
QUALITY = ANALYSIS / "data_quality"
OUT = ANALYSIS / "ablation"

FEATURE_SAMPLE = FEATURES / "contact_continuum_feature_sample.csv"
RECOMMENDATIONS = QUALITY / "data_quality_recommendations_by_run.csv"
CONSTANTS = FEATURES / "plant_mirror_constants.csv"

TARGET = "residual_opposes_yaw_nm"


def f(row: dict[str, str], key: str, default: float = 0.0) -> float:
    try:
        value = row.get(key, "")
        if value == "":
            return default
        x = float(value)
        return x if math.isfinite(x) else default
    except (TypeError, ValueError):
        return default


def q(values: list[float], p: float) -> float:
    clean = sorted(x for x in values if math.isfinite(x))
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


def trimmed_mean(values: list[float], trim: float = 0.1) -> float:
    clean = sorted(x for x in values if math.isfinite(x))
    if not clean:
        return 0.0
    cut = int(len(clean) * trim)
    if cut * 2 >= len(clean):
        return sum(clean) / len(clean)
    keep = clean[cut : len(clean) - cut]
    return sum(keep) / len(keep)


def rmse(values: list[float]) -> float:
    clean = [x for x in values if math.isfinite(x)]
    if not clean:
        return 0.0
    return math.sqrt(sum(x * x for x in clean) / len(clean))


def mad(values: list[float]) -> float:
    med = median(values)
    return median([abs(x - med) for x in values if math.isfinite(x)])


def sign(x: float, eps: float = 1.0e-9) -> float:
    if x > eps:
        return 1.0
    if x < -eps:
        return -1.0
    return 0.0


def flag_present(text: str) -> bool:
    value = (text or "").strip().lower()
    if value in ("", "0", "0.0", "none", "false", "no", "[]"):
        return False
    try:
        return float(value) != 0.0
    except ValueError:
        return True


def load_recommendations() -> dict[str, dict[str, str]]:
    with RECOMMENDATIONS.open(newline="", encoding="utf-8") as fh:
        return {row["run_id"]: row for row in csv.DictReader(fh)}


def load_constants() -> dict[str, float]:
    with CONSTANTS.open(newline="", encoding="utf-8") as fh:
        return {row["name"]: float(row["value"]) for row in csv.DictReader(fh)}


def dataset_split(row: dict[str, object]) -> str:
    family = str(row["family"])
    rec = str(row["recommendation"])
    if family == "open_floor" and rec == "fit_authoritative":
        return "primary_open_floor_fit_authoritative"
    if family == "open_floor" and rec == "fit_downweighted":
        return "open_floor_fit_downweighted"
    if family == "open_floor" and rec == "validation_only":
        return "open_floor_validation_only"
    if family == "competition_diag" and rec == "validation_only":
        return "diag_validation_only"
    if family == "competition_aux" and rec == "validation_only_downweighted":
        return "aux_downweighted_validation"
    return "excluded_or_unclassified"


def derive_row(row: dict[str, str], recs: dict[str, dict[str, str]], constants: dict[str, float]) -> dict[str, object]:
    run_id = row["run_id"]
    rec = recs.get(run_id, {})
    out: dict[str, object] = dict(row)
    out["recommendation"] = rec.get("recommendation", "missing_quality_recommendation")
    out["quality_reason"] = rec.get("reason", "")
    out["schema_kind"] = rec.get("schema_kind", "")

    yaw = f(row, "yaw_rate_radps")
    raw_residual = f(row, "residual_additive_yaw_torque_nm")
    yaw_sign = sign(yaw, 1.0e-5)
    out["sign_yaw"] = yaw_sign
    out["abs_yaw_rate_radps"] = abs(yaw)
    out["abs_forward_velocity_mps"] = abs(f(row, "forward_velocity_mps"))
    out["abs_measured_yaw_accel_radps2"] = abs(f(row, "measured_yaw_accel_radps2"))
    out["abs_residual_additive_yaw_torque_nm"] = abs(raw_residual)
    out[TARGET] = -raw_residual * yaw_sign
    out["gyro_derivative_spike"] = 1.0 if abs(f(row, "measured_yaw_accel_radps2")) >= 500.0 else 0.0
    out["limiter_active"] = 1.0 if f(row, "max_force_limiter_activity") > 1.0e-9 else 0.0
    out["hardware_saturation_evidence"] = 1.0 if flag_present(row.get("saturation_flags", "")) else 0.0

    left_cmd = f(row, "left_command")
    right_cmd = f(row, "right_command")
    out["cmd_mean_abs"] = 0.5 * (abs(left_cmd) + abs(right_cmd))
    out["cmd_yaw_abs"] = 0.5 * abs(right_cmd - left_cmd)

    half_track = 0.5 * constants.get("track_width_m", 0.084635)
    front_f = constants.get("drive_wheel_longitudinal_offset_m", 0.01475)
    locs = {
        "fl": (-half_track, front_f),
        "fr": (half_track, front_f),
        "rl": (-half_track, -front_f),
        "rr": (half_track, -front_f),
    }
    total_normal = max(f(row, "total_normal_load_n"), 1.0e-9)
    patch_velocity_basis = 0.0
    patch_abs_velocity_basis = 0.0
    patch_req_basis = 0.0
    patch_force_basis = 0.0
    for name, (r_pos, f_pos) in locs.items():
        normal = f(row, f"{name}_normal_n")
        vf_rel = f(row, f"{name}_v_rel_f_mps")
        vr_rel = f(row, f"{name}_v_rel_r_mps")
        req_f = f(row, f"{name}_req_f_n")
        req_r = f(row, f"{name}_req_r_n")
        force_f = f(row, f"{name}_force_f_n")
        force_r = f(row, f"{name}_force_r_n")
        local_vel = f_pos * vr_rel - r_pos * vf_rel
        patch_velocity_basis += normal * local_vel
        patch_abs_velocity_basis += normal * abs(local_vel)
        patch_req_basis += f_pos * req_r - r_pos * req_f
        patch_force_basis += f_pos * force_r - r_pos * force_f
    out["patch_yaw_velocity_basis_m2ps"] = yaw_sign * patch_velocity_basis / total_normal
    out["patch_yaw_abs_velocity_basis_m2ps"] = patch_abs_velocity_basis / total_normal
    out["patch_yaw_req_basis_nm"] = yaw_sign * patch_req_basis
    out["patch_yaw_force_basis_nm"] = yaw_sign * patch_force_basis

    fl_f = abs(f(row, "fl_v_rel_f_mps"))
    fr_f = abs(f(row, "fr_v_rel_f_mps"))
    rl_f = abs(f(row, "rl_v_rel_f_mps"))
    rr_f = abs(f(row, "rr_v_rel_f_mps"))
    fl_r = abs(f(row, "fl_v_rel_r_mps"))
    fr_r = abs(f(row, "fr_v_rel_r_mps"))
    rl_r = abs(f(row, "rl_v_rel_r_mps"))
    rr_r = abs(f(row, "rr_v_rel_r_mps"))
    out["avg_abs_vrel_f_mps"] = 0.25 * (fl_f + fr_f + rl_f + rr_f)
    out["avg_abs_vrel_r_mps"] = 0.25 * (fl_r + fr_r + rl_r + rr_r)
    out["front_rear_vrel_f_abs_delta_mps"] = 0.5 * (fl_f + fr_f) - 0.5 * (rl_f + rr_f)
    out["left_right_vrel_f_abs_delta_mps"] = 0.5 * (fl_f + rl_f) - 0.5 * (fr_f + rr_f)
    out["front_rear_vrel_r_abs_delta_mps"] = 0.5 * (fl_r + fr_r) - 0.5 * (rl_r + rr_r)
    out["left_right_vrel_r_abs_delta_mps"] = 0.5 * (fl_r + rl_r) - 0.5 * (fr_r + rr_r)

    front_normal = f(row, "fl_normal_n") + f(row, "fr_normal_n")
    rear_normal = f(row, "rl_normal_n") + f(row, "rr_normal_n")
    left_normal = f(row, "fl_normal_n") + f(row, "rl_normal_n")
    right_normal = f(row, "fr_normal_n") + f(row, "rr_normal_n")
    out["front_rear_normal_delta_n"] = front_normal - rear_normal
    out["left_right_normal_delta_n"] = left_normal - right_normal
    out["left_right_drive_force_delta_n"] = f(row, "left_drive_force_n") - f(row, "right_drive_force_n")

    out["dataset_split"] = dataset_split(out)
    return out


def add_slopes_and_phases(rows: list[dict[str, object]]) -> None:
    by_run: dict[str, list[dict[str, object]]] = defaultdict(list)
    for row in rows:
        by_run[str(row["run_id"])].append(row)

    active_rows: list[dict[str, object]] = []
    for run_id, group in by_run.items():
        group.sort(key=lambda r: (float(r.get("time_us", 0.0)), float(r.get("row_index", 0.0))))
        abs_yaws = [float(r["abs_yaw_rate_radps"]) for r in group]
        rels = [float(r.get("vbar_rel_mps", 0.0)) for r in group]
        cmd_yaws = [float(r["cmd_yaw_abs"]) for r in group]
        yaw_gate = max(0.10, 0.25 * q(abs_yaws, 0.80))
        rel_gate = max(0.010, 0.25 * q(rels, 0.80))
        cmd_gate = max(0.010, 0.25 * q(cmd_yaws, 0.80))

        for i, row in enumerate(group):
            prev_row = group[max(0, i - 1)]
            next_row = group[min(len(group) - 1, i + 1)]
            t0 = float(prev_row.get("time_us", 0.0)) / 1.0e6
            t1 = float(next_row.get("time_us", 0.0)) / 1.0e6
            dt = max(t1 - t0, 1.0e-6)
            row["yaw_speed_slope_radps2"] = (float(next_row["abs_yaw_rate_radps"]) - float(prev_row["abs_yaw_rate_radps"])) / dt
            row["vbar_rel_slope_mps2"] = (float(next_row.get("vbar_rel_mps", 0.0)) - float(prev_row.get("vbar_rel_mps", 0.0))) / dt
            row["cmd_yaw_slope_per_s"] = (float(next_row["cmd_yaw_abs"]) - float(prev_row["cmd_yaw_abs"])) / dt
            active = (
                float(row["abs_yaw_rate_radps"]) >= yaw_gate
                or float(row.get("vbar_rel_mps", 0.0)) >= rel_gate
                or float(row["cmd_yaw_abs"]) >= cmd_gate
                or float(row.get("max_force_preprojection_utilization", 0.0)) >= 0.35
            )
            row["physics_active"] = 1.0 if active else 0.0
            if active:
                active_rows.append(row)

        segment_id = 0
        in_segment = False
        last_t = None
        for row in group:
            t = float(row.get("time_us", 0.0)) / 1.0e6
            active = bool(row["physics_active"])
            if active and (not in_segment or last_t is None or t - last_t > 0.25):
                segment_id += 1
                in_segment = True
            elif not active:
                in_segment = False
            row["physics_segment_id"] = f"{run_id}:{segment_id}" if active else ""
            last_t = t if active else None

    yaw_scale = max(1.0, q([abs(float(r["yaw_speed_slope_radps2"])) for r in active_rows], 0.65))
    rel_scale = max(0.025, q([abs(float(r["vbar_rel_slope_mps2"])) for r in active_rows], 0.65))
    cmd_scale = max(0.025, q([abs(float(r["cmd_yaw_slope_per_s"])) for r in active_rows], 0.65))

    segments: dict[str, list[dict[str, object]]] = defaultdict(list)
    for row in rows:
        sid = str(row["physics_segment_id"])
        if sid:
            segments[sid].append(row)

    for sid, group in segments.items():
        times = [float(r.get("time_us", 0.0)) / 1.0e6 for r in group]
        start = min(times)
        end = max(times)
        duration = max(end - start, 1.0e-6)
        for row in group:
            progress = (float(row.get("time_us", 0.0)) / 1.0e6 - start) / duration
            row["phase_progress"] = max(0.0, min(1.0, progress))

    for row in rows:
        progress = float(row.get("phase_progress", 0.5))
        yaw_slope = float(row["yaw_speed_slope_radps2"])
        rel_slope = float(row["vbar_rel_slope_mps2"])
        cmd_slope = float(row["cmd_yaw_slope_per_s"])
        growth = max(0.0, yaw_slope) / yaw_scale + max(0.0, rel_slope) / rel_scale + 0.5 * max(0.0, cmd_slope) / cmd_scale
        decay = max(0.0, -yaw_slope) / yaw_scale + max(0.0, -rel_slope) / rel_scale + 0.5 * max(0.0, -cmd_slope) / cmd_scale
        row["phase_growth_score"] = growth
        row["phase_decay_score"] = decay
        if not row["physics_active"]:
            phase = "plateau"
            basis = "stable_low_activity"
        elif progress <= 0.20 and growth >= 0.50 * decay:
            phase = "entry"
            basis = "early_physics_segment"
        elif progress >= 0.80 and decay >= 0.50 * growth:
            phase = "exit"
            basis = "late_physics_segment"
        elif growth >= max(1.5, 1.25 * decay):
            phase = "entry"
            basis = "rising_yaw_contact_command"
        elif decay >= max(1.5, 1.25 * growth):
            phase = "exit"
            basis = "falling_yaw_contact_command"
        else:
            phase = "plateau"
            basis = "stable_contact_continuum"
        row["physics_phase"] = phase
        row["physics_phase_basis"] = basis
        row["phase_entry"] = 1.0 if phase == "entry" else 0.0
        row["phase_plateau"] = 1.0 if phase == "plateau" else 0.0
        row["phase_exit"] = 1.0 if phase == "exit" else 0.0


def dataset_rows(rows: list[dict[str, object]], split: str, phase: str = "all") -> list[dict[str, object]]:
    out = [r for r in rows if r["dataset_split"] == split]
    if phase != "all":
        out = [r for r in out if r["physics_phase"] == phase]
    return out


def run_balanced_weights(rows: list[dict[str, object]]) -> list[float]:
    counts: dict[str, int] = defaultdict(int)
    for row in rows:
        counts[str(row["run_id"])] += 1
    weights = []
    for row in rows:
        scale = 0.25 if row["dataset_split"] == "aux_downweighted_validation" else 1.0
        weights.append(scale / max(counts[str(row["run_id"])], 1))
    return weights


def weighted_mean(values: list[float], weights: list[float]) -> float:
    total = sum(weights)
    if total <= 0.0:
        return 0.0
    return sum(v * w for v, w in zip(values, weights)) / total


def weighted_r2(y: list[float], pred: list[float], weights: list[float]) -> float:
    if not y:
        return 0.0
    mean_y = weighted_mean(y, weights)
    ss_tot = sum(w * (v - mean_y) ** 2 for v, w in zip(y, weights))
    ss_res = sum(w * (v - p) ** 2 for v, p, w in zip(y, pred, weights))
    if ss_tot <= 1.0e-18:
        return 0.0
    return 1.0 - ss_res / ss_tot


def winsorized_corr(rows: list[dict[str, object]], x_key: str, y_key: str = TARGET) -> float:
    pairs = []
    for row in rows:
        x_val = float(row.get(x_key, 0.0))
        y_val = float(row.get(y_key, 0.0))
        if math.isfinite(x_val) and math.isfinite(y_val):
            pairs.append((x_val, y_val, str(row["run_id"])))
    if len(pairs) < 3:
        return 0.0
    xs = [p[0] for p in pairs]
    ys = [p[1] for p in pairs]
    xlo, xhi = q(xs, 0.025), q(xs, 0.975)
    ylo, yhi = q(ys, 0.025), q(ys, 0.975)
    clipped = [(min(max(x, xlo), xhi), min(max(y, ylo), yhi), run) for x, y, run in pairs]
    counts: dict[str, int] = defaultdict(int)
    for _, _, run in clipped:
        counts[run] += 1
    weights = [1.0 / counts[run] for _, _, run in clipped]
    xs = [x for x, _, _ in clipped]
    ys = [y for _, y, _ in clipped]
    mx = weighted_mean(xs, weights)
    my = weighted_mean(ys, weights)
    cov = sum(w * (x - mx) * (y - my) for x, y, w in zip(xs, ys, weights))
    vx = sum(w * (x - mx) ** 2 for x, w in zip(xs, weights))
    vy = sum(w * (y - my) ** 2 for y, w in zip(ys, weights))
    if vx <= 1.0e-18 or vy <= 1.0e-18:
        return 0.0
    return cov / math.sqrt(vx * vy)


def solve_linear(a: list[list[float]], b: list[float]) -> list[float]:
    n = len(b)
    aug = [row[:] + [rhs] for row, rhs in zip(a, b)]
    for col in range(n):
        pivot = max(range(col, n), key=lambda r: abs(aug[r][col]))
        if abs(aug[pivot][col]) < 1.0e-12:
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


def model_score(rows: list[dict[str, object]], features: list[str]) -> tuple[float, float, float]:
    clean = [r for r in rows if abs(float(r.get("sign_yaw", 0.0))) > 0.0]
    if len(clean) < max(20, len(features) + 3):
        return 0.0, 0.0, 0.0
    weights = run_balanced_weights(clean)
    y = [float(r[TARGET]) for r in clean]

    centers = []
    scales = []
    for key in features:
        vals = [float(r.get(key, 0.0)) for r in clean]
        center = median(vals)
        scale = q(vals, 0.75) - q(vals, 0.25)
        if abs(scale) < 1.0e-12:
            mean = sum(vals) / len(vals)
            var = sum((v - mean) ** 2 for v in vals) / len(vals)
            scale = math.sqrt(var)
        centers.append(center)
        scales.append(scale if abs(scale) > 1.0e-12 else 1.0)

    cols = len(features) + 1
    xtx = [[0.0 for _ in range(cols)] for _ in range(cols)]
    xty = [0.0 for _ in range(cols)]
    for row, yy, w in zip(clean, y, weights):
        x = [1.0]
        for key, center, scale in zip(features, centers, scales):
            value = float(row.get(key, 0.0))
            lo = center - 8.0 * scale
            hi = center + 8.0 * scale
            x.append((min(max(value, lo), hi) - center) / scale)
        for i in range(cols):
            xty[i] += w * x[i] * yy
            for j in range(cols):
                xtx[i][j] += w * x[i] * x[j]
    for i in range(1, cols):
        xtx[i][i] += 1.0e-8
    beta = solve_linear(xtx, xty)
    pred = []
    for row in clean:
        value = beta[0]
        for b, key, center, scale in zip(beta[1:], features, centers, scales):
            value += b * ((float(row.get(key, 0.0)) - center) / scale)
        pred.append(value)
    residuals = [yy - pp for yy, pp in zip(y, pred)]
    return weighted_r2(y, pred, weights), weighted_mean([abs(x) for x in residuals], weights), math.sqrt(weighted_mean([x * x for x in residuals], weights))


def group_dominance_score(rows: list[dict[str, object]], keys: list[str]) -> float:
    clean = [r for r in rows if abs(float(r.get("sign_yaw", 0.0))) > 0.0]
    if len(clean) < 3:
        return 0.0
    weights = run_balanced_weights(clean)
    y = [float(r[TARGET]) for r in clean]
    sums: dict[tuple[str, ...], list[float]] = defaultdict(lambda: [0.0, 0.0])
    for row, yy, w in zip(clean, y, weights):
        key = tuple(str(row.get(k, "")) for k in keys)
        sums[key][0] += w * yy
        sums[key][1] += w
    means = {key: total / weight for key, (total, weight) in sums.items() if weight > 0.0}
    pred = [means[tuple(str(row.get(k, "")) for k in keys)] for row in clean]
    return weighted_r2(y, pred, weights)


def summarize(rows: list[dict[str, object]]) -> dict[str, object]:
    residual = [float(r.get("residual_additive_yaw_torque_nm", 0.0)) for r in rows]
    aligned = [float(r.get(TARGET, 0.0)) for r in rows if abs(float(r.get("sign_yaw", 0.0))) > 0.0]
    return {
        "count": len(rows),
        "run_count": len({r["run_id"] for r in rows}),
        "median_residual_nm": median(residual),
        "trimmed_mean_residual_nm": trimmed_mean(residual),
        "rmse_residual_nm": rmse(residual),
        "median_abs_residual_nm": median([abs(x) for x in residual]),
        "mad_residual_nm": mad(residual),
        "median_residual_opposes_yaw_nm": median(aligned),
        "median_abs_residual_opposes_yaw_nm": median([abs(x) for x in aligned]),
        "mean_vbar_rel_mps": sum(float(r.get("vbar_rel_mps", 0.0)) for r in rows) / len(rows) if rows else 0.0,
        "mean_vbar_yaw_mps": sum(float(r.get("vbar_yaw_mps", 0.0)) for r in rows) / len(rows) if rows else 0.0,
        "limiter_active_fraction": sum(float(r.get("limiter_active", 0.0)) for r in rows) / len(rows) if rows else 0.0,
        "saturation_evidence_fraction": sum(float(r.get("hardware_saturation_evidence", 0.0)) for r in rows) / len(rows) if rows else 0.0,
        "gyro_spike_fraction": sum(float(r.get("gyro_derivative_spike", 0.0)) for r in rows) / len(rows) if rows else 0.0,
    }


def write_csv(path: Path, rows: list[dict[str, object]], fieldnames: list[str]) -> None:
    with path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def fmt(x: object, digits: int = 6) -> str:
    if isinstance(x, float):
        return f"{x:.{digits}f}"
    return str(x)


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    recs = load_recommendations()
    constants = load_constants()
    with FEATURE_SAMPLE.open(newline="", encoding="utf-8") as fh:
        rows = [derive_row(row, recs, constants) for row in csv.DictReader(fh)]
    add_slopes_and_phases(rows)

    rows = [r for r in rows if r["dataset_split"] != "excluded_or_unclassified"]

    selected_fields = [
        "run_id", "family", "schema", "recommendation", "dataset_split", "row_index", "time_us",
        "section_id", "phase_id", "primitive_id", "physics_phase", "physics_phase_basis",
        "physics_active", "phase_progress", "phase_growth_score", "phase_decay_score",
        "forward_velocity_mps", "yaw_rate_radps", "measured_yaw_accel_radps2",
        "vbar_rel_mps", "vbar_lat_mps", "vbar_yaw_mps", "max_force_preprojection_utilization",
        "max_force_limiter_activity", "limiter_active", "hardware_saturation_evidence",
        "gyro_derivative_spike", "residual_additive_yaw_torque_nm", TARGET,
        "patch_yaw_velocity_basis_m2ps", "patch_yaw_abs_velocity_basis_m2ps",
        "patch_yaw_req_basis_nm", "patch_yaw_force_basis_nm",
        "front_rear_vrel_f_abs_delta_mps", "left_right_vrel_f_abs_delta_mps",
        "front_rear_vrel_r_abs_delta_mps", "left_right_vrel_r_abs_delta_mps",
        "front_rear_normal_delta_n", "left_right_normal_delta_n",
        "left_right_drive_force_delta_n", "schema_kind", "quality_reason",
    ]
    write_csv(OUT / "phase_classified_feature_sample.csv", rows, selected_fields)

    splits = [
        "primary_open_floor_fit_authoritative",
        "open_floor_fit_downweighted",
        "open_floor_validation_only",
        "diag_validation_only",
        "aux_downweighted_validation",
    ]
    phases = ["all", "entry", "plateau", "exit"]

    summary_rows: list[dict[str, object]] = []
    count_rows: list[dict[str, object]] = []
    for split in splits:
        all_split = dataset_rows(rows, split)
        total = len(all_split)
        for phase in phases:
            subset = dataset_rows(rows, split, phase)
            if not subset:
                continue
            summary = summarize(subset)
            summary.update({"dataset_split": split, "physics_phase": phase})
            summary_rows.append(summary)
            count_rows.append({
                "dataset_split": split,
                "physics_phase": phase,
                "count": len(subset),
                "fraction_of_split": len(subset) / total if total else 0.0,
                "run_count": len({r["run_id"] for r in subset}),
                "stable_contact_fraction": sum(1 for r in subset if r["physics_phase_basis"] == "stable_contact_continuum") / len(subset),
                "early_late_segment_fraction": sum(1 for r in subset if str(r["physics_phase_basis"]).endswith("physics_segment")) / len(subset),
            })
    write_csv(OUT / "phase_split_counts.csv", count_rows, list(count_rows[0]))
    write_csv(OUT / "phase_split_summary.csv", summary_rows, list(summary_rows[0]))

    candidate_features = {
        "yaw_loss_aggregate": [
            "vbar_yaw_mps",
            "vbar_rel_mps",
            "max_force_limiter_activity",
            "max_force_preprojection_utilization",
            "current_proxy_abs_raw_over_unit_command_prior",
            "abs_yaw_rate_radps",
        ],
        "patch_force_contact_features": [
            "patch_yaw_velocity_basis_m2ps",
            "patch_yaw_abs_velocity_basis_m2ps",
            "patch_yaw_req_basis_nm",
            "patch_yaw_force_basis_nm",
            "avg_abs_vrel_f_mps",
            "avg_abs_vrel_r_mps",
            "front_rear_vrel_f_abs_delta_mps",
            "left_right_vrel_f_abs_delta_mps",
            "front_rear_vrel_r_abs_delta_mps",
            "left_right_vrel_r_abs_delta_mps",
            "front_rear_normal_delta_n",
            "left_right_normal_delta_n",
            "left_right_drive_force_delta_n",
        ],
        "artifact_transition_schema_run_proxy": [
            "phase_entry",
            "phase_exit",
            "phase_progress",
            "abs_measured_yaw_accel_radps2",
            "gyro_derivative_spike",
            "hardware_saturation_evidence",
        ],
    }
    model_rows: list[dict[str, object]] = []
    for split in splits:
        for phase in phases:
            subset = dataset_rows(rows, split, phase)
            if not subset:
                continue
            for candidate, features in candidate_features.items():
                r2, mae, rms = model_score(subset, features)
                model_rows.append({
                    "dataset_split": split,
                    "physics_phase": phase,
                    "candidate": candidate,
                    "count": len(subset),
                    "run_count": len({r["run_id"] for r in subset}),
                    "weighted_r2_in_sample": r2,
                    "weighted_mae_after_fit_nm": mae,
                    "weighted_rmse_after_fit_nm": rms,
                    "features": ";".join(features),
                })
    write_csv(OUT / "candidate_model_scores.csv", model_rows, list(model_rows[0]))

    corr_features = sorted({key for vals in candidate_features.values() for key in vals})
    corr_rows: list[dict[str, object]] = []
    for split in splits:
        for phase in phases:
            subset = dataset_rows(rows, split, phase)
            if len(subset) < 3:
                continue
            for key in corr_features:
                corr_rows.append({
                    "dataset_split": split,
                    "physics_phase": phase,
                    "feature": key,
                    "count": len(subset),
                    "winsorized_run_balanced_corr_to_residual_opposes_yaw": winsorized_corr(subset, key),
                })
    write_csv(OUT / "correlation_scores.csv", corr_rows, list(corr_rows[0]))

    dominance_rows: list[dict[str, object]] = []
    groupings = {
        "run_id": ["run_id"],
        "physics_phase": ["physics_phase"],
        "schema_kind": ["schema_kind"],
        "run_id_plus_phase": ["run_id", "physics_phase"],
        "schema_plus_phase": ["schema_kind", "physics_phase"],
    }
    for split in splits:
        for phase in phases:
            subset = dataset_rows(rows, split, phase)
            if not subset:
                continue
            for name, keys in groupings.items():
                dominance_rows.append({
                    "dataset_split": split,
                    "physics_phase": phase,
                    "grouping": name,
                    "count": len(subset),
                    "weighted_r2_group_mean": group_dominance_score(subset, keys),
                })
    write_csv(OUT / "artifact_group_dominance_scores.csv", dominance_rows, list(dominance_rows[0]))

    drop_rows: list[dict[str, object]] = []
    for split in splits:
        all_rows = dataset_rows(rows, split)
        plateau_rows = dataset_rows(rows, split, "plateau")
        transient_rows = [r for r in all_rows if r["physics_phase"] != "plateau"]
        if not all_rows or not plateau_rows or not transient_rows:
            continue
        all_summary = summarize(all_rows)
        plateau_summary = summarize(plateau_rows)
        transient_summary = summarize(transient_rows)
        drop_rows.append({
            "dataset_split": split,
            "all_median_abs_residual_nm": all_summary["median_abs_residual_nm"],
            "plateau_median_abs_residual_nm": plateau_summary["median_abs_residual_nm"],
            "transient_median_abs_residual_nm": transient_summary["median_abs_residual_nm"],
            "plateau_vs_all_abs_reduction_fraction": 1.0 - float(plateau_summary["median_abs_residual_nm"]) / max(float(all_summary["median_abs_residual_nm"]), 1.0e-12),
            "plateau_vs_transient_abs_reduction_fraction": 1.0 - float(plateau_summary["median_abs_residual_nm"]) / max(float(transient_summary["median_abs_residual_nm"]), 1.0e-12),
            "all_rmse_residual_nm": all_summary["rmse_residual_nm"],
            "plateau_rmse_residual_nm": plateau_summary["rmse_residual_nm"],
            "transient_rmse_residual_nm": transient_summary["rmse_residual_nm"],
            "plateau_count": len(plateau_rows),
            "transient_count": len(transient_rows),
        })
    write_csv(OUT / "plateau_residual_drop.csv", drop_rows, list(drop_rows[0]))

    primary_models = [r for r in model_rows if r["dataset_split"] == "primary_open_floor_fit_authoritative" and r["physics_phase"] == "plateau"]
    primary_models.sort(key=lambda r: float(r["weighted_r2_in_sample"]), reverse=True)
    primary_drop = next((r for r in drop_rows if r["dataset_split"] == "primary_open_floor_fit_authoritative"), {})
    primary_counts = [r for r in count_rows if r["dataset_split"] == "primary_open_floor_fit_authoritative"]
    primary_summary = [r for r in summary_rows if r["dataset_split"] == "primary_open_floor_fit_authoritative"]

    top_candidate = primary_models[0]["candidate"] if primary_models else "none"
    plateau_reduction = float(primary_drop.get("plateau_vs_transient_abs_reduction_fraction", 0.0) or 0.0)
    plateau_abs = float(primary_drop.get("plateau_median_abs_residual_nm", 0.0) or 0.0)
    transient_abs = float(primary_drop.get("transient_median_abs_residual_nm", 0.0) or 0.0)
    best_r2 = float(primary_models[0]["weighted_r2_in_sample"]) if primary_models else 0.0
    second_r2 = float(primary_models[1]["weighted_r2_in_sample"]) if len(primary_models) > 1 else 0.0
    if plateau_reduction >= 0.35 and top_candidate == "artifact_transition_schema_run_proxy":
        decision = "artifact_or_transition_dominated"
    elif top_candidate == "patch_force_contact_features" and best_r2 - second_r2 >= 0.20:
        decision = "patch_force_contact_feature_candidate_with_validation_gaps"
    elif top_candidate == "yaw_loss_aggregate" and plateau_abs > 0.5 * transient_abs:
        decision = "continuous_yaw_loss_candidate"
    else:
        decision = "insufficient_identification_for_production_design"

    report = []
    report.append("# Contact-Continuum Phase Split and Ablation")
    report.append("")
    report.append("Analysis-only output. Production code was not modified.")
    report.append("")
    report.append("## Reproduce")
    report.append("")
    report.append("```powershell")
    report.append("python codex_analysis\\contact_continuum_yaw_identification\\ablation\\analyze_phase_ablation.py")
    report.append("```")
    report.append("")
    report.append("## Phase Classifier")
    report.append("")
    report.append(
        "Samples are split by physics-active contact/yaw segments from `yaw_rate_radps`, "
        "`vbar_rel_mps`, yaw-command difference, force-utilization evidence, and local slopes. "
        "Existing section/phase/primitive labels are carried only as metadata in the classified CSV."
    )
    report.append("")
    report.append("## Primary Open-Floor Counts")
    report.append("")
    report.append("| Phase | Count | Fraction | Runs |")
    report.append("| --- | ---: | ---: | ---: |")
    for r in primary_counts:
        if r["physics_phase"] == "all":
            continue
        report.append(f"| {r['physics_phase']} | {r['count']} | {float(r['fraction_of_split']):.3f} | {r['run_count']} |")
    report.append("")
    report.append("## Primary Open-Floor Residual Summary")
    report.append("")
    report.append("| Phase | Median abs residual Nm | RMSE Nm | Median opposing-yaw residual Nm | Mean vbar_rel m/s | Limiter frac | Saturation frac | Gyro spike frac |")
    report.append("| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |")
    for r in primary_summary:
        report.append(
            f"| {r['physics_phase']} | {float(r['median_abs_residual_nm']):.6f} | "
            f"{float(r['rmse_residual_nm']):.6f} | {float(r['median_residual_opposes_yaw_nm']):.6f} | "
            f"{float(r['mean_vbar_rel_mps']):.6f} | {float(r['limiter_active_fraction']):.3f} | "
            f"{float(r['saturation_evidence_fraction']):.3f} | {float(r['gyro_spike_fraction']):.4f} |"
        )
    report.append("")
    report.append("## Plateau Drop")
    report.append("")
    report.append(
        f"Primary plateau median absolute residual is {plateau_abs:.6f} Nm versus "
        f"{transient_abs:.6f} Nm for entry+exit, a {plateau_reduction:.1%} reduction."
    )
    report.append("")
    report.append("## Candidate Fits")
    report.append("")
    report.append("| Candidate | Plateau weighted R2 | Plateau weighted MAE Nm | Plateau weighted RMSE Nm |")
    report.append("| --- | ---: | ---: | ---: |")
    for r in primary_models:
        report.append(
            f"| {r['candidate']} | {float(r['weighted_r2_in_sample']):.4f} | "
            f"{float(r['weighted_mae_after_fit_nm']):.6f} | {float(r['weighted_rmse_after_fit_nm']):.6f} |"
        )
    report.append("")
    report.append("## Plateau Validation Scores")
    report.append("")
    report.append("| Split | Yaw-loss R2 | Patch-force R2 | Artifact R2 |")
    report.append("| --- | ---: | ---: | ---: |")
    for split in [
        "open_floor_fit_downweighted",
        "open_floor_validation_only",
        "diag_validation_only",
        "aux_downweighted_validation",
    ]:
        split_rows = [r for r in model_rows if r["dataset_split"] == split and r["physics_phase"] == "plateau"]
        values = {r["candidate"]: float(r["weighted_r2_in_sample"]) for r in split_rows}
        report.append(
            f"| {split} | {values.get('yaw_loss_aggregate', 0.0):.4f} | "
            f"{values.get('patch_force_contact_features', 0.0):.4f} | "
            f"{values.get('artifact_transition_schema_run_proxy', 0.0):.4f} |"
        )
    report.append("")
    report.append("## Decision")
    report.append("")
    report.append(f"Decision: `{decision}`.")
    report.append("")
    report.append(
        "This is not a production table or production fit. The outputs identify which explanation "
        "is most consistent with the sampled logs and where the data are still confounded."
    )
    report.append("")
    report.append("## Files")
    report.append("")
    for name in [
        "phase_classified_feature_sample.csv",
        "phase_split_counts.csv",
        "phase_split_summary.csv",
        "candidate_model_scores.csv",
        "correlation_scores.csv",
        "artifact_group_dominance_scores.csv",
        "plateau_residual_drop.csv",
        "commands_run.txt",
    ]:
        report.append(f"- `{name}`")
    report.append("")
    (OUT / "phase_ablation_decision_report.md").write_text("\n".join(report) + "\n", encoding="utf-8")
    (OUT / "commands_run.txt").write_text(
        "python codex_analysis\\contact_continuum_yaw_identification\\ablation\\analyze_phase_ablation.py\n",
        encoding="utf-8",
    )


if __name__ == "__main__":
    main()
