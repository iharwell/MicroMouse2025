#!/usr/bin/env python3
from __future__ import annotations

import csv
import json
import math
from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "codex_tmp" / "agent_b_linear_anisotropic_slip_eval"
PRIMARY = ROOT / "codex_analysis" / "contact_continuum_yaw_identification" / "ablation" / "phase_classified_feature_sample.csv"
SECONDARY = ROOT / "codex_analysis" / "contact_continuum_yaw_identification" / "features" / "contact_continuum_feature_sample.csv"
CONSTANTS = ROOT / "codex_analysis" / "contact_continuum_yaw_identification" / "features" / "plant_mirror_constants.csv"
INVENTORY = ROOT / "codex_analysis" / "yaw_torque_expanded_validation" / "discovered_logs.csv"

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

PATCHES = ("fl", "fr", "rl", "rr")


@dataclass(frozen=True)
class Sample:
    run_id: str
    split: str
    row_index: int
    time_us: float
    vf: float
    yaw_rate: float
    af: float
    yaw_accel: float
    observed_moment: float
    baseline_moment: float
    left_drive_force: float
    right_drive_force: float
    vrel_f: tuple[float, float, float, float]
    vrel_r: tuple[float, float, float, float]
    force_f: tuple[float, float, float, float]
    force_r: tuple[float, float, float, float]
    left_wheel_speed: float
    right_wheel_speed: float


def finite(value: object, default: float = 0.0) -> float:
    try:
        out = float(value)
        return out if math.isfinite(out) else default
    except (TypeError, ValueError):
        return default


def sign(value: float, eps: float = 1.0e-9) -> float:
    if value > eps:
        return 1.0
    if value < -eps:
        return -1.0
    return 0.0


def rmse(values: list[float]) -> float:
    return math.sqrt(sum(v * v for v in values) / len(values)) if values else 0.0


def quantile(values: list[float], p: float) -> float:
    clean = sorted(v for v in values if math.isfinite(v))
    if not clean:
        return 0.0
    pos = (len(clean) - 1) * p
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return clean[lo]
    frac = pos - lo
    return clean[lo] * (1.0 - frac) + clean[hi] * frac


def read_constants() -> dict[str, float]:
    with CONSTANTS.open(newline="", encoding="utf-8") as handle:
        return {row["name"]: finite(row["value"]) for row in csv.DictReader(handle)}


def read_secondary() -> dict[tuple[str, str], dict[str, str]]:
    fields = {
        "observed_yaw_moment_nm",
        "model_yaw_moment_nm",
        "left_drive_force_n",
        "right_drive_force_n",
        "left_wheel_speed_radps",
        "right_wheel_speed_radps",
    }
    for patch in PATCHES:
        fields.add(f"{patch}_v_rel_f_mps")
        fields.add(f"{patch}_v_rel_r_mps")
        fields.add(f"{patch}_force_f_n")
        fields.add(f"{patch}_force_r_n")
    out: dict[tuple[str, str], dict[str, str]] = {}
    with SECONDARY.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            out[(row["run_id"], row["row_index"])] = {field: row.get(field, "") for field in fields}
    return out


def included_inventory() -> tuple[set[str], dict[str, str]]:
    included: set[str] = set()
    statuses: dict[str, str] = {}
    with INVENTORY.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            run_id = row.get("run_id", "")
            status = row.get("status", "")
            statuses[run_id] = status
            if status.startswith("included"):
                included.add(run_id)
    return included, statuses


def load_samples() -> list[Sample]:
    secondary = read_secondary()
    included, _statuses = included_inventory()
    rows: list[Sample] = []
    with PRIMARY.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            if row.get("dataset_split") == "excluded_or_unclassified":
                continue
            if row.get("run_id", "").lower().startswith("fwc"):
                continue
            if row.get("run_id") not in included:
                continue
            extra = secondary.get((row["run_id"], row["row_index"]))
            if extra is None:
                continue
            rows.append(
                Sample(
                    run_id=row["run_id"],
                    split=row["dataset_split"],
                    row_index=int(finite(row["row_index"])),
                    time_us=finite(row.get("time_us")),
                    vf=finite(row.get("forward_velocity_mps")),
                    yaw_rate=finite(row.get("yaw_rate_radps")),
                    af=0.0,
                    yaw_accel=finite(row.get("measured_yaw_accel_radps2")),
                    observed_moment=finite(extra.get("observed_yaw_moment_nm")),
                    baseline_moment=finite(extra.get("model_yaw_moment_nm")),
                    left_drive_force=finite(extra.get("left_drive_force_n")),
                    right_drive_force=finite(extra.get("right_drive_force_n")),
                    vrel_f=tuple(finite(extra.get(f"{p}_v_rel_f_mps")) for p in PATCHES),
                    vrel_r=tuple(finite(extra.get(f"{p}_v_rel_r_mps")) for p in PATCHES),
                    force_f=tuple(finite(extra.get(f"{p}_force_f_n")) for p in PATCHES),
                    force_r=tuple(finite(extra.get(f"{p}_force_r_n")) for p in PATCHES),
                    left_wheel_speed=finite(extra.get("left_wheel_speed_radps")),
                    right_wheel_speed=finite(extra.get("right_wheel_speed_radps")),
                )
            )
    return add_forward_accel(rows)


def add_forward_accel(samples: list[Sample]) -> list[Sample]:
    by_run: dict[str, list[int]] = defaultdict(list)
    for index, sample in enumerate(samples):
        by_run[sample.run_id].append(index)
    accel = [0.0] * len(samples)
    for indices in by_run.values():
        ordered = sorted(indices, key=lambda i: (samples[i].time_us, samples[i].row_index))
        for pos, index in enumerate(ordered):
            before = ordered[pos - 1] if pos > 0 else None
            after = ordered[pos + 1] if pos + 1 < len(ordered) else None
            if before is not None and after is not None:
                dt = (samples[after].time_us - samples[before].time_us) / 1.0e6
                if 0.0 < dt <= 0.5:
                    accel[index] = (samples[after].vf - samples[before].vf) / dt
                    continue
            if before is not None:
                dt = (samples[index].time_us - samples[before].time_us) / 1.0e6
                if 0.0 < dt <= 0.25:
                    accel[index] = (samples[index].vf - samples[before].vf) / dt
                    continue
            if after is not None:
                dt = (samples[after].time_us - samples[index].time_us) / 1.0e6
                if 0.0 < dt <= 0.25:
                    accel[index] = (samples[after].vf - samples[index].vf) / dt
    return [
        Sample(
            s.run_id,
            s.split,
            s.row_index,
            s.time_us,
            s.vf,
            s.yaw_rate,
            accel[i],
            s.yaw_accel,
            s.observed_moment,
            s.baseline_moment,
            s.left_drive_force,
            s.right_drive_force,
            s.vrel_f,
            s.vrel_r,
            s.force_f,
            s.force_r,
            s.left_wheel_speed,
            s.right_wheel_speed,
        )
        for i, s in enumerate(samples)
    ]


def geometry(constants: dict[str, float]) -> tuple[tuple[float, float, str], ...]:
    half_track = 0.5 * constants["track_width_m"]
    offset = constants["drive_wheel_longitudinal_offset_m"]
    return (
        (-half_track, offset, "left"),
        (half_track, offset, "right"),
        (-half_track, -offset, "left"),
        (half_track, -offset, "right"),
    )


def model_basis(sample: Sample, constants: dict[str, float]) -> tuple[float, float, float, float, float, float]:
    force_drive = sample.left_drive_force + sample.right_drive_force
    force_kf = sum(sample.vrel_f)
    force_kr = sum(sample.vrel_r)
    moment_drive = 0.0
    moment_kf = 0.0
    moment_kr = 0.0
    for index, (x_right, y_forward, side) in enumerate(geometry(constants)):
        drive_force = 0.5 * (sample.left_drive_force if side == "left" else sample.right_drive_force)
        moment_drive += -x_right * drive_force
        moment_kf += -x_right * sample.vrel_f[index]
        moment_kr += y_forward * sample.vrel_r[index]
    return force_drive, force_kf, force_kr, moment_drive, moment_kf, moment_kr


def predict(sample: Sample, constants: dict[str, float], params: tuple[float, float, float]) -> tuple[float, float, float, float]:
    kx_right, ky_forward, drive_scale = params
    force_drive, force_kf, force_kr, moment_drive, moment_kf, moment_kr = model_basis(sample, constants)
    force_n = (drive_scale * force_drive) + (ky_forward * force_kf) + (kx_right * force_kr)
    moment_nm = (drive_scale * moment_drive) + (ky_forward * moment_kf) + (kx_right * moment_kr)
    return force_n, moment_nm, force_n / constants["mass_kg"], moment_nm / constants["yaw_denominator_including_wheel_spinup_kg_m2"]


def plant_reference(sample: Sample, constants: dict[str, float]) -> tuple[float, float, float, float]:
    force_n = sum(sample.force_f)
    moment_nm = sample.baseline_moment
    return force_n, moment_nm, force_n / constants["mass_kg"], moment_nm / constants["yaw_denominator_including_wheel_spinup_kg_m2"]


def bucket_edges(samples: list[Sample], count: int = 4) -> dict[str, list[float]]:
    axes = {
        "velocity_mps": [s.vf for s in samples],
        "yaw_rate_radps": [s.yaw_rate for s in samples],
        "accel_mps2": [s.af for s in samples],
        "yaw_accel_radps2": [s.yaw_accel for s in samples],
    }
    return {name: [quantile(values, i / count) for i in range(count + 1)] for name, values in axes.items()}


def bin_index(value: float, edges: list[float]) -> int:
    for index in range(len(edges) - 1):
        if value <= edges[index + 1] or index == len(edges) - 2:
            return index
    return len(edges) - 2


def bucket_key(sample: Sample, edges: dict[str, list[float]]) -> tuple[int, int, int, int]:
    return (
        bin_index(sample.vf, edges["velocity_mps"]),
        bin_index(sample.yaw_rate, edges["yaw_rate_radps"]),
        bin_index(sample.af, edges["accel_mps2"]),
        bin_index(sample.yaw_accel, edges["yaw_accel_radps2"]),
    )


def even_bucket_weights(samples: list[Sample], edges: dict[str, list[float]]) -> list[float]:
    keys = [bucket_key(s, edges) for s in samples]
    counts = Counter(keys)
    return [1.0 / counts[key] for key in keys]


def fit_rows(samples: list[Sample], constants: dict[str, float], weights: list[float]) -> list[tuple[list[float], float, float]]:
    # Unknowns are [kx_right, ky_forward, drive_scale]. Rows are stacked in acceleration units.
    rows: list[tuple[list[float], float, float]] = []
    yaw_scale = 0.025
    for sample, weight in zip(samples, weights):
        force_drive, force_kf, force_kr, moment_drive, moment_kf, moment_kr = model_basis(sample, constants)
        rows.append(([force_kr / constants["mass_kg"], force_kf / constants["mass_kg"], force_drive / constants["mass_kg"]], sample.af, weight))
        rows.append((
            [
                yaw_scale * moment_kr / constants["yaw_denominator_including_wheel_spinup_kg_m2"],
                yaw_scale * moment_kf / constants["yaw_denominator_including_wheel_spinup_kg_m2"],
                yaw_scale * moment_drive / constants["yaw_denominator_including_wheel_spinup_kg_m2"],
            ],
            yaw_scale * sample.yaw_accel,
            weight,
        ))
    return rows


def solve_fit_rows(rows: list[tuple[list[float], float, float]], active_columns: tuple[int, ...] = (0, 1, 2)) -> tuple[float, float, float]:
    n = len(active_columns)
    if n == 0:
        return 0.0, 0.0, 0.0

    lhs = [[0.0] * n for _ in range(n)]
    rhs = [0.0] * n
    for x, y, weight in rows:
        for i in range(n):
            xi = x[active_columns[i]]
            rhs[i] += weight * xi * y
            for j in range(n):
                lhs[i][j] += weight * xi * x[active_columns[j]]
    for i in range(n):
        lhs[i][i] += 1.0e-9
    aug = [lhs[i][:] + [rhs[i]] for i in range(n)]
    for col in range(n):
        pivot = max(range(col, n), key=lambda r: abs(aug[r][col]))
        aug[col], aug[pivot] = aug[pivot], aug[col]
        denom = aug[col][col]
        if abs(denom) < 1.0e-18:
            continue
        for j in range(col, n + 1):
            aug[col][j] /= denom
        for row in range(n):
            if row == col:
                continue
            factor = aug[row][col]
            for j in range(col, n + 1):
                aug[row][j] -= factor * aug[col][j]
    out = [0.0, 0.0, 0.0]
    for i, col in enumerate(active_columns):
        out[col] = aug[i][n]
    return out[0], out[1], out[2]


def weighted_sse(rows: list[tuple[list[float], float, float]], params: tuple[float, float, float]) -> float:
    return sum(weight * (sum(p * xi for p, xi in zip(params, x)) - y) ** 2 for x, y, weight in rows)


def solve_linear(samples: list[Sample], constants: dict[str, float], weights: list[float]) -> tuple[float, float, float]:
    return solve_fit_rows(fit_rows(samples, constants, weights))


def solve_nonnegative(samples: list[Sample], constants: dict[str, float], weights: list[float]) -> tuple[float, float, float]:
    rows = fit_rows(samples, constants, weights)
    best = (0.0, 0.0, 0.0)
    best_sse = weighted_sse(rows, best)
    for mask in range(1, 8):
        active = tuple(index for index in range(3) if (mask & (1 << index)) != 0)
        candidate = solve_fit_rows(rows, active)
        if any(value < -1.0e-9 for value in candidate):
            continue
        clipped = tuple(max(0.0, value) for value in candidate)
        sse = weighted_sse(rows, clipped)
        if sse < best_sse:
            best_sse = sse
            best = clipped
    return best


def metrics(samples: list[Sample], constants: dict[str, float], edges: dict[str, list[float]], params: tuple[float, float, float] | None) -> dict[str, float]:
    by_bucket: dict[tuple[int, int, int, int], dict[str, list[float]]] = defaultdict(lambda: defaultdict(list))
    overall: dict[str, list[float]] = defaultdict(list)
    for sample in samples:
        if params is None:
            force_n, moment_nm, accel, yaw_accel = plant_reference(sample, constants)
        else:
            force_n, moment_nm, accel, yaw_accel = predict(sample, constants, params)
        errors = {
            "force_rmse_n": force_n - constants["mass_kg"] * sample.af,
            "accel_rmse_mps2": accel - sample.af,
            "moment_rmse_nm": moment_nm - sample.observed_moment,
            "yaw_accel_rmse_radps2": yaw_accel - sample.yaw_accel,
        }
        key = bucket_key(sample, edges)
        for name, value in errors.items():
            overall[name].append(value)
            by_bucket[key][name].append(value)

    out: dict[str, float] = {"rows": float(len(samples)), "occupied_buckets": float(len(by_bucket))}
    for name, values in overall.items():
        out[f"overall_{name}"] = rmse(values)
        out[f"bucket_mean_{name}"] = sum(rmse(bucket[name]) for bucket in by_bucket.values()) / len(by_bucket)
    return out


def torque_from_command(command: float, wheel_speed_radps: float, constants: dict[str, float]) -> float:
    applied_voltage = command * constants["drive_voltage_v"]
    back_emf = wheel_speed_radps * constants["gear_ratio"] / constants["speed_constant_radps_per_volt"]
    current = (applied_voltage - back_emf) / constants["drive_resistance_ohms"]
    no_load_sign = sign(current) or sign(wheel_speed_radps)
    load_current = current - no_load_sign * constants["no_load_current_a"]
    if no_load_sign > 0.0 and load_current < 0.0:
        load_current = 0.0
    if no_load_sign < 0.0 and load_current > 0.0:
        load_current = 0.0
    return constants["torque_constant_nm_per_a"] * constants["gear_ratio"] * load_current


def command_from_torque(torque: float, wheel_speed_radps: float, constants: dict[str, float]) -> float:
    motor_torque = torque / constants["gear_ratio"]
    no_load_sign = sign(motor_torque) or sign(wheel_speed_radps)
    current = (motor_torque / constants["torque_constant_nm_per_a"]) + no_load_sign * constants["no_load_current_a"]
    back_emf = wheel_speed_radps * constants["gear_ratio"] / constants["speed_constant_radps_per_volt"]
    return ((current * constants["drive_resistance_ohms"]) + back_emf) / constants["drive_voltage_v"]


def in_place_projection(constants: dict[str, float], params: tuple[float, float, float], command: float = 0.54) -> dict[str, float | bool]:
    half_track = 0.5 * constants["track_width_m"]
    offset = constants["drive_wheel_longitudinal_offset_m"]
    wheel_radius = constants["wheel_radius_m"]
    wheel_speed = half_track / wheel_radius
    kx_right, _ky_forward, drive_scale = params
    # At +1 rad/s clockwise, front contacts have right-relative velocity -offset,
    # rear contacts +offset. That produces a negative/opposing yaw moment for kx>0.
    free_opposing = max(0.0, 4.0 * kx_right * offset * offset)
    needed_bank_torque = free_opposing * wheel_radius / (constants["track_width_m"] * max(drive_scale, 1.0e-12))
    threshold_left = command_from_torque(needed_bank_torque, wheel_speed, constants)
    threshold_right = command_from_torque(-needed_bank_torque, -wheel_speed, constants)
    commanded_left_force = torque_from_command(command, wheel_speed, constants) / wheel_radius
    commanded_right_force = torque_from_command(-command, -wheel_speed, constants) / wheel_radius
    drive_moment = drive_scale * half_track * (commanded_left_force - commanded_right_force)
    net_moment = drive_moment - free_opposing
    return {
        "yaw_rate_radps": 1.0,
        "opposing_slip_moment_nm": free_opposing,
        "threshold_left_command": threshold_left,
        "threshold_right_command": threshold_right,
        "threshold_max_abs_command": max(abs(threshold_left), abs(threshold_right)),
        "passes_min_0p54_threshold": max(abs(threshold_left), abs(threshold_right)) >= 0.54,
        "command_left": command,
        "command_right": -command,
        "net_moment_at_command_nm": net_moment,
        "net_yaw_accel_at_command_radps2": net_moment / constants["yaw_denominator_including_wheel_spinup_kg_m2"],
    }


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fields = list(rows[0].keys())
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    constants = read_constants()
    samples = load_samples()
    edges = bucket_edges(samples, count=5)
    weights = even_bucket_weights(samples, edges)
    unconstrained_params = solve_linear(samples, constants, weights)
    params = solve_nonnegative(samples, constants, weights)

    rows = []
    for label, subset in [
        ("all_available", samples),
        ("primary_open_floor_fit_authoritative", [s for s in samples if s.split == "primary_open_floor_fit_authoritative"]),
        ("validation_non_authoritative", [s for s in samples if s.split != "primary_open_floor_fit_authoritative"]),
    ]:
        if not subset:
            continue
        subset_edges = edges
        model_metrics = metrics(subset, constants, subset_edges, params)
        plant_metrics = metrics(subset, constants, subset_edges, None)
        rows.append({"model": "linear_anisotropic_patch", "group": label, **model_metrics})
        rows.append({"model": "current_plantmodel_reference", "group": label, **plant_metrics})
    write_csv(OUT / "rmse_metrics.csv", rows)

    selected = []
    for run_id in SELECTED_LOGS:
        subset = [s for s in samples if s.run_id == run_id]
        if not subset:
            selected.append({"run_id": run_id, "present": "no"})
            continue
        model_metrics = metrics(subset, constants, edges, params)
        selected.append({
            "run_id": run_id,
            "present": "yes",
            "split": subset[0].split,
            "rows": len(subset),
            "linear_overall_moment_rmse_nm": model_metrics["overall_moment_rmse_nm"],
            "linear_overall_yaw_accel_rmse_radps2": model_metrics["overall_yaw_accel_rmse_radps2"],
            "linear_overall_accel_rmse_mps2": model_metrics["overall_accel_rmse_mps2"],
        })
    write_csv(OUT / "selected_log_metrics.csv", selected)

    summary = {
        "input_primary": str(PRIMARY),
        "input_secondary": str(SECONDARY),
        "rows": len(samples),
        "runs": len(set(s.run_id for s in samples)),
        "bucket_design": {
            "type": "quintile quantile bins per axis over all evaluated rows",
            "axis_order": ["Velocity", "YawRate", "Acceleration", "YawAcceleration"],
            "edges": edges,
            "occupied_buckets": int(metrics(samples, constants, edges, params)["occupied_buckets"]),
            "scoring": "RMSE is computed per occupied 4D bucket, then averaged unweighted across occupied buckets; overall RMSE is ordinary row-weighted RMSE.",
        },
        "constants": constants,
        "inventory": {
            "path": str(INVENTORY),
            "filter": "kept status values starting with 'included'; fwc* rows explicitly excluded",
            "evaluated_run_count": len(set(s.run_id for s in samples)),
            "aux011_rows": sum(1 for s in samples if s.run_id == "aux011"),
        },
        "parameters": {
            "kx_right_N_per_mps": params[0],
            "ky_forward_N_per_mps": params[1],
            "drive_scale": params[2],
        },
        "unconstrained_diagnostic_parameters": {
            "kx_right_N_per_mps": unconstrained_params[0],
            "ky_forward_N_per_mps": unconstrained_params[1],
            "drive_scale": unconstrained_params[2],
            "note": "Unconstrained fit is not accepted when any damping/drive coefficient is negative.",
        },
        "formula": {
            "contact_velocity_definition": "vx,vy are body patch velocity relative floor; logged v_rel_r/v_rel_f are their negatives for passive slip, so fitted force uses kx*v_rel_r and ky*v_rel_f.",
            "per_patch_force": "F_right_i = kx * v_rel_r_i; F_forward_i = drive_scale * drive_force_side/2 + ky * v_rel_f_i.",
            "accumulation": "F_body = sum_i F_i; M_yaw = sum_i(y_i*F_right_i - x_i*F_forward_i).",
        },
        "in_place_projection": in_place_projection(constants, params),
    }
    (OUT / "summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False), encoding="utf-8")
    print(json.dumps({
        "out": str(OUT),
        "rows": len(samples),
        "runs": summary["runs"],
        "params": summary["parameters"],
        "in_place": summary["in_place_projection"],
    }, indent=2, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
