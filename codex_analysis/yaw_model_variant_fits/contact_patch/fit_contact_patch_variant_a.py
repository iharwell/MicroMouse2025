#!/usr/bin/env python3
"""Fit Variant A: contact-patch yaw correction from shared feature sample.

Analysis-only tooling. This script reads the shared phase-classified feature
sample and writes all outputs next to this file.
"""

from __future__ import annotations

import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Iterable


REPO_ROOT = Path(__file__).resolve().parents[3]
OUT_DIR = Path(__file__).resolve().parent
INPUT = (
    REPO_ROOT
    / "codex_analysis"
    / "contact_continuum_yaw_identification"
    / "ablation"
    / "phase_classified_feature_sample.csv"
)
CONSTANTS_INPUT = (
    REPO_ROOT
    / "codex_analysis"
    / "contact_continuum_yaw_identification"
    / "features"
    / "plant_mirror_constants.csv"
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

PRIMARY_SPLIT = "primary_open_floor_fit_authoritative"
DOWNWEIGHTED_FIT_SPLIT = "open_floor_fit_downweighted"
PRIMARY_HOLDOUT_RUNS = {
    "2026-04-20_12-10-58",
    "2026-04-21_05-32-06",
    "2026-04-22_01-06-32",
}

FEATURE_NAMES = [
    "q_patch_yaw_velocity_m2ps",
    "q_low_contact_speed",
    "q_low_forward_speed",
    "q_low_contact_and_forward",
    "tanh_q_low_contact_and_forward",
    "tanh_q_low_contact",
    "q_force_utilization",
    "q_force_utilization_low_contact",
    "q_abs_patch_velocity",
]


@dataclass(frozen=True)
class Sample:
    run_id: str
    family: str
    recommendation: str
    dataset_split: str
    physics_phase: str
    residual_nm: float
    yaw_rate_radps: float
    forward_velocity_mps: float
    vbar_rel_mps: float
    patch_yaw_velocity_basis_m2ps: float
    patch_yaw_abs_velocity_basis_m2ps: float
    max_force_preprojection_utilization: float
    limiter_active: float
    hardware_saturation_evidence: float
    gyro_derivative_spike: float


@dataclass(frozen=True)
class HyperParams:
    rel_speed_scale_mps: float
    forward_speed_scale_mps: float
    q_tanh_scale_m2ps: float
    ridge_lambda: float


def to_float(row: dict[str, str], key: str, default: float = 0.0) -> float:
    text = row.get(key, "")
    if text is None:
        return default
    text = text.strip()
    if not text:
        return default
    try:
        value = float(text)
    except ValueError:
        return default
    return value if math.isfinite(value) else default


def sign_for_reconstructed_basis(value: float, eps: float = 1.0e-5) -> float:
    if value > eps:
        return 1.0
    if value < -eps:
        return -1.0
    return 0.0


def load_samples() -> list[Sample]:
    samples: list[Sample] = []
    with INPUT.open("r", newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            residual = to_float(row, "residual_additive_yaw_torque_nm")
            yaw_rate = to_float(row, "yaw_rate_radps")
            patch_basis = to_float(row, "patch_yaw_velocity_basis_m2ps")
            if not math.isfinite(residual) or not math.isfinite(yaw_rate) or not math.isfinite(patch_basis):
                continue
            if abs(residual) > 2.0:
                continue
            samples.append(
                Sample(
                    run_id=row.get("run_id", ""),
                    family=row.get("family", ""),
                    recommendation=row.get("recommendation", ""),
                    dataset_split=row.get("dataset_split", ""),
                    physics_phase=row.get("physics_phase", ""),
                    residual_nm=residual,
                    yaw_rate_radps=yaw_rate,
                    forward_velocity_mps=to_float(row, "forward_velocity_mps"),
                    vbar_rel_mps=to_float(row, "vbar_rel_mps"),
                    patch_yaw_velocity_basis_m2ps=patch_basis,
                    patch_yaw_abs_velocity_basis_m2ps=to_float(row, "patch_yaw_abs_velocity_basis_m2ps"),
                    max_force_preprojection_utilization=to_float(row, "max_force_preprojection_utilization"),
                    limiter_active=to_float(row, "limiter_active"),
                    hardware_saturation_evidence=to_float(row, "hardware_saturation_evidence"),
                    gyro_derivative_spike=to_float(row, "gyro_derivative_spike"),
                )
            )
    return samples


def load_constants() -> dict[str, float]:
    constants: dict[str, float] = {}
    if not CONSTANTS_INPUT.is_file():
        return constants
    with CONSTANTS_INPUT.open("r", newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            constants[row["name"]] = float(row["value"])
    return constants


def fit_weight(sample: Sample) -> float:
    weight = 1.0
    if sample.dataset_split == DOWNWEIGHTED_FIT_SPLIT:
        weight *= 0.30
    if sample.physics_phase != "plateau":
        weight *= 0.65
    if sample.limiter_active > 0.5:
        weight *= 0.65
    if sample.hardware_saturation_evidence > 0.5:
        weight *= 0.35
    if sample.gyro_derivative_spike > 0.5:
        weight *= 0.25
    return weight


def smooth_gate(value: float, scale: float) -> float:
    if scale <= 0.0:
        return 0.0
    ratio = abs(value) / scale
    return 1.0 / (1.0 + ratio * ratio)


def sample_features(sample: Sample, hp: HyperParams) -> list[float]:
    # phase_classified_feature_sample stores patch_yaw_velocity_basis with yaw sign
    # folded in. Reconstruct the signed PlantModel-local q basis for fitting.
    yaw_sign = sign_for_reconstructed_basis(sample.yaw_rate_radps)
    q = yaw_sign * sample.patch_yaw_velocity_basis_m2ps
    low_rel = smooth_gate(sample.vbar_rel_mps, hp.rel_speed_scale_mps)
    low_forward = smooth_gate(sample.forward_velocity_mps, hp.forward_speed_scale_mps)
    q_tanh_scale = max(hp.q_tanh_scale_m2ps, 1.0e-9)
    tanh_q = math.tanh(q / q_tanh_scale)
    util = min(max(sample.max_force_preprojection_utilization, 0.0), 4.0)
    util_squash = util / (1.0 + util)
    return [
        q,
        q * low_rel,
        q * low_forward,
        q * low_rel * low_forward,
        tanh_q * low_rel * low_forward,
        tanh_q * low_rel,
        q * util_squash,
        q * util_squash * low_rel,
        q * sample.patch_yaw_abs_velocity_basis_m2ps,
    ]


def solve_linear_system(matrix: list[list[float]], rhs: list[float]) -> list[float] | None:
    n = len(rhs)
    a = [row[:] + [rhs[index]] for index, row in enumerate(matrix)]
    for col in range(n):
        pivot = max(range(col, n), key=lambda r: abs(a[r][col]))
        if abs(a[pivot][col]) < 1.0e-18:
            return None
        if pivot != col:
            a[col], a[pivot] = a[pivot], a[col]
        pivot_value = a[col][col]
        for j in range(col, n + 1):
            a[col][j] /= pivot_value
        for r in range(n):
            if r == col:
                continue
            factor = a[r][col]
            if factor == 0.0:
                continue
            for j in range(col, n + 1):
                a[r][j] -= factor * a[col][j]
    return [a[i][n] for i in range(n)]


def fit_model(samples: Iterable[Sample], hp: HyperParams) -> list[float] | None:
    rows = list(samples)
    feature_count = len(FEATURE_NAMES)
    total_weight = 0.0
    xtx = [[0.0 for _ in range(feature_count)] for _ in range(feature_count)]
    xty = [0.0 for _ in range(feature_count)]
    x2 = [0.0 for _ in range(feature_count)]

    for sample in rows:
        weight = fit_weight(sample)
        if weight <= 0.0:
            continue
        x = sample_features(sample, hp)
        y = sample.residual_nm
        total_weight += weight
        for j in range(feature_count):
            x2[j] += weight * x[j] * x[j]
            xty[j] += weight * x[j] * y
            row_j = xtx[j]
            xj = x[j]
            for k in range(j, feature_count):
                row_j[k] += weight * xj * x[k]

    if total_weight <= 0.0:
        return None

    scales = [math.sqrt(max(value / total_weight, 1.0e-24)) for value in x2]
    normalized = [[0.0 for _ in range(feature_count)] for _ in range(feature_count)]
    normalized_rhs = [0.0 for _ in range(feature_count)]
    for j in range(feature_count):
        normalized_rhs[j] = (xty[j] / total_weight) / scales[j]
        for k in range(j, feature_count):
            value = (xtx[j][k] / total_weight) / (scales[j] * scales[k])
            normalized[j][k] = value
            normalized[k][j] = value
    for j in range(feature_count):
        normalized[j][j] += hp.ridge_lambda

    solved = solve_linear_system(normalized, normalized_rhs)
    if solved is None:
        return None
    return [solved[j] / scales[j] for j in range(feature_count)]


def predict(sample: Sample, hp: HyperParams, coefficients: list[float]) -> float:
    return sum(coefficients[index] * value for index, value in enumerate(sample_features(sample, hp)))


def median(values: list[float]) -> float:
    if not values:
        return float("nan")
    ordered = sorted(values)
    n = len(ordered)
    mid = n // 2
    if n % 2:
        return ordered[mid]
    return 0.5 * (ordered[mid - 1] + ordered[mid])


def rmse(values: list[float]) -> float:
    if not values:
        return float("nan")
    return math.sqrt(sum(value * value for value in values) / len(values))


def mae(values: list[float]) -> float:
    if not values:
        return float("nan")
    return sum(abs(value) for value in values) / len(values)


def metrics_for(samples: list[Sample], hp: HyperParams, coefficients: list[float]) -> dict[str, float | int]:
    baseline = [sample.residual_nm for sample in samples]
    corrected = [sample.residual_nm - predict(sample, hp, coefficients) for sample in samples]
    predictions = [predict(sample, hp, coefficients) for sample in samples]
    baseline_rmse = rmse(baseline)
    corrected_rmse = rmse(corrected)
    baseline_sse = sum(value * value for value in baseline)
    corrected_sse = sum(value * value for value in corrected)
    return {
        "count": len(samples),
        "baseline_rmse_nm": baseline_rmse,
        "corrected_rmse_nm": corrected_rmse,
        "rmse_delta_nm": corrected_rmse - baseline_rmse,
        "rmse_delta_pct": ((corrected_rmse - baseline_rmse) / baseline_rmse * 100.0) if baseline_rmse else float("nan"),
        "baseline_mae_nm": mae(baseline),
        "corrected_mae_nm": mae(corrected),
        "baseline_median_abs_nm": median([abs(value) for value in baseline]),
        "corrected_median_abs_nm": median([abs(value) for value in corrected]),
        "baseline_signed_median_nm": median(baseline),
        "corrected_signed_median_nm": median(corrected),
        "r2_vs_zero_correction": 1.0 - (corrected_sse / baseline_sse) if baseline_sse > 0.0 else float("nan"),
        "worsened_abs_residual_fraction": (
            sum(1 for raw, corr in zip(baseline, corrected) if abs(corr) > abs(raw)) / len(samples)
            if samples
            else float("nan")
        ),
        "mean_prediction_nm": sum(predictions) / len(predictions) if predictions else float("nan"),
        "rmse_prediction_nm": rmse(predictions),
    }


def write_csv(path: Path, rows: list[dict[str, object]], fields: list[str]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def format_float(value: object) -> object:
    if isinstance(value, float):
        if math.isnan(value):
            return ""
        return f"{value:.9g}"
    return value


def metric_row(group: dict[str, object], samples: list[Sample], hp: HyperParams, coefficients: list[float]) -> dict[str, object]:
    row: dict[str, object] = dict(group)
    row.update(metrics_for(samples, hp, coefficients))
    return {key: format_float(value) for key, value in row.items()}


def by_group(samples: list[Sample], key_fn: Callable[[Sample], tuple[object, ...]]) -> dict[tuple[object, ...], list[Sample]]:
    grouped: dict[tuple[object, ...], list[Sample]] = {}
    for sample in samples:
        grouped.setdefault(key_fn(sample), []).append(sample)
    return grouped


def motion_bucket(sample: Sample) -> str:
    abs_v = abs(sample.forward_velocity_mps)
    abs_yaw = abs(sample.yaw_rate_radps)
    if abs_v < 0.05 and abs_yaw >= 0.20:
        return "low_forward_yaw"
    if abs_v >= 0.50 and abs_yaw >= 0.50:
        return "high_forward_yaw"
    if abs_v >= 0.50 and abs_yaw < 0.20:
        return "high_forward_low_yaw"
    if abs_v >= 0.05 and abs_yaw >= 0.20:
        return "moving_yaw"
    if abs_v >= 0.05:
        return "mostly_forward"
    return "low_motion"


def hyperparameter_grid() -> list[HyperParams]:
    rel_scales = [0.025, 0.06, 0.12]
    forward_scales = [0.06, 0.18, 0.45]
    q_scales = [0.0008, 0.0025, 0.008]
    ridge_values = [1.0e-7, 1.0e-5, 1.0e-3]
    return [
        HyperParams(rel, fwd, q, ridge)
        for rel in rel_scales
        for fwd in forward_scales
        for q in q_scales
        for ridge in ridge_values
    ]


def hp_to_dict(hp: HyperParams) -> dict[str, object]:
    return {
        "rel_speed_scale_mps": hp.rel_speed_scale_mps,
        "forward_speed_scale_mps": hp.forward_speed_scale_mps,
        "q_tanh_scale_m2ps": hp.q_tanh_scale_m2ps,
        "ridge_lambda": hp.ridge_lambda,
    }


def write_report(
    samples: list[Sample],
    train_rows: list[Sample],
    holdout_rows: list[Sample],
    hp: HyperParams,
    coefficients: list[float],
    constants: dict[str, float],
    selected_rows: list[dict[str, object]],
) -> None:
    split_metrics = metrics_for(train_rows, hp, coefficients)
    holdout_metrics = metrics_for(holdout_rows, hp, coefficients)
    downweighted = [s for s in samples if s.dataset_split == "open_floor_fit_downweighted"]
    validation = [s for s in samples if s.dataset_split in {"open_floor_validation_only", "diag_validation_only", "aux_downweighted_validation"}]
    downweighted_metrics = metrics_for(downweighted, hp, coefficients)
    validation_metrics = metrics_for(validation, hp, coefficients)
    high_forward_yaw = [s for s in samples if motion_bucket(s) == "high_forward_yaw"]
    high_forward_metrics = metrics_for(high_forward_yaw, hp, coefficients)
    phase_summary_rows: list[tuple[str, str, dict[str, float | int]]] = []
    for key, grouped in sorted(by_group(samples, lambda s: (s.dataset_split, s.physics_phase)).items()):
        phase_summary_rows.append((str(key[0]), str(key[1]), metrics_for(grouped, hp, coefficients)))

    coeff_lines = [
        f"- `{name}`: `{coefficients[index]:.9g}`"
        for index, name in enumerate(FEATURE_NAMES)
    ]
    denominator = constants.get("yaw_denominator_including_wheel_spinup_kg_m2", float("nan"))
    lines = [
        "# Variant A Contact-Patch Fit",
        "",
        "Analysis-only output. Production code, build metadata, and tests were not edited.",
        "",
        "## Model Form",
        "",
        "The fitted correction is a PlantModel-local force-level patch-yaw correction. In production terms, the signed contact patch yaw velocity basis is",
        "",
        "`q = sum_i load_fraction_i * (f_i * v_rel_r_i - r_i * v_rel_f_i)`.",
        "",
        "The analysis model predicts an added yaw moment:",
        "",
        "`delta_Mz = beta dot [q, q*g_rel, q*g_vf, q*g_rel*g_vf, tanh(q/q0)*g_rel*g_vf, tanh(q/q0)*g_rel, q*u, q*u*g_rel, q*abs_q]`.",
        "",
        "`g_rel = 1 / (1 + (vbar_rel / s_rel)^2)`, `g_vf = 1 / (1 + (Vf / s_vf)^2)`, and `u` is a squashed preprojection contact-utilization signal. This corresponds to distributing `delta_Mz` back into raw contact forces before friction projection with the existing load-weighted patch-yaw basis.",
        "",
        "## Selected Hyperparameters",
        "",
        f"- `s_rel`: `{hp.rel_speed_scale_mps:.9g} m/s`",
        f"- `s_vf`: `{hp.forward_speed_scale_mps:.9g} m/s`",
        f"- `q0`: `{hp.q_tanh_scale_m2ps:.9g} m^2/s`",
        f"- `ridge_lambda`: `{hp.ridge_lambda:.9g}`",
        f"- yaw denominator reference from constants: `{denominator:.9g} kg*m^2`",
        "",
        "## Coefficients",
        "",
        *coeff_lines,
        "",
        "## Performance Summary",
        "",
        "Fit input is authoritative open-floor train rows plus `open_floor_fit_downweighted` rows at 30% weight. Primary selected runs remain held out from the authoritative training subset.",
        "",
        "| Set | Rows | Baseline RMSE Nm | Corrected RMSE Nm | Delta % | Baseline MAE Nm | Corrected MAE Nm | R2 vs zero |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
        (
            f"| Weighted fit input | {split_metrics['count']} | {split_metrics['baseline_rmse_nm']:.6f} | "
            f"{split_metrics['corrected_rmse_nm']:.6f} | {split_metrics['rmse_delta_pct']:.2f} | "
            f"{split_metrics['baseline_mae_nm']:.6f} | {split_metrics['corrected_mae_nm']:.6f} | "
            f"{split_metrics['r2_vs_zero_correction']:.4f} |"
        ),
        (
            f"| Primary selected-run holdout | {holdout_metrics['count']} | {holdout_metrics['baseline_rmse_nm']:.6f} | "
            f"{holdout_metrics['corrected_rmse_nm']:.6f} | {holdout_metrics['rmse_delta_pct']:.2f} | "
            f"{holdout_metrics['baseline_mae_nm']:.6f} | {holdout_metrics['corrected_mae_nm']:.6f} | "
            f"{holdout_metrics['r2_vs_zero_correction']:.4f} |"
        ),
        (
            f"| Open-floor downweighted | {downweighted_metrics['count']} | {downweighted_metrics['baseline_rmse_nm']:.6f} | "
            f"{downweighted_metrics['corrected_rmse_nm']:.6f} | {downweighted_metrics['rmse_delta_pct']:.2f} | "
            f"{downweighted_metrics['baseline_mae_nm']:.6f} | {downweighted_metrics['corrected_mae_nm']:.6f} | "
            f"{downweighted_metrics['r2_vs_zero_correction']:.4f} |"
        ),
        (
            f"| Validation/competition | {validation_metrics['count']} | {validation_metrics['baseline_rmse_nm']:.6f} | "
            f"{validation_metrics['corrected_rmse_nm']:.6f} | {validation_metrics['rmse_delta_pct']:.2f} | "
            f"{validation_metrics['baseline_mae_nm']:.6f} | {validation_metrics['corrected_mae_nm']:.6f} | "
            f"{validation_metrics['r2_vs_zero_correction']:.4f} |"
        ),
        "",
        "## Phase Summary",
        "",
        "| Split | Phase | Rows | Baseline RMSE Nm | Corrected RMSE Nm | Delta % | Median abs before Nm | Median abs after Nm |",
        "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for split, phase, metrics in phase_summary_rows:
        lines.append(
            f"| {split} | {phase} | {metrics['count']} | {metrics['baseline_rmse_nm']:.6f} | "
            f"{metrics['corrected_rmse_nm']:.6f} | {metrics['rmse_delta_pct']:.2f} | "
            f"{metrics['baseline_median_abs_nm']:.6f} | {metrics['corrected_median_abs_nm']:.6f} |"
        )

    lines.extend(
        [
            "",
            "## High-Speed Sanity",
            "",
            (
                f"`high_forward_yaw` rows (`|Vf| >= 0.5 m/s`, `|yaw| >= 0.5 rad/s`): "
                f"{high_forward_metrics['count']} rows, RMSE {high_forward_metrics['baseline_rmse_nm']:.6f} -> "
                f"{high_forward_metrics['corrected_rmse_nm']:.6f} Nm "
                f"({high_forward_metrics['rmse_delta_pct']:.2f}%)."
            ),
            "",
            "## Selected Logs",
            "",
            "| Run | Present | Split | Rows | Baseline RMSE Nm | Corrected RMSE Nm | Median signed before Nm | Median signed after Nm |",
            "| --- | --- | --- | ---: | ---: | ---: | ---: | ---: |",
        ]
    )
    for row in selected_rows:
        if not row.get("present"):
            lines.append(f"| {row['run_id']} | no |  | 0 |  |  |  |  |")
            continue
        lines.append(
            f"| {row['run_id']} | yes | {row['dataset_split']} | {row['count']} | "
            f"{float(row['baseline_rmse_nm']):.6f} | {float(row['corrected_rmse_nm']):.6f} | "
            f"{float(row['baseline_signed_median_nm']):.6f} | {float(row['corrected_signed_median_nm']):.6f} |"
        )

    lines.extend(
        [
            "",
            "## Failure Modes",
            "",
            "- The model still cannot represent true static yaw breakaway at exactly zero contact-patch velocity; the `tanh(q/q0)` term is continuous and steep near zero, but it is still velocity-basis driven.",
            "- Entry and exit phases remain contaminated by timing delay and gyro differentiation noise, so coefficients were fit with lower transient weight and phase metrics should be read separately.",
            "- High-utilization rows use the preprojection utilization only as a smooth gain schedule. The analysis does not replay the full force projection after correction, so saturation-bound behavior is approximate.",
            "- Lateral body velocity is unavailable in the source logs; right-relative patch velocity assumes the existing feature extractor's `Vr = 0` reconstruction.",
            "",
            "## Files",
            "",
            "- `fit_contact_patch_variant_a.py`",
            "- `variant_a_coefficients.json`",
            "- `variant_a_hyperparameter_grid.csv`",
            "- `variant_a_metrics_by_split.csv`",
            "- `variant_a_metrics_by_phase.csv`",
            "- `variant_a_selected_log_metrics.csv`",
            "- `variant_a_motion_bucket_metrics.csv`",
            "- `variant_a_report.md`",
            "- `commands_run.txt`",
        ]
    )
    (OUT_DIR / "variant_a_report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    samples = load_samples()
    constants = load_constants()

    primary_train_rows = [
        s
        for s in samples
        if s.dataset_split == PRIMARY_SPLIT and s.run_id not in PRIMARY_HOLDOUT_RUNS
    ]
    downweighted_fit_rows = [
        s
        for s in samples
        if s.dataset_split == DOWNWEIGHTED_FIT_SPLIT
    ]
    train_rows = primary_train_rows + downweighted_fit_rows
    holdout_rows = [
        s
        for s in samples
        if s.dataset_split == PRIMARY_SPLIT and s.run_id in PRIMARY_HOLDOUT_RUNS
    ]

    grid_rows: list[dict[str, object]] = []
    best: tuple[float, HyperParams, list[float]] | None = None
    for hp in hyperparameter_grid():
        coefficients = fit_model(train_rows, hp)
        if coefficients is None:
            continue
        train_metrics = metrics_for(train_rows, hp, coefficients)
        holdout_metrics = metrics_for(holdout_rows, hp, coefficients)
        score = float(holdout_metrics["corrected_rmse_nm"])
        grid_row: dict[str, object] = hp_to_dict(hp)
        grid_row.update(
            {
                "train_count": train_metrics["count"],
                "train_baseline_rmse_nm": train_metrics["baseline_rmse_nm"],
                "train_corrected_rmse_nm": train_metrics["corrected_rmse_nm"],
                "train_r2_vs_zero_correction": train_metrics["r2_vs_zero_correction"],
                "holdout_count": holdout_metrics["count"],
                "holdout_baseline_rmse_nm": holdout_metrics["baseline_rmse_nm"],
                "holdout_corrected_rmse_nm": holdout_metrics["corrected_rmse_nm"],
                "holdout_r2_vs_zero_correction": holdout_metrics["r2_vs_zero_correction"],
            }
        )
        grid_rows.append({key: format_float(value) for key, value in grid_row.items()})
        if best is None or score < best[0]:
            best = (score, hp, coefficients)

    if best is None:
        raise RuntimeError("No hyperparameter fit succeeded")

    _, best_hp, best_coefficients = best

    split_rows: list[dict[str, object]] = []
    for key, grouped in sorted(by_group(samples, lambda s: (s.dataset_split,)).items()):
        split_rows.append(metric_row({"dataset_split": key[0]}, grouped, best_hp, best_coefficients))

    phase_rows: list[dict[str, object]] = []
    for key, grouped in sorted(by_group(samples, lambda s: (s.dataset_split, s.physics_phase)).items()):
        phase_rows.append(
            metric_row(
                {"dataset_split": key[0], "physics_phase": key[1]},
                grouped,
                best_hp,
                best_coefficients,
            )
        )

    selected_rows: list[dict[str, object]] = []
    all_by_run = by_group(samples, lambda s: (s.run_id,))
    for run_id in SELECTED_LOGS:
        grouped = all_by_run.get((run_id,), [])
        if not grouped:
            selected_rows.append({"run_id": run_id, "present": False, "dataset_split": "", "count": 0})
            continue
        split_names = sorted({sample.dataset_split for sample in grouped})
        selected_rows.append(
            metric_row(
                {
                    "run_id": run_id,
                    "present": True,
                    "dataset_split": ";".join(split_names),
                    "family": ";".join(sorted({sample.family for sample in grouped})),
                },
                grouped,
                best_hp,
                best_coefficients,
            )
        )

    motion_rows: list[dict[str, object]] = []
    for key, grouped in sorted(by_group(samples, lambda s: (s.dataset_split, motion_bucket(s))).items()):
        motion_rows.append(
            metric_row(
                {"dataset_split": key[0], "motion_bucket": key[1]},
                grouped,
                best_hp,
                best_coefficients,
            )
        )

    coefficient_payload = {
        "variant": "A_contact_patch_yaw_correction",
        "input": str(INPUT.relative_to(REPO_ROOT)),
        "training_split": PRIMARY_SPLIT,
        "primary_holdout_runs": sorted(PRIMARY_HOLDOUT_RUNS),
        "hyperparameters": hp_to_dict(best_hp),
        "feature_names": FEATURE_NAMES,
        "coefficients": {
            name: best_coefficients[index]
            for index, name in enumerate(FEATURE_NAMES)
        },
        "fit_weighting": {
            "open_floor_fit_downweighted_multiplier": 0.30,
            "entry_exit_phase_multiplier": 0.65,
            "limiter_active_multiplier": 0.65,
            "hardware_saturation_multiplier": 0.35,
            "gyro_derivative_spike_multiplier": 0.25,
        },
        "row_counts": {
            "all_loaded": len(samples),
            "primary_train": len(primary_train_rows),
            "downweighted_fit": len(downweighted_fit_rows),
            "weighted_fit_input": len(train_rows),
            "primary_holdout": len(holdout_rows),
        },
    }

    write_csv(
        OUT_DIR / "variant_a_hyperparameter_grid.csv",
        grid_rows,
        [
            "rel_speed_scale_mps",
            "forward_speed_scale_mps",
            "q_tanh_scale_m2ps",
            "ridge_lambda",
            "train_count",
            "train_baseline_rmse_nm",
            "train_corrected_rmse_nm",
            "train_r2_vs_zero_correction",
            "holdout_count",
            "holdout_baseline_rmse_nm",
            "holdout_corrected_rmse_nm",
            "holdout_r2_vs_zero_correction",
        ],
    )
    write_csv(
        OUT_DIR / "variant_a_metrics_by_split.csv",
        split_rows,
        [
            "dataset_split",
            "count",
            "baseline_rmse_nm",
            "corrected_rmse_nm",
            "rmse_delta_nm",
            "rmse_delta_pct",
            "baseline_mae_nm",
            "corrected_mae_nm",
            "baseline_median_abs_nm",
            "corrected_median_abs_nm",
            "baseline_signed_median_nm",
            "corrected_signed_median_nm",
            "r2_vs_zero_correction",
            "worsened_abs_residual_fraction",
            "mean_prediction_nm",
            "rmse_prediction_nm",
        ],
    )
    write_csv(
        OUT_DIR / "variant_a_metrics_by_phase.csv",
        phase_rows,
        [
            "dataset_split",
            "physics_phase",
            "count",
            "baseline_rmse_nm",
            "corrected_rmse_nm",
            "rmse_delta_nm",
            "rmse_delta_pct",
            "baseline_mae_nm",
            "corrected_mae_nm",
            "baseline_median_abs_nm",
            "corrected_median_abs_nm",
            "baseline_signed_median_nm",
            "corrected_signed_median_nm",
            "r2_vs_zero_correction",
            "worsened_abs_residual_fraction",
            "mean_prediction_nm",
            "rmse_prediction_nm",
        ],
    )
    write_csv(
        OUT_DIR / "variant_a_selected_log_metrics.csv",
        selected_rows,
        [
            "run_id",
            "present",
            "dataset_split",
            "family",
            "count",
            "baseline_rmse_nm",
            "corrected_rmse_nm",
            "rmse_delta_nm",
            "rmse_delta_pct",
            "baseline_mae_nm",
            "corrected_mae_nm",
            "baseline_median_abs_nm",
            "corrected_median_abs_nm",
            "baseline_signed_median_nm",
            "corrected_signed_median_nm",
            "r2_vs_zero_correction",
            "worsened_abs_residual_fraction",
            "mean_prediction_nm",
            "rmse_prediction_nm",
        ],
    )
    write_csv(
        OUT_DIR / "variant_a_motion_bucket_metrics.csv",
        motion_rows,
        [
            "dataset_split",
            "motion_bucket",
            "count",
            "baseline_rmse_nm",
            "corrected_rmse_nm",
            "rmse_delta_nm",
            "rmse_delta_pct",
            "baseline_mae_nm",
            "corrected_mae_nm",
            "baseline_median_abs_nm",
            "corrected_median_abs_nm",
            "baseline_signed_median_nm",
            "corrected_signed_median_nm",
            "r2_vs_zero_correction",
            "worsened_abs_residual_fraction",
            "mean_prediction_nm",
            "rmse_prediction_nm",
        ],
    )
    (OUT_DIR / "variant_a_coefficients.json").write_text(
        json.dumps(coefficient_payload, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    (OUT_DIR / "commands_run.txt").write_text(
        "python codex_analysis\\yaw_model_variant_fits\\contact_patch\\fit_contact_patch_variant_a.py\n",
        encoding="utf-8",
    )
    write_report(samples, train_rows, holdout_rows, best_hp, best_coefficients, constants, selected_rows)

    print(f"Loaded {len(samples)} samples")
    print(f"Primary train rows: {len(primary_train_rows)}")
    print(f"Downweighted fit rows: {len(downweighted_fit_rows)}")
    print(f"Primary holdout rows: {len(holdout_rows)}")
    print(f"Best hyperparameters: {hp_to_dict(best_hp)}")
    print(f"Report: {OUT_DIR / 'variant_a_report.md'}")


if __name__ == "__main__":
    main()
