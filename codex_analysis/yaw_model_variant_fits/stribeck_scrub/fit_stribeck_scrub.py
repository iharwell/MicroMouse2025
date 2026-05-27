#!/usr/bin/env python3
"""Fit Variant B: Stribeck/static-to-sliding yaw scrub correction.

This is analysis tooling only. It reads the shared feature table and writes all
outputs beside this script. It does not touch production code, build metadata,
or tests.
"""

from __future__ import annotations

import itertools
import math
from pathlib import Path
from typing import Iterable

import numpy as np
import pandas as pd


ROOT = Path(__file__).resolve().parents[3]
OUT_DIR = Path(__file__).resolve().parent
PRIMARY_INPUT = (
    ROOT
    / "codex_analysis"
    / "contact_continuum_yaw_identification"
    / "ablation"
    / "phase_classified_feature_sample.csv"
)
CONSTANTS_INPUT = (
    ROOT
    / "codex_analysis"
    / "contact_continuum_yaw_identification"
    / "features"
    / "plant_mirror_constants.csv"
)

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
    "forward_velocity_mps",
    "yaw_rate_radps",
    "measured_yaw_accel_radps2",
    "vbar_rel_mps",
    "vbar_yaw_mps",
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


def read_constants() -> dict[str, float]:
    if not CONSTANTS_INPUT.exists():
        return {}
    constants = {}
    table = pd.read_csv(CONSTANTS_INPUT)
    for row in table.itertuples(index=False):
        constants[str(row.name)] = float(row.value)
    return constants


def weighted_nnls(
    features: np.ndarray,
    target: np.ndarray,
    weights: np.ndarray,
    max_iter: int = 80,
    tolerance: float = 1.0e-11,
) -> np.ndarray:
    """Small dense non-negative least-squares solver.

    The model has only two or three columns, so cyclic coordinate descent is
    simpler and avoids requiring scipy in the analysis environment.
    """

    sqrt_w = np.sqrt(np.clip(weights, 0.0, None))
    xw = features * sqrt_w[:, None]
    yw = target * sqrt_w
    beta = np.zeros(features.shape[1], dtype=float)
    col_norm = np.sum(xw * xw, axis=0)
    col_norm = np.where(col_norm > 1.0e-18, col_norm, 1.0)
    residual = yw.copy()

    for _ in range(max_iter):
        max_delta = 0.0
        for col in range(features.shape[1]):
            old = beta[col]
            residual += xw[:, col] * old
            new = np.dot(xw[:, col], residual) / col_norm[col]
            if new < 0.0:
                new = 0.0
            beta[col] = new
            residual -= xw[:, col] * new
            max_delta = max(max_delta, abs(new - old))
        if max_delta < tolerance:
            break

    return beta


def smooth_positive(values: np.ndarray, epsilon: float = 1.0e-6) -> np.ndarray:
    return 0.5 * (values + np.sqrt(values * values + epsilon * epsilon))


def build_basis(
    frame: pd.DataFrame,
    yaw_activation_mps: float,
    req_activation_nm: float,
    stribeck_speed_mps: float,
    forward_fade_mps: float,
    rel_weight: float,
    include_viscous: bool,
    activation_mode: str,
) -> np.ndarray:
    abs_vf = frame["abs_forward_velocity_mps"].to_numpy()
    vbar_rel = frame["vbar_rel_mps"].to_numpy()
    vbar_yaw = frame["vbar_yaw_mps"].to_numpy()
    req_pos = frame["positive_patch_yaw_req_basis_nm"].to_numpy()

    yaw_motion_activation = 1.0 - np.exp(-np.square(vbar_yaw / yaw_activation_mps))
    request_activation = 1.0 - np.exp(-np.square(req_pos / req_activation_nm))
    if activation_mode == "request_only":
        activation = request_activation
    elif activation_mode == "request_or_yaw":
        activation = 1.0 - (1.0 - yaw_motion_activation) * (1.0 - request_activation)
    else:
        raise ValueError(f"unknown activation_mode: {activation_mode}")

    transition_speed = np.sqrt(np.square(rel_weight * vbar_rel) + np.square(abs_vf))
    stribeck = np.exp(-np.square(transition_speed / stribeck_speed_mps))
    speed_relief = 1.0 / (1.0 + np.square(transition_speed / forward_fade_mps))

    static_extra_basis = activation * stribeck * speed_relief
    sliding_basis = activation * speed_relief
    if include_viscous:
        yaw_viscous_basis = vbar_yaw * speed_relief
        return np.column_stack([static_extra_basis, sliding_basis, yaw_viscous_basis])
    return np.column_stack([static_extra_basis, sliding_basis])


def corrected_residuals(frame: pd.DataFrame, pred_opposes: np.ndarray) -> np.ndarray:
    pred_additive = -frame["yaw_sign"].to_numpy() * pred_opposes
    return frame["residual_additive_yaw_torque_nm"].to_numpy() - pred_additive


def metric_row(frame: pd.DataFrame, pred_opposes: np.ndarray) -> dict[str, float]:
    baseline = frame["residual_additive_yaw_torque_nm"].to_numpy()
    corrected = corrected_residuals(frame, pred_opposes)
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
            "rmse_improvement_pct": math.nan,
            "mae_improvement_pct": math.nan,
        }

    baseline_rmse = float(np.sqrt(np.mean(np.square(baseline))))
    corrected_rmse = float(np.sqrt(np.mean(np.square(corrected))))
    baseline_mae = float(np.mean(np.abs(baseline)))
    corrected_mae = float(np.mean(np.abs(corrected)))
    return {
        "count": int(len(frame)),
        "run_count": int(frame["run_id"].nunique()),
        "baseline_rmse_nm": baseline_rmse,
        "corrected_rmse_nm": corrected_rmse,
        "baseline_mae_nm": baseline_mae,
        "corrected_mae_nm": corrected_mae,
        "baseline_median_abs_nm": float(np.median(np.abs(baseline))),
        "corrected_median_abs_nm": float(np.median(np.abs(corrected))),
        "baseline_signed_median_nm": float(np.median(baseline)),
        "corrected_signed_median_nm": float(np.median(corrected)),
        "rmse_improvement_pct": 100.0 * (baseline_rmse - corrected_rmse) / baseline_rmse
        if baseline_rmse > 0.0
        else math.nan,
        "mae_improvement_pct": 100.0 * (baseline_mae - corrected_mae) / baseline_mae
        if baseline_mae > 0.0
        else math.nan,
    }


def add_predictions(frame: pd.DataFrame, pred_opposes: np.ndarray) -> pd.DataFrame:
    out = frame.copy()
    out["variant_b_predicted_opposing_yaw_scrub_nm"] = pred_opposes
    out["variant_b_predicted_additive_yaw_torque_nm"] = -out["yaw_sign"] * pred_opposes
    out["variant_b_corrected_residual_nm"] = (
        out["residual_additive_yaw_torque_nm"]
        - out["variant_b_predicted_additive_yaw_torque_nm"]
    )
    out["variant_b_corrected_opposes_yaw_nm"] = (
        -out["yaw_sign"] * out["variant_b_corrected_residual_nm"]
    )
    return out


def training_weights(frame: pd.DataFrame) -> np.ndarray:
    base = np.zeros(len(frame), dtype=float)
    split = frame["dataset_split"].to_numpy()
    recommendation = frame["recommendation"].to_numpy()
    family = frame["family"].to_numpy()

    base[split == "primary_open_floor_fit_authoritative"] = 1.0
    downweighted_fit = (
        (recommendation == "fit_downweighted")
        & (family == "open_floor")
        & (split == "open_floor_fit_downweighted")
    )
    base[downweighted_fit] = 0.25

    limiter = np.clip(frame["max_force_limiter_activity"].to_numpy(), 0.0, 1.0)
    saturation = np.clip(frame["hardware_saturation_evidence"].to_numpy(), 0.0, 1.0)
    spike = np.clip(frame["gyro_derivative_spike"].to_numpy(), 0.0, 1.0)
    quality = (1.0 / (1.0 + 4.0 * limiter)) * (1.0 - 0.75 * saturation) * (
        1.0 - 0.75 * spike
    )
    quality = np.clip(quality, 0.02, 1.0)

    weights = base * quality
    fit_counts = frame.loc[weights > 0.0, "run_id"].value_counts()
    if not fit_counts.empty:
        run_scale = frame["run_id"].map(
            {run: 1.0 / math.sqrt(count) for run, count in fit_counts.items()}
        ).fillna(0.0)
        weights *= run_scale.to_numpy()
        positive = weights > 0.0
        weights[positive] *= positive.sum() / weights[positive].sum()
    return weights


def fit_one(
    frame: pd.DataFrame,
    weights: np.ndarray,
    yaw_activation_mps: float,
    req_activation_nm: float,
    stribeck_speed_mps: float,
    forward_fade_mps: float,
    rel_weight: float,
    include_viscous: bool,
    activation_mode: str,
) -> tuple[np.ndarray, np.ndarray]:
    x = build_basis(
        frame,
        yaw_activation_mps=yaw_activation_mps,
        req_activation_nm=req_activation_nm,
        stribeck_speed_mps=stribeck_speed_mps,
        forward_fade_mps=forward_fade_mps,
        rel_weight=rel_weight,
        include_viscous=include_viscous,
        activation_mode=activation_mode,
    )
    y = frame["residual_opposes_yaw_nm"].to_numpy()
    beta = weighted_nnls(x, y, weights)
    return beta, x @ beta


def weighted_rmse(values: np.ndarray, weights: np.ndarray) -> float:
    positive = weights > 0.0
    if not np.any(positive):
        return math.nan
    return float(np.sqrt(np.average(np.square(values[positive]), weights=weights[positive])))


def hyperparameter_grid() -> Iterable[dict[str, object]]:
    yaw_activations = [0.002, 0.004, 0.008, 0.016]
    req_activations = [0.006, 0.015, 0.035, 0.080]
    stribeck_speeds = [0.006, 0.012, 0.025, 0.050, 0.100]
    forward_fades = [0.08, 0.16, 0.32, 0.64]
    rel_weights = [0.75, 1.25]
    include_viscous_options = [False, True]
    activation_modes = ["request_only", "request_or_yaw"]
    for values in itertools.product(
        yaw_activations,
        req_activations,
        stribeck_speeds,
        forward_fades,
        rel_weights,
        include_viscous_options,
        activation_modes,
    ):
        yield {
            "yaw_activation_mps": values[0],
            "req_activation_nm": values[1],
            "stribeck_speed_mps": values[2],
            "forward_fade_mps": values[3],
            "rel_weight": values[4],
            "include_viscous": values[5],
            "activation_mode": values[6],
        }


def motion_slice(frame: pd.DataFrame) -> pd.Series:
    abs_vf = frame["abs_forward_velocity_mps"]
    abs_yaw = frame["abs_yaw_rate_radps"]
    limiter = frame["max_force_limiter_activity"]
    slices = pd.Series("general", index=frame.index, dtype="object")
    slices[(abs_vf < 0.03) & (abs_yaw < 0.30)] = "low_speed_low_yaw_breakaway"
    slices[(abs_vf < 0.08) & (abs_yaw >= 0.30)] = "low_speed_turn_scrub"
    slices[(abs_vf >= 0.08) & (abs_vf < 0.35) & (abs_yaw >= 0.25)] = "mid_speed_turn"
    slices[(abs_vf >= 0.35) & (abs_yaw >= 0.25)] = "high_forward_turn"
    slices[(abs_yaw < 0.05) & (abs_vf >= 0.08)] = "straight_or_near_straight"
    slices[limiter > 0.15] = "force_limited_or_saturated_contact"
    return slices


def write_metric_tables(scored: pd.DataFrame) -> None:
    split_rows = []
    for split, subset in scored.groupby("dataset_split", sort=True):
        row = {"dataset_split": split}
        row.update(metric_row(subset, subset["variant_b_predicted_opposing_yaw_scrub_nm"].to_numpy()))
        split_rows.append(row)
    pd.DataFrame(split_rows).to_csv(OUT_DIR / "metrics_by_split.csv", index=False)

    phase_rows = []
    for (split, phase), subset in scored.groupby(["dataset_split", "physics_phase"], sort=True):
        row = {"dataset_split": split, "physics_phase": phase}
        row.update(metric_row(subset, subset["variant_b_predicted_opposing_yaw_scrub_nm"].to_numpy()))
        phase_rows.append(row)
    pd.DataFrame(phase_rows).to_csv(OUT_DIR / "metrics_by_split_phase.csv", index=False)

    motion_rows = []
    for (split, name), subset in scored.groupby(["dataset_split", "motion_slice"], sort=True):
        row = {"dataset_split": split, "motion_slice": name}
        row.update(metric_row(subset, subset["variant_b_predicted_opposing_yaw_scrub_nm"].to_numpy()))
        motion_rows.append(row)
    pd.DataFrame(motion_rows).to_csv(OUT_DIR / "metrics_by_motion_slice.csv", index=False)

    selected_rows = []
    for run in SELECTED_RUNS:
        subset = scored[scored["run_id"] == run]
        row = {"run_id": run, "present": bool(len(subset))}
        if len(subset):
            row.update(
                {
                    "dataset_split": ";".join(sorted(subset["dataset_split"].unique())),
                    "recommendation": ";".join(sorted(subset["recommendation"].unique())),
                    "family": ";".join(sorted(subset["family"].unique())),
                }
            )
            row.update(
                metric_row(
                    subset,
                    subset["variant_b_predicted_opposing_yaw_scrub_nm"].to_numpy(),
                )
            )
            row["median_predicted_opposing_scrub_nm"] = float(
                np.median(subset["variant_b_predicted_opposing_yaw_scrub_nm"].to_numpy())
            )
            row["median_residual_opposes_yaw_before_nm"] = float(
                np.median(subset["residual_opposes_yaw_nm"].to_numpy())
            )
            row["median_residual_opposes_yaw_after_nm"] = float(
                np.median(subset["variant_b_corrected_opposes_yaw_nm"].to_numpy())
            )
        selected_rows.append(row)
    pd.DataFrame(selected_rows).to_csv(OUT_DIR / "metrics_by_selected_run.csv", index=False)

    bin_frame = scored.copy()
    bin_frame["abs_forward_bin"] = pd.cut(
        bin_frame["abs_forward_velocity_mps"],
        bins=[-0.001, 0.03, 0.08, 0.20, 0.35, 0.60, 10.0],
        labels=["0-0.03", "0.03-0.08", "0.08-0.20", "0.20-0.35", "0.35-0.60", "0.60+"],
    )
    bin_frame["abs_yaw_bin"] = pd.cut(
        bin_frame["abs_yaw_rate_radps"],
        bins=[-0.001, 0.05, 0.30, 1.0, 3.0, 8.0, 100.0],
        labels=["0-0.05", "0.05-0.30", "0.30-1", "1-3", "3-8", "8+"],
    )
    bin_rows = []
    grouped = bin_frame.groupby(["abs_forward_bin", "abs_yaw_bin"], observed=True)
    for (vf_bin, yaw_bin), subset in grouped:
        if len(subset) < 100:
            continue
        corrected_opposes = subset["variant_b_corrected_opposes_yaw_nm"].to_numpy()
        row = {
            "abs_forward_bin_mps": vf_bin,
            "abs_yaw_bin_radps": yaw_bin,
            "count": len(subset),
            "run_count": subset["run_id"].nunique(),
            "median_target_opposes_yaw_nm": float(
                np.median(subset["residual_opposes_yaw_nm"].to_numpy())
            ),
            "median_predicted_opposing_scrub_nm": float(
                np.median(subset["variant_b_predicted_opposing_yaw_scrub_nm"].to_numpy())
            ),
            "median_corrected_opposes_yaw_nm": float(np.median(corrected_opposes)),
            "p10_corrected_opposes_yaw_nm": float(np.quantile(corrected_opposes, 0.10)),
            "p90_corrected_opposes_yaw_nm": float(np.quantile(corrected_opposes, 0.90)),
        }
        if row["median_corrected_opposes_yaw_nm"] > 0.003:
            row["fit_direction"] = "underfit_remaining_opposing_residual"
        elif row["median_corrected_opposes_yaw_nm"] < -0.003:
            if row["median_predicted_opposing_scrub_nm"] > 0.003:
                row["fit_direction"] = "overfit_added_too_much_resistance"
            else:
                row["fit_direction"] = "unaddressed_current_model_overresistance"
        else:
            row["fit_direction"] = "near_balanced"
        bin_rows.append(row)
    pd.DataFrame(bin_rows).to_csv(OUT_DIR / "over_under_fit_bins.csv", index=False)


def report_table(frame: pd.DataFrame, columns: list[str], limit: int | None = None) -> list[str]:
    if limit is not None:
        frame = frame.head(limit)
    lines = []
    lines.append("| " + " | ".join(columns) + " |")
    lines.append("| " + " | ".join("---" for _ in columns) + " |")
    for _, row in frame.iterrows():
        values = []
        for col in columns:
            value = row.get(col, "")
            if isinstance(value, float):
                if math.isnan(value):
                    values.append("")
                else:
                    values.append(f"{value:.6g}")
            else:
                values.append(str(value))
        lines.append("| " + " | ".join(values) + " |")
    return lines


def write_report(
    scored: pd.DataFrame,
    best: dict[str, object],
    constants: dict[str, float],
) -> None:
    split_metrics = pd.read_csv(OUT_DIR / "metrics_by_split.csv")
    selected_metrics = pd.read_csv(OUT_DIR / "metrics_by_selected_run.csv")
    motion_metrics = pd.read_csv(OUT_DIR / "metrics_by_motion_slice.csv")
    over_under = pd.read_csv(OUT_DIR / "over_under_fit_bins.csv")
    coeffs = pd.read_csv(OUT_DIR / "stribeck_coefficients.csv")

    report = [
        "# Variant B: Stribeck Static-to-Sliding Yaw Scrub Fit",
        "",
        "Analysis-only output. Production code, build metadata, and tests were not modified.",
        "",
        "## Model Form",
        "",
        "The fitted correction predicts an additional yaw torque residual that opposes the current yaw direction:",
        "",
        "`M_opp = A(v_yaw, M_req) * R(v_transition) * (K_slide + K_static * exp(-(v_transition / v_s)^2)) + K_viscous * v_yaw * R(v_transition)`",
        "",
        "where:",
        "",
        "- `A(v_yaw, M_req)` is selected by grid search as either request-only breakaway activation or a smooth union of yaw-patch motion and positive requested yaw-moment activation.",
        "- `v_transition = sqrt((rel_weight * vbar_rel)^2 + abs(Vf)^2)` drives the static-to-sliding Stribeck fade.",
        "- `R(v_transition) = 1 / (1 + (v_transition / speed_fade)^2)` limits this variant to low/contact-speed scrub rather than adding high-speed yaw drag.",
        "- The fitted torque is converted back to additive yaw torque as `-sign(yaw_rate) * M_opp`.",
        "",
        "This intentionally captures breakaway/static scrub and sliding scrub; it is not a full combined-slip contact model.",
        "",
        "## Data Basis",
        "",
        f"- Primary input: `{PRIMARY_INPUT.relative_to(ROOT)}`",
        f"- Rows evaluated: {len(scored)}",
        f"- Runs evaluated: {scored['run_id'].nunique()}",
        "- Fit-authoritative rows are the primary training signal.",
        "- Open-floor `fit_downweighted` rows contribute at reduced weight so the May 4 low-speed yaw-launch data influences the static breakaway term.",
        "- Validation-only and competition rows are reported only; they are not used for fitting.",
        "",
        "## Coefficients",
        "",
    ]
    report.extend(
        report_table(
            coeffs,
            [
                "parameter",
                "value",
                "unit",
            ],
        )
    )
    report.extend(
        [
            "",
            "## Split Metrics",
            "",
        ]
    )
    report.extend(
        report_table(
            split_metrics,
            [
                "dataset_split",
                "count",
                "run_count",
                "baseline_rmse_nm",
                "corrected_rmse_nm",
                "rmse_improvement_pct",
                "baseline_mae_nm",
                "corrected_mae_nm",
                "mae_improvement_pct",
                "baseline_median_abs_nm",
                "corrected_median_abs_nm",
            ],
        )
    )
    report.extend(
        [
            "",
            "## Selected Log Metrics",
            "",
        ]
    )
    report.extend(
        report_table(
            selected_metrics,
            [
                "run_id",
                "present",
                "dataset_split",
                "count",
                "baseline_rmse_nm",
                "corrected_rmse_nm",
                "rmse_improvement_pct",
                "baseline_signed_median_nm",
                "corrected_signed_median_nm",
                "median_residual_opposes_yaw_before_nm",
                "median_residual_opposes_yaw_after_nm",
            ],
        )
    )
    report.extend(
        [
            "",
            "## Motion Slice Metrics",
            "",
        ]
    )
    report.extend(
        report_table(
            motion_metrics.sort_values(["dataset_split", "motion_slice"]),
            [
                "dataset_split",
                "motion_slice",
                "count",
                "baseline_rmse_nm",
                "corrected_rmse_nm",
                "rmse_improvement_pct",
                "baseline_median_abs_nm",
                "corrected_median_abs_nm",
            ],
            limit=40,
        )
    )
    report.extend(
        [
            "",
            "## Over/Under Fit Summary",
            "",
        ]
    )
    directional = over_under.copy()
    directional["abs_median_corrected"] = directional["median_corrected_opposes_yaw_nm"].abs()
    directional = directional.sort_values("abs_median_corrected", ascending=False)
    report.extend(
        report_table(
            directional,
            [
                "abs_forward_bin_mps",
                "abs_yaw_bin_radps",
                "count",
                "run_count",
                "median_target_opposes_yaw_nm",
                "median_predicted_opposing_scrub_nm",
                "median_corrected_opposes_yaw_nm",
                "fit_direction",
            ],
            limit=12,
        )
    )
    report.extend(
        [
            "",
            "## Interpretation",
            "",
            "- The model is strongest in low-speed and low/medium-speed yaw-scrub rows where an attempted yaw moment exists but contact-relative speed is still low.",
            "- It sacrifices high-speed coverage deliberately: the transition-speed relief term prevents the May 4 static/breakaway fit from becoming a large high-speed or high-yaw-rate damper.",
            "- Remaining positive `median_corrected_opposes_yaw_nm` means this variant still underfits opposing resistance. Negative values with near-zero predicted scrub are existing current-model over-resistance that this one-sided scrub term cannot fix; negative values with nontrivial predicted scrub indicate over-added resistance.",
            "- Because the model is always opposing-yaw, it cannot represent rows where the current plant over-resists and the correction should assist yaw.",
            "- The static term relies on logged/contact-reconstructed requested yaw moment for near-zero-yaw launch behavior; a pure yaw-rate-only Stribeck model cannot explain the May 4 stalled-demand rows.",
            "",
            "## Output Files",
            "",
            "- `stribeck_coefficients.csv`",
            "- `hyperparameter_grid_top.csv`",
            "- `metrics_by_split.csv`",
            "- `metrics_by_split_phase.csv`",
            "- `metrics_by_motion_slice.csv`",
            "- `metrics_by_selected_run.csv`",
            "- `over_under_fit_bins.csv`",
            "- `prediction_sample.csv`",
            "",
            "## Constants Read",
            "",
            f"- yaw denominator including wheel spin-up: {constants.get('yaw_denominator_including_wheel_spinup_kg_m2', math.nan):.12g} kg m^2",
            f"- track width: {constants.get('track_width_m', math.nan):.12g} m",
            f"- drive wheel longitudinal offset: {constants.get('drive_wheel_longitudinal_offset_m', math.nan):.12g} m",
        ]
    )
    (OUT_DIR / "stribeck_scrub_report.md").write_text("\n".join(report) + "\n", encoding="utf-8")


def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    constants = read_constants()
    frame = pd.read_csv(PRIMARY_INPUT, usecols=INPUT_COLUMNS)
    numeric_columns = [
        "forward_velocity_mps",
        "yaw_rate_radps",
        "measured_yaw_accel_radps2",
        "vbar_rel_mps",
        "vbar_yaw_mps",
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
    for column in numeric_columns:
        frame[column] = pd.to_numeric(frame[column], errors="coerce")
    frame = frame.replace([np.inf, -np.inf], np.nan).dropna(
        subset=[
            "forward_velocity_mps",
            "yaw_rate_radps",
            "vbar_rel_mps",
            "vbar_yaw_mps",
            "residual_additive_yaw_torque_nm",
            "residual_opposes_yaw_nm",
            "patch_yaw_req_basis_nm",
        ]
    )
    frame["abs_forward_velocity_mps"] = frame["forward_velocity_mps"].abs()
    frame["abs_yaw_rate_radps"] = frame["yaw_rate_radps"].abs()
    frame["yaw_sign"] = np.sign(frame["yaw_rate_radps"].to_numpy())
    near_zero = frame["yaw_sign"] == 0.0
    frame.loc[near_zero, "yaw_sign"] = np.sign(frame.loc[near_zero, "patch_yaw_req_basis_nm"])
    frame["yaw_sign"] = frame["yaw_sign"].replace(0.0, 1.0)
    frame["positive_patch_yaw_req_basis_nm"] = smooth_positive(
        frame["patch_yaw_req_basis_nm"].to_numpy(), epsilon=1.0e-6
    )
    frame["motion_slice"] = motion_slice(frame)

    weights = training_weights(frame)
    y_opp = frame["residual_opposes_yaw_nm"].to_numpy()

    grid_rows = []
    best = None
    for params in hyperparameter_grid():
        beta, pred = fit_one(frame, weights, **params)
        fit_resid = y_opp - pred
        score = weighted_rmse(fit_resid, weights)
        corrected = corrected_residuals(frame, pred)
        primary = frame["dataset_split"] == "primary_open_floor_fit_authoritative"
        validation = frame["dataset_split"] == "open_floor_validation_only"
        grid_row = dict(params)
        grid_row.update(
            {
                "weighted_train_opposes_rmse_nm": score,
                "primary_corrected_rmse_nm": float(
                    np.sqrt(np.mean(np.square(corrected[primary.to_numpy()])))
                )
                if primary.any()
                else math.nan,
                "open_floor_validation_corrected_rmse_nm": float(
                    np.sqrt(np.mean(np.square(corrected[validation.to_numpy()])))
                )
                if validation.any()
                else math.nan,
                "coef_static_extra_nm": float(beta[0]),
                "coef_sliding_nm": float(beta[1]),
                "coef_yaw_viscous_nm_per_mps": float(beta[2]) if len(beta) > 2 else 0.0,
                "coef_count": int(len(beta)),
            }
        )
        grid_rows.append(grid_row)
        if best is None or score < best["weighted_train_opposes_rmse_nm"]:
            best = grid_row

    assert best is not None
    grid = pd.DataFrame(grid_rows).sort_values("weighted_train_opposes_rmse_nm")
    grid.head(40).to_csv(OUT_DIR / "hyperparameter_grid_top.csv", index=False)

    final_params = {
        "yaw_activation_mps": float(best["yaw_activation_mps"]),
        "req_activation_nm": float(best["req_activation_nm"]),
        "stribeck_speed_mps": float(best["stribeck_speed_mps"]),
        "forward_fade_mps": float(best["forward_fade_mps"]),
        "rel_weight": float(best["rel_weight"]),
        "include_viscous": bool(best["include_viscous"]),
        "activation_mode": str(best["activation_mode"]),
    }
    beta, pred = fit_one(frame, weights, **final_params)
    scored = add_predictions(frame, pred)
    write_metric_tables(scored)

    coefficient_rows = [
        {
            "parameter": "yaw_activation_mps",
            "value": final_params["yaw_activation_mps"],
            "unit": "m/s",
        },
        {
            "parameter": "req_activation_nm",
            "value": final_params["req_activation_nm"],
            "unit": "Nm",
        },
        {
            "parameter": "stribeck_speed_mps",
            "value": final_params["stribeck_speed_mps"],
            "unit": "m/s",
        },
        {
            "parameter": "speed_fade_mps",
            "value": final_params["forward_fade_mps"],
            "unit": "m/s",
        },
        {
            "parameter": "rel_weight",
            "value": final_params["rel_weight"],
            "unit": "dimensionless",
        },
        {
            "parameter": "activation_mode_request_only",
            "value": 1.0 if final_params["activation_mode"] == "request_only" else 0.0,
            "unit": "boolean",
        },
        {
            "parameter": "include_yaw_viscous_basis",
            "value": 1.0 if final_params["include_viscous"] else 0.0,
            "unit": "boolean",
        },
        {
            "parameter": "static_extra_nm",
            "value": float(beta[0]),
            "unit": "Nm",
        },
        {
            "parameter": "sliding_nm",
            "value": float(beta[1]),
            "unit": "Nm",
        },
        {
            "parameter": "yaw_viscous_nm_per_mps",
            "value": float(beta[2]) if len(beta) > 2 else 0.0,
            "unit": "Nm per (m/s)",
        },
        {
            "parameter": "weighted_train_opposes_rmse_nm",
            "value": float(best["weighted_train_opposes_rmse_nm"]),
            "unit": "Nm",
        },
    ]
    pd.DataFrame(coefficient_rows).to_csv(OUT_DIR / "stribeck_coefficients.csv", index=False)

    sample_columns = [
        "run_id",
        "dataset_split",
        "physics_phase",
        "row_index",
        "forward_velocity_mps",
        "yaw_rate_radps",
        "vbar_rel_mps",
        "vbar_yaw_mps",
        "patch_yaw_req_basis_nm",
        "residual_additive_yaw_torque_nm",
        "residual_opposes_yaw_nm",
        "variant_b_predicted_opposing_yaw_scrub_nm",
        "variant_b_predicted_additive_yaw_torque_nm",
        "variant_b_corrected_residual_nm",
        "variant_b_corrected_opposes_yaw_nm",
        "motion_slice",
    ]
    prediction_sample = pd.concat(
        [
            scored[scored["run_id"].isin(SELECTED_RUNS)].head(500),
            scored[scored["dataset_split"] == "primary_open_floor_fit_authoritative"].head(500),
        ],
        ignore_index=True,
    ).drop_duplicates(subset=["run_id", "row_index"])
    prediction_sample[sample_columns].to_csv(OUT_DIR / "prediction_sample.csv", index=False)

    command = (
        "& 'C:\\Users\\thene\\.cache\\codex-runtimes\\codex-primary-runtime\\dependencies\\python\\python.exe' "
        "codex_analysis\\yaw_model_variant_fits\\stribeck_scrub\\fit_stribeck_scrub.py"
    )
    (OUT_DIR / "commands_run.txt").write_text(command + "\n", encoding="utf-8")
    write_report(scored, best, constants)


if __name__ == "__main__":
    main()
