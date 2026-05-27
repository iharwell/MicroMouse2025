#!/usr/bin/env python3
"""Dynamic bristle-state yaw residual fit.

Analysis-only tooling. This is intentionally separate from the rejected
gate-shaped hybrid. The proposed traction law depends on contact-relative
velocity, normal load, projected contact force state, and an internal bristle
deflection state evolved from velocity history. Command/request values are not
traction-model inputs.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pandas as pd

import fit_round2_hybrid_b_c as base


ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent

RUNTIME_PYTHON = (
    Path.home()
    / ".cache"
    / "codex-runtimes"
    / "codex-primary-runtime"
    / "dependencies"
    / "python"
    / "python.exe"
)

SELECTED_LOGS = base.SELECTED_LOGS
IN_PLACE_ACCEPT_MIN_ABS_COMMAND = base.IN_PLACE_ACCEPT_MIN_ABS_COMMAND

MOVING_FEATURES = [
    "gain_front_right_basis__base",
    "gain_front_right_basis__low_rel",
    "gain_front_right_basis__high_forward",
    "gain_front_right_basis__force_util",
    "gain_front_right_basis__load_delta",
    "gain_rear_right_basis__base",
    "gain_rear_right_basis__low_rel",
    "gain_rear_right_basis__high_forward",
    "gain_rear_right_basis__force_util",
    "gain_rear_right_basis__load_delta",
    "gain_left_long_basis__base",
    "gain_left_long_basis__low_rel",
    "gain_left_long_basis__high_forward",
    "gain_left_long_basis__force_util",
    "gain_left_long_basis__load_delta",
    "gain_right_long_basis__base",
    "gain_right_long_basis__low_rel",
    "gain_right_long_basis__high_forward",
    "gain_right_long_basis__force_util",
    "gain_right_long_basis__load_delta",
    "force_moment_opposes_yaw_nm__base",
    "force_moment_opposes_yaw_nm__low_rel",
    "force_moment_opposes_yaw_nm__high_forward",
    "force_moment_opposes_yaw_nm__force_util",
    "force_abs_contact_moment_nm__force_util_signed",
]


@dataclass(frozen=True)
class DynamicConfig:
    vrel_knee_mps: float
    fwd_knee_mps: float
    bristle_length_m: float
    stribeck_speed_mps: float
    coulomb_fraction: float
    ridge: float
    rel_weight: float = 0.75


@dataclass
class DynamicModel:
    config: DynamicConfig
    nominal_load_n: float
    static_gain_nm: float
    feature_names: list[str]
    scales: np.ndarray
    beta: np.ndarray
    pred_by_index: pd.Series
    anchor_extra_nm: float

    def predict_frame(self, frame: pd.DataFrame) -> np.ndarray:
        if set(frame.index).issubset(set(self.pred_by_index.index)):
            return self.pred_by_index.loc[frame.index].to_numpy()
        state = steady_bristle_basis(frame, self.config, self.nominal_load_n)
        x = moving_feature_matrix(frame, self.config, self.feature_names, self.nominal_load_n)
        return self.static_gain_nm * state + base.clip_and_scale(x, self.scales) @ self.beta


def finite_metric_rows(frame: pd.DataFrame) -> pd.DataFrame:
    return frame[np.isfinite(frame["target_opposes_yaw_nm"])].copy()


def nominal_load(frame: pd.DataFrame) -> float:
    values = frame[
        (frame["dataset_split"] == "primary_open_floor_fit_authoritative")
        & (frame["total_normal_load_n"] > 0.0)
    ]["total_normal_load_n"]
    return float(values.median()) if len(values) else 1.0


def load_ratio(frame: pd.DataFrame, nominal_load_n: float) -> np.ndarray:
    return frame["total_normal_load_n"].to_numpy() / max(nominal_load_n, 1.0e-9)


def transition_speed(frame: pd.DataFrame, config: DynamicConfig) -> np.ndarray:
    vrel = np.maximum(frame["vbar_rel_mps"].to_numpy(), frame["load_weighted_rel_mps"].to_numpy())
    return np.hypot(config.rel_weight * vrel, frame["abs_forward_velocity_mps"].to_numpy())


def yaw_contact_speed(frame: pd.DataFrame, config: DynamicConfig) -> np.ndarray:
    vyaw = np.maximum(frame["vbar_yaw_mps"].to_numpy(), frame["load_weighted_lat_mps"].to_numpy())
    return config.rel_weight * vyaw


def lugre_g(vt: np.ndarray, config: DynamicConfig) -> np.ndarray:
    return config.coulomb_fraction + (1.0 - config.coulomb_fraction) * np.exp(
        -np.square(vt / max(config.stribeck_speed_mps, 1.0e-9))
    )


def compute_dynamic_bristle_basis(
    frame: pd.DataFrame,
    config: DynamicConfig,
    nominal_load_n: float,
) -> pd.Series:
    basis_values = np.zeros(len(frame), dtype="float64")
    run_codes = pd.factorize(frame["run_id"], sort=False)[0]
    order = np.lexsort(
        (
            frame["row_index"].to_numpy(),
            frame["time_us"].to_numpy(),
            run_codes,
        )
    )
    time_us = frame["time_us"].to_numpy(dtype="float64")
    yaw_dir = frame["yaw_direction"].to_numpy(dtype="float64")
    vbar_yaw = frame["vbar_yaw_mps"].to_numpy(dtype="float64")
    load_lat = frame["load_weighted_lat_mps"].to_numpy(dtype="float64")
    vbar_rel = frame["vbar_rel_mps"].to_numpy(dtype="float64")
    load_rel = frame["load_weighted_rel_mps"].to_numpy(dtype="float64")
    forward = frame["forward_velocity_mps"].to_numpy(dtype="float64")
    normal = frame["total_normal_load_n"].to_numpy(dtype="float64")
    state = 0.0
    prev_run: int | None = None
    prev_time: float | None = None
    for pos in order:
        run = int(run_codes[pos])
        current_time = float(time_us[pos])
        if run != prev_run:
            state = 0.0
            dt = 0.0
        else:
            raw_dt = (current_time - float(prev_time or current_time)) * 1.0e-6
            if raw_dt <= 0.0 or raw_dt > 0.250:
                state = 0.0
                dt = 0.0
            else:
                dt = min(raw_dt, 0.020)
        prev_run = run
        prev_time = current_time

        vy = config.rel_weight * max(float(vbar_yaw[pos]), float(load_lat[pos]))
        direction = float(yaw_dir[pos])
        signed_v = direction * vy
        vt = math.hypot(
            config.rel_weight * max(float(vbar_rel[pos]), float(load_rel[pos])),
            abs(float(forward[pos])),
        )
        g = float(lugre_g(np.array([vt]), config)[0])
        if dt > 0.0 and abs(signed_v) > 1.0e-9:
            rate = abs(signed_v) / max(config.bristle_length_m, 1.0e-9)
            steady = math.copysign(g, signed_v)
            state = steady + (state - steady) * math.exp(-rate * dt)
        state = min(max(state, -1.5), 1.5)
        opposes = max(0.0, direction * state)
        basis_values[pos] = (float(normal[pos]) / max(nominal_load_n, 1.0e-9)) * opposes
    return pd.Series(basis_values, index=frame.index)


def steady_bristle_basis(
    frame: pd.DataFrame,
    config: DynamicConfig,
    nominal_load_n: float,
) -> np.ndarray:
    vt = transition_speed(frame, config)
    g = lugre_g(vt, config)
    return load_ratio(frame, nominal_load_n) * g


def moving_feature_matrix(
    frame: pd.DataFrame,
    config: DynamicConfig,
    names: list[str],
    nominal_load_n: float,
) -> np.ndarray:
    vrel = np.maximum(frame["vbar_rel_mps"].to_numpy(), frame["load_weighted_rel_mps"].to_numpy())
    vf = frame["abs_forward_velocity_mps"].to_numpy()
    low_rel = 1.0 / (1.0 + np.square(vrel / max(config.vrel_knee_mps, 1.0e-9)))
    high_forward = 1.0 - 1.0 / (1.0 + np.square(vf / max(config.fwd_knee_mps, 1.0e-9)))
    force_util = frame["actual_force_util_smooth"].to_numpy()
    load_delta = load_ratio(frame, nominal_load_n) - 1.0
    schedules = {
        "base": np.ones(len(frame)),
        "low_rel": low_rel,
        "high_forward": high_forward,
        "force_util": force_util,
        "load_delta": load_delta,
    }
    values = []
    for name in names:
        field, suffix = name.split("__", 1)
        if suffix == "force_util_signed":
            values.append(frame[field].to_numpy() * force_util * frame["yaw_direction"].to_numpy())
        else:
            values.append(frame[field].to_numpy() * schedules[suffix])
    return np.column_stack(values)


def fit_weights(frame: pd.DataFrame) -> np.ndarray:
    weights = np.zeros(len(frame))
    split = frame["dataset_split"]
    weights[split == "primary_open_floor_fit_authoritative"] = 1.0
    down = (
        (split == "open_floor_fit_downweighted")
        & (frame["recommendation"] == "fit_downweighted")
        & (frame["family"] == "open_floor")
    )
    weights[down.to_numpy()] = 0.25
    saturation = np.clip(frame["hardware_saturation_evidence"].to_numpy(), 0.0, 1.0)
    spike = np.clip(frame["gyro_derivative_spike"].to_numpy(), 0.0, 1.0)
    quality = (1.0 - 0.75 * saturation) * (1.0 - 0.75 * spike)
    weights *= np.clip(quality, 0.02, 1.0)
    counts = frame.loc[weights > 0.0, "run_id"].value_counts()
    if len(counts):
        run_scale = frame["run_id"].map({run: 1.0 / math.sqrt(count) for run, count in counts.items()}).fillna(0.0)
        weights *= run_scale.to_numpy()
        positive = weights > 0.0
        weights[positive] *= positive.sum() / max(weights[positive].sum(), 1.0e-12)
    return weights


def fit_dynamic_model(
    frame: pd.DataFrame,
    constants: dict[str, float],
    config: DynamicConfig,
    anchor_extra_nm: float,
    nominal_load_n: float,
) -> DynamicModel:
    train_weights = fit_weights(frame)
    state_basis = compute_dynamic_bristle_basis(frame, config, nominal_load_n)
    synth = base.synthetic_base_frame(constants, base.HybridConfig(0.06, 0.70, 0.0, 0.0, 0.0, 0.008, 2.0), nominal_load_n, 0.0, 1.0, anchor_extra_nm)
    anchor_basis = float(steady_bristle_basis(synth, config, nominal_load_n)[0])
    static_gain = anchor_extra_nm / max(anchor_basis, 1.0e-12)
    static_pred = static_gain * state_basis.to_numpy()

    names = MOVING_FEATURES[:]
    raw_x = moving_feature_matrix(frame, config, names, nominal_load_n)
    fit_mask = train_weights > 0.0
    scales = np.array([base.quantile_abs(raw_x[fit_mask, idx], 0.80) for idx in range(raw_x.shape[1])])
    x = base.clip_and_scale(raw_x, scales)
    y = frame["target_opposes_yaw_nm"].to_numpy() - static_pred

    sqrt_w = np.sqrt(np.clip(train_weights, 0.0, None))
    xw = x * sqrt_w[:, None]
    yw = y * sqrt_w
    xtx = xw.T @ xw + np.eye(len(names)) * config.ridge
    xty = xw.T @ yw
    try:
        beta = np.linalg.solve(xtx, xty)
    except np.linalg.LinAlgError:
        beta = np.linalg.lstsq(xtx, xty, rcond=None)[0]
    pred = static_pred + x @ beta
    pred_by_index = pd.Series(pred, index=frame.index)
    return DynamicModel(config, nominal_load_n, static_gain, names, scales, beta, pred_by_index, anchor_extra_nm)


def dynamic_extra(model: DynamicModel, constants: dict[str, float], vf_mps: float, yaw_rate: float) -> float:
    extra = model.anchor_extra_nm if abs(vf_mps) < 1.0e-12 and abs(yaw_rate - 1.0) < 1.0e-12 else 0.0
    for _ in range(80):
        synth = base.synthetic_base_frame(
            constants,
            base.HybridConfig(0.06, 0.70, 0.0, 0.0, 0.0, 0.008, 2.0),
            model.nominal_load_n,
            vf_mps,
            yaw_rate,
            extra,
        )
        state = steady_bristle_basis(synth, model.config, model.nominal_load_n)
        x = moving_feature_matrix(synth, model.config, model.feature_names, model.nominal_load_n)
        pred = float(model.static_gain_nm * state[0] + (base.clip_and_scale(x, model.scales) @ model.beta)[0])
        if abs(pred - extra) < 1.0e-12:
            return pred
        extra = 0.65 * extra + 0.35 * pred
    return extra


def command_rows(model: DynamicModel, constants: dict[str, float], vf_mps: float, yaw_rate: float) -> list[dict[str, object]]:
    base_opp = base.baseline_opposing_yaw_torque(constants, yaw_rate)
    b = base.variant_b_extra(base_opp, constants, vf_mps, yaw_rate)
    c = base.variant_c_extra(base_opp, constants, vf_mps, yaw_rate)
    d = dynamic_extra(model, constants, vf_mps, yaw_rate)
    return [
        base.make_command_row("Baseline", 0.0, constants, vf_mps, yaw_rate, "current raw right-contact scrub approximation"),
        base.make_command_row("B_stribeck", b, constants, vf_mps, yaw_rate, "request-activated B reference; comparison only"),
        base.make_command_row("C_combined_slip", c, constants, vf_mps, yaw_rate, "request-aware C reference; comparison only"),
        base.make_command_row("Dynamic_LuGre_bristle", d, constants, vf_mps, yaw_rate, "physical dynamic bristle state plus force-state slip surface"),
    ]


def split_metrics(frame: pd.DataFrame, model: DynamicModel) -> pd.DataFrame:
    rows = []
    for split in [
        "primary_open_floor_fit_authoritative",
        "open_floor_fit_downweighted",
        "open_floor_validation_only",
        "diag_validation_only",
        "aux_downweighted_validation",
    ]:
        subset = frame[frame["dataset_split"] == split]
        if len(subset):
            rows.append(base.metric_row(split, subset, model))
    validation = frame[
        frame["dataset_split"].isin(
            ["open_floor_fit_downweighted", "open_floor_validation_only", "diag_validation_only", "aux_downweighted_validation"]
        )
    ]
    rows.append(base.metric_row("validation_non_authoritative", validation, model))
    return pd.DataFrame(rows)


def selected_metrics(frame: pd.DataFrame, model: DynamicModel) -> pd.DataFrame:
    rows = []
    for run in SELECTED_LOGS:
        subset = frame[frame["run_id"] == run]
        if len(subset):
            row = base.metric_row(run, subset, model)
            row["run_id"] = run
            row["present"] = True
            row["dataset_split"] = ";".join(sorted(subset["dataset_split"].unique()))
            rows.append(row)
        else:
            rows.append({"run_id": run, "present": False, "count": 0, "dataset_split": ""})
    return pd.DataFrame(rows)


def risk_metrics(frame: pd.DataFrame, model: DynamicModel) -> pd.DataFrame:
    rows = []
    for group, subset in base.risk_groups(frame).items():
        if len(subset):
            rows.append(base.metric_row(group, subset, model))
    return pd.DataFrame(rows)


def coefficients(model: DynamicModel) -> pd.DataFrame:
    rows = [
        {
            "branch": "dynamic_bristle",
            "feature": "lugre_bristle_state_basis",
            "standardized_coefficient_nm": model.static_gain_nm,
            "feature_scale": 1.0,
            "raw_coefficient_nm_per_feature": model.static_gain_nm,
            "abs_standardized_coefficient_nm": abs(model.static_gain_nm),
        }
    ]
    for name, scale, beta in zip(model.feature_names, model.scales, model.beta):
        rows.append(
            {
                "branch": "moving_contact",
                "feature": name,
                "standardized_coefficient_nm": beta,
                "feature_scale": scale,
                "raw_coefficient_nm_per_feature": beta / scale,
                "abs_standardized_coefficient_nm": abs(beta),
            }
        )
    return pd.DataFrame(rows).sort_values("abs_standardized_coefficient_nm", ascending=False)


def write_report(
    model: DynamicModel,
    tuning: pd.DataFrame,
    coeffs: pd.DataFrame,
    split_compare: pd.DataFrame,
    selected_compare: pd.DataFrame,
    risks: pd.DataFrame,
    grid: pd.DataFrame,
    in_place: pd.DataFrame,
) -> None:
    hybrid = in_place[in_place["variant"] == "Dynamic_LuGre_bristle"].iloc[0]
    pass_gate = bool(hybrid["in_place_acceptance_pass"])
    val_row = split_compare[split_compare["split"] == "validation_non_authoritative"].iloc[0]
    primary = split_compare[split_compare["split"] == "primary_open_floor_fit_authoritative"].iloc[0]
    h_val = float(val_row["hybrid_corrected_rmse_nm"])
    c_val = float(val_row["c_corrected_rmse_nm"])

    lines = [
        "# Dynamic LuGre/Bristle Yaw Model",
        "",
        "Analysis-only output. Production code, build metadata, and tests were not edited.",
        "",
        "## Reproduce",
        "",
        "```powershell",
        f"& '{RUNTIME_PYTHON}' codex_analysis\\yaw_model_variant_fits\\round2_hybrid_b_c\\fit_dynamic_bristle_lugre.py",
        "```",
        "",
        "## Model",
        "",
        "This is a different model family from the rejected gated hybrid. It introduces an internal LuGre/Dahl-style bristle deflection state `z` per log, evolved from physical yaw/contact relative velocity history:",
        "",
        "`z_dot = v_y / L - |v_y| * z / (L * g(v_t))`",
        "",
        "`g(v_t) = mu_c + (1 - mu_c) * exp(-(v_t / v_s)^2)`",
        "",
        "`M_opp = K_z * (N/N0) * max(0, d_yaw*z) + X_force_state(v_contact, F_projected, N, utilization_actual) * beta`",
        "",
        "The model does not use command, requested force, requested yaw moment, selector labels, or a residual lookup table as traction inputs. B and C appear only as comparison references and for the physical +1 rad/s calibration target.",
        "",
        "## Selected Parameters",
        "",
    ]
    hp = pd.DataFrame(
        [
            {
                "vrel_knee_mps": model.config.vrel_knee_mps,
                "fwd_knee_mps": model.config.fwd_knee_mps,
                "bristle_length_m": model.config.bristle_length_m,
                "stribeck_speed_mps": model.config.stribeck_speed_mps,
                "coulomb_fraction": model.config.coulomb_fraction,
                "ridge": model.config.ridge,
                "static_gain_nm": model.static_gain_nm,
                "nominal_load_n": model.nominal_load_n,
            }
        ]
    )
    lines.extend(base.table_lines(hp, list(hp.columns)))
    lines.extend(["", "## +1 rad/s In-Place Command", ""])
    lines.extend(
        base.table_lines(
            in_place,
            ["variant", "extra_opposing_yaw_torque_nm", "total_opposing_yaw_torque_nm", "left_command", "right_command", "lr_delta_command", "max_abs_command", "in_place_acceptance_pass"],
            ["variant", "extra Nm", "total opp Nm", "left cmd", "right cmd", "L-R delta", "max abs cmd", "gate"],
        )
    )
    lines.append("")
    lines.append(
        f"Hard gate at `Vf=0`, `Vr=0`, `yaw=+1 rad/s`: {'PASS' if pass_gate else 'FAIL'}. "
        f"Dynamic model predicts left/right `{float(hybrid['left_command']):.3f}/{float(hybrid['right_command']):.3f}`, "
        f"`max |cmd|={float(hybrid['max_abs_command']):.3f}` versus required `>= {IN_PLACE_ACCEPT_MIN_ABS_COMMAND:.3f}`."
    )
    lines.extend(["", "## Decision", ""])
    if pass_gate and h_val <= 1.10 * c_val:
        lines.append("Candidate status: provisionally acceptable for further validation.")
    else:
        lines.append("Candidate status: rejected as a production tune. It is physically compliant and materially different, but it does not retain C-like broad residual performance.")
    lines.append(
        f"Primary RMSE: dynamic {float(primary['hybrid_corrected_rmse_nm']):.6f} Nm, B {float(primary['b_corrected_rmse_nm']):.6f}, C {float(primary['c_corrected_rmse_nm']):.6f}. "
        f"Validation RMSE: dynamic {h_val:.6f} Nm, C {c_val:.6f}."
    )
    lines.extend(["", "## Split RMSE Versus B/C", ""])
    lines.extend(
        base.table_lines(
            split_compare,
            ["split", "count", "hybrid_baseline_rmse_nm", "b_corrected_rmse_nm", "c_corrected_rmse_nm", "hybrid_corrected_rmse_nm", "hybrid_vs_b_rmse_pct", "hybrid_vs_c_rmse_pct"],
            ["split", "count", "baseline", "B RMSE", "C RMSE", "Dynamic RMSE", "Dynamic vs B", "Dynamic vs C"],
        )
    )
    lines.extend(["", "## Selected Log RMSE Versus B/C", ""])
    lines.extend(
        base.table_lines(
            selected_compare,
            ["run_id", "dataset_split", "count", "hybrid_baseline_rmse_nm", "b_corrected_rmse_nm", "c_corrected_rmse_nm", "hybrid_corrected_rmse_nm", "hybrid_vs_b_rmse_pct", "hybrid_vs_c_rmse_pct"],
            ["run", "split", "count", "baseline", "B RMSE", "C RMSE", "Dynamic RMSE", "Dynamic vs B", "Dynamic vs C"],
        )
    )
    lines.extend(["", "## Risk Slices", ""])
    lines.extend(
        base.table_lines(
            risks,
            ["group", "count", "baseline_rmse_nm", "corrected_rmse_nm", "baseline_median_abs_nm", "corrected_median_abs_nm", "run_balanced_rmse_improvement_pct"],
            ["group", "count", "baseline", "dynamic", "median abs before", "median abs after", "RB change"],
        )
    )
    dyn_grid = grid[grid["variant"] == "Dynamic_LuGre_bristle"]
    b_grid = grid[grid["variant"] == "B_stribeck"].set_index(["vf_mps", "yaw_rate_radps"])
    c_grid = grid[grid["variant"] == "C_combined_slip"].set_index(["vf_mps", "yaw_rate_radps"])
    d_grid = dyn_grid.set_index(["vf_mps", "yaw_rate_radps"])
    lines.extend(["", "## 6x10 Vf/Yaw L-R Delta Grid Summary", ""])
    lines.append(
        f"Dynamic absolute L/R-delta difference from B: median {float((d_grid['lr_delta_command'] - b_grid['lr_delta_command']).abs().median()):.3f}, "
        f"p90 {float((d_grid['lr_delta_command'] - b_grid['lr_delta_command']).abs().quantile(0.90)):.3f}."
    )
    lines.append(
        f"Dynamic absolute L/R-delta difference from C: median {float((d_grid['lr_delta_command'] - c_grid['lr_delta_command']).abs().median()):.3f}, "
        f"p90 {float((d_grid['lr_delta_command'] - c_grid['lr_delta_command']).abs().quantile(0.90)):.3f}."
    )
    lines.append(
        f"Rows with `|cmd| > 1` for dynamic grid: {int(dyn_grid['command_outside_unit'].sum())} of {len(dyn_grid)}; raw contact-utilization > 1: {int(dyn_grid['contact_projection_sensitive'].sum())} of {len(dyn_grid)}."
    )
    lines.append("")
    lines.extend(base.pivot_table_lines(grid, "Dynamic_LuGre_bristle"))
    lines.extend(["## Tuning Scores", ""])
    lines.extend(
        base.table_lines(
            tuning.head(12),
            ["objective_with_gate", "objective_score", "bristle_length_m", "stribeck_speed_mps", "coulomb_fraction", "ridge", "in_place_max_abs_command", "in_place_acceptance_pass", "validation_objective_rb_corrected_rmse_nm"],
            ["gated obj", "objective", "L", "v_s", "mu_c", "ridge", "1rad |cmd|", "gate", "validation RB"],
        )
    )
    lines.extend(["", "## Dominant Coefficients", ""])
    lines.extend(
        base.table_lines(
            coeffs.head(16),
            ["branch", "feature", "standardized_coefficient_nm", "feature_scale", "raw_coefficient_nm_per_feature"],
            ["branch", "feature", "std coeff Nm", "scale", "raw coeff"],
        )
    )
    lines.extend(
        [
            "",
            "## Production Risks",
            "",
            "- The dynamic state must be integrated at the control-loop boundary with clear reset behavior after lift/service/discontinuous state.",
            "- The current fit uses sparse analysis rows, not a continuous replay of every control tick; state identification should be repeated on full-rate logs before production.",
            "- The +1 rad/s calibration is from the B command inversion/reference condition and should be validated against direct in-place launch measurements.",
            "- The moving force-state surface can still learn large coefficients; coefficient sign and scaling need a stricter physical prior before code implementation.",
            "- B/C comparisons include request-aware reference models only as benchmarks; the dynamic candidate itself does not use request/command traction inputs.",
            "",
            "## Output Files",
            "",
            "- `fit_dynamic_bristle_lugre.py`",
            "- `dynamic_bristle_lugre_report.md`",
            "- `dynamic_bristle_coefficients.csv`",
            "- `dynamic_selected_hyperparameters.json`",
            "- `dynamic_candidate_tuning_scores.csv`",
            "- `dynamic_split_metrics.csv`",
            "- `dynamic_split_rmse_comparison_vs_b_c.csv`",
            "- `dynamic_selected_log_metrics.csv`",
            "- `dynamic_selected_log_rmse_comparison_vs_b_c.csv`",
            "- `dynamic_risk_metrics.csv`",
            "- `dynamic_in_place_1radps_command.csv`",
            "- `dynamic_lr_delta_grid.csv`",
            "- `dynamic_commands_run.txt`",
        ]
    )
    (OUT / "dynamic_bristle_lugre_report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    constants = base.read_constants()
    frame = finite_metric_rows(base.load_rows(constants))
    nominal = nominal_load(frame)
    anchor_extra = base.variant_b_extra(base.baseline_opposing_yaw_torque(constants, 1.0), constants, 0.0, 1.0)

    configs = [
        DynamicConfig(vrel, 0.70, length, vs, mu_c, ridge)
        for vrel in [0.05, 0.08]
        for length in [0.0005, 0.0010, 0.0020]
        for vs in [0.06, 0.10]
        for mu_c in [0.65, 0.85]
        for ridge in [0.03]
    ]

    best_model: DynamicModel | None = None
    best_score = float("inf")
    tuning_rows = []
    validation = frame[
        frame["dataset_split"].isin(["open_floor_validation_only", "diag_validation_only", "aux_downweighted_validation"])
    ]
    for config in configs:
        model = fit_dynamic_model(frame, constants, config, anchor_extra, nominal)
        score = base.objective_score(model, frame)
        in_place_extra = dynamic_extra(model, constants, 0.0, 1.0)
        total = base.baseline_opposing_yaw_torque(constants, 1.0) + in_place_extra
        cmd = base.motor_commands_for_opposing_torque(total, constants, 0.0, 1.0)
        max_abs = max(abs(cmd["left_command"]), abs(cmd["right_command"]))
        passes = max_abs >= IN_PLACE_ACCEPT_MIN_ABS_COMMAND
        objective_with_gate = float(score["objective_score"]) if passes else 10.0 + float(score["objective_score"])
        row = {
            "vrel_knee_mps": config.vrel_knee_mps,
            "fwd_knee_mps": config.fwd_knee_mps,
            "bristle_length_m": config.bristle_length_m,
            "stribeck_speed_mps": config.stribeck_speed_mps,
            "coulomb_fraction": config.coulomb_fraction,
            "ridge": config.ridge,
            "static_gain_nm": model.static_gain_nm,
            "anchor_extra_nm": anchor_extra,
            "in_place_max_abs_command": max_abs,
            "in_place_acceptance_pass": passes,
            "objective_with_gate": objective_with_gate,
            **score,
        }
        tuning_rows.append(row)
        if objective_with_gate < best_score:
            best_score = objective_with_gate
            best_model = model
    if best_model is None:
        raise RuntimeError("no dynamic model selected")

    tuning = pd.DataFrame(tuning_rows).sort_values("objective_with_gate")
    split = split_metrics(frame, best_model)
    selected = selected_metrics(frame, best_model)
    risks = risk_metrics(frame, best_model)
    split_compare = base.compare_split_metrics(split)
    selected_compare = base.compare_selected_metrics(selected)
    coeffs = coefficients(best_model)

    grid_rows = []
    for vf in base.vf_grid():
        for yaw in base.yaw_grid():
            grid_rows.extend(command_rows(best_model, constants, vf, yaw))
    grid = pd.DataFrame(grid_rows)
    in_place = pd.DataFrame(command_rows(best_model, constants, 0.0, 1.0))

    base.write_frame(OUT / "dynamic_candidate_tuning_scores.csv", tuning)
    base.write_frame(OUT / "dynamic_bristle_coefficients.csv", coeffs)
    base.write_frame(OUT / "dynamic_split_metrics.csv", split)
    base.write_frame(OUT / "dynamic_split_rmse_comparison_vs_b_c.csv", split_compare)
    base.write_frame(OUT / "dynamic_selected_log_metrics.csv", selected)
    base.write_frame(OUT / "dynamic_selected_log_rmse_comparison_vs_b_c.csv", selected_compare)
    base.write_frame(OUT / "dynamic_risk_metrics.csv", risks)
    base.write_frame(OUT / "dynamic_lr_delta_grid.csv", grid)
    base.write_frame(OUT / "dynamic_in_place_1radps_command.csv", in_place)

    payload = {
        "model": "Dynamic_LuGre_bristle",
        "vrel_knee_mps": best_model.config.vrel_knee_mps,
        "fwd_knee_mps": best_model.config.fwd_knee_mps,
        "bristle_length_m": best_model.config.bristle_length_m,
        "stribeck_speed_mps": best_model.config.stribeck_speed_mps,
        "coulomb_fraction": best_model.config.coulomb_fraction,
        "ridge": best_model.config.ridge,
        "static_gain_nm": best_model.static_gain_nm,
        "nominal_load_n": best_model.nominal_load_n,
        "anchor_extra_opposing_yaw_torque_nm": best_model.anchor_extra_nm,
        "uses_command_or_request_as_traction_input": False,
    }
    (OUT / "dynamic_selected_hyperparameters.json").write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    (OUT / "dynamic_commands_run.txt").write_text(
        f"& '{RUNTIME_PYTHON}' codex_analysis\\yaw_model_variant_fits\\round2_hybrid_b_c\\fit_dynamic_bristle_lugre.py\n",
        encoding="utf-8",
    )
    write_report(best_model, tuning, coeffs, split_compare, selected_compare, risks, grid, in_place)


if __name__ == "__main__":
    main()
