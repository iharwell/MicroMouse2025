#!/usr/bin/env python3
"""Rational transition between force-domain Stribeck and Variant C.

Analysis-only tooling. Writes outputs only beside this script.
"""

from __future__ import annotations

import csv
import math
import sys
from collections import Counter
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pandas as pd


ROOT = Path(__file__).resolve().parents[4]
OUT = Path(__file__).resolve().parent
FORCE_DIR = ROOT / "codex_analysis" / "yaw_model_variant_fits" / "round2_force_domain_stribeck"
C_DIR = ROOT / "codex_analysis" / "yaw_model_variant_fits" / "combined_slip_surface"
UKF_AUDIT = (
    ROOT
    / "codex_analysis"
    / "yaw_model_variant_fits"
    / "round2_ukf_dependency_audit"
    / "ukf_dependency_audit_report.md"
)

sys.path.insert(0, str(FORCE_DIR))
sys.path.insert(0, str(C_DIR))

import fit_force_domain_stribeck as fd  # noqa: E402
import fit_combined_slip_surface as cc  # noqa: E402


SELECTED_RUNS = fd.SELECTED_RUNS
SPLITS = [
    "primary_open_floor_fit_authoritative",
    "open_floor_fit_downweighted",
    "open_floor_validation_only",
    "diag_validation_only",
    "aux_downweighted_validation",
]


@dataclass(frozen=True)
class BlendCandidate:
    name: str
    speed_knee_mps: float
    util_knee: float
    alpha: float
    rel_weight: float
    mode: str


def read_force_candidate() -> tuple[dict[str, float | str], dict[str, float]]:
    rows = list(csv.DictReader((FORCE_DIR / "force_domain_coefficients.csv").open(newline="", encoding="utf-8")))
    values: dict[str, float | str] = {}
    metadata: dict[str, float] = {}
    for row in rows:
        name = row["parameter"]
        raw = row["value"]
        if row["unit"] == "enum":
            values[name] = raw
        else:
            values[name] = float(raw)
        if name in {"nominal_longitudinal_yield_nm", "nominal_full_yaw_yield_nm"}:
            metadata[name] = float(raw)
    return values, metadata


def read_variant_c_model(rows: list[dict[str, object]]) -> cc.Model:
    coeff_rows = list(csv.DictReader((C_DIR / "model_coefficients.csv").open(newline="", encoding="utf-8")))
    names = [row["feature"] for row in coeff_rows]
    scales = [float(row["feature_scale"]) for row in coeff_rows]
    beta = [float(row["standardized_coefficient_nm"]) for row in coeff_rows]
    nominal = cc.selected_nominal_load(rows)
    return cc.Model(
        candidate="saturation_aware_surface",
        vrel_knee=0.060,
        fwd_knee=0.700,
        ridge=0.001,
        nominal_load=nominal,
        names=names,
        scales=scales,
        beta=beta,
    )


def key(run_id: object, row_index: object) -> tuple[str, int]:
    return str(run_id), int(float(row_index))


def build_frame() -> tuple[pd.DataFrame, dict[str, float | str], dict[str, float], dict[str, float]]:
    constants = fd.read_constants()
    force_values, force_metadata = read_force_candidate()
    force_frame, force_loader_metadata = fd.load_frame(constants)
    force_metadata = {**force_loader_metadata, **force_metadata}

    force_pred = fd.build_basis(
        force_frame,
        str(force_values["activation_source"]).replace("projected_force_moment_utilization", "patch_yaw_force_basis_nm")
        if False
        else "patch_yaw_force_basis_nm",
        "yield_longitudinal_moment_nm",
        float(force_values["utilization_activation"]),
        float(force_values["stribeck_speed_mps"]),
        float(force_values["speed_fade_mps"]),
        float(force_values["rel_weight"]),
    ) @ np.array([float(force_values["static_extra_nm"]), float(force_values["sliding_nm"])])

    c_rows = cc.load_rows()
    c_model = read_variant_c_model(c_rows)
    c_pred_by_key = {
        key(row["run_id"], row["row_index"]): c_model.predict_opposes(row)
        for row in c_rows
    }
    sign_by_key = {key(row["run_id"], row["row_index"]): cc.f(row, "sign_yaw") for row in c_rows}

    out = force_frame.copy()
    out["_row_key"] = [key(r.run_id, r.row_index) for r in out.itertuples(index=False)]
    out["force_pred_opposes_nm"] = force_pred
    out["variant_c_pred_opposes_nm"] = [c_pred_by_key.get(k, np.nan) for k in out["_row_key"]]
    out["variant_c_sign_yaw"] = [sign_by_key.get(k, np.nan) for k in out["_row_key"]]
    out = out.dropna(subset=["variant_c_pred_opposes_nm"]).copy()
    out["baseline_raw_residual_nm"] = out["residual_additive_yaw_torque_nm"]
    out["variant_c_raw_pred_nm"] = -out["variant_c_sign_yaw"] * out["variant_c_pred_opposes_nm"]
    out["force_raw_pred_nm"] = -out["yaw_sign"] * out["force_pred_opposes_nm"]

    source = fd.smooth_positive(out["patch_yaw_force_basis_nm"].to_numpy(), epsilon=1.0e-6)
    yield_moment = np.maximum(out["yield_longitudinal_moment_nm"].to_numpy(), 1.0e-12)
    out["force_utilization"] = source / yield_moment
    out["transition_v2_mps2"] = (
        float(force_values["rel_weight"]) ** 2 * np.square(out["vbar_rel_mps"].to_numpy())
        + np.square(out["abs_forward_velocity_mps"].to_numpy())
    )
    return out, force_values, force_metadata, constants


def blend_factor(frame: pd.DataFrame, cand: BlendCandidate) -> np.ndarray:
    kv2 = max(cand.speed_knee_mps * cand.speed_knee_mps, 1.0e-12)
    speed_low = kv2 / (kv2 + frame["transition_v2_mps2"].to_numpy())
    if cand.mode == "speed_only_partition":
        return np.clip(cand.alpha * speed_low, 0.0, 1.0)

    u = np.maximum(frame["force_utilization"].to_numpy(), 0.0)
    ku2 = max(cand.util_knee * cand.util_knee, 1.0e-12)
    util_gate = np.square(u) / (np.square(u) + ku2)
    if cand.mode in {"speed_force_partition", "positive_launch_overlay"}:
        return np.clip(cand.alpha * speed_low * util_gate, 0.0, 1.0)
    if cand.mode == "speed_force_partition_squared":
        return np.clip(cand.alpha * speed_low * np.square(util_gate), 0.0, 1.0)
    raise ValueError(cand.mode)


def predict_opposes(frame: pd.DataFrame, cand: BlendCandidate) -> np.ndarray:
    g = blend_factor(frame, cand)
    c_pred = frame["variant_c_pred_opposes_nm"].to_numpy()
    f_pred = frame["force_pred_opposes_nm"].to_numpy()
    if cand.mode == "positive_launch_overlay":
        return c_pred + g * np.maximum(f_pred - c_pred, 0.0)
    return c_pred + g * (f_pred - c_pred)


def corrected_raw_residual(frame: pd.DataFrame, pred_opposes: np.ndarray) -> np.ndarray:
    pred_raw = -frame["yaw_sign"].to_numpy() * pred_opposes
    return frame["baseline_raw_residual_nm"].to_numpy() - pred_raw


def rmse(values: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.square(values)))) if len(values) else math.nan


def mae(values: np.ndarray) -> float:
    return float(np.mean(np.abs(values))) if len(values) else math.nan


def median_abs(values: np.ndarray) -> float:
    return float(np.median(np.abs(values))) if len(values) else math.nan


def run_balanced_rmse(frame: pd.DataFrame, values: np.ndarray) -> float:
    counts = Counter(frame["run_id"].astype(str))
    weights = np.array([1.0 / max(counts[str(run)], 1) for run in frame["run_id"]], dtype=float)
    return float(np.sqrt(np.average(np.square(values), weights=weights))) if len(values) else math.nan


def metric_row(label: str, frame: pd.DataFrame, pred_opposes: np.ndarray) -> dict[str, object]:
    baseline = frame["baseline_raw_residual_nm"].to_numpy()
    corrected = corrected_raw_residual(frame, pred_opposes)
    base_rmse = rmse(baseline)
    corr_rmse = rmse(corrected)
    return {
        "group": label,
        "count": int(len(frame)),
        "run_count": int(frame["run_id"].nunique()) if len(frame) else 0,
        "baseline_rmse_nm": base_rmse,
        "corrected_rmse_nm": corr_rmse,
        "rmse_delta_vs_baseline_pct": 100.0 * (corr_rmse - base_rmse) / base_rmse if base_rmse > 0 else math.nan,
        "corrected_mae_nm": mae(corrected),
        "corrected_median_abs_nm": median_abs(corrected),
        "run_balanced_corrected_rmse_nm": run_balanced_rmse(frame, corrected),
        "median_pred_opposes_nm": float(np.median(pred_opposes)) if len(pred_opposes) else math.nan,
        "mean_blend": "",
    }


def baseline_metric_rows(frame: pd.DataFrame) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for name, column in [
        ("force_domain_stribeck_reference", "force_pred_opposes_nm"),
        ("variant_c_reference", "variant_c_pred_opposes_nm"),
    ]:
        pred = frame[column].to_numpy()
        validation = frame[frame["dataset_split"] != "primary_open_floor_fit_authoritative"]
        val_pred = validation[column].to_numpy()
        rows.append(
            {
                "candidate": name,
                "mode": "reference",
                "speed_knee_mps": "",
                "util_knee": "",
                "alpha": "",
                "primary_corrected_rmse_nm": metric_row(
                    "primary",
                    frame[frame["dataset_split"] == "primary_open_floor_fit_authoritative"],
                    pred[frame["dataset_split"].to_numpy() == "primary_open_floor_fit_authoritative"],
                )["corrected_rmse_nm"],
                "validation_corrected_rmse_nm": metric_row("validation", validation, val_pred)["corrected_rmse_nm"],
                "validation_rb_corrected_rmse_nm": metric_row("validation", validation, val_pred)[
                    "run_balanced_corrected_rmse_nm"
                ],
                "in_place_left_command": "",
                "in_place_right_command": "",
                "in_place_max_abs_command": "",
                "in_place_extra_opposing_nm": "",
            }
        )
    return rows


def objective(frame: pd.DataFrame, cand: BlendCandidate) -> dict[str, float]:
    pred = predict_opposes(frame, cand)
    train_mask = frame["dataset_split"].to_numpy() == "primary_open_floor_fit_authoritative"
    validation_mask = ~train_mask
    train = frame[train_mask]
    validation = frame[validation_mask]
    train_pred = pred[train_mask]
    validation_pred = pred[validation_mask]
    train_corr = corrected_raw_residual(train, train_pred)
    val_corr = corrected_raw_residual(validation, validation_pred)
    straight_mask = (frame["abs_yaw_rate_radps"].to_numpy() < 0.05) & (
        frame["abs_forward_velocity_mps"].to_numpy() >= 0.05
    )
    straight_corr = corrected_raw_residual(frame[straight_mask], pred[straight_mask])
    return {
        "primary_corrected_rmse_nm": rmse(train_corr),
        "validation_corrected_rmse_nm": rmse(val_corr),
        "validation_rb_corrected_rmse_nm": run_balanced_rmse(validation, val_corr),
        "straight_forward_corrected_rmse_nm": rmse(straight_corr),
        "mean_blend": float(np.mean(blend_factor(frame, cand))),
        "validation_mean_blend": float(np.mean(blend_factor(validation, cand))),
    }


def candidate_grid() -> list[BlendCandidate]:
    speed_knees = [0.025, 0.04, 0.06, 0.09, 0.13, 0.18, 0.26, 0.36, 0.50, 0.70]
    util_knees = [0.10, 0.18, 0.28, 0.40, 0.60, 0.85, 1.20]
    alphas = [0.50, 0.75, 1.00, 1.25]
    out: list[BlendCandidate] = []
    for kv in speed_knees:
        for alpha in alphas:
            out.append(BlendCandidate("speed_only_partition", kv, 0.0, alpha, 0.75, "speed_only_partition"))
        for ku in util_knees:
            for alpha in alphas:
                out.append(BlendCandidate("speed_force_partition", kv, ku, alpha, 0.75, "speed_force_partition"))
                out.append(
                    BlendCandidate(
                        "speed_force_partition_squared",
                        kv,
                        ku,
                        alpha,
                        0.75,
                        "speed_force_partition_squared",
                    )
                )
                out.append(BlendCandidate("positive_launch_overlay", kv, ku, alpha, 0.75, "positive_launch_overlay"))
    return out


def in_place_row(
    cand: BlendCandidate,
    force_values: dict[str, float | str],
    force_metadata: dict[str, float],
    constants: dict[str, float],
) -> dict[str, object]:
    force_command = fd.force_domain_extra_and_command(
        constants, force_values, force_metadata, vf_mps=0.0, yaw_rate=1.0
    )
    c_ref = {}
    with (FORCE_DIR / "in_place_1radps_command.csv").open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            c_ref[row["variant"]] = row
    c_extra = float(c_ref["Variant C combined slip"]["extra_opposing_yaw_torque_nm"])
    force_extra = float(force_command["extra_opposing_yaw_torque_nm"])
    base = float(force_command["baseline_opposing_yaw_torque_nm"])
    longitudinal = constants["drive_wheel_longitudinal_offset_m"]
    vbar_rel = longitudinal * 1.0
    v2 = (float(force_values["rel_weight"]) * vbar_rel) ** 2
    kv2 = cand.speed_knee_mps * cand.speed_knee_mps
    speed_low = kv2 / max(kv2 + v2, 1.0e-12)
    u = min(base + force_extra, float(force_metadata["nominal_longitudinal_yield_nm"])) / max(
        float(force_metadata["nominal_longitudinal_yield_nm"]), 1.0e-12
    )
    if cand.mode == "speed_only_partition":
        g = cand.alpha * speed_low
    else:
        util_gate = (u * u) / (u * u + cand.util_knee * cand.util_knee)
        if cand.mode == "speed_force_partition_squared":
            util_gate *= util_gate
        g = cand.alpha * speed_low * util_gate
    g = max(0.0, min(g, 1.0))
    if cand.mode == "positive_launch_overlay":
        extra = c_extra + g * max(force_extra - c_extra, 0.0)
    else:
        extra = c_extra + g * (force_extra - c_extra)
    command = fd.motor_commands_for_opposing_torque(base + extra, constants, 0.0, 1.0)
    return {
        "candidate": cand.name,
        "mode": cand.mode,
        "speed_knee_mps": cand.speed_knee_mps,
        "util_knee": cand.util_knee,
        "alpha": cand.alpha,
        "blend_gate": g,
        "baseline_opposing_yaw_torque_nm": base,
        "extra_opposing_yaw_torque_nm": extra,
        "total_opposing_yaw_torque_nm": base + extra,
        "left_command": command["left_command"],
        "right_command": command["right_command"],
        "lr_delta_command": command["lr_delta_command"],
        "max_abs_command": max(abs(command["left_command"]), abs(command["right_command"])),
    }


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    fields: list[str] = []
    for row in rows:
        for field in row:
            if field not in fields:
                fields.append(field)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fields, extrasaction="ignore", lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def fmt(value: object, digits: int = 6) -> str:
    if value == "" or value is None:
        return ""
    try:
        return f"{float(value):.{digits}f}"
    except (TypeError, ValueError):
        return str(value)


def row_matches_candidate(row: dict[str, object], cand: BlendCandidate) -> bool:
    return (
        row.get("mode") == cand.mode
        and row.get("speed_knee_mps") == cand.speed_knee_mps
        and row.get("util_knee") == cand.util_knee
        and row.get("alpha") == cand.alpha
    )


def make_report(
    best_by_mode: list[tuple[BlendCandidate, dict[str, float]]],
    selected: BlendCandidate,
    candidate_rows: list[dict[str, object]],
    split_rows: list[dict[str, object]],
    selected_log_rows: list[dict[str, object]],
    in_place_rows: list[dict[str, object]],
) -> None:
    selected_metrics = next(row for row in candidate_rows if row["candidate"] == selected.name and row["selected"])
    selected_in_place = next(row for row in in_place_rows if row_matches_candidate(row, selected))
    c_ref = next(row for row in candidate_rows if row["candidate"] == "variant_c_reference")
    f_ref = next(row for row in candidate_rows if row["candidate"] == "force_domain_stribeck_reference")

    lines = [
        "# Rational Speed/Force Blend Transition",
        "",
        "Analysis-only output. Production code, build metadata, and tests were not modified.",
        "",
        "## Recommendation",
        "",
        (
            "Recommended transition is `speed_force_partition` with "
            f"`k_v={selected.speed_knee_mps:.3f} m/s`, `k_u={selected.util_knee:.2f}`, "
            f"and `alpha={selected.alpha:.2f}`:"
        ),
        "",
        "`v2 = Vf_abs^2 + (0.75*vbar_rel)^2`",
        "",
        "`speed_low = k_v^2 / (k_v^2 + v2)`",
        "",
        "`force_gate = u^2 / (u^2 + k_u^2)` where `u = smooth_positive(M_projected_yaw)/M_yield`",
        "",
        "`blend = clamp(alpha * speed_low * force_gate, 0, 1)`",
        "",
        "`M_opposes = M_C + blend * (M_force_stribeck - M_C)`",
        "",
        (
            "This is a partitioned blend: at moving speed it returns to Variant C, while low-speed, high-force "
            "launch rows move toward the force-domain Stribeck prediction without adding both models together."
        ),
        "",
        "## Candidate Summary",
        "",
        "| Candidate | k_v | k_u | alpha | Primary RMSE | Validation RMSE | Validation RB RMSE | In-place cmd | In-place extra | Mean blend |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for row in [f_ref, c_ref] + [r for r in candidate_rows if r.get("selected")]:
        lines.append(
            f"| {row['candidate']} | {fmt(row.get('speed_knee_mps'), 3)} | {fmt(row.get('util_knee'), 2)} | "
            f"{fmt(row.get('alpha'), 2)} | {fmt(row['primary_corrected_rmse_nm'])} | "
            f"{fmt(row['validation_corrected_rmse_nm'])} | {fmt(row['validation_rb_corrected_rmse_nm'])} | "
            f"{fmt(row.get('in_place_max_abs_command'))} | {fmt(row.get('in_place_extra_opposing_nm'))} | "
            f"{fmt(row.get('mean_blend'))} |"
        )
    lines.extend(
        [
            "",
            "Best candidate by family:",
            "",
            "| Mode | k_v | k_u | alpha | Validation RB RMSE | In-place max command |",
            "| --- | ---: | ---: | ---: | ---: | ---: |",
        ]
    )
    for cand, metrics in best_by_mode:
        ip = next(row for row in in_place_rows if row_matches_candidate(row, cand))
        lines.append(
            f"| {cand.mode} | {cand.speed_knee_mps:.3f} | {cand.util_knee:.2f} | {cand.alpha:.2f} | "
            f"{metrics['validation_rb_corrected_rmse_nm']:.6f} | {float(ip['max_abs_command']):.6f} |"
        )
    lines.extend(
        [
            "",
            "## Split RMSE",
            "",
            "| Split | Count | Baseline RMSE | Corrected RMSE | Corrected MAE | Median abs after | Mean blend |",
            "| --- | ---: | ---: | ---: | ---: | ---: | ---: |",
        ]
    )
    for row in split_rows:
        lines.append(
            f"| {row['group']} | {row['count']} | {fmt(row['baseline_rmse_nm'])} | "
            f"{fmt(row['corrected_rmse_nm'])} | {fmt(row['corrected_mae_nm'])} | "
            f"{fmt(row['corrected_median_abs_nm'])} | {fmt(row['mean_blend'])} |"
        )
    lines.extend(
        [
            "",
            "## Selected Logs",
            "",
            "| Run | Split | Count | Baseline RMSE | Corrected RMSE | Mean blend |",
            "| --- | --- | ---: | ---: | ---: | ---: |",
        ]
    )
    for row in selected_log_rows:
        lines.append(
            f"| {row['run_id']} | {row['dataset_split']} | {row['count']} | {fmt(row['baseline_rmse_nm'])} | "
            f"{fmt(row['corrected_rmse_nm'])} | {fmt(row['mean_blend'])} |"
        )
    lines.extend(
        [
            "",
            "## In-Place Command",
            "",
            "Synthetic command estimate for `Vf=0`, `Vr=0`, `yawRate=+1 rad/s`:",
            "",
            "| Candidate | Blend gate | Extra opposing Nm | Total opposing Nm | Left cmd | Right cmd | Max abs cmd |",
            "| --- | ---: | ---: | ---: | ---: | ---: | ---: |",
        ]
    )
    for row in in_place_rows:
        if row["candidate"] in {"force_domain_stribeck_reference", "variant_c_reference"} or row is selected_in_place:
            lines.append(
                f"| {row['candidate']} | {fmt(row.get('blend_gate'))} | {fmt(row['extra_opposing_yaw_torque_nm'])} | "
                f"{fmt(row['total_opposing_yaw_torque_nm'])} | {fmt(row['left_command'])} | "
                f"{fmt(row['right_command'])} | {fmt(row['max_abs_command'])} |"
            )
    lines.extend(
        [
            "",
            "The selected blend gives the launch branch an in-place gate of "
            f"{float(selected_in_place['blend_gate']):.6f}, producing max command "
            f"{float(selected_in_place['max_abs_command']):.6f}. It clears the practical `|cmd| >= 0.6` "
            "launch check while avoiding the pure force-domain model's broader validation penalty.",
            "",
            "## Computational Cost",
            "",
            "Transition-only cost, assuming `Vf_abs`, `vbar_rel`, projected yaw moment, and yield moment are already available:",
            "",
            "- `speed_force_partition`: about 5 multiplies, 2 adds, 2 divides, 1 absolute/max or smooth-positive source clamp, and 1 clamp. No sqrt, trig, exp, or table.",
            "- If `u = smooth_positive(M)` is implemented branchlessly, add one multiply, two adds, one sqrt, and one multiply by 0.5. A branch/clamp `max(M,0)` is cheaper and acceptable if the sign convention is explicit.",
            "- The selected transition has one final clamp. The partition equation itself has no data-dependent branch.",
            "- `positive_launch_overlay` adds one `max(F-C,0)` branch/clamp; it was kept as a comparison because it prevents low-speed double counting, but it is less clean as the canonical partition.",
            "",
            "## Strengths And Failures",
            "",
            "- Strength: no trig, no exp, no table, and the selected transition can be evaluated with squared speeds rather than `sqrt(v2)`.",
            "- Strength: force utilization prevents a low-speed zero-force row from suppressing Variant C just because the robot is slow.",
            "- Strength: the in-place command lands near the force-domain branch rather than Variant C's underpowered in-place estimate.",
            "- Caveat: validation RMSE improves slightly versus pure Variant C, but the margin is small; treat it as transition-shape evidence, not as proof that the launch branch is globally better.",
            "- Failure: the transition relies on projected/actual contact yaw-moment utilization, so it belongs after contact projection or inside the same plant solve. Using pre-projection command/request moment would reintroduce the rejected command-conditioned path.",
            "- Failure: the force-domain source model still uses its existing Stribeck exponent internally; this work only removes expensive functions from the transition gate.",
            "",
            "## Provenance",
            "",
            f"UKF fields were not used. Feature provenance follows `{UKF_AUDIT.relative_to(ROOT)}`, which reports no fitted input, target, or residual path directly uses logged `ukf_state_*`, estimator state-vector, Kalman, or estimator yaw-rate columns.",
            "",
            "## Reproduce",
            "",
            "```powershell",
            "& 'C:\\Users\\thene\\.cache\\codex-runtimes\\codex-primary-runtime\\dependencies\\python\\python.exe' codex_analysis\\yaw_model_variant_fits\\transition_options\\rational_speed_force_blend\\fit_rational_speed_force_blend.py",
            "```",
            "",
            "## Output Files",
            "",
            "- `fit_rational_speed_force_blend.py`",
            "- `rational_speed_force_blend_report.md`",
            "- `candidate_scores.csv`",
            "- `split_metrics.csv`",
            "- `selected_log_metrics.csv`",
            "- `in_place_1radps_command.csv`",
            "- `commands_run.txt`",
            "",
        ]
    )
    (OUT / "rational_speed_force_blend_report.md").write_text("\n".join(lines), encoding="utf-8")


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    frame, force_values, force_metadata, constants = build_frame()

    candidate_rows = baseline_metric_rows(frame)
    in_place_rows: list[dict[str, object]] = []
    base_in_place = {}
    with (FORCE_DIR / "in_place_1radps_command.csv").open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            base_in_place[row["variant"]] = row
    for candidate_name, variant_name in [
        ("force_domain_stribeck_reference", "ForceDomainStribeck_projected_force"),
        ("variant_c_reference", "Variant C combined slip"),
    ]:
        row = base_in_place[variant_name]
        in_place_rows.append(
            {
                "candidate": candidate_name,
                "mode": "reference",
                "blend_gate": "",
                "extra_opposing_yaw_torque_nm": float(row["extra_opposing_yaw_torque_nm"]),
                "total_opposing_yaw_torque_nm": float(row["total_opposing_yaw_torque_nm"]),
                "left_command": float(row["left_command"]),
                "right_command": float(row["right_command"]),
                "lr_delta_command": float(row["lr_delta_command"]),
                "max_abs_command": float(row["max_abs_command"]),
            }
        )

    scored: list[tuple[BlendCandidate, dict[str, float]]] = []
    for cand in candidate_grid():
        metrics = objective(frame, cand)
        ip = in_place_row(cand, force_values, force_metadata, constants)
        scored.append((cand, metrics))
        candidate_rows.append(
            {
                "candidate": cand.name,
                "mode": cand.mode,
                "speed_knee_mps": cand.speed_knee_mps,
                "util_knee": cand.util_knee,
                "alpha": cand.alpha,
                "primary_corrected_rmse_nm": metrics["primary_corrected_rmse_nm"],
                "validation_corrected_rmse_nm": metrics["validation_corrected_rmse_nm"],
                "validation_rb_corrected_rmse_nm": metrics["validation_rb_corrected_rmse_nm"],
                "straight_forward_corrected_rmse_nm": metrics["straight_forward_corrected_rmse_nm"],
                "mean_blend": metrics["mean_blend"],
                "validation_mean_blend": metrics["validation_mean_blend"],
                "in_place_left_command": ip["left_command"],
                "in_place_right_command": ip["right_command"],
                "in_place_max_abs_command": ip["max_abs_command"],
                "in_place_extra_opposing_nm": ip["extra_opposing_yaw_torque_nm"],
                "in_place_blend_gate": ip["blend_gate"],
                "passes_in_place_0p6": abs(float(ip["left_command"])) >= 0.6 and abs(float(ip["right_command"])) >= 0.6,
                "selected": False,
            }
        )
        in_place_rows.append(ip)

    # Prefer the cheapest clean partition that passes the in-place command gate.
    eligible = [
        (cand, metrics)
        for cand, metrics in scored
        if cand.mode == "speed_force_partition"
        and float(
            next(
                row
                for row in candidate_rows
                if row["candidate"] == cand.name
                and row["mode"] == cand.mode
                and row["speed_knee_mps"] == cand.speed_knee_mps
                and row["util_knee"] == cand.util_knee
                and row["alpha"] == cand.alpha
            )["in_place_max_abs_command"]
        )
        >= 0.6
    ]
    selected = min(eligible, key=lambda item: item[1]["validation_rb_corrected_rmse_nm"])[0]
    for row in candidate_rows:
        row["selected"] = (
            row.get("mode") == selected.mode
            and row.get("speed_knee_mps") == selected.speed_knee_mps
            and row.get("util_knee") == selected.util_knee
            and row.get("alpha") == selected.alpha
        )

    best_by_mode: list[tuple[BlendCandidate, dict[str, float]]] = []
    for mode in ["speed_only_partition", "speed_force_partition", "speed_force_partition_squared", "positive_launch_overlay"]:
        mode_rows = [
            (cand, metrics)
            for cand, metrics in scored
            if cand.mode == mode
            and next(
                row
                for row in candidate_rows
                if row["candidate"] == cand.name
                and row["mode"] == cand.mode
                and row["speed_knee_mps"] == cand.speed_knee_mps
                and row["util_knee"] == cand.util_knee
                and row["alpha"] == cand.alpha
            )["in_place_max_abs_command"]
            >= 0.6
        ]
        best_by_mode.append(min(mode_rows, key=lambda item: item[1]["validation_rb_corrected_rmse_nm"]))

    selected_pred = predict_opposes(frame, selected)
    selected_blend = blend_factor(frame, selected)
    split_rows = []
    for split in SPLITS:
        mask = frame["dataset_split"].to_numpy() == split
        subset = frame[mask]
        row = metric_row(split, subset, selected_pred[mask])
        row["mean_blend"] = float(np.mean(selected_blend[mask])) if len(subset) else math.nan
        split_rows.append(row)
    validation_mask = frame["dataset_split"].to_numpy() != "primary_open_floor_fit_authoritative"
    row = metric_row("validation_non_authoritative", frame[validation_mask], selected_pred[validation_mask])
    row["mean_blend"] = float(np.mean(selected_blend[validation_mask]))
    split_rows.append(row)

    selected_log_rows = []
    for run_id in SELECTED_RUNS:
        mask = frame["run_id"].to_numpy() == run_id
        subset = frame[mask]
        row = metric_row(run_id, subset, selected_pred[mask])
        row["run_id"] = run_id
        row["dataset_split"] = ";".join(sorted(str(x) for x in subset["dataset_split"].unique())) if len(subset) else ""
        row["mean_blend"] = float(np.mean(selected_blend[mask])) if len(subset) else math.nan
        selected_log_rows.append(row)

    candidate_rows.sort(
        key=lambda row: (
            0 if row.get("selected") else 1,
            float(row["validation_rb_corrected_rmse_nm"]) if row.get("validation_rb_corrected_rmse_nm") != "" else 999.0,
        )
    )
    write_csv(OUT / "candidate_scores.csv", candidate_rows)
    write_csv(OUT / "split_metrics.csv", split_rows)
    write_csv(OUT / "selected_log_metrics.csv", selected_log_rows)
    write_csv(OUT / "in_place_1radps_command.csv", in_place_rows)
    make_report(best_by_mode, selected, candidate_rows, split_rows, selected_log_rows, in_place_rows)
    (OUT / "commands_run.txt").write_text(
        "& 'C:\\Users\\thene\\.cache\\codex-runtimes\\codex-primary-runtime\\dependencies\\python\\python.exe' codex_analysis\\yaw_model_variant_fits\\transition_options\\rational_speed_force_blend\\fit_rational_speed_force_blend.py\n",
        encoding="utf-8",
    )


if __name__ == "__main__":
    main()
