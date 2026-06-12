#!/usr/bin/env python3
"""Prediction-only static propagation for the longest static traction segment."""

from __future__ import annotations

import csv
import json
import math
import statistics
import sys
from dataclasses import replace
from pathlib import Path
from typing import Any, Iterable


SCRIPT_DIR = Path(__file__).resolve().parent
ROUND_DIR = SCRIPT_DIR.parent
TESTBED_DIR = ROUND_DIR.parent
REPO_ROOT = TESTBED_DIR.parents[1]
TOOL_DIR = REPO_ROOT / "Tools" / "TractionRmsNisTestbed"
sys.path.insert(0, str(TOOL_DIR))

from traction_rms_nis_testbed.estimator_core import (  # noqa: E402
    HEADING,
    VF,
    VR,
    YAW_RATE,
    CandidatePlant,
    ReplaySample,
    diagnostic_row,
    format_number,
    load_candidates,
    load_covariance,
    measured_yaw_accel_from_previous,
    read_segment_samples,
    segment_specs_from_manifest,
)


SELECTED_MANIFEST = ROUND_DIR / "static_stability_analysis" / "selected_static_segment_manifest.json"
COMBINED_CONFIG = ROUND_DIR / "static_stability_analysis" / "combined_static_candidate_config.json"
COVARIANCE_CONFIG = TESTBED_DIR / "covariance_conservative.json"
FULL_EKF_METRICS = ROUND_DIR / "static_stability_analysis" / "static_stability_metrics.csv"
OUTPUT_ROWS = SCRIPT_DIR / "prediction_only_rows.csv"
OUTPUT_METRICS = SCRIPT_DIR / "static_encoder_ablation_metrics.csv"
OUTPUT_SUMMARY = SCRIPT_DIR / "static_encoder_ablation_summary.json"
OUTPUT_REPORT = SCRIPT_DIR / "static_encoder_ablation_report.md"

ROW_FIELDS = (
    "candidate_id",
    "segment_id",
    "source_row_index",
    "master_time_us",
    "dt_s",
    "vf_mps",
    "vr_mps",
    "yaw_rate_radps",
    "heading_rad",
    "predicted_forward_accel_mps2",
    "predicted_right_accel_mps2",
    "predicted_yaw_accel_radps2",
    "max_contact_relative_speed_mps",
    "max_contact_utilization",
    "max_contact_saturation",
    "ground_use",
)

METRIC_FIELDS = (
    "candidate_id",
    "samples",
    "duration_s",
    "finite",
    "bounded",
    "final_abs_vf_mps",
    "final_abs_vr_mps",
    "final_abs_yaw_rate_radps",
    "final_abs_heading_rad",
    "max_abs_vf_mps",
    "max_abs_vr_mps",
    "max_abs_yaw_rate_radps",
    "max_abs_heading_rad",
    "pred_forward_accel_mean_mps2",
    "pred_right_accel_mean_mps2",
    "pred_yaw_accel_mean_radps2",
    "pred_forward_accel_max_abs_mps2",
    "pred_right_accel_max_abs_mps2",
    "pred_yaw_accel_max_abs_radps2",
    "full_ekf_final_abs_vf_mps",
    "full_ekf_final_abs_vr_mps",
    "full_ekf_final_abs_yaw_rate_radps",
    "full_ekf_final_abs_heading_rad",
)


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def finite(value: float) -> bool:
    return math.isfinite(value)


def finite_mean(values: list[float]) -> float:
    return statistics.fmean(values) if values else math.nan


def max_abs(values: list[float]) -> float:
    return max((abs(value) for value in values), default=math.nan)


def zero_encoder_only_sample(sample: ReplaySample, previous: ReplaySample | None) -> ReplaySample:
    previous_yaw = 0.0 if previous is not None else None
    previous_wheel = 0.0 if previous is not None else None
    return replace(
        sample,
        left_command=0.0,
        right_command=0.0,
        left_wheel_rate_radps=0.0,
        right_wheel_rate_radps=0.0,
        yaw_rate_radps=0.0,
        accel_forward_mps2=math.nan,
        accel_right_mps2=math.nan,
        accel_valid=False,
        gyro_valid=False,
        previous_left_wheel_rate_radps=previous_wheel,
        previous_right_wheel_rate_radps=previous_wheel,
        previous_yaw_rate_radps=previous_yaw,
    )


def read_full_ekf_reference() -> dict[str, dict[str, str]]:
    if not FULL_EKF_METRICS.exists():
        return {}
    with FULL_EKF_METRICS.open(newline="", encoding="utf-8") as handle:
        return {row["candidate_id"]: row for row in csv.DictReader(handle)}


def build_metrics(
    candidate_id: str,
    rows: list[dict[str, float]],
    duration_s: float,
    full_ekf_reference: dict[str, dict[str, str]],
) -> dict[str, Any]:
    state_fields = {
        "vf_mps": [row["vf_mps"] for row in rows],
        "vr_mps": [row["vr_mps"] for row in rows],
        "yaw_rate_radps": [row["yaw_rate_radps"] for row in rows],
        "heading_rad": [row["heading_rad"] for row in rows],
    }
    pred_fields = {
        "predicted_forward_accel_mps2": [row["predicted_forward_accel_mps2"] for row in rows],
        "predicted_right_accel_mps2": [row["predicted_right_accel_mps2"] for row in rows],
        "predicted_yaw_accel_radps2": [row["predicted_yaw_accel_radps2"] for row in rows],
    }
    all_values = [value for values in [*state_fields.values(), *pred_fields.values()] for value in values]
    finite_all = all(finite(value) for value in all_values)
    final = rows[-1] if rows else {}
    reference = full_ekf_reference.get(candidate_id, {})
    metric = {
        "candidate_id": candidate_id,
        "samples": len(rows),
        "duration_s": duration_s,
        "finite": finite_all,
        "bounded": finite_all and max_abs(state_fields["vf_mps"]) < 1.0 and max_abs(state_fields["vr_mps"]) < 1.0 and max_abs(state_fields["yaw_rate_radps"]) < 1.0,
        "final_abs_vf_mps": abs(float(final.get("vf_mps", math.nan))),
        "final_abs_vr_mps": abs(float(final.get("vr_mps", math.nan))),
        "final_abs_yaw_rate_radps": abs(float(final.get("yaw_rate_radps", math.nan))),
        "final_abs_heading_rad": abs(float(final.get("heading_rad", math.nan))),
        "max_abs_vf_mps": max_abs(state_fields["vf_mps"]),
        "max_abs_vr_mps": max_abs(state_fields["vr_mps"]),
        "max_abs_yaw_rate_radps": max_abs(state_fields["yaw_rate_radps"]),
        "max_abs_heading_rad": max_abs(state_fields["heading_rad"]),
        "pred_forward_accel_mean_mps2": finite_mean(pred_fields["predicted_forward_accel_mps2"]),
        "pred_right_accel_mean_mps2": finite_mean(pred_fields["predicted_right_accel_mps2"]),
        "pred_yaw_accel_mean_radps2": finite_mean(pred_fields["predicted_yaw_accel_radps2"]),
        "pred_forward_accel_max_abs_mps2": max_abs(pred_fields["predicted_forward_accel_mps2"]),
        "pred_right_accel_max_abs_mps2": max_abs(pred_fields["predicted_right_accel_mps2"]),
        "pred_yaw_accel_max_abs_radps2": max_abs(pred_fields["predicted_yaw_accel_radps2"]),
        "full_ekf_final_abs_vf_mps": reference.get("final_abs_vf_mps", ""),
        "full_ekf_final_abs_vr_mps": reference.get("final_abs_vr_mps", ""),
        "full_ekf_final_abs_yaw_rate_radps": reference.get("final_abs_yaw_rate_radps", ""),
        "full_ekf_final_abs_heading_rad": reference.get("final_abs_heading_rad", ""),
    }
    return metric


def write_report(metrics: list[dict[str, Any]], segment: dict[str, Any]) -> None:
    lines = [
        "# Static Encoder Ablation",
        "",
        f"- Segment: `{segment.get('segment_id')}`",
        f"- Duration: `{segment.get('segment_duration_s')}` s",
        "- Inputs forced for ablation: zero left/right commands, zero left/right wheel rates, yaw/accel measurements invalid.",
        "- Propagation: candidate plant mean propagation only; no yaw-rate update, no accel updates, no logged UKF state, no encoder NIS.",
        f"- Candidate config: `{COMBINED_CONFIG}`",
        f"- Full-EKF reference metrics: `{FULL_EKF_METRICS}`",
        "",
        "| Model | Bounded | Final abs vf/vr/yaw_rate/yaw | Max abs vf/vr/yaw_rate/yaw | Pred accel mean f/r/yaw | Full-EKF final abs vf/vr/yaw_rate/yaw |",
        "| --- | --- | --- | --- | --- | --- |",
    ]
    for row in metrics:
        final = (
            f"{format_number(row['final_abs_vf_mps'])}/"
            f"{format_number(row['final_abs_vr_mps'])}/"
            f"{format_number(row['final_abs_yaw_rate_radps'])}/"
            f"{format_number(row['final_abs_heading_rad'])}"
        )
        maxes = (
            f"{format_number(row['max_abs_vf_mps'])}/"
            f"{format_number(row['max_abs_vr_mps'])}/"
            f"{format_number(row['max_abs_yaw_rate_radps'])}/"
            f"{format_number(row['max_abs_heading_rad'])}"
        )
        pred = (
            f"{format_number(row['pred_forward_accel_mean_mps2'])}/"
            f"{format_number(row['pred_right_accel_mean_mps2'])}/"
            f"{format_number(row['pred_yaw_accel_mean_radps2'])}"
        )
        reference = (
            f"{row['full_ekf_final_abs_vf_mps']}/"
            f"{row['full_ekf_final_abs_vr_mps']}/"
            f"{row['full_ekf_final_abs_yaw_rate_radps']}/"
            f"{row['full_ekf_final_abs_heading_rad']}"
        )
        lines.append(
            f"| `{row['candidate_id']}` | `{str(row['bounded']).lower()}` | "
            f"{final} | {maxes} | {pred} | {reference} |"
        )
    OUTPUT_REPORT.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_csv(path: Path, fieldnames: Iterable[str], rows: list[dict[str, Any]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=tuple(fieldnames))
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    SCRIPT_DIR.mkdir(parents=True, exist_ok=True)
    selected_manifest = load_json(SELECTED_MANIFEST)
    segment_payload = selected_manifest["segments"][0]
    segments = segment_specs_from_manifest(SELECTED_MANIFEST, REPO_ROOT)
    if len(segments) != 1:
        raise RuntimeError(f"Expected one selected static segment, got {len(segments)}")
    segment = segments[0]
    duration_s = float(segment_payload.get("segment_duration_s", 0.0) or 0.0)
    candidates = load_candidates(COMBINED_CONFIG)
    vehicle, _covariance = load_covariance(COVARIANCE_CONFIG)
    full_ekf_reference = read_full_ekf_reference()

    row_artifacts: list[dict[str, str]] = []
    metrics: list[dict[str, Any]] = []
    for candidate in candidates:
        plant = CandidatePlant(vehicle, candidate)
        state = [0.0 for _ in range(9)]
        previous_zeroed: ReplaySample | None = None
        previous_yaw_for_diagnostic: float | None = None
        numeric_rows: list[dict[str, float]] = []
        for sample in read_segment_samples(segment, vehicle, max_rows=0):
            zeroed = zero_encoder_only_sample(sample, previous_zeroed)
            state = plant.propagate(state, zeroed, zeroed.dt_s)
            plant_result = plant.plant_result(state, zeroed)
            measured_yaw_accel = measured_yaw_accel_from_previous(previous_yaw_for_diagnostic, zeroed)
            diagnostic = diagnostic_row(
                candidate,
                zeroed,
                state,
                plant_result,
                plant_result,
                measured_yaw_accel,
                vehicle,
            )
            artifact_row = {
                field: diagnostic[field]
                for field in ROW_FIELDS
            }
            row_artifacts.append(artifact_row)
            numeric_rows.append(
                {
                    "vf_mps": state[VF],
                    "vr_mps": state[VR],
                    "yaw_rate_radps": state[YAW_RATE],
                    "heading_rad": state[HEADING],
                    "predicted_forward_accel_mps2": plant_result.imu_forward_accel_mps2,
                    "predicted_right_accel_mps2": plant_result.imu_right_accel_mps2,
                    "predicted_yaw_accel_radps2": plant_result.yaw_accel_radps2,
                }
            )
            previous_zeroed = zeroed
            previous_yaw_for_diagnostic = zeroed.yaw_rate_radps if zeroed.gyro_valid else previous_yaw_for_diagnostic
        metrics.append(build_metrics(candidate.candidate_id, numeric_rows, duration_s, full_ekf_reference))

    write_csv(OUTPUT_ROWS, ROW_FIELDS, row_artifacts)
    write_csv(OUTPUT_METRICS, METRIC_FIELDS, metrics)
    summary = {
        "schema_version": 1,
        "segment": segment_payload,
        "candidate_config": str(COMBINED_CONFIG),
        "covariance_config": str(COVARIANCE_CONFIG),
        "full_ekf_reference_metrics": str(FULL_EKF_METRICS),
        "prediction_only_rows": str(OUTPUT_ROWS),
        "metrics_csv": str(OUTPUT_METRICS),
        "report_md": str(OUTPUT_REPORT),
        "uses_logged_ukf_state": False,
        "uses_measurement_updates": False,
        "uses_encoder_nis": False,
        "forced_inputs": {
            "left_command": 0.0,
            "right_command": 0.0,
            "left_wheel_rate_radps": 0.0,
            "right_wheel_rate_radps": 0.0,
            "gyro_valid": False,
            "accel_valid": False,
        },
        "models": {row["candidate_id"]: row for row in metrics},
    }
    OUTPUT_SUMMARY.write_text(
        json.dumps(summary, indent=2, sort_keys=True, default=str) + "\n",
        encoding="utf-8",
    )
    write_report(metrics, segment_payload)
    print(f"Wrote {OUTPUT_METRICS}")
    print(f"Wrote {OUTPUT_SUMMARY}")
    print(f"Wrote {OUTPUT_REPORT}")
    print(f"Wrote {OUTPUT_ROWS}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
