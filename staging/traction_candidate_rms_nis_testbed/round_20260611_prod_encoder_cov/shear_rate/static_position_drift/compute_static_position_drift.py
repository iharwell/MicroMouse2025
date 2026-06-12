#!/usr/bin/env python3
"""Compute static position drift for corrected-covariance shear_rate replay."""

from __future__ import annotations

import csv
import json
import math
import sys
from dataclasses import replace
from pathlib import Path
from typing import Any, Iterable


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[4]
TESTBED_DIR = REPO_ROOT / "staging" / "traction_candidate_rms_nis_testbed"
TOOL_DIR = REPO_ROOT / "Tools" / "TractionRmsNisTestbed"
STATIC_SIMPLEX_DIR = TESTBED_DIR / "round_20260611" / "static_simplex_ukf_analysis"

sys.path.insert(0, str(TOOL_DIR))
sys.path.insert(0, str(STATIC_SIMPLEX_DIR))

from run_static_simplex_ukf_analysis import (  # noqa: E402
    SimplexReplay,
    format_number,
    segment_specs_from_manifest,
    zero_encoder_only_sample,
)
from traction_rms_nis_testbed.estimator_core import (  # noqa: E402
    HEADING,
    PX,
    PY,
    VF,
    VR,
    YAW_RATE,
    CandidatePlant,
    EkfReplay,
    ReplaySample,
    load_candidates,
    load_covariance,
    read_segment_samples,
)


CANDIDATE_CONFIG = SCRIPT_DIR.parent / "shear_rate_refined_tuned_only.json"
COVARIANCE_CONFIG = TESTBED_DIR / "covariance_conservative.json"
STATIC_MANIFEST = (
    TESTBED_DIR
    / "round_20260611"
    / "static_stability_analysis"
    / "selected_static_segment_manifest.json"
)

OUTPUT_ROWS = SCRIPT_DIR / "static_position_drift_rows.csv"
OUTPUT_METRICS = SCRIPT_DIR / "static_position_drift_metrics.csv"
OUTPUT_SUMMARY = SCRIPT_DIR / "static_position_drift_summary.json"
OUTPUT_REPORT = SCRIPT_DIR / "static_position_drift_report.md"
OUTPUT_INPUTS = SCRIPT_DIR / "run_inputs.json"

PASS_THRESHOLD_M = 0.005

ROW_FIELDS = (
    "estimator",
    "case_id",
    "candidate_id",
    "segment_id",
    "source_row_index",
    "master_time_us",
    "dt_s",
    "px_m",
    "py_m",
    "radial_m",
    "heading_rad",
    "vf_mps",
    "vr_mps",
    "yaw_rate_radps",
    "yaw_rate_nis",
    "forward_accel_nis",
    "right_accel_nis",
)

METRIC_FIELDS = (
    "estimator",
    "case_id",
    "candidate_id",
    "segment_id",
    "samples",
    "duration_s",
    "final_x_m",
    "final_y_m",
    "final_radial_m",
    "max_radial_m",
    "yaw_drift_rad",
    "yaw_drift_deg",
    "final_vf_mps",
    "final_vr_mps",
    "final_yaw_rate_radps",
    "pass_threshold_m",
    "passes_5mm",
    "finite",
    "sigma_nonfinite_count",
    "factorization_failure_count",
)


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def finite(value: float) -> bool:
    return math.isfinite(value)


def write_csv(path: Path, fieldnames: Iterable[str], rows: list[dict[str, Any]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=tuple(fieldnames))
        writer.writeheader()
        writer.writerows(rows)


def zero_measurement_sample(sample: ReplaySample, previous: ReplaySample | None) -> ReplaySample:
    return zero_encoder_only_sample(sample, previous)


def update_ekf(replay: EkfReplay, sample: ReplaySample) -> dict[str, float]:
    values: dict[str, float] = {}
    for result in (
        replay.update_yaw_rate(sample),
        replay.update_accel_forward(sample),
        replay.update_accel_right(sample),
    ):
        if result is not None:
            values[result.log_parameter] = result.nis
    return values


def row_from_state(
    estimator: str,
    case_id: str,
    candidate_id: str,
    sample: ReplaySample,
    state: list[float],
    nis_values: dict[str, float],
) -> dict[str, Any]:
    px = float(state[PX])
    py = float(state[PY])
    return {
        "estimator": estimator,
        "case_id": case_id,
        "candidate_id": candidate_id,
        "segment_id": sample.segment_id,
        "source_row_index": sample.source_row_index,
        "master_time_us": sample.master_time_us,
        "dt_s": sample.dt_s,
        "px_m": px,
        "py_m": py,
        "radial_m": math.hypot(px, py),
        "heading_rad": float(state[HEADING]),
        "vf_mps": float(state[VF]),
        "vr_mps": float(state[VR]),
        "yaw_rate_radps": float(state[YAW_RATE]),
        "yaw_rate_nis": nis_values.get("yaw_rate_nis", math.nan),
        "forward_accel_nis": nis_values.get("forward_accel_nis", math.nan),
        "right_accel_nis": nis_values.get("right_accel_nis", math.nan),
    }


def metric_from_rows(
    estimator: str,
    case_id: str,
    candidate_id: str,
    segment_id: str,
    rows: list[dict[str, Any]],
    duration_s: float,
    sigma_nonfinite_count: int | None = None,
    factorization_failure_count: int | None = None,
) -> dict[str, Any]:
    final = rows[-1]
    numeric_fields = ("px_m", "py_m", "radial_m", "heading_rad", "vf_mps", "vr_mps", "yaw_rate_radps")
    finite_all = all(finite(float(row[field])) for row in rows for field in numeric_fields)
    final_radial = float(final["radial_m"])
    yaw_drift = float(final["heading_rad"])
    return {
        "estimator": estimator,
        "case_id": case_id,
        "candidate_id": candidate_id,
        "segment_id": segment_id,
        "samples": len(rows),
        "duration_s": duration_s,
        "final_x_m": float(final["px_m"]),
        "final_y_m": float(final["py_m"]),
        "final_radial_m": final_radial,
        "max_radial_m": max(float(row["radial_m"]) for row in rows),
        "yaw_drift_rad": yaw_drift,
        "yaw_drift_deg": math.degrees(yaw_drift),
        "final_vf_mps": float(final["vf_mps"]),
        "final_vr_mps": float(final["vr_mps"]),
        "final_yaw_rate_radps": float(final["yaw_rate_radps"]),
        "pass_threshold_m": PASS_THRESHOLD_M,
        "passes_5mm": finite_all and final_radial <= PASS_THRESHOLD_M and max(float(row["radial_m"]) for row in rows) <= PASS_THRESHOLD_M,
        "finite": finite_all,
        "sigma_nonfinite_count": "" if sigma_nonfinite_count is None else sigma_nonfinite_count,
        "factorization_failure_count": "" if factorization_failure_count is None else factorization_failure_count,
    }


def run_simplex(
    candidate: Any,
    vehicle: Any,
    covariance: Any,
    samples: list[ReplaySample],
    case_id: str,
    duration_s: float,
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    plant = CandidatePlant(vehicle, candidate)
    replay = SimplexReplay(plant, covariance)
    previous_zeroed: ReplaySample | None = None
    rows: list[dict[str, Any]] = []
    for sample in samples:
        replay_sample = zero_measurement_sample(sample, previous_zeroed) if case_id == "prediction_only_encoder_only" else sample
        replay.predict(replay_sample)
        nis_values = replay.update_production_measurements(replay_sample) if case_id == "full_static_replay" else {}
        rows.append(row_from_state("simplex_ukf", case_id, candidate.candidate_id, replay_sample, replay.state, nis_values))
        previous_zeroed = replay_sample
    metric = metric_from_rows(
        "simplex_ukf",
        case_id,
        candidate.candidate_id,
        samples[0].segment_id,
        rows,
        duration_s,
        replay.sigma_nonfinite_count,
        replay.factorization_failure_count,
    )
    return rows, metric


def run_ekf(
    candidate: Any,
    vehicle: Any,
    covariance: Any,
    samples: list[ReplaySample],
    case_id: str,
    duration_s: float,
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    plant = CandidatePlant(vehicle, candidate)
    replay = EkfReplay(plant, covariance)
    previous_zeroed: ReplaySample | None = None
    rows: list[dict[str, Any]] = []
    for sample in samples:
        replay_sample = zero_measurement_sample(sample, previous_zeroed) if case_id == "prediction_only_encoder_only" else sample
        replay.predict(replay_sample)
        nis_values = update_ekf(replay, replay_sample) if case_id == "full_static_replay" else {}
        rows.append(row_from_state("ekf", case_id, candidate.candidate_id, replay_sample, replay.state, nis_values))
        previous_zeroed = replay_sample
    metric = metric_from_rows(
        "ekf",
        case_id,
        candidate.candidate_id,
        samples[0].segment_id,
        rows,
        duration_s,
    )
    return rows, metric


def write_report(metrics: list[dict[str, Any]]) -> None:
    lines = [
        "# Static Position Drift",
        "",
        f"- Candidate config: `{CANDIDATE_CONFIG}`",
        f"- Covariance config: `{COVARIANCE_CONFIG}`",
        f"- Static manifest: `{STATIC_MANIFEST}`",
        "- Segment: `ofnis_001617`, longest prior static segment; zero commands and zero wheel rates.",
        "- No logged UKF state and no encoder NIS are consumed.",
        f"- Pass threshold: `{PASS_THRESHOLD_M}` m on both final and max radial drift.",
        "",
        "| Estimator | Case | Final x/y (m) | Final radial (m) | Max radial (m) | Yaw drift (rad/deg) | Pass 5 mm |",
        "| --- | --- | ---: | ---: | ---: | ---: | --- |",
    ]
    for row in metrics:
        lines.append(
            f"| `{row['estimator']}` | `{row['case_id']}` | "
            f"{format_number(row['final_x_m'])}/{format_number(row['final_y_m'])} | "
            f"{format_number(row['final_radial_m'])} | "
            f"{format_number(row['max_radial_m'])} | "
            f"{format_number(row['yaw_drift_rad'])}/{format_number(row['yaw_drift_deg'])} | "
            f"`{str(row['passes_5mm']).lower()}` |"
        )
    lines.extend(
        [
            "",
            "## Artifacts",
            "",
            f"- Rows: `{OUTPUT_ROWS}`",
            f"- Metrics: `{OUTPUT_METRICS}`",
            f"- Summary: `{OUTPUT_SUMMARY}`",
            f"- Inputs: `{OUTPUT_INPUTS}`",
        ]
    )
    OUTPUT_REPORT.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    SCRIPT_DIR.mkdir(parents=True, exist_ok=True)
    manifest = load_json(STATIC_MANIFEST)
    segment_payload = manifest["segments"][0]
    if segment_payload.get("segment_id") != "ofnis_001617":
        raise RuntimeError(f"Expected ofnis_001617, got {segment_payload.get('segment_id')}")
    segments = segment_specs_from_manifest(STATIC_MANIFEST, REPO_ROOT)
    if len(segments) != 1:
        raise RuntimeError(f"Expected one static segment, got {len(segments)}")

    vehicle, covariance = load_covariance(COVARIANCE_CONFIG)
    candidates = load_candidates(CANDIDATE_CONFIG)
    if len(candidates) != 1 or candidates[0].candidate_id != "shear_rate":
        raise RuntimeError("Expected exactly the corrected-round shear_rate candidate")
    samples = list(read_segment_samples(segments[0], vehicle, max_rows=0))
    duration_s = float(segment_payload.get("segment_duration_s", 0.0) or sum(sample.dt_s for sample in samples))

    all_rows: list[dict[str, Any]] = []
    metrics: list[dict[str, Any]] = []
    for runner in (run_simplex, run_ekf):
        for case_id in ("full_static_replay", "prediction_only_encoder_only"):
            rows, metric = runner(candidates[0], vehicle, covariance, samples, case_id, duration_s)
            all_rows.extend(rows)
            metrics.append(metric)

    write_csv(
        OUTPUT_ROWS,
        ROW_FIELDS,
        [
            {
                key: format_number(value) if isinstance(value, float) else value
                for key, value in row.items()
            }
            for row in all_rows
        ],
    )
    write_csv(
        OUTPUT_METRICS,
        METRIC_FIELDS,
        [
            {
                key: format_number(value) if isinstance(value, float) else value
                for key, value in row.items()
            }
            for row in metrics
        ],
    )
    summary = {
        "schema_version": 1,
        "candidate_config": str(CANDIDATE_CONFIG),
        "covariance_config": str(COVARIANCE_CONFIG),
        "static_manifest": str(STATIC_MANIFEST),
        "segment": segment_payload,
        "samples": len(samples),
        "duration_s_from_samples": sum(sample.dt_s for sample in samples),
        "duration_s": duration_s,
        "pass_threshold_m": PASS_THRESHOLD_M,
        "uses_logged_ukf_state": False,
        "uses_encoder_nis": False,
        "cases": {
            "full_static_replay": "logged zero commands/wheel rates with yaw and accel updates when valid",
            "prediction_only_encoder_only": "commands/wheel rates forced to zero; yaw/accel updates disabled",
        },
        "row_csv": str(OUTPUT_ROWS),
        "metrics_csv": str(OUTPUT_METRICS),
        "report_md": str(OUTPUT_REPORT),
        "inputs_json": str(OUTPUT_INPUTS),
        "metrics": {f"{row['estimator']}:{row['case_id']}": row for row in metrics},
    }
    OUTPUT_SUMMARY.write_text(json.dumps(summary, indent=2, sort_keys=True, default=str) + "\n", encoding="utf-8")
    OUTPUT_INPUTS.write_text(
        json.dumps(
            {
                "candidate_config": str(CANDIDATE_CONFIG),
                "covariance_config": str(COVARIANCE_CONFIG),
                "static_manifest": str(STATIC_MANIFEST),
                "timeout_policy": "runner command timeout set above 90 minutes",
                "jobs": "not applicable; single-process script with no parallelism option",
            },
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    write_report(metrics)
    print(f"Wrote {OUTPUT_METRICS}")
    print(f"Wrote {OUTPUT_SUMMARY}")
    print(f"Wrote {OUTPUT_REPORT}")
    print(f"Wrote {OUTPUT_ROWS}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
