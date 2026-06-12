#!/usr/bin/env python3
"""One-log simplex UKF validation for the production-encoder-covariance shear_rate winner."""

from __future__ import annotations

import csv
import json
import math
import statistics
import sys
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
    HEADING,
    N,
    VF,
    VR,
    YAW_RATE,
    CandidatePlant,
    SimplexReplay,
    format_number,
    load_candidates,
    load_covariance,
    max_abs,
    segment_specs_from_manifest,
)
from traction_rms_nis_testbed.estimator_core import (  # noqa: E402
    EkfReplay,
    SourceLogSampleCache,
    segment_row_indices,
    segment_sample_key,
)


CANDIDATE_CONFIG = (
    TESTBED_DIR
    / "round_20260611_prod_encoder_cov"
    / "shear_rate"
    / "shear_rate_refined_tuned_only.json"
)
COVARIANCE_CONFIG = TESTBED_DIR / "covariance_conservative.json"
SEGMENT_MANIFEST = (
    TESTBED_DIR
    / "parallel_replace_20260420_102209_20260610"
    / "ukf_one_log_20260610_070622_active_manifest.json"
)
BIAS_SEGMENT_MANIFEST = TESTBED_DIR / "representative_corpus" / "segment_manifest.json"

OUTPUT_ROWS = SCRIPT_DIR / "simplex_ukf_rows.csv"
OUTPUT_METRICS = SCRIPT_DIR / "simplex_ukf_metrics.csv"
OUTPUT_SUMMARY = SCRIPT_DIR / "ukf_one_log_simplex_summary.json"
OUTPUT_REPORT = SCRIPT_DIR / "ukf_one_log_simplex_report.md"
OUTPUT_INPUTS = SCRIPT_DIR / "run_inputs.json"

ACTIVE_SIGMA_POINTS = N + 2
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
    "max_abs_sigma_state",
    "max_abs_sigma_prediction",
    "covariance_trace",
    "yaw_rate_nis",
    "forward_accel_nis",
    "right_accel_nis",
)
METRIC_FIELDS = (
    "candidate_id",
    "status",
    "segments",
    "samples",
    "active_sigma_points",
    "prediction_sigma_points",
    "measurement_update_count",
    "finite",
    "sigma_nonfinite_count",
    "factorization_failure_count",
    "max_abs_state",
    "max_abs_prediction",
    "max_abs_sigma_state",
    "max_abs_sigma_prediction",
    "max_covariance_trace",
    "max_yaw_rate_nis",
    "max_forward_accel_nis",
    "max_right_accel_nis",
    "yaw_rate_rms_nis",
    "forward_accel_rms_nis",
    "right_accel_rms_nis",
)


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def finite(value: float) -> bool:
    return math.isfinite(value)


def finite_values(rows: list[dict[str, Any]], field: str) -> list[float]:
    return [float(row[field]) for row in rows if finite(float(row[field]))]


def rms(values: list[float]) -> float:
    return math.sqrt(statistics.fmean(value * value for value in values)) if values else math.nan


def write_csv(path: Path, fieldnames: Iterable[str], rows: list[dict[str, Any]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=tuple(fieldnames))
        writer.writeheader()
        writer.writerows(rows)


class CachedNoiseSimplexReplay(SimplexReplay):
    def __init__(self, plant: CandidatePlant, covariance_config: Any):
        super().__init__(plant, covariance_config)
        self._noise_replay = EkfReplay(plant, covariance_config)

    def process_noise_like(self, state: list[float], sample: Any) -> list[list[float]]:
        return self._noise_replay.process_noise(state, sample, sample.dt_s)


def build_metric(candidate_id: str, rows: list[dict[str, Any]], replay: CachedNoiseSimplexReplay, segment_count: int) -> dict[str, Any]:
    state_fields = ("vf_mps", "vr_mps", "yaw_rate_radps", "heading_rad")
    prediction_fields = (
        "predicted_forward_accel_mps2",
        "predicted_right_accel_mps2",
        "predicted_yaw_accel_radps2",
    )
    state_values = [float(row[field]) for row in rows for field in state_fields]
    prediction_values = [float(row[field]) for row in rows for field in prediction_fields]
    finite_all = all(finite(value) for value in (*state_values, *prediction_values))
    max_covariance_trace = max_abs(float(row["covariance_trace"]) for row in rows)
    status = (
        "pass"
        if finite_all
        and replay.sigma_nonfinite_count == 0
        and max_abs(state_values) < 1.0e6
        and max_abs(prediction_values) < 1.0e7
        and max_covariance_trace < 1.0e10
        else "fail"
    )
    measurement_update_count = sum(
        1
        for row in rows
        for field in ("yaw_rate_nis", "forward_accel_nis", "right_accel_nis")
        if finite(float(row[field]))
    )
    return {
        "candidate_id": candidate_id,
        "status": status,
        "segments": segment_count,
        "samples": len(rows),
        "active_sigma_points": ACTIVE_SIGMA_POINTS,
        "prediction_sigma_points": len(rows) * ACTIVE_SIGMA_POINTS,
        "measurement_update_count": measurement_update_count,
        "finite": finite_all,
        "sigma_nonfinite_count": replay.sigma_nonfinite_count,
        "factorization_failure_count": replay.factorization_failure_count,
        "max_abs_state": max_abs(state_values),
        "max_abs_prediction": max_abs(prediction_values),
        "max_abs_sigma_state": max_abs(float(row["max_abs_sigma_state"]) for row in rows),
        "max_abs_sigma_prediction": max_abs(float(row["max_abs_sigma_prediction"]) for row in rows),
        "max_covariance_trace": max_covariance_trace,
        "max_yaw_rate_nis": max_abs(finite_values(rows, "yaw_rate_nis")),
        "max_forward_accel_nis": max_abs(finite_values(rows, "forward_accel_nis")),
        "max_right_accel_nis": max_abs(finite_values(rows, "right_accel_nis")),
        "yaw_rate_rms_nis": rms(finite_values(rows, "yaw_rate_nis")),
        "forward_accel_rms_nis": rms(finite_values(rows, "forward_accel_nis")),
        "right_accel_rms_nis": rms(finite_values(rows, "right_accel_nis")),
    }


def write_report(summary: dict[str, Any]) -> None:
    metric = next(iter(summary["candidates"].values()))
    lines = [
        "# One-Log Simplex UKF Validation",
        "",
        f"- Candidate: `{metric['candidate_id']}`",
        f"- Status: `{metric['status']}`",
        f"- Segment manifest: `{summary['segment_manifest']}`",
        f"- Selected log: `{summary['selected_log']}`",
        f"- Segments: `{metric['segments']}`",
        f"- Samples: `{metric['samples']}`",
        "- Sigma policy: production simplex geometry, 11 active sigma points for 9 states.",
        "- Measurement policy: production yaw-rate and planar-accel streams only; no encoder NIS.",
        "- Uses logged UKF state: `false`",
        f"- Candidate config: `{summary['candidate_config']}`",
        f"- Covariance config: `{summary['covariance_config']}`",
        "",
        "| Metric | Value |",
        "| --- | --- |",
    ]
    for field in METRIC_FIELDS:
        lines.append(f"| `{field}` | `{format_number(metric[field]) if isinstance(metric[field], float) else metric[field]}` |")
    lines.extend(
        [
            "",
            "## Artifacts",
            "",
            f"- Rows: `{summary['row_csv']}`",
            f"- Metrics CSV: `{summary['metrics_csv']}`",
            f"- Summary JSON: `{OUTPUT_SUMMARY}`",
            f"- Inputs JSON: `{OUTPUT_INPUTS}`",
        ]
    )
    OUTPUT_REPORT.write_text("\n".join(lines) + "\n", encoding="utf-8")


def load_samples_by_segment(segments: list[Any], vehicle: Any) -> dict[Any, list[Any]]:
    cache = SourceLogSampleCache()
    by_log: dict[Path, list[Any]] = {}
    for segment in segments:
        by_log.setdefault(segment.log_path, []).append(segment)

    samples_by_segment: dict[Any, list[Any]] = {}
    for log_path, log_segments in by_log.items():
        index = cache.index_for_log(log_path)
        targets = {
            segment_sample_key(segment): segment_row_indices(segment, 0, len(index.row_offsets))
            for segment in log_segments
        }
        loaded = cache.read_targeted_segment_samples(log_path, log_segments, targets, vehicle)
        for segment in log_segments:
            samples_by_segment[segment_sample_key(segment)] = loaded[segment_sample_key(segment)]
    return samples_by_segment


def main() -> int:
    SCRIPT_DIR.mkdir(parents=True, exist_ok=True)
    manifest_payload = load_json(SEGMENT_MANIFEST)
    segments = segment_specs_from_manifest(SEGMENT_MANIFEST, REPO_ROOT)
    candidates = load_candidates(CANDIDATE_CONFIG)
    if len(candidates) != 1 or candidates[0].candidate_id != "shear_rate":
        raise RuntimeError("Expected exactly the shear_rate candidate")
    vehicle, covariance = load_covariance(COVARIANCE_CONFIG)
    plant = CandidatePlant(vehicle, candidates[0])
    replay = CachedNoiseSimplexReplay(plant, covariance)
    samples_by_segment = load_samples_by_segment(segments, vehicle)

    row_artifacts: list[dict[str, Any]] = []
    numeric_rows: list[dict[str, Any]] = []
    for segment in segments:
        for sample in samples_by_segment[segment_sample_key(segment)]:
            plant_before_update, max_sigma_state, max_sigma_prediction, covariance_trace = replay.predict(sample)
            del plant_before_update
            nis_values = replay.update_production_measurements(sample)
            plant_after_update = plant.plant_result(replay.state, sample)
            row = {
                "candidate_id": candidates[0].candidate_id,
                "segment_id": sample.segment_id,
                "source_row_index": sample.source_row_index,
                "master_time_us": sample.master_time_us,
                "dt_s": sample.dt_s,
                "vf_mps": replay.state[VF],
                "vr_mps": replay.state[VR],
                "yaw_rate_radps": replay.state[YAW_RATE],
                "heading_rad": replay.state[HEADING],
                "predicted_forward_accel_mps2": plant_after_update.imu_forward_accel_mps2,
                "predicted_right_accel_mps2": plant_after_update.imu_right_accel_mps2,
                "predicted_yaw_accel_radps2": plant_after_update.yaw_accel_radps2,
                "max_abs_sigma_state": max_sigma_state,
                "max_abs_sigma_prediction": max_sigma_prediction,
                "covariance_trace": covariance_trace,
                "yaw_rate_nis": nis_values.get("yaw_rate_nis", math.nan),
                "forward_accel_nis": nis_values.get("forward_accel_nis", math.nan),
                "right_accel_nis": nis_values.get("right_accel_nis", math.nan),
            }
            numeric_rows.append(row)
            row_artifacts.append(
                {key: format_number(value) if isinstance(value, float) else value for key, value in row.items()}
            )

    metric = build_metric(candidates[0].candidate_id, numeric_rows, replay, len(segments))
    metric_csv = {key: format_number(value) if isinstance(value, float) else value for key, value in metric.items()}
    selected_log = manifest_payload.get("newest_log_selected") or (
        manifest_payload.get("segments", [{}])[0].get("log_id")
    )
    summary = {
        "schema_version": 1,
        "validation_kind": "simplex_one_log_candidate_plant",
        "candidate_config": str(CANDIDATE_CONFIG),
        "covariance_config": str(COVARIANCE_CONFIG),
        "segment_manifest": str(SEGMENT_MANIFEST),
        "bias_segment_manifest": str(BIAS_SEGMENT_MANIFEST),
        "selected_log": selected_log,
        "source_log_count": len({segment.get("log_id") for segment in manifest_payload.get("segments", [])}),
        "processed_segments": len(segments),
        "processed_samples": len(numeric_rows),
        "passed": metric["status"] == "pass",
        "uses_logged_ukf_state": False,
        "uses_encoder_nis": False,
        "sigma_policy": "production simplex geometry, N+2 active sigma points",
        "active_sigma_points": ACTIVE_SIGMA_POINTS,
        "measurement_policy": "production yaw-rate and planar-accel measurement streams only",
        "thresholds": {
            "state_abs_limit": 1.0e6,
            "prediction_abs_limit": 1.0e7,
            "covariance_trace_limit": 1.0e10,
        },
        "row_csv": str(OUTPUT_ROWS),
        "metrics_csv": str(OUTPUT_METRICS),
        "report_md": str(OUTPUT_REPORT),
        "inputs_json": str(OUTPUT_INPUTS),
        "candidates": {metric["candidate_id"]: metric},
    }
    inputs = {
        "candidate_config": str(CANDIDATE_CONFIG),
        "covariance_config": str(COVARIANCE_CONFIG),
        "segment_manifest": str(SEGMENT_MANIFEST),
        "bias_segment_manifest": str(BIAS_SEGMENT_MANIFEST),
        "timeout_policy": "runner command timeout set above 90 minutes",
        "jobs": "not applicable; runner is single-process and exposes no parallelism option",
    }
    write_csv(OUTPUT_ROWS, ROW_FIELDS, row_artifacts)
    write_csv(OUTPUT_METRICS, METRIC_FIELDS, [metric_csv])
    OUTPUT_SUMMARY.write_text(json.dumps(summary, indent=2, sort_keys=True, default=str) + "\n", encoding="utf-8")
    OUTPUT_INPUTS.write_text(json.dumps(inputs, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    write_report(summary)
    print(f"Wrote {OUTPUT_METRICS}")
    print(f"Wrote {OUTPUT_SUMMARY}")
    print(f"Wrote {OUTPUT_REPORT}")
    print(f"Wrote {OUTPUT_ROWS}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
