#!/usr/bin/env python3
"""Narrow propagation-cost benchmark for standalone traction testbed models."""

from __future__ import annotations

import csv
import json
import statistics
import sys
import time
from dataclasses import asdict
from pathlib import Path
from typing import Any


SCRIPT_PATH = Path(__file__).resolve()
ROUND_DIR = SCRIPT_PATH.parents[1]
REPO_ROOT = SCRIPT_PATH.parents[4]
TOOL_ROOT = REPO_ROOT / "Tools" / "TractionRmsNisTestbed"
OUTPUT_DIR = SCRIPT_PATH.parent

sys.path.insert(0, str(TOOL_ROOT))

from traction_rms_nis_testbed.estimator_core import (  # noqa: E402
    CandidateConfig,
    CandidatePlant,
    ReplaySample,
    load_candidates,
    load_covariance,
)


CANDIDATES_JSON = REPO_ROOT / "staging" / "traction_candidate_rms_nis_testbed" / "candidates.json"
COVARIANCE_JSON = (
    REPO_ROOT / "staging" / "traction_candidate_rms_nis_testbed" / "covariance_conservative.json"
)

REQUESTED = (
    "baseline/current_holdover",
    "slip_envelope",
    "stribeck_fade",
    "skew_shear",
    "shear_rate",
    "in_shear",
)

PARAMETER_SOURCES = {
    "baseline/current_holdover": None,
    "slip_envelope": ROUND_DIR / "slip_envelope" / "tune_expanded" / "tuned_parameters.json",
    "stribeck_fade": ROUND_DIR / "stribeck_fade" / "tune" / "tuned_parameters.json",
    "skew_shear": ROUND_DIR / "skew_shear" / "tune_refined" / "tuned_parameters.json",
    "shear_rate": ROUND_DIR / "shear_rate" / "refined_tuning" / "tuned_parameters.json",
    "in_shear": ROUND_DIR / "in_shear" / "tune" / "tuned_parameters.json",
}

DEFAULT_ID_BY_REQUEST = {
    "baseline/current_holdover": "baseline/current_holdover",
    "slip_envelope": "candidate_1_algebraic_envelope",
    "stribeck_fade": "candidate_2_stribeck",
    "skew_shear": "skew_shear",
    "shear_rate": "shear_rate",
    "in_shear": "in_shear",
}

SAMPLE_SET_REPEATS = 1_500
REPEAT_COUNT = 9
WARMUP_LOOPS = 500


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def tuned_candidate(path: Path) -> CandidateConfig:
    payload = load_json(path)
    entries = payload.get("candidates", [])
    if len(entries) != 1:
        raise RuntimeError(f"Expected one tuned candidate in {path}, found {len(entries)}")
    raw = entries[0]
    candidate_id = str(raw.get("id", raw.get("candidate_id", "")))
    return CandidateConfig(
        candidate_id=candidate_id,
        label=candidate_id,
        model=str(raw["model"]),
        parameters={str(key): float(value) for key, value in dict(raw.get("parameters", {})).items()},
    )


def candidate_for_request(request_id: str) -> tuple[CandidateConfig, str]:
    source = PARAMETER_SOURCES[request_id]
    if source is not None and source.exists():
        return tuned_candidate(source), str(source)

    default_id = DEFAULT_ID_BY_REQUEST[request_id]
    for candidate in load_candidates(CANDIDATES_JSON):
        if candidate.candidate_id == default_id:
            return candidate, str(CANDIDATES_JSON)
    raise RuntimeError(f"No candidate config for {request_id}")


def sample(
    name: str,
    dt_s: float,
    left_command: float,
    right_command: float,
    left_rate: float,
    right_rate: float,
    yaw_rate: float,
    accel_f: float,
    accel_r: float,
    previous_left_rate: float | None,
    previous_right_rate: float | None,
    previous_yaw_rate: float | None,
) -> ReplaySample:
    return ReplaySample(
        source_path=Path(f"synthetic/{name}.csv"),
        source_row_index=0,
        master_time_us=0,
        dt_s=dt_s,
        left_command=left_command,
        right_command=right_command,
        left_wheel_rate_radps=left_rate,
        right_wheel_rate_radps=right_rate,
        yaw_rate_radps=yaw_rate,
        accel_forward_mps2=accel_f,
        accel_right_mps2=accel_r,
        accel_valid=True,
        gyro_valid=True,
        fan_duty_cycle=0.8,
        segment_id=name,
        stage=name,
        split="benchmark",
        run_id="synthetic_cost",
        previous_left_wheel_rate_radps=previous_left_rate,
        previous_right_wheel_rate_radps=previous_right_rate,
        previous_yaw_rate_radps=previous_yaw_rate,
    )


def representative_cases() -> list[tuple[str, list[float], ReplaySample]]:
    tick = 0.001
    return [
        (
            "static",
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            sample("static", tick, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        ),
        (
            "straight_launch",
            [0.002, 0.045, 0.0, 0.42, 0.015, 0.03, 0.06, -0.015, 0.9],
            sample("straight_launch", tick, 0.58, 0.58, 61.0, 61.0, 0.03, 5.8, 0.18, 54.0, 54.0, 0.02),
        ),
        (
            "mixed_launch",
            [0.006, 0.030, 0.020, 0.32, -0.045, 2.4, 0.10, -0.05, 3.0],
            sample("mixed_launch", tick, 0.36, 0.52, 47.0, 64.0, 2.4, 4.5, -1.2, 41.0, 55.0, 2.0),
        ),
        (
            "yaw_launch",
            [-0.004, 0.010, -0.040, 0.04, 0.02, 5.8, -0.05, 0.04, 8.0],
            sample("yaw_launch", tick, -0.68, 0.68, -78.0, 78.0, 5.8, 0.4, -2.8, -68.0, 68.0, 4.9),
        ),
    ]


def run_one(plant: CandidatePlant, cases: list[tuple[str, list[float], ReplaySample]]) -> tuple[float, float]:
    checksum = 0.0
    start = time.perf_counter_ns()
    for _ in range(SAMPLE_SET_REPEATS):
        for _, state, replay_sample in cases:
            result = plant.propagate(state, replay_sample, replay_sample.dt_s)
            checksum += result[0] + 0.5 * result[3] + 0.25 * result[5]
    elapsed_ns = time.perf_counter_ns() - start
    propagations = SAMPLE_SET_REPEATS * len(cases)
    return elapsed_ns / propagations, checksum


def main() -> int:
    vehicle, _covariance = load_covariance(COVARIANCE_JSON)
    cases = representative_cases()
    case_payload = [
        {"name": name, "state": state, "sample": asdict(replay_sample)}
        for name, state, replay_sample in cases
    ]

    raw_results = []
    for request_id in REQUESTED:
        candidate, parameter_source = candidate_for_request(request_id)
        plant = CandidatePlant(vehicle, candidate)
        for _ in range(WARMUP_LOOPS):
            for _, state, replay_sample in cases:
                plant.propagate(state, replay_sample, replay_sample.dt_s)

        repeat_ns = []
        checksum = 0.0
        for _ in range(REPEAT_COUNT):
            elapsed_ns, checksum = run_one(plant, cases)
            repeat_ns.append(elapsed_ns)

        raw_results.append(
            {
                "requested_model": request_id,
                "candidate_id": candidate.candidate_id,
                "plant_model": candidate.model,
                "parameter_source": parameter_source,
                "propagations_per_repeat": SAMPLE_SET_REPEATS * len(cases),
                "repeat_count": REPEAT_COUNT,
                "median_ns_per_propagation": statistics.median(repeat_ns),
                "mean_ns_per_propagation": statistics.fmean(repeat_ns),
                "stdev_ns_per_propagation": statistics.stdev(repeat_ns),
                "min_ns_per_propagation": min(repeat_ns),
                "max_ns_per_propagation": max(repeat_ns),
                "checksum": checksum,
                "parameters": candidate.parameters,
            }
        )

    cheapest = min(row["median_ns_per_propagation"] for row in raw_results)
    for row in raw_results:
        row["median_us_per_propagation"] = row["median_ns_per_propagation"] / 1_000.0
        row["relative_vs_cheapest"] = row["median_ns_per_propagation"] / cheapest

    csv_path = OUTPUT_DIR / "plant_propagation_cost.csv"
    json_path = OUTPUT_DIR / "plant_propagation_cost.json"
    md_path = OUTPUT_DIR / "plant_propagation_cost.md"

    fieldnames = [
        "requested_model",
        "candidate_id",
        "plant_model",
        "median_us_per_propagation",
        "relative_vs_cheapest",
        "mean_ns_per_propagation",
        "stdev_ns_per_propagation",
        "min_ns_per_propagation",
        "max_ns_per_propagation",
        "propagations_per_repeat",
        "repeat_count",
        "parameter_source",
        "checksum",
    ]
    with csv_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in raw_results:
            writer.writerow({key: row[key] for key in fieldnames})

    summary = {
        "schema_version": 1,
        "benchmark": "CandidatePlant.propagate direct-call wall-clock timing",
        "clock": "time.perf_counter_ns",
        "single_process_thread": True,
        "sample_source": "synthetic representative static/straight_launch/mixed_launch/yaw_launch cases",
        "sample_set_repeats": SAMPLE_SET_REPEATS,
        "repeat_count": REPEAT_COUNT,
        "warmup_loops": WARMUP_LOOPS,
        "cases": case_payload,
        "results": raw_results,
        "caveats": [
            "Python interpreter overhead and object allocation dominate absolute wall-clock values.",
            "Relative ordering is useful for this standalone Python testbed only; it is not an embedded C++ cycle estimate.",
            "The benchmark excludes CSV IO, full replay, EKF update, covariance propagation, and NIS scoring.",
        ],
    }
    json_path.write_text(json.dumps(summary, indent=2, sort_keys=True, default=str) + "\n", encoding="utf-8")

    lines = [
        "# Plant Propagation Cost",
        "",
        "Direct wall-clock timing of `CandidatePlant.propagate()` only. CSV IO, full replay, EKF update, covariance propagation, and NIS scoring are excluded.",
        "",
        "| Model | Plant model | us/prop median | Relative vs cheapest | Parameter source |",
        "|---|---:|---:|---:|---|",
    ]
    for row in raw_results:
        lines.append(
            "| {requested_model} | {plant_model} | {median_us_per_propagation:.3f} | {relative_vs_cheapest:.2f}x | {parameter_source} |".format(
                **row
            )
        )
    lines.extend(
        [
            "",
            "Caveats: Python interpreter overhead and list/dataclass object traffic dominate the absolute timings. Treat these results as a narrow standalone-testbed relative cost check, not as embedded C++ operation cost.",
            "",
            f"Iterations: {SAMPLE_SET_REPEATS * len(cases)} propagations/repeat x {REPEAT_COUNT} repeats after warmup.",
        ]
    )
    md_path.write_text("\n".join(lines) + "\n", encoding="utf-8")

    print(md_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
