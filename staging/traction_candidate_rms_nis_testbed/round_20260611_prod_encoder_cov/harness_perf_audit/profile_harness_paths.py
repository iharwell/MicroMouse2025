#!/usr/bin/env python3
"""Profile representative standalone traction/UKF harness paths.

This is an audit-only script. It imports the testbed implementation unchanged
and writes small profiling artifacts beside this file.
"""

from __future__ import annotations

import cProfile
import csv
import io
import json
import pstats
import statistics
import sys
import time
from pathlib import Path
from typing import Any, Callable


SCRIPT_DIR = Path(__file__).resolve().parent
ROUND_DIR = SCRIPT_DIR.parent
REPO_ROOT = SCRIPT_DIR.parents[3]
TOOL_DIR = REPO_ROOT / "Tools" / "TractionRmsNisTestbed"
STATIC_SIMPLEX_DIR = REPO_ROOT / "staging" / "traction_candidate_rms_nis_testbed" / "round_20260611" / "static_simplex_ukf_analysis"

sys.path.insert(0, str(TOOL_DIR))
sys.path.insert(0, str(STATIC_SIMPLEX_DIR))

from traction_rms_nis_testbed.estimator_core import (  # noqa: E402
    CandidatePlant,
    EkfReplay,
    format_number,
    load_candidates,
    load_covariance,
    read_segment_samples,
    segment_specs_from_manifest,
)
from run_static_simplex_ukf_analysis import SimplexReplay, max_abs  # noqa: E402


CANDIDATE_CONFIG = ROUND_DIR / "shear_rate" / "shear_rate_refined_tuned_only.json"
COVARIANCE_CONFIG = REPO_ROOT / "staging" / "traction_candidate_rms_nis_testbed" / "covariance_conservative.json"
SEGMENT_MANIFEST = (
    REPO_ROOT
    / "staging"
    / "traction_candidate_rms_nis_testbed"
    / "parallel_replace_20260420_102209_20260610"
    / "ukf_one_log_20260610_070622_active_manifest.json"
)

PROFILE_ROWS = 128
TIME_REPEATS = 5
PROPAGATE_LOOPS = 600


class CachedNoiseSimplexReplay(SimplexReplay):
    def __init__(self, plant: CandidatePlant, covariance_config: Any):
        super().__init__(plant, covariance_config)
        self._noise_replay = EkfReplay(plant, covariance_config)

    def process_noise_like(self, state: list[float], sample: Any) -> list[list[float]]:
        return self._noise_replay.process_noise(state, sample, sample.dt_s)


def load_fixture() -> tuple[Any, Any, CandidatePlant, list[Any]]:
    candidates = load_candidates(CANDIDATE_CONFIG)
    if len(candidates) != 1:
        raise RuntimeError(f"Expected one candidate in {CANDIDATE_CONFIG}")
    vehicle, covariance = load_covariance(COVARIANCE_CONFIG)
    plant = CandidatePlant(vehicle, candidates[0])
    segments = segment_specs_from_manifest(SEGMENT_MANIFEST, REPO_ROOT)
    samples: list[Any] = []
    for segment in segments:
        samples.extend(read_segment_samples(segment, vehicle, max_rows=PROFILE_ROWS - len(samples)))
        if len(samples) >= PROFILE_ROWS:
            break
    if not samples:
        raise RuntimeError("No samples loaded")
    return vehicle, covariance, plant, samples[:PROFILE_ROWS]


def elapsed_seconds(function: Callable[[], Any]) -> float:
    start = time.perf_counter()
    function()
    return time.perf_counter() - start


def repeated_timing(name: str, function: Callable[[], Any], units: int) -> dict[str, Any]:
    values = [elapsed_seconds(function) for _ in range(TIME_REPEATS)]
    per_unit = [value / units for value in values]
    return {
        "name": name,
        "repeats": TIME_REPEATS,
        "units": units,
        "median_s": statistics.median(values),
        "mean_s": statistics.fmean(values),
        "min_s": min(values),
        "max_s": max(values),
        "median_us_per_unit": statistics.median(per_unit) * 1_000_000.0,
        "mean_us_per_unit": statistics.fmean(per_unit) * 1_000_000.0,
    }


def profile_text(name: str, function: Callable[[], Any]) -> str:
    profiler = cProfile.Profile()
    profiler.enable()
    function()
    profiler.disable()
    output = io.StringIO()
    stats = pstats.Stats(profiler, stream=output).strip_dirs().sort_stats("cumtime")
    stats.print_stats(35)
    path = SCRIPT_DIR / f"{name}.pstats"
    stats.dump_stats(str(path))
    return output.getvalue()


def run_propagate_loop(plant: CandidatePlant, samples: list[Any]) -> float:
    state = [0.006, 0.030, 0.020, 0.32, -0.045, 2.4, 0.10, -0.05, 3.0]
    checksum = 0.0
    for _ in range(PROPAGATE_LOOPS):
        for sample in samples[:4]:
            state = plant.propagate(state, sample, sample.dt_s)
            checksum += state[0] + 0.5 * state[3] + 0.25 * state[5]
    return checksum


def run_plant_result_loop(plant: CandidatePlant, samples: list[Any]) -> float:
    state = [0.006, 0.030, 0.020, 0.32, -0.045, 2.4, 0.10, -0.05, 3.0]
    checksum = 0.0
    for _ in range(PROPAGATE_LOOPS * 2):
        for sample in samples[:4]:
            result = plant.plant_result(state, sample)
            checksum += result.imu_forward_accel_mps2 + 0.1 * result.yaw_accel_radps2
    return checksum


def run_simplex_loop(plant: CandidatePlant, covariance: Any, samples: list[Any]) -> float:
    replay = CachedNoiseSimplexReplay(plant, covariance)
    checksum = 0.0
    for sample in samples:
        plant_result, max_sigma_state, max_sigma_prediction, covariance_trace = replay.predict(sample)
        nis_values = replay.update_production_measurements(sample)
        after = plant.plant_result(replay.state, sample)
        checksum += (
            plant_result.imu_forward_accel_mps2
            + after.imu_right_accel_mps2
            + max_sigma_state
            + max_sigma_prediction
            + covariance_trace
            + sum(value for value in nis_values.values() if value == value)
        )
    return checksum


def run_row_format_loop(plant: CandidatePlant, covariance: Any, samples: list[Any]) -> list[dict[str, Any]]:
    replay = CachedNoiseSimplexReplay(plant, covariance)
    rows: list[dict[str, Any]] = []
    for sample in samples:
        _, max_sigma_state, max_sigma_prediction, covariance_trace = replay.predict(sample)
        nis_values = replay.update_production_measurements(sample)
        after = plant.plant_result(replay.state, sample)
        row = {
            "source_row_index": sample.source_row_index,
            "master_time_us": sample.master_time_us,
            "dt_s": sample.dt_s,
            "vf_mps": replay.state[3],
            "vr_mps": replay.state[4],
            "yaw_rate_radps": replay.state[5],
            "heading_rad": replay.state[2],
            "predicted_forward_accel_mps2": after.imu_forward_accel_mps2,
            "predicted_right_accel_mps2": after.imu_right_accel_mps2,
            "predicted_yaw_accel_radps2": after.yaw_accel_radps2,
            "max_abs_sigma_state": max_sigma_state,
            "max_abs_sigma_prediction": max_sigma_prediction,
            "covariance_trace": covariance_trace,
            "max_abs_state": max_abs(replay.state),
            "yaw_rate_nis": nis_values.get("yaw_rate_nis", float("nan")),
            "forward_accel_nis": nis_values.get("forward_accel_nis", float("nan")),
            "right_accel_nis": nis_values.get("right_accel_nis", float("nan")),
        }
        rows.append({key: format_number(value) if isinstance(value, float) else value for key, value in row.items()})
    return rows


def run_csv_write(rows: list[dict[str, Any]]) -> None:
    with (SCRIPT_DIR / "row_write_probe.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=tuple(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def run_sample_load(vehicle: Any) -> int:
    segments = segment_specs_from_manifest(SEGMENT_MANIFEST, REPO_ROOT)
    loaded = 0
    for segment in segments[:4]:
        loaded += len(list(read_segment_samples(segment, vehicle, max_rows=PROFILE_ROWS // 4)))
    return loaded


def main() -> int:
    SCRIPT_DIR.mkdir(parents=True, exist_ok=True)
    vehicle, covariance, plant, samples = load_fixture()
    for _ in range(3):
        run_propagate_loop(plant, samples)
        run_simplex_loop(plant, covariance, samples[:8])

    row_probe = run_row_format_loop(plant, covariance, samples[:32])
    measurements = [
        repeated_timing("direct_propagate", lambda: run_propagate_loop(plant, samples), PROPAGATE_LOOPS * 4),
        repeated_timing("plant_result_only", lambda: run_plant_result_loop(plant, samples), PROPAGATE_LOOPS * 2 * 4),
        repeated_timing("simplex_predict_update", lambda: run_simplex_loop(plant, covariance, samples[:32]), 32),
        repeated_timing("row_format_after_simplex", lambda: run_row_format_loop(plant, covariance, samples[:32]), 32),
        repeated_timing("csv_write_32_rows", lambda: run_csv_write(row_probe), len(row_probe)),
        repeated_timing("csv_sample_load", lambda: run_sample_load(vehicle), PROFILE_ROWS),
    ]

    profiles = {
        "direct_propagate": profile_text("direct_propagate", lambda: run_propagate_loop(plant, samples)),
        "simplex_predict_update": profile_text("simplex_predict_update", lambda: run_simplex_loop(plant, covariance, samples[:32])),
        "row_format_after_simplex": profile_text("row_format_after_simplex", lambda: run_row_format_loop(plant, covariance, samples[:32])),
        "csv_sample_load": profile_text("csv_sample_load", lambda: run_sample_load(vehicle)),
    }

    summary = {
        "schema_version": 1,
        "profile_rows": PROFILE_ROWS,
        "time_repeats": TIME_REPEATS,
        "propagate_loops": PROPAGATE_LOOPS,
        "candidate_config": str(CANDIDATE_CONFIG),
        "covariance_config": str(COVARIANCE_CONFIG),
        "segment_manifest": str(SEGMENT_MANIFEST),
        "sample_count_loaded_for_fixture": len(samples),
        "measurements": measurements,
        "profile_files": {
            key: str(SCRIPT_DIR / f"{key}.pstats")
            for key in profiles
        },
    }
    (SCRIPT_DIR / "profile_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    with (SCRIPT_DIR / "profile_measurements.csv").open("w", newline="", encoding="utf-8") as handle:
        fieldnames = tuple(measurements[0].keys())
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(measurements)

    lines = [
        "# Harness Performance Profile",
        "",
        "| Path | Median us/unit | Mean us/unit | Units | Median total s |",
        "| --- | ---: | ---: | ---: | ---: |",
    ]
    for row in measurements:
        lines.append(
            f"| `{row['name']}` | {row['median_us_per_unit']:.3f} | "
            f"{row['mean_us_per_unit']:.3f} | {row['units']} | {row['median_s']:.6f} |"
        )
    lines.append("")
    lines.append("## cProfile Hot Spots")
    for name, text in profiles.items():
        lines.append("")
        lines.append(f"### {name}")
        lines.append("```")
        lines.extend(text.splitlines()[:42])
        lines.append("```")
    (SCRIPT_DIR / "profile_report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")

    print(SCRIPT_DIR / "profile_report.md")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
