#!/usr/bin/env python3
"""Benchmark old per-segment CSV loading against grouped cached loading."""

from __future__ import annotations

import csv
import json
import statistics
import sys
import time
from pathlib import Path
from typing import Any, Callable


SCRIPT_DIR = Path(__file__).resolve().parent
ROUND_DIR = SCRIPT_DIR.parent
REPO_ROOT = SCRIPT_DIR.parents[3]
TOOL_DIR = REPO_ROOT / "Tools" / "TractionRmsNisTestbed"
sys.path.insert(0, str(TOOL_DIR))

from traction_rms_nis_testbed.estimator_core import (  # noqa: E402
    SourceLogSampleCache,
    load_covariance,
    read_segment_samples,
    segment_row_indices,
    segment_sample_key,
    segment_specs_from_manifest,
)


COVARIANCE_CONFIG = REPO_ROOT / "staging" / "traction_candidate_rms_nis_testbed" / "covariance_conservative.json"
SEGMENT_MANIFEST = (
    REPO_ROOT
    / "staging"
    / "traction_candidate_rms_nis_testbed"
    / "parallel_replace_20260420_102209_20260610"
    / "ukf_one_log_20260610_070622_active_manifest.json"
)
SEGMENT_COUNT = 4
MAX_ROWS_PER_SEGMENT = 32
TIME_REPEATS = 9


def elapsed_seconds(function: Callable[[], Any]) -> float:
    start = time.perf_counter()
    function()
    return time.perf_counter() - start


def repeated_timing(name: str, function: Callable[[], int], units: int) -> dict[str, Any]:
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
        "median_us_per_sample": statistics.median(per_unit) * 1_000_000.0,
        "mean_us_per_sample": statistics.fmean(per_unit) * 1_000_000.0,
    }


def load_old(segments: list[Any], vehicle: Any) -> int:
    loaded = 0
    for segment in segments:
        loaded += len(list(read_segment_samples(segment, vehicle, MAX_ROWS_PER_SEGMENT)))
    return loaded


def load_grouped_cached(
    segments: list[Any],
    vehicle: Any,
    cache: SourceLogSampleCache | None = None,
) -> tuple[int, SourceLogSampleCache]:
    cache = cache or SourceLogSampleCache()
    loaded = 0
    by_log: dict[Path, list[Any]] = {}
    for segment in segments:
        by_log.setdefault(segment.log_path, []).append(segment)
    for log_path, log_segments in by_log.items():
        targets = {
            segment_sample_key(segment): segment_row_indices(segment, MAX_ROWS_PER_SEGMENT)
            for segment in log_segments
        }
        grouped = cache.read_targeted_segment_samples(log_path, log_segments, targets, vehicle)
        loaded += sum(len(samples) for samples in grouped.values())
    return loaded, cache


def exact_signature(samples: list[Any]) -> list[tuple[Any, ...]]:
    return [
        (
            sample.segment_id,
            sample.source_row_index,
            sample.master_time_us,
            sample.dt_s,
            sample.left_command,
            sample.right_command,
            sample.left_wheel_rate_radps,
            sample.right_wheel_rate_radps,
            sample.yaw_rate_radps,
            sample.accel_forward_mps2,
            sample.accel_right_mps2,
            sample.previous_left_wheel_rate_radps,
            sample.previous_right_wheel_rate_radps,
            sample.previous_yaw_rate_radps,
        )
        for sample in samples
    ]


def verify_equivalence(segments: list[Any], vehicle: Any) -> None:
    old_samples = []
    for segment in segments:
        old_samples.extend(read_segment_samples(segment, vehicle, MAX_ROWS_PER_SEGMENT))
    cache = SourceLogSampleCache()
    new_samples = []
    for log_path in sorted({segment.log_path for segment in segments}, key=str):
        log_segments = [segment for segment in segments if segment.log_path == log_path]
        targets = {
            segment_sample_key(segment): segment_row_indices(segment, MAX_ROWS_PER_SEGMENT)
            for segment in log_segments
        }
        loaded = cache.read_targeted_segment_samples(log_path, log_segments, targets, vehicle)
        for segment in log_segments:
            new_samples.extend(loaded[segment_sample_key(segment)])
    if exact_signature(old_samples) != exact_signature(new_samples):
        raise RuntimeError("Grouped cached loader did not match old per-segment output")


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    vehicle, _covariance = load_covariance(COVARIANCE_CONFIG)
    segments = segment_specs_from_manifest(SEGMENT_MANIFEST, REPO_ROOT)[:SEGMENT_COUNT]
    verify_equivalence(segments, vehicle)
    sample_count = SEGMENT_COUNT * MAX_ROWS_PER_SEGMENT

    old_measurement = repeated_timing(
        "old_read_segment_samples_per_segment",
        lambda: load_old(segments, vehicle),
        sample_count,
    )
    cached_measurement = repeated_timing(
        "grouped_source_log_cold_cached_offsets",
        lambda: load_grouped_cached(segments, vehicle)[0],
        sample_count,
    )
    warm_cache = SourceLogSampleCache()
    load_grouped_cached(segments, vehicle, warm_cache)
    warm_measurement = repeated_timing(
        "grouped_source_log_warm_cached_offsets",
        lambda: load_grouped_cached(segments, vehicle, warm_cache)[0],
        sample_count,
    )
    loaded, cache = load_grouped_cached(segments, vehicle)
    summary = {
        "schema_version": 1,
        "segment_manifest": str(SEGMENT_MANIFEST.relative_to(REPO_ROOT)),
        "segment_count": SEGMENT_COUNT,
        "max_rows_per_segment": MAX_ROWS_PER_SEGMENT,
        "loaded_samples": loaded,
        "source_logs": len({segment.log_path for segment in segments}),
        "cached_index_build_count": cache.index_build_count,
        "cached_indexed_row_count": cache.indexed_row_count,
        "cached_file_read_count": cache.file_read_count,
        "measurements": [old_measurement, cached_measurement, warm_measurement],
    }

    summary_path = SCRIPT_DIR / "segment_loading_benchmark_summary.json"
    csv_path = SCRIPT_DIR / "segment_loading_benchmark_measurements.csv"
    report_path = SCRIPT_DIR / "segment_loading_benchmark_report.md"
    summary_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    write_csv(csv_path, summary["measurements"])
    old_us = old_measurement["median_us_per_sample"]
    cached_us = cached_measurement["median_us_per_sample"]
    warm_us = warm_measurement["median_us_per_sample"]
    speedup = old_us / cached_us if cached_us > 0.0 else float("inf")
    warm_speedup = old_us / warm_us if warm_us > 0.0 else float("inf")
    report_path.write_text(
        "\n".join(
            [
                "# Segment Loading Benchmark",
                "",
                f"- Manifest: `{summary['segment_manifest']}`",
                f"- Segments: `{SEGMENT_COUNT}`",
                f"- Samples: `{sample_count}`",
                f"- Source logs: `{summary['source_logs']}`",
                f"- Old median: `{old_us:.3f} us/sample`",
                f"- Grouped cold-cache median: `{cached_us:.3f} us/sample`",
                f"- Grouped warm-cache median: `{warm_us:.3f} us/sample`",
                f"- Cold-cache speedup: `{speedup:.2f}x`",
                f"- Warm-cache speedup: `{warm_speedup:.2f}x`",
                f"- Cached index builds: `{cache.index_build_count}`",
                f"- Cached file reads: `{cache.file_read_count}`",
                "",
                "The grouped cached path was verified against the old per-segment sample sequence before timing.",
                "",
            ]
        ),
        encoding="utf-8",
    )
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
