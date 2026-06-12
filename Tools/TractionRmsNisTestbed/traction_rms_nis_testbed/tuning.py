"""Bounded standalone tuning for traction residual-tail candidate parameters.

This module intentionally uses only the standalone testbed data layer and
estimator core. It does not import production replay code or consume logged UKF
state columns.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import random
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable, Iterator, Sequence

from .data_layer import SegmentStream
from .estimator_core import (
    AccelBiasEstimate,
    CandidateConfig as EstimatorCandidateConfig,
    CandidatePlant,
    N,
    PRODUCTION_MEASUREMENT_RESIDUAL_TAIL_LOG_PARAMETERS,
    ReplaySample,
    SegmentSpec,
    SourceLogSampleCache,
    VehicleConfig,
    finite,
    finite_or,
    estimate_accel_bias_for_log,
    is_accel_bias_assessment_label,
    invalid_residual_tail_updates,
    load_covariance,
    replace_sample_accel_bias,
    residual_tail_updates,
    segment_sample_key,
)
from .scoring import (
    COMMAND_BIN_WIDTH,
    SPLITS,
    canonical_log_field,
    expected_rms_for_dimension,
    format_number,
    json_safe,
    launch_command_signature,
    stable_split,
)


REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_CONFIG_PATH = REPO_ROOT / "staging" / "traction_candidate_rms_nis_testbed" / "config.example.json"
DEFAULT_CANDIDATE_PATH = REPO_ROOT / "staging" / "traction_candidate_rms_nis_testbed" / "candidates.json"
DEFAULT_COVARIANCE_PATH = (
    REPO_ROOT / "staging" / "traction_candidate_rms_nis_testbed" / "covariance_conservative.json"
)
DEFAULT_OUTPUT_DIR = REPO_ROOT / "staging" / "traction_candidate_rms_nis_testbed" / "broad_tuning_latest"
TARGET_CANDIDATES = (
    "baseline/current_holdover",
    "candidate_1_algebraic_envelope",
    "candidate_2_stribeck",
    "candidate_3_load_sensitive",
    "skew_shear",
    "shear_rate",
    "in_shear",
)
FALSE_TEXT_VALUES = ("0", "false", "no")
FORBIDDEN_TUNING_NAME_TOKENS = (
    "covariance",
    "measurement_noise",
    "process_noise",
    "noise_scale",
    "nis_scale",
    "r_scale",
    "q_scale",
    "sigma",
)
DEFAULT_ACTIVE_METRIC_WEIGHTS = {
    "launch_active_pulse": 1.0,
    "yaw_launch": 1.0,
    "yaw_calibration": 1.0,
    "mixed_launch": 1.0,
    "straight_active": 1.0,
    "smooth_turn_active": 1.0,
}
PRODUCTION_MEASUREMENT_RESIDUAL_TAIL_LOG_FIELDS = frozenset(
    PRODUCTION_MEASUREMENT_RESIDUAL_TAIL_LOG_PARAMETERS
)


class TuningError(RuntimeError):
    """Raised when standalone tuning inputs or outputs are unusable."""


@dataclass(frozen=True)
class CandidateSpec:
    candidate_id: str
    label: str
    model: str
    parameters: dict[str, float]
    search: dict[str, dict[str, Any]]
    fixed: bool = False


@dataclass(frozen=True)
class TrialSpec:
    candidate_id: str
    trial_id: str
    model: str
    parameters: dict[str, float]
    is_nominal: bool
    is_fixed: bool = False

    @property
    def key(self) -> tuple[str, str]:
        return self.candidate_id, self.trial_id

    def estimator_candidate(self) -> EstimatorCandidateConfig:
        return EstimatorCandidateConfig(
            candidate_id=self.candidate_id,
            label=self.trial_id,
            model=self.model,
            parameters=self.parameters,
        )


@dataclass(frozen=True)
class SegmentMeta:
    segment_id: str
    stage: str
    family: str
    log_path: str
    log_id: str
    row_count: int
    start_row_index: int = 0
    end_row_index: int = -1
    active_start_row_index: int | None = None
    active_end_row_index: int | None = None
    corrupted: bool = False
    parameter_fields: dict[str, Any] = field(default_factory=dict)
    observed_command: dict[str, Any] = field(default_factory=dict)
    split: str = ""
    coverage_key: str = ""


@dataclass
class BucketStats:
    count: int = 0
    finite_count: int = 0
    accepted_count: int = 0
    nis_square_sum: float = 0.0
    accepted_nis_square_sum: float = 0.0
    rejected_count: int = 0

    def add(self, nis: float, accepted: bool) -> None:
        self.count += 1
        if finite(nis):
            self.finite_count += 1
            self.nis_square_sum += nis * nis
        if accepted:
            self.accepted_count += 1
            self.accepted_nis_square_sum += nis * nis
        else:
            self.rejected_count += 1

    @property
    def rms_nis(self) -> float:
        if self.finite_count <= 0:
            return math.nan
        return math.sqrt(self.nis_square_sum / self.finite_count)

    @property
    def accepted_only_rms_nis(self) -> float:
        if self.accepted_count <= 0:
            return math.nan
        return math.sqrt(self.accepted_nis_square_sum / self.accepted_count)

    @property
    def rejected_rate(self) -> float:
        if self.count <= 0:
            return 0.0
        return self.rejected_count / self.count


@dataclass(frozen=True)
class ScoreRow:
    phase: str
    candidate_id: str
    trial_id: str
    split: str
    stage: str
    log_parameter: str
    count: int
    accepted_count: int
    finite_count: int
    nonfinite_count: int
    rms_nis: float
    accepted_only_rms_nis: float
    expected_rms_nis: float
    guarded_rms_nis: float
    under_expected_penalty: float
    inflation_flag: bool
    rejected_count: int
    rejected_rate: float


@dataclass(frozen=True)
class SelectionScore:
    phase: str
    candidate_id: str
    trial_id: str
    selection_score: float
    sample_count: int


@dataclass(frozen=True)
class BootstrapRow:
    candidate_id: str
    trial_id: str
    baseline_candidate_id: str
    baseline_trial_id: str
    split: str
    source_log_count: int
    bootstrap_iterations: int
    observed_delta_vs_baseline: float
    ci_low: float
    ci_high: float
    probability_candidate_better: float


@dataclass
class EvaluationResult:
    phase: str
    sample_count: int = 0
    segment_count: int = 0
    detail: dict[tuple[str, str, str, str, str, str], BucketStats] = field(default_factory=dict)
    report: dict[tuple[str, str, str, str, str], BucketStats] = field(default_factory=dict)
    source_log_detail: dict[tuple[str, str, str, str, str, str, str], BucketStats] = field(default_factory=dict)

    def add(
        self,
        trial: TrialSpec,
        split: str,
        stage: str,
        source_log: str,
        log_parameter: str,
        coverage_key: str,
        nis: float,
        accepted: bool,
    ) -> None:
        log_field = canonical_log_field(log_parameter)
        detail_key = (trial.candidate_id, trial.trial_id, split, stage, log_field, coverage_key)
        report_key = (trial.candidate_id, trial.trial_id, split, stage, log_field)
        source_key = (trial.candidate_id, trial.trial_id, split, source_log, stage, log_field, coverage_key)
        self.detail.setdefault(detail_key, BucketStats()).add(nis, accepted)
        self.report.setdefault(report_key, BucketStats()).add(nis, accepted)
        self.source_log_detail.setdefault(source_key, BucketStats()).add(nis, accepted)


class EvaluationSampleSource:
    """Batch-load and cache replay samples for repeated tuning phases."""

    def __init__(
        self,
        manifest_path: Path,
        vehicle: VehicleConfig,
        bias_segments: Sequence["SegmentMeta"] | None = None,
    ):
        self.manifest_path = manifest_path
        self.vehicle = vehicle
        self._bias_segments = list(bias_segments) if bias_segments is not None else None
        self._accel_bias_by_log: dict[Path, AccelBiasEstimate] = {}
        self._cache: dict[tuple[str, str, int], list[ReplaySample]] = {}
        self._source_log_cache = SourceLogSampleCache()
        self.source_log_scan_count = 0
        self._reported_source_log_file_count = 0
        self._reported_source_log_index_build_count = 0
        self._reported_source_log_indexed_rows = 0
        self.streamed_segment_count = 0
        self.cached_segment_count = 0
        self.cache_hit_count = 0
        self.cache_miss_count = 0
        self.cached_sample_count = 0
        self.loaded_sample_count = 0

    def iter_segment_samples(
        self,
        segment_metas: Sequence[SegmentMeta],
        max_rows_per_segment: int,
        metric_scope: str,
    ) -> Iterator[tuple[SegmentMeta, list[ReplaySample]]]:
        if not segment_metas:
            return
        cacheable = metric_scope == "active" or max_rows_per_segment > 0
        if not cacheable:
            yield from self._stream_uncached(segment_metas, max_rows_per_segment, metric_scope)
            return

        missing: dict[str, SegmentMeta] = {}
        for meta in segment_metas:
            key = self._cache_key(meta, max_rows_per_segment, metric_scope)
            if key in self._cache:
                self.cache_hit_count += 1
            else:
                self.cache_miss_count += 1
                missing[meta.segment_id] = meta

        if missing:
            self.source_log_scan_count += 1
            loaded = self._load_targeted_samples(
                list(missing.values()),
                max_rows_per_segment,
                metric_scope,
            )
            for segment_id, samples in loaded.items():
                meta = missing[segment_id]
                key = self._cache_key(meta, max_rows_per_segment, metric_scope)
                self._cache[key] = samples
                self.cached_segment_count += 1
                self.cached_sample_count += len(samples)

        for meta in segment_metas:
            yield meta, self._cache[self._cache_key(meta, max_rows_per_segment, metric_scope)]

    def _stream_uncached(
        self,
        segment_metas: Sequence[SegmentMeta],
        max_rows_per_segment: int,
        metric_scope: str,
    ) -> Iterator[tuple[SegmentMeta, list[ReplaySample]]]:
        self.source_log_scan_count += 1
        loaded = self._load_targeted_samples(
            segment_metas,
            max_rows_per_segment,
            metric_scope,
        )
        for meta in segment_metas:
            self.streamed_segment_count += 1
            yield meta, loaded.get(meta.segment_id, [])

    def _load_targeted_samples(
        self,
        segment_metas: Sequence[SegmentMeta],
        max_rows_per_segment: int,
        metric_scope: str,
    ) -> dict[str, list[ReplaySample]]:
        samples_by_segment: dict[str, list[ReplaySample]] = {
            meta.segment_id: [] for meta in segment_metas
        }
        specs_by_segment = {
            meta.segment_id: segment_spec_from_meta(meta) for meta in segment_metas
        }
        metas_by_log: dict[Path, list[SegmentMeta]] = {}
        targets_by_log: dict[Path, dict[tuple[Path, str, int, int], list[int]]] = {}
        expected_counts: dict[str, int] = {}
        for meta in segment_metas:
            row_indices = target_row_indices(meta, max_rows_per_segment, metric_scope)
            expected_counts[meta.segment_id] = len(row_indices)
            if not row_indices:
                continue
            spec = specs_by_segment[meta.segment_id]
            key = segment_sample_key(spec)
            metas_by_log.setdefault(spec.log_path, []).append(meta)
            targets_by_log.setdefault(spec.log_path, {})[key] = row_indices

        for log_path, segment_targets in sorted(targets_by_log.items(), key=lambda item: str(item[0])):
            accel_bias = self._accel_bias_for_log(log_path)
            log_metas = metas_by_log.get(log_path, [])
            log_specs = [specs_by_segment[meta.segment_id] for meta in log_metas]
            loaded = self._source_log_cache.read_targeted_segment_samples(
                log_path,
                log_specs,
                segment_targets,
                self.vehicle,
            )
            for meta in log_metas:
                spec = specs_by_segment[meta.segment_id]
                samples = [
                    replace_sample_accel_bias(sample, accel_bias)
                    for sample in loaded.get(segment_sample_key(spec), [])
                ]
                samples_by_segment[meta.segment_id].extend(samples)
                self.loaded_sample_count += len(samples)

        missing = [
            segment_id
            for segment_id, expected_count in expected_counts.items()
            if expected_count != len(samples_by_segment.get(segment_id, []))
        ]
        if missing:
            raise TuningError(
                "Segment target rows not found before end of log: "
                + ", ".join(sorted(missing))
            )
        self._reported_source_log_file_count = self._source_log_cache.file_read_count
        self._reported_source_log_index_build_count = self._source_log_cache.index_build_count
        self._reported_source_log_indexed_rows = self._source_log_cache.indexed_row_count
        return samples_by_segment

    def _accel_bias_for_log(self, log_path: Path) -> AccelBiasEstimate:
        cached = self._accel_bias_by_log.get(log_path)
        if cached is not None:
            return cached
        bias_specs: list[SegmentSpec] = []
        for meta in self._all_bias_segments():
            spec = segment_spec_from_meta(meta)
            if spec.log_path != log_path:
                continue
            if meta.corrupted:
                continue
            if not is_accel_bias_assessment_label(meta.stage, meta.parameter_fields, meta.family):
                continue
            bias_specs.append(spec)
        estimate = estimate_accel_bias_for_log(log_path, bias_specs, self.vehicle)
        self._accel_bias_by_log[log_path] = estimate
        return estimate

    def _all_bias_segments(self) -> list["SegmentMeta"]:
        if self._bias_segments is None:
            self._bias_segments = load_manifest_segments(self.manifest_path, {})
        return self._bias_segments

    def _cache_key(
        self,
        meta: SegmentMeta,
        max_rows_per_segment: int,
        metric_scope: str,
    ) -> tuple[str, str, int]:
        return (meta.segment_id, metric_scope, max_rows_per_segment)

    def stats(self) -> dict[str, int]:
        return {
            "source_log_scan_batches": self.source_log_scan_count,
            "source_log_files_read": self._reported_source_log_file_count,
            "source_log_indexes_built": self._reported_source_log_index_build_count,
            "source_log_indexed_rows": self._reported_source_log_indexed_rows,
            "streamed_uncached_segments": self.streamed_segment_count,
            "cached_segments": self.cached_segment_count,
            "cache_hits": self.cache_hit_count,
            "cache_misses": self.cache_miss_count,
            "cached_samples": self.cached_sample_count,
            "loaded_samples": self.loaded_sample_count,
        }


def load_json(path: Path) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise TuningError(f"Invalid JSON in {path}: {exc}") from exc
    if not isinstance(payload, dict):
        raise TuningError(f"JSON root must be an object: {path}")
    return payload


def resolve_path(path_text: str, base_dir: Path) -> Path:
    path = Path(path_text)
    return path if path.is_absolute() else (base_dir / path).resolve()


def resolve_existing_config_path(path_text: str, config_dir: Path) -> Path:
    path = Path(path_text)
    if path.is_absolute():
        return path
    config_relative = (config_dir / path).resolve()
    if config_relative.exists():
        return config_relative
    return (REPO_ROOT / path).resolve()


def load_candidate_specs(config: dict[str, Any], candidate_config: dict[str, Any]) -> list[CandidateSpec]:
    defaults = candidate_defaults(candidate_config)
    configured = {
        str(raw.get("id", raw.get("candidate_id", ""))).strip(): raw
        for raw in config.get("candidates", [])
        if isinstance(raw, dict)
    }
    specs: list[CandidateSpec] = []
    for candidate_id in configured_candidate_targets(config):
        default = defaults.get(candidate_id)
        if default is None:
            raise TuningError(f"Candidate config missing required candidate: {candidate_id}")
        raw = configured.get(candidate_id, {})
        default_params = dict(default.get("parameters", {}))
        configured_params = dict(raw.get("parameters", {}))
        parameters = configured_params if configured_params else default_params
        search = normalize_search(dict(raw.get("search", {})))
        validate_no_covariance_tuning(candidate_id, parameters, search)
        specs.append(
            CandidateSpec(
                candidate_id=candidate_id,
                label=str(raw.get("label", default.get("label", candidate_id))),
                model=str(raw.get("model", default.get("model", ""))),
                parameters={str(key): float(value) for key, value in parameters.items()},
                search=search,
                fixed=candidate_id == "baseline/current_holdover",
            )
        )
    return specs


def candidate_defaults(candidate_config: dict[str, Any]) -> dict[str, dict[str, Any]]:
    raw_candidates = candidate_config.get("candidates", [])
    if not isinstance(raw_candidates, list):
        raise TuningError("Candidate config must contain a candidates list")
    result: dict[str, dict[str, Any]] = {}
    for raw in raw_candidates:
        if not isinstance(raw, dict):
            continue
        if str(raw.get("enabled", True)).strip().lower() in FALSE_TEXT_VALUES:
            continue
        candidate_id = str(raw.get("id", raw.get("candidate_id", ""))).strip()
        if candidate_id:
            result[candidate_id] = raw
    return result


def configured_candidate_targets(config: dict[str, Any]) -> list[str]:
    raw_candidates = config.get("candidates", [])
    if not isinstance(raw_candidates, list) or not raw_candidates:
        return list(TARGET_CANDIDATES)
    targets: list[str] = []
    seen: set[str] = set()
    for raw in raw_candidates:
        if not isinstance(raw, dict):
            continue
        candidate_id = str(raw.get("id", raw.get("candidate_id", ""))).strip()
        if candidate_id not in TARGET_CANDIDATES or candidate_id in seen:
            continue
        if str(raw.get("enabled", True)).strip().lower() in FALSE_TEXT_VALUES:
            continue
        targets.append(candidate_id)
        seen.add(candidate_id)
    if not targets:
        raise TuningError("Tuning config must enable at least one known candidate")
    return targets


def normalize_search(raw: dict[str, Any]) -> dict[str, dict[str, Any]]:
    result: dict[str, dict[str, Any]] = {}
    for name, bounds in raw.items():
        if not isinstance(bounds, dict):
            continue
        result[str(name)] = {
            "min": float(bounds.get("min", 0.0)),
            "max": float(bounds.get("max", bounds.get("min", 0.0))),
            "scale": str(bounds.get("scale", "linear")),
        }
    return result


def validate_no_covariance_tuning(
    candidate_id: str,
    parameters: dict[str, Any],
    search: dict[str, Any],
) -> None:
    for name in list(flatten_leaf_names(parameters)) + list(flatten_leaf_names(search)):
        lowered = name.lower()
        if any(token in lowered for token in FORBIDDEN_TUNING_NAME_TOKENS):
            raise TuningError(f"Refusing covariance/noise tuning field: {candidate_id}.{name}")


def flatten_leaf_names(value: Any, prefix: str = "") -> Iterator[str]:
    if isinstance(value, dict):
        for key, child in value.items():
            child_prefix = f"{prefix}.{key}" if prefix else str(key)
            yield from flatten_leaf_names(child, child_prefix)
    elif isinstance(value, list):
        for index, child in enumerate(value):
            child_prefix = f"{prefix}[{index}]"
            yield from flatten_leaf_names(child, child_prefix)
    elif prefix:
        yield prefix


def generate_trials(
    candidates: Sequence[CandidateSpec],
    trial_count: int,
    seed: str,
    include_nominal: bool,
) -> list[TrialSpec]:
    trials: list[TrialSpec] = []
    for candidate in candidates:
        if candidate.fixed:
            trials.append(
                TrialSpec(
                    candidate_id=candidate.candidate_id,
                    trial_id=f"{candidate.candidate_id}:fixed",
                    model=candidate.model,
                    parameters=dict(candidate.parameters),
                    is_nominal=True,
                    is_fixed=True,
                )
            )
            continue
        if include_nominal:
            trials.append(
                TrialSpec(
                    candidate_id=candidate.candidate_id,
                    trial_id=f"{candidate.candidate_id}:nominal",
                    model=candidate.model,
                    parameters=dict(candidate.parameters),
                    is_nominal=True,
                )
            )
        trials.extend(latin_hypercube_trials(candidate, trial_count, seed))
    return trials


def latin_hypercube_trials(candidate: CandidateSpec, trial_count: int, seed: str) -> list[TrialSpec]:
    if trial_count <= 0 or not candidate.search:
        return []
    rng = random.Random(f"{seed}:{candidate.candidate_id}")
    names = sorted(candidate.search)
    fractions_by_name: dict[str, list[float]] = {}
    for name in names:
        fractions = [(index + rng.random()) / trial_count for index in range(trial_count)]
        rng.shuffle(fractions)
        fractions_by_name[name] = fractions
    trials: list[TrialSpec] = []
    for index in range(trial_count):
        params = dict(candidate.parameters)
        for name in names:
            params[name] = search_value(candidate.search[name], fractions_by_name[name][index])
        trials.append(
            TrialSpec(
                candidate_id=candidate.candidate_id,
                trial_id=f"{candidate.candidate_id}:trial_{index + 1:03d}",
                model=candidate.model,
                parameters=params,
                is_nominal=False,
            )
        )
    return trials


def search_value(bounds: dict[str, Any], fraction: float) -> float:
    lower = float(bounds.get("min", 0.0))
    upper = float(bounds.get("max", lower))
    fraction = max(0.0, min(1.0, fraction))
    if str(bounds.get("scale", "linear")).lower() == "log" and lower > 0.0 and upper > 0.0:
        return math.exp(math.log(lower) + fraction * (math.log(upper) - math.log(lower)))
    return lower + fraction * (upper - lower)


def load_manifest_segments(manifest_path: Path, split_config: dict[str, Any]) -> list[SegmentMeta]:
    manifest = load_json(manifest_path)
    segments: list[SegmentMeta] = []
    for raw in manifest.get("segments", []):
        if not isinstance(raw, dict):
            continue
        segment_id = str(raw.get("segment_id", "")).strip()
        if not segment_id:
            continue
        row_count = int(
            raw.get(
                "segment_row_count",
                int(raw.get("segment_end_row_index", 0)) - int(raw.get("segment_start_row_index", 0)) + 1,
            )
        )
        start_row_index = int(raw.get("segment_start_row_index", 0))
        end_row_index = int(raw.get("segment_end_row_index", -1))
        active_start_row_index = optional_int(raw.get("active_start_row_index"))
        active_end_row_index = optional_int(raw.get("active_end_row_index"))
        corrupted = is_corrupted_segment(raw)
        parameter_fields = dict(raw.get("parameter_fields") or {})
        observed_command = dict(raw.get("observed_command") or {})
        stage = str(raw.get("stage", ""))
        family = str(raw.get("family", ""))
        split = str(raw.get("split", "")).strip()
        if split not in SPLITS:
            split = stable_split(segment_id, split_config)
        segments.append(
            SegmentMeta(
                segment_id=segment_id,
                stage=stage,
                family=family,
                log_path=str(raw.get("log_path", "")),
                log_id=str(raw.get("log_id", raw.get("run_id", ""))),
                row_count=max(0, row_count),
                start_row_index=start_row_index,
                end_row_index=end_row_index,
                active_start_row_index=active_start_row_index,
                active_end_row_index=active_end_row_index,
                corrupted=corrupted,
                parameter_fields=parameter_fields,
                observed_command=observed_command,
                split=split,
                coverage_key=segment_coverage_key(stage, family, parameter_fields, observed_command),
            )
        )
    return assign_coverage_splits(segments, split_config)


def optional_int(value: Any) -> int | None:
    if value is None or value == "":
        return None
    try:
        return int(float(value))
    except (TypeError, ValueError):
        return None


def is_corrupted_segment(raw: dict[str, Any]) -> bool:
    explicitly_marked = (
        raw.get("corrupted") is True
        or bool(raw.get("corruption_note"))
        or str(raw.get("end_reason", "")).strip().lower() == "corruption_boundary"
    )
    return explicitly_marked and is_terminal_external_force_boundary(raw)


def is_terminal_external_force_boundary(raw: dict[str, Any]) -> bool:
    text = " ".join(
        str(raw.get(name, "") or "")
        for name in (
            "end_reason",
            "end_criterion",
            "corruption_note",
            "terminal_reason",
            "boundary_reason",
        )
    ).lower()
    normalized = text.replace("_", " ").replace("-", " ")
    markers = (
        "pickup",
        "pick up",
        "picked up",
        "runoff",
        "run off",
        "external force",
        "terminal external",
        "workspace violation",
    )
    return any(marker in normalized for marker in markers)


def assign_coverage_splits(
    segments: Sequence[SegmentMeta],
    split_config: dict[str, Any],
) -> list[SegmentMeta]:
    if str(split_config.get("strategy", "")).strip().lower() in (
        "source_log_heldout_then_whole_segment",
        "whole_source_log_heldout_then_whole_segment",
    ):
        return assign_source_log_heldout_then_segment_splits(segments, split_config)

    grouped: dict[str, list[SegmentMeta]] = {}
    for segment in segments:
        if segment.corrupted:
            grouped.setdefault(f"corrupted:{segment.coverage_key}", []).append(segment)
        else:
            grouped.setdefault(segment.coverage_key, []).append(segment)
    result: list[SegmentMeta] = []
    for key, bucket in sorted(grouped.items()):
        ordered = sorted(bucket, key=lambda segment: stable_hash_float(split_seed(split_config), key, segment.segment_id))
        counts = split_counts_for_bucket(len(ordered), split_config)
        split_names = (
            ["train"] * counts["train"]
            + ["validation"] * counts["validation"]
            + ["held_out"] * counts["held_out"]
        )
        for segment, split in zip(ordered, split_names):
            result.append(segment_with_split(segment, split))
    return sorted(result, key=lambda segment: segment.segment_id)


def assign_source_log_heldout_then_segment_splits(
    segments: Sequence[SegmentMeta],
    split_config: dict[str, Any],
) -> list[SegmentMeta]:
    seed = split_seed(split_config)
    clean_segments = [segment for segment in segments if not segment.corrupted]
    held_out_logs = choose_heldout_source_logs(clean_segments, split_config)
    remaining = [segment for segment in segments if source_log_key(segment) not in held_out_logs]
    grouped: dict[str, list[SegmentMeta]] = {}
    for segment in remaining:
        if segment.corrupted:
            grouped.setdefault(f"corrupted:{segment.coverage_key}", []).append(segment)
        else:
            grouped.setdefault(segment.coverage_key, []).append(segment)

    result = [
        segment_with_split(segment, "held_out")
        for segment in segments
        if source_log_key(segment) in held_out_logs
    ]
    for key, bucket in sorted(grouped.items()):
        ordered = sorted(bucket, key=lambda segment: stable_hash_float(seed, key, segment.segment_id))
        counts = train_validation_split_counts(len(ordered), split_config)
        split_names = ["train"] * counts["train"] + ["validation"] * counts["validation"]
        for segment, split in zip(ordered, split_names):
            result.append(segment_with_split(segment, split))
    return sorted(result, key=lambda segment: segment.segment_id)


def choose_heldout_source_logs(
    segments: Sequence[SegmentMeta],
    split_config: dict[str, Any],
) -> set[str]:
    by_log: dict[str, list[SegmentMeta]] = {}
    for segment in segments:
        by_log.setdefault(source_log_key(segment), []).append(segment)
    if len(by_log) <= 1:
        return set()

    held_ratio = split_ratio(split_config, "held_out", 0.2)
    train_ratio = split_ratio(split_config, "train", 0.6)
    validation_ratio = split_ratio(split_config, "validation", 0.2)
    total_ratio = train_ratio + validation_ratio + held_ratio
    if held_ratio <= 0.0 or total_ratio <= 0.0:
        return set()

    total_segments = sum(len(bucket) for bucket in by_log.values())
    target_segments = max(1, int(round(total_segments * held_ratio / total_ratio)))
    ordered_logs = sorted(
        by_log,
        key=lambda log_key: stable_hash_float(split_seed(split_config), "heldout-source-log", log_key),
    )
    held_logs: set[str] = set()
    held_segments = 0
    for log_key in ordered_logs:
        if len(held_logs) >= len(ordered_logs) - 1:
            break
        held_logs.add(log_key)
        held_segments += len(by_log[log_key])
        if held_segments >= target_segments:
            break
    return held_logs


def train_validation_split_counts(size: int, split_config: dict[str, Any]) -> dict[str, int]:
    if size <= 0:
        return {"train": 0, "validation": 0}
    if size == 1:
        return {"train": 1, "validation": 0}
    train_ratio = split_ratio(split_config, "train", 0.6)
    validation_ratio = split_ratio(split_config, "validation", 0.2)
    total = train_ratio + validation_ratio
    if total <= 0.0:
        train_ratio, validation_ratio, total = 0.75, 0.25, 1.0
    validation = max(1, int(round(size * validation_ratio / total)))
    train = size - validation
    if train < 1:
        train = 1
        validation = size - train
    return {"train": train, "validation": validation}


def split_ratio(split_config: dict[str, Any], name: str, default: float) -> float:
    try:
        return max(0.0, float(split_config.get(name, default)))
    except (TypeError, ValueError):
        return default


def segment_with_split(segment: SegmentMeta, split: str) -> SegmentMeta:
    return SegmentMeta(
        segment_id=segment.segment_id,
        stage=segment.stage,
        family=segment.family,
        log_path=segment.log_path,
        log_id=segment.log_id,
        row_count=segment.row_count,
        start_row_index=segment.start_row_index,
        end_row_index=segment.end_row_index,
        active_start_row_index=segment.active_start_row_index,
        active_end_row_index=segment.active_end_row_index,
        corrupted=segment.corrupted,
        parameter_fields=segment.parameter_fields,
        observed_command=segment.observed_command,
        split=split,
        coverage_key=segment.coverage_key,
    )


def split_counts_for_bucket(size: int, split_config: dict[str, Any]) -> dict[str, int]:
    if size <= 0:
        return {"train": 0, "validation": 0, "held_out": 0}
    if size == 1:
        return {"train": 1, "validation": 0, "held_out": 0}
    if size == 2:
        return {"train": 1, "validation": 1, "held_out": 0}
    train_ratio = float(split_config.get("train", 0.6))
    validation_ratio = float(split_config.get("validation", 0.2))
    held_ratio = float(split_config.get("held_out", 0.2))
    total = train_ratio + validation_ratio + held_ratio
    if total <= 0.0:
        train_ratio, validation_ratio, held_ratio, total = 0.6, 0.2, 0.2, 1.0
    validation = max(1, int(round(size * validation_ratio / total)))
    held_out = max(1, int(round(size * held_ratio / total)))
    train = size - validation - held_out
    while train < 1:
        if held_out >= validation and held_out > 1:
            held_out -= 1
        elif validation > 1:
            validation -= 1
        else:
            break
        train = size - validation - held_out
    return {"train": train, "validation": validation, "held_out": held_out}


def split_seed(split_config: dict[str, Any]) -> str:
    return str(split_config.get("seed", "traction_rms_nis_testbed_v1"))


def source_log_key(segment: SegmentMeta) -> str:
    return segment.log_id or segment.log_path or segment.segment_id


def segment_coverage_key(
    stage: str,
    family: str,
    parameter_fields: dict[str, Any],
    observed_command: dict[str, Any],
) -> str:
    base = segment_coverage_base_key(stage, family, parameter_fields)
    if is_launch_stage(stage, family):
        command = launch_command_signature(observed_command)
        return f"{base}|observed_command={command}"
    return base


def segment_coverage_base_key(
    stage: str,
    family: str,
    parameter_fields: dict[str, Any],
) -> str:
    parameter = format_bucket_value(parameter_fields.get("parameter"))
    value_kind = format_bucket_value(parameter_fields.get("test_value_kind"))
    test_value = format_bucket_value(parameter_fields.get("test_value"))
    return f"stage={stage}|family={family}|parameter={parameter}|kind={value_kind}|value={test_value}"


def format_bucket_value(value: Any) -> str:
    if isinstance(value, float):
        return f"{value:.12g}"
    if isinstance(value, list):
        return ",".join(format_bucket_value(child) for child in value)
    return "" if value is None else str(value)


def is_launch_stage(stage: str, family: str) -> bool:
    text = f"{stage} {family}".lower()
    return "launch" in text


def active_metric_name(stage: str, family: str) -> str:
    stage_text = stage.lower()
    family_text = family.lower()
    if "recovery" in family_text or "static_hold" in family_text:
        return ""
    if "mixed_launch" in stage_text:
        return "mixed_launch"
    if "yaw_launch" in stage_text:
        return "yaw_launch"
    if (
        "in_place_turn" in family_text
        or "sec_40_yaw" in stage_text
        or "yaw_maneuver" in stage_text
        or "yaw" in stage_text
        or "yaw" in family_text
    ):
        return "yaw_calibration"
    if "smooth_turn" in family_text:
        return "smooth_turn_active"
    if "straight" in family_text:
        return "straight_active"
    if "launch" in family_text:
        return "launch_active_pulse"
    return ""


def active_metric_name_from_coverage(stage: str, coverage_key: str) -> str:
    family = ""
    for part in coverage_key.split("|"):
        if part.startswith("family="):
            family = part[len("family="):]
            break
    return active_metric_name(stage, family)


def is_primary_active_metric(segment: SegmentMeta, metric_weights: dict[str, float]) -> bool:
    metric = active_metric_name(segment.stage, segment.family)
    return bool(metric) and (not metric_weights or metric in metric_weights)


def stable_hash_float(seed: str, *parts: str) -> float:
    text = ":".join([seed, *parts])
    value = int(hashlib.sha256(text.encode("utf-8")).hexdigest()[:16], 16)
    return value / float(0xFFFFFFFFFFFFFFFF)


def select_tuning_subset(
    segments: Sequence[SegmentMeta],
    split_name: str,
    target_count: int,
    seed: str,
    launch_fraction: float,
    include_corrupted: bool = False,
) -> list[SegmentMeta]:
    available = [
        segment
        for segment in segments
        if (include_corrupted or not segment.corrupted) and segment.split == split_name
    ]
    if target_count <= 0 or target_count >= len(available):
        return sorted(available, key=lambda segment: segment.segment_id)

    selected: dict[str, SegmentMeta] = {}
    launch_target = min(target_count, max(1, int(round(target_count * launch_fraction))))
    launch_groups = group_segments(
        [segment for segment in available if is_launch_stage(segment.stage, segment.family)]
    )
    other_groups = group_segments(
        [segment for segment in available if not is_launch_stage(segment.stage, segment.family)]
    )
    add_one_per_group(selected, launch_groups, launch_target, seed)
    add_one_per_group(selected, other_groups, target_count, seed)
    if len(selected) < target_count:
        add_remaining(selected, available, target_count, seed)
    return sorted(selected.values(), key=lambda segment: segment.segment_id)


def group_segments(segments: Sequence[SegmentMeta]) -> dict[str, list[SegmentMeta]]:
    groups: dict[str, list[SegmentMeta]] = {}
    for segment in segments:
        groups.setdefault(segment.coverage_key, []).append(segment)
    return groups


def add_one_per_group(
    selected: dict[str, SegmentMeta],
    groups: dict[str, list[SegmentMeta]],
    target_count: int,
    seed: str,
) -> None:
    ordered_groups = sorted(
        groups.items(),
        key=lambda item: stable_hash_float(seed, "group", item[0]),
    )
    for group_key, members in ordered_groups:
        if len(selected) >= target_count:
            break
        choice = sorted(
            members,
            key=lambda segment: (
                segment.row_count,
                stable_hash_float(seed, "member", group_key, segment.segment_id),
            ),
        )[0]
        selected.setdefault(choice.segment_id, choice)


def add_remaining(
    selected: dict[str, SegmentMeta],
    available: Sequence[SegmentMeta],
    target_count: int,
    seed: str,
) -> None:
    remaining = [
        segment for segment in available if segment.segment_id not in selected
    ]
    remaining.sort(
        key=lambda segment: (
            0 if is_launch_stage(segment.stage, segment.family) else 1,
            stable_hash_float(seed, "fill", segment.coverage_key, segment.segment_id),
        )
    )
    for segment in remaining:
        if len(selected) >= target_count:
            break
        selected[segment.segment_id] = segment


def evaluate_trials(
    phase: str,
    manifest_path: Path,
    segment_metas: Sequence[SegmentMeta],
    trials: Sequence[TrialSpec],
    vehicle: VehicleConfig,
    covariance_path: Path,
    max_rows_per_segment: int = 0,
    metric_scope: str = "all",
    metric_weights: dict[str, float] | None = None,
    sample_source: EvaluationSampleSource | None = None,
) -> EvaluationResult:
    _vehicle_from_covariance, covariance = load_covariance(covariance_path)
    del _vehicle_from_covariance
    scoped_segments = [
        segment
        for segment in segment_metas
        if segment_in_metric_scope(segment, metric_scope, metric_weights or {})
    ]
    plants = {
        trial.key: CandidatePlant(vehicle, trial.estimator_candidate())
        for trial in trials
    }
    result = EvaluationResult(phase=phase, segment_count=len(scoped_segments))
    source = sample_source or EvaluationSampleSource(manifest_path, vehicle)
    for meta, samples in source.iter_segment_samples(
        scoped_segments,
        max_rows_per_segment,
        metric_scope,
    ):
        evaluate_segment_samples(result, meta, samples, trials, plants, covariance)
    return result


def segment_in_metric_scope(
    segment: SegmentMeta,
    metric_scope: str,
    metric_weights: dict[str, float],
) -> bool:
    if metric_scope == "active":
        return is_primary_active_metric(segment, metric_weights)
    return True


def evaluate_segment_samples(
    result: EvaluationResult,
    meta: SegmentMeta,
    samples: Sequence[ReplaySample],
    trials: Sequence[TrialSpec],
    plants: dict[tuple[str, str], CandidatePlant],
    covariance: Any,
) -> None:
    if not samples:
        return
    result.sample_count += len(samples)
    source_log = source_log_key(meta)
    keyed_samples = [
        (sample, sample_coverage_key(meta, sample))
        for sample in samples
    ]
    for trial in trials:
        plant = plants[trial.key]
        state = [0.0 for _ in range(N)]
        state_valid = True
        for sample, coverage_key in keyed_samples:
            if state_valid:
                try:
                    state = plant.propagate(state, sample, sample.dt_s)
                    plant_result = plant.plant_result(state, sample)
                    updates = residual_tail_updates(
                        sample,
                        state,
                        plant_result,
                        covariance,
                        plant.vehicle,
                    )
                except (ArithmeticError, OverflowError, ValueError):
                    state_valid = False
                    updates = invalid_residual_tail_updates(sample, covariance)
            else:
                updates = invalid_residual_tail_updates(sample, covariance)
            for update in updates:
                if update is None:
                    continue
                if update.log_parameter not in PRODUCTION_MEASUREMENT_RESIDUAL_TAIL_LOG_FIELDS:
                    continue
                result.add(
                    trial=trial,
                    split=meta.split,
                    stage=meta.stage,
                    source_log=source_log,
                    log_parameter=update.log_parameter,
                    coverage_key=coverage_key,
                    nis=update.nis,
                    accepted=update.accepted,
                )


def segment_spec_from_meta(meta: SegmentMeta) -> SegmentSpec:
    log_path = Path(meta.log_path)
    if not log_path.is_absolute():
        log_path = REPO_ROOT / log_path
    return SegmentSpec(
        log_path=log_path,
        segment_id=meta.segment_id,
        stage=meta.stage,
        family=meta.family,
        split=meta.split,
        run_id=meta.log_id,
        start_row_index=meta.start_row_index,
        end_row_index=meta.end_row_index,
        corrupted=meta.corrupted,
        parameter_fields=meta.parameter_fields,
        observed_command=meta.observed_command,
    )


def sample_coverage_key(meta: SegmentMeta, sample: ReplaySample) -> str:
    if not is_launch_stage(meta.stage, meta.family):
        return meta.coverage_key
    base = segment_coverage_base_key(
        meta.stage,
        meta.family,
        meta.parameter_fields,
    )
    return f"{base}|command_bucket={sample_launch_command_signature(sample)}"


def sample_launch_command_signature(sample: ReplaySample) -> str:
    linear_command = 0.5 * (sample.left_command + sample.right_command)
    yaw_command = 0.5 * (sample.right_command - sample.left_command)
    return (
        f"pair={command_bin(sample.left_command)},{command_bin(sample.right_command)};"
        f"linear={command_bin(linear_command)};"
        f"yaw={command_bin(yaw_command)}"
    )


def command_bin(value: float) -> str:
    if not finite(value):
        return ""
    binned = round(value / COMMAND_BIN_WIDTH) * COMMAND_BIN_WIDTH
    return format_number(0.0 if abs(binned) < 0.5 * COMMAND_BIN_WIDTH else binned)


def samples_from_stream(
    stream: SegmentStream,
    meta: SegmentMeta,
    vehicle: VehicleConfig,
    max_rows_per_segment: int,
    metric_scope: str,
) -> Iterator[ReplaySample]:
    indices = indices_for_metric_scope(stream, meta, metric_scope)
    if max_rows_per_segment > 0 and len(indices) > max_rows_per_segment:
        indices = [indices[index] for index in bounded_indices(len(indices), max_rows_per_segment)]
    for index in indices:
        yield ReplaySample(
            source_path=stream.definition.log_absolute_path,
            source_row_index=int(stream.streams["row_index"][index]),
            master_time_us=int_or_default(stream_value(stream, "metadata", "master_time_us", index), 0),
            dt_s=dt_seconds(stream.streams["dt_us"][index]),
            left_command=float_or_default(stream_value(stream, "drive_commands", "left_drive_command", index), 0.0),
            right_command=float_or_default(stream_value(stream, "drive_commands", "right_drive_command", index), 0.0),
            left_wheel_rate_radps=wheel_rate(stream, "left", index, vehicle),
            right_wheel_rate_radps=wheel_rate(stream, "right", index, vehicle),
            yaw_rate_radps=yaw_rate(stream, index),
            accel_forward_mps2=float_or_default(
                stream_value(stream, "accel", "accel_body_y_mps2", index),
                math.nan,
            ),
            accel_right_mps2=float_or_default(
                stream_value(stream, "accel", "accel_body_x_mps2", index),
                math.nan,
            ),
            accel_valid=accel_valid(stream, index),
            gyro_valid=finite(yaw_rate(stream, index)),
            fan_duty_cycle=float_or_default(
                stream_value(stream, "fan", "fan_duty_cycle", index),
                vehicle.default_fan_duty_cycle,
            ),
            segment_id=meta.segment_id,
            stage=meta.stage,
            split=meta.split,
            run_id=meta.log_id or Path(meta.log_path).parent.name,
            corrupted=meta.corrupted,
        )


def samples_for_metric_scope(
    samples: Sequence[ReplaySample],
    meta: SegmentMeta,
    metric_scope: str,
) -> list[ReplaySample]:
    if metric_scope != "active":
        return list(samples)
    return [sample for sample in samples if row_is_active(sample.source_row_index, meta)]


def indices_for_metric_scope(
    stream: SegmentStream,
    meta: SegmentMeta,
    metric_scope: str,
) -> list[int]:
    if metric_scope != "active":
        return bounded_indices(stream.row_count, 0)
    row_indices = stream.streams.get("row_index", [])
    return [
        index
        for index in range(stream.row_count)
        if index < len(row_indices) and row_is_active(int(row_indices[index]), meta)
    ]


def row_is_active(row_index: int, meta: SegmentMeta) -> bool:
    if meta.active_start_row_index is None or meta.active_end_row_index is None:
        return True
    return meta.active_start_row_index <= row_index <= meta.active_end_row_index


def target_row_indices(
    meta: SegmentMeta,
    max_rows_per_segment: int,
    metric_scope: str,
) -> list[int]:
    row_range = target_row_range(meta, metric_scope)
    if row_range is None:
        return []
    start, end = row_range
    count = end - start + 1
    if count <= 0:
        return []
    if max_rows_per_segment <= 0 or count <= max_rows_per_segment:
        return list(range(start, end + 1))
    return [start + index for index in bounded_indices(count, max_rows_per_segment)]


def target_row_range(meta: SegmentMeta, metric_scope: str) -> tuple[int, int] | None:
    if metric_scope == "active":
        start = meta.active_start_row_index
        end = meta.active_end_row_index
        if start is None or end is None:
            start = meta.start_row_index
            end = meta.end_row_index
    else:
        start = meta.start_row_index
        end = meta.end_row_index
    if end < start:
        return None
    return start, end


def stream_value(stream: SegmentStream, group: str, field_name: str, index: int) -> Any:
    group_values = stream.streams.get(group, {})
    if not isinstance(group_values, dict):
        return None
    values = group_values.get(field_name)
    if not isinstance(values, list) or index >= len(values):
        return None
    return values[index]


def dt_seconds(value: Any) -> float:
    return max(0.0, min(finite_or(float_or_default(value, 1000.0), 1000.0) * 1.0e-6, 0.050))


def wheel_rate(stream: SegmentStream, side: str, index: int, vehicle: VehicleConfig) -> float:
    omega = float_or_default(
        stream_value(stream, "encoder", f"{side}_encoder_omega_radps", index),
        math.nan,
    )
    if finite(omega):
        return omega
    velocity = float_or_default(
        stream_value(stream, "encoder", f"{side}_encoder_velocity_mps", index),
        math.nan,
    )
    if finite(velocity):
        return velocity / max(vehicle.wheel_radius_m, 1.0e-9)
    return 0.0


def yaw_rate(stream: SegmentStream, index: int) -> float:
    gyro = float_or_default(stream_value(stream, "gyro", "gyro_radps", index), math.nan)
    if finite(gyro):
        return gyro
    raw_gyro = float_or_default(stream_value(stream, "gyro", "gyro_raw_radps", index), math.nan)
    gyro_bias = float_or_default(stream_value(stream, "gyro", "gyro_bias_radps", index), 0.0)
    if finite(raw_gyro):
        return raw_gyro - gyro_bias
    measured = float_or_default(
        stream_value(stream, "encoder", "measured_angular_speed_radps", index),
        math.nan,
    )
    return measured


def accel_valid(stream: SegmentStream, index: int) -> bool:
    explicit = stream_value(stream, "metadata", "accel_valid", index)
    if explicit is None:
        explicit = stream_value(stream, "metadata", "imu_accel_valid", index)
    if explicit is not None:
        text = str(explicit).strip().lower()
        return text in ("1", "true", "yes", "valid")
    forward = float_or_default(stream_value(stream, "accel", "accel_body_y_mps2", index), math.nan)
    right = float_or_default(stream_value(stream, "accel", "accel_body_x_mps2", index), math.nan)
    return finite(forward) and finite(right)


def float_or_default(value: Any, default: float) -> float:
    if value is None:
        return default
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


def int_or_default(value: Any, default: int) -> int:
    try:
        return int(float(value))
    except (TypeError, ValueError):
        return default


def rank_trials(
    phase: str,
    result: EvaluationResult,
    split_name: str,
    floor_ratio: float,
    under_expected_weight: float,
    inflation_weight: float,
    launch_weight: float,
    metric_weights: dict[str, float] | None = None,
    row_weighted: bool = False,
) -> list[SelectionScore]:
    by_trial: dict[tuple[str, str], list[tuple[BucketStats, str, str]]] = {}
    for key, stats in result.detail.items():
        candidate_id, trial_id, split, stage, _log_field, coverage_key = key
        if split != split_name:
            continue
        by_trial.setdefault((candidate_id, trial_id), []).append((stats, stage, coverage_key))
    rankings: list[SelectionScore] = []
    for (candidate_id, trial_id), buckets in sorted(by_trial.items()):
        weighted_sum = 0.0
        total_weight = 0.0
        sample_count = 0
        for stats, stage, coverage_key in buckets:
            if stats.finite_count <= 0:
                continue
            expected = expected_rms_for_dimension(1)
            rms = stats.rms_nis
            guarded = max(rms, expected)
            under = max(0.0, expected - rms)
            inflation_flag = rms < expected * floor_ratio
            score = item_score_from_values(
                rms_nis=rms,
                expected=expected,
                guarded=guarded,
                under=under,
                inflation_flag=inflation_flag,
                under_expected_weight=under_expected_weight,
                inflation_weight=inflation_weight,
            )
            metric = active_metric_name_from_coverage(stage, coverage_key)
            if metric_weights is not None and metric not in metric_weights:
                continue
            sample_count += stats.finite_count
            weight = float(stats.finite_count) if row_weighted else 1.0
            if metric_weights is not None:
                weight *= float(metric_weights.get(metric, 0.0))
            elif "launch" in coverage_key.lower():
                weight *= launch_weight
            weighted_sum += score * weight
            total_weight += weight
        rankings.append(
            SelectionScore(
                phase=phase,
                candidate_id=candidate_id,
                trial_id=trial_id,
                selection_score=weighted_sum / total_weight if total_weight else math.inf,
                sample_count=sample_count,
            )
        )
    rankings.sort(key=lambda row: (row.candidate_id, row.selection_score, row.trial_id))
    return rankings


def item_score_from_values(
    rms_nis: float,
    expected: float,
    guarded: float,
    under: float,
    inflation_flag: bool,
    under_expected_weight: float,
    inflation_weight: float,
) -> float:
    normalized = guarded / expected
    normalized += under_expected_weight * (under / expected)
    if inflation_flag:
        normalized += inflation_weight * ((expected - rms_nis) / expected)
    return normalized


def top_trials_by_candidate(
    trials: Sequence[TrialSpec],
    scores: Sequence[SelectionScore],
    top_k: int,
) -> list[TrialSpec]:
    trial_by_key = {trial.key: trial for trial in trials}
    by_candidate: dict[str, list[SelectionScore]] = {}
    for score in scores:
        by_candidate.setdefault(score.candidate_id, []).append(score)
    selected: dict[tuple[str, str], TrialSpec] = {}
    for candidate_id, rows in by_candidate.items():
        ordered = sorted(rows, key=lambda row: (row.selection_score, row.trial_id))
        for row in ordered[: max(1, top_k)]:
            trial = trial_by_key.get((row.candidate_id, row.trial_id))
            if trial is not None:
                selected[trial.key] = trial
        nominal = next(
            (trial for trial in trials if trial.candidate_id == candidate_id and trial.is_nominal),
            None,
        )
        if nominal is not None:
            selected[nominal.key] = nominal
    return sorted(selected.values(), key=lambda trial: (trial.candidate_id, trial.trial_id))


def best_trials_by_candidate(
    trials: Sequence[TrialSpec],
    validation_scores: Sequence[SelectionScore],
    candidate_ids: Sequence[str] = TARGET_CANDIDATES,
) -> list[TrialSpec]:
    trial_by_key = {trial.key: trial for trial in trials}
    best: dict[str, SelectionScore] = {}
    for score in validation_scores:
        current = best.get(score.candidate_id)
        if current is None or (score.selection_score, score.trial_id) < (
            current.selection_score,
            current.trial_id,
        ):
            best[score.candidate_id] = score
    selected: list[TrialSpec] = []
    for candidate_id in candidate_ids:
        score = best.get(candidate_id)
        if score is None:
            continue
        trial = trial_by_key.get((score.candidate_id, score.trial_id))
        if trial is not None:
            selected.append(trial)
    return selected


def score_rows(
    phase: str,
    result: EvaluationResult,
    split_filter: set[str] | None = None,
) -> list[ScoreRow]:
    rows: list[ScoreRow] = []
    for key, stats in sorted(result.report.items()):
        candidate_id, trial_id, split, stage, log_field = key
        if split_filter is not None and split not in split_filter:
            continue
        expected = expected_rms_for_dimension(1)
        rms = stats.rms_nis
        rows.append(
            ScoreRow(
                phase=phase,
                candidate_id=candidate_id,
                trial_id=trial_id,
                split=split,
                stage=stage,
                log_parameter=log_field,
                count=stats.count,
                accepted_count=stats.accepted_count,
                finite_count=stats.finite_count,
                nonfinite_count=stats.count - stats.finite_count,
                rms_nis=rms,
                accepted_only_rms_nis=stats.accepted_only_rms_nis,
                expected_rms_nis=expected,
                guarded_rms_nis=max(rms, expected),
                under_expected_penalty=max(0.0, expected - rms),
                inflation_flag=rms < expected * 0.75,
                rejected_count=stats.rejected_count,
                rejected_rate=stats.rejected_rate,
            )
        )
    return rows


def bootstrap_confidence_rows(
    result: EvaluationResult,
    selected_trials: Sequence[TrialSpec],
    baseline_candidate_id: str,
    split_names: set[str],
    iterations: int,
    seed: str,
    floor_ratio: float,
    under_expected_weight: float,
    inflation_weight: float,
    launch_weight: float,
    metric_weights: dict[str, float] | None,
    row_weighted: bool,
) -> list[BootstrapRow]:
    if iterations <= 0:
        return []
    baseline = next(
        (trial for trial in selected_trials if trial.candidate_id == baseline_candidate_id),
        None,
    )
    if baseline is None:
        return []
    source_scores = source_log_trial_scores(
        result=result,
        split_names=split_names,
        floor_ratio=floor_ratio,
        under_expected_weight=under_expected_weight,
        inflation_weight=inflation_weight,
        launch_weight=launch_weight,
        metric_weights=metric_weights,
        row_weighted=row_weighted,
    )
    rows: list[BootstrapRow] = []
    for trial in selected_trials:
        if trial.key == baseline.key:
            continue
        paired_logs = sorted(
            log_key
            for log_key in source_log_keys_for_trial(source_scores, trial.key)
            if (log_key, baseline.key) in source_scores
            and math.isfinite(source_scores[(log_key, trial.key)])
            and math.isfinite(source_scores[(log_key, baseline.key)])
        )
        if not paired_logs:
            continue
        observed_delta = mean(
            source_scores[(log_key, trial.key)] - source_scores[(log_key, baseline.key)]
            for log_key in paired_logs
        )
        rng = random.Random(f"{seed}:source-log-bootstrap:{trial.candidate_id}:{trial.trial_id}")
        deltas: list[float] = []
        for _ in range(iterations):
            sampled = [paired_logs[rng.randrange(len(paired_logs))] for _ in paired_logs]
            deltas.append(
                mean(
                    source_scores[(log_key, trial.key)] - source_scores[(log_key, baseline.key)]
                    for log_key in sampled
                )
            )
        deltas.sort()
        rows.append(
            BootstrapRow(
                candidate_id=trial.candidate_id,
                trial_id=trial.trial_id,
                baseline_candidate_id=baseline.candidate_id,
                baseline_trial_id=baseline.trial_id,
                split=",".join(sorted(split_names)),
                source_log_count=len(paired_logs),
                bootstrap_iterations=iterations,
                observed_delta_vs_baseline=observed_delta,
                ci_low=percentile_sorted(deltas, 0.025),
                ci_high=percentile_sorted(deltas, 0.975),
                probability_candidate_better=sum(1 for value in deltas if value < 0.0) / len(deltas),
            )
        )
    return rows


def source_log_trial_scores(
    result: EvaluationResult,
    split_names: set[str],
    floor_ratio: float,
    under_expected_weight: float,
    inflation_weight: float,
    launch_weight: float,
    metric_weights: dict[str, float] | None,
    row_weighted: bool,
) -> dict[tuple[str, tuple[str, str]], float]:
    by_log_trial: dict[tuple[str, tuple[str, str]], list[tuple[BucketStats, str, str]]] = {}
    for key, stats in result.source_log_detail.items():
        candidate_id, trial_id, split, source_log, stage, _log_field, coverage_key = key
        if split not in split_names:
            continue
        by_log_trial.setdefault((source_log, (candidate_id, trial_id)), []).append((stats, stage, coverage_key))

    scores: dict[tuple[str, tuple[str, str]], float] = {}
    for key, buckets in by_log_trial.items():
        weighted_sum = 0.0
        total_weight = 0.0
        for stats, stage, coverage_key in buckets:
            if stats.finite_count <= 0:
                continue
            metric = active_metric_name_from_coverage(stage, coverage_key)
            if metric_weights is not None and metric not in metric_weights:
                continue
            expected = expected_rms_for_dimension(1)
            rms = stats.rms_nis
            score = item_score_from_values(
                rms_nis=rms,
                expected=expected,
                guarded=max(rms, expected),
                under=max(0.0, expected - rms),
                inflation_flag=rms < expected * floor_ratio,
                under_expected_weight=under_expected_weight,
                inflation_weight=inflation_weight,
            )
            weight = float(stats.finite_count) if row_weighted else 1.0
            if metric_weights is not None:
                weight *= float(metric_weights.get(metric, 0.0))
            elif "launch" in coverage_key.lower():
                weight *= launch_weight
            weighted_sum += score * weight
            total_weight += weight
        scores[key] = weighted_sum / total_weight if total_weight else math.inf
    return scores


def source_log_keys_for_trial(
    source_scores: dict[tuple[str, tuple[str, str]], float],
    trial_key: tuple[str, str],
) -> set[str]:
    return {
        source_log
        for source_log, key in source_scores
        if key == trial_key
    }


def percentile_sorted(values: Sequence[float], quantile: float) -> float:
    if not values:
        return math.nan
    index = max(0, min(len(values) - 1, int(round(quantile * (len(values) - 1)))))
    return values[index]


def mean(values: Iterable[float]) -> float:
    total = 0.0
    count = 0
    for value in values:
        total += value
        count += 1
    return total / count if count else math.nan


def write_outputs(
    output_dir: Path,
    manifest_path: Path,
    config_path: Path,
    covariance_path: Path,
    candidates: Sequence[CandidateSpec],
    trials: Sequence[TrialSpec],
    train_subset: Sequence[SegmentMeta],
    validation_subset: Sequence[SegmentMeta],
    all_segments: Sequence[SegmentMeta],
    train_scores: Sequence[SelectionScore],
    validation_scores: Sequence[SelectionScore],
    selected_trials: Sequence[TrialSpec],
    final_result: EvaluationResult,
    stress_result: EvaluationResult,
    bootstrap_rows: Sequence[BootstrapRow],
    bias_source_manifest_path: Path,
    metric_scope: str,
    metric_weights: dict[str, float],
    row_weighted_selection: bool,
    args: argparse.Namespace,
    sample_source_stats: dict[str, int] | None = None,
) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    selected_by_key = {trial.key: trial for trial in selected_trials}
    write_split_counts(output_dir, all_segments)
    write_source_log_splits(output_dir, all_segments)
    write_subset_segments(output_dir, "train_subset_segments.csv", train_subset)
    write_subset_segments(output_dir, "validation_subset_segments.csv", validation_subset)
    write_trial_scores(output_dir, [*train_scores, *validation_scores])
    write_selected_trials(output_dir, selected_trials, train_scores, validation_scores)
    final_rows = score_rows("full_selected", final_result, {"validation", "held_out"})
    write_score_rows(output_dir, "validation_heldout_residual_tail.csv", final_rows)
    stress_rows = score_rows("stress_full_rows", stress_result, {"validation", "held_out"})
    write_score_rows(output_dir, "stress_full_row_weighted_residual_tail.csv", stress_rows)
    write_bootstrap_rows(output_dir, bootstrap_rows)
    write_tuned_parameters(output_dir, selected_trials, covariance_path)

    split_counts = summarize_split_counts(all_segments)
    payload = {
        "schema_version": 1,
        "uses_logged_ukf_state": False,
        "production_or_hardware_hooks": False,
        "covariance_policy": "fixed_testbed_covariance_not_tuned",
        "fixed_noise_schedule_path": str(covariance_path),
        "candidate_specific_covariance_or_noise": False,
        "primary_metric_scope": "active_traction_rows",
        "primary_metric_policy": "metric_balanced_active_traction_production_measurement_residual_tail_only",
        "production_measurement_residual_tail_fields": list(
            PRODUCTION_MEASUREMENT_RESIDUAL_TAIL_LOG_PARAMETERS
        ),
        "boundary_policy": "primary_active_rows_before_explicit_corruption_boundaries_are_valid",
        "primary_metric_scope_config": metric_scope,
        "primary_row_weighted_selection": row_weighted_selection,
        "stress_metric_policy": "full_row_weighted_residual_tail_diagnostic_only",
        "replay_mode": "aggregate_only_residual_tail",
        "manifest_path": str(manifest_path),
        "bias_source_manifest_path": str(bias_source_manifest_path),
        "config_path": str(config_path),
        "covariance_path": str(covariance_path),
        "candidate_count": len(candidates),
        "trial_count": len(trials),
        "train_subset_segments": len(train_subset),
        "validation_subset_segments": len(validation_subset),
        "train_subset_boundary_ended_segments": sum(1 for segment in train_subset if segment.corrupted),
        "validation_subset_boundary_ended_segments": sum(1 for segment in validation_subset if segment.corrupted),
        "final_selected_candidate_count": len(selected_trials),
        "final_segments": final_result.segment_count,
        "final_samples_per_candidate": final_result.sample_count,
        "stress_segments": stress_result.segment_count,
        "stress_samples_per_candidate": stress_result.sample_count,
        "source_log_count": len({source_log_key(segment) for segment in all_segments}),
        "bootstrap_rows": [row.__dict__ for row in bootstrap_rows],
        "active_metric_weights": metric_weights,
        "sample_source_cache": sample_source_stats or {},
        "split_counts": split_counts,
        "selected_trials": [
            {
                "candidate_id": trial.candidate_id,
                "trial_id": trial.trial_id,
                "parameters": trial.parameters,
                "train_score": score_lookup(train_scores).get(trial.key),
                "validation_score": score_lookup(validation_scores).get(trial.key),
            }
            for trial in selected_trials
            if trial.key in selected_by_key
        ],
        "limits": {
            "trial_count_per_candidate": args.trial_count,
            "train_subset_target": args.train_segments,
            "validation_subset_target": args.validation_segments,
            "validation_top_k_per_candidate": args.validation_top_k,
            "tuning_max_rows_per_segment": args.tuning_max_rows_per_segment,
            "final_max_segments": args.final_max_segments,
            "final_max_rows_per_segment": args.final_max_rows_per_segment,
            "stress_max_rows_per_segment": args.stress_max_rows_per_segment,
            "launch_weight": args.launch_weight,
        },
    }
    (output_dir / "tuning_summary.json").write_text(
        json.dumps(json_safe(payload), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    write_report(output_dir, payload, final_rows, bootstrap_rows)


def score_lookup(scores: Sequence[SelectionScore]) -> dict[tuple[str, str], float]:
    return {
        (score.candidate_id, score.trial_id): score.selection_score
        for score in scores
    }


def write_split_counts(output_dir: Path, segments: Sequence[SegmentMeta]) -> None:
    rows = summarize_split_counts(segments)
    with (output_dir / "split_counts.csv").open("w", newline="", encoding="utf-8") as handle:
        fieldnames = ["split", "stage", "family", "segments", "total_rows"]
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def summarize_split_counts(segments: Sequence[SegmentMeta]) -> list[dict[str, Any]]:
    counts: dict[tuple[str, str, str], dict[str, Any]] = {}
    for segment in segments:
        if segment.corrupted:
            continue
        key = (segment.split, segment.stage, segment.family)
        row = counts.setdefault(
            key,
            {"split": segment.split, "stage": segment.stage, "family": segment.family, "segments": 0, "total_rows": 0},
        )
        row["segments"] += 1
        row["total_rows"] += segment.row_count
    return [counts[key] for key in sorted(counts)]


def write_source_log_splits(output_dir: Path, segments: Sequence[SegmentMeta]) -> None:
    rows: dict[tuple[str, str], dict[str, Any]] = {}
    for segment in segments:
        if segment.corrupted:
            continue
        key = (source_log_key(segment), segment.split)
        row = rows.setdefault(
            key,
            {
                "source_log": source_log_key(segment),
                "split": segment.split,
                "segments": 0,
                "total_rows": 0,
                "stage_family_count": {},
            },
        )
        row["segments"] += 1
        row["total_rows"] += segment.row_count
        stage_key = f"{segment.stage}/{segment.family}"
        row["stage_family_count"][stage_key] = row["stage_family_count"].get(stage_key, 0) + 1

    with (output_dir / "source_log_splits.csv").open("w", newline="", encoding="utf-8") as handle:
        fieldnames = ["source_log", "split", "segments", "total_rows", "stage_family_count_json"]
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for key in sorted(rows):
            row = rows[key]
            writer.writerow(
                {
                    "source_log": row["source_log"],
                    "split": row["split"],
                    "segments": row["segments"],
                    "total_rows": row["total_rows"],
                    "stage_family_count_json": json.dumps(row["stage_family_count"], sort_keys=True),
                }
            )


def write_subset_segments(output_dir: Path, filename: str, segments: Sequence[SegmentMeta]) -> None:
    with (output_dir / filename).open("w", newline="", encoding="utf-8") as handle:
        fieldnames = [
            "segment_id",
            "split",
            "stage",
            "family",
            "corrupted",
            "row_count",
            "coverage_key",
            "parameter_json",
            "observed_command_signature",
        ]
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for segment in segments:
            writer.writerow(
                {
                    "segment_id": segment.segment_id,
                    "split": segment.split,
                    "stage": segment.stage,
                    "family": segment.family,
                    "corrupted": "true" if segment.corrupted else "false",
                    "row_count": segment.row_count,
                    "coverage_key": segment.coverage_key,
                    "parameter_json": json.dumps(segment.parameter_fields, sort_keys=True),
                    "observed_command_signature": launch_command_signature(segment.observed_command),
                }
            )


def write_trial_scores(output_dir: Path, scores: Sequence[SelectionScore]) -> None:
    with (output_dir / "trial_scores.csv").open("w", newline="", encoding="utf-8") as handle:
        fieldnames = ["phase", "candidate_id", "trial_id", "selection_score", "sample_count"]
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for score in sorted(scores, key=lambda row: (row.phase, row.candidate_id, row.selection_score, row.trial_id)):
            writer.writerow(
                {
                    "phase": score.phase,
                    "candidate_id": score.candidate_id,
                    "trial_id": score.trial_id,
                    "selection_score": format_number(score.selection_score),
                    "sample_count": score.sample_count,
                }
            )


def write_selected_trials(
    output_dir: Path,
    trials: Sequence[TrialSpec],
    train_scores: Sequence[SelectionScore],
    validation_scores: Sequence[SelectionScore],
) -> None:
    train = score_lookup(train_scores)
    validation = score_lookup(validation_scores)
    with (output_dir / "selected_trials.csv").open("w", newline="", encoding="utf-8") as handle:
        fieldnames = [
            "candidate_id",
            "trial_id",
            "train_score",
            "validation_score",
            "is_nominal",
            "is_fixed",
            "parameter_json",
        ]
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for trial in sorted(trials, key=lambda row: row.candidate_id):
            writer.writerow(
                {
                    "candidate_id": trial.candidate_id,
                    "trial_id": trial.trial_id,
                    "train_score": format_number(train.get(trial.key, math.inf)),
                    "validation_score": format_number(validation.get(trial.key, math.inf)),
                    "is_nominal": "true" if trial.is_nominal else "false",
                    "is_fixed": "true" if trial.is_fixed else "false",
                    "parameter_json": json.dumps(trial.parameters, sort_keys=True),
                }
            )


def write_score_rows(output_dir: Path, filename: str, rows: Sequence[ScoreRow]) -> None:
    with (output_dir / filename).open("w", newline="", encoding="utf-8") as handle:
        fieldnames = list(ScoreRow.__dataclass_fields__)
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            payload = dict(row.__dict__)
            for key, value in list(payload.items()):
                if isinstance(value, float):
                    payload[key] = format_number(value)
                elif isinstance(value, bool):
                    payload[key] = "true" if value else "false"
            writer.writerow(payload)


def write_bootstrap_rows(output_dir: Path, rows: Sequence[BootstrapRow]) -> None:
    with (output_dir / "source_log_bootstrap_confidence.csv").open("w", newline="", encoding="utf-8") as handle:
        fieldnames = list(BootstrapRow.__dataclass_fields__)
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            payload = dict(row.__dict__)
            for key, value in list(payload.items()):
                if isinstance(value, float):
                    payload[key] = format_number(value)
            writer.writerow(payload)


def write_tuned_parameters(
    output_dir: Path,
    selected_trials: Sequence[TrialSpec],
    covariance_path: Path,
) -> None:
    payload = {
        "schema_version": 1,
        "description": "Standalone traction residual-tail tuned candidate parameters. Not production configuration.",
        "covariance_policy": "fixed_testbed_covariance_not_tuned",
        "covariance_config": str(covariance_path),
        "candidate_specific_covariance_or_noise": False,
        "candidates": [
            {
                "id": trial.candidate_id,
                "trial_id": trial.trial_id,
                "model": trial.model,
                "parameters": trial.parameters,
            }
            for trial in selected_trials
        ],
    }
    (output_dir / "tuned_parameters.json").write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def bounded_indices(row_count: int, max_rows_per_segment: int) -> list[int]:
    if row_count <= 0:
        return []
    if max_rows_per_segment <= 0 or row_count <= max_rows_per_segment:
        return list(range(row_count))
    if max_rows_per_segment == 1:
        return [row_count // 2]
    return sorted(
        {
            int(round(index * (row_count - 1) / float(max_rows_per_segment - 1)))
            for index in range(max_rows_per_segment)
        }
    )


def write_report(
    output_dir: Path,
    summary: dict[str, Any],
    rows: Sequence[ScoreRow],
    bootstrap_rows: Sequence[BootstrapRow],
) -> None:
    lines = [
        "# Traction Residual-Tail Fair Tuning Report",
        "",
        f"- Trials evaluated: `{summary['trial_count']}`",
        f"- Train subset segments: `{summary['train_subset_segments']}`",
        f"- Validation subset segments: `{summary['validation_subset_segments']}`",
        f"- Boundary-ended active rows in train/validation: `{summary['train_subset_boundary_ended_segments']}` / `{summary['validation_subset_boundary_ended_segments']}`",
        f"- Final active-metric segments: `{summary['final_segments']}`",
        f"- Stress diagnostic segments: `{summary['stress_segments']}`",
        f"- Source logs: `{summary['source_log_count']}`",
        "- Split policy: reserve whole source logs for held-out first, then assign remaining whole segments to train/validation.",
        "- Selection policy: metric-balanced active traction rows first, using only production measurement residual-tail streams.",
        "- Main score policy: production-equivalent channels are yaw rate, forward accel, and right accel; encoder residuals are diagnostic only and excluded.",
        f"- Fixed noise schedule: `{summary.get('fixed_noise_schedule_path', '')}`; candidate-specific covariance/noise changes: `false`.",
        "- Boundary policy: active rows before explicit pickup/runoff/terminal external-force boundaries remain eligible for primary tuning; full-row stress diagnostics exclude boundary-corrupted segments.",
        "- Stress policy: full row-weighted residual tails are diagnostic-only.",
        "- Covariance policy: fixed standalone testbed covariance; candidate search contains physical model parameters only.",
        "- Logged UKF state policy: not consumed; data layer rejects/ignores `ukf_state*`, `logged_ukf_state*`, and replay-state columns.",
        "",
        "## Selected Trials",
        "",
        "| Candidate | Trial | Validation score |",
        "| --- | --- | ---: |",
    ]
    for item in summary["selected_trials"]:
        lines.append(
            f"| `{item['candidate_id']}` | `{item['trial_id']}` | {format_number(item['validation_score'])} |"
        )
    lines.extend(
        [
            "",
            "## Validation And Held-Out Residual Tail",
            "",
            "Full table: `validation_heldout_residual_tail.csv`.",
            "",
            "| Candidate | Split | Stage | yaw | forward accel | right accel |",
            "| --- | --- | --- | ---: | ---: | ---: |",
        ]
    )
    for line in compact_metric_lines(rows):
        lines.append(line)
    lines.extend(
        [
            "",
            "## Source-Log Bootstrap",
            "",
            "Full table: `source_log_bootstrap_confidence.csv`.",
            "",
            "| Candidate | Delta vs holdover | 95% CI | P(candidate better) | Logs |",
            "| --- | ---: | ---: | ---: | ---: |",
        ]
    )
    for row in bootstrap_rows:
        lines.append(
            "| "
            f"`{row.candidate_id}` | "
            f"{format_number(row.observed_delta_vs_baseline)} | "
            f"{format_number(row.ci_low)} .. {format_number(row.ci_high)} | "
            f"{format_number(row.probability_candidate_better)} | "
            f"{row.source_log_count} |"
        )
    lines.extend(
        [
            "",
            "## Limitations",
            "",
            "- This is a standalone residual-tail replay, not the production UKF path.",
            "- Candidate search is broad but finite Latin-hypercube sampling, so it is not a global optimum proof.",
            "- Small coverage buckets with fewer than two non-held-out segments cannot populate both train and validation.",
            "- Bootstrap confidence is over source logs, so it is only meaningful when enough held-out logs cover the active metrics.",
            "- Encoder wheel-rate residuals may still appear in diagnostic artifacts, but they are not production-equivalent NIS evidence.",
            "- Aggregate scoring does not emit per-row diagnostics for full-manifest selected trials.",
            "- Row caps are disabled for corrected runs unless explicitly supplied for a smoke-only command.",
        ]
    )
    (output_dir / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def compact_metric_lines(rows: Sequence[ScoreRow]) -> list[str]:
    grouped: dict[tuple[str, str, str], dict[str, float]] = {}
    for row in rows:
        key = (row.candidate_id, row.split, row.stage)
        grouped.setdefault(key, {})[row.log_parameter] = row.rms_nis
    lines: list[str] = []
    for (candidate_id, split, stage), values in sorted(grouped.items()):
        lines.append(
            "| "
            f"`{candidate_id}` | `{split}` | `{stage}` | "
            f"{format_number(values.get('yaw_rate_residual_tail', math.nan))} | "
            f"{format_number(values.get('forward_accel_residual_tail', math.nan))} | "
            f"{format_number(values.get('right_accel_residual_tail', math.nan))} |"
        )
    return lines


def metric_weights_from_config(scoring_config: dict[str, Any]) -> dict[str, float]:
    primary = dict(scoring_config.get("primary_metrics", {}))
    raw = primary.get("weights", DEFAULT_ACTIVE_METRIC_WEIGHTS)
    weights = {
        str(name): float(value)
        for name, value in dict(raw).items()
        if str(name) in DEFAULT_ACTIVE_METRIC_WEIGHTS and float(value) > 0.0
    }
    return weights or dict(DEFAULT_ACTIVE_METRIC_WEIGHTS)


def validate_primary_log_fields(scoring_config: dict[str, Any]) -> None:
    raw_fields = dict(scoring_config.get("log_fields", {}))
    invalid = [
        canonical_log_field(str(name))
        for name in raw_fields
        if canonical_log_field(str(name)) not in PRODUCTION_MEASUREMENT_RESIDUAL_TAIL_LOG_FIELDS
    ]
    if invalid:
        allowed = ", ".join(PRODUCTION_MEASUREMENT_RESIDUAL_TAIL_LOG_PARAMETERS)
        raise TuningError(
            "Production-equivalent tuning log_fields may only contain "
            f"{allowed}; got {', '.join(sorted(invalid))}"
        )


def primary_metric_scope(scoring_config: dict[str, Any]) -> str:
    primary = dict(scoring_config.get("primary_metrics", {}))
    value = str(primary.get("metric_scope", "active")).strip().lower()
    return value if value in ("active", "all") else "active"


def primary_row_weighted(scoring_config: dict[str, Any]) -> bool:
    primary = dict(scoring_config.get("primary_metrics", {}))
    return bool(primary.get("row_weighted", False))


def bootstrap_split_names(value: str) -> set[str]:
    splits = {item.strip() for item in value.split(",") if item.strip()}
    invalid = splits - set(SPLITS)
    if invalid:
        raise TuningError(f"Invalid bootstrap split(s): {', '.join(sorted(invalid))}")
    return splits or {"held_out"}


def parse_args(argv: Iterable[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(DEFAULT_CONFIG_PATH))
    parser.add_argument("--candidate-config", default=str(DEFAULT_CANDIDATE_PATH))
    parser.add_argument("--covariance-config", default=str(DEFAULT_COVARIANCE_PATH))
    parser.add_argument("--manifest", default="")
    parser.add_argument("--output-dir", default=str(DEFAULT_OUTPUT_DIR))
    parser.add_argument("--trial-count", type=int, default=None)
    parser.add_argument("--train-segments", type=int, default=None)
    parser.add_argument("--validation-segments", type=int, default=None)
    parser.add_argument("--validation-top-k", type=int, default=None)
    parser.add_argument("--launch-subset-fraction", type=float, default=None)
    parser.add_argument("--launch-weight", type=float, default=None)
    parser.add_argument("--tuning-max-rows-per-segment", type=int, default=None)
    parser.add_argument("--final-max-segments", type=int, default=0)
    parser.add_argument("--final-max-rows-per-segment", type=int, default=None)
    parser.add_argument("--stress-max-rows-per-segment", type=int, default=None)
    parser.add_argument("--bootstrap-iterations", type=int, default=None)
    parser.add_argument("--bootstrap-split", default="")
    return parser.parse_args(list(argv))


def main(argv: Iterable[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    config_path = Path(args.config).resolve()
    candidate_path = Path(args.candidate_config).resolve()
    covariance_path = Path(args.covariance_config).resolve()
    output_dir = Path(args.output_dir).resolve()
    config = load_json(config_path)
    candidate_config = load_json(candidate_path)
    manifest_path = (
        Path(args.manifest).resolve()
        if args.manifest
        else resolve_path(str(config.get("manifest_path", "")), config_path.parent)
    )
    if not manifest_path.exists():
        raise TuningError(f"Segment manifest not found: {manifest_path}")

    split_config = dict(config.get("split", {}))
    tuning_config = dict(config.get("tuning", {}))
    scoring_config = dict(config.get("scoring", {}))
    bootstrap_config = dict(scoring_config.get("bootstrap", {}))
    seed = str(tuning_config.get("seed", split_seed(split_config)))
    trial_count = (
        int(args.trial_count)
        if args.trial_count is not None
        else int(tuning_config.get("trial_count_per_candidate", 24))
    )
    args.trial_count = trial_count
    if args.train_segments is None:
        args.train_segments = int(tuning_config.get("train_segments", 140))
    if args.validation_segments is None:
        args.validation_segments = int(tuning_config.get("validation_segments", 90))
    if args.validation_top_k is None:
        args.validation_top_k = int(tuning_config.get("validation_top_k_per_candidate", 5))
    if args.launch_subset_fraction is None:
        args.launch_subset_fraction = float(tuning_config.get("launch_subset_fraction", 0.75))
    if args.tuning_max_rows_per_segment is None:
        args.tuning_max_rows_per_segment = int(tuning_config.get("tuning_max_rows_per_segment", 0))
    if args.final_max_rows_per_segment is None:
        args.final_max_rows_per_segment = int(tuning_config.get("final_max_rows_per_segment", 0))
    if args.launch_weight is None:
        args.launch_weight = float(scoring_config.get("launch_weight", 1.5))
    if args.stress_max_rows_per_segment is None:
        args.stress_max_rows_per_segment = int(scoring_config.get("stress_max_rows_per_segment", 0))
    bootstrap_iterations = (
        int(args.bootstrap_iterations)
        if args.bootstrap_iterations is not None
        else int(bootstrap_config.get("iterations", 0))
    )
    bootstrap_splits = bootstrap_split_names(
        args.bootstrap_split or str(bootstrap_config.get("split", "held_out"))
    )
    validate_primary_log_fields(scoring_config)
    metric_weights = metric_weights_from_config(scoring_config)
    metric_scope = primary_metric_scope(scoring_config)
    row_weighted_selection = primary_row_weighted(scoring_config)

    candidates = load_candidate_specs(config, candidate_config)
    trials = generate_trials(
        candidates=candidates,
        trial_count=trial_count,
        seed=seed,
        include_nominal=bool(tuning_config.get("include_nominal_trial", True)),
    )
    all_segments = load_manifest_segments(manifest_path, split_config)
    bias_source_manifest_path = resolve_existing_config_path(
        str(config.get("bias_source_manifest", "")) or str(manifest_path),
        config_path.parent,
    )
    bias_segments = load_manifest_segments(bias_source_manifest_path, split_config)
    clean_segments = [segment for segment in all_segments if not segment.corrupted]
    primary_segments = [
        segment
        for segment in all_segments
        if segment_in_metric_scope(segment, metric_scope, metric_weights)
    ]
    train_subset = select_tuning_subset(
        primary_segments,
        "train",
        args.train_segments,
        seed,
        args.launch_subset_fraction,
        include_corrupted=True,
    )
    validation_subset = select_tuning_subset(
        primary_segments,
        "validation",
        args.validation_segments,
        seed,
        args.launch_subset_fraction,
        include_corrupted=True,
    )

    vehicle, _covariance = load_covariance(covariance_path)
    sample_source = EvaluationSampleSource(manifest_path, vehicle, bias_segments)
    train_result = evaluate_trials(
        phase="train_screen",
        manifest_path=manifest_path,
        segment_metas=train_subset,
        trials=trials,
        vehicle=vehicle,
        covariance_path=covariance_path,
        max_rows_per_segment=args.tuning_max_rows_per_segment,
        metric_scope=metric_scope,
        metric_weights=metric_weights,
        sample_source=sample_source,
    )
    floor_ratio = float(scoring_config.get("inflation_floor_ratio", 0.75))
    under_expected_weight = float(scoring_config.get("under_expected_penalty_weight", 1.0))
    inflation_weight = float(scoring_config.get("inflation_floor_penalty_weight", 1.0))
    train_scores = rank_trials(
        phase="train_screen",
        result=train_result,
        split_name="train",
        floor_ratio=floor_ratio,
        under_expected_weight=under_expected_weight,
        inflation_weight=inflation_weight,
        launch_weight=args.launch_weight,
        metric_weights=metric_weights,
        row_weighted=row_weighted_selection,
    )
    validation_trials = top_trials_by_candidate(trials, train_scores, args.validation_top_k)
    validation_result = evaluate_trials(
        phase="validation_select",
        manifest_path=manifest_path,
        segment_metas=validation_subset,
        trials=validation_trials,
        vehicle=vehicle,
        covariance_path=covariance_path,
        max_rows_per_segment=args.tuning_max_rows_per_segment,
        metric_scope=metric_scope,
        metric_weights=metric_weights,
        sample_source=sample_source,
    )
    validation_scores = rank_trials(
        phase="validation_select",
        result=validation_result,
        split_name="validation",
        floor_ratio=floor_ratio,
        under_expected_weight=under_expected_weight,
        inflation_weight=inflation_weight,
        launch_weight=args.launch_weight,
        metric_weights=metric_weights,
        row_weighted=row_weighted_selection,
    )
    selected_trials = best_trials_by_candidate(
        validation_trials,
        validation_scores,
        [candidate.candidate_id for candidate in candidates],
    )
    final_segments = primary_segments
    if args.final_max_segments > 0:
        final_segments = sorted(
            final_segments,
            key=lambda segment: stable_hash_float(seed, "final", segment.coverage_key, segment.segment_id),
        )[: args.final_max_segments]
    final_result = evaluate_trials(
        phase="full_selected",
        manifest_path=manifest_path,
        segment_metas=final_segments,
        trials=selected_trials,
        vehicle=vehicle,
        covariance_path=covariance_path,
        max_rows_per_segment=args.final_max_rows_per_segment,
        metric_scope=metric_scope,
        metric_weights=metric_weights,
        sample_source=sample_source,
    )
    stress_result = evaluate_trials(
        phase="stress_full_rows",
        manifest_path=manifest_path,
        segment_metas=clean_segments,
        trials=selected_trials,
        vehicle=vehicle,
        covariance_path=covariance_path,
        max_rows_per_segment=args.stress_max_rows_per_segment,
        metric_scope="all",
        sample_source=sample_source,
    )
    bootstrap_rows = bootstrap_confidence_rows(
        result=final_result,
        selected_trials=selected_trials,
        baseline_candidate_id="baseline/current_holdover",
        split_names=bootstrap_splits,
        iterations=bootstrap_iterations,
        seed=seed,
        floor_ratio=floor_ratio,
        under_expected_weight=under_expected_weight,
        inflation_weight=inflation_weight,
        launch_weight=args.launch_weight,
        metric_weights=metric_weights,
        row_weighted=row_weighted_selection,
    )
    write_outputs(
        output_dir=output_dir,
        manifest_path=manifest_path,
        config_path=config_path,
        covariance_path=covariance_path,
        candidates=candidates,
        trials=trials,
        train_subset=train_subset,
        validation_subset=validation_subset,
        all_segments=all_segments,
        train_scores=train_scores,
        validation_scores=validation_scores,
        selected_trials=selected_trials,
        final_result=final_result,
        stress_result=stress_result,
        bootstrap_rows=bootstrap_rows,
        bias_source_manifest_path=bias_source_manifest_path,
        metric_scope=metric_scope,
        metric_weights=metric_weights,
        row_weighted_selection=row_weighted_selection,
        args=args,
        sample_source_stats=sample_source.stats(),
    )
    print(f"Wrote traction residual-tail tuning report to {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
