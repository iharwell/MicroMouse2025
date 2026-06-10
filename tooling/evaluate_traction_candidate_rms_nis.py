#!/usr/bin/env python3
"""Score first-round traction candidates from replay-exported NIS artifacts.

The evaluator intentionally consumes replay artifacts instead of logged UKF
state. Candidate model execution is external until OpenFloorUkfReplay exposes a
canonical PlantModel candidate hook.
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import hashlib
import json
import math
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable, Iterator


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG_PATH = (
    REPO_ROOT
    / "staging"
    / "traction_candidate_rms_nis_eval"
    / "candidate_eval_config.json"
)
DEFAULT_OUTPUT_DIR = (
    REPO_ROOT / "staging" / "traction_candidate_rms_nis_eval" / "last_run"
)
SPLITS = ("train", "validation", "held_out")
FORBIDDEN_UKF_COLUMN_PREFIXES = ("ukf_state", "logged_ukf_state")
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


class EvalConfigError(RuntimeError):
    pass


@dataclass(frozen=True)
class CandidateConfig:
    candidate_id: str
    label: str
    model: str
    enabled: bool
    parameters: dict[str, Any]
    search: dict[str, Any]


@dataclass
class SegmentInfo:
    segment_id: str
    split: str | None = None
    corrupted: bool = False
    stage: str = ""


@dataclass(frozen=True)
class RawNisRecord:
    candidate_id: str
    segment_id: str
    stage: str
    log_parameter: str
    nis: float
    measurement_dimension: int | None
    split: str | None
    corrupted: bool
    source_path: Path
    source_line: int


@dataclass(frozen=True)
class NisRecord:
    candidate_id: str
    segment_id: str
    split: str
    stage: str
    log_parameter: str
    nis: float
    measurement_dimension: int | None
    source_path: Path
    source_line: int


@dataclass
class NisStats:
    count: int = 0
    sum_square: float = 0.0
    segment_ids: set[str] = field(default_factory=set)

    def add(self, record: NisRecord) -> None:
        self.count += 1
        self.sum_square += record.nis * record.nis
        self.segment_ids.add(record.segment_id)

    @property
    def rms(self) -> float:
        if self.count == 0:
            return math.nan
        return math.sqrt(self.sum_square / self.count)


@dataclass(frozen=True)
class ParameterScoreSpec:
    expected_rms: float
    weight: float
    inflation_floor_ratio: float


@dataclass(frozen=True)
class ItemizedScore:
    candidate_id: str
    split: str
    stage: str
    log_parameter: str
    count: int
    segment_count: int
    rms_nis: float
    expected_rms_nis: float
    guarded_rms_nis: float
    normalized_guarded_score: float
    inflation_floor_rms_nis: float
    inflation_flag: bool


@dataclass(frozen=True)
class CandidateScore:
    candidate_id: str
    label: str
    model: str
    selection_split: str
    selection_score: float
    train_score: float
    validation_score: float
    held_out_score: float
    raw_train_rms_nis: float
    raw_validation_rms_nis: float
    raw_held_out_rms_nis: float
    sample_count: int
    segment_count: int
    inflation_bucket_count: int
    missing: bool


def timestamp() -> str:
    return dt.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")


def load_json(path: Path) -> dict[str, Any]:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise EvalConfigError(f"Invalid JSON in {path}: {exc}") from exc


def resolve_path(path_text: str, base_dir: Path) -> Path:
    path = Path(path_text)
    if path.is_absolute():
        return path
    return (base_dir / path).resolve()


def parse_bool(value: Any, default: bool = False) -> bool:
    if value is None or value == "":
        return default
    if isinstance(value, bool):
        return value
    text = str(value).strip().lower()
    if text in ("1", "true", "yes", "y", "corrupt", "corrupted"):
        return True
    if text in ("0", "false", "no", "n", "clean", "valid"):
        return False
    return default


def parse_float(value: Any, context: str) -> float:
    try:
        result = float(str(value).strip())
    except (TypeError, ValueError) as exc:
        raise EvalConfigError(f"Expected numeric value for {context}, got {value!r}") from exc
    if not math.isfinite(result):
        raise EvalConfigError(f"Expected finite value for {context}, got {value!r}")
    return result


def parse_optional_int(value: Any) -> int | None:
    if value is None or str(value).strip() == "":
        return None
    try:
        result = int(str(value).strip())
    except ValueError:
        return None
    return result if result > 0 else None


def normalize_name(value: Any) -> str:
    return str(value).strip()


def csv_value(row: dict[str, str], names: Iterable[str]) -> str:
    lower_to_name = {key.lower(): key for key in row}
    for name in names:
        key = lower_to_name.get(name.lower())
        if key is not None:
            value = row.get(key, "")
            if value is not None and str(value).strip() != "":
                return str(value).strip()
    return ""


def expected_rms_for_dimension(dimension: int) -> float:
    return math.sqrt(float(dimension) * float(dimension + 2))


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


def validate_no_covariance_tuning(candidates: Iterable[CandidateConfig]) -> None:
    for candidate in candidates:
        names = list(flatten_leaf_names(candidate.parameters))
        names.extend(flatten_leaf_names(candidate.search))
        for name in names:
            normalized = name.lower()
            if any(token in normalized for token in FORBIDDEN_TUNING_NAME_TOKENS):
                raise EvalConfigError(
                    "Candidate tuning must not tune covariance/noise inflation fields: "
                    f"{candidate.candidate_id}.{name}"
                )


def load_candidates(config: dict[str, Any]) -> list[CandidateConfig]:
    raw_candidates = config.get("candidates")
    if not isinstance(raw_candidates, list) or not raw_candidates:
        raise EvalConfigError("Config must contain a non-empty candidates list")
    candidates: list[CandidateConfig] = []
    seen: set[str] = set()
    for raw in raw_candidates:
        if not isinstance(raw, dict):
            raise EvalConfigError("Each candidate entry must be an object")
        candidate_id = normalize_name(raw.get("id", ""))
        if not candidate_id:
            raise EvalConfigError("Candidate id is required")
        if candidate_id in seen:
            raise EvalConfigError(f"Duplicate candidate id: {candidate_id}")
        seen.add(candidate_id)
        candidates.append(
            CandidateConfig(
                candidate_id=candidate_id,
                label=normalize_name(raw.get("label", candidate_id)),
                model=normalize_name(raw.get("model", "")),
                enabled=parse_bool(raw.get("enabled", True), default=True),
                parameters=dict(raw.get("parameters", {})),
                search=dict(raw.get("search", {})),
            )
        )
    validate_no_covariance_tuning(candidates)
    return candidates


def scoring_specs(config: dict[str, Any]) -> dict[str, ParameterScoreSpec]:
    scoring = dict(config.get("scoring", {}))
    default_floor = float(scoring.get("inflation_floor_ratio", 0.75))
    specs: dict[str, ParameterScoreSpec] = {}
    raw_specs = scoring.get("log_parameters", {})
    if not isinstance(raw_specs, dict):
        raise EvalConfigError("scoring.log_parameters must be an object")
    for name, raw in raw_specs.items():
        if not isinstance(raw, dict):
            raise EvalConfigError(f"scoring.log_parameters.{name} must be an object")
        dimension = int(raw.get("dimension", 1))
        if dimension <= 0:
            raise EvalConfigError(f"Invalid measurement dimension for {name}: {dimension}")
        expected_rms = float(raw.get("expected_rms", expected_rms_for_dimension(dimension)))
        if expected_rms <= 0.0 or not math.isfinite(expected_rms):
            raise EvalConfigError(f"Invalid expected RMS NIS for {name}: {expected_rms}")
        weight = float(raw.get("weight", 1.0))
        if weight <= 0.0 or not math.isfinite(weight):
            raise EvalConfigError(f"Invalid weight for {name}: {weight}")
        floor_ratio = float(raw.get("inflation_floor_ratio", default_floor))
        specs[normalize_name(name)] = ParameterScoreSpec(
            expected_rms=expected_rms,
            weight=weight,
            inflation_floor_ratio=floor_ratio,
        )
    return specs


def canonical_log_parameter(name: str, specs: dict[str, ParameterScoreSpec]) -> str:
    clean = normalize_name(name)
    if clean in specs:
        return clean
    if not clean.endswith("_nis") and f"{clean}_nis" in specs:
        return f"{clean}_nis"
    return clean


def load_manifest(path: Path | None) -> dict[str, Any]:
    if path is None:
        return {}
    if not path.exists():
        raise EvalConfigError(f"Manifest not found: {path}")
    manifest = load_json(path)
    if not isinstance(manifest, dict):
        raise EvalConfigError("Manifest root must be an object")
    return manifest


def manifest_segment_info(manifest: dict[str, Any]) -> dict[str, SegmentInfo]:
    result: dict[str, SegmentInfo] = {}
    raw_segments = manifest.get("segments", [])
    if raw_segments is None:
        return result
    if not isinstance(raw_segments, list):
        raise EvalConfigError("manifest.segments must be a list")
    for raw in raw_segments:
        if not isinstance(raw, dict):
            raise EvalConfigError("manifest.segments entries must be objects")
        segment_id = normalize_name(raw.get("segment_id", ""))
        if not segment_id:
            raise EvalConfigError("manifest.segments entry missing segment_id")
        split = normalize_name(raw.get("split", "")) or None
        if split is not None and split not in SPLITS:
            raise EvalConfigError(f"Invalid split for segment {segment_id}: {split}")
        result[segment_id] = SegmentInfo(
            segment_id=segment_id,
            split=split,
            corrupted=parse_bool(raw.get("corrupted", raw.get("is_corrupted", False))),
            stage=normalize_name(raw.get("stage", raw.get("stage_name", ""))),
        )
    return result


def add_artifact(
    artifacts: list[dict[str, Any]],
    raw: Any,
    defaults: dict[str, Any],
) -> None:
    if isinstance(raw, str):
        entry = {"path": raw}
    elif isinstance(raw, dict):
        entry = dict(raw)
    else:
        raise EvalConfigError("NIS artifact entries must be strings or objects")
    merged = dict(defaults)
    merged.update(entry)
    if not normalize_name(merged.get("path", "")):
        raise EvalConfigError("NIS artifact entry missing path")
    artifacts.append(merged)


def manifest_artifacts(manifest: dict[str, Any]) -> list[dict[str, Any]]:
    artifacts: list[dict[str, Any]] = []
    for key in ("artifacts", "nis_artifacts"):
        for raw in manifest.get(key, []) or []:
            add_artifact(artifacts, raw, {})
    if manifest.get("nis_csv"):
        add_artifact(artifacts, manifest["nis_csv"], {})
    for raw_candidate in manifest.get("candidates", []) or []:
        if not isinstance(raw_candidate, dict):
            raise EvalConfigError("manifest.candidates entries must be objects")
        defaults = {
            "candidate_id": raw_candidate.get("id", raw_candidate.get("candidate_id", "")),
        }
        if raw_candidate.get("nis_csv"):
            add_artifact(artifacts, raw_candidate["nis_csv"], defaults)
        for raw in raw_candidate.get("artifacts", []) or []:
            add_artifact(artifacts, raw, defaults)
        for raw in raw_candidate.get("nis_artifacts", []) or []:
            add_artifact(artifacts, raw, defaults)
    for raw_segment in manifest.get("segments", []) or []:
        if not isinstance(raw_segment, dict):
            continue
        defaults = {
            "segment_id": raw_segment.get("segment_id", ""),
            "stage": raw_segment.get("stage", raw_segment.get("stage_name", "")),
            "split": raw_segment.get("split", ""),
            "corrupted": raw_segment.get("corrupted", raw_segment.get("is_corrupted", False)),
        }
        for raw in raw_segment.get("artifacts", []) or []:
            add_artifact(artifacts, raw, defaults)
        for raw in raw_segment.get("nis_artifacts", []) or []:
            add_artifact(artifacts, raw, defaults)
    return artifacts


def validate_csv_header(path: Path, fieldnames: list[str] | None) -> list[str]:
    if not fieldnames:
        raise EvalConfigError(f"NIS CSV header missing: {path}")
    forbidden = [
        name
        for name in fieldnames
        if any(name.strip().lower().startswith(prefix) for prefix in FORBIDDEN_UKF_COLUMN_PREFIXES)
    ]
    if forbidden:
        raise EvalConfigError(
            f"NIS CSV must not contain logged UKF state columns: {path}: {', '.join(forbidden)}"
        )
    return fieldnames


def derive_segment_id(row: dict[str, str], defaults: dict[str, Any], path: Path, line_number: int) -> str:
    segment_id = csv_value(row, ("segment_id", "segment", "segment_key"))
    if not segment_id:
        segment_id = normalize_name(defaults.get("segment_id", ""))
    if segment_id:
        return segment_id
    run_id = csv_value(row, ("run_id",)) or normalize_name(defaults.get("run_id", ""))
    section_id = csv_value(row, ("section_id",))
    phase_id = csv_value(row, ("phase_id",))
    repeat_index = csv_value(row, ("repeat_index",))
    if run_id and section_id and phase_id and repeat_index:
        return f"{run_id}:section_{section_id}:phase_{phase_id}:repeat_{repeat_index}"
    raise EvalConfigError(f"Missing segment_id in {path}:{line_number}")


def derive_stage(row: dict[str, str], defaults: dict[str, Any], path: Path, line_number: int) -> str:
    stage = (
        csv_value(row, ("stage", "stage_name"))
        or normalize_name(defaults.get("stage", defaults.get("stage_name", "")))
    )
    if stage:
        return stage
    section_name = csv_value(row, ("section_name",))
    phase_name = csv_value(row, ("phase_name",))
    if section_name and phase_name:
        return f"{section_name}/{phase_name}"
    section_id = csv_value(row, ("section_id",))
    phase_id = csv_value(row, ("phase_id",))
    if section_id and phase_id:
        return f"section_{section_id}/phase_{phase_id}"
    raise EvalConfigError(f"Missing stage in {path}:{line_number}")


def row_corrupted(row: dict[str, str], defaults: dict[str, Any]) -> bool:
    return parse_bool(
        csv_value(row, ("corrupted", "is_corrupted", "segment_corrupted", "exclude"))
        or defaults.get("corrupted", defaults.get("is_corrupted", False))
    )


def row_split(row: dict[str, str], defaults: dict[str, Any], path: Path, line_number: int) -> str | None:
    split = csv_value(row, ("split", "dataset_split")) or normalize_name(defaults.get("split", ""))
    if not split:
        return None
    if split not in SPLITS:
        raise EvalConfigError(f"Invalid split {split!r} in {path}:{line_number}")
    return split


def make_raw_record(
    row: dict[str, str],
    defaults: dict[str, Any],
    path: Path,
    line_number: int,
    log_parameter: str,
    nis_value: Any,
    specs: dict[str, ParameterScoreSpec],
) -> RawNisRecord:
    candidate_id = (
        csv_value(row, ("candidate_id", "candidate", "model_id"))
        or normalize_name(defaults.get("candidate_id", ""))
    )
    if not candidate_id:
        raise EvalConfigError(f"Missing candidate_id in {path}:{line_number}")
    nis = parse_float(nis_value, f"{path}:{line_number}:{log_parameter}")
    if nis < 0.0:
        raise EvalConfigError(f"NIS must be non-negative in {path}:{line_number}:{log_parameter}")
    return RawNisRecord(
        candidate_id=candidate_id,
        segment_id=derive_segment_id(row, defaults, path, line_number),
        stage=derive_stage(row, defaults, path, line_number),
        log_parameter=canonical_log_parameter(log_parameter, specs),
        nis=nis,
        measurement_dimension=parse_optional_int(
            csv_value(row, ("measurement_dimension", "dimension", "dof"))
            or defaults.get("measurement_dimension")
        ),
        split=row_split(row, defaults, path, line_number),
        corrupted=row_corrupted(row, defaults),
        source_path=path,
        source_line=line_number,
    )


def read_nis_csv(
    path: Path,
    defaults: dict[str, Any],
    specs: dict[str, ParameterScoreSpec],
    included_log_parameters: set[str],
) -> list[RawNisRecord]:
    records: list[RawNisRecord] = []
    if not path.exists():
        raise EvalConfigError(f"NIS CSV not found: {path}")
    with path.open(newline="", encoding="utf-8-sig") as csv_file:
        reader = csv.DictReader(csv_file)
        fieldnames = validate_csv_header(path, reader.fieldnames)
        lower_fields = {name.lower(): name for name in fieldnames}
        has_long_shape = (
            "nis" in lower_fields
            and ("log_parameter" in lower_fields or normalize_name(defaults.get("log_parameter", "")))
        )
        if has_long_shape:
            for line_number, row in enumerate(reader, start=2):
                log_parameter = (
                    csv_value(row, ("log_parameter", "parameter", "measurement"))
                    or normalize_name(defaults.get("log_parameter", ""))
                )
                canonical = canonical_log_parameter(log_parameter, specs)
                if included_log_parameters and canonical not in included_log_parameters:
                    continue
                records.append(
                    make_raw_record(
                        row,
                        defaults,
                        path,
                        line_number,
                        canonical,
                        csv_value(row, ("nis",)),
                        specs,
                    )
                )
            return records

        nis_columns = [
            name
            for name in fieldnames
            if name.strip().lower().endswith("_nis")
            and name.strip().lower() != "last_update_nis"
        ]
        if not nis_columns:
            raise EvalConfigError(
                f"NIS CSV must contain long columns log_parameter,nis or wide *_nis columns: {path}"
            )
        for line_number, row in enumerate(reader, start=2):
            for column in nis_columns:
                canonical = canonical_log_parameter(column, specs)
                if included_log_parameters and canonical not in included_log_parameters:
                    continue
                value = row.get(column, "")
                if value is None or str(value).strip() == "":
                    continue
                records.append(
                    make_raw_record(row, defaults, path, line_number, canonical, value, specs)
                )
    return records


def stable_split(segment_id: str, split_config: dict[str, Any]) -> str:
    seed = str(split_config.get("seed", "traction_candidate_rms_nis_eval_v1"))
    ratios = [
        float(split_config.get("train", 0.60)),
        float(split_config.get("validation", 0.20)),
        float(split_config.get("held_out", 0.20)),
    ]
    if any(ratio < 0.0 or not math.isfinite(ratio) for ratio in ratios) or sum(ratios) <= 0.0:
        raise EvalConfigError("Split ratios must be finite non-negative values with positive sum")
    total = sum(ratios)
    normalized = [ratio / total for ratio in ratios]
    digest = hashlib.sha256(f"{seed}:{segment_id}".encode("utf-8")).digest()
    value = int.from_bytes(digest[:8], "big") / float(1 << 64)
    if value < normalized[0]:
        return "train"
    if value < normalized[0] + normalized[1]:
        return "validation"
    return "held_out"


def merge_segment_record(info: SegmentInfo, record: RawNisRecord) -> None:
    if record.split is not None:
        if info.split is not None and info.split != record.split:
            raise EvalConfigError(
                f"Segment {record.segment_id} has conflicting splits: {info.split} and {record.split}"
            )
        info.split = record.split
    if record.corrupted:
        info.corrupted = True
    if not info.stage:
        info.stage = record.stage


def finalized_records(
    raw_records: list[RawNisRecord],
    segments: dict[str, SegmentInfo],
    split_config: dict[str, Any],
) -> list[NisRecord]:
    for record in raw_records:
        info = segments.setdefault(record.segment_id, SegmentInfo(segment_id=record.segment_id))
        merge_segment_record(info, record)
    for info in segments.values():
        if info.split is None:
            info.split = stable_split(info.segment_id, split_config)
    records: list[NisRecord] = []
    for record in raw_records:
        info = segments[record.segment_id]
        if info.corrupted:
            continue
        assert info.split is not None
        records.append(
            NisRecord(
                candidate_id=record.candidate_id,
                segment_id=record.segment_id,
                split=info.split,
                stage=record.stage,
                log_parameter=record.log_parameter,
                nis=record.nis,
                measurement_dimension=record.measurement_dimension,
                source_path=record.source_path,
                source_line=record.source_line,
            )
        )
    return records


def load_records(
    config: dict[str, Any],
    config_path: Path,
    manifest_path: Path | None,
) -> tuple[list[NisRecord], dict[str, SegmentInfo]]:
    manifest = load_manifest(manifest_path)
    manifest_dir = manifest_path.parent if manifest_path is not None else config_path.parent
    specs = scoring_specs(config)
    included = set(specs)
    raw_records: list[RawNisRecord] = []
    for artifact in manifest_artifacts(manifest):
        path = resolve_path(str(artifact["path"]), manifest_dir)
        raw_records.extend(read_nis_csv(path, artifact, specs, included))
    segments = manifest_segment_info(manifest)
    records = finalized_records(raw_records, segments, dict(config.get("split", {})))
    return records, segments


def spec_for_parameter(
    log_parameter: str,
    stats_dimension: int | None,
    specs: dict[str, ParameterScoreSpec],
) -> ParameterScoreSpec:
    spec = specs.get(log_parameter)
    if spec is not None:
        return spec
    dimension = stats_dimension or 1
    return ParameterScoreSpec(
        expected_rms=expected_rms_for_dimension(dimension),
        weight=1.0,
        inflation_floor_ratio=0.75,
    )


def evaluate_records(
    config: dict[str, Any],
    candidates: list[CandidateConfig],
    records: list[NisRecord],
) -> tuple[list[ItemizedScore], list[CandidateScore]]:
    specs = scoring_specs(config)
    candidate_map = {candidate.candidate_id: candidate for candidate in candidates if candidate.enabled}
    for record in records:
        if record.candidate_id not in candidate_map:
            candidate_map[record.candidate_id] = CandidateConfig(
                candidate_id=record.candidate_id,
                label=record.candidate_id,
                model="manifest_only",
                enabled=True,
                parameters={},
                search={},
            )

    stats_by_item: dict[tuple[str, str, str, str], NisStats] = {}
    dimension_by_item: dict[tuple[str, str, str, str], int | None] = {}
    raw_stats_by_split: dict[tuple[str, str], NisStats] = {}
    for record in records:
        key = (record.candidate_id, record.split, record.stage, record.log_parameter)
        stats_by_item.setdefault(key, NisStats()).add(record)
        if key not in dimension_by_item and record.measurement_dimension:
            dimension_by_item[key] = record.measurement_dimension
        raw_stats_by_split.setdefault((record.candidate_id, record.split), NisStats()).add(record)

    itemized: list[ItemizedScore] = []
    for key, stats in sorted(stats_by_item.items()):
        candidate_id, split, stage, log_parameter = key
        spec = spec_for_parameter(log_parameter, dimension_by_item.get(key), specs)
        rms = stats.rms
        guarded = max(rms, spec.expected_rms)
        inflation_floor = spec.expected_rms * spec.inflation_floor_ratio
        itemized.append(
            ItemizedScore(
                candidate_id=candidate_id,
                split=split,
                stage=stage,
                log_parameter=log_parameter,
                count=stats.count,
                segment_count=len(stats.segment_ids),
                rms_nis=rms,
                expected_rms_nis=spec.expected_rms,
                guarded_rms_nis=guarded,
                normalized_guarded_score=guarded / spec.expected_rms,
                inflation_floor_rms_nis=inflation_floor,
                inflation_flag=rms < inflation_floor,
            )
        )

    selection_split = str(config.get("scoring", {}).get("selection_split", "validation"))
    if selection_split not in SPLITS:
        raise EvalConfigError(f"Invalid scoring.selection_split: {selection_split}")

    itemized_by_candidate_split: dict[tuple[str, str], list[ItemizedScore]] = {}
    for item in itemized:
        itemized_by_candidate_split.setdefault((item.candidate_id, item.split), []).append(item)

    def split_score(candidate_id: str, split: str) -> float:
        rows = itemized_by_candidate_split.get((candidate_id, split), [])
        numerator = 0.0
        denominator = 0.0
        for item in rows:
            spec = specs.get(item.log_parameter, ParameterScoreSpec(item.expected_rms_nis, 1.0, 0.75))
            weight = spec.weight * item.count
            numerator += weight * item.normalized_guarded_score
            denominator += weight
        return numerator / denominator if denominator > 0.0 else math.inf

    candidate_scores: list[CandidateScore] = []
    for candidate_id, candidate in sorted(candidate_map.items()):
        train_score = split_score(candidate_id, "train")
        validation_score = split_score(candidate_id, "validation")
        held_out_score = split_score(candidate_id, "held_out")
        raw_train = raw_stats_by_split.get((candidate_id, "train"), NisStats()).rms
        raw_validation = raw_stats_by_split.get((candidate_id, "validation"), NisStats()).rms
        raw_held_out = raw_stats_by_split.get((candidate_id, "held_out"), NisStats()).rms
        candidate_records = [record for record in records if record.candidate_id == candidate_id]
        selection_score = {
            "train": train_score,
            "validation": validation_score,
            "held_out": held_out_score,
        }[selection_split]
        candidate_scores.append(
            CandidateScore(
                candidate_id=candidate_id,
                label=candidate.label,
                model=candidate.model,
                selection_split=selection_split,
                selection_score=selection_score,
                train_score=train_score,
                validation_score=validation_score,
                held_out_score=held_out_score,
                raw_train_rms_nis=raw_train,
                raw_validation_rms_nis=raw_validation,
                raw_held_out_rms_nis=raw_held_out,
                sample_count=len(candidate_records),
                segment_count=len({record.segment_id for record in candidate_records}),
                inflation_bucket_count=sum(
                    1
                    for item in itemized
                    if item.candidate_id == candidate_id and item.inflation_flag
                ),
                missing=len(candidate_records) == 0,
            )
        )
    return itemized, candidate_scores


def format_number(value: float) -> str:
    if math.isinf(value):
        return "inf"
    if math.isnan(value):
        return ""
    return f"{value:.12g}"


def write_candidate_configs(output_dir: Path, candidates: list[CandidateConfig]) -> None:
    config_dir = output_dir / "candidate_configs"
    config_dir.mkdir(parents=True, exist_ok=True)
    for candidate in candidates:
        payload = {
            "schema_version": 1,
            "candidate_id": candidate.candidate_id,
            "label": candidate.label,
            "model": candidate.model,
            "parameters": candidate.parameters,
            "search": candidate.search,
            "covariance_policy": "fixed_production_estimator_covariance",
        }
        (config_dir / f"{candidate.candidate_id}.json").write_text(
            json.dumps(payload, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )


def write_hook_gap_report(output_dir: Path) -> None:
    lines = [
        "# Traction Candidate Replay Hook Gap",
        "",
        "The Python driver is ready to score replay-exported NIS artifacts, but the current replay surface does not yet execute candidate PlantModel profiles or export NIS CSV directly.",
        "",
        "Smallest safe code hook:",
        "",
        "1. `PlantModel`: add one canonical candidate/profile input at construction or through a narrowly scoped replay-only profile setter. The default production path must keep using the current built-in profile. The profile should carry traction-model kind plus bounded physical parameters only; it must not carry estimator covariance, process-noise, measurement-noise, or NIS scaling values.",
        "2. `OpenFloorUkfReplay`: add CLI options equivalent to `--plant-candidate-config <json>` and `--nis-csv <path>`. For each replayed update, write `candidate_id`, `run_id`, `segment_id`, `split` when known, `stage`, `log_parameter`, `measurement_dimension`, `nis`, `accepted`, and corruption metadata.",
        "3. `OpenFloorUkfReplay`: use the existing `Estimator::LastYawRateNis()`, `LastForwardAccelNis()`, and `LastRightAccelNis()` accessors after `updateYawRate(...)` and `updatePlanarAccel(...)`. Do not export or consume logged `ukf_state_*` columns for this scoring path.",
        "",
        "Until that hook lands, produce the NIS CSV artifacts externally and list them in the manifest consumed by this driver.",
    ]
    (output_dir / "hook_gap.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_outputs(
    output_dir: Path,
    candidates: list[CandidateConfig],
    segments: dict[str, SegmentInfo],
    itemized: list[ItemizedScore],
    candidate_scores: list[CandidateScore],
) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    write_candidate_configs(output_dir, candidates)

    with (output_dir / "itemized_rms_nis.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=[
                "candidate_id",
                "split",
                "stage",
                "log_parameter",
                "count",
                "segment_count",
                "rms_nis",
                "expected_rms_nis",
                "guarded_rms_nis",
                "normalized_guarded_score",
                "inflation_floor_rms_nis",
                "inflation_flag",
            ],
        )
        writer.writeheader()
        for item in itemized:
            writer.writerow(
                {
                    "candidate_id": item.candidate_id,
                    "split": item.split,
                    "stage": item.stage,
                    "log_parameter": item.log_parameter,
                    "count": item.count,
                    "segment_count": item.segment_count,
                    "rms_nis": format_number(item.rms_nis),
                    "expected_rms_nis": format_number(item.expected_rms_nis),
                    "guarded_rms_nis": format_number(item.guarded_rms_nis),
                    "normalized_guarded_score": format_number(item.normalized_guarded_score),
                    "inflation_floor_rms_nis": format_number(item.inflation_floor_rms_nis),
                    "inflation_flag": "true" if item.inflation_flag else "false",
                }
            )

    ranked = sorted(
        candidate_scores,
        key=lambda score: (
            math.inf if score.missing else score.selection_score,
            score.candidate_id,
        ),
    )
    with (output_dir / "candidate_scores.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=[
                "rank",
                "candidate_id",
                "label",
                "model",
                "selection_split",
                "selection_score",
                "train_score",
                "validation_score",
                "held_out_score",
                "raw_train_rms_nis",
                "raw_validation_rms_nis",
                "raw_held_out_rms_nis",
                "sample_count",
                "segment_count",
                "inflation_bucket_count",
                "missing",
            ],
        )
        writer.writeheader()
        for rank, score in enumerate(ranked, start=1):
            writer.writerow(
                {
                    "rank": rank if math.isfinite(score.selection_score) else "",
                    "candidate_id": score.candidate_id,
                    "label": score.label,
                    "model": score.model,
                    "selection_split": score.selection_split,
                    "selection_score": format_number(score.selection_score),
                    "train_score": format_number(score.train_score),
                    "validation_score": format_number(score.validation_score),
                    "held_out_score": format_number(score.held_out_score),
                    "raw_train_rms_nis": format_number(score.raw_train_rms_nis),
                    "raw_validation_rms_nis": format_number(score.raw_validation_rms_nis),
                    "raw_held_out_rms_nis": format_number(score.raw_held_out_rms_nis),
                    "sample_count": score.sample_count,
                    "segment_count": score.segment_count,
                    "inflation_bucket_count": score.inflation_bucket_count,
                    "missing": "true" if score.missing else "false",
                }
            )

    with (output_dir / "segment_splits.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=["segment_id", "split", "corrupted", "stage"])
        writer.writeheader()
        for segment_id, info in sorted(segments.items()):
            writer.writerow(
                {
                    "segment_id": segment_id,
                    "split": info.split or "",
                    "corrupted": "true" if info.corrupted else "false",
                    "stage": info.stage,
                }
            )

    summary = {
        "generated": timestamp(),
        "selection_order": [score.candidate_id for score in ranked],
        "candidate_scores": [score.__dict__ for score in ranked],
        "itemized": [item.__dict__ for item in itemized],
    }
    (output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True, default=str) + "\n",
        encoding="utf-8",
    )

    lines = [
        "# Traction Candidate RMS NIS Evaluation",
        "",
        f"- Generated: `{timestamp()}`",
        "- Ranking uses validation split by default; held-out rows are reported but should not be used for candidate selection.",
        "- Raw RMS NIS is reported directly. Ranking uses guarded RMS NIS, floored at expected chi-square RMS, so covariance inflation cannot improve a score below calibrated expectation.",
        "- Logged `ukf_state_*` columns are rejected at CSV load time.",
        "",
        "## Candidate Ranking",
        "",
        "| Rank | Candidate | Selection score | Train | Validation | Held-out | Samples | Inflation flags |",
        "| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for rank, score in enumerate(ranked, start=1):
        rank_text = str(rank) if math.isfinite(score.selection_score) else ""
        lines.append(
            "| "
            f"{rank_text} | `{score.candidate_id}` | {format_number(score.selection_score)} | "
            f"{format_number(score.train_score)} | {format_number(score.validation_score)} | "
            f"{format_number(score.held_out_score)} | {score.sample_count} | "
            f"{score.inflation_bucket_count} |"
        )
    (output_dir / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def run_replay_commands(
    config: dict[str, Any],
    candidates: list[CandidateConfig],
    output_dir: Path,
) -> None:
    replay = dict(config.get("replay", {}))
    if not parse_bool(replay.get("candidate_hook_available", False)):
        write_hook_gap_report(output_dir)
        raise EvalConfigError(
            "Replay candidate hook is not available; wrote hook_gap.md with the required C++ surface."
        )
    template = replay.get("command_template")
    if not isinstance(template, list) or not template:
        raise EvalConfigError("replay.command_template must be a non-empty list when --run-replay is used")
    for candidate in candidates:
        if not candidate.enabled:
            continue
        candidate_config = output_dir / "candidate_configs" / f"{candidate.candidate_id}.json"
        nis_csv = output_dir / "replay" / candidate.candidate_id / "nis_samples.csv"
        candidate_output = output_dir / "replay" / candidate.candidate_id
        candidate_output.mkdir(parents=True, exist_ok=True)
        replacements = {
            "candidate_id": candidate.candidate_id,
            "candidate_config": str(candidate_config),
            "nis_csv": str(nis_csv),
            "output": str(candidate_output),
            "repo_root": str(REPO_ROOT),
        }
        command = [str(part).format(**replacements) for part in template]
        completed = subprocess.run(
            command,
            cwd=REPO_ROOT,
            text=True,
            capture_output=True,
            check=False,
        )
        (candidate_output / "stdout.txt").write_text(completed.stdout, encoding="utf-8", errors="replace")
        (candidate_output / "stderr.txt").write_text(completed.stderr, encoding="utf-8", errors="replace")
        if completed.returncode != 0:
            raise EvalConfigError(
                f"Replay command failed for {candidate.candidate_id} with exit code {completed.returncode}"
            )


def parse_args(argv: Iterable[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(DEFAULT_CONFIG_PATH))
    parser.add_argument("--manifest", default="")
    parser.add_argument("--output-dir", default="")
    parser.add_argument("--run-replay", action="store_true")
    parser.add_argument("--write-hook-gap", action="store_true")
    return parser.parse_args(list(argv))


def main(argv: Iterable[str]) -> int:
    args = parse_args(argv)
    config_path = Path(args.config).resolve()
    config = load_json(config_path)
    candidates = load_candidates(config)

    output_dir = Path(args.output_dir).resolve() if args.output_dir else DEFAULT_OUTPUT_DIR
    output_dir.mkdir(parents=True, exist_ok=True)
    write_candidate_configs(output_dir, candidates)

    if args.write_hook_gap:
        write_hook_gap_report(output_dir)

    if args.run_replay:
        try:
            run_replay_commands(config, candidates, output_dir)
        except EvalConfigError:
            raise

    manifest_text = args.manifest or str(config.get("manifest_path", ""))
    manifest_path = resolve_path(manifest_text, config_path.parent) if manifest_text else None
    records, segments = load_records(config, config_path, manifest_path)
    itemized, candidate_scores = evaluate_records(config, candidates, records)
    write_outputs(output_dir, candidates, segments, itemized, candidate_scores)
    if not records:
        write_hook_gap_report(output_dir)
        print(f"No NIS records found. Wrote scaffolding and hook gap report to {output_dir}", file=sys.stderr)
        return 2
    print(f"Wrote traction candidate RMS NIS evaluation to {output_dir}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv[1:]))
    except EvalConfigError as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(1)
