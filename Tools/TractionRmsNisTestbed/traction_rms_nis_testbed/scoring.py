"""Standalone accepted-ANIS scoring helpers for replay artifacts."""

from __future__ import annotations

import csv
import hashlib
import json
import math
import argparse
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable


SPLITS = ("train", "validation", "held_out")
REPORT_SPLITS = (*SPLITS, "all")
REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_CONFIG_PATH = REPO_ROOT / "staging" / "traction_candidate_rms_nis_testbed" / "config.example.json"
DEFAULT_OUTPUT_DIR = REPO_ROOT / "staging" / "traction_candidate_rms_nis_testbed" / "last_run"
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
COMMAND_BIN_WIDTH = 0.02


class TestbedConfigError(RuntimeError):
    pass


@dataclass(frozen=True)
class CandidateConfig:
    candidate_id: str
    label: str
    model: str
    parameters: dict[str, Any]
    search: dict[str, Any]


@dataclass(frozen=True)
class TrialConfig:
    trial_id: str
    candidate_id: str
    parameters: dict[str, float]
    is_nominal: bool


@dataclass
class SegmentInfo:
    segment_id: str
    split: str | None = None
    corrupted: bool = False
    stage: str = ""
    parameter_fields: dict[str, Any] = field(default_factory=dict)
    observed_command: dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class NisRecord:
    candidate_id: str
    trial_id: str
    segment_id: str
    split: str
    stage: str
    log_field: str
    nis: float
    accepted: bool
    measurement_dimension: int | None
    command_bucket: str
    parameter_fields: dict[str, Any]
    observed_command: dict[str, Any]


@dataclass(frozen=True)
class ItemizedScore:
    candidate_id: str
    trial_id: str
    split: str
    stage: str
    log_field: str
    count: int
    accepted_count: int
    finite_count: int
    nonfinite_count: int
    rejected_count: int
    rejected_rate: float
    rejected_rate_penalty: float
    segment_count: int
    score_partition: str
    stage_weight: float
    channel_weight: float
    selection_weight: float
    rms_nis: float
    accepted_only_rms_nis: float
    expected_rms_nis: float
    guarded_rms_nis: float
    under_expected_penalty: float
    inflation_flag: bool
    parameter_field: str
    parameter_value_kind: str
    parameter_value: str
    launch_command_signature: str
    launch_cmd_linear_mps_median_mean: float


@dataclass(frozen=True)
class TrialScore:
    candidate_id: str
    trial_id: str
    selection_score: float
    sample_count: int


@dataclass(frozen=True)
class CandidateScore:
    candidate_id: str
    selection_score: float
    sample_count: int


@dataclass
class SegmentScoreStats:
    total_count: int = 0
    finite_count: int = 0
    accepted_count: int = 0
    rejected_count: int = 0
    sum_nis_sq: float = 0.0
    accepted_sum_nis_sq: float = 0.0

    def add(self, record: NisRecord) -> None:
        self.total_count += 1
        if math.isfinite(record.nis):
            self.finite_count += 1
            self.sum_nis_sq += record.nis * record.nis
        if record.accepted:
            self.accepted_count += 1
            self.accepted_sum_nis_sq += record.nis * record.nis
        else:
            self.rejected_count += 1

    def rms_nis(self) -> float:
        if self.finite_count <= 0:
            return math.nan
        return math.sqrt(self.sum_nis_sq / self.finite_count)

    def accepted_only_rms_nis(self) -> float:
        if self.accepted_count <= 0:
            return math.nan
        return math.sqrt(self.accepted_sum_nis_sq / self.accepted_count)

    def rejected_rate(self) -> float:
        if self.total_count <= 0:
            return 0.0
        return self.rejected_count / self.total_count


def load_candidates(config: dict[str, Any]) -> list[CandidateConfig]:
    candidates: list[CandidateConfig] = []
    seen: set[str] = set()
    for raw in config.get("candidates", []):
        if not isinstance(raw, dict):
            raise TestbedConfigError("Candidate entries must be objects")
        candidate_id = str(raw.get("id", raw.get("candidate_id", ""))).strip()
        if not candidate_id:
            raise TestbedConfigError("Candidate id is required")
        if candidate_id in seen:
            raise TestbedConfigError(f"Duplicate candidate id: {candidate_id}")
        seen.add(candidate_id)
        candidate = CandidateConfig(
            candidate_id=candidate_id,
            label=str(raw.get("label", candidate_id)),
            model=str(raw.get("model", "")),
            parameters=dict(raw.get("parameters", {})),
            search=dict(raw.get("search", {})),
        )
        validate_no_covariance_tuning(candidate)
        candidates.append(candidate)
    if not candidates:
        raise TestbedConfigError("Config must contain at least one candidate")
    return candidates


def validate_no_covariance_tuning(candidate: CandidateConfig) -> None:
    for name in list(flatten_leaf_names(candidate.parameters)) + list(flatten_leaf_names(candidate.search)):
        lowered = name.lower()
        if any(token in lowered for token in FORBIDDEN_TUNING_NAME_TOKENS):
            raise TestbedConfigError(
                "Candidate tuning must not include covariance/noise fields: "
                f"{candidate.candidate_id}.{name}"
            )


def flatten_leaf_names(value: Any, prefix: str = "") -> Iterable[str]:
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


def generate_trial_plan(config: dict[str, Any], candidates: list[CandidateConfig]) -> list[TrialConfig]:
    tuning = dict(config.get("tuning", {}))
    trial_count = max(0, int(tuning.get("trial_count_per_candidate", 0)))
    include_nominal = bool(tuning.get("include_nominal_trial", True))
    trials: list[TrialConfig] = []
    for candidate in candidates:
        if include_nominal:
            trials.append(
                TrialConfig(
                    trial_id=f"{candidate.candidate_id}:nominal",
                    candidate_id=candidate.candidate_id,
                    parameters={key: float(value) for key, value in candidate.parameters.items()},
                    is_nominal=True,
                )
            )
        searchable = sorted(candidate.search.items())
        for index in range(trial_count):
            params = {key: float(value) for key, value in candidate.parameters.items()}
            for name, bounds in searchable:
                if not isinstance(bounds, dict):
                    continue
                params[name] = trial_value(bounds, index, trial_count)
            trials.append(
                TrialConfig(
                    trial_id=f"{candidate.candidate_id}:trial_{index + 1:03d}",
                    candidate_id=candidate.candidate_id,
                    parameters=params,
                    is_nominal=False,
                )
            )
    return trials


def trial_value(bounds: dict[str, Any], index: int, count: int) -> float:
    lower = float(bounds.get("min", 0.0))
    upper = float(bounds.get("max", lower))
    if count <= 1:
        fraction = 0.5
    else:
        fraction = index / float(count - 1)
    if str(bounds.get("scale", "linear")).lower() == "log" and lower > 0.0 and upper > 0.0:
        return math.exp(math.log(lower) + fraction * (math.log(upper) - math.log(lower)))
    return lower + fraction * (upper - lower)


def load_records(
    config: dict[str, Any],
    config_path: Path,
    manifest_path: Path,
    trials: list[TrialConfig],
    extra_artifacts: list[dict[str, Any]] | None = None,
) -> tuple[list[NisRecord], dict[str, SegmentInfo]]:
    del trials
    manifest = load_json(manifest_path)
    manifest_dir = manifest_path.parent
    segments = manifest_segments(manifest, dict(config.get("split", {})))
    raw_records: list[dict[str, Any]] = []
    for artifact in [*manifest_artifacts(manifest), *(extra_artifacts or [])]:
        path = resolve_path(str(artifact["path"]), manifest_dir)
        raw_records.extend(read_nis_csv(path, artifact))
    for raw in raw_records:
        info = segments.setdefault(raw["segment_id"], SegmentInfo(segment_id=raw["segment_id"]))
        merge_record_info(info, raw)
    for info in segments.values():
        if info.split is None:
            info.split = stable_split(info.segment_id, dict(config.get("split", {})))
    records: list[NisRecord] = []
    for raw in raw_records:
        info = segments[raw["segment_id"]]
        if info.corrupted:
            continue
        records.append(
            NisRecord(
                candidate_id=raw["candidate_id"],
                trial_id=raw["trial_id"],
                segment_id=raw["segment_id"],
                split=info.split or stable_split(info.segment_id, dict(config.get("split", {}))),
                stage=raw["stage"] or info.stage,
                log_field=canonical_log_field(raw["log_field"]),
                nis=raw["nis"],
                accepted=raw["accepted"],
                measurement_dimension=raw["measurement_dimension"],
                command_bucket=record_command_bucket(raw, info),
                parameter_fields=info.parameter_fields,
                observed_command=info.observed_command,
            )
        )
    del config_path
    return records, segments


def load_json(path: Path) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise TestbedConfigError(f"Invalid JSON in {path}: {exc}") from exc
    if not isinstance(payload, dict):
        raise TestbedConfigError(f"JSON root must be an object: {path}")
    return payload


def manifest_segments(manifest: dict[str, Any], split_config: dict[str, Any]) -> dict[str, SegmentInfo]:
    result: dict[str, SegmentInfo] = {}
    for raw in manifest.get("segments", []) or []:
        if not isinstance(raw, dict):
            continue
        segment_id = str(raw.get("segment_id", "")).strip()
        if not segment_id:
            continue
        split = str(raw.get("split", "")).strip() or None
        if split and split not in SPLITS:
            raise TestbedConfigError(f"Invalid split for {segment_id}: {split}")
        result[segment_id] = SegmentInfo(
            segment_id=segment_id,
            split=split or stable_split(segment_id, split_config),
            corrupted=is_terminal_external_force_corruption(raw),
            stage=str(raw.get("stage", "")),
            parameter_fields=dict(raw.get("parameter_fields") or {}),
            observed_command=dict(raw.get("observed_command") or {}),
        )
    return result


def manifest_artifacts(manifest: dict[str, Any]) -> list[dict[str, Any]]:
    artifacts: list[dict[str, Any]] = []
    for raw in manifest.get("artifacts", []) or []:
        artifacts.append(raw if isinstance(raw, dict) else {"path": raw})
    for raw in manifest.get("nis_artifacts", []) or []:
        artifacts.append(raw if isinstance(raw, dict) else {"path": raw})
    return artifacts


def resolve_path(path_text: str, base_dir: Path) -> Path:
    path = Path(path_text)
    return path if path.is_absolute() else (base_dir / path).resolve()


def read_nis_csv(path: Path, defaults: dict[str, Any]) -> list[dict[str, Any]]:
    if not path.exists():
        raise TestbedConfigError(f"NIS CSV not found: {path}")
    records: list[dict[str, Any]] = []
    with path.open(newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        if not reader.fieldnames:
            raise TestbedConfigError(f"NIS CSV header missing: {path}")
        forbidden = [
            name
            for name in reader.fieldnames
            if any(name.lower().startswith(prefix) for prefix in FORBIDDEN_UKF_COLUMN_PREFIXES)
        ]
        if forbidden:
            raise TestbedConfigError(
                f"NIS CSV must not contain logged UKF state columns: {path}: {', '.join(forbidden)}"
            )
        field_lookup = {name.lower(): name for name in reader.fieldnames}
        has_long = "nis" in field_lookup and (
            "log_field" in field_lookup or "log_parameter" in field_lookup
        )
        for row in reader:
            if has_long:
                records.append(make_record(row, defaults, path))
            else:
                for field in reader.fieldnames:
                    if not field.lower().endswith("_nis") or field.lower() == "last_update_nis":
                        continue
                    value = row.get(field, "")
                    if str(value).strip():
                        wide = dict(row)
                        wide["log_field"] = field
                        wide["nis"] = value
                        records.append(make_record(wide, defaults, path))
    return records


def make_record(row: dict[str, str], defaults: dict[str, Any], path: Path) -> dict[str, Any]:
    candidate_id = csv_value(row, ("candidate_id", "candidate")) or str(defaults.get("candidate_id", ""))
    segment_id = csv_value(row, ("segment_id", "segment")) or str(defaults.get("segment_id", ""))
    stage = csv_value(row, ("stage", "stage_name")) or str(defaults.get("stage", ""))
    log_field = csv_value(row, ("log_field", "log_parameter", "parameter"))
    if not candidate_id or not segment_id:
        raise TestbedConfigError(f"NIS CSV row missing candidate_id or segment_id: {path}")
    return {
        "candidate_id": candidate_id,
        "trial_id": csv_value(row, ("trial_id",)) or f"{candidate_id}:nominal",
        "segment_id": segment_id,
        "stage": stage,
        "log_field": log_field,
        "nis": parse_float(csv_value(row, ("nis",))),
        "accepted": accepted_from_row(row, log_field),
        "measurement_dimension": parse_optional_int(csv_value(row, ("measurement_dimension", "dimension"))),
        "command_bucket": row_command_signature(row),
        "split": csv_value(row, ("split", "dataset_split")),
        "corrupted": parse_bool(csv_value(row, ("corrupted", "is_corrupted"))),
    }


def merge_record_info(info: SegmentInfo, record: dict[str, Any]) -> None:
    split = record.get("split")
    if split:
        if split not in SPLITS:
            raise TestbedConfigError(f"Invalid split for segment {info.segment_id}: {split}")
        if info.split and info.split != split:
            raise TestbedConfigError(f"Conflicting split for segment {info.segment_id}")
        info.split = split
    # Row artifacts do not carry enough boundary context to classify corruption.
    # The manifest owns terminal external-force exclusion.
    if record.get("stage") and not info.stage:
        info.stage = str(record["stage"])


def accepted_from_row(row: dict[str, str], log_field: str = "") -> bool:
    if is_ungated_measurement_log_field(log_field):
        return True
    rejected_text = csv_value(row, ("rejected",))
    if rejected_text:
        return not parse_bool(rejected_text)
    accepted_text = csv_value(row, ("accepted",))
    if accepted_text:
        return parse_bool(accepted_text)
    return True


def is_ungated_measurement_log_field(log_field: str) -> bool:
    field = canonical_log_field(log_field).lower()
    return (
        field.startswith("yaw_rate")
        or field.startswith("gyro")
        or field.startswith("measured_yaw_rate")
        or "encoder" in field
        or "wheel_rate" in field
    )


def record_command_bucket(raw: dict[str, Any], info: SegmentInfo) -> str:
    row_bucket = str(raw.get("command_bucket", "")).strip()
    stage = str(raw.get("stage") or info.stage)
    if is_launch_stage(stage):
        return row_bucket or launch_command_signature(info.observed_command)
    return ""


def row_command_signature(row: dict[str, str]) -> str:
    explicit = csv_value(
        row,
        (
            "command_bucket",
            "launch_command_bucket",
            "launch_command_signature",
            "observed_command_signature",
        ),
    )
    if explicit:
        return explicit
    left = parse_optional_command(row, ("left_command", "left_drive_command"))
    right = parse_optional_command(row, ("right_command", "right_drive_command"))
    linear = parse_optional_command(row, ("cmd_linear_mps", "cmd_linear_command"))
    yaw = parse_optional_command(row, ("cmd_yaw_radps", "cmd_yaw_command", "cmd_angular_radps"))
    parts: list[str] = []
    if left is not None and right is not None:
        parts.append(f"pair={command_bin(left)},{command_bin(right)}")
        if linear is None:
            linear = 0.5 * (left + right)
        if yaw is None:
            yaw = 0.5 * (right - left)
    if linear is not None:
        parts.append(f"linear={command_bin(linear)}")
    if yaw is not None:
        parts.append(f"yaw={command_bin(yaw)}")
    return ";".join(parts)


def parse_optional_command(row: dict[str, str], names: tuple[str, ...]) -> float | None:
    text = csv_value(row, names)
    if not text:
        return None
    try:
        value = float(text)
    except ValueError:
        return None
    return value if math.isfinite(value) else None


def command_bin(value: float) -> str:
    binned = round(value / COMMAND_BIN_WIDTH) * COMMAND_BIN_WIDTH
    return format_number(0.0 if abs(binned) < 0.5 * COMMAND_BIN_WIDTH else binned)


def is_launch_stage(stage: str) -> bool:
    return "launch" in stage.lower()


def score_partition(stage: str, parameter_fields: dict[str, Any]) -> str:
    text = " ".join([stage, *(str(value) for value in parameter_fields.values())]).lower()
    if is_active_yaw_calibration(text):
        return "active_traction"
    if any(token in text for token in ("stationary", "static", "bias", "idle", "zero")):
        return "stationary_bias_validation"
    return "active_traction"


def is_active_yaw_calibration(text: str) -> bool:
    return "yaw" in text and any(
        token in text
        for token in ("launch", "maneuver", "turn", "sec_40_yaw", "in_place", "calibration")
    )


def is_terminal_external_force_corruption(raw: dict[str, Any]) -> bool:
    if parse_bool(raw.get("corrupted", raw.get("is_corrupted", False))):
        return True
    has_boundary_context = bool(raw.get("corruption_note")) or (
        str(raw.get("end_reason", "")).strip().lower() == "corruption_boundary"
    )
    if not has_boundary_context:
        return False
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


def evaluate_records(
    config: dict[str, Any],
    candidates: list[CandidateConfig],
    trials: list[TrialConfig],
    records: list[NisRecord],
) -> tuple[list[ItemizedScore], list[TrialScore], list[CandidateScore]]:
    specs = scoring_specs(config)
    segment_stats: dict[
        tuple[str, str, str, str, str, str, str, str, str, str, str],
        SegmentScoreStats,
    ] = {}
    for record in records:
        parameter_field, parameter_value_kind, parameter_value = parameter_bucket(record)
        command_signature = record.command_bucket
        partition = score_partition(record.stage, record.parameter_fields)
        for split in (record.split, "all"):
            key = (
                record.candidate_id,
                record.trial_id,
                split,
                record.stage,
                record.log_field,
                parameter_field,
                parameter_value_kind,
                parameter_value,
                command_signature,
                partition,
                record.segment_id,
            )
            segment_stats.setdefault(key, SegmentScoreStats()).add(record)

    itemized: list[ItemizedScore] = []
    scoring = dict(config.get("scoring", {}))
    under_expected_weight = float(scoring.get("under_expected_penalty_weight", 1.0))
    inflation_weight = float(scoring.get("inflation_floor_penalty_weight", 1.0))
    rejected_rate_weight = float(scoring.get("rejected_rate_penalty_weight", 1.0))
    bucketed_stats: dict[
        tuple[str, str, str, str, str, str, str, str, str, str],
        list[SegmentScoreStats],
    ] = {}
    for key, stats in segment_stats.items():
        bucketed_stats.setdefault(key[:-1], []).append(stats)

    for key, segment_buckets in sorted(bucketed_stats.items()):
        (
            candidate_id,
            trial_id,
            split,
            stage,
            log_field,
            parameter_field,
            parameter_value_kind,
            parameter_value,
            command_signature,
            partition,
        ) = key
        total_count = sum(bucket.total_count for bucket in segment_buckets)
        finite_count = sum(bucket.finite_count for bucket in segment_buckets)
        accepted_count = sum(bucket.accepted_count for bucket in segment_buckets)
        rejected_count = sum(bucket.rejected_count for bucket in segment_buckets)
        rms = (
            math.sqrt(
                sum(bucket.sum_nis_sq for bucket in segment_buckets)
                / finite_count
            )
            if finite_count
            else math.nan
        )
        accepted_only_rms = (
            math.sqrt(
                sum(bucket.accepted_sum_nis_sq for bucket in segment_buckets)
                / accepted_count
            )
            if accepted_count
            else math.nan
        )
        spec = specs.get(
            log_field,
            {"expected": expected_rms_for_dimension(1), "floor_ratio": 0.75, "weight": 1.0},
        )
        expected = spec["expected"]
        inflation_floor = expected * spec["floor_ratio"]
        guarded = max(rms, expected) if math.isfinite(rms) else math.inf
        under_expected = max(0.0, expected - rms) if math.isfinite(rms) else expected
        rejected_rate = rejected_count / total_count if total_count else 0.0
        stage_w = stage_weight(config, stage)
        channel_w = spec["weight"]
        itemized.append(
            ItemizedScore(
                candidate_id=candidate_id,
                trial_id=trial_id,
                split=split,
                stage=stage,
                log_field=log_field,
                count=total_count,
                accepted_count=accepted_count,
                finite_count=finite_count,
                nonfinite_count=total_count - finite_count,
                rejected_count=rejected_count,
                rejected_rate=rejected_rate,
                rejected_rate_penalty=rejected_rate_weight * rejected_rate,
                segment_count=len(segment_buckets),
                score_partition=partition,
                stage_weight=stage_w,
                channel_weight=channel_w,
                selection_weight=stage_w * channel_w,
                rms_nis=rms,
                accepted_only_rms_nis=accepted_only_rms,
                expected_rms_nis=expected,
                guarded_rms_nis=guarded,
                under_expected_penalty=under_expected,
                inflation_flag=math.isfinite(rms) and rms < inflation_floor,
                parameter_field=parameter_field,
                parameter_value_kind=parameter_value_kind,
                parameter_value=parameter_value,
                launch_command_signature=command_signature,
                launch_cmd_linear_mps_median_mean=mean_observed_command_value_for_key(
                    records,
                    candidate_id,
                    trial_id,
                    split,
                    stage,
                    log_field,
                    command_signature,
                    "cmd_linear_mps_median",
                ),
            )
        )

    selection_split = str(config.get("scoring", {}).get("selection_split", "validation"))
    if selection_split not in REPORT_SPLITS:
        raise TestbedConfigError(f"Invalid scoring.selection_split: {selection_split}")
    trial_scores: list[TrialScore] = []
    trial_keys = {(trial.candidate_id, trial.trial_id) for trial in trials}
    trial_keys.update((record.candidate_id, record.trial_id) for record in records)
    for candidate_id, trial_id in sorted(trial_keys):
        active_keys = [
            (key, stats)
            for key, stats in segment_stats.items()
            if key[0] == candidate_id
            and key[1] == trial_id
            and key[2] == selection_split
            and key[9] == "active_traction"
        ]
        sample_count = sum(stats.finite_count for _key, stats in active_keys)
        score = weighted_trial_score(
            config,
            specs,
            active_keys,
            under_expected_weight,
            inflation_weight,
            rejected_rate_weight,
        )
        trial_scores.append(TrialScore(candidate_id, trial_id, score, sample_count))

    rankings: list[CandidateScore] = []
    for candidate in candidates:
        candidate_trials = [score for score in trial_scores if score.candidate_id == candidate.candidate_id]
        best = min(candidate_trials, key=lambda score: (score.selection_score, score.trial_id), default=None)
        rankings.append(
            CandidateScore(
                candidate_id=candidate.candidate_id,
                selection_score=best.selection_score if best else math.inf,
                sample_count=best.sample_count if best else 0,
            )
        )
    rankings.sort(key=lambda score: (score.selection_score, score.candidate_id))
    return itemized, trial_scores, rankings


def write_trial_plan(output_dir: Path, trials: list[TrialConfig]) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    config_root = output_dir / "trial_configs"
    config_root.mkdir(parents=True, exist_ok=True)
    with (output_dir / "trial_plan.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=["candidate_id", "trial_id", "is_nominal", "parameter_json"],
        )
        writer.writeheader()
        for trial in sorted(trials, key=lambda item: (item.candidate_id, item.trial_id)):
            candidate_dir = config_root / safe_filename(trial.candidate_id)
            candidate_dir.mkdir(parents=True, exist_ok=True)
            payload = {
                "schema_version": 1,
                "candidate_id": trial.candidate_id,
                "trial_id": trial.trial_id,
                "is_nominal": trial.is_nominal,
                "parameters": trial.parameters,
                "covariance_policy": "fixed_testbed_estimator_covariance_not_tuned",
            }
            (candidate_dir / f"{safe_filename(trial.trial_id)}.json").write_text(
                json.dumps(payload, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
            writer.writerow(
                {
                    "candidate_id": trial.candidate_id,
                    "trial_id": trial.trial_id,
                    "is_nominal": "true" if trial.is_nominal else "false",
                    "parameter_json": json.dumps(trial.parameters, sort_keys=True),
                }
            )


def safe_filename(value: str) -> str:
    return "".join(ch if ch.isalnum() or ch in "._-" else "_" for ch in value)


def write_integration_todos(output_dir: Path) -> None:
    lines = [
        "# Traction ANIS Testbed Integration TODOs",
        "",
        "The tuning/ranking harness is present. Standalone data/model modules must produce estimator artifacts before rankings are meaningful.",
        "",
        "Required integration points:",
        "",
        "1. Use a testbed-only data loader for decoded open-floor logs. It must reject or ignore `ukf_state_*` and `logged_ukf_state*` columns.",
        "2. Use a testbed-only traction/estimator replay path that consumes `trial_configs/**.json` and emits accepted/rejected estimator rows.",
        "3. Keep estimator covariance fixed. Candidate bounds may tune physical traction/model parameters only.",
        "4. Carry launch observed-command summaries from the segment manifest so itemized reports can separate launch behavior by command bucket.",
        "5. Point the manifest at generated estimator artifacts, then run `score`.",
        "",
        "This intentionally does not request production or hardware hooks.",
    ]
    (output_dir / "integration_todos.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_outputs(
    output_dir: Path,
    trials: list[TrialConfig],
    segments: dict[str, SegmentInfo],
    records: list[NisRecord],
    itemized: list[ItemizedScore],
    trial_scores: list[TrialScore],
    rankings: list[CandidateScore],
) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    write_trial_plan(output_dir, trials)
    write_integration_todos(output_dir)
    write_itemized_csv(output_dir, itemized)
    write_trial_scores_csv(output_dir, trial_scores)
    write_candidate_rankings_csv(output_dir, rankings)
    write_segment_splits_csv(output_dir, segments)
    payload = {
        "uses_logged_ukf_state": False,
        "production_or_hardware_hooks": False,
        "non_corrupted_nis_sample_count": len(records),
        "finite_nis_sample_count": sum(1 for record in records if math.isfinite(record.nis)),
        "accepted_nis_sample_count": sum(1 for record in records if record.accepted),
        "rejected_nis_sample_count": sum(1 for record in records if not record.accepted),
        "selection_scoring": (
            "active traction only; main RMS uses all finite NIS; stationary/bias validation is itemized but excluded from ranking"
        ),
        "itemized": [item.__dict__ for item in itemized],
        "trial_scores": [score.__dict__ for score in trial_scores],
        "rankings": [score.__dict__ for score in rankings],
    }
    (output_dir / "scoring_summary.json").write_text(
        json.dumps(json_safe(payload), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    write_report(output_dir, records, rankings)


def write_itemized_csv(output_dir: Path, itemized: list[ItemizedScore]) -> None:
    fieldnames = list(ItemizedScore.__dataclass_fields__)
    with (output_dir / "itemized_rms_nis.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for item in itemized:
            row = dict(item.__dict__)
            for key, value in list(row.items()):
                if isinstance(value, float):
                    row[key] = format_number(value)
                elif isinstance(value, bool):
                    row[key] = "true" if value else "false"
            writer.writerow(row)


def write_trial_scores_csv(output_dir: Path, scores: list[TrialScore]) -> None:
    fieldnames = ["rank", *TrialScore.__dataclass_fields__]
    ranked = sorted(scores, key=lambda score: (math.isinf(score.selection_score), score.selection_score, score.trial_id))
    with (output_dir / "trial_scores.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for rank, score in enumerate(ranked, start=1):
            row = dict(score.__dict__)
            row["rank"] = "" if math.isinf(score.selection_score) else rank
            row["selection_score"] = format_number(score.selection_score)
            writer.writerow(row)


def write_candidate_rankings_csv(output_dir: Path, rankings: list[CandidateScore]) -> None:
    fieldnames = ["rank", *CandidateScore.__dataclass_fields__]
    with (output_dir / "candidate_rankings.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for rank, score in enumerate(rankings, start=1):
            row = dict(score.__dict__)
            row["rank"] = rank
            row["selection_score"] = format_number(score.selection_score)
            writer.writerow(row)


def write_segment_splits_csv(output_dir: Path, segments: dict[str, SegmentInfo]) -> None:
    fieldnames = [
        "segment_id",
        "split",
        "corrupted",
        "stage",
        "parameter_json",
        "observed_command_json",
        "launch_command_signature",
    ]
    with (output_dir / "segment_splits.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for segment_id, info in sorted(segments.items()):
            writer.writerow(
                {
                    "segment_id": segment_id,
                    "split": info.split or "",
                    "corrupted": "true" if info.corrupted else "false",
                    "stage": info.stage,
                    "parameter_json": json.dumps(info.parameter_fields, sort_keys=True),
                    "observed_command_json": json.dumps(info.observed_command, sort_keys=True),
                    "launch_command_signature": launch_command_signature(info.observed_command),
                }
            )


def write_report(output_dir: Path, records: list[NisRecord], rankings: list[CandidateScore]) -> None:
    lines = [
        "# Traction ANIS Testbed Report",
        "",
        f"- Generated samples scored: `{len(records)}`",
        "- Split policy: whole-segment assignment only; missing splits are stable-hashed by `segment_id`.",
        "- Logged UKF state policy: `ukf_state*` and `logged_ukf_state*` CSV columns are rejected.",
        "- Scoring policy: all finite estimator NIS is aggregated per segment, then per stage/command/channel bucket with explicit stage and channel weights.",
        "- Accepted-only RMS is diagnostic; rejected finite accelerometer rows remain in the main NIS average.",
        "- Yaw/gyro and encoder NIS rows are ungated and accepted when finite, including rows whose input artifact says rejected.",
        "- Stationary/bias validation buckets are itemized but excluded from active traction ranking.",
        "- Production/hardware hooks: none.",
        "",
        "## Candidate Ranking",
        "",
        "| Rank | Candidate | Selection score | Samples |",
        "| ---: | --- | ---: | ---: |",
    ]
    for rank, score in enumerate(rankings, start=1):
        lines.append(
            f"| {rank} | `{score.candidate_id}` | {format_number(score.selection_score)} | {score.sample_count} |"
        )
    if not records:
        lines.extend(
            [
                "",
                "## Integration Status",
                "",
                "No NIS artifacts were found in the manifest. `integration_todos.md` lists the standalone data/model work still needed.",
            ]
        )
    (output_dir / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def format_number(value: float) -> str:
    if math.isinf(value):
        return "inf"
    if math.isnan(value):
        return ""
    return f"{value:.12g}"


def json_safe(value: Any) -> Any:
    if isinstance(value, float):
        if math.isinf(value):
            return "inf"
        if math.isnan(value):
            return None
        return value
    if isinstance(value, dict):
        return {str(key): json_safe(child) for key, child in value.items()}
    if isinstance(value, list):
        return [json_safe(child) for child in value]
    if isinstance(value, tuple):
        return [json_safe(child) for child in value]
    return value


def item_score(item: ItemizedScore, under_expected_weight: float, inflation_weight: float) -> float:
    if not math.isfinite(item.rms_nis):
        return math.inf
    normalized = item.guarded_rms_nis / item.expected_rms_nis
    normalized += under_expected_weight * (item.under_expected_penalty / item.expected_rms_nis)
    if item.inflation_flag:
        normalized += inflation_weight * (
            (item.expected_rms_nis - item.rms_nis) / item.expected_rms_nis
        )
    normalized += item.rejected_rate_penalty
    return normalized


def scoring_specs(config: dict[str, Any]) -> dict[str, dict[str, float]]:
    scoring = dict(config.get("scoring", {}))
    floor = float(scoring.get("inflation_floor_ratio", 0.75))
    raw_fields = scoring.get("log_fields", scoring.get("log_parameters", {}))
    result: dict[str, dict[str, float]] = {}
    for name, raw in dict(raw_fields).items():
        dimension = int(dict(raw).get("dimension", 1))
        result[canonical_log_field(str(name))] = {
            "expected": expected_rms_for_dimension(dimension),
            "floor_ratio": floor,
            "weight": float(dict(raw).get("weight", 1.0)),
        }
    return result


def canonical_log_field(value: str) -> str:
    text = str(value).strip()
    if not text or text.endswith("_nis") or text.endswith("_residual_tail"):
        return text
    return f"{text}_nis"


def parameter_bucket(record: NisRecord) -> tuple[str, str, str]:
    return (
        str(record.parameter_fields.get("parameter", "")),
        str(record.parameter_fields.get("test_value_kind", "")),
        str(record.parameter_fields.get("test_value", "")),
    )


def mean_observed_command_value(records: list[NisRecord], field_name: str) -> float:
    values = [
        parse_float_or_nan(record.observed_command.get(field_name))
        for record in records
    ]
    finite_values = [value for value in values if math.isfinite(value)]
    if not finite_values:
        return math.nan
    return sum(finite_values) / len(finite_values)


def mean_observed_command_value_for_key(
    records: list[NisRecord],
    candidate_id: str,
    trial_id: str,
    split: str,
    stage: str,
    log_field: str,
    command_signature: str,
    field_name: str,
) -> float:
    matching = [
        record
        for record in records
        if record.candidate_id == candidate_id
        and record.trial_id == trial_id
        and split in (record.split, "all")
        and record.stage == stage
        and record.log_field == log_field
        and record.command_bucket == command_signature
    ]
    return mean_observed_command_value(matching, field_name)


def weighted_trial_score(
    config: dict[str, Any],
    specs: dict[str, dict[str, float]],
    active_segment_stats: list[
        tuple[tuple[str, str, str, str, str, str, str, str, str, str, str], SegmentScoreStats]
    ],
    under_expected_weight: float,
    inflation_weight: float,
    rejected_rate_weight: float,
) -> float:
    by_bucket: dict[tuple[str, str, str], list[SegmentScoreStats]] = {}
    for key, stats in active_segment_stats:
        _candidate_id, _trial_id, _split, stage, log_field, *_rest = key
        command_signature = key[8]
        by_bucket.setdefault((stage, command_signature, log_field), []).append(stats)
    weighted_sum = 0.0
    total_weight = 0.0
    for (stage, _command_signature, log_field), segment_buckets in by_bucket.items():
        spec = specs.get(
            log_field,
            {"expected": expected_rms_for_dimension(1), "floor_ratio": 0.75, "weight": 1.0},
        )
        segment_scores = [
            item_score_from_values(
                rms_nis=stats.rms_nis(),
                expected=spec["expected"],
                floor_ratio=spec["floor_ratio"],
                under_expected_weight=under_expected_weight,
                inflation_weight=inflation_weight,
            )
            for stats in segment_buckets
            if stats.finite_count > 0
        ]
        total_count = sum(stats.total_count for stats in segment_buckets)
        rejected_count = sum(stats.rejected_count for stats in segment_buckets)
        rejected_rate = rejected_count / total_count if total_count else 0.0
        bucket_score = (
            sum(segment_scores) / len(segment_scores)
            if segment_scores
            else math.inf
        )
        bucket_score += rejected_rate_weight * rejected_rate
        weight = stage_weight(config, stage) * spec["weight"]
        if math.isinf(bucket_score):
            return math.inf
        weighted_sum += bucket_score * weight
        total_weight += weight
    return weighted_sum / total_weight if total_weight else math.inf


def item_score_from_values(
    rms_nis: float,
    expected: float,
    floor_ratio: float,
    under_expected_weight: float,
    inflation_weight: float,
) -> float:
    if not math.isfinite(rms_nis):
        return math.inf
    guarded = max(rms_nis, expected)
    under_expected = max(0.0, expected - rms_nis)
    normalized = guarded / expected
    normalized += under_expected_weight * (under_expected / expected)
    if rms_nis < expected * floor_ratio:
        normalized += inflation_weight * ((expected - rms_nis) / expected)
    return normalized


def stage_weight(config: dict[str, Any], stage: str) -> float:
    scoring = dict(config.get("scoring", {}))
    stage_weights = dict(scoring.get("stage_weights", {}))
    return float(stage_weights.get(stage, scoring.get("default_stage_weight", 1.0)))


def expected_rms_for_dimension(dimension: int) -> float:
    return math.sqrt(float(dimension) * float(dimension + 2))


def stable_split(segment_id: str, split_config: dict[str, Any]) -> str:
    seed = str(split_config.get("seed", "traction_rms_nis_testbed"))
    ratios = [
        float(split_config.get("train", 0.6)),
        float(split_config.get("validation", 0.2)),
        float(split_config.get("held_out", 0.2)),
    ]
    total = sum(ratios) or 1.0
    value = int(hashlib.sha256(f"{seed}:{segment_id}".encode("utf-8")).hexdigest()[:16], 16)
    unit = value / float(0xFFFFFFFFFFFFFFFF)
    train_cut = ratios[0] / total
    validation_cut = train_cut + ratios[1] / total
    if unit < train_cut:
        return "train"
    if unit < validation_cut:
        return "validation"
    return "held_out"


def launch_command_signature(observed: dict[str, Any]) -> str:
    parts: list[str] = []
    pair = observed.get("observed_command_pair_mode")
    if pair not in (None, ""):
        parts.append(f"pair={format_command_value(pair)}")
    magnitude = observed.get("observed_command_magnitude_median")
    if magnitude not in (None, ""):
        parts.append(f"mag={format_command_value(magnitude)}")
    linear = observed.get("cmd_linear_mps_median")
    if linear not in (None, ""):
        parts.append(f"linear={format_command_value(linear)}")
    yaw = observed.get("cmd_yaw_radps_median")
    if yaw not in (None, ""):
        parts.append(f"yaw={format_command_value(yaw)}")
    return ";".join(parts)


def format_command_value(value: Any) -> str:
    if isinstance(value, list):
        return ",".join(format_command_value(child) for child in value)
    if isinstance(value, float):
        return format_number(value)
    return str(value)


def csv_value(row: dict[str, str], names: tuple[str, ...]) -> str:
    lookup = {key.lower(): key for key in row}
    for name in names:
        key = lookup.get(name.lower())
        if key is not None and str(row.get(key, "")).strip():
            return str(row[key]).strip()
    return ""


def parse_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("1", "true", "yes", "y", "corrupt", "corrupted")


def parse_float(value: Any) -> float:
    try:
        result = float(str(value).strip())
    except (TypeError, ValueError) as exc:
        raise TestbedConfigError(f"Expected numeric NIS value, got {value!r}") from exc
    if not math.isfinite(result) or result < 0.0:
        raise TestbedConfigError(f"Expected finite non-negative NIS value, got {value!r}")
    return result


def parse_float_or_nan(value: Any) -> float:
    try:
        return float(str(value).strip())
    except (TypeError, ValueError):
        return math.nan


def parse_optional_int(value: Any) -> int | None:
    text = str(value).strip()
    if not text:
        return None
    try:
        return int(text)
    except ValueError:
        return None


def parse_args(argv: Iterable[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("plan", "score"))
    parser.add_argument("--config", default=str(DEFAULT_CONFIG_PATH))
    parser.add_argument("--manifest", default="")
    parser.add_argument(
        "--nis-artifact",
        action="append",
        default=[],
        help="Replay-produced NIS CSV to score in addition to manifest-listed artifacts.",
    )
    parser.add_argument("--output-dir", default="")
    parser.add_argument("--trial-count", type=int, default=None)
    return parser.parse_args(list(argv))


def main(argv: Iterable[str]) -> int:
    args = parse_args(argv)
    config_path = Path(args.config).resolve()
    config = load_json(config_path)
    if args.trial_count is not None:
        config.setdefault("tuning", {})["trial_count_per_candidate"] = args.trial_count
    candidates = load_candidates(config)
    trials = generate_trial_plan(config, candidates)
    output_dir = Path(args.output_dir).resolve() if args.output_dir else DEFAULT_OUTPUT_DIR
    if args.command == "plan":
        write_trial_plan(output_dir, trials)
        write_integration_todos(output_dir)
        print(f"Wrote traction ANIS trial plan to {output_dir}")
        return 0

    if args.manifest:
        manifest_path = Path(args.manifest).resolve()
    else:
        manifest_text = str(config.get("manifest_path", ""))
        if not manifest_text:
            raise TestbedConfigError("score requires --manifest or config.manifest_path")
        manifest_path = resolve_path(manifest_text, config_path.parent)
    if not manifest_path:
        raise TestbedConfigError("score requires --manifest or config.manifest_path")
    extra_artifacts = [{"path": str(Path(path_text).resolve())} for path_text in args.nis_artifact]
    records, segments = load_records(config, config_path, manifest_path, trials, extra_artifacts)
    itemized, trial_scores, rankings = evaluate_records(config, candidates, trials, records)
    write_outputs(output_dir, trials, segments, records, itemized, trial_scores, rankings)
    print(f"Wrote traction ANIS scoring report to {output_dir}")
    return 0
