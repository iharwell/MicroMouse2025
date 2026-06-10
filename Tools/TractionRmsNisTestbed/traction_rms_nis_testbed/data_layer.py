"""Standalone data layer for traction RMS/NIS observable streams.

The manifest owns segmentation. Decoded CSV rows provide observable samples.
Logged UKF state and replay state columns are never surfaced by this layer.
"""

from __future__ import annotations

import csv
import json
from collections import defaultdict, deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Iterator


REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_MANIFEST_PATH = (
    REPO_ROOT
    / "staging"
    / "traction_candidate_rms_nis_segments"
    / "segment_manifest.json"
)
DEFAULT_OUTPUT_DIR = REPO_ROOT / "staging" / "traction_candidate_rms_nis_testbed" / "latest"

IGNORED_STATE_COLUMN_PREFIXES = (
    "ukf_state",
    "logged_ukf_state",
    "replay_state",
    "replayed_state",
)

DRIVE_COMMAND_COLUMNS = (
    "cmd_linear_mps",
    "cmd_angular_radps",
    "cmd_yaw_radps",
    "left_drive_command",
    "right_drive_command",
    "left_feedforward_command",
    "right_feedforward_command",
    "left_feedback_command",
    "right_feedback_command",
    "left_target_velocity_mps",
    "right_target_velocity_mps",
    "left_launch_assist_floor",
    "right_launch_assist_floor",
)

ENCODER_COLUMNS = (
    "encoder_timestamp_us",
    "left_encoder_count",
    "right_encoder_count",
    "left_encoder_omega_radps",
    "right_encoder_omega_radps",
    "left_encoder_distance_m",
    "right_encoder_distance_m",
    "left_encoder_velocity_mps",
    "right_encoder_velocity_mps",
    "measured_linear_speed_mps",
    "measured_angular_speed_radps",
)

GYRO_COLUMNS = (
    "gyro_radps",
    "gyro_bias_radps",
    "gyro_raw_radps",
    "gyro_bias_anchor_radps",
    "yaw_consistency_lp_radps",
    "yaw_window_mismatch_rad",
    "imu_gyro_x",
    "imu_gyro_y",
    "imu_gyro_z",
)

ACCEL_COLUMNS = (
    "accel_body_x_mps2",
    "accel_body_y_mps2",
    "planar_accel_mps2",
    "imu_accel_x",
    "imu_accel_y",
    "imu_accel_z",
    "imu_temp",
)

FAN_COLUMNS = ("fan_duty_cycle",)

METADATA_COLUMNS = (
    "master_time_us",
    "control_tick_sequence",
    "section_id",
    "primitive_id",
    "primitive_family",
    "direction_id",
    "phase_id",
    "speed_bin",
    "start_marker_id",
    "repeat_index",
    "progress_norm",
    "mode_flags",
    "clipping_flags",
    "saturation_flags",
    "watchdog_flags",
    "measurement_flags",
    "ukf_mode_id",
    "ukf_yaw_valid_for_feedforward",
    "bias_update_enabled",
    "imu_timestamp_us",
    "imu_status",
    "imu_interrupt_high",
    "accel_valid",
    "imu_accel_valid",
    "accel_bias_valid",
    "front_timestamp_us",
    "left_timestamp_us",
    "right_timestamp_us",
)

COLUMN_GROUPS = {
    "drive_commands": DRIVE_COMMAND_COLUMNS,
    "encoder": ENCODER_COLUMNS,
    "gyro": GYRO_COLUMNS,
    "accel": ACCEL_COLUMNS,
    "fan": FAN_COLUMNS,
    "metadata": METADATA_COLUMNS,
}


class DataLayerError(RuntimeError):
    """Raised when manifest or decoded log data is unusable."""


@dataclass(frozen=True)
class SegmentDefinition:
    segment_id: str
    log_path: str
    log_absolute_path: Path
    segment_start_row_index: int
    segment_end_row_index: int
    segment_start_time_us: int | None
    segment_end_time_us: int | None
    active_start_row_index: int | None
    active_end_row_index: int | None
    active_start_time_us: int | None
    active_end_time_us: int | None
    slot_start_row_index: int | None
    slot_end_row_index: int | None
    stage: str
    stage_id: int | None
    family: str
    phase_id: int | None
    phase_name: str
    primitive_id: int | None
    primitive_name: str
    direction_id: int | None
    direction_name: str
    repeat_index: int | None
    schema_kind: str
    schema_version: int | None
    speed_bin: float | None
    speed_bin_label: str
    fan_duty_cycle: float | None
    fan_source: str
    start_criterion: str
    end_criterion: str
    end_reason: str
    corruption_note: str | None
    parameter_fields: dict[str, Any]
    observed_command: dict[str, Any]


@dataclass
class SegmentStream:
    definition: SegmentDefinition
    row_count: int
    streams: dict[str, Any]
    boundaries: dict[str, Any]
    log_metadata: dict[str, Any]

    def to_json_dict(self) -> dict[str, Any]:
        return {
            "segment_id": self.definition.segment_id,
            "source": {
                "log_path": self.definition.log_path,
                "log_absolute_path": str(self.definition.log_absolute_path),
            },
            "metadata": {
                "stage": self.definition.stage,
                "stage_id": self.definition.stage_id,
                "family": self.definition.family,
                "phase_id": self.definition.phase_id,
                "phase_name": self.definition.phase_name,
                "primitive_id": self.definition.primitive_id,
                "primitive_name": self.definition.primitive_name,
                "direction_id": self.definition.direction_id,
                "direction_name": self.definition.direction_name,
                "repeat_index": self.definition.repeat_index,
                "schema_kind": self.definition.schema_kind,
                "schema_version": self.definition.schema_version,
                "speed_bin": self.definition.speed_bin,
                "speed_bin_label": self.definition.speed_bin_label,
            },
            "parameters": {
                "parameter_fields": self.definition.parameter_fields,
                "observed_command": self.definition.observed_command,
            },
            "fan": {
                "manifest_fan_duty_cycle": self.definition.fan_duty_cycle,
                "manifest_fan_source": self.definition.fan_source,
                "sidecar_fan_duty_cycle": self.log_metadata.get("fan_duty_cycle"),
            },
            "boundaries": self.boundaries,
            "row_count": self.row_count,
            "streams": self.streams,
        }


class SegmentAccumulator:
    def __init__(self, definition: SegmentDefinition, available_columns: dict[str, tuple[str, ...]]):
        self.definition = definition
        self.available_columns = available_columns
        self.row_index: list[int] = []
        self.is_active: list[bool] = []
        self.is_stationary_context: list[bool] = []
        self.dt_us: list[int | float | str | None] = []
        self.previous_master_time_us: int | float | None = None
        self.groups: dict[str, dict[str, list[Any]]] = {
            group: {column: [] for column in columns}
            for group, columns in available_columns.items()
            if columns
        }

    def append(self, row_index: int, row: dict[str, str]) -> None:
        self.row_index.append(row_index)
        active = self.definition.active_start_row_index is not None and (
            self.definition.active_start_row_index
            <= row_index
            <= (self.definition.active_end_row_index or self.definition.active_start_row_index)
        )
        self.is_active.append(active)
        self.is_stationary_context.append(not active)
        self.dt_us.append(self._dt_value(row))
        for group, columns in self.available_columns.items():
            group_values = self.groups.get(group)
            if group_values is None:
                continue
            for column in columns:
                group_values[column].append(parse_cell(row.get(column)))

    def finish(self, log_metadata: dict[str, Any]) -> SegmentStream:
        streams: dict[str, Any] = {
            "row_index": self.row_index,
            "is_active": self.is_active,
            "is_stationary_context": self.is_stationary_context,
            "dt_us": self.dt_us,
        }
        streams.update(self.groups)
        return SegmentStream(
            definition=self.definition,
            row_count=len(self.row_index),
            streams=streams,
            boundaries=boundary_metadata(self.definition),
            log_metadata=log_metadata,
        )

    def _dt_value(self, row: dict[str, str]) -> int | float | str | None:
        if "dt_us" in row:
            return parse_cell(row.get("dt_us"))
        master_time = parse_cell(row.get("master_time_us"))
        if not isinstance(master_time, (int, float)):
            return None
        if self.previous_master_time_us is None:
            self.previous_master_time_us = master_time
            return None
        dt_us = master_time - self.previous_master_time_us
        self.previous_master_time_us = master_time
        return dt_us


class TractionObservableDataLayer:
    def __init__(self, manifest_path: Path, repo_root: Path | None = None):
        self.manifest_path = manifest_path
        self.manifest = load_json_object(manifest_path)
        self.repo_root = repo_root or manifest_repo_root(self.manifest, manifest_path)
        self.ignored_state_column_prefixes = list(IGNORED_STATE_COLUMN_PREFIXES)
        self.segments = [
            segment_definition(raw, self.repo_root)
            for raw in required_list(self.manifest, "segments")
        ]

    def iter_segment_streams(
        self,
        segment_ids: set[str] | None = None,
        limit: int | None = None,
    ) -> Iterator[SegmentStream]:
        selected = self._selected_segments(segment_ids=segment_ids, limit=limit)
        by_log: dict[Path, list[SegmentDefinition]] = defaultdict(list)
        for segment in selected:
            by_log[segment.log_absolute_path].append(segment)

        for log_path in sorted(by_log, key=str):
            yield from self._streams_for_log(log_path, by_log[log_path])

    def _selected_segments(
        self,
        segment_ids: set[str] | None,
        limit: int | None,
    ) -> list[SegmentDefinition]:
        selected = [
            segment
            for segment in self.segments
            if segment_ids is None or segment.segment_id in segment_ids
        ]
        if segment_ids is not None:
            found = {segment.segment_id for segment in selected}
            missing = sorted(segment_ids - found)
            if missing:
                raise DataLayerError(f"Segment id(s) not found in manifest: {', '.join(missing)}")
        if limit is not None:
            if limit < 0:
                raise DataLayerError("--limit must be non-negative")
            selected = selected[:limit]
        return selected

    def _streams_for_log(
        self,
        log_path: Path,
        segments: list[SegmentDefinition],
    ) -> Iterator[SegmentStream]:
        if not log_path.exists():
            raise DataLayerError(f"Decoded open-floor log not found: {log_path}")

        sorted_segments = sorted(segments, key=lambda segment: segment.segment_start_row_index)
        pending = deque(sorted_segments)
        active: list[SegmentAccumulator] = []
        finished: list[SegmentStream] = []
        log_metadata = read_sidecar_metadata(log_path.with_suffix(".sidecar"))

        with log_path.open(newline="", encoding="utf-8-sig") as csv_file:
            reader = csv.DictReader(csv_file)
            if not reader.fieldnames:
                raise DataLayerError(f"CSV header missing: {log_path}")
            available_columns = observable_columns(reader.fieldnames)

            for row_index, row in enumerate(reader):
                while pending and pending[0].segment_start_row_index <= row_index:
                    active.append(SegmentAccumulator(pending.popleft(), available_columns))

                if active:
                    still_active: list[SegmentAccumulator] = []
                    for accumulator in active:
                        if row_index <= accumulator.definition.segment_end_row_index:
                            accumulator.append(row_index, row)
                        if row_index >= accumulator.definition.segment_end_row_index:
                            finished.append(accumulator.finish(log_metadata))
                        else:
                            still_active.append(accumulator)
                    active = still_active

                if not pending and not active:
                    break

        for accumulator in active:
            raise DataLayerError(
                "Segment extends beyond decoded log rows: "
                f"{accumulator.definition.segment_id} in {log_path}"
            )
        if pending:
            missing = ", ".join(segment.segment_id for segment in pending)
            raise DataLayerError(f"Segment start row not found before end of log {log_path}: {missing}")

        by_id = {stream.definition.segment_id: stream for stream in finished}
        for segment in sorted_segments:
            stream = by_id.get(segment.segment_id)
            if stream is None:
                raise DataLayerError(f"Segment produced no stream: {segment.segment_id}")
            yield stream


def load_json_object(path: Path) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise DataLayerError(f"Invalid JSON in {path}: {exc}") from exc
    if not isinstance(payload, dict):
        raise DataLayerError(f"Manifest root must be an object: {path}")
    return payload


def manifest_repo_root(manifest: dict[str, Any], manifest_path: Path) -> Path:
    raw_repo_root = manifest.get("repo_root")
    if raw_repo_root:
        repo_root = Path(str(raw_repo_root))
        if not repo_root.is_absolute():
            repo_root = manifest_path.parent / repo_root
        return repo_root.resolve()
    return REPO_ROOT


def required_list(payload: dict[str, Any], key: str) -> list[Any]:
    value = payload.get(key)
    if not isinstance(value, list):
        raise DataLayerError(f"Manifest field {key!r} must be a list")
    return value


def segment_definition(raw: Any, repo_root: Path) -> SegmentDefinition:
    if not isinstance(raw, dict):
        raise DataLayerError("Manifest segments must be objects")
    segment_id = required_text(raw, "segment_id")
    log_path = required_text(raw, "log_path")
    resolved_log_path = resolve_log_path(log_path, repo_root)
    return SegmentDefinition(
        segment_id=segment_id,
        log_path=log_path,
        log_absolute_path=resolved_log_path,
        segment_start_row_index=required_int(raw, "segment_start_row_index"),
        segment_end_row_index=required_int(raw, "segment_end_row_index"),
        segment_start_time_us=optional_int(raw.get("segment_start_time_us")),
        segment_end_time_us=optional_int(raw.get("segment_end_time_us")),
        active_start_row_index=optional_int(raw.get("active_start_row_index")),
        active_end_row_index=optional_int(raw.get("active_end_row_index")),
        active_start_time_us=optional_int(raw.get("active_start_time_us")),
        active_end_time_us=optional_int(raw.get("active_end_time_us")),
        slot_start_row_index=optional_int(raw.get("slot_start_row_index")),
        slot_end_row_index=optional_int(raw.get("slot_end_row_index")),
        stage=str(raw.get("stage", "")),
        stage_id=optional_int(raw.get("stage_id")),
        family=str(raw.get("family", "")),
        phase_id=optional_int(raw.get("phase_id")),
        phase_name=str(raw.get("phase_name", "")),
        primitive_id=optional_int(raw.get("primitive_id")),
        primitive_name=str(raw.get("primitive_name", "")),
        direction_id=optional_int(raw.get("direction_id")),
        direction_name=str(raw.get("direction_name", "")),
        repeat_index=optional_int(raw.get("repeat_index")),
        schema_kind=str(raw.get("schema_kind", "")),
        schema_version=optional_int(raw.get("schema_version")),
        speed_bin=optional_float(raw.get("speed_bin")),
        speed_bin_label=str(raw.get("speed_bin_label", "")),
        fan_duty_cycle=optional_float(raw.get("fan_duty_cycle")),
        fan_source=str(raw.get("fan_source", "")),
        start_criterion=str(raw.get("start_criterion", "")),
        end_criterion=str(raw.get("end_criterion", "")),
        end_reason=str(raw.get("end_reason", "")),
        corruption_note=optional_text(raw.get("corruption_note")),
        parameter_fields=dict(raw.get("parameter_fields") or {}),
        observed_command=dict(raw.get("observed_command") or {}),
    )


def resolve_log_path(path_text: str, repo_root: Path) -> Path:
    path = Path(path_text)
    if path.is_absolute():
        return path
    return (repo_root / path).resolve()


def required_text(payload: dict[str, Any], key: str) -> str:
    value = str(payload.get(key, "")).strip()
    if not value:
        raise DataLayerError(f"Manifest segment missing required field {key!r}")
    return value


def required_int(payload: dict[str, Any], key: str) -> int:
    value = optional_int(payload.get(key))
    if value is None:
        raise DataLayerError(f"Manifest segment missing integer field {key!r}")
    return value


def optional_int(value: Any) -> int | None:
    if value is None or str(value).strip() == "":
        return None
    try:
        return int(value)
    except (TypeError, ValueError) as exc:
        raise DataLayerError(f"Expected integer value, got {value!r}") from exc


def optional_float(value: Any) -> float | None:
    if value is None or str(value).strip() == "":
        return None
    try:
        return float(value)
    except (TypeError, ValueError) as exc:
        raise DataLayerError(f"Expected numeric value, got {value!r}") from exc


def optional_text(value: Any) -> str | None:
    if value is None:
        return None
    text = str(value)
    return text if text else None


def observable_columns(fieldnames: list[str]) -> dict[str, tuple[str, ...]]:
    available = set(fieldnames)
    result: dict[str, tuple[str, ...]] = {}
    for group, columns in COLUMN_GROUPS.items():
        selected = tuple(
            column
            for column in columns
            if column in available and not is_ignored_state_column(column)
        )
        result[group] = selected
    return result


def is_ignored_state_column(column_name: str) -> bool:
    lowered = column_name.strip().lower()
    return any(lowered.startswith(prefix) for prefix in IGNORED_STATE_COLUMN_PREFIXES)


def parse_cell(value: str | None) -> int | float | str | None:
    if value is None:
        return None
    text = str(value).strip()
    if text == "":
        return None
    try:
        result = int(text, 10)
        if str(result) == text:
            return result
    except ValueError:
        pass
    try:
        return float(text)
    except ValueError:
        return text


def read_sidecar_metadata(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    result: dict[str, Any] = {}
    with path.open(encoding="utf-8", errors="replace") as handle:
        for line in handle:
            text = line.strip()
            if not text or "=" not in text:
                continue
            key, value = text.split("=", 1)
            result[key.strip()] = parse_cell(value)
    return result


def boundary_metadata(definition: SegmentDefinition) -> dict[str, Any]:
    corrupted = is_terminal_external_force_boundary(definition)
    return {
        "segment": {
            "start_row_index": definition.segment_start_row_index,
            "end_row_index": definition.segment_end_row_index,
            "start_time_us": definition.segment_start_time_us,
            "end_time_us": definition.segment_end_time_us,
        },
        "active": {
            "start_row_index": definition.active_start_row_index,
            "end_row_index": definition.active_end_row_index,
            "start_time_us": definition.active_start_time_us,
            "end_time_us": definition.active_end_time_us,
        },
        "slot": {
            "start_row_index": definition.slot_start_row_index,
            "end_row_index": definition.slot_end_row_index,
        },
        "stationary_context": {
            "start_criterion": definition.start_criterion,
            "end_criterion": definition.end_criterion,
            "end_reason": definition.end_reason,
            "starts_from_stationary": "stationary" in definition.start_criterion.lower(),
            "ends_at_stationary": definition.end_reason == "reliable_stationary",
            "pre_active_row_range": pre_active_row_range(definition),
            "post_active_row_range": post_active_row_range(definition),
        },
        "corruption": {
            "is_corrupted": corrupted,
            "note": definition.corruption_note,
            "end_reason": definition.end_reason,
        },
    }


def is_terminal_external_force_boundary(definition: SegmentDefinition) -> bool:
    explicitly_marked = definition.end_reason == "corruption_boundary" or bool(definition.corruption_note)
    if not explicitly_marked:
        return False
    text = " ".join(
        (
            definition.end_reason,
            definition.end_criterion,
            definition.corruption_note or "",
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


def pre_active_row_range(definition: SegmentDefinition) -> list[int] | None:
    if definition.active_start_row_index is None:
        return [definition.segment_start_row_index, definition.segment_end_row_index]
    end = definition.active_start_row_index - 1
    if end < definition.segment_start_row_index:
        return None
    return [definition.segment_start_row_index, end]


def post_active_row_range(definition: SegmentDefinition) -> list[int] | None:
    if definition.active_end_row_index is None:
        return None
    start = definition.active_end_row_index + 1
    if start > definition.segment_end_row_index:
        return None
    return [start, definition.segment_end_row_index]
