#!/usr/bin/env python3
"""Generate open-floor segment manifests for RMS/NIS replay tests."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import re
from bisect import bisect_left, bisect_right
from collections import Counter
from collections import defaultdict
from dataclasses import dataclass
from datetime import UTC
from datetime import datetime
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ROOT = REPO_ROOT / "TestResults"
DEFAULT_OUTPUT = REPO_ROOT / "staging" / "traction_candidate_rms_nis_segments"

COMMAND_ZERO_EPS = 0.01
COMMAND_ACTIVE_EPS = 0.015
CMD_LINEAR_ACTIVE_EPS_MPS = 0.01
CMD_YAW_ACTIVE_EPS_RADPS = 0.05
STATIONARY_LINEAR_EPS_MPS = 0.035
STATIONARY_YAW_EPS_RADPS = 0.4
DEFAULT_MIN_STATIONARY_ROWS = 25

SECTION_NAMES = {
    0: "SEC_00_TIMING",
    1: "SEC_10_STATIC",
    2: "SEC_20_LAUNCH",
    3: "SEC_30_STRAIGHT",
    4: "SEC_40_YAW",
    5: "SEC_50_SMOOTH",
    6: "SEC_60_LOOP_CW",
    7: "SEC_70_LOOP_CCW",
}

LEGACY_PHASE_NAMES = {
    1: "hold",
    2: "launch_pulse",
    3: "recovery",
    4: "accel",
    5: "cruise",
    6: "brake",
    7: "startup",
    8: "steady_rotation",
    9: "stop",
    10: "entry",
    11: "middle",
    12: "exit",
}

LEGACY_PRIMITIVE_NAMES = {
    0: "NONE",
    1: "TIMING_NO_MOTION",
    2: "STATIC_HOLD",
    3: "OPEN_LOOP_LAUNCH",
    4: "STR1",
    5: "STR2",
    6: "STR4",
    7: "IP90",
    8: "IP90_M",
    9: "IP180",
    10: "S45SD",
    11: "S45SD_M",
    12: "S45SS",
    13: "S45SS_M",
    14: "S45LS",
    15: "S45LS_M",
    16: "S45LD",
    17: "S45LD_M",
    18: "S90SD",
    19: "S90SD_M",
    20: "S90SS",
    21: "S90SS_M",
    22: "S90LS",
    23: "S90LS_M",
    24: "S90LD",
    25: "S90LD_M",
    26: "S135SD",
    27: "S135SD_M",
    28: "S135SS",
    29: "S135SS_M",
    30: "S135LS",
    31: "S135LS_M",
    32: "S135LD",
    33: "S135LD_M",
    34: "S180SS",
    35: "S180SS_M",
    36: "S180LS",
    37: "S180LS_M",
    38: "RECOVERY",
}

BASE_MANEUVER_NAMES = {
    0: "MC_NONE",
    **{index: f"S{index}" for index in range(1, 32)},
    32: "IP45",
    33: "IP90",
    34: "IP135",
    35: "IP180",
    36: "S45SS",
    37: "S45SD",
    38: "S45LS",
    39: "S45LD",
    40: "S90SS",
    41: "S90SD",
    42: "S90LS",
    43: "S90LD",
    44: "S135SS",
    45: "S135SD",
    46: "S135LS",
    47: "S135LD",
    48: "S180SS",
    49: "S180LS",
    50: "S90ELD",
    51: "S180ELS",
}
COMPACT_PRIMITIVE_NAMES = dict(BASE_MANEUVER_NAMES)
for code, name in list(BASE_MANEUVER_NAMES.items()):
    if code >= 32:
        COMPACT_PRIMITIVE_NAMES[code | 0x80] = f"{name}_M"

DIRECTION_NAMES = {
    0: "NONE",
    1: "POSITIVE",
    2: "NEGATIVE",
    3: "NORTHBOUND",
    4: "SOUTHBOUND",
    5: "CLOCKWISE",
    6: "COUNTERCLOCKWISE",
    7: "FLIP",
    8: "LEFT",
    9: "RIGHT",
}

SPEED_BIN_LABELS = {
    0: "NONE",
    1: "LOW",
    2: "MEDIUM",
    3: "HIGH",
}

PRIMITIVE_FAMILY_NAMES = {
    0: "none",
    1: "timing",
    2: "static_hold",
    3: "launch",
    4: "straight",
    5: "in_place_turn",
    6: "smooth_turn",
    7: "recovery",
}

COMPACT_STAGE_NAMES = {
    1: "static",
    2: "launch",
    3: "straight",
    4: "yaw",
    5: "smooth",
    6: "loop_clockwise",
    7: "loop_counter_clockwise",
    20: "yaw_launch",
    25: "mixed_launch",
}

COMPACT_SECTION_NAMES = {
    1: "SEC_10_STATIC",
    2: "SEC_20_LAUNCH",
    3: "SEC_30_STRAIGHT",
    4: "SEC_40_YAW",
    5: "SEC_50_SMOOTH",
    6: "SEC_60_LOOP_CW",
    7: "SEC_70_LOOP_CCW",
    20: "SEC_20_YAW_LAUNCH",
    25: "SEC_20_MIXED_LAUNCH",
}

STRAIGHT_SPEED_BY_BIN = {1: 0.10, 2: 0.30, 3: 0.55}
YAW_RATE_BY_BIN = {1: 9.0, 2: 18.0, 3: 27.0}
SMOOTH_SPEED_BY_BIN = {1: 0.40, 2: 0.45, 3: 0.45}


@dataclass(frozen=True, slots=True)
class FaultInfo:
    time_us: int
    fault_class: str
    reason: str
    line: str


@dataclass(frozen=True, slots=True)
class SourceInfo:
    log_id: str
    csv_path: Path
    relative_csv_path: str
    sidecar_path: Path | None
    logging_path: Path | None
    metadata: dict[str, str]
    phase_names: dict[int, str]
    schema_kind: str
    file_size: int
    content_sha256: str | None


@dataclass(frozen=True, slots=True)
class RowInfo:
    row_index: int
    time_us: int
    tick: int | None
    dt_us: int | None
    schema_kind: str
    section_id: int | None
    section_name: str
    stage_id: int | None
    stage_name: str
    phase_id: int | None
    phase_name: str | None
    primitive_id: int | None
    primitive_name: str
    family: str
    direction_id: int | None
    direction_name: str | None
    speed_bin_raw: float | None
    speed_bin_label: str | None
    repeat_index: int | None
    start_marker_id: int | None
    fan_duty: float | None
    left_cmd: float | None
    right_cmd: float | None
    cmd_linear_mps: float | None
    cmd_yaw_radps: float | None
    measured_linear_mps: float | None
    measured_yaw_radps: float | None
    gyro_raw_radps: float | None
    left_velocity_mps: float | None
    right_velocity_mps: float | None
    slot_key: tuple[Any, ...]


@dataclass(frozen=True, slots=True)
class StationaryRun:
    start: int
    end: int

    @property
    def count(self) -> int:
        return self.end - self.start + 1


@dataclass(frozen=True, slots=True)
class SlotRun:
    start: int
    end: int
    occurrence_index: int

    @property
    def count(self) -> int:
        return self.end - self.start + 1


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate a segment manifest for open-floor RMS/NIS testing."
    )
    parser.add_argument("--root", type=Path, default=DEFAULT_ROOT)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--min-stationary-rows", type=int, default=DEFAULT_MIN_STATIONARY_ROWS)
    parser.add_argument("--include-duplicates", action="store_true")
    return parser.parse_args()


def parse_float(text: str | None) -> float | None:
    if text is None or text == "":
        return None
    try:
        value = float(text)
    except ValueError:
        return None
    return value if math.isfinite(value) else None


def parse_int(text: str | None) -> int | None:
    if text is None or text == "":
        return None
    try:
        return int(float(text))
    except ValueError:
        return None


def rounded_or_none(value: float | None, digits: int = 6) -> float | None:
    if value is None:
        return None
    return round(value, digits)


def relative_path(path: Path) -> str:
    try:
        return path.resolve().relative_to(REPO_ROOT.resolve()).as_posix()
    except ValueError:
        return path.as_posix()


def read_sidecar_metadata(path: Path) -> dict[str, str]:
    if not path.is_file():
        return {}
    metadata: dict[str, str] = {}
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if "=" not in line:
            break
        key, value = line.split("=", 1)
        metadata[key] = value
    return metadata


def parse_phase_names(metadata: dict[str, str]) -> dict[int, str]:
    by_ordinal: dict[str, dict[str, str]] = defaultdict(dict)
    for key, value in metadata.items():
        match = re.fullmatch(r"phase_battery_(\d+)_(name|phase_id)", key)
        if match is None:
            continue
        by_ordinal[match.group(1)][match.group(2)] = value
    result: dict[int, str] = {}
    for fields in by_ordinal.values():
        phase_id = parse_int(fields.get("phase_id"))
        name = fields.get("name")
        if phase_id is not None and name:
            result[phase_id] = normalize_name(name)
    return result


def normalize_name(text: str) -> str:
    return re.sub(r"[^a-z0-9]+", "_", text.strip().lower()).strip("_")


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as data:
        for chunk in iter(lambda: data.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def discover_sources(root: Path, include_duplicates: bool) -> tuple[list[SourceInfo], list[dict[str, Any]]]:
    csv_paths = sorted(root.rglob("open_floor_main.csv"))
    sizes: dict[int, list[Path]] = defaultdict(list)
    for path in csv_paths:
        sizes[path.stat().st_size].append(path)

    hashes_by_path: dict[Path, str | None] = {}
    duplicates: list[dict[str, Any]] = []
    canonical_by_hash: dict[str, Path] = {}
    for size, paths in sizes.items():
        if len(paths) == 1:
            hashes_by_path[paths[0]] = None
            continue
        for path in paths:
            digest = file_sha256(path)
            hashes_by_path[path] = digest
            canonical = canonical_by_hash.setdefault(digest, path)
            if canonical != path:
                duplicates.append(
                    {
                        "path": relative_path(path),
                        "duplicate_of": relative_path(canonical),
                        "file_size": size,
                        "sha256": digest,
                    }
                )

    duplicate_paths = {
        (REPO_ROOT / duplicate["path"]).resolve()
        for duplicate in duplicates
    }

    sources: list[SourceInfo] = []
    for path in csv_paths:
        if not include_duplicates and path.resolve() in duplicate_paths:
            continue
        sidecar_path = path.with_suffix(".sidecar")
        logging_path = path.with_name("logging.txt")
        metadata = read_sidecar_metadata(sidecar_path)
        with path.open("r", newline="", encoding="utf-8", errors="replace") as csv_file:
            header = csv_file.readline()
        if "master_time_us" not in header or "phase_id" not in header:
            continue
        schema_kind = "legacy_section_phase" if "section_id" in header else "compact_phase_battery"
        sources.append(
            SourceInfo(
                log_id=path.parent.name,
                csv_path=path,
                relative_csv_path=relative_path(path),
                sidecar_path=sidecar_path if sidecar_path.is_file() else None,
                logging_path=logging_path if logging_path.is_file() else None,
                metadata=metadata,
                phase_names=parse_phase_names(metadata),
                schema_kind=schema_kind,
                file_size=path.stat().st_size,
                content_sha256=hashes_by_path[path],
            )
        )
    return sources, duplicates


def parse_fault_info(path: Path | None) -> FaultInfo | None:
    if path is None or not path.is_file():
        return None
    best: FaultInfo | None = None
    pattern = re.compile(r"\[(\d+)\].*?\bfault:\s*(.*)", re.IGNORECASE)
    for raw_line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if "fault:" not in raw_line.lower():
            continue
        match = pattern.search(raw_line)
        if match is None:
            continue
        time_us = int(match.group(1))
        reason = re.sub(r"\s+", " ", match.group(2).strip())
        fault = FaultInfo(
            time_us=time_us,
            fault_class=classify_fault(reason),
            reason=reason,
            line=raw_line.strip(),
        )
        if best is None or fault.time_us < best.time_us:
            best = fault
    return best


def classify_fault(reason: str) -> str:
    lower = reason.lower()
    if "selector" in lower and "removed" in lower:
        return "selector_removed"
    if "workspace" in lower:
        return "workspace_violation"
    if "recovery" in lower and ("timeout" in lower or "timed out" in lower):
        return "recovery_timed_out"
    if "runoff" in lower:
        return "runoff"
    if "stall" in lower:
        return "watchdog_stall"
    if "bound" in lower:
        return "bound_exceeded"
    if "capture failed" in lower or "setup failed" in lower:
        return "capture_or_setup_failed"
    if "timed out" in lower or "timeout" in lower:
        return "timed_out"
    return "fault"


def primitive_name(schema_kind: str, primitive_id: int | None) -> str:
    if primitive_id is None:
        return "UNKNOWN"
    if schema_kind == "legacy_section_phase":
        return LEGACY_PRIMITIVE_NAMES.get(primitive_id, f"legacy_{primitive_id}")
    return COMPACT_PRIMITIVE_NAMES.get(primitive_id, f"maneuver_{primitive_id}")


def compact_family(stage_id: int | None, primitive_id: int | None) -> str:
    if stage_id == 1:
        return "static_hold"
    if stage_id in (2, 20, 25):
        return "launch"
    if primitive_id is None:
        return "unknown"
    base_code = primitive_id & 0x7F
    if 1 <= base_code <= 31:
        return "straight"
    if base_code in (32, 33, 34, 35):
        return "in_place_turn"
    if base_code in range(36, 52):
        return "smooth_turn"
    return "none"


def legacy_family(row: dict[str, str], section_id: int | None, primitive_id: int | None) -> str:
    primitive_family = parse_int(row.get("primitive_family"))
    if primitive_family is not None:
        return PRIMITIVE_FAMILY_NAMES.get(primitive_family, f"family_{primitive_family}")
    if primitive_id == 38:
        return "recovery"
    if section_id == 1:
        return "static_hold"
    if section_id == 2:
        return "launch"
    if section_id == 3:
        return "straight"
    if section_id == 4:
        return "in_place_turn"
    if section_id in (5, 6, 7):
        return "smooth_turn"
    return "unknown"


def row_to_info(row: dict[str, str], row_index: int, source: SourceInfo) -> RowInfo | None:
    time_us = parse_int(row.get("master_time_us"))
    if time_us is None:
        return None
    phase_id = parse_int(row.get("phase_id"))
    primitive_id = parse_int(row.get("primitive_id"))
    direction_id = parse_int(row.get("direction_id"))
    repeat_index = parse_int(row.get("repeat_index"))
    speed_bin_raw = parse_float(row.get("speed_bin"))
    fan_duty = parse_float(row.get("fan_duty_cycle"))
    if fan_duty is None:
        fan_duty = parse_float(source.metadata.get("fan_duty_cycle"))

    if source.schema_kind == "legacy_section_phase":
        section_id = parse_int(row.get("section_id"))
        section_name = SECTION_NAMES.get(section_id, f"SEC_{section_id}") if section_id is not None else "UNKNOWN"
        stage_id = section_id
        stage_name = section_name
        phase_name = LEGACY_PHASE_NAMES.get(phase_id) if phase_id is not None else None
        speed_bin_label = (
            SPEED_BIN_LABELS.get(int(speed_bin_raw))
            if speed_bin_raw is not None and float(speed_bin_raw).is_integer()
            else None
        )
        family = legacy_family(row, section_id, primitive_id)
        slot_key = (
            source.schema_kind,
            section_id,
            primitive_id,
            direction_id,
            round(speed_bin_raw, 6) if speed_bin_raw is not None else None,
            parse_int(row.get("start_marker_id")),
            repeat_index,
        )
    else:
        section_id = phase_id
        section_name = COMPACT_SECTION_NAMES.get(phase_id, f"COMPACT_PHASE_{phase_id}") if phase_id is not None else "UNKNOWN"
        stage_id = phase_id
        stage_name = source.phase_names.get(phase_id) or COMPACT_STAGE_NAMES.get(phase_id, section_name.lower())
        phase_name = stage_name
        speed_bin_label = None
        family = compact_family(stage_id, primitive_id)
        slot_key = (
            source.schema_kind,
            phase_id,
            primitive_id,
            round(speed_bin_raw, 6) if speed_bin_raw is not None else None,
            repeat_index,
        )

    return RowInfo(
        row_index=row_index,
        time_us=time_us,
        tick=parse_int(row.get("control_tick_sequence")),
        dt_us=parse_int(row.get("dt_us")),
        schema_kind=source.schema_kind,
        section_id=section_id,
        section_name=section_name,
        stage_id=stage_id,
        stage_name=stage_name,
        phase_id=phase_id,
        phase_name=phase_name,
        primitive_id=primitive_id,
        primitive_name=primitive_name(source.schema_kind, primitive_id),
        family=family,
        direction_id=direction_id,
        direction_name=DIRECTION_NAMES.get(direction_id) if direction_id is not None else None,
        speed_bin_raw=speed_bin_raw,
        speed_bin_label=speed_bin_label,
        repeat_index=repeat_index,
        start_marker_id=parse_int(row.get("start_marker_id")),
        fan_duty=fan_duty,
        left_cmd=parse_float(row.get("left_drive_command")),
        right_cmd=parse_float(row.get("right_drive_command")),
        cmd_linear_mps=parse_float(row.get("cmd_linear_mps")),
        cmd_yaw_radps=parse_float(row.get("cmd_yaw_rate_radps") or row.get("cmd_angular_radps")),
        measured_linear_mps=parse_float(row.get("measured_linear_speed_mps")),
        measured_yaw_radps=parse_float(row.get("measured_yaw_rate_radps") or row.get("measured_angular_speed_radps")),
        gyro_raw_radps=parse_float(row.get("gyro_raw_radps")),
        left_velocity_mps=parse_float(row.get("left_encoder_velocity_mps")),
        right_velocity_mps=parse_float(row.get("right_encoder_velocity_mps")),
        slot_key=slot_key,
    )


def load_rows(source: SourceInfo) -> list[RowInfo]:
    rows: list[RowInfo] = []
    with source.csv_path.open("r", newline="", encoding="utf-8", errors="replace") as csv_file:
        reader = csv.DictReader(csv_file)
        for row_index, row in enumerate(reader):
            info = row_to_info(row, row_index, source)
            if info is not None:
                rows.append(info)
    return rows


def is_active_command(row: RowInfo) -> bool:
    drive_values = [value for value in (row.left_cmd, row.right_cmd) if value is not None]
    if drive_values and max(abs(value) for value in drive_values) > COMMAND_ACTIVE_EPS:
        return True
    if row.cmd_linear_mps is not None and abs(row.cmd_linear_mps) > CMD_LINEAR_ACTIVE_EPS_MPS:
        return True
    if row.cmd_yaw_radps is not None and abs(row.cmd_yaw_radps) > CMD_YAW_ACTIVE_EPS_RADPS:
        return True
    return False


def is_stationary(row: RowInfo) -> bool:
    command_values = [value for value in (row.left_cmd, row.right_cmd) if value is not None]
    command_zero = not command_values or max(abs(value) for value in command_values) <= COMMAND_ZERO_EPS
    if row.cmd_linear_mps is not None and abs(row.cmd_linear_mps) > CMD_LINEAR_ACTIVE_EPS_MPS:
        command_zero = False
    if row.cmd_yaw_radps is not None and abs(row.cmd_yaw_radps) > CMD_YAW_ACTIVE_EPS_RADPS:
        command_zero = False
    if not command_zero:
        return False

    linear_values = [
        value
        for value in (row.measured_linear_mps, row.left_velocity_mps, row.right_velocity_mps)
        if value is not None
    ]
    if linear_values and max(abs(value) for value in linear_values) > STATIONARY_LINEAR_EPS_MPS:
        return False

    yaw_values = [value for value in (row.measured_yaw_radps, row.gyro_raw_radps) if value is not None]
    if yaw_values and max(abs(value) for value in yaw_values) > STATIONARY_YAW_EPS_RADPS:
        return False
    return True


def stationary_runs(rows: list[RowInfo], min_rows: int) -> list[StationaryRun]:
    result: list[StationaryRun] = []
    start: int | None = None
    for index, row in enumerate(rows):
        if is_stationary(row):
            if start is None:
                start = index
        elif start is not None:
            if index - start >= min_rows:
                result.append(StationaryRun(start, index - 1))
            start = None
    if start is not None and len(rows) - start >= min_rows:
        result.append(StationaryRun(start, len(rows) - 1))
    return result


def slot_runs(rows: list[RowInfo]) -> list[SlotRun]:
    if not rows:
        return []
    result: list[SlotRun] = []
    occurrence_counts: Counter[tuple[Any, ...]] = Counter()
    start = 0
    key = rows[0].slot_key
    for index in range(1, len(rows)):
        if rows[index].slot_key != key:
            occurrence_counts[key] += 1
            result.append(SlotRun(start, index - 1, occurrence_counts[key]))
            start = index
            key = rows[index].slot_key
    occurrence_counts[key] += 1
    result.append(SlotRun(start, len(rows) - 1, occurrence_counts[key]))
    return result


def valid_end_index(rows: list[RowInfo], fault: FaultInfo | None) -> int:
    if not rows:
        return -1
    if fault is None:
        return len(rows) - 1
    times = [row.time_us for row in rows]
    index = bisect_right(times, fault.time_us) - 1
    return max(-1, min(index, len(rows) - 1))


def preceding_stationary_run(runs: list[StationaryRun], index: int) -> StationaryRun | None:
    ends = [run.end for run in runs]
    position = bisect_left(ends, index) - 1
    return runs[position] if position >= 0 else None


def following_stationary_run(runs: list[StationaryRun], index: int) -> StationaryRun | None:
    starts = [run.start for run in runs]
    position = bisect_right(starts, index)
    return runs[position] if position < len(runs) else None


def has_later_active_command(rows: list[RowInfo], index: int) -> bool:
    return any(is_active_command(row) for row in rows[index + 1 :])


def containing_stationary_runs(runs: list[StationaryRun], start: int, end: int) -> list[StationaryRun]:
    return [run for run in runs if run.start <= end and run.end >= start]


def median(values: list[float]) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    middle = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[middle]
    return 0.5 * (ordered[middle - 1] + ordered[middle])


def command_stats(rows: list[RowInfo], active_indexes: list[int], family: str, stage_name: str) -> dict[str, Any]:
    sample_rows = [rows[index] for index in active_indexes] if active_indexes else []
    left_values = [row.left_cmd for row in sample_rows if row.left_cmd is not None]
    right_values = [row.right_cmd for row in sample_rows if row.right_cmd is not None]
    cmd_linear_values = [row.cmd_linear_mps for row in sample_rows if row.cmd_linear_mps is not None]
    cmd_yaw_values = [row.cmd_yaw_radps for row in sample_rows if row.cmd_yaw_radps is not None]
    pair_counts: Counter[tuple[float, float]] = Counter()
    for row in sample_rows:
        if row.left_cmd is None or row.right_cmd is None:
            continue
        pair_counts[(round(row.left_cmd, 6), round(row.right_cmd, 6))] += 1

    mode_pair: tuple[float, float] | None = None
    if pair_counts:
        mode_pair = pair_counts.most_common(1)[0][0]
    magnitude_values = [
        max(abs(left), abs(right))
        for left, right in pair_counts.elements()
    ]
    if not magnitude_values:
        magnitude_values = [
            max(abs(row.left_cmd or 0.0), abs(row.right_cmd or 0.0))
            for row in sample_rows
            if row.left_cmd is not None or row.right_cmd is not None
        ]
    pair_limit = 64 if family == "launch" or "launch" in stage_name else 16
    pair_items = pair_counts.most_common(pair_limit)
    return {
        "active_command_rows": len(sample_rows),
        "left_drive_command_min": min(left_values) if left_values else None,
        "left_drive_command_max": max(left_values) if left_values else None,
        "left_drive_command_mean": (sum(left_values) / len(left_values)) if left_values else None,
        "left_drive_command_median": median(left_values),
        "right_drive_command_min": min(right_values) if right_values else None,
        "right_drive_command_max": max(right_values) if right_values else None,
        "right_drive_command_mean": (sum(right_values) / len(right_values)) if right_values else None,
        "right_drive_command_median": median(right_values),
        "cmd_linear_mps_median": median(cmd_linear_values),
        "cmd_yaw_radps_median": median(cmd_yaw_values),
        "observed_command_pair_mode": list(mode_pair) if mode_pair is not None else None,
        "observed_command_pair_distinct_count": len(pair_counts),
        "observed_command_pairs": [
            {"left": left, "right": right, "rows": count}
            for (left, right), count in pair_items
        ],
        "observed_command_pairs_truncated": len(pair_counts) > pair_limit,
        "observed_command_magnitude_max": max(magnitude_values) if magnitude_values else None,
        "observed_command_magnitude_median": median(magnitude_values),
    }


def infer_test_value(row: RowInfo, stats: dict[str, Any]) -> dict[str, Any]:
    stage_name = row.stage_name
    if row.schema_kind == "compact_phase_battery":
        if stage_name in {"launch", "yaw_launch"}:
            return {
                "parameter": "drive_command_bin",
                "test_value_kind": "logged_drive_command_bin",
                "test_value": rounded_or_none(row.speed_bin_raw),
            }
        if stage_name == "mixed_launch":
            return {
                "parameter": "observed_drive_command_pair",
                "test_value_kind": "observed_drive_command_pair",
                "test_value": stats.get("observed_command_pair_mode"),
            }
        if stage_name in {"straight", "smooth", "loop_clockwise", "loop_counter_clockwise"}:
            return {
                "parameter": "speed_mps",
                "test_value_kind": "logged_speed_mps",
                "test_value": rounded_or_none(row.speed_bin_raw),
            }
        if stage_name == "yaw":
            return {
                "parameter": "yaw_rate_radps",
                "test_value_kind": "logged_yaw_rate_radps",
                "test_value": rounded_or_none(row.speed_bin_raw),
            }
    speed_id = int(row.speed_bin_raw) if row.speed_bin_raw is not None and row.speed_bin_raw.is_integer() else None
    if row.section_id == 2:
        return {
            "parameter": "observed_drive_command_magnitude",
            "test_value_kind": "observed_drive_command_magnitude",
            "test_value": rounded_or_none(stats.get("observed_command_magnitude_median")),
        }
    if row.section_id == 3:
        return {
            "parameter": "straight_speed_mps",
            "test_value_kind": "legacy_straight_speed_bin_mps",
            "test_value": STRAIGHT_SPEED_BY_BIN.get(speed_id),
        }
    if row.section_id == 4:
        return {
            "parameter": "yaw_rate_radps",
            "test_value_kind": "legacy_yaw_rate_bin_radps",
            "test_value": YAW_RATE_BY_BIN.get(speed_id),
        }
    if row.section_id in (5, 6, 7):
        return {
            "parameter": "smooth_speed_mps",
            "test_value_kind": "legacy_smooth_speed_bin_mps",
            "test_value": SMOOTH_SPEED_BY_BIN.get(speed_id),
        }
    return {"parameter": "none", "test_value_kind": "none", "test_value": None}


def fan_summary(rows: list[RowInfo], start: int, end: int, source: SourceInfo) -> dict[str, Any]:
    row_values = [rows[index].fan_duty for index in range(start, end + 1) if rows[index].fan_duty is not None]
    if row_values:
        return {
            "fan_duty_cycle": median(row_values),
            "fan_source": "row_or_sidecar",
        }
    sidecar_value = parse_float(source.metadata.get("fan_duty_cycle"))
    return {
        "fan_duty_cycle": sidecar_value,
        "fan_source": "sidecar" if sidecar_value is not None else "unavailable",
    }


def make_segment(
    source: SourceInfo,
    rows: list[RowInfo],
    run: SlotRun,
    active_indexes: list[int],
    start_index: int,
    end_index: int,
    active_start_index: int | None,
    active_end_index: int | None,
    start_criterion: str,
    end_criterion: str,
    end_reason: str,
    corruption_note: str | None,
) -> dict[str, Any]:
    metadata_row = rows[active_start_index if active_start_index is not None else run.start]
    stats = command_stats(rows, active_indexes, metadata_row.family, metadata_row.stage_name)
    test_value = infer_test_value(metadata_row, stats)
    unique_phase_ids = sorted({row.phase_id for row in rows[run.start : run.end + 1] if row.phase_id is not None})
    segment = {
        "schema_version": 1,
        "segment_id": "",
        "log_id": source.log_id,
        "log_path": source.relative_csv_path,
        "schema_kind": source.schema_kind,
        "stage_id": metadata_row.stage_id,
        "stage": metadata_row.stage_name,
        "family": metadata_row.family,
        "section_id": metadata_row.section_id,
        "section_name": metadata_row.section_name,
        "phase_id": metadata_row.phase_id,
        "phase_name": metadata_row.phase_name,
        "phase_ids_in_slot": unique_phase_ids,
        "primitive_id": metadata_row.primitive_id,
        "primitive_name": metadata_row.primitive_name,
        "direction_id": metadata_row.direction_id,
        "direction_name": metadata_row.direction_name,
        "speed_bin": metadata_row.speed_bin_raw,
        "speed_bin_label": metadata_row.speed_bin_label,
        "repeat_index": metadata_row.repeat_index,
        "slot_occurrence_index": run.occurrence_index,
        "slot_start_row_index": rows[run.start].row_index,
        "slot_end_row_index": rows[run.end].row_index,
        "slot_row_count": run.count,
        "segment_start_row_index": rows[start_index].row_index,
        "segment_end_row_index": rows[end_index].row_index,
        "segment_row_count": end_index - start_index + 1,
        "segment_start_time_us": rows[start_index].time_us,
        "segment_end_time_us": rows[end_index].time_us,
        "segment_duration_s": (rows[end_index].time_us - rows[start_index].time_us) * 1.0e-6,
        "active_start_row_index": rows[active_start_index].row_index if active_start_index is not None else None,
        "active_end_row_index": rows[active_end_index].row_index if active_end_index is not None else None,
        "active_start_time_us": rows[active_start_index].time_us if active_start_index is not None else None,
        "active_end_time_us": rows[active_end_index].time_us if active_end_index is not None else None,
        "start_criterion": start_criterion,
        "end_criterion": end_criterion,
        "end_reason": end_reason,
        "corruption_note": corruption_note,
        "parameter_fields": {
            "parameter": test_value["parameter"],
            "test_value_kind": test_value["test_value_kind"],
            "test_value": test_value["test_value"],
            "speed_bin": metadata_row.speed_bin_raw,
            "speed_bin_label": metadata_row.speed_bin_label,
            "cmd_linear_mps_median": stats["cmd_linear_mps_median"],
            "cmd_yaw_radps_median": stats["cmd_yaw_radps_median"],
        },
        "observed_command": stats,
        **fan_summary(rows, start_index, end_index, source),
    }
    return json_ready(segment)


def json_ready(value: Any) -> Any:
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {key: json_ready(item) for key, item in value.items()}
    if isinstance(value, list):
        return [json_ready(item) for item in value]
    return value


def build_segments_for_source(source: SourceInfo, min_stationary_rows: int) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    all_rows = load_rows(source)
    fault = parse_fault_info(source.logging_path)
    end_index = valid_end_index(all_rows, fault)
    rows = all_rows[: end_index + 1] if end_index >= 0 else []
    if not rows:
        return [], {
            "log_id": source.log_id,
            "path": source.relative_csv_path,
            "schema_kind": source.schema_kind,
            "row_count": len(all_rows),
            "kept_row_count": 0,
            "segments": 0,
            "fault": fault_to_dict(fault),
            "skip_reason": "no rows before terminal fault boundary",
        }

    stationary = stationary_runs(rows, min_stationary_rows)
    slots = slot_runs(rows)
    segments: list[dict[str, Any]] = []
    skipped_no_start = 0
    skipped_no_end = 0
    skipped_no_motion = 0

    for slot in slots:
        slot_rows = rows[slot.start : slot.end + 1]
        active_indexes = [
            slot.start + offset
            for offset, row in enumerate(slot_rows)
            if is_active_command(row)
        ]
        if not active_indexes:
            contained_stationary = containing_stationary_runs(stationary, slot.start, slot.end)
            first = slot_rows[0]
            if first.family == "static_hold" and contained_stationary:
                start_run = contained_stationary[0]
                end_run = contained_stationary[-1]
                segments.append(
                    make_segment(
                        source,
                        rows,
                        slot,
                        [],
                        start_run.start,
                        end_run.end,
                        None,
                        None,
                        f"static reliable_stationary_run rows={start_run.count}",
                        f"static reliable_stationary_run rows={end_run.count}",
                        "reliable_stationary",
                        None,
                    )
                )
            else:
                skipped_no_motion += 1
            continue

        active_start = active_indexes[0]
        active_end = active_indexes[-1]
        start_run = preceding_stationary_run(stationary, active_start)
        if start_run is None:
            skipped_no_start += 1
            continue
        next_stationary = following_stationary_run(stationary, active_end)
        if next_stationary is not None:
            end_reason = "reliable_stationary"
            end = next_stationary.end
            end_criterion = (
                f"first reliable stationary run after active command; "
                f"rows={next_stationary.count}; row_index={rows[next_stationary.start].row_index}..{rows[next_stationary.end].row_index}"
            )
            corruption_note = None
        elif (
            fault is not None
            and rows[active_end].time_us <= fault.time_us
            and not has_later_active_command(rows, active_end)
        ):
            end_reason = "corruption_boundary"
            end = len(rows) - 1
            end_criterion = (
                f"predetermined terminal fault boundary at {fault.time_us} us; "
                f"kept last row not after fault"
            )
            corruption_note = f"{fault.fault_class}: {fault.reason}"
        else:
            skipped_no_end += 1
            continue
        segments.append(
            make_segment(
                source,
                rows,
                slot,
                active_indexes,
                start_run.end,
                end,
                active_start,
                active_end,
                (
                    f"last row of preceding reliable stationary run; rows={start_run.count}; "
                    f"row_index={rows[start_run.start].row_index}..{rows[start_run.end].row_index}"
                ),
                end_criterion,
                end_reason,
                corruption_note,
            )
        )

    source_summary = {
        "log_id": source.log_id,
        "path": source.relative_csv_path,
        "schema_kind": source.schema_kind,
        "format_version": source.metadata.get("format_version"),
        "file_size": source.file_size,
        "sha256": source.content_sha256,
        "row_count": len(all_rows),
        "kept_row_count": len(rows),
        "rows_after_fault_excluded": len(all_rows) - len(rows),
        "stationary_run_count": len(stationary),
        "slot_run_count": len(slots),
        "segments": len(segments),
        "skipped_no_motion_slots": skipped_no_motion,
        "skipped_no_reliable_start": skipped_no_start,
        "skipped_no_reliable_end": skipped_no_end,
        "fault": fault_to_dict(fault),
    }
    return segments, json_ready(source_summary)


def fault_to_dict(fault: FaultInfo | None) -> dict[str, Any] | None:
    if fault is None:
        return None
    return {
        "time_us": fault.time_us,
        "fault_class": fault.fault_class,
        "reason": fault.reason,
        "line": fault.line,
    }


def flatten_segment(segment: dict[str, Any]) -> dict[str, Any]:
    observed = segment["observed_command"]
    params = segment["parameter_fields"]
    return {
        "segment_id": segment["segment_id"],
        "log_id": segment["log_id"],
        "log_path": segment["log_path"],
        "schema_kind": segment["schema_kind"],
        "stage": segment["stage"],
        "family": segment["family"],
        "section_id": segment["section_id"],
        "section_name": segment["section_name"],
        "phase_id": segment["phase_id"],
        "phase_name": segment["phase_name"],
        "primitive_id": segment["primitive_id"],
        "primitive_name": segment["primitive_name"],
        "direction_id": segment["direction_id"],
        "direction_name": segment["direction_name"],
        "speed_bin": segment["speed_bin"],
        "speed_bin_label": segment["speed_bin_label"],
        "repeat_index": segment["repeat_index"],
        "slot_occurrence_index": segment["slot_occurrence_index"],
        "parameter": params["parameter"],
        "test_value_kind": params["test_value_kind"],
        "test_value": json.dumps(params["test_value"]) if isinstance(params["test_value"], list) else params["test_value"],
        "fan_duty_cycle": segment["fan_duty_cycle"],
        "active_command_rows": observed["active_command_rows"],
        "observed_command_pair_mode": json.dumps(observed["observed_command_pair_mode"]),
        "observed_command_pair_distinct_count": observed["observed_command_pair_distinct_count"],
        "observed_command_magnitude_max": observed["observed_command_magnitude_max"],
        "observed_command_magnitude_median": observed["observed_command_magnitude_median"],
        "left_drive_command_min": observed["left_drive_command_min"],
        "left_drive_command_max": observed["left_drive_command_max"],
        "right_drive_command_min": observed["right_drive_command_min"],
        "right_drive_command_max": observed["right_drive_command_max"],
        "cmd_linear_mps_median": params["cmd_linear_mps_median"],
        "cmd_yaw_radps_median": params["cmd_yaw_radps_median"],
        "segment_start_row_index": segment["segment_start_row_index"],
        "segment_end_row_index": segment["segment_end_row_index"],
        "segment_start_time_us": segment["segment_start_time_us"],
        "segment_end_time_us": segment["segment_end_time_us"],
        "segment_duration_s": segment["segment_duration_s"],
        "active_start_row_index": segment["active_start_row_index"],
        "active_end_row_index": segment["active_end_row_index"],
        "active_start_time_us": segment["active_start_time_us"],
        "active_end_time_us": segment["active_end_time_us"],
        "start_criterion": segment["start_criterion"],
        "end_criterion": segment["end_criterion"],
        "end_reason": segment["end_reason"],
        "corruption_note": segment["corruption_note"],
    }


def write_csv(path: Path, segments: list[dict[str, Any]]) -> None:
    rows = [flatten_segment(segment) for segment in segments]
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    with path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def render_summary(
    manifest: dict[str, Any],
    segments: list[dict[str, Any]],
    source_summaries: list[dict[str, Any]],
    duplicate_sources: list[dict[str, Any]],
) -> str:
    stage_counts = Counter(segment["stage"] for segment in segments)
    family_counts = Counter(segment["family"] for segment in segments)
    schema_counts = Counter(summary["schema_kind"] for summary in source_summaries)
    end_counts = Counter(segment["end_reason"] for segment in segments)
    fault_counts = Counter(
        summary["fault"]["fault_class"]
        for summary in source_summaries
        if summary.get("fault") is not None
    )
    lines = [
        "# Open-Floor RMS/NIS Segment Manifest Summary",
        "",
        f"- Sources discovered: {manifest['source_count_discovered']}",
        f"- Sources included after duplicate collapse: {len(source_summaries)}",
        f"- Duplicate sources collapsed: {len(duplicate_sources)}",
        f"- Segments emitted: {len(segments)}",
        f"- Segments ending at reliable stationary points: {end_counts.get('reliable_stationary', 0)}",
        f"- Segments ending at corruption boundaries: {end_counts.get('corruption_boundary', 0)}",
        "",
        "## Method",
        "",
        "- Legacy logs with `section_id` are normalized as `legacy_section_phase`; compact logs without `section_id` are normalized from sidecar `phase_battery_*` metadata.",
        "- Segmentation ignores every `ukf_state_*` column. Boundaries use CSV metadata, drive commands, encoder/gyro stillness, and parsed terminal fault timestamps.",
        f"- Reliable stationary runs require at least {manifest['thresholds']['min_stationary_rows']} consecutive rows with zero command and low direct sensor motion.",
        "- Selector-removal, workspace, recovery-timeout, runoff-like, and related terminal faults trim only the terminal segment; earlier non-corrupted segments remain valid.",
        "",
        "## Schemas",
        "",
        "| schema | sources |",
        "| --- | ---: |",
    ]
    for schema, count in sorted(schema_counts.items()):
        lines.append(f"| {schema} | {count} |")
    lines.extend(["", "## Stages", "", "| stage | segments |", "| --- | ---: |"])
    for stage, count in sorted(stage_counts.items()):
        lines.append(f"| {stage} | {count} |")
    lines.extend(["", "## Families", "", "| family | segments |", "| --- | ---: |"])
    for family, count in sorted(family_counts.items()):
        lines.append(f"| {family} | {count} |")
    lines.extend(["", "## Terminal Faults", "", "| fault_class | sources |", "| --- | ---: |"])
    for fault_class, count in sorted(fault_counts.items()):
        lines.append(f"| {fault_class} | {count} |")
    if duplicate_sources:
        lines.extend(["", "## Collapsed Duplicates", "", "| skipped path | duplicate of |", "| --- | --- |"])
        for duplicate in duplicate_sources[:20]:
            lines.append(f"| `{duplicate['path']}` | `{duplicate['duplicate_of']}` |")
        if len(duplicate_sources) > 20:
            lines.append(f"| ... | {len(duplicate_sources) - 20} more |")
    lines.extend([
        "",
        "## Artifacts",
        "",
        "- `segment_manifest.json`: full machine-readable manifest with nested command summaries.",
        "- `segment_manifest.csv`: flat table for quick filtering.",
        "- `summary.md`: this report.",
    ])
    return "\n".join(lines) + "\n"


def main() -> int:
    args = parse_args()
    root = args.root.resolve()
    output = args.output.resolve()
    sources, duplicate_sources = discover_sources(root, args.include_duplicates)
    all_sources_count = len(list(root.rglob("open_floor_main.csv")))

    all_segments: list[dict[str, Any]] = []
    source_summaries: list[dict[str, Any]] = []
    for source in sources:
        segments, summary = build_segments_for_source(source, args.min_stationary_rows)
        source_summaries.append(summary)
        for segment in segments:
            segment["segment_id"] = f"ofnis_{len(all_segments) + 1:06d}"
            all_segments.append(segment)

    manifest = {
        "schema_version": 1,
        "generated_at_utc": datetime.now(UTC).replace(microsecond=0).isoformat().replace("+00:00", "Z"),
        "repo_root": str(REPO_ROOT),
        "test_results_root": relative_path(root),
        "source_count_discovered": all_sources_count,
        "source_count_included": len(source_summaries),
        "segment_count": len(all_segments),
        "thresholds": {
            "min_stationary_rows": args.min_stationary_rows,
            "command_zero_eps": COMMAND_ZERO_EPS,
            "command_active_eps": COMMAND_ACTIVE_EPS,
            "cmd_linear_active_eps_mps": CMD_LINEAR_ACTIVE_EPS_MPS,
            "cmd_yaw_active_eps_radps": CMD_YAW_ACTIVE_EPS_RADPS,
            "stationary_linear_eps_mps": STATIONARY_LINEAR_EPS_MPS,
            "stationary_yaw_eps_radps": STATIONARY_YAW_EPS_RADPS,
        },
        "segmentation_inputs": {
            "ignored_columns_pattern": "ukf_state_*",
            "uses_logged_ukf_state_for_segmentation": False,
            "fault_source": "sibling logging.txt fault timestamps",
            "duplicate_policy": "collapse exact SHA-256 matches when file sizes match",
        },
        "duplicate_sources": duplicate_sources,
        "sources": source_summaries,
        "segments": all_segments,
    }

    output.mkdir(parents=True, exist_ok=True)
    json_path = output / "segment_manifest.json"
    csv_path = output / "segment_manifest.csv"
    summary_path = output / "summary.md"
    json_path.write_text(json.dumps(manifest, indent=2, sort_keys=True), encoding="utf-8")
    write_csv(csv_path, all_segments)
    summary_path.write_text(
        render_summary(manifest, all_segments, source_summaries, duplicate_sources),
        encoding="utf-8",
    )

    print(f"wrote {relative_path(json_path)}")
    print(f"wrote {relative_path(csv_path)}")
    print(f"wrote {relative_path(summary_path)}")
    print(f"sources={len(source_summaries)} segments={len(all_segments)} duplicates={len(duplicate_sources)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
