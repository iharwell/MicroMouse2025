#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
import math
import re
import statistics
from collections import Counter
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path

from open_floor_recovery import IMU_POSITION_BODY_X_M
from open_floor_recovery import IMU_POSITION_BODY_Y_M


STATIC_SECTION_ID = 1
STATIC_PRIMITIVE_ID = 2
YAW_SECTION_ID = 4

PHASE_STARTUP_ID = 7
PHASE_STEADY_ID = 8
PHASE_STOP_ID = 9

DIRECTION_CLOCKWISE_ID = 5
DIRECTION_COUNTERCLOCKWISE_ID = 6

PRIMITIVE_NAMES = {
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

YAW_CLOCKWISE_PRIMITIVE_IDS = {7, 9}
YAW_COUNTERCLOCKWISE_PRIMITIVE_IDS = {8, 37}


@dataclass(frozen=True)
class VehicleTurnGeometry:
    width_m: float
    effective_track_width_m: float
    tire_bank_inner_x_m: float
    tire_bank_outer_x_m: float
    imu_position_body_x_m: float
    imu_position_body_y_m: float


@dataclass(frozen=True)
class SensorBiasSummary:
    gyro_bias_radps: float
    accel_bias_x_mps2: float
    accel_bias_y_mps2: float
    static_row_count: int


@dataclass(frozen=True)
class TurnBiasThresholds:
    center_band_m: float = 0.005
    core_min_abs_gyro_radps: float = 4.0
    core_peak_fraction: float = 0.35
    trail_min_abs_gyro_radps: float = 1.0
    min_pivot_run_samples: int = 3


@dataclass(frozen=True)
class TurnKey:
    primitive_id: int
    repeat_index: int
    direction_id: int
    speed_bin: int


@dataclass(frozen=True)
class TurnSample:
    phase_id: int
    dt_s: float
    master_time_s: float
    corrected_gyro_radps: float
    gyro_alpha_radps2: float
    left_velocity_mps: float
    right_velocity_mps: float
    average_velocity_mps: float
    encoder_yaw_radps: float
    center_accel_x_mps2: float
    center_accel_y_mps2: float
    cor_x_encoder_gyro_m: float | None
    cor_x_encoder_encoder_m: float | None
    cor_x_accel_gyro_m: float | None


@dataclass(frozen=True)
class WindowBiasSummary:
    label: str
    sample_count: int
    fused_median_cor_x_m: float | None
    fused_classification: str
    encoder_gyro_median_cor_x_m: float | None
    encoder_encoder_median_cor_x_m: float | None
    accel_gyro_median_cor_x_m: float | None
    median_center_accel_x_mps2: float | None
    median_center_accel_y_mps2: float | None
    median_average_velocity_mps: float | None
    median_encoder_yaw_radps: float | None
    median_corrected_gyro_radps: float | None
    encoder_bank_zone_side: str | None
    encoder_bank_zone_run_samples: int
    accel_bank_zone_side: str | None
    accel_bank_zone_run_samples: int
    consensus_pivot_side: str | None
    consensus_pivot_run_samples: int


@dataclass(frozen=True)
class TurnBiasSummary:
    key: TurnKey
    primitive_name: str
    direction_name: str
    peak_abs_corrected_gyro_radps: float
    core: WindowBiasSummary
    trail: WindowBiasSummary
    overall_classification: str


@dataclass(frozen=True)
class TurnBiasAggregateSummary:
    turn_count: int
    core_classification_counts: dict[str, int]
    trail_classification_counts: dict[str, int]
    encoder_bank_zone_counts: dict[str, int]
    accel_bank_zone_counts: dict[str, int]
    consensus_pivot_counts: dict[str, int]
    overall_classification_counts: dict[str, int]


@dataclass(frozen=True)
class TurnBiasRunSummary:
    main_csv_path: Path
    geometry: VehicleTurnGeometry
    biases: SensorBiasSummary
    thresholds: TurnBiasThresholds
    turn_summaries: list[TurnBiasSummary]
    aggregate: TurnBiasAggregateSummary


def parse_args() -> argparse.Namespace:
    repo_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(
        description=(
            "Evaluate open-floor yaw-turn center-of-rotation bias from raw encoders, raw gyro, "
            "and accelerometer moment-arm inversion."
        )
    )
    parser.add_argument(
        "--main",
        type=Path,
        help="Optional path to open_floor_main.csv. If omitted, the latest decoded capture under TestResults is used.",
    )
    parser.add_argument(
        "--repo-root",
        type=Path,
        default=repo_root,
        help="Repository root used to discover the latest log and authoritative vehicle geometry.",
    )
    parser.add_argument("--center-band-mm", type=float, default=5.0)
    parser.add_argument("--core-min-gyro-radps", type=float, default=4.0)
    parser.add_argument("--core-peak-fraction", type=float, default=0.35)
    parser.add_argument("--trail-min-gyro-radps", type=float, default=1.0)
    parser.add_argument("--min-pivot-run-samples", type=int, default=3)
    parser.add_argument("--no-per-turn", action="store_true")
    return parser.parse_args()


def strip_cpp_comments(text: str) -> str:
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.DOTALL)
    return re.sub(r"//.*", "", text)


def split_top_level_items(text: str) -> list[str]:
    items: list[str] = []
    current: list[str] = []
    paren_depth = 0
    brace_depth = 0
    for character in text:
        if character == "," and paren_depth == 0 and brace_depth == 0:
            item = "".join(current).strip()
            if item:
                items.append(item)
            current = []
            continue
        current.append(character)
        if character == "(":
            paren_depth += 1
        elif character == ")":
            paren_depth -= 1
        elif character == "{":
            brace_depth += 1
        elif character == "}":
            brace_depth -= 1
    item = "".join(current).strip()
    if item:
        items.append(item)
    return items


def extract_initializer_entries(path: Path, marker: str) -> list[str]:
    text = strip_cpp_comments(path.read_text(encoding="utf-8"))
    marker_index = text.find(marker)
    if marker_index < 0:
        raise ValueError(f"could not find initializer marker {marker!r} in {path}")
    brace_start = text.find("{", marker_index)
    if brace_start < 0:
        raise ValueError(f"could not find initializer start for {marker!r} in {path}")
    depth = 0
    brace_end = -1
    for index in range(brace_start, len(text)):
        if text[index] == "{":
            depth += 1
        elif text[index] == "}":
            depth -= 1
            if depth == 0:
                brace_end = index
                break
    if brace_end < 0:
        raise ValueError(f"could not find initializer end for {marker!r} in {path}")
    return split_top_level_items(text[brace_start + 1:brace_end])


def evaluate_cpp_expr(expression: str) -> float:
    sanitized = strip_cpp_comments(expression).strip()
    sanitized = re.sub(r"(?<=\d)(?:f|UL|U|L)\b", "", sanitized)
    return float(eval(sanitized, {"__builtins__": {}}, {"PI_F": math.pi}))


def load_named_float(path: Path, name: str) -> float:
    text = strip_cpp_comments(path.read_text(encoding="utf-8"))
    match = re.search(
        rf"(?:inline\s+)?static\s+constexpr\s+float\s+{re.escape(name)}\s*=\s*([^;]+);",
        text,
    )
    if match is None:
        raise ValueError(f"could not find float constant {name!r} in {path}")
    return evaluate_cpp_expr(match.group(1))


def load_vehicle_turn_geometry(repo_root: Path) -> VehicleTurnGeometry:
    vehicle_header = repo_root / "MazeMap" / "MazeMap" / "Vehicle.h"
    vehicle_cpp = repo_root / "MazeMap" / "MazeMap" / "Vehicle.cpp"
    width_m = load_named_float(vehicle_header, "kPhysicalWidthM")
    effective_track_width_m = load_named_float(vehicle_header, "kPhysicalTrackWidthM")
    track_width_physical_min_m = load_named_float(vehicle_header, "kTrackWidthPhysicalMinM")
    track_width_physical_max_m = load_named_float(vehicle_header, "kTrackWidthPhysicalMaxM")

    cpp_text = strip_cpp_comments(vehicle_cpp.read_text(encoding="utf-8"))
    position_match = re.search(
        r"positionBodyM\s*=\s*Eigen::Vector2f\(([^,]+),\s*([^)]+)\)",
        cpp_text,
    )
    if position_match is None:
        raise ValueError(f"could not find IMU position in {vehicle_cpp}")
    imu_position_body_x_m = evaluate_cpp_expr(position_match.group(1))
    imu_position_body_y_m = evaluate_cpp_expr(position_match.group(2))
    return VehicleTurnGeometry(
        width_m=width_m,
        effective_track_width_m=effective_track_width_m,
        tire_bank_inner_x_m=0.5 * track_width_physical_min_m,
        tire_bank_outer_x_m=0.5 * track_width_physical_max_m,
        imu_position_body_x_m=imu_position_body_x_m,
        imu_position_body_y_m=imu_position_body_y_m,
    )


def discover_latest_open_floor_main(repo_root: Path) -> Path:
    candidates = sorted(
        (repo_root / "TestResults").rglob("open_floor_main.csv"),
        key=lambda path: path.stat().st_mtime,
        reverse=True,
    )
    if not candidates:
        raise FileNotFoundError(f"no open_floor_main.csv found under {repo_root / 'TestResults'}")
    return candidates[0]


def load_csv_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as csv_file:
        return list(csv.DictReader(csv_file))


def estimate_static_sensor_bias(rows: list[dict[str, str]]) -> SensorBiasSummary:
    static_rows = [
        row
        for row in rows
        if int(row["section_id"]) == STATIC_SECTION_ID and int(row["primitive_id"]) == STATIC_PRIMITIVE_ID
    ]
    if not static_rows:
        return SensorBiasSummary(
            gyro_bias_radps=0.0,
            accel_bias_x_mps2=0.0,
            accel_bias_y_mps2=0.0,
            static_row_count=0,
        )
    return SensorBiasSummary(
        gyro_bias_radps=statistics.fmean(float(row["gyro_raw_radps"]) for row in static_rows),
        accel_bias_x_mps2=statistics.fmean(float(row["accel_body_x_mps2"]) for row in static_rows),
        accel_bias_y_mps2=statistics.fmean(float(row["accel_body_y_mps2"]) for row in static_rows),
        static_row_count=len(static_rows),
    )


def _median_finite(values: list[float | None]) -> float | None:
    finite_values = [value for value in values if value is not None and math.isfinite(value)]
    return statistics.median(finite_values) if finite_values else None


def _counter_to_plain_dict(counter: Counter[str]) -> dict[str, int]:
    return dict(sorted(counter.items(), key=lambda item: item[0]))


def _safe_divide(numerator: float, denominator: float, min_abs_denominator: float) -> float | None:
    if not math.isfinite(numerator) or not math.isfinite(denominator):
        return None
    if abs(denominator) < min_abs_denominator:
        return None
    return numerator / denominator


def center_accel_from_imu(
    accel_body_x_mps2: float,
    accel_body_y_mps2: float,
    gyro_radps: float,
    gyro_alpha_radps2: float,
    geometry: VehicleTurnGeometry,
    accel_bias_x_mps2: float = 0.0,
    accel_bias_y_mps2: float = 0.0,
) -> tuple[float, float]:
    corrected_x_mps2 = accel_body_x_mps2 - accel_bias_x_mps2
    corrected_y_mps2 = accel_body_y_mps2 - accel_bias_y_mps2
    center_accel_x_mps2 = (
        corrected_x_mps2 +
        ((gyro_radps * gyro_radps) * geometry.imu_position_body_x_m) -
        (gyro_alpha_radps2 * geometry.imu_position_body_y_m)
    )
    center_accel_y_mps2 = (
        corrected_y_mps2 +
        ((gyro_radps * gyro_radps) * geometry.imu_position_body_y_m) +
        (gyro_alpha_radps2 * geometry.imu_position_body_x_m)
    )
    return center_accel_x_mps2, center_accel_y_mps2


def cor_x_from_average_velocity(
    average_velocity_mps: float,
    yaw_rate_radps: float,
    min_abs_yaw_rate_radps: float = 1.0e-6,
) -> float | None:
    return _safe_divide(average_velocity_mps, yaw_rate_radps, min_abs_yaw_rate_radps)


def cor_x_from_center_accel(
    center_accel_x_mps2: float,
    yaw_rate_radps: float,
    min_abs_yaw_rate_radps: float = 1.0e-6,
) -> float | None:
    return _safe_divide(center_accel_x_mps2, yaw_rate_radps * yaw_rate_radps, min_abs_yaw_rate_radps * min_abs_yaw_rate_radps)


def classify_cor_x(
    cor_x_m: float | None,
    geometry: VehicleTurnGeometry,
    thresholds: TurnBiasThresholds,
) -> str:
    if cor_x_m is None or not math.isfinite(cor_x_m):
        return "unknown"
    magnitude_m = abs(cor_x_m)
    if magnitude_m <= thresholds.center_band_m:
        return "center"
    side = "right" if cor_x_m > 0.0 else "left"
    if magnitude_m < geometry.tire_bank_inner_x_m:
        return f"{side}_bias"
    if magnitude_m <= geometry.tire_bank_outer_x_m:
        return f"{side}_pivot"
    return f"{side}_outboard"


def _pivot_zone_side(cor_x_m: float | None, geometry: VehicleTurnGeometry) -> str | None:
    if cor_x_m is None or not math.isfinite(cor_x_m):
        return None
    magnitude_m = abs(cor_x_m)
    if magnitude_m < geometry.tire_bank_inner_x_m or magnitude_m > geometry.tire_bank_outer_x_m:
        return None
    return "right" if cor_x_m > 0.0 else "left"


def _longest_run(sides: list[str | None], target: str) -> int:
    best = 0
    current = 0
    for side in sides:
        if side == target:
            current += 1
            best = max(best, current)
        else:
            current = 0
    return best


def _pick_side_from_runs(right_run: int, left_run: int, minimum_run: int) -> tuple[str | None, int]:
    if right_run < minimum_run and left_run < minimum_run:
        return None, max(right_run, left_run)
    if right_run >= left_run:
        return "right", right_run
    return "left", left_run


def direction_id_for_yaw_primitive(primitive_id: int) -> int:
    if primitive_id in YAW_CLOCKWISE_PRIMITIVE_IDS:
        return DIRECTION_CLOCKWISE_ID
    if primitive_id in YAW_COUNTERCLOCKWISE_PRIMITIVE_IDS:
        return DIRECTION_COUNTERCLOCKWISE_ID
    return 0


def _commanded_turn_sign(turn_key: TurnKey, samples: list[TurnSample]) -> float:
    if turn_key.direction_id == DIRECTION_CLOCKWISE_ID:
        return 1.0
    if turn_key.direction_id == DIRECTION_COUNTERCLOCKWISE_ID:
        return -1.0
    median_gyro = _median_finite([sample.corrected_gyro_radps for sample in samples])
    return 1.0 if median_gyro is None or median_gyro >= 0.0 else -1.0


def _build_turn_samples(
    rows: list[dict[str, str]],
    geometry: VehicleTurnGeometry,
    biases: SensorBiasSummary,
) -> list[TurnSample]:
    sorted_rows = sorted(
        rows,
        key=lambda row: (
            float(row.get("master_time_us", "0")),
            int(row.get("control_tick_sequence", "0")),
        ),
    )
    if not sorted_rows:
        return []

    corrected_gyro_values = [
        float(row["gyro_raw_radps"]) - biases.gyro_bias_radps
        for row in sorted_rows
    ]
    master_times_s: list[float] = []
    elapsed_time_s = 0.0
    for row in sorted_rows:
        if "master_time_us" in row and row["master_time_us"]:
            master_times_s.append(1.0e-6 * float(row["master_time_us"]))
        else:
            elapsed_time_s += 1.0e-6 * int(row["dt_us"])
            master_times_s.append(elapsed_time_s)

    gyro_alphas_radps2: list[float] = []
    for index in range(len(sorted_rows)):
        if len(sorted_rows) == 1:
            gyro_alphas_radps2.append(0.0)
            continue
        if index == 0:
            dt_s = max(master_times_s[1] - master_times_s[0], 1.0e-6)
            gyro_alphas_radps2.append((corrected_gyro_values[1] - corrected_gyro_values[0]) / dt_s)
            continue
        if index == len(sorted_rows) - 1:
            dt_s = max(master_times_s[index] - master_times_s[index - 1], 1.0e-6)
            gyro_alphas_radps2.append((corrected_gyro_values[index] - corrected_gyro_values[index - 1]) / dt_s)
            continue
        dt_s = max(master_times_s[index + 1] - master_times_s[index - 1], 1.0e-6)
        gyro_alphas_radps2.append((corrected_gyro_values[index + 1] - corrected_gyro_values[index - 1]) / dt_s)

    samples: list[TurnSample] = []
    for row, master_time_s, corrected_gyro_radps, gyro_alpha_radps2 in zip(
        sorted_rows,
        master_times_s,
        corrected_gyro_values,
        gyro_alphas_radps2,
    ):
        left_velocity_mps = float(row["left_encoder_velocity_mps"])
        right_velocity_mps = float(row["right_encoder_velocity_mps"])
        average_velocity_mps = 0.5 * (left_velocity_mps + right_velocity_mps)
        encoder_yaw_radps = (
            (left_velocity_mps - right_velocity_mps) / geometry.effective_track_width_m
            if geometry.effective_track_width_m > 0.0 else 0.0
        )
        center_accel_x_mps2, center_accel_y_mps2 = center_accel_from_imu(
            accel_body_x_mps2=float(row["accel_body_x_mps2"]),
            accel_body_y_mps2=float(row["accel_body_y_mps2"]),
            gyro_radps=corrected_gyro_radps,
            gyro_alpha_radps2=gyro_alpha_radps2,
            geometry=geometry,
            accel_bias_x_mps2=biases.accel_bias_x_mps2,
            accel_bias_y_mps2=biases.accel_bias_y_mps2,
        )
        samples.append(
            TurnSample(
                phase_id=int(row["phase_id"]),
                dt_s=1.0e-6 * int(row["dt_us"]),
                master_time_s=master_time_s,
                corrected_gyro_radps=corrected_gyro_radps,
                gyro_alpha_radps2=gyro_alpha_radps2,
                left_velocity_mps=left_velocity_mps,
                right_velocity_mps=right_velocity_mps,
                average_velocity_mps=average_velocity_mps,
                encoder_yaw_radps=encoder_yaw_radps,
                center_accel_x_mps2=center_accel_x_mps2,
                center_accel_y_mps2=center_accel_y_mps2,
                cor_x_encoder_gyro_m=cor_x_from_average_velocity(average_velocity_mps, corrected_gyro_radps),
                cor_x_encoder_encoder_m=cor_x_from_average_velocity(average_velocity_mps, encoder_yaw_radps),
                cor_x_accel_gyro_m=cor_x_from_center_accel(center_accel_x_mps2, corrected_gyro_radps),
            )
        )
    return samples


def summarize_turn_window(
    *,
    label: str,
    samples: list[TurnSample],
    geometry: VehicleTurnGeometry,
    thresholds: TurnBiasThresholds,
) -> WindowBiasSummary:
    encoder_gyro_median_cor_x_m = _median_finite([sample.cor_x_encoder_gyro_m for sample in samples])
    encoder_encoder_median_cor_x_m = _median_finite([sample.cor_x_encoder_encoder_m for sample in samples])
    accel_gyro_median_cor_x_m = _median_finite([sample.cor_x_accel_gyro_m for sample in samples])
    fused_median_cor_x_m = _median_finite(
        [
            encoder_gyro_median_cor_x_m,
            encoder_encoder_median_cor_x_m,
            accel_gyro_median_cor_x_m,
        ]
    )

    encoder_support_sides: list[str | None] = []
    accel_support_sides: list[str | None] = []
    consensus_support_sides: list[str | None] = []
    for sample in samples:
        encoder_metric_sides = {
            side
            for side in (
                _pivot_zone_side(sample.cor_x_encoder_gyro_m, geometry),
                _pivot_zone_side(sample.cor_x_encoder_encoder_m, geometry),
            )
            if side is not None
        }
        if len(encoder_metric_sides) == 1:
            encoder_support_side = next(iter(encoder_metric_sides))
        else:
            encoder_support_side = None
        accel_support_side = _pivot_zone_side(sample.cor_x_accel_gyro_m, geometry)
        consensus_support_side = (
            encoder_support_side
            if encoder_support_side is not None and encoder_support_side == accel_support_side
            else None
        )
        encoder_support_sides.append(encoder_support_side)
        accel_support_sides.append(accel_support_side)
        consensus_support_sides.append(consensus_support_side)

    encoder_right_run = _longest_run(encoder_support_sides, "right")
    encoder_left_run = _longest_run(encoder_support_sides, "left")
    accel_right_run = _longest_run(accel_support_sides, "right")
    accel_left_run = _longest_run(accel_support_sides, "left")
    consensus_right_run = _longest_run(consensus_support_sides, "right")
    consensus_left_run = _longest_run(consensus_support_sides, "left")

    encoder_bank_zone_side, encoder_bank_zone_run_samples = _pick_side_from_runs(
        encoder_right_run,
        encoder_left_run,
        thresholds.min_pivot_run_samples,
    )
    accel_bank_zone_side, accel_bank_zone_run_samples = _pick_side_from_runs(
        accel_right_run,
        accel_left_run,
        thresholds.min_pivot_run_samples,
    )
    consensus_pivot_side, consensus_pivot_run_samples = _pick_side_from_runs(
        consensus_right_run,
        consensus_left_run,
        thresholds.min_pivot_run_samples,
    )

    return WindowBiasSummary(
        label=label,
        sample_count=len(samples),
        fused_median_cor_x_m=fused_median_cor_x_m,
        fused_classification=classify_cor_x(fused_median_cor_x_m, geometry, thresholds),
        encoder_gyro_median_cor_x_m=encoder_gyro_median_cor_x_m,
        encoder_encoder_median_cor_x_m=encoder_encoder_median_cor_x_m,
        accel_gyro_median_cor_x_m=accel_gyro_median_cor_x_m,
        median_center_accel_x_mps2=_median_finite([sample.center_accel_x_mps2 for sample in samples]),
        median_center_accel_y_mps2=_median_finite([sample.center_accel_y_mps2 for sample in samples]),
        median_average_velocity_mps=_median_finite([sample.average_velocity_mps for sample in samples]),
        median_encoder_yaw_radps=_median_finite([sample.encoder_yaw_radps for sample in samples]),
        median_corrected_gyro_radps=_median_finite([sample.corrected_gyro_radps for sample in samples]),
        encoder_bank_zone_side=encoder_bank_zone_side,
        encoder_bank_zone_run_samples=encoder_bank_zone_run_samples,
        accel_bank_zone_side=accel_bank_zone_side,
        accel_bank_zone_run_samples=accel_bank_zone_run_samples,
        consensus_pivot_side=consensus_pivot_side,
        consensus_pivot_run_samples=consensus_pivot_run_samples,
    )


def summarize_yaw_turn_rows(
    rows: list[dict[str, str]],
    *,
    biases: SensorBiasSummary,
    geometry: VehicleTurnGeometry,
    thresholds: TurnBiasThresholds,
) -> TurnBiasSummary:
    if not rows:
        raise ValueError("turn summary requires at least one row")

    turn_key = TurnKey(
        primitive_id=int(rows[0]["primitive_id"]),
        repeat_index=int(rows[0]["repeat_index"]),
        direction_id=direction_id_for_yaw_primitive(int(rows[0]["primitive_id"])),
        speed_bin=int(rows[0]["speed_bin"]),
    )
    samples = _build_turn_samples(rows, geometry, biases)
    peak_abs_corrected_gyro_radps = max(abs(sample.corrected_gyro_radps) for sample in samples)
    turn_sign = _commanded_turn_sign(turn_key, samples)

    core_min_abs_gyro_radps = max(
        thresholds.core_min_abs_gyro_radps,
        thresholds.core_peak_fraction * peak_abs_corrected_gyro_radps,
    )
    core_samples = [
        sample
        for sample in samples
        if sample.phase_id in (PHASE_STARTUP_ID, PHASE_STEADY_ID)
        and (sample.corrected_gyro_radps * turn_sign) > 0.0
        and abs(sample.corrected_gyro_radps) >= core_min_abs_gyro_radps
    ]
    trail_samples = [
        sample
        for sample in samples
        if sample.phase_id == PHASE_STOP_ID
        and (sample.corrected_gyro_radps * turn_sign) > 0.0
        and abs(sample.corrected_gyro_radps) >= thresholds.trail_min_abs_gyro_radps
    ]

    core_summary = summarize_turn_window(
        label="core",
        samples=core_samples,
        geometry=geometry,
        thresholds=thresholds,
    )
    trail_summary = summarize_turn_window(
        label="trail",
        samples=trail_samples,
        geometry=geometry,
        thresholds=thresholds,
    )

    if trail_summary.consensus_pivot_side is not None:
        overall_classification = f"{trail_summary.consensus_pivot_side}_pivot"
    elif trail_summary.encoder_bank_zone_side is not None:
        overall_classification = f"{trail_summary.encoder_bank_zone_side}_bank_zone"
    elif trail_summary.fused_classification != "unknown":
        overall_classification = trail_summary.fused_classification
    else:
        overall_classification = core_summary.fused_classification

    return TurnBiasSummary(
        key=turn_key,
        primitive_name=PRIMITIVE_NAMES.get(turn_key.primitive_id, str(turn_key.primitive_id)),
        direction_name=DIRECTION_NAMES.get(turn_key.direction_id, str(turn_key.direction_id)),
        peak_abs_corrected_gyro_radps=peak_abs_corrected_gyro_radps,
        core=core_summary,
        trail=trail_summary,
        overall_classification=overall_classification,
    )


def summarize_turn_bias_csv(
    main_csv_path: Path,
    *,
    repo_root: Path,
    thresholds: TurnBiasThresholds,
) -> TurnBiasRunSummary:
    rows = load_csv_rows(main_csv_path)
    geometry = load_vehicle_turn_geometry(repo_root)
    biases = estimate_static_sensor_bias(rows)

    yaw_rows_by_turn: dict[TurnKey, list[dict[str, str]]] = defaultdict(list)
    for row in rows:
        if int(row["section_id"]) != YAW_SECTION_ID:
            continue
        key = TurnKey(
            primitive_id=int(row["primitive_id"]),
            repeat_index=int(row["repeat_index"]),
            direction_id=direction_id_for_yaw_primitive(int(row["primitive_id"])),
            speed_bin=int(row["speed_bin"]),
        )
        yaw_rows_by_turn[key].append(row)

    turn_summaries = [
        summarize_yaw_turn_rows(
            rows=turn_rows,
            biases=biases,
            geometry=geometry,
            thresholds=thresholds,
        )
        for _, turn_rows in sorted(
            yaw_rows_by_turn.items(),
            key=lambda item: (
                item[0].repeat_index,
                item[0].primitive_id,
                item[0].direction_id,
                item[0].speed_bin,
            ),
        )
    ]

    core_classification_counts = Counter(summary.core.fused_classification for summary in turn_summaries)
    trail_classification_counts = Counter(summary.trail.fused_classification for summary in turn_summaries)
    encoder_bank_zone_counts = Counter(
        "none" if summary.trail.encoder_bank_zone_side is None else summary.trail.encoder_bank_zone_side
        for summary in turn_summaries
    )
    accel_bank_zone_counts = Counter(
        "none" if summary.trail.accel_bank_zone_side is None else summary.trail.accel_bank_zone_side
        for summary in turn_summaries
    )
    consensus_pivot_counts = Counter(
        "none" if summary.trail.consensus_pivot_side is None else summary.trail.consensus_pivot_side
        for summary in turn_summaries
    )
    overall_classification_counts = Counter(summary.overall_classification for summary in turn_summaries)

    return TurnBiasRunSummary(
        main_csv_path=main_csv_path,
        geometry=geometry,
        biases=biases,
        thresholds=thresholds,
        turn_summaries=turn_summaries,
        aggregate=TurnBiasAggregateSummary(
            turn_count=len(turn_summaries),
            core_classification_counts=_counter_to_plain_dict(core_classification_counts),
            trail_classification_counts=_counter_to_plain_dict(trail_classification_counts),
            encoder_bank_zone_counts=_counter_to_plain_dict(encoder_bank_zone_counts),
            accel_bank_zone_counts=_counter_to_plain_dict(accel_bank_zone_counts),
            consensus_pivot_counts=_counter_to_plain_dict(consensus_pivot_counts),
            overall_classification_counts=_counter_to_plain_dict(overall_classification_counts),
        ),
    )


def _format_optional_mm(value_m: float | None) -> str:
    return "n/a" if value_m is None or not math.isfinite(value_m) else f"{1000.0 * value_m:+.1f}"


def _format_optional_float(value: float | None, decimals: int) -> str:
    return "n/a" if value is None or not math.isfinite(value) else f"{value:.{decimals}f}"


def print_turn_bias_summary(summary: TurnBiasRunSummary, *, include_per_turn: bool) -> None:
    print("Open-floor turn-bias summary")
    print(f"main_csv={summary.main_csv_path}")
    print(
        "vehicle_geometry: "
        f"width_m={summary.geometry.width_m:.6f}, "
        f"effective_track_width_m={summary.geometry.effective_track_width_m:.6f}, "
        f"tire_bank_inner_x_mm={1000.0 * summary.geometry.tire_bank_inner_x_m:.2f}, "
        f"tire_bank_outer_x_mm={1000.0 * summary.geometry.tire_bank_outer_x_m:.2f}, "
        f"imu_position_body_mm=({1000.0 * summary.geometry.imu_position_body_x_m:.1f},"
        f"{1000.0 * summary.geometry.imu_position_body_y_m:.1f})"
    )
    print(
        "independent_sensor_biases: "
        f"static_rows={summary.biases.static_row_count}, "
        f"gyro_bias_radps={summary.biases.gyro_bias_radps:.6f}, "
        f"accel_bias_x_mps2={summary.biases.accel_bias_x_mps2:.6f}, "
        f"accel_bias_y_mps2={summary.biases.accel_bias_y_mps2:.6f}"
    )
    print(
        "thresholds: "
        f"center_band_mm={1000.0 * summary.thresholds.center_band_m:.1f}, "
        f"core_min_abs_gyro_radps={summary.thresholds.core_min_abs_gyro_radps:.3f}, "
        f"core_peak_fraction={summary.thresholds.core_peak_fraction:.3f}, "
        f"trail_min_abs_gyro_radps={summary.thresholds.trail_min_abs_gyro_radps:.3f}, "
        f"min_pivot_run_samples={summary.thresholds.min_pivot_run_samples}"
    )
    print(
        "aggregate_counts: "
        f"turns={summary.aggregate.turn_count}, "
        f"core={summary.aggregate.core_classification_counts}, "
        f"trail={summary.aggregate.trail_classification_counts}, "
        f"encoder_bank_zone={summary.aggregate.encoder_bank_zone_counts}, "
        f"accel_bank_zone={summary.aggregate.accel_bank_zone_counts}, "
        f"consensus_pivot={summary.aggregate.consensus_pivot_counts}, "
        f"overall={summary.aggregate.overall_classification_counts}"
    )

    if not include_per_turn:
        return

    for turn_summary in summary.turn_summaries:
        print()
        print(
            f"repeat={turn_summary.key.repeat_index}, "
            f"primitive={turn_summary.primitive_name}, "
            f"direction={turn_summary.direction_name}, "
            f"speed_bin={turn_summary.key.speed_bin}, "
            f"peak_abs_gyro_radps={turn_summary.peak_abs_corrected_gyro_radps:.3f}, "
            f"overall={turn_summary.overall_classification}"
        )
        for window in (turn_summary.core, turn_summary.trail):
            print(
                f"  {window.label}: samples={window.sample_count}, "
                f"fused={window.fused_classification}, "
                f"fused_cor_x_mm={_format_optional_mm(window.fused_median_cor_x_m)}, "
                f"enc_vs_gyro_cor_x_mm={_format_optional_mm(window.encoder_gyro_median_cor_x_m)}, "
                f"enc_vs_enc_cor_x_mm={_format_optional_mm(window.encoder_encoder_median_cor_x_m)}, "
                f"accel_vs_gyro_cor_x_mm={_format_optional_mm(window.accel_gyro_median_cor_x_m)}"
            )
            print(
                f"    kinematics: "
                f"median_gyro_radps={_format_optional_float(window.median_corrected_gyro_radps, 3)}, "
                f"median_encoder_yaw_radps={_format_optional_float(window.median_encoder_yaw_radps, 3)}, "
                f"median_u_mps={_format_optional_float(window.median_average_velocity_mps, 4)}, "
                f"median_center_accel_x_mps2={_format_optional_float(window.median_center_accel_x_mps2, 4)}, "
                f"median_center_accel_y_mps2={_format_optional_float(window.median_center_accel_y_mps2, 4)}"
            )
            print(
                f"    bank_runs: "
                f"encoder_side={window.encoder_bank_zone_side or 'none'}({window.encoder_bank_zone_run_samples}), "
                f"accel_side={window.accel_bank_zone_side or 'none'}({window.accel_bank_zone_run_samples}), "
                f"consensus_side={window.consensus_pivot_side or 'none'}({window.consensus_pivot_run_samples})"
            )


def main() -> int:
    args = parse_args()
    repo_root = args.repo_root.resolve()
    main_csv_path = args.main.resolve() if args.main is not None else discover_latest_open_floor_main(repo_root)
    thresholds = TurnBiasThresholds(
        center_band_m=0.001 * args.center_band_mm,
        core_min_abs_gyro_radps=args.core_min_gyro_radps,
        core_peak_fraction=args.core_peak_fraction,
        trail_min_abs_gyro_radps=args.trail_min_gyro_radps,
        min_pivot_run_samples=args.min_pivot_run_samples,
    )
    summary = summarize_turn_bias_csv(
        main_csv_path,
        repo_root=repo_root,
        thresholds=thresholds,
    )
    print_turn_bias_summary(summary, include_per_turn=not args.no_per_turn)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
