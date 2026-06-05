#!/usr/bin/env python3

from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
import random
import statistics
from dataclasses import dataclass
from dataclasses import field
from pathlib import Path
from typing import Iterator


DEFAULT_TENSOR_FILE_NAME = "feedforward_tensor.json"
DEFAULT_CELL_CSV_FILE_NAME = "feedforward_tensor_cells.csv"
DEFAULT_SUMMARY_FILE_NAME = "feedforward_tensor_summary.txt"
DEFAULT_FEEDFORWARD_PATH_ID = "state_closed_velocity"
DEFAULT_AXIS_BIN_COUNT = 17
DEFAULT_RESERVOIR_SIZE = 200000
DEFAULT_RANDOM_SEED = 0


@dataclass(frozen=True)
class FeedforwardSample:
    present_velocity_mps: float
    present_yaw_rate_radps: float
    desired_accel_mps2: float
    desired_alpha_radps2: float
    left_raw_command: float
    right_raw_command: float
    dt_seconds: float
    source_kind: str
    source_path: str


@dataclass
class SourceStats:
    file_count: int = 0
    sample_count: int = 0


@dataclass
class TensorAxis:
    name: str
    centers: list[float]

    def clamp_value(self, value: float) -> float:
        if not self.centers:
            raise ValueError(f"axis {self.name} has no centers")
        return max(self.centers[0], min(self.centers[-1], value))

    def boundary_values(self) -> list[float]:
        if len(self.centers) <= 1:
            return []
        return [
            0.5 * (left + right)
            for left, right in zip(self.centers, self.centers[1:])
        ]

    def locate_bin(self, value: float) -> int:
        boundaries = self.boundary_values()
        return bisect.bisect_left(boundaries, self.clamp_value(value))

    def interpolation_points(self, value: float) -> list[tuple[int, float]]:
        if len(self.centers) == 1:
            return [(0, 1.0)]
        clamped_value = self.clamp_value(value)
        upper_index = bisect.bisect_right(self.centers, clamped_value)
        if upper_index <= 0:
            return [(0, 1.0)]
        if upper_index >= len(self.centers):
            return [(len(self.centers) - 1, 1.0)]
        lower_index = upper_index - 1
        lower_center = self.centers[lower_index]
        upper_center = self.centers[upper_index]
        if upper_center <= lower_center:
            return [(lower_index, 1.0)]
        upper_weight = (clamped_value - lower_center) / (upper_center - lower_center)
        lower_weight = 1.0 - upper_weight
        if lower_weight <= 1.0e-9:
            return [(upper_index, 1.0)]
        if upper_weight <= 1.0e-9:
            return [(lower_index, 1.0)]
        return [
            (lower_index, lower_weight),
            (upper_index, upper_weight),
        ]


@dataclass
class FeedforwardTensor:
    axes: list[TensorAxis]
    left_command_mean: list[float]
    right_command_mean: list[float]
    left_command_std: list[float]
    right_command_std: list[float]
    counts: list[int]
    source_stats: dict[str, SourceStats]
    sample_count: int
    occupied_cell_count: int

    def shape(self) -> list[int]:
        return [len(axis.centers) for axis in self.axes] + [2]

    def flat_index(self, indices: tuple[int, int, int, int]) -> int:
        axis_lengths = [len(axis.centers) for axis in self.axes]
        if len(indices) != len(axis_lengths):
            raise ValueError("expected one index per axis")
        index = 0
        for axis_index, axis_length in zip(indices, axis_lengths):
            index = (index * axis_length) + axis_index
        return index

    def occupied_indices(self) -> Iterator[tuple[int, int, int, int]]:
        axis_lengths = [len(axis.centers) for axis in self.axes]
        if len(axis_lengths) != 4:
            raise ValueError("feedforward tensor currently expects four input axes")
        for flat_index, count in enumerate(self.counts):
            if count <= 0:
                continue
            remainder = flat_index
            indices: list[int] = []
            for axis_length in reversed(axis_lengths):
                indices.append(remainder % axis_length)
                remainder //= axis_length
            yield tuple(reversed(indices))  # type: ignore[return-value]

    def evaluate(
        self,
        present_velocity_mps: float,
        present_yaw_rate_radps: float,
        desired_accel_mps2: float,
        desired_alpha_radps2: float,
    ) -> tuple[float, float]:
        axis_points = [
            self.axes[0].interpolation_points(present_velocity_mps),
            self.axes[1].interpolation_points(present_yaw_rate_radps),
            self.axes[2].interpolation_points(desired_accel_mps2),
            self.axes[3].interpolation_points(desired_alpha_radps2),
        ]
        left_sum = 0.0
        right_sum = 0.0
        weight_sum = 0.0
        for velocity_index, velocity_weight in axis_points[0]:
            for yaw_index, yaw_weight in axis_points[1]:
                for accel_index, accel_weight in axis_points[2]:
                    for alpha_index, alpha_weight in axis_points[3]:
                        weight = velocity_weight * yaw_weight * accel_weight * alpha_weight
                        if weight <= 0.0:
                            continue
                        flat_index = self.flat_index(
                            (velocity_index, yaw_index, accel_index, alpha_index)
                        )
                        if self.counts[flat_index] <= 0:
                            continue
                        left_sum += weight * self.left_command_mean[flat_index]
                        right_sum += weight * self.right_command_mean[flat_index]
                        weight_sum += weight
        if weight_sum > 0.0:
            return left_sum / weight_sum, right_sum / weight_sum
        return self._nearest_neighbor_evaluate(
            present_velocity_mps,
            present_yaw_rate_radps,
            desired_accel_mps2,
            desired_alpha_radps2,
        )

    def _nearest_neighbor_evaluate(
        self,
        present_velocity_mps: float,
        present_yaw_rate_radps: float,
        desired_accel_mps2: float,
        desired_alpha_radps2: float,
    ) -> tuple[float, float]:
        best_distance = math.inf
        best_index = -1
        target = (
            self.axes[0].clamp_value(present_velocity_mps),
            self.axes[1].clamp_value(present_yaw_rate_radps),
            self.axes[2].clamp_value(desired_accel_mps2),
            self.axes[3].clamp_value(desired_alpha_radps2),
        )
        axis_spans = [
            max(axis.centers[-1] - axis.centers[0], 1.0)
            for axis in self.axes
        ]
        for indices in self.occupied_indices():
            velocity_delta = (self.axes[0].centers[indices[0]] - target[0]) / axis_spans[0]
            yaw_delta = (self.axes[1].centers[indices[1]] - target[1]) / axis_spans[1]
            accel_delta = (self.axes[2].centers[indices[2]] - target[2]) / axis_spans[2]
            alpha_delta = (self.axes[3].centers[indices[3]] - target[3]) / axis_spans[3]
            distance = (
                velocity_delta * velocity_delta +
                yaw_delta * yaw_delta +
                accel_delta * accel_delta +
                alpha_delta * alpha_delta
            )
            if distance < best_distance:
                best_distance = distance
                best_index = self.flat_index(indices)
        if best_index < 0:
            raise ValueError("tensor has no populated cells")
        return self.left_command_mean[best_index], self.right_command_mean[best_index]

    def to_json_dict(self) -> dict[str, object]:
        return {
            "schema_version": 1,
            "axes": [
                {"name": axis.name, "centers": axis.centers}
                for axis in self.axes
            ],
            "shape": self.shape(),
            "sample_count": self.sample_count,
            "occupied_cell_count": self.occupied_cell_count,
            "source_stats": {
                name: {
                    "file_count": stats.file_count,
                    "sample_count": stats.sample_count,
                }
                for name, stats in sorted(self.source_stats.items())
            },
            "cells": {
                "count": self.counts,
                "left_command_mean": self.left_command_mean,
                "right_command_mean": self.right_command_mean,
                "left_command_std": self.left_command_std,
                "right_command_std": self.right_command_std,
            },
        }

    @classmethod
    def from_json_dict(cls, payload: dict[str, object]) -> FeedforwardTensor:
        axes = [
            TensorAxis(
                name=str(axis_payload["name"]),
                centers=[float(value) for value in axis_payload["centers"]],
            )
            for axis_payload in payload["axes"]  # type: ignore[index]
        ]
        source_stats = {
            name: SourceStats(
                file_count=int(stats_payload["file_count"]),
                sample_count=int(stats_payload["sample_count"]),
            )
            for name, stats_payload in (payload.get("source_stats") or {}).items()  # type: ignore[union-attr]
        }
        cells = payload["cells"]  # type: ignore[index]
        return cls(
            axes=axes,
            left_command_mean=[float(value) for value in cells["left_command_mean"]],  # type: ignore[index]
            right_command_mean=[float(value) for value in cells["right_command_mean"]],  # type: ignore[index]
            left_command_std=[float(value) for value in cells["left_command_std"]],  # type: ignore[index]
            right_command_std=[float(value) for value in cells["right_command_std"]],  # type: ignore[index]
            counts=[int(value) for value in cells["count"]],  # type: ignore[index]
            source_stats=source_stats,
            sample_count=int(payload["sample_count"]),  # type: ignore[index]
            occupied_cell_count=int(payload["occupied_cell_count"]),  # type: ignore[index]
        )


@dataclass
class TensorBuildConfig:
    velocity_bins: int = DEFAULT_AXIS_BIN_COUNT
    yaw_bins: int = DEFAULT_AXIS_BIN_COUNT
    accel_bins: int = DEFAULT_AXIS_BIN_COUNT
    alpha_bins: int = DEFAULT_AXIS_BIN_COUNT
    reservoir_size: int = DEFAULT_RESERVOIR_SIZE
    random_seed: int = DEFAULT_RANDOM_SEED
    feedforward_path_id: str = DEFAULT_FEEDFORWARD_PATH_ID


@dataclass
class RunningCellStats:
    count: int = 0
    left_sum: float = 0.0
    right_sum: float = 0.0
    left_sum_squares: float = 0.0
    right_sum_squares: float = 0.0

    def add(self, left_value: float, right_value: float) -> None:
        self.count += 1
        self.left_sum += left_value
        self.right_sum += right_value
        self.left_sum_squares += left_value * left_value
        self.right_sum_squares += right_value * right_value


@dataclass
class ReservoirSampler:
    size: int
    seed: int
    items: list[tuple[float, float, float, float]] = field(default_factory=list)
    count: int = 0

    def __post_init__(self) -> None:
        self._rng = random.Random(self.seed)

    def add(self, sample: FeedforwardSample) -> None:
        item = (
            sample.present_velocity_mps,
            sample.present_yaw_rate_radps,
            sample.desired_accel_mps2,
            sample.desired_alpha_radps2,
        )
        self.count += 1
        if len(self.items) < self.size:
            self.items.append(item)
            return
        replacement_index = self._rng.randrange(self.count)
        if replacement_index < self.size:
            self.items[replacement_index] = item


def parse_float(row: dict[str, str], *names: str) -> float | None:
    for name in names:
        value = row.get(name)
        if value is None or value == "":
            continue
        try:
            result = float(value)
        except ValueError:
            continue
        if math.isfinite(result):
            return result
    return None


def parse_int(row: dict[str, str], *names: str) -> int | None:
    for name in names:
        value = row.get(name)
        if value is None or value == "":
            continue
        try:
            return int(value)
        except ValueError:
            continue
    return None


def sensor_yaw_rate_radps(row: dict[str, str]) -> float | None:
    gyro_raw = parse_float(row, "gyro_raw_radps")
    if gyro_raw is not None:
        return gyro_raw
    return parse_float(
        row,
        "current_yaw_rate_sensor_radps",
        "target_yaw_rate_sensor_radps",
        "measured_angular_speed_radps",
        "angular_speed_radps",
        "gyro_radps",
    )


def sensor_forward_velocity_mps(row: dict[str, str]) -> float | None:
    left_velocity = parse_float(row, "left_encoder_velocity_mps", "left_velocity_mps", "current_left_velocity_mps")
    right_velocity = parse_float(row, "right_encoder_velocity_mps", "right_velocity_mps", "current_right_velocity_mps")
    if left_velocity is not None and right_velocity is not None:
        return 0.5 * (left_velocity + right_velocity)
    return parse_float(
        row,
        "current_forward_sensor_mps",
        "target_forward_sensor_mps",
        "measured_linear_speed_mps",
        "linear_speed_mps",
    )


def is_comparable_open_floor_transition(current_row: dict[str, str], next_row: dict[str, str]) -> bool:
    return (
        current_row.get("section_id") == next_row.get("section_id") and
        current_row.get("primitive_id") == next_row.get("primitive_id") and
        current_row.get("repeat_index") == next_row.get("repeat_index")
    )


def is_comparable_diag_transition(current_row: dict[str, str], next_row: dict[str, str]) -> bool:
    return current_row.get("phase_id") == next_row.get("phase_id")


def flags_are_clear(row: dict[str, str]) -> bool:
    saturation_flags = parse_int(row, "saturation_flags")
    watchdog_flags = parse_int(row, "watchdog_flags")
    return (
        (saturation_flags in (None, 0)) and
        (watchdog_flags in (None, 0))
    )


def commands_are_valid(left_command: float | None, right_command: float | None) -> bool:
    return (
        left_command is not None and
        right_command is not None and
        math.isfinite(left_command) and
        math.isfinite(right_command) and
        abs(left_command) <= 1.0 and
        abs(right_command) <= 1.0
    )


def build_transition_sample(
    current_row: dict[str, str],
    next_row: dict[str, str],
    dt_seconds: float,
    left_command: float,
    right_command: float,
    source_kind: str,
    source_path: str,
) -> FeedforwardSample | None:
    if not (math.isfinite(dt_seconds) and dt_seconds > 0.0):
        return None
    current_velocity = sensor_forward_velocity_mps(current_row)
    next_velocity = sensor_forward_velocity_mps(next_row)
    current_yaw = sensor_yaw_rate_radps(current_row)
    next_yaw = sensor_yaw_rate_radps(next_row)
    if (
        current_velocity is None or
        next_velocity is None or
        current_yaw is None or
        next_yaw is None
    ):
        return None
    desired_accel = (next_velocity - current_velocity) / dt_seconds
    desired_alpha = (next_yaw - current_yaw) / dt_seconds
    if not all(
        math.isfinite(value)
        for value in (
            current_velocity,
            current_yaw,
            desired_accel,
            desired_alpha,
            left_command,
            right_command,
        )
    ):
        return None
    return FeedforwardSample(
        present_velocity_mps=current_velocity,
        present_yaw_rate_radps=current_yaw,
        desired_accel_mps2=desired_accel,
        desired_alpha_radps2=desired_alpha,
        left_raw_command=left_command,
        right_raw_command=right_command,
        dt_seconds=dt_seconds,
        source_kind=source_kind,
        source_path=source_path,
    )


def iter_open_floor_main_samples(path: Path) -> Iterator[FeedforwardSample]:
    with path.open(newline="", encoding="utf-8", errors="replace") as csv_file:
        reader = csv.DictReader(csv_file)
        previous_row: dict[str, str] | None = None
        for row in reader:
            if previous_row is None:
                previous_row = row
                continue
            if not is_comparable_open_floor_transition(previous_row, row):
                previous_row = row
                continue
            if not flags_are_clear(previous_row):
                previous_row = row
                continue
            dt_seconds = (parse_int(row, "dt_us") or 0) * 1.0e-6
            left_command = parse_float(previous_row, "left_drive_command")
            right_command = parse_float(previous_row, "right_drive_command")
            if not commands_are_valid(left_command, right_command):
                previous_row = row
                continue
            sample = build_transition_sample(
                previous_row,
                row,
                dt_seconds,
                left_command=left_command or 0.0,
                right_command=right_command or 0.0,
                source_kind="open_floor_main",
                source_path=str(path),
            )
            if sample is not None:
                yield sample
            previous_row = row


def iter_competition_diag_samples(path: Path) -> Iterator[FeedforwardSample]:
    with path.open(newline="", encoding="utf-8", errors="replace") as csv_file:
        reader = csv.reader(csv_file)
        header: list[str] | None = None
        previous_row: dict[str, str] | None = None
        for tokens in reader:
            if not tokens:
                continue
            if tokens[0] == "sample":
                header = tokens
                previous_row = None
                continue
            if tokens[0].startswith("#") or header is None:
                continue
            row = dict(zip(header, tokens))
            if previous_row is None:
                previous_row = row
                continue
            if not is_comparable_diag_transition(previous_row, row):
                previous_row = row
                continue
            dt_seconds = (parse_int(row, "dt_us") or 0) * 1.0e-6
            left_command = parse_float(previous_row, "left_drive_cmd")
            right_command = parse_float(previous_row, "right_drive_cmd")
            if not commands_are_valid(left_command, right_command):
                previous_row = row
                continue
            sample = build_transition_sample(
                previous_row,
                row,
                dt_seconds,
                left_command=left_command or 0.0,
                right_command=right_command or 0.0,
                source_kind="competition_diag",
                source_path=str(path),
            )
            if sample is not None:
                yield sample
            previous_row = row


def iter_sensor_feedforward_samples(path: Path) -> Iterator[FeedforwardSample]:
    with path.open(newline="", encoding="utf-8", errors="replace") as csv_file:
        reader = csv.DictReader(csv_file)
        for row in reader:
            if not flags_are_clear(row):
                continue
            dt_seconds = (parse_int(row, "dt_us") or 0) * 1.0e-6
            left_command = parse_float(row, "logged_left_drive_command")
            right_command = parse_float(row, "logged_right_drive_command")
            if not commands_are_valid(left_command, right_command):
                continue
            current_row = {
                "current_forward_sensor_mps": row.get("current_forward_sensor_mps", ""),
                "current_yaw_rate_sensor_radps": row.get("current_yaw_rate_sensor_radps", ""),
            }
            next_row = {
                "target_forward_sensor_mps": row.get("target_forward_sensor_mps", ""),
                "target_yaw_rate_sensor_radps": row.get("target_yaw_rate_sensor_radps", ""),
            }
            sample = build_transition_sample(
                current_row,
                next_row,
                dt_seconds,
                left_command=left_command or 0.0,
                right_command=right_command or 0.0,
                source_kind="sensor_feedforward",
                source_path=str(path),
            )
            if sample is not None:
                yield sample


def iter_feedforward_path_samples(path: Path, feedforward_path_id: str) -> Iterator[FeedforwardSample]:
    with path.open(newline="", encoding="utf-8", errors="replace") as csv_file:
        reader = csv.DictReader(csv_file)
        for row in reader:
            if row.get("path_id") != feedforward_path_id:
                continue
            if not flags_are_clear(row):
                continue
            dt_seconds = (parse_int(row, "dt_us") or 0) * 1.0e-6
            left_command = parse_float(row, "logged_left_drive_command")
            right_command = parse_float(row, "logged_right_drive_command")
            if not commands_are_valid(left_command, right_command):
                continue
            current_row = {
                "current_forward_sensor_mps": row.get("current_forward_sensor_mps", ""),
                "current_yaw_rate_sensor_radps": row.get("current_yaw_rate_sensor_radps", ""),
            }
            next_row = {
                "target_forward_sensor_mps": row.get("target_forward_sensor_mps", ""),
                "target_yaw_rate_sensor_radps": row.get("target_yaw_rate_sensor_radps", ""),
            }
            sample = build_transition_sample(
                current_row,
                next_row,
                dt_seconds,
                left_command=left_command or 0.0,
                right_command=right_command or 0.0,
                source_kind=f"feedforward_paths:{feedforward_path_id}",
                source_path=str(path),
            )
            if sample is not None:
                yield sample


def iter_feedforward_samples(root: Path, config: TensorBuildConfig) -> Iterator[FeedforwardSample]:
    for path in sorted(root.rglob("open_floor_main.csv")):
        yield from iter_open_floor_main_samples(path)
    for path in sorted(root.rglob("diag*.csv")):
        yield from iter_competition_diag_samples(path)
    for path in sorted(root.rglob("sensor_feedforward.csv")):
        yield from iter_sensor_feedforward_samples(path)
    for path in sorted(root.rglob("feedforward_paths.csv")):
        yield from iter_feedforward_path_samples(path, config.feedforward_path_id)


def interpolated_percentile(sorted_values: list[float], fraction: float) -> float:
    if not sorted_values:
        raise ValueError("percentile requires at least one value")
    if len(sorted_values) == 1:
        return sorted_values[0]
    clamped_fraction = max(0.0, min(1.0, fraction))
    index = clamped_fraction * (len(sorted_values) - 1)
    lower_index = math.floor(index)
    upper_index = math.ceil(index)
    if lower_index == upper_index:
        return sorted_values[lower_index]
    upper_weight = index - lower_index
    lower_weight = 1.0 - upper_weight
    return (
        sorted_values[lower_index] * lower_weight +
        sorted_values[upper_index] * upper_weight
    )


def build_axis(name: str, values: list[float], bin_count: int) -> TensorAxis:
    if not values:
        raise ValueError(f"cannot build axis {name} without values")
    sorted_values = sorted(values)
    minimum = sorted_values[0]
    maximum = sorted_values[-1]
    if bin_count <= 1 or math.isclose(minimum, maximum, rel_tol=0.0, abs_tol=1.0e-12):
        return TensorAxis(name=name, centers=[statistics.median(sorted_values)])
    candidate_centers = [
        interpolated_percentile(sorted_values, index / (bin_count - 1))
        for index in range(bin_count)
    ]
    centers: list[float] = []
    for center in candidate_centers:
        if centers and math.isclose(center, centers[-1], rel_tol=0.0, abs_tol=1.0e-9):
            continue
        centers.append(center)
    if len(centers) == 1:
        centers = [minimum, maximum]
    return TensorAxis(name=name, centers=centers)


def axis_index(sample: FeedforwardSample, axes: list[TensorAxis]) -> tuple[int, int, int, int]:
    return (
        axes[0].locate_bin(sample.present_velocity_mps),
        axes[1].locate_bin(sample.present_yaw_rate_radps),
        axes[2].locate_bin(sample.desired_accel_mps2),
        axes[3].locate_bin(sample.desired_alpha_radps2),
    )


def flat_tensor_size(axes: list[TensorAxis]) -> int:
    size = 1
    for axis in axes:
        size *= len(axis.centers)
    return size


def discover_source_file_counts(root: Path, feedforward_path_id: str) -> dict[str, SourceStats]:
    return {
        "open_floor_main": SourceStats(
            file_count=sum(1 for _ in root.rglob("open_floor_main.csv")),
        ),
        "competition_diag": SourceStats(
            file_count=sum(1 for _ in root.rglob("diag*.csv")),
        ),
        "sensor_feedforward": SourceStats(
            file_count=sum(1 for _ in root.rglob("sensor_feedforward.csv")),
        ),
        f"feedforward_paths:{feedforward_path_id}": SourceStats(
            file_count=sum(1 for _ in root.rglob("feedforward_paths.csv")),
        ),
    }


def build_feedforward_tensor(root: Path, config: TensorBuildConfig) -> FeedforwardTensor:
    reservoir = ReservoirSampler(size=config.reservoir_size, seed=config.random_seed)
    for sample in iter_feedforward_samples(root, config):
        reservoir.add(sample)
    if not reservoir.items:
        raise ValueError(f"no feedforward-capable samples found under {root}")

    velocity_values = [item[0] for item in reservoir.items]
    yaw_values = [item[1] for item in reservoir.items]
    accel_values = [item[2] for item in reservoir.items]
    alpha_values = [item[3] for item in reservoir.items]
    axes = [
        build_axis("present_velocity_mps", velocity_values, config.velocity_bins),
        build_axis("present_yaw_rate_radps", yaw_values, config.yaw_bins),
        build_axis("desired_accel_mps2", accel_values, config.accel_bins),
        build_axis("desired_alpha_radps2", alpha_values, config.alpha_bins),
    ]

    axis_lengths = [len(axis.centers) for axis in axes]
    cell_stats = [RunningCellStats() for _ in range(flat_tensor_size(axes))]
    source_stats = discover_source_file_counts(root, config.feedforward_path_id)
    sample_count = 0
    for sample in iter_feedforward_samples(root, config):
        indices = axis_index(sample, axes)
        flat_index = 0
        for axis_length, axis_entry in zip(axis_lengths, indices):
            flat_index = (flat_index * axis_length) + axis_entry
        cell_stats[flat_index].add(sample.left_raw_command, sample.right_raw_command)
        source_stats[sample.source_kind].sample_count += 1
        sample_count += 1

    left_command_mean: list[float] = []
    right_command_mean: list[float] = []
    left_command_std: list[float] = []
    right_command_std: list[float] = []
    counts: list[int] = []
    occupied_cell_count = 0
    for stats in cell_stats:
        counts.append(stats.count)
        if stats.count <= 0:
            left_command_mean.append(math.nan)
            right_command_mean.append(math.nan)
            left_command_std.append(math.nan)
            right_command_std.append(math.nan)
            continue
        occupied_cell_count += 1
        left_mean = stats.left_sum / stats.count
        right_mean = stats.right_sum / stats.count
        left_variance = max(0.0, (stats.left_sum_squares / stats.count) - (left_mean * left_mean))
        right_variance = max(0.0, (stats.right_sum_squares / stats.count) - (right_mean * right_mean))
        left_command_mean.append(left_mean)
        right_command_mean.append(right_mean)
        left_command_std.append(math.sqrt(left_variance))
        right_command_std.append(math.sqrt(right_variance))

    return FeedforwardTensor(
        axes=axes,
        left_command_mean=left_command_mean,
        right_command_mean=right_command_mean,
        left_command_std=left_command_std,
        right_command_std=right_command_std,
        counts=counts,
        source_stats=source_stats,
        sample_count=sample_count,
        occupied_cell_count=occupied_cell_count,
    )


def write_tensor_json(path: Path, tensor: FeedforwardTensor) -> None:
    path.write_text(
        json.dumps(tensor.to_json_dict(), indent=2) + "\n",
        encoding="utf-8",
    )


def write_tensor_cell_csv(path: Path, tensor: FeedforwardTensor) -> None:
    with path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(
            [
                "velocity_index",
                "yaw_index",
                "accel_index",
                "alpha_index",
                "present_velocity_mps",
                "present_yaw_rate_radps",
                "desired_accel_mps2",
                "desired_alpha_radps2",
                "sample_count",
                "left_raw_command_mean",
                "right_raw_command_mean",
                "left_raw_command_std",
                "right_raw_command_std",
            ]
        )
        for indices in tensor.occupied_indices():
            flat_index = tensor.flat_index(indices)
            writer.writerow(
                [
                    indices[0],
                    indices[1],
                    indices[2],
                    indices[3],
                    tensor.axes[0].centers[indices[0]],
                    tensor.axes[1].centers[indices[1]],
                    tensor.axes[2].centers[indices[2]],
                    tensor.axes[3].centers[indices[3]],
                    tensor.counts[flat_index],
                    tensor.left_command_mean[flat_index],
                    tensor.right_command_mean[flat_index],
                    tensor.left_command_std[flat_index],
                    tensor.right_command_std[flat_index],
                ]
            )


def format_axis_summary(axis: TensorAxis) -> str:
    return (
        f"{axis.name}: bins={len(axis.centers)}, "
        f"min={axis.centers[0]:+.6f}, max={axis.centers[-1]:+.6f}"
    )


def build_summary_text(root: Path, tensor: FeedforwardTensor, config: TensorBuildConfig) -> str:
    lines = [
        "Feedforward tensor build summary",
        f"root={root}",
        f"path_id_for_feedforward_paths={config.feedforward_path_id}",
        f"sample_count={tensor.sample_count}",
        f"occupied_cell_count={tensor.occupied_cell_count}",
        f"tensor_shape={tensor.shape()}",
        "axes:",
    ]
    lines.extend(f"  {format_axis_summary(axis)}" for axis in tensor.axes)
    lines.append("sources:")
    for source_name, stats in sorted(tensor.source_stats.items()):
        lines.append(
            f"  {source_name}: files={stats.file_count}, samples={stats.sample_count}"
        )
    return "\n".join(lines) + "\n"


def write_summary_file(path: Path, root: Path, tensor: FeedforwardTensor, config: TensorBuildConfig) -> None:
    path.write_text(build_summary_text(root, tensor, config), encoding="utf-8")


def load_tensor_json(path: Path) -> FeedforwardTensor:
    return FeedforwardTensor.from_json_dict(
        json.loads(path.read_text(encoding="utf-8"))
    )


def parse_build_args(parser: argparse.ArgumentParser) -> None:
    repo_root = Path(__file__).resolve().parents[1]
    parser.add_argument(
        "--root",
        type=Path,
        default=repo_root / "TestResults",
        help="TestResults root to scan for feedforward-relevant CSV files.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=repo_root / "TestResults" / "feedforward_tensor_dataset",
        help="Directory where the tensor JSON, populated-cell CSV, and summary text are written.",
    )
    parser.add_argument(
        "--velocity-bins",
        type=int,
        default=DEFAULT_AXIS_BIN_COUNT,
        help="Quantile-bin count for the present forward-velocity axis.",
    )
    parser.add_argument(
        "--yaw-bins",
        type=int,
        default=DEFAULT_AXIS_BIN_COUNT,
        help="Quantile-bin count for the present yaw-rate axis.",
    )
    parser.add_argument(
        "--accel-bins",
        type=int,
        default=DEFAULT_AXIS_BIN_COUNT,
        help="Quantile-bin count for the desired longitudinal-acceleration axis.",
    )
    parser.add_argument(
        "--alpha-bins",
        type=int,
        default=DEFAULT_AXIS_BIN_COUNT,
        help="Quantile-bin count for the desired yaw-acceleration axis.",
    )
    parser.add_argument(
        "--reservoir-size",
        type=int,
        default=DEFAULT_RESERVOIR_SIZE,
        help="Reservoir sample size used to fit the four tensor axes.",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=DEFAULT_RANDOM_SEED,
        help="Deterministic seed for axis-fit reservoir sampling.",
    )
    parser.add_argument(
        "--feedforward-path-id",
        type=str,
        default=DEFAULT_FEEDFORWARD_PATH_ID,
        help="Canonical path_id to harvest from feedforward_paths.csv exports.",
    )


def parse_evaluate_args(parser: argparse.ArgumentParser) -> None:
    repo_root = Path(__file__).resolve().parents[1]
    parser.add_argument(
        "--tensor-json",
        type=Path,
        default=repo_root / "TestResults" / "feedforward_tensor_dataset" / DEFAULT_TENSOR_FILE_NAME,
        help="Previously built feedforward tensor JSON file.",
    )
    parser.add_argument("--present-velocity-mps", type=float, required=True)
    parser.add_argument("--present-yaw-rate-radps", type=float, required=True)
    parser.add_argument("--desired-accel-mps2", type=float, required=True)
    parser.add_argument("--desired-alpha-radps2", type=float, required=True)


def build_command(args: argparse.Namespace) -> int:
    config = TensorBuildConfig(
        velocity_bins=args.velocity_bins,
        yaw_bins=args.yaw_bins,
        accel_bins=args.accel_bins,
        alpha_bins=args.alpha_bins,
        reservoir_size=args.reservoir_size,
        random_seed=args.seed,
        feedforward_path_id=args.feedforward_path_id,
    )
    root = args.root.resolve()
    tensor = build_feedforward_tensor(root, config)
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    tensor_json_path = output_dir / DEFAULT_TENSOR_FILE_NAME
    cell_csv_path = output_dir / DEFAULT_CELL_CSV_FILE_NAME
    summary_path = output_dir / DEFAULT_SUMMARY_FILE_NAME
    write_tensor_json(tensor_json_path, tensor)
    write_tensor_cell_csv(cell_csv_path, tensor)
    write_summary_file(summary_path, root, tensor, config)

    print(build_summary_text(root, tensor, config), end="")
    print(f"tensor_json={tensor_json_path}")
    print(f"tensor_cells_csv={cell_csv_path}")
    print(f"summary_txt={summary_path}")
    return 0


def evaluate_command(args: argparse.Namespace) -> int:
    tensor = load_tensor_json(args.tensor_json.resolve())
    left_command, right_command = tensor.evaluate(
        present_velocity_mps=args.present_velocity_mps,
        present_yaw_rate_radps=args.present_yaw_rate_radps,
        desired_accel_mps2=args.desired_accel_mps2,
        desired_alpha_radps2=args.desired_alpha_radps2,
    )
    print(
        "feedforward_evaluation: "
        f"left_raw_command={left_command:+.6f}, "
        f"right_raw_command={right_command:+.6f}"
    )
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Build or evaluate a 4D feedforward tensor over "
            "(present velocity, present yaw rate, desired acceleration, desired alpha)."
        )
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    build_parser = subparsers.add_parser(
        "build",
        help="Scan TestResults and build the feedforward tensor dataset.",
    )
    parse_build_args(build_parser)

    evaluate_parser = subparsers.add_parser(
        "evaluate",
        help="Evaluate a previously built tensor at one operating point.",
    )
    parse_evaluate_args(evaluate_parser)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.command == "build":
        return build_command(args)
    if args.command == "evaluate":
        return evaluate_command(args)
    raise ValueError(f"unsupported command {args.command!r}")


if __name__ == "__main__":
    raise SystemExit(main())
