from __future__ import annotations

import csv
import math
import statistics
from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable


ROOT = Path(__file__).resolve().parents[1]
TEST_RESULTS = ROOT / "TestResults"
REPORT_PATH = TEST_RESULTS / "traction_model_realistic_error_2026-06-08.md"

PREVIOUS_REPORT_PATH = TEST_RESULTS / "traction_model_report_2026-06-08.md"

FORWARD_SETTLE_S = 0.050
FORWARD_SPEED_THRESHOLD_MPS = 0.010
FORWARD_DWELL_MIN_S = 0.010
FORWARD_DISTANCE_MIN_M = 0.010

YAW_SETTLE_S = 0.075
YAW_SENSITIVITY_SETTLES_S = (0.050, 0.075, 0.100)
YAW_GYRO_THRESHOLD_RADPS = 0.500
YAW_DWELL_MIN_S = 0.025
YAW_ENCODER_DIFF_MIN_M = 0.003

CONTACT_LATERAL_ACCEL_CAP_MPS2 = 16.5
SMOOTH_WIDTH_COMMAND = 0.04
FORWARD_THRESHOLD_COMMAND = 0.23
YAW_THRESHOLD_FAN_1_COMMAND = 0.55
YAW_THRESHOLD_FAN_08_COMMAND = 0.65

FORWARD_ROLLOUT_WINDOW_S = 0.200
YAW_ROLLOUT_WINDOW_S = 0.200
TURN_ROLLOUT_WINDOW_S = 0.200

YAW_LOGS = (
    ("may04_yaw_fan1", TEST_RESULTS / "mmlog_decode_2026-05-04_16-57-53" / "open_floor_main.csv", 1.0),
    ("may04_yaw_fan08", TEST_RESULTS / "mmlog_decode_2026-05-04_20-35-47" / "open_floor_main.csv", 0.8),
    ("may27_yaw_fan08", TEST_RESULTS / "mmlog_decode_2026-05-27_04-44-04" / "open_floor_main.csv", 0.8),
)

SMOOTH_LOGS = (
    ("apr20_smooth_rich", TEST_RESULTS / "mmlog_decode_2026-04-20_02-33-07" / "open_floor_main.csv", 0.8),
    ("apr20_smooth_partial", TEST_RESULTS / "mmlog_decode_2026-04-20_12-10-58" / "open_floor_main.csv", 0.8),
    ("apr21_latest", TEST_RESULTS / "mmlog_decode_2026-04-21_01-09-34" / "open_floor_main.csv", 0.8),
)


@dataclass(frozen=True, slots=True)
class Sample:
    label: str
    path: Path
    time_s: float
    dt_s: float
    section_id: str
    phase_id: str
    repeat_index: str
    left_command: float
    right_command: float
    linear_speed_mps: float
    yaw_rate_radps: float
    left_encoder_distance_m: float
    right_encoder_distance_m: float
    fan_duty: float


@dataclass(frozen=True, slots=True)
class Segment:
    label: str
    path: Path
    fan_duty: float
    command: float
    rows: tuple[Sample, ...]


@dataclass(frozen=True, slots=True)
class Window:
    label: str
    regime: str
    rows: tuple[Sample, ...]
    start_index: int
    end_index: int

    @property
    def duration_s(self) -> float:
        return self.rows[self.end_index].time_s - self.rows[self.start_index].time_s


@dataclass(frozen=True, slots=True)
class ModelFit:
    forward_gain_mps2_per_command: float
    forward_damping_per_s: float
    yaw_gain_radps2_per_command: float
    yaw_damping_per_s: float


@dataclass(frozen=True, slots=True)
class Metrics:
    count: int
    linear_speed_rmse_mps: float
    linear_accel_rmse_mps2: float
    yaw_rate_rmse_radps: float
    yaw_accel_rmse_radps2: float
    lateral_accel_rmse_mps2: float


@dataclass(slots=True)
class MetricAccumulator:
    count: int = 0
    se_linear: float = 0.0
    se_linear_accel: float = 0.0
    se_yaw: float = 0.0
    se_yaw_accel: float = 0.0
    se_lateral: float = 0.0

    def add(
        self,
        predicted_linear: float,
        observed_linear: float,
        predicted_yaw: float,
        observed_yaw: float,
        predicted_linear_accel: float,
        observed_linear_accel: float,
        predicted_yaw_accel: float,
        observed_yaw_accel: float,
    ) -> None:
        self.count += 1
        self.se_linear += (predicted_linear - observed_linear) ** 2
        self.se_linear_accel += (predicted_linear_accel - observed_linear_accel) ** 2
        self.se_yaw += (predicted_yaw - observed_yaw) ** 2
        self.se_yaw_accel += (predicted_yaw_accel - observed_yaw_accel) ** 2
        self.se_lateral += ((predicted_linear * predicted_yaw) - (observed_linear * observed_yaw)) ** 2

    def merge(self, other: "MetricAccumulator") -> None:
        self.count += other.count
        self.se_linear += other.se_linear
        self.se_linear_accel += other.se_linear_accel
        self.se_yaw += other.se_yaw
        self.se_yaw_accel += other.se_yaw_accel
        self.se_lateral += other.se_lateral

    def metrics(self) -> Metrics:
        if self.count == 0:
            return Metrics(0, float("nan"), float("nan"), float("nan"), float("nan"), float("nan"))
        return Metrics(
            count=self.count,
            linear_speed_rmse_mps=math.sqrt(self.se_linear / self.count),
            linear_accel_rmse_mps2=math.sqrt(self.se_linear_accel / self.count),
            yaw_rate_rmse_radps=math.sqrt(self.se_yaw / self.count),
            yaw_accel_rmse_radps2=math.sqrt(self.se_yaw_accel / self.count),
            lateral_accel_rmse_mps2=math.sqrt(self.se_lateral / self.count),
        )


@dataclass(slots=True)
class TerminalAccumulator:
    windows: int = 0
    se_linear: float = 0.0
    se_yaw: float = 0.0
    se_lateral: float = 0.0
    abs_linear: list[float] = field(default_factory=list)
    abs_yaw: list[float] = field(default_factory=list)
    abs_lateral: list[float] = field(default_factory=list)

    def add(self, predicted_linear: float, observed_linear: float, predicted_yaw: float, observed_yaw: float) -> None:
        linear_error = predicted_linear - observed_linear
        yaw_error = predicted_yaw - observed_yaw
        lateral_error = (predicted_linear * predicted_yaw) - (observed_linear * observed_yaw)
        self.windows += 1
        self.se_linear += linear_error * linear_error
        self.se_yaw += yaw_error * yaw_error
        self.se_lateral += lateral_error * lateral_error
        self.abs_linear.append(abs(linear_error))
        self.abs_yaw.append(abs(yaw_error))
        self.abs_lateral.append(abs(lateral_error))

    def merge(self, other: "TerminalAccumulator") -> None:
        self.windows += other.windows
        self.se_linear += other.se_linear
        self.se_yaw += other.se_yaw
        self.se_lateral += other.se_lateral
        self.abs_linear.extend(other.abs_linear)
        self.abs_yaw.extend(other.abs_yaw)
        self.abs_lateral.extend(other.abs_lateral)

    def rmse_linear(self) -> float:
        return math.sqrt(self.se_linear / self.windows) if self.windows else float("nan")

    def rmse_yaw(self) -> float:
        return math.sqrt(self.se_yaw / self.windows) if self.windows else float("nan")

    def rmse_lateral(self) -> float:
        return math.sqrt(self.se_lateral / self.windows) if self.windows else float("nan")

    def p95_linear(self) -> float:
        return percentile(self.abs_linear, 0.95)

    def p95_yaw(self) -> float:
        return percentile(self.abs_yaw, 0.95)

    def p95_lateral(self) -> float:
        return percentile(self.abs_lateral, 0.95)


@dataclass(slots=True)
class RolloutResult:
    samples: MetricAccumulator = field(default_factory=MetricAccumulator)
    terminal: TerminalAccumulator = field(default_factory=TerminalAccumulator)

    def merge(self, other: "RolloutResult") -> None:
        self.samples.merge(other.samples)
        self.terminal.merge(other.terminal)


@dataclass(slots=True)
class NormalAccumulator:
    forward_matrix: list[list[float]] = field(default_factory=lambda: [[0.0, 0.0], [0.0, 0.0]])
    forward_vector: list[float] = field(default_factory=lambda: [0.0, 0.0])
    yaw_matrix: list[list[float]] = field(default_factory=lambda: [[0.0, 0.0], [0.0, 0.0]])
    yaw_vector: list[float] = field(default_factory=lambda: [0.0, 0.0])

    def add_forward(self, features: tuple[float, float], target: float) -> None:
        add_normal(self.forward_matrix, self.forward_vector, features, target)

    def add_yaw(self, features: tuple[float, float], target: float) -> None:
        add_normal(self.yaw_matrix, self.yaw_vector, features, target)

    def merge(self, other: "NormalAccumulator", scale: float = 1.0) -> None:
        for row in range(2):
            self.forward_vector[row] += scale * other.forward_vector[row]
            self.yaw_vector[row] += scale * other.yaw_vector[row]
            for column in range(2):
                self.forward_matrix[row][column] += scale * other.forward_matrix[row][column]
                self.yaw_matrix[row][column] += scale * other.yaw_matrix[row][column]

    def fit(self) -> ModelFit:
        forward_gain, forward_damping = solve_normal(self.forward_matrix, self.forward_vector)
        yaw_gain, yaw_damping = solve_normal(self.yaw_matrix, self.yaw_vector)
        return ModelFit(
            forward_gain_mps2_per_command=forward_gain,
            forward_damping_per_s=forward_damping,
            yaw_gain_radps2_per_command=yaw_gain,
            yaw_damping_per_s=yaw_damping,
        )

    def without(self, other: "NormalAccumulator") -> "NormalAccumulator":
        result = NormalAccumulator()
        result.merge(self)
        result.merge(other, scale=-1.0)
        return result


@dataclass(slots=True)
class RunData:
    label: str
    path: Path
    forward_segments: list[Segment] = field(default_factory=list)
    yaw_segments: list[Segment] = field(default_factory=list)
    turn_segments: list[Segment] = field(default_factory=list)
    forward_pairs: list[tuple[Sample, Sample]] = field(default_factory=list)
    yaw_pairs: list[tuple[Sample, Sample]] = field(default_factory=list)
    turn_pairs: list[tuple[Sample, Sample]] = field(default_factory=list)
    forward_windows: list[Window] = field(default_factory=list)
    yaw_windows: list[Window] = field(default_factory=list)
    turn_windows: list[Window] = field(default_factory=list)
    normals: NormalAccumulator = field(default_factory=NormalAccumulator)


@dataclass(frozen=True, slots=True)
class ClearEvidence:
    label: str
    command: float
    fan_duty: float
    settle_s: float
    clear: bool
    dwell_s: float
    distance_m: float
    peak: float


@dataclass(frozen=True, slots=True)
class Confusion:
    true_positive: int
    false_positive: int
    false_negative: int
    true_negative: int

    @property
    def total(self) -> int:
        return self.true_positive + self.false_positive + self.false_negative + self.true_negative

    @property
    def false_launch_rate(self) -> float:
        predicted_positive = self.true_positive + self.false_positive
        return self.false_positive / predicted_positive if predicted_positive else 0.0

    @property
    def missed_launch_rate(self) -> float:
        actual_positive = self.true_positive + self.false_negative
        return self.false_negative / actual_positive if actual_positive else 0.0

    @property
    def accuracy(self) -> float:
        return (self.true_positive + self.true_negative) / self.total if self.total else float("nan")


class CsvColumns:
    def __init__(self, fieldnames: list[str]) -> None:
        self.fieldnames = fieldnames
        self.indexes = {name: index for index, name in enumerate(fieldnames)}

    def text(self, row: list[str], name: str, default: str = "") -> str:
        index = self.indexes.get(name)
        if index is None or index >= len(row):
            return default
        return row[index]

    def number(self, row: list[str], *names: str, default: float = float("nan")) -> float:
        for name in names:
            index = self.indexes.get(name)
            if index is None or index >= len(row):
                continue
            try:
                return float(row[index])
            except ValueError:
                continue
        return default


def finite(value: float) -> bool:
    return math.isfinite(value)


def percentile(values: list[float], fraction: float) -> float:
    if not values:
        return float("nan")
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, math.ceil(fraction * len(ordered)) - 1))
    return ordered[index]


def median(values: Iterable[float]) -> float:
    collected = list(values)
    return statistics.median(collected) if collected else float("nan")


def smooth_plus(value: float, width: float) -> float:
    return 0.5 * (value + math.sqrt((value * value) + (width * width)))


def yaw_threshold_for_fan(fan_duty: float) -> float:
    return YAW_THRESHOLD_FAN_1_COMMAND if fan_duty >= 0.95 else YAW_THRESHOLD_FAN_08_COMMAND


def radial_deadband(command_forward: float, command_yaw: float, fan_duty: float) -> tuple[float, float]:
    yaw_threshold = yaw_threshold_for_fan(fan_duty)
    q = math.sqrt(
        (command_forward / FORWARD_THRESHOLD_COMMAND) ** 2
        + (command_yaw / yaw_threshold) ** 2
        + (SMOOTH_WIDTH_COMMAND * SMOOTH_WIDTH_COMMAND)
    )
    scale = smooth_plus(q - 1.0, SMOOTH_WIDTH_COMMAND) / (q + 1.0e-9)
    return command_forward * scale, command_yaw * scale


def parse_sample(columns: CsvColumns, row: list[str], label: str, path: Path, fan_hint: float | None) -> Sample:
    fan_duty = columns.number(row, "fan_duty_cycle")
    if not finite(fan_duty) or (fan_duty == 0.0 and fan_hint is not None):
        fan_duty = fan_hint if fan_hint is not None else 0.8
    return Sample(
        label=label,
        path=path,
        time_s=columns.number(row, "master_time_us") / 1.0e6,
        dt_s=columns.number(row, "dt_us") / 1.0e6,
        section_id=columns.text(row, "section_id"),
        phase_id=columns.text(row, "phase_id"),
        repeat_index=columns.text(row, "repeat_index"),
        left_command=columns.number(row, "left_drive_command", "left_drive_cmd"),
        right_command=columns.number(row, "right_drive_command", "right_drive_cmd"),
        linear_speed_mps=columns.number(row, "measured_linear_speed_mps"),
        yaw_rate_radps=columns.number(row, "gyro_radps", "measured_yaw_rate_radps", "measured_angular_speed_radps"),
        left_encoder_distance_m=columns.number(row, "left_encoder_distance_m"),
        right_encoder_distance_m=columns.number(row, "right_encoder_distance_m"),
        fan_duty=fan_duty,
    )


def sample_is_valid(sample: Sample) -> bool:
    return all(
        finite(value)
        for value in (
            sample.time_s,
            sample.dt_s,
            sample.left_command,
            sample.right_command,
            sample.linear_speed_mps,
            sample.yaw_rate_radps,
            sample.left_encoder_distance_m,
            sample.right_encoder_distance_m,
            sample.fan_duty,
        )
    )


def append_command_segment(segments: list[Segment], rows: list[Sample], yaw: bool) -> None:
    if not rows:
        return
    if yaw:
        command = median((row.left_command - row.right_command) * 0.5 for row in rows)
    else:
        command = median((row.left_command + row.right_command) * 0.5 for row in rows)
    segments.append(
        Segment(
            label=rows[0].label,
            path=rows[0].path,
            fan_duty=rows[0].fan_duty,
            command=command,
            rows=tuple(rows),
        )
    )


def append_turn_segment(segments: list[Segment], rows: list[Sample]) -> None:
    if not rows:
        return
    command = median(0.5 * (row.left_command - row.right_command) for row in rows)
    segments.append(
        Segment(
            label=rows[0].label,
            path=rows[0].path,
            fan_duty=rows[0].fan_duty,
            command=command,
            rows=tuple(rows),
        )
    )


def extract_forward_segments(path: Path) -> list[Segment]:
    label = path.parent.name
    segments: list[Segment] = []
    active: list[Sample] = []
    active_key: tuple[str, float] | None = None
    with path.open(newline="") as handle:
        reader = csv.reader(handle)
        fieldnames = next(reader, None)
        if fieldnames is None:
            return segments
        columns = CsvColumns(fieldnames)
        if "section_id" not in columns.indexes:
            return segments
        for row in reader:
            if columns.text(row, "section_id") != "2":
                append_command_segment(segments, active, yaw=False)
                active = []
                active_key = None
                continue
            left_command = columns.number(row, "left_drive_command", "left_drive_cmd")
            right_command = columns.number(row, "right_drive_command", "right_drive_cmd")
            if (
                not finite(left_command)
                or not finite(right_command)
                or abs(left_command - right_command) > 1.0e-5
                or abs(left_command) <= 1.0e-6
            ):
                append_command_segment(segments, active, yaw=False)
                active = []
                active_key = None
                continue
            sample = parse_sample(columns, row, label, path, None)
            if not sample_is_valid(sample):
                append_command_segment(segments, active, yaw=False)
                active = []
                active_key = None
                continue
            key = (sample.repeat_index, round(sample.left_command, 3))
            if active_key is not None and key != active_key:
                append_command_segment(segments, active, yaw=False)
                active = []
            active_key = key
            active.append(sample)
    append_command_segment(segments, active, yaw=False)
    return segments


def extract_yaw_segments(label: str, path: Path, fan_hint: float) -> list[Segment]:
    segments: list[Segment] = []
    active: list[Sample] = []
    active_key: tuple[str, float] | None = None
    with path.open(newline="") as handle:
        reader = csv.reader(handle)
        fieldnames = next(reader, None)
        if fieldnames is None:
            return segments
        columns = CsvColumns(fieldnames)
        for row in reader:
            if columns.text(row, "phase_id") != "20":
                append_command_segment(segments, active, yaw=True)
                active = []
                active_key = None
                continue
            left_command = columns.number(row, "left_drive_command", "left_drive_cmd")
            right_command = columns.number(row, "right_drive_command", "right_drive_cmd")
            if (
                not finite(left_command)
                or not finite(right_command)
                or abs(left_command + right_command) > 1.0e-5
                or abs(left_command) <= 1.0e-6
                or abs(right_command) <= 1.0e-6
            ):
                append_command_segment(segments, active, yaw=True)
                active = []
                active_key = None
                continue
            sample = parse_sample(columns, row, label, path, fan_hint)
            if not sample_is_valid(sample):
                append_command_segment(segments, active, yaw=True)
                active = []
                active_key = None
                continue
            command = round((sample.left_command - sample.right_command) * 0.5, 3)
            key = (sample.repeat_index, command)
            if active_key is not None and key != active_key:
                append_command_segment(segments, active, yaw=True)
                active = []
            active_key = key
            active.append(sample)
    append_command_segment(segments, active, yaw=True)
    return segments


def extract_turn_segments(label: str, path: Path, fan_hint: float) -> list[Segment]:
    segments: list[Segment] = []
    active: list[Sample] = []
    active_key: str | None = None
    with path.open(newline="") as handle:
        reader = csv.reader(handle)
        fieldnames = next(reader, None)
        if fieldnames is None:
            return segments
        columns = CsvColumns(fieldnames)
        for row in reader:
            valid_phase = columns.text(row, "section_id") == "5" and columns.text(row, "phase_id") == "11"
            if not valid_phase:
                append_turn_segment(segments, active)
                active = []
                active_key = None
                continue
            left_command = columns.number(row, "left_drive_command", "left_drive_cmd")
            right_command = columns.number(row, "right_drive_command", "right_drive_cmd")
            if (
                not finite(left_command)
                or not finite(right_command)
                or (abs(left_command) <= 1.0e-6 and abs(right_command) <= 1.0e-6)
            ):
                append_turn_segment(segments, active)
                active = []
                active_key = None
                continue
            sample = parse_sample(columns, row, label, path, fan_hint)
            if not sample_is_valid(sample):
                append_turn_segment(segments, active)
                active = []
                active_key = None
                continue
            key = sample.repeat_index
            if active_key is not None and key != active_key:
                append_turn_segment(segments, active)
                active = []
            active_key = key
            active.append(sample)
    append_turn_segment(segments, active)
    return segments


def pair_is_contiguous(left: Sample, right: Sample) -> bool:
    return (
        left.dt_s > 0.0
        and left.dt_s <= 0.005
        and right.time_s - left.time_s <= 0.005
        and sample_is_valid(left)
        and sample_is_valid(right)
    )


def segment_pairs(segment: Segment, settle_s: float = 0.0) -> list[tuple[Sample, Sample]]:
    start_time = segment.rows[0].time_s
    pairs: list[tuple[Sample, Sample]] = []
    for left, right in zip(segment.rows, segment.rows[1:]):
        if left.time_s - start_time < settle_s:
            continue
        if pair_is_contiguous(left, right):
            pairs.append((left, right))
    return pairs


def first_index_at_or_after(rows: tuple[Sample, ...], target_time_s: float) -> int | None:
    for index, row in enumerate(rows):
        if row.time_s >= target_time_s:
            return index
    return None


def window_is_contiguous(rows: tuple[Sample, ...], start_index: int, end_index: int) -> bool:
    if end_index <= start_index:
        return False
    return all(pair_is_contiguous(rows[index], rows[index + 1]) for index in range(start_index, end_index))


def fixed_window(segment: Segment, regime: str, start_offset_s: float, length_s: float) -> Window | None:
    rows = segment.rows
    start_index = first_index_at_or_after(rows, rows[0].time_s + start_offset_s)
    if start_index is None:
        return None
    end_index = first_index_at_or_after(rows, rows[start_index].time_s + length_s)
    if end_index is None or not window_is_contiguous(rows, start_index, end_index):
        return None
    return Window(segment.label, regime, rows, start_index, end_index)


def nonoverlapping_windows(segment: Segment, regime: str, length_s: float) -> list[Window]:
    windows: list[Window] = []
    rows = segment.rows
    start_index = 0
    while start_index < len(rows) - 1:
        end_index = first_index_at_or_after(rows, rows[start_index].time_s + length_s)
        if end_index is None:
            break
        if window_is_contiguous(rows, start_index, end_index):
            windows.append(Window(segment.label, regime, rows, start_index, end_index))
            start_index = end_index
        else:
            start_index += 1
    return windows


def add_normal(matrix: list[list[float]], vector: list[float], features: tuple[float, ...], target: float) -> None:
    for row, feature in enumerate(features):
        vector[row] += feature * target
        for column, other in enumerate(features):
            matrix[row][column] += feature * other


def solve_normal(matrix: list[list[float]], vector: list[float]) -> list[float]:
    count = len(vector)
    rows = [matrix[index][:] + [vector[index]] for index in range(count)]
    for index in range(count):
        rows[index][index] += 1.0e-9
    for column in range(count):
        pivot = max(range(column, count), key=lambda row: abs(rows[row][column]))
        rows[column], rows[pivot] = rows[pivot], rows[column]
        pivot_value = rows[column][column]
        if abs(pivot_value) < 1.0e-12:
            return [0.0 for _ in range(count)]
        for item in range(column, count + 1):
            rows[column][item] /= pivot_value
        for row in range(count):
            if row == column:
                continue
            factor = rows[row][column]
            if factor == 0.0:
                continue
            for item in range(column, count + 1):
                rows[row][item] -= factor * rows[column][item]
    return [rows[index][count] for index in range(count)]


def add_pair_normals(accumulator: NormalAccumulator, pairs: list[tuple[Sample, Sample]], yaw_axis: bool) -> None:
    for left, right in pairs:
        command_forward = (left.left_command + left.right_command) * 0.5
        command_yaw = (left.left_command - left.right_command) * 0.5
        effective_forward, effective_yaw = radial_deadband(command_forward, command_yaw, left.fan_duty)
        if yaw_axis:
            lateral_accel = left.linear_speed_mps * left.yaw_rate_radps
            contact_scale = 1.0 / math.sqrt(1.0 + (lateral_accel / CONTACT_LATERAL_ACCEL_CAP_MPS2) ** 2)
            target = (right.yaw_rate_radps - left.yaw_rate_radps) / left.dt_s
            accumulator.add_yaw((effective_yaw * contact_scale, -left.yaw_rate_radps), target)
        else:
            target = (right.linear_speed_mps - left.linear_speed_mps) / left.dt_s
            accumulator.add_forward((effective_forward, -left.linear_speed_mps), target)


def acceleration_from_model(fit: ModelFit, sample: Sample, linear_speed_mps: float, yaw_rate_radps: float) -> tuple[float, float]:
    command_forward = (sample.left_command + sample.right_command) * 0.5
    command_yaw = (sample.left_command - sample.right_command) * 0.5
    effective_forward, effective_yaw = radial_deadband(command_forward, command_yaw, sample.fan_duty)
    linear_accel = fit.forward_gain_mps2_per_command * effective_forward - fit.forward_damping_per_s * linear_speed_mps
    lateral_accel = linear_speed_mps * yaw_rate_radps
    contact_scale = 1.0 / math.sqrt(1.0 + (lateral_accel / CONTACT_LATERAL_ACCEL_CAP_MPS2) ** 2)
    yaw_accel = fit.yaw_gain_radps2_per_command * effective_yaw * contact_scale - fit.yaw_damping_per_s * yaw_rate_radps
    return linear_accel, yaw_accel


def evaluate_pairs(pairs: list[tuple[Sample, Sample]], fit: ModelFit) -> MetricAccumulator:
    accumulator = MetricAccumulator()
    for left, right in pairs:
        linear_accel, yaw_accel = acceleration_from_model(fit, left, left.linear_speed_mps, left.yaw_rate_radps)
        predicted_linear = left.linear_speed_mps + left.dt_s * linear_accel
        predicted_yaw = left.yaw_rate_radps + left.dt_s * yaw_accel
        observed_linear_accel = (right.linear_speed_mps - left.linear_speed_mps) / left.dt_s
        observed_yaw_accel = (right.yaw_rate_radps - left.yaw_rate_radps) / left.dt_s
        accumulator.add(
            predicted_linear,
            right.linear_speed_mps,
            predicted_yaw,
            right.yaw_rate_radps,
            linear_accel,
            observed_linear_accel,
            yaw_accel,
            observed_yaw_accel,
        )
    return accumulator


def evaluate_windows(windows: list[Window], fit: ModelFit) -> RolloutResult:
    result = RolloutResult()
    for window in windows:
        rows = window.rows
        start = rows[window.start_index]
        linear_speed = start.linear_speed_mps
        yaw_rate = start.yaw_rate_radps
        terminal_linear = linear_speed
        terminal_yaw = yaw_rate
        terminal_observed_linear = linear_speed
        terminal_observed_yaw = yaw_rate
        for index in range(window.start_index, window.end_index):
            current = rows[index]
            observed = rows[index + 1]
            linear_accel, yaw_accel = acceleration_from_model(fit, current, linear_speed, yaw_rate)
            next_linear = linear_speed + current.dt_s * linear_accel
            next_yaw = yaw_rate + current.dt_s * yaw_accel
            observed_linear_accel = (observed.linear_speed_mps - current.linear_speed_mps) / current.dt_s
            observed_yaw_accel = (observed.yaw_rate_radps - current.yaw_rate_radps) / current.dt_s
            result.samples.add(
                next_linear,
                observed.linear_speed_mps,
                next_yaw,
                observed.yaw_rate_radps,
                linear_accel,
                observed_linear_accel,
                yaw_accel,
                observed_yaw_accel,
            )
            linear_speed = next_linear
            yaw_rate = next_yaw
            terminal_linear = next_linear
            terminal_yaw = next_yaw
            terminal_observed_linear = observed.linear_speed_mps
            terminal_observed_yaw = observed.yaw_rate_radps
        result.terminal.add(terminal_linear, terminal_observed_linear, terminal_yaw, terminal_observed_yaw)
    return result


def forward_evidence(segment: Segment) -> ClearEvidence | None:
    sign = 1.0 if segment.command >= 0.0 else -1.0
    start_time = segment.rows[0].time_s
    evidence = [row for row in segment.rows if row.time_s - start_time >= FORWARD_SETTLE_S]
    if not evidence:
        return None
    dwell = sum(row.dt_s for row in evidence if sign * row.linear_speed_mps >= FORWARD_SPEED_THRESHOLD_MPS)
    distance = sign * 0.5 * (
        (evidence[-1].left_encoder_distance_m - evidence[0].left_encoder_distance_m)
        + (evidence[-1].right_encoder_distance_m - evidence[0].right_encoder_distance_m)
    )
    peak_speed = max(sign * row.linear_speed_mps for row in evidence)
    clear = dwell >= FORWARD_DWELL_MIN_S and distance >= FORWARD_DISTANCE_MIN_M
    return ClearEvidence(
        label=segment.label,
        command=round(abs(segment.command), 2),
        fan_duty=segment.fan_duty,
        settle_s=FORWARD_SETTLE_S,
        clear=clear,
        dwell_s=dwell,
        distance_m=distance,
        peak=peak_speed,
    )


def yaw_evidence(segment: Segment, settle_s: float) -> ClearEvidence | None:
    sign = 1.0 if segment.command >= 0.0 else -1.0
    start_time = segment.rows[0].time_s
    evidence = [row for row in segment.rows if row.time_s - start_time >= settle_s]
    if not evidence:
        return None
    dwell = sum(row.dt_s for row in evidence if sign * row.yaw_rate_radps >= YAW_GYRO_THRESHOLD_RADPS)
    differential_distance = sign * (
        (evidence[-1].left_encoder_distance_m - evidence[0].left_encoder_distance_m)
        - (evidence[-1].right_encoder_distance_m - evidence[0].right_encoder_distance_m)
    )
    peak_yaw = max(sign * row.yaw_rate_radps for row in evidence)
    clear = dwell >= YAW_DWELL_MIN_S and differential_distance >= YAW_ENCODER_DIFF_MIN_M
    return ClearEvidence(
        label=segment.label,
        command=round(abs(segment.command), 2),
        fan_duty=segment.fan_duty,
        settle_s=settle_s,
        clear=clear,
        dwell_s=dwell,
        distance_m=differential_distance,
        peak=peak_yaw,
    )


def confusion_for(evidence: list[ClearEvidence], threshold: float) -> Confusion:
    true_positive = false_positive = false_negative = true_negative = 0
    for item in evidence:
        predicted = item.command >= threshold
        if predicted and item.clear:
            true_positive += 1
        elif predicted and not item.clear:
            false_positive += 1
        elif not predicted and item.clear:
            false_negative += 1
        else:
            true_negative += 1
    return Confusion(true_positive, false_positive, false_negative, true_negative)


def command_confusions(evidence: list[ClearEvidence], threshold: float) -> list[tuple[float, Confusion]]:
    by_command: dict[float, list[ClearEvidence]] = defaultdict(list)
    for item in evidence:
        by_command[item.command].append(item)
    return [(command, confusion_for(items, threshold)) for command, items in sorted(by_command.items())]


def load_runs() -> dict[str, RunData]:
    runs: dict[str, RunData] = {}

    for path in sorted(TEST_RESULTS.glob("mmlog_decode_*/open_floor_main.csv")):
        segments = extract_forward_segments(path)
        if not segments:
            continue
        label = path.parent.name
        run = runs.setdefault(label, RunData(label=label, path=path))
        run.forward_segments.extend(segments)

    for label, path, fan_hint in YAW_LOGS:
        segments = extract_yaw_segments(label, path, fan_hint)
        if not segments:
            continue
        run = runs.setdefault(label, RunData(label=label, path=path))
        run.yaw_segments.extend(segments)

    for label, path, fan_hint in SMOOTH_LOGS:
        segments = extract_turn_segments(label, path, fan_hint)
        if not segments:
            continue
        run = runs.setdefault(label, RunData(label=label, path=path))
        run.turn_segments.extend(segments)

    for run in runs.values():
        for segment in run.forward_segments:
            run.forward_pairs.extend(segment_pairs(segment))
            window = fixed_window(segment, "forward", 0.0, FORWARD_ROLLOUT_WINDOW_S)
            if window is not None:
                run.forward_windows.append(window)
        for segment in run.yaw_segments:
            run.yaw_pairs.extend(segment_pairs(segment, YAW_SETTLE_S))
            window = fixed_window(segment, "yaw", YAW_SETTLE_S, YAW_ROLLOUT_WINDOW_S)
            if window is not None:
                run.yaw_windows.append(window)
        for segment in run.turn_segments:
            run.turn_pairs.extend(segment_pairs(segment))
            run.turn_windows.extend(nonoverlapping_windows(segment, "turn", TURN_ROLLOUT_WINDOW_S))

        add_pair_normals(run.normals, run.forward_pairs, yaw_axis=False)
        add_pair_normals(run.normals, run.yaw_pairs, yaw_axis=True)
        add_pair_normals(run.normals, run.turn_pairs, yaw_axis=True)

    return runs


def collect_total_normals(runs: dict[str, RunData]) -> NormalAccumulator:
    total = NormalAccumulator()
    for run in runs.values():
        total.merge(run.normals)
    return total


def aggregate_pairs(runs: dict[str, RunData], regime: str) -> list[tuple[Sample, Sample]]:
    pairs: list[tuple[Sample, Sample]] = []
    for run in runs.values():
        if regime == "forward":
            pairs.extend(run.forward_pairs)
        elif regime == "yaw":
            pairs.extend(run.yaw_pairs)
        elif regime == "turn":
            pairs.extend(run.turn_pairs)
    return pairs


def run_pairs(run: RunData, regime: str) -> list[tuple[Sample, Sample]]:
    if regime == "forward":
        return run.forward_pairs
    if regime == "yaw":
        return run.yaw_pairs
    if regime == "turn":
        return run.turn_pairs
    raise ValueError(regime)


def run_windows(run: RunData, regime: str) -> list[Window]:
    if regime == "forward":
        return run.forward_windows
    if regime == "yaw":
        return run.yaw_windows
    if regime == "turn":
        return run.turn_windows
    raise ValueError(regime)


def leave_one_run_out(runs: dict[str, RunData], total_normals: NormalAccumulator) -> tuple[
    dict[str, MetricAccumulator],
    dict[str, RolloutResult],
    list[tuple[str, dict[str, Metrics], dict[str, RolloutResult], ModelFit]],
    list[ModelFit],
]:
    aggregate_one_tick = {regime: MetricAccumulator() for regime in ("forward", "yaw", "turn")}
    aggregate_rollout = {regime: RolloutResult() for regime in ("forward", "yaw", "turn")}
    per_run: list[tuple[str, dict[str, Metrics], dict[str, RolloutResult], ModelFit]] = []
    fits: list[ModelFit] = []

    for label, run in sorted(runs.items()):
        training_normals = total_normals.without(run.normals)
        fit = training_normals.fit()
        fits.append(fit)
        run_pair_metrics: dict[str, Metrics] = {}
        run_rollouts: dict[str, RolloutResult] = {}
        for regime in ("forward", "yaw", "turn"):
            one_tick = evaluate_pairs(run_pairs(run, regime), fit)
            aggregate_one_tick[regime].merge(one_tick)
            run_pair_metrics[regime] = one_tick.metrics()
            rollout = evaluate_windows(run_windows(run, regime), fit)
            aggregate_rollout[regime].merge(rollout)
            run_rollouts[regime] = rollout
        per_run.append((label, run_pair_metrics, run_rollouts, fit))
    return aggregate_one_tick, aggregate_rollout, per_run, fits


def fit_range(fits: list[ModelFit], attr: str) -> tuple[float, float, float]:
    values = [getattr(fit, attr) for fit in fits if finite(getattr(fit, attr))]
    if not values:
        return float("nan"), float("nan"), float("nan")
    return min(values), statistics.median(values), max(values)


def format_float(value: float, digits: int = 3) -> str:
    if not finite(value):
        return "n/a"
    return f"{value:.{digits}f}"


def format_int(value: int) -> str:
    return f"{value:d}"


def metrics_row(label: str, metrics: Metrics) -> str:
    return (
        f"| {label} | {metrics.count} | {metrics.linear_speed_rmse_mps:.5f} | "
        f"{metrics.linear_accel_rmse_mps2:.3f} | {metrics.yaw_rate_rmse_radps:.5f} | "
        f"{metrics.yaw_accel_rmse_radps2:.3f} | {metrics.lateral_accel_rmse_mps2:.5f} |"
    )


def rollout_row(label: str, result: RolloutResult) -> str:
    samples = result.samples.metrics()
    terminal = result.terminal
    return (
        f"| {label} | {terminal.windows} | {samples.count} | "
        f"{samples.linear_speed_rmse_mps:.5f} | {samples.yaw_rate_rmse_radps:.5f} | "
        f"{samples.lateral_accel_rmse_mps2:.5f} | {terminal.rmse_linear():.5f} | "
        f"{terminal.rmse_yaw():.5f} | {terminal.rmse_lateral():.5f} | "
        f"{terminal.p95_linear():.5f} | {terminal.p95_yaw():.5f} |"
    )


def confusion_row(label: str, confusion: Confusion) -> str:
    return (
        f"| {label} | {confusion.total} | {confusion.true_positive} | {confusion.false_positive} | "
        f"{confusion.false_negative} | {confusion.true_negative} | "
        f"{confusion.false_launch_rate:.3f} | {confusion.missed_launch_rate:.3f} | {confusion.accuracy:.3f} |"
    )


def build_classification_tables(runs: dict[str, RunData]) -> tuple[list[ClearEvidence], dict[tuple[float, float], list[ClearEvidence]]]:
    forward_items: list[ClearEvidence] = []
    yaw_items: dict[tuple[float, float], list[ClearEvidence]] = defaultdict(list)
    for run in runs.values():
        for segment in run.forward_segments:
            item = forward_evidence(segment)
            if item is not None:
                forward_items.append(item)
        for segment in run.yaw_segments:
            fan_bucket = 1.0 if segment.fan_duty >= 0.95 else 0.8
            for settle_s in YAW_SENSITIVITY_SETTLES_S:
                item = yaw_evidence(segment, settle_s)
                if item is not None:
                    yaw_items[(fan_bucket, settle_s)].append(item)
    return forward_items, yaw_items


def build_report() -> str:
    runs = load_runs()
    total_normals = collect_total_normals(runs)
    full_fit = total_normals.fit()

    previous_one_tick = {
        regime: evaluate_pairs(aggregate_pairs(runs, regime), full_fit).metrics()
        for regime in ("forward", "yaw", "turn")
    }
    loo_one_tick, loo_rollout, per_run, loo_fits = leave_one_run_out(runs, total_normals)

    forward_evidence_items, yaw_evidence_items = build_classification_tables(runs)
    forward_confusion = confusion_for(forward_evidence_items, FORWARD_THRESHOLD_COMMAND)

    lines: list[str] = []
    lines.append("# Smooth Traction Model Realistic Error Report")
    lines.append("")
    lines.append("## Scope")
    lines.append("")
    lines.append("- Report-only offline analysis. No production code, tests, or build artifacts were modified.")
    lines.append("- Wrote only `tooling/traction_model_realistic_error_report.py` and this report under `TestResults/`.")
    lines.append("- Sensor-only inputs: motor commands, encoder-derived linear speed, encoder distances, and corrected gyro yaw rate.")
    lines.append("- Deliberately excluded: `ukf_state_*`, estimator-predicted columns, yaw-consistency state, NHC state, and UKF-derived pose drift.")
    lines.append("- Phase-20 yaw launch is classified only by post-command evidence windows after a settling interval, not tick-by-tick.")
    lines.append("")
    lines.append("## Data And Model")
    lines.append("")
    lines.append(f"- Runs loaded: `{len(runs)}`.")
    lines.append(f"- Previous one-tick report compared: `TestResults/{PREVIOUS_REPORT_PATH.name}`.")
    lines.append(f"- Forward launch rollout window: `{FORWARD_ROLLOUT_WINDOW_S:.3f} s` from command-window start.")
    lines.append(f"- Yaw launch rollout window: `{YAW_ROLLOUT_WINDOW_S:.3f} s` starting after `{YAW_SETTLE_S:.3f} s` drivetrain-play settling.")
    lines.append(f"- Sustained-turn rollout window: non-overlapping `{TURN_ROLLOUT_WINDOW_S:.3f} s` windows over `section_id=5`, `phase_id=11`.")
    lines.append("- Dynamic parameters are fit by ordinary least squares on training runs, using the same smooth command-space model and launch thresholds as the previous report.")
    lines.append("")
    lines.append("Full-data fit, for comparison with the previous one-tick headline:")
    lines.append("")
    lines.append("| parameter | value |")
    lines.append("| --- | ---: |")
    lines.append(f"| `k_f` | {full_fit.forward_gain_mps2_per_command:.3f} |")
    lines.append(f"| `d_f` | {full_fit.forward_damping_per_s:.3f} |")
    lines.append(f"| `k_y` | {full_fit.yaw_gain_radps2_per_command:.3f} |")
    lines.append(f"| `d_y` | {full_fit.yaw_damping_per_s:.3f} |")
    lines.append("")
    lines.append("Leave-one-run-out fit range:")
    lines.append("")
    lines.append("| parameter | min | median | max |")
    lines.append("| --- | ---: | ---: | ---: |")
    for attr, name in (
        ("forward_gain_mps2_per_command", "`k_f`"),
        ("forward_damping_per_s", "`d_f`"),
        ("yaw_gain_radps2_per_command", "`k_y`"),
        ("yaw_damping_per_s", "`d_y`"),
    ):
        lo, med, hi = fit_range(loo_fits, attr)
        lines.append(f"| {name} | {lo:.3f} | {med:.3f} | {hi:.3f} |")
    lines.append("")
    lines.append("## Previous One-Tick Versus Holdout")
    lines.append("")
    lines.append("The previous headline was an in-sample, one-control-tick prediction from measured state every tick. The holdout columns below refit parameters on all other runs before evaluating the held-out run.")
    lines.append("")
    lines.append("| regime | fit/eval | samples | linear speed RMSE (m/s) | linear accel RMSE (m/s^2) | yaw-rate RMSE (rad/s) | yaw accel RMSE (rad/s^2) | lateral accel RMSE (m/s^2) |")
    lines.append("| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |")
    for regime, label in (("forward", "forward launch"), ("yaw", "phase-20 yaw launch"), ("turn", "sustained smooth turns")):
        lines.append(metrics_row(f"{label} / previous in-sample one-tick", previous_one_tick[regime]))
        lines.append(metrics_row(f"{label} / LOO one-tick", loo_one_tick[regime].metrics()))
    lines.append("")
    lines.append("## Open-Loop Rollout Metrics")
    lines.append("")
    lines.append("Each rollout starts from the measured sensor state at the window start, then integrates the model forward with logged commands. It is not reset to measured speed or yaw inside the window.")
    lines.append("")
    lines.append("| regime | windows | compared ticks | speed RMSE (m/s) | yaw-rate RMSE (rad/s) | lateral accel RMSE (m/s^2) | terminal speed RMSE | terminal yaw RMSE | terminal lateral RMSE | terminal speed p95 abs | terminal yaw p95 abs |")
    lines.append("| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |")
    for regime, label in (("forward", "forward launch"), ("yaw", "phase-20 yaw launch post-settle"), ("turn", "sustained smooth turns")):
        lines.append(rollout_row(f"{label} / LOO rollout", loo_rollout[regime]))
    lines.append("")
    lines.append("## Holdout By Run")
    lines.append("")
    lines.append("Per-run rows use parameters fit on all other runs. Empty cells mean that run did not contain that regime.")
    lines.append("")
    lines.append("| held-out run | fwd windows | fwd terminal speed RMSE | yaw windows | yaw terminal yaw RMSE | turn windows | turn yaw RMSE | turn lateral accel RMSE |")
    lines.append("| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |")
    for label, _pair_metrics, rollouts, _fit in per_run:
        fwd = rollouts["forward"]
        yaw = rollouts["yaw"]
        turn = rollouts["turn"]
        lines.append(
            "| "
            f"{label} | "
            f"{fwd.terminal.windows} | {format_float(fwd.terminal.rmse_linear(), 5)} | "
            f"{yaw.terminal.windows} | {format_float(yaw.terminal.rmse_yaw(), 5)} | "
            f"{turn.terminal.windows} | {format_float(turn.samples.metrics().yaw_rate_rmse_radps, 5)} | "
            f"{format_float(turn.samples.metrics().lateral_accel_rmse_mps2, 5)} |"
        )
    lines.append("")
    lines.append("## Launch Classification")
    lines.append("")
    lines.append("Forward launch classifier: section `2`, equal nonzero commands, first `0.050 s` ignored; clear means signed encoder speed >= `0.010 m/s` for `0.010 s` and signed average encoder distance >= `10 mm`. The model predicts launch when `abs(command) >= 0.23`.")
    lines.append("")
    lines.append("| classifier | windows | TP | FP | FN | TN | false launch rate | missed launch rate | accuracy |")
    lines.append("| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |")
    lines.append(confusion_row("forward threshold 0.23", forward_confusion))
    lines.append("")
    lines.append("| command | predicted launch | windows | TP | FP | FN | TN | clear fraction |")
    lines.append("| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: |")
    by_command_clear: dict[float, tuple[int, int]] = {}
    for command in sorted({item.command for item in forward_evidence_items}):
        items = [item for item in forward_evidence_items if item.command == command]
        clear_count = sum(1 for item in items if item.clear)
        by_command_clear[command] = (clear_count, len(items))
    for command, confusion in command_confusions(forward_evidence_items, FORWARD_THRESHOLD_COMMAND):
        clear_count, total = by_command_clear[command]
        lines.append(
            f"| {command:.2f} | {'yes' if command >= FORWARD_THRESHOLD_COMMAND else 'no'} | {total} | "
            f"{confusion.true_positive} | {confusion.false_positive} | {confusion.false_negative} | "
            f"{confusion.true_negative} | {clear_count / total:.3f} |"
        )
    lines.append("")
    lines.append("Yaw launch classifier: phase `20`, opposite equal commands, post-settle evidence window; clear means sign-aligned corrected gyro >= `0.500 rad/s` for `0.025 s` and sign-aligned encoder differential distance >= `3 mm`. The model predicts launch at `0.55` command for fan 1.0 and `0.65` command for fan 0.8.")
    lines.append("")
    lines.append("| fan | settle (s) | windows | TP | FP | FN | TN | false launch rate | missed launch rate | accuracy |")
    lines.append("| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |")
    for fan in (1.0, 0.8):
        threshold = YAW_THRESHOLD_FAN_1_COMMAND if fan >= 0.95 else YAW_THRESHOLD_FAN_08_COMMAND
        for settle_s in YAW_SENSITIVITY_SETTLES_S:
            confusion = confusion_for(yaw_evidence_items[(fan, settle_s)], threshold)
            lines.append(
                f"| {fan:.1f} | {settle_s:.3f} | {confusion.total} | {confusion.true_positive} | "
                f"{confusion.false_positive} | {confusion.false_negative} | {confusion.true_negative} | "
                f"{confusion.false_launch_rate:.3f} | {confusion.missed_launch_rate:.3f} | {confusion.accuracy:.3f} |"
            )
    lines.append("")
    lines.append("Yaw command detail at the default `0.075 s` settling interval:")
    lines.append("")
    lines.append("| fan | command | predicted launch | windows | TP | FP | FN | TN | clear fraction |")
    lines.append("| ---: | ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: |")
    for fan in (1.0, 0.8):
        threshold = YAW_THRESHOLD_FAN_1_COMMAND if fan >= 0.95 else YAW_THRESHOLD_FAN_08_COMMAND
        items = yaw_evidence_items[(fan, YAW_SETTLE_S)]
        by_command: dict[float, list[ClearEvidence]] = defaultdict(list)
        for item in items:
            by_command[item.command].append(item)
        for command in sorted(by_command):
            confusion = confusion_for(by_command[command], threshold)
            clear_count = sum(1 for item in by_command[command] if item.clear)
            total = len(by_command[command])
            lines.append(
                f"| {fan:.1f} | {command:.2f} | {'yes' if command >= threshold else 'no'} | {total} | "
                f"{confusion.true_positive} | {confusion.false_positive} | {confusion.false_negative} | "
                f"{confusion.true_negative} | {clear_count / total:.3f} |"
            )
    lines.append("")
    lines.append("## Gap Explanation")
    lines.append("")
    lines.append("- One-tick RMSE is small because every sample starts from measured sensor speed and measured gyro yaw rate; most accumulated model bias is erased before the next prediction.")
    lines.append("- In-sample fitting also lets the same runs set `k_f`, `d_f`, `k_y`, and `d_y` before measuring error on those runs.")
    lines.append("- Open-loop rollout exposes what the model does as a plant predictor: launch delay, drivetrain play, damping mismatch, and yaw-gain mismatch compound over roughly 200 ticks.")
    lines.append("- The launch threshold classifier shows why a smooth deadband cannot be interpreted as a deterministic launch rule at threshold commands. Near 0.23 forward command and near the fan-specific yaw thresholds, real windows are mixed rather than cleanly launched or unlaunched.")
    lines.append("")
    lines.append("## Limitations")
    lines.append("")
    lines.append("- The dynamic model is still sensor-level acceleration per normalized command; these CSVs do not identify motor current, wheel torque, or contact patch physics directly.")
    lines.append("- Fan buckets for May phase-20 yaw logs use the fan metadata from the prior offline report because those CSVs do not contain `fan_duty_cycle`.")
    lines.append("- The report uses corrected gyro yaw rate and encoder speed/distance only. Raw IMU lateral acceleration spikes are not used.")
    lines.append("- Forward rollout windows are all-log section-2 windows with enough contiguous samples for 200 ms; shorter pulses are excluded from rollout but still participate in classification and one-tick fits when they have contiguous pairs.")
    lines.append("")
    lines.append("## Files")
    lines.append("")
    lines.append("- Analysis script: `tooling/traction_model_realistic_error_report.py`")
    lines.append("- Report: `TestResults/traction_model_realistic_error_2026-06-08.md`")
    lines.append("")
    return "\n".join(lines)


def main() -> None:
    report = build_report()
    REPORT_PATH.write_text(report, encoding="utf-8")
    print(REPORT_PATH)


if __name__ == "__main__":
    main()
