from __future__ import annotations

import csv
import math
import statistics
from dataclasses import dataclass
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
TEST_RESULTS = ROOT / "TestResults"
REPORT_PATH = TEST_RESULTS / "traction_model_report_2026-06-08.md"

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
FORWARD_RELIABLE_COMMAND = 0.26
YAW_THRESHOLD_FAN_1_COMMAND = 0.55
YAW_THRESHOLD_FAN_08_COMMAND = 0.65

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


@dataclass(frozen=True)
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


@dataclass(frozen=True)
class Segment:
    label: str
    path: Path
    fan_duty: float
    command: float
    rows: tuple[Sample, ...]


@dataclass(frozen=True)
class ClearSummary:
    command: float
    count: int
    clear_count: int
    median_dwell_s: float
    median_distance_mm: float
    median_peak: float

    @property
    def clear_fraction(self) -> float:
        return self.clear_count / self.count if self.count else 0.0


@dataclass(frozen=True)
class ModelFit:
    forward_gain_mps2_per_command: float
    forward_damping_per_s: float
    yaw_gain_radps2_per_command: float
    yaw_damping_per_s: float


@dataclass(frozen=True)
class Metrics:
    count: int
    linear_speed_rmse_mps: float
    linear_accel_rmse_mps2: float
    yaw_rate_rmse_radps: float
    yaw_accel_rmse_radps2: float
    lateral_accel_rmse_mps2: float


def parse_float(row: dict[str, str], *names: str, default: float = float("nan")) -> float:
    for name in names:
        if name not in row:
            continue
        try:
            return float(row[name])
        except ValueError:
            continue
    return default


def finite(value: float) -> bool:
    return math.isfinite(value)


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


def read_samples(label: str, path: Path, fan_hint: float | None) -> list[Sample]:
    samples: list[Sample] = []
    with path.open(newline="") as handle:
        for row in csv.DictReader(handle):
            fan_duty = parse_float(row, "fan_duty_cycle")
            if not finite(fan_duty) or (fan_duty == 0.0 and fan_hint is not None):
                fan_duty = fan_hint if fan_hint is not None else 0.8
            samples.append(
                Sample(
                    label=label,
                    path=path,
                    time_s=parse_float(row, "master_time_us") / 1.0e6,
                    dt_s=parse_float(row, "dt_us") / 1.0e6,
                    section_id=row.get("section_id", ""),
                    phase_id=row.get("phase_id", ""),
                    repeat_index=row.get("repeat_index", ""),
                    left_command=parse_float(row, "left_drive_command", "left_drive_cmd"),
                    right_command=parse_float(row, "right_drive_command", "right_drive_cmd"),
                    linear_speed_mps=parse_float(row, "measured_linear_speed_mps"),
                    yaw_rate_radps=parse_float(
                        row,
                        "gyro_radps",
                        "measured_yaw_rate_radps",
                        "measured_angular_speed_radps",
                    ),
                    left_encoder_distance_m=parse_float(row, "left_encoder_distance_m"),
                    right_encoder_distance_m=parse_float(row, "right_encoder_distance_m"),
                    fan_duty=fan_duty,
                )
            )
    return samples


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


def append_segment(segments: list[Segment], rows: list[Sample]) -> None:
    if not rows:
        return
    command = statistics.median((row.left_command + row.right_command) * 0.5 for row in rows)
    segments.append(
        Segment(
            label=rows[0].label,
            path=rows[0].path,
            fan_duty=rows[0].fan_duty,
            command=command,
            rows=tuple(rows),
        )
    )


def append_yaw_segment(segments: list[Segment], rows: list[Sample]) -> None:
    if not rows:
        return
    command = statistics.median((row.left_command - row.right_command) * 0.5 for row in rows)
    segments.append(
        Segment(
            label=rows[0].label,
            path=rows[0].path,
            fan_duty=rows[0].fan_duty,
            command=command,
            rows=tuple(rows),
        )
    )


def extract_forward_segments(path: Path, label: str | None = None) -> list[Segment]:
    rows = read_samples(label or path.parent.name, path, None)
    segments: list[Segment] = []
    active: list[Sample] = []
    active_key: tuple[str, float] | None = None
    for sample in rows:
        valid = (
            sample_is_valid(sample)
            and sample.section_id == "2"
            and abs(sample.left_command - sample.right_command) <= 1.0e-5
            and abs(sample.left_command) > 1.0e-6
        )
        if not valid:
            append_segment(segments, active)
            active = []
            active_key = None
            continue
        key = (sample.repeat_index, round(sample.left_command, 3))
        if active_key is not None and key != active_key:
            append_segment(segments, active)
            active = []
        active_key = key
        active.append(sample)
    append_segment(segments, active)
    return segments


def extract_yaw_segments(label: str, path: Path, fan_hint: float) -> list[Segment]:
    rows = read_samples(label, path, fan_hint)
    segments: list[Segment] = []
    active: list[Sample] = []
    active_key: tuple[str, float] | None = None
    for sample in rows:
        valid = (
            sample_is_valid(sample)
            and sample.phase_id == "20"
            and abs(sample.left_command + sample.right_command) <= 1.0e-5
            and abs(sample.left_command) > 1.0e-6
            and abs(sample.right_command) > 1.0e-6
        )
        if not valid:
            append_yaw_segment(segments, active)
            active = []
            active_key = None
            continue
        command = round((sample.left_command - sample.right_command) * 0.5, 3)
        key = (sample.repeat_index, command)
        if active_key is not None and key != active_key:
            append_yaw_segment(segments, active)
            active = []
        active_key = key
        active.append(sample)
    append_yaw_segment(segments, active)
    return segments


def classify_forward_segments(segments: list[Segment]) -> dict[float, ClearSummary]:
    raw: dict[float, list[tuple[float, float, float, bool]]] = {}
    for segment in segments:
        sign = 1.0 if segment.command >= 0.0 else -1.0
        start_time = segment.rows[0].time_s
        evidence = [row for row in segment.rows if row.time_s - start_time >= FORWARD_SETTLE_S]
        if not evidence:
            continue
        dwell = sum(
            row.dt_s
            for row in evidence
            if sign * row.linear_speed_mps >= FORWARD_SPEED_THRESHOLD_MPS
        )
        distance = sign * 0.5 * (
            (evidence[-1].left_encoder_distance_m - evidence[0].left_encoder_distance_m)
            + (evidence[-1].right_encoder_distance_m - evidence[0].right_encoder_distance_m)
        )
        peak_speed = max(sign * row.linear_speed_mps for row in evidence)
        clear = dwell >= FORWARD_DWELL_MIN_S and distance >= FORWARD_DISTANCE_MIN_M
        raw.setdefault(round(abs(segment.command), 2), []).append((dwell, distance, peak_speed, clear))
    return summarize_clear(raw)


def classify_yaw_segments(segments: list[Segment], settle_s: float) -> dict[float, ClearSummary]:
    raw: dict[float, list[tuple[float, float, float, bool]]] = {}
    for segment in segments:
        sign = 1.0 if segment.command >= 0.0 else -1.0
        start_time = segment.rows[0].time_s
        evidence = [row for row in segment.rows if row.time_s - start_time >= settle_s]
        if not evidence:
            continue
        dwell = sum(
            row.dt_s
            for row in evidence
            if sign * row.yaw_rate_radps >= YAW_GYRO_THRESHOLD_RADPS
        )
        differential_distance = sign * (
            (evidence[-1].left_encoder_distance_m - evidence[0].left_encoder_distance_m)
            - (evidence[-1].right_encoder_distance_m - evidence[0].right_encoder_distance_m)
        )
        peak_yaw = max(sign * row.yaw_rate_radps for row in evidence)
        clear = dwell >= YAW_DWELL_MIN_S and differential_distance >= YAW_ENCODER_DIFF_MIN_M
        raw.setdefault(round(abs(segment.command), 2), []).append((dwell, differential_distance, peak_yaw, clear))
    return summarize_clear(raw)


def summarize_clear(raw: dict[float, list[tuple[float, float, float, bool]]]) -> dict[float, ClearSummary]:
    summaries: dict[float, ClearSummary] = {}
    for command, values in raw.items():
        summaries[command] = ClearSummary(
            command=command,
            count=len(values),
            clear_count=sum(1 for _, _, _, clear in values if clear),
            median_dwell_s=statistics.median(value[0] for value in values),
            median_distance_mm=1000.0 * statistics.median(value[1] for value in values),
            median_peak=statistics.median(value[2] for value in values),
        )
    return summaries


def threshold_at_fraction(summaries: dict[float, ClearSummary], fraction: float) -> float | None:
    for command in sorted(summaries):
        summary = summaries[command]
        if summary.clear_fraction >= fraction:
            return command
    return None


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


def smooth_pairs(label: str, path: Path, fan_hint: float) -> list[tuple[Sample, Sample]]:
    rows = read_samples(label, path, fan_hint)
    pairs: list[tuple[Sample, Sample]] = []
    for left, right in zip(rows, rows[1:]):
        if not pair_is_contiguous(left, right):
            continue
        if (
            left.section_id == "5"
            and right.section_id == "5"
            and left.phase_id == "11"
            and right.phase_id == "11"
            and left.repeat_index == right.repeat_index
            and (abs(left.left_command) > 1.0e-6 or abs(left.right_command) > 1.0e-6)
        ):
            pairs.append((left, right))
    return pairs


def fit_model(
    forward_pairs: list[tuple[Sample, Sample]],
    yaw_pairs: list[tuple[Sample, Sample]],
    turn_pairs: list[tuple[Sample, Sample]],
) -> ModelFit:
    forward_matrix = [[0.0, 0.0], [0.0, 0.0]]
    forward_vector = [0.0, 0.0]
    for left, right in forward_pairs:
        command_forward = (left.left_command + left.right_command) * 0.5
        command_yaw = (left.left_command - left.right_command) * 0.5
        effective_forward, _ = radial_deadband(command_forward, command_yaw, left.fan_duty)
        target = (right.linear_speed_mps - left.linear_speed_mps) / left.dt_s
        add_normal(forward_matrix, forward_vector, (effective_forward, -left.linear_speed_mps), target)

    yaw_matrix = [[0.0, 0.0], [0.0, 0.0]]
    yaw_vector = [0.0, 0.0]
    for left, right in yaw_pairs + turn_pairs:
        command_forward = (left.left_command + left.right_command) * 0.5
        command_yaw = (left.left_command - left.right_command) * 0.5
        _, effective_yaw = radial_deadband(command_forward, command_yaw, left.fan_duty)
        lateral_accel = left.linear_speed_mps * left.yaw_rate_radps
        contact_scale = 1.0 / math.sqrt(1.0 + (lateral_accel / CONTACT_LATERAL_ACCEL_CAP_MPS2) ** 2)
        target = (right.yaw_rate_radps - left.yaw_rate_radps) / left.dt_s
        add_normal(yaw_matrix, yaw_vector, (effective_yaw * contact_scale, -left.yaw_rate_radps), target)

    forward_gain, forward_damping = solve_normal(forward_matrix, forward_vector)
    yaw_gain, yaw_damping = solve_normal(yaw_matrix, yaw_vector)
    return ModelFit(
        forward_gain_mps2_per_command=forward_gain,
        forward_damping_per_s=forward_damping,
        yaw_gain_radps2_per_command=yaw_gain,
        yaw_damping_per_s=yaw_damping,
    )


def evaluate_pairs(pairs: list[tuple[Sample, Sample]], fit: ModelFit) -> Metrics:
    if not pairs:
        return Metrics(0, float("nan"), float("nan"), float("nan"), float("nan"), float("nan"))
    se_linear = 0.0
    se_linear_accel = 0.0
    se_yaw = 0.0
    se_yaw_accel = 0.0
    se_lateral = 0.0
    for left, right in pairs:
        command_forward = (left.left_command + left.right_command) * 0.5
        command_yaw = (left.left_command - left.right_command) * 0.5
        effective_forward, effective_yaw = radial_deadband(command_forward, command_yaw, left.fan_duty)
        linear_accel = (
            fit.forward_gain_mps2_per_command * effective_forward
            - fit.forward_damping_per_s * left.linear_speed_mps
        )
        lateral_accel = left.linear_speed_mps * left.yaw_rate_radps
        contact_scale = 1.0 / math.sqrt(1.0 + (lateral_accel / CONTACT_LATERAL_ACCEL_CAP_MPS2) ** 2)
        yaw_accel = (
            fit.yaw_gain_radps2_per_command * effective_yaw * contact_scale
            - fit.yaw_damping_per_s * left.yaw_rate_radps
        )
        predicted_linear = left.linear_speed_mps + left.dt_s * linear_accel
        predicted_yaw = left.yaw_rate_radps + left.dt_s * yaw_accel
        observed_linear_accel = (right.linear_speed_mps - left.linear_speed_mps) / left.dt_s
        observed_yaw_accel = (right.yaw_rate_radps - left.yaw_rate_radps) / left.dt_s
        se_linear += (predicted_linear - right.linear_speed_mps) ** 2
        se_linear_accel += (linear_accel - observed_linear_accel) ** 2
        se_yaw += (predicted_yaw - right.yaw_rate_radps) ** 2
        se_yaw_accel += (yaw_accel - observed_yaw_accel) ** 2
        se_lateral += ((predicted_linear * predicted_yaw) - (right.linear_speed_mps * right.yaw_rate_radps)) ** 2
    count = len(pairs)
    return Metrics(
        count=count,
        linear_speed_rmse_mps=math.sqrt(se_linear / count),
        linear_accel_rmse_mps2=math.sqrt(se_linear_accel / count),
        yaw_rate_rmse_radps=math.sqrt(se_yaw / count),
        yaw_accel_rmse_radps2=math.sqrt(se_yaw_accel / count),
        lateral_accel_rmse_mps2=math.sqrt(se_lateral / count),
    )


def format_float(value: float, digits: int = 3) -> str:
    if not finite(value):
        return "n/a"
    return f"{value:.{digits}f}"


def clear_table(summaries: dict[float, ClearSummary], commands: list[float]) -> list[str]:
    lines = [
        "| command | clear windows | fraction | median dwell (s) | median encoder distance (mm) | median peak |",
        "| ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for command in commands:
        summary = summaries.get(command)
        if summary is None:
            continue
        lines.append(
            "| "
            f"{command:.2f} | {summary.clear_count}/{summary.count} | {summary.clear_fraction:.3f} | "
            f"{summary.median_dwell_s:.3f} | {summary.median_distance_mm:.2f} | {summary.median_peak:.3f} |"
        )
    return lines


def metrics_line(name: str, metrics: Metrics) -> str:
    return (
        f"| {name} | {metrics.count} | {metrics.linear_speed_rmse_mps:.5f} | "
        f"{metrics.linear_accel_rmse_mps2:.3f} | {metrics.yaw_rate_rmse_radps:.5f} | "
        f"{metrics.yaw_accel_rmse_radps2:.3f} | {metrics.lateral_accel_rmse_mps2:.5f} |"
    )


def build_report() -> str:
    launch_segments: list[Segment] = []
    for path in sorted(TEST_RESULTS.glob("mmlog_decode_*/open_floor_main.csv")):
        launch_segments.extend(extract_forward_segments(path))
    forward_summary = classify_forward_segments(launch_segments)

    yaw_segments: list[Segment] = []
    yaw_segments_by_fan: dict[float, list[Segment]] = {0.8: [], 1.0: []}
    for label, path, fan_hint in YAW_LOGS:
        segments = extract_yaw_segments(label, path, fan_hint)
        yaw_segments.extend(segments)
        yaw_segments_by_fan.setdefault(fan_hint, []).extend(segments)

    yaw_summary_by_fan = {
        fan: classify_yaw_segments(segments, YAW_SETTLE_S)
        for fan, segments in yaw_segments_by_fan.items()
    }
    yaw_sensitivity_rows: list[tuple[float, str, str]] = []
    for settle_s in YAW_SENSITIVITY_SETTLES_S:
        fan_1_threshold = threshold_at_fraction(classify_yaw_segments(yaw_segments_by_fan[1.0], settle_s), 0.5)
        fan_08_threshold = threshold_at_fraction(classify_yaw_segments(yaw_segments_by_fan[0.8], settle_s), 0.5)
        yaw_sensitivity_rows.append(
            (
                settle_s,
                format_float(fan_1_threshold, 2) if fan_1_threshold is not None else ">0.55",
                format_float(fan_08_threshold, 2) if fan_08_threshold is not None else ">0.70",
            )
        )

    forward_pairs: list[tuple[Sample, Sample]] = []
    for segment in launch_segments:
        forward_pairs.extend(segment_pairs(segment))
    yaw_pairs: list[tuple[Sample, Sample]] = []
    for segment in yaw_segments:
        yaw_pairs.extend(segment_pairs(segment, YAW_SETTLE_S))
    turn_pairs: list[tuple[Sample, Sample]] = []
    turn_pairs_by_label: dict[str, list[tuple[Sample, Sample]]] = {}
    for label, path, fan_hint in SMOOTH_LOGS:
        pairs = smooth_pairs(label, path, fan_hint)
        turn_pairs.extend(pairs)
        turn_pairs_by_label[label] = pairs

    fit = fit_model(forward_pairs, yaw_pairs, turn_pairs)
    forward_metrics = evaluate_pairs(forward_pairs, fit)
    yaw_metrics = evaluate_pairs(yaw_pairs, fit)
    smooth_metrics = evaluate_pairs(turn_pairs, fit)
    smooth_label_metrics = {label: evaluate_pairs(pairs, fit) for label, pairs in turn_pairs_by_label.items()}

    forward_50 = threshold_at_fraction(forward_summary, 0.5)
    forward_90 = threshold_at_fraction(forward_summary, 0.9)
    forward_95 = threshold_at_fraction(forward_summary, 0.95)

    lines: list[str] = []
    lines.append("# Smooth Traction Model Quantitative Report")
    lines.append("")
    lines.append("## Scope")
    lines.append("")
    lines.append("- Report-only offline analysis. No production code, tests, or build artifacts were modified.")
    lines.append("- Raw/sensor-derived data only: motor commands, encoder-derived linear speed, encoder distances, and corrected gyro yaw rate.")
    lines.append("- Deliberately excluded: `ukf_state_*`, yaw-consistency/NHC estimator columns, estimator-predicted values, and UKF-derived pose drift.")
    lines.append("- `PlantModel` remains the intended future owner. This report does not introduce a production wrapper or alternate owner.")
    lines.append("")
    lines.append("## Candidate Model")
    lines.append("")
    lines.append("Command coordinates:")
    lines.append("")
    lines.append("```text")
    lines.append("c_f = 0.5 * (left_command + right_command)")
    lines.append("c_y = 0.5 * (left_command - right_command)")
    lines.append("```")
    lines.append("")
    lines.append("Smooth generalized launch/contact deadband:")
    lines.append("")
    lines.append("```text")
    lines.append("smooth_plus(x,w) = 0.5 * (x + sqrt(x*x + w*w))")
    lines.append("q = sqrt((c_f/c_f0)^2 + (c_y/c_y0(fan))^2 + w^2)")
    lines.append("scale = smooth_plus(q - 1, w) / (q + 1e-9)")
    lines.append("e_f = c_f * scale")
    lines.append("e_y = c_y * scale")
    lines.append("contact = 1 / sqrt(1 + ((u * r) / a_lat_cap)^2)")
    lines.append("u_dot = k_f * e_f - d_f * u")
    lines.append("r_dot = k_y * e_y * contact - d_y * r")
    lines.append("```")
    lines.append("")
    lines.append(
        "This is UKF-safe because it is continuous and differentiable around launch, zero speed, "
        "and contact-limit regions. It avoids hard `sign()`, hard positive-part functions, "
        "`speed == 0` branches, slip-ratio division by speed, hidden latches, and hard saturation near sigma-point-sensitive regions."
    )
    lines.append("")
    lines.append("## Fitted Parameters")
    lines.append("")
    lines.append("| parameter | value | units / meaning |")
    lines.append("| --- | ---: | --- |")
    lines.append(f"| `c_f0` | {FORWARD_THRESHOLD_COMMAND:.3f} | normalized forward command, 50% raw-sensor launch onset |")
    lines.append(f"| reliable forward command | {FORWARD_RELIABLE_COMMAND:.3f} | normalized command, >=95% clear-launch aggregate |")
    lines.append(f"| `c_y0(fan=1.0)` | {YAW_THRESHOLD_FAN_1_COMMAND:.3f} | normalized in-place yaw command, 75 ms settle classifier |")
    lines.append(f"| `c_y0(fan=0.8)` | {YAW_THRESHOLD_FAN_08_COMMAND:.3f} | normalized in-place yaw command, 75 ms settle classifier |")
    lines.append(f"| `w` | {SMOOTH_WIDTH_COMMAND:.3f} | normalized command smoothing width |")
    lines.append(f"| `a_lat_cap` | {CONTACT_LATERAL_ACCEL_CAP_MPS2:.1f} | m/s^2 kinematic lateral acceleration cap from project note |")
    lines.append(f"| `k_f` | {fit.forward_gain_mps2_per_command:.3f} | m/s^2 per effective command |")
    lines.append(f"| `d_f` | {fit.forward_damping_per_s:.3f} | 1/s longitudinal damping |")
    lines.append(f"| `k_y` | {fit.yaw_gain_radps2_per_command:.3f} | rad/s^2 per effective yaw command |")
    lines.append(f"| `d_y` | {fit.yaw_damping_per_s:.3f} | 1/s yaw-rate damping |")
    lines.append("")
    lines.append("## Forward Launch Threshold")
    lines.append("")
    lines.append(
        f"Window definition: section `2`, equal nonzero left/right commands, first {FORWARD_SETTLE_S:.3f} s ignored, "
        f"clear when signed encoder-derived speed is at least {FORWARD_SPEED_THRESHOLD_MPS:.3f} m/s for "
        f"{FORWARD_DWELL_MIN_S:.3f} s and signed average encoder distance is at least {1000.0 * FORWARD_DISTANCE_MIN_M:.1f} mm."
    )
    lines.append("")
    lines.append(f"- Aggregate 50% onset command: `{format_float(forward_50, 2)}`.")
    lines.append(f"- Aggregate 90% onset command: `{format_float(forward_90, 2)}`.")
    lines.append(f"- Aggregate 95% reliable command: `{format_float(forward_95, 2)}`.")
    lines.append("- The old `0.25`-only statement was wrong; lower-command logs exist and are included here.")
    lines.append("")
    lines.extend(clear_table(forward_summary, [0.17, 0.18, 0.19, 0.20, 0.21, 0.22, 0.23, 0.24, 0.25, 0.26, 0.29, 0.30, 0.35]))
    lines.append("")
    lines.append("## In-Place Yaw Initiation")
    lines.append("")
    lines.append(
        f"Default window definition: phase `20`, opposite equal commands, first {YAW_SETTLE_S:.3f} s ignored for drivetrain play/backlash, "
        f"clear when sign-aligned corrected gyro is at least {YAW_GYRO_THRESHOLD_RADPS:.3f} rad/s for "
        f"{YAW_DWELL_MIN_S:.3f} s and sign-aligned encoder differential distance is at least "
        f"{1000.0 * YAW_ENCODER_DIFF_MIN_M:.1f} mm."
    )
    lines.append("")
    lines.append("| fan duty | predicted command to initiate | evidence |")
    lines.append("| ---: | ---: | --- |")
    for fan in (1.0, 0.8):
        summary = yaw_summary_by_fan[fan]
        threshold = threshold_at_fraction(summary, 0.5)
        threshold_text = format_float(threshold, 2) if threshold is not None else ("`>0.55`" if fan == 1.0 else "`>0.70`")
        lines.append(f"| {fan:.1f} | {threshold_text} | first command with >=50% clear windows after {YAW_SETTLE_S:.3f} s settle |")
    lines.append("")
    lines.append("Sensitivity to settling interval:")
    lines.append("")
    lines.append("| settling interval (s) | fan 1.0 threshold | fan 0.8 threshold |")
    lines.append("| ---: | ---: | ---: |")
    for settle_s, fan_1_text, fan_08_text in yaw_sensitivity_rows:
        lines.append(f"| {settle_s:.3f} | {fan_1_text} | {fan_08_text} |")
    lines.append("")
    lines.append("Fan 1.0 command table:")
    lines.append("")
    lines.extend(clear_table(yaw_summary_by_fan[1.0], [0.30, 0.35, 0.40, 0.45, 0.50, 0.55]))
    lines.append("")
    lines.append("Fan 0.8 command table:")
    lines.append("")
    lines.extend(clear_table(yaw_summary_by_fan[0.8], [0.50, 0.55, 0.60, 0.65, 0.70]))
    lines.append("")
    lines.append("## RMSE Evaluation")
    lines.append("")
    lines.append(
        "RMSE is one control-tick prediction error from measured sensor state and logged commands. "
        "Yaw-launch RMSE uses only post-settling phase-20 evidence samples. Sustained-turn RMSE uses active `section_id=5`, `phase_id=11` rows. "
        "Lateral acceleration is kinematic `u_encoder * gyro_radps`, not raw IMU spike magnitude."
    )
    lines.append("")
    lines.append("| regime | samples | linear speed RMSE (m/s) | linear accel RMSE (m/s^2) | yaw-rate RMSE (rad/s) | yaw accel RMSE (rad/s^2) | lateral accel RMSE (m/s^2) |")
    lines.append("| --- | ---: | ---: | ---: | ---: | ---: | ---: |")
    lines.append(metrics_line("forward launch, all decoded section-2 windows", forward_metrics))
    lines.append(metrics_line("in-place yaw launch, post-settle phase-20 windows", yaw_metrics))
    lines.append(metrics_line("sustained smooth turns, active April windows", smooth_metrics))
    for label, metrics in sorted(smooth_label_metrics.items()):
        lines.append(metrics_line(f"sustained smooth turns, {label}", metrics))
    lines.append("")
    lines.append("## Limitations")
    lines.append("")
    lines.append("- The motor-current and wheel-torque physical chain is not identifiable from these CSVs alone; fitted gains are sensor-level acceleration per normalized command.")
    lines.append("- May phase-20 yaw logs do not contain `fan_duty_cycle`; fan buckets use the user-provided fan metadata.")
    lines.append("- Older April logs sometimes report `fan_duty_cycle=0`; for the smooth-turn fit those are treated as legacy/unknown and assigned the 0.8 contact cap only for this offline model.")
    lines.append("- The phase-20 yaw threshold is sensitive to the settling interval. At 100 ms, fan 1.0 does not reach a 50% clear fraction at any tested command up to 0.55.")
    lines.append("- This is not a long open-loop replay or closed-loop acceptance test. It is a UKF-relevant one-tick plant prediction report using raw/sensor-derived data.")
    lines.append("- Raw IMU acceleration was not used for sustained-turn fit because prior analysis showed non-credible spikes; the report uses kinematic lateral acceleration from encoder speed and gyro yaw rate.")
    lines.append("")
    lines.append("## Files")
    lines.append("")
    lines.append(f"- Analysis script: `tooling/{Path(__file__).name}`")
    lines.append(f"- Report: `TestResults/{REPORT_PATH.name}`")
    lines.append("")
    return "\n".join(lines)


def main() -> None:
    report = build_report()
    REPORT_PATH.write_text(report, encoding="utf-8")
    print(REPORT_PATH)


if __name__ == "__main__":
    main()
