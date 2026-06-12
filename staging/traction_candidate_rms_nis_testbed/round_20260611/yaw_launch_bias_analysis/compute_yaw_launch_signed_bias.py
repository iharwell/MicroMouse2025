import argparse
import csv
import json
import math
from collections import defaultdict, deque
from dataclasses import dataclass, field
from pathlib import Path
import sys


REPO_ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(REPO_ROOT / "Tools" / "TractionRmsNisTestbed"))

from traction_rms_nis_testbed.estimator_core import (  # noqa: E402
    CandidatePlant,
    ReplayColumnBinding,
    SegmentRuntime,
    estimate_sensor_bias_for_log,
    finite,
    format_number,
    group_segments_by_log,
    load_candidates,
    load_covariance,
    make_candidate_runtime,
    replace_sample_accel_bias,
    sample_from_bound_row,
    segment_specs_from_manifest,
)


STREAMS = (
    "yaw_rate",
    "forward_accel",
    "right_accel",
)


@dataclass
class SignedStats:
    values: list[float] = field(default_factory=list)
    prediction_sum: float = 0.0
    measurement_sum: float = 0.0
    segment_ids: set[str] = field(default_factory=set)

    def add(self, value: float, measurement: float, prediction: float, segment_id: str) -> None:
        if not finite(value):
            return
        self.values.append(value)
        self.measurement_sum += measurement
        self.prediction_sum += prediction
        self.segment_ids.add(segment_id)

    def row(self, candidate_id: str, stream: str, bin_kind: str, bin_value: str) -> dict[str, str]:
        values = sorted(self.values)
        count = len(values)
        if count == 0:
            mean = median = rms = math.nan
            pos_fraction = neg_fraction = zero_fraction = math.nan
            measurement_mean = prediction_mean = math.nan
        else:
            mean = sum(values) / count
            mid = count // 2
            median = values[mid] if count % 2 else 0.5 * (values[mid - 1] + values[mid])
            rms = math.sqrt(sum(value * value for value in values) / count)
            pos_fraction = sum(1 for value in values if value > 0.0) / count
            neg_fraction = sum(1 for value in values if value < 0.0) / count
            zero_fraction = 1.0 - pos_fraction - neg_fraction
            measurement_mean = self.measurement_sum / count
            prediction_mean = self.prediction_sum / count
        return {
            "candidate_id": candidate_id,
            "stream": stream,
            "bin_kind": bin_kind,
            "bin_value": bin_value,
            "count": str(count),
            "segment_count": str(len(self.segment_ids)),
            "mean_residual": format_number(mean),
            "median_residual": format_number(median),
            "rms_residual": format_number(rms),
            "measurement_mean": format_number(measurement_mean),
            "prediction_mean": format_number(prediction_mean),
            "positive_fraction": format_number(pos_fraction),
            "negative_fraction": format_number(neg_fraction),
            "zero_fraction": format_number(zero_fraction),
            "mean_sign": sign_label(mean),
            "median_sign": sign_label(median),
        }


def sign_label(value: float, epsilon: float = 1.0e-9) -> str:
    if not finite(value) or abs(value) <= epsilon:
        return "zero"
    return "positive" if value > 0.0 else "negative"


def yaw_command(sample) -> float:
    return 0.5 * (sample.right_command - sample.left_command)


def command_signature(sample) -> str:
    return f"pair={sample.left_command:.2f},{sample.right_command:.2f};yaw={yaw_command(sample):.2f}"


def polarity(value: float) -> str:
    if value > 1.0e-9:
        return "positive_yaw_command"
    if value < -1.0e-9:
        return "negative_yaw_command"
    return "zero_yaw_command"


def high_command_bin(value: float, threshold: float) -> str:
    return "high_command" if abs(value) >= threshold else "low_command"


def add_bins(stats, candidate_id, stream, residual, measurement, prediction, sample, high_threshold):
    yc = yaw_command(sample)
    for bin_kind, bin_value in (
        ("all", "all"),
        ("command_polarity", polarity(yc)),
        ("high_command", high_command_bin(yc, high_threshold)),
        ("command_bucket", command_signature(sample)),
    ):
        stats[(candidate_id, stream, bin_kind, bin_value)].add(
            residual,
            measurement,
            prediction,
            sample.segment_id,
        )


def compute(args: argparse.Namespace) -> dict[str, object]:
    candidates = load_candidates(Path(args.candidate_config).resolve())
    vehicle, covariance = load_covariance(Path(args.covariance_config).resolve())
    segments = [
        segment
        for segment in segment_specs_from_manifest(Path(args.segment_manifest).resolve(), REPO_ROOT)
        if segment.stage == "yaw_launch"
    ]
    bias_segments = segment_specs_from_manifest(Path(args.bias_segment_manifest).resolve(), REPO_ROOT)
    bias_groups = {
        path: grouped
        for path, grouped in group_segments_by_log(bias_segments)
    }
    candidate_plants = {
        candidate.candidate_id: CandidatePlant(vehicle, candidate)
        for candidate in candidates
    }
    stats: dict[tuple[str, str, str, str], SignedStats] = defaultdict(SignedStats)
    processed_samples = 0
    processed_segments = 0
    log_count = 0

    for log_path, log_segments in group_segments_by_log(segments):
        log_count += 1
        sorted_segments = sorted(log_segments, key=lambda item: item.start_row_index)
        processed_segments += len(sorted_segments)
        bias_summary = estimate_sensor_bias_for_log(
            log_path,
            bias_groups.get(log_path, sorted_segments),
            vehicle,
        )
        pending = deque(sorted_segments)
        active: list[SegmentRuntime] = []
        with log_path.open(newline="", encoding="utf-8-sig") as handle:
            reader = csv.reader(handle)
            fieldnames = next(reader, None)
            if not fieldnames:
                continue
            columns = ReplayColumnBinding.from_fieldnames(fieldnames)
            for row_index, row in enumerate(reader):
                while pending and pending[0].start_row_index <= row_index:
                    segment = pending.popleft()
                    active.append(
                        SegmentRuntime(
                            segment=segment,
                            candidate_runtime=make_candidate_runtime(
                                candidates,
                                vehicle,
                                covariance,
                                "residual",
                                candidate_plants,
                            ),
                            aggregate_keys={},
                        )
                    )
                if not active:
                    if not pending:
                        break
                    continue

                still_active = []
                for runtime in active:
                    segment = runtime.segment
                    if segment.end_row_index >= 0 and row_index > segment.end_row_index:
                        continue
                    sample = sample_from_bound_row(
                        row,
                        columns,
                        segment,
                        row_index,
                        runtime.previous_time_us,
                        vehicle,
                        runtime.previous_sample,
                    )
                    sample = replace_sample_accel_bias(sample, bias_summary.accel)
                    runtime.previous_time_us = sample.master_time_us
                    runtime.previous_sample = sample
                    runtime.emitted_samples += 1
                    processed_samples += 1

                    for candidate in candidates:
                        plant, replay, residual_state = runtime.candidate_runtime[candidate.candidate_id]
                        del replay
                        try:
                            residual_state = plant.propagate(residual_state, sample, sample.dt_s)
                            plant_result = plant.plant_result(residual_state, sample)
                        except (ArithmeticError, OverflowError, ValueError):
                            residual_state = [math.nan] * 9
                            plant_result = None
                        runtime.candidate_runtime[candidate.candidate_id] = (
                            plant,
                            None,
                            residual_state,
                        )
                        if plant_result is None:
                            continue
                        if sample.gyro_valid and finite(sample.yaw_rate_radps) and finite(residual_state[5]):
                            add_bins(
                                stats,
                                candidate.candidate_id,
                                "yaw_rate",
                                sample.yaw_rate_radps - residual_state[5],
                                sample.yaw_rate_radps,
                                residual_state[5],
                                sample,
                                args.high_command_threshold,
                            )
                        if sample.accel_valid and finite(sample.accel_forward_mps2):
                            add_bins(
                                stats,
                                candidate.candidate_id,
                                "forward_accel",
                                sample.accel_forward_mps2 - plant_result.imu_forward_accel_mps2,
                                sample.accel_forward_mps2,
                                plant_result.imu_forward_accel_mps2,
                                sample,
                                args.high_command_threshold,
                            )
                        if sample.accel_valid and finite(sample.accel_right_mps2):
                            add_bins(
                                stats,
                                candidate.candidate_id,
                                "right_accel",
                                sample.accel_right_mps2 - plant_result.imu_right_accel_mps2,
                                sample.accel_right_mps2,
                                plant_result.imu_right_accel_mps2,
                                sample,
                                args.high_command_threshold,
                            )
                    if segment.end_row_index < 0 or row_index < segment.end_row_index:
                        still_active.append(runtime)
                active = still_active

    output_dir = Path(args.output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    rows = [
        stat.row(candidate_id, stream, bin_kind, bin_value)
        for (candidate_id, stream, bin_kind, bin_value), stat in stats.items()
    ]
    rows.sort(key=lambda row: (row["candidate_id"], row["stream"], row["bin_kind"], row["bin_value"]))
    summary_csv = output_dir / "signed_residual_summary.csv"
    with summary_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()) if rows else [])
        if rows:
            writer.writeheader()
            writer.writerows(rows)

    overall_rows = [
        row
        for row in rows
        if row["bin_kind"] in ("all", "command_polarity", "high_command")
    ]
    compact_csv = output_dir / "signed_residual_compact.csv"
    with compact_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()) if rows else [])
        if rows:
            writer.writeheader()
            writer.writerows(overall_rows)

    summary = {
        "schema_version": 1,
        "sign_convention": "residual = measurement - prediction; positive means model under-predicted the measured stream, negative means model over-predicted it",
        "replay_mode": "residual plant replay, no EKF/logged UKF state",
        "production_equivalent_streams": list(STREAMS),
        "processed_segments": processed_segments,
        "processed_samples": processed_samples,
        "source_log_count": log_count,
        "candidate_config": str(Path(args.candidate_config).resolve()),
        "covariance_config": str(Path(args.covariance_config).resolve()),
        "segment_manifest": str(Path(args.segment_manifest).resolve()),
        "bias_segment_manifest": str(Path(args.bias_segment_manifest).resolve()),
        "high_command_threshold_abs_yaw_command": args.high_command_threshold,
        "summary_csv": str(summary_csv),
        "compact_csv": str(compact_csv),
    }
    (output_dir / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return summary


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--candidate-config", required=True)
    parser.add_argument("--covariance-config", required=True)
    parser.add_argument("--segment-manifest", required=True)
    parser.add_argument("--bias-segment-manifest", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--high-command-threshold", type=float, default=0.65)
    return parser.parse_args()


if __name__ == "__main__":
    result = compute(parse_args())
    print(json.dumps(result, indent=2, sort_keys=True))
