#!/usr/bin/env python3
"""Static-section simplex UKF boundedness check for the traction testbed."""

from __future__ import annotations

import csv
import json
import math
import statistics
import sys
from dataclasses import replace
from pathlib import Path
from typing import Any, Callable, Iterable


SCRIPT_DIR = Path(__file__).resolve().parent
ROUND_DIR = SCRIPT_DIR.parent
TESTBED_DIR = ROUND_DIR.parent
REPO_ROOT = TESTBED_DIR.parents[1]
TOOL_DIR = REPO_ROOT / "Tools" / "TractionRmsNisTestbed"
sys.path.insert(0, str(TOOL_DIR))

from traction_rms_nis_testbed.estimator_core import (  # noqa: E402
    HEADING,
    N,
    VF,
    VR,
    YAW_RATE,
    CandidatePlant,
    ReplaySample,
    add_matrix,
    diagonal,
    format_number,
    load_candidates,
    load_covariance,
    normalize_angle,
    read_segment_samples,
    segment_specs_from_manifest,
)


SELECTED_MANIFEST = ROUND_DIR / "static_stability_analysis" / "selected_static_segment_manifest.json"
COMBINED_CONFIG = ROUND_DIR / "static_stability_analysis" / "combined_static_candidate_config.json"
COVARIANCE_CONFIG = TESTBED_DIR / "covariance_conservative.json"
OUTPUT_ROWS = SCRIPT_DIR / "simplex_ukf_rows.csv"
OUTPUT_METRICS = SCRIPT_DIR / "static_simplex_ukf_metrics.csv"
OUTPUT_SUMMARY = SCRIPT_DIR / "static_simplex_ukf_summary.json"
OUTPUT_REPORT = SCRIPT_DIR / "static_simplex_ukf_report.md"

STATE_FIELDS = ("vf_mps", "vr_mps", "yaw_rate_radps", "heading_rad")
PREDICTION_FIELDS = (
    "predicted_forward_accel_mps2",
    "predicted_right_accel_mps2",
    "predicted_yaw_accel_radps2",
)
PRODUCTION_NIS_FIELDS = ("yaw_rate_nis", "forward_accel_nis", "right_accel_nis")

ROW_FIELDS = (
    "candidate_id",
    "case_id",
    "segment_id",
    "source_row_index",
    "master_time_us",
    "dt_s",
    "vf_mps",
    "vr_mps",
    "yaw_rate_radps",
    "heading_rad",
    "predicted_forward_accel_mps2",
    "predicted_right_accel_mps2",
    "predicted_yaw_accel_radps2",
    "max_abs_sigma_state",
    "max_abs_sigma_prediction",
    "covariance_trace",
    "yaw_rate_nis",
    "forward_accel_nis",
    "right_accel_nis",
)

METRIC_FIELDS = (
    "candidate_id",
    "case_id",
    "pass",
    "samples",
    "duration_s",
    "finite",
    "sigma_nonfinite_count",
    "factorization_failure_count",
    "final_abs_vf_mps",
    "final_abs_vr_mps",
    "final_abs_yaw_rate_radps",
    "final_abs_heading_rad",
    "max_abs_vf_mps",
    "max_abs_vr_mps",
    "max_abs_yaw_rate_radps",
    "max_abs_heading_rad",
    "pred_forward_accel_mean_mps2",
    "pred_right_accel_mean_mps2",
    "pred_yaw_accel_mean_radps2",
    "pred_forward_accel_max_abs_mps2",
    "pred_right_accel_max_abs_mps2",
    "pred_yaw_accel_max_abs_radps2",
    "max_abs_sigma_state",
    "max_abs_sigma_prediction",
    "max_covariance_trace",
    "yaw_rate_rms_nis",
    "forward_accel_rms_nis",
    "right_accel_rms_nis",
)


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def finite(value: float) -> bool:
    return math.isfinite(value)


def finite_mean(values: list[float]) -> float:
    return statistics.fmean(values) if values else math.nan


def max_abs(values: Iterable[float]) -> float:
    values = list(values)
    if not values:
        return math.nan
    if not all(finite(value) for value in values):
        return math.inf
    return max(abs(value) for value in values)


def rms(values: list[float]) -> float:
    if not values:
        return math.nan
    return math.sqrt(statistics.fmean(value * value for value in values))


def transpose(matrix: list[list[float]]) -> list[list[float]]:
    return [list(row) for row in zip(*matrix)]


def matvec(matrix: list[list[float]], vector: list[float]) -> list[float]:
    return [sum(row[col] * vector[col] for col in range(len(vector))) for row in matrix]


def outer(left: list[float], right: list[float]) -> list[list[float]]:
    return [[lval * rval for rval in right] for lval in left]


def matrix_trace(matrix: list[list[float]]) -> float:
    return sum(matrix[index][index] for index in range(len(matrix)))


def symmetrize(matrix: list[list[float]]) -> list[list[float]]:
    return [
        [0.5 * (matrix[row][col] + matrix[col][row]) for col in range(len(matrix))]
        for row in range(len(matrix))
    ]


def add_scaled_outer(target: list[list[float]], vector: list[float], scale: float) -> None:
    for row in range(N):
        row_value = vector[row]
        for col in range(N):
            target[row][col] += scale * row_value * vector[col]


def regularized_cholesky(covariance: list[list[float]]) -> tuple[list[list[float]], bool]:
    candidate = symmetrize(covariance)
    scale = max(1.0, max(abs(candidate[index][index]) for index in range(N)))
    for attempt in range(14):
        jitter = 0.0 if attempt == 0 else (10.0 ** (attempt - 10)) * scale
        matrix = [row.copy() for row in candidate]
        for index in range(N):
            matrix[index][index] = max(matrix[index][index] + jitter, 1.0e-12 * scale)
        lower = [[0.0 for _ in range(N)] for _ in range(N)]
        ok = True
        for row in range(N):
            for col in range(row + 1):
                value = matrix[row][col] - sum(lower[row][k] * lower[col][k] for k in range(col))
                if row == col:
                    if value <= 0.0 or not finite(value):
                        ok = False
                        break
                    lower[row][col] = math.sqrt(value)
                else:
                    lower[row][col] = value / lower[col][col]
            if not ok:
                break
        if ok:
            return lower, attempt > 0
    fallback = math.sqrt(max(scale, 1.0e-9)) * 1.0e-3
    return diagonal([fallback for _ in range(N)]), True


def solve_linear(matrix: list[list[float]], rhs: list[float]) -> list[float]:
    a = [row.copy() + [rhs[index]] for index, row in enumerate(matrix)]
    for col in range(N):
        pivot = max(range(col, N), key=lambda row: abs(a[row][col]))
        if abs(a[pivot][col]) < 1.0e-18:
            raise ArithmeticError("singular innovation covariance")
        if pivot != col:
            a[col], a[pivot] = a[pivot], a[col]
        pivot_value = a[col][col]
        for item in range(col, N + 1):
            a[col][item] /= pivot_value
        for row in range(N):
            if row == col:
                continue
            factor = a[row][col]
            if factor == 0.0:
                continue
            for item in range(col, N + 1):
                a[row][item] -= factor * a[col][item]
    return [a[row][N] for row in range(N)]


def simplex_points(mean: list[float], covariance: list[list[float]]) -> tuple[list[list[float]], bool]:
    sqrt_covariance, repaired = regularized_cholesky(covariance)
    outer_weight = 1.0 / float(N + 1)
    canonical = [[0.0 for _ in range(N + 2)] for _ in range(N)]
    first_axis_scale = 1.0 / math.sqrt(2.0 * outer_weight)
    canonical[0][1] = -first_axis_scale
    canonical[0][2] = first_axis_scale
    for dimension in range(2, N + 1):
        row = dimension - 1
        appended_scale = math.sqrt(1.0 / (float(dimension) * float(dimension + 1) * outer_weight))
        terminal_scale = math.sqrt(float(dimension) / (float(dimension + 1) * outer_weight))
        for col in range(1, dimension + 1):
            canonical[row][col] = -appended_scale
        canonical[row][dimension + 1] = terminal_scale

    points = [mean.copy()]
    for col in range(1, N + 2):
        delta = matvec(sqrt_covariance, [canonical[row][col] for row in range(N)])
        point = [mean[index] + delta[index] for index in range(N)]
        point[HEADING] = normalize_angle(point[HEADING])
        points.append(point)
    return points, repaired


def simplex_mean(points: list[list[float]]) -> list[float]:
    weight = 1.0 / float(N + 1)
    base = points[0]
    mean_delta = [0.0 for _ in range(N)]
    for point in points[1:]:
        for index in range(N):
            delta = point[index] - base[index]
            if index == HEADING:
                delta = normalize_angle(delta)
            mean_delta[index] += weight * delta
    mean = [base[index] + mean_delta[index] for index in range(N)]
    mean[HEADING] = normalize_angle(mean[HEADING])
    return mean


def simplex_covariance(points: list[list[float]], mean: list[float]) -> list[list[float]]:
    covariance = [[0.0 for _ in range(N)] for _ in range(N)]
    weights = [2.0] + [1.0 / float(N + 1) for _ in range(N + 1)]
    for point, weight in zip(points, weights):
        delta = []
        for index in range(N):
            value = point[index] - mean[index]
            if index == HEADING:
                value = normalize_angle(value)
            delta.append(value)
        add_scaled_outer(covariance, delta, weight)
    return symmetrize(covariance)


def measurement_update(
    state: list[float],
    covariance: list[list[float]],
    h: Callable[[list[float]], float],
    measurement: float,
    measurement_variance: float,
) -> tuple[list[float], list[list[float]], float, bool, bool]:
    points, repaired = simplex_points(state, covariance)
    predicted = [h(point) for point in points]
    if not all(finite(value) for value in predicted):
        return state, covariance, math.nan, False, repaired
    mean_z = predicted[0] + sum((1.0 / float(N + 1)) * (value - predicted[0]) for value in predicted[1:])
    weights = [2.0] + [1.0 / float(N + 1) for _ in range(N + 1)]
    s = measurement_variance
    cross = [0.0 for _ in range(N)]
    for point, z, weight in zip(points, predicted, weights):
        dz = z - mean_z
        s += weight * dz * dz
        for index in range(N):
            dx = point[index] - state[index]
            if index == HEADING:
                dx = normalize_angle(dx)
            cross[index] += weight * dx * dz
    if not finite(s) or s <= 1.0e-12:
        return state, covariance, math.nan, False, repaired
    innovation = measurement - mean_z
    nis = innovation * innovation / s
    gain = [value / s for value in cross]
    updated = [state[index] + gain[index] * innovation for index in range(N)]
    updated[HEADING] = normalize_angle(updated[HEADING])
    updated_covariance = [
        [covariance[row][col] - gain[row] * s * gain[col] for col in range(N)]
        for row in range(N)
    ]
    return updated, symmetrize(updated_covariance), nis, True, repaired


class SimplexReplay:
    def __init__(self, plant: CandidatePlant, covariance_config: Any):
        self.plant = plant
        self.covariance_config = covariance_config
        self.state = [0.0 for _ in range(N)]
        self.covariance = diagonal([std * std for std in covariance_config.initial_state_std])
        self.sigma_nonfinite_count = 0
        self.factorization_failure_count = 0

    def predict(self, sample: ReplaySample) -> tuple[Any, float, float, float]:
        previous_state = self.state.copy()
        points, repaired = simplex_points(self.state, self.covariance)
        if repaired:
            self.factorization_failure_count += 1
        propagated: list[list[float]] = []
        prediction_values: list[float] = []
        for point in points:
            try:
                next_point = self.plant.propagate(point, sample, sample.dt_s)
                plant_result = self.plant.plant_result(next_point, sample)
                values = [
                    *next_point,
                    plant_result.imu_forward_accel_mps2,
                    plant_result.imu_right_accel_mps2,
                    plant_result.yaw_accel_radps2,
                ]
            except (ArithmeticError, OverflowError, ValueError):
                values = [math.nan]
                next_point = [math.nan for _ in range(N)]
            if not all(finite(value) for value in values):
                self.sigma_nonfinite_count += 1
            propagated.append(next_point)
            prediction_values.extend(values)
        self.state = simplex_mean(propagated)
        self.covariance = add_matrix(
            simplex_covariance(propagated, self.state),
            self.process_noise_like(previous_state, sample),
        )
        plant_result = self.plant.plant_result(self.state, sample)
        return plant_result, max_abs(value for point in propagated for value in point), max_abs(prediction_values), matrix_trace(self.covariance)

    def process_noise_like(self, state: list[float], sample: ReplaySample) -> list[list[float]]:
        # Reuse the testbed covariance recipe without invoking EKF propagation.
        from traction_rms_nis_testbed.estimator_core import EkfReplay

        return EkfReplay(self.plant, self.covariance_config).process_noise(state, sample, sample.dt_s)

    def update_production_measurements(self, sample: ReplaySample) -> dict[str, float]:
        nis_values: dict[str, float] = {}
        measurement = self.covariance_config.measurement
        if sample.gyro_valid and finite(sample.yaw_rate_radps):
            self.state, self.covariance, nis, ok, repaired = measurement_update(
                self.state,
                self.covariance,
                lambda state: state[YAW_RATE],
                sample.yaw_rate_radps,
                measurement.yaw_rate_sigma_radps**2,
            )
            nis_values["yaw_rate_nis"] = nis
            self.sigma_nonfinite_count += 0 if ok else 1
            self.factorization_failure_count += 1 if repaired else 0
        if sample.accel_valid and finite(sample.accel_forward_mps2):
            self.state, self.covariance, nis, ok, repaired = measurement_update(
                self.state,
                self.covariance,
                lambda state: self.plant.plant_result(state, sample).imu_forward_accel_mps2,
                sample.accel_forward_mps2,
                measurement.accel_sigma_mps2**2,
            )
            nis_values["forward_accel_nis"] = nis
            self.sigma_nonfinite_count += 0 if ok else 1
            self.factorization_failure_count += 1 if repaired else 0
        if sample.accel_valid and finite(sample.accel_right_mps2):
            self.state, self.covariance, nis, ok, repaired = measurement_update(
                self.state,
                self.covariance,
                lambda state: self.plant.plant_result(state, sample).imu_right_accel_mps2,
                sample.accel_right_mps2,
                measurement.accel_sigma_mps2**2,
            )
            nis_values["right_accel_nis"] = nis
            self.sigma_nonfinite_count += 0 if ok else 1
            self.factorization_failure_count += 1 if repaired else 0
        return nis_values


def zero_encoder_only_sample(sample: ReplaySample, previous: ReplaySample | None) -> ReplaySample:
    previous_zero = 0.0 if previous is not None else None
    return replace(
        sample,
        left_command=0.0,
        right_command=0.0,
        left_wheel_rate_radps=0.0,
        right_wheel_rate_radps=0.0,
        yaw_rate_radps=0.0,
        accel_forward_mps2=math.nan,
        accel_right_mps2=math.nan,
        accel_valid=False,
        gyro_valid=False,
        previous_left_wheel_rate_radps=previous_zero,
        previous_right_wheel_rate_radps=previous_zero,
        previous_yaw_rate_radps=previous_zero,
    )


def build_metrics(
    candidate_id: str,
    case_id: str,
    replay: SimplexReplay,
    rows: list[dict[str, float]],
    duration_s: float,
) -> dict[str, Any]:
    state_values = {field: [row[field] for row in rows] for field in STATE_FIELDS}
    pred_values = {field: [row[field] for row in rows] for field in PREDICTION_FIELDS}
    nis_values = {field: [row[field] for row in rows if finite(row[field])] for field in PRODUCTION_NIS_FIELDS}
    final = rows[-1] if rows else {}
    all_state_prediction_values = [
        value
        for field_values in (*state_values.values(), *pred_values.values())
        for value in field_values
    ]
    finite_all = all(finite(value) for value in all_state_prediction_values)
    bounded = (
        finite_all
        and replay.sigma_nonfinite_count == 0
        and max_abs(state_values["vf_mps"]) < 1.0
        and max_abs(state_values["vr_mps"]) < 1.0
        and max_abs(state_values["yaw_rate_radps"]) < 1.0
    )
    return {
        "candidate_id": candidate_id,
        "case_id": case_id,
        "pass": bounded,
        "samples": len(rows),
        "duration_s": duration_s,
        "finite": finite_all,
        "sigma_nonfinite_count": replay.sigma_nonfinite_count,
        "factorization_failure_count": replay.factorization_failure_count,
        "final_abs_vf_mps": abs(float(final.get("vf_mps", math.nan))),
        "final_abs_vr_mps": abs(float(final.get("vr_mps", math.nan))),
        "final_abs_yaw_rate_radps": abs(float(final.get("yaw_rate_radps", math.nan))),
        "final_abs_heading_rad": abs(float(final.get("heading_rad", math.nan))),
        "max_abs_vf_mps": max_abs(state_values["vf_mps"]),
        "max_abs_vr_mps": max_abs(state_values["vr_mps"]),
        "max_abs_yaw_rate_radps": max_abs(state_values["yaw_rate_radps"]),
        "max_abs_heading_rad": max_abs(state_values["heading_rad"]),
        "pred_forward_accel_mean_mps2": finite_mean(pred_values["predicted_forward_accel_mps2"]),
        "pred_right_accel_mean_mps2": finite_mean(pred_values["predicted_right_accel_mps2"]),
        "pred_yaw_accel_mean_radps2": finite_mean(pred_values["predicted_yaw_accel_radps2"]),
        "pred_forward_accel_max_abs_mps2": max_abs(pred_values["predicted_forward_accel_mps2"]),
        "pred_right_accel_max_abs_mps2": max_abs(pred_values["predicted_right_accel_mps2"]),
        "pred_yaw_accel_max_abs_radps2": max_abs(pred_values["predicted_yaw_accel_radps2"]),
        "max_abs_sigma_state": max_abs(row["max_abs_sigma_state"] for row in rows),
        "max_abs_sigma_prediction": max_abs(row["max_abs_sigma_prediction"] for row in rows),
        "max_covariance_trace": max_abs(row["covariance_trace"] for row in rows),
        "yaw_rate_rms_nis": rms(nis_values["yaw_rate_nis"]),
        "forward_accel_rms_nis": rms(nis_values["forward_accel_nis"]),
        "right_accel_rms_nis": rms(nis_values["right_accel_nis"]),
    }


def write_csv(path: Path, fieldnames: Iterable[str], rows: list[dict[str, Any]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=tuple(fieldnames))
        writer.writeheader()
        writer.writerows(rows)


def write_report(metrics: list[dict[str, Any]], segment: dict[str, Any]) -> None:
    lines = [
        "# Static Simplex UKF Analysis",
        "",
        f"- Segment: `{segment.get('segment_id')}`",
        f"- Duration: `{segment.get('segment_duration_s')}` s",
        "- Sigma policy: production simplex geometry, 11 active sigma points for 9 states.",
        "- Full static replay case: zero logged commands and wheel rates from the segment, with gyro/accel updates when valid.",
        "- Prediction-only case: commands and wheel rates forced to zero, gyro/accel updates disabled.",
        "- No logged UKF state and no encoder NIS are consumed.",
        f"- Candidate config: `{COMBINED_CONFIG}`",
        f"- Covariance config: `{COVARIANCE_CONFIG}`",
        "",
        "| Model | Case | Pass | Final abs vf/vr/yaw_rate/yaw | Max abs vf/vr/yaw_rate/yaw | Pred accel mean f/r/yaw | Pred accel max abs f/r/yaw | RMS NIS yaw/f/r | Sigma nonfinite |",
        "| --- | --- | --- | --- | --- | --- | --- | --- | --- |",
    ]
    for row in metrics:
        final = "/".join(
            format_number(row[field])
            for field in (
                "final_abs_vf_mps",
                "final_abs_vr_mps",
                "final_abs_yaw_rate_radps",
                "final_abs_heading_rad",
            )
        )
        maxes = "/".join(
            format_number(row[field])
            for field in (
                "max_abs_vf_mps",
                "max_abs_vr_mps",
                "max_abs_yaw_rate_radps",
                "max_abs_heading_rad",
            )
        )
        pred_mean = "/".join(
            format_number(row[field])
            for field in (
                "pred_forward_accel_mean_mps2",
                "pred_right_accel_mean_mps2",
                "pred_yaw_accel_mean_radps2",
            )
        )
        pred_max = "/".join(
            format_number(row[field])
            for field in (
                "pred_forward_accel_max_abs_mps2",
                "pred_right_accel_max_abs_mps2",
                "pred_yaw_accel_max_abs_radps2",
            )
        )
        nis = "/".join(
            format_number(row[field])
            for field in ("yaw_rate_rms_nis", "forward_accel_rms_nis", "right_accel_rms_nis")
        )
        lines.append(
            f"| `{row['candidate_id']}` | `{row['case_id']}` | `{str(row['pass']).lower()}` | "
            f"{final} | {maxes} | {pred_mean} | {pred_max} | {nis} | "
            f"{row['sigma_nonfinite_count']} |"
        )
    OUTPUT_REPORT.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    SCRIPT_DIR.mkdir(parents=True, exist_ok=True)
    selected_manifest = load_json(SELECTED_MANIFEST)
    segment_payload = selected_manifest["segments"][0]
    segments = segment_specs_from_manifest(SELECTED_MANIFEST, REPO_ROOT)
    if len(segments) != 1:
        raise RuntimeError(f"Expected one selected static segment, got {len(segments)}")
    segment = segments[0]
    duration_s = float(segment_payload.get("segment_duration_s", 0.0) or 0.0)
    candidates = load_candidates(COMBINED_CONFIG)
    vehicle, covariance = load_covariance(COVARIANCE_CONFIG)
    samples = list(read_segment_samples(segment, vehicle, max_rows=0))

    row_artifacts: list[dict[str, Any]] = []
    metrics: list[dict[str, Any]] = []
    cases = ("full_static_replay", "prediction_only_zero_encoder")
    for candidate in candidates:
        for case_id in cases:
            plant = CandidatePlant(vehicle, candidate)
            replay = SimplexReplay(plant, covariance)
            numeric_rows: list[dict[str, float]] = []
            previous_zeroed: ReplaySample | None = None
            for sample in samples:
                replay_sample = (
                    zero_encoder_only_sample(sample, previous_zeroed)
                    if case_id == "prediction_only_zero_encoder"
                    else sample
                )
                plant_before_update, max_sigma_state, max_sigma_prediction, covariance_trace = replay.predict(replay_sample)
                nis_values = (
                    replay.update_production_measurements(replay_sample)
                    if case_id == "full_static_replay"
                    else {}
                )
                plant_after_update = plant.plant_result(replay.state, replay_sample)
                row = {
                    "candidate_id": candidate.candidate_id,
                    "case_id": case_id,
                    "segment_id": replay_sample.segment_id,
                    "source_row_index": replay_sample.source_row_index,
                    "master_time_us": replay_sample.master_time_us,
                    "dt_s": replay_sample.dt_s,
                    "vf_mps": replay.state[VF],
                    "vr_mps": replay.state[VR],
                    "yaw_rate_radps": replay.state[YAW_RATE],
                    "heading_rad": replay.state[HEADING],
                    "predicted_forward_accel_mps2": plant_after_update.imu_forward_accel_mps2,
                    "predicted_right_accel_mps2": plant_after_update.imu_right_accel_mps2,
                    "predicted_yaw_accel_radps2": plant_after_update.yaw_accel_radps2,
                    "max_abs_sigma_state": max_sigma_state,
                    "max_abs_sigma_prediction": max_sigma_prediction,
                    "covariance_trace": covariance_trace,
                    "yaw_rate_nis": nis_values.get("yaw_rate_nis", math.nan),
                    "forward_accel_nis": nis_values.get("forward_accel_nis", math.nan),
                    "right_accel_nis": nis_values.get("right_accel_nis", math.nan),
                }
                numeric_rows.append(row)
                row_artifacts.append(
                    {
                        key: format_number(value) if isinstance(value, float) else value
                        for key, value in row.items()
                    }
                )
                previous_zeroed = replay_sample
            metrics.append(build_metrics(candidate.candidate_id, case_id, replay, numeric_rows, duration_s))

    write_csv(OUTPUT_ROWS, ROW_FIELDS, row_artifacts)
    write_csv(
        OUTPUT_METRICS,
        METRIC_FIELDS,
        [
            {
                key: format_number(value) if isinstance(value, float) else value
                for key, value in row.items()
            }
            for row in metrics
        ],
    )
    summary = {
        "schema_version": 1,
        "segment": segment_payload,
        "candidate_config": str(COMBINED_CONFIG),
        "covariance_config": str(COVARIANCE_CONFIG),
        "row_csv": str(OUTPUT_ROWS),
        "metrics_csv": str(OUTPUT_METRICS),
        "report_md": str(OUTPUT_REPORT),
        "uses_logged_ukf_state": False,
        "uses_encoder_nis": False,
        "sigma_policy": "production simplex geometry, N+2 active sigma points",
        "cases": {
            "full_static_replay": "logged zero commands/rates with gyro and accel updates when valid",
            "prediction_only_zero_encoder": "commands/rates forced to zero, gyro/accel invalid",
        },
        "models": {f"{row['candidate_id']}:{row['case_id']}": row for row in metrics},
    }
    OUTPUT_SUMMARY.write_text(json.dumps(summary, indent=2, sort_keys=True, default=str) + "\n", encoding="utf-8")
    write_report(metrics, segment_payload)
    print(f"Wrote {OUTPUT_METRICS}")
    print(f"Wrote {OUTPUT_SUMMARY}")
    print(f"Wrote {OUTPUT_REPORT}")
    print(f"Wrote {OUTPUT_ROWS}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
