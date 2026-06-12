"""Host-only traction candidate replay and accepted-ANIS/residual artifact generation.

This module is deliberately standalone. It does not import or execute production
robot code; JSON files provide the vehicle, candidate, and covariance defaults.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from concurrent.futures import ProcessPoolExecutor, as_completed
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Iterable, Iterator, Sequence


STATE_NAMES = (
    "px_m",
    "py_m",
    "heading_rad",
    "vf_mps",
    "vr_mps",
    "yaw_rate_radps",
    "delta_af_mps2",
    "delta_ar_mps2",
    "delta_yaw_accel_radps2",
)
PX, PY, HEADING, VF, VR, YAW_RATE, DAF, DAR, DYAW = range(9)
N = len(STATE_NAMES)

NIS_FIELDS = (
    "candidate_id",
    "segment_id",
    "split",
    "stage",
    "run_id",
    "source_path",
    "source_row_index",
    "master_time_us",
    "log_parameter",
    "metric_kind",
    "measurement_dimension",
    "command_bucket",
    "nis",
    "accepted",
    "rejected",
    "gate_threshold",
    "innovation",
    "innovation_variance",
    "measurement",
    "prediction",
    "corrupted",
)

DIAGNOSTIC_FIELDS = (
    "candidate_id",
    "segment_id",
    "split",
    "stage",
    "run_id",
    "source_path",
    "source_row_index",
    "master_time_us",
    "dt_s",
    "left_command",
    "right_command",
    "left_wheel_rate_radps",
    "right_wheel_rate_radps",
    "predicted_left_encoder_wheel_rate_radps",
    "predicted_right_encoder_wheel_rate_radps",
    "fan_duty_cycle",
    "px_m",
    "py_m",
    "heading_rad",
    "vf_mps",
    "vr_mps",
    "yaw_rate_radps",
    "delta_af_mps2",
    "delta_ar_mps2",
    "delta_yaw_accel_radps2",
    "predicted_forward_accel_mps2",
    "predicted_right_accel_mps2",
    "predicted_yaw_accel_radps2",
    "measured_yaw_rate_radps",
    "measured_left_encoder_wheel_rate_radps",
    "measured_right_encoder_wheel_rate_radps",
    "measured_yaw_accel_radps2",
    "measured_forward_accel_mps2",
    "measured_right_accel_mps2",
    "yaw_rate_residual_radps",
    "left_encoder_wheel_rate_residual_radps",
    "right_encoder_wheel_rate_residual_radps",
    "yaw_accel_residual_radps2",
    "forward_accel_residual_mps2",
    "right_accel_residual_mps2",
    "max_contact_relative_speed_mps",
    "load_weighted_contact_relative_speed_mps",
    "load_weighted_lateral_relative_speed_mps",
    "load_weighted_yaw_contact_speed_mps",
    "max_contact_utilization",
    "max_contact_saturation",
    "total_normal_load_n",
    "ground_use",
)

BIAS_SUMMARY_FIELDS = (
    "log_id",
    "log_path",
    "bias_segment_count",
    "accel_bias_forward_mps2",
    "accel_bias_right_mps2",
    "accel_bias_sample_count",
    "gyro_bias_radps",
    "gyro_bias_sample_count",
    "gyro_bias_source",
)

AGGREGATE_FIELDS = (
    "candidate_id",
    "split",
    "stage",
    "log_parameter",
    "measurement_dimension",
    "parameter_field",
    "parameter_value_kind",
    "parameter_value",
    "launch_command_signature",
    "count",
    "accepted_count",
    "rejected_count",
    "rejected_rate",
    "rejected_rate_penalty",
    "finite_count",
    "nonfinite_count",
    "sum_nis",
    "sum_nis_sq",
    "rms_nis",
    "sqrt_mean_nis",
    "accepted_only_sum_nis",
    "accepted_only_sum_nis_sq",
    "accepted_only_rms_nis",
    "accepted_only_sqrt_mean_nis",
    "physical_residual_rms",
    "segment_count",
)

ITEMIZED_AGGREGATE_FIELDS = (
    "candidate_id",
    "split",
    "stage",
    "log_parameter",
    "measurement_dimension",
    "count",
    "accepted_count",
    "rejected_count",
    "rejected_rate",
    "rejected_rate_penalty",
    "finite_count",
    "nonfinite_count",
    "segment_count",
    "score_partition",
    "rms_nis",
    "sqrt_mean_nis",
    "accepted_only_rms_nis",
    "accepted_only_sqrt_mean_nis",
    "expected_rms_nis",
    "guarded_rms_nis",
    "under_expected_penalty",
    "inflation_flag",
    "physical_residual_rms",
    "parameter_field",
    "parameter_value_kind",
    "parameter_value",
    "launch_command_signature",
)

COMMAND_BIN_WIDTH = 0.02
REJECTED_RATE_PENALTY_WEIGHT = 2.0
UNGATED_YAW_RATE_NIS_GATE = math.inf
UNGATED_ENCODER_NIS_GATE = math.inf
ACCEL_BIAS_MAX_DRIVE_COMMAND = 0.08
ACCEL_BIAS_MAX_ENCODER_WHEEL_SPEED_RADPS = 1.0
ACCEL_BIAS_MAX_YAW_RATE_RADPS = 0.12
PRODUCTION_ENCODER_LINEAR_SPEED_SIGMA_MPS = 0.021187
PRODUCTION_ENCODER_YAW_RATE_SIGMA_RADPS = 0.111268
PRODUCTION_MEASUREMENT_NIS_LOG_PARAMETERS = (
    "yaw_rate_nis",
    "forward_accel_nis",
    "right_accel_nis",
)
PRODUCTION_MEASUREMENT_RESIDUAL_TAIL_LOG_PARAMETERS = (
    "yaw_rate_residual_tail",
    "forward_accel_residual_tail",
    "right_accel_residual_tail",
)

DEFAULT_SPLIT_CONFIG = {
    "seed": "traction_rms_nis_testbed_v1",
    "train": 0.6,
    "validation": 0.2,
    "held_out": 0.2,
}
REPORT_SPLITS = ("train", "validation", "held_out", "all")
DEFAULT_TESTBED_STAGING_DIR = (
    Path(__file__).resolve().parents[3] / "staging" / "traction_candidate_rms_nis_testbed"
)
DEFAULT_UKF_VALIDATION_MANIFEST_PATH = DEFAULT_TESTBED_STAGING_DIR / "ukf_validation_5log_manifest.json"
DEFAULT_UKF_VALIDATION_OUTPUT_DIR = DEFAULT_TESTBED_STAGING_DIR / "ukf_validation_5log"

UKF_VALIDATION_CANDIDATE_FIELDS = (
    "candidate_id",
    "status",
    "segments",
    "samples",
    "sigma_points",
    "finite_failures",
    "continuity_failures",
    "covariance_failures",
    "innovation_failures",
    "zero_crossing_events",
    "zero_crossing_failures",
    "max_abs_state",
    "max_abs_prediction",
    "max_state_sigma_span",
    "max_measurement_sigma_span",
    "max_covariance_trace",
    "max_innovation_nis",
    "issue_count",
)

UKF_VALIDATION_EVENT_FIELDS = (
    "candidate_id",
    "segment_id",
    "stage",
    "run_id",
    "source_path",
    "source_row_index",
    "master_time_us",
    "category",
    "severity",
    "detail",
    "metric",
    "value",
    "threshold",
)


class ConfigError(RuntimeError):
    pass


SUPPORTED_CANDIDATE_MODELS = frozenset(
    (
        "current_holdover_approximation",
        "algebraic_envelope",
        "slip_zlock",
        "stribeck_algebraic",
        "load_sensitive_anisotropic",
        "skew_shear",
        "shear_rate",
        "in_shear",
    )
)

ALGEBRAIC_MODEL_DEFAULTS = {
    "peak_friction_coefficient_at_80pct_fan": 1.68,
    "longitudinal_slip_gain_n_per_mps": 18.0,
    "lateral_slip_gain_n_per_mps": 18.0,
    "combined_slip_envelope_exponent": 2.0,
    "low_speed_blend_mps": 0.08,
}

MODEL_PARAMETER_DEFAULTS = {
    "current_holdover_approximation": {
        "peak_friction_coefficient_at_80pct_fan": 1.65,
        "longitudinal_slip_gain_n_per_mps": 16.0,
        "lateral_slip_gain_n_per_mps": 16.0,
        "low_speed_blend_mps": 0.07,
    },
    "algebraic_envelope": ALGEBRAIC_MODEL_DEFAULTS,
    "slip_zlock": {
        **ALGEBRAIC_MODEL_DEFAULTS,
        "stationary_zlock_enabled": 1.0,
        "stationary_zlock_window_s": 0.20,
        "stationary_zlock_command_epsilon": 0.015,
        "stationary_zlock_wheel_rate_epsilon_radps": 0.02,
        "stationary_zlock_yaw_rate_epsilon_radps": 0.06,
        "stationary_zlock_position_sigma_m": 0.00025,
        "stationary_zlock_velocity_sigma_mps": 0.0010,
        "stationary_zlock_yaw_rate_sigma_radps": 0.0030,
    },
    "stribeck_algebraic": {
        **ALGEBRAIC_MODEL_DEFAULTS,
        "dynamic_to_static_grip_ratio": 0.82,
        "stribeck_velocity_mps": 0.16,
        "viscous_slip_damping_n_per_mps": 2.0,
    },
    "load_sensitive_anisotropic": {
        **ALGEBRAIC_MODEL_DEFAULTS,
        "longitudinal_load_transfer_gain": 0.18,
        "lateral_load_transfer_gain": 0.26,
        "longitudinal_load_sensitivity": 0.18,
        "lateral_load_sensitivity": 0.26,
        "yaw_coupling_gain": 0.12,
        "yaw_damping_speed_blend_mps": 0.02,
    },
    "skew_shear": {
        **ALGEBRAIC_MODEL_DEFAULTS,
        "shear_coupling_gain": 0.18,
        "shear_activation_speed_mps": 0.06,
        "shear_drive_force_blend_n": 0.04,
    },
    "shear_rate": {
        **ALGEBRAIC_MODEL_DEFAULTS,
        "shear_rate_peak_force_n": 0.035,
        "shear_rate_activation_mps2": 35.0,
        "shear_rate_breakaway_speed_mps": 0.10,
    },
    "in_shear": {
        **ALGEBRAIC_MODEL_DEFAULTS,
        "inward_lateral_stiffness_gain": 0.25,
        "inward_lateral_grip_gain": 0.18,
        "inward_shear_blend_speed_mps": 0.04,
    },
}


def validate_candidate_model(model: str) -> str:
    if model not in SUPPORTED_CANDIDATE_MODELS:
        supported = ", ".join(sorted(SUPPORTED_CANDIDATE_MODELS))
        raise ConfigError(f"Unsupported candidate model {model!r}; supported models: {supported}")
    return model


def model_parameters(model: str, parameters: dict[str, float]) -> dict[str, float]:
    validate_candidate_model(model)
    merged = dict(MODEL_PARAMETER_DEFAULTS[model])
    merged.update(parameters)
    return merged


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def finite(value: float) -> bool:
    return math.isfinite(value)


def finite_or(value: float, fallback: float) -> float:
    return value if math.isfinite(value) else fallback


def parse_bool(value: Any, default: bool = False) -> bool:
    if value is None or value == "":
        return default
    if isinstance(value, bool):
        return value
    text = str(value).strip().lower()
    if text in ("1", "true", "yes", "y", "valid"):
        return True
    if text in ("0", "false", "no", "n", "invalid"):
        return False
    return default


def parse_float(value: Any, default: float = math.nan) -> float:
    if value is None:
        return default
    text = str(value).strip()
    if not text:
        return default
    try:
        return float(text)
    except ValueError:
        return default


def parse_int(value: Any, default: int = 0) -> int:
    if value is None:
        return default
    text = str(value).strip()
    if not text:
        return default
    try:
        return int(float(text))
    except ValueError:
        return default


def csv_value(row: dict[str, str], names: Sequence[str]) -> str:
    lower_to_name = {key.lower(): key for key in row}
    for name in names:
        key = lower_to_name.get(name.lower())
        if key is None:
            continue
        value = row.get(key, "")
        if value is not None and str(value).strip():
            return str(value).strip()
    return ""


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return angle


def smooth_sign(value: float, epsilon: float) -> float:
    scale = max(abs(epsilon), 1.0e-9)
    return math.tanh(value / scale)


def smooth_scale_to_unit(utilization: float, smoothness: float = 1.0e-6) -> float:
    if not finite(utilization) or utilization <= 1.0:
        return 1.0
    smooth_max = 0.5 * (1.0 + utilization + math.sqrt((utilization - 1.0) ** 2 + smoothness**2))
    return 1.0 / max(smooth_max, 1.0e-9)


def zeros(rows: int, cols: int) -> list[list[float]]:
    return [[0.0 for _ in range(cols)] for _ in range(rows)]


def identity(size: int) -> list[list[float]]:
    matrix = zeros(size, size)
    for index in range(size):
        matrix[index][index] = 1.0
    return matrix


def diagonal(values: Sequence[float]) -> list[list[float]]:
    matrix = zeros(len(values), len(values))
    for index, value in enumerate(values):
        matrix[index][index] = value
    return matrix


def transpose(matrix: Sequence[Sequence[float]]) -> list[list[float]]:
    return [list(row) for row in zip(*matrix)]


def matmul(left: Sequence[Sequence[float]], right: Sequence[Sequence[float]]) -> list[list[float]]:
    rows = len(left)
    cols = len(right[0])
    inner = len(right)
    result = zeros(rows, cols)
    for row in range(rows):
        for mid in range(inner):
            value = left[row][mid]
            if value == 0.0:
                continue
            for col in range(cols):
                result[row][col] += value * right[mid][col]
    return result


def matvec(matrix: Sequence[Sequence[float]], vector: Sequence[float]) -> list[float]:
    return [sum(row[col] * vector[col] for col in range(len(vector))) for row in matrix]


def add_matrix(left: Sequence[Sequence[float]], right: Sequence[Sequence[float]]) -> list[list[float]]:
    return [
        [left[row][col] + right[row][col] for col in range(len(left[row]))]
        for row in range(len(left))
    ]


def symmetrize(matrix: Sequence[Sequence[float]]) -> list[list[float]]:
    size = len(matrix)
    result = zeros(size, size)
    for row in range(size):
        for col in range(size):
            result[row][col] = 0.5 * (matrix[row][col] + matrix[col][row])
    return result


def covariance_sandwich(
    jacobian: Sequence[Sequence[float]],
    covariance: Sequence[Sequence[float]],
) -> list[list[float]]:
    return matmul(matmul(jacobian, covariance), transpose(jacobian))


def encoder_pair_covariance_radps(
    vehicle: VehicleConfig,
    linear_speed_sigma_mps: float,
    yaw_rate_sigma_radps: float,
) -> list[list[float]]:
    half_track_m = 0.5 * vehicle.track_width_m
    variance_u_mps2 = linear_speed_sigma_mps * linear_speed_sigma_mps
    variance_yaw_radps2 = yaw_rate_sigma_radps * yaw_rate_sigma_radps
    variance_wheel_linear_mps2 = (
        variance_u_mps2 + half_track_m * half_track_m * variance_yaw_radps2
    )
    covariance_wheel_linear_mps2 = (
        variance_u_mps2 - half_track_m * half_track_m * variance_yaw_radps2
    )
    inv_wheel_radius2 = 1.0 / (vehicle.wheel_radius_m * vehicle.wheel_radius_m)
    diagonal_value = variance_wheel_linear_mps2 * inv_wheel_radius2
    off_diagonal_value = covariance_wheel_linear_mps2 * inv_wheel_radius2
    return [
        [diagonal_value, off_diagonal_value],
        [off_diagonal_value, diagonal_value],
    ]


def finite_difference_jacobian(
    function: Callable[[list[float]], list[float]],
    state: Sequence[float],
    steps: Sequence[float],
) -> list[list[float]]:
    baseline = list(state)
    jacobian = zeros(N, N)
    for col in range(N):
        step = steps[col]
        if step <= 0.0 or not finite(step):
            step = 1.0e-5
        plus = baseline.copy()
        minus = baseline.copy()
        plus[col] += step
        minus[col] -= step
        if col == HEADING:
            plus[col] = normalize_angle(plus[col])
            minus[col] = normalize_angle(minus[col])
        f_plus = function(plus)
        f_minus = function(minus)
        for row in range(N):
            diff = f_plus[row] - f_minus[row]
            if row == HEADING:
                diff = normalize_angle(diff)
            jacobian[row][col] = diff / (2.0 * step)
    return jacobian


def format_number(value: float) -> str:
    if value is None or math.isnan(value):
        return ""
    if math.isinf(value):
        return "inf"
    return f"{value:.12g}"


@dataclass(frozen=True)
class CandidateConfig:
    candidate_id: str
    label: str
    model: str
    parameters: dict[str, float] = field(default_factory=dict)


@dataclass(frozen=True)
class VehicleConfig:
    mass_kg: float = 0.140
    yaw_inertia_kg_m2: float = 0.000220
    track_width_m: float = 0.084635
    wheel_radius_m: float = 0.012610
    contact_longitudinal_offset_m: float = 0.01475
    fan_downforce_at_full_duty_n: float = 0.7
    max_drive_force_per_bank_n: float = 1.05
    forward_accel_limit_mps2: float = 16.5
    reverse_accel_limit_mps2: float = 16.5
    imu_right_offset_m: float = -0.030
    imu_forward_offset_m: float = -0.030
    default_fan_duty_cycle: float = 0.8
    gravity_mps2: float = 9.80665

    @staticmethod
    def from_json(raw: dict[str, Any]) -> "VehicleConfig":
        values = {field_name: parse_float(raw.get(field_name), getattr(VehicleConfig(), field_name))
                  for field_name in VehicleConfig.__dataclass_fields__}
        return VehicleConfig(**values)


@dataclass(frozen=True)
class MeasurementConfig:
    yaw_rate_sigma_radps: float = 0.035
    encoder_wheel_rate_sigma_radps: float = 1.0
    accel_sigma_mps2: float = 1.20
    yaw_rate_gate_nis: float = UNGATED_YAW_RATE_NIS_GATE
    encoder_gate_nis: float = UNGATED_ENCODER_NIS_GATE
    accel_gate_nis: float = 25.0
    finite_difference_step: float = 1.0e-5

    @staticmethod
    def from_json(raw: dict[str, Any]) -> "MeasurementConfig":
        return MeasurementConfig(
            yaw_rate_sigma_radps=parse_float(raw.get("yaw_rate_sigma_radps"), 0.035),
            encoder_wheel_rate_sigma_radps=parse_float(
                raw.get("encoder_wheel_rate_sigma_radps"), 1.0
            ),
            accel_sigma_mps2=parse_float(raw.get("accel_sigma_mps2"), 1.20),
            yaw_rate_gate_nis=UNGATED_YAW_RATE_NIS_GATE,
            encoder_gate_nis=UNGATED_ENCODER_NIS_GATE,
            accel_gate_nis=parse_gate(raw.get("accel_gate_nis"), 25.0),
            finite_difference_step=parse_float(raw.get("finite_difference_step"), 1.0e-5),
        )


@dataclass(frozen=True)
class ProcessConfig:
    state_jacobian_steps: tuple[float, ...] = (
        1.0e-4,
        1.0e-4,
        1.0e-5,
        1.0e-4,
        1.0e-4,
        1.0e-5,
        1.0e-4,
        1.0e-4,
        1.0e-4,
    )
    residual_tau_s: tuple[float, float, float] = (0.075, 0.075, 0.075)
    residual_ss_sigma: tuple[float, float, float] = (0.18, 0.22, 4.0)
    encoder_linear_speed_sigma_mps: float = PRODUCTION_ENCODER_LINEAR_SPEED_SIGMA_MPS
    encoder_yaw_rate_sigma_radps: float = PRODUCTION_ENCODER_YAW_RATE_SIGMA_RADPS
    base_position_sigma_per_tick_m: float = 2.0e-5
    base_heading_sigma_per_tick_rad: float = 2.0e-5
    base_velocity_sigma_per_tick_mps: float = 5.0e-4
    base_yaw_rate_sigma_per_tick_radps: float = 8.0e-4

    @staticmethod
    def from_json(raw: dict[str, Any]) -> "ProcessConfig":
        steps = tuple(parse_float(v, 1.0e-5) for v in raw.get("state_jacobian_steps", ()))
        if len(steps) != N:
            steps = ProcessConfig().state_jacobian_steps
        return ProcessConfig(
            state_jacobian_steps=steps,
            residual_tau_s=parse_triplet(raw.get("residual_tau_s"), (0.075, 0.075, 0.075)),
            residual_ss_sigma=parse_triplet(raw.get("residual_ss_sigma"), (0.18, 0.22, 4.0)),
            encoder_linear_speed_sigma_mps=parse_float(
                raw.get("encoder_linear_speed_sigma_mps"),
                PRODUCTION_ENCODER_LINEAR_SPEED_SIGMA_MPS,
            ),
            encoder_yaw_rate_sigma_radps=parse_float(
                raw.get("encoder_yaw_rate_sigma_radps"),
                PRODUCTION_ENCODER_YAW_RATE_SIGMA_RADPS,
            ),
            base_position_sigma_per_tick_m=parse_float(
                raw.get("base_position_sigma_per_tick_m"), 2.0e-5
            ),
            base_heading_sigma_per_tick_rad=parse_float(
                raw.get("base_heading_sigma_per_tick_rad"), 2.0e-5
            ),
            base_velocity_sigma_per_tick_mps=parse_float(
                raw.get("base_velocity_sigma_per_tick_mps"), 5.0e-4
            ),
            base_yaw_rate_sigma_per_tick_radps=parse_float(
                raw.get("base_yaw_rate_sigma_per_tick_radps"), 8.0e-4
            ),
        )


@dataclass(frozen=True)
class CovarianceConfig:
    initial_state_std: tuple[float, ...] = (
        0.020,
        0.020,
        0.050,
        0.18,
        0.18,
        0.16,
        0.80,
        0.80,
        12.0,
    )
    process: ProcessConfig = field(default_factory=ProcessConfig)
    measurement: MeasurementConfig = field(default_factory=MeasurementConfig)

    @staticmethod
    def from_json(raw: dict[str, Any]) -> "CovarianceConfig":
        initial = tuple(parse_float(v, 0.0) for v in raw.get("initial_state_std", ()))
        if len(initial) != N:
            initial = CovarianceConfig().initial_state_std
        return CovarianceConfig(
            initial_state_std=initial,
            process=ProcessConfig.from_json(dict(raw.get("process", {}))),
            measurement=MeasurementConfig.from_json(dict(raw.get("measurement", {}))),
        )


def parse_gate(value: Any, default: float) -> float:
    if value is None:
        return default
    text = str(value).strip().lower()
    if text in ("", "none", "null", "inf", "infinity"):
        return math.inf
    return parse_float(value, default)


def parse_triplet(value: Any, fallback: tuple[float, float, float]) -> tuple[float, float, float]:
    if not isinstance(value, list) or len(value) != 3:
        return fallback
    return (
        parse_float(value[0], fallback[0]),
        parse_float(value[1], fallback[1]),
        parse_float(value[2], fallback[2]),
    )


@dataclass(frozen=True)
class ReplaySample:
    source_path: Path
    source_row_index: int
    master_time_us: int
    dt_s: float
    left_command: float
    right_command: float
    left_wheel_rate_radps: float
    right_wheel_rate_radps: float
    yaw_rate_radps: float
    accel_forward_mps2: float
    accel_right_mps2: float
    accel_valid: bool
    gyro_valid: bool
    fan_duty_cycle: float
    segment_id: str
    stage: str
    split: str
    run_id: str
    corrupted: bool = False
    previous_left_wheel_rate_radps: float | None = None
    previous_right_wheel_rate_radps: float | None = None
    previous_yaw_rate_radps: float | None = None


@dataclass(frozen=True)
class AccelBiasEstimate:
    forward_mps2: float = 0.0
    right_mps2: float = 0.0
    sample_count: int = 0

    @property
    def valid(self) -> bool:
        return (
            self.sample_count > 0
            and finite(self.forward_mps2)
            and finite(self.right_mps2)
        )


@dataclass(frozen=True)
class SensorBiasEstimate:
    accel: AccelBiasEstimate = field(default_factory=AccelBiasEstimate)
    gyro_bias_radps: float = 0.0
    gyro_sample_count: int = 0
    gyro_source: str = "unavailable"
    bias_segment_count: int = 0

    def row(self, log_path: Path, log_id: str) -> dict[str, str]:
        return {
            "log_id": log_id,
            "log_path": str(log_path),
            "bias_segment_count": str(self.bias_segment_count),
            "accel_bias_forward_mps2": format_number(self.accel.forward_mps2),
            "accel_bias_right_mps2": format_number(self.accel.right_mps2),
            "accel_bias_sample_count": str(self.accel.sample_count),
            "gyro_bias_radps": format_number(self.gyro_bias_radps),
            "gyro_bias_sample_count": str(self.gyro_sample_count),
            "gyro_bias_source": self.gyro_source,
        }


@dataclass
class AccelBiasAccumulator:
    forward_sum: float = 0.0
    right_sum: float = 0.0
    sample_count: int = 0

    def add(self, sample: ReplaySample) -> None:
        if not sample_qualifies_for_accel_bias(sample):
            return
        self.forward_sum += sample.accel_forward_mps2
        self.right_sum += sample.accel_right_mps2
        self.sample_count += 1

    def estimate(self) -> AccelBiasEstimate:
        if self.sample_count <= 0:
            return AccelBiasEstimate()
        return AccelBiasEstimate(
            forward_mps2=self.forward_sum / self.sample_count,
            right_mps2=self.right_sum / self.sample_count,
            sample_count=self.sample_count,
        )


@dataclass
class GyroBiasAccumulator:
    bias_sum: float = 0.0
    sample_count: int = 0
    source_counts: dict[str, int] = field(default_factory=dict)

    def add(self, value: float, source: str) -> None:
        if not finite(value):
            return
        self.bias_sum += value
        self.sample_count += 1
        self.source_counts[source] = self.source_counts.get(source, 0) + 1

    def estimate(self) -> tuple[float, int, str]:
        if self.sample_count <= 0:
            return 0.0, 0, "unavailable"
        source = ",".join(
            f"{name}:{count}"
            for name, count in sorted(self.source_counts.items())
        )
        return self.bias_sum / self.sample_count, self.sample_count, source


@dataclass(frozen=True)
class ReplayColumnBinding:
    master_time_us: tuple[int, ...]
    dt_us: tuple[int, ...]
    left_rate: tuple[int, ...]
    right_rate: tuple[int, ...]
    left_velocity: tuple[int, ...]
    right_velocity: tuple[int, ...]
    gyro: tuple[int, ...]
    raw_gyro: tuple[int, ...]
    gyro_bias: tuple[int, ...]
    measured_angular_speed: tuple[int, ...]
    accel_forward: tuple[int, ...]
    accel_right: tuple[int, ...]
    accel_valid: tuple[int, ...]
    stage: tuple[int, ...]
    run_id: tuple[int, ...]
    left_command: tuple[int, ...]
    right_command: tuple[int, ...]
    fan_duty_cycle: tuple[int, ...]

    @staticmethod
    def from_fieldnames(fieldnames: Sequence[str]) -> "ReplayColumnBinding":
        lookup = {name.lower(): index for index, name in enumerate(fieldnames)}

        def indices(names: Sequence[str]) -> tuple[int, ...]:
            return tuple(
                lookup[name.lower()]
                for name in names
                if name.lower() in lookup
            )

        return ReplayColumnBinding(
            master_time_us=indices(("master_time_us", "timestamp_us", "time_us")),
            dt_us=indices(("dt_us",)),
            left_rate=indices(
                (
                    "left_encoder_omega_radps",
                    "left_encoder_wheel_speed_radps",
                    "left_wheel_rate_radps",
                )
            ),
            right_rate=indices(
                (
                    "right_encoder_omega_radps",
                    "right_encoder_wheel_speed_radps",
                    "right_wheel_rate_radps",
                )
            ),
            left_velocity=indices(("left_encoder_velocity_mps", "left_wheel_velocity_mps")),
            right_velocity=indices(("right_encoder_velocity_mps", "right_wheel_velocity_mps")),
            gyro=indices(("gyro_radps", "yaw_rate_radps")),
            raw_gyro=indices(("gyro_raw_radps", "imu_gyro_z")),
            gyro_bias=indices(("gyro_bias_radps",)),
            measured_angular_speed=indices(("measured_angular_speed_radps",)),
            accel_forward=indices(
                (
                    "accel_body_forward_mps2",
                    "accel_body_y_mps2",
                    "forward_accel_mps2",
                )
            ),
            accel_right=indices(
                (
                    "accel_body_right_mps2",
                    "accel_body_x_mps2",
                    "right_accel_mps2",
                )
            ),
            accel_valid=indices(("accel_valid", "imu_accel_valid")),
            stage=indices(("stage", "section_name", "phase_name")),
            run_id=indices(("run_id", "log_id")),
            left_command=indices(("left_drive_command", "left_command")),
            right_command=indices(("right_drive_command", "right_command")),
            fan_duty_cycle=indices(("fan_duty_cycle", "fan_command")),
        )


@dataclass(frozen=True)
class SegmentSpec:
    log_path: Path
    segment_id: str
    stage: str = ""
    family: str = ""
    split: str = ""
    run_id: str = ""
    start_row_index: int = 0
    end_row_index: int = -1
    corrupted: bool = False
    parameter_fields: dict[str, Any] = field(default_factory=dict)
    observed_command: dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class Contact:
    name: str
    side: str
    right_m: float
    forward_m: float


@dataclass(frozen=True)
class ContactDiagnostic:
    name: str
    rel_forward_mps: float
    rel_right_mps: float
    normal_load_n: float
    force_forward_n: float
    force_right_n: float
    utilization: float
    saturation: float


@dataclass(frozen=True)
class PlantResult:
    forward_accel_mps2: float
    right_accel_mps2: float
    yaw_accel_radps2: float
    imu_forward_accel_mps2: float
    imu_right_accel_mps2: float
    contacts: tuple[ContactDiagnostic, ...]
    max_contact_relative_speed_mps: float
    load_weighted_contact_relative_speed_mps: float
    load_weighted_lateral_relative_speed_mps: float
    load_weighted_yaw_contact_speed_mps: float
    max_contact_utilization: float
    max_contact_saturation: float
    total_normal_load_n: float
    ground_use: float


@dataclass(frozen=True)
class UpdateResult:
    log_parameter: str
    measurement_dimension: int
    nis: float
    accepted: bool
    gate_threshold: float
    innovation: float
    innovation_variance: float
    measurement: float
    prediction: float
    metric_kind: str = "anis"


@dataclass
class UkfValidationStats:
    candidate_id: str
    segment_ids: set[str] = field(default_factory=set)
    sample_count: int = 0
    sigma_point_count: int = 0
    finite_failures: int = 0
    continuity_failures: int = 0
    covariance_failures: int = 0
    innovation_failures: int = 0
    zero_crossing_events: int = 0
    zero_crossing_failures: int = 0
    max_abs_state: float = 0.0
    max_abs_prediction: float = 0.0
    max_state_sigma_span: float = 0.0
    max_measurement_sigma_span: float = 0.0
    max_covariance_trace: float = 0.0
    max_innovation_nis: float = 0.0

    def issue_count(self) -> int:
        return (
            self.finite_failures
            + self.continuity_failures
            + self.covariance_failures
            + self.innovation_failures
            + self.zero_crossing_failures
        )

    def passed(self) -> bool:
        return self.issue_count() == 0


class CandidatePlant:
    def __init__(self, vehicle: VehicleConfig, candidate: CandidateConfig):
        self.vehicle = vehicle
        self.candidate = candidate
        self.model = validate_candidate_model(candidate.model)
        self.params = model_parameters(self.model, candidate.parameters)
        half_track = 0.5 * abs(vehicle.track_width_m)
        f = abs(vehicle.contact_longitudinal_offset_m)
        self.contacts = (
            Contact("front_left", "left", -half_track, f),
            Contact("front_right", "right", half_track, f),
            Contact("rear_left", "left", -half_track, -f),
            Contact("rear_right", "right", half_track, -f),
        )

    def propagate(self, state: Sequence[float], sample: ReplaySample, dt_s: float) -> list[float]:
        dt_s = max(0.0, min(finite_or(dt_s, 0.0), 0.050))
        if dt_s <= 0.0:
            return list(state)

        k1 = self.derivative(state, sample)
        midpoint = [state[index] + 0.5 * dt_s * k1[index] for index in range(N)]
        midpoint[HEADING] = normalize_angle(midpoint[HEADING])
        k2 = self.derivative(midpoint, sample)
        result = [state[index] + dt_s * k2[index] for index in range(N)]
        result[HEADING] = normalize_angle(result[HEADING])
        return result

    def derivative(self, state: Sequence[float], sample: ReplaySample) -> list[float]:
        plant = self.plant_result(state, sample)
        theta = state[HEADING]
        vf = state[VF]
        vr = state[VR]
        yaw = state[YAW_RATE]
        return [
            vf * math.sin(theta) + vr * math.cos(theta),
            vf * math.cos(theta) - vr * math.sin(theta),
            yaw,
            plant.forward_accel_mps2 + yaw * vr,
            plant.right_accel_mps2 - yaw * vf,
            plant.yaw_accel_radps2,
            -state[DAF] / 0.075,
            -state[DAR] / 0.075,
            -state[DYAW] / 0.075,
        ]

    def plant_result(self, state: Sequence[float], sample: ReplaySample) -> PlantResult:
        model = self.model
        params = self.params
        if model == "current_holdover_approximation":
            return self.current_holdover_result(state, sample)
        loads = self.normal_loads(state, sample, model, params)
        force_forward_sum = 0.0
        force_right_sum = 0.0
        yaw_moment_nm = 0.0
        contacts: list[ContactDiagnostic] = []
        bank_loads = {
            "left": sum(load for contact, load in zip(self.contacts, loads) if contact.side == "left"),
            "right": sum(load for contact, load in zip(self.contacts, loads) if contact.side == "right"),
        }

        for contact, normal_load_n in zip(self.contacts, loads):
            rel_f, rel_r = self.relative_velocity(state, sample, contact)
            force_f, force_r, utilization, saturation = self.contact_force(
                contact,
                normal_load_n,
                bank_loads[contact.side],
                rel_f,
                rel_r,
                sample,
                model,
                params,
            )
            force_forward_sum += force_f
            force_right_sum += force_r
            yaw_moment_nm += (contact.forward_m * force_r) - (contact.right_m * force_f)
            contacts.append(
                ContactDiagnostic(
                    name=contact.name,
                    rel_forward_mps=rel_f,
                    rel_right_mps=rel_r,
                    normal_load_n=normal_load_n,
                    force_forward_n=force_f,
                    force_right_n=force_r,
                    utilization=utilization,
                    saturation=saturation,
                )
            )

        yaw_loss, yaw_loss_direction = self.yaw_loss_damping(state, contacts, params, model)
        yaw_damping_moment_nm = -yaw_loss_direction * yaw_loss
        ground_use = max((contact.saturation for contact in contacts), default=0.0)
        return self.make_plant_result(
            state,
            force_forward_sum,
            force_right_sum,
            yaw_moment_nm + yaw_damping_moment_nm,
            contacts,
            ground_use,
        )

    def current_holdover_result(self, state: Sequence[float], sample: ReplaySample) -> PlantResult:
        params = self.params
        loads = self.normal_loads(state, sample, self.model, params)
        force_forward_sum = 0.0
        force_right_sum = 0.0
        yaw_moment_nm = 0.0
        contacts: list[ContactDiagnostic] = []
        bank_loads = {
            "left": sum(load for contact, load in zip(self.contacts, loads) if contact.side == "left"),
            "right": sum(load for contact, load in zip(self.contacts, loads) if contact.side == "right"),
        }

        for contact, normal_load_n in zip(self.contacts, loads):
            rel_f, rel_r = self.relative_velocity(state, sample, contact)
            force_f, force_r, utilization, saturation = self.current_holdover_contact_force(
                contact,
                normal_load_n,
                bank_loads[contact.side],
                rel_f,
                rel_r,
                sample,
                params,
            )
            force_forward_sum += force_f
            force_right_sum += force_r
            yaw_moment_nm += (contact.forward_m * force_r) - (contact.right_m * force_f)
            contacts.append(
                ContactDiagnostic(
                    name=contact.name,
                    rel_forward_mps=rel_f,
                    rel_right_mps=rel_r,
                    normal_load_n=normal_load_n,
                    force_forward_n=force_f,
                    force_right_n=force_r,
                    utilization=utilization,
                    saturation=saturation,
                )
            )

        raw_forward_accel = force_forward_sum / max(self.vehicle.mass_kg, 1.0e-9)
        clipped_forward_accel, ground_use = self.apply_forward_accel_envelope(raw_forward_accel)
        return self.make_plant_result(
            state,
            clipped_forward_accel * self.vehicle.mass_kg,
            force_right_sum,
            yaw_moment_nm,
            contacts,
            ground_use,
        )

    def make_plant_result(
        self,
        state: Sequence[float],
        force_forward_sum: float,
        force_right_sum: float,
        yaw_moment_nm: float,
        contacts: Sequence[ContactDiagnostic],
        ground_use: float,
    ) -> PlantResult:
        vehicle = self.vehicle
        forward_accel = force_forward_sum / max(vehicle.mass_kg, 1.0e-9)
        right_accel = force_right_sum / max(vehicle.mass_kg, 1.0e-9)
        yaw_accel = yaw_moment_nm / max(vehicle.yaw_inertia_kg_m2, 1.0e-12)

        forward_accel += state[DAF]
        right_accel += state[DAR]
        yaw_accel += state[DYAW]

        imu_forward = (
            forward_accel
            - yaw_accel * vehicle.imu_right_offset_m
            - (state[YAW_RATE] ** 2) * vehicle.imu_forward_offset_m
        )
        imu_right = (
            right_accel
            + yaw_accel * vehicle.imu_forward_offset_m
            - (state[YAW_RATE] ** 2) * vehicle.imu_right_offset_m
        )
        total_load = sum(max(0.0, contact.normal_load_n) for contact in contacts)
        weighted_rel2 = 0.0
        weighted_lat2 = 0.0
        weighted_yaw2 = 0.0
        max_rel = 0.0
        for contact, geometry in zip(contacts, self.contacts):
            rel2 = contact.rel_forward_mps**2 + contact.rel_right_mps**2
            max_rel = max(max_rel, math.sqrt(rel2))
            weighted_rel2 += max(0.0, contact.normal_load_n) * rel2
            weighted_lat2 += max(0.0, contact.normal_load_n) * contact.rel_right_mps**2
            yaw_contact_speed = abs(state[YAW_RATE]) * math.hypot(geometry.right_m, geometry.forward_m)
            weighted_yaw2 += max(0.0, contact.normal_load_n) * yaw_contact_speed**2
        denom = max(total_load, 1.0e-9)
        return PlantResult(
            forward_accel_mps2=forward_accel,
            right_accel_mps2=right_accel,
            yaw_accel_radps2=yaw_accel,
            imu_forward_accel_mps2=imu_forward,
            imu_right_accel_mps2=imu_right,
            contacts=tuple(contacts),
            max_contact_relative_speed_mps=max_rel,
            load_weighted_contact_relative_speed_mps=math.sqrt(weighted_rel2 / denom),
            load_weighted_lateral_relative_speed_mps=math.sqrt(weighted_lat2 / denom),
            load_weighted_yaw_contact_speed_mps=math.sqrt(weighted_yaw2 / denom),
            max_contact_utilization=max((c.utilization for c in contacts), default=0.0),
            max_contact_saturation=max((c.saturation for c in contacts), default=0.0),
            total_normal_load_n=total_load,
            ground_use=ground_use,
        )

    def normal_loads(
        self,
        state: Sequence[float],
        sample: ReplaySample,
        model: str,
        params: dict[str, float],
    ) -> list[float]:
        vehicle = self.vehicle
        static_load = vehicle.mass_kg * vehicle.gravity_mps2
        fan_load = max(0.0, sample.fan_duty_cycle) * vehicle.fan_downforce_at_full_duty_n
        base = 0.25 * (static_load + fan_load)
        loads = [base for _ in self.contacts]
        if model != "load_sensitive_anisotropic":
            return loads

        long_transfer_gain = max(0.0, params.get("longitudinal_load_transfer_gain", 0.0))
        lateral_transfer_gain = max(0.0, params.get("lateral_load_transfer_gain", 0.0))
        command_accel_proxy = (
            (sample.left_command + sample.right_command)
            * vehicle.max_drive_force_per_bank_n
            / max(vehicle.mass_kg, 1.0e-9)
        )
        lateral_accel_proxy = state[YAW_RATE] * max(abs(state[VF]), 0.05)
        max_f = max(abs(c.forward_m) for c in self.contacts) or 1.0
        max_r = max(abs(c.right_m) for c in self.contacts) or 1.0
        total_load = static_load + fan_load
        for index, contact in enumerate(self.contacts):
            long_transfer = (
                -0.25
                * long_transfer_gain
                * total_load
                * (command_accel_proxy / vehicle.gravity_mps2)
                * (contact.forward_m / max_f)
            )
            lateral_transfer = (
                -0.25
                * lateral_transfer_gain
                * total_load
                * (lateral_accel_proxy / vehicle.gravity_mps2)
                * (contact.right_m / max_r)
            )
            loads[index] = max(0.0, loads[index] + long_transfer + lateral_transfer)
        return loads

    def relative_velocity(
        self,
        state: Sequence[float],
        sample: ReplaySample,
        contact: Contact,
    ) -> tuple[float, float]:
        wheel_rate = (
            sample.left_wheel_rate_radps if contact.side == "left" else sample.right_wheel_rate_radps
        )
        surface_forward_mps = self.vehicle.wheel_radius_m * wheel_rate
        body_forward_mps = state[VF] - state[YAW_RATE] * contact.right_m
        body_right_mps = state[VR] + state[YAW_RATE] * contact.forward_m
        return surface_forward_mps - body_forward_mps, -body_right_mps

    def contact_force(
        self,
        contact: Contact,
        normal_load_n: float,
        bank_load_n: float,
        rel_forward_mps: float,
        rel_right_mps: float,
        sample: ReplaySample,
        model: str,
        params: dict[str, float],
    ) -> tuple[float, float, float, float]:
        rel_mag = math.hypot(rel_forward_mps, rel_right_mps)
        low_speed_blend = max(params.get("low_speed_blend_mps", 0.06), 1.0e-6)
        blend = rel_mag / math.sqrt(rel_mag**2 + low_speed_blend**2)
        long_gain = params.get("longitudinal_slip_gain_n_per_mps", 18.0)
        lateral_gain = params.get("lateral_slip_gain_n_per_mps", 18.0)
        exponent = max(params.get("combined_slip_envelope_exponent", 2.0), 1.0)
        peak_mu = max(0.0, params.get("peak_friction_coefficient_at_80pct_fan", 1.65))
        lateral_mu = peak_mu
        longitudinal_mu = peak_mu

        if model == "stribeck_algebraic":
            ratio = max(0.05, min(params.get("dynamic_to_static_grip_ratio", 0.82), 1.0))
            stribeck_v = max(params.get("stribeck_velocity_mps", 0.16), 1.0e-6)
            slide_mu = peak_mu * ratio
            mu = slide_mu + (peak_mu - slide_mu) * math.exp(-((rel_mag / stribeck_v) ** 2))
            longitudinal_mu = lateral_mu = mu
            damping = params.get("viscous_slip_damping_n_per_mps", 0.0)
            long_gain += damping
            lateral_gain += damping
        elif model == "load_sensitive_anisotropic":
            mean_load = 0.25 * (
                self.vehicle.mass_kg * self.vehicle.gravity_mps2
                + max(0.0, sample.fan_duty_cycle) * self.vehicle.fan_downforce_at_full_duty_n
            )
            load_ratio = (normal_load_n / max(mean_load, 1.0e-6)) - 1.0
            longitudinal_sensitivity = max(0.0, params.get("longitudinal_load_sensitivity", 0.0))
            lateral_sensitivity = max(0.0, params.get("lateral_load_sensitivity", 0.0))
            longitudinal_mu = peak_mu * (1.0 - longitudinal_sensitivity * load_ratio)
            lateral_mu = peak_mu * (1.0 - lateral_sensitivity * load_ratio)
            longitudinal_mu = max(0.10, longitudinal_mu)
            lateral_mu = max(0.10, lateral_mu)
        elif model == "in_shear":
            side = 1.0 if contact.right_m >= 0.0 else -1.0
            inward_sign = -side
            inward_velocity = inward_sign * rel_right_mps
            blend_speed = max(params.get("inward_shear_blend_speed_mps", 0.04), 1.0e-6)
            inward_blend = 0.5 * (1.0 + math.tanh(inward_velocity / blend_speed))
            stiffness_gain = params.get("inward_lateral_stiffness_gain", 0.0)
            grip_gain = params.get("inward_lateral_grip_gain", 0.0)
            lateral_gain *= max(0.05, 1.0 + stiffness_gain * inward_blend)
            lateral_mu = max(0.10, lateral_mu * max(0.05, 1.0 + grip_gain * inward_blend))

        command = sample.left_command if contact.side == "left" else sample.right_command
        drive_force_bank = command * self.vehicle.max_drive_force_per_bank_n
        bank_weight = max(0.0, normal_load_n) / max(bank_load_n, 1.0e-9)
        drive_share = bank_weight * drive_force_bank

        requested_forward = drive_share + blend * long_gain * rel_forward_mps
        requested_right = blend * lateral_gain * rel_right_mps
        if model == "skew_shear":
            shear_speed = max(params.get("shear_activation_speed_mps", 0.06), 1.0e-6)
            drive_blend = max(params.get("shear_drive_force_blend_n", 0.04), 1.0e-6)
            contact_gate = rel_mag / math.sqrt(rel_mag**2 + shear_speed**2)
            drive_gate = abs(drive_share) / math.sqrt(drive_share**2 + drive_blend**2)
            drive_direction = smooth_sign(drive_share, drive_blend)
            coupling = params.get("shear_coupling_gain", 0.0) * contact_gate * drive_gate * drive_direction
            requested_forward += blend * coupling * lateral_gain * rel_right_mps
            requested_right += blend * coupling * long_gain * rel_forward_mps
        elif model == "shear_rate":
            rate_forward, rate_right = self.replayable_contact_velocity_rates(contact, sample)
            rate_mag = math.hypot(rate_forward, rate_right)
            rate_blend = max(params.get("shear_rate_activation_mps2", 35.0), 1.0e-6)
            rate_gate = rate_mag / math.sqrt(rate_mag**2 + rate_blend**2)
            breakaway_speed = max(params.get("shear_rate_breakaway_speed_mps", 0.10), 1.0e-6)
            breakaway_gate = breakaway_speed / math.sqrt(rel_mag**2 + breakaway_speed**2)
            peak_force = max(0.0, params.get("shear_rate_peak_force_n", 0.0))
            transient_force = peak_force * rate_gate * breakaway_gate
            requested_forward += transient_force * smooth_sign(rate_forward, rate_blend)
            requested_right += transient_force * smooth_sign(rate_right, rate_blend)
        forward_limit = max(longitudinal_mu * normal_load_n, 1.0e-6)
        right_limit = max(lateral_mu * normal_load_n, 1.0e-6)
        utilization = (
            (abs(requested_forward) / forward_limit) ** exponent
            + (abs(requested_right) / right_limit) ** exponent
        ) ** (1.0 / exponent)
        scale = smooth_scale_to_unit(utilization)
        return requested_forward * scale, requested_right * scale, utilization, 1.0 - scale

    def replayable_contact_velocity_rates(
        self,
        contact: Contact,
        sample: ReplaySample,
    ) -> tuple[float, float]:
        dt_s = max(sample.dt_s, 1.0e-6)
        if contact.side == "left":
            previous_wheel_rate = sample.previous_left_wheel_rate_radps
            current_wheel_rate = sample.left_wheel_rate_radps
        else:
            previous_wheel_rate = sample.previous_right_wheel_rate_radps
            current_wheel_rate = sample.right_wheel_rate_radps
        if (
            previous_wheel_rate is not None
            and finite(previous_wheel_rate)
            and finite(current_wheel_rate)
        ):
            forward_rate = self.vehicle.wheel_radius_m * (current_wheel_rate - previous_wheel_rate) / dt_s
        else:
            forward_rate = 0.0

        if (
            sample.previous_yaw_rate_radps is not None
            and finite(sample.previous_yaw_rate_radps)
            and finite(sample.yaw_rate_radps)
        ):
            right_rate = (
                -(sample.yaw_rate_radps - sample.previous_yaw_rate_radps)
                * contact.forward_m
                / dt_s
            )
        else:
            right_rate = 0.0
        return forward_rate, right_rate

    def current_holdover_contact_force(
        self,
        contact: Contact,
        normal_load_n: float,
        bank_load_n: float,
        _rel_forward_mps: float,
        rel_right_mps: float,
        sample: ReplaySample,
        params: dict[str, float],
    ) -> tuple[float, float, float, float]:
        wheel_rate = (
            sample.left_wheel_rate_radps if contact.side == "left" else sample.right_wheel_rate_radps
        )
        surface_forward_mps = self.vehicle.wheel_radius_m * wheel_rate
        rel_mag = math.hypot(surface_forward_mps, rel_right_mps)
        low_speed_blend = max(params.get("low_speed_blend_mps", 0.06), 1.0e-6)
        blend = rel_mag / math.sqrt(rel_mag**2 + low_speed_blend**2)
        long_gain = params.get("longitudinal_slip_gain_n_per_mps", 16.0)
        lateral_gain = params.get("lateral_slip_gain_n_per_mps", 16.0)
        peak_mu = max(0.0, params.get("peak_friction_coefficient_at_80pct_fan", 1.65))
        command = sample.left_command if contact.side == "left" else sample.right_command
        drive_force_bank = command * self.vehicle.max_drive_force_per_bank_n
        bank_weight = max(0.0, normal_load_n) / max(bank_load_n, 1.0e-9)
        drive_share = bank_weight * drive_force_bank

        requested_forward = drive_share + blend * long_gain * surface_forward_mps
        requested_right = blend * lateral_gain * rel_right_mps
        limit = max(peak_mu * normal_load_n, 1.0e-6)
        force_forward = clamp(requested_forward, -limit, limit)
        force_right = clamp(requested_right, -limit, limit)
        utilization = max(abs(requested_forward), abs(requested_right)) / limit
        saturation = max(0.0, 1.0 - min(1.0, limit / max(abs(requested_forward), abs(requested_right), limit)))
        return force_forward, force_right, utilization, saturation

    def apply_forward_accel_envelope(self, raw_forward_accel: float) -> tuple[float, float]:
        upper = max(self.vehicle.forward_accel_limit_mps2, 0.1)
        lower = -max(self.vehicle.reverse_accel_limit_mps2, 0.1)
        clipped = max(lower, min(upper, raw_forward_accel))
        if abs(raw_forward_accel) <= 1.0e-9:
            return clipped, 0.0
        return clipped, abs(raw_forward_accel - clipped) / (abs(raw_forward_accel) + 1.0e-9)

    def yaw_loss_damping(
        self,
        state: Sequence[float],
        contacts: Sequence[ContactDiagnostic],
        params: dict[str, float],
        model: str,
    ) -> tuple[float, float]:
        if model != "load_sensitive_anisotropic":
            return 0.0, 0.0
        total_load = sum(max(0.0, c.normal_load_n) for c in contacts)
        gain = max(0.0, params.get("yaw_coupling_gain", 0.0))
        if total_load <= 1.0e-9 or gain <= 0.0:
            return 0.0, 0.0
        yaw_rate = state[YAW_RATE]
        if abs(yaw_rate) <= 1.0e-12:
            return 0.0, 0.0
        weighted_yaw_contact_speed = 0.0
        weighted_lever = 0.0
        for contact, geometry in zip(contacts, self.contacts):
            load = max(0.0, contact.normal_load_n)
            lever = math.hypot(geometry.right_m, geometry.forward_m)
            if load <= 0.0 or lever <= 1.0e-9:
                continue
            weighted_yaw_contact_speed += load * abs(yaw_rate) * lever
            weighted_lever += load * lever
        if weighted_lever <= 0.0:
            return 0.0, 0.0
        mean_abs_speed = weighted_yaw_contact_speed / total_load
        mean_lever = weighted_lever / total_load
        speed_blend = max(params.get("yaw_damping_speed_blend_mps", 0.02), 1.0e-6)
        direction = smooth_sign(yaw_rate * mean_lever, speed_blend)
        loss_magnitude = max(0.0, gain * total_load * mean_abs_speed * mean_lever)
        return loss_magnitude, direction


class EkfReplay:
    def __init__(self, plant: CandidatePlant, covariance: CovarianceConfig):
        self.plant = plant
        self.covariance = covariance
        self.state = [0.0 for _ in range(N)]
        self.P = diagonal([std * std for std in covariance.initial_state_std])
        self.stationary_zlock_time_s = 0.0
        self.stationary_zlock_anchor: tuple[float, float] | None = None
        self.stationary_zlock_applied_count = 0
        self.stationary_zlock_released_count = 0

    def predict(self, sample: ReplaySample) -> PlantResult:
        previous_state = self.state.copy()
        dt_s = sample.dt_s
        self.state = self.plant.propagate(previous_state, sample, dt_s)
        jacobian = finite_difference_jacobian(
            lambda candidate_state: self.plant.propagate(candidate_state, sample, dt_s),
            previous_state,
            self.covariance.process.state_jacobian_steps,
        )
        propagated = covariance_sandwich(jacobian, self.P)
        self.P = symmetrize(add_matrix(propagated, self.process_noise(previous_state, sample, dt_s)))
        return self.plant.plant_result(self.state, sample)

    def process_noise(
        self,
        state: Sequence[float],
        sample: ReplaySample,
        dt_s: float,
    ) -> list[list[float]]:
        process = self.covariance.process
        q = diagonal(
            [
                process.base_position_sigma_per_tick_m**2,
                process.base_position_sigma_per_tick_m**2,
                process.base_heading_sigma_per_tick_rad**2,
                process.base_velocity_sigma_per_tick_mps**2,
                process.base_velocity_sigma_per_tick_mps**2,
                process.base_yaw_rate_sigma_per_tick_radps**2,
                0.0,
                0.0,
                0.0,
            ]
        )
        theta = state[HEADING]
        dt = max(0.0, finite_or(dt_s, 0.0))
        residual_columns = (
            [
                0.5 * dt * dt * math.sin(theta),
                0.5 * dt * dt * math.cos(theta),
                0.0,
                dt,
                0.0,
                0.0,
                1.0,
                0.0,
                0.0,
            ],
            [
                0.5 * dt * dt * math.cos(theta),
                -0.5 * dt * dt * math.sin(theta),
                0.0,
                0.0,
                dt,
                0.0,
                0.0,
                1.0,
                0.0,
            ],
            [
                0.0,
                0.0,
                0.5 * dt * dt,
                0.0,
                0.0,
                dt,
                0.0,
                0.0,
                1.0,
            ],
        )
        for column, tau, sigma_ss in zip(
            residual_columns,
            process.residual_tau_s,
            process.residual_ss_sigma,
        ):
            phi = math.exp(-dt / max(tau, 1.0e-6))
            variance = sigma_ss * sigma_ss * max(0.0, 1.0 - phi * phi)
            for row in range(N):
                for col in range(N):
                    q[row][col] += column[row] * variance * column[col]

        if (
            (
                process.encoder_linear_speed_sigma_mps > 0.0
                or process.encoder_yaw_rate_sigma_radps > 0.0
            )
            and dt > 0.0
        ):
            q = add_matrix(q, self.encoder_input_noise(state, sample, dt))
        return q

    def encoder_input_noise(
        self,
        state: Sequence[float],
        sample: ReplaySample,
        dt_s: float,
    ) -> list[list[float]]:
        step = 1.0e-3
        columns: list[list[float]] = []
        for side in ("left", "right"):
            plus = replace_sample_wheel_rate(sample, side, step)
            minus = replace_sample_wheel_rate(sample, side, -step)
            x_plus = self.plant.propagate(state, plus, dt_s)
            x_minus = self.plant.propagate(state, minus, dt_s)
            columns.append([(x_plus[index] - x_minus[index]) / (2.0 * step) for index in range(N)])
        jacobian = [[columns[0][row], columns[1][row]] for row in range(N)]
        wheel_rate_covariance = encoder_pair_covariance_radps(
            self.plant.vehicle,
            self.covariance.process.encoder_linear_speed_sigma_mps,
            self.covariance.process.encoder_yaw_rate_sigma_radps,
        )
        return covariance_sandwich(jacobian, wheel_rate_covariance)

    def update_yaw_rate(self, sample: ReplaySample) -> UpdateResult | None:
        if not sample.gyro_valid or not finite(sample.yaw_rate_radps):
            return None
        return self.scalar_update(
            "yaw_rate_nis",
            sample.yaw_rate_radps,
            lambda state: state[YAW_RATE],
            self.covariance.measurement.yaw_rate_sigma_radps**2,
            self.covariance.measurement.yaw_rate_gate_nis,
            h_steps=self.covariance.process.state_jacobian_steps,
        )

    def update_accel_forward(self, sample: ReplaySample) -> UpdateResult | None:
        if not sample.accel_valid or not finite(sample.accel_forward_mps2):
            return None
        return self.scalar_update(
            "forward_accel_nis",
            sample.accel_forward_mps2,
            lambda state: self.plant.plant_result(state, sample).imu_forward_accel_mps2,
            self.covariance.measurement.accel_sigma_mps2**2,
            self.covariance.measurement.accel_gate_nis,
            h_steps=self.covariance.process.state_jacobian_steps,
        )

    def update_accel_right(self, sample: ReplaySample) -> UpdateResult | None:
        if not sample.accel_valid or not finite(sample.accel_right_mps2):
            return None
        return self.scalar_update(
            "right_accel_nis",
            sample.accel_right_mps2,
            lambda state: self.plant.plant_result(state, sample).imu_right_accel_mps2,
            self.covariance.measurement.accel_sigma_mps2**2,
            self.covariance.measurement.accel_gate_nis,
            h_steps=self.covariance.process.state_jacobian_steps,
        )

    def apply_stationary_zlock(self, sample: ReplaySample) -> bool:
        if not self.stationary_zlock_sample_eligible(sample):
            if self.stationary_zlock_time_s > 0.0 or self.stationary_zlock_anchor is not None:
                self.stationary_zlock_released_count += 1
            self.stationary_zlock_time_s = 0.0
            self.stationary_zlock_anchor = None
            return False

        self.stationary_zlock_time_s += max(0.0, sample.dt_s)
        window_s = max(0.0, self.plant.params.get("stationary_zlock_window_s", 0.20))
        if self.stationary_zlock_time_s < window_s:
            return False

        if self.stationary_zlock_anchor is None:
            self.stationary_zlock_anchor = (self.state[PX], self.state[PY])

        px_anchor, py_anchor = self.stationary_zlock_anchor
        position_variance = max(
            self.plant.params.get("stationary_zlock_position_sigma_m", 0.00025) ** 2,
            1.0e-12,
        )
        velocity_variance = max(
            self.plant.params.get("stationary_zlock_velocity_sigma_mps", 0.0010) ** 2,
            1.0e-12,
        )
        yaw_rate_variance = max(
            self.plant.params.get("stationary_zlock_yaw_rate_sigma_radps", 0.0030) ** 2,
            1.0e-12,
        )
        steps = self.covariance.process.state_jacobian_steps
        self.scalar_update(
            "stationary_zlock_internal_px",
            px_anchor,
            lambda state: state[PX],
            position_variance,
            math.inf,
            h_steps=steps,
        )
        self.scalar_update(
            "stationary_zlock_internal_py",
            py_anchor,
            lambda state: state[PY],
            position_variance,
            math.inf,
            h_steps=steps,
        )
        self.scalar_update(
            "stationary_zlock_internal_vf",
            0.0,
            lambda state: state[VF],
            velocity_variance,
            math.inf,
            h_steps=steps,
        )
        self.scalar_update(
            "stationary_zlock_internal_vr",
            0.0,
            lambda state: state[VR],
            velocity_variance,
            math.inf,
            h_steps=steps,
        )
        self.scalar_update(
            "stationary_zlock_internal_yaw_rate",
            0.0,
            lambda state: state[YAW_RATE],
            yaw_rate_variance,
            math.inf,
            h_steps=steps,
        )
        self.stationary_zlock_applied_count += 1
        return True

    def stationary_zlock_sample_eligible(self, sample: ReplaySample) -> bool:
        params = self.plant.params
        if params.get("stationary_zlock_enabled", 0.0) <= 0.0:
            return False
        command_epsilon = max(0.0, params.get("stationary_zlock_command_epsilon", 0.015))
        wheel_rate_epsilon = max(
            0.0,
            params.get("stationary_zlock_wheel_rate_epsilon_radps", 0.02),
        )
        yaw_rate_epsilon = max(
            0.0,
            params.get("stationary_zlock_yaw_rate_epsilon_radps", 0.06),
        )
        if max(abs(sample.left_command), abs(sample.right_command)) > command_epsilon:
            return False
        wheel_rates = (
            sample.left_wheel_rate_radps,
            sample.right_wheel_rate_radps,
            sample.previous_left_wheel_rate_radps,
            sample.previous_right_wheel_rate_radps,
        )
        if any(value is None or not finite(value) or abs(value) > wheel_rate_epsilon for value in wheel_rates):
            return False
        if not sample.gyro_valid or not finite(sample.yaw_rate_radps):
            return False
        if abs(sample.yaw_rate_radps) > yaw_rate_epsilon:
            return False
        return True

    def scalar_update(
        self,
        log_parameter: str,
        measurement: float,
        h: Callable[[list[float]], float],
        measurement_variance: float,
        gate_threshold: float,
        h_steps: Sequence[float],
    ) -> UpdateResult:
        gate_threshold = effective_gate_threshold(log_parameter, gate_threshold)
        prior = self.state.copy()
        prediction = h(prior)
        innovation = measurement - prediction
        H = self.measurement_jacobian(h, prior, h_steps)
        PHt = matvec(self.P, H)
        innovation_variance = sum(H[index] * PHt[index] for index in range(N)) + measurement_variance
        innovation_variance = max(innovation_variance, 1.0e-12)
        nis = (innovation * innovation) / innovation_variance
        accepted = finite(nis) and not (math.isfinite(gate_threshold) and nis > gate_threshold)
        if accepted:
            gain = [value / innovation_variance for value in PHt]
            self.state = [prior[index] + gain[index] * innovation for index in range(N)]
            self.state[HEADING] = normalize_angle(self.state[HEADING])
            updated = zeros(N, N)
            for row in range(N):
                for col in range(N):
                    updated[row][col] = self.P[row][col] - PHt[row] * PHt[col] / innovation_variance
            self.P = symmetrize(updated)
        return UpdateResult(
            log_parameter=log_parameter,
            measurement_dimension=1,
            nis=nis,
            accepted=accepted,
            gate_threshold=gate_threshold,
            innovation=innovation,
            innovation_variance=innovation_variance,
            measurement=measurement,
            prediction=prediction,
        )

    def measurement_jacobian(
        self,
        h: Callable[[list[float]], float],
        state: Sequence[float],
        steps: Sequence[float],
    ) -> list[float]:
        result = [0.0 for _ in range(N)]
        baseline = list(state)
        for index in range(N):
            step = steps[index] if index < len(steps) else 1.0e-5
            plus = baseline.copy()
            minus = baseline.copy()
            plus[index] += step
            minus[index] -= step
            if index == HEADING:
                plus[index] = normalize_angle(plus[index])
                minus[index] = normalize_angle(minus[index])
            result[index] = (h(plus) - h(minus)) / (2.0 * step)
        return result


def replace_sample_wheel_rate(sample: ReplaySample, side: str, delta: float) -> ReplaySample:
    data = sample.__dict__.copy()
    field_name = "left_wheel_rate_radps" if side == "left" else "right_wheel_rate_radps"
    data[field_name] = data[field_name] + delta
    return ReplaySample(**data)


def replace_sample_accel_bias(sample: ReplaySample, bias: AccelBiasEstimate) -> ReplaySample:
    if not bias.valid:
        return sample
    corrected_forward = (
        sample.accel_forward_mps2 - bias.forward_mps2
        if finite(sample.accel_forward_mps2)
        else sample.accel_forward_mps2
    )
    corrected_right = (
        sample.accel_right_mps2 - bias.right_mps2
        if finite(sample.accel_right_mps2)
        else sample.accel_right_mps2
    )
    data = sample.__dict__.copy()
    data["accel_forward_mps2"] = corrected_forward
    data["accel_right_mps2"] = corrected_right
    data["accel_valid"] = (
        sample.accel_valid
        and finite(corrected_forward)
        and finite(corrected_right)
    )
    return ReplaySample(**data)


def predicted_encoder_wheel_rate_radps(
    state: Sequence[float],
    vehicle: VehicleConfig,
    side: str,
) -> float:
    half_track = 0.5 * abs(vehicle.track_width_m)
    right_m = -half_track if side == "left" else half_track
    body_forward_mps = state[VF] - state[YAW_RATE] * right_m
    return body_forward_mps / max(vehicle.wheel_radius_m, 1.0e-9)


def effective_gate_threshold(log_parameter: str, gate_threshold: float) -> float:
    lowered = log_parameter.lower()
    if lowered.startswith("yaw_rate") or lowered.startswith("gyro"):
        return UNGATED_YAW_RATE_NIS_GATE
    if "encoder" in lowered or "wheel_rate" in lowered:
        return UNGATED_ENCODER_NIS_GATE
    return gate_threshold


def is_active_yaw_bias_label(text: str) -> bool:
    return "yaw" in text and any(
        token in text
        for token in ("launch", "maneuver", "turn", "sec 40", "sec_40", "calibration")
    )


def is_accel_bias_assessment_label(
    stage: str,
    parameter_fields: dict[str, Any] | None = None,
    family: str = "",
) -> bool:
    text = " ".join(
        [
            stage,
            family,
            *(str(value) for value in (parameter_fields or {}).values()),
        ]
    ).lower()
    normalized = text.replace("_", " ").replace("-", " ")
    if is_active_yaw_bias_label(normalized):
        return False
    return any(
        token in normalized
        for token in ("stationary", "static", "idle", "zero", "bias")
    )


def is_accel_bias_assessment_segment(segment: "SegmentSpec") -> bool:
    return (
        not segment.corrupted
        and is_accel_bias_assessment_label(segment.stage, segment.parameter_fields, segment.family)
    )


def sample_qualifies_for_accel_bias(sample: ReplaySample) -> bool:
    if not (
        sample.accel_valid
        and finite(sample.accel_forward_mps2)
        and finite(sample.accel_right_mps2)
    ):
        return False
    if max(abs(sample.left_command), abs(sample.right_command)) > ACCEL_BIAS_MAX_DRIVE_COMMAND:
        return False
    if (
        max(abs(sample.left_wheel_rate_radps), abs(sample.right_wheel_rate_radps))
        > ACCEL_BIAS_MAX_ENCODER_WHEEL_SPEED_RADPS
    ):
        return False
    if finite(sample.yaw_rate_radps) and abs(sample.yaw_rate_radps) > ACCEL_BIAS_MAX_YAW_RATE_RADPS:
        return False
    return True


def estimate_accel_bias_from_samples(samples: Iterable[ReplaySample]) -> AccelBiasEstimate:
    accumulator = AccelBiasAccumulator()
    for sample in samples:
        accumulator.add(sample)
    return accumulator.estimate()


def bound_gyro_bias_radps(
    row: Sequence[str],
    columns: ReplayColumnBinding,
) -> tuple[float, str]:
    raw_gyro = parse_float(bound_csv_value(row, columns.raw_gyro))
    used_gyro = parse_float(bound_csv_value(row, columns.gyro))
    if finite(raw_gyro) and finite(used_gyro):
        return raw_gyro - used_gyro, "raw_minus_used_gyro"
    logged_bias = parse_float(bound_csv_value(row, columns.gyro_bias))
    if finite(logged_bias):
        return logged_bias, "logged_gyro_bias"
    return math.nan, "unavailable"


def load_json(path: Path) -> dict[str, Any]:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise ConfigError(f"Invalid JSON in {path}: {exc}") from exc


def load_candidates(path: Path) -> list[CandidateConfig]:
    raw = load_json(path)
    if "candidates" in raw:
        raw_candidates = raw["candidates"]
    else:
        raw_candidates = [raw]
    candidates: list[CandidateConfig] = []
    seen: set[str] = set()
    for raw_candidate in raw_candidates:
        if not isinstance(raw_candidate, dict):
            raise ConfigError(f"Candidate entries in {path} must be objects")
        enabled = parse_bool(raw_candidate.get("enabled", True), True)
        if not enabled:
            continue
        candidate_id = str(raw_candidate.get("candidate_id", raw_candidate.get("id", ""))).strip()
        model = str(raw_candidate.get("model", "")).strip()
        if not candidate_id or not model:
            raise ConfigError(f"Candidate missing candidate_id/id or model in {path}")
        validate_candidate_model(model)
        if candidate_id in seen:
            raise ConfigError(f"Duplicate candidate id {candidate_id} in {path}")
        seen.add(candidate_id)
        params = {
            str(key): parse_float(value)
            for key, value in dict(raw_candidate.get("parameters", {})).items()
        }
        candidates.append(
            CandidateConfig(
                candidate_id=candidate_id,
                label=str(raw_candidate.get("label", candidate_id)),
                model=model,
                parameters=params,
            )
        )
    if not candidates:
        raise ConfigError(f"No enabled candidates found in {path}")
    return candidates


def load_covariance(path: Path) -> tuple[VehicleConfig, CovarianceConfig]:
    raw = load_json(path)
    vehicle = VehicleConfig.from_json(dict(raw.get("vehicle", {})))
    covariance = CovarianceConfig.from_json(dict(raw.get("covariance", raw)))
    return vehicle, covariance


def stable_split(segment_id: str, split_config: dict[str, Any] | None = None) -> str:
    config = split_config or DEFAULT_SPLIT_CONFIG
    seed = str(config.get("seed", DEFAULT_SPLIT_CONFIG["seed"]))
    ratios = [
        float(config.get("train", DEFAULT_SPLIT_CONFIG["train"])),
        float(config.get("validation", DEFAULT_SPLIT_CONFIG["validation"])),
        float(config.get("held_out", DEFAULT_SPLIT_CONFIG["held_out"])),
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


def segment_is_corrupted(row: dict[str, Any]) -> bool:
    explicitly_marked = (
        parse_bool(row.get("corrupted", row.get("is_corrupted", False)), False)
        or str(row.get("end_reason", "") or "").strip().lower() == "corruption_boundary"
        or bool(str(row.get("corruption_note", "") or "").strip())
    )
    return explicitly_marked and is_terminal_external_force_boundary(row)


def is_terminal_external_force_boundary(row: dict[str, Any]) -> bool:
    text = " ".join(
        str(row.get(name, "") or "")
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


def segment_parameter_fields(row: dict[str, Any]) -> dict[str, Any]:
    raw = row.get("parameter_fields")
    if isinstance(raw, dict):
        return dict(raw)
    fields: dict[str, Any] = {}
    for name in (
        "parameter",
        "test_value_kind",
        "test_value",
        "speed_bin",
        "speed_bin_label",
        "cmd_linear_mps_median",
        "cmd_yaw_radps_median",
    ):
        if name in row:
            fields[name] = row.get(name)
    return fields


def segment_observed_command(row: dict[str, Any]) -> dict[str, Any]:
    raw = row.get("observed_command")
    if isinstance(raw, dict):
        return dict(raw)
    fields: dict[str, Any] = {}
    for name in (
        "active_command_rows",
        "observed_command_pair_mode",
        "observed_command_magnitude_median",
        "cmd_linear_mps_median",
        "cmd_yaw_radps_median",
    ):
        if name in row:
            fields[name] = row.get(name)
    return fields


def segment_specs_from_manifest(path: Path, repo_root: Path) -> list[SegmentSpec]:
    if path.suffix.lower() == ".csv":
        with path.open(newline="", encoding="utf-8-sig") as handle:
            rows = list(csv.DictReader(handle))
    else:
        raw = load_json(path)
        rows = list(raw.get("segments", []))
    specs: list[SegmentSpec] = []
    for row in rows:
        if not isinstance(row, dict):
            continue
        log_text = str(row.get("log_path", row.get("path", ""))).strip()
        segment_id = str(row.get("segment_id", "")).strip()
        if not log_text or not segment_id:
            continue
        log_path = Path(log_text)
        if not log_path.is_absolute():
            log_path = repo_root / log_path
        split = str(row.get("split", "") or "").strip() or stable_split(segment_id)
        specs.append(
            SegmentSpec(
                log_path=log_path,
                segment_id=segment_id,
                stage=str(row.get("stage", row.get("section_name", ""))),
                family=str(row.get("family", "")),
                split=split,
                run_id=str(row.get("log_id", row.get("run_id", ""))),
                start_row_index=parse_int(row.get("segment_start_row_index"), 0),
                end_row_index=parse_int(row.get("segment_end_row_index"), -1),
                corrupted=segment_is_corrupted(row),
                parameter_fields=segment_parameter_fields(row),
                observed_command=segment_observed_command(row),
            )
        )
    return specs


def direct_input_segment(
    input_csv: Path,
    segment_id: str,
    stage: str,
    split: str,
    run_id: str,
) -> SegmentSpec:
    return SegmentSpec(
        log_path=input_csv,
        segment_id=segment_id or input_csv.stem,
        stage=stage,
        split=split,
        run_id=run_id or input_csv.parent.name,
        start_row_index=0,
        end_row_index=-1,
    )


@dataclass(frozen=True)
class SourceLogIndex:
    fieldnames: list[str]
    columns: ReplayColumnBinding
    row_offsets: list[int]


class SourceLogSampleCache:
    """Caches source-log row offsets and reads segment rows without rescanning CSV prefixes."""

    def __init__(self) -> None:
        self._indexes: dict[Path, SourceLogIndex] = {}
        self.index_build_count = 0
        self.indexed_row_count = 0
        self.file_read_count = 0

    def index_for_log(self, log_path: Path) -> SourceLogIndex:
        cached = self._indexes.get(log_path)
        if cached is not None:
            return cached
        if not log_path.exists():
            raise ConfigError(f"Input CSV not found: {log_path}")
        with log_path.open("rb") as handle:
            header = handle.readline()
            if not header:
                raise ConfigError(f"CSV header missing: {log_path}")
            header_text = header.decode("utf-8-sig", errors="replace")
            try:
                fieldnames = next(csv.reader([header_text]))
            except StopIteration as exc:
                raise ConfigError(f"CSV header missing: {log_path}") from exc
            offsets: list[int] = []
            while True:
                offset = handle.tell()
                line = handle.readline()
                if not line:
                    break
                offsets.append(offset)
        index = SourceLogIndex(
            fieldnames=fieldnames,
            columns=ReplayColumnBinding.from_fieldnames(fieldnames),
            row_offsets=offsets,
        )
        self._indexes[log_path] = index
        self.index_build_count += 1
        self.indexed_row_count += len(offsets)
        return index

    def read_targeted_segment_samples(
        self,
        log_path: Path,
        segments: Sequence[SegmentSpec],
        target_rows_by_segment: dict[tuple[Path, str, int, int], Sequence[int]],
        vehicle: VehicleConfig,
    ) -> dict[tuple[Path, str, int, int], list[ReplaySample]]:
        result: dict[tuple[Path, str, int, int], list[ReplaySample]] = {
            segment_sample_key(segment): [] for segment in segments
        }
        row_targets: dict[int, list[SegmentSpec]] = {}
        for segment in segments:
            key = segment_sample_key(segment)
            for row_index in target_rows_by_segment.get(key, ()):
                row_targets.setdefault(row_index, []).append(segment)
        if not row_targets:
            return result

        index = self.index_for_log(log_path)
        previous_time_by_segment: dict[tuple[Path, str, int, int], int] = {}
        previous_sample_by_segment: dict[tuple[Path, str, int, int], ReplaySample] = {}
        self.file_read_count += 1
        with log_path.open("rb") as handle:
            for row_index in sorted(row_targets):
                if row_index >= len(index.row_offsets):
                    continue
                handle.seek(index.row_offsets[row_index])
                line = handle.readline().decode("utf-8", errors="replace")
                try:
                    row = next(csv.reader([line]))
                except StopIteration:
                    continue
                for segment in row_targets[row_index]:
                    key = segment_sample_key(segment)
                    sample = sample_from_bound_row(
                        row,
                        index.columns,
                        segment,
                        row_index,
                        previous_time_by_segment.get(key),
                        vehicle,
                        previous_sample_by_segment.get(key),
                    )
                    previous_time_by_segment[key] = sample.master_time_us
                    previous_sample_by_segment[key] = sample
                    result[key].append(sample)
        return result


def read_segment_samples(
    spec: SegmentSpec,
    vehicle: VehicleConfig,
    max_rows: int,
) -> Iterator[ReplaySample]:
    if not spec.log_path.exists():
        raise ConfigError(f"Input CSV not found: {spec.log_path}")
    emitted = 0
    previous_time_us: int | None = None
    previous_sample: ReplaySample | None = None
    with spec.log_path.open(newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        if not reader.fieldnames:
            raise ConfigError(f"CSV header missing: {spec.log_path}")
        for row_index, row in enumerate(reader):
            if row_index < spec.start_row_index:
                continue
            if spec.end_row_index >= 0 and row_index > spec.end_row_index:
                break
            sample = sample_from_row(row, spec, row_index, previous_time_us, vehicle, previous_sample)
            previous_time_us = sample.master_time_us
            yield sample
            previous_sample = sample
            emitted += 1
            if max_rows > 0 and emitted >= max_rows:
                break


def segment_row_indices(spec: SegmentSpec, max_rows: int, row_count: int | None = None) -> list[int]:
    if max_rows > 0:
        end_row_index = spec.start_row_index + max_rows - 1
        if spec.end_row_index >= 0:
            end_row_index = min(end_row_index, spec.end_row_index)
    elif spec.end_row_index >= 0:
        end_row_index = spec.end_row_index
    elif row_count is not None:
        end_row_index = row_count - 1
    else:
        return []
    if end_row_index < spec.start_row_index:
        return []
    return list(range(spec.start_row_index, end_row_index + 1))


def read_segment_samples_cached(
    cache: SourceLogSampleCache,
    spec: SegmentSpec,
    vehicle: VehicleConfig,
    max_rows: int,
) -> Iterator[ReplaySample]:
    key = segment_sample_key(spec)
    index = cache.index_for_log(spec.log_path)
    loaded = cache.read_targeted_segment_samples(
        spec.log_path,
        [spec],
        {key: segment_row_indices(spec, max_rows, len(index.row_offsets))},
        vehicle,
    )
    yield from loaded[key]


def representative_row_indices(start_row_index: int, end_row_index: int, max_rows: int) -> set[int] | None:
    if end_row_index < start_row_index or max_rows <= 0:
        return None
    row_count = end_row_index - start_row_index + 1
    if row_count <= max_rows:
        return set(range(start_row_index, end_row_index + 1))
    if max_rows <= 1:
        return {start_row_index}
    return {
        start_row_index + int(round(index * (row_count - 1) / float(max_rows - 1)))
        for index in range(max_rows)
    }


def read_representative_segment_samples(
    spec: SegmentSpec,
    vehicle: VehicleConfig,
    max_rows: int,
) -> Iterator[ReplaySample]:
    if not spec.log_path.exists():
        raise ConfigError(f"Input CSV not found: {spec.log_path}")
    target_rows = representative_row_indices(spec.start_row_index, spec.end_row_index, max_rows)
    emitted = 0
    previous_time_us: int | None = None
    previous_sample: ReplaySample | None = None
    with spec.log_path.open(newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        if not reader.fieldnames:
            raise ConfigError(f"CSV header missing: {spec.log_path}")
        for row_index, row in enumerate(reader):
            if row_index < spec.start_row_index:
                continue
            if spec.end_row_index >= 0 and row_index > spec.end_row_index:
                break
            if target_rows is not None and row_index not in target_rows:
                continue
            sample = sample_from_row(row, spec, row_index, previous_time_us, vehicle, previous_sample)
            previous_time_us = sample.master_time_us
            yield sample
            previous_sample = sample
            emitted += 1
            if target_rows is None and max_rows > 0 and emitted >= max_rows:
                break


def segment_sample_key(segment: SegmentSpec) -> tuple[Path, str, int, int]:
    return (
        segment.log_path,
        segment.segment_id,
        segment.start_row_index,
        segment.end_row_index,
    )


def read_representative_samples_by_log(
    log_path: Path,
    segments: Sequence[SegmentSpec],
    vehicle: VehicleConfig,
    max_rows: int,
) -> dict[tuple[Path, str, int, int], list[ReplaySample]]:
    result: dict[tuple[Path, str, int, int], list[ReplaySample]] = {
        segment_sample_key(segment): [] for segment in segments
    }
    selected_segments = sorted(segments, key=lambda segment: segment.start_row_index)
    if not selected_segments:
        return result
    if not log_path.exists():
        raise ConfigError(f"Input CSV not found: {log_path}")

    target_rows_by_segment = {
        segment_sample_key(segment): representative_row_indices(
            segment.start_row_index,
            segment.end_row_index,
            max_rows,
        )
        for segment in selected_segments
    }
    previous_time_by_segment: dict[tuple[Path, str, int, int], int] = {}
    previous_sample_by_segment: dict[tuple[Path, str, int, int], ReplaySample] = {}
    pending = deque(selected_segments)
    active: list[SegmentSpec] = []
    with log_path.open(newline="", encoding="utf-8-sig") as handle:
        reader = csv.reader(handle)
        fieldnames = next(reader, None)
        if not fieldnames:
            raise ConfigError(f"CSV header missing: {log_path}")
        columns = ReplayColumnBinding.from_fieldnames(fieldnames)
        for row_index, row in enumerate(reader):
            while pending and pending[0].start_row_index <= row_index:
                active.append(pending.popleft())

            if not active:
                if not pending:
                    break
                continue

            still_active: list[SegmentSpec] = []
            for segment in active:
                key = segment_sample_key(segment)
                if segment.end_row_index >= 0 and row_index > segment.end_row_index:
                    continue
                target_rows = target_rows_by_segment[key]
                if target_rows is None or row_index in target_rows:
                    sample = sample_from_bound_row(
                        row,
                        columns,
                        segment,
                        row_index,
                        previous_time_by_segment.get(key),
                        vehicle,
                        previous_sample_by_segment.get(key),
                    )
                    previous_time_by_segment[key] = sample.master_time_us
                    previous_sample_by_segment[key] = sample
                    result[key].append(sample)
                if segment.end_row_index < 0 or row_index < segment.end_row_index:
                    still_active.append(segment)
            active = still_active

            if not pending and not active:
                break
    return result


def estimate_accel_bias_for_log(
    log_path: Path,
    log_segments: Sequence["SegmentSpec"],
    vehicle: VehicleConfig,
) -> AccelBiasEstimate:
    return estimate_sensor_bias_for_log(log_path, log_segments, vehicle).accel


def estimate_sensor_bias_for_log(
    log_path: Path,
    log_segments: Sequence["SegmentSpec"],
    vehicle: VehicleConfig,
) -> SensorBiasEstimate:
    bias_segments = sorted(
        [segment for segment in log_segments if is_accel_bias_assessment_segment(segment)],
        key=lambda segment: segment.start_row_index,
    )
    if not bias_segments:
        return SensorBiasEstimate()
    if not log_path.exists():
        raise ConfigError(f"Input CSV not found: {log_path}")

    pending = deque(bias_segments)
    active: list[tuple[SegmentSpec, int, int | None]] = []
    accel_accumulator = AccelBiasAccumulator()
    gyro_accumulator = GyroBiasAccumulator()
    with log_path.open(newline="", encoding="utf-8-sig") as handle:
        reader = csv.reader(handle)
        fieldnames = next(reader, None)
        if not fieldnames:
            raise ConfigError(f"CSV header missing: {log_path}")
        columns = ReplayColumnBinding.from_fieldnames(fieldnames)
        for row_index, row in enumerate(reader):
            while pending and pending[0].start_row_index <= row_index:
                active.append((pending.popleft(), 0, None))
            if not active:
                if not pending:
                    break
                continue
            still_active: list[tuple[SegmentSpec, int, int | None]] = []
            for segment, emitted, previous_time_us in active:
                if segment.end_row_index >= 0 and row_index > segment.end_row_index:
                    continue
                sample = sample_from_bound_row(
                    row,
                    columns,
                    segment,
                    row_index,
                    previous_time_us,
                    vehicle,
                )
                if sample_qualifies_for_accel_bias(sample):
                    gyro_bias, gyro_source = bound_gyro_bias_radps(row, columns)
                    gyro_accumulator.add(gyro_bias, gyro_source)
                accel_accumulator.add(sample)
                emitted += 1
                previous_time_us = sample.master_time_us
                if segment.end_row_index < 0 or row_index < segment.end_row_index:
                    still_active.append((segment, emitted, previous_time_us))
            active = still_active
    gyro_bias, gyro_count, gyro_source = gyro_accumulator.estimate()
    return SensorBiasEstimate(
        accel=accel_accumulator.estimate(),
        gyro_bias_radps=gyro_bias,
        gyro_sample_count=gyro_count,
        gyro_source=gyro_source,
        bias_segment_count=len(bias_segments),
    )


def sample_from_row(
    row: dict[str, str],
    spec: SegmentSpec,
    row_index: int,
    previous_time_us: int | None,
    vehicle: VehicleConfig,
    previous_sample: ReplaySample | None = None,
) -> ReplaySample:
    master_time_us = parse_int(csv_value(row, ("master_time_us", "timestamp_us", "time_us")), 0)
    dt_us = parse_float(csv_value(row, ("dt_us",)), math.nan)
    if not finite(dt_us) and previous_time_us is not None and master_time_us > previous_time_us:
        dt_us = float(master_time_us - previous_time_us)
    dt_s = max(0.0, min(finite_or(dt_us, 1000.0) * 1.0e-6, 0.050))
    left_rate = parse_float(
        csv_value(row, ("left_encoder_omega_radps", "left_encoder_wheel_speed_radps", "left_wheel_rate_radps"))
    )
    right_rate = parse_float(
        csv_value(row, ("right_encoder_omega_radps", "right_encoder_wheel_speed_radps", "right_wheel_rate_radps"))
    )
    if not finite(left_rate):
        left_velocity = parse_float(csv_value(row, ("left_encoder_velocity_mps", "left_wheel_velocity_mps")))
        left_rate = left_velocity / vehicle.wheel_radius_m if finite(left_velocity) else 0.0
    if not finite(right_rate):
        right_velocity = parse_float(csv_value(row, ("right_encoder_velocity_mps", "right_wheel_velocity_mps")))
        right_rate = right_velocity / vehicle.wheel_radius_m if finite(right_velocity) else 0.0

    gyro = parse_float(csv_value(row, ("gyro_radps", "yaw_rate_radps")))
    if not finite(gyro):
        raw_gyro = parse_float(csv_value(row, ("gyro_raw_radps", "imu_gyro_z")))
        gyro_bias = parse_float(csv_value(row, ("gyro_bias_radps",)), 0.0)
        gyro = raw_gyro - gyro_bias if finite(raw_gyro) else math.nan
    if not finite(gyro):
        gyro = parse_float(csv_value(row, ("measured_angular_speed_radps",)))

    accel_forward = parse_float(
        csv_value(row, ("accel_body_forward_mps2", "accel_body_y_mps2", "forward_accel_mps2"))
    )
    accel_right = parse_float(
        csv_value(row, ("accel_body_right_mps2", "accel_body_x_mps2", "right_accel_mps2"))
    )
    accel_valid_text = csv_value(row, ("accel_valid", "imu_accel_valid"))
    accel_valid = parse_bool(accel_valid_text, finite(accel_forward) and finite(accel_right))
    gyro_valid = finite(gyro)
    stage = spec.stage or csv_value(row, ("stage", "section_name", "phase_name")) or "unlabeled"
    run_id = spec.run_id or csv_value(row, ("run_id", "log_id")) or spec.log_path.parent.name
    sample = ReplaySample(
        source_path=spec.log_path,
        source_row_index=row_index,
        master_time_us=master_time_us,
        dt_s=dt_s,
        left_command=finite_or(parse_float(csv_value(row, ("left_drive_command", "left_command")), 0.0), 0.0),
        right_command=finite_or(parse_float(csv_value(row, ("right_drive_command", "right_command")), 0.0), 0.0),
        left_wheel_rate_radps=left_rate,
        right_wheel_rate_radps=right_rate,
        yaw_rate_radps=gyro,
        accel_forward_mps2=accel_forward,
        accel_right_mps2=accel_right,
        accel_valid=accel_valid,
        gyro_valid=gyro_valid,
        fan_duty_cycle=finite_or(
            parse_float(csv_value(row, ("fan_duty_cycle", "fan_command")), vehicle.default_fan_duty_cycle),
            vehicle.default_fan_duty_cycle,
        ),
        segment_id=spec.segment_id,
        stage=stage,
        split=spec.split,
        run_id=run_id,
        corrupted=spec.corrupted,
    )
    return sample_with_previous_inputs(sample, previous_sample)


def bound_csv_value(row: Sequence[str], indices: Sequence[int]) -> str:
    row_len = len(row)
    for index in indices:
        if index >= row_len:
            continue
        value = row[index]
        if value is not None and str(value).strip():
            return str(value).strip()
    return ""


def sample_from_bound_row(
    row: Sequence[str],
    columns: ReplayColumnBinding,
    spec: SegmentSpec,
    row_index: int,
    previous_time_us: int | None,
    vehicle: VehicleConfig,
    previous_sample: ReplaySample | None = None,
) -> ReplaySample:
    master_time_us = parse_int(bound_csv_value(row, columns.master_time_us), 0)
    dt_us = parse_float(bound_csv_value(row, columns.dt_us), math.nan)
    if not finite(dt_us) and previous_time_us is not None and master_time_us > previous_time_us:
        dt_us = float(master_time_us - previous_time_us)
    dt_s = max(0.0, min(finite_or(dt_us, 1000.0) * 1.0e-6, 0.050))

    left_rate = parse_float(bound_csv_value(row, columns.left_rate))
    right_rate = parse_float(bound_csv_value(row, columns.right_rate))
    if not finite(left_rate):
        left_velocity = parse_float(bound_csv_value(row, columns.left_velocity))
        left_rate = left_velocity / vehicle.wheel_radius_m if finite(left_velocity) else 0.0
    if not finite(right_rate):
        right_velocity = parse_float(bound_csv_value(row, columns.right_velocity))
        right_rate = right_velocity / vehicle.wheel_radius_m if finite(right_velocity) else 0.0

    gyro = parse_float(bound_csv_value(row, columns.gyro))
    if not finite(gyro):
        raw_gyro = parse_float(bound_csv_value(row, columns.raw_gyro))
        gyro_bias = parse_float(bound_csv_value(row, columns.gyro_bias), 0.0)
        gyro = raw_gyro - gyro_bias if finite(raw_gyro) else math.nan
    if not finite(gyro):
        gyro = parse_float(bound_csv_value(row, columns.measured_angular_speed))

    accel_forward = parse_float(bound_csv_value(row, columns.accel_forward))
    accel_right = parse_float(bound_csv_value(row, columns.accel_right))
    accel_valid_text = bound_csv_value(row, columns.accel_valid)
    accel_valid = parse_bool(accel_valid_text, finite(accel_forward) and finite(accel_right))
    gyro_valid = finite(gyro)
    stage = spec.stage or bound_csv_value(row, columns.stage) or "unlabeled"
    run_id = spec.run_id or bound_csv_value(row, columns.run_id) or spec.log_path.parent.name

    sample = ReplaySample(
        source_path=spec.log_path,
        source_row_index=row_index,
        master_time_us=master_time_us,
        dt_s=dt_s,
        left_command=finite_or(
            parse_float(bound_csv_value(row, columns.left_command), 0.0),
            0.0,
        ),
        right_command=finite_or(
            parse_float(bound_csv_value(row, columns.right_command), 0.0),
            0.0,
        ),
        left_wheel_rate_radps=left_rate,
        right_wheel_rate_radps=right_rate,
        yaw_rate_radps=gyro,
        accel_forward_mps2=accel_forward,
        accel_right_mps2=accel_right,
        accel_valid=accel_valid,
        gyro_valid=gyro_valid,
        fan_duty_cycle=finite_or(
            parse_float(bound_csv_value(row, columns.fan_duty_cycle), vehicle.default_fan_duty_cycle),
            vehicle.default_fan_duty_cycle,
        ),
        segment_id=spec.segment_id,
        stage=stage,
        split=spec.split,
        run_id=run_id,
        corrupted=spec.corrupted,
    )
    return sample_with_previous_inputs(sample, previous_sample)


def sample_with_previous_inputs(
    sample: ReplaySample,
    previous_sample: ReplaySample | None,
) -> ReplaySample:
    if previous_sample is None:
        return sample
    data = sample.__dict__.copy()
    data["previous_left_wheel_rate_radps"] = previous_sample.left_wheel_rate_radps
    data["previous_right_wheel_rate_radps"] = previous_sample.right_wheel_rate_radps
    data["previous_yaw_rate_radps"] = previous_sample.yaw_rate_radps
    return ReplaySample(**data)


def measured_yaw_accel_from_previous(
    previous_yaw_rate_radps: float | None,
    sample: ReplaySample,
) -> float:
    if (
        previous_yaw_rate_radps is None
        or not sample.gyro_valid
        or not finite(previous_yaw_rate_radps)
        or not finite(sample.yaw_rate_radps)
        or sample.dt_s <= 0.0
    ):
        return math.nan
    return (sample.yaw_rate_radps - previous_yaw_rate_radps) / sample.dt_s


@dataclass
class CandidateAccumulator:
    nis_count: int = 0
    finite_count: int = 0
    nis_sum: float = 0.0
    nis_sum_square: float = 0.0
    accepted_count: int = 0
    rejected_count: int = 0
    accepted_nis_sum: float = 0.0
    accepted_nis_sum_square: float = 0.0
    segment_ids: set[str] = field(default_factory=set)
    residual_sum_square: dict[str, float] = field(default_factory=dict)
    residual_count: dict[str, int] = field(default_factory=dict)

    def add_update(self, update: UpdateResult, segment_id: str) -> None:
        self.add_values(
            update.log_parameter,
            update.nis,
            update.accepted,
            update.innovation,
            segment_id,
        )

    def add_values(
        self,
        log_parameter: str,
        nis: float,
        accepted: bool,
        innovation: float,
        segment_id: str,
    ) -> None:
        self.nis_count += 1
        self.segment_ids.add(segment_id)
        if finite(nis):
            self.finite_count += 1
            self.nis_sum += nis
            self.nis_sum_square += nis * nis
        if accepted:
            self.accepted_count += 1
            self.accepted_nis_sum += nis
            self.accepted_nis_sum_square += nis * nis
        else:
            self.rejected_count += 1
        residual_key = physical_residual_key(log_parameter)
        if finite(innovation):
            self.residual_sum_square[residual_key] = (
                self.residual_sum_square.get(residual_key, 0.0) + innovation * innovation
            )
            self.residual_count[residual_key] = self.residual_count.get(residual_key, 0) + 1

    def rms_nis(self) -> float:
        if self.finite_count == 0:
            return math.nan
        return math.sqrt(self.nis_sum_square / self.finite_count)

    def sqrt_mean_nis(self) -> float:
        if self.finite_count == 0:
            return math.nan
        return math.sqrt(self.nis_sum / self.finite_count)

    def accepted_only_rms_nis(self) -> float:
        if self.accepted_count == 0:
            return math.nan
        return math.sqrt(self.accepted_nis_sum_square / self.accepted_count)

    def accepted_only_sqrt_mean_nis(self) -> float:
        if self.accepted_count == 0:
            return math.nan
        return math.sqrt(self.accepted_nis_sum / self.accepted_count)

    def rejected_rate(self) -> float:
        if self.nis_count == 0:
            return 0.0
        return self.rejected_count / self.nis_count

    def add_physical_residual(self, key: str, residual: float) -> None:
        if not finite(residual):
            return
        self.residual_sum_square[key] = self.residual_sum_square.get(key, 0.0) + residual * residual
        self.residual_count[key] = self.residual_count.get(key, 0) + 1


@dataclass(frozen=True)
class AggregateKey:
    candidate_id: str
    split: str
    stage: str
    log_parameter: str
    measurement_dimension: int
    parameter_field: str
    parameter_value_kind: str
    parameter_value: str
    launch_command_signature: str


@dataclass
class NisAggregate:
    count: int = 0
    finite_count: int = 0
    sum_nis: float = 0.0
    sum_nis_sq: float = 0.0
    accepted_count: int = 0
    rejected_count: int = 0
    accepted_sum_nis: float = 0.0
    accepted_sum_nis_sq: float = 0.0
    segment_ids: set[str] = field(default_factory=set)
    residual_sum_square: float = 0.0
    residual_count: int = 0

    def add(self, segment_id: str, update: UpdateResult) -> None:
        self.add_values(segment_id, update.nis, update.accepted, update.innovation)

    def add_values(self, segment_id: str, nis: float, accepted: bool, innovation: float) -> None:
        self.count += 1
        self.segment_ids.add(segment_id)
        if finite(nis):
            self.finite_count += 1
            self.sum_nis += nis
            self.sum_nis_sq += nis * nis
        if accepted:
            self.accepted_count += 1
            self.accepted_sum_nis += nis
            self.accepted_sum_nis_sq += nis * nis
        else:
            self.rejected_count += 1
        if finite(innovation):
            self.residual_sum_square += innovation * innovation
            self.residual_count += 1

    def rms_nis(self) -> float:
        if self.finite_count == 0:
            return math.nan
        return math.sqrt(self.sum_nis_sq / self.finite_count)

    def sqrt_mean_nis(self) -> float:
        if self.finite_count == 0:
            return math.nan
        return math.sqrt(self.sum_nis / self.finite_count)

    def accepted_only_rms_nis(self) -> float:
        if self.accepted_count == 0:
            return math.nan
        return math.sqrt(self.accepted_sum_nis_sq / self.accepted_count)

    def accepted_only_sqrt_mean_nis(self) -> float:
        if self.accepted_count == 0:
            return math.nan
        return math.sqrt(self.accepted_sum_nis / self.accepted_count)

    def rejected_rate(self) -> float:
        if self.count == 0:
            return 0.0
        return self.rejected_count / self.count

    def rejected_rate_penalty(self) -> float:
        return REJECTED_RATE_PENALTY_WEIGHT * self.rejected_rate()

    def physical_residual_rms(self) -> float:
        if self.residual_count == 0:
            return math.nan
        return math.sqrt(self.residual_sum_square / self.residual_count)


def aggregate_splits(split: str) -> tuple[str, ...]:
    clean = split.strip() if split else "unassigned"
    if clean == "all":
        return ("all",)
    return (clean, "all")


def aggregate_key(
    candidate: CandidateConfig,
    segment: SegmentSpec,
    sample: ReplaySample,
    update: UpdateResult,
    split: str,
) -> AggregateKey:
    parameter_fields = segment.parameter_fields
    return AggregateKey(
        candidate_id=candidate.candidate_id,
        split=split,
        stage=sample.stage,
        log_parameter=update.log_parameter,
        measurement_dimension=update.measurement_dimension,
        parameter_field=format_metadata_value(parameter_fields.get("parameter", "")),
        parameter_value_kind=format_metadata_value(parameter_fields.get("test_value_kind", "")),
        parameter_value=format_metadata_value(parameter_fields.get("test_value", "")),
        launch_command_signature=command_bucket_for_sample(segment, sample),
    )


def add_aggregate_update(
    aggregates: dict[AggregateKey, NisAggregate],
    candidate: CandidateConfig,
    segment: SegmentSpec,
    sample: ReplaySample,
    update: UpdateResult,
) -> None:
    for split in aggregate_splits(sample.split):
        key = aggregate_key(candidate, segment, sample, update, split)
        aggregates.setdefault(key, NisAggregate()).add(sample.segment_id, update)


def precomputed_aggregate_keys(
    segment: SegmentSpec,
    candidates: Sequence[CandidateConfig],
) -> dict[tuple[str, str, int], tuple[AggregateKey, ...]]:
    if not segment.stage or is_launch_stage(segment.stage):
        return {}
    result: dict[tuple[str, str, int], tuple[AggregateKey, ...]] = {}
    for candidate in candidates:
        for log_parameter in (
            *PRODUCTION_MEASUREMENT_NIS_LOG_PARAMETERS,
            *PRODUCTION_MEASUREMENT_RESIDUAL_TAIL_LOG_PARAMETERS,
        ):
            dimension = 1
            keys = tuple(
                AggregateKey(
                    candidate_id=candidate.candidate_id,
                    split=split,
                    stage=segment.stage,
                    log_parameter=log_parameter,
                    measurement_dimension=dimension,
                    parameter_field=format_metadata_value(
                        segment.parameter_fields.get("parameter", "")
                    ),
                    parameter_value_kind=format_metadata_value(
                        segment.parameter_fields.get("test_value_kind", "")
                    ),
                    parameter_value=format_metadata_value(
                        segment.parameter_fields.get("test_value", "")
                    ),
                    launch_command_signature=launch_command_signature(
                        segment.observed_command
                    ),
                )
                for split in aggregate_splits(segment.split)
            )
            result[(candidate.candidate_id, log_parameter, dimension)] = keys
    return result


def format_metadata_value(value: Any) -> str:
    if value is None:
        return ""
    if isinstance(value, float):
        return format_number(value)
    if isinstance(value, int):
        return str(value)
    if isinstance(value, list):
        return ",".join(format_metadata_value(child) for child in value)
    return str(value)


def launch_command_signature(observed: dict[str, Any]) -> str:
    parts: list[str] = []
    pair = observed.get("observed_command_pair_mode")
    if pair not in (None, ""):
        parts.append(f"pair={format_metadata_value(pair)}")
    magnitude = observed.get("observed_command_magnitude_median")
    if magnitude not in (None, ""):
        parts.append(f"mag={format_metadata_value(magnitude)}")
    linear = observed.get("cmd_linear_mps_median")
    if linear not in (None, ""):
        parts.append(f"linear={format_metadata_value(linear)}")
    yaw = observed.get("cmd_yaw_radps_median")
    if yaw not in (None, ""):
        parts.append(f"yaw={format_metadata_value(yaw)}")
    return ";".join(parts)


def sample_launch_command_signature(sample: ReplaySample) -> str:
    linear_command = 0.5 * (sample.left_command + sample.right_command)
    yaw_command = 0.5 * (sample.right_command - sample.left_command)
    return (
        f"pair={command_bin(sample.left_command)},{command_bin(sample.right_command)};"
        f"linear={command_bin(linear_command)};"
        f"yaw={command_bin(yaw_command)}"
    )


def command_bucket_for_sample(segment: SegmentSpec, sample: ReplaySample) -> str:
    if is_launch_stage(sample.stage):
        return sample_launch_command_signature(sample)
    return launch_command_signature(segment.observed_command)


def command_bin(value: float) -> str:
    if not finite(value):
        return ""
    binned = round(value / COMMAND_BIN_WIDTH) * COMMAND_BIN_WIDTH
    return format_number(0.0 if abs(binned) < 0.5 * COMMAND_BIN_WIDTH else binned)


def is_launch_stage(stage: str) -> bool:
    return "launch" in stage.lower()


def score_partition(stage: str) -> str:
    if is_accel_bias_assessment_label(stage):
        return "stationary_bias_validation"
    return "active_traction"


def metric_kind_for_log_parameter(log_parameter: str) -> str:
    return "residual_tail" if log_parameter.endswith("_residual_tail") else "anis"


def physical_residual_key(log_parameter: str) -> str:
    if log_parameter.startswith("left_encoder"):
        return "left_encoder_wheel_rate_radps"
    if log_parameter.startswith("right_encoder"):
        return "right_encoder_wheel_rate_radps"
    if log_parameter.startswith("yaw_accel"):
        return "yaw_accel_radps2"
    if log_parameter.startswith("yaw_rate"):
        return "yaw_rate_radps"
    if log_parameter.startswith("forward_accel"):
        return "forward_accel_mps2"
    if log_parameter.startswith("right_accel"):
        return "right_accel_mps2"
    return log_parameter


def expected_rms_for_dimension(dimension: int) -> float:
    return math.sqrt(float(dimension) * float(dimension + 2))


def write_aggregate_csv(
    output_dir: Path,
    aggregates: dict[AggregateKey, NisAggregate],
) -> Path:
    path = output_dir / "nis_aggregates.csv"
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=AGGREGATE_FIELDS)
        writer.writeheader()
        for key, stats in sorted(aggregates.items(), key=aggregate_sort_key):
            writer.writerow(aggregate_row(key, stats))
    return path


def aggregate_sort_key(item: tuple[AggregateKey, NisAggregate]) -> tuple[str, ...]:
    key = item[0]
    return (
        key.candidate_id,
        key.split,
        key.stage,
        key.log_parameter,
        str(key.measurement_dimension),
        key.parameter_field,
        key.parameter_value_kind,
        key.parameter_value,
        key.launch_command_signature,
    )


def aggregate_row(key: AggregateKey, stats: NisAggregate) -> dict[str, str]:
    return {
        "candidate_id": key.candidate_id,
        "split": key.split,
        "stage": key.stage,
        "log_parameter": key.log_parameter,
        "measurement_dimension": str(key.measurement_dimension),
        "parameter_field": key.parameter_field,
        "parameter_value_kind": key.parameter_value_kind,
        "parameter_value": key.parameter_value,
        "launch_command_signature": key.launch_command_signature,
        "count": str(stats.count),
        "accepted_count": str(stats.accepted_count),
        "rejected_count": str(stats.rejected_count),
        "rejected_rate": format_number(stats.rejected_rate()),
        "rejected_rate_penalty": format_number(stats.rejected_rate_penalty()),
        "finite_count": str(stats.finite_count),
        "nonfinite_count": str(stats.count - stats.finite_count),
        "sum_nis": format_number(stats.sum_nis),
        "sum_nis_sq": format_number(stats.sum_nis_sq),
        "rms_nis": format_number(stats.rms_nis()),
        "sqrt_mean_nis": format_number(stats.sqrt_mean_nis()),
        "accepted_only_sum_nis": format_number(stats.accepted_sum_nis),
        "accepted_only_sum_nis_sq": format_number(stats.accepted_sum_nis_sq),
        "accepted_only_rms_nis": format_number(stats.accepted_only_rms_nis()),
        "accepted_only_sqrt_mean_nis": format_number(stats.accepted_only_sqrt_mean_nis()),
        "physical_residual_rms": format_number(stats.physical_residual_rms()),
        "segment_count": str(len(stats.segment_ids)),
    }


def write_itemized_aggregate_csv(
    output_dir: Path,
    aggregates: dict[AggregateKey, NisAggregate],
    inflation_floor_ratio: float = 0.75,
) -> Path:
    path = output_dir / "itemized_rms_nis.csv"
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=ITEMIZED_AGGREGATE_FIELDS)
        writer.writeheader()
        for key, stats in sorted(aggregates.items(), key=aggregate_sort_key):
            rms = stats.rms_nis()
            expected = expected_rms_for_dimension(key.measurement_dimension)
            guarded = max(rms, expected) if finite(rms) else math.inf
            under_expected = max(0.0, expected - rms) if finite(rms) else expected
            writer.writerow(
                {
                    "candidate_id": key.candidate_id,
                    "split": key.split,
                    "stage": key.stage,
                    "log_parameter": key.log_parameter,
                    "measurement_dimension": key.measurement_dimension,
                    "count": stats.count,
                    "accepted_count": stats.accepted_count,
                    "rejected_count": stats.rejected_count,
                    "rejected_rate": format_number(stats.rejected_rate()),
                    "rejected_rate_penalty": format_number(stats.rejected_rate_penalty()),
                    "finite_count": stats.finite_count,
                    "nonfinite_count": stats.count - stats.finite_count,
                    "segment_count": len(stats.segment_ids),
                    "rms_nis": format_number(rms),
                    "sqrt_mean_nis": format_number(stats.sqrt_mean_nis()),
                    "accepted_only_rms_nis": format_number(stats.accepted_only_rms_nis()),
                    "accepted_only_sqrt_mean_nis": format_number(
                        stats.accepted_only_sqrt_mean_nis()
                    ),
                    "expected_rms_nis": format_number(expected),
                    "guarded_rms_nis": format_number(guarded),
                    "under_expected_penalty": format_number(under_expected),
                    "inflation_flag": (
                        "true" if finite(rms) and rms < expected * inflation_floor_ratio else "false"
                    ),
                    "physical_residual_rms": format_number(stats.physical_residual_rms()),
                    "parameter_field": key.parameter_field,
                    "parameter_value_kind": key.parameter_value_kind,
                    "parameter_value": key.parameter_value,
                    "launch_command_signature": key.launch_command_signature,
                    "score_partition": score_partition(key.stage),
                }
            )
    return path


def write_candidate_aggregate_csv(
    output_dir: Path,
    aggregates: dict[AggregateKey, NisAggregate],
) -> Path:
    path = output_dir / "candidate_rms_nis.csv"
    by_candidate_split: dict[tuple[str, str], NisAggregate] = {}
    for key, stats in aggregates.items():
        if key.split not in REPORT_SPLITS:
            continue
        combined = by_candidate_split.setdefault((key.candidate_id, key.split), NisAggregate())
        combined.count += stats.count
        combined.finite_count += stats.finite_count
        combined.sum_nis += stats.sum_nis
        combined.sum_nis_sq += stats.sum_nis_sq
        combined.accepted_count += stats.accepted_count
        combined.rejected_count += stats.rejected_count
        combined.accepted_sum_nis += stats.accepted_sum_nis
        combined.accepted_sum_nis_sq += stats.accepted_sum_nis_sq
        combined.segment_ids.update(stats.segment_ids)
        combined.residual_sum_square += stats.residual_sum_square
        combined.residual_count += stats.residual_count
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=[
                "candidate_id",
                "split",
                "count",
                "sum_nis",
                "sum_nis_sq",
                "rms_nis",
                "sqrt_mean_nis",
                "accepted_count",
                "rejected_count",
                "rejected_rate",
                "rejected_rate_penalty",
                "finite_count",
                "nonfinite_count",
                "accepted_only_sum_nis",
                "accepted_only_sum_nis_sq",
                "accepted_only_rms_nis",
                "accepted_only_sqrt_mean_nis",
                "physical_residual_rms",
                "segment_count",
            ],
        )
        writer.writeheader()
        for (candidate_id, split), stats in sorted(by_candidate_split.items()):
            writer.writerow(
                {
                    "candidate_id": candidate_id,
                    "split": split,
                    "count": stats.count,
                    "sum_nis": format_number(stats.sum_nis),
                    "sum_nis_sq": format_number(stats.sum_nis_sq),
                    "rms_nis": format_number(stats.rms_nis()),
                    "sqrt_mean_nis": format_number(stats.sqrt_mean_nis()),
                    "accepted_count": stats.accepted_count,
                    "rejected_count": stats.rejected_count,
                    "rejected_rate": format_number(stats.rejected_rate()),
                    "rejected_rate_penalty": format_number(stats.rejected_rate_penalty()),
                    "finite_count": stats.finite_count,
                    "nonfinite_count": stats.count - stats.finite_count,
                    "accepted_only_sum_nis": format_number(stats.accepted_sum_nis),
                    "accepted_only_sum_nis_sq": format_number(stats.accepted_sum_nis_sq),
                    "accepted_only_rms_nis": format_number(stats.accepted_only_rms_nis()),
                    "accepted_only_sqrt_mean_nis": format_number(
                        stats.accepted_only_sqrt_mean_nis()
                    ),
                    "physical_residual_rms": format_number(stats.physical_residual_rms()),
                    "segment_count": len(stats.segment_ids),
                }
            )
    return path


def validation_state_from_sample(sample: ReplaySample, vehicle: VehicleConfig) -> list[float]:
    state = [0.0 for _ in range(N)]
    left_v = sample.left_wheel_rate_radps * vehicle.wheel_radius_m
    right_v = sample.right_wheel_rate_radps * vehicle.wheel_radius_m
    state[VF] = finite_or(0.5 * (left_v + right_v), 0.0)
    if sample.gyro_valid and finite(sample.yaw_rate_radps):
        state[YAW_RATE] = sample.yaw_rate_radps
    else:
        state[YAW_RATE] = finite_or((right_v - left_v) / max(vehicle.track_width_m, 1.0e-9), 0.0)
    return state


def sigma_points_around_state(
    state: Sequence[float],
    covariance: CovarianceConfig,
    sigma_scale: float,
) -> list[list[float]]:
    scale = max(0.0, finite_or(sigma_scale, 1.0))
    points = [list(state)]
    for index, std in enumerate(covariance.initial_state_std):
        spread = abs(std) * scale
        if spread <= 0.0 or not finite(spread):
            continue
        plus = list(state)
        minus = list(state)
        plus[index] += spread
        minus[index] -= spread
        if index == HEADING:
            plus[index] = normalize_angle(plus[index])
            minus[index] = normalize_angle(minus[index])
        points.append(plus)
        points.append(minus)
    return points


def state_abs_delta(left: Sequence[float], right: Sequence[float]) -> float:
    result = 0.0
    for index, left_value in enumerate(left):
        delta = left_value - right[index]
        if index == HEADING:
            delta = normalize_angle(delta)
        result = max(result, abs(delta))
    return result


def max_abs(values: Iterable[float]) -> float:
    result = 0.0
    for value in values:
        if finite(value):
            result = max(result, abs(value))
        else:
            return math.inf
    return result


def mean_state(points: Sequence[Sequence[float]]) -> list[float]:
    if not points:
        return [math.nan for _ in range(N)]
    mean = [0.0 for _ in range(N)]
    for point in points:
        for index, value in enumerate(point):
            mean[index] += value
    return [value / len(points) for value in mean]


def covariance_from_sigma_points(
    points: Sequence[Sequence[float]],
    mean: Sequence[float],
) -> list[list[float]]:
    result = zeros(N, N)
    if not points:
        return result
    weight = 1.0 / len(points)
    for point in points:
        deltas = []
        for index, value in enumerate(point):
            delta = value - mean[index]
            if index == HEADING:
                delta = normalize_angle(delta)
            deltas.append(delta)
        for row in range(N):
            for col in range(N):
                result[row][col] += weight * deltas[row] * deltas[col]
    return result


def validation_measurements(
    plant: CandidatePlant,
    state: Sequence[float],
    sample: ReplaySample,
) -> tuple[dict[str, float], tuple[float, ...]]:
    plant_result = plant.plant_result(state, sample)
    predictions = {
        "yaw_rate_nis": state[YAW_RATE],
        "forward_accel_nis": plant_result.imu_forward_accel_mps2,
        "right_accel_nis": plant_result.imu_right_accel_mps2,
    }
    values = (
        predictions["yaw_rate_nis"],
        predictions["forward_accel_nis"],
        predictions["right_accel_nis"],
        plant_result.forward_accel_mps2,
        plant_result.right_accel_mps2,
        plant_result.yaw_accel_radps2,
        plant_result.max_contact_relative_speed_mps,
        plant_result.max_contact_utilization,
        plant_result.max_contact_saturation,
        plant_result.ground_use,
    )
    return predictions, values


def mean_prediction(predictions: Sequence[dict[str, float]], name: str) -> float:
    if not predictions:
        return math.nan
    return sum(item[name] for item in predictions) / len(predictions)


def prediction_variance(
    predictions: Sequence[dict[str, float]],
    name: str,
    mean: float,
) -> float:
    if not predictions:
        return math.nan
    return sum((item[name] - mean) ** 2 for item in predictions) / len(predictions)


def record_ukf_issue(
    stats: UkfValidationStats,
    events: list[dict[str, Any]],
    candidate: CandidateConfig,
    sample: ReplaySample,
    category: str,
    detail: str,
    metric: str = "",
    value: float = math.nan,
    threshold: float = math.nan,
) -> None:
    if category == "finite":
        stats.finite_failures += 1
    elif category == "continuity":
        stats.continuity_failures += 1
    elif category == "covariance":
        stats.covariance_failures += 1
    elif category == "innovation":
        stats.innovation_failures += 1
    elif category == "zero_crossing":
        stats.zero_crossing_failures += 1
    events.append(
        {
            "candidate_id": candidate.candidate_id,
            "segment_id": sample.segment_id,
            "stage": sample.stage,
            "run_id": sample.run_id,
            "source_path": str(sample.source_path),
            "source_row_index": sample.source_row_index,
            "master_time_us": sample.master_time_us,
            "category": category,
            "severity": "error",
            "detail": detail,
            "metric": metric,
            "value": value,
            "threshold": threshold,
        }
    )


def validate_zero_crossing_probe(
    candidate: CandidateConfig,
    plant: CandidatePlant,
    sample: ReplaySample,
    stats: UkfValidationStats,
    events: list[dict[str, Any]],
    base_state: Sequence[float],
    state_index: int,
    label: str,
    spread: float,
    epsilon: float,
    jump_limit: float,
) -> None:
    if base_state[state_index] - spread > 0.0 or base_state[state_index] + spread < 0.0:
        return
    stats.zero_crossing_events += 1
    plus = list(base_state)
    minus = list(base_state)
    plus[state_index] = abs(epsilon)
    minus[state_index] = -abs(epsilon)
    try:
        plus_state = plant.propagate(plus, sample, sample.dt_s)
        minus_state = plant.propagate(minus, sample, sample.dt_s)
        plus_predictions, plus_values = validation_measurements(plant, plus_state, sample)
        minus_predictions, minus_values = validation_measurements(plant, minus_state, sample)
    except (ArithmeticError, OverflowError, ValueError) as exc:
        record_ukf_issue(
            stats,
            events,
            candidate,
            sample,
            "zero_crossing",
            f"{label} zero-crossing probe raised {type(exc).__name__}: {exc}",
            label,
        )
        return
    if not all(finite(value) for value in [*plus_state, *minus_state, *plus_values, *minus_values]):
        record_ukf_issue(
            stats,
            events,
            candidate,
            sample,
            "zero_crossing",
            f"{label} zero-crossing probe produced non-finite output",
            label,
        )
        return
    jump = max(
        state_abs_delta(plus_state, minus_state),
        max(abs(plus_predictions[name] - minus_predictions[name]) for name in plus_predictions),
    )
    if jump > jump_limit:
        record_ukf_issue(
            stats,
            events,
            candidate,
            sample,
            "zero_crossing",
            f"{label} zero-crossing response exceeded continuity limit",
            label,
            jump,
            jump_limit,
        )


def validate_ukf_sample(
    candidate: CandidateConfig,
    plant: CandidatePlant,
    vehicle: VehicleConfig,
    covariance: CovarianceConfig,
    sample: ReplaySample,
    stats: UkfValidationStats,
    events: list[dict[str, Any]],
    sigma_scale: float,
    zero_vf_epsilon_mps: float,
    zero_yaw_epsilon_radps: float,
) -> None:
    thresholds = ukf_validation_thresholds()
    stats.sample_count += 1
    stats.segment_ids.add(sample.segment_id)
    base_state = validation_state_from_sample(sample, vehicle)
    sigma_points = sigma_points_around_state(base_state, covariance, sigma_scale)
    stats.sigma_point_count += len(sigma_points)
    propagated: list[list[float]] = []
    predictions: list[dict[str, float]] = []
    prediction_values: list[tuple[float, ...]] = []
    try:
        central_state = plant.propagate(base_state, sample, sample.dt_s)
        central_predictions, central_values = validation_measurements(plant, central_state, sample)
    except (ArithmeticError, OverflowError, ValueError) as exc:
        record_ukf_issue(
            stats,
            events,
            candidate,
            sample,
            "finite",
            f"central propagation raised {type(exc).__name__}: {exc}",
        )
        return

    for sigma_index, sigma_state in enumerate(sigma_points):
        try:
            propagated_state = plant.propagate(sigma_state, sample, sample.dt_s)
            measurement_predictions, values = validation_measurements(plant, propagated_state, sample)
        except (ArithmeticError, OverflowError, ValueError) as exc:
            record_ukf_issue(
                stats,
                events,
                candidate,
                sample,
                "finite",
                f"sigma point {sigma_index} raised {type(exc).__name__}: {exc}",
            )
            continue
        propagated.append(propagated_state)
        predictions.append(measurement_predictions)
        prediction_values.append(values)

    if len(propagated) != len(sigma_points):
        return
    flat_state_values = [value for state in propagated for value in state]
    flat_prediction_values = [value for values in prediction_values for value in values]
    if not all(finite(value) for value in [*central_state, *central_values, *flat_state_values, *flat_prediction_values]):
        record_ukf_issue(
            stats,
            events,
            candidate,
            sample,
            "finite",
            "sigma propagation produced non-finite state or plant output",
        )
        return

    stats.max_abs_state = max(stats.max_abs_state, max_abs([*central_state, *flat_state_values]))
    stats.max_abs_prediction = max(stats.max_abs_prediction, max_abs([*central_values, *flat_prediction_values]))
    if stats.max_abs_state > thresholds["state_abs_limit"]:
        record_ukf_issue(
            stats,
            events,
            candidate,
            sample,
            "continuity",
            "absolute propagated state exceeded validation limit",
            "max_abs_state",
            stats.max_abs_state,
            thresholds["state_abs_limit"],
        )
    if stats.max_abs_prediction > thresholds["prediction_abs_limit"]:
        record_ukf_issue(
            stats,
            events,
            candidate,
            sample,
            "continuity",
            "absolute plant prediction exceeded validation limit",
            "max_abs_prediction",
            stats.max_abs_prediction,
            thresholds["prediction_abs_limit"],
        )

    state_span = max(state_abs_delta(state, central_state) for state in propagated)
    measurement_span = max(
        abs(prediction[name] - central_predictions[name])
        for prediction in predictions
        for name in central_predictions
    )
    stats.max_state_sigma_span = max(stats.max_state_sigma_span, state_span)
    stats.max_measurement_sigma_span = max(stats.max_measurement_sigma_span, measurement_span)
    if state_span > thresholds["state_sigma_span_limit"]:
        record_ukf_issue(
            stats,
            events,
            candidate,
            sample,
            "continuity",
            "sigma state span exceeded validation limit",
            "max_state_sigma_span",
            state_span,
            thresholds["state_sigma_span_limit"],
        )
    if measurement_span > thresholds["measurement_sigma_span_limit"]:
        record_ukf_issue(
            stats,
            events,
            candidate,
            sample,
            "continuity",
            "sigma measurement span exceeded validation limit",
            "max_measurement_sigma_span",
            measurement_span,
            thresholds["measurement_sigma_span_limit"],
        )

    propagated_mean = mean_state(propagated)
    sample_covariance = covariance_from_sigma_points(propagated, propagated_mean)
    try:
        process_noise = EkfReplay(plant, covariance).process_noise(base_state, sample, sample.dt_s)
        predicted_covariance = symmetrize(add_matrix(sample_covariance, process_noise))
    except (ArithmeticError, OverflowError, ValueError) as exc:
        record_ukf_issue(
            stats,
            events,
            candidate,
            sample,
            "covariance",
            f"process covariance calculation raised {type(exc).__name__}: {exc}",
        )
        return
    covariance_diag = [predicted_covariance[index][index] for index in range(N)]
    covariance_trace = sum(covariance_diag)
    stats.max_covariance_trace = max(stats.max_covariance_trace, covariance_trace)
    if (
        not finite(covariance_trace)
        or not all(finite(value) for value in covariance_diag)
        or min(covariance_diag) < -1.0e-10
        or covariance_trace > thresholds["covariance_trace_limit"]
    ):
        record_ukf_issue(
            stats,
            events,
            candidate,
            sample,
            "covariance",
            "predicted covariance diagonal or trace failed sanity checks",
            "covariance_trace",
            covariance_trace,
            thresholds["covariance_trace_limit"],
        )

    measurement_noise = {
        "yaw_rate_nis": covariance.measurement.yaw_rate_sigma_radps**2,
        "forward_accel_nis": covariance.measurement.accel_sigma_mps2**2,
        "right_accel_nis": covariance.measurement.accel_sigma_mps2**2,
    }
    measurements = {
        "yaw_rate_nis": (sample.yaw_rate_radps, sample.gyro_valid),
        "forward_accel_nis": (sample.accel_forward_mps2, sample.accel_valid),
        "right_accel_nis": (sample.accel_right_mps2, sample.accel_valid),
    }
    for name, (measurement, valid_measurement) in measurements.items():
        mean = mean_prediction(predictions, name)
        variance = prediction_variance(predictions, name, mean) + measurement_noise[name]
        if not finite(mean) or not finite(variance) or variance <= 0.0:
            record_ukf_issue(
                stats,
                events,
                candidate,
                sample,
                "innovation",
                "innovation mean or variance failed sanity checks",
                name,
                variance,
            )
            continue
        if valid_measurement and finite(measurement):
            nis = ((measurement - mean) ** 2) / variance
            if finite(nis):
                stats.max_innovation_nis = max(stats.max_innovation_nis, nis)
            else:
                record_ukf_issue(
                    stats,
                    events,
                    candidate,
                    sample,
                    "innovation",
                    "innovation NIS was non-finite",
                    name,
                )

    spreads = [abs(std) * max(0.0, finite_or(sigma_scale, 1.0)) for std in covariance.initial_state_std]
    validate_zero_crossing_probe(
        candidate,
        plant,
        sample,
        stats,
        events,
        base_state,
        VF,
        "vf_mps",
        spreads[VF],
        zero_vf_epsilon_mps,
        thresholds["zero_crossing_jump_limit"],
    )
    validate_zero_crossing_probe(
        candidate,
        plant,
        sample,
        stats,
        events,
        base_state,
        YAW_RATE,
        "yaw_rate_radps",
        spreads[YAW_RATE],
        zero_yaw_epsilon_radps,
        thresholds["zero_crossing_jump_limit"],
    )


def ukf_validation_thresholds() -> dict[str, float]:
    return {
        "state_abs_limit": 1.0e6,
        "prediction_abs_limit": 1.0e7,
        "state_sigma_span_limit": 1.0e5,
        "measurement_sigma_span_limit": 1.0e6,
        "covariance_trace_limit": 1.0e10,
        "zero_crossing_jump_limit": 1.0e5,
    }


def write_ukf_candidate_summary(output_dir: Path, stats_by_candidate: dict[str, UkfValidationStats]) -> Path:
    path = output_dir / "ukf_validation_candidate_summary.csv"
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=UKF_VALIDATION_CANDIDATE_FIELDS)
        writer.writeheader()
        for candidate_id, stats in sorted(stats_by_candidate.items()):
            writer.writerow(
                {
                    "candidate_id": candidate_id,
                    "status": "pass" if stats.passed() else "fail",
                    "segments": len(stats.segment_ids),
                    "samples": stats.sample_count,
                    "sigma_points": stats.sigma_point_count,
                    "finite_failures": stats.finite_failures,
                    "continuity_failures": stats.continuity_failures,
                    "covariance_failures": stats.covariance_failures,
                    "innovation_failures": stats.innovation_failures,
                    "zero_crossing_events": stats.zero_crossing_events,
                    "zero_crossing_failures": stats.zero_crossing_failures,
                    "max_abs_state": format_number(stats.max_abs_state),
                    "max_abs_prediction": format_number(stats.max_abs_prediction),
                    "max_state_sigma_span": format_number(stats.max_state_sigma_span),
                    "max_measurement_sigma_span": format_number(stats.max_measurement_sigma_span),
                    "max_covariance_trace": format_number(stats.max_covariance_trace),
                    "max_innovation_nis": format_number(stats.max_innovation_nis),
                    "issue_count": stats.issue_count(),
                }
            )
    return path


def write_ukf_validation_events(output_dir: Path, events: Sequence[dict[str, Any]]) -> Path:
    path = output_dir / "ukf_validation_events.csv"
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=UKF_VALIDATION_EVENT_FIELDS)
        writer.writeheader()
        for event in events:
            row = dict(event)
            row["value"] = format_number(float(row["value"])) if finite_or(float(row["value"]), math.nan) == row["value"] else ""
            row["threshold"] = (
                format_number(float(row["threshold"]))
                if finite_or(float(row["threshold"]), math.nan) == row["threshold"]
                else ""
            )
            writer.writerow(row)
    return path


def write_ukf_validation_report(output_dir: Path, summary: dict[str, Any]) -> Path:
    path = output_dir / "ukf_validation_report.md"
    lines = [
        "# UKF Sigma-Point Validation Report",
        "",
        f"- Validation kind: `{summary['validation_kind']}`",
        f"- Segment manifest: `{summary.get('segment_manifest', '')}`",
        f"- Bias source manifest: `{summary.get('bias_segment_manifest', '')}`",
        f"- Processed segments: `{summary['processed_segments']}`",
        f"- Source logs: `{summary['source_log_count']}`",
        f"- Representative row samples: `{summary['processed_samples']}`",
        f"- Sigma policy: `{summary['sigma_policy']}`",
        f"- Uses logged UKF state: `{str(summary['uses_logged_ukf_state']).lower()}`",
        f"- Bias summary CSV: `{summary.get('bias_summary_csv', '')}`",
        f"- Overall status: `{'pass' if summary['passed'] else 'fail'}`",
        "",
        "| Candidate | Status | Samples | Sigma points | Issues | Zero-crossing probes | Max prediction | Max covariance trace | Max innovation NIS |",
        "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for candidate_id, stats in sorted(dict(summary.get("candidates", {})).items()):
        lines.append(
            "| "
            f"`{candidate_id}` | "
            f"{stats['status']} | "
            f"{stats['samples']} | "
            f"{stats['sigma_points']} | "
            f"{stats['issue_count']} | "
            f"{stats['zero_crossing_events']} | "
            f"{format_number(float(stats['max_abs_prediction']))} | "
            f"{format_number(float(stats['max_covariance_trace']))} | "
            f"{format_number(float(stats['max_innovation_nis']))} |"
        )
    if summary.get("events"):
        lines.extend(
            [
                "",
                "## Issues",
                "",
                "| Candidate | Segment | Row | Category | Detail |",
                "| --- | --- | ---: | --- | --- |",
            ]
        )
        for event in summary["events"][:20]:
            lines.append(
                "| "
                f"`{event['candidate_id']}` | "
                f"`{event['segment_id']}` | "
                f"{event['source_row_index']} | "
                f"{event['category']} | "
                f"{event['detail']} |"
            )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


def run_ukf_validation(
    candidates: Sequence[CandidateConfig],
    vehicle: VehicleConfig,
    covariance: CovarianceConfig,
    segments: Sequence[SegmentSpec],
    output_dir: Path,
    bias_segments: Sequence[SegmentSpec] | None = None,
    max_segments: int = 0,
    max_rows_per_segment: int = 0,
    include_corrupted: bool = False,
    sigma_scale: float = 1.0,
    zero_vf_epsilon_mps: float = 0.02,
    zero_yaw_epsilon_radps: float = 0.05,
    segment_manifest: Path | None = None,
    bias_segment_manifest: Path | None = None,
) -> dict[str, Any]:
    output_dir.mkdir(parents=True, exist_ok=True)
    selected_segments, skipped_corrupted_segments = selected_replay_segments(
        segments,
        max_segments,
        include_corrupted,
    )
    stats_by_candidate = {
        candidate.candidate_id: UkfValidationStats(candidate.candidate_id)
        for candidate in candidates
    }
    bias_groups = {
        path: grouped
        for path, grouped in group_segments_by_log(bias_segments if bias_segments is not None else segments)
    }
    bias_results: list[LogReplayResult] = []
    samples_by_segment: dict[tuple[Path, str, int, int], list[ReplaySample]] = {}
    for log_path, log_segments in group_segments_by_log(selected_segments):
        estimate = estimate_sensor_bias_for_log(
            log_path,
            bias_groups.get(log_path, log_segments),
            vehicle,
        )
        bias_results.append(
            LogReplayResult(
                accumulators={},
                aggregates={},
                bias_summary=estimate,
                log_path=log_path,
                log_id=segment_log_id(log_path, log_segments),
            )
        )
        log_samples = read_representative_samples_by_log(
            log_path,
            log_segments,
            vehicle,
            max_rows_per_segment,
        )
        accel_bias = estimate.accel
        for segment in log_segments:
            samples_by_segment[segment_sample_key(segment)] = [
                replace_sample_accel_bias(sample, accel_bias)
                for sample in log_samples.get(segment_sample_key(segment), [])
            ]
    bias_summary_path = write_bias_summary_csv(output_dir, bias_results)
    events: list[dict[str, Any]] = []
    processed_samples = 0
    candidate_plants = {
        candidate.candidate_id: CandidatePlant(vehicle, candidate)
        for candidate in candidates
    }
    for segment in selected_segments:
        samples = samples_by_segment.get(segment_sample_key(segment), [])
        processed_samples += len(samples)
        for candidate in candidates:
            plant = candidate_plants[candidate.candidate_id]
            stats = stats_by_candidate[candidate.candidate_id]
            for sample in samples:
                validate_ukf_sample(
                    candidate,
                    plant,
                    vehicle,
                    covariance,
                    sample,
                    stats,
                    events,
                    sigma_scale,
                    zero_vf_epsilon_mps,
                    zero_yaw_epsilon_radps,
                )

    candidate_summary_path = write_ukf_candidate_summary(output_dir, stats_by_candidate)
    events_path = write_ukf_validation_events(output_dir, events)
    summary = {
        "schema_version": 1,
        "validation_kind": "diagonal_sigma_point_candidate_plant",
        "segment_manifest": str(segment_manifest) if segment_manifest is not None else "",
        "processed_segments": len(selected_segments),
        "processed_samples": processed_samples,
        "skipped_corrupted_segments": skipped_corrupted_segments,
        "source_log_count": len({segment.log_path for segment in selected_segments}),
        "max_rows_per_segment": max_rows_per_segment,
        "sigma_scale": sigma_scale,
        "sigma_policy": "2N+1 diagonal sigma points from fixed testbed covariance",
        "uses_logged_ukf_state": False,
        "candidate_summary_csv": str(candidate_summary_path),
        "events_csv": str(events_path),
        "bias_summary_csv": str(bias_summary_path),
        "bias_segment_manifest": str(bias_segment_manifest) if bias_segment_manifest is not None else "",
        "thresholds": ukf_validation_thresholds(),
        "passed": all(stats.passed() for stats in stats_by_candidate.values()),
        "candidates": {
            candidate_id: {
                "status": "pass" if stats.passed() else "fail",
                "segments": len(stats.segment_ids),
                "samples": stats.sample_count,
                "sigma_points": stats.sigma_point_count,
                "finite_failures": stats.finite_failures,
                "continuity_failures": stats.continuity_failures,
                "covariance_failures": stats.covariance_failures,
                "innovation_failures": stats.innovation_failures,
                "zero_crossing_events": stats.zero_crossing_events,
                "zero_crossing_failures": stats.zero_crossing_failures,
                "max_abs_state": stats.max_abs_state,
                "max_abs_prediction": stats.max_abs_prediction,
                "max_state_sigma_span": stats.max_state_sigma_span,
                "max_measurement_sigma_span": stats.max_measurement_sigma_span,
                "max_covariance_trace": stats.max_covariance_trace,
                "max_innovation_nis": stats.max_innovation_nis,
                "issue_count": stats.issue_count(),
            }
            for candidate_id, stats in sorted(stats_by_candidate.items())
        },
        "events": events[:50],
    }
    report_path = write_ukf_validation_report(output_dir, summary)
    summary["report_md"] = str(report_path)
    (output_dir / "ukf_validation_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True, default=str) + "\n",
        encoding="utf-8",
    )
    return summary


def write_replay_report(output_dir: Path, summary: dict[str, Any]) -> Path:
    path = output_dir / "report.md"
    lines = [
        "# Full Traction ANIS Replay Report",
        "",
        f"- Replay mode: `{summary['replay_mode']}`",
        f"- Processed non-corrupted segments: `{summary['processed_segments']}`",
        f"- Skipped corrupted segments: `{summary['skipped_corrupted_segments']}`",
        f"- Source logs: `{summary.get('source_log_count', 0)}`",
        f"- Jobs used: `{summary.get('jobs_used', 1)}`",
        f"- Segment-row samples processed: `{summary['processed_samples']}`",
        f"- Row artifacts enabled: `{str(summary['row_artifacts_enabled']).lower()}`",
        f"- Uses logged UKF state: `{str(summary['uses_logged_ukf_state']).lower()}`",
        f"- Bias source manifest: `{summary.get('bias_segment_manifest', '')}`",
        f"- Bias summary CSV: `{summary.get('bias_summary_csv', '')}`",
        "",
        "| Candidate | All-finite RMS NIS | sqrt(mean finite NIS) | Finite | Accepted-only RMS | NIS count | Accepted | Rejected | Rejected rate | Segments |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    candidates = dict(summary.get("candidates", {}))
    for candidate_id, stats in sorted(candidates.items()):
        lines.append(
            "| "
            f"`{candidate_id}` | "
            f"{format_number(float(stats.get('rms_nis', math.nan)))} | "
            f"{format_number(float(stats.get('sqrt_mean_nis', math.nan)))} | "
            f"{stats.get('finite_count', 0)} | "
            f"{format_number(float(stats.get('accepted_only_rms_nis', math.nan)))} | "
            f"{stats.get('nis_count', 0)} | "
            f"{stats.get('accepted_count', 0)} | "
            f"{stats.get('rejected_count', 0)} | "
            f"{format_number(float(stats.get('rejected_rate', 0.0)))} | "
            f"{stats.get('segment_count', 0)} |"
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


@dataclass
class SegmentRuntime:
    segment: SegmentSpec
    candidate_runtime: dict[str, tuple[CandidatePlant, EkfReplay | None, list[float]]]
    aggregate_keys: dict[tuple[str, str, int], tuple[AggregateKey, ...]]
    previous_time_us: int | None = None
    previous_yaw_rate_radps: float | None = None
    previous_sample: ReplaySample | None = None
    emitted_samples: int = 0


@dataclass
class LogReplayResult:
    accumulators: dict[str, CandidateAccumulator]
    aggregates: dict[AggregateKey, NisAggregate]
    processed_samples: int = 0
    bias_summary: SensorBiasEstimate = field(default_factory=SensorBiasEstimate)
    log_path: Path | None = None
    log_id: str = ""


def selected_replay_segments(
    segments: Sequence[SegmentSpec],
    max_segments: int,
    include_corrupted: bool,
) -> tuple[list[SegmentSpec], int]:
    selected: list[SegmentSpec] = []
    skipped_corrupted = 0
    for segment in segments:
        if segment.corrupted and not include_corrupted:
            skipped_corrupted += 1
            continue
        if max_segments > 0 and len(selected) >= max_segments:
            break
        selected.append(segment)
    return selected, skipped_corrupted


def group_segments_by_log(segments: Sequence[SegmentSpec]) -> list[tuple[Path, list[SegmentSpec]]]:
    grouped: dict[Path, list[SegmentSpec]] = {}
    order: list[Path] = []
    for segment in segments:
        if segment.log_path not in grouped:
            grouped[segment.log_path] = []
            order.append(segment.log_path)
        grouped[segment.log_path].append(segment)
    return [(path, grouped[path]) for path in order]


def segment_log_id(log_path: Path, segments: Sequence[SegmentSpec]) -> str:
    for segment in segments:
        if segment.run_id:
            return segment.run_id
    return log_path.parent.name


def write_bias_summary_csv(
    output_dir: Path,
    results: Sequence[LogReplayResult],
) -> Path:
    path = output_dir / "bias_summary.csv"
    rows = [
        result.bias_summary.row(
            result.log_path or Path(""),
            result.log_id,
        )
        for result in results
    ]
    rows.sort(key=lambda row: (row["log_id"], row["log_path"]))
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=BIAS_SUMMARY_FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    return path


def make_candidate_runtime(
    candidates: Sequence[CandidateConfig],
    vehicle: VehicleConfig,
    covariance: CovarianceConfig,
    replay_mode: str,
    candidate_plants: dict[str, CandidatePlant] | None = None,
) -> dict[str, tuple[CandidatePlant, EkfReplay | None, list[float]]]:
    runtime: dict[str, tuple[CandidatePlant, EkfReplay | None, list[float]]] = {}
    for candidate in candidates:
        plant = (
            candidate_plants.get(candidate.candidate_id)
            if candidate_plants is not None
            else None
        )
        if plant is None:
            plant = CandidatePlant(vehicle, candidate)
        runtime[candidate.candidate_id] = (
            plant,
            EkfReplay(plant, covariance) if replay_mode == "ekf" else None,
            [0.0 for _ in range(N)],
        )
    return runtime


def merge_candidate_accumulator(target: CandidateAccumulator, source: CandidateAccumulator) -> None:
    target.nis_count += source.nis_count
    target.finite_count += source.finite_count
    target.nis_sum += source.nis_sum
    target.nis_sum_square += source.nis_sum_square
    target.accepted_count += source.accepted_count
    target.rejected_count += source.rejected_count
    target.accepted_nis_sum += source.accepted_nis_sum
    target.accepted_nis_sum_square += source.accepted_nis_sum_square
    target.segment_ids.update(source.segment_ids)
    for key, value in source.residual_sum_square.items():
        target.residual_sum_square[key] = target.residual_sum_square.get(key, 0.0) + value
    for key, value in source.residual_count.items():
        target.residual_count[key] = target.residual_count.get(key, 0) + value


def merge_nis_aggregate(target: NisAggregate, source: NisAggregate) -> None:
    target.count += source.count
    target.finite_count += source.finite_count
    target.sum_nis += source.sum_nis
    target.sum_nis_sq += source.sum_nis_sq
    target.accepted_count += source.accepted_count
    target.rejected_count += source.rejected_count
    target.accepted_sum_nis += source.accepted_sum_nis
    target.accepted_sum_nis_sq += source.accepted_sum_nis_sq
    target.segment_ids.update(source.segment_ids)
    target.residual_sum_square += source.residual_sum_square
    target.residual_count += source.residual_count


def merge_log_result(
    accumulators: dict[str, CandidateAccumulator],
    aggregates: dict[AggregateKey, NisAggregate],
    result: LogReplayResult,
) -> int:
    for candidate_id, source in result.accumulators.items():
        merge_candidate_accumulator(accumulators.setdefault(candidate_id, CandidateAccumulator()), source)
    for key, source in result.aggregates.items():
        merge_nis_aggregate(aggregates.setdefault(key, NisAggregate()), source)
    return result.processed_samples


def add_precomputed_aggregate_update(
    aggregates: dict[AggregateKey, NisAggregate],
    aggregate_keys: dict[tuple[str, str, int], tuple[AggregateKey, ...]],
    candidate: CandidateConfig,
    segment: SegmentSpec,
    sample: ReplaySample,
    update: UpdateResult,
) -> None:
    keys = aggregate_keys.get(
        (candidate.candidate_id, update.log_parameter, update.measurement_dimension)
    )
    if not keys:
        add_aggregate_update(aggregates, candidate, segment, sample, update)
        return
    for key in keys:
        aggregates.setdefault(key, NisAggregate()).add(sample.segment_id, update)


def add_scalar_residual_aggregate(
    accumulators: dict[str, CandidateAccumulator],
    aggregates: dict[AggregateKey, NisAggregate],
    aggregate_keys: dict[tuple[str, str, int], tuple[AggregateKey, ...]],
    candidate: CandidateConfig,
    segment: SegmentSpec,
    sample: ReplaySample,
    log_parameter: str,
    measurement: float,
    prediction: float,
    variance: float,
    gate_threshold: float,
    valid: bool,
) -> None:
    gate_threshold = effective_gate_threshold(log_parameter, gate_threshold)
    if not valid or not finite(measurement):
        return
    innovation_variance = max(variance, 1.0e-12)
    if not finite(prediction):
        innovation = math.nan
        nis = math.nan
        accepted = False
    else:
        innovation = measurement - prediction
        try:
            nis = (innovation * innovation) / innovation_variance
        except (ArithmeticError, OverflowError, ValueError):
            nis = math.nan
            accepted = False
        else:
            accepted = finite(nis) and not (math.isfinite(gate_threshold) and nis > gate_threshold)
    accumulators[candidate.candidate_id].add_values(
        log_parameter,
        nis,
        accepted,
        innovation,
        sample.segment_id,
    )
    keys = aggregate_keys.get((candidate.candidate_id, log_parameter, 1))
    if keys:
        for key in keys:
            aggregates.setdefault(key, NisAggregate()).add_values(
                sample.segment_id,
                nis,
                accepted,
                innovation,
            )
        return
    update = UpdateResult(
        log_parameter=log_parameter,
        measurement_dimension=1,
        nis=nis,
        accepted=accepted,
        gate_threshold=gate_threshold,
        innovation=innovation,
        innovation_variance=innovation_variance,
        measurement=measurement,
        prediction=prediction,
        metric_kind=metric_kind_for_log_parameter(log_parameter),
    )
    add_aggregate_update(aggregates, candidate, segment, sample, update)


def process_replay_log(
    log_path: Path,
    log_segments: Sequence[SegmentSpec],
    bias_segments: Sequence[SegmentSpec],
    candidates: Sequence[CandidateConfig],
    vehicle: VehicleConfig,
    covariance: CovarianceConfig,
    replay_mode: str,
    max_rows_per_segment: int,
    nis_writer: csv.DictWriter | None = None,
    diagnostic_writer: csv.DictWriter | None = None,
) -> LogReplayResult:
    if not log_path.exists():
        raise ConfigError(f"Input CSV not found: {log_path}")
    accumulators = {candidate.candidate_id: CandidateAccumulator() for candidate in candidates}
    aggregates: dict[AggregateKey, NisAggregate] = {}
    processed_samples = 0
    measurement = covariance.measurement
    yaw_variance = measurement.yaw_rate_sigma_radps**2
    accel_variance = measurement.accel_sigma_mps2**2
    bias_summary = estimate_sensor_bias_for_log(log_path, bias_segments, vehicle)
    accel_bias = bias_summary.accel
    sorted_segments = sorted(log_segments, key=lambda item: item.start_row_index)
    pending = deque(sorted_segments)
    active: list[SegmentRuntime] = []
    candidate_plants = {
        candidate.candidate_id: CandidatePlant(vehicle, candidate)
        for candidate in candidates
    }

    with log_path.open(newline="", encoding="utf-8-sig") as handle:
        reader = csv.reader(handle)
        fieldnames = next(reader, None)
        if not fieldnames:
            raise ConfigError(f"CSV header missing: {log_path}")
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
                            replay_mode,
                            candidate_plants,
                        ),
                        aggregate_keys=precomputed_aggregate_keys(segment, candidates),
                    )
                )

            if not active:
                if not pending:
                    break
                continue

            still_active: list[SegmentRuntime] = []
            for runtime in active:
                segment = runtime.segment
                segment_has_row = segment.end_row_index < 0 or row_index <= segment.end_row_index
                segment_under_limit = (
                    max_rows_per_segment <= 0
                    or runtime.emitted_samples < max_rows_per_segment
                )
                if segment_has_row and segment_under_limit:
                    sample = sample_from_bound_row(
                        row,
                        columns,
                        segment,
                        row_index,
                        runtime.previous_time_us,
                        vehicle,
                        runtime.previous_sample,
                    )
                    sample = replace_sample_accel_bias(sample, accel_bias)
                    measured_yaw_accel = measured_yaw_accel_from_previous(
                        runtime.previous_yaw_rate_radps,
                        sample,
                    )
                    runtime.previous_time_us = sample.master_time_us
                    runtime.emitted_samples += 1
                    processed_samples += 1
                    for candidate in candidates:
                        plant, replay, residual_state = runtime.candidate_runtime[
                            candidate.candidate_id
                        ]
                        try:
                            if replay is not None:
                                plant_before_updates = replay.predict(sample)
                                updates = [
                                    replay.update_yaw_rate(sample),
                                    replay.update_accel_forward(sample),
                                    replay.update_accel_right(sample),
                                ]
                                replay.apply_stationary_zlock(sample)
                                state = replay.state
                                plant_after_updates = (
                                    replay.plant.plant_result(state, sample)
                                    if diagnostic_writer is not None
                                    else plant_before_updates
                                )
                            else:
                                residual_state = plant.propagate(
                                    residual_state,
                                    sample,
                                    sample.dt_s,
                                )
                                runtime.candidate_runtime[candidate.candidate_id] = (
                                    plant,
                                    replay,
                                    residual_state,
                                )
                                state = residual_state
                                plant_after_updates = plant.plant_result(state, sample)
                                plant_before_updates = plant_after_updates
                                if nis_writer is None and diagnostic_writer is None:
                                    add_scalar_residual_aggregate(
                                        accumulators,
                                        aggregates,
                                        runtime.aggregate_keys,
                                        candidate,
                                        segment,
                                        sample,
                                        "yaw_rate_residual_tail",
                                        sample.yaw_rate_radps,
                                        state[YAW_RATE],
                                        yaw_variance,
                                        measurement.yaw_rate_gate_nis,
                                        sample.gyro_valid,
                                    )
                                    add_scalar_residual_aggregate(
                                        accumulators,
                                        aggregates,
                                        runtime.aggregate_keys,
                                        candidate,
                                        segment,
                                        sample,
                                        "forward_accel_residual_tail",
                                        sample.accel_forward_mps2,
                                        plant_after_updates.imu_forward_accel_mps2,
                                        accel_variance,
                                        measurement.accel_gate_nis,
                                        sample.accel_valid,
                                    )
                                    add_scalar_residual_aggregate(
                                        accumulators,
                                        aggregates,
                                        runtime.aggregate_keys,
                                        candidate,
                                        segment,
                                        sample,
                                        "right_accel_residual_tail",
                                        sample.accel_right_mps2,
                                        plant_after_updates.imu_right_accel_mps2,
                                        accel_variance,
                                        measurement.accel_gate_nis,
                                        sample.accel_valid,
                                    )
                                    accumulators[candidate.candidate_id].add_physical_residual(
                                        "yaw_accel_radps2",
                                        measured_yaw_accel - plant_after_updates.yaw_accel_radps2,
                                    )
                                    continue
                                updates = residual_tail_updates(
                                    sample,
                                    state,
                                    plant_after_updates,
                                    covariance,
                                    vehicle,
                                )
                        except (ArithmeticError, OverflowError, ValueError):
                            if replay is None:
                                runtime.candidate_runtime[candidate.candidate_id] = (
                                    plant,
                                    replay,
                                    [math.nan for _ in range(N)],
                                )
                            updates = invalid_residual_tail_updates(sample, covariance)
                        for update in updates:
                            if update is None:
                                continue
                            accumulators[candidate.candidate_id].add_update(
                                update,
                                sample.segment_id,
                            )
                            add_precomputed_aggregate_update(
                                aggregates,
                                runtime.aggregate_keys,
                                candidate,
                                segment,
                                sample,
                                update,
                            )
                            if nis_writer is not None:
                                nis_writer.writerow(nis_row(candidate, sample, update))
                        accumulators[candidate.candidate_id].add_physical_residual(
                            "yaw_accel_radps2",
                            measured_yaw_accel - plant_after_updates.yaw_accel_radps2,
                        )
                        if diagnostic_writer is not None:
                            diagnostic_writer.writerow(
                                diagnostic_row(
                                    candidate,
                                    sample,
                                    state,
                                    plant_after_updates,
                                    plant_before_updates,
                                    measured_yaw_accel,
                                    vehicle,
                                )
                            )
                    runtime.previous_yaw_rate_radps = (
                        sample.yaw_rate_radps if sample.gyro_valid else runtime.previous_yaw_rate_radps
                    )
                    runtime.previous_sample = sample
                reached_row_limit = (
                    max_rows_per_segment > 0
                    and runtime.emitted_samples >= max_rows_per_segment
                )
                reached_end = segment.end_row_index >= 0 and row_index >= segment.end_row_index
                if not reached_row_limit and not reached_end:
                    still_active.append(runtime)
            active = still_active

            if not pending and not active:
                break
    if pending:
        missing = ", ".join(segment.segment_id for segment in pending)
        raise ConfigError(f"Segment start row not found before end of log {log_path}: {missing}")
    unfinished = [runtime for runtime in active if runtime.segment.end_row_index >= 0]
    if unfinished:
        missing = ", ".join(runtime.segment.segment_id for runtime in unfinished)
        raise ConfigError(f"Segment extends beyond decoded log rows {log_path}: {missing}")
    return LogReplayResult(
        accumulators=accumulators,
        aggregates=aggregates,
        processed_samples=processed_samples,
        bias_summary=bias_summary,
        log_path=log_path,
        log_id=segment_log_id(log_path, log_segments),
    )


def run_replay(
    candidates: Sequence[CandidateConfig],
    vehicle: VehicleConfig,
    covariance: CovarianceConfig,
    segments: Sequence[SegmentSpec],
    output_dir: Path,
    bias_segments: Sequence[SegmentSpec] | None = None,
    max_segments: int = 0,
    max_rows_per_segment: int = 0,
    include_corrupted: bool = False,
    replay_mode: str = "ekf",
    write_row_artifacts: bool = False,
    jobs: int = 1,
    bias_segment_manifest: Path | None = None,
) -> dict[str, Any]:
    if replay_mode not in ("ekf", "residual"):
        raise ConfigError(f"Unsupported replay mode: {replay_mode}")
    output_dir.mkdir(parents=True, exist_ok=True)
    nis_path = output_dir / "nis_samples.csv"
    diagnostics_path = output_dir / "residual_diagnostics.csv"
    aggregate_path = output_dir / "nis_aggregates.csv"
    itemized_path = output_dir / "itemized_rms_nis.csv"
    candidate_rms_path = output_dir / "candidate_rms_nis.csv"
    accumulators = {candidate.candidate_id: CandidateAccumulator() for candidate in candidates}
    aggregates: dict[AggregateKey, NisAggregate] = {}
    selected_segments, skipped_corrupted_segments = selected_replay_segments(
        segments,
        max_segments,
        include_corrupted,
    )
    processed_segments = len(selected_segments)
    processed_samples = 0
    log_groups = group_segments_by_log(selected_segments)
    bias_groups = {
        path: grouped
        for path, grouped in group_segments_by_log(bias_segments if bias_segments is not None else segments)
    }
    requested_jobs = max(1, int(jobs))
    effective_jobs = min(requested_jobs, max(1, len(log_groups)))
    if write_row_artifacts:
        effective_jobs = 1
    log_results: list[LogReplayResult] = []

    nis_handle = None
    diagnostic_handle = None
    nis_writer: csv.DictWriter | None = None
    diagnostic_writer: csv.DictWriter | None = None
    if write_row_artifacts:
        nis_handle = nis_path.open("w", newline="", encoding="utf-8")
        diagnostic_handle = diagnostics_path.open("w", newline="", encoding="utf-8")
        nis_writer = csv.DictWriter(nis_handle, fieldnames=NIS_FIELDS)
        diagnostic_writer = csv.DictWriter(diagnostic_handle, fieldnames=DIAGNOSTIC_FIELDS)
        nis_writer.writeheader()
        diagnostic_writer.writeheader()

    try:
        if effective_jobs > 1:
            with ProcessPoolExecutor(max_workers=effective_jobs) as executor:
                futures = [
                    executor.submit(
                        process_replay_log,
                        log_path,
                        log_segments,
                        bias_groups.get(log_path, log_segments),
                        list(candidates),
                        vehicle,
                        covariance,
                        replay_mode,
                        max_rows_per_segment,
                    )
                    for log_path, log_segments in log_groups
                ]
                for future in as_completed(futures):
                    result = future.result()
                    log_results.append(result)
                    processed_samples += merge_log_result(accumulators, aggregates, result)
        else:
            for log_path, log_segments in log_groups:
                result = process_replay_log(
                    log_path,
                    log_segments,
                    bias_groups.get(log_path, log_segments),
                    candidates,
                    vehicle,
                    covariance,
                    replay_mode,
                    max_rows_per_segment,
                    nis_writer,
                    diagnostic_writer,
                )
                log_results.append(result)
                processed_samples += merge_log_result(accumulators, aggregates, result)
    finally:
        if nis_handle is not None:
            nis_handle.close()
        if diagnostic_handle is not None:
            diagnostic_handle.close()

    aggregate_path = write_aggregate_csv(output_dir, aggregates)
    itemized_path = write_itemized_aggregate_csv(output_dir, aggregates)
    candidate_rms_path = write_candidate_aggregate_csv(output_dir, aggregates)
    bias_summary_path = write_bias_summary_csv(output_dir, log_results)
    report_path = output_dir / "report.md"

    summary = {
        "schema_version": 1,
        "processed_segments": processed_segments,
        "processed_samples": processed_samples,
        "skipped_corrupted_segments": skipped_corrupted_segments,
        "source_log_count": len(log_groups),
        "jobs_requested": requested_jobs,
        "jobs_used": effective_jobs,
        "replay_mode": replay_mode,
        "uses_logged_ukf_state": False,
        "row_artifacts_enabled": write_row_artifacts,
        "nis_csv": str(nis_path) if write_row_artifacts else None,
        "diagnostics_csv": str(diagnostics_path) if write_row_artifacts else None,
        "aggregate_csv": str(aggregate_path),
        "itemized_csv": str(itemized_path),
        "candidate_rms_csv": str(candidate_rms_path),
        "bias_summary_csv": str(bias_summary_path),
        "bias_segment_manifest": str(bias_segment_manifest) if bias_segment_manifest is not None else "",
        "report_md": str(report_path),
        "state_names": list(STATE_NAMES),
        "candidates": {
            candidate_id: {
                "rms_nis": accumulator.rms_nis(),
                "sqrt_mean_nis": accumulator.sqrt_mean_nis(),
                "nis_count": accumulator.nis_count,
                "finite_count": accumulator.finite_count,
                "nonfinite_count": accumulator.nis_count - accumulator.finite_count,
                "accepted_count": accumulator.accepted_count,
                "rejected_count": accumulator.rejected_count,
                "accepted_only_rms_nis": accumulator.accepted_only_rms_nis(),
                "accepted_only_sqrt_mean_nis": accumulator.accepted_only_sqrt_mean_nis(),
                "rejected_rate": accumulator.rejected_rate(),
                "rejected_rate_penalty": REJECTED_RATE_PENALTY_WEIGHT * accumulator.rejected_rate(),
                "segment_count": len(accumulator.segment_ids),
                "residual_rms": {
                    key: math.sqrt(accumulator.residual_sum_square[key] / accumulator.residual_count[key])
                    for key in sorted(accumulator.residual_count)
                },
            }
            for candidate_id, accumulator in sorted(accumulators.items())
        },
    }
    write_replay_report(output_dir, summary)
    (output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True, default=str) + "\n",
        encoding="utf-8",
    )
    return summary


def residual_tail_updates(
    sample: ReplaySample,
    state: Sequence[float],
    plant: PlantResult,
    covariance: CovarianceConfig,
    vehicle: VehicleConfig | None = None,
) -> list[UpdateResult | None]:
    measurement = covariance.measurement
    del vehicle
    return [
        scalar_residual_update(
            "yaw_rate_residual_tail",
            sample.yaw_rate_radps,
            state[YAW_RATE],
            measurement.yaw_rate_sigma_radps**2,
            measurement.yaw_rate_gate_nis,
            sample.gyro_valid,
        ),
        scalar_residual_update(
            "forward_accel_residual_tail",
            sample.accel_forward_mps2,
            plant.imu_forward_accel_mps2,
            measurement.accel_sigma_mps2**2,
            measurement.accel_gate_nis,
            sample.accel_valid,
        ),
        scalar_residual_update(
            "right_accel_residual_tail",
            sample.accel_right_mps2,
            plant.imu_right_accel_mps2,
            measurement.accel_sigma_mps2**2,
            measurement.accel_gate_nis,
            sample.accel_valid,
        ),
    ]


def invalid_residual_tail_updates(
    sample: ReplaySample,
    covariance: CovarianceConfig,
) -> list[UpdateResult | None]:
    measurement = covariance.measurement
    return [
        scalar_residual_update(
            "yaw_rate_residual_tail",
            sample.yaw_rate_radps,
            math.nan,
            measurement.yaw_rate_sigma_radps**2,
            measurement.yaw_rate_gate_nis,
            sample.gyro_valid,
        ),
        scalar_residual_update(
            "forward_accel_residual_tail",
            sample.accel_forward_mps2,
            math.nan,
            measurement.accel_sigma_mps2**2,
            measurement.accel_gate_nis,
            sample.accel_valid,
        ),
        scalar_residual_update(
            "right_accel_residual_tail",
            sample.accel_right_mps2,
            math.nan,
            measurement.accel_sigma_mps2**2,
            measurement.accel_gate_nis,
            sample.accel_valid,
        ),
    ]


def scalar_residual_update(
    log_parameter: str,
    measurement: float,
    prediction: float,
    variance: float,
    gate_threshold: float,
    valid: bool,
) -> UpdateResult | None:
    gate_threshold = effective_gate_threshold(log_parameter, gate_threshold)
    if not valid or not finite(measurement):
        return None
    innovation_variance = max(variance, 1.0e-12)
    if not finite(prediction):
        innovation = math.nan
        nis = math.nan
        accepted = False
    else:
        innovation = measurement - prediction
        try:
            nis = (innovation * innovation) / innovation_variance
        except (ArithmeticError, OverflowError, ValueError):
            nis = math.nan
            accepted = False
        else:
            accepted = finite(nis) and not (math.isfinite(gate_threshold) and nis > gate_threshold)
    return UpdateResult(
        log_parameter=log_parameter,
        measurement_dimension=1,
        nis=nis,
        accepted=accepted,
        gate_threshold=gate_threshold,
        innovation=innovation,
        innovation_variance=innovation_variance,
        measurement=measurement,
        prediction=prediction,
        metric_kind=metric_kind_for_log_parameter(log_parameter),
    )


def nis_row(candidate: CandidateConfig, sample: ReplaySample, update: UpdateResult) -> dict[str, str]:
    return {
        "candidate_id": candidate.candidate_id,
        "segment_id": sample.segment_id,
        "split": sample.split,
        "stage": sample.stage,
        "run_id": sample.run_id,
        "source_path": str(sample.source_path),
        "source_row_index": str(sample.source_row_index),
        "master_time_us": str(sample.master_time_us),
        "log_parameter": update.log_parameter,
        "metric_kind": update.metric_kind,
        "measurement_dimension": str(update.measurement_dimension),
        "command_bucket": sample_launch_command_signature(sample) if is_launch_stage(sample.stage) else "",
        "nis": format_number(update.nis),
        "accepted": "true" if update.accepted else "false",
        "rejected": "false" if update.accepted else "true",
        "gate_threshold": format_number(update.gate_threshold),
        "innovation": format_number(update.innovation),
        "innovation_variance": format_number(update.innovation_variance),
        "measurement": format_number(update.measurement),
        "prediction": format_number(update.prediction),
        "corrupted": "true" if sample.corrupted else "false",
    }


def diagnostic_row(
    candidate: CandidateConfig,
    sample: ReplaySample,
    state: Sequence[float],
    plant: PlantResult,
    pre_update_plant: PlantResult,
    measured_yaw_accel_radps2: float,
    vehicle: VehicleConfig,
) -> dict[str, str]:
    _ = pre_update_plant
    predicted_left_encoder = predicted_encoder_wheel_rate_radps(state, vehicle, "left")
    predicted_right_encoder = predicted_encoder_wheel_rate_radps(state, vehicle, "right")
    yaw_residual = sample.yaw_rate_radps - state[YAW_RATE]
    left_encoder_residual = sample.left_wheel_rate_radps - predicted_left_encoder
    right_encoder_residual = sample.right_wheel_rate_radps - predicted_right_encoder
    yaw_accel_residual = measured_yaw_accel_radps2 - plant.yaw_accel_radps2
    forward_residual = sample.accel_forward_mps2 - plant.imu_forward_accel_mps2
    right_residual = sample.accel_right_mps2 - plant.imu_right_accel_mps2
    row = {
        "candidate_id": candidate.candidate_id,
        "segment_id": sample.segment_id,
        "split": sample.split,
        "stage": sample.stage,
        "run_id": sample.run_id,
        "source_path": str(sample.source_path),
        "source_row_index": str(sample.source_row_index),
        "master_time_us": str(sample.master_time_us),
        "dt_s": format_number(sample.dt_s),
        "left_command": format_number(sample.left_command),
        "right_command": format_number(sample.right_command),
        "left_wheel_rate_radps": format_number(sample.left_wheel_rate_radps),
        "right_wheel_rate_radps": format_number(sample.right_wheel_rate_radps),
        "predicted_left_encoder_wheel_rate_radps": format_number(predicted_left_encoder),
        "predicted_right_encoder_wheel_rate_radps": format_number(predicted_right_encoder),
        "fan_duty_cycle": format_number(sample.fan_duty_cycle),
        "predicted_forward_accel_mps2": format_number(plant.imu_forward_accel_mps2),
        "predicted_right_accel_mps2": format_number(plant.imu_right_accel_mps2),
        "predicted_yaw_accel_radps2": format_number(plant.yaw_accel_radps2),
        "measured_yaw_rate_radps": format_number(sample.yaw_rate_radps),
        "measured_left_encoder_wheel_rate_radps": format_number(sample.left_wheel_rate_radps),
        "measured_right_encoder_wheel_rate_radps": format_number(sample.right_wheel_rate_radps),
        "measured_yaw_accel_radps2": format_number(measured_yaw_accel_radps2),
        "measured_forward_accel_mps2": format_number(sample.accel_forward_mps2),
        "measured_right_accel_mps2": format_number(sample.accel_right_mps2),
        "yaw_rate_residual_radps": format_number(yaw_residual),
        "left_encoder_wheel_rate_residual_radps": format_number(left_encoder_residual),
        "right_encoder_wheel_rate_residual_radps": format_number(right_encoder_residual),
        "yaw_accel_residual_radps2": format_number(yaw_accel_residual),
        "forward_accel_residual_mps2": format_number(forward_residual),
        "right_accel_residual_mps2": format_number(right_residual),
        "max_contact_relative_speed_mps": format_number(plant.max_contact_relative_speed_mps),
        "load_weighted_contact_relative_speed_mps": format_number(
            plant.load_weighted_contact_relative_speed_mps
        ),
        "load_weighted_lateral_relative_speed_mps": format_number(
            plant.load_weighted_lateral_relative_speed_mps
        ),
        "load_weighted_yaw_contact_speed_mps": format_number(
            plant.load_weighted_yaw_contact_speed_mps
        ),
        "max_contact_utilization": format_number(plant.max_contact_utilization),
        "max_contact_saturation": format_number(plant.max_contact_saturation),
        "total_normal_load_n": format_number(plant.total_normal_load_n),
        "ground_use": format_number(plant.ground_use),
    }
    for index, name in enumerate(STATE_NAMES):
        row[name] = format_number(state[index])
    return row


def build_segments_from_args(args: argparse.Namespace, repo_root: Path) -> list[SegmentSpec]:
    segments: list[SegmentSpec] = []
    if args.segment_manifest:
        segments.extend(segment_specs_from_manifest(Path(args.segment_manifest).resolve(), repo_root))
    if args.input_csv:
        segments.append(
            direct_input_segment(
                Path(args.input_csv).resolve(),
                args.segment_id,
                args.stage,
                args.split,
                args.run_id,
            )
        )
    if not segments:
        raise ConfigError("Provide --input-csv or --segment-manifest")
    return segments


def parse_args(argv: Iterable[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--candidate-config", required=True)
    parser.add_argument("--covariance-config", required=True)
    parser.add_argument("--input-csv", default="")
    parser.add_argument("--segment-manifest", default="")
    parser.add_argument(
        "--bias-segment-manifest",
        default="",
        help=(
            "Optional full manifest used only for stationary/static sensor-bias assessment. "
            "The replay/evaluation segment set still comes from --segment-manifest or --input-csv."
        ),
    )
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--segment-id", default="")
    parser.add_argument("--stage", default="")
    parser.add_argument("--split", default="")
    parser.add_argument("--run-id", default="")
    parser.add_argument("--max-segments", type=int, default=0)
    parser.add_argument("--max-rows-per-segment", type=int, default=0)
    parser.add_argument(
        "--replay-mode",
        choices=("ekf", "residual"),
        default="ekf",
        help="Use true EKF ANIS replay or faster fixed-variance residual_tail comparison.",
    )
    parser.add_argument(
        "--include-corrupted",
        action="store_true",
        help="Replay corrupted manifest segments too. By default they are skipped.",
    )
    parser.add_argument(
        "--write-row-artifacts",
        action="store_true",
        help=(
            "Also write per-update nis_samples.csv and per-sample residual_diagnostics.csv. "
            "Default replay writes aggregate artifacts only."
        ),
    )
    parser.add_argument(
        "--jobs",
        type=int,
        default=1,
        help=(
            "Number of source-log replay workers. Parallel replay is aggregate-only; "
            "row artifact runs are forced to one worker."
        ),
    )
    return parser.parse_args(list(argv))


def parse_ukf_validation_args(argv: Iterable[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run a standalone UKF-style sigma-point validation over a small fixed corpus."
    )
    parser.add_argument(
        "--candidate-config",
        default=str(DEFAULT_TESTBED_STAGING_DIR / "candidates.json"),
    )
    parser.add_argument(
        "--covariance-config",
        default=str(DEFAULT_TESTBED_STAGING_DIR / "covariance_conservative.json"),
    )
    parser.add_argument(
        "--segment-manifest",
        default=str(DEFAULT_UKF_VALIDATION_MANIFEST_PATH),
    )
    parser.add_argument(
        "--bias-segment-manifest",
        default="",
        help="Optional full manifest used only for stationary/static sensor-bias assessment.",
    )
    parser.add_argument(
        "--output-dir",
        default=str(DEFAULT_UKF_VALIDATION_OUTPUT_DIR),
    )
    parser.add_argument("--max-segments", type=int, default=0)
    parser.add_argument("--max-rows-per-segment", type=int, default=0)
    parser.add_argument("--include-corrupted", action="store_true")
    parser.add_argument("--sigma-scale", type=float, default=1.0)
    parser.add_argument("--zero-vf-epsilon-mps", type=float, default=0.02)
    parser.add_argument("--zero-yaw-epsilon-radps", type=float, default=0.05)
    return parser.parse_args(list(argv))


def ukf_validation_main(argv: Iterable[str]) -> int:
    args = parse_ukf_validation_args(argv)
    repo_root = Path(__file__).resolve().parents[3]
    manifest_path = Path(args.segment_manifest).resolve()
    candidates = load_candidates(Path(args.candidate_config).resolve())
    vehicle, covariance = load_covariance(Path(args.covariance_config).resolve())
    segments = segment_specs_from_manifest(manifest_path, repo_root)
    bias_segment_manifest = Path(args.bias_segment_manifest).resolve() if args.bias_segment_manifest else None
    bias_segments = (
        segment_specs_from_manifest(bias_segment_manifest, repo_root)
        if bias_segment_manifest is not None
        else None
    )
    summary = run_ukf_validation(
        candidates=candidates,
        vehicle=vehicle,
        covariance=covariance,
        segments=segments,
        output_dir=Path(args.output_dir).resolve(),
        bias_segments=bias_segments,
        max_segments=args.max_segments,
        max_rows_per_segment=args.max_rows_per_segment,
        include_corrupted=args.include_corrupted,
        sigma_scale=args.sigma_scale,
        zero_vf_epsilon_mps=args.zero_vf_epsilon_mps,
        zero_yaw_epsilon_radps=args.zero_yaw_epsilon_radps,
        segment_manifest=manifest_path,
        bias_segment_manifest=bias_segment_manifest,
    )
    print(
        "Wrote UKF sigma-point validation artifacts: "
        f"{summary['candidate_summary_csv']}, {summary['events_csv']}, and {summary['report_md']}"
    )
    return 0 if summary["passed"] else 2


def main(argv: Iterable[str]) -> int:
    args = parse_args(argv)
    repo_root = Path(__file__).resolve().parents[3]
    candidates = load_candidates(Path(args.candidate_config).resolve())
    vehicle, covariance = load_covariance(Path(args.covariance_config).resolve())
    segments = build_segments_from_args(args, repo_root)
    bias_segment_manifest = Path(args.bias_segment_manifest).resolve() if args.bias_segment_manifest else None
    bias_segments = (
        segment_specs_from_manifest(bias_segment_manifest, repo_root)
        if bias_segment_manifest is not None
        else None
    )
    summary = run_replay(
        candidates=candidates,
        vehicle=vehicle,
        covariance=covariance,
        segments=segments,
        output_dir=Path(args.output_dir).resolve(),
        bias_segments=bias_segments,
        max_segments=args.max_segments,
        max_rows_per_segment=args.max_rows_per_segment,
        include_corrupted=args.include_corrupted,
        replay_mode=args.replay_mode,
        write_row_artifacts=args.write_row_artifacts,
        jobs=args.jobs,
        bias_segment_manifest=bias_segment_manifest,
    )
    if args.write_row_artifacts:
        print(
            "Wrote traction ANIS/residual aggregate and row artifacts: "
            f"{summary['aggregate_csv']}, {summary['nis_csv']}, and {summary['diagnostics_csv']}"
        )
    else:
        print(
            "Wrote traction ANIS/residual aggregate artifacts: "
            f"{summary['aggregate_csv']} and {summary['itemized_csv']}"
        )
    return 0
