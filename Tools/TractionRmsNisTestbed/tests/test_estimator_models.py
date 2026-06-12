from __future__ import annotations

import csv
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path


TESTBED_ROOT = Path(__file__).resolve().parents[1]
if str(TESTBED_ROOT) not in sys.path:
    sys.path.insert(0, str(TESTBED_ROOT))

from traction_rms_nis_testbed.estimator_core import (
    N,
    PX,
    PY,
    VF,
    VR,
    YAW_RATE,
    CandidateConfig,
    CandidateAccumulator,
    CandidatePlant,
    ConfigError,
    CovarianceConfig,
    EkfReplay,
    NisAggregate,
    ProcessConfig,
    ReplaySample,
    SegmentSpec,
    VehicleConfig,
    covariance_sandwich,
    encoder_pair_covariance_radps,
    estimate_accel_bias_from_samples,
    invalid_residual_tail_updates,
    load_candidates,
    model_parameters,
    replace_sample_accel_bias,
    read_segment_samples,
    read_segment_samples_cached,
    sample_from_row,
    SourceLogSampleCache,
    scalar_residual_update,
)


class TractionEstimatorModelTest(unittest.TestCase):
    def make_sample(self) -> ReplaySample:
        return ReplaySample(
            source_path=Path("unit.csv"),
            source_row_index=0,
            master_time_us=1000,
            dt_s=0.001,
            left_command=0.72,
            right_command=-0.34,
            left_wheel_rate_radps=42.0,
            right_wheel_rate_radps=-18.0,
            yaw_rate_radps=1.4,
            accel_forward_mps2=0.0,
            accel_right_mps2=0.0,
            accel_valid=True,
            gyro_valid=True,
            fan_duty_cycle=0.8,
            segment_id="unit",
            stage="model",
            split="train",
            run_id="unit",
        )

    def make_state(self) -> list[float]:
        state = [0.0 for _ in range(N)]
        state[VF] = 0.22
        state[VR] = -0.04
        state[YAW_RATE] = 1.1
        return state

    def candidate(self, model: str, params: dict[str, float] | None = None) -> CandidateConfig:
        return CandidateConfig(
            candidate_id=model,
            label=model,
            model=model,
            parameters=dict(params or {}),
        )

    def result_signature(self, plant: CandidatePlant) -> tuple[float, float, float]:
        result = plant.plant_result(self.make_state(), self.make_sample())
        return (
            round(result.forward_accel_mps2, 7),
            round(result.right_accel_mps2, 7),
            round(result.yaw_accel_radps2, 7),
        )

    def test_candidate_model_paths_produce_distinct_predictions(self) -> None:
        vehicle = VehicleConfig()
        base_params = {
            "peak_friction_coefficient_at_80pct_fan": 1.7,
            "longitudinal_slip_gain_n_per_mps": 20.0,
            "lateral_slip_gain_n_per_mps": 16.0,
            "combined_slip_envelope_exponent": 2.0,
            "low_speed_blend_mps": 0.05,
        }
        plants = [
            CandidatePlant(vehicle, self.candidate("current_holdover_approximation", base_params)),
            CandidatePlant(vehicle, self.candidate("algebraic_envelope", base_params)),
            CandidatePlant(
                vehicle,
                self.candidate(
                    "stribeck_algebraic",
                    {
                        **base_params,
                        "dynamic_to_static_grip_ratio": 0.55,
                        "stribeck_velocity_mps": 0.05,
                        "viscous_slip_damping_n_per_mps": 4.0,
                    },
                ),
            ),
            CandidatePlant(
                vehicle,
                self.candidate(
                    "load_sensitive_anisotropic",
                    {
                        **base_params,
                        "longitudinal_load_transfer_gain": 0.22,
                        "lateral_load_transfer_gain": 0.31,
                        "longitudinal_load_sensitivity": 0.20,
                        "lateral_load_sensitivity": 0.30,
                        "yaw_coupling_gain": 0.20,
                    },
                ),
            ),
        ]

        signatures = [self.result_signature(plant) for plant in plants]

        self.assertEqual(len(signatures), len(set(signatures)))

    def test_short_round_models_register_and_produce_finite_predictions(self) -> None:
        vehicle = VehicleConfig()
        sample = ReplaySample(
            **{
                **self.make_sample().__dict__,
                "previous_left_wheel_rate_radps": 35.0,
                "previous_right_wheel_rate_radps": -8.0,
                "previous_yaw_rate_radps": 0.8,
            }
        )
        for model in ("skew_shear", "shear_rate", "in_shear"):
            with self.subTest(model=model):
                params = model_parameters(model, {})
                plant = CandidatePlant(vehicle, self.candidate(model, params))
                for vf in (-1.0e-8, 0.0, 1.0e-8):
                    state = self.make_state()
                    state[VF] = vf
                    result = plant.plant_result(state, sample)
                    values = (
                        result.forward_accel_mps2,
                        result.right_accel_mps2,
                        result.yaw_accel_radps2,
                        result.imu_forward_accel_mps2,
                        result.imu_right_accel_mps2,
                    )
                    self.assertTrue(all(math.isfinite(value) for value in values))

    def test_shear_rate_uses_replayable_previous_input_history(self) -> None:
        vehicle = VehicleConfig()
        plant = CandidatePlant(
            vehicle,
            self.candidate(
                "shear_rate",
                {
                    "peak_friction_coefficient_at_80pct_fan": 4.0,
                    "longitudinal_slip_gain_n_per_mps": 12.0,
                    "lateral_slip_gain_n_per_mps": 12.0,
                    "combined_slip_envelope_exponent": 2.0,
                    "low_speed_blend_mps": 0.06,
                    "shear_rate_peak_force_n": 0.12,
                    "shear_rate_activation_mps2": 8.0,
                    "shear_rate_breakaway_speed_mps": 0.25,
                },
            ),
        )
        state = self.make_state()
        no_history = self.make_sample()
        history = ReplaySample(
            **{
                **no_history.__dict__,
                "previous_left_wheel_rate_radps": no_history.left_wheel_rate_radps - 30.0,
                "previous_right_wheel_rate_radps": no_history.right_wheel_rate_radps + 22.0,
                "previous_yaw_rate_radps": no_history.yaw_rate_radps - 0.6,
            }
        )

        without_rate = plant.plant_result(state, no_history)
        with_rate = plant.plant_result(state, history)

        self.assertNotAlmostEqual(without_rate.forward_accel_mps2, with_rate.forward_accel_mps2)
        self.assertNotAlmostEqual(without_rate.yaw_accel_radps2, with_rate.yaw_accel_radps2)

    def test_in_shear_inward_lateral_velocity_changes_lateral_shear(self) -> None:
        vehicle = VehicleConfig()
        plant = CandidatePlant(
            vehicle,
            self.candidate(
                "in_shear",
                {
                    "peak_friction_coefficient_at_80pct_fan": 10.0,
                    "longitudinal_slip_gain_n_per_mps": 0.0,
                    "lateral_slip_gain_n_per_mps": 20.0,
                    "combined_slip_envelope_exponent": 2.0,
                    "low_speed_blend_mps": 0.001,
                    "inward_lateral_stiffness_gain": 1.0,
                    "inward_lateral_grip_gain": 0.0,
                    "inward_shear_blend_speed_mps": 0.01,
                },
            ),
        )
        left_contact = plant.contacts[0]
        sample = ReplaySample(
            **{
                **self.make_sample().__dict__,
                "left_command": 0.0,
                "right_command": 0.0,
            }
        )

        _force_f, inward_right, _utilization, _saturation = plant.contact_force(
            left_contact,
            normal_load_n=0.5,
            bank_load_n=1.0,
            rel_forward_mps=0.0,
            rel_right_mps=0.04,
            sample=sample,
            model=plant.model,
            params=plant.params,
        )
        _force_f, outward_right, _utilization, _saturation = plant.contact_force(
            left_contact,
            normal_load_n=0.5,
            bank_load_n=1.0,
            rel_forward_mps=0.0,
            rel_right_mps=-0.04,
            sample=sample,
            model=plant.model,
            params=plant.params,
        )

        self.assertGreater(abs(inward_right), abs(outward_right))

    def test_candidate_two_inherits_candidate_one_algebraic_tunables(self) -> None:
        vehicle = VehicleConfig()
        candidate_one = CandidatePlant(vehicle, self.candidate("algebraic_envelope"))
        candidate_two = CandidatePlant(
            vehicle,
            self.candidate(
                "stribeck_algebraic",
                {
                    "dynamic_to_static_grip_ratio": 1.0,
                    "stribeck_velocity_mps": 0.10,
                    "viscous_slip_damping_n_per_mps": 0.0,
                },
            ),
        )

        one = candidate_one.plant_result(self.make_state(), self.make_sample())
        two = candidate_two.plant_result(self.make_state(), self.make_sample())

        self.assertAlmostEqual(one.forward_accel_mps2, two.forward_accel_mps2)
        self.assertAlmostEqual(one.right_accel_mps2, two.right_accel_mps2)
        self.assertAlmostEqual(one.yaw_accel_radps2, two.yaw_accel_radps2)

    def test_candidate_one_contact_envelope_scales_force_components_before_summation(self) -> None:
        vehicle = VehicleConfig(forward_accel_limit_mps2=0.10, reverse_accel_limit_mps2=0.10)
        plant = CandidatePlant(
            vehicle,
            self.candidate(
                "algebraic_envelope",
                {
                    "peak_friction_coefficient_at_80pct_fan": 0.40,
                    "longitudinal_slip_gain_n_per_mps": 80.0,
                    "lateral_slip_gain_n_per_mps": 80.0,
                    "combined_slip_envelope_exponent": 2.0,
                    "low_speed_blend_mps": 0.01,
                },
            ),
        )
        sample = self.make_sample()
        sample = ReplaySample(**{**sample.__dict__, "left_command": 0.0, "right_command": 0.0})
        contact = plant.contacts[0]

        force_f, force_r, utilization, _saturation = plant.contact_force(
            contact,
            normal_load_n=0.20,
            bank_load_n=0.40,
            rel_forward_mps=2.0,
            rel_right_mps=1.5,
            sample=sample,
            model=plant.model,
            params=plant.params,
        )

        self.assertGreater(utilization, 1.0)
        self.assertAlmostEqual(force_f / force_r, 2.0 / 1.5, places=6)

        high_limit_vehicle = VehicleConfig(forward_accel_limit_mps2=100.0, reverse_accel_limit_mps2=100.0)
        low_limit_result = plant.plant_result(self.make_state(), sample)
        high_limit_result = CandidatePlant(
            high_limit_vehicle,
            self.candidate("algebraic_envelope", plant.params),
        ).plant_result(self.make_state(), sample)

        self.assertAlmostEqual(low_limit_result.forward_accel_mps2, high_limit_result.forward_accel_mps2)
        self.assertAlmostEqual(low_limit_result.yaw_accel_radps2, high_limit_result.yaw_accel_radps2)

    def test_negative_candidate_three_yaw_loss_gain_is_clamped_not_thrust(self) -> None:
        vehicle = VehicleConfig()
        state = [0.0 for _ in range(N)]
        state[YAW_RATE] = 2.0
        sample = self.make_sample()
        sample = ReplaySample(**{**sample.__dict__, "left_command": 0.0, "right_command": 0.0})
        base_params = {
            "peak_friction_coefficient_at_80pct_fan": 1.7,
            "longitudinal_slip_gain_n_per_mps": 20.0,
            "lateral_slip_gain_n_per_mps": 20.0,
            "longitudinal_load_transfer_gain": 0.0,
            "lateral_load_transfer_gain": 0.0,
            "longitudinal_load_sensitivity": 0.0,
            "lateral_load_sensitivity": 0.0,
        }
        zero = CandidatePlant(
            vehicle,
            self.candidate("load_sensitive_anisotropic", {**base_params, "yaw_coupling_gain": 0.0}),
        ).plant_result(state, sample)
        negative = CandidatePlant(
            vehicle,
            self.candidate("load_sensitive_anisotropic", {**base_params, "yaw_coupling_gain": -5.0}),
        ).plant_result(state, sample)
        positive = CandidatePlant(
            vehicle,
            self.candidate("load_sensitive_anisotropic", {**base_params, "yaw_coupling_gain": 0.5}),
        ).plant_result(state, sample)

        self.assertAlmostEqual(zero.yaw_accel_radps2, negative.yaw_accel_radps2)
        self.assertLess(positive.yaw_accel_radps2, zero.yaw_accel_radps2)

    def test_logged_ukf_state_columns_do_not_change_sample_or_prediction(self) -> None:
        vehicle = VehicleConfig()
        spec = SegmentSpec(log_path=Path("unit.csv"), segment_id="seg", split="train")
        row = {
            "master_time_us": "1000",
            "dt_us": "1000",
            "left_drive_command": "0.3",
            "right_drive_command": "0.1",
            "left_encoder_omega_radps": "12.0",
            "right_encoder_omega_radps": "8.0",
            "gyro_radps": "0.2",
            "accel_body_y_mps2": "1.0",
            "accel_body_x_mps2": "0.5",
            "accel_bias_valid": "1",
            "fan_duty_cycle": "0.8",
            "ukf_state_px_m": "1.0",
            "logged_ukf_state_vf_mps": "2.0",
        }
        changed_ukf = {
            **row,
            "ukf_state_px_m": "9001.0",
            "logged_ukf_state_vf_mps": "-9001.0",
        }

        sample_a = sample_from_row(row, spec, 0, None, vehicle)
        sample_b = sample_from_row(changed_ukf, spec, 0, None, vehicle)
        plant = CandidatePlant(vehicle, self.candidate("algebraic_envelope"))
        result_a = plant.plant_result([0.0 for _ in range(N)], sample_a)
        result_b = plant.plant_result([0.0 for _ in range(N)], sample_b)

        self.assertEqual(sample_a, sample_b)
        self.assertEqual(result_a, result_b)

    def test_cached_segment_reader_matches_per_segment_reader(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            log_path = Path(temp_dir) / "open_floor_main.csv"
            with log_path.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(
                    handle,
                    fieldnames=[
                        "master_time_us",
                        "dt_us",
                        "left_drive_command",
                        "right_drive_command",
                        "left_encoder_omega_radps",
                        "right_encoder_omega_radps",
                        "gyro_radps",
                        "accel_body_y_mps2",
                        "accel_body_x_mps2",
                        "accel_bias_valid",
                        "fan_duty_cycle",
                    ],
                )
                writer.writeheader()
                for index in range(12):
                    writer.writerow(
                        {
                            "master_time_us": str(index * 1000),
                            "dt_us": "1000",
                            "left_drive_command": str(0.10 + 0.01 * index),
                            "right_drive_command": str(0.12 + 0.01 * index),
                            "left_encoder_omega_radps": str(1.0 + index),
                            "right_encoder_omega_radps": str(1.5 + index),
                            "gyro_radps": str(0.01 * index),
                            "accel_body_y_mps2": str(0.20 + 0.01 * index),
                            "accel_body_x_mps2": str(0.10 + 0.01 * index),
                            "accel_bias_valid": "1",
                            "fan_duty_cycle": "0.8",
                        }
                    )

            vehicle = VehicleConfig()
            segments = [
                SegmentSpec(
                    log_path=log_path,
                    segment_id="seg_a",
                    stage="SEC_20_LAUNCH",
                    split="train",
                    run_id="log",
                    start_row_index=2,
                    end_row_index=5,
                ),
                SegmentSpec(
                    log_path=log_path,
                    segment_id="seg_b",
                    stage="SEC_20_LAUNCH",
                    split="train",
                    run_id="log",
                    start_row_index=8,
                    end_row_index=10,
                ),
            ]
            cache = SourceLogSampleCache()

            for segment in segments:
                expected = list(read_segment_samples(segment, vehicle, max_rows=0))
                actual = list(read_segment_samples_cached(cache, segment, vehicle, max_rows=0))
                self.assertEqual(expected, actual)
            self.assertEqual(cache.index_build_count, 1)

    def test_accel_bias_valid_flag_does_not_reject_finite_accel(self) -> None:
        vehicle = VehicleConfig()
        spec = SegmentSpec(log_path=Path("unit.csv"), segment_id="seg", split="train")
        row = {
            "master_time_us": "1000",
            "dt_us": "1000",
            "gyro_radps": "0.2",
            "accel_body_y_mps2": "1.0",
            "accel_body_x_mps2": "0.5",
            "accel_bias_valid": "false",
        }

        sample = sample_from_row(row, spec, 0, None, vehicle)
        explicitly_invalid = sample_from_row(
            {**row, "accel_valid": "false"},
            spec,
            0,
            None,
            vehicle,
        )

        self.assertTrue(sample.accel_valid)
        self.assertFalse(explicitly_invalid.accel_valid)

    def test_encoder_and_gyro_validity_flags_do_not_reject_finite_data(self) -> None:
        vehicle = VehicleConfig()
        spec = SegmentSpec(log_path=Path("unit.csv"), segment_id="seg", split="train")
        row = {
            "master_time_us": "1000",
            "dt_us": "1000",
            "left_encoder_omega_radps": "12.0",
            "right_encoder_omega_radps": "8.0",
            "gyro_radps": "0.2",
            "accel_body_y_mps2": "1.0",
            "accel_body_x_mps2": "0.5",
            "encoder_valid": "false",
            "left_encoder_valid": "false",
            "right_encoder_valid": "false",
            "gyro_valid": "false",
            "ukf_yaw_valid_for_feedforward": "false",
        }

        sample = sample_from_row(row, spec, 0, None, vehicle)

        self.assertEqual(sample.left_wheel_rate_radps, 12.0)
        self.assertEqual(sample.right_wheel_rate_radps, 8.0)
        self.assertEqual(sample.yaw_rate_radps, 0.2)
        self.assertTrue(sample.gyro_valid)

    def test_encoder_rates_remain_plant_prediction_inputs(self) -> None:
        vehicle = VehicleConfig()
        plant = CandidatePlant(vehicle, self.candidate("algebraic_envelope"))
        state = [0.0 for _ in range(N)]
        base = self.make_sample()
        stopped = ReplaySample(
            **{
                **base.__dict__,
                "left_command": 0.0,
                "right_command": 0.0,
                "left_wheel_rate_radps": 0.0,
                "right_wheel_rate_radps": 0.0,
            }
        )
        rolling = ReplaySample(
            **{
                **stopped.__dict__,
                "left_wheel_rate_radps": 18.0,
                "right_wheel_rate_radps": 18.0,
            }
        )

        stopped_prediction = plant.plant_result(state, stopped)
        rolling_prediction = plant.plant_result(state, rolling)

        self.assertNotAlmostEqual(
            stopped_prediction.imu_forward_accel_mps2,
            rolling_prediction.imu_forward_accel_mps2,
        )

    def test_slip_zlock_applies_only_after_sustained_static_evidence(self) -> None:
        vehicle = VehicleConfig()
        plant = CandidatePlant(vehicle, self.candidate("slip_zlock"))
        replay = EkfReplay(plant, CovarianceConfig())
        replay.state[PX] = 0.012
        replay.state[PY] = -0.008
        replay.state[VF] = 0.05
        replay.state[VR] = -0.04
        replay.state[YAW_RATE] = 0.02
        sample = ReplaySample(
            **{
                **self.make_sample().__dict__,
                "stage": "static",
                "dt_s": 0.1,
                "left_command": 0.0,
                "right_command": 0.0,
                "left_wheel_rate_radps": 0.0,
                "right_wheel_rate_radps": 0.0,
                "previous_left_wheel_rate_radps": 0.0,
                "previous_right_wheel_rate_radps": 0.0,
                "yaw_rate_radps": 0.0,
            }
        )

        self.assertFalse(replay.apply_stationary_zlock(sample))
        self.assertTrue(replay.apply_stationary_zlock(sample))

        self.assertLess(abs(replay.state[VF]), 0.005)
        self.assertLess(abs(replay.state[VR]), 0.005)
        self.assertLess(abs(replay.state[YAW_RATE]), 0.005)

    def test_slip_zlock_releases_during_yaw_launch(self) -> None:
        vehicle = VehicleConfig()
        plant = CandidatePlant(vehicle, self.candidate("slip_zlock"))
        replay = EkfReplay(plant, CovarianceConfig())
        static_sample = ReplaySample(
            **{
                **self.make_sample().__dict__,
                "stage": "static",
                "dt_s": 0.2,
                "left_command": 0.0,
                "right_command": 0.0,
                "left_wheel_rate_radps": 0.0,
                "right_wheel_rate_radps": 0.0,
                "previous_left_wheel_rate_radps": 0.0,
                "previous_right_wheel_rate_radps": 0.0,
                "yaw_rate_radps": 0.0,
            }
        )
        launch_sample = ReplaySample(
            **{
                **static_sample.__dict__,
                "stage": "yaw_launch",
                "left_command": -0.8,
                "right_command": 0.8,
            }
        )

        self.assertTrue(replay.apply_stationary_zlock(static_sample))
        self.assertFalse(replay.apply_stationary_zlock(launch_sample))

        self.assertEqual(replay.stationary_zlock_time_s, 0.0)
        self.assertIsNone(replay.stationary_zlock_anchor)

    def test_slip_zlock_is_not_returned_as_scored_measurement_update(self) -> None:
        vehicle = VehicleConfig()
        plant = CandidatePlant(vehicle, self.candidate("slip_zlock"))
        replay = EkfReplay(plant, CovarianceConfig())
        sample = ReplaySample(
            **{
                **self.make_sample().__dict__,
                "stage": "static",
                "dt_s": 0.2,
                "left_command": 0.0,
                "right_command": 0.0,
                "left_wheel_rate_radps": 0.0,
                "right_wheel_rate_radps": 0.0,
                "previous_left_wheel_rate_radps": 0.0,
                "previous_right_wheel_rate_radps": 0.0,
                "yaw_rate_radps": 0.0,
            }
        )

        updates = [
            replay.update_yaw_rate(sample),
            replay.update_accel_forward(sample),
            replay.update_accel_right(sample),
        ]
        replay.apply_stationary_zlock(sample)
        log_parameters = {update.log_parameter for update in updates if update is not None}

        self.assertFalse(any("encoder" in name for name in log_parameters))
        self.assertFalse(any("zlock" in name for name in log_parameters))

    def test_production_encoder_pair_covariance_numeric_values(self) -> None:
        covariance = encoder_pair_covariance_radps(
            VehicleConfig(),
            0.021187,
            0.111268,
        )

        self.assertAlmostEqual(covariance[0][0], 2.9624143598, places=10)
        self.assertAlmostEqual(covariance[1][1], 2.9624143598, places=10)
        self.assertAlmostEqual(covariance[0][1], 2.6835581039, places=10)
        self.assertAlmostEqual(covariance[1][0], 2.6835581039, places=10)
        self.assertGreater(covariance[0][1], 0.0)
        self.assertNotAlmostEqual(covariance[0][1], 0.0)

    def test_process_config_uses_production_encoder_sigmas_not_scalar_rate_sigma(self) -> None:
        process = ProcessConfig.from_json({"encoder_wheel_rate_sigma_radps": 1.0})

        self.assertAlmostEqual(process.encoder_linear_speed_sigma_mps, 0.021187)
        self.assertAlmostEqual(process.encoder_yaw_rate_sigma_radps, 0.111268)
        self.assertFalse(hasattr(process, "encoder_wheel_rate_sigma_radps"))

    def test_staged_production_process_config_uses_encoder_pair_sigmas(self) -> None:
        path = (
            TESTBED_ROOT.parents[1]
            / "staging"
            / "traction_candidate_rms_nis_testbed"
            / "covariance_conservative.json"
        )
        raw = json.loads(path.read_text(encoding="utf-8"))
        process = raw["covariance"]["process"]

        self.assertNotIn("encoder_wheel_rate_sigma_radps", process)
        self.assertEqual(process["encoder_linear_speed_sigma_mps"], 0.021187)
        self.assertEqual(process["encoder_yaw_rate_sigma_radps"], 0.111268)

    def test_encoder_input_process_noise_uses_correlated_wheel_covariance(self) -> None:
        vehicle = VehicleConfig()
        plant = CandidatePlant(vehicle, self.candidate("algebraic_envelope"))
        replay = EkfReplay(plant, CovarianceConfig())
        state = self.make_state()
        sample = self.make_sample()

        q = replay.encoder_input_noise(state, sample, sample.dt_s)
        columns: list[list[float]] = []
        step = 1.0e-3
        for side in ("left", "right"):
            plus = ReplaySample(
                **{
                    **sample.__dict__,
                    f"{side}_wheel_rate_radps": getattr(sample, f"{side}_wheel_rate_radps") + step,
                }
            )
            minus = ReplaySample(
                **{
                    **sample.__dict__,
                    f"{side}_wheel_rate_radps": getattr(sample, f"{side}_wheel_rate_radps") - step,
                }
            )
            x_plus = plant.propagate(state, plus, sample.dt_s)
            x_minus = plant.propagate(state, minus, sample.dt_s)
            columns.append([(x_plus[index] - x_minus[index]) / (2.0 * step) for index in range(N)])
        jacobian = [[columns[0][row], columns[1][row]] for row in range(N)]
        full_expected = covariance_sandwich(
            jacobian,
            encoder_pair_covariance_radps(vehicle, 0.021187, 0.111268),
        )
        scalar_independent = covariance_sandwich(
            jacobian,
            [
                [2.9624143598129784, 0.0],
                [0.0, 2.9624143598129784],
            ],
        )

        self.assertTrue(
            any(
                abs(q[row][col] - scalar_independent[row][col]) > 1.0e-16
                for row in range(N)
                for col in range(N)
            )
        )
        for row in range(N):
            for col in range(N):
                self.assertAlmostEqual(q[row][col], full_expected[row][col], places=18)

    def test_yaw_residual_update_is_ungated_but_accel_is_not(self) -> None:
        yaw_update = scalar_residual_update(
            "yaw_rate_residual_tail",
            measurement=10.0,
            prediction=0.0,
            variance=1.0,
            gate_threshold=1.0,
            valid=True,
        )
        accel_update = scalar_residual_update(
            "forward_accel_residual_tail",
            measurement=10.0,
            prediction=0.0,
            variance=1.0,
            gate_threshold=1.0,
            valid=True,
        )

        self.assertIsNotNone(yaw_update)
        assert yaw_update is not None
        self.assertTrue(yaw_update.accepted)
        self.assertTrue(math.isinf(yaw_update.gate_threshold))
        self.assertIsNotNone(accel_update)
        assert accel_update is not None
        self.assertFalse(accel_update.accepted)

    def test_accel_bias_estimate_corrects_finite_accel_samples(self) -> None:
        stationary = ReplaySample(
            **{
                **self.make_sample().__dict__,
                "left_command": 0.0,
                "right_command": 0.0,
                "left_wheel_rate_radps": 0.0,
                "right_wheel_rate_radps": 0.0,
                "yaw_rate_radps": 0.0,
                "accel_forward_mps2": 0.30,
                "accel_right_mps2": -0.20,
            }
        )
        active = ReplaySample(
            **{
                **stationary.__dict__,
                "left_command": 0.3,
                "right_command": 0.3,
                "left_wheel_rate_radps": 4.0,
                "right_wheel_rate_radps": 4.0,
                "accel_forward_mps2": 1.30,
                "accel_right_mps2": 0.80,
            }
        )

        bias = estimate_accel_bias_from_samples([stationary])
        corrected = replace_sample_accel_bias(active, bias)

        self.assertEqual(bias.sample_count, 1)
        self.assertAlmostEqual(corrected.accel_forward_mps2, 1.0)
        self.assertAlmostEqual(corrected.accel_right_mps2, 1.0)
        self.assertTrue(corrected.accel_valid)

    def test_candidate_model_names_are_validated(self) -> None:
        with self.assertRaises(ConfigError):
            CandidatePlant(VehicleConfig(), self.candidate("typo_model"))

        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "candidates.json"
            path.write_text(
                json.dumps({"candidates": [{"id": "bad", "model": "typo_model"}]}) + "\n",
                encoding="utf-8",
            )

            with self.assertRaises(ConfigError):
                load_candidates(path)

    def test_invalid_prediction_counts_as_rejected_without_residual_overflow(self) -> None:
        sample = self.make_sample()
        covariance = CovarianceConfig()

        updates = [
            update
            for update in invalid_residual_tail_updates(sample, covariance)
            if update is not None
        ]

        self.assertEqual(len(updates), 3)
        self.assertTrue(all(not update.accepted for update in updates))
        self.assertTrue(all(math.isnan(update.nis) for update in updates))

        accumulator = CandidateAccumulator()
        aggregate = NisAggregate()
        for update in updates:
            accumulator.add_update(update, sample.segment_id)
            aggregate.add(sample.segment_id, update)

        self.assertEqual(accumulator.nis_count, 3)
        self.assertEqual(accumulator.finite_count, 0)
        self.assertEqual(accumulator.accepted_count, 0)
        self.assertEqual(accumulator.rejected_count, 3)
        self.assertEqual(accumulator.residual_count, {})
        self.assertEqual(aggregate.count, 3)
        self.assertEqual(aggregate.finite_count, 0)
        self.assertEqual(aggregate.accepted_count, 0)
        self.assertEqual(aggregate.rejected_count, 3)
        self.assertEqual(aggregate.residual_count, 0)


if __name__ == "__main__":
    unittest.main()
