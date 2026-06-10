from __future__ import annotations

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
    VF,
    VR,
    YAW_RATE,
    CandidateConfig,
    CandidateAccumulator,
    CandidatePlant,
    ConfigError,
    CovarianceConfig,
    NisAggregate,
    ReplaySample,
    SegmentSpec,
    VehicleConfig,
    estimate_accel_bias_from_samples,
    invalid_residual_tail_updates,
    load_candidates,
    replace_sample_accel_bias,
    sample_from_row,
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

        self.assertEqual(len(updates), 5)
        self.assertTrue(all(not update.accepted for update in updates))
        self.assertTrue(all(math.isnan(update.nis) for update in updates))

        accumulator = CandidateAccumulator()
        aggregate = NisAggregate()
        for update in updates:
            accumulator.add_update(update, sample.segment_id)
            aggregate.add(sample.segment_id, update)

        self.assertEqual(accumulator.nis_count, 5)
        self.assertEqual(accumulator.finite_count, 0)
        self.assertEqual(accumulator.accepted_count, 0)
        self.assertEqual(accumulator.rejected_count, 5)
        self.assertEqual(accumulator.residual_count, {})
        self.assertEqual(aggregate.count, 5)
        self.assertEqual(aggregate.finite_count, 0)
        self.assertEqual(aggregate.accepted_count, 0)
        self.assertEqual(aggregate.rejected_count, 5)
        self.assertEqual(aggregate.residual_count, 0)


if __name__ == "__main__":
    unittest.main()
