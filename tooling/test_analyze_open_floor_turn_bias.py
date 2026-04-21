import unittest

import analyze_open_floor_turn_bias as turn_bias


class AnalyzeOpenFloorTurnBiasTest(unittest.TestCase):
    def setUp(self) -> None:
        self.geometry = turn_bias.VehicleTurnGeometry(
            width_m=0.0842,
            effective_track_width_m=0.084635,
            tire_bank_inner_x_m=0.5 * 0.07004,
            tire_bank_outer_x_m=0.5 * 0.07868,
            imu_position_body_x_m=turn_bias.IMU_POSITION_BODY_X_M,
            imu_position_body_y_m=turn_bias.IMU_POSITION_BODY_Y_M,
        )
        self.biases = turn_bias.SensorBiasSummary(
            gyro_bias_radps=0.0,
            accel_bias_x_mps2=0.0,
            accel_bias_y_mps2=0.0,
            static_row_count=0,
        )
        self.thresholds = turn_bias.TurnBiasThresholds(
            center_band_m=0.005,
            core_min_abs_gyro_radps=4.0,
            core_peak_fraction=0.35,
            trail_min_abs_gyro_radps=1.0,
            min_pivot_run_samples=3,
        )

    def make_turn_row(
        self,
        *,
        phase_id: int,
        master_time_us: int,
        gyro_radps: float,
        cor_x_m: float,
        primitive_id: int = 7,
        repeat_index: int = 1,
        direction_id: int = turn_bias.DIRECTION_CLOCKWISE_ID,
        speed_bin: int = 2,
        center_accel_y_mps2: float = 0.0,
        gyro_alpha_radps2: float = 0.0,
    ) -> dict[str, str]:
        half_track_m = 0.5 * self.geometry.effective_track_width_m
        average_velocity_mps = cor_x_m * gyro_radps
        left_velocity_mps = average_velocity_mps + (half_track_m * gyro_radps)
        right_velocity_mps = average_velocity_mps - (half_track_m * gyro_radps)
        center_accel_x_mps2 = cor_x_m * gyro_radps * gyro_radps
        imu_accel_x_mps2 = (
            center_accel_x_mps2 -
            ((gyro_radps * gyro_radps) * self.geometry.imu_position_body_x_m) +
            (gyro_alpha_radps2 * self.geometry.imu_position_body_y_m)
        )
        imu_accel_y_mps2 = (
            center_accel_y_mps2 -
            ((gyro_radps * gyro_radps) * self.geometry.imu_position_body_y_m) -
            (gyro_alpha_radps2 * self.geometry.imu_position_body_x_m)
        )
        return {
            "section_id": str(turn_bias.YAW_SECTION_ID),
            "primitive_id": str(primitive_id),
            "repeat_index": str(repeat_index),
            "direction_id": str(direction_id),
            "speed_bin": str(speed_bin),
            "phase_id": str(phase_id),
            "dt_us": "1000",
            "master_time_us": str(master_time_us),
            "left_encoder_velocity_mps": f"{left_velocity_mps}",
            "right_encoder_velocity_mps": f"{right_velocity_mps}",
            "gyro_raw_radps": f"{gyro_radps}",
            "accel_body_x_mps2": f"{imu_accel_x_mps2}",
            "accel_body_y_mps2": f"{imu_accel_y_mps2}",
        }

    def test_center_accel_from_imu_inverts_offset_moment_arm(self) -> None:
        gyro_radps = 5.0
        gyro_alpha_radps2 = -12.0
        center_accel_x_mps2 = 0.82
        center_accel_y_mps2 = -0.37
        imu_accel_x_mps2 = (
            center_accel_x_mps2 -
            ((gyro_radps * gyro_radps) * self.geometry.imu_position_body_x_m) +
            (gyro_alpha_radps2 * self.geometry.imu_position_body_y_m)
        )
        imu_accel_y_mps2 = (
            center_accel_y_mps2 -
            ((gyro_radps * gyro_radps) * self.geometry.imu_position_body_y_m) -
            (gyro_alpha_radps2 * self.geometry.imu_position_body_x_m)
        )

        recovered_x_mps2, recovered_y_mps2 = turn_bias.center_accel_from_imu(
            accel_body_x_mps2=imu_accel_x_mps2,
            accel_body_y_mps2=imu_accel_y_mps2,
            gyro_radps=gyro_radps,
            gyro_alpha_radps2=gyro_alpha_radps2,
            geometry=self.geometry,
        )

        self.assertAlmostEqual(recovered_x_mps2, center_accel_x_mps2, places=9)
        self.assertAlmostEqual(recovered_y_mps2, center_accel_y_mps2, places=9)
        self.assertAlmostEqual(
            turn_bias.cor_x_from_center_accel(recovered_x_mps2, gyro_radps) or 0.0,
            center_accel_x_mps2 / (gyro_radps * gyro_radps),
            places=9,
        )

    def test_right_bias_turn_stays_bias_and_not_pivot(self) -> None:
        rows: list[dict[str, str]] = []
        for index in range(12):
            rows.append(
                self.make_turn_row(
                    phase_id=turn_bias.PHASE_STEADY_ID,
                    master_time_us=index * 1000,
                    gyro_radps=8.0,
                    cor_x_m=0.010,
                )
            )
        for index in range(12, 20):
            rows.append(
                self.make_turn_row(
                    phase_id=turn_bias.PHASE_STOP_ID,
                    master_time_us=index * 1000,
                    gyro_radps=2.0,
                    cor_x_m=0.010,
                )
            )

        summary = turn_bias.summarize_yaw_turn_rows(
            rows,
            biases=self.biases,
            geometry=self.geometry,
            thresholds=self.thresholds,
        )

        self.assertEqual(summary.core.fused_classification, "right_bias")
        self.assertEqual(summary.trail.fused_classification, "right_bias")
        self.assertIsNone(summary.trail.encoder_bank_zone_side)
        self.assertIsNone(summary.trail.consensus_pivot_side)
        self.assertEqual(summary.overall_classification, "right_bias")

    def test_right_pivot_requires_encoder_and_accel_agreement(self) -> None:
        rows: list[dict[str, str]] = []
        for index in range(12):
            rows.append(
                self.make_turn_row(
                    phase_id=turn_bias.PHASE_STEADY_ID,
                    master_time_us=index * 1000,
                    gyro_radps=8.0,
                    cor_x_m=0.0,
                    primitive_id=37,
                    repeat_index=24,
                    direction_id=turn_bias.DIRECTION_COUNTERCLOCKWISE_ID,
                    speed_bin=2,
                )
            )
        for index in range(12, 20):
            rows.append(
                self.make_turn_row(
                    phase_id=turn_bias.PHASE_STOP_ID,
                    master_time_us=index * 1000,
                    gyro_radps=-2.0,
                    cor_x_m=0.037,
                    primitive_id=37,
                    repeat_index=24,
                    direction_id=turn_bias.DIRECTION_COUNTERCLOCKWISE_ID,
                    speed_bin=2,
                )
            )

        summary = turn_bias.summarize_yaw_turn_rows(
            rows,
            biases=self.biases,
            geometry=self.geometry,
            thresholds=self.thresholds,
        )

        self.assertEqual(summary.trail.fused_classification, "right_pivot")
        self.assertEqual(summary.trail.encoder_bank_zone_side, "right")
        self.assertEqual(summary.trail.accel_bank_zone_side, "right")
        self.assertEqual(summary.trail.consensus_pivot_side, "right")
        self.assertEqual(summary.overall_classification, "right_pivot")

    def test_left_pivot_is_classified_symmetrically(self) -> None:
        rows: list[dict[str, str]] = []
        for index in range(10):
            rows.append(
                self.make_turn_row(
                    phase_id=turn_bias.PHASE_STEADY_ID,
                    master_time_us=index * 1000,
                    gyro_radps=7.0,
                    cor_x_m=0.0,
                    primitive_id=9,
                    repeat_index=31,
                    direction_id=turn_bias.DIRECTION_CLOCKWISE_ID,
                    speed_bin=3,
                )
            )
        for index in range(10, 18):
            rows.append(
                self.make_turn_row(
                    phase_id=turn_bias.PHASE_STOP_ID,
                    master_time_us=index * 1000,
                    gyro_radps=1.8,
                    cor_x_m=-0.037,
                    primitive_id=9,
                    repeat_index=31,
                    direction_id=turn_bias.DIRECTION_CLOCKWISE_ID,
                    speed_bin=3,
                )
            )

        summary = turn_bias.summarize_yaw_turn_rows(
            rows,
            biases=self.biases,
            geometry=self.geometry,
            thresholds=self.thresholds,
        )

        self.assertEqual(summary.trail.fused_classification, "left_pivot")
        self.assertEqual(summary.trail.encoder_bank_zone_side, "left")
        self.assertEqual(summary.trail.consensus_pivot_side, "left")
        self.assertEqual(summary.overall_classification, "left_pivot")

    def test_outboard_tail_is_not_misclassified_as_pivot(self) -> None:
        rows: list[dict[str, str]] = []
        for index in range(8):
            rows.append(
                self.make_turn_row(
                    phase_id=turn_bias.PHASE_STEADY_ID,
                    master_time_us=index * 1000,
                    gyro_radps=8.5,
                    cor_x_m=0.0,
                    primitive_id=9,
                    repeat_index=35,
                    direction_id=turn_bias.DIRECTION_CLOCKWISE_ID,
                    speed_bin=3,
                )
            )
        for index in range(8, 16):
            rows.append(
                self.make_turn_row(
                    phase_id=turn_bias.PHASE_STOP_ID,
                    master_time_us=index * 1000,
                    gyro_radps=1.1,
                    cor_x_m=0.060,
                    primitive_id=9,
                    repeat_index=35,
                    direction_id=turn_bias.DIRECTION_CLOCKWISE_ID,
                    speed_bin=3,
                )
            )

        summary = turn_bias.summarize_yaw_turn_rows(
            rows,
            biases=self.biases,
            geometry=self.geometry,
            thresholds=self.thresholds,
        )

        self.assertEqual(summary.trail.fused_classification, "right_outboard")
        self.assertIsNone(summary.trail.encoder_bank_zone_side)
        self.assertIsNone(summary.trail.consensus_pivot_side)
        self.assertEqual(summary.overall_classification, "right_outboard")


if __name__ == "__main__":
    unittest.main()
