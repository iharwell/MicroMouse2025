import math
import unittest

import open_floor_recovery


class OpenFloorRecoveryTest(unittest.TestCase):
    def make_row(
        self,
        dt_us: int,
        gyro_radps: float,
        gyro_bias_radps: float,
        logged_corrected_gyro_radps: float,
        left_velocity_mps: float,
        right_velocity_mps: float,
        left_distance_m: float,
        right_distance_m: float,
    ) -> dict[str, str]:
        predicted_x = -(gyro_radps * gyro_radps * open_floor_recovery.IMU_POSITION_BODY_X_M)
        predicted_y = -(gyro_radps * gyro_radps * open_floor_recovery.IMU_POSITION_BODY_Y_M)
        return {
            "dt_us": str(dt_us),
            "section_id": "3",
            "primitive_id": str(open_floor_recovery.RECOVERY_PRIMITIVE_ID),
            "repeat_index": "2",
            "start_marker_id": "2",
            "gyro_raw_radps": f"{gyro_radps + gyro_bias_radps}",
            "gyro_bias_radps": f"{gyro_bias_radps}",
            "gyro_radps": f"{logged_corrected_gyro_radps}",
            "accel_body_x_mps2": f"{predicted_x}",
            "accel_body_y_mps2": f"{predicted_y}",
            "left_encoder_velocity_mps": f"{left_velocity_mps}",
            "right_encoder_velocity_mps": f"{right_velocity_mps}",
            "left_encoder_distance_m": f"{left_distance_m}",
            "right_encoder_distance_m": f"{right_distance_m}",
            "left_drive_command": "0.9",
            "right_drive_command": "-0.9",
            "left_encoder_omega_radps": f"{left_velocity_mps / 0.01261}",
            "right_encoder_omega_radps": f"{right_velocity_mps / 0.01261}",
            "saturation_flags": "0",
        }

    def test_sensor_only_turn_summary_uses_gyro_and_encoder(self) -> None:
        track_width_m = 0.1
        gyro_radps = 6.0
        gyro_bias_radps = 0.35
        dt_s = 0.001
        active_rows = 100
        segment: list[dict[str, str]] = []
        left_distance_m = 0.0
        right_distance_m = 0.0

        segment.append(self.make_row(1000, 0.0, gyro_bias_radps, 0.7, 0.0, 0.0, left_distance_m, right_distance_m))
        for _ in range(active_rows):
            left_velocity_mps = -0.5 * track_width_m * gyro_radps
            right_velocity_mps = 0.5 * track_width_m * gyro_radps
            left_distance_m += left_velocity_mps * dt_s
            right_distance_m += right_velocity_mps * dt_s
            segment.append(
                self.make_row(
                    1000,
                    gyro_radps,
                    gyro_bias_radps,
                    gyro_radps + 1.5,
                    left_velocity_mps,
                    right_velocity_mps,
                    left_distance_m,
                    right_distance_m,
                )
            )
        for _ in range(40):
            segment.append(
                self.make_row(1000, 0.0, gyro_bias_radps, -0.4, 0.0, 0.0, left_distance_m, right_distance_m)
            )

        summaries, aggregate = open_floor_recovery.summarize_recovery_segments(
            [segment],
            gyro_bias_radps=gyro_bias_radps,
            accel_bias_x_mps2=0.0,
            accel_bias_y_mps2=0.0,
            control_log_path=None,
        )

        self.assertEqual(len(summaries), 1)
        self.assertIsNotNone(aggregate)
        summary = summaries[0]
        self.assertAlmostEqual(summary.angle_rad, gyro_radps * active_rows * dt_s, places=2)
        self.assertAlmostEqual(summary.effective_track_width_m, track_width_m, delta=0.0011)
        self.assertAlmostEqual(summary.angle_deg, math.degrees(summary.angle_rad), places=6)
        self.assertGreater(summary.median_rotation_alignment, 0.9)
        self.assertIsNotNone(summary.sample_effective_track_width_stats)
        assert summary.sample_effective_track_width_stats is not None
        self.assertEqual(summary.sample_effective_track_width_stats.count, active_rows)
        self.assertAlmostEqual(summary.sample_effective_track_width_stats.l50, track_width_m, delta=1.0e-6)
        self.assertAlmostEqual(summary.sample_effective_track_width_stats.mean, track_width_m, delta=1.0e-6)
        self.assertAlmostEqual(summary.sample_effective_track_width_stats.sigma, 0.0, delta=1.0e-9)
        self.assertEqual(summary.watchdog_flags, 0)
        assert aggregate is not None
        self.assertAlmostEqual(aggregate.median_effective_track_width_m, track_width_m, delta=0.0011)
        self.assertIsNotNone(aggregate.sample_effective_track_width_stats)
        assert aggregate.sample_effective_track_width_stats is not None
        self.assertAlmostEqual(aggregate.sample_effective_track_width_stats.l95, track_width_m, delta=1.0e-6)


if __name__ == "__main__":
    unittest.main()
