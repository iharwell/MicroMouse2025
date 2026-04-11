import unittest

import open_floor_launch_floor


class OpenFloorLaunchFloorTest(unittest.TestCase):
    def make_row(
        self,
        dt_us: int,
        command: float,
        measured_linear_speed_mps: float,
        accel_body_y_mps2: float,
        left_encoder_count: int,
        right_encoder_count: int,
        ukf_state_py_m: float,
    ) -> dict[str, str]:
        return {
            "dt_us": str(dt_us),
            "left_drive_command": f"{command}",
            "right_drive_command": f"{command}",
            "measured_linear_speed_mps": f"{measured_linear_speed_mps}",
            "accel_body_y_mps2": f"{accel_body_y_mps2}",
            "left_encoder_count": str(left_encoder_count),
            "right_encoder_count": str(right_encoder_count),
            "ukf_state_py_m": f"{ukf_state_py_m}",
            "saturation_flags": "0",
            "clipping_flags": "0",
            "watchdog_flags": "0",
        }

    def make_backlash_repeat(
        self,
        command: float,
        left_start_count: int,
        right_start_count: int,
        start_py_m: float,
    ) -> list[dict[str, str]]:
        rows: list[dict[str, str]] = []
        left_count = left_start_count
        right_count = right_start_count
        py_m = start_py_m
        count_step = 1 if command >= 0.0 else -1
        py_step_m = 0.000005 if command >= 0.0 else -0.000005
        speed_pattern = [0.0, 0.003, 0.0, 0.003, 0.0, 0.0, 0.003, 0.0, 0.0, 0.0, 0.003, 0.0]
        for index, speed_mps in enumerate(speed_pattern):
            if speed_mps > 0.0:
                left_count += count_step
                if index % 2 == 0:
                    right_count += count_step
                py_m += py_step_m
            rows.append(
                self.make_row(
                    dt_us=1000,
                    command=command,
                    measured_linear_speed_mps=speed_mps if command >= 0.0 else -speed_mps,
                    accel_body_y_mps2=(0.02 if speed_mps > 0.0 else -0.01) if command >= 0.0 else (-0.02 if speed_mps > 0.0 else 0.01),
                    left_encoder_count=left_count,
                    right_encoder_count=right_count,
                    ukf_state_py_m=py_m,
                )
            )
        return rows

    def make_clear_repeat(
        self,
        command: float,
        left_start_count: int,
        right_start_count: int,
        start_py_m: float,
    ) -> list[dict[str, str]]:
        rows: list[dict[str, str]] = []
        left_count = left_start_count
        right_count = right_start_count
        py_m = start_py_m
        count_step = 6 if command >= 0.0 else -6
        py_step_m = 0.00012 if command >= 0.0 else -0.00012
        for index in range(20):
            speed_mps = 0.0 if index < 2 else 0.015
            if speed_mps > 0.0:
                left_count += count_step
                right_count += count_step
                py_m += py_step_m
            rows.append(
                self.make_row(
                    dt_us=1000,
                    command=command,
                    measured_linear_speed_mps=speed_mps if command >= 0.0 else -speed_mps,
                    accel_body_y_mps2=(0.5 if speed_mps > 0.0 else 0.0) if command >= 0.0 else (-0.5 if speed_mps > 0.0 else 0.0),
                    left_encoder_count=left_count,
                    right_encoder_count=right_count,
                    ukf_state_py_m=py_m,
                )
            )
        return rows

    def test_summarize_launch_floor_reports_backlash_safe_breakaway(self) -> None:
        launch_rows_by_repeat = {
            1: self.make_backlash_repeat(0.17, 100, 120, 0.2250),
            2: self.make_backlash_repeat(-0.17, 200, 220, 0.2250),
            3: self.make_clear_repeat(0.18, 300, 320, 0.2250),
            4: self.make_clear_repeat(-0.18, 400, 420, 0.2250),
            5: self.make_clear_repeat(0.19, 500, 520, 0.2250),
            6: self.make_clear_repeat(-0.19, 600, 620, 0.2250),
        }

        summary = open_floor_launch_floor.summarize_launch_floor(
            launch_rows_by_repeat=launch_rows_by_repeat,
            accel_bias_y_mps2=0.0,
        )

        self.assertIsNotNone(summary)
        assert summary is not None
        self.assertAlmostEqual(summary.speed_quantum_mps, 0.003, delta=1.0e-6)
        self.assertAlmostEqual(summary.sustained_speed_threshold_mps, 0.01, delta=1.0e-9)
        self.assertEqual(summary.backlash_repeat_count, 2)
        self.assertAlmostEqual(summary.observed_clear_breakaway_command or 0.0, 0.18, delta=1.0e-9)
        self.assertAlmostEqual(summary.effective_launch_floor_command or 0.0, 0.18, delta=1.0e-9)
        command_summaries = {summary.abs_command: summary for summary in summary.command_summaries}
        self.assertEqual(command_summaries[0.17].clear_motion_count, 0)
        self.assertEqual(command_summaries[0.18].clear_motion_count, 2)
        self.assertEqual(command_summaries[0.19].clear_motion_count, 2)

    def test_nonmonotonic_clear_motion_is_flagged(self) -> None:
        launch_rows_by_repeat = {
            1: self.make_clear_repeat(0.17, 100, 120, 0.2250),
            2: self.make_clear_repeat(-0.17, 200, 220, 0.2250),
            3: self.make_backlash_repeat(0.18, 300, 320, 0.2250),
            4: self.make_backlash_repeat(-0.18, 400, 420, 0.2250),
            5: self.make_clear_repeat(0.19, 500, 520, 0.2250),
            6: self.make_clear_repeat(-0.19, 600, 620, 0.2250),
        }

        summary = open_floor_launch_floor.summarize_launch_floor(
            launch_rows_by_repeat=launch_rows_by_repeat,
            accel_bias_y_mps2=0.0,
        )

        self.assertIsNotNone(summary)
        assert summary is not None
        self.assertTrue(summary.nonmonotonic_clear_motion)


if __name__ == "__main__":
    unittest.main()
