import unittest
from pathlib import Path

import open_floor_plant_fit


TEST_TEMP_ROOT = Path(__file__).resolve().parent


class OpenFloorPlantFitTest(unittest.TestCase):
    def make_launch_row(
        self,
        dt_us: int,
        command: float,
        measured_linear_speed_mps: float,
        encoder_omega_radps: float,
        accel_body_y_mps2: float = 0.0,
    ) -> dict[str, str]:
        return {
            "dt_us": str(dt_us),
            "left_drive_command": f"{command}",
            "right_drive_command": f"{command}",
            "measured_linear_speed_mps": f"{measured_linear_speed_mps}",
            "left_encoder_omega_radps": f"{encoder_omega_radps}",
            "right_encoder_omega_radps": f"{encoder_omega_radps}",
            "gyro_raw_radps": "0.0",
            "accel_body_y_mps2": f"{accel_body_y_mps2}",
            "saturation_flags": "0",
            "clipping_flags": "0",
            "watchdog_flags": "0",
        }

    def test_load_run_id_accepts_prefixed_fields(self) -> None:
        control_log_path = TEST_TEMP_ROOT / "_open_floor_plant_fit_test_run_id_logging.txt"
        self.addCleanup(lambda: control_log_path.unlink(missing_ok=True))
        control_log_path.write_text(
            "open_floor_measurement [1] run_start: run_id=ofm_test;fan_duty_cycle_start=0.8\n",
            encoding="utf-8",
        )
        self.assertEqual(open_floor_plant_fit.load_run_id(control_log_path), "ofm_test")

    def test_summarize_tire_plant_fit_reports_current_run_apparent_launch_parameters(self) -> None:
        command = 0.30
        dt_us = 10000
        encoder_omega_radps = [0.0, 0.1, 0.2, 0.9, 1.0, 1.1, 1.2]
        measured_speed_mps = [0.0, 0.0, 0.00004, 0.00008, 0.00015, 0.00022, 0.000288]

        launch_rows_by_repeat: dict[int, list[dict[str, str]]] = {}
        for repeat_index in (0, 1):
            launch_rows_by_repeat[repeat_index] = [
                self.make_launch_row(dt_us, command, speed_mps, omega_radps)
                for speed_mps, omega_radps in zip(measured_speed_mps, encoder_omega_radps)
            ]

        control_log_path = TEST_TEMP_ROOT / "_open_floor_plant_fit_test_summary_logging.txt"
        self.addCleanup(lambda: control_log_path.unlink(missing_ok=True))
        control_log_path.write_text(
            "\n".join(
                [
                    "open_floor_measurement [1] run_start: run_id=ofm_test;fan_duty_cycle_start=0.8",
                    (
                        "open_floor_main [2] ukf_dump_params_mass_geometry: "
                        "mass_kg=10.0;effective_longitudinal_mass_kg=10.0;yaw_inertia_kg_m2=0.001;"
                        "track_width_m=0.1;contact_patch_longitudinal_offset_m=0.0;wheel_radius_m=0.1;"
                        "equivalent_wheel_inertia_kg_m2=0.0001"
                    ),
                    (
                        "open_floor_main [3] ukf_dump_params_drive_electrical: "
                        "supply_voltage_v=8.0;drive_resistance_ohms=4.0;torque_constant_nm_per_a=0.01;"
                        "speed_constant_radps_per_volt=1000000000.0;no_load_current_a=0.0;gear_ratio=1.0"
                    ),
                ]
            ),
            encoding="utf-8",
        )

        summary = open_floor_plant_fit.summarize_tire_plant_fit(
            launch_rows_by_repeat=launch_rows_by_repeat,
            gyro_bias_radps=0.0,
            accel_bias_y_mps2=0.0,
            control_log_path=control_log_path,
            available_section_ids={1, 2, 3},
        )

        self.assertIsNotNone(summary)
        assert summary is not None
        self.assertEqual(summary.run_id, "ofm_test")
        self.assertEqual(summary.launch_command_bin_count, 1)
        self.assertAlmostEqual(summary.apparent_equivalent_wheel_inertia_kg_m2 or 0.0, 1.0e-4, delta=1.0e-7)
        self.assertEqual(summary.apparent_equivalent_wheel_inertia_bin_count, 1)
        self.assertAlmostEqual(summary.apparent_rolling_friction_torque_nm or 0.0, 0.002, delta=1.0e-9)
        self.assertAlmostEqual(summary.apparent_viscous_friction_nm_per_radps or 0.0, 0.0005, delta=1.0e-9)
        self.assertEqual(summary.apparent_drag_fit_row_count, 2)
        self.assertFalse(summary.can_identify_cornering_stiffness)
        self.assertFalse(summary.can_identify_lateral_damping)
        self.assertFalse(summary.can_identify_peak_friction)
        self.assertIsNotNone(summary.lateral_identifiability_reason)

    def test_summarize_feedforward_alignment_inverts_configured_command(self) -> None:
        command = 0.30
        dt_us = 10000
        encoder_omega_radps = [1.0] * 7
        measured_speed_mps = [0.0, 0.00008, 0.00016, 0.00024, 0.00032, 0.00040, 0.00048]

        launch_rows_by_repeat: dict[int, list[dict[str, str]]] = {}
        for repeat_index in (0, 1):
            launch_rows_by_repeat[repeat_index] = [
                self.make_launch_row(dt_us, command, speed_mps, omega_radps)
                for speed_mps, omega_radps in zip(measured_speed_mps, encoder_omega_radps)
            ]

        control_log_path = TEST_TEMP_ROOT / "_open_floor_plant_fit_test_alignment_logging.txt"
        self.addCleanup(lambda: control_log_path.unlink(missing_ok=True))
        control_log_path.write_text(
            "\n".join(
                [
                    "open_floor_measurement [1] run_start: run_id=ofm_alignment_test;fan_duty_cycle_start=0.8",
                    (
                        "open_floor_main [2] ukf_dump_params_mass_geometry: "
                        "mass_kg=10.0;effective_longitudinal_mass_kg=10.0;yaw_inertia_kg_m2=0.001;"
                        "track_width_m=0.1;contact_patch_longitudinal_offset_m=0.0;wheel_radius_m=0.1;"
                        "equivalent_wheel_inertia_kg_m2=0.0001"
                    ),
                    (
                        "open_floor_main [3] ukf_dump_params_drive_electrical: "
                        "supply_voltage_v=8.0;drive_resistance_ohms=4.0;torque_constant_nm_per_a=0.01;"
                        "speed_constant_radps_per_volt=1000000000.0;no_load_current_a=0.0;motor_current_limit_a=2.0;"
                        "gear_ratio=1.0"
                    ),
                    (
                        "open_floor_main [4] ukf_dump_params_tire_friction: "
                        "drivetrain_efficiency=1.0;rolling_friction_torque_nm=0.002;"
                        "viscous_friction_nm_per_radps=0.0;longitudinal_tire_stiffness_n=6.0"
                    ),
                    (
                        "open_floor_main [5] ukf_dump_params_static_friction: "
                        "static_friction_torque_nm=0.0;static_friction_max_speed_mps=0.005"
                    ),
                ]
            ),
            encoding="utf-8",
        )

        summary = open_floor_plant_fit.summarize_feedforward_alignment(
            launch_rows_by_repeat=launch_rows_by_repeat,
            control_log_path=control_log_path,
        )

        self.assertIsNotNone(summary)
        assert summary is not None
        self.assertEqual(summary.run_id, "ofm_alignment_test")
        self.assertEqual(summary.command_bin_count, 1)
        self.assertEqual(summary.sample_count, 5)
        self.assertAlmostEqual(summary.overall_mean_command_error, 0.0, delta=1.0e-9)
        self.assertAlmostEqual(summary.overall_rmse_command_error, 0.0, delta=1.0e-9)
        self.assertEqual(len(summary.command_summaries), 1)
        command_summary = summary.command_summaries[0]
        self.assertEqual(command_summary.sample_count, 5)
        self.assertAlmostEqual(command_summary.required_command_p10, command, delta=1.0e-9)
        self.assertAlmostEqual(command_summary.required_command_median, command, delta=1.0e-9)
        self.assertAlmostEqual(command_summary.required_command_p90, command, delta=1.0e-9)
        self.assertAlmostEqual(command_summary.steady_required_command_median or 0.0, command, delta=1.0e-9)
        self.assertAlmostEqual(command_summary.mean_command_error, 0.0, delta=1.0e-9)
        self.assertAlmostEqual(command_summary.rmse_command_error, 0.0, delta=1.0e-9)


if __name__ == "__main__":
    unittest.main()
