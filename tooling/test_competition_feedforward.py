import unittest
from pathlib import Path

from competition_feedforward import breakaway_torque_nm_from_command
from competition_feedforward import CurrentFeedforwardSetup
from competition_feedforward import fully_successful_command_bounds
from competition_feedforward import analyze_competition_feedforward
from open_floor_plant_fit import LaunchFitParameters
from open_floor_recovery import RunPlantParameters


TEST_TEMP_ROOT = Path(__file__).resolve().parent


class CompetitionFeedforwardTest(unittest.TestCase):
    def make_setup(self) -> CurrentFeedforwardSetup:
        launch_fit = LaunchFitParameters(
            drive=RunPlantParameters(
                battery_voltage_v=8.0,
                drive_resistance_ohms=4.0,
                torque_constant_nm_per_a=0.01,
                speed_constant_radps_per_volt=1.0e9,
                no_load_current_a=0.0,
                gear_ratio=1.0,
                wheel_radius_m=0.1,
                nominal_track_width_m=0.1,
            ),
            effective_longitudinal_mass_kg=10.0,
            equivalent_wheel_inertia_kg_m2=0.0001,
            rolling_friction_torque_nm=0.002,
            static_friction_torque_nm=0.0,
            static_friction_max_speed_mps=0.005,
            viscous_friction_nm_per_radps=0.0,
            drivetrain_efficiency=1.0,
            motor_current_limit_a=2.0,
        )
        return CurrentFeedforwardSetup(
            launch_fit=launch_fit,
            wheel_static_feedforward=0.0,
            wheel_rest_launch_drive_command=0.30,
            wheel_rest_launch_max_drive_command=0.55,
            wheel_rest_launch_ramp_ms=250,
            wheel_rest_launch_speed_threshold_mps=0.02,
            wheel_rest_launch_drive_threshold=0.05,
            wheel_velocity_feedforward=0.0,
            wheel_acceleration_response_gain_per_mps2=0.20,
            wheel_velocity_kp=1.10,
            wheel_velocity_ki=1.50,
            wheel_integral_limit=0.25,
            plant_breakaway_drive_command=0.30,
        )

    def test_launch_success_bounds_bracket_reliable_threshold(self) -> None:
        lower_command, upper_command = fully_successful_command_bounds(
            {
                0.15: [0, 0, 0],
                0.20: [1, 0, 0],
                0.25: [1, 1, 0],
                0.30: [1, 1, 1],
                0.35: [1, 1, 1],
            }
        )
        self.assertEqual(lower_command, 0.25)
        self.assertEqual(upper_command, 0.30)
        self.assertAlmostEqual(
            breakaway_torque_nm_from_command(0.275, self.make_setup().launch_fit),
            0.0055,
            delta=1.0e-12,
        )

    def test_analyze_competition_feedforward_uses_hold_segment_and_outcomes(self) -> None:
        csv_path = TEST_TEMP_ROOT / "_competition_feedforward_test_diag.csv"
        self.addCleanup(lambda: csv_path.unlink(missing_ok=True))

        def sample_row(
            sample: int,
            phase_id: int,
            dt_us: int,
            linear_speed_mps: float,
            drive_command: float,
            wheel_velocity_mps: float,
        ) -> str:
            return (
                f"{sample},{phase_id},{sample * dt_us},{dt_us},{linear_speed_mps:.6f},"
                f"{drive_command:.4f},{drive_command:.4f},{wheel_velocity_mps:.6f},{wheel_velocity_mps:.6f},0.0"
            )

        rows = [
            "# phase,4,1000,kickoff_030_probe",
            "# phase,5,2000,forward_010_probe",
            "sample,phase_id,t_us,dt_us,linear_speed_mps,left_drive_cmd,right_drive_cmd,left_velocity_mps,right_velocity_mps,gyro_raw_radps",
        ]

        kickoff_speeds = [0.0, 0.00008, 0.00016, 0.00024, 0.00032, 0.00040, 0.00048]
        sample_index = 1
        for speed in kickoff_speeds:
            rows.append(sample_row(sample_index, 4, 10000, speed, 0.30, 0.1))
            sample_index += 1
        rows.append(sample_row(sample_index, 4, 10000, 0.0, 0.0, 0.0))
        sample_index += 1

        for _ in range(3):
            rows.append(sample_row(sample_index, 5, 10000, 0.0, 0.35, 0.1))
            sample_index += 1
        for _ in range(7):
            rows.append(sample_row(sample_index, 5, 10000, 0.05, 0.10, 0.1))
            sample_index += 1
        rows.append(sample_row(sample_index, 5, 10000, 0.0, 0.0, 0.0))
        rows.extend(
            [
                "# event,3000,kickoff_result,kickoff_030;cmd=0.30;dist_m=0.0200;max_speed_mps=0.077;moved=1;travel_limited=0",
                "# event,4000,forward_result,forward_010;kickoff=0.35;hold=0.10;hold_dist_m=0.0099;hold_avg_speed_mps=0.045;total_dist_m=0.0128;max_speed_mps=0.084;carried=1;travel_limited=0",
            ]
        )
        csv_path.write_text("\n".join(rows) + "\n", encoding="utf-8")

        report = analyze_competition_feedforward([csv_path], self.make_setup())

        self.assertEqual(report.kickoff.source_file_count, 1)
        self.assertEqual(report.kickoff.trace_count, 1)
        self.assertEqual(report.forward_hold.source_file_count, 1)
        self.assertEqual(report.forward_hold.trace_count, 1)

        kickoff_alignment = report.kickoff.feedforward_alignment
        self.assertIsNotNone(kickoff_alignment)
        assert kickoff_alignment is not None
        self.assertEqual(kickoff_alignment.command_bin_count, 1)
        self.assertAlmostEqual(kickoff_alignment.command_summaries[0].required_command_median, 0.30, delta=1.0e-9)
        self.assertAlmostEqual(kickoff_alignment.overall_rmse_command_error, 0.0, delta=1.0e-9)

        forward_alignment = report.forward_hold.feedforward_alignment
        self.assertIsNotNone(forward_alignment)
        assert forward_alignment is not None
        self.assertEqual(forward_alignment.command_bin_count, 1)
        self.assertAlmostEqual(forward_alignment.command_summaries[0].required_command_median, 0.10, delta=1.0e-9)
        self.assertAlmostEqual(forward_alignment.overall_rmse_command_error, 0.0, delta=1.0e-9)

        self.assertEqual(report.kickoff.outcome_summaries[0].success_count, 1)
        self.assertEqual(report.kickoff.outcome_summaries[0].total_count, 1)
        self.assertEqual(report.forward_hold.outcome_summaries[0].success_count, 1)
        self.assertEqual(report.forward_hold.outcome_summaries[0].total_count, 1)


if __name__ == "__main__":
    unittest.main()
