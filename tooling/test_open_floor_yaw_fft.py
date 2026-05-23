import cmath
import math
import unittest
from pathlib import Path

import open_floor_yaw_fft


TEST_TEMP_ROOT = Path(__file__).resolve().parent


class OpenFloorYawFftTest(unittest.TestCase):
    def test_summarize_yaw_fft_recovers_frequency_phase_and_wheel_side_inertia(self) -> None:
        wheel_radius_m = 0.1
        track_width_m = 0.2
        yaw_inertia_kg_m2 = 2.0e-6
        wheel_side_inertia_kg_m2 = 1.2e-7
        configured_equivalent_wheel_inertia_kg_m2 = 1.6e-5
        rigid_body_equivalent_inertia_kg_m2 = (
            (2.0 * wheel_radius_m * wheel_radius_m * yaw_inertia_kg_m2) /
            (track_width_m * track_width_m)
        )
        damping_nm_per_radps = 8.0e-6
        wheel_speed_amplitude_radps = 1000.0
        coupling_magnitude = 0.4
        coupling_phase_deg = -70.0
        dominant_frequency_hz = 8.0
        angular_frequency_radps = 2.0 * math.pi * dominant_frequency_hz
        turn_gain = (2.0 * wheel_radius_m) / track_width_m
        coupling = coupling_magnitude * cmath.exp(1.0j * math.radians(coupling_phase_deg))
        wheel_speed_phasor = complex(wheel_speed_amplitude_radps, 0.0)
        torque_phasor = (
            damping_nm_per_radps +
            (1.0j * angular_frequency_radps * (wheel_side_inertia_kg_m2 + (rigid_body_equivalent_inertia_kg_m2 * coupling)))
        ) * wheel_speed_phasor
        expected_wheel_speed_vs_torque_phase_deg = math.degrees(
            cmath.phase(wheel_speed_phasor / torque_phasor)
        )

        sample_count = 4096
        dt_us = 1000
        torque_command_gain_nm_per_command = 0.02
        yaw_rows_by_key: dict[tuple[int, int], list[dict[str, str]]] = {(8, 2): []}
        for sample_index in range(sample_count):
            time_s = sample_index * (dt_us * 1.0e-6)
            rotation = cmath.exp(1.0j * angular_frequency_radps * time_s)
            wheel_speed_radps = (wheel_speed_phasor * rotation).real
            yaw_rate_radps = (turn_gain * coupling * wheel_speed_phasor * rotation).real
            differential_torque_nm = (torque_phasor * rotation).real
            left_command = differential_torque_nm / torque_command_gain_nm_per_command
            right_command = -left_command
            yaw_rows_by_key[(8, 2)].append(
                {
                    "section_id": "4",
                    "primitive_id": "8",
                    "repeat_index": "2",
                    "phase_id": "9",
                    "dt_us": str(dt_us),
                    "saturation_flags": "0",
                    "left_drive_command": f"{left_command}",
                    "right_drive_command": f"{right_command}",
                    "left_encoder_omega_radps": f"{wheel_speed_radps}",
                    "right_encoder_omega_radps": f"{-wheel_speed_radps}",
                    "gyro_radps": f"{yaw_rate_radps}",
                }
            )

        control_log_path = TEST_TEMP_ROOT / "_open_floor_yaw_fft_primary_logging.txt"
        fallback_control_log_path = TEST_TEMP_ROOT / "_open_floor_yaw_fft_fallback_logging.txt"
        self.addCleanup(lambda: control_log_path.unlink(missing_ok=True))
        self.addCleanup(lambda: fallback_control_log_path.unlink(missing_ok=True))
        control_log_path.write_text(
            "\n".join(
                [
                    "open_floor_measurement [1] run_start: run_id=ofm_yaw_fft_test;fan_duty_cycle_start=0.8",
                    (
                        "open_floor_main [2] plant_dump_params_mass_geometry: "
                        f"mass_kg=0.140;effective_longitudinal_mass_kg=0.140;yaw_inertia_kg_m2={yaw_inertia_kg_m2};"
                        f"track_width_m={track_width_m};contact_patch_longitudinal_offset_m=0.0;wheel_radius_m={wheel_radius_m};"
                        f"equivalent_wheel_inertia_kg_m2={configured_equivalent_wheel_inertia_kg_m2}"
                    ),
                ]
            ),
            encoding="utf-8",
        )
        fallback_control_log_path.write_text(
            "\n".join(
                [
                    (
                        "open_floor_main [2] plant_dump_params_mass_geometry: "
                        f"mass_kg=0.140;effective_longitudinal_mass_kg=0.140;yaw_inertia_kg_m2={yaw_inertia_kg_m2};"
                        f"track_width_m={track_width_m};contact_patch_longitudinal_offset_m=0.0;wheel_radius_m={wheel_radius_m};"
                        f"equivalent_wheel_inertia_kg_m2={configured_equivalent_wheel_inertia_kg_m2}"
                    ),
                    (
                        "open_floor_main [3] plant_dump_params_drive_electrical: "
                        "supply_voltage_v=8.0;drive_resistance_ohms=4.0;torque_constant_nm_per_a=0.01;"
                        "speed_constant_radps_per_volt=1000000000000.0;no_load_current_a=0.0;gear_ratio=1.0"
                    ),
                ]
            ),
            encoding="utf-8",
        )

        summary = open_floor_yaw_fft.summarize_yaw_fft(
            yaw_rows_by_key,
            control_log_path,
            fallback_control_log_path,
        )

        self.assertIsNotNone(summary)
        assert summary is not None
        self.assertEqual(summary.run_id, "ofm_yaw_fft_test")
        self.assertTrue(summary.used_fallback_drive_parameters)
        self.assertEqual(summary.drive_parameter_source_path, fallback_control_log_path)
        self.assertAlmostEqual(
            summary.rigid_body_equivalent_inertia_kg_m2,
            rigid_body_equivalent_inertia_kg_m2,
            delta=1.0e-12,
        )
        self.assertEqual(len(summary.repeat_summaries), 1)
        repeat = summary.repeat_summaries[0]
        self.assertTrue(repeat.strong_peak)
        self.assertAlmostEqual(repeat.dominant_frequency_hz, dominant_frequency_hz, delta=0.3)
        self.assertAlmostEqual(
            repeat.wheel_speed_vs_torque_phase_deg,
            expected_wheel_speed_vs_torque_phase_deg,
            delta=3.0,
        )
        self.assertAlmostEqual(repeat.yaw_vs_rigid_wheel_phase_deg, coupling_phase_deg, delta=3.0)
        self.assertAlmostEqual(repeat.yaw_coupling_magnitude, coupling_magnitude, delta=0.05)
        self.assertAlmostEqual(
            repeat.phase_corrected_wheel_inertia_kg_m2,
            wheel_side_inertia_kg_m2,
            delta=7.0e-8,
        )
        self.assertIsNotNone(summary.aggregate)
        assert summary.aggregate is not None
        self.assertEqual(summary.aggregate.strong_repeat_count, 1)
        self.assertAlmostEqual(
            summary.aggregate.recommended_wheel_inertia_kg_m2 or 0.0,
            wheel_side_inertia_kg_m2,
            delta=7.0e-8,
        )


if __name__ == "__main__":
    unittest.main()
