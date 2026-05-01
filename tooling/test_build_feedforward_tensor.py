import csv
import math
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import build_feedforward_tensor as fft


class BuildFeedforwardTensorTest(unittest.TestCase):
    def write_csv(self, path: Path, header: list[str], rows: list[list[object]]) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", newline="", encoding="utf-8") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(header)
            writer.writerows(rows)

    def test_iter_open_floor_main_samples_uses_sensor_transition(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            csv_path = Path(temp_dir) / "open_floor_main.csv"
            self.write_csv(
                csv_path,
                [
                    "section_id",
                    "primitive_id",
                    "repeat_index",
                    "dt_us",
                    "saturation_flags",
                    "watchdog_flags",
                    "left_drive_command",
                    "right_drive_command",
                    "left_encoder_velocity_mps",
                    "right_encoder_velocity_mps",
                    "gyro_raw_radps",
                    "gyro_bias_radps",
                ],
                [
                    [1, 7, 2, 10000, 0, 0, 0.30, 0.50, 0.18, 0.22, 0.12, 0.02],
                    [1, 7, 2, 10000, 0, 0, 0.31, 0.51, 0.22, 0.26, 0.10, 0.04],
                ],
            )

            samples = list(fft.iter_open_floor_main_samples(csv_path))

            self.assertEqual(len(samples), 1)
            sample = samples[0]
            self.assertAlmostEqual(sample.present_velocity_mps, 0.20, places=9)
            self.assertAlmostEqual(sample.present_yaw_rate_radps, 0.10, places=9)
            self.assertAlmostEqual(sample.desired_accel_mps2, 4.0, places=9)
            self.assertAlmostEqual(sample.desired_alpha_radps2, -4.0, places=9)
            self.assertAlmostEqual(sample.left_raw_command, 0.30, places=9)
            self.assertAlmostEqual(sample.right_raw_command, 0.50, places=9)

    def test_build_feedforward_tensor_aggregates_recognized_sources(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)

            self.write_csv(
                root / "decoded" / "open_floor_main.csv",
                [
                    "section_id",
                    "primitive_id",
                    "repeat_index",
                    "dt_us",
                    "saturation_flags",
                    "watchdog_flags",
                    "left_drive_command",
                    "right_drive_command",
                    "left_encoder_velocity_mps",
                    "right_encoder_velocity_mps",
                    "gyro_raw_radps",
                    "gyro_bias_radps",
                ],
                [
                    [1, 1, 1, 10000, 0, 0, 0.25, 0.35, 0.18, 0.22, 0.24, 0.04],
                    [1, 1, 1, 10000, 0, 0, 0.00, 0.00, 0.22, 0.26, 0.10, 0.02],
                ],
            )

            diag_path = root / "Competition Testing Data" / "diag000.csv"
            diag_path.parent.mkdir(parents=True, exist_ok=True)
            diag_path.write_text(
                "\n".join(
                    [
                        "# phase,4,1000,forward_probe",
                        "sample,phase_id,t_us,dt_us,left_drive_cmd,right_drive_cmd,left_velocity_mps,right_velocity_mps,gyro_raw_radps,gyro_bias_radps",
                        "1,4,0,20000,-0.20,0.40,0.55,0.65,-0.10,0.10",
                        "2,4,20000,20000,-0.10,0.20,0.51,0.61,0.02,-0.02",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )

            self.write_csv(
                root / "replay" / "sensor_feedforward.csv",
                [
                    "dt_us",
                    "saturation_flags",
                    "logged_left_drive_command",
                    "logged_right_drive_command",
                    "current_forward_sensor_mps",
                    "current_yaw_rate_sensor_radps",
                    "target_forward_sensor_mps",
                    "target_yaw_rate_sensor_radps",
                ],
                [
                    [5000, 0, 0.60, 0.70, 0.30, 0.40, 0.32, 0.45],
                ],
            )

            self.write_csv(
                root / "replay" / "feedforward_paths.csv",
                [
                    "path_id",
                    "dt_us",
                    "saturation_flags",
                    "logged_left_drive_command",
                    "logged_right_drive_command",
                    "current_forward_sensor_mps",
                    "current_yaw_rate_sensor_radps",
                    "target_forward_sensor_mps",
                    "target_yaw_rate_sensor_radps",
                ],
                [
                    ["wrong_path", 5000, 0, 0.10, 0.20, 0.10, 0.10, 0.11, 0.11],
                    ["state_closed_velocity", 5000, 0, 0.80, 0.90, 0.40, -0.20, 0.35, -0.10],
                ],
            )

            tensor = fft.build_feedforward_tensor(
                root,
                fft.TensorBuildConfig(
                    velocity_bins=2,
                    yaw_bins=2,
                    accel_bins=2,
                    alpha_bins=2,
                    reservoir_size=64,
                    random_seed=0,
                    feedforward_path_id="state_closed_velocity",
                ),
            )

            self.assertEqual(tensor.sample_count, 4)
            self.assertEqual(tensor.source_stats["open_floor_main"].sample_count, 1)
            self.assertEqual(tensor.source_stats["competition_diag"].sample_count, 1)
            self.assertEqual(tensor.source_stats["sensor_feedforward"].sample_count, 1)
            self.assertEqual(tensor.source_stats["feedforward_paths:state_closed_velocity"].sample_count, 1)

            sensor_sample = fft.FeedforwardSample(
                present_velocity_mps=0.30,
                present_yaw_rate_radps=0.40,
                desired_accel_mps2=4.0,
                desired_alpha_radps2=10.0,
                left_raw_command=0.60,
                right_raw_command=0.70,
                dt_seconds=0.005,
                source_kind="sensor_feedforward",
                source_path="synthetic",
            )
            flat_index = tensor.flat_index(fft.axis_index(sensor_sample, tensor.axes))
            self.assertAlmostEqual(tensor.left_command_mean[flat_index], 0.60, places=9)
            self.assertAlmostEqual(tensor.right_command_mean[flat_index], 0.70, places=9)

    def test_feedforward_tensor_evaluate_interpolates_and_falls_back(self) -> None:
        tensor = fft.FeedforwardTensor(
            axes=[
                fft.TensorAxis("present_velocity_mps", [0.0, 1.0]),
                fft.TensorAxis("present_yaw_rate_radps", [0.0]),
                fft.TensorAxis("desired_accel_mps2", [0.0]),
                fft.TensorAxis("desired_alpha_radps2", [0.0]),
            ],
            left_command_mean=[0.20, 0.60],
            right_command_mean=[0.30, 0.90],
            left_command_std=[0.0, 0.0],
            right_command_std=[0.0, 0.0],
            counts=[1, 1],
            source_stats={},
            sample_count=2,
            occupied_cell_count=2,
        )

        left_command, right_command = tensor.evaluate(
            present_velocity_mps=0.25,
            present_yaw_rate_radps=0.0,
            desired_accel_mps2=0.0,
            desired_alpha_radps2=0.0,
        )
        self.assertAlmostEqual(left_command, 0.30, places=9)
        self.assertAlmostEqual(right_command, 0.45, places=9)

        sparse_tensor = fft.FeedforwardTensor(
            axes=tensor.axes,
            left_command_mean=[0.20, math.nan],
            right_command_mean=[0.30, math.nan],
            left_command_std=[0.0, math.nan],
            right_command_std=[0.0, math.nan],
            counts=[1, 0],
            source_stats={},
            sample_count=1,
            occupied_cell_count=1,
        )
        left_command, right_command = sparse_tensor.evaluate(
            present_velocity_mps=0.90,
            present_yaw_rate_radps=0.0,
            desired_accel_mps2=0.0,
            desired_alpha_radps2=0.0,
        )
        self.assertAlmostEqual(left_command, 0.20, places=9)
        self.assertAlmostEqual(right_command, 0.30, places=9)


if __name__ == "__main__":
    unittest.main()
