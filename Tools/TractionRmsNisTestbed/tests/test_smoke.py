from __future__ import annotations

import csv
import json
import sys
import tempfile
import unittest
from pathlib import Path

TOOL_ROOT = Path(__file__).resolve().parents[1]
if str(TOOL_ROOT) not in sys.path:
    sys.path.insert(0, str(TOOL_ROOT))

from traction_rms_nis_testbed.estimator_core import (
    CandidateConfig,
    CovarianceConfig,
    SegmentSpec,
    VehicleConfig,
    build_segments_from_args,
    load_candidates,
    load_covariance,
    representative_row_indices,
    run_replay,
    run_ukf_validation,
    sample_from_row,
)


REPO_ROOT = Path(__file__).resolve().parents[3]
STAGING = REPO_ROOT / "staging" / "traction_candidate_rms_nis_testbed"


class Args:
    candidate_config = ""
    covariance_config = ""
    input_csv = ""
    segment_manifest = str(STAGING / "smoke_segments.json")
    output_dir = ""
    segment_id = ""
    stage = ""
    split = ""
    run_id = ""
    max_segments = 0
    max_rows_per_segment = 0


class TractionRmsNisSmokeTest(unittest.TestCase):
    def test_smoke_replay_defaults_to_aggregate_artifacts_without_ukf_state_columns(self) -> None:
        candidates = load_candidates(STAGING / "candidates.json")
        vehicle, covariance = load_covariance(STAGING / "covariance_conservative.json")
        segments = build_segments_from_args(Args(), REPO_ROOT)
        with tempfile.TemporaryDirectory() as temp_dir:
            output_dir = Path(temp_dir)

            summary = run_replay(candidates, vehicle, covariance, segments, output_dir)

            self.assertEqual(summary["processed_segments"], 1)
            self.assertEqual(summary["processed_samples"], 5)
            self.assertEqual(summary["skipped_corrupted_segments"], 0)
            self.assertFalse(summary["row_artifacts_enabled"])
            aggregate_path = output_dir / "nis_aggregates.csv"
            itemized_path = output_dir / "itemized_rms_nis.csv"
            candidate_rms_path = output_dir / "candidate_rms_nis.csv"
            summary_path = output_dir / "summary.json"
            self.assertTrue(aggregate_path.exists())
            self.assertTrue(itemized_path.exists())
            self.assertTrue(candidate_rms_path.exists())
            self.assertTrue(summary_path.exists())
            self.assertFalse((output_dir / "nis_samples.csv").exists())
            self.assertFalse((output_dir / "residual_diagnostics.csv").exists())

            with aggregate_path.open(newline="", encoding="utf-8") as handle:
                reader = csv.DictReader(handle)
                self.assertIsNotNone(reader.fieldnames)
                assert reader.fieldnames is not None
                self.assertFalse(any(name.startswith("ukf_state") for name in reader.fieldnames))
                rows = list(reader)

            self.assertGreater(len(rows), 0)
            self.assertEqual(
                {
                    "left_encoder_wheel_rate_nis",
                    "right_encoder_wheel_rate_nis",
                    "yaw_rate_nis",
                    "forward_accel_nis",
                    "right_accel_nis",
                },
                {row["log_parameter"] for row in rows},
            )
            self.assertIn("rejected_count", rows[0])

            loaded_summary = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertFalse(loaded_summary["uses_logged_ukf_state"])
            self.assertIn("candidate_1_algebraic_envelope", loaded_summary["candidates"])
            self.assertGreater(
                loaded_summary["candidates"]["candidate_1_algebraic_envelope"]["nis_count"],
                0,
            )
            self.assertIn(
                "yaw_accel_radps2",
                loaded_summary["candidates"]["candidate_1_algebraic_envelope"]["residual_rms"],
            )

    def test_row_artifacts_remain_available_for_debug_runs(self) -> None:
        candidates = load_candidates(STAGING / "candidates.json")
        vehicle, covariance = load_covariance(STAGING / "covariance_conservative.json")
        segments = build_segments_from_args(Args(), REPO_ROOT)
        with tempfile.TemporaryDirectory() as temp_dir:
            output_dir = Path(temp_dir)

            summary = run_replay(
                candidates,
                vehicle,
                covariance,
                segments,
                output_dir,
                write_row_artifacts=True,
            )

            self.assertTrue(summary["row_artifacts_enabled"])
            nis_path = output_dir / "nis_samples.csv"
            diagnostics_path = output_dir / "residual_diagnostics.csv"
            self.assertTrue(nis_path.exists())
            self.assertTrue(diagnostics_path.exists())

            with nis_path.open(newline="", encoding="utf-8") as handle:
                reader = csv.DictReader(handle)
                self.assertIsNotNone(reader.fieldnames)
                assert reader.fieldnames is not None
                self.assertFalse(any(name.startswith("ukf_state") for name in reader.fieldnames))
                rows = list(reader)

            self.assertGreater(len(rows), 0)
            self.assertIn("false", {row["accepted"] for row in rows})
            self.assertIn("true", {row["rejected"] for row in rows})

            with diagnostics_path.open(newline="", encoding="utf-8") as handle:
                reader = csv.DictReader(handle)
                self.assertIsNotNone(reader.fieldnames)
                assert reader.fieldnames is not None
                self.assertIn("max_contact_relative_speed_mps", reader.fieldnames)
                self.assertIn("measured_yaw_accel_radps2", reader.fieldnames)
                self.assertIn("yaw_accel_residual_radps2", reader.fieldnames)
                self.assertFalse(any(name.startswith("ukf_state") for name in reader.fieldnames))

    def test_all_expected_candidate_models_are_available(self) -> None:
        candidates = load_candidates(STAGING / "candidates.json")
        ids = {candidate.candidate_id for candidate in candidates}
        models = {candidate.model for candidate in candidates}
        self.assertEqual(
            {
                "baseline/current_holdover",
                "candidate_1_algebraic_envelope",
                "candidate_2_stribeck",
                "candidate_3_load_sensitive",
            },
            ids,
        )
        self.assertEqual(
            {
                "current_holdover_approximation",
                "algebraic_envelope",
                "stribeck_algebraic",
                "load_sensitive_anisotropic",
            },
            models,
        )

    def test_compact_log_nan_commands_are_clamped_for_replay(self) -> None:
        spec = SegmentSpec(
            log_path=Path("compact.csv"),
            segment_id="compact",
            stage="launch",
            split="train",
        )
        row = {
            "master_time_us": "1000",
            "dt_us": "1000",
            "left_drive_command": "nan",
            "right_drive_command": "nan",
            "left_encoder_wheel_speed_radps": "12.5",
            "right_encoder_wheel_speed_radps": "13.5",
            "gyro_radps": "0.25",
            "accel_body_forward_mps2": "1.0",
            "accel_body_right_mps2": "0.5",
            "accel_bias_valid": "1",
        }

        sample = sample_from_row(row, spec, 0, None, VehicleConfig())

        self.assertEqual(sample.left_command, 0.0)
        self.assertEqual(sample.right_command, 0.0)
        self.assertEqual(sample.left_wheel_rate_radps, 12.5)
        self.assertEqual(sample.right_wheel_rate_radps, 13.5)
        self.assertTrue(sample.gyro_valid)
        self.assertTrue(sample.accel_valid)

    def test_representative_row_indices_cover_start_middle_and_end(self) -> None:
        self.assertEqual(
            representative_row_indices(10, 18, 3),
            {10, 14, 18},
        )

    def test_ukf_validation_writes_report_without_logged_ukf_state(self) -> None:
        candidates = load_candidates(STAGING / "candidates.json")
        vehicle, covariance = load_covariance(STAGING / "covariance_conservative.json")
        segments = build_segments_from_args(Args(), REPO_ROOT)
        with tempfile.TemporaryDirectory() as temp_dir:
            output_dir = Path(temp_dir)

            summary = run_ukf_validation(
                candidates,
                vehicle,
                covariance,
                segments,
                output_dir,
                max_rows_per_segment=3,
            )

            self.assertTrue(summary["passed"])
            self.assertEqual(summary["processed_segments"], 1)
            self.assertEqual(summary["processed_samples"], 3)
            self.assertFalse(summary["uses_logged_ukf_state"])
            self.assertTrue((output_dir / "ukf_validation_summary.json").exists())
            self.assertTrue((output_dir / "ukf_validation_candidate_summary.csv").exists())
            self.assertTrue((output_dir / "ukf_validation_events.csv").exists())
            self.assertTrue((output_dir / "ukf_validation_report.md").exists())
            for candidate_summary in summary["candidates"].values():
                self.assertEqual(candidate_summary["status"], "pass")
                self.assertGreater(candidate_summary["sigma_points"], 0)
                self.assertGreater(candidate_summary["zero_crossing_events"], 0)

    def test_ukf_validation_fails_pathological_sigma_instability(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            log_path = root / "unstable.csv"
            log_path.write_text(
                "\n".join(
                    [
                        "master_time_us,dt_us,left_drive_command,right_drive_command,left_encoder_omega_radps,right_encoder_omega_radps,gyro_radps,accel_body_y_mps2,accel_body_x_mps2,accel_bias_valid,fan_duty_cycle",
                        "1000,1000,0,0,100,100,0,0,0,1,0.8",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )
            segment = SegmentSpec(
                log_path=log_path,
                segment_id="unstable",
                stage="unit",
                split="validation",
            )
            candidate = CandidateConfig(
                candidate_id="pathological",
                label="pathological",
                model="algebraic_envelope",
                parameters={"longitudinal_slip_gain_n_per_mps": 1.0e300},
            )

            summary = run_ukf_validation(
                [candidate],
                VehicleConfig(),
                CovarianceConfig(),
                [segment],
                root / "out",
                max_rows_per_segment=1,
            )

            candidate_summary = summary["candidates"]["pathological"]
            self.assertFalse(summary["passed"])
            self.assertEqual(candidate_summary["status"], "fail")
            self.assertGreater(candidate_summary["finite_failures"], 0)


if __name__ == "__main__":
    unittest.main()
