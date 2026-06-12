import csv
import math
import sys
import tempfile
import unittest
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parent))

from traction_rms_nis_testbed import scoring as testbed


class TractionRmsNisTestbedTest(unittest.TestCase):
    def make_config(self) -> dict:
        return {
            "split": {
                "seed": "unit-test",
                "train": 0.6,
                "validation": 0.2,
                "held_out": 0.2,
            },
            "tuning": {
                "seed": "unit-test",
                "trial_count_per_candidate": 3,
                "include_nominal_trial": True,
            },
            "scoring": {
                "selection_split": "validation",
                "inflation_floor_ratio": 0.75,
                "under_expected_penalty_weight": 1.0,
                "inflation_floor_penalty_weight": 1.0,
                "log_fields": {
                    "yaw_rate_nis": {"dimension": 1, "weight": 1.0},
                    "forward_accel_nis": {"dimension": 1, "weight": 1.0},
                    "right_accel_nis": {"dimension": 1, "weight": 1.0},
                },
            },
            "candidates": [
                {
                    "id": "candidate_good",
                    "label": "Good candidate",
                    "model": "test_model",
                    "parameters": {"slip_gain": 12.0},
                    "search": {"slip_gain": {"min": 4.0, "max": 64.0, "scale": "log"}},
                },
                {
                    "id": "candidate_low_nis",
                    "label": "Low NIS candidate",
                    "model": "test_model",
                    "parameters": {"slip_gain": 12.0},
                    "search": {"slip_gain": {"min": 4.0, "max": 64.0, "scale": "log"}},
                },
            ],
        }

    def write_csv(self, path: Path, header: list[str], rows: list[list[object]]) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(header)
            writer.writerows(rows)

    def load_records(
        self,
        root: Path,
        config: dict,
        manifest_text: str,
    ) -> tuple[list[testbed.NisRecord], dict[str, testbed.SegmentInfo], list[testbed.TrialConfig]]:
        config_path = root / "config.json"
        config_path.write_text("{}\n", encoding="utf-8")
        manifest_path = root / "manifest.json"
        manifest_path.write_text(manifest_text, encoding="utf-8")
        candidates = testbed.load_candidates(config)
        trials = testbed.generate_trial_plan(config, candidates)
        records, segments = testbed.load_records(config, config_path, manifest_path, trials)
        return records, segments, trials

    def test_split_is_assigned_by_whole_segment(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            self.write_csv(
                root / "nis.csv",
                ["candidate_id", "segment_id", "stage", "log_field", "nis"],
                [
                    ["candidate_good", "same_segment", "SEC_20_LAUNCH", "yaw_rate_nis", 1.0],
                    ["candidate_good", "same_segment", "SEC_20_LAUNCH", "forward_accel_nis", 2.0],
                    ["candidate_low_nis", "same_segment", "SEC_20_LAUNCH", "yaw_rate_nis", 3.0],
                ],
            )
            records, segments, _trials = self.load_records(
                root,
                self.make_config(),
                '{"artifacts": [{"path": "nis.csv"}]}\n',
            )

            self.assertEqual(len({record.split for record in records}), 1)
            self.assertIn(segments["same_segment"].split, testbed.SPLITS)

    def test_corrupted_segments_are_excluded(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            self.write_csv(
                root / "nis.csv",
                ["candidate_id", "segment_id", "stage", "log_field", "nis"],
                [
                    ["candidate_good", "clean", "SEC_20_LAUNCH", "yaw_rate_nis", 1.0],
                    ["candidate_good", "bad", "SEC_20_LAUNCH", "yaw_rate_nis", 100.0],
                ],
            )
            records, segments, _trials = self.load_records(
                root,
                self.make_config(),
                """{
  "segments": [
    {"segment_id": "clean", "split": "validation", "stage": "SEC_20_LAUNCH"},
    {"segment_id": "bad", "split": "validation", "stage": "SEC_20_LAUNCH", "corrupted": true}
  ],
  "artifacts": [{"path": "nis.csv"}]
}
""",
            )

            self.assertEqual({record.segment_id for record in records}, {"clean"})
            self.assertTrue(segments["bad"].corrupted)

    def test_rejects_logged_ukf_state_columns(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            self.write_csv(
                root / "nis.csv",
                ["candidate_id", "segment_id", "stage", "log_field", "nis", "ukf_state_px_m"],
                [["candidate_good", "seg", "SEC_20_LAUNCH", "yaw_rate_nis", 1.0, 0.5]],
            )

            with self.assertRaises(testbed.TestbedConfigError):
                self.load_records(
                    root,
                    self.make_config(),
                    '{"artifacts": [{"path": "nis.csv"}]}\n',
                )

    def test_low_nis_is_penalized_for_candidate_ranking(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            expected = math.sqrt(3.0)
            self.write_csv(
                root / "nis.csv",
                ["candidate_id", "segment_id", "split", "stage", "log_field", "nis"],
                [
                    ["candidate_good", "seg_good", "validation", "SEC_40_YAW", "yaw_rate_nis", expected],
                    ["candidate_low_nis", "seg_low", "validation", "SEC_40_YAW", "yaw_rate_nis", 0.01],
                ],
            )
            config = self.make_config()
            records, _segments, trials = self.load_records(
                root,
                config,
                '{"artifacts": [{"path": "nis.csv"}]}\n',
            )
            candidates = testbed.load_candidates(config)
            itemized, trial_scores, rankings = testbed.evaluate_records(config, candidates, trials, records)
            score_by_candidate = {score.candidate_id: score for score in rankings}
            low_item = next(item for item in itemized if item.candidate_id == "candidate_low_nis")

            self.assertGreater(
                score_by_candidate["candidate_low_nis"].selection_score,
                score_by_candidate["candidate_good"].selection_score,
            )
            self.assertGreater(low_item.under_expected_penalty, 0.0)
            self.assertTrue(low_item.inflation_flag)
            self.assertTrue(any(score.sample_count > 0 for score in trial_scores))

    def test_trial_plan_stays_inside_bounds(self) -> None:
        candidates = testbed.load_candidates(self.make_config())
        trials = testbed.generate_trial_plan(self.make_config(), candidates)
        tuned = [trial for trial in trials if not trial.is_nominal]

        self.assertTrue(tuned)
        for trial in tuned:
            self.assertGreaterEqual(trial.parameters["slip_gain"], 4.0)
            self.assertLessEqual(trial.parameters["slip_gain"], 64.0)

    def test_rejects_covariance_tuning_fields(self) -> None:
        config = self.make_config()
        config["candidates"][0]["search"] = {
            "measurement_noise_scale": {"min": 0.1, "max": 2.0, "scale": "linear"}
        }

        with self.assertRaises(testbed.TestbedConfigError):
            testbed.load_candidates(config)

    def test_itemization_carries_parameter_and_launch_command_fields(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            self.write_csv(
                root / "nis.csv",
                ["candidate_id", "segment_id", "split", "stage", "log_field", "nis"],
                [["candidate_good", "launch_seg", "validation", "SEC_20_LAUNCH", "yaw_rate_nis", math.sqrt(3.0)]],
            )
            config = self.make_config()
            records, _segments, trials = self.load_records(
                root,
                config,
                """{
  "segments": [
    {
      "segment_id": "launch_seg",
      "split": "validation",
      "stage": "SEC_20_LAUNCH",
      "parameter_fields": {
        "parameter": "launch_drive_command",
        "test_value_kind": "drive_command_pair",
        "test_value": "0.24,0.24"
      },
      "observed_command": {
        "active_command_rows": 12,
        "observed_command_pair_mode": "0.24,0.24",
        "left_drive_command_median": 0.24,
        "right_drive_command_median": 0.24,
        "cmd_linear_mps_median": 0.31,
        "cmd_yaw_radps_median": 0.0,
        "observed_command_magnitude_median": 0.24
      }
    }
  ],
  "artifacts": [{"path": "nis.csv"}]
}
""",
            )
            candidates = testbed.load_candidates(config)
            itemized, _trial_scores, _rankings = testbed.evaluate_records(config, candidates, trials, records)
            row = next(item for item in itemized if item.split == "validation")

            self.assertEqual(row.parameter_field, "launch_drive_command")
            self.assertIn("pair=0.24,0.24", row.launch_command_signature)
            self.assertAlmostEqual(row.launch_cmd_linear_mps_median_mean, 0.31)

    def test_stage_and_channel_weights_are_used_for_selection_score(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            expected = math.sqrt(3.0)
            bad = expected * 3.0
            self.write_csv(
                root / "nis.csv",
                ["candidate_id", "segment_id", "split", "stage", "log_field", "nis"],
                [
                    ["candidate_good", "seg_high_good", "validation", "HIGH_TRACTION", "yaw_rate_nis", expected],
                    ["candidate_good", "seg_low_bad", "validation", "LOW_TRACTION", "forward_accel_nis", bad],
                    ["candidate_low_nis", "seg_high_bad", "validation", "HIGH_TRACTION", "yaw_rate_nis", bad],
                    ["candidate_low_nis", "seg_low_good", "validation", "LOW_TRACTION", "forward_accel_nis", expected],
                ],
            )
            config = self.make_config()
            config["scoring"]["stage_weights"] = {"HIGH_TRACTION": 10.0, "LOW_TRACTION": 1.0}
            config["scoring"]["log_fields"]["yaw_rate_nis"]["weight"] = 5.0
            config["scoring"]["log_fields"]["forward_accel_nis"]["weight"] = 0.1
            records, _segments, trials = self.load_records(
                root,
                config,
                '{"artifacts": [{"path": "nis.csv"}]}\n',
            )
            candidates = testbed.load_candidates(config)

            _itemized, _trial_scores, rankings = testbed.evaluate_records(config, candidates, trials, records)
            score_by_candidate = {score.candidate_id: score for score in rankings}

            expected_weighted_score = ((1.0 * 10.0 * 5.0) + (3.0 * 1.0 * 0.1)) / (
                (10.0 * 5.0) + (1.0 * 0.1)
            )
            self.assertAlmostEqual(
                score_by_candidate["candidate_good"].selection_score,
                expected_weighted_score,
            )
            self.assertLess(
                score_by_candidate["candidate_good"].selection_score,
                score_by_candidate["candidate_low_nis"].selection_score,
            )

    def test_rejected_finite_rows_remain_in_main_score(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            expected = math.sqrt(3.0)
            self.write_csv(
                root / "nis.csv",
                ["candidate_id", "segment_id", "split", "stage", "log_field", "nis", "accepted", "rejected"],
                [
                    ["candidate_good", "seg", "validation", "ACTIVE", "forward_accel_nis", expected, "true", "false"],
                    ["candidate_good", "seg", "validation", "ACTIVE", "forward_accel_nis", 100.0, "false", "true"],
                    ["candidate_low_nis", "other", "validation", "ACTIVE", "forward_accel_nis", expected * 3.0, "true", "false"],
                ],
            )
            config = self.make_config()
            config["scoring"]["rejected_rate_penalty_weight"] = 1.0
            records, _segments, trials = self.load_records(
                root,
                config,
                '{"artifacts": [{"path": "nis.csv"}]}\n',
            )
            candidates = testbed.load_candidates(config)

            itemized, _trial_scores, rankings = testbed.evaluate_records(config, candidates, trials, records)
            row = next(item for item in itemized if item.candidate_id == "candidate_good" and item.split == "validation")
            score_by_candidate = {score.candidate_id: score for score in rankings}

            self.assertAlmostEqual(row.rms_nis, math.sqrt((expected * expected + 100.0 * 100.0) / 2.0))
            self.assertAlmostEqual(row.accepted_only_rms_nis, expected)
            self.assertEqual(row.finite_count, 2)
            self.assertEqual(row.accepted_count, 1)
            self.assertEqual(row.rejected_count, 1)
            self.assertAlmostEqual(row.rejected_rate_penalty, 0.5)
            self.assertGreater(
                score_by_candidate["candidate_good"].selection_score,
                score_by_candidate["candidate_low_nis"].selection_score,
            )

    def test_yaw_rejected_flags_are_ignored(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            self.write_csv(
                root / "nis.csv",
                ["candidate_id", "segment_id", "split", "stage", "log_field", "nis", "accepted", "rejected"],
                [
                    ["candidate_good", "seg", "validation", "ACTIVE", "yaw_rate_nis", 100.0, "false", "true"],
                ],
            )
            records, _segments, trials = self.load_records(
                root,
                self.make_config(),
                '{"artifacts": [{"path": "nis.csv"}]}\n',
            )
            candidates = testbed.load_candidates(self.make_config())

            itemized, _trial_scores, _rankings = testbed.evaluate_records(
                self.make_config(),
                candidates,
                trials,
                records,
            )
            row = next(item for item in itemized if item.candidate_id == "candidate_good" and item.split == "validation")

            self.assertEqual(len(records), 1)
            self.assertTrue(records[0].accepted)
            self.assertEqual(row.accepted_count, 1)
            self.assertEqual(row.rejected_count, 0)

    def test_encoder_nis_rows_are_excluded_from_production_scoring(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            self.write_csv(
                root / "nis.csv",
                ["candidate_id", "segment_id", "split", "stage", "log_field", "nis", "accepted", "rejected"],
                [
                    ["candidate_good", "seg", "validation", "ACTIVE", "left_encoder_wheel_rate_nis", 100.0, "false", "true"],
                ],
            )
            records, _segments, trials = self.load_records(
                root,
                self.make_config(),
                '{"artifacts": [{"path": "nis.csv"}]}\n',
            )
            candidates = testbed.load_candidates(self.make_config())

            itemized, trial_scores, rankings = testbed.evaluate_records(
                self.make_config(),
                candidates,
                trials,
                records,
            )

            self.assertEqual(len(records), 1)
            self.assertFalse(records[0].accepted)
            self.assertEqual(itemized, [])
            self.assertTrue(all(score.sample_count == 0 for score in trial_scores))
            self.assertTrue(all(math.isinf(score.selection_score) for score in rankings))

    def test_launch_rows_are_bucketed_by_per_row_command(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            expected = math.sqrt(3.0)
            self.write_csv(
                root / "nis.csv",
                [
                    "candidate_id",
                    "segment_id",
                    "split",
                    "stage",
                    "log_field",
                    "nis",
                    "left_drive_command",
                    "right_drive_command",
                ],
                [
                    ["candidate_good", "launch_seg", "validation", "SEC_20_LAUNCH", "yaw_rate_nis", expected, 0.24, 0.24],
                    ["candidate_good", "launch_seg", "validation", "SEC_20_LAUNCH", "yaw_rate_nis", expected, 0.36, 0.36],
                ],
            )
            config = self.make_config()
            records, _segments, trials = self.load_records(
                root,
                config,
                """{
  "segments": [
    {
      "segment_id": "launch_seg",
      "split": "validation",
      "stage": "SEC_20_LAUNCH",
      "observed_command": {
        "observed_command_pair_mode": "0.24,0.24"
      }
    }
  ],
  "artifacts": [{"path": "nis.csv"}]
}
""",
            )
            candidates = testbed.load_candidates(config)

            itemized, _trial_scores, _rankings = testbed.evaluate_records(config, candidates, trials, records)
            signatures = {
                item.launch_command_signature
                for item in itemized
                if item.candidate_id == "candidate_good" and item.split == "validation"
            }

            self.assertIn("pair=0.24,0.24;linear=0.24;yaw=0", signatures)
            self.assertIn("pair=0.36,0.36;linear=0.36;yaw=0", signatures)
            self.assertEqual(len(signatures), 2)

    def test_yaw_maneuver_sections_remain_active_even_when_calibration_mentions_bias(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            expected = math.sqrt(3.0)
            self.write_csv(
                root / "nis.csv",
                ["candidate_id", "segment_id", "split", "stage", "log_field", "nis"],
                [
                    ["candidate_good", "yaw_seg", "validation", "YAW_MANEUVER_BIAS_CHECK", "yaw_rate_nis", expected],
                    ["candidate_low_nis", "stationary_seg", "validation", "STATIONARY_BIAS_CHECK", "yaw_rate_nis", expected * 10.0],
                ],
            )
            config = self.make_config()
            records, _segments, trials = self.load_records(
                root,
                config,
                '{"artifacts": [{"path": "nis.csv"}]}\n',
            )
            candidates = testbed.load_candidates(config)

            itemized, _trial_scores, rankings = testbed.evaluate_records(config, candidates, trials, records)
            partitions = {
                (item.candidate_id, item.stage): item.score_partition
                for item in itemized
                if item.split == "validation"
            }
            score_by_candidate = {score.candidate_id: score for score in rankings}

            self.assertEqual(partitions[("candidate_good", "YAW_MANEUVER_BIAS_CHECK")], "active_traction")
            self.assertEqual(partitions[("candidate_low_nis", "STATIONARY_BIAS_CHECK")], "stationary_bias_validation")
            self.assertTrue(math.isfinite(score_by_candidate["candidate_good"].selection_score))
            self.assertTrue(math.isinf(score_by_candidate["candidate_low_nis"].selection_score))

    def test_only_terminal_external_force_corruption_excludes_segments(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            self.write_csv(
                root / "nis.csv",
                ["candidate_id", "segment_id", "split", "stage", "log_field", "nis"],
                [
                    ["candidate_good", "valid_yaw", "validation", "SEC_40_YAW", "yaw_rate_nis", 1.0],
                    ["candidate_good", "pickup", "validation", "SEC_40_YAW", "yaw_rate_nis", 2.0],
                ],
            )

            records, segments, _trials = self.load_records(
                root,
                self.make_config(),
                """{
  "segments": [
    {
      "segment_id": "valid_yaw",
      "split": "validation",
      "stage": "SEC_40_YAW",
      "corruption_note": "yaw calibration still valid before service boundary"
    },
    {
      "segment_id": "pickup",
      "split": "validation",
      "stage": "SEC_40_YAW",
      "corrupted": true,
      "corruption_note": "pickup terminal external-force boundary"
    }
  ],
  "artifacts": [{"path": "nis.csv"}]
}
""",
            )

            self.assertEqual({record.segment_id for record in records}, {"valid_yaw"})
            self.assertFalse(segments["valid_yaw"].corrupted)
            self.assertTrue(segments["pickup"].corrupted)


if __name__ == "__main__":
    unittest.main()
