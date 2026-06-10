import csv
import math
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import evaluate_traction_candidate_rms_nis as nis_eval


class TractionCandidateRmsNisEvalTest(unittest.TestCase):
    def make_config(self) -> dict:
        return {
            "split": {
                "seed": "unit-test",
                "train": 0.6,
                "validation": 0.2,
                "held_out": 0.2,
            },
            "scoring": {
                "selection_split": "validation",
                "inflation_floor_ratio": 0.75,
                "log_parameters": {
                    "yaw_rate_nis": {"dimension": 1, "weight": 1.0},
                    "forward_accel_nis": {"dimension": 1, "weight": 1.0},
                    "right_accel_nis": {"dimension": 1, "weight": 1.0},
                },
            },
            "candidates": [
                {
                    "id": "baseline_holdover",
                    "label": "Current holdover baseline",
                    "model": "current_plantmodel",
                    "parameters": {},
                    "search": {},
                },
                {
                    "id": "candidate_1_algebraic_envelope",
                    "label": "Candidate 1 algebraic envelope",
                    "model": "algebraic_envelope",
                    "parameters": {"longitudinal_slip_gain_n_per_mps": 18.0},
                    "search": {
                        "longitudinal_slip_gain_n_per_mps": {
                            "min": 4.0,
                            "max": 48.0,
                        }
                    },
                },
            ],
        }

    def write_csv(self, path: Path, header: list[str], rows: list[list[object]]) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(header)
            writer.writerows(rows)

    def load_from_manifest(
        self,
        config: dict,
        manifest_path: Path,
    ) -> tuple[list[nis_eval.NisRecord], dict[str, nis_eval.SegmentInfo]]:
        return nis_eval.load_records(config, manifest_path.parent / "config.json", manifest_path)

    def test_scores_itemized_rms_nis_and_excludes_corrupted_segments(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            nis_csv = root / "nis.csv"
            self.write_csv(
                nis_csv,
                ["candidate_id", "segment_id", "stage", "log_parameter", "nis"],
                [
                    ["baseline_holdover", "seg_train", "SEC_20_LAUNCH", "yaw_rate_nis", 1.0],
                    ["baseline_holdover", "seg_train", "SEC_20_LAUNCH", "yaw_rate_nis", 2.0],
                    ["baseline_holdover", "seg_val", "SEC_40_YAW", "yaw_rate_nis", 4.0],
                    ["baseline_holdover", "seg_bad", "SEC_40_YAW", "yaw_rate_nis", 100.0],
                    ["candidate_1_algebraic_envelope", "seg_train", "SEC_20_LAUNCH", "yaw_rate_nis", 1.0],
                    ["candidate_1_algebraic_envelope", "seg_val", "SEC_40_YAW", "yaw_rate_nis", 2.0],
                    ["candidate_1_algebraic_envelope", "seg_val", "SEC_40_YAW", "forward_accel_nis", 2.0],
                    ["candidate_1_algebraic_envelope", "seg_bad", "SEC_40_YAW", "yaw_rate_nis", 100.0],
                ],
            )
            manifest_path = root / "manifest.json"
            manifest_path.write_text(
                """{
  "segments": [
    {"segment_id": "seg_train", "stage": "SEC_20_LAUNCH", "split": "train", "corrupted": false},
    {"segment_id": "seg_val", "stage": "SEC_40_YAW", "split": "validation", "corrupted": false},
    {"segment_id": "seg_bad", "stage": "SEC_40_YAW", "split": "validation", "corrupted": true}
  ],
  "artifacts": [{"path": "nis.csv"}]
}
""",
                encoding="utf-8",
            )

            config = self.make_config()
            records, segments = self.load_from_manifest(config, manifest_path)
            candidates = nis_eval.load_candidates(config)
            itemized, scores = nis_eval.evaluate_records(config, candidates, records)

            self.assertEqual({record.segment_id for record in records}, {"seg_train", "seg_val"})
            self.assertTrue(segments["seg_bad"].corrupted)
            baseline_val = next(
                item
                for item in itemized
                if item.candidate_id == "baseline_holdover"
                and item.split == "validation"
                and item.log_parameter == "yaw_rate_nis"
            )
            self.assertEqual(baseline_val.count, 1)
            self.assertAlmostEqual(baseline_val.rms_nis, 4.0, delta=1.0e-12)

            score_by_id = {score.candidate_id: score for score in scores}
            self.assertLess(
                score_by_id["candidate_1_algebraic_envelope"].selection_score,
                score_by_id["baseline_holdover"].selection_score,
            )

    def test_hash_split_is_assigned_by_whole_segment(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            nis_csv = root / "nis.csv"
            self.write_csv(
                nis_csv,
                ["candidate_id", "segment_id", "stage", "log_parameter", "nis"],
                [
                    ["baseline_holdover", "same_segment", "SEC_20_LAUNCH", "yaw_rate_nis", 1.0],
                    ["baseline_holdover", "same_segment", "SEC_20_LAUNCH", "forward_accel_nis", 2.0],
                    ["candidate_1_algebraic_envelope", "same_segment", "SEC_20_LAUNCH", "yaw_rate_nis", 3.0],
                ],
            )
            manifest_path = root / "manifest.json"
            manifest_path.write_text('{"artifacts": [{"path": "nis.csv"}]}\n', encoding="utf-8")

            records, segments = self.load_from_manifest(self.make_config(), manifest_path)

            self.assertEqual(len({record.split for record in records}), 1)
            self.assertIn(segments["same_segment"].split, nis_eval.SPLITS)

    def test_rejects_logged_ukf_state_columns(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            nis_csv = root / "nis.csv"
            self.write_csv(
                nis_csv,
                ["candidate_id", "segment_id", "stage", "log_parameter", "nis", "ukf_state_px_m"],
                [["baseline_holdover", "seg", "SEC_20_LAUNCH", "yaw_rate_nis", 1.0, 0.5]],
            )
            manifest_path = root / "manifest.json"
            manifest_path.write_text('{"artifacts": [{"path": "nis.csv"}]}\n', encoding="utf-8")

            with self.assertRaises(nis_eval.EvalConfigError):
                self.load_from_manifest(self.make_config(), manifest_path)

    def test_guarded_score_does_not_reward_under_expected_nis(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            expected = math.sqrt(3.0)
            nis_csv = root / "nis.csv"
            self.write_csv(
                nis_csv,
                ["candidate_id", "segment_id", "stage", "log_parameter", "nis"],
                [
                    ["baseline_holdover", "seg_val", "SEC_40_YAW", "yaw_rate_nis", expected],
                    ["candidate_1_algebraic_envelope", "seg_val", "SEC_40_YAW", "yaw_rate_nis", 0.01],
                ],
            )
            manifest_path = root / "manifest.json"
            manifest_path.write_text(
                """{
  "segments": [{"segment_id": "seg_val", "stage": "SEC_40_YAW", "split": "validation"}],
  "artifacts": [{"path": "nis.csv"}]
}
""",
                encoding="utf-8",
            )

            config = self.make_config()
            records, _segments = self.load_from_manifest(config, manifest_path)
            candidates = nis_eval.load_candidates(config)
            itemized, scores = nis_eval.evaluate_records(config, candidates, records)
            score_by_id = {score.candidate_id: score for score in scores}

            self.assertAlmostEqual(
                score_by_id["candidate_1_algebraic_envelope"].selection_score,
                score_by_id["baseline_holdover"].selection_score,
                delta=1.0e-12,
            )
            low_item = next(
                item
                for item in itemized
                if item.candidate_id == "candidate_1_algebraic_envelope"
            )
            self.assertTrue(low_item.inflation_flag)

    def test_candidate_config_rejects_covariance_tuning_fields(self) -> None:
        config = self.make_config()
        config["candidates"][0]["parameters"] = {"measurement_noise_scale": 2.0}

        with self.assertRaises(nis_eval.EvalConfigError):
            nis_eval.load_candidates(config)


if __name__ == "__main__":
    unittest.main()
