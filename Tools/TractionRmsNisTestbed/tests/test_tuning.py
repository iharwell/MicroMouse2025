from __future__ import annotations

import csv
import math
import tempfile
import unittest

from pathlib import Path
import sys


TESTBED_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TESTBED_ROOT))

from traction_rms_nis_testbed import tuning
from traction_rms_nis_testbed.estimator_core import CovarianceConfig

REPO_ROOT = Path(__file__).resolve().parents[3]
STAGING = REPO_ROOT / "staging" / "traction_candidate_rms_nis_testbed"


class OverflowPlant:
    def propagate(self, _state, _sample, _dt_s):
        raise OverflowError("unit overflow")


class TractionRmsNisTuningTest(unittest.TestCase):
    def make_candidate(self) -> tuning.CandidateSpec:
        return tuning.CandidateSpec(
            candidate_id="candidate_1_algebraic_envelope",
            label="Candidate",
            model="algebraic_envelope",
            parameters={
                "peak_friction_coefficient_at_80pct_fan": 1.68,
                "longitudinal_slip_gain_n_per_mps": 18.0,
            },
            search={
                "peak_friction_coefficient_at_80pct_fan": {
                    "min": 1.2,
                    "max": 2.4,
                    "scale": "linear",
                },
                "longitudinal_slip_gain_n_per_mps": {
                    "min": 4.0,
                    "max": 64.0,
                    "scale": "log",
                },
            },
        )

    def test_latin_hypercube_trials_stay_within_bounds(self) -> None:
        trials = tuning.latin_hypercube_trials(self.make_candidate(), 12, "unit-test")

        self.assertEqual(len(trials), 12)
        for trial in trials:
            self.assertGreaterEqual(trial.parameters["peak_friction_coefficient_at_80pct_fan"], 1.2)
            self.assertLessEqual(trial.parameters["peak_friction_coefficient_at_80pct_fan"], 2.4)
            self.assertGreaterEqual(trial.parameters["longitudinal_slip_gain_n_per_mps"], 4.0)
            self.assertLessEqual(trial.parameters["longitudinal_slip_gain_n_per_mps"], 64.0)

    def test_coverage_split_keeps_whole_segments_and_covers_large_buckets(self) -> None:
        segments = [
            tuning.SegmentMeta(
                segment_id=f"seg_{index}",
                stage="SEC_20_LAUNCH",
                family="launch",
                log_path="log.csv",
                log_id="log",
                row_count=10,
                corrupted=False,
                parameter_fields={
                    "parameter": "observed_drive_command_magnitude",
                    "test_value_kind": "observed_drive_command_magnitude",
                    "test_value": 0.25,
                },
                observed_command={
                    "observed_command_pair_mode": [0.25, 0.25],
                    "observed_command_magnitude_median": 0.25,
                },
                coverage_key="launch-bin",
            )
            for index in range(6)
        ]

        assigned = tuning.assign_coverage_splits(
            segments,
            {"seed": "unit-test", "train": 0.6, "validation": 0.2, "held_out": 0.2},
        )

        split_by_segment = {segment.segment_id: segment.split for segment in assigned}
        self.assertEqual(len(split_by_segment), len(segments))
        self.assertEqual(set(split_by_segment.values()), {"train", "validation", "held_out"})

    def test_source_log_heldout_split_keeps_entire_logs_held_out_first(self) -> None:
        segments = [
            tuning.SegmentMeta(
                segment_id=f"{log_id}_{index}",
                stage="SEC_20_LAUNCH",
                family="launch",
                log_path=f"logs/{log_id}.csv",
                log_id=log_id,
                row_count=10,
                parameter_fields={"parameter": "drive", "test_value_kind": "bin", "test_value": index},
                observed_command={"observed_command_magnitude_median": index},
                coverage_key=f"launch-{index}",
            )
            for log_id in ("log_a", "log_b", "log_c", "log_d")
            for index in range(4)
        ]

        assigned = tuning.assign_coverage_splits(
            segments,
            {
                "strategy": "source_log_heldout_then_whole_segment",
                "seed": "unit-test-source-log",
                "train": 0.5,
                "validation": 0.25,
                "held_out": 0.25,
            },
        )

        splits_by_log: dict[str, set[str]] = {}
        for segment in assigned:
            splits_by_log.setdefault(segment.log_id, set()).add(segment.split)
        heldout_logs = [log_id for log_id, splits in splits_by_log.items() if splits == {"held_out"}]

        self.assertGreaterEqual(len(heldout_logs), 1)
        for log_id, splits in splits_by_log.items():
            if "held_out" in splits:
                self.assertEqual({log_id: splits}[log_id], {"held_out"})

    def test_active_metric_scope_uses_only_manifest_active_rows(self) -> None:
        meta = tuning.SegmentMeta(
            segment_id="seg",
            stage="yaw_launch",
            family="launch",
            log_path="log.csv",
            log_id="log",
            row_count=5,
            active_start_row_index=11,
            active_end_row_index=13,
        )
        samples = [
            tuning.ReplaySample(
                source_path=Path("log.csv"),
                source_row_index=row_index,
                master_time_us=0,
                dt_s=0.001,
                left_command=0.0,
                right_command=0.0,
                left_wheel_rate_radps=0.0,
                right_wheel_rate_radps=0.0,
                yaw_rate_radps=0.0,
                accel_forward_mps2=0.0,
                accel_right_mps2=0.0,
                accel_valid=True,
                gyro_valid=True,
                fan_duty_cycle=0.8,
                segment_id="seg",
                stage="yaw_launch",
                split="train",
                run_id="log",
                corrupted=False,
            )
            for row_index in range(10, 15)
        ]

        active = tuning.samples_for_metric_scope(samples, meta, "active")

        self.assertEqual([sample.source_row_index for sample in active], [11, 12, 13])

    def test_yaw_calibration_is_primary_even_when_segment_ends_at_boundary(self) -> None:
        yaw_segment = tuning.SegmentMeta(
            segment_id="yaw_boundary",
            stage="SEC_40_YAW",
            family="in_place_turn",
            log_path="log.csv",
            log_id="log",
            row_count=20,
            active_start_row_index=4,
            active_end_row_index=12,
            corrupted=True,
            coverage_key="stage=SEC_40_YAW|family=in_place_turn",
            split="train",
        )

        self.assertEqual(tuning.active_metric_name(yaw_segment.stage, yaw_segment.family), "yaw_calibration")
        self.assertTrue(
            tuning.segment_in_metric_scope(
                yaw_segment,
                "active",
                {"yaw_calibration": 1.0},
            )
        )

        selected = tuning.select_tuning_subset(
            [yaw_segment],
            "train",
            1,
            "unit-test",
            0.5,
            include_corrupted=True,
        )

        self.assertEqual([segment.segment_id for segment in selected], ["yaw_boundary"])
        self.assertEqual(tuning.active_metric_name("SEC_20_LAUNCH", "recovery"), "")
        self.assertEqual(tuning.active_metric_name("SEC_50_SMOOTH", "static_hold"), "")
        self.assertEqual(tuning.active_metric_name("yaw_maneuver", ""), "yaw_calibration")
        self.assertEqual(tuning.DEFAULT_ACTIVE_METRIC_WEIGHTS["yaw_calibration"], 1.0)

    def test_only_terminal_external_force_notes_mark_corrupted(self) -> None:
        self.assertFalse(
            tuning.is_corrupted_segment(
                {
                    "stage": "SEC_40_YAW",
                    "corruption_note": "yaw calibration note before service boundary",
                }
            )
        )
        self.assertFalse(
            tuning.is_corrupted_segment(
                {
                    "stage": "SEC_40_YAW",
                    "end_reason": "corruption_boundary",
                    "end_criterion": "predetermined terminal fault boundary at 164938393 us; kept last row not after fault",
                    "corruption_note": "selector_removed: Primary diagnostic selector jumper removed",
                }
            )
        )
        self.assertTrue(
            tuning.is_corrupted_segment(
                {
                    "stage": "SEC_40_YAW",
                    "corrupted": True,
                    "corruption_note": "pickup terminal external-force boundary",
                }
            )
        )

    def test_covariance_search_fields_are_rejected(self) -> None:
        with self.assertRaises(tuning.TuningError):
            tuning.validate_no_covariance_tuning(
                "candidate",
                {"peak_friction_coefficient_at_80pct_fan": 1.7},
                {"measurement_noise_scale": {"min": 0.5, "max": 2.0}},
            )

    def test_explicitly_disabled_tuning_candidates_are_not_generated(self) -> None:
        config = {
            "candidates": [
                {
                    "id": "baseline/current_holdover",
                    "label": "Baseline",
                    "model": "current_holdover_approximation",
                    "enabled": True,
                    "parameters": {},
                    "search": {},
                },
                {
                    "id": "candidate_1_algebraic_envelope",
                    "label": "Candidate 1",
                    "model": "algebraic_envelope",
                    "enabled": True,
                    "parameters": {
                        "peak_friction_coefficient_at_80pct_fan": 1.68,
                    },
                    "search": {
                        "peak_friction_coefficient_at_80pct_fan": {
                            "min": 1.2,
                            "max": 2.4,
                            "scale": "linear",
                        },
                    },
                },
                {"id": "candidate_2_stribeck", "enabled": False},
                {"id": "candidate_3_load_sensitive", "enabled": False},
            ]
        }
        candidate_config = {
            "candidates": [
                {
                    "id": "baseline/current_holdover",
                    "label": "Baseline",
                    "model": "current_holdover_approximation",
                    "parameters": {},
                },
                {
                    "id": "candidate_1_algebraic_envelope",
                    "label": "Candidate 1",
                    "model": "algebraic_envelope",
                    "parameters": {
                        "peak_friction_coefficient_at_80pct_fan": 1.68,
                    },
                },
            ]
        }

        candidates = tuning.load_candidate_specs(config, candidate_config)
        trials = tuning.generate_trials(candidates, 2, "candidate-subset", True)
        selected = tuning.best_trials_by_candidate(
            trials,
            [
                tuning.SelectionScore(
                    phase="validation",
                    candidate_id="candidate_1_algebraic_envelope",
                    trial_id="candidate_1_algebraic_envelope:nominal",
                    selection_score=1.0,
                    sample_count=1,
                )
            ],
            [candidate.candidate_id for candidate in candidates],
        )

        self.assertEqual(
            ["baseline/current_holdover", "candidate_1_algebraic_envelope"],
            [candidate.candidate_id for candidate in candidates],
        )
        self.assertEqual(
            {"baseline/current_holdover", "candidate_1_algebraic_envelope"},
            {trial.candidate_id for trial in trials},
        )
        self.assertEqual(["candidate_1_algebraic_envelope"], [trial.candidate_id for trial in selected])

    def test_round_20260611_candidate_only_configs_parse(self) -> None:
        round_dir = STAGING / "round_20260611"
        candidate_config = tuning.load_json(STAGING / "candidates.json")
        expected_ids = ("skew_shear", "shear_rate", "in_shear")

        for candidate_id in expected_ids:
            with self.subTest(candidate_id=candidate_id):
                config = tuning.load_json(round_dir / f"{candidate_id}.json")
                candidates = tuning.load_candidate_specs(config, candidate_config)

                self.assertEqual([candidate_id], [candidate.candidate_id for candidate in candidates])
                self.assertNotIn("baseline/current_holdover", [candidate.candidate_id for candidate in candidates])
                self.assertEqual(
                    {
                        "yaw_rate_residual_tail",
                        "forward_accel_residual_tail",
                        "right_accel_residual_tail",
                    },
                    set(config["scoring"]["log_fields"]),
                )
                self.assertTrue(
                    tuning.resolve_path(config["manifest_path"], round_dir).exists()
                )
                self.assertTrue(
                    tuning.resolve_path(config["assessment_manifest_path"], round_dir).exists()
                )
                self.assertTrue(
                    tuning.resolve_existing_config_path(config["bias_source_manifest"], round_dir).exists()
                )
                trials = tuning.generate_trials(candidates, 2, f"unit-{candidate_id}", True)
                self.assertEqual({candidate_id}, {trial.candidate_id for trial in trials})

    def test_bucket_stats_rms_uses_all_finite_rows(self) -> None:
        stats = tuning.BucketStats()

        stats.add(100.0, accepted=False)
        stats.add(4.0, accepted=True)

        self.assertEqual(stats.count, 2)
        self.assertEqual(stats.finite_count, 2)
        self.assertEqual(stats.accepted_count, 1)
        self.assertEqual(stats.rejected_count, 1)
        self.assertAlmostEqual(stats.rms_nis, math.sqrt((100.0 * 100.0 + 4.0 * 4.0) / 2.0))
        self.assertEqual(stats.accepted_only_rms_nis, 4.0)

    def test_evaluation_records_rejected_updates_after_candidate_overflow(self) -> None:
        meta = tuning.SegmentMeta(
            segment_id="overflow",
            stage="SEC_20_LAUNCH",
            family="launch",
            log_path="logs/open_floor_main.csv",
            log_id="overflow_log",
            row_count=1,
            split="validation",
            coverage_key="stage=SEC_20_LAUNCH|family=launch",
        )
        trial = tuning.TrialSpec(
            candidate_id="candidate_1_algebraic_envelope",
            trial_id="overflow",
            model="algebraic_envelope",
            parameters={},
            is_nominal=False,
        )
        result = tuning.EvaluationResult(phase="unit", segment_count=1)

        tuning.evaluate_segment_samples(
            result,
            meta,
            [self.make_sample("overflow", 0, left_command=0.2, right_command=0.2)],
            [trial],
            {trial.key: OverflowPlant()},
            CovarianceConfig(),
        )

        self.assertEqual(result.sample_count, 1)
        self.assertEqual(
            {
                key[4]
                for key in result.report
            },
            {
                "yaw_rate_residual_tail",
                "forward_accel_residual_tail",
                "right_accel_residual_tail",
            },
        )
        for stats in result.report.values():
            self.assertEqual(stats.count, 1)
            self.assertEqual(stats.finite_count, 0)
            self.assertEqual(stats.accepted_count, 0)
            self.assertEqual(stats.rejected_count, 1)

    def test_launch_coverage_key_uses_per_row_command_bucket(self) -> None:
        meta = tuning.SegmentMeta(
            segment_id="launch",
            stage="SEC_20_LAUNCH",
            family="launch",
            log_path="log.csv",
            log_id="log",
            row_count=2,
            coverage_key="stage=SEC_20_LAUNCH|family=launch",
        )
        first = self.make_sample("launch", 0, left_command=0.12, right_command=0.16)
        second = self.make_sample("launch", 1, left_command=0.22, right_command=0.18)

        first_key = tuning.sample_coverage_key(meta, first)
        second_key = tuning.sample_coverage_key(meta, second)

        self.assertIn("command_bucket=pair=0.12,0.16", first_key)
        self.assertIn("linear=0.14", first_key)
        self.assertIn("yaw=0.02", first_key)
        self.assertNotEqual(first_key, second_key)

    def test_evaluation_sample_source_reuses_cached_segment_samples(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            log_path = root / "open_floor_main.csv"
            manifest_path = root / "manifest.json"
            self.write_fixture_log(log_path)
            manifest_path.write_text(
                """
{
  "segments": [
    {
      "segment_id": "seg_a",
      "log_path": "%s",
      "segment_start_row_index": 0,
      "segment_end_row_index": 4,
      "active_start_row_index": 1,
      "active_end_row_index": 3,
      "stage": "SEC_20_LAUNCH",
      "family": "launch"
    },
    {
      "segment_id": "seg_b",
      "log_path": "%s",
      "segment_start_row_index": 5,
      "segment_end_row_index": 9,
      "active_start_row_index": 6,
      "active_end_row_index": 8,
      "stage": "SEC_20_LAUNCH",
      "family": "launch"
    }
  ]
}
"""
                % (str(log_path).replace("\\", "\\\\"), str(log_path).replace("\\", "\\\\"))
            )
            vehicle, _covariance = tuning.load_covariance(STAGING / "covariance_conservative.json")
            sample_source = tuning.EvaluationSampleSource(manifest_path, vehicle)
            segments = [
                tuning.SegmentMeta(
                    segment_id="seg_a",
                    stage="SEC_20_LAUNCH",
                    family="launch",
                    log_path=str(log_path),
                    log_id="log",
                    row_count=5,
                    active_start_row_index=1,
                    active_end_row_index=3,
                    split="train",
                    coverage_key="stage=SEC_20_LAUNCH|family=launch",
                ),
                tuning.SegmentMeta(
                    segment_id="seg_b",
                    stage="SEC_20_LAUNCH",
                    family="launch",
                    log_path=str(log_path),
                    log_id="log",
                    row_count=5,
                    active_start_row_index=6,
                    active_end_row_index=8,
                    split="train",
                    coverage_key="stage=SEC_20_LAUNCH|family=launch",
                ),
            ]
            trials = [
                tuning.TrialSpec(
                    candidate_id="candidate_1_algebraic_envelope",
                    trial_id="candidate_1_algebraic_envelope:nominal",
                    model="algebraic_envelope",
                    parameters={},
                    is_nominal=True,
                )
            ]

            first = tuning.evaluate_trials(
                phase="unit",
                manifest_path=manifest_path,
                segment_metas=segments,
                trials=trials,
                vehicle=vehicle,
                covariance_path=STAGING / "covariance_conservative.json",
                max_rows_per_segment=2,
                metric_scope="active",
                metric_weights={"launch_active_pulse": 1.0},
                sample_source=sample_source,
            )
            second = tuning.evaluate_trials(
                phase="unit",
                manifest_path=manifest_path,
                segment_metas=segments,
                trials=trials,
                vehicle=vehicle,
                covariance_path=STAGING / "covariance_conservative.json",
                max_rows_per_segment=2,
                metric_scope="active",
                metric_weights={"launch_active_pulse": 1.0},
                sample_source=sample_source,
            )

            stats = sample_source.stats()
            self.assertEqual(first.sample_count, 4)
            self.assertEqual(second.sample_count, 4)
            self.assertEqual(stats["source_log_scan_batches"], 1)
            self.assertEqual(stats["cached_segments"], 2)
            self.assertGreaterEqual(stats["cache_hits"], 2)

    def test_evaluation_sample_source_applies_static_accel_bias_to_active_rows(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            log_path = root / "open_floor_main.csv"
            manifest_path = root / "manifest.json"
            self.write_csv_log(
                log_path,
                [
                    {
                        "master_time_us": "0",
                        "dt_us": "1000",
                        "left_drive_command": "0",
                        "right_drive_command": "0",
                        "left_encoder_omega_radps": "0",
                        "right_encoder_omega_radps": "0",
                        "gyro_radps": "0",
                        "accel_body_y_mps2": "0.30",
                        "accel_body_x_mps2": "-0.20",
                        "accel_bias_valid": "false",
                        "fan_duty_cycle": "0.8",
                    },
                    {
                        "master_time_us": "1000",
                        "dt_us": "1000",
                        "left_drive_command": "0.2",
                        "right_drive_command": "0.2",
                        "left_encoder_omega_radps": "3.0",
                        "right_encoder_omega_radps": "3.0",
                        "gyro_radps": "0.01",
                        "accel_body_y_mps2": "1.30",
                        "accel_body_x_mps2": "0.80",
                        "accel_bias_valid": "false",
                        "fan_duty_cycle": "0.8",
                    },
                ],
            )
            static_meta = tuning.SegmentMeta(
                segment_id="static",
                stage="SEC_10_STATIC",
                family="static_hold",
                log_path=str(log_path),
                log_id="log",
                row_count=1,
                start_row_index=0,
                end_row_index=0,
                split="train",
            )
            active_meta = tuning.SegmentMeta(
                segment_id="active",
                stage="SEC_20_LAUNCH",
                family="launch",
                log_path=str(log_path),
                log_id="log",
                row_count=1,
                start_row_index=1,
                end_row_index=1,
                split="train",
            )
            vehicle, _covariance = tuning.load_covariance(STAGING / "covariance_conservative.json")
            sample_source = tuning.EvaluationSampleSource(
                manifest_path,
                vehicle,
                [static_meta, active_meta],
            )

            loaded = list(sample_source.iter_segment_samples([active_meta], 0, "all"))

            self.assertEqual(len(loaded), 1)
            samples = loaded[0][1]
            self.assertEqual(len(samples), 1)
            self.assertTrue(samples[0].accel_valid)
            self.assertAlmostEqual(samples[0].accel_forward_mps2, 1.0)
            self.assertAlmostEqual(samples[0].accel_right_mps2, 1.0)

    def make_sample(
        self,
        segment_id: str,
        row_index: int,
        left_command: float,
        right_command: float,
    ) -> tuning.ReplaySample:
        return tuning.ReplaySample(
            source_path=Path("log.csv"),
            source_row_index=row_index,
            master_time_us=row_index * 1000,
            dt_s=0.001,
            left_command=left_command,
            right_command=right_command,
            left_wheel_rate_radps=0.0,
            right_wheel_rate_radps=0.0,
            yaw_rate_radps=0.0,
            accel_forward_mps2=0.0,
            accel_right_mps2=0.0,
            accel_valid=True,
            gyro_valid=True,
            fan_duty_cycle=0.8,
            segment_id=segment_id,
            stage="SEC_20_LAUNCH",
            split="train",
            run_id="log",
            corrupted=False,
        )

    def write_fixture_log(self, path: Path) -> None:
        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(
                handle,
                fieldnames=[
                    "master_time_us",
                    "dt_us",
                    "left_drive_command",
                    "right_drive_command",
                    "left_encoder_omega_radps",
                    "right_encoder_omega_radps",
                    "gyro_radps",
                    "accel_body_y_mps2",
                    "accel_body_x_mps2",
                    "accel_bias_valid",
                    "fan_duty_cycle",
                ],
            )
            writer.writeheader()
            for index in range(10):
                writer.writerow(
                    {
                        "master_time_us": str(index * 1000),
                        "dt_us": "1000",
                        "left_drive_command": "0.1",
                        "right_drive_command": "0.12",
                        "left_encoder_omega_radps": "1.0",
                        "right_encoder_omega_radps": "1.1",
                        "gyro_radps": "0.01",
                        "accel_body_y_mps2": "0.2",
                        "accel_body_x_mps2": "0.1",
                        "accel_bias_valid": "1",
                        "fan_duty_cycle": "0.8",
                    }
                )

    def write_csv_log(self, path: Path, rows: list[dict[str, str]]) -> None:
        fieldnames = [
            "master_time_us",
            "dt_us",
            "left_drive_command",
            "right_drive_command",
            "left_encoder_omega_radps",
            "right_encoder_omega_radps",
            "gyro_radps",
            "accel_body_y_mps2",
            "accel_body_x_mps2",
            "accel_bias_valid",
            "fan_duty_cycle",
        ]
        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)


if __name__ == "__main__":
    unittest.main()
