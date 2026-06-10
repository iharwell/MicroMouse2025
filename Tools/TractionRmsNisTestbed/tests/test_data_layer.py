from __future__ import annotations

import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


TESTBED_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TESTBED_ROOT))

from traction_rms_nis_testbed.data_layer import TractionObservableDataLayer


FIXTURE_MANIFEST = TESTBED_ROOT / "tests" / "fixtures" / "segment_manifest.json"


class TractionObservableDataLayerTest(unittest.TestCase):
    def load_fixture_streams(self):
        layer = TractionObservableDataLayer(FIXTURE_MANIFEST)
        return list(layer.iter_segment_streams())

    def test_loads_manifest_rows_as_observable_streams(self) -> None:
        launch, corrupt = self.load_fixture_streams()

        self.assertEqual(launch.definition.segment_id, "fixture_launch")
        self.assertEqual(launch.row_count, 3)
        self.assertEqual(launch.streams["dt_us"], [1000, 1000, 1000])
        self.assertEqual(launch.streams["is_active"], [False, True, False])
        self.assertEqual(
            launch.streams["drive_commands"]["left_drive_command"],
            [0.0, 0.25, 0.0],
        )
        self.assertEqual(
            launch.streams["encoder"]["left_encoder_count"],
            [10, 20, 25],
        )
        self.assertEqual(
            launch.streams["gyro"]["gyro_radps"],
            [0.09, 0.19, 0.14],
        )
        self.assertEqual(
            launch.streams["accel"]["accel_body_x_mps2"],
            [0.1, 1.1, 0.2],
        )
        self.assertEqual(launch.streams["fan"]["fan_duty_cycle"], [0.8, 0.8, 0.8])
        self.assertEqual(
            launch.definition.parameter_fields["test_value_kind"],
            "logged_drive_command_bin",
        )
        self.assertEqual(
            launch.boundaries["stationary_context"]["pre_active_row_range"],
            [0, 0],
        )
        self.assertEqual(
            launch.boundaries["stationary_context"]["post_active_row_range"],
            [2, 2],
        )

        self.assertTrue(corrupt.boundaries["corruption"]["is_corrupted"])
        self.assertEqual(corrupt.row_count, 2)

    def test_ignores_ukf_and_replay_state_columns(self) -> None:
        stream = self.load_fixture_streams()[0].to_json_dict()
        serialized = json.dumps(stream, sort_keys=True)

        self.assertNotIn("ukf_state_px_m", serialized)
        self.assertNotIn("replay_state_px_m", serialized)
        self.assertNotIn("logged_ukf_state", serialized)

    def test_cli_writes_jsonl_and_summary(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            output_dir = Path(temp_dir)
            completed = subprocess.run(
                [
                    sys.executable,
                    str(TESTBED_ROOT / "export_observable_streams.py"),
                    "--manifest",
                    str(FIXTURE_MANIFEST),
                    "--output-dir",
                    str(output_dir),
                    "--limit",
                    "1",
                ],
                cwd=str(TESTBED_ROOT),
                text=True,
                capture_output=True,
                check=False,
            )

            self.assertEqual(completed.returncode, 0, completed.stderr)
            jsonl_path = output_dir / "segment_streams.jsonl"
            summary_path = output_dir / "summary.json"
            self.assertTrue(jsonl_path.exists())
            self.assertTrue(summary_path.exists())
            lines = jsonl_path.read_text(encoding="utf-8").splitlines()
            self.assertEqual(len(lines), 1)
            payload = json.loads(lines[0])
            self.assertEqual(payload["segment_id"], "fixture_launch")
            summary = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(summary["segments_exported"], 1)


if __name__ == "__main__":
    unittest.main()
