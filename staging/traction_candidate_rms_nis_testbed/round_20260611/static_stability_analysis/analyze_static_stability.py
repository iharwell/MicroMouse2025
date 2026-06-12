#!/usr/bin/env python3
"""Prepare and summarize a narrow static-only traction stability replay."""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from collections import defaultdict
from pathlib import Path
from typing import Any


SCRIPT_DIR = Path(__file__).resolve().parent
ROUND_DIR = SCRIPT_DIR.parent
TESTBED_DIR = ROUND_DIR.parent
REPO_ROOT = TESTBED_DIR.parents[1]

STATIC_MANIFEST_CANDIDATES = (
    TESTBED_DIR / "representative_corpus_stationary_split_manifest.json",
    TESTBED_DIR / "representative_corpus" / "segment_manifest.json",
    TESTBED_DIR / "representative_corpus_active_split_expanded_manifest.json",
)

OUTPUT_MANIFEST = SCRIPT_DIR / "selected_static_segment_manifest.json"
OUTPUT_CONFIG = SCRIPT_DIR / "combined_static_candidate_config.json"
OUTPUT_REPORT = SCRIPT_DIR / "static_stability_report.md"
OUTPUT_SUMMARY = SCRIPT_DIR / "static_stability_summary.json"
OUTPUT_CSV = SCRIPT_DIR / "static_stability_metrics.csv"


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, payload: dict[str, Any]) -> None:
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def row_count(segment: dict[str, Any]) -> int:
    explicit = segment.get("segment_row_count")
    if explicit is not None:
        return int(explicit)
    start = int(segment.get("segment_start_row_index", 0) or 0)
    end = int(segment.get("segment_end_row_index", -1) or -1)
    return max(0, end - start + 1)


def text_blob(segment: dict[str, Any]) -> str:
    parts = [
        segment.get("stage", ""),
        segment.get("section_name", ""),
        segment.get("family", ""),
        segment.get("phase_name", ""),
        segment.get("start_criterion", ""),
        segment.get("end_criterion", ""),
        segment.get("end_reason", ""),
    ]
    return " ".join(str(part) for part in parts if part is not None).lower()


def observed_command_is_static(segment: dict[str, Any]) -> bool:
    observed = segment.get("observed_command") or {}
    if not isinstance(observed, dict):
        return False
    active_rows = observed.get("active_command_rows")
    if active_rows is not None and int(active_rows) == 0:
        return True
    magnitude = observed.get("observed_command_magnitude_median")
    return magnitude is not None and abs(float(magnitude)) <= 1.0e-12


def is_reliably_static(segment: dict[str, Any]) -> bool:
    blob = text_blob(segment)
    has_static_label = any(
        token in blob
        for token in (
            "static",
            "stationary",
            "bias",
            "idle",
            "zero",
            "static_hold",
            "reliable_stationary",
        )
    )
    reliable = "reliable_stationary" in blob or "bias" in blob or "static" in blob
    return has_static_label and reliable and observed_command_is_static(segment)


def find_longest_static_segment() -> tuple[Path, dict[str, Any]]:
    best: tuple[Path, dict[str, Any]] | None = None
    for manifest_path in STATIC_MANIFEST_CANDIDATES:
        if not manifest_path.exists():
            continue
        manifest = load_json(manifest_path)
        for segment in manifest.get("segments", []):
            if isinstance(segment, dict) and is_reliably_static(segment):
                if best is None or row_count(segment) > row_count(best[1]):
                    best = (manifest_path, dict(segment))
    if best is None:
        raise RuntimeError("No explicit static/stationary/bias segment found.")
    return best


def single_candidate(path: Path, preferred_id: str | None = None) -> dict[str, Any]:
    candidates = [
        dict(item)
        for item in load_json(path).get("candidates", [])
        if isinstance(item, dict) and item.get("enabled", True)
    ]
    if preferred_id is not None:
        candidates = [item for item in candidates if item.get("id") == preferred_id]
    if not candidates:
        raise RuntimeError(f"No enabled candidate found in {path}")
    return candidates[-1]


def candidate_from_tuned(path: Path, source_id: str, alias: str, label: str) -> dict[str, Any]:
    item = single_candidate(path, source_id)
    return {
        "id": alias,
        "label": label,
        "model": item["model"],
        "enabled": True,
        "parameters": item["parameters"],
        "source_candidate_id": item.get("id", source_id),
        "source_config": str(path),
    }


def build_combined_candidate_config() -> dict[str, Any]:
    candidates = [
        candidate_from_tuned(
            ROUND_DIR / "stribeck_fade" / "tuned_parameters.json",
            "candidate_2_stribeck",
            "stribeck_fade",
            "StribeckFade carried-forward tuned",
        ),
        candidate_from_tuned(
            ROUND_DIR / "slip_envelope" / "tuned_parameters.json",
            "candidate_1_algebraic_envelope",
            "slip_envelope",
            "SlipEnvelope carried-forward tuned",
        ),
        {
            **single_candidate(ROUND_DIR / "in_shear" / "in_shear_tuned_only_config.json"),
            "source_config": str(ROUND_DIR / "in_shear" / "in_shear_tuned_only_config.json"),
        },
        {
            **single_candidate(ROUND_DIR / "shear_rate" / "shear_rate_tuned_only.json"),
            "source_config": str(ROUND_DIR / "shear_rate" / "shear_rate_tuned_only.json"),
        },
        {
            **single_candidate(ROUND_DIR / "skew_shear" / "skew_shear_tuned_only_config.json"),
            "source_config": str(ROUND_DIR / "skew_shear" / "skew_shear_tuned_only_config.json"),
        },
        {
            **single_candidate(ROUND_DIR / "baseline" / "baseline_candidate_config.json"),
            "id": "baseline",
            "source_candidate_id": "baseline/current_holdover",
            "source_config": str(ROUND_DIR / "baseline" / "baseline_candidate_config.json"),
        },
    ]
    return {
        "schema_version": 1,
        "description": "Static-only combined candidate config assembled from round_20260611 tuned/carried-forward configs.",
        "covariance_config": str(TESTBED_DIR / "covariance_conservative.json"),
        "candidate_specific_covariance_or_noise": False,
        "candidates": candidates,
    }


def prepare() -> None:
    manifest_path, segment = find_longest_static_segment()
    selected_manifest = {
        "schema_version": 1,
        "source_manifest": str(manifest_path),
        "selection_policy": "longest explicit reliable static/stationary/bias segment with zero logged active command rows",
        "uses_logged_ukf_state": False,
        "segments": [segment],
    }
    write_json(OUTPUT_MANIFEST, selected_manifest)
    write_json(OUTPUT_CONFIG, build_combined_candidate_config())
    print(f"Wrote {OUTPUT_MANIFEST}")
    print(f"Wrote {OUTPUT_CONFIG}")
    print(
        "Selected "
        f"{segment.get('segment_id')} rows={row_count(segment)} "
        f"stage={segment.get('stage')} family={segment.get('family')} "
        f"log={segment.get('log_path')}"
    )


def finite_float(value: str | None) -> float | None:
    if value is None or value == "":
        return None
    try:
        parsed = float(value)
    except ValueError:
        return None
    return parsed if math.isfinite(parsed) else None


def summarize_values(values: list[float]) -> dict[str, float | int | None]:
    if not values:
        return {"count": 0, "mean": None, "median": None, "max_abs": None}
    return {
        "count": len(values),
        "mean": statistics.fmean(values),
        "median": statistics.median(values),
        "max_abs": max(abs(value) for value in values),
    }


def rms(values: list[float]) -> float | None:
    if not values:
        return None
    return math.sqrt(statistics.fmean(value * value for value in values))


def summarize_rows(output_dir: Path) -> None:
    diagnostics_path = output_dir / "residual_diagnostics.csv"
    nis_path = output_dir / "nis_samples.csv"
    if not diagnostics_path.exists() or not nis_path.exists():
        raise RuntimeError(f"Missing row artifacts under {output_dir}")

    diagnostics: dict[str, dict[str, list[float]]] = defaultdict(lambda: defaultdict(list))
    initial_state: dict[str, dict[str, float]] = {}
    final_state: dict[str, dict[str, float]] = {}
    command_stats: dict[str, dict[str, float]] = {}
    state_fields = ("vf_mps", "vr_mps", "yaw_rate_radps", "heading_rad")
    prediction_fields = (
        "predicted_forward_accel_mps2",
        "predicted_right_accel_mps2",
        "predicted_yaw_accel_radps2",
    )
    residual_fields = (
        "yaw_rate_residual_radps",
        "forward_accel_residual_mps2",
        "right_accel_residual_mps2",
    )
    command_fields = ("left_command", "right_command", "left_wheel_rate_radps", "right_wheel_rate_radps")

    with diagnostics_path.open(newline="", encoding="utf-8-sig") as handle:
        for row in csv.DictReader(handle):
            candidate_id = row["candidate_id"]
            state_snapshot = {
                field: finite_float(row.get(field)) or 0.0
                for field in state_fields
            }
            initial_state.setdefault(candidate_id, state_snapshot)
            final_state[candidate_id] = state_snapshot
            for field in (*state_fields, *prediction_fields, *residual_fields, *command_fields):
                value = finite_float(row.get(field))
                if value is not None:
                    diagnostics[candidate_id][field].append(value)

    nis_values: dict[str, dict[str, list[float]]] = defaultdict(lambda: defaultdict(list))
    with nis_path.open(newline="", encoding="utf-8-sig") as handle:
        for row in csv.DictReader(handle):
            parameter = row.get("log_parameter", "")
            if parameter not in ("yaw_rate_nis", "forward_accel_nis", "right_accel_nis"):
                continue
            value = finite_float(row.get("nis"))
            if value is not None:
                nis_values[row["candidate_id"]][parameter].append(value)

    summary: dict[str, Any] = {
        "schema_version": 1,
        "diagnostics_csv": str(diagnostics_path),
        "nis_csv": str(nis_path),
        "selected_static_manifest": str(OUTPUT_MANIFEST),
        "combined_candidate_config": str(OUTPUT_CONFIG),
        "fixed_covariance_config": str(TESTBED_DIR / "covariance_conservative.json"),
        "uses_logged_ukf_state": False,
        "models": {},
    }
    rows: list[dict[str, Any]] = []
    for candidate_id in sorted(diagnostics):
        model_summary: dict[str, Any] = {}
        model_summary["sample_count"] = len(diagnostics[candidate_id]["vf_mps"])
        model_summary["state_final"] = final_state.get(candidate_id, {})
        model_summary["state_final_abs"] = {
            field: abs(final_state.get(candidate_id, {}).get(field, 0.0))
            for field in state_fields
        }
        model_summary["state_max_abs"] = {
            field: summarize_values(diagnostics[candidate_id][field])["max_abs"]
            for field in state_fields
        }
        model_summary["predicted_accel"] = {
            field: summarize_values(diagnostics[candidate_id][field])
            for field in prediction_fields
        }
        model_summary["signed_residual"] = {
            field: summarize_values(diagnostics[candidate_id][field])
            for field in residual_fields
        }
        all_nis = []
        model_summary["rms_nis_by_channel"] = {}
        for parameter in ("yaw_rate_nis", "forward_accel_nis", "right_accel_nis"):
            values = nis_values[candidate_id][parameter]
            all_nis.extend(values)
            model_summary["rms_nis_by_channel"][parameter] = rms(values)
        model_summary["rms_nis_all_production_measurements"] = rms(all_nis)
        model_summary["command_max_abs"] = {
            field: summarize_values(diagnostics[candidate_id][field])["max_abs"]
            for field in command_fields
        }
        summary["models"][candidate_id] = model_summary

        rows.append(
            {
                "candidate_id": candidate_id,
                "samples": model_summary["sample_count"],
                "final_abs_vf_mps": model_summary["state_final_abs"]["vf_mps"],
                "final_abs_vr_mps": model_summary["state_final_abs"]["vr_mps"],
                "final_abs_yaw_rate_radps": model_summary["state_final_abs"]["yaw_rate_radps"],
                "final_abs_heading_rad": model_summary["state_final_abs"]["heading_rad"],
                "max_abs_vf_mps": model_summary["state_max_abs"]["vf_mps"],
                "max_abs_vr_mps": model_summary["state_max_abs"]["vr_mps"],
                "max_abs_yaw_rate_radps": model_summary["state_max_abs"]["yaw_rate_radps"],
                "max_abs_heading_rad": model_summary["state_max_abs"]["heading_rad"],
                "pred_forward_accel_mean_mps2": model_summary["predicted_accel"]["predicted_forward_accel_mps2"]["mean"],
                "pred_forward_accel_median_mps2": model_summary["predicted_accel"]["predicted_forward_accel_mps2"]["median"],
                "pred_forward_accel_max_abs_mps2": model_summary["predicted_accel"]["predicted_forward_accel_mps2"]["max_abs"],
                "pred_right_accel_mean_mps2": model_summary["predicted_accel"]["predicted_right_accel_mps2"]["mean"],
                "pred_right_accel_median_mps2": model_summary["predicted_accel"]["predicted_right_accel_mps2"]["median"],
                "pred_right_accel_max_abs_mps2": model_summary["predicted_accel"]["predicted_right_accel_mps2"]["max_abs"],
                "pred_yaw_accel_mean_radps2": model_summary["predicted_accel"]["predicted_yaw_accel_radps2"]["mean"],
                "pred_yaw_accel_median_radps2": model_summary["predicted_accel"]["predicted_yaw_accel_radps2"]["median"],
                "pred_yaw_accel_max_abs_radps2": model_summary["predicted_accel"]["predicted_yaw_accel_radps2"]["max_abs"],
                "yaw_rate_residual_mean_radps": model_summary["signed_residual"]["yaw_rate_residual_radps"]["mean"],
                "yaw_rate_residual_median_radps": model_summary["signed_residual"]["yaw_rate_residual_radps"]["median"],
                "forward_accel_residual_mean_mps2": model_summary["signed_residual"]["forward_accel_residual_mps2"]["mean"],
                "forward_accel_residual_median_mps2": model_summary["signed_residual"]["forward_accel_residual_mps2"]["median"],
                "right_accel_residual_mean_mps2": model_summary["signed_residual"]["right_accel_residual_mps2"]["mean"],
                "right_accel_residual_median_mps2": model_summary["signed_residual"]["right_accel_residual_mps2"]["median"],
                "rms_nis_all": model_summary["rms_nis_all_production_measurements"],
                "rms_yaw_rate_nis": model_summary["rms_nis_by_channel"]["yaw_rate_nis"],
                "rms_forward_accel_nis": model_summary["rms_nis_by_channel"]["forward_accel_nis"],
                "rms_right_accel_nis": model_summary["rms_nis_by_channel"]["right_accel_nis"],
                "max_abs_left_command": model_summary["command_max_abs"]["left_command"],
                "max_abs_right_command": model_summary["command_max_abs"]["right_command"],
                "max_abs_left_wheel_rate_radps": model_summary["command_max_abs"]["left_wheel_rate_radps"],
                "max_abs_right_wheel_rate_radps": model_summary["command_max_abs"]["right_wheel_rate_radps"],
            }
        )

    write_json(OUTPUT_SUMMARY, summary)
    with OUTPUT_CSV.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    write_report(rows, summary)
    print(f"Wrote {OUTPUT_SUMMARY}")
    print(f"Wrote {OUTPUT_CSV}")
    print(f"Wrote {OUTPUT_REPORT}")


def format_cell(value: Any) -> str:
    if value is None:
        return ""
    if isinstance(value, float):
        return f"{value:.9g}"
    return str(value)


def write_report(rows: list[dict[str, Any]], summary: dict[str, Any]) -> None:
    lines = [
        "# Static Stability Analysis",
        "",
        f"- Selected manifest: `{OUTPUT_MANIFEST}`",
        f"- Candidate config: `{OUTPUT_CONFIG}`",
        f"- Row diagnostics: `{summary['diagnostics_csv']}`",
        f"- NIS rows: `{summary['nis_csv']}`",
        f"- Fixed covariance: `{summary['fixed_covariance_config']}`",
        "- Logged UKF states used: `false`",
        "",
        "| Model | Samples | Final | Max abs | Pred accel mean f/r/yaw | Pred accel max abs f/r/yaw | Residual mean yaw/f/r | RMS NIS all/yaw/f/r | Command max abs L/R wheel L/R |",
        "| --- | ---: | --- | --- | --- | --- | --- | --- | --- |",
    ]
    for row in rows:
        final = (
            f"vf={format_cell(row['final_abs_vf_mps'])}, "
            f"vr={format_cell(row['final_abs_vr_mps'])}, "
            f"yr={format_cell(row['final_abs_yaw_rate_radps'])}, "
            f"yaw={format_cell(row['final_abs_heading_rad'])}"
        )
        max_abs = (
            f"vf={format_cell(row['max_abs_vf_mps'])}, "
            f"vr={format_cell(row['max_abs_vr_mps'])}, "
            f"yr={format_cell(row['max_abs_yaw_rate_radps'])}, "
            f"yaw={format_cell(row['max_abs_heading_rad'])}"
        )
        pred_mean = (
            f"{format_cell(row['pred_forward_accel_mean_mps2'])}/"
            f"{format_cell(row['pred_right_accel_mean_mps2'])}/"
            f"{format_cell(row['pred_yaw_accel_mean_radps2'])}"
        )
        pred_max = (
            f"{format_cell(row['pred_forward_accel_max_abs_mps2'])}/"
            f"{format_cell(row['pred_right_accel_max_abs_mps2'])}/"
            f"{format_cell(row['pred_yaw_accel_max_abs_radps2'])}"
        )
        residual_mean = (
            f"{format_cell(row['yaw_rate_residual_mean_radps'])}/"
            f"{format_cell(row['forward_accel_residual_mean_mps2'])}/"
            f"{format_cell(row['right_accel_residual_mean_mps2'])}"
        )
        rms_nis = (
            f"{format_cell(row['rms_nis_all'])}/"
            f"{format_cell(row['rms_yaw_rate_nis'])}/"
            f"{format_cell(row['rms_forward_accel_nis'])}/"
            f"{format_cell(row['rms_right_accel_nis'])}"
        )
        commands = (
            f"{format_cell(row['max_abs_left_command'])}/"
            f"{format_cell(row['max_abs_right_command'])} "
            f"{format_cell(row['max_abs_left_wheel_rate_radps'])}/"
            f"{format_cell(row['max_abs_right_wheel_rate_radps'])}"
        )
        lines.append(
            f"| `{row['candidate_id']}` | {row['samples']} | {final} | {max_abs} | "
            f"{pred_mean} | {pred_max} | {residual_mean} | {rms_nis} | {commands} |"
        )
    OUTPUT_REPORT.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("prepare")
    summarize_parser = subparsers.add_parser("summarize")
    summarize_parser.add_argument("--replay-output-dir", required=True)
    args = parser.parse_args()
    if args.command == "prepare":
        prepare()
    elif args.command == "summarize":
        summarize_rows(Path(args.replay_output_dir).resolve())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
