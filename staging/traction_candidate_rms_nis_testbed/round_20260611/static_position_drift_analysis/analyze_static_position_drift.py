#!/usr/bin/env python3
"""Re-score static stability artifacts against a 5 mm position-drift limit."""

from __future__ import annotations

import csv
import json
import math
from collections import defaultdict
from pathlib import Path
from typing import Any, Iterable


SCRIPT_DIR = Path(__file__).resolve().parent
ROUND_DIR = SCRIPT_DIR.parent

STATIC_STABILITY_DIR = ROUND_DIR / "static_stability_analysis"
STATIC_ENCODER_DIR = ROUND_DIR / "static_encoder_ablation"
STATIC_SIMPLEX_DIR = ROUND_DIR / "static_simplex_ukf_analysis"

SELECTED_MANIFEST = STATIC_STABILITY_DIR / "selected_static_segment_manifest.json"
EKF_FULL_ROWS = STATIC_STABILITY_DIR / "replay_rows" / "residual_diagnostics.csv"
EKF_PREDICTION_ROWS = STATIC_ENCODER_DIR / "prediction_only_rows.csv"
SIMPLEX_ROWS = STATIC_SIMPLEX_DIR / "simplex_ukf_rows.csv"

OUTPUT_METRICS = SCRIPT_DIR / "static_position_drift_metrics.csv"
OUTPUT_SUMMARY = SCRIPT_DIR / "static_position_drift_summary.json"
OUTPUT_REPORT = SCRIPT_DIR / "static_position_drift_report.md"

POSITION_LIMIT_M = 0.005
MODEL_ORDER = (
    "stribeck_fade",
    "slip_envelope",
    "in_shear",
    "shear_rate",
    "skew_shear",
    "baseline",
)
CASE_ORDER = (
    "ekf_full",
    "ekf_prediction_only_zero_encoder",
    "simplex_ukf_full",
    "simplex_ukf_prediction_only_zero_encoder",
)
METRIC_FIELDS = (
    "model",
    "case_id",
    "samples",
    "duration_s",
    "final_global_x_m",
    "final_global_y_m",
    "final_body_right_m",
    "final_body_forward_m",
    "final_global_radial_m",
    "final_body_radial_m",
    "max_global_radial_m",
    "max_body_radial_m",
    "position_limit_m",
    "pass_5mm",
    "source_rows",
    "position_source",
)


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def finite_float(value: str | None, default: float = 0.0) -> float:
    if value is None or value == "":
        return default
    parsed = float(value)
    return parsed if math.isfinite(parsed) else default


def format_number(value: float | int | str | bool) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, int):
        return str(value)
    if isinstance(value, str):
        return value
    if not math.isfinite(value):
        return str(value)
    return f"{value:.12g}"


def read_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def grouped_rows(rows: Iterable[dict[str, str]], keys: tuple[str, ...]) -> dict[tuple[str, ...], list[dict[str, str]]]:
    groups: dict[tuple[str, ...], list[dict[str, str]]] = defaultdict(list)
    for row in rows:
        groups[tuple(row[key] for key in keys)].append(row)
    return groups


def sort_by_time(rows: list[dict[str, str]]) -> list[dict[str, str]]:
    return sorted(rows, key=lambda row: (int(row.get("source_row_index", "0") or 0), int(row.get("master_time_us", "0") or 0)))


def metrics_from_position_rows(
    model: str,
    case_id: str,
    rows: list[dict[str, str]],
    duration_s: float,
    source_rows: Path,
) -> dict[str, Any]:
    ordered = sort_by_time(rows)
    if not ordered:
        raise RuntimeError(f"No rows for {model}:{case_id}")
    start_x = finite_float(ordered[0].get("px_m"))
    start_y = finite_float(ordered[0].get("py_m"))
    max_global = 0.0
    body_right = body_forward = 0.0
    max_body = 0.0
    for row in ordered:
        dx = finite_float(row.get("px_m")) - start_x
        dy = finite_float(row.get("py_m")) - start_y
        max_global = max(max_global, math.hypot(dx, dy))
        dt_s = max(0.0, min(finite_float(row.get("dt_s")), 0.050))
        body_right += finite_float(row.get("vr_mps")) * dt_s
        body_forward += finite_float(row.get("vf_mps")) * dt_s
        max_body = max(max_body, math.hypot(body_right, body_forward))
    final_x = finite_float(ordered[-1].get("px_m")) - start_x
    final_y = finite_float(ordered[-1].get("py_m")) - start_y
    return {
        "model": model,
        "case_id": case_id,
        "samples": len(ordered),
        "duration_s": duration_s,
        "final_global_x_m": final_x,
        "final_global_y_m": final_y,
        "final_body_right_m": body_right,
        "final_body_forward_m": body_forward,
        "final_global_radial_m": math.hypot(final_x, final_y),
        "final_body_radial_m": math.hypot(body_right, body_forward),
        "max_global_radial_m": max_global,
        "max_body_radial_m": max_body,
        "position_limit_m": POSITION_LIMIT_M,
        "pass_5mm": max_global <= POSITION_LIMIT_M and max_body <= POSITION_LIMIT_M,
        "source_rows": str(source_rows),
        "position_source": "artifact_px_py",
    }


def metrics_from_state_rows(
    model: str,
    case_id: str,
    rows: list[dict[str, str]],
    duration_s: float,
    source_rows: Path,
) -> dict[str, Any]:
    ordered = sort_by_time(rows)
    x = y = body_right = body_forward = 0.0
    max_global = max_body = 0.0
    for row in ordered:
        dt_s = max(0.0, min(finite_float(row.get("dt_s")), 0.050))
        vf = finite_float(row.get("vf_mps"))
        vr = finite_float(row.get("vr_mps"))
        heading = finite_float(row.get("heading_rad"))
        x += (vf * math.sin(heading) + vr * math.cos(heading)) * dt_s
        y += (vf * math.cos(heading) - vr * math.sin(heading)) * dt_s
        body_right += vr * dt_s
        body_forward += vf * dt_s
        max_global = max(max_global, math.hypot(x, y))
        max_body = max(max_body, math.hypot(body_right, body_forward))
    return {
        "model": model,
        "case_id": case_id,
        "samples": len(ordered),
        "duration_s": duration_s,
        "final_global_x_m": x,
        "final_global_y_m": y,
        "final_body_right_m": body_right,
        "final_body_forward_m": body_forward,
        "final_global_radial_m": math.hypot(x, y),
        "final_body_radial_m": math.hypot(body_right, body_forward),
        "max_global_radial_m": max_global,
        "max_body_radial_m": max_body,
        "position_limit_m": POSITION_LIMIT_M,
        "pass_5mm": max_global <= POSITION_LIMIT_M and max_body <= POSITION_LIMIT_M,
        "source_rows": str(source_rows),
        "position_source": "integrated_vf_vr_heading",
    }


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=METRIC_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: format_number(row[field]) for field in METRIC_FIELDS})


def write_report(rows: list[dict[str, Any]], segment: dict[str, Any]) -> None:
    lines = [
        "# Static Position Drift Re-Evaluation",
        "",
        f"- Segment: `{segment.get('segment_id')}`",
        f"- Duration: `{segment.get('segment_duration_s')}` s",
        "- Criterion: pass only if static position drift stays within `0.005 m`.",
        "- EKF full uses existing `px_m`/`py_m` replay diagnostics. EKF prediction-only and simplex rows did not contain position, so this script integrates `vf/vr/heading/dt` from the row artifacts.",
        "- No production code or model tuning is changed. No launch/open-floor assessment is run. No logged UKF states or encoder NIS are used.",
        "",
        "| Model | Case | Final global x/y (mm) | Final body right/forward (mm) | Max global radial (mm) | Max body radial (mm) | Pass 5 mm |",
        "| --- | --- | ---: | ---: | ---: | ---: | --- |",
    ]
    for row in sorted(rows, key=lambda item: (MODEL_ORDER.index(item["model"]), CASE_ORDER.index(item["case_id"]))):
        final_global = f"{row['final_global_x_m'] * 1000.0:.3f}/{row['final_global_y_m'] * 1000.0:.3f}"
        if math.isfinite(float(row["final_body_right_m"])):
            final_body = f"{row['final_body_right_m'] * 1000.0:.3f}/{row['final_body_forward_m'] * 1000.0:.3f}"
            max_body = f"{row['max_body_radial_m'] * 1000.0:.3f}"
        else:
            final_body = "n/a"
            max_body = "n/a"
        lines.append(
            f"| `{row['model']}` | `{row['case_id']}` | {final_global} | {final_body} | "
            f"{row['max_global_radial_m'] * 1000.0:.3f} | {max_body} | `{str(row['pass_5mm']).lower()}` |"
        )
    OUTPUT_REPORT.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    SCRIPT_DIR.mkdir(parents=True, exist_ok=True)
    segment = load_json(SELECTED_MANIFEST)["segments"][0]
    duration_s = float(segment.get("segment_duration_s", 0.0) or 0.0)

    metrics: list[dict[str, Any]] = []

    ekf_full_groups = grouped_rows(read_rows(EKF_FULL_ROWS), ("candidate_id",))
    for model in MODEL_ORDER:
        metrics.append(metrics_from_position_rows(model, "ekf_full", ekf_full_groups[(model,)], duration_s, EKF_FULL_ROWS))

    ekf_prediction_groups = grouped_rows(read_rows(EKF_PREDICTION_ROWS), ("candidate_id",))
    for model in MODEL_ORDER:
        metrics.append(
            metrics_from_state_rows(
                model,
                "ekf_prediction_only_zero_encoder",
                ekf_prediction_groups[(model,)],
                duration_s,
                EKF_PREDICTION_ROWS,
            )
        )

    simplex_groups = grouped_rows(read_rows(SIMPLEX_ROWS), ("candidate_id", "case_id"))
    for model in MODEL_ORDER:
        metrics.append(
            metrics_from_state_rows(
                model,
                "simplex_ukf_full",
                simplex_groups[(model, "full_static_replay")],
                duration_s,
                SIMPLEX_ROWS,
            )
        )
        metrics.append(
            metrics_from_state_rows(
                model,
                "simplex_ukf_prediction_only_zero_encoder",
                simplex_groups[(model, "prediction_only_zero_encoder")],
                duration_s,
                SIMPLEX_ROWS,
            )
        )

    metrics = sorted(metrics, key=lambda item: (MODEL_ORDER.index(item["model"]), CASE_ORDER.index(item["case_id"])))
    write_csv(OUTPUT_METRICS, metrics)
    write_report(metrics, segment)
    summary = {
        "schema_version": 1,
        "segment": segment,
        "position_limit_m": POSITION_LIMIT_M,
        "input_artifacts": {
            "ekf_full_rows": str(EKF_FULL_ROWS),
            "ekf_prediction_only_rows": str(EKF_PREDICTION_ROWS),
            "simplex_ukf_rows": str(SIMPLEX_ROWS),
        },
        "outputs": {
            "metrics_csv": str(OUTPUT_METRICS),
            "report_md": str(OUTPUT_REPORT),
        },
        "notes": [
            "No production code or model tuning changed.",
            "No launch/open-floor assessment rerun.",
            "No logged UKF states or encoder NIS consumed.",
            "Measurement-update artifacts use the fixed covariance config referenced by their original summaries.",
        ],
        "metrics": {f"{row['model']}:{row['case_id']}": row for row in metrics},
    }
    OUTPUT_SUMMARY.write_text(json.dumps(summary, indent=2, sort_keys=True, default=format_number) + "\n", encoding="utf-8")
    print(f"Wrote {OUTPUT_METRICS}")
    print(f"Wrote {OUTPUT_SUMMARY}")
    print(f"Wrote {OUTPUT_REPORT}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
