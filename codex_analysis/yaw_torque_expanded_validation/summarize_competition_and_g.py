#!/usr/bin/env python3
from __future__ import annotations

import csv
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
OUT_DIR = Path(__file__).resolve().parent


G_CANDIDATES = [
    ("2026-04-20_08-38-39", "TestResults/mmlog_decode_2026-04-20_08-38-39/open_floor_main.csv"),
    ("2026-04-21_00-16-10", "TestResults/mmlog_decode_2026-04-21_00-16-10/open_floor_main.csv"),
    ("2026-04-20_04-54-09", "TestResults/mmlog_decode_2026-04-20_04-54-09/open_floor_main.csv"),
    ("2026-04-21_05-32-06", "TestResults/mmlog_decode_2026-04-21_05-32-06/open_floor_main.csv"),
]


def load_run_summary() -> dict[str, dict[str, str]]:
    with (OUT_DIR / "run_summary.csv").open(newline="", encoding="utf-8") as f:
        return {row["run_id"]: row for row in csv.DictReader(f)}


def write_explorer_g_comparison() -> None:
    summary = load_run_summary()
    rows: list[dict[str, str]] = []
    for run_id, rel_path in G_CANDIDATES:
        moving = 0
        saturation = 0
        cmd_angular_zero = 0
        max_vf = 0.0
        max_raw_gyro = 0.0
        with (REPO_ROOT / rel_path).open(newline="", encoding="utf-8", errors="replace") as f:
            for row in csv.DictReader(f):
                try:
                    vf = 0.5 * (float(row["left_encoder_velocity_mps"]) + float(row["right_encoder_velocity_mps"]))
                    gyro = float(row["gyro_raw_radps"])
                    cmd_angular = float(row.get("cmd_angular_radps") or 0.0)
                    flags = int(float(row.get("saturation_flags") or 0))
                except (KeyError, ValueError):
                    continue
                max_vf = max(max_vf, abs(vf))
                max_raw_gyro = max(max_raw_gyro, abs(gyro))
                if abs(vf) > 0.02 and abs(gyro) > 0.2:
                    moving += 1
                    if flags != 0:
                        saturation += 1
                    if abs(cmd_angular) < 1.0e-9:
                        cmd_angular_zero += 1
        worker = summary[run_id]
        rows.append(
            {
                "run_id": run_id,
                "g_style_moving_yaw_rows": str(moving),
                "g_style_saturation_rows": str(saturation),
                "g_style_saturation_fraction": f"{(saturation / moving if moving else 0.0):.6f}",
                "g_style_cmd_angular_zero_rows": str(cmd_angular_zero),
                "g_style_cmd_angular_zero_fraction": f"{(cmd_angular_zero / moving if moving else 0.0):.6f}",
                "g_style_max_abs_vf_mps": f"{max_vf:.6f}",
                "g_style_max_abs_raw_gyro_radps": f"{max_raw_gyro:.6f}",
                "worker_i_extracted_samples": worker["extracted_samples"],
                "worker_i_nonzero_forward_yaw_rows": worker["nonzero_forward_yaw_sample_count"],
                "worker_i_status": worker["status"],
            }
        )
    with (OUT_DIR / "explorer_g_comparison.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def write_competition_contribution() -> None:
    with (OUT_DIR / "nonzero_vf_torque_bins.csv").open(newline="", encoding="utf-8") as f:
        bins = list(csv.DictReader(f))
    by_bin = {(row["forward_velocity_bin_mps"], row["yaw_rate_bin_radps"]): row for row in bins}
    run_map: dict[tuple[str, str], list[dict[str, str]]] = {}
    with (OUT_DIR / "bin_run_consistency.csv").open(newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            key = (row["forward_velocity_bin_mps"], row["yaw_rate_bin_radps"])
            run_map.setdefault(key, []).append(row)

    rows: list[dict[str, str]] = []
    for key, row in by_bin.items():
        runs = run_map.get(key, [])
        competition_count = sum(int(item["count"]) for item in runs if item["kind"] == "competition")
        decoded_count = sum(int(item["count"]) for item in runs if item["kind"] == "decoded_open_floor")
        total_count = int(row["count"])
        competition_run_count = sum(1 for item in runs if item["kind"] == "competition")
        decoded_run_count = sum(1 for item in runs if item["kind"] == "decoded_open_floor")
        rows.append(
            {
                "forward_velocity_bin_mps": key[0],
                "yaw_rate_bin_radps": key[1],
                "total_count": str(total_count),
                "competition_count_ge10_runs": str(competition_count),
                "decoded_count_ge10_runs": str(decoded_count),
                "competition_fraction_approx": f"{(competition_count / total_count if total_count else 0.0):.6f}",
                "competition_run_count_ge10": str(competition_run_count),
                "decoded_run_count_ge10": str(decoded_run_count),
                "competition_only": str(competition_count > 0 and decoded_count == 0),
                "competition_enabled_bin": str(decoded_count < 50 <= total_count and competition_count > 0),
                "median_opposing_nm": row["median_opposing_nm"],
                "run_median_spread_opposing_nm": row["run_median_spread_opposing_nm"],
                "consistency": row["consistency"],
            }
        )
    with (OUT_DIR / "competition_bin_contribution.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(
            sorted(
                rows,
                key=lambda row: (-int(row["competition_count_ge10_runs"]), -int(row["total_count"])),
            )
        )


def weighted_rmse_for_rows(rows: list[dict[str, str]]) -> tuple[int, float, float]:
    samples = 0
    current_sum = 0.0
    corrected_sum = 0.0
    for row in rows:
        count = int(row["samples"])
        samples += count
        current_sum += count * (float(row["current_rmse_radps"]) ** 2)
        corrected_sum += count * (float(row["surface_corrected_rmse_radps"]) ** 2)
    if samples == 0:
        return 0, 0.0, 0.0
    return samples, (current_sum / samples) ** 0.5, (corrected_sum / samples) ** 0.5


def refresh_report_sections() -> None:
    report_path = OUT_DIR / "expanded_yaw_torque_validation_report.md"
    if not report_path.is_file():
        return
    with (OUT_DIR / "explorer_g_comparison.csv").open(newline="", encoding="utf-8") as f:
        g_rows = list(csv.DictReader(f))
    with (OUT_DIR / "competition_bin_contribution.csv").open(newline="", encoding="utf-8") as f:
        contribution_rows = list(csv.DictReader(f))
    with (OUT_DIR / "rmse_leave_one_run_out.csv").open(newline="", encoding="utf-8") as f:
        rmse_rows = list(csv.DictReader(f))
    competition_rmse = [row for row in rmse_rows if row["kind"] == "competition"]
    diag_rmse = [row for row in rmse_rows if row["holdout_run_id"].startswith("diag")]
    aux_rmse = [row for row in rmse_rows if row["holdout_run_id"].startswith("aux")]
    comp_samples, comp_current, comp_corrected = weighted_rmse_for_rows(competition_rmse)
    diag_samples, diag_current, diag_corrected = weighted_rmse_for_rows(diag_rmse)
    aux_samples, aux_current, aux_corrected = weighted_rmse_for_rows(aux_rmse)

    bins_total = len(contribution_rows)
    bins_with_comp = sum(int(row["competition_count_ge10_runs"]) > 0 for row in contribution_rows)
    comp_major = sum(float(row["competition_fraction_approx"]) >= 0.25 for row in contribution_rows)
    comp_enabled = sum(row["competition_enabled_bin"] == "True" for row in contribution_rows)
    comp_only = sum(row["competition_only"] == "True" for row in contribution_rows)
    supported_with_comp = sum(
        int(row["total_count"]) >= 250
        and (int(row["competition_run_count_ge10"]) + int(row["decoded_run_count_ge10"])) >= 2
        and int(row["competition_count_ge10_runs"]) > 0
        and float(row["run_median_spread_opposing_nm"] or 0.0) <= 0.12
        for row in contribution_rows
    )
    supported_total = sum(
        int(row["total_count"]) >= 250
        and (int(row["competition_run_count_ge10"]) + int(row["decoded_run_count_ge10"])) >= 2
        and float(row["run_median_spread_opposing_nm"] or 0.0) <= 0.12
        for row in contribution_rows
    )

    lines = [
        "## Explorer G Candidate Comparison",
        "",
        "Explorer G's moving-yaw counts use a looser row predicate (`|Vf| > 0.02`, `|gyro| > 0.2`). Worker I's extraction is stricter: adjacent same-phase pairs, `|Vf| >= 0.05`, `|yaw| >= 0.25`, active yaw/command evidence, finite yaw acceleration, nonzero rounded bins, and decoded-log saturation flags skipped.",
        "",
        "| Run | G-style moving yaw rows | Saturation fraction | Cmd angular zero fraction | Worker I extracted samples | Note |",
        "| --- | ---: | ---: | ---: | ---: | --- |",
    ]
    notes = {
        "2026-04-20_08-38-39": "Still a major contributor, but saturation removes a large fraction.",
        "2026-04-21_00-16-10": "Best decoded moving-yaw run: broad range and low saturation.",
        "2026-04-20_04-54-09": "Useful, but every G-style row has zero `cmd_angular_radps`; extraction relies on wheel-command delta and sensor yaw.",
        "2026-04-21_05-32-06": "Useful lower-speed coverage with low saturation.",
    }
    for row in g_rows:
        lines.append(
            f"| `{row['run_id']}` | {row['g_style_moving_yaw_rows']} | {row['g_style_saturation_fraction']} | {row['g_style_cmd_angular_zero_fraction']} | {row['worker_i_extracted_samples']} | {notes.get(row['run_id'], '')} |"
        )
    lines.extend(
        [
            "",
            "The comparison table is in `explorer_g_comparison.csv`.",
            "",
            "## Competition Contribution",
            "",
            "Competition logs do help populate nonzero-Vf bins, especially real maze bins around `Vf=0.10..0.30 m/s`, `|yaw|=0.50..4.50 rad/s`, and a few higher-yaw maneuver bins.",
            "",
            f"They contributed {comp_samples:,} extracted samples. Of {bins_total} reported bins, {bins_with_comp} had competition contribution, {comp_major} were at least approximately 25% competition by per-run counted samples, {comp_enabled} bins crossed the report threshold because competition data was present, and {comp_only} reported bins were competition-only. Of the {supported_total} bins passing the stricter exploratory support rule used below, {supported_with_comp} had competition contribution.",
            "",
            f"The competition split is mixed for prediction. `diag` holdouts improved from {diag_current:.9f} to {diag_corrected:.9f} rad/s over {diag_samples:,} samples, while `aux` holdouts worsened from {aux_current:.9f} to {aux_corrected:.9f} rad/s over {aux_samples:,} samples. The combined competition holdout result is nearly flat: {comp_current:.9f} to {comp_corrected:.9f} rad/s.",
            "",
            "That makes `diag000`, `diag001`, and especially `diag003` useful coverage validators, but the `aux` logs are too path/procedure dependent to trust as torque-surface fitting authority without tighter phase labeling and repeat structure.",
            "",
            "Competition contribution by bin is in `competition_bin_contribution.csv`.",
            "",
        ]
    )
    inserted = "\n".join(lines)
    report = report_path.read_text(encoding="utf-8")
    start = report.find("## Explorer G Candidate Comparison")
    end = report.find("## Highest-Coverage Nonzero-Vf Bins")
    if end == -1:
        return
    if start == -1 or start > end:
        replacement = inserted + "\n"
        report = report[:end] + replacement + report[end:]
    else:
        report = report[:start] + inserted + "\n" + report[end:]
    report_path.write_text(report, encoding="utf-8")


def main() -> int:
    write_explorer_g_comparison()
    write_competition_contribution()
    refresh_report_sections()
    print(OUT_DIR / "explorer_g_comparison.csv")
    print(OUT_DIR / "competition_bin_contribution.csv")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
