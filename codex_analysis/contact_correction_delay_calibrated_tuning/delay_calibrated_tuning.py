#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
from dataclasses import dataclass
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
OUT_DIR = Path(__file__).resolve().parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from codex_analysis.contact_correction_log_eval import evaluate_contact_correction as replay  # noqa: E402
from codex_analysis.contact_correction_tuning import tune_contact_correction_alignment as align  # noqa: E402
from codex_analysis.contact_continuum_yaw_identification.features import (  # noqa: E402
    extract_contact_continuum_features as prior,
)


@dataclass(frozen=True)
class YawLaunchSample:
    response_lag_samples: int
    launch_set: str
    amplitude: float
    direction: str
    old_error_radps: float
    unit_patch_error_radps: float


def pct_delta(new_value: float, old_value: float) -> float:
    return ((new_value / old_value) - 1.0) * 100.0 if old_value > 0.0 else 0.0


def rmse(values: list[float]) -> float:
    return math.sqrt(statistics.fmean(value * value for value in values)) if values else 0.0


def write_csv(path: Path, rows: list[dict[str, str]], fieldnames: list[str] | None = None) -> None:
    if fieldnames is None:
        fieldnames = list(rows[0].keys()) if rows else ["dataset"]
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def rmse_for_gain(stats: align.GroupStats, gain: float) -> float:
    return stats.new_rmse(gain)


def group_row(
    lag: int,
    label: str,
    stats: align.GroupStats,
    gain_label: str,
    gain: float,
) -> dict[str, str]:
    old = stats.old_rmse()
    new = rmse_for_gain(stats, gain)
    return {
        "response_lag_samples": str(lag),
        "dataset": label,
        "gain_label": gain_label,
        "gain_ns_per_m": f"{gain:.9f}",
        "samples": str(stats.count),
        "old_rmse_radps": f"{old:.9f}",
        "candidate_rmse_radps": f"{new:.9f}",
        "delta_rmse_radps": f"{new - old:.9f}",
        "relative_delta_pct": f"{pct_delta(new, old):.6f}",
    }


def build_aggregate_rows(
    groups: dict[tuple[int, str], align.GroupStats],
    lags: list[int],
    gains: list[tuple[str, float]],
) -> list[dict[str, str]]:
    labels = [
        "fit_authoritative_open_floor",
        "fit_downweighted_open_floor",
        "validation_only_open_floor",
        "open_floor_only",
        "competition_stress",
        "all_included",
    ]
    rows: list[dict[str, str]] = []
    for lag in lags:
        for label in labels:
            stats = groups.get((lag, label))
            if stats is None:
                continue
            for gain_label, gain in gains:
                rows.append(group_row(lag, label, stats, gain_label, gain))
    return rows


def build_motion_rows(
    groups: dict[tuple[int, str], align.GroupStats],
    lags: list[int],
    gains: list[tuple[str, float]],
) -> list[dict[str, str]]:
    rows: list[dict[str, str]] = []
    for lag in lags:
        for family in ["open_floor", "all"]:
            for motion_class in ["in_place_yaw", "moving_yaw", "mostly_forward", "low_motion_commanded"]:
                key = (lag, f"motion:{family}:{motion_class}")
                stats = groups.get(key)
                if stats is None:
                    continue
                for gain_label, gain in gains:
                    row = group_row(lag, key[1], stats, gain_label, gain)
                    row["family"] = family
                    row["motion_class"] = motion_class
                    rows.append(row)
    return rows


def same_window(rows: list[prior.NormalizedRow], start: int, end: int) -> bool:
    if end >= len(rows):
        return False
    key = prior.row_key(rows[start], "open_floor")
    for index in range(start, end):
        if not prior.valid_adjacent(rows[index], rows[index + 1], "open_floor"):
            return False
    return all(prior.row_key(rows[index], "open_floor") == key for index in range(start + 1, end + 1))


def yaw_launch_set(row: prior.NormalizedRow) -> str | None:
    if row.phase_id != "20":
        return None
    amplitude = max(abs(row.left_command), abs(row.right_command))
    if 0.625 <= amplitude <= 0.725:
        return "yaw_launch_sustained_0p65_0p70"
    if 0.475 <= amplitude <= 0.575:
        return "yaw_launch_twitch_only_0p50_0p55"
    if 0.575 < amplitude < 0.625:
        return "yaw_launch_intermediate_0p60"
    return "yaw_launch_other"


def collect_yaw_launch_samples(lags: list[int], params: replay.Params) -> list[YawLaunchSample]:
    path = REPO_ROOT / "TestResults" / "mmlog_decode_2026-05-04_20-35-47" / "open_floor_main.csv"
    candidate = prior.LogCandidate(
        run_id="2026-05-04_20-35-47",
        family="open_floor",
        schema="decoded_open_floor_main",
        path=path,
    )
    _, rows, _, _ = prior.read_normalized_rows(candidate, params)
    if not rows:
        return []
    bias, _ = prior.estimate_bias(rows)
    cutoff_index, _, _, _ = prior.tail_cut_index(rows, bias)
    rows = rows[:cutoff_index]
    denom = replay.yaw_denominator_kg_m2(params)
    base_gain = params.contact_yaw_patch_force_gain_ns_per_m
    samples: list[YawLaunchSample] = []
    for index, command_row in enumerate(rows):
        launch_set = yaw_launch_set(command_row)
        if launch_set is None:
            continue
        command_yaw_rate = command_row.gyro_raw_radps - bias
        command_forward_velocity = 0.5 * (command_row.left_velocity_mps + command_row.right_velocity_mps)
        if not prior.active_contact_sample(command_row, command_yaw_rate, command_forward_velocity):
            continue
        for lag in lags:
            target_end_index = index + lag + 1
            if not same_window(rows, index, target_end_index):
                continue
            target_start = rows[index + lag]
            target_end = rows[target_end_index]
            start_yaw_rate = target_start.gyro_raw_radps - bias
            end_yaw_rate = target_end.gyro_raw_radps - bias
            dt_s = target_end.dt_us * 1.0e-6
            measured_yaw_accel = (end_yaw_rate - start_yaw_rate) / dt_s
            if not math.isfinite(measured_yaw_accel) or abs(measured_yaw_accel) > 4000.0:
                continue
            old_result = replay.model_result(
                command_row,
                command_forward_velocity,
                command_yaw_rate,
                params,
                include_contact_correction=False,
            )
            current_result = replay.model_result(
                command_row,
                command_forward_velocity,
                command_yaw_rate,
                params,
                include_contact_correction=True,
            )
            observed_yaw_moment_nm = denom * measured_yaw_accel
            old_residual_nm = observed_yaw_moment_nm - old_result.model_yaw_moment_nm
            if not math.isfinite(old_residual_nm) or abs(old_residual_nm) > 2.0:
                continue
            old_error = start_yaw_rate + ((old_result.model_yaw_moment_nm / denom) * dt_s) - end_yaw_rate
            current_error = start_yaw_rate + ((current_result.model_yaw_moment_nm / denom) * dt_s) - end_yaw_rate
            unit_patch_error = ((current_error - old_error) / base_gain) if base_gain != 0.0 else 0.0
            direction = "CW" if command_row.left_command > command_row.right_command else "CCW"
            samples.append(
                YawLaunchSample(
                    response_lag_samples=lag,
                    launch_set=launch_set,
                    amplitude=max(abs(command_row.left_command), abs(command_row.right_command)),
                    direction=direction,
                    old_error_radps=old_error,
                    unit_patch_error_radps=unit_patch_error,
                )
            )
    return samples


def build_yaw_launch_rows(
    samples: list[YawLaunchSample],
    lags: list[int],
    gains: list[tuple[str, float]],
) -> list[dict[str, str]]:
    rows: list[dict[str, str]] = []
    set_names = sorted({sample.launch_set for sample in samples})
    for lag in lags:
        for set_name in set_names:
            subset = [
                sample
                for sample in samples
                if sample.response_lag_samples == lag and sample.launch_set == set_name
            ]
            if not subset:
                continue
            old = rmse([sample.old_error_radps for sample in subset])
            for gain_label, gain in gains:
                candidate = rmse(
                    [
                        sample.old_error_radps + (gain * sample.unit_patch_error_radps)
                        for sample in subset
                    ]
                )
                rows.append(
                    {
                        "response_lag_samples": str(lag),
                        "launch_set": set_name,
                        "gain_label": gain_label,
                        "gain_ns_per_m": f"{gain:.9f}",
                        "samples": str(len(subset)),
                        "old_rmse_radps": f"{old:.9f}",
                        "candidate_rmse_radps": f"{candidate:.9f}",
                        "delta_rmse_radps": f"{candidate - old:.9f}",
                        "relative_delta_pct": f"{pct_delta(candidate, old):.6f}",
                    }
                )
    return rows


def select_primary_gain(groups: dict[tuple[int, str], align.GroupStats], primary_lag: int) -> float:
    fit = groups[(primary_lag, "fit_authoritative_open_floor")]
    return fit.optimal_gain()


def write_report(
    path: Path,
    primary_lag: int,
    secondary_lag: int,
    current_gain: float,
    selected_gain: float,
    aggregate_rows: list[dict[str, str]],
    motion_rows: list[dict[str, str]],
    yaw_rows: list[dict[str, str]],
) -> None:
    def table(rows: list[dict[str, str]], columns: list[str]) -> list[str]:
        lines = [
            "| " + " | ".join(columns) + " |",
            "| " + " | ".join("---" for _ in columns) + " |",
        ]
        for row in rows:
            lines.append("| " + " | ".join(row[column] for column in columns) + " |")
        return lines

    report_rows = [
        row
        for row in aggregate_rows
        if row["response_lag_samples"] in {str(primary_lag), str(secondary_lag)}
        and row["dataset"] in {"fit_authoritative_open_floor", "open_floor_only", "validation_only_open_floor", "competition_stress"}
    ]
    motion_report_rows = [
        row
        for row in motion_rows
        if row["response_lag_samples"] == str(primary_lag)
        and row["family"] == "open_floor"
        and row["motion_class"] in {"in_place_yaw", "moving_yaw", "mostly_forward"}
    ]
    yaw_report_rows = [
        row
        for row in yaw_rows
        if row["response_lag_samples"] in {str(primary_lag), str(secondary_lag)}
        and row["launch_set"] in {"yaw_launch_sustained_0p65_0p70", "yaw_launch_twitch_only_0p50_0p55"}
    ]

    lines = [
        "# Delay-Calibrated Contact Correction Tuning",
        "",
        f"Primary alignment: +{primary_lag} samples.",
        f"Secondary derivative/onset check: +{secondary_lag} samples.",
        f"Current production gain: {current_gain:.9f} N*s/m.",
        f"Selected +{primary_lag} fit-authoritative gain: {selected_gain:.9f} N*s/m.",
        "",
        "Targets use raw gyro minus independent bias, encoders, drive commands, and timestamps. UKF targets are not used.",
        "Fit authority is fit-authoritative open-floor only. Validation-only open-floor and competition/aux data are reported as stress/off-distribution checks.",
        "",
        "## Aggregate",
        "",
    ]
    lines.extend(
        table(
            report_rows,
            [
                "response_lag_samples",
                "dataset",
                "gain_label",
                "samples",
                "old_rmse_radps",
                "candidate_rmse_radps",
                "relative_delta_pct",
            ],
        )
    )
    lines.extend(["", "## Open-Floor Motion (+4)", ""])
    lines.extend(
        table(
            motion_report_rows,
            [
                "motion_class",
                "gain_label",
                "samples",
                "old_rmse_radps",
                "candidate_rmse_radps",
                "relative_delta_pct",
            ],
        )
    )
    lines.extend(["", "## Yaw Launch", ""])
    lines.extend(
        table(
            yaw_report_rows,
            [
                "response_lag_samples",
                "launch_set",
                "gain_label",
                "samples",
                "old_rmse_radps",
                "candidate_rmse_radps",
                "relative_delta_pct",
            ],
        )
    )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Delay-calibrated PlantModel contact correction tuning.")
    parser.add_argument("--out-dir", type=Path, default=OUT_DIR)
    parser.add_argument("--response-lags", type=int, nargs="+", default=[0, 2, 4, 5])
    parser.add_argument("--primary-lag", type=int, default=4)
    parser.add_argument("--secondary-lag", type=int, default=5)
    parser.add_argument("--selected-gain", type=float)
    parser.add_argument("--include-competition", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)
    params = replay.source_params()
    current_gain = params.contact_yaw_patch_force_gain_ns_per_m
    collect_args = argparse.Namespace(
        out_dir=args.out_dir,
        response_lags=args.response_lags,
        forced_gain=current_gain,
        no_competition=not args.include_competition,
        include_uncertainty=False,
    )
    samples = align.collect_aligned_samples(collect_args, params)
    groups = align.build_group_stats(samples)
    selected_gain = (
        args.selected_gain
        if args.selected_gain is not None
        else select_primary_gain(groups, args.primary_lag)
    )
    gains = [
        ("old_pre_correction", 0.0),
        ("current_production", current_gain),
        ("delay_calibrated_retuned", selected_gain),
    ]
    aggregate_rows = build_aggregate_rows(groups, args.response_lags, gains)
    motion_rows = build_motion_rows(groups, args.response_lags, gains)
    yaw_samples = collect_yaw_launch_samples(args.response_lags, params)
    yaw_rows = build_yaw_launch_rows(yaw_samples, args.response_lags, gains)

    write_csv(args.out_dir / "aggregate_rmse_by_lag_gain.csv", aggregate_rows)
    write_csv(
        args.out_dir / "motion_rmse_by_lag_gain.csv",
        motion_rows,
        [
            "response_lag_samples",
            "family",
            "motion_class",
            "dataset",
            "gain_label",
            "gain_ns_per_m",
            "samples",
            "old_rmse_radps",
            "candidate_rmse_radps",
            "delta_rmse_radps",
            "relative_delta_pct",
        ],
    )
    write_csv(args.out_dir / "yaw_launch_rmse_by_lag_gain.csv", yaw_rows)
    write_report(
        args.out_dir / "delay_calibrated_tuning_report.md",
        args.primary_lag,
        args.secondary_lag,
        current_gain,
        selected_gain,
        aggregate_rows,
        motion_rows,
        yaw_rows,
    )
    (args.out_dir / "selected_gain.txt").write_text(f"{selected_gain:.9f}\n", encoding="utf-8")
    command = (
        "python codex_analysis\\contact_correction_delay_calibrated_tuning\\delay_calibrated_tuning.py "
        f"--out-dir {args.out_dir} --response-lags {' '.join(str(lag) for lag in args.response_lags)} "
        f"--primary-lag {args.primary_lag} --secondary-lag {args.secondary_lag}"
    )
    if args.selected_gain is not None:
        command += f" --selected-gain {args.selected_gain:.9f}"
    if args.include_competition:
        command += " --include-competition"
    (args.out_dir / "commands_run.txt").write_text(command + "\n", encoding="utf-8")
    print(f"current_gain={current_gain:.9f}")
    print(f"selected_gain={selected_gain:.9f}")
    print(f"aligned_samples={len(samples)}")
    print(f"yaw_launch_samples={len(yaw_samples)}")
    print(f"out_dir={args.out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
