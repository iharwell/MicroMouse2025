#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
from dataclasses import dataclass, replace
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
OUT_DIR = Path(__file__).resolve().parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from codex_analysis.contact_correction_log_eval import evaluate_contact_correction as replay  # noqa: E402


@dataclass
class GroupStats:
    label: str
    count: int = 0
    sum_e2: float = 0.0
    sum_ex: float = 0.0
    sum_x2: float = 0.0

    def add(self, old_error_radps: float, unit_patch_error_radps: float) -> None:
        self.count += 1
        self.sum_e2 += old_error_radps * old_error_radps
        self.sum_ex += old_error_radps * unit_patch_error_radps
        self.sum_x2 += unit_patch_error_radps * unit_patch_error_radps

    def old_rmse(self) -> float:
        return math.sqrt(self.sum_e2 / self.count) if self.count else 0.0

    def new_rmse(self, gain: float) -> float:
        if not self.count:
            return 0.0
        sse = self.sum_e2 + (2.0 * gain * self.sum_ex) + (gain * gain * self.sum_x2)
        return math.sqrt(max(0.0, sse / self.count))

    def optimal_gain(self) -> float:
        return (-self.sum_ex / self.sum_x2) if self.sum_x2 > 0.0 else 0.0


def pct_delta(new_value: float, old_value: float) -> float:
    return ((new_value / old_value) - 1.0) * 100.0 if old_value > 0.0 else 0.0


def write_csv(path: Path, rows: list[dict[str, str]], fieldnames: list[str] | None = None) -> None:
    if fieldnames is None:
        fieldnames = list(rows[0].keys()) if rows else ["dataset"]
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def add_to_group(groups: dict[str, GroupStats], label: str, sample: replay.Sample, base_gain: float, denom: float) -> None:
    if base_gain == 0.0:
        unit_patch_error_radps = 0.0
    else:
        unit_patch_error_radps = (sample.patch_delta_yaw_moment_nm / base_gain / denom) * sample.dt_s
    groups.setdefault(label, GroupStats(label)).add(sample.old_error_radps, unit_patch_error_radps)


def build_stats(samples: list[replay.Sample], base_gain: float, denom: float) -> dict[str, GroupStats]:
    groups: dict[str, GroupStats] = {}
    for sample in samples:
        add_to_group(groups, "all_included", sample, base_gain, denom)
        add_to_group(groups, f"motion:all:{sample.motion_class}", sample, base_gain, denom)
        if sample.family == "open_floor":
            add_to_group(groups, "open_floor_only", sample, base_gain, denom)
            add_to_group(groups, f"motion:open_floor:{sample.motion_class}", sample, base_gain, denom)
            if sample.recommendation == "fit_authoritative":
                add_to_group(groups, "fit_authoritative_open_floor", sample, base_gain, denom)
            elif sample.recommendation == "fit_downweighted":
                add_to_group(groups, "fit_downweighted_open_floor", sample, base_gain, denom)
            elif sample.recommendation == "validation_only":
                add_to_group(groups, "validation_only_open_floor", sample, base_gain, denom)
        if sample.family.startswith("competition"):
            add_to_group(groups, "competition_only", sample, base_gain, denom)
            add_to_group(groups, f"motion:competition:{sample.motion_class}", sample, base_gain, denom)
    return groups


def row_from_stats(stats: GroupStats, gain: float) -> dict[str, str]:
    old = stats.old_rmse()
    new = stats.new_rmse(gain)
    return {
        "dataset": stats.label,
        "samples": str(stats.count),
        "old_rmse_radps": f"{old:.9f}",
        "new_rmse_radps": f"{new:.9f}",
        "delta_rmse_radps": f"{new - old:.9f}",
        "relative_delta_pct": f"{pct_delta(new, old):.6f}",
    }


def sweep_rows(groups: dict[str, GroupStats], gains: list[float]) -> list[dict[str, str]]:
    rows: list[dict[str, str]] = []
    for gain in gains:
        all_stats = groups["all_included"]
        open_stats = groups["open_floor_only"]
        comp_stats = groups["competition_only"]
        fit_stats = groups["fit_authoritative_open_floor"]
        val_stats = groups["validation_only_open_floor"]
        in_place = groups["motion:all:in_place_yaw"]
        moving = groups["motion:all:moving_yaw"]
        rows.append(
            {
                "gain_ns_per_m": f"{gain:.9f}",
                "all_old_rmse_radps": f"{all_stats.old_rmse():.9f}",
                "all_new_rmse_radps": f"{all_stats.new_rmse(gain):.9f}",
                "open_floor_new_rmse_radps": f"{open_stats.new_rmse(gain):.9f}",
                "competition_new_rmse_radps": f"{comp_stats.new_rmse(gain):.9f}",
                "fit_new_rmse_radps": f"{fit_stats.new_rmse(gain):.9f}",
                "fit_relative_delta_pct": f"{pct_delta(fit_stats.new_rmse(gain), fit_stats.old_rmse()):.6f}",
                "validation_new_rmse_radps": f"{val_stats.new_rmse(gain):.9f}",
                "validation_relative_delta_pct": f"{pct_delta(val_stats.new_rmse(gain), val_stats.old_rmse()):.6f}",
                "in_place_new_rmse_radps": f"{in_place.new_rmse(gain):.9f}",
                "in_place_relative_delta_pct": f"{pct_delta(in_place.new_rmse(gain), in_place.old_rmse()):.6f}",
                "moving_new_rmse_radps": f"{moving.new_rmse(gain):.9f}",
                "moving_relative_delta_pct": f"{pct_delta(moving.new_rmse(gain), moving.old_rmse()):.6f}",
            }
        )
    return rows


def score(groups: dict[str, GroupStats], gain: float) -> tuple[float, float, float, float, float]:
    return (
        groups["fit_authoritative_open_floor"].new_rmse(gain),
        groups["validation_only_open_floor"].new_rmse(gain),
        groups["motion:all:in_place_yaw"].new_rmse(gain),
        groups["motion:all:moving_yaw"].new_rmse(gain),
        groups["competition_only"].new_rmse(gain),
    )


def material_without_clear_yaw_regression(groups: dict[str, GroupStats], gain: float) -> bool:
    fit = groups["fit_authoritative_open_floor"]
    in_place = groups["motion:all:in_place_yaw"]
    moving = groups["motion:all:moving_yaw"]
    fit_delta = pct_delta(fit.new_rmse(gain), fit.old_rmse())
    in_place_delta = pct_delta(in_place.new_rmse(gain), in_place.old_rmse())
    moving_delta = pct_delta(moving.new_rmse(gain), moving.old_rmse())
    return fit_delta <= -1.0 and in_place_delta <= 0.05 and moving_delta <= 0.05


def exact_aggregate(samples: list[replay.Sample]) -> list[dict[str, str]]:
    specs = [
        ("all_included", lambda sample: True),
        ("open_floor_only", lambda sample: sample.family == "open_floor"),
        ("competition_only", lambda sample: sample.family.startswith("competition")),
        (
            "fit_authoritative_open_floor",
            lambda sample: sample.family == "open_floor" and sample.recommendation == "fit_authoritative",
        ),
        (
            "fit_downweighted_open_floor",
            lambda sample: sample.family == "open_floor" and sample.recommendation == "fit_downweighted",
        ),
        (
            "validation_only_open_floor",
            lambda sample: sample.family == "open_floor" and sample.recommendation == "validation_only",
        ),
    ]
    rows: list[dict[str, str]] = []
    for label, predicate in specs:
        subset = [sample for sample in samples if predicate(sample)]
        if subset:
            rows.append(replay.aggregate_row(label, subset))
    return rows


def exact_motion(samples: list[replay.Sample]) -> list[dict[str, str]]:
    rows: list[dict[str, str]] = []
    families = [
        ("all", lambda sample: True),
        ("open_floor", lambda sample: sample.family == "open_floor"),
        ("competition", lambda sample: sample.family.startswith("competition")),
    ]
    for family, family_predicate in families:
        for motion_class in ["in_place_yaw", "moving_yaw", "mostly_forward", "low_motion_commanded"]:
            subset = [
                sample
                for sample in samples
                if family_predicate(sample) and sample.motion_class == motion_class
            ]
            if subset:
                row = replay.aggregate_row(f"{family}:{motion_class}", subset)
                row["family"] = family
                row["motion_class"] = motion_class
                rows.append(row)
    return rows


def run_tuning(args: argparse.Namespace) -> int:
    args.out_dir.mkdir(parents=True, exist_ok=True)
    base_params = replay.source_params()
    base_gain = base_params.contact_yaw_patch_force_gain_ns_per_m
    denom = replay.yaw_denominator_kg_m2(base_params)

    print(f"collecting_base_samples_gain={base_gain:.9f}", flush=True)
    _, samples = replay.collect_samples(args, base_params)
    groups = build_stats(samples, base_gain, denom)

    all_gain = groups["all_included"].optimal_gain()
    fit_gain = groups["fit_authoritative_open_floor"].optimal_gain()
    validation_gain = groups["validation_only_open_floor"].optimal_gain()
    open_gain = groups["open_floor_only"].optimal_gain()

    seed_gains = {
        0.0,
        base_gain,
        -base_gain,
        all_gain,
        open_gain,
        fit_gain,
        validation_gain,
        -2.0,
        -1.0,
        -0.5,
        -0.25,
        -0.10,
        -0.05,
        -0.02,
        0.02,
        0.05,
        0.10,
        0.25,
        0.50,
        1.0,
        2.0,
    }
    for center in [all_gain, open_gain, fit_gain, validation_gain]:
        step = max(0.0025, min(0.10, abs(center) * 0.20))
        for index in range(-12, 13):
            seed_gains.add(center + (index * step))
    gains = sorted(gain for gain in seed_gains if math.isfinite(gain) and -5.0 <= gain <= 5.0)
    rows = sweep_rows(groups, gains)

    best_gain = min(gains, key=lambda gain: score(groups, gain))
    material_gains = [gain for gain in gains if material_without_clear_yaw_regression(groups, gain)]
    selected_gain = min(material_gains, key=lambda gain: score(groups, gain)) if material_gains else best_gain
    print(f"linear_selected_gain={selected_gain:.9f}", flush=True)

    print("collecting_exact_selected_samples", flush=True)
    selected_params = replace(base_params, contact_yaw_patch_force_gain_ns_per_m=selected_gain)
    selected_runs, selected_samples = replay.collect_samples(args, selected_params)
    aggregate = exact_aggregate(selected_samples)
    motion = exact_motion(selected_samples)

    write_csv(args.out_dir / "gain_sweep_linear.csv", rows)
    write_csv(args.out_dir / "selected_exact_aggregate_rmse.csv", aggregate)
    write_csv(
        args.out_dir / "selected_exact_motion_rmse.csv",
        motion,
        ["family", "motion_class"] + [key for key in motion[0].keys() if key not in {"family", "motion_class"}],
    )
    write_csv(
        args.out_dir / "selected_exact_per_run_rmse.csv",
        [
            {
                "run_id": run.run_id,
                "family": run.family,
                "recommendation": run.recommendation,
                "samples": str(run.samples),
                "old_rmse_radps": f"{run.old_rmse_radps:.9f}",
                "new_rmse_radps": f"{run.new_rmse_radps:.9f}",
                "delta_rmse_radps": f"{run.delta_rmse_radps:.9f}",
            }
            for run in selected_runs
            if run.samples > 0
        ],
    )

    report = [
        "# Contact Correction Tuning",
        "",
        "Targets use raw gyro minus independently estimated stationary bias where available, encoders, drive commands, and timestamps. UKF targets are not used.",
        "",
        f"Base gain: {base_gain:.9f} N*s/m.",
        f"Least-squares all-sample gain: {all_gain:.9f} N*s/m.",
        f"Least-squares open-floor gain: {open_gain:.9f} N*s/m.",
        f"Least-squares fit-authoritative gain: {fit_gain:.9f} N*s/m.",
        f"Least-squares validation-only gain: {validation_gain:.9f} N*s/m.",
        f"Selected exact gain: {selected_gain:.9f} N*s/m.",
        f"Yaw denominator including wheel spin-up: {denom:.12g} kg*m^2.",
        "",
        "## Selected Exact Aggregate",
        "",
        "| Dataset | Samples | Old RMSE | Tuned RMSE | Relative delta |",
        "| --- | ---: | ---: | ---: | ---: |",
    ]
    for row in aggregate:
        report.append(
            f"| {row['dataset']} | {row['samples']} | {row['old_rmse_radps']} | {row['new_rmse_radps']} | {row['relative_delta_pct']}% |"
        )
    report.extend(
        [
            "",
            "## Selected Exact Motion Split",
            "",
            "| Family | Motion | Samples | Old RMSE | Tuned RMSE | Relative delta |",
            "| --- | --- | ---: | ---: | ---: | ---: |",
        ]
    )
    for row in motion:
        report.append(
            f"| {row['family']} | {row['motion_class']} | {row['samples']} | {row['old_rmse_radps']} | {row['new_rmse_radps']} | {row['relative_delta_pct']}% |"
        )
    report.append("")
    report.append("The linear sweep chooses candidates from one identical sample set; the selected tables above are from an exact replay with the selected coefficient.")
    (args.out_dir / "tuning_report.md").write_text("\n".join(report) + "\n", encoding="utf-8")

    command = (
        "python codex_analysis\\contact_correction_tuning\\tune_contact_correction.py "
        f"--out-dir {args.out_dir} --min-bin-count {args.min_bin_count} --sample-every {args.sample_every}"
    )
    if args.no_competition:
        command += " --no-competition"
    if args.include_uncertainty:
        command += " --include-uncertainty"
    (args.out_dir / "commands_run.txt").write_text(command + "\n", encoding="utf-8")
    print(f"selected_gain={selected_gain:.9f}", flush=True)
    print(f"out_dir={args.out_dir}", flush=True)
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Tune PlantModel contact-continuum correction gain.")
    parser.add_argument("--out-dir", type=Path, default=OUT_DIR)
    parser.add_argument("--min-bin-count", type=int, default=80)
    parser.add_argument("--sample-every", type=int, default=200)
    parser.add_argument("--no-competition", action="store_true")
    parser.add_argument("--include-uncertainty", action="store_true")
    return parser.parse_args()


def main() -> int:
    return run_tuning(parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
