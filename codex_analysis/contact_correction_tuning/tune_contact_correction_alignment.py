#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Callable


REPO_ROOT = Path(__file__).resolve().parents[2]
OUT_DIR = Path(__file__).resolve().parent / "alignment_tuning"
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from codex_analysis.contact_correction_log_eval import evaluate_contact_correction as replay  # noqa: E402
from codex_analysis.contact_continuum_yaw_identification.features import (  # noqa: E402
    extract_contact_continuum_features as prior,
)


@dataclass(frozen=True)
class AlignedSample:
    run_id: str
    family: str
    recommendation: str
    response_lag_samples: int
    command_tick: int
    target_start_tick: int
    dt_s: float
    old_error_radps: float
    new_error_radps: float
    unit_patch_error_radps: float
    patch_delta_yaw_moment_nm: float
    motion_class: str


@dataclass
class GroupStats:
    label: str
    count: int = 0
    sum_e2: float = 0.0
    sum_ex: float = 0.0
    sum_x2: float = 0.0
    worsened_at_base: int = 0

    def add(self, sample: AlignedSample) -> None:
        self.count += 1
        self.sum_e2 += sample.old_error_radps * sample.old_error_radps
        self.sum_ex += sample.old_error_radps * sample.unit_patch_error_radps
        self.sum_x2 += sample.unit_patch_error_radps * sample.unit_patch_error_radps
        if abs(sample.new_error_radps) > abs(sample.old_error_radps):
            self.worsened_at_base += 1

    def old_rmse(self) -> float:
        return math.sqrt(self.sum_e2 / self.count) if self.count else 0.0

    def new_rmse(self, gain: float) -> float:
        if not self.count:
            return 0.0
        sse = self.sum_e2 + (2.0 * gain * self.sum_ex) + (gain * gain * self.sum_x2)
        return math.sqrt(max(0.0, sse / self.count))

    def optimal_gain(self) -> float:
        return (-self.sum_ex / self.sum_x2) if self.sum_x2 > 0.0 else 0.0


def rmse(values: list[float]) -> float:
    if not values:
        return 0.0
    return math.sqrt(statistics.fmean(value * value for value in values))


def pct_delta(new_value: float, old_value: float) -> float:
    return ((new_value / old_value) - 1.0) * 100.0 if old_value > 0.0 else 0.0


def write_csv(path: Path, rows: list[dict[str, str]], fieldnames: list[str] | None = None) -> None:
    if fieldnames is None:
        fieldnames = list(rows[0].keys()) if rows else ["dataset"]
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def load_recommendations() -> dict[str, str]:
    path = (
        REPO_ROOT
        / "codex_analysis"
        / "contact_continuum_yaw_identification"
        / "data_quality"
        / "data_quality_recommendations_by_run.csv"
    )
    if not path.is_file():
        return {}
    with path.open("r", newline="", encoding="utf-8") as f:
        return {row["run_id"]: row["recommendation"] for row in csv.DictReader(f)}


def same_window(rows: list[prior.NormalizedRow], start: int, end: int, family: str) -> bool:
    if end >= len(rows):
        return False
    for index in range(start, end):
        if not prior.valid_adjacent(rows[index], rows[index + 1], family):
            return False
    key = prior.row_key(rows[start], family)
    return all(prior.row_key(rows[index], family) == key for index in range(start + 1, end + 1))


def motion_class(row: prior.NormalizedRow, bias_radps: float) -> str:
    yaw_rate = row.gyro_raw_radps - bias_radps
    forward_velocity = 0.5 * (row.left_velocity_mps + row.right_velocity_mps)
    if abs(yaw_rate) >= 0.20 and abs(forward_velocity) < 0.05:
        return "in_place_yaw"
    if abs(yaw_rate) >= 0.20 and abs(forward_velocity) >= 0.05:
        return "moving_yaw"
    if abs(forward_velocity) >= 0.05:
        return "mostly_forward"
    return "low_motion_commanded"


def evaluate_aligned_pair(
    rows: list[prior.NormalizedRow],
    index: int,
    response_lag_samples: int,
    candidate: prior.LogCandidate,
    recommendation: str,
    params: replay.Params,
    denom: float,
    bias_radps: float,
    base_gain: float,
) -> AlignedSample | None:
    command_row = rows[index]
    target_start = rows[index + response_lag_samples]
    target_end = rows[index + response_lag_samples + 1]
    command_yaw_rate = command_row.gyro_raw_radps - bias_radps
    command_forward_velocity = 0.5 * (command_row.left_velocity_mps + command_row.right_velocity_mps)
    if not prior.active_contact_sample(command_row, command_yaw_rate, command_forward_velocity):
        return None

    start_yaw_rate = target_start.gyro_raw_radps - bias_radps
    end_yaw_rate = target_end.gyro_raw_radps - bias_radps
    dt_s = target_end.dt_us * 1.0e-6
    measured_yaw_accel = (end_yaw_rate - start_yaw_rate) / dt_s
    if not math.isfinite(measured_yaw_accel) or abs(measured_yaw_accel) > 4000.0:
        return None

    old_result = replay.model_result(
        command_row,
        command_forward_velocity,
        command_yaw_rate,
        params,
        include_contact_correction=False,
    )
    new_result = replay.model_result(
        command_row,
        command_forward_velocity,
        command_yaw_rate,
        params,
        include_contact_correction=True,
    )
    observed_yaw_moment_nm = denom * measured_yaw_accel
    old_residual_nm = observed_yaw_moment_nm - old_result.model_yaw_moment_nm
    new_residual_nm = observed_yaw_moment_nm - new_result.model_yaw_moment_nm
    if not math.isfinite(old_residual_nm) or not math.isfinite(new_residual_nm) or abs(old_residual_nm) > 2.0:
        return None

    old_error = start_yaw_rate + ((old_result.model_yaw_moment_nm / denom) * dt_s) - end_yaw_rate
    new_error = start_yaw_rate + ((new_result.model_yaw_moment_nm / denom) * dt_s) - end_yaw_rate
    unit_patch_error = ((new_error - old_error) / base_gain) if base_gain != 0.0 else 0.0
    return AlignedSample(
        run_id=candidate.run_id,
        family=candidate.family,
        recommendation=recommendation,
        response_lag_samples=response_lag_samples,
        command_tick=command_row.tick,
        target_start_tick=target_start.tick,
        dt_s=dt_s,
        old_error_radps=old_error,
        new_error_radps=new_error,
        unit_patch_error_radps=unit_patch_error,
        patch_delta_yaw_moment_nm=new_result.model_yaw_moment_nm - old_result.model_yaw_moment_nm,
        motion_class=motion_class(command_row, bias_radps),
    )


def collect_aligned_samples(args: argparse.Namespace, params: replay.Params) -> list[AlignedSample]:
    recommendations = load_recommendations()
    denom = replay.yaw_denominator_kg_m2(params)
    base_gain = params.contact_yaw_patch_force_gain_ns_per_m
    samples: list[AlignedSample] = []
    candidates = prior.discover_logs(
        include_competition=not args.no_competition,
        include_uncertainty=args.include_uncertainty,
    )
    for candidate in candidates:
        recommendation = recommendations.get(candidate.run_id, "not_in_data_quality")
        _, rows, _, input_rows = prior.read_normalized_rows(candidate, params)
        run_counts = {lag: 0 for lag in args.response_lags}
        if rows:
            bias, _ = prior.estimate_bias(rows)
            cutoff_index, _, _, _ = prior.tail_cut_index(rows, bias)
            kept_rows = rows[:cutoff_index]
            for index in range(0, len(kept_rows)):
                for lag in args.response_lags:
                    target_end_index = index + lag + 1
                    if not same_window(kept_rows, index, target_end_index, candidate.family):
                        continue
                    sample = evaluate_aligned_pair(
                        kept_rows,
                        index,
                        lag,
                        candidate,
                        recommendation,
                        params,
                        denom,
                        bias,
                        base_gain,
                    )
                    if sample is not None:
                        samples.append(sample)
                        run_counts[lag] += 1
        print(
            f"{candidate.family}:{candidate.run_id}: rows={input_rows} "
            + " ".join(f"lag{lag}={run_counts[lag]}" for lag in args.response_lags),
            flush=True,
        )
    return samples


def group_predicates() -> list[tuple[str, Callable[[AlignedSample], bool]]]:
    return [
        ("all_included", lambda sample: True),
        ("open_floor_only", lambda sample: sample.family == "open_floor"),
        ("fit_authoritative_open_floor", lambda sample: sample.family == "open_floor" and sample.recommendation == "fit_authoritative"),
        ("fit_downweighted_open_floor", lambda sample: sample.family == "open_floor" and sample.recommendation == "fit_downweighted"),
        ("validation_only_open_floor", lambda sample: sample.family == "open_floor" and sample.recommendation == "validation_only"),
        ("competition_stress", lambda sample: sample.family.startswith("competition")),
    ]


def build_group_stats(samples: list[AlignedSample]) -> dict[tuple[int, str], GroupStats]:
    groups: dict[tuple[int, str], GroupStats] = {}
    for sample in samples:
        for label, predicate in group_predicates():
            if predicate(sample):
                key = (sample.response_lag_samples, label)
                groups.setdefault(key, GroupStats(label)).add(sample)
        key = (sample.response_lag_samples, f"motion:all:{sample.motion_class}")
        groups.setdefault(key, GroupStats(key[1])).add(sample)
        if sample.family == "open_floor":
            key = (sample.response_lag_samples, f"motion:open_floor:{sample.motion_class}")
            groups.setdefault(key, GroupStats(key[1])).add(sample)
    return groups


def stats_row(lag: int, label: str, stats: GroupStats, gain: float) -> dict[str, str]:
    old = stats.old_rmse()
    new = stats.new_rmse(gain)
    return {
        "response_lag_samples": str(lag),
        "dataset": label,
        "samples": str(stats.count),
        "old_rmse_radps": f"{old:.9f}",
        "new_rmse_radps": f"{new:.9f}",
        "delta_rmse_radps": f"{new - old:.9f}",
        "relative_delta_pct": f"{pct_delta(new, old):.6f}",
    }


def sweep_candidate_gains(groups: dict[tuple[int, str], GroupStats], lags: list[int]) -> list[dict[str, str]]:
    rows: list[dict[str, str]] = []
    for lag in lags:
        fit = groups[(lag, "fit_authoritative_open_floor")]
        open_floor = groups[(lag, "open_floor_only")]
        all_samples = groups[(lag, "all_included")]
        validation = groups.get((lag, "validation_only_open_floor"))
        competition = groups.get((lag, "competition_stress"))
        gain_seeds = {
            0.0,
            fit.optimal_gain(),
            open_floor.optimal_gain(),
            all_samples.optimal_gain(),
            -0.02,
            -0.05,
            -0.10,
            -0.25,
            -0.50,
            -1.0,
            -2.0,
            -3.0,
            -4.0,
            -5.0,
            -6.0,
            -8.0,
            -10.0,
            0.02,
            0.05,
            0.10,
            0.25,
            0.50,
            1.0,
        }
        for center in [fit.optimal_gain(), open_floor.optimal_gain(), all_samples.optimal_gain()]:
            step = max(0.05, min(0.50, abs(center) * 0.10))
            for index in range(-10, 11):
                gain_seeds.add(center + (index * step))
        gains = sorted(gain for gain in gain_seeds if math.isfinite(gain) and -20.0 <= gain <= 5.0)
        for gain in gains:
            fit_old = fit.old_rmse()
            fit_new = fit.new_rmse(gain)
            row = {
                "response_lag_samples": str(lag),
                "gain_ns_per_m": f"{gain:.9f}",
                "fit_old_rmse_radps": f"{fit_old:.9f}",
                "fit_new_rmse_radps": f"{fit_new:.9f}",
                "fit_relative_delta_pct": f"{pct_delta(fit_new, fit_old):.6f}",
                "open_floor_new_rmse_radps": f"{open_floor.new_rmse(gain):.9f}",
                "all_new_rmse_radps": f"{all_samples.new_rmse(gain):.9f}",
                "in_place_new_rmse_radps": f"{groups[(lag, 'motion:all:in_place_yaw')].new_rmse(gain):.9f}",
                "moving_new_rmse_radps": f"{groups[(lag, 'motion:all:moving_yaw')].new_rmse(gain):.9f}",
            }
            if validation is not None:
                row["validation_new_rmse_radps"] = f"{validation.new_rmse(gain):.9f}"
                row["validation_relative_delta_pct"] = f"{pct_delta(validation.new_rmse(gain), validation.old_rmse()):.6f}"
            if competition is not None:
                row["competition_new_rmse_radps"] = f"{competition.new_rmse(gain):.9f}"
                row["competition_relative_delta_pct"] = f"{pct_delta(competition.new_rmse(gain), competition.old_rmse()):.6f}"
            rows.append(row)
    return rows


def select_policy(groups: dict[tuple[int, str], GroupStats], lags: list[int]) -> tuple[int, float, str]:
    best: tuple[float, int, float, str] | None = None
    for lag in lags:
        fit = groups[(lag, "fit_authoritative_open_floor")]
        validation = groups.get((lag, "validation_only_open_floor"))
        candidates = [
            fit.optimal_gain(),
            groups[(lag, "open_floor_only")].optimal_gain(),
            groups[(lag, "all_included")].optimal_gain(),
            -0.5,
            -1.0,
            -2.0,
            -3.0,
            -4.0,
            -5.0,
            -6.0,
            -8.0,
        ]
        for gain in candidates:
            if not math.isfinite(gain) or not (-20.0 <= gain <= 5.0):
                continue
            fit_delta = pct_delta(fit.new_rmse(gain), fit.old_rmse())
            if fit_delta > -1.0:
                continue
            validation_delta = 0.0
            if validation is not None:
                validation_delta = pct_delta(validation.new_rmse(gain), validation.old_rmse())
            # Validation-only data is reported, but not allowed to veto unless the same-family holdout clearly degrades.
            if validation_delta > 2.0:
                continue
            score = fit.new_rmse(gain)
            reason = f"fit_primary_validation_delta_pct={validation_delta:.6f}"
            if best is None or score < best[0]:
                best = (score, lag, gain, reason)
    if best is None:
        return 0, 0.0, "no_material_fit_candidate"
    return best[1], best[2], best[3]


def exact_rows(samples: list[AlignedSample], gain: float) -> tuple[list[dict[str, str]], list[dict[str, str]]]:
    groups = build_group_stats(samples)
    aggregate_rows: list[dict[str, str]] = []
    motion_rows: list[dict[str, str]] = []
    for (lag, label), stats in sorted(groups.items()):
        if label.startswith("motion:"):
            parts = label.split(":")
            row = stats_row(lag, label, stats, gain)
            row["family"] = parts[1]
            row["motion_class"] = parts[2]
            motion_rows.append(row)
        else:
            aggregate_rows.append(stats_row(lag, label, stats, gain))
    return aggregate_rows, motion_rows


def split_quality_rows(groups: dict[tuple[int, str], GroupStats], lags: list[int]) -> list[dict[str, str]]:
    rows: list[dict[str, str]] = []
    for lag in lags:
        fit_old = groups[(lag, "fit_authoritative_open_floor")].old_rmse()
        for label in ["fit_downweighted_open_floor", "validation_only_open_floor", "competition_stress"]:
            stats = groups.get((lag, label))
            if stats is None:
                continue
            old = stats.old_rmse()
            offset_pct = pct_delta(old, fit_old)
            policy = "keep_validation"
            if abs(offset_pct) >= 50.0:
                policy = "stress_only_off_distribution"
            rows.append(
                {
                    "response_lag_samples": str(lag),
                    "dataset": label,
                    "samples": str(stats.count),
                    "fit_authoritative_old_rmse_radps": f"{fit_old:.9f}",
                    "dataset_old_rmse_radps": f"{old:.9f}",
                    "offset_vs_fit_pct": f"{offset_pct:.6f}",
                    "policy": policy,
                }
            )
    return rows


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Tune contact correction with command-to-gyro response alignment variants.")
    parser.add_argument("--out-dir", type=Path, default=OUT_DIR)
    parser.add_argument("--response-lags", type=int, nargs="+", default=[0, 1, 2])
    parser.add_argument("--forced-gain", type=float)
    parser.add_argument("--no-competition", action="store_true")
    parser.add_argument("--include-uncertainty", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)
    params = replay.source_params()
    if args.forced_gain is not None:
        params = replace(params, contact_yaw_patch_force_gain_ns_per_m=args.forced_gain)
    samples = collect_aligned_samples(args, params)
    groups = build_group_stats(samples)
    split_rows = split_quality_rows(groups, args.response_lags)
    sweep_rows = sweep_candidate_gains(groups, args.response_lags)
    if args.forced_gain is None:
        selected_lag, selected_gain, selected_reason = select_policy(groups, args.response_lags)
    else:
        selected_lag = max(args.response_lags)
        selected_gain = args.forced_gain
        selected_reason = "forced_gain_exact_replay"
    aggregate_rows, motion_rows = exact_rows(samples, selected_gain)

    write_csv(args.out_dir / "split_quality.csv", split_rows)
    write_csv(args.out_dir / "gain_sweep_by_alignment.csv", sweep_rows)
    write_csv(args.out_dir / "selected_aggregate_by_alignment.csv", aggregate_rows)
    write_csv(
        args.out_dir / "selected_motion_by_alignment.csv",
        motion_rows,
        ["response_lag_samples", "family", "motion_class", "dataset", "samples", "old_rmse_radps", "new_rmse_radps", "delta_rmse_radps", "relative_delta_pct"],
    )
    summary = [
        "# Contact Correction Alignment Tuning",
        "",
        "Targets use raw gyro minus independently estimated stationary bias where available, encoders, drive commands, and timestamps. UKF targets are not used.",
        "",
        f"Selected response lag: +{selected_lag} sample(s).",
        f"Selected gain: {selected_gain:.9f} N*s/m.",
        f"Selection reason: {selected_reason}.",
        "",
        "## Split Quality",
        "",
        "| Lag | Dataset | Samples | Fit old RMSE | Dataset old RMSE | Offset vs fit | Policy |",
        "| ---: | --- | ---: | ---: | ---: | ---: | --- |",
    ]
    for row in split_rows:
        summary.append(
            f"| {row['response_lag_samples']} | {row['dataset']} | {row['samples']} | {row['fit_authoritative_old_rmse_radps']} | {row['dataset_old_rmse_radps']} | {row['offset_vs_fit_pct']}% | {row['policy']} |"
        )
    summary.extend(
        [
            "",
            "## Selected Gain By Alignment",
            "",
            "| Lag | Dataset | Samples | Old RMSE | Tuned RMSE | Relative delta |",
            "| ---: | --- | ---: | ---: | ---: | ---: |",
        ]
    )
    for row in aggregate_rows:
        if row["dataset"] in {"fit_authoritative_open_floor", "validation_only_open_floor", "competition_stress", "open_floor_only"}:
            summary.append(
                f"| {row['response_lag_samples']} | {row['dataset']} | {row['samples']} | {row['old_rmse_radps']} | {row['new_rmse_radps']} | {row['relative_delta_pct']}% |"
            )
    (args.out_dir / "alignment_tuning_report.md").write_text("\n".join(summary) + "\n", encoding="utf-8")
    (args.out_dir / "commands_run.txt").write_text(
        "python codex_analysis\\contact_correction_tuning\\tune_contact_correction_alignment.py "
        f"--out-dir {args.out_dir} --response-lags {' '.join(str(lag) for lag in args.response_lags)}"
        + ("" if args.forced_gain is None else f" --forced-gain {args.forced_gain:.9f}")
        + "\n",
        encoding="utf-8",
    )
    print(f"selected_lag={selected_lag}")
    print(f"selected_gain={selected_gain:.9f}")
    print(f"out_dir={args.out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
