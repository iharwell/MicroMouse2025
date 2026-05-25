#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import hashlib
import math
import statistics
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from codex_analysis.yaw_torque_surface.extract_yaw_torque_surface import (
    SourceParams,
    current_model_yaw_moment_nm,
    median_abs_deviation,
    percentile,
    robust_bias,
    source_params,
    trimmed_mean,
    yaw_denominator_kg_m2,
)


OUT_DIR = Path(__file__).resolve().parent


@dataclass(frozen=True)
class LogCandidate:
    run_id: str
    family: str
    path: Path


@dataclass(frozen=True)
class NormalizedRow:
    row_index: int
    raw: dict[str, str]
    time_us: int
    tick: int
    dt_us: int
    phase_id: str
    section_id: str
    primitive_id: str
    speed_bin: str
    repeat_index: str
    left_command: float
    right_command: float
    left_velocity_mps: float
    right_velocity_mps: float
    gyro_raw_radps: float
    stationary_flag: bool
    invalid: bool


@dataclass(frozen=True)
class RunSummary:
    run_id: str
    family: str
    path: Path
    input_rows: int
    kept_rows: int
    bias_radps: float
    bias_rows: int
    extracted_samples: int
    max_abs_forward_mps: float
    max_abs_yaw_radps: float
    nonzero_forward_samples: int
    cutoff_reason: str
    cutoff_tick: int | None
    cutoff_time_us: int | None
    limitation: str


@dataclass(frozen=True, slots=True)
class Sample:
    run_id: str
    family: str
    dt_s: float
    forward_velocity_mps: float
    yaw_rate_radps: float
    next_yaw_rate_radps: float
    model_yaw_moment_nm: float
    residual_additive_nm: float
    opposing_nm: float
    forward_bin_mps: float
    abs_forward_bin_mps: float
    yaw_bin_radps: float
    abs_yaw_bin_radps: float


def finite_float(row: dict[str, str], key: str, default: float | None = None) -> float:
    text = row.get(key, "")
    if text == "":
        if default is not None:
            return default
        raise ValueError(key)
    value = float(text)
    if not math.isfinite(value):
        raise ValueError(key)
    return value


def finite_int(row: dict[str, str], key: str, default: int | None = None) -> int:
    text = row.get(key, "")
    if text == "":
        if default is not None:
            return default
        raise ValueError(key)
    return int(float(text))


def round_to_step(value: float, step: float) -> float:
    if abs(value) < 0.5 * step:
        return 0.0
    return round(value / step) * step


def csv_rows_with_metadata(path: Path) -> tuple[dict[str, str], list[dict[str, str]]]:
    metadata: dict[str, str] = {}
    data_lines: list[str] = []
    with path.open("r", encoding="utf-8", errors="replace", newline="") as f:
        for line in f:
            if line.startswith("# meta,"):
                parts = line.strip().split(",", 3)
                if len(parts) >= 4:
                    metadata[parts[2]] = parts[3]
                elif len(parts) == 3:
                    metadata[parts[2]] = ""
                continue
            if line.startswith("#"):
                continue
            data_lines.append(line)
    if not data_lines:
        return metadata, []
    return metadata, list(csv.DictReader(data_lines))


def discover_logs(include_competition: bool, include_d_decode: bool) -> list[LogCandidate]:
    candidates: list[LogCandidate] = []
    for path in sorted((REPO_ROOT / "TestResults").glob("mmlog_decode_*/open_floor_main.csv")):
        run_id = path.parent.name.replace("mmlog_decode_", "")
        candidates.append(LogCandidate(run_id=run_id, family="open_floor", path=path))
    if include_d_decode:
        d_csv = OUT_DIR / "decoded_d" / "open_floor_main.csv"
        if d_csv.is_file():
            candidates.append(LogCandidate(run_id="D_open_floor_main", family="d_root_open_floor", path=d_csv))
    if include_competition:
        comp_dir = REPO_ROOT / "TestResults" / "Competition Testing Data"
        for path in sorted(comp_dir.glob("diag*.csv")):
            candidates.append(LogCandidate(run_id=path.stem, family="competition_diag", path=path))
        for path in sorted(comp_dir.glob("aux*.csv")):
            candidates.append(LogCandidate(run_id=path.stem, family="competition_aux", path=path))
    return candidates


def normalize_open_floor(row: dict[str, str], index: int) -> NormalizedRow:
    return NormalizedRow(
        row_index=index,
        raw=row,
        time_us=finite_int(row, "master_time_us", 0),
        tick=finite_int(row, "control_tick_sequence", index),
        dt_us=finite_int(row, "dt_us", 1000),
        phase_id=row.get("phase_id", ""),
        section_id=row.get("section_id", ""),
        primitive_id=row.get("primitive_id", ""),
        speed_bin=row.get("speed_bin", ""),
        repeat_index=row.get("repeat_index", ""),
        left_command=finite_float(row, "left_drive_command"),
        right_command=finite_float(row, "right_drive_command"),
        left_velocity_mps=finite_float(row, "left_encoder_velocity_mps"),
        right_velocity_mps=finite_float(row, "right_encoder_velocity_mps"),
        gyro_raw_radps=finite_float(row, "gyro_raw_radps"),
        stationary_flag=False,
        invalid=False,
    )


def normalize_competition(row: dict[str, str], index: int, metadata: dict[str, str], params: SourceParams) -> NormalizedRow:
    fan_duty = metadata.get("kRacingFanDutyCycle", "0.8")
    normalized = dict(row)
    left_velocity = finite_float(row, "left_velocity_mps")
    right_velocity = finite_float(row, "right_velocity_mps")
    normalized.update(
        {
            "master_time_us": row.get("t_us", "0"),
            "control_tick_sequence": row.get("sample", str(index)),
            "left_drive_command": row.get("left_drive_cmd", "0"),
            "right_drive_command": row.get("right_drive_cmd", "0"),
            "left_encoder_velocity_mps": f"{left_velocity:.9f}",
            "right_encoder_velocity_mps": f"{right_velocity:.9f}",
            "left_encoder_omega_radps": f"{left_velocity / params.wheel_radius_m:.9f}",
            "right_encoder_omega_radps": f"{right_velocity / params.wheel_radius_m:.9f}",
            "saturation_flags": "0",
            "watchdog_flags": "0",
            "section_id": "",
            "primitive_id": "",
            "speed_bin": "",
            "repeat_index": "",
            "fan_duty_cycle": fan_duty,
        }
    )
    return NormalizedRow(
        row_index=index,
        raw=normalized,
        time_us=finite_int(row, "t_us", 0),
        tick=finite_int(row, "sample", index),
        dt_us=finite_int(row, "dt_us", 1000),
        phase_id=row.get("phase_id", ""),
        section_id="",
        primitive_id="",
        speed_bin="",
        repeat_index="",
        left_command=finite_float(row, "left_drive_cmd"),
        right_command=finite_float(row, "right_drive_cmd"),
        left_velocity_mps=left_velocity,
        right_velocity_mps=right_velocity,
        gyro_raw_radps=finite_float(row, "gyro_raw_radps"),
        stationary_flag=row.get("stationary", "0") in {"1", "true", "True"},
        invalid=False,
    )


def read_normalized_rows(candidate: LogCandidate, params: SourceParams) -> tuple[dict[str, str], list[NormalizedRow], str]:
    metadata, rows = csv_rows_with_metadata(candidate.path)
    if not rows:
        return metadata, [], "empty csv"
    headers = set(rows[0].keys())
    normalized: list[NormalizedRow] = []
    if {"left_drive_command", "right_drive_command", "left_encoder_velocity_mps", "right_encoder_velocity_mps", "gyro_raw_radps"}.issubset(headers):
        for index, row in enumerate(rows):
            try:
                normalized.append(normalize_open_floor(row, index))
            except ValueError:
                continue
        return metadata, normalized, ""
    if {"left_drive_cmd", "right_drive_cmd", "left_velocity_mps", "right_velocity_mps", "gyro_raw_radps"}.issubset(headers):
        for index, row in enumerate(rows):
            try:
                normalized.append(normalize_competition(row, index, metadata, params))
            except ValueError:
                continue
        return metadata, normalized, ""
    return metadata, [], "missing required command/encoder/raw gyro columns"


def row_key(row: NormalizedRow) -> tuple[str, str, str, str, str]:
    return (row.section_id, row.phase_id, row.primitive_id, row.speed_bin, row.repeat_index)


def estimate_bias(rows: list[NormalizedRow]) -> tuple[float, int]:
    stationary: list[float] = []
    for row in rows:
        if not (250 <= row.dt_us <= 5000):
            continue
        near_zero_command = max(abs(row.left_command), abs(row.right_command)) <= 0.025
        near_zero_velocity = max(abs(row.left_velocity_mps), abs(row.right_velocity_mps)) <= 0.020
        if (row.stationary_flag or (near_zero_command and near_zero_velocity)) and abs(row.gyro_raw_radps) <= 0.6:
            stationary.append(row.gyro_raw_radps)
    return robust_bias(stationary)


def is_tail_quiescent_or_invalid(row: NormalizedRow, bias_radps: float) -> bool:
    if row.invalid or row.tick < 0 or not (250 <= row.dt_us <= 5000):
        return True
    yaw_rate = row.gyro_raw_radps - bias_radps
    return (
        max(abs(row.left_command), abs(row.right_command)) <= 0.03
        and max(abs(row.left_velocity_mps), abs(row.right_velocity_mps)) <= 0.03
        and abs(yaw_rate) <= 0.08
    )


def tail_cut_index(rows: list[NormalizedRow], bias_radps: float) -> tuple[int, str, int | None, int | None]:
    tail_start: int | None = None
    tail_tick: int | None = None
    tail_time: int | None = None
    for index, row in enumerate(rows):
        if is_tail_quiescent_or_invalid(row, bias_radps):
            if tail_start is None:
                tail_start = index
                tail_tick = row.tick
                tail_time = row.time_us
        else:
            tail_start = None
            tail_tick = None
            tail_time = None
    if tail_start is not None and rows:
        duration_s = max(0.0, (rows[-1].time_us - (tail_time or rows[-1].time_us)) * 1.0e-6)
        if duration_s >= 0.25:
            return tail_start, "dropped final sensor-quiescent/invalid tail >= 0.25 s", tail_tick, tail_time
    return len(rows), "kept through final sensor-active row", None, None


def valid_adjacent(current: NormalizedRow, nxt: NormalizedRow, family: str) -> bool:
    if nxt.tick <= current.tick:
        return False
    if not (250 <= nxt.dt_us <= 5000):
        return False
    if abs(current.gyro_raw_radps) > 40.0 or abs(nxt.gyro_raw_radps) > 40.0:
        return False
    if max(abs(current.left_command), abs(current.right_command), abs(nxt.left_command), abs(nxt.right_command)) > 1.05:
        return False
    if max(abs(current.left_velocity_mps), abs(current.right_velocity_mps), abs(nxt.left_velocity_mps), abs(nxt.right_velocity_mps)) > 4.0:
        return False
    if family.startswith("competition"):
        return current.phase_id == nxt.phase_id
    if current.raw.get("saturation_flags", "0") not in {"", "0"} or nxt.raw.get("saturation_flags", "0") not in {"", "0"}:
        return False
    if current.raw.get("watchdog_flags", "0") not in {"", "0"} or nxt.raw.get("watchdog_flags", "0") not in {"", "0"}:
        return False
    return row_key(current) == row_key(nxt)


def extract_samples(candidate: LogCandidate, params: SourceParams) -> tuple[RunSummary, list[Sample]]:
    _, rows, limitation = read_normalized_rows(candidate, params)
    if not rows:
        return (
            RunSummary(candidate.run_id, candidate.family, candidate.path, 0, 0, 0.0, 0, 0, 0.0, 0.0, 0, "excluded", None, None, limitation),
            [],
        )
    bias, bias_rows = estimate_bias(rows)
    cutoff_index, cutoff_reason, cutoff_tick, cutoff_time = tail_cut_index(rows, bias)
    kept_rows = rows[:cutoff_index]
    denom = yaw_denominator_kg_m2(params)
    samples: list[Sample] = []
    max_abs_forward = 0.0
    max_abs_yaw = 0.0
    nonzero_forward = 0
    for current, nxt in zip(kept_rows, kept_rows[1:]):
        if not valid_adjacent(current, nxt, candidate.family):
            continue
        yaw_rate = current.gyro_raw_radps - bias
        next_yaw_rate = nxt.gyro_raw_radps - bias
        dt_s = nxt.dt_us * 1.0e-6
        measured_yaw_accel = (next_yaw_rate - yaw_rate) / dt_s
        if abs(measured_yaw_accel) > 4000.0:
            continue
        forward_velocity = 0.5 * (current.left_velocity_mps + current.right_velocity_mps)
        if abs(yaw_rate) < 0.08 and abs(forward_velocity) < 0.05:
            continue
        try:
            model_moment = current_model_yaw_moment_nm(current.raw, forward_velocity, yaw_rate, params)
        except ValueError:
            continue
        observed_moment = denom * measured_yaw_accel
        residual = observed_moment - model_moment
        if abs(residual) > 2.0:
            continue
        opposing = -math.copysign(1.0, yaw_rate) * residual if abs(yaw_rate) > 1.0e-6 else 0.0
        forward_bin = round_to_step(forward_velocity, 0.10)
        yaw_bin = round_to_step(yaw_rate, 0.50)
        max_abs_forward = max(max_abs_forward, abs(forward_velocity))
        max_abs_yaw = max(max_abs_yaw, abs(yaw_rate))
        if abs(forward_velocity) >= 0.15:
            nonzero_forward += 1
        samples.append(
            Sample(
                run_id=candidate.run_id,
                family=candidate.family,
                dt_s=dt_s,
                forward_velocity_mps=forward_velocity,
                yaw_rate_radps=yaw_rate,
                next_yaw_rate_radps=next_yaw_rate,
                model_yaw_moment_nm=model_moment,
                residual_additive_nm=residual,
                opposing_nm=opposing,
                forward_bin_mps=forward_bin,
                abs_forward_bin_mps=abs(forward_bin),
                yaw_bin_radps=yaw_bin,
                abs_yaw_bin_radps=abs(yaw_bin),
            )
        )
    if candidate.family.startswith("competition"):
        limitation = (
            "legacy competition schema: no saturation/watchdog fields, no per-row fan duty, wheel omega derived from velocity/current radius; "
            "treated as approximate current PlantModel mirror replay"
        )
    return (
        RunSummary(
            run_id=candidate.run_id,
            family=candidate.family,
            path=candidate.path,
            input_rows=len(rows),
            kept_rows=len(kept_rows),
            bias_radps=bias,
            bias_rows=bias_rows,
            extracted_samples=len(samples),
            max_abs_forward_mps=max_abs_forward,
            max_abs_yaw_radps=max_abs_yaw,
            nonzero_forward_samples=nonzero_forward,
            cutoff_reason=cutoff_reason,
            cutoff_tick=cutoff_tick,
            cutoff_time_us=cutoff_time,
            limitation=limitation,
        ),
        samples,
    )


def rmse(values: Iterable[float]) -> float:
    materialized = list(values)
    if not materialized:
        return 0.0
    return math.sqrt(statistics.fmean(value * value for value in materialized))


def grouped_values(samples: Iterable[Sample], key_fn, value_fn) -> dict[tuple[float, float], list[float]]:
    groups: dict[tuple[float, float], list[float]] = {}
    for sample in samples:
        groups.setdefault(key_fn(sample), []).append(value_fn(sample))
    return groups


def write_signed_bins(path: Path, samples: list[Sample], min_count: int) -> int:
    sample_groups: dict[tuple[float, float], list[Sample]] = {}
    for sample in samples:
        sample_groups.setdefault((sample.forward_bin_mps, sample.yaw_bin_radps), []).append(sample)
    rows: list[dict[str, str]] = []
    for (forward_bin, yaw_bin), group in sample_groups.items():
        if len(group) < min_count:
            continue
        residuals = [sample.residual_additive_nm for sample in group]
        opposing = [sample.opposing_nm for sample in group]
        residual_ordered = sorted(residuals)
        opposing_ordered = sorted(opposing)
        median_residual = statistics.median(residuals)
        median_opposing = statistics.median(opposing)
        rows.append(
            {
                "forward_velocity_bin_mps": f"{forward_bin:.2f}",
                "yaw_rate_bin_radps": f"{yaw_bin:.2f}",
                "abs_forward_velocity_bin_mps": f"{abs(forward_bin):.2f}",
                "abs_yaw_rate_bin_radps": f"{abs(yaw_bin):.2f}",
                "count": str(len(residuals)),
                "median_residual_additive_yaw_torque_nm": f"{median_residual:.9f}",
                "trimmed_mean_residual_additive_yaw_torque_nm": f"{trimmed_mean(residuals):.9f}",
                "mad_residual_additive_yaw_torque_nm": f"{median_abs_deviation(residuals, median_residual):.9f}",
                "iqr_residual_additive_yaw_torque_nm": f"{percentile(residual_ordered, 0.75) - percentile(residual_ordered, 0.25):.9f}",
                "median_opposing_yaw_resistance_nm": f"{median_opposing:.9f}",
                "trimmed_mean_opposing_yaw_resistance_nm": f"{trimmed_mean(opposing):.9f}",
                "mad_opposing_yaw_resistance_nm": f"{median_abs_deviation(opposing, median_opposing):.9f}",
                "iqr_opposing_yaw_resistance_nm": f"{percentile(opposing_ordered, 0.75) - percentile(opposing_ordered, 0.25):.9f}",
            }
        )
    rows.sort(key=lambda row: (float(row["forward_velocity_bin_mps"]), float(row["yaw_rate_bin_radps"])))
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()) if rows else [
            "forward_velocity_bin_mps", "yaw_rate_bin_radps", "abs_forward_velocity_bin_mps", "abs_yaw_rate_bin_radps",
            "count", "median_residual_additive_yaw_torque_nm", "trimmed_mean_residual_additive_yaw_torque_nm",
            "mad_residual_additive_yaw_torque_nm", "iqr_residual_additive_yaw_torque_nm",
            "median_opposing_yaw_resistance_nm", "trimmed_mean_opposing_yaw_resistance_nm",
            "mad_opposing_yaw_resistance_nm", "iqr_opposing_yaw_resistance_nm",
        ])
        writer.writeheader()
        writer.writerows(rows)
    return len(rows)


def write_abs_summary(path: Path, samples: list[Sample], min_count: int) -> int:
    groups: dict[tuple[float, float], list[Sample]] = {}
    for sample in samples:
        groups.setdefault((sample.abs_forward_bin_mps, sample.abs_yaw_bin_radps), []).append(sample)
    rows: list[dict[str, str]] = []
    for (forward_bin, yaw_bin), group in groups.items():
        if len(group) < min_count:
            continue
        opposing = [sample.opposing_nm for sample in group]
        residual = [sample.residual_additive_nm for sample in group]
        median_opposing = statistics.median(opposing)
        ordered = sorted(opposing)
        pos_count = sum(1 for sample in group if sample.forward_velocity_mps >= 0.0)
        yaw_pos_count = sum(1 for sample in group if sample.yaw_rate_radps >= 0.0)
        rows.append(
            {
                "abs_forward_velocity_bin_mps": f"{forward_bin:.2f}",
                "abs_yaw_rate_bin_radps": f"{yaw_bin:.2f}",
                "count": str(len(group)),
                "forward_positive_count": str(pos_count),
                "forward_negative_count": str(len(group) - pos_count),
                "yaw_positive_count": str(yaw_pos_count),
                "yaw_negative_count": str(len(group) - yaw_pos_count),
                "median_opposing_yaw_resistance_nm": f"{median_opposing:.9f}",
                "trimmed_mean_opposing_yaw_resistance_nm": f"{trimmed_mean(opposing):.9f}",
                "mad_opposing_yaw_resistance_nm": f"{median_abs_deviation(opposing, median_opposing):.9f}",
                "iqr_opposing_yaw_resistance_nm": f"{percentile(ordered, 0.75) - percentile(ordered, 0.25):.9f}",
                "median_abs_residual_additive_yaw_torque_nm": f"{statistics.median(abs(value) for value in residual):.9f}",
            }
        )
    rows.sort(key=lambda row: (float(row["abs_forward_velocity_bin_mps"]), float(row["abs_yaw_rate_bin_radps"])))
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()) if rows else [
            "abs_forward_velocity_bin_mps", "abs_yaw_rate_bin_radps", "count", "forward_positive_count",
            "forward_negative_count", "yaw_positive_count", "yaw_negative_count", "median_opposing_yaw_resistance_nm",
            "trimmed_mean_opposing_yaw_resistance_nm", "mad_opposing_yaw_resistance_nm",
            "iqr_opposing_yaw_resistance_nm", "median_abs_residual_additive_yaw_torque_nm",
        ])
        writer.writeheader()
        writer.writerows(rows)
    return len(rows)


def holdout_split(run_id: str, family: str) -> bool:
    digest = hashlib.sha1(f"{family}:{run_id}".encode("utf-8")).digest()[0]
    return digest % 5 == 0


def build_surface(samples: Iterable[Sample], min_count: int) -> dict[tuple[float, float], float]:
    groups = grouped_values(samples, lambda s: (s.forward_bin_mps, s.yaw_bin_radps), lambda s: s.residual_additive_nm)
    return {key: statistics.median(values) for key, values in groups.items() if len(values) >= min_count}


def evaluate_holdout(samples: list[Sample], min_count: int) -> tuple[list[dict[str, str]], dict[str, float]]:
    train = [sample for sample in samples if not holdout_split(sample.run_id, sample.family)]
    holdout = [sample for sample in samples if holdout_split(sample.run_id, sample.family)]
    if not holdout:
        holdout = samples[::5]
        train = [sample for index, sample in enumerate(samples) if index % 5 != 0]
    surface = build_surface(train, min_count)
    by_run: dict[str, list[Sample]] = {}
    for sample in holdout:
        by_run.setdefault(f"{sample.family}:{sample.run_id}", []).append(sample)
    denom = yaw_denominator_kg_m2(source_params())
    rows: list[dict[str, str]] = []
    all_current: list[float] = []
    all_corrected: list[float] = []
    covered_current: list[float] = []
    covered_corrected: list[float] = []
    for run_id, group in sorted(by_run.items()):
        current_errors: list[float] = []
        corrected_errors: list[float] = []
        run_covered_current: list[float] = []
        run_covered_corrected: list[float] = []
        for sample in group:
            current_next = sample.yaw_rate_radps + ((sample.model_yaw_moment_nm / denom) * sample.dt_s)
            correction = surface.get((sample.forward_bin_mps, sample.yaw_bin_radps), 0.0)
            corrected_next = sample.yaw_rate_radps + (((sample.model_yaw_moment_nm + correction) / denom) * sample.dt_s)
            current_error = current_next - sample.next_yaw_rate_radps
            corrected_error = corrected_next - sample.next_yaw_rate_radps
            current_errors.append(current_error)
            corrected_errors.append(corrected_error)
            all_current.append(current_error)
            all_corrected.append(corrected_error)
            if correction != 0.0:
                run_covered_current.append(current_error)
                run_covered_corrected.append(corrected_error)
                covered_current.append(current_error)
                covered_corrected.append(corrected_error)
        rows.append(
            {
                "holdout_run": run_id,
                "samples": str(len(group)),
                "samples_with_surface_bin": str(len(run_covered_current)),
                "current_rmse_radps": f"{rmse(current_errors):.9f}",
                "surface_corrected_rmse_radps": f"{rmse(corrected_errors):.9f}",
                "covered_current_rmse_radps": f"{rmse(run_covered_current):.9f}",
                "covered_surface_corrected_rmse_radps": f"{rmse(run_covered_corrected):.9f}",
            }
        )
    aggregate = {
        "train_samples": float(len(train)),
        "holdout_samples": float(len(holdout)),
        "surface_bins": float(len(surface)),
        "samples_with_surface_bin": float(len(covered_current)),
        "current_rmse_radps": rmse(all_current),
        "surface_corrected_rmse_radps": rmse(all_corrected),
        "covered_current_rmse_radps": rmse(covered_current),
        "covered_surface_corrected_rmse_radps": rmse(covered_corrected),
    }
    return rows, aggregate


def write_run_summary(path: Path, runs: list[RunSummary]) -> None:
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "run_id", "family", "path", "input_rows", "kept_rows", "bias_radps", "bias_rows",
                "extracted_samples", "max_abs_forward_mps", "max_abs_yaw_radps", "nonzero_forward_samples",
                "cutoff_reason", "cutoff_tick", "cutoff_time_us", "limitation",
            ],
        )
        writer.writeheader()
        for run in runs:
            try:
                path_text = str(run.path.relative_to(REPO_ROOT))
            except ValueError:
                path_text = str(run.path)
            writer.writerow(
                {
                    "run_id": run.run_id,
                    "family": run.family,
                    "path": path_text,
                    "input_rows": run.input_rows,
                    "kept_rows": run.kept_rows,
                    "bias_radps": f"{run.bias_radps:.9f}",
                    "bias_rows": run.bias_rows,
                    "extracted_samples": run.extracted_samples,
                    "max_abs_forward_mps": f"{run.max_abs_forward_mps:.6f}",
                    "max_abs_yaw_radps": f"{run.max_abs_yaw_radps:.6f}",
                    "nonzero_forward_samples": run.nonzero_forward_samples,
                    "cutoff_reason": run.cutoff_reason,
                    "cutoff_tick": "" if run.cutoff_tick is None else run.cutoff_tick,
                    "cutoff_time_us": "" if run.cutoff_time_us is None else run.cutoff_time_us,
                    "limitation": run.limitation,
                }
            )


def write_rmse(path: Path, rows: list[dict[str, str]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()) if rows else [
            "holdout_run", "samples", "samples_with_surface_bin", "current_rmse_radps",
            "surface_corrected_rmse_radps", "covered_current_rmse_radps", "covered_surface_corrected_rmse_radps",
        ])
        writer.writeheader()
        writer.writerows(rows)


def coverage_by_forward(samples: list[Sample]) -> list[dict[str, str]]:
    groups: dict[float, list[Sample]] = {}
    for sample in samples:
        groups.setdefault(sample.abs_forward_bin_mps, []).append(sample)
    rows: list[dict[str, str]] = []
    for forward_bin, group in sorted(groups.items()):
        yaw_bins = {sample.abs_yaw_bin_radps for sample in group if sample.abs_yaw_bin_radps >= 0.5}
        families = sorted({sample.family for sample in group})
        rows.append(
            {
                "abs_forward_velocity_bin_mps": f"{forward_bin:.2f}",
                "count": str(len(group)),
                "yaw_bin_count": str(len(yaw_bins)),
                "max_abs_yaw_rate_bin_radps": f"{max(yaw_bins) if yaw_bins else 0.0:.2f}",
                "families": ";".join(families),
            }
        )
    return rows


def write_coverage(path: Path, rows: list[dict[str, str]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()) if rows else [
            "abs_forward_velocity_bin_mps", "count", "yaw_bin_count", "max_abs_yaw_rate_bin_radps", "families",
        ])
        writer.writeheader()
        writer.writerows(rows)


def moving_yaw_count(samples: list[Sample]) -> int:
    return sum(1 for sample in samples if abs(sample.forward_velocity_mps) > 0.02 and abs(sample.yaw_rate_radps) > 0.20)


def nonzero_forward_count(samples: list[Sample]) -> int:
    return sum(1 for sample in samples if abs(sample.forward_velocity_mps) >= 0.15)


def dataset_comparison(samples: list[Sample], min_count: int) -> list[dict[str, str]]:
    datasets = [
        ("all_included", samples),
        ("open_floor_only", [sample for sample in samples if sample.family == "open_floor"]),
        ("competition_only", [sample for sample in samples if sample.family.startswith("competition")]),
    ]
    rows: list[dict[str, str]] = []
    for label, subset in datasets:
        if not subset:
            continue
        _, aggregate = evaluate_holdout(subset, min_count)
        rows.append(
            {
                "dataset": label,
                "samples": str(len(subset)),
                "moving_yaw_samples_abs_vf_gt_0p02_abs_yaw_gt_0p2": str(moving_yaw_count(subset)),
                "nonzero_forward_samples_abs_vf_ge_0p15": str(nonzero_forward_count(subset)),
                "max_abs_forward_mps": f"{max(abs(sample.forward_velocity_mps) for sample in subset):.6f}",
                "max_abs_yaw_radps": f"{max(abs(sample.yaw_rate_radps) for sample in subset):.6f}",
                "train_surface_bins": f"{aggregate['surface_bins']:.0f}",
                "holdout_samples": f"{aggregate['holdout_samples']:.0f}",
                "holdout_samples_with_surface_bin": f"{aggregate['samples_with_surface_bin']:.0f}",
                "holdout_current_rmse_radps": f"{aggregate['current_rmse_radps']:.9f}",
                "holdout_surface_corrected_rmse_radps": f"{aggregate['surface_corrected_rmse_radps']:.9f}",
                "covered_current_rmse_radps": f"{aggregate['covered_current_rmse_radps']:.9f}",
                "covered_surface_corrected_rmse_radps": f"{aggregate['covered_surface_corrected_rmse_radps']:.9f}",
            }
        )
    return rows


def write_dataset_comparison(path: Path, rows: list[dict[str, str]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()) if rows else [
            "dataset", "samples", "moving_yaw_samples_abs_vf_gt_0p02_abs_yaw_gt_0p2",
            "nonzero_forward_samples_abs_vf_ge_0p15", "max_abs_forward_mps", "max_abs_yaw_radps",
            "train_surface_bins", "holdout_samples", "holdout_samples_with_surface_bin",
            "holdout_current_rmse_radps", "holdout_surface_corrected_rmse_radps",
            "covered_current_rmse_radps", "covered_surface_corrected_rmse_radps",
        ])
        writer.writeheader()
        writer.writerows(rows)


def top_abs_rows(samples: list[Sample], min_count: int, limit: int = 24) -> list[dict[str, str]]:
    groups: dict[tuple[float, float], list[Sample]] = {}
    for sample in samples:
        if sample.abs_forward_bin_mps < 0.20 or sample.abs_yaw_bin_radps < 0.50:
            continue
        groups.setdefault((sample.abs_forward_bin_mps, sample.abs_yaw_bin_radps), []).append(sample)
    rows: list[dict[str, str]] = []
    for (forward_bin, yaw_bin), group in groups.items():
        if len(group) < min_count:
            continue
        opposing = [sample.opposing_nm for sample in group]
        rows.append(
            {
                "abs_forward_velocity_bin_mps": f"{forward_bin:.2f}",
                "abs_yaw_rate_bin_radps": f"{yaw_bin:.2f}",
                "count": str(len(group)),
                "median_opposing_yaw_resistance_nm": f"{statistics.median(opposing):.9f}",
                "trimmed_mean_opposing_yaw_resistance_nm": f"{trimmed_mean(opposing):.9f}",
            }
        )
    rows.sort(key=lambda row: (-int(row["count"]), float(row["abs_forward_velocity_bin_mps"]), float(row["abs_yaw_rate_bin_radps"])))
    return rows[:limit]


def markdown_report(
    runs: list[RunSummary],
    samples: list[Sample],
    signed_bins: int,
    abs_bins: int,
    rmse_rows: list[dict[str, str]],
    aggregate: dict[str, float],
    comparison_rows: list[dict[str, str]],
    min_count: int,
) -> str:
    included = [run for run in runs if run.extracted_samples > 0]
    excluded = [run for run in runs if run.extracted_samples == 0]
    by_family: dict[str, int] = {}
    for sample in samples:
        by_family[sample.family] = by_family.get(sample.family, 0) + 1
    nonzero_by_family: dict[str, int] = {}
    moving_by_family: dict[str, int] = {}
    for sample in samples:
        if abs(sample.forward_velocity_mps) >= 0.15:
            nonzero_by_family[sample.family] = nonzero_by_family.get(sample.family, 0) + 1
        if abs(sample.forward_velocity_mps) > 0.02 and abs(sample.yaw_rate_radps) > 0.20:
            moving_by_family[sample.family] = moving_by_family.get(sample.family, 0) + 1
    coverage_rows = coverage_by_forward(samples)
    compact_rows = top_abs_rows(samples, min_count)
    lines = [
        "# Expanded Yaw Torque Extraction",
        "",
        "Scratch analysis only. No production code was modified.",
        "",
        "## Method",
        "",
        "Targets are sensor-derived only: raw gyro yaw rate minus an independently estimated stationary raw-gyro bias where stationary rows exist, encoder-derived forward velocity, logged drive commands, logged or derived wheel speeds, and timestamps. `ukf_state_*` and estimator diagnostics are not fit targets.",
        "",
        "Residual sign convention follows the reconciled defensible basis: `residual_additive_yaw_torque_nm = observed_yaw_moment_nm - current_model_yaw_moment_nm`. This is the additive yaw torque that would be added to the current yaw-relevant PlantModel mirror for one sample. `+Yaw` is clockwise. `opposing_yaw_resistance_nm = -sign(sensor_yaw_rate) * residual_additive_yaw_torque_nm`, so positive opposing torque resists the current yaw motion.",
        "",
        "The current-model mirror is the prior reconciled scratch mirror of the yaw-relevant `PlantModel` terms. For legacy competition CSVs, wheel omega is derived from encoder velocity and current wheel radius, saturation/watchdog fields are unavailable, phase identity and command consistency carry more of the gating, and fan duty comes from metadata/default 0.8. Those runs are useful for coverage but are less authoritative than current decoded `open_floor_main.csv` logs.",
        "",
        "The D:\\ raw open-floor capture is not included by default because it duplicates `TestResults\\mmlog_decode_2026-05-04_20-35-47`; pass `--include-d-decode` only when intentionally checking that duplicate decode.",
        "",
        f"Signed surface bins require at least {min_count} samples. Forward velocity is binned at 0.10 m/s and yaw rate at 0.50 rad/s.",
        "",
        "## Log Use",
        "",
        f"Included runs: {len(included)}; excluded/zero-sample candidates: {len(excluded)}; extracted samples: {len(samples)}.",
        "",
        "| Family | Samples |",
        "| --- | ---: |",
    ]
    for family, count in sorted(by_family.items()):
        lines.append(f"| {family} | {count} |")
    lines.extend([
        "",
        "| Family | Moving-yaw samples `|Vf|>0.02`, `|gyro|>0.2` | Nonzero-Vf samples `|Vf|>=0.15` |",
        "| --- | ---: | ---: |",
    ])
    for family in sorted(by_family):
        lines.append(f"| {family} | {moving_by_family.get(family, 0)} | {nonzero_by_family.get(family, 0)} |")
    lines.extend([
        "",
        "Full per-run inventory, tail cuts, bias rows, and limitations are in `expanded_yaw_torque_run_summary.csv`.",
        "",
        "## Competition Impact",
        "",
        "| Dataset | Samples | Moving-yaw samples | Nonzero-Vf samples | Holdout current RMSE | Holdout surface RMSE | Covered current RMSE | Covered surface RMSE |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ])
    for row in comparison_rows:
        lines.append(
            f"| {row['dataset']} | {row['samples']} | {row['moving_yaw_samples_abs_vf_gt_0p02_abs_yaw_gt_0p2']} | {row['nonzero_forward_samples_abs_vf_ge_0p15']} | {row['holdout_current_rmse_radps']} | {row['holdout_surface_corrected_rmse_radps']} | {row['covered_current_rmse_radps']} | {row['covered_surface_corrected_rmse_radps']} |"
        )
    lines.extend([
        "",
        "Competition logs materially add maze-turn coverage and make the surface less dominated by open-floor characterization phases. Because the competition schema is older and lacks saturation/watchdog fields, the all-included surface should be interpreted as an exploratory calibration candidate; `open_floor_only` remains the cleaner validation subset.",
        "",
        "## Coverage By Absolute Forward Velocity",
        "",
        "| Abs Vf bin m/s | Count | Yaw bins | Max abs yaw bin rad/s | Families |",
        "| ---: | ---: | ---: | ---: | --- |",
    ])
    for row in coverage_rows:
        if int(row["count"]) >= min_count:
            lines.append(
                f"| {row['abs_forward_velocity_bin_mps']} | {row['count']} | {row['yaw_bin_count']} | {row['max_abs_yaw_rate_bin_radps']} | {row['families']} |"
            )
    lines.extend([
        "",
        "## Useful Nonzero-Vf Counter-Yaw Surface Rows",
        "",
        "This compact table combines signs by absolute forward velocity and absolute yaw rate for readability. Use `expanded_yaw_torque_surface_signed_bins.csv` for the signed additive residual table.",
        "",
        "| Abs Vf bin m/s | Abs yaw bin rad/s | Count | Median opposing Nm | Trimmed mean opposing Nm |",
        "| ---: | ---: | ---: | ---: | ---: |",
    ])
    for row in compact_rows:
        lines.append(
            f"| {row['abs_forward_velocity_bin_mps']} | {row['abs_yaw_rate_bin_radps']} | {row['count']} | {row['median_opposing_yaw_resistance_nm']} | {row['trimmed_mean_opposing_yaw_resistance_nm']} |"
        )
    lines.extend([
        "",
        "## Holdout RMSE",
        "",
        f"Deterministic run-level holdout split: {int(aggregate['train_samples'])} train samples, {int(aggregate['holdout_samples'])} holdout samples, {int(aggregate['surface_bins'])} train surface bins.",
        "",
        "| Metric | RMSE rad/s |",
        "| --- | ---: |",
        f"| Current model, all holdout samples | {aggregate['current_rmse_radps']:.9f} |",
        f"| Surface corrected, all holdout samples | {aggregate['surface_corrected_rmse_radps']:.9f} |",
        f"| Current model, holdout samples with trained surface bin | {aggregate['covered_current_rmse_radps']:.9f} |",
        f"| Surface corrected, holdout samples with trained surface bin | {aggregate['covered_surface_corrected_rmse_radps']:.9f} |",
        "",
        f"Holdout samples with a trained signed bin: {int(aggregate['samples_with_surface_bin'])}. Per-run holdout rows are in `expanded_yaw_torque_holdout_rmse.csv`.",
        "",
        "## Recommendation",
        "",
        "The expanded logs are enough to show a real velocity/yaw-rate-dependent residual structure beyond `Vf=0`, especially through 0.2-0.5 m/s with yaw-rate bins spanning normal maze turns and loops. They are not yet strong enough to install a production PlantModel-owned tire/yaw-resistance surface directly: several high-count bins change sign, competition rows are legacy-schema approximations, and high-speed forward coverage above about 0.7 m/s is thin.",
        "",
        "A future production change should keep ownership in `PlantModel` and derive constants from `Vehicle`/drive owners. The still-needed targeted data is: symmetric clockwise/counter-clockwise maneuver runs at 0.6, 0.8, and 1.0+ m/s; repeated yaw-rate plateaus around 2-8 rad/s while moving; and clean per-row fan duty/saturation/watchdog logging for competition-like maze runs.",
        "",
        "## Outputs",
        "",
        f"- Signed bins: {signed_bins}",
        f"- Absolute summary bins: {abs_bins}",
    ])
    return "\n".join(lines) + "\n"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Expanded sensor-only yaw torque residual extraction.")
    parser.add_argument("--out-dir", type=Path, default=OUT_DIR)
    parser.add_argument("--min-bin-count", type=int, default=80)
    parser.add_argument("--no-competition", action="store_true")
    parser.add_argument("--include-d-decode", action="store_true", help="Include scratch decoded D:\\ open-floor capture, even though it duplicates 2026-05-04_20-35-47.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)
    params = source_params()
    candidates = discover_logs(include_competition=not args.no_competition, include_d_decode=args.include_d_decode)
    runs: list[RunSummary] = []
    samples: list[Sample] = []
    for candidate in candidates:
        run, run_samples = extract_samples(candidate, params)
        runs.append(run)
        samples.extend(run_samples)
        print(f"{candidate.family}:{candidate.run_id}: rows={run.input_rows} samples={len(run_samples)}")
    if not samples:
        raise SystemExit("No usable yaw-torque samples found.")
    signed_bins = write_signed_bins(args.out_dir / "expanded_yaw_torque_surface_signed_bins.csv", samples, args.min_bin_count)
    abs_bins = write_abs_summary(args.out_dir / "expanded_yaw_torque_surface_abs_summary.csv", samples, args.min_bin_count)
    rmse_rows, aggregate = evaluate_holdout(samples, args.min_bin_count)
    comparison_rows = dataset_comparison(samples, args.min_bin_count)
    write_run_summary(args.out_dir / "expanded_yaw_torque_run_summary.csv", runs)
    write_rmse(args.out_dir / "expanded_yaw_torque_holdout_rmse.csv", rmse_rows)
    write_dataset_comparison(args.out_dir / "expanded_yaw_torque_dataset_comparison.csv", comparison_rows)
    write_coverage(args.out_dir / "expanded_yaw_torque_coverage_by_forward.csv", coverage_by_forward(samples))
    report = markdown_report(runs, samples, signed_bins, abs_bins, rmse_rows, aggregate, comparison_rows, args.min_bin_count)
    (args.out_dir / "expanded_yaw_torque_report.md").write_text(report, encoding="utf-8")
    print(f"out_dir={args.out_dir}")
    print(f"samples={len(samples)}")
    print(f"signed_bins={signed_bins}")
    print(f"abs_bins={abs_bins}")
    print(f"holdout_current_rmse_radps={aggregate['current_rmse_radps']:.9f}")
    print(f"holdout_surface_corrected_rmse_radps={aggregate['surface_corrected_rmse_radps']:.9f}")
    print(f"covered_current_rmse_radps={aggregate['covered_current_rmse_radps']:.9f}")
    print(f"covered_surface_corrected_rmse_radps={aggregate['covered_surface_corrected_rmse_radps']:.9f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
