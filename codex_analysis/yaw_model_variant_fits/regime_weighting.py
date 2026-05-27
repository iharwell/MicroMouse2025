#!/usr/bin/env python3
"""Shared 4D operating-regime weights for yaw/traction analysis fits.

Analysis-only utility. Regimes are fixed physical cells in
(`Vf`, `YawRate`, `Af`, `YawAccel`) space. The bins are intentionally not
quantiles: changing the sample population must not redefine the regime map.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence


PRIMARY_SPLIT = "primary_open_floor_fit_authoritative"
DOWNWEIGHTED_SPLIT = "open_floor_fit_downweighted"
VALIDATION_SPLITS = {
    "open_floor_validation_only",
    "diag_validation_only",
    "aux_downweighted_validation",
}

DEFAULT_FIT_SPLIT_WEIGHTS: dict[str, float] = {
    PRIMARY_SPLIT: 1.0,
    DOWNWEIGHTED_SPLIT: 0.25,
    "open_floor_validation_only": 0.0,
    "diag_validation_only": 0.0,
    "aux_downweighted_validation": 0.0,
    "excluded_or_unclassified": 0.0,
}

PRIMARY_ONLY_SPLIT_WEIGHTS: dict[str, float] = {
    PRIMARY_SPLIT: 1.0,
    DOWNWEIGHTED_SPLIT: 0.0,
    "open_floor_validation_only": 0.0,
    "diag_validation_only": 0.0,
    "aux_downweighted_validation": 0.0,
    "excluded_or_unclassified": 0.0,
}

LATEST_FIT_RUNS = (
    "2026-05-04_20-35-47",
    "2026-05-04_16-57-53",
)

LATEST_INCLUSIVE_FIT_SPLIT_WEIGHTS: dict[str, float] = {
    PRIMARY_SPLIT: 1.0,
    DOWNWEIGHTED_SPLIT: 0.25,
    "open_floor_validation_only": 0.25,
    "diag_validation_only": 0.25,
    "aux_downweighted_validation": 0.25,
    "excluded_or_unclassified": 0.0,
}

# Fixed signed physical bins. These preserve low-speed calibration resolution
# around zero while keeping high-yaw/high-accel tails from creating one row per
# regime. The names stay local to this analysis utility.
FORWARD_VELOCITY_BINS_MPS = (
    -math.inf,
    -0.75,
    -0.50,
    -0.30,
    -0.15,
    -0.05,
    0.05,
    0.15,
    0.30,
    0.50,
    0.75,
    1.00,
    math.inf,
)

YAW_RATE_BINS_RADPS = (
    -math.inf,
    -12.0,
    -8.0,
    -5.0,
    -3.0,
    -1.5,
    -0.5,
    -0.1,
    0.1,
    0.5,
    1.5,
    3.0,
    5.0,
    8.0,
    12.0,
    math.inf,
)

FORWARD_ACCEL_BINS_MPS2 = (
    -math.inf,
    -8.0,
    -4.0,
    -2.0,
    -1.0,
    -0.25,
    0.25,
    1.0,
    2.0,
    4.0,
    8.0,
    math.inf,
)

YAW_ACCEL_BINS_RADPS2 = (
    -math.inf,
    -800.0,
    -400.0,
    -200.0,
    -100.0,
    -40.0,
    -10.0,
    10.0,
    40.0,
    100.0,
    200.0,
    400.0,
    800.0,
    math.inf,
)


@dataclass(frozen=True)
class QualityConfig:
    gyro_spike_multiplier: float = 0.10
    saturation_multiplier: float = 0.35
    use_limiter_penalty: bool = False
    limiter_gain: float = 4.0
    use_low_yaw_no_motion_penalty: bool = False
    low_yaw_multiplier: float = 0.25
    low_yaw_rate_radps: float = 0.02
    low_yaw_motion_mps: float = 0.002
    bad_af_gap_multiplier: float = 0.25
    quality_floor: float = 0.02


@dataclass(frozen=True)
class RegimeWeightConfig:
    split_weights: Mapping[str, float] | None = None
    quality: QualityConfig = QualityConfig()
    sparse_min_rows: int = 20
    sparse_min_runs: int = 2
    bad_af_gap_s: float = 0.25
    calibration_priority: float = 1.35
    sparse_cell_priority_cap: float = 0.35


@dataclass
class RegimeWeightResult:
    weights: list[float]
    forward_accel_mps2: list[float]
    bad_af_gap: list[bool]
    quality: list[float]
    full_cell_keys: list[tuple[str, str, str, str]]
    effective_cell_keys: list[tuple[str, str, str, str]]
    merge_levels: list[str]
    cell_rows: list[dict[str, Any]]
    marginal_rows: list[dict[str, Any]]
    summary: dict[str, Any]


def dataframe_columns(frame: Any) -> dict[str, list[Any]]:
    """Extract columns from a pandas-like frame without importing pandas here."""
    return {str(column): list(frame[column]) for column in frame.columns}


def finite_float(value: Any, default: float = 0.0) -> float:
    try:
        out = float(value)
        return out if math.isfinite(out) else default
    except (TypeError, ValueError):
        return default


def _string_column(columns: Mapping[str, Sequence[Any]], name: str, n: int, default: str = "") -> list[str]:
    values = columns.get(name)
    if values is None:
        return [default] * n
    return [str(value) for value in values]


def _float_column(columns: Mapping[str, Sequence[Any]], name: str, n: int, default: float = 0.0) -> list[float]:
    values = columns.get(name)
    if values is None:
        return [default] * n
    return [finite_float(value, default) for value in values]


def _bool_mask(mask: Sequence[Any] | None, n: int) -> list[bool]:
    if mask is None:
        return [True] * n
    return [bool(value) for value in mask]


def _bin_label(value: float, edges: Sequence[float]) -> str:
    if not math.isfinite(value):
        value = 0.0
    for index in range(len(edges) - 1):
        lo = edges[index]
        hi = edges[index + 1]
        if value < hi or index == len(edges) - 2:
            lo_text = "-inf" if lo == -math.inf else f"{lo:g}"
            hi_text = "inf" if hi == math.inf else f"{hi:g}"
            return f"[{lo_text},{hi_text})"
    return "[bad,bad)"


def _derive_forward_accel(
    run_ids: Sequence[str],
    row_indices: Sequence[float],
    time_us: Sequence[float],
    forward_mps: Sequence[float],
    max_gap_s: float,
) -> tuple[list[float], list[bool]]:
    by_run: dict[str, list[int]] = defaultdict(list)
    for index, run_id in enumerate(run_ids):
        by_run[run_id].append(index)

    accel = [0.0] * len(run_ids)
    bad = [True] * len(run_ids)

    for indices in by_run.values():
        ordered = sorted(indices, key=lambda i: (time_us[i], row_indices[i], i))
        for position, index in enumerate(ordered):
            prev_index = ordered[position - 1] if position > 0 else None
            next_index = ordered[position + 1] if position + 1 < len(ordered) else None
            prev_ok = False
            next_ok = False
            if prev_index is not None:
                dt_prev = (time_us[index] - time_us[prev_index]) / 1.0e6
                prev_ok = math.isfinite(dt_prev) and 0.0 < dt_prev <= max_gap_s
            else:
                dt_prev = math.inf
            if next_index is not None:
                dt_next = (time_us[next_index] - time_us[index]) / 1.0e6
                next_ok = math.isfinite(dt_next) and 0.0 < dt_next <= max_gap_s
            else:
                dt_next = math.inf

            if prev_ok and next_ok:
                dt = (time_us[next_index] - time_us[prev_index]) / 1.0e6
                if math.isfinite(dt) and dt > 0.0:
                    accel[index] = (forward_mps[next_index] - forward_mps[prev_index]) / dt
                    bad[index] = False
                    continue
            if prev_ok:
                accel[index] = (forward_mps[index] - forward_mps[prev_index]) / dt_prev
                bad[index] = False
                continue
            if next_ok:
                accel[index] = (forward_mps[next_index] - forward_mps[index]) / dt_next
                bad[index] = False
                continue

            accel[index] = 0.0
            bad[index] = True

    return accel, bad


def _cell_ok(indices: Sequence[int], run_ids: Sequence[str], min_rows: int, min_runs: int) -> bool:
    return len(indices) >= min_rows and len({run_ids[index] for index in indices}) >= min_runs


def _quality_weights(
    yaw_rate: Sequence[float],
    vbar_yaw: Sequence[float],
    gyro_spike: Sequence[float],
    saturation: Sequence[float],
    limiter: Sequence[float],
    bad_af_gap: Sequence[bool],
    config: QualityConfig,
) -> list[float]:
    out: list[float] = []
    for index in range(len(yaw_rate)):
        quality = 1.0
        if gyro_spike[index] > 0.0:
            quality *= config.gyro_spike_multiplier
        if saturation[index] > 0.0:
            quality *= config.saturation_multiplier
        if config.use_limiter_penalty:
            quality *= 1.0 / (1.0 + config.limiter_gain * max(limiter[index], 0.0))
        if (
            config.use_low_yaw_no_motion_penalty
            and abs(yaw_rate[index]) < config.low_yaw_rate_radps
            and abs(vbar_yaw[index]) < config.low_yaw_motion_mps
        ):
            quality *= config.low_yaw_multiplier
        if bad_af_gap[index]:
            quality *= config.bad_af_gap_multiplier
        out.append(max(config.quality_floor, min(1.0, quality)))
    return out


def compute_regime_weights(
    columns: Mapping[str, Sequence[Any]],
    config: RegimeWeightConfig | None = None,
    eligible_mask: Sequence[Any] | None = None,
) -> RegimeWeightResult:
    """Return 4D regime-balanced row weights for the provided rows.

    Weighting order:
    1. split authority chooses which rows can contribute;
    2. each non-empty effective 4D cell gets equal priority-weighted mass
       within that split;
    3. each run inside that cell gets equal mass;
    4. rows inside the run/cell divide that mass by quality weights.
    """
    cfg = config or RegimeWeightConfig()
    split_weights = dict(DEFAULT_FIT_SPLIT_WEIGHTS if cfg.split_weights is None else cfg.split_weights)

    n = max((len(values) for values in columns.values()), default=0)
    run_ids = _string_column(columns, "run_id", n)
    splits = _string_column(columns, "dataset_split", n)
    row_indices = _float_column(columns, "row_index", n)
    time_us = _float_column(columns, "time_us", n)
    forward = _float_column(columns, "forward_velocity_mps", n)
    yaw_rate = _float_column(columns, "yaw_rate_radps", n)
    yaw_accel = _float_column(columns, "measured_yaw_accel_radps2", n)
    vbar_yaw = _float_column(columns, "vbar_yaw_mps", n)
    limiter = _float_column(columns, "max_force_limiter_activity", n)
    if "max_force_limiter_activity" not in columns:
        limiter = _float_column(columns, "limiter_active", n)
    saturation = _float_column(columns, "hardware_saturation_evidence", n)
    gyro_spike = _float_column(columns, "gyro_derivative_spike", n)
    eligible = _bool_mask(eligible_mask, n)

    if "derived_forward_accel_mps2" in columns and "bad_forward_accel_gap" in columns:
        forward_accel = _float_column(columns, "derived_forward_accel_mps2", n)
        bad_af_gap = [finite_float(value, 1.0) > 0.0 for value in columns["bad_forward_accel_gap"]]
    else:
        forward_accel, bad_af_gap = _derive_forward_accel(
            run_ids, row_indices, time_us, forward, cfg.bad_af_gap_s
        )
    quality = _quality_weights(
        yaw_rate, vbar_yaw, gyro_spike, saturation, limiter, bad_af_gap, cfg.quality
    )

    vf_bins = [_bin_label(value, FORWARD_VELOCITY_BINS_MPS) for value in forward]
    yaw_bins = [_bin_label(value, YAW_RATE_BINS_RADPS) for value in yaw_rate]
    af_bins = [_bin_label(value, FORWARD_ACCEL_BINS_MPS2) for value in forward_accel]
    yaw_accel_bins = [_bin_label(value, YAW_ACCEL_BINS_RADPS2) for value in yaw_accel]
    full_keys = list(zip(vf_bins, yaw_bins, af_bins, yaw_accel_bins))

    weights = [0.0] * n
    effective_keys: list[tuple[str, str, str, str]] = [("", "", "", "") for _ in range(n)]
    merge_levels = ["excluded"] * n
    cell_rows: list[dict[str, Any]] = []

    split_to_indices: dict[str, list[int]] = defaultdict(list)
    for index, split in enumerate(splits):
        if eligible[index] and split_weights.get(split, 0.0) > 0.0:
            split_to_indices[split].append(index)

    for split, split_indices in sorted(split_to_indices.items()):
        full_groups: dict[tuple[str, str, str, str], list[int]] = defaultdict(list)
        drop_af_groups: dict[tuple[str, str, str, str], list[int]] = defaultdict(list)
        vf_yaw_groups: dict[tuple[str, str, str, str], list[int]] = defaultdict(list)
        for index in split_indices:
            full = full_keys[index]
            drop_af = (full[0], full[1], "*", full[3])
            vf_yaw = (full[0], full[1], "*", "*")
            full_groups[full].append(index)
            drop_af_groups[drop_af].append(index)
            vf_yaw_groups[vf_yaw].append(index)

        effective_groups: dict[tuple[str, str, str, str], list[int]] = defaultdict(list)
        group_level: dict[tuple[str, str, str, str], str] = {}
        for index in split_indices:
            full = full_keys[index]
            drop_af = (full[0], full[1], "*", full[3])
            vf_yaw = (full[0], full[1], "*", "*")
            if _cell_ok(full_groups[full], run_ids, cfg.sparse_min_rows, cfg.sparse_min_runs):
                key = full
                level = "full_4d"
            elif _cell_ok(drop_af_groups[drop_af], run_ids, cfg.sparse_min_rows, cfg.sparse_min_runs):
                key = drop_af
                level = "merged_drop_af"
            elif _cell_ok(vf_yaw_groups[vf_yaw], run_ids, cfg.sparse_min_rows, cfg.sparse_min_runs):
                key = vf_yaw
                level = "merged_drop_af_yawaccel"
            else:
                key = vf_yaw
                level = "sparse_vf_yaw_capped"
            effective_keys[index] = key
            merge_levels[index] = level
            effective_groups[key].append(index)
            group_level.setdefault(key, level)

        cell_priority: dict[tuple[str, str, str, str], float] = {}
        for key, indices in effective_groups.items():
            calibration_rows = sum(
                1
                for index in indices
                if abs(forward[index]) < 0.15 and abs(yaw_rate[index]) >= 0.1
            )
            is_calibration = calibration_rows >= max(1, len(indices) // 2)
            priority = cfg.calibration_priority if is_calibration else 1.0
            if group_level[key] == "sparse_vf_yaw_capped":
                priority = min(priority, cfg.sparse_cell_priority_cap)
            cell_priority[key] = priority

        split_weight = split_weights.get(split, 0.0)
        split_total = split_weight * len(split_indices)
        priority_total = sum(cell_priority.values())
        if priority_total <= 0.0:
            continue

        for key, indices in sorted(effective_groups.items()):
            priority = cell_priority[key]
            cell_budget = split_total * priority / priority_total
            by_run: dict[str, list[int]] = defaultdict(list)
            for index in indices:
                by_run[run_ids[index]].append(index)
            run_budget = cell_budget / max(len(by_run), 1)
            for run_index_list in by_run.values():
                q_total = sum(quality[index] for index in run_index_list)
                if q_total <= 0.0:
                    equal = run_budget / len(run_index_list)
                    for index in run_index_list:
                        weights[index] = equal
                else:
                    for index in run_index_list:
                        weights[index] = run_budget * quality[index] / q_total

            raw_rows = len(indices)
            raw_weight = sum(quality[index] for index in indices)
            final_weight = sum(weights[index] for index in indices)
            calibration_rows = sum(
                1
                for index in indices
                if abs(forward[index]) < 0.15 and abs(yaw_rate[index]) >= 0.1
            )
            cell_rows.append(
                {
                    "dataset_split": split,
                    "vf_bin": key[0],
                    "yaw_rate_bin": key[1],
                    "af_bin": key[2],
                    "yaw_accel_bin": key[3],
                    "merge_level": group_level[key],
                    "rows": raw_rows,
                    "run_count": len(by_run),
                    "raw_quality_sum": raw_weight,
                    "final_weight_sum": final_weight,
                    "priority": priority,
                    "calibration_row_share": calibration_rows / raw_rows if raw_rows else 0.0,
                }
            )

    marginal_rows = _make_marginals(splits, vf_bins, yaw_bins, af_bins, yaw_accel_bins, weights, eligible)
    summary = _make_summary(
        splits=splits,
        eligible=eligible,
        split_weights=split_weights,
        weights=weights,
        quality=quality,
        bad_af_gap=bad_af_gap,
        forward=forward,
        yaw_rate=yaw_rate,
        cell_rows=cell_rows,
        config=cfg,
    )

    return RegimeWeightResult(
        weights=weights,
        forward_accel_mps2=forward_accel,
        bad_af_gap=bad_af_gap,
        quality=quality,
        full_cell_keys=full_keys,
        effective_cell_keys=effective_keys,
        merge_levels=merge_levels,
        cell_rows=cell_rows,
        marginal_rows=marginal_rows,
        summary=summary,
    )


def compute_regime_weights_for_frame(
    frame: Any,
    config: RegimeWeightConfig | None = None,
    eligible_mask: Sequence[Any] | None = None,
) -> RegimeWeightResult:
    return compute_regime_weights(dataframe_columns(frame), config=config, eligible_mask=eligible_mask)


def latest_inclusive_fit_mask(columns: Mapping[str, Sequence[Any]]) -> list[bool]:
    n = max((len(values) for values in columns.values()), default=0)
    run_ids = _string_column(columns, "run_id", n)
    splits = _string_column(columns, "dataset_split", n)
    latest = set(LATEST_FIT_RUNS)
    return [split == PRIMARY_SPLIT or run_id in latest for split, run_id in zip(splits, run_ids)]


def latest_inclusive_fit_mask_for_frame(frame: Any) -> list[bool]:
    return latest_inclusive_fit_mask(dataframe_columns(frame))


def add_forward_accel_columns_to_frame(frame: Any, max_gap_s: float = 0.25) -> Any:
    """Return a copy with derived Af columns that survive later row sampling."""
    columns = dataframe_columns(frame)
    n = max((len(values) for values in columns.values()), default=0)
    accel, bad = _derive_forward_accel(
        _string_column(columns, "run_id", n),
        _float_column(columns, "row_index", n),
        _float_column(columns, "time_us", n),
        _float_column(columns, "forward_velocity_mps", n),
        max_gap_s,
    )
    out = frame.copy()
    out["derived_forward_accel_mps2"] = accel
    out["bad_forward_accel_gap"] = [1.0 if value else 0.0 for value in bad]
    return out


def _make_marginals(
    splits: Sequence[str],
    vf_bins: Sequence[str],
    yaw_bins: Sequence[str],
    af_bins: Sequence[str],
    yaw_accel_bins: Sequence[str],
    weights: Sequence[float],
    eligible: Sequence[bool],
) -> list[dict[str, Any]]:
    axis_values = {
        "Vf": vf_bins,
        "YawRate": yaw_bins,
        "Af": af_bins,
        "YawAccel": yaw_accel_bins,
    }
    rows: list[dict[str, Any]] = []
    for split in sorted(set(splits)):
        split_indices = [i for i, value in enumerate(splits) if value == split and eligible[i] and weights[i] > 0.0]
        if not split_indices:
            continue
        total_rows = len(split_indices)
        total_weight = sum(weights[i] for i in split_indices)
        for axis, values in axis_values.items():
            grouped: dict[str, list[int]] = defaultdict(list)
            for index in split_indices:
                grouped[values[index]].append(index)
            for label, indices in sorted(grouped.items()):
                w_sum = sum(weights[index] for index in indices)
                rows.append(
                    {
                        "dataset_split": split,
                        "axis": axis,
                        "bin": label,
                        "rows": len(indices),
                        "raw_row_share": len(indices) / total_rows if total_rows else 0.0,
                        "final_weight_sum": w_sum,
                        "final_weight_share": w_sum / total_weight if total_weight > 0.0 else 0.0,
                    }
                )
    return rows


def _make_summary(
    splits: Sequence[str],
    eligible: Sequence[bool],
    split_weights: Mapping[str, float],
    weights: Sequence[float],
    quality: Sequence[float],
    bad_af_gap: Sequence[bool],
    forward: Sequence[float],
    yaw_rate: Sequence[float],
    cell_rows: Sequence[Mapping[str, Any]],
    config: RegimeWeightConfig,
) -> dict[str, Any]:
    positive = [i for i, w in enumerate(weights) if eligible[i] and w > 0.0]
    total_weight = sum(weights[i] for i in positive)
    calibration = [i for i in positive if abs(forward[i]) < 0.15 and abs(yaw_rate[i]) >= 0.1]
    bad_af = [i for i in positive if bad_af_gap[i]]

    by_split: dict[str, dict[str, Any]] = {}
    for split in sorted(set(splits)):
        indices = [i for i in positive if splits[i] == split]
        if not indices:
            continue
        split_weight_total = sum(weights[i] for i in indices)
        split_cal = [i for i in indices if abs(forward[i]) < 0.15 and abs(yaw_rate[i]) >= 0.1]
        by_split[split] = {
            "rows": len(indices),
            "split_authority_weight": split_weights.get(split, 0.0),
            "final_weight_sum": split_weight_total,
            "low_vf_nonzero_yaw_rows": len(split_cal),
            "low_vf_nonzero_yaw_raw_row_share": len(split_cal) / len(indices) if indices else 0.0,
            "low_vf_nonzero_yaw_final_weight_share": (
                sum(weights[i] for i in split_cal) / split_weight_total if split_weight_total > 0.0 else 0.0
            ),
            "bad_af_gap_rows": sum(1 for i in indices if bad_af_gap[i]),
            "bad_af_gap_final_weight_share": (
                sum(weights[i] for i in indices if bad_af_gap[i]) / split_weight_total
                if split_weight_total > 0.0
                else 0.0
            ),
        }

    sorted_cells = sorted(
        cell_rows,
        key=lambda row: float(row.get("final_weight_sum", 0.0)),
        reverse=True,
    )
    sorted_raw_cells = sorted(
        cell_rows,
        key=lambda row: int(row.get("rows", 0)),
        reverse=True,
    )

    def json_edges(edges: Sequence[float]) -> list[str | float]:
        out: list[str | float] = []
        for value in edges:
            if value == -math.inf:
                out.append("-inf")
            elif value == math.inf:
                out.append("inf")
            else:
                out.append(value)
        return out

    return {
        "weighting": "4d_operating_regime_cells",
        "axes": ["Vf", "YawRate", "Af", "YawAccel"],
        "forward_velocity_bins_mps": json_edges(FORWARD_VELOCITY_BINS_MPS),
        "yaw_rate_bins_radps": json_edges(YAW_RATE_BINS_RADPS),
        "forward_accel_bins_mps2": json_edges(FORWARD_ACCEL_BINS_MPS2),
        "yaw_accel_bins_radps2": json_edges(YAW_ACCEL_BINS_RADPS2),
        "split_weights": dict(split_weights),
        "latest_fit_runs": list(LATEST_FIT_RUNS),
        "latest_fit_policy": "migrated fit callers use primary rows plus these latest runs; non-primary latest rows use downweighted split authority",
        "sparse_min_rows": config.sparse_min_rows,
        "sparse_min_runs": config.sparse_min_runs,
        "bad_af_gap_s": config.bad_af_gap_s,
        "bad_af_gap_multiplier": config.quality.bad_af_gap_multiplier,
        "calibration_definition": "abs(Vf)<0.15 and abs(YawRate)>=0.1",
        "calibration_priority": config.calibration_priority,
        "calibration_priority_policy": "fixed 1.35x cell priority; diagnostics report the resulting share",
        "no_motion_policy": "low-yaw/no-motion rows are valid calibration evidence and are not downweighted unless a caller explicitly enables that penalty",
        "eligible_positive_rows": len(positive),
        "final_weight_sum": total_weight,
        "low_vf_nonzero_yaw_rows": len(calibration),
        "low_vf_nonzero_yaw_final_weight_share": (
            sum(weights[i] for i in calibration) / total_weight if total_weight > 0.0 else 0.0
        ),
        "bad_af_gap_rows": len(bad_af),
        "bad_af_gap_final_weight_share": (
            sum(weights[i] for i in bad_af) / total_weight if total_weight > 0.0 else 0.0
        ),
        "mean_quality_positive": (
            sum(quality[i] for i in positive) / len(positive) if positive else 0.0
        ),
        "by_split": by_split,
        "top_cells_after_weighting": sorted_cells[:10],
        "top_cells_before_weighting_by_rows": sorted_raw_cells[:10],
    }


def _write_csv(path: Path, rows: Sequence[Mapping[str, Any]]) -> None:
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fieldnames: list[str] = []
    seen: set[str] = set()
    for row in rows:
        for key in row.keys():
            if key not in seen:
                fieldnames.append(key)
                seen.add(key)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def write_regime_diagnostics(output_dir: Path, prefix: str, result: RegimeWeightResult) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    clean_prefix = prefix or ""
    (output_dir / f"{clean_prefix}regime_weighting_summary.json").write_text(
        json.dumps(result.summary, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    _write_csv(output_dir / f"{clean_prefix}regime_weighting_cells.csv", result.cell_rows)
    _write_csv(output_dir / f"{clean_prefix}regime_weighting_marginals.csv", result.marginal_rows)


def _read_csv_columns(path: Path) -> dict[str, list[str]]:
    columns: dict[str, list[str]] = {}
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            if row.get("dataset_split") == "excluded_or_unclassified":
                continue
            for key, value in row.items():
                columns.setdefault(key, []).append(value)
    return columns


def main() -> int:
    root = Path(__file__).resolve().parents[2]
    default_input = (
        root
        / "codex_analysis"
        / "contact_continuum_yaw_identification"
        / "ablation"
        / "phase_classified_feature_sample.csv"
    )
    parser = argparse.ArgumentParser(description="Write 4D regime weighting diagnostics.")
    parser.add_argument("--input", type=Path, default=default_input)
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=Path(__file__).resolve().parent / "regime_weighting_latest_fit_diagnostics",
    )
    parser.add_argument("--prefix", default="")
    args = parser.parse_args()

    columns = _read_csv_columns(args.input)
    result = compute_regime_weights(
        columns,
        RegimeWeightConfig(
            split_weights=LATEST_INCLUSIVE_FIT_SPLIT_WEIGHTS,
            quality=QualityConfig(
                gyro_spike_multiplier=0.10,
                saturation_multiplier=0.35,
                use_limiter_penalty=False,
                use_low_yaw_no_motion_penalty=False,
            ),
        ),
        eligible_mask=latest_inclusive_fit_mask(columns),
    )
    write_regime_diagnostics(args.out_dir, args.prefix, result)
    print(
        "rows={rows} weight_sum={weight_sum:.6f} calibration_share={cal:.6f}".format(
            rows=result.summary["eligible_positive_rows"],
            weight_sum=result.summary["final_weight_sum"],
            cal=result.summary["low_vf_nonzero_yaw_final_weight_share"],
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
