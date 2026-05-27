#!/usr/bin/env python3
"""Shared operating-range RMSE summaries for yaw/traction fit scripts.

Analysis-only utility. These ranges are named around the robot operating
envelope, not around the current dataset's row distribution.
"""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence


LATEST_LOG_RUNS = frozenset({"2026-05-04_20-35-47", "2026-05-04_16-57-53"})


@dataclass(frozen=True)
class CommonRangeDefinition:
    name: str
    definition: str


COMMON_RANGE_DEFINITIONS = (
    CommonRangeDefinition(
        "calibration_low_vf_nonzero_yaw",
        "abs(Vf)<0.15 and abs(YawRate)>=0.1",
    ),
    CommonRangeDefinition(
        "in_place_scrub",
        "abs(Vf)<0.05 and abs(YawRate)>=0.2",
    ),
    CommonRangeDefinition(
        "slow_forward_turn",
        "0.15<=abs(Vf)<0.70 and abs(YawRate)>=0.1",
    ),
    CommonRangeDefinition(
        "pre_design_turn_speed",
        "0.70<=abs(Vf)<0.95 and abs(YawRate)>=0.1",
    ),
    CommonRangeDefinition(
        "design_turn_speed_and_up",
        "abs(Vf)>=0.95 and abs(YawRate)>=0.1",
    ),
    CommonRangeDefinition(
        "fast_forward",
        "abs(Vf)>=1.50",
    ),
    CommonRangeDefinition(
        "straightish_forward",
        "abs(YawRate)<0.05 and abs(Vf)>=0.05",
    ),
    CommonRangeDefinition(
        "limiter_active",
        "max_force_limiter_activity>0 or limiter_active>0",
    ),
    CommonRangeDefinition(
        "hardware_saturation_evidence",
        "hardware_saturation_evidence>0",
    ),
    CommonRangeDefinition(
        "may4_latest_logs",
        "run_id in {2026-05-04_20-35-47, 2026-05-04_16-57-53}",
    ),
)


COMMON_RANGE_REPORT_COLUMNS = [
    "range_name",
    "count",
    "baseline_rmse_nm",
    "candidate_rmse_nm",
    "candidate_mae_nm",
    "candidate_median_abs_nm",
]


def frame_columns(frame_or_columns: Any) -> dict[str, list[Any]]:
    if isinstance(frame_or_columns, Mapping):
        return {str(key): list(value) for key, value in frame_or_columns.items()}
    return {str(column): list(frame_or_columns[column]) for column in frame_or_columns.columns}


def finite_float(value: Any, default: float = 0.0) -> float:
    try:
        out = float(value)
        return out if math.isfinite(out) else default
    except (TypeError, ValueError):
        return default


def _string_column(columns: Mapping[str, Sequence[Any]], name: str, n: int) -> list[str]:
    values = columns.get(name)
    if values is None:
        return [""] * n
    return [str(value) for value in values]


def _float_column(columns: Mapping[str, Sequence[Any]], name: str, n: int) -> list[float]:
    values = columns.get(name)
    if values is None:
        return [0.0] * n
    return [finite_float(value) for value in values]


def _candidate_indices(
    definition: CommonRangeDefinition,
    run_ids: Sequence[str],
    vf: Sequence[float],
    yaw_rate: Sequence[float],
    limiter: Sequence[float],
    saturation: Sequence[float],
) -> list[int]:
    indices: list[int] = []
    for index in range(len(vf)):
        abs_vf = abs(vf[index])
        abs_yaw = abs(yaw_rate[index])
        keep = False
        if definition.name == "calibration_low_vf_nonzero_yaw":
            keep = abs_vf < 0.15 and abs_yaw >= 0.1
        elif definition.name == "in_place_scrub":
            keep = abs_vf < 0.05 and abs_yaw >= 0.2
        elif definition.name == "slow_forward_turn":
            keep = 0.15 <= abs_vf < 0.70 and abs_yaw >= 0.1
        elif definition.name == "pre_design_turn_speed":
            keep = 0.70 <= abs_vf < 0.95 and abs_yaw >= 0.1
        elif definition.name == "design_turn_speed_and_up":
            keep = abs_vf >= 0.95 and abs_yaw >= 0.1
        elif definition.name == "fast_forward":
            keep = abs_vf >= 1.50
        elif definition.name == "straightish_forward":
            keep = abs_yaw < 0.05 and abs_vf >= 0.05
        elif definition.name == "limiter_active":
            keep = limiter[index] > 0.0
        elif definition.name == "hardware_saturation_evidence":
            keep = saturation[index] > 0.0
        elif definition.name == "may4_latest_logs":
            keep = run_ids[index] in LATEST_LOG_RUNS
        if keep:
            indices.append(index)
    return indices


def _rmse(values: Sequence[float]) -> float:
    return math.sqrt(sum(value * value for value in values) / len(values)) if values else math.nan


def _mae(values: Sequence[float]) -> float:
    return sum(abs(value) for value in values) / len(values) if values else math.nan


def _median_abs(values: Sequence[float]) -> float:
    if not values:
        return math.nan
    ordered = sorted(abs(value) for value in values)
    mid = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[mid]
    return 0.5 * (ordered[mid - 1] + ordered[mid])


def common_range_metric_rows(
    frame_or_columns: Any,
    baseline_error_nm: Sequence[Any],
    candidate_error_nm: Sequence[Any],
    candidate_label: str,
) -> list[dict[str, object]]:
    columns = frame_columns(frame_or_columns)
    n = min(
        max((len(values) for values in columns.values()), default=0),
        len(baseline_error_nm),
        len(candidate_error_nm),
    )
    run_ids = _string_column(columns, "run_id", n)
    vf = _float_column(columns, "forward_velocity_mps", n)
    yaw_rate = _float_column(columns, "yaw_rate_radps", n)
    limiter_activity = _float_column(columns, "max_force_limiter_activity", n)
    limiter_active = _float_column(columns, "limiter_active", n)
    limiter = [max(limiter_activity[index], limiter_active[index]) for index in range(n)]
    saturation = _float_column(columns, "hardware_saturation_evidence", n)
    baseline = [finite_float(value, math.nan) for value in baseline_error_nm[:n]]
    candidate = [finite_float(value, math.nan) for value in candidate_error_nm[:n]]

    rows: list[dict[str, object]] = []
    for definition in COMMON_RANGE_DEFINITIONS:
        indices = _candidate_indices(definition, run_ids, vf, yaw_rate, limiter, saturation)
        finite_pairs = [
            index
            for index in indices
            if math.isfinite(baseline[index]) and math.isfinite(candidate[index])
        ]
        baseline_values = [baseline[index] for index in finite_pairs]
        candidate_values = [candidate[index] for index in finite_pairs]
        base_rmse = _rmse(baseline_values)
        candidate_rmse = _rmse(candidate_values)
        rows.append(
            {
                "range_name": definition.name,
                "range_definition": definition.definition,
                "candidate_label": candidate_label,
                "count": len(indices),
                "finite_count": len(finite_pairs),
                "run_count": len({run_ids[index] for index in indices}),
                "baseline_rmse_nm": base_rmse,
                "candidate_rmse_nm": candidate_rmse,
                "baseline_mae_nm": _mae(baseline_values),
                "candidate_mae_nm": _mae(candidate_values),
                "baseline_median_abs_nm": _median_abs(baseline_values),
                "candidate_median_abs_nm": _median_abs(candidate_values),
                "rmse_improvement_fraction": (
                    (base_rmse - candidate_rmse) / base_rmse
                    if math.isfinite(base_rmse) and base_rmse > 0.0 and math.isfinite(candidate_rmse)
                    else math.nan
                ),
            }
        )
    return rows


def _csv_value(value: object) -> object:
    if isinstance(value, float) and not math.isfinite(value):
        return ""
    return value


def write_common_range_metrics(
    path: Path,
    frame_or_columns: Any,
    baseline_error_nm: Sequence[Any],
    candidate_error_nm: Sequence[Any],
    candidate_label: str,
) -> list[dict[str, object]]:
    rows = common_range_metric_rows(
        frame_or_columns,
        baseline_error_nm,
        candidate_error_nm,
        candidate_label,
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        fieldnames = list(rows[0].keys()) if rows else []
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: _csv_value(value) for key, value in row.items()})
    return rows
