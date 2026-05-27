#!/usr/bin/env python3
"""Variant D residual surface baselines for yaw model fitting.

Analysis-only script. It reads the shared feature sample and prior yaw-torque
validation bins, fits residual surfaces from fit-authoritative rows, and writes
all artifacts beside this script.
"""

from __future__ import annotations

import csv
import math
from collections import defaultdict
from pathlib import Path
from typing import Callable


ROOT = Path(__file__).resolve().parents[3]
OUT = ROOT / "codex_analysis" / "yaw_model_variant_fits" / "residual_surface"
FEATURE_SAMPLE = (
    ROOT
    / "codex_analysis"
    / "contact_continuum_yaw_identification"
    / "ablation"
    / "phase_classified_feature_sample.csv"
)
VALIDATION = ROOT / "codex_analysis" / "yaw_torque_expanded_validation"
EXPANDED_BINS = VALIDATION / "nonzero_vf_torque_bins.csv"
EXPANDED_RMSE = VALIDATION / "rmse_leave_one_run_out.csv"

SELECTED_RUNS = [
    "2026-05-04_20-35-47",
    "2026-05-04_16-57-53",
    "2026-04-22_12-10-34",
    "2026-04-22_01-06-32",
    "2026-04-21_05-32-06",
    "2026-04-21_00-16-10",
    "2026-04-20_12-10-58",
    "2026-04-20_08-38-39",
    "diag003",
]

TRAIN_SPLIT = "primary_open_floor_fit_authoritative"
PHASES = ["all", "entry", "plateau", "exit"]


def f(row: dict[str, object], key: str, default: float = 0.0) -> float:
    try:
        value = row.get(key, "")
        if value == "":
            return default
        x = float(value)
        return x if math.isfinite(x) else default
    except (TypeError, ValueError):
        return default


def sign(x: float, eps: float = 1.0e-9) -> float:
    if x > eps:
        return 1.0
    if x < -eps:
        return -1.0
    return 0.0


def finite(x: float) -> bool:
    return math.isfinite(x)


def q(values: list[float], p: float) -> float:
    clean = sorted(x for x in values if finite(x))
    if not clean:
        return 0.0
    if len(clean) == 1:
        return clean[0]
    pos = (len(clean) - 1) * p
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return clean[lo]
    frac = pos - lo
    return clean[lo] * (1.0 - frac) + clean[hi] * frac


def median(values: list[float]) -> float:
    return q(values, 0.5)


def mean(values: list[float]) -> float:
    clean = [x for x in values if finite(x)]
    return sum(clean) / len(clean) if clean else 0.0


def rmse(values: list[float]) -> float:
    clean = [x for x in values if finite(x)]
    return math.sqrt(sum(x * x for x in clean) / len(clean)) if clean else 0.0


def mae(values: list[float]) -> float:
    clean = [abs(x) for x in values if finite(x)]
    return sum(clean) / len(clean) if clean else 0.0


def median_abs(values: list[float]) -> float:
    return median([abs(x) for x in values if finite(x)])


def round_to_step(value: float, step: float) -> float:
    return round(round(value / step) * step, 10)


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as fh:
        return list(csv.DictReader(fh))


def write_csv(path: Path, rows: list[dict[str, object]], fields: list[str] | None = None) -> None:
    if fields is None:
        fields = []
        for row in rows:
            for key in row:
                if key not in fields:
                    fields.append(key)
    with path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def load_rows() -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for raw in read_csv(FEATURE_SAMPLE):
        residual = f(raw, "residual_additive_yaw_torque_nm", float("nan"))
        if not finite(residual):
            continue
        row: dict[str, object] = dict(raw)
        yaw = f(raw, "yaw_rate_radps")
        vf = f(raw, "forward_velocity_mps")
        row["target_nm"] = residual
        row["abs_target_nm"] = abs(residual)
        row["abs_vf_mps"] = abs(vf)
        row["abs_yaw_rate_radps"] = abs(yaw)
        row["sign_yaw_rate"] = sign(yaw, 1.0e-6)
        row["forward_bin_0p1_mps"] = round_to_step(vf, 0.10)
        row["yaw_bin_0p5_radps"] = round_to_step(yaw, 0.50)
        rows.append(row)
    return rows


def load_expanded_bin_lookup() -> dict[tuple[float, float], float]:
    lookup: dict[tuple[float, float], float] = {}
    if not EXPANDED_BINS.exists():
        return lookup
    for row in read_csv(EXPANDED_BINS):
        key = (
            round(float(row["forward_velocity_bin_mps"]), 10),
            round(float(row["yaw_rate_bin_radps"]), 10),
        )
        lookup[key] = float(row["median_residual_additive_nm"])
    return lookup


def metric_row(
    model_name: str,
    group_name: str,
    rows: list[dict[str, object]],
    predictions: dict[int, float],
) -> dict[str, object]:
    before = [f(row, "target_nm") for row in rows]
    after: list[float] = []
    used = 0
    for row in rows:
        pred = predictions.get(id(row), 0.0)
        if abs(pred) > 1.0e-12:
            used += 1
        after.append(f(row, "target_nm") - pred)
    return {
        "model": model_name,
        "group": group_name,
        "rows": len(rows),
        "runs": len({str(row["run_id"]) for row in rows}),
        "coverage_fraction": used / len(rows) if rows else 0.0,
        "baseline_rmse_nm": rmse(before),
        "corrected_rmse_nm": rmse(after),
        "rmse_delta_nm": rmse(after) - rmse(before),
        "rmse_improvement_fraction": 1.0 - (rmse(after) / max(rmse(before), 1.0e-12)),
        "baseline_mae_nm": mae(before),
        "corrected_mae_nm": mae(after),
        "mae_delta_nm": mae(after) - mae(before),
        "baseline_median_abs_nm": median_abs(before),
        "corrected_median_abs_nm": median_abs(after),
        "median_residual_before_nm": median(before),
        "median_residual_after_nm": median(after),
    }


def selected_split_label(row: dict[str, object]) -> str:
    run_id = str(row["run_id"])
    if run_id in SELECTED_RUNS:
        return f"selected:{run_id}"
    return "not_selected"


def load_yaw_rate_reference() -> list[dict[str, object]]:
    if not EXPANDED_RMSE.exists():
        return []
    rows: list[dict[str, object]] = []
    for row in read_csv(EXPANDED_RMSE):
        run_id = row["holdout_run_id"]
        if run_id not in SELECTED_RUNS:
            continue
        current = float(row["current_rmse_radps"])
        corrected = float(row["surface_corrected_rmse_radps"])
        rows.append(
            {
                "run_id": run_id,
                "kind": row["kind"],
                "samples": int(row["samples"]),
                "signed_bin_corrected_samples": int(row["samples_with_surface_bin"]),
                "current_one_step_rmse_radps": current,
                "expanded_signed_bin_one_step_rmse_radps": corrected,
                "signed_bin_rmse_improvement_fraction": 1.0 - corrected / max(current, 1.0e-12),
            }
        )
    return rows


class ExpandedBinLookup:
    name = "expanded_signed_bin_lookup"

    def __init__(self, lookup: dict[tuple[float, float], float]):
        self.lookup = lookup

    def predict(self, row: dict[str, object]) -> float:
        key = (f(row, "forward_bin_0p1_mps"), f(row, "yaw_bin_0p5_radps"))
        return self.lookup.get(key, 0.0)


def solve_linear(a: list[list[float]], b: list[float]) -> list[float]:
    n = len(b)
    aug = [row[:] + [rhs] for row, rhs in zip(a, b)]
    for col in range(n):
        pivot = max(range(col, n), key=lambda r: abs(aug[r][col]))
        if abs(aug[pivot][col]) < 1.0e-12:
            continue
        aug[col], aug[pivot] = aug[pivot], aug[col]
        div = aug[col][col]
        for j in range(col, n + 1):
            aug[col][j] /= div
        for r in range(n):
            if r == col:
                continue
            factor = aug[r][col]
            if abs(factor) < 1.0e-18:
                continue
            for j in range(col, n + 1):
                aug[r][j] -= factor * aug[col][j]
    return [aug[i][n] for i in range(n)]


class RidgeSurface:
    def __init__(self, name: str, features: list[tuple[str, Callable[[dict[str, object]], float]]], ridge: float):
        self.name = name
        self.features = features
        self.ridge = ridge
        self.centers: list[float] = []
        self.scales: list[float] = []
        self.beta: list[float] = []

    def _feature_values(self, row: dict[str, object]) -> list[float]:
        return [fn(row) for _, fn in self.features]

    def fit(self, rows: list[dict[str, object]]) -> None:
        values_by_feature = list(zip(*(self._feature_values(row) for row in rows)))
        self.centers = [median(list(values)) for values in values_by_feature]
        self.scales = [
            max(q([abs(v - c) for v in values], 0.75), 1.0e-9)
            for values, c in zip(values_by_feature, self.centers)
        ]
        cols = 1 + len(self.features)
        xtx = [[0.0 for _ in range(cols)] for _ in range(cols)]
        xty = [0.0 for _ in range(cols)]
        for row in rows:
            xs = [1.0]
            raw = self._feature_values(row)
            xs.extend((v - c) / s for v, c, s in zip(raw, self.centers, self.scales))
            y = f(row, "target_nm")
            for i in range(cols):
                xty[i] += xs[i] * y
                for j in range(cols):
                    xtx[i][j] += xs[i] * xs[j]
        for i in range(1, cols):
            xtx[i][i] += self.ridge * len(rows)
        self.beta = solve_linear(xtx, xty)

    def predict(self, row: dict[str, object]) -> float:
        if not self.beta:
            return 0.0
        raw = self._feature_values(row)
        value = self.beta[0]
        for coeff, v, c, s in zip(self.beta[1:], raw, self.centers, self.scales):
            value += coeff * ((v - c) / s)
        return value

    def coefficients(self) -> list[dict[str, object]]:
        rows = [{"model": self.name, "feature": "intercept", "coefficient": self.beta[0] if self.beta else 0.0}]
        for (feature, _), coeff, center, scale in zip(self.features, self.beta[1:], self.centers, self.scales):
            rows.append(
                {
                    "model": self.name,
                    "feature": feature,
                    "coefficient": coeff,
                    "center": center,
                    "scale": scale,
                }
            )
        return rows


def vf_yaw_features() -> list[tuple[str, Callable[[dict[str, object]], float]]]:
    return [
        ("forward_velocity_mps", lambda r: f(r, "forward_velocity_mps")),
        ("yaw_rate_radps", lambda r: f(r, "yaw_rate_radps")),
        ("abs_forward_velocity_mps", lambda r: f(r, "abs_vf_mps")),
        ("abs_yaw_rate_radps", lambda r: f(r, "abs_yaw_rate_radps")),
        ("vf_times_yaw", lambda r: f(r, "forward_velocity_mps") * f(r, "yaw_rate_radps")),
        ("yaw_times_abs_yaw", lambda r: f(r, "yaw_rate_radps") * f(r, "abs_yaw_rate_radps")),
        ("vf_times_abs_yaw", lambda r: f(r, "forward_velocity_mps") * f(r, "abs_yaw_rate_radps")),
        ("sign_yaw_rate", lambda r: f(r, "sign_yaw_rate")),
        ("vbar_rel_mps", lambda r: f(r, "vbar_rel_mps")),
        ("max_force_preprojection_utilization", lambda r: f(r, "max_force_preprojection_utilization")),
    ]


def contact_features() -> list[tuple[str, Callable[[dict[str, object]], float]]]:
    names = [
        "forward_velocity_mps",
        "yaw_rate_radps",
        "abs_vf_mps",
        "abs_yaw_rate_radps",
        "vbar_rel_mps",
        "vbar_lat_mps",
        "vbar_yaw_mps",
        "max_force_preprojection_utilization",
        "max_force_limiter_activity",
        "patch_yaw_velocity_basis_m2ps",
        "patch_yaw_abs_velocity_basis_m2ps",
        "patch_yaw_req_basis_nm",
        "patch_yaw_force_basis_nm",
        "front_rear_vrel_f_abs_delta_mps",
        "left_right_vrel_f_abs_delta_mps",
        "front_rear_vrel_r_abs_delta_mps",
        "left_right_vrel_r_abs_delta_mps",
        "front_rear_normal_delta_n",
        "left_right_normal_delta_n",
        "left_right_drive_force_delta_n",
    ]
    return [(name, lambda r, key=name: f(r, key)) for name in names]


class CellKernelSurface:
    VF_STEP = 0.10
    YAW_STEP = 0.50
    REL_STEP = 0.025

    def __init__(self, name: str, min_cell_count: int, sigma_vf: float, sigma_yaw: float, sigma_rel: float):
        self.name = name
        self.min_cell_count = min_cell_count
        self.sigma_vf = sigma_vf
        self.sigma_yaw = sigma_yaw
        self.sigma_rel = sigma_rel
        self.cells: list[tuple[float, float, float, float, int]] = []
        self.index: dict[tuple[int, int, int], list[tuple[float, float, float, float, int]]] = defaultdict(list)
        self.index2: dict[tuple[int, int], list[tuple[float, float, float, float, int]]] = defaultdict(list)

    @classmethod
    def _ikey(cls, vf: float, yaw: float, rel: float) -> tuple[int, int, int]:
        return (
            int(round(vf / cls.VF_STEP)),
            int(round(yaw / cls.YAW_STEP)),
            int(round(rel / cls.REL_STEP)),
        )

    def fit(self, rows: list[dict[str, object]]) -> None:
        grouped: dict[tuple[float, float, float], list[float]] = defaultdict(list)
        for row in rows:
            key = (
                round_to_step(f(row, "forward_velocity_mps"), self.VF_STEP),
                round_to_step(f(row, "yaw_rate_radps"), self.YAW_STEP),
                round_to_step(f(row, "vbar_rel_mps"), self.REL_STEP),
            )
            grouped[key].append(f(row, "target_nm"))
        self.cells = []
        self.index = defaultdict(list)
        self.index2 = defaultdict(list)
        for (vf, yaw, rel), values in grouped.items():
            if len(values) < self.min_cell_count:
                continue
            cell = (vf, yaw, rel, median(values), len(values))
            self.cells.append(cell)
            ivf, iyaw, irel = self._ikey(vf, yaw, rel)
            self.index[(ivf, iyaw, irel)].append(cell)
            self.index2[(ivf, iyaw)].append(cell)

    def predict(self, row: dict[str, object]) -> float:
        vf = f(row, "forward_velocity_mps")
        yaw = f(row, "yaw_rate_radps")
        rel = f(row, "vbar_rel_mps")
        weighted = 0.0
        total = 0.0
        center = self._ikey(
            round_to_step(vf, self.VF_STEP),
            round_to_step(yaw, self.YAW_STEP),
            round_to_step(rel, self.REL_STEP),
        )
        vf_radius = max(1, int(math.ceil((4.0 * self.sigma_vf) / self.VF_STEP)))
        yaw_radius = max(1, int(math.ceil((4.0 * self.sigma_yaw) / self.YAW_STEP)))
        for ivf in range(center[0] - vf_radius, center[0] + vf_radius + 1):
            for iyaw in range(center[1] - yaw_radius, center[1] + yaw_radius + 1):
                for cvf, cyaw, crel, value, count in self.index2.get((ivf, iyaw), []):
                    dv = (vf - cvf) / self.sigma_vf
                    dy = (yaw - cyaw) / self.sigma_yaw
                    dr = (rel - crel) / self.sigma_rel
                    d2 = (dv * dv) + (dy * dy) + (dr * dr)
                    if d2 > 16.0:
                        continue
                    w = math.exp(-0.5 * d2) * min(count, 500)
                    weighted += w * value
                    total += w
        return weighted / total if total > 1.0e-12 else 0.0

    def export_cells(self) -> list[dict[str, object]]:
        return [
            {
                "model": self.name,
                "forward_velocity_cell_mps": vf,
                "yaw_rate_cell_radps": yaw,
                "vbar_rel_cell_mps": rel,
                "median_residual_additive_nm": value,
                "count": count,
            }
            for vf, yaw, rel, value, count in self.cells
        ]


def train_rows_for_holdout(rows: list[dict[str, object]], holdout_run_id: str | None = None) -> list[dict[str, object]]:
    out = [row for row in rows if row["dataset_split"] == TRAIN_SPLIT]
    if holdout_run_id:
        out = [row for row in out if row["run_id"] != holdout_run_id]
    return out


def instantiate_models(bin_lookup: dict[tuple[float, float], float]) -> list[object]:
    return [
        ExpandedBinLookup(bin_lookup),
        RidgeSurface("vf_yaw_ridge_surface", vf_yaw_features(), ridge=2.0e-3),
        RidgeSurface("contact_feature_ridge_surface", contact_features(), ridge=3.0e-3),
        CellKernelSurface(
            "vf_yaw_vbar_kernel_cell_surface",
            min_cell_count=8,
            sigma_vf=0.16,
            sigma_yaw=0.80,
            sigma_rel=0.05,
        ),
    ]


def fit_fitted_model(model: object, rows: list[dict[str, object]]) -> object:
    if hasattr(model, "fit"):
        model.fit(rows)  # type: ignore[attr-defined]
    return model


def predict_rows(model: object, rows: list[dict[str, object]]) -> dict[int, float]:
    return {id(row): model.predict(row) for row in rows}  # type: ignore[attr-defined]


def all_group_specs(rows: list[dict[str, object]]) -> list[tuple[str, list[dict[str, object]]]]:
    specs: list[tuple[str, list[dict[str, object]]]] = []
    for split in [
        TRAIN_SPLIT,
        "open_floor_fit_downweighted",
        "open_floor_validation_only",
        "diag_validation_only",
        "aux_downweighted_validation",
    ]:
        split_rows = [row for row in rows if row["dataset_split"] == split]
        if split_rows:
            specs.append((f"split:{split}:all", split_rows))
            for phase in ["entry", "plateau", "exit"]:
                phase_rows = [row for row in split_rows if row["physics_phase"] == phase]
                if phase_rows:
                    specs.append((f"split:{split}:phase:{phase}", phase_rows))
    selected_rows = [row for row in rows if str(row["run_id"]) in SELECTED_RUNS]
    if selected_rows:
        specs.append(("selected:all", selected_rows))
    for run_id in SELECTED_RUNS:
        run_rows = [row for row in rows if row["run_id"] == run_id]
        if run_rows:
            specs.append((f"selected_run:{run_id}:all", run_rows))
            for phase in ["entry", "plateau", "exit"]:
                phase_rows = [row for row in run_rows if row["physics_phase"] == phase]
                if phase_rows:
                    specs.append((f"selected_run:{run_id}:phase:{phase}", phase_rows))
    return specs


def evaluate_full_train(rows: list[dict[str, object]], models: list[object]) -> tuple[list[dict[str, object]], list[dict[str, object]], list[dict[str, object]]]:
    train_rows = train_rows_for_holdout(rows)
    eval_specs = all_group_specs(rows)
    metric_rows: list[dict[str, object]] = []
    coeff_rows: list[dict[str, object]] = []
    cell_rows: list[dict[str, object]] = []
    for model in models:
        fit_fitted_model(model, train_rows)
        preds = predict_rows(model, rows)
        for group_name, group_rows in eval_specs:
            metric_rows.append(metric_row(model.name, group_name, group_rows, preds))  # type: ignore[attr-defined]
        if hasattr(model, "coefficients"):
            coeff_rows.extend(model.coefficients())  # type: ignore[attr-defined]
        if hasattr(model, "export_cells"):
            cell_rows.extend(model.export_cells())  # type: ignore[attr-defined]
    return metric_rows, coeff_rows, cell_rows


def evaluate_leave_selected_run_out(rows: list[dict[str, object]], bin_lookup: dict[tuple[float, float], float]) -> list[dict[str, object]]:
    output: list[dict[str, object]] = []
    for run_id in SELECTED_RUNS:
        holdout = [row for row in rows if row["run_id"] == run_id]
        if not holdout:
            output.append(
                {
                    "run_id": run_id,
                    "model": "all",
                    "status": "absent_from_primary_feature_input",
                }
            )
            continue
        train = train_rows_for_holdout(rows, run_id if any(row["dataset_split"] == TRAIN_SPLIT for row in holdout) else None)
        for model in instantiate_models(bin_lookup):
            fit_fitted_model(model, train)
            preds = predict_rows(model, holdout)
            out = metric_row(model.name, f"leave_selected_run_out:{run_id}", holdout, preds)
            out["run_id"] = run_id
            out["dataset_split"] = holdout[0]["dataset_split"]
            out["physics_phases_present"] = ";".join(sorted({str(row["physics_phase"]) for row in holdout}))
            out["training_rows"] = len(train)
            out["status"] = "held_out_if_fit_authoritative_else_train_primary_only"
            output.append(out)
    return output


def tune_kernel(rows: list[dict[str, object]]) -> list[dict[str, object]]:
    train_runs = sorted({str(row["run_id"]) for row in rows if row["dataset_split"] == TRAIN_SPLIT})
    candidates = [
        (5, 0.12, 0.60, 0.035),
        (8, 0.16, 0.80, 0.050),
        (12, 0.20, 1.10, 0.070),
    ]
    out: list[dict[str, object]] = []
    for min_count, sigma_vf, sigma_yaw, sigma_rel in candidates:
        errors: list[float] = []
        covered = 0
        total = 0
        for holdout_run in train_runs:
            train = train_rows_for_holdout(rows, holdout_run)
            holdout = [row for row in rows if row["dataset_split"] == TRAIN_SPLIT and row["run_id"] == holdout_run]
            model = CellKernelSurface("kernel_tune", min_count, sigma_vf, sigma_yaw, sigma_rel)
            model.fit(train)
            for row in holdout:
                pred = model.predict(row)
                if abs(pred) > 1.0e-12:
                    covered += 1
                errors.append(f(row, "target_nm") - pred)
                total += 1
        out.append(
            {
                "model": "vf_yaw_vbar_kernel_cell_surface",
                "min_cell_count": min_count,
                "sigma_vf": sigma_vf,
                "sigma_yaw": sigma_yaw,
                "sigma_vbar_rel": sigma_rel,
                "loro_primary_rows": total,
                "coverage_fraction": covered / total if total else 0.0,
                "loro_corrected_rmse_nm": rmse(errors),
                "loro_corrected_mae_nm": mae(errors),
                "loro_corrected_median_abs_nm": median_abs(errors),
            }
        )
    return out


def coverage_holes(rows: list[dict[str, object]], bin_lookup: dict[tuple[float, float], float]) -> list[dict[str, object]]:
    groups: dict[str, list[dict[str, object]]] = {
        "selected_all": [row for row in rows if str(row["run_id"]) in SELECTED_RUNS],
        "high_forward_abs_ge_0p6": [row for row in rows if abs(f(row, "forward_velocity_mps")) >= 0.6],
        "low_speed_abs_vf_lt_0p05_abs_yaw_ge_0p25": [
            row for row in rows if abs(f(row, "forward_velocity_mps")) < 0.05 and abs(f(row, "yaw_rate_radps")) >= 0.25
        ],
        "straightish_abs_yaw_lt_0p25": [row for row in rows if abs(f(row, "yaw_rate_radps")) < 0.25],
    }
    out: list[dict[str, object]] = []
    for name, group in groups.items():
        if not group:
            out.append({"region": name, "rows": 0})
            continue
        expanded_hits = 0
        run_counts: dict[str, int] = defaultdict(int)
        for row in group:
            key = (f(row, "forward_bin_0p1_mps"), f(row, "yaw_bin_0p5_radps"))
            if key in bin_lookup:
                expanded_hits += 1
            run_counts[str(row["run_id"])] += 1
        top_run, top_count = max(run_counts.items(), key=lambda item: item[1])
        out.append(
            {
                "region": name,
                "rows": len(group),
                "runs": len(run_counts),
                "expanded_bin_lookup_coverage_fraction": expanded_hits / len(group),
                "top_run": top_run,
                "top_run_fraction": top_count / len(group),
                "baseline_rmse_nm": rmse([f(row, "target_nm") for row in group]),
                "baseline_median_abs_nm": median_abs([f(row, "target_nm") for row in group]),
            }
        )
    for run_id in SELECTED_RUNS:
        group = [row for row in rows if row["run_id"] == run_id]
        if not group:
            out.append({"region": f"selected_run:{run_id}", "rows": 0, "status": "absent"})
            continue
        expanded_hits = sum(
            1
            for row in group
            if (f(row, "forward_bin_0p1_mps"), f(row, "yaw_bin_0p5_radps")) in bin_lookup
        )
        out.append(
            {
                "region": f"selected_run:{run_id}",
                "rows": len(group),
                "runs": 1,
                "dataset_split": group[0]["dataset_split"],
                "expanded_bin_lookup_coverage_fraction": expanded_hits / len(group),
                "top_run": run_id,
                "top_run_fraction": 1.0,
                "max_abs_forward_mps": max(abs(f(row, "forward_velocity_mps")) for row in group),
                "max_abs_yaw_rate_radps": max(abs(f(row, "yaw_rate_radps")) for row in group),
                "baseline_rmse_nm": rmse([f(row, "target_nm") for row in group]),
                "baseline_median_abs_nm": median_abs([f(row, "target_nm") for row in group]),
            }
        )
    return out


def best_model_name(selected_loro: list[dict[str, object]]) -> str:
    aggregates: dict[str, list[float]] = defaultdict(list)
    for row in selected_loro:
        if row.get("status") == "absent_from_primary_feature_input":
            continue
        aggregates[str(row["model"])].append(float(row["corrected_rmse_nm"]))
    if not aggregates:
        return "none"
    return min(aggregates.items(), key=lambda item: mean(item[1]))[0]


def write_report(
    rows: list[dict[str, object]],
    metrics: list[dict[str, object]],
    selected_loro: list[dict[str, object]],
    kernel_tune: list[dict[str, object]],
    holes: list[dict[str, object]],
    yaw_rate_ref: list[dict[str, object]],
    best: str,
) -> None:
    split_counts: dict[str, int] = defaultdict(int)
    for row in rows:
        split_counts[str(row["dataset_split"])] += 1
    selected_present = sorted({str(row["run_id"]) for row in rows if str(row["run_id"]) in SELECTED_RUNS})
    selected_absent = [run for run in SELECTED_RUNS if run not in selected_present]

    def metric(model: str, group: str) -> dict[str, object]:
        return next((r for r in metrics if r["model"] == model and r["group"] == group), {})

    primary_rows = [r for r in metrics if r["group"] == f"split:{TRAIN_SPLIT}:all"]
    selected_rows = [r for r in metrics if r["group"] == "selected:all"]
    selected_loro_by_model: dict[str, list[dict[str, object]]] = defaultdict(list)
    for row in selected_loro:
        if row.get("status") != "absent_from_primary_feature_input":
            selected_loro_by_model[str(row["model"])].append(row)

    lines = [
        "# Variant D Residual Surface Fit",
        "",
        "Analysis-only output. Production code, build metadata, and tests were not modified.",
        "",
        "## Contract Alignment",
        "",
        f"- Primary input: `{FEATURE_SAMPLE.relative_to(ROOT)}`.",
        f"- Existing yaw-torque bins: `{EXPANDED_BINS.relative_to(ROOT)}`.",
        "- Target: `residual_additive_yaw_torque_nm`; correction is subtracted from that residual after fitting.",
        "- Training signal: `primary_open_floor_fit_authoritative`; downweighted, validation-only, `diag`, and `aux` rows are reported separately.",
        "- Predictors are continuous log/feature variables only. Phase labels are used for reporting, not fitting.",
        "",
        "## Data Used",
        "",
        "| Split | Rows |",
        "| --- | ---: |",
    ]
    for split, count in sorted(split_counts.items()):
        lines.append(f"| {split} | {count} |")
    lines.extend(["", f"Selected runs present: {', '.join(selected_present) if selected_present else 'none'}."])
    if selected_absent:
        lines.append(f"Selected runs absent from the primary feature input: {', '.join(selected_absent)}.")
    lines.extend(
        [
            "",
            "## Model Forms",
            "",
            "- `expanded_signed_bin_lookup`: existing `0.10 m/s x 0.50 rad/s` signed-bin median residual table from expanded validation.",
            "- `vf_yaw_ridge_surface`: continuous ridge fit over forward speed, yaw rate, absolute terms, interactions, `vbar_rel`, and force utilization.",
            "- `contact_feature_ridge_surface`: continuous ridge fit over contact-continuum velocity, force, load, and request primitives.",
            "- `vf_yaw_vbar_kernel_cell_surface`: median residual cells keyed by `Vf`, yaw rate, and `vbar_rel`, blended with a Gaussian kernel.",
            "",
            "## Primary Fit-Authoritative Performance",
            "",
            "| Model | Coverage | Baseline RMSE Nm | Corrected RMSE Nm | RMSE improvement | Corrected MAE Nm | Corrected median abs Nm |",
            "| --- | ---: | ---: | ---: | ---: | ---: | ---: |",
        ]
    )
    for row in sorted(primary_rows, key=lambda r: float(r["corrected_rmse_nm"])):
        lines.append(
            f"| {row['model']} | {float(row['coverage_fraction']):.3f} | "
            f"{float(row['baseline_rmse_nm']):.6f} | {float(row['corrected_rmse_nm']):.6f} | "
            f"{float(row['rmse_improvement_fraction']):.1%} | {float(row['corrected_mae_nm']):.6f} | "
            f"{float(row['corrected_median_abs_nm']):.6f} |"
        )
    lines.extend(
        [
            "",
            "## Selected-Log Aggregate",
            "",
            "| Model | Coverage | Baseline RMSE Nm | Corrected RMSE Nm | RMSE improvement | Corrected MAE Nm | Corrected median abs Nm |",
            "| --- | ---: | ---: | ---: | ---: | ---: | ---: |",
        ]
    )
    for row in sorted(selected_rows, key=lambda r: float(r["corrected_rmse_nm"])):
        lines.append(
            f"| {row['model']} | {float(row['coverage_fraction']):.3f} | "
            f"{float(row['baseline_rmse_nm']):.6f} | {float(row['corrected_rmse_nm']):.6f} | "
            f"{float(row['rmse_improvement_fraction']):.1%} | {float(row['corrected_mae_nm']):.6f} | "
            f"{float(row['corrected_median_abs_nm']):.6f} |"
        )
    lines.extend(
        [
            "",
            "## Leave-Selected-Run-Out",
            "",
            "Fit-authoritative selected runs are held out of training; validation/downweighted selected runs use the full fit-authoritative training set.",
            "",
            "| Model | Mean corrected RMSE Nm | Mean improvement | Mean coverage | Runs |",
            "| --- | ---: | ---: | ---: | ---: |",
        ]
    )
    for model, model_rows in sorted(selected_loro_by_model.items()):
        lines.append(
            f"| {model} | {mean([float(r['corrected_rmse_nm']) for r in model_rows]):.6f} | "
            f"{mean([float(r['rmse_improvement_fraction']) for r in model_rows]):.1%} | "
            f"{mean([float(r['coverage_fraction']) for r in model_rows]):.3f} | {len(model_rows)} |"
        )
    lines.extend(
        [
            "",
            "## Phase Performance Of Best Variant",
            "",
            f"Best selected leave-run-out mean RMSE: `{best}`.",
            "",
            "| Group | Rows | Baseline RMSE Nm | Corrected RMSE Nm | Improvement | Corrected median abs Nm |",
            "| --- | ---: | ---: | ---: | ---: | ---: |",
        ]
    )
    for split in [
        TRAIN_SPLIT,
        "open_floor_fit_downweighted",
        "open_floor_validation_only",
        "diag_validation_only",
        "aux_downweighted_validation",
    ]:
        for phase in PHASES:
            group = f"split:{split}:all" if phase == "all" else f"split:{split}:phase:{phase}"
            row = metric(best, group)
            if not row:
                continue
            lines.append(
                f"| {group} | {row['rows']} | {float(row['baseline_rmse_nm']):.6f} | "
                f"{float(row['corrected_rmse_nm']):.6f} | {float(row['rmse_improvement_fraction']):.1%} | "
                f"{float(row['corrected_median_abs_nm']):.6f} |"
            )
    lines.extend(
        [
            "",
            "## Existing One-Step Yaw-Rate Reference",
            "",
            "The common contract asks that one-step yaw-rate error use the existing validation summaries. This table is copied from the expanded signed-bin leave-run-out reference for selected runs.",
            "",
            "| Run | Samples | Current RMSE rad/s | Signed-bin RMSE rad/s | Improvement |",
            "| --- | ---: | ---: | ---: | ---: |",
        ]
    )
    for row in yaw_rate_ref:
        lines.append(
            f"| {row['run_id']} | {row['samples']} | {float(row['current_one_step_rmse_radps']):.6f} | "
            f"{float(row['expanded_signed_bin_one_step_rmse_radps']):.6f} | "
            f"{float(row['signed_bin_rmse_improvement_fraction']):.1%} |"
        )
    lines.extend(
        [
            "",
            "## Kernel Tuning",
            "",
            "| min cells | sigma Vf | sigma yaw | sigma vbar | LORO RMSE Nm | LORO MAE Nm | Coverage |",
            "| ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
        ]
    )
    for row in kernel_tune:
        lines.append(
            f"| {row['min_cell_count']} | {float(row['sigma_vf']):.3f} | {float(row['sigma_yaw']):.3f} | "
            f"{float(row['sigma_vbar_rel']):.3f} | {float(row['loro_corrected_rmse_nm']):.6f} | "
            f"{float(row['loro_corrected_mae_nm']):.6f} | {float(row['coverage_fraction']):.3f} |"
        )
    lines.extend(
        [
            "",
            "## Coverage Holes",
            "",
            "| Region | Rows | Runs | Existing-bin coverage | Top run fraction | Baseline RMSE Nm |",
            "| --- | ---: | ---: | ---: | ---: | ---: |",
        ]
    )
    for row in holes:
        lines.append(
            f"| {row.get('region', '')} | {row.get('rows', 0)} | {row.get('runs', '')} | "
            f"{float(row.get('expanded_bin_lookup_coverage_fraction', 0.0) or 0.0):.3f} | "
            f"{float(row.get('top_run_fraction', 0.0) or 0.0):.3f} | "
            f"{float(row.get('baseline_rmse_nm', 0.0) or 0.0):.6f} |"
        )
    best_primary = metric(best, f"split:{TRAIN_SPLIT}:all")
    best_selected = metric(best, "selected:all")
    lines.extend(
        [
            "",
            "## Productionization Judgment",
            "",
            "Do not productionize Variant D as the PlantModel correction.",
            "",
            f"The best residual-surface baseline, `{best}`, reduces sampled residual error "
            f"from {float(best_primary.get('baseline_rmse_nm', 0.0)):.6f} to "
            f"{float(best_primary.get('corrected_rmse_nm', 0.0)):.6f} Nm on the primary fit-authoritative sample and "
            f"from {float(best_selected.get('baseline_rmse_nm', 0.0)):.6f} to "
            f"{float(best_selected.get('corrected_rmse_nm', 0.0)):.6f} Nm on the selected-log aggregate. "
            "That is useful as a diagnostic baseline, but it is still a learned residual map layered on top of the physics model.",
            "",
            "The strongest production-relevant signal is that contact-feature predictors outperform the raw `Vf/yaw` table shape while preserving continuous variables. A production version should move that behavior into `PlantModel` contact-force equations, not ship a runtime residual table keyed by fitted errors.",
            "",
            "Main blockers: sparse high-forward/high-yaw coverage, selected low-speed launch logs have little nonzero-forward coverage in the common feature input, no independent lateral velocity measurement, no measured motor current, and residual targets are gyro-differentiated single-sample torques.",
            "",
            "## Files",
            "",
            "- `fit_residual_surface.py`",
            "- `surface_metrics_by_group.csv`",
            "- `selected_leave_run_out_metrics.csv`",
            "- `selected_run_signed_medians.csv`",
            "- `ridge_coefficients.csv`",
            "- `kernel_surface_cells.csv`",
            "- `kernel_tuning.csv`",
            "- `coverage_holes.csv`",
            "- `existing_yaw_rate_reference.csv`",
            "- `residual_surface_report.md`",
            "- `commands_run.txt`",
            "",
        ]
    )
    (OUT / "residual_surface_report.md").write_text("\n".join(lines), encoding="utf-8")


def selected_signed_medians(
    rows: list[dict[str, object]],
    best_name: str,
    bin_lookup: dict[tuple[float, float], float],
) -> list[dict[str, object]]:
    out: list[dict[str, object]] = []
    for run_id in SELECTED_RUNS:
        group = [row for row in rows if row["run_id"] == run_id]
        if not group:
            out.append({"run_id": run_id, "status": "absent_from_primary_feature_input"})
            continue
        train = train_rows_for_holdout(
            rows,
            run_id if any(row["dataset_split"] == TRAIN_SPLIT for row in group) else None,
        )
        best_model = next(model for model in instantiate_models(bin_lookup) if model.name == best_name)  # type: ignore[attr-defined]
        fit_fitted_model(best_model, train)
        bin_model = ExpandedBinLookup(bin_lookup)
        preds_best = predict_rows(best_model, group)
        preds_bin = predict_rows(bin_model, group)
        before = [f(row, "target_nm") for row in group]
        after_best = [f(row, "target_nm") - preds_best.get(id(row), 0.0) for row in group]
        after_bin = [f(row, "target_nm") - preds_bin.get(id(row), 0.0) for row in group]
        out.append(
            {
                "run_id": run_id,
                "dataset_split": group[0]["dataset_split"],
                "rows": len(group),
                "median_residual_before_nm": median(before),
                "median_residual_after_best_nm": median(after_best),
                "median_residual_after_expanded_bin_nm": median(after_bin),
                "best_model": best_name,
                "training_policy": "held_out_if_fit_authoritative_else_train_primary_only",
                "baseline_rmse_nm": rmse(before),
                "best_corrected_rmse_nm": rmse(after_best),
                "expanded_bin_corrected_rmse_nm": rmse(after_bin),
            }
        )
    return out


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    rows = load_rows()
    bin_lookup = load_expanded_bin_lookup()
    models = instantiate_models(bin_lookup)
    kernel_tune = tune_kernel(rows)
    metrics, coeffs, cells = evaluate_full_train(rows, models)
    selected_loro = evaluate_leave_selected_run_out(rows, bin_lookup)
    holes = coverage_holes(rows, bin_lookup)
    yaw_ref = load_yaw_rate_reference()
    best = best_model_name(selected_loro)

    signed = selected_signed_medians(rows, best, bin_lookup)

    write_csv(OUT / "surface_metrics_by_group.csv", metrics)
    write_csv(OUT / "selected_leave_run_out_metrics.csv", selected_loro)
    write_csv(OUT / "selected_run_signed_medians.csv", signed)
    write_csv(OUT / "ridge_coefficients.csv", coeffs)
    write_csv(OUT / "kernel_surface_cells.csv", cells)
    write_csv(OUT / "kernel_tuning.csv", kernel_tune)
    write_csv(OUT / "coverage_holes.csv", holes)
    write_csv(OUT / "existing_yaw_rate_reference.csv", yaw_ref)
    write_report(rows, metrics, selected_loro, kernel_tune, holes, yaw_ref, best)
    (OUT / "commands_run.txt").write_text(
        "python codex_analysis\\yaw_model_variant_fits\\residual_surface\\fit_residual_surface.py\n",
        encoding="utf-8",
    )

    best_primary = next(
        row for row in metrics if row["model"] == best and row["group"] == f"split:{TRAIN_SPLIT}:all"
    )
    best_selected = next(row for row in metrics if row["model"] == best and row["group"] == "selected:all")
    print(f"rows={len(rows)} train_rows={sum(1 for r in rows if r['dataset_split'] == TRAIN_SPLIT)}")
    print(f"expanded_bins={len(bin_lookup)}")
    print(f"best={best}")
    print(
        "primary_rmse_nm "
        f"{float(best_primary['baseline_rmse_nm']):.9f}->{float(best_primary['corrected_rmse_nm']):.9f}"
    )
    print(
        "selected_rmse_nm "
        f"{float(best_selected['baseline_rmse_nm']):.9f}->{float(best_selected['corrected_rmse_nm']):.9f}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
