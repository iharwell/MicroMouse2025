#!/usr/bin/env python3
"""Fit Variant E aggregate yaw-loss baselines.

Analysis-only tooling. Reads the shared phase-classified contact-continuum
feature sample and writes outputs beside this script. It does not edit or
import production code.
"""

from __future__ import annotations

import csv
import math
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Callable


ROOT = Path(__file__).resolve().parents[3]
OUT = ROOT / "codex_analysis" / "yaw_model_variant_fits" / "aggregate_yaw_loss"
INPUT = ROOT / "codex_analysis" / "contact_continuum_yaw_identification" / "ablation" / "phase_classified_feature_sample.csv"
CONSTANTS = ROOT / "codex_analysis" / "contact_continuum_yaw_identification" / "features" / "plant_mirror_constants.csv"

SELECTED_LOGS = [
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

SPLITS = [
    "primary_open_floor_fit_authoritative",
    "open_floor_fit_downweighted",
    "open_floor_validation_only",
    "diag_validation_only",
    "aux_downweighted_validation",
]

PHASES = ["all", "entry", "plateau", "exit"]


def num(row: dict[str, str], key: str, default: float = 0.0) -> float:
    try:
        value = row.get(key, "")
        if value == "":
            return default
        result = float(value)
        return result if math.isfinite(result) else default
    except (TypeError, ValueError):
        return default


def sign(x: float, eps: float = 1.0e-5) -> float:
    if x > eps:
        return 1.0
    if x < -eps:
        return -1.0
    return 0.0


def q(values: list[float], p: float) -> float:
    clean = sorted(v for v in values if math.isfinite(v))
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
    return sum(values) / len(values) if values else 0.0


def rmse(values: list[float]) -> float:
    return math.sqrt(mean([v * v for v in values])) if values else 0.0


def mae(values: list[float]) -> float:
    return mean([abs(v) for v in values])


def weighted_mean(values: list[float], weights: list[float]) -> float:
    total = sum(weights)
    if total <= 0.0:
        return 0.0
    return sum(v * w for v, w in zip(values, weights)) / total


def weighted_rmse(errors: list[float], weights: list[float]) -> float:
    return math.sqrt(weighted_mean([e * e for e in errors], weights)) if errors else 0.0


def weighted_mae(errors: list[float], weights: list[float]) -> float:
    return weighted_mean([abs(e) for e in errors], weights)


def weighted_r2(y: list[float], pred: list[float], weights: list[float]) -> float:
    if not y:
        return 0.0
    y_mean = weighted_mean(y, weights)
    ss_tot = sum(w * (yy - y_mean) ** 2 for yy, w in zip(y, weights))
    ss_res = sum(w * (yy - pp) ** 2 for yy, pp, w in zip(y, pred, weights))
    if ss_tot <= 1.0e-18:
        return 0.0
    return 1.0 - ss_res / ss_tot


@dataclass(frozen=True)
class Sample:
    run_id: str
    family: str
    dataset_split: str
    physics_phase: str
    row_index: int
    abs_forward_mps: float
    abs_yaw_radps: float
    vbar_rel_mps: float
    vbar_yaw_mps: float
    max_force_utilization: float
    limiter_activity: float
    saturation_evidence: float
    residual_nm: float
    yaw_sign: float
    target_opposes_yaw_nm: float


@dataclass(frozen=True)
class ModelSpec:
    name: str
    family: str
    params: str
    physical_yaw_loss: bool
    feature_names: tuple[str, ...]
    features: Callable[[Sample], list[float]]
    nonnegative_coefficients: bool


@dataclass
class FitResult:
    spec: ModelSpec
    coefficients: list[float]
    train_target_r2: float
    train_target_rmse_nm: float
    train_target_mae_nm: float
    primary_corrected_rmse_nm: float
    validation_corrected_rmse_nm: float
    selection_score_nm: float

    def predict_loss(self, sample: Sample) -> float:
        return sum(c * x for c, x in zip(self.coefficients, self.spec.features(sample)))


def load_constants() -> dict[str, float]:
    with CONSTANTS.open(newline="", encoding="utf-8") as fh:
        return {row["name"]: float(row["value"]) for row in csv.DictReader(fh)}


def load_samples() -> list[Sample]:
    samples: list[Sample] = []
    with INPUT.open(newline="", encoding="utf-8") as fh:
        for row in csv.DictReader(fh):
            split = row.get("dataset_split", "")
            if split not in SPLITS:
                continue
            yaw = num(row, "yaw_rate_radps")
            yaw_s = sign(yaw)
            residual = num(row, "residual_additive_yaw_torque_nm")
            target = row.get("residual_opposes_yaw_nm", "")
            if target == "":
                target_value = -yaw_s * residual
            else:
                target_value = num(row, "residual_opposes_yaw_nm")
            samples.append(
                Sample(
                    run_id=row.get("run_id", ""),
                    family=row.get("family", ""),
                    dataset_split=split,
                    physics_phase=row.get("physics_phase", ""),
                    row_index=int(num(row, "row_index")),
                    abs_forward_mps=abs(num(row, "forward_velocity_mps")),
                    abs_yaw_radps=abs(yaw),
                    vbar_rel_mps=max(0.0, num(row, "vbar_rel_mps")),
                    vbar_yaw_mps=max(0.0, num(row, "vbar_yaw_mps")),
                    max_force_utilization=max(0.0, num(row, "max_force_preprojection_utilization")),
                    limiter_activity=max(0.0, num(row, "max_force_limiter_activity")),
                    saturation_evidence=max(0.0, num(row, "hardware_saturation_evidence")),
                    residual_nm=residual,
                    yaw_sign=yaw_s,
                    target_opposes_yaw_nm=target_value,
                )
            )
    return samples


def run_balanced_weights(samples: list[Sample]) -> list[float]:
    counts: dict[str, int] = defaultdict(int)
    for sample in samples:
        counts[sample.run_id] += 1
    weights = []
    for sample in samples:
        weights.append(1.0 / max(counts[sample.run_id], 1))
    return weights


def solve_linear(a: list[list[float]], b: list[float]) -> list[float]:
    n = len(b)
    aug = [row[:] + [rhs] for row, rhs in zip(a, b)]
    for col in range(n):
        pivot = max(range(col, n), key=lambda r: abs(aug[r][col]))
        if abs(aug[pivot][col]) < 1.0e-18:
            continue
        if pivot != col:
            aug[col], aug[pivot] = aug[pivot], aug[col]
        div = aug[col][col]
        aug[col] = [v / div for v in aug[col]]
        for row_idx in range(n):
            if row_idx == col:
                continue
            factor = aug[row_idx][col]
            if factor:
                aug[row_idx] = [x - factor * y for x, y in zip(aug[row_idx], aug[col])]
    return [aug[i][-1] for i in range(n)]


def build_gram(samples: list[Sample], spec: ModelSpec, weights: list[float]) -> tuple[list[list[float]], list[float]]:
    p = len(spec.feature_names)
    xtx = [[0.0 for _ in range(p)] for _ in range(p)]
    xty = [0.0 for _ in range(p)]
    for sample, weight in zip(samples, weights):
        x = spec.features(sample)
        y = sample.target_opposes_yaw_nm
        for i in range(p):
            xty[i] += weight * x[i] * y
            for j in range(p):
                xtx[i][j] += weight * x[i] * x[j]
    for i in range(p):
        xtx[i][i] += 1.0e-12
    return xtx, xty


def solve_nnls_from_gram(xtx: list[list[float]], xty: list[float]) -> list[float]:
    p = len(xty)
    beta = [0.0 for _ in range(p)]
    for _ in range(400):
        max_delta = 0.0
        for j in range(p):
            denom = xtx[j][j]
            if denom <= 1.0e-18:
                continue
            other = sum(xtx[j][k] * beta[k] for k in range(p) if k != j)
            candidate = max(0.0, (xty[j] - other) / denom)
            max_delta = max(max_delta, abs(candidate - beta[j]))
            beta[j] = candidate
        if max_delta < 1.0e-13:
            break
    return beta


def fit_model(spec: ModelSpec, train: list[Sample], validation: list[Sample]) -> FitResult:
    fit_rows = [sample for sample in train if sample.yaw_sign != 0.0]
    weights = run_balanced_weights(fit_rows)
    xtx, xty = build_gram(fit_rows, spec, weights)
    if spec.nonnegative_coefficients:
        coefficients = solve_nnls_from_gram(xtx, xty)
    else:
        coefficients = solve_linear(xtx, xty)

    pred = [sum(c * x for c, x in zip(coefficients, spec.features(sample))) for sample in fit_rows]
    target = [sample.target_opposes_yaw_nm for sample in fit_rows]
    errors = [yy - pp for yy, pp in zip(target, pred)]

    train_metrics = residual_metrics(train, lambda s: sum(c * x for c, x in zip(coefficients, spec.features(s))))
    validation_metrics = residual_metrics(validation, lambda s: sum(c * x for c, x in zip(coefficients, spec.features(s))))
    return FitResult(
        spec=spec,
        coefficients=coefficients,
        train_target_r2=weighted_r2(target, pred, weights),
        train_target_rmse_nm=weighted_rmse(errors, weights),
        train_target_mae_nm=weighted_mae(errors, weights),
        primary_corrected_rmse_nm=float(train_metrics["corrected_rmse_nm"]),
        validation_corrected_rmse_nm=float(validation_metrics["corrected_rmse_nm"]),
        selection_score_nm=float(validation_metrics["corrected_rmse_nm"]),
    )


def correction_residual(sample: Sample, loss_prediction_nm: float) -> float:
    if sample.yaw_sign == 0.0:
        return sample.residual_nm
    # The correction adds a yaw torque opposing current yaw:
    # M_corrected = M_current - sign(yaw_rate) * loss_prediction.
    return sample.residual_nm + sample.yaw_sign * loss_prediction_nm


def residual_metrics(samples: list[Sample], predictor: Callable[[Sample], float] | None = None) -> dict[str, object]:
    if predictor is None:
        predictor = lambda _s: 0.0
    residuals = [sample.residual_nm for sample in samples]
    corrected = [correction_residual(sample, predictor(sample)) for sample in samples]
    abs_old = [abs(v) for v in residuals]
    abs_new = [abs(v) for v in corrected]
    worsened = sum(1 for old, new in zip(abs_old, abs_new) if new > old + 1.0e-12)
    target_negative = sum(1 for sample in samples if sample.yaw_sign != 0.0 and sample.target_opposes_yaw_nm < 0.0)
    return {
        "count": len(samples),
        "run_count": len({sample.run_id for sample in samples}),
        "baseline_rmse_nm": rmse(residuals),
        "corrected_rmse_nm": rmse(corrected),
        "baseline_mae_nm": mae(residuals),
        "corrected_mae_nm": mae(corrected),
        "baseline_median_abs_nm": median(abs_old),
        "corrected_median_abs_nm": median(abs_new),
        "baseline_signed_median_nm": median(residuals),
        "corrected_signed_median_nm": median(corrected),
        "rmse_delta_nm": rmse(corrected) - rmse(residuals),
        "rmse_delta_fraction": (rmse(corrected) / rmse(residuals) - 1.0) if rmse(residuals) > 0.0 else 0.0,
        "mae_delta_fraction": (mae(corrected) / mae(residuals) - 1.0) if mae(residuals) > 0.0 else 0.0,
        "worsened_abs_residual_fraction": worsened / len(samples) if samples else 0.0,
        "negative_opposes_target_fraction": target_negative / len(samples) if samples else 0.0,
        "mean_abs_forward_mps": mean([sample.abs_forward_mps for sample in samples]),
        "mean_abs_yaw_radps": mean([sample.abs_yaw_radps for sample in samples]),
        "mean_vbar_rel_mps": mean([sample.vbar_rel_mps for sample in samples]),
        "limiter_activity_fraction": mean([1.0 if sample.limiter_activity > 1.0e-12 else 0.0 for sample in samples]),
        "saturation_evidence_fraction": mean([1.0 if sample.saturation_evidence > 0.0 else 0.0 for sample in samples]),
    }


def make_models() -> list[ModelSpec]:
    models: list[ModelSpec] = []

    def yaw_tanh(yaw0: float) -> Callable[[Sample], float]:
        return lambda s: math.tanh(s.abs_yaw_radps / yaw0) if yaw0 > 0.0 else 0.0

    def forward_gate(vf0: float, sample: Sample) -> float:
        x = sample.abs_forward_mps / vf0 if vf0 > 0.0 else 0.0
        return 1.0 / (1.0 + x * x)

    def contact_gate(vrel0: float, sample: Sample) -> float:
        x = sample.vbar_rel_mps / vrel0 if vrel0 > 0.0 else 0.0
        return 1.0 / (1.0 + x * x)

    def high_forward_gate(vf0: float, sample: Sample) -> float:
        x = sample.abs_forward_mps / vf0 if vf0 > 0.0 else 0.0
        return (x * x) / (1.0 + x * x)

    def high_contact_gate(vrel0: float, sample: Sample) -> float:
        x = sample.vbar_rel_mps / vrel0 if vrel0 > 0.0 else 0.0
        return (x * x) / (1.0 + x * x)

    models.append(
        ModelSpec(
            name="E1_viscous_yaw_rate_nnls",
            family="pure_loss_viscous",
            params="loss = k_yaw * |yaw_rate|",
            physical_yaw_loss=True,
            feature_names=("abs_yaw_rate_radps",),
            features=lambda s: [s.abs_yaw_radps],
            nonnegative_coefficients=True,
        )
    )

    yaw0s = [0.05, 0.10, 0.25, 0.50, 1.00, 2.00]
    vf0s = [0.05, 0.10, 0.20, 0.40, 0.80]
    vrel0s = [0.01, 0.03, 0.06, 0.12, 0.25, 0.50]

    for yaw0 in yaw0s:
        models.append(
            ModelSpec(
                name=f"E2_smooth_coulomb_nnls_yaw0_{yaw0:g}",
                family="pure_loss_smooth_coulomb",
                params=f"loss = tau_c * tanh(|yaw_rate|/{yaw0:g})",
                physical_yaw_loss=True,
                feature_names=(f"tanh_abs_yaw_over_{yaw0:g}",),
                features=lambda s, yaw0=yaw0: [yaw_tanh(yaw0)(s)],
                nonnegative_coefficients=True,
            )
        )
        models.append(
            ModelSpec(
                name=f"E3_coulomb_viscous_nnls_yaw0_{yaw0:g}",
                family="pure_loss_coulomb_viscous",
                params=f"loss = tau_c * tanh(|yaw_rate|/{yaw0:g}) + k_yaw * |yaw_rate|",
                physical_yaw_loss=True,
                feature_names=(f"tanh_abs_yaw_over_{yaw0:g}", "abs_yaw_rate_radps"),
                features=lambda s, yaw0=yaw0: [yaw_tanh(yaw0)(s), s.abs_yaw_radps],
                nonnegative_coefficients=True,
            )
        )

    for yaw0 in yaw0s:
        for vf0 in vf0s:
            models.append(
                ModelSpec(
                    name=f"E4_forward_faded_loss_nnls_yaw0_{yaw0:g}_vf0_{vf0:g}",
                    family="pure_loss_forward_faded",
                    params=f"loss = gate_vf({vf0:g}) * [tau_c*tanh(|yaw|/{yaw0:g}) + k_yaw*|yaw|]",
                    physical_yaw_loss=True,
                    feature_names=(f"vf_gate_{vf0:g}_tanh_yaw_{yaw0:g}", f"vf_gate_{vf0:g}_abs_yaw"),
                    features=lambda s, yaw0=yaw0, vf0=vf0: [
                        forward_gate(vf0, s) * yaw_tanh(yaw0)(s),
                        forward_gate(vf0, s) * s.abs_yaw_radps,
                    ],
                    nonnegative_coefficients=True,
                )
            )
        for vrel0 in vrel0s:
            models.append(
                ModelSpec(
                    name=f"E5_contact_faded_loss_nnls_yaw0_{yaw0:g}_vrel0_{vrel0:g}",
                    family="pure_loss_contact_faded",
                    params=f"loss = gate_vrel({vrel0:g}) * [tau_c*tanh(|yaw|/{yaw0:g}) + k_yaw*|yaw|]",
                    physical_yaw_loss=True,
                    feature_names=(f"vrel_gate_{vrel0:g}_tanh_yaw_{yaw0:g}", f"vrel_gate_{vrel0:g}_abs_yaw"),
                    features=lambda s, yaw0=yaw0, vrel0=vrel0: [
                        contact_gate(vrel0, s) * yaw_tanh(yaw0)(s),
                        contact_gate(vrel0, s) * s.abs_yaw_radps,
                    ],
                    nonnegative_coefficients=True,
                )
            )

    for yaw0 in [0.10, 0.25, 0.50, 1.00]:
        for vf0 in [0.10, 0.20, 0.40]:
            for vrel0 in [0.03, 0.06, 0.12]:
                models.append(
                    ModelSpec(
                        name=f"E6_forward_contact_faded_loss_nnls_yaw0_{yaw0:g}_vf0_{vf0:g}_vrel0_{vrel0:g}",
                        family="pure_loss_forward_contact_faded",
                        params=(
                            f"loss = gate_vf({vf0:g})*gate_vrel({vrel0:g})"
                            f" * [tau_c*tanh(|yaw|/{yaw0:g}) + k_yaw*|yaw|]"
                        ),
                        physical_yaw_loss=True,
                        feature_names=(
                            f"vf_vrel_gate_tanh_yaw_{yaw0:g}_{vf0:g}_{vrel0:g}",
                            f"vf_vrel_gate_abs_yaw_{vf0:g}_{vrel0:g}",
                        ),
                        features=lambda s, yaw0=yaw0, vf0=vf0, vrel0=vrel0: [
                            forward_gate(vf0, s) * contact_gate(vrel0, s) * yaw_tanh(yaw0)(s),
                            forward_gate(vf0, s) * contact_gate(vrel0, s) * s.abs_yaw_radps,
                        ],
                        nonnegative_coefficients=True,
                    )
                )

    for yaw0 in [0.10, 0.25, 0.50, 1.00]:
        for vf0 in [0.10, 0.20, 0.40, 0.80]:
            models.append(
                ModelSpec(
                    name=f"E7_signed_forward_relief_yaw0_{yaw0:g}_vf0_{vf0:g}",
                    family="signed_forward_relief_not_pure_loss",
                    params=(
                        f"loss_or_relief = tau_c*tanh(|yaw|/{yaw0:g}) + k_yaw*|yaw|"
                        f" + tau_h*high_vf({vf0:g})*tanh(|yaw|/{yaw0:g}) + k_h*high_vf({vf0:g})*|yaw|"
                    ),
                    physical_yaw_loss=False,
                    feature_names=(
                        f"tanh_yaw_{yaw0:g}",
                        "abs_yaw_rate_radps",
                        f"high_vf_{vf0:g}_tanh_yaw_{yaw0:g}",
                        f"high_vf_{vf0:g}_abs_yaw",
                    ),
                    features=lambda s, yaw0=yaw0, vf0=vf0: [
                        yaw_tanh(yaw0)(s),
                        s.abs_yaw_radps,
                        high_forward_gate(vf0, s) * yaw_tanh(yaw0)(s),
                        high_forward_gate(vf0, s) * s.abs_yaw_radps,
                    ],
                    nonnegative_coefficients=False,
                )
            )
        for vrel0 in [0.03, 0.06, 0.12, 0.25]:
            models.append(
                ModelSpec(
                    name=f"E8_signed_contact_relief_yaw0_{yaw0:g}_vrel0_{vrel0:g}",
                    family="signed_contact_relief_not_pure_loss",
                    params=(
                        f"loss_or_relief = tau_c*tanh(|yaw|/{yaw0:g}) + k_yaw*|yaw|"
                        f" + tau_h*high_vrel({vrel0:g})*tanh(|yaw|/{yaw0:g}) + k_h*high_vrel({vrel0:g})*|yaw|"
                    ),
                    physical_yaw_loss=False,
                    feature_names=(
                        f"tanh_yaw_{yaw0:g}",
                        "abs_yaw_rate_radps",
                        f"high_vrel_{vrel0:g}_tanh_yaw_{yaw0:g}",
                        f"high_vrel_{vrel0:g}_abs_yaw",
                    ),
                    features=lambda s, yaw0=yaw0, vrel0=vrel0: [
                        yaw_tanh(yaw0)(s),
                        s.abs_yaw_radps,
                        high_contact_gate(vrel0, s) * yaw_tanh(yaw0)(s),
                        high_contact_gate(vrel0, s) * s.abs_yaw_radps,
                    ],
                    nonnegative_coefficients=False,
                )
            )

    return models


def row_filter(samples: list[Sample], split: str | None = None, phase: str = "all", run_id: str | None = None) -> list[Sample]:
    out = samples
    if split is not None:
        out = [sample for sample in out if sample.dataset_split == split]
    if phase != "all":
        out = [sample for sample in out if sample.physics_phase == phase]
    if run_id is not None:
        out = [sample for sample in out if sample.run_id == run_id]
    return out


def failure_groups(samples: list[Sample]) -> dict[str, list[Sample]]:
    groups: dict[str, list[Sample]] = {
        "low_motion_near_zero_yaw": [],
        "in_place_or_scrub_yaw": [],
        "slow_forward_turn": [],
        "moderate_forward_turn": [],
        "high_forward_turn": [],
        "high_yaw_low_forward_scrub": [],
        "mostly_straight_forward": [],
        "target_negative_over_resisted": [],
    }
    for sample in samples:
        if sample.abs_forward_mps < 0.05 and sample.abs_yaw_radps < 0.25:
            groups["low_motion_near_zero_yaw"].append(sample)
        if sample.abs_forward_mps < 0.05 and sample.abs_yaw_radps >= 0.25:
            groups["in_place_or_scrub_yaw"].append(sample)
        if 0.05 <= sample.abs_forward_mps < 0.25 and sample.abs_yaw_radps >= 0.25:
            groups["slow_forward_turn"].append(sample)
        if 0.25 <= sample.abs_forward_mps < 0.60 and sample.abs_yaw_radps >= 0.25:
            groups["moderate_forward_turn"].append(sample)
        if sample.abs_forward_mps >= 0.60 and sample.abs_yaw_radps >= 0.25:
            groups["high_forward_turn"].append(sample)
        if sample.abs_forward_mps < 0.25 and sample.abs_yaw_radps >= 8.0:
            groups["high_yaw_low_forward_scrub"].append(sample)
        if sample.abs_forward_mps >= 0.25 and sample.abs_yaw_radps < 0.25:
            groups["mostly_straight_forward"].append(sample)
        if sample.yaw_sign != 0.0 and sample.target_opposes_yaw_nm < 0.0:
            groups["target_negative_over_resisted"].append(sample)
    return groups


def write_csv(path: Path, rows: list[dict[str, object]], fieldnames: list[str]) -> None:
    with path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def pct(value: object) -> str:
    try:
        return f"{100.0 * float(value):.3f}%"
    except (TypeError, ValueError):
        return str(value)


def fmt(value: object, digits: int = 9) -> str:
    if isinstance(value, float):
        return f"{value:.{digits}f}"
    return str(value)


def make_predictor(result: FitResult) -> Callable[[Sample], float]:
    return result.predict_loss


def coefficient_rows(result: FitResult) -> list[dict[str, object]]:
    rows = []
    for name, coeff in zip(result.spec.feature_names, result.coefficients):
        rows.append(
            {
                "model": result.spec.name,
                "family": result.spec.family,
                "physical_yaw_loss": result.spec.physical_yaw_loss,
                "params": result.spec.params,
                "feature": name,
                "coefficient_nm_per_feature_unit": coeff,
            }
        )
    return rows


def write_report(
    samples: list[Sample],
    constants: dict[str, float],
    results: list[FitResult],
    selected_physical: FitResult,
    best_nonzero_physical: FitResult,
    selected_signed: FitResult,
    overview_rows: list[dict[str, object]],
    split_rows: list[dict[str, object]],
    selected_log_rows: list[dict[str, object]],
    failure_rows: list[dict[str, object]],
) -> None:
    top_physical = [row for row in overview_rows if row["physical_yaw_loss"] == True][:5]
    top_signed = [row for row in overview_rows if row["physical_yaw_loss"] == False][:5]
    primary = next(
        row
        for row in split_rows
        if row["model"] == selected_physical.spec.name
        and row["dataset_split"] == "primary_open_floor_fit_authoritative"
        and row["physics_phase"] == "all"
    )
    validation = next(
        row
        for row in split_rows
        if row["model"] == selected_physical.spec.name
        and row["dataset_split"] == "validation_combined_non_authoritative"
        and row["physics_phase"] == "all"
    )
    nonzero_primary = next(
        row
        for row in split_rows
        if row["model"] == best_nonzero_physical.spec.name
        and row["dataset_split"] == "primary_open_floor_fit_authoritative"
        and row["physics_phase"] == "all"
    )
    nonzero_validation = next(
        row
        for row in split_rows
        if row["model"] == best_nonzero_physical.spec.name
        and row["dataset_split"] == "validation_combined_non_authoritative"
        and row["physics_phase"] == "all"
    )
    signed_validation = next(
        row
        for row in split_rows
        if row["model"] == selected_signed.spec.name
        and row["dataset_split"] == "validation_combined_non_authoritative"
        and row["physics_phase"] == "all"
    )

    selected_low = [row for row in selected_log_rows if row["model"] == best_nonzero_physical.spec.name]
    report = [
        "# Variant E: Aggregate Yaw-Loss Baseline",
        "",
        "Analysis-only output. Production code, build metadata, and tests were not modified.",
        "",
        "## Input and Split Basis",
        "",
        f"- Primary input: `{INPUT.relative_to(ROOT)}`",
        f"- Samples used: {len(samples)} across {len({sample.run_id for sample in samples})} runs.",
        "- Training signal: `primary_open_floor_fit_authoritative` rows from the shared `dataset_split` column.",
        "- Validation: downweighted open-floor, validation-only open-floor, `diag`, and `aux` rows reported separately and combined.",
        "- Target convention: positive predicted loss means an added yaw torque opposing the measured yaw rate. Corrected residual is `residual_additive_yaw_torque_nm + sign(yaw_rate) * predicted_loss_nm`.",
        f"- Yaw denominator reference from constants: `{constants.get('yaw_denominator_including_wheel_spinup_kg_m2', 0.0):.12f} kg*m^2`; this report keeps the required metrics in Nm.",
        "",
        "## Validation-Guarded Physical Selection",
        "",
        f"- Model: `{selected_physical.spec.name}`",
        f"- Form: `{selected_physical.spec.params}`",
        f"- Train target weighted R2 against `residual_opposes_yaw_nm`: {selected_physical.train_target_r2:.6f}",
        f"- Primary fit-authoritative residual RMSE: {float(primary['baseline_rmse_nm']):.9f} -> {float(primary['corrected_rmse_nm']):.9f} Nm ({pct(primary['rmse_delta_fraction'])})",
        f"- Combined non-authoritative validation RMSE: {float(validation['baseline_rmse_nm']):.9f} -> {float(validation['corrected_rmse_nm']):.9f} Nm ({pct(validation['rmse_delta_fraction'])})",
        f"- Validation worsened-sample fraction: {pct(validation['worsened_abs_residual_fraction'])}",
        "",
        "This selection is intentionally allowed to choose zero. It is the validation-safe physical aggregate-loss result: do not add an aggregate yaw-loss term from this family.",
        "",
        "Coefficients:",
        "",
        "| Feature | Coefficient |",
        "| --- | ---: |",
    ]
    for row in coefficient_rows(selected_physical):
        report.append(f"| `{row['feature']}` | {float(row['coefficient_nm_per_feature_unit']):.12g} |")

    report.extend(
        [
            "",
            "## Best Nonzero Physical Aggregate Loss",
            "",
            f"- Model: `{best_nonzero_physical.spec.name}`",
            f"- Form: `{best_nonzero_physical.spec.params}`",
            f"- Train target weighted R2: {best_nonzero_physical.train_target_r2:.6f}",
            f"- Primary fit-authoritative residual RMSE: {float(nonzero_primary['baseline_rmse_nm']):.9f} -> {float(nonzero_primary['corrected_rmse_nm']):.9f} Nm ({pct(nonzero_primary['rmse_delta_fraction'])})",
            f"- Combined non-authoritative validation RMSE: {float(nonzero_validation['baseline_rmse_nm']):.9f} -> {float(nonzero_validation['corrected_rmse_nm']):.9f} Nm ({pct(nonzero_validation['rmse_delta_fraction'])})",
            f"- Validation worsened-sample fraction: {pct(nonzero_validation['worsened_abs_residual_fraction'])}",
            "",
            "| Feature | Coefficient |",
            "| --- | ---: |",
        ]
    )
    for row in coefficient_rows(best_nonzero_physical):
        report.append(f"| `{row['feature']}` | {float(row['coefficient_nm_per_feature_unit']):.12g} |")

    report.extend(
        [
            "",
            "## Signed Relief Diagnostic",
            "",
            "The best signed aggregate surface is reported as a diagnostic because it can predict negative loss, which means adding yaw torque in the direction of rotation to compensate current over-resistance. That is not a pure aggregate Coulomb/damping loss.",
            "",
            f"- Model: `{selected_signed.spec.name}`",
            f"- Form: `{selected_signed.spec.params}`",
            f"- Train target weighted R2: {selected_signed.train_target_r2:.6f}",
            f"- Combined validation RMSE: {float(signed_validation['baseline_rmse_nm']):.9f} -> {float(signed_validation['corrected_rmse_nm']):.9f} Nm ({pct(signed_validation['rmse_delta_fraction'])})",
            "",
            "## Top Candidate Overview",
            "",
            "Top physical yaw-loss candidates by combined validation corrected RMSE:",
            "",
            "| Model | Train target R2 | Primary delta | Validation delta | Validation RMSE Nm |",
            "| --- | ---: | ---: | ---: | ---: |",
        ]
    )
    for row in top_physical:
        report.append(
            f"| `{row['model']}` | {float(row['train_target_r2']):.6f} | {pct(row['primary_rmse_delta_fraction'])} | {pct(row['validation_rmse_delta_fraction'])} | {float(row['validation_corrected_rmse_nm']):.9f} |"
        )
    report.extend(
        [
            "",
            "Top signed-relief diagnostics:",
            "",
            "| Model | Train target R2 | Primary delta | Validation delta | Validation RMSE Nm |",
            "| --- | ---: | ---: | ---: | ---: |",
        ]
    )
    for row in top_signed:
        report.append(
            f"| `{row['model']}` | {float(row['train_target_r2']):.6f} | {pct(row['primary_rmse_delta_fraction'])} | {pct(row['validation_rmse_delta_fraction'])} | {float(row['validation_corrected_rmse_nm']):.9f} |"
        )

    report.extend(
        [
            "",
            "## Selected Log Results",
            "",
            f"These rows use the best nonzero physical aggregate-loss fit, `{best_nonzero_physical.spec.name}`. The validation-guarded physical selection is the zero-loss model above.",
            "",
            "| Run | Present | Rows | Baseline RMSE Nm | Corrected RMSE Nm | Delta | Median residual before Nm | Median residual after Nm | Negative target frac |",
            "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
        ]
    )
    for row in selected_low:
        if not row["present"]:
            report.append(f"| `{row['run_id']}` | no | 0 |  |  |  |  |  |  |")
        else:
            report.append(
                f"| `{row['run_id']}` | yes | {row['count']} | {float(row['baseline_rmse_nm']):.9f} | {float(row['corrected_rmse_nm']):.9f} | {pct(row['rmse_delta_fraction'])} | {float(row['baseline_signed_median_nm']):.9f} | {float(row['corrected_signed_median_nm']):.9f} | {pct(row['negative_opposes_target_fraction'])} |"
            )

    report.extend(
        [
            "",
            "## Failure Modes",
            "",
            f"Failure groups below use the best nonzero physical aggregate-loss fit, `{best_nonzero_physical.spec.name}`.",
            "",
            "| Group | Rows | Baseline RMSE Nm | Physical corrected RMSE Nm | Delta | Worsened frac | Negative target frac |",
            "| --- | ---: | ---: | ---: | ---: | ---: | ---: |",
        ]
    )
    for row in failure_rows:
        if row["model"] != best_nonzero_physical.spec.name:
            continue
        report.append(
            f"| `{row['failure_group']}` | {row['count']} | {float(row['baseline_rmse_nm']):.9f} | {float(row['corrected_rmse_nm']):.9f} | {pct(row['rmse_delta_fraction'])} | {pct(row['worsened_abs_residual_fraction'])} | {pct(row['negative_opposes_target_fraction'])} |"
        )

    report.extend(
        [
            "",
            "Interpretation:",
            "",
            "- The physical aggregate-loss family can only add resistance. Rows whose target is negative are already over-resisted by the current mirror; a pure loss cannot fix those rows and often worsens them.",
            "- Forward/contact fading helps avoid some high-speed conflict, but it cannot express front/rear or left/right patch asymmetry, so it misses the contact-continuum behavior that the prior ablation found.",
            "- The signed-relief diagnostic usually scores better when high-speed over-resistance dominates, but it is no longer an aggregate yaw-loss model and would be less production-eligible than a contact-patch force formulation.",
            "",
            "## Output Files",
            "",
            "- `fit_aggregate_yaw_loss.py`",
            "- `candidate_overview.csv`",
            "- `selected_model_coefficients.csv`",
            "- `split_phase_metrics.csv`",
            "- `selected_log_metrics.csv`",
            "- `failure_mode_metrics.csv`",
            "- `commands_run.txt`",
        ]
    )
    (OUT / "aggregate_yaw_loss_report.md").write_text("\n".join(report) + "\n", encoding="utf-8")


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    constants = load_constants()
    samples = load_samples()
    train = row_filter(samples, split="primary_open_floor_fit_authoritative")
    validation = [sample for sample in samples if sample.dataset_split != "primary_open_floor_fit_authoritative"]

    results = [fit_model(spec, train, validation) for spec in make_models()]

    def candidate_sort_key(result: FitResult) -> tuple[float, float, float]:
        return (result.validation_corrected_rmse_nm, result.primary_corrected_rmse_nm, -result.train_target_r2)

    physical_results = sorted([result for result in results if result.spec.physical_yaw_loss], key=candidate_sort_key)
    signed_results = sorted([result for result in results if not result.spec.physical_yaw_loss], key=candidate_sort_key)
    selected_physical = physical_results[0]
    best_nonzero_physical = next(
        result for result in physical_results if sum(abs(coeff) for coeff in result.coefficients) > 1.0e-12
    )
    selected_signed = signed_results[0]
    selected_results = [selected_physical]
    if best_nonzero_physical.spec.name != selected_physical.spec.name:
        selected_results.append(best_nonzero_physical)
    selected_results.append(selected_signed)

    baseline_primary = residual_metrics(train)
    baseline_validation = residual_metrics(validation)
    overview_rows: list[dict[str, object]] = []
    for result in sorted(results, key=candidate_sort_key):
        primary = residual_metrics(train, make_predictor(result))
        val = residual_metrics(validation, make_predictor(result))
        overview_rows.append(
            {
                "model": result.spec.name,
                "family": result.spec.family,
                "physical_yaw_loss": result.spec.physical_yaw_loss,
                "params": result.spec.params,
                "feature_count": len(result.spec.feature_names),
                "train_target_r2": result.train_target_r2,
                "train_target_rmse_nm": result.train_target_rmse_nm,
                "train_target_mae_nm": result.train_target_mae_nm,
                "primary_baseline_rmse_nm": baseline_primary["baseline_rmse_nm"],
                "primary_corrected_rmse_nm": primary["corrected_rmse_nm"],
                "primary_rmse_delta_fraction": primary["rmse_delta_fraction"],
                "validation_baseline_rmse_nm": baseline_validation["baseline_rmse_nm"],
                "validation_corrected_rmse_nm": val["corrected_rmse_nm"],
                "validation_rmse_delta_fraction": val["rmse_delta_fraction"],
                "validation_worsened_abs_residual_fraction": val["worsened_abs_residual_fraction"],
                "selected_physical": result.spec.name == selected_physical.spec.name,
                "best_nonzero_physical": result.spec.name == best_nonzero_physical.spec.name,
                "selected_signed_diagnostic": result.spec.name == selected_signed.spec.name,
            }
        )

    coefficient_output: list[dict[str, object]] = []
    for result in selected_results:
        coefficient_output.extend(coefficient_rows(result))

    split_rows: list[dict[str, object]] = []
    for result in selected_results:
        predictor = make_predictor(result)
        for split in SPLITS:
            for phase in PHASES:
                rows = row_filter(samples, split=split, phase=phase)
                if not rows:
                    continue
                metrics = residual_metrics(rows, predictor)
                split_rows.append(
                    {
                        "model": result.spec.name,
                        "physical_yaw_loss": result.spec.physical_yaw_loss,
                        "dataset_split": split,
                        "physics_phase": phase,
                        **metrics,
                    }
                )
        for phase in PHASES:
            rows = [sample for sample in validation if phase == "all" or sample.physics_phase == phase]
            if not rows:
                continue
            split_rows.append(
                {
                    "model": result.spec.name,
                    "physical_yaw_loss": result.spec.physical_yaw_loss,
                    "dataset_split": "validation_combined_non_authoritative",
                    "physics_phase": phase,
                    **residual_metrics(rows, predictor),
                }
            )

    selected_log_rows: list[dict[str, object]] = []
    for result in selected_results:
        predictor = make_predictor(result)
        for run_id in SELECTED_LOGS:
            rows = row_filter(samples, run_id=run_id)
            if not rows:
                selected_log_rows.append(
                    {
                        "model": result.spec.name,
                        "physical_yaw_loss": result.spec.physical_yaw_loss,
                        "run_id": run_id,
                        "present": False,
                        "count": 0,
                    }
                )
                continue
            selected_log_rows.append(
                {
                    "model": result.spec.name,
                    "physical_yaw_loss": result.spec.physical_yaw_loss,
                    "run_id": run_id,
                    "present": True,
                    "dataset_split": rows[0].dataset_split,
                    **residual_metrics(rows, predictor),
                }
            )

    failure_rows: list[dict[str, object]] = []
    groups = failure_groups(samples)
    for result in selected_results:
        predictor = make_predictor(result)
        for name, rows in groups.items():
            if not rows:
                continue
            failure_rows.append(
                {
                    "model": result.spec.name,
                    "physical_yaw_loss": result.spec.physical_yaw_loss,
                    "failure_group": name,
                    **residual_metrics(rows, predictor),
                }
            )

    write_csv(
        OUT / "candidate_overview.csv",
        overview_rows,
        [
            "model",
            "family",
            "physical_yaw_loss",
            "params",
            "feature_count",
            "train_target_r2",
            "train_target_rmse_nm",
            "train_target_mae_nm",
            "primary_baseline_rmse_nm",
            "primary_corrected_rmse_nm",
            "primary_rmse_delta_fraction",
            "validation_baseline_rmse_nm",
            "validation_corrected_rmse_nm",
            "validation_rmse_delta_fraction",
            "validation_worsened_abs_residual_fraction",
            "selected_physical",
            "best_nonzero_physical",
            "selected_signed_diagnostic",
        ],
    )
    write_csv(
        OUT / "selected_model_coefficients.csv",
        coefficient_output,
        ["model", "family", "physical_yaw_loss", "params", "feature", "coefficient_nm_per_feature_unit"],
    )

    metric_fields = [
        "model",
        "physical_yaw_loss",
        "dataset_split",
        "physics_phase",
        "count",
        "run_count",
        "baseline_rmse_nm",
        "corrected_rmse_nm",
        "baseline_mae_nm",
        "corrected_mae_nm",
        "baseline_median_abs_nm",
        "corrected_median_abs_nm",
        "baseline_signed_median_nm",
        "corrected_signed_median_nm",
        "rmse_delta_nm",
        "rmse_delta_fraction",
        "mae_delta_fraction",
        "worsened_abs_residual_fraction",
        "negative_opposes_target_fraction",
        "mean_abs_forward_mps",
        "mean_abs_yaw_radps",
        "mean_vbar_rel_mps",
        "limiter_activity_fraction",
        "saturation_evidence_fraction",
    ]
    write_csv(OUT / "split_phase_metrics.csv", split_rows, metric_fields)
    write_csv(
        OUT / "selected_log_metrics.csv",
        selected_log_rows,
        [
            "model",
            "physical_yaw_loss",
            "run_id",
            "present",
            "dataset_split",
            "count",
            "run_count",
            "baseline_rmse_nm",
            "corrected_rmse_nm",
            "baseline_mae_nm",
            "corrected_mae_nm",
            "baseline_median_abs_nm",
            "corrected_median_abs_nm",
            "baseline_signed_median_nm",
            "corrected_signed_median_nm",
            "rmse_delta_nm",
            "rmse_delta_fraction",
            "mae_delta_fraction",
            "worsened_abs_residual_fraction",
            "negative_opposes_target_fraction",
            "mean_abs_forward_mps",
            "mean_abs_yaw_radps",
            "mean_vbar_rel_mps",
            "limiter_activity_fraction",
            "saturation_evidence_fraction",
        ],
    )
    write_csv(
        OUT / "failure_mode_metrics.csv",
        failure_rows,
        ["model", "physical_yaw_loss", "failure_group"] + metric_fields[4:],
    )
    write_report(
        samples,
        constants,
        results,
        selected_physical,
        best_nonzero_physical,
        selected_signed,
        overview_rows,
        split_rows,
        selected_log_rows,
        failure_rows,
    )
    (OUT / "commands_run.txt").write_text(
        "python codex_analysis\\yaw_model_variant_fits\\aggregate_yaw_loss\\fit_aggregate_yaw_loss.py\n",
        encoding="utf-8",
    )


if __name__ == "__main__":
    main()
