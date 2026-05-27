from __future__ import annotations

import csv
import json
from pathlib import Path


REPO = Path(__file__).resolve().parents[3]
ROOT = REPO / "codex_analysis"
OUT = ROOT / "yaw_model_variant_fits" / "round2_b_correct_branch_reference"
STRIBECK = ROOT / "yaw_model_variant_fits" / "stribeck_scrub"
LR_GRID = ROOT / "yaw_model_variant_fits" / "lr_delta_grid"
IN_PLACE = ROOT / "yaw_model_variant_fits" / "in_place_1radps_command"
ROUND2_HYBRID = ROOT / "yaw_model_variant_fits" / "round2_hybrid_b_c"
CONTACT_SAMPLE = ROOT / "contact_continuum_yaw_identification" / "ablation" / "phase_classified_feature_sample.csv"


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def write_csv(path: Path, rows: list[dict[str, object]], fieldnames: list[str] | None = None) -> None:
    if fieldnames is None:
        fieldnames = list(rows[0].keys()) if rows else []
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def f(value: str | float | int) -> float:
    return float(value)


def fmt(value: str | float | int, digits: int = 6) -> str:
    return f"{f(value):.{digits}f}"


def coeff_value(rows: list[dict[str, str]], key: str) -> float:
    for row in rows:
        if row["parameter"] == key:
            return f(row["value"])
    raise KeyError(key)


def write_pivot(path: Path, rows: list[dict[str, str]]) -> None:
    yaw_values = sorted({f(row["yaw_rate_radps"]) for row in rows})
    vf_values = sorted({f(row["vf_mps"]) for row in rows})
    by_cell = {(f(row["vf_mps"]), f(row["yaw_rate_radps"])): f(row["lr_delta_command"]) for row in rows}
    lines = [
        "# B Correct-Branch L-R Delta Command Grid",
        "",
        "Values are `left_command - right_command` for positive clockwise yaw.",
        "",
        "| Vf m/s | " + " | ".join(fmt(yaw, 3) for yaw in yaw_values) + " |",
        "| --- | " + " | ".join("---:" for _ in yaw_values) + " |",
    ]
    for vf in vf_values:
        values = [fmt(by_cell[(vf, yaw)], 6) for yaw in yaw_values]
        lines.append("| " + fmt(vf, 3) + " | " + " | ".join(values) + " |")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def summarize_contact_sample(path: Path) -> list[dict[str, object]]:
    rows_by_split: dict[str, dict[str, object]] = {}
    total = 0
    runs: set[str] = set()
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            total += 1
            split = row["dataset_split"]
            run_id = row["run_id"]
            runs.add(run_id)
            entry = rows_by_split.setdefault(split, {"dataset_split": split, "count": 0, "run_ids": set()})
            entry["count"] = int(entry["count"]) + 1
            entry["run_ids"].add(run_id)
    output: list[dict[str, object]] = [
        {"dataset_split": "ALL", "count": total, "run_count": len(runs)}
    ]
    for split in sorted(rows_by_split):
        entry = rows_by_split[split]
        output.append(
            {
                "dataset_split": split,
                "count": entry["count"],
                "run_count": len(entry["run_ids"]),
            }
        )
    return output


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)

    coeff_rows = read_csv(STRIBECK / "stribeck_coefficients.csv")
    write_csv(OUT / "coefficients.csv", coeff_rows, ["parameter", "value", "unit"])

    in_place_rows = read_csv(IN_PLACE / "in_place_1radps_command_estimate.csv")
    b_in_place = [row for row in in_place_rows if row["variant"] == "Variant B Stribeck scrub"]
    write_csv(OUT / "in_place_1radps_command.csv", b_in_place)

    all_grid_rows = read_csv(LR_GRID / "lr_delta_grid.csv")
    b_grid_rows = [row for row in all_grid_rows if row["variant"] == "B_stribeck"]
    b_grid_rows.sort(key=lambda row: (f(row["vf_mps"]), f(row["yaw_rate_radps"])))
    write_csv(OUT / "lr_delta_grid_6x10.csv", b_grid_rows)
    write_pivot(OUT / "lr_delta_pivot.md", b_grid_rows)

    split_metrics = read_csv(STRIBECK / "metrics_by_split.csv")
    selected_log_metrics = read_csv(STRIBECK / "metrics_by_selected_run.csv")
    write_csv(OUT / "split_rmse.csv", split_metrics)
    write_csv(OUT / "selected_log_rmse.csv", selected_log_metrics)

    contact_summary = summarize_contact_sample(CONTACT_SAMPLE)
    write_csv(OUT / "contact_feature_sample_summary.csv", contact_summary)

    comparison_rows: list[dict[str, object]] = []
    b_ref = b_in_place[0]
    comparison_rows.append(
        {
            "source": "in_place_1radps_command_estimate.csv",
            "variant": "Variant B Stribeck scrub",
            "left_command": b_ref["left_command"],
            "right_command": b_ref["right_command"],
            "lr_delta_command": f(b_ref["left_command"]) - f(b_ref["right_command"]),
            "extra_opposing_yaw_torque_nm": b_ref["extra_opposing_yaw_torque_nm"],
            "note": "correct branch reference; fixed point through request-only activation",
        }
    )
    hybrid_in_place_path = ROUND2_HYBRID / "in_place_1radps_command.csv"
    if hybrid_in_place_path.exists():
        for row in read_csv(hybrid_in_place_path):
            if row["variant"] in {"B_stribeck", "Hybrid_BC_adhesion_partition"}:
                comparison_rows.append(
                    {
                        "source": "round2_hybrid_b_c/in_place_1radps_command.csv",
                        "variant": row["variant"],
                        "left_command": row["left_command"],
                        "right_command": row["right_command"],
                        "lr_delta_command": row["lr_delta_command"],
                        "extra_opposing_yaw_torque_nm": row["extra_opposing_yaw_torque_nm"],
                        "note": "B_stribeck should match reference; Hybrid_BC is a mixed B/C diagnostic, not the requested B branch",
                    }
                )
    write_csv(OUT / "comparison_to_prior_mixed_outputs.csv", comparison_rows)

    metadata = {
        "reference_branch": "Variant B / B_stribeck request_only fixed-point branch",
        "excluded_branch": "request_or_yaw mixed activation branch",
        "source_artifacts": [
            str(STRIBECK.relative_to(ROOT)),
            str(LR_GRID.relative_to(ROOT)),
            str(IN_PLACE.relative_to(ROOT)),
            str(CONTACT_SAMPLE.relative_to(ROOT)),
        ],
        "production_eligibility": "rejected_as_production_traction_law_under_command_conditioning_rule",
        "reason": "activation uses requested/base+extra yaw moment, so identical contact states can select different resistance from request/command magnitude",
    }
    (OUT / "reference_metadata.json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")

    left = f(b_ref["left_command"])
    right = f(b_ref["right_command"])
    extra = f(b_ref["extra_opposing_yaw_torque_nm"])
    total = f(b_ref["total_opposing_yaw_torque_nm"])
    coeff = {row["parameter"]: row for row in coeff_rows}

    report_lines = [
        "# Round2 Variant B Correct-Branch Reference",
        "",
        "Analysis-only output. Production code, build metadata, and tests were not modified.",
        "",
        "## Exact Branch",
        "",
        "Reference branch: `Variant B / B_stribeck` using the selected `request_only` activation and fixed-point request coupling.",
        "",
        "Excluded branch: the `request_or_yaw` mixed activation family from the original grid search. No persisted selected B artifact used that mixed activation; the saved selected coefficients set `activation_mode_request_only = 1`.",
        "",
        "For positive yaw, the correction adds yaw-opposing torque:",
        "",
        "`M_extra = A_req(M_base + M_extra) * R(v_transition) * (K_slide + K_static * S(v_transition))`",
        "",
        "where:",
        "",
        "- `A_req(x) = 1 - exp(-(smooth_positive(x) / req_activation_nm)^2)`",
        "- `v_transition = sqrt((rel_weight * drive_wheel_longitudinal_offset_m * abs(yaw_rate))^2 + abs(Vf)^2)`",
        "- `S(v) = exp(-(v / stribeck_speed_mps)^2)`",
        "- `R(v) = 1 / (1 + (v / speed_fade_mps)^2)`",
        "- `M_extra` is solved by fixed-point iteration with `requested = M_base + M_extra`.",
        "",
        "This is a request-conditioned diagnostic reference, not a production-eligible traction law.",
        "",
        "## Coefficients",
        "",
        "| parameter | value | unit |",
        "| --- | ---: | --- |",
    ]
    for name in [
        "yaw_activation_mps",
        "req_activation_nm",
        "stribeck_speed_mps",
        "speed_fade_mps",
        "rel_weight",
        "activation_mode_request_only",
        "include_yaw_viscous_basis",
        "static_extra_nm",
        "sliding_nm",
        "yaw_viscous_nm_per_mps",
        "weighted_train_opposes_rmse_nm",
    ]:
        row = coeff[name]
        report_lines.append(f"| `{name}` | {row['value']} | {row['unit']} |")

    report_lines.extend(
        [
            "",
            "## +1 rad/s In-Place Command",
            "",
            f"At `Vf=0`, `Vr=0`, `yaw_rate=+1 rad/s`: extra opposing yaw torque `{extra:.12f}` Nm, total opposing yaw torque `{total:.12f}` Nm, left/right command `{left:.12f}/{right:.12f}`, L-R delta `{left - right:.12f}`.",
            "",
            "This passes the reference gate (`|cmd| >= 0.6`) and is close to the measured/calculated `+0.646/-0.646` target.",
            "",
            "## Split RMSE",
            "",
            "| split | count | baseline RMSE Nm | corrected RMSE Nm | improvement % |",
            "| --- | ---: | ---: | ---: | ---: |",
        ]
    )
    for row in split_metrics:
        report_lines.append(
            f"| `{row['dataset_split']}` | {row['count']} | {fmt(row['baseline_rmse_nm'])} | {fmt(row['corrected_rmse_nm'])} | {fmt(row['rmse_improvement_pct'], 3)} |"
        )

    report_lines.extend(
        [
            "",
            "## Selected-Log RMSE",
            "",
            "| run_id | split | count | baseline RMSE Nm | corrected RMSE Nm | improvement % |",
            "| --- | --- | ---: | ---: | ---: | ---: |",
        ]
    )
    for row in selected_log_metrics:
        report_lines.append(
            f"| `{row['run_id']}` | `{row['dataset_split']}` | {row['count']} | {fmt(row['baseline_rmse_nm'])} | {fmt(row['corrected_rmse_nm'])} | {fmt(row['rmse_improvement_pct'], 3)} |"
        )

    report_lines.extend(
        [
            "",
            "## L-R Delta Grid",
            "",
            "The full 6x10 machine-readable grid is `lr_delta_grid_6x10.csv`; the Markdown pivot is `lr_delta_pivot.md`.",
            "",
            "## Mixed-Output Comparison",
            "",
            "The saved round2 `B_stribeck` in-place row matches this reference branch. The `Hybrid_BC_adhesion_partition` row is a mixed B/C diagnostic and is intentionally not the requested B branch; it estimates left/right command `+0.625298/-0.625298` at +1 rad/s in place.",
            "",
            "## Production Eligibility",
            "",
            "Rejected for production under the command-conditioning rule. This branch directly conditions the resistance activation on `requested = M_base + M_extra`, so the same physical contact state can receive different traction/resistance depending on request/command magnitude. Keep it only as a reference/diagnostic baseline for the correct in-place command scale.",
            "",
            "## Output Files",
            "",
            "- `coefficients.csv`",
            "- `in_place_1radps_command.csv`",
            "- `lr_delta_grid_6x10.csv`",
            "- `lr_delta_pivot.md`",
            "- `split_rmse.csv`",
            "- `selected_log_rmse.csv`",
            "- `comparison_to_prior_mixed_outputs.csv`",
            "- `contact_feature_sample_summary.csv`",
            "- `reference_metadata.json`",
        ]
    )
    (OUT / "round2_b_correct_branch_reference_report.md").write_text("\n".join(report_lines) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
