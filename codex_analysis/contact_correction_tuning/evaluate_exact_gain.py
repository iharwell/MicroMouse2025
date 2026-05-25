#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import sys
from dataclasses import replace
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
OUT_DIR = Path(__file__).resolve().parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from codex_analysis.contact_correction_log_eval import evaluate_contact_correction as replay  # noqa: E402
from codex_analysis.contact_correction_tuning import tune_contact_correction as tuning  # noqa: E402


def write_csv(path: Path, rows: list[dict[str, str]], fieldnames: list[str] | None = None) -> None:
    if fieldnames is None:
        fieldnames = list(rows[0].keys()) if rows else ["dataset"]
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Exact replay for one contact-continuum gain.")
    parser.add_argument("--gain", type=float, required=True)
    parser.add_argument("--out-dir", type=Path, default=OUT_DIR)
    parser.add_argument("--min-bin-count", type=int, default=80)
    parser.add_argument("--sample-every", type=int, default=200)
    parser.add_argument("--no-competition", action="store_true")
    parser.add_argument("--include-uncertainty", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)
    base_params = replay.source_params()
    params = replace(base_params, contact_yaw_patch_force_gain_ns_per_m=args.gain)
    runs, samples = replay.collect_samples(args, params)
    aggregate = tuning.exact_aggregate(samples)
    motion = tuning.exact_motion(samples)
    suffix = f"{args.gain:.6f}".replace("-", "neg_").replace(".", "p")
    write_csv(args.out_dir / f"exact_gain_{suffix}_aggregate_rmse.csv", aggregate)
    write_csv(
        args.out_dir / f"exact_gain_{suffix}_motion_rmse.csv",
        motion,
        ["family", "motion_class"] + [key for key in motion[0].keys() if key not in {"family", "motion_class"}],
    )
    write_csv(
        args.out_dir / f"exact_gain_{suffix}_per_run_rmse.csv",
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
            for run in runs
            if run.samples > 0
        ],
    )
    print(f"gain={args.gain:.9f}")
    print(f"samples={len(samples)}")
    print(f"aggregate={args.out_dir / f'exact_gain_{suffix}_aggregate_rmse.csv'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
