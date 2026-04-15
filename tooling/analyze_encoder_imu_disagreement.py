from __future__ import annotations

import argparse
import csv
import math
import statistics
from dataclasses import dataclass
from pathlib import Path


GYRO_TURN_THRESHOLD_RADPS = 0.25
ENCODER_DIFF_THRESHOLD_MPS = 0.02
LINEAR_SPEED_THRESHOLD_MPS = 0.10
MODERN_ACCEL_X_THRESHOLD = 0.50


DERIVED_PATH_FRAGMENTS = (
    "open_floor_ukf_replay",
    "yaw_spec_analysis",
    "\\segments\\",
    "\\phases\\",
)


@dataclass
class LogSummary:
    family: str
    log_kind: str
    path: Path
    run_id: str | None
    row_count: int
    raw_left_minus_right_gyro_corr: float | None
    raw_left_minus_right_accel_corr: float | None
    raw_gyro_mismatch_fraction: float | None
    raw_accel_mismatch_fraction: float | None
    raw_positive_gyro_right_faster_fraction: float | None
    gyro_turn_sample_count: int
    accel_turn_sample_count: int
    inferred_gyro_turn_sign: str | None
    accel_source: str | None
    notes: str | None


def correlation(xs: list[float], ys: list[float]) -> float | None:
    if len(xs) < 2 or len(xs) != len(ys):
        return None
    mean_x = statistics.fmean(xs)
    mean_y = statistics.fmean(ys)
    sum_xx = sum((x - mean_x) * (x - mean_x) for x in xs)
    sum_yy = sum((y - mean_y) * (y - mean_y) for y in ys)
    if sum_xx <= 0.0 or sum_yy <= 0.0:
        return None
    sum_xy = sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys))
    return sum_xy / math.sqrt(sum_xx * sum_yy)


def has_derived_fragment(path: Path) -> bool:
    normalized = str(path).lower().replace("/", "\\")
    return any(fragment in normalized for fragment in DERIVED_PATH_FRAGMENTS)


def discover_modern_logs(root: Path) -> list[tuple[str, Path]]:
    discovered: list[tuple[str, Path]] = []
    for name, kind in (
        ("open_floor_main.csv", "open_floor"),
        ("top_speed_measurement_main.csv", "top_speed"),
    ):
        for path in sorted(root.rglob(name)):
            if has_derived_fragment(path):
                continue
            discovered.append((kind, path))
    return discovered


def discover_legacy_logs(root: Path) -> list[Path]:
    legacy_root = root / "Competition Testing Data"
    if not legacy_root.is_dir():
        return []
    return sorted(legacy_root.glob("*.csv"))


def parse_run_id_from_logging(log_path: Path, log_kind: str) -> str | None:
    if not log_path.is_file():
        return None
    target_prefix = "open_floor_measurement" if log_kind == "open_floor" else "top_speed_measurement_main"
    for line in log_path.read_text(encoding="utf-8", errors="replace").splitlines():
        if target_prefix not in line or "run_start:" not in line or "run_id=" not in line:
            continue
        for part in line.split(";"):
            part = part.strip()
            if "run_id=" in part:
                value = part.split("run_id=", 1)[1].strip()
                if value:
                    return value
    return None


def analyze_rows(
    row_count: int,
    left_velocity_values: list[float],
    right_velocity_values: list[float],
    gyro_values: list[float],
    linear_speed_values: list[float],
    accel_values: list[float] | None,
    accel_threshold: float | None,
    accel_source: str | None,
) -> tuple[float | None, float | None, float | None, float | None, float | None, int, int, str | None]:
    encoder_turn_values: list[float] = []
    accel_encoder_turn_values: list[float] = []
    accel_signed_values: list[float] = []

    gyro_turn_sample_count = 0
    accel_turn_sample_count = 0
    gyro_sign_mismatch_count = 0
    accel_sign_mismatch_count = 0
    positive_gyro_sample_count = 0
    positive_gyro_right_faster_count = 0

    for index, (left_velocity_mps, right_velocity_mps, gyro_radps, linear_speed_mps) in enumerate(
        zip(left_velocity_values, right_velocity_values, gyro_values, linear_speed_values)
    ):
        if not all(math.isfinite(value) for value in (left_velocity_mps, right_velocity_mps, gyro_radps, linear_speed_mps)):
            continue
        left_minus_right_mps = left_velocity_mps - right_velocity_mps
        encoder_turn_values.append(left_minus_right_mps)

        if abs(gyro_radps) >= GYRO_TURN_THRESHOLD_RADPS and abs(left_minus_right_mps) >= ENCODER_DIFF_THRESHOLD_MPS:
            gyro_turn_sample_count += 1
            if math.copysign(1.0, left_minus_right_mps) != math.copysign(1.0, gyro_radps):
                gyro_sign_mismatch_count += 1

        if gyro_radps >= GYRO_TURN_THRESHOLD_RADPS:
            positive_gyro_sample_count += 1
            if (right_velocity_mps - left_velocity_mps) >= ENCODER_DIFF_THRESHOLD_MPS:
                positive_gyro_right_faster_count += 1

        if accel_values is None or index >= len(accel_values):
            continue

        accel_x_value = accel_values[index]
        if not math.isfinite(accel_x_value) or abs(linear_speed_mps) < LINEAR_SPEED_THRESHOLD_MPS:
            continue

        signed_accel_x = accel_x_value if linear_speed_mps >= 0.0 else -accel_x_value
        accel_encoder_turn_values.append(left_minus_right_mps)
        accel_signed_values.append(signed_accel_x)

        if accel_threshold is not None and abs(accel_x_value) >= accel_threshold and abs(left_minus_right_mps) >= ENCODER_DIFF_THRESHOLD_MPS:
            accel_turn_sample_count += 1
            if math.copysign(1.0, left_minus_right_mps) != math.copysign(1.0, signed_accel_x):
                accel_sign_mismatch_count += 1

    raw_corr = correlation(encoder_turn_values, gyro_values)
    raw_accel_corr = correlation(accel_encoder_turn_values, accel_signed_values)
    inferred_turn_sign = None
    if raw_corr is not None:
        inferred_turn_sign = "left_minus_right" if raw_corr >= 0.0 else "right_minus_left"

    return (
        raw_corr,
        raw_accel_corr,
        (gyro_sign_mismatch_count / gyro_turn_sample_count) if gyro_turn_sample_count else None,
        (accel_sign_mismatch_count / accel_turn_sample_count) if accel_turn_sample_count else None,
        (positive_gyro_right_faster_count / positive_gyro_sample_count) if positive_gyro_sample_count else None,
        gyro_turn_sample_count,
        accel_turn_sample_count,
        inferred_turn_sign,
    )


def analyze_modern_log(log_kind: str, path: Path) -> LogSummary:
    with path.open(newline="", encoding="utf-8", errors="replace") as csv_file:
        rows = list(csv.DictReader(csv_file))

    left_velocity_values: list[float] = []
    right_velocity_values: list[float] = []
    gyro_values: list[float] = []
    linear_speed_values: list[float] = []
    accel_values: list[float] = []

    for row in rows:
        try:
            left_velocity_values.append(float(row["left_encoder_velocity_mps"]))
            right_velocity_values.append(float(row["right_encoder_velocity_mps"]))
            gyro_values.append(float(row["gyro_raw_radps"]))
            linear_speed_values.append(float(row["measured_linear_speed_mps"]))
            accel_values.append(float(row["accel_body_x_mps2"]))
        except (KeyError, ValueError):
            continue

    (
        raw_corr,
        raw_accel_corr,
        raw_gyro_mismatch_fraction,
        raw_accel_mismatch_fraction,
        raw_positive_gyro_right_faster_fraction,
        gyro_turn_sample_count,
        accel_turn_sample_count,
        inferred_turn_sign,
    ) = analyze_rows(
        row_count=len(rows),
        left_velocity_values=left_velocity_values,
        right_velocity_values=right_velocity_values,
        gyro_values=gyro_values,
        linear_speed_values=linear_speed_values,
        accel_values=accel_values,
        accel_threshold=MODERN_ACCEL_X_THRESHOLD,
        accel_source="accel_body_x_mps2",
    )

    return LogSummary(
        family="modern",
        log_kind=log_kind,
        path=path,
        run_id=parse_run_id_from_logging(path.with_name("logging.txt"), log_kind),
        row_count=len(rows),
        raw_left_minus_right_gyro_corr=raw_corr,
        raw_left_minus_right_accel_corr=raw_accel_corr,
        raw_gyro_mismatch_fraction=raw_gyro_mismatch_fraction,
        raw_accel_mismatch_fraction=raw_accel_mismatch_fraction,
        raw_positive_gyro_right_faster_fraction=raw_positive_gyro_right_faster_fraction,
        gyro_turn_sample_count=gyro_turn_sample_count,
        accel_turn_sample_count=accel_turn_sample_count,
        inferred_gyro_turn_sign=inferred_turn_sign,
        accel_source="accel_body_x_mps2",
        notes=None,
    )


def load_legacy_rows(path: Path) -> list[dict[str, str]]:
    header: list[str] | None = None
    rows: list[dict[str, str]] = []
    with path.open(newline="", encoding="utf-8", errors="replace") as csv_file:
        reader = csv.reader(csv_file)
        for row in reader:
            if not row:
                continue
            if row[0] == "sample":
                header = row
                continue
            if header is None or row[0].startswith("#") or len(row) != len(header):
                continue
            rows.append(dict(zip(header, row)))
    return rows


def analyze_legacy_log(path: Path) -> LogSummary | None:
    rows = load_legacy_rows(path)
    if not rows:
        return None

    required_columns = {
        "left_velocity_mps",
        "right_velocity_mps",
        "gyro_raw_radps",
        "linear_speed_mps",
    }
    if not required_columns.issubset(rows[0].keys()):
        return None

    left_velocity_values: list[float] = []
    right_velocity_values: list[float] = []
    gyro_values: list[float] = []
    linear_speed_values: list[float] = []
    accel_values: list[float] = []
    accel_available = "imu_bl_accel_x" in rows[0]

    for row in rows:
        try:
            left_velocity_values.append(float(row["left_velocity_mps"]))
            right_velocity_values.append(float(row["right_velocity_mps"]))
            gyro_values.append(float(row["gyro_raw_radps"]))
            linear_speed_values.append(float(row["linear_speed_mps"]))
            if accel_available:
                accel_values.append(float(row["imu_bl_accel_x"]))
        except ValueError:
            continue

    (
        raw_corr,
        raw_accel_corr,
        raw_gyro_mismatch_fraction,
        _raw_accel_mismatch_fraction,
        raw_positive_gyro_right_faster_fraction,
        gyro_turn_sample_count,
        _accel_turn_sample_count,
        inferred_turn_sign,
    ) = analyze_rows(
        row_count=len(rows),
        left_velocity_values=left_velocity_values,
        right_velocity_values=right_velocity_values,
        gyro_values=gyro_values,
        linear_speed_values=linear_speed_values,
        accel_values=accel_values if accel_available else None,
        accel_threshold=None,
        accel_source="imu_bl_accel_x_raw" if accel_available else None,
    )

    return LogSummary(
        family="legacy",
        log_kind="competition_motion_csv",
        path=path,
        run_id=None,
        row_count=len(rows),
        raw_left_minus_right_gyro_corr=raw_corr,
        raw_left_minus_right_accel_corr=raw_accel_corr,
        raw_gyro_mismatch_fraction=raw_gyro_mismatch_fraction,
        raw_accel_mismatch_fraction=None,
        raw_positive_gyro_right_faster_fraction=raw_positive_gyro_right_faster_fraction,
        gyro_turn_sample_count=gyro_turn_sample_count,
        accel_turn_sample_count=0,
        inferred_gyro_turn_sign=inferred_turn_sign,
        accel_source="imu_bl_accel_x_raw" if accel_available else None,
        notes="legacy competition archive uses raw IMU accel-x counts and appears to use a different yaw sign convention"
        if inferred_turn_sign == "right_minus_left"
        else None,
    )


def format_float(value: float | None) -> str:
    return "" if value is None else f"{value:.6f}"


def write_summary_csv(output_path: Path, summaries: list[LogSummary]) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(
            [
                "family",
                "log_kind",
                "path",
                "run_id",
                "row_count",
                "raw_left_minus_right_gyro_corr",
                "raw_left_minus_right_accel_corr",
                "raw_gyro_mismatch_fraction",
                "raw_accel_mismatch_fraction",
                "raw_positive_gyro_right_faster_fraction",
                "gyro_turn_sample_count",
                "accel_turn_sample_count",
                "inferred_gyro_turn_sign",
                "accel_source",
                "notes",
            ]
        )
        for summary in summaries:
            writer.writerow(
                [
                    summary.family,
                    summary.log_kind,
                    str(summary.path),
                    "" if summary.run_id is None else summary.run_id,
                    summary.row_count,
                    format_float(summary.raw_left_minus_right_gyro_corr),
                    format_float(summary.raw_left_minus_right_accel_corr),
                    format_float(summary.raw_gyro_mismatch_fraction),
                    format_float(summary.raw_accel_mismatch_fraction),
                    format_float(summary.raw_positive_gyro_right_faster_fraction),
                    summary.gyro_turn_sample_count,
                    summary.accel_turn_sample_count,
                    "" if summary.inferred_gyro_turn_sign is None else summary.inferred_gyro_turn_sign,
                    "" if summary.accel_source is None else summary.accel_source,
                    "" if summary.notes is None else summary.notes,
                ]
            )


def print_report(summaries: list[LogSummary]) -> None:
    modern_open_floor = [summary for summary in summaries if summary.family == "modern" and summary.log_kind == "open_floor"]
    modern_top_speed = [summary for summary in summaries if summary.family == "modern" and summary.log_kind == "top_speed"]
    legacy = [summary for summary in summaries if summary.family == "legacy"]

    print(f"Analyzed {len(summaries)} motion logs")
    print(f"  modern open-floor logs: {len(modern_open_floor)}")
    print(f"  modern top-speed logs:  {len(modern_top_speed)}")
    print(f"  legacy competition logs: {len(legacy)}")

    if modern_top_speed:
        print("\nModern top-speed logs:")
        for summary in sorted(modern_top_speed, key=lambda item: item.path.parent.name):
            print(
                "  "
                f"{summary.path.parent.name} "
                f"run_id={summary.run_id or '?'} "
                f"gyro_mismatch={format_float(summary.raw_gyro_mismatch_fraction) or 'n/a'} "
                f"enc_gyro_corr={format_float(summary.raw_left_minus_right_gyro_corr) or 'n/a'} "
                f"accel_mismatch={format_float(summary.raw_accel_mismatch_fraction) or 'n/a'} "
                f"enc_accel_corr={format_float(summary.raw_left_minus_right_accel_corr) or 'n/a'} "
                f"posgyro_rightfaster={format_float(summary.raw_positive_gyro_right_faster_fraction) or 'n/a'}"
            )

    if modern_open_floor:
        median_open_floor_corr = statistics.median(
            summary.raw_left_minus_right_gyro_corr
            for summary in modern_open_floor
            if summary.raw_left_minus_right_gyro_corr is not None
        )
        median_open_floor_mismatch = statistics.median(
            summary.raw_gyro_mismatch_fraction
            for summary in modern_open_floor
            if summary.raw_gyro_mismatch_fraction is not None
        )
        print("\nModern open-floor median:")
        print(f"  raw encoder-vs-gyro corr: {median_open_floor_corr:.6f}")
        print(f"  raw gyro mismatch fraction: {median_open_floor_mismatch:.6f}")

    if legacy:
        left_minus_right_count = sum(
            1 for summary in legacy if summary.inferred_gyro_turn_sign == "left_minus_right"
        )
        right_minus_left_count = sum(
            1 for summary in legacy if summary.inferred_gyro_turn_sign == "right_minus_left"
        )
        print("\nLegacy competition archive inferred yaw sign:")
        print(f"  left_minus_right:  {left_minus_right_count}")
        print(f"  right_minus_left: {right_minus_left_count}")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Summarize encoder-vs-gyro and encoder-vs-accel disagreement across archived motion logs."
    )
    repo_root = Path(__file__).resolve().parents[1]
    parser.add_argument(
        "--root",
        type=Path,
        default=repo_root / "TestResults",
        help="Root directory containing decoded TestResults logs.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=repo_root / "TestResults" / "encoder_imu_disagreement_summary.csv",
        help="CSV path for the per-log summary output.",
    )
    args = parser.parse_args()

    summaries: list[LogSummary] = []
    for log_kind, path in discover_modern_logs(args.root):
        summaries.append(analyze_modern_log(log_kind, path))
    for path in discover_legacy_logs(args.root):
        summary = analyze_legacy_log(path)
        if summary is not None:
            summaries.append(summary)

    summaries.sort(key=lambda item: (item.family, item.log_kind, str(item.path)))
    write_summary_csv(args.output, summaries)
    print_report(summaries)
    print(f"\nWrote {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
