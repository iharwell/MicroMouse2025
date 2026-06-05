#!/usr/bin/env python3
"""Print high-pass-filtered raw accel-X RMS from the current open-floor mmlog."""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from pathlib import Path

sys.dont_write_bytecode = True

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from decode_mmlog_to_csv import MMLogError
from decode_mmlog_to_csv import SidecarSpec
from decode_mmlog_to_csv import iter_rows
from decode_mmlog_to_csv import parse_sidecar
from decode_mmlog_to_csv import read_mmlog_header_and_payload
from decode_mmlog_to_csv import resolve_sidecar_path


DEFAULT_MMLOG_PATH = Path(r"D:\open_floor_main.mmlog")
DEFAULT_ACCEL_FIELD = "imu_accel_x"
DEFAULT_TIME_FIELD = "master_time_us"
DEFAULT_IGNORE_TAIL_S = 1.0
UINT32_MODULUS = 1 << 32


@dataclass(frozen=True)
class RmsResult:
    rms: float
    sample_count: int
    elapsed_start_s: float
    elapsed_end_s: float
    log_end_s: float
    ignored_tail_s: float


def field_index(spec: SidecarSpec, field_name: str) -> int:
    for index, field in enumerate(spec.fields):
        if field.field_name == field_name:
            return index
    available = ", ".join(field.field_name for field in spec.fields)
    raise MMLogError(f"Field {field_name!r} not found in sidecar. Available fields: {available}")


def elapsed_u32_seconds(current_us: int, start_us: int) -> float:
    if current_us >= start_us:
        return 1.0e-6 * (current_us - start_us)
    return 1.0e-6 * (current_us + UINT32_MODULUS - start_us)


def delta_u32_seconds(current_us: int, previous_us: int) -> float:
    if current_us >= previous_us:
        return 1.0e-6 * (current_us - previous_us)
    return 1.0e-6 * (current_us + UINT32_MODULUS - previous_us)


def log_duration_seconds(payload: bytes, spec: SidecarSpec, time_index: int, mmlog_path: Path) -> float:
    first_time_us: int | None = None
    last_time_us: int | None = None
    for row in iter_rows(payload, spec):
        current_time_us = int(row[time_index])
        if first_time_us is None:
            first_time_us = current_time_us
        last_time_us = current_time_us

    if first_time_us is None or last_time_us is None:
        raise MMLogError(f"No rows found in {mmlog_path}")
    return elapsed_u32_seconds(last_time_us, first_time_us)


def high_pass_rms_from_mmlog(
    mmlog_path: Path,
    accel_field_name: str,
    time_field_name: str,
    cutoff_hz: float,
    start_s: float,
    ignore_tail_s: float,
) -> RmsResult:
    if cutoff_hz <= 0.0:
        raise ValueError("cutoff_hz must be positive")
    if start_s < 0.0:
        raise ValueError("start_s must be non-negative")
    if ignore_tail_s < 0.0:
        raise ValueError("ignore_tail_s must be non-negative")

    sidecar_ref, payload = read_mmlog_header_and_payload(mmlog_path)
    sidecar_path = resolve_sidecar_path(mmlog_path, sidecar_ref)
    spec = parse_sidecar(sidecar_path, require_string_hash_metadata=False)

    accel_index = field_index(spec, accel_field_name)
    time_index = field_index(spec, time_field_name)
    accel_field = spec.fields[accel_index]
    if accel_field.type_name != "i16":
        raise MMLogError(
            f"Field {accel_field_name!r} is {accel_field.type_name}, expected i16 raw accelerometer data"
        )

    log_end_s = log_duration_seconds(payload, spec, time_index, mmlog_path)
    rms_end_s = log_end_s - ignore_tail_s
    if rms_end_s < start_s:
        raise MMLogError(
            f"No RMS window remains after ignoring the last {ignore_tail_s:.6g} s; "
            f"log duration is {log_end_s:.6g} s and start is {start_s:.6g} s"
        )

    rc_s = 1.0 / (2.0 * math.pi * cutoff_hz)
    first_time_us: int | None = None
    previous_time_us: int | None = None
    previous_input: float | None = None
    previous_output = 0.0
    sum_squares = 0.0
    rms_sample_count = 0

    for row in iter_rows(payload, spec):
        current_time_us = int(row[time_index])
        current_input = float(row[accel_index])

        if first_time_us is None:
            first_time_us = current_time_us
            previous_time_us = current_time_us
            previous_input = current_input
            if start_s <= 0.0 <= rms_end_s:
                rms_sample_count = 1
            continue

        assert previous_time_us is not None
        assert previous_input is not None

        dt_s = delta_u32_seconds(current_time_us, previous_time_us)
        if dt_s <= 0.0:
            raise MMLogError(
                f"Non-advancing timestamp at {current_time_us} us after {previous_time_us} us"
            )

        alpha = rc_s / (rc_s + dt_s)
        current_output = alpha * (previous_output + current_input - previous_input)
        elapsed_s = elapsed_u32_seconds(current_time_us, first_time_us)

        if start_s <= elapsed_s <= rms_end_s:
            sum_squares += current_output * current_output
            rms_sample_count += 1

        previous_time_us = current_time_us
        previous_input = current_input
        previous_output = current_output

    if first_time_us is None:
        raise MMLogError(f"No rows found in {mmlog_path}")
    if rms_sample_count == 0:
        raise MMLogError(
            f"No samples from {start_s:.6g} s through {rms_end_s:.6g} s; "
            f"log duration is {log_end_s:.6g} s"
        )

    return RmsResult(
        rms=math.sqrt(sum_squares / rms_sample_count),
        sample_count=rms_sample_count,
        elapsed_start_s=start_s,
        elapsed_end_s=rms_end_s,
        log_end_s=log_end_s,
        ignored_tail_s=ignore_tail_s,
    )


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Print the RMS of raw accel-X after a 100 Hz high-pass filter, "
            "using D:\\open_floor_main.mmlog and ignoring the final second by default."
        )
    )
    parser.add_argument(
        "--mmlog",
        type=Path,
        default=DEFAULT_MMLOG_PATH,
        help=r"Path to the source .mmlog file. Default: D:\open_floor_main.mmlog",
    )
    parser.add_argument(
        "--field",
        default=DEFAULT_ACCEL_FIELD,
        help=f"Raw accel field name after the type prefix is stripped. Default: {DEFAULT_ACCEL_FIELD}",
    )
    parser.add_argument(
        "--time-field",
        default=DEFAULT_TIME_FIELD,
        help=f"Timestamp field name after the type prefix is stripped. Default: {DEFAULT_TIME_FIELD}",
    )
    parser.add_argument(
        "--cutoff-hz",
        type=float,
        default=100.0,
        help="High-pass cutoff frequency. Default: 100.0",
    )
    parser.add_argument(
        "--start-s",
        type=float,
        default=2.0,
        help="Elapsed log time where RMS accumulation starts. Default: 2.0",
    )
    parser.add_argument(
        "--ignore-tail-s",
        type=float,
        default=DEFAULT_IGNORE_TAIL_S,
        help="Seconds to ignore at the end of the log. Default: 1.0",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Print source and sample-count context in addition to the RMS value.",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    try:
        result = high_pass_rms_from_mmlog(
            mmlog_path=args.mmlog,
            accel_field_name=args.field,
            time_field_name=args.time_field,
            cutoff_hz=args.cutoff_hz,
            start_s=args.start_s,
            ignore_tail_s=args.ignore_tail_s,
        )
    except (OSError, MMLogError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2

    if args.verbose:
        print(f"mmlog={args.mmlog}")
        print(f"field={args.field}")
        print(f"cutoff_hz={args.cutoff_hz:.6g}")
        print(f"ignored_tail_s={result.ignored_tail_s:.6g}")
        print(f"log_end_s={result.log_end_s:.6g}")
        print(f"rms_window_s={result.elapsed_start_s:.6g}..{result.elapsed_end_s:.6g}")
        print(f"samples={result.sample_count}")
        print(f"rms={result.rms:.6f}")
    else:
        print(f"{result.rms:.6f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
