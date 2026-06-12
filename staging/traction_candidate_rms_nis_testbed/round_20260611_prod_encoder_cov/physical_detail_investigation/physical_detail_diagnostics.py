import csv
import json
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[4]
OUT_DIR = Path(__file__).resolve().parent
STATIC_LOG = ROOT / "TestResults/mmlog_decode_2026-06-08_23-37-25/open_floor_main.csv"
DRIFT_METRICS = (
    ROOT
    / "staging/traction_candidate_rms_nis_testbed/round_20260611_prod_encoder_cov/"
    / "shear_rate/static_position_drift/static_position_drift_metrics.csv"
)
LAUNCH_ITEMIZED = (
    ROOT
    / "staging/traction_candidate_rms_nis_testbed/round_20260611_prod_encoder_cov/"
    / "shear_rate/refined_assessment_ekf/itemized_rms_nis.csv"
)
STATIC_SUMMARY = (
    ROOT
    / "staging/traction_candidate_rms_nis_testbed/round_20260611_prod_encoder_cov/"
    / "shear_rate/static_position_drift/static_position_drift_summary.json"
)
STATIC_MANIFEST = (
    ROOT
    / "staging/traction_candidate_rms_nis_testbed/round_20260611/"
    / "static_stability_analysis/selected_static_segment_manifest.json"
)


def finite_float(row, name):
    try:
        value = float(row[name])
    except (KeyError, TypeError, ValueError):
        return None
    if not math.isfinite(value):
        return None
    return value


def stats(values):
    values = [v for v in values if v is not None and math.isfinite(v)]
    n = len(values)
    if n == 0:
        return {"count": 0}
    mean = sum(values) / n
    centered = [v - mean for v in values]
    rms = math.sqrt(sum(v * v for v in values) / n)
    std = math.sqrt(sum(v * v for v in centered) / n)
    sorted_values = sorted(values)
    def percentile(p):
        if n == 1:
            return sorted_values[0]
        idx = (n - 1) * p / 100.0
        lo = math.floor(idx)
        hi = math.ceil(idx)
        if lo == hi:
            return sorted_values[lo]
        return sorted_values[lo] * (hi - idx) + sorted_values[hi] * (idx - lo)

    return {
        "count": n,
        "mean": mean,
        "rms": rms,
        "std": std,
        "p50_abs": percentile_abs(values, 50),
        "p95_abs": percentile_abs(values, 95),
        "p99_abs": percentile_abs(values, 99),
        "max_abs": max(abs(v) for v in values),
    }


def percentile_abs(values, p):
    vals = sorted(abs(v) for v in values if v is not None and math.isfinite(v))
    n = len(vals)
    if n == 0:
        return None
    if n == 1:
        return vals[0]
    idx = (n - 1) * p / 100.0
    lo = math.floor(idx)
    hi = math.ceil(idx)
    if lo == hi:
        return vals[lo]
    return vals[lo] * (hi - idx) + vals[hi] * (idx - lo)


def first_difference(values):
    out = []
    prev = None
    for value in values:
        if value is not None and prev is not None:
            out.append(value - prev)
        if value is not None:
            prev = value
    return out


def load_static_sensor_rows():
    channels = {
        "left_encoder_wheel_speed_radps": [],
        "right_encoder_wheel_speed_radps": [],
        "left_encoder_velocity_mps": [],
        "right_encoder_velocity_mps": [],
        "gyro_radps": [],
        "accel_body_right_mps2": [],
        "accel_body_forward_mps2": [],
        "left_encoder_distance_m": [],
        "right_encoder_distance_m": [],
    }
    times = []
    manifest = json.loads(STATIC_MANIFEST.read_text())
    segment = manifest["segments"][0]
    start_index = int(segment["segment_start_row_index"])
    end_index = int(segment["segment_end_row_index"])

    with STATIC_LOG.open(newline="") as handle:
        reader = csv.DictReader(handle)
        for index, row in enumerate(reader):
            if index < start_index:
                continue
            if index > end_index:
                break
            times.append(finite_float(row, "master_time_us"))
            for name in channels:
                channels[name].append(finite_float(row, name))
    return times, channels, segment


def load_csv_rows(path):
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def main():
    times, channels, segment = load_static_sensor_rows()
    duration_s = (times[-1] - times[0]) * 1e-6 if len(times) > 1 else None
    dt_values = [
        (b - a) * 1e-6 for a, b in zip(times, times[1:]) if a is not None and b is not None
    ]

    sensor_stats = {name: stats(values) for name, values in channels.items()}
    sensor_stats["encoder_distance_first_difference_m"] = {
        "left": stats(first_difference(channels["left_encoder_distance_m"])),
        "right": stats(first_difference(channels["right_encoder_distance_m"])),
    }
    sensor_stats["sample_dt_s"] = stats(dt_values)

    drift_rows = load_csv_rows(DRIFT_METRICS)
    drift_by_case = {
        row["case_id"]: {
            "final_radial_m": float(row["final_radial_m"]),
            "final_x_m": float(row["final_x_m"]),
            "final_y_m": float(row["final_y_m"]),
            "yaw_drift_rad": float(row["yaw_drift_rad"]),
            "passes_5mm": row["passes_5mm"],
        }
        for row in drift_rows
        if row.get("estimator") == "simplex_ukf" and row.get("candidate_id") == "shear_rate"
    }

    launch_rows = load_csv_rows(LAUNCH_ITEMIZED)
    yaw_launch = [
        row
        for row in launch_rows
        if row.get("stage") == "SEC_40_YAW_LAUNCH"
        or row.get("stage") == "yaw_launch"
        or "yaw" in row.get("stage", "").lower()
    ]
    yaw_launch_summary = {}
    for dim in ("forward_accel_nis", "right_accel_nis", "yaw_rate_nis"):
        physical = [
            float(row["physical_residual_rms"])
            for row in yaw_launch
            if row.get("log_parameter") == dim
            and row.get("physical_residual_rms")
            and row["physical_residual_rms"].lower() != "nan"
        ]
        guarded = [
            float(row["guarded_rms_nis"])
            for row in yaw_launch
            if row.get("log_parameter") == dim
            and row.get("guarded_rms_nis")
            and row["guarded_rms_nis"].lower() != "nan"
        ]
        yaw_launch_summary[dim] = {
            "physical_residual_rms_stats": stats(physical),
            "guarded_rms_nis_stats": stats(guarded),
        }

    lateral_compliance_m = 0.0006
    # A 0.6 mm side deflection over a 1 ms differentiated sample can look enormous
    # as acceleration, but over launch-scale windows it is small. Report both.
    compliance_scale = {
        "deflection_m": lateral_compliance_m,
        "equivalent_accel_if_released_over_1ms_mps2": 2 * lateral_compliance_m / (0.001**2),
        "equivalent_accel_if_released_over_10ms_mps2": 2 * lateral_compliance_m / (0.010**2),
        "equivalent_accel_if_released_over_30ms_mps2": 2 * lateral_compliance_m / (0.030**2),
        "equivalent_accel_if_released_over_100ms_mps2": 2 * lateral_compliance_m / (0.100**2),
        "fraction_of_static_shear_rate_drift_full": (
            lateral_compliance_m / drift_by_case["full_static_replay"]["final_radial_m"]
        ),
        "fraction_of_static_shear_rate_drift_prediction_only": (
            lateral_compliance_m
            / drift_by_case["prediction_only_encoder_only"]["final_radial_m"]
        ),
    }

    summary = {
        "inputs": {
            "static_log": str(STATIC_LOG),
            "drift_metrics": str(DRIFT_METRICS),
            "launch_itemized": str(LAUNCH_ITEMIZED),
        },
        "static_segment": {
            "row_count": len(times),
            "duration_s": duration_s,
            "fan_duty_cycle": segment.get("fan_duty_cycle"),
            "segment_id": segment.get("segment_id"),
            "segment_start_row_index": segment.get("segment_start_row_index"),
            "segment_end_row_index": segment.get("segment_end_row_index"),
        },
        "sensor_stats": sensor_stats,
        "static_shear_rate_drift": drift_by_case,
        "yaw_launch_residual_scale": yaw_launch_summary,
        "lateral_compliance_scale": compliance_scale,
    }

    with (OUT_DIR / "physical_detail_diagnostics_summary.json").open("w") as handle:
        json.dump(summary, handle, indent=2, sort_keys=True)

    with (OUT_DIR / "physical_detail_diagnostics_summary.md").open("w") as handle:
        handle.write("# Physical Detail Diagnostics\n\n")
        handle.write(f"- Static log: `{STATIC_LOG}`\n")
        handle.write(
            f"- Rows: {len(times)}, duration: {duration_s:.3f} s, "
            f"fan duty: {segment.get('fan_duty_cycle')}\n"
        )
        handle.write(
            "- Encoder velocity RMS: "
            f"L={sensor_stats['left_encoder_velocity_mps']['rms']:.6g} m/s, "
            f"R={sensor_stats['right_encoder_velocity_mps']['rms']:.6g} m/s\n"
        )
        handle.write(
            "- Encoder velocity p99 abs: "
            f"L={sensor_stats['left_encoder_velocity_mps']['p99_abs']:.6g} m/s, "
            f"R={sensor_stats['right_encoder_velocity_mps']['p99_abs']:.6g} m/s\n"
        )
        handle.write(
            "- IMU RMS: "
            f"gyro={sensor_stats['gyro_radps']['rms']:.6g} rad/s, "
            f"right accel={sensor_stats['accel_body_right_mps2']['rms']:.6g} m/s^2, "
            f"forward accel={sensor_stats['accel_body_forward_mps2']['rms']:.6g} m/s^2\n"
        )
        handle.write(
            "- Demeaned IMU std: "
            f"gyro={sensor_stats['gyro_radps']['std']:.6g} rad/s, "
            f"right accel={sensor_stats['accel_body_right_mps2']['std']:.6g} m/s^2, "
            f"forward accel={sensor_stats['accel_body_forward_mps2']['std']:.6g} m/s^2\n"
        )
        handle.write(
            "- Shear-rate simplex static drift: "
            f"full={drift_by_case['full_static_replay']['final_radial_m'] * 1000:.3f} mm, "
            "prediction-only="
            f"{drift_by_case['prediction_only_encoder_only']['final_radial_m'] * 1000:.3f} mm\n"
        )
        handle.write(
            "- 0.6 mm compliance as fraction of drift: "
            f"full={compliance_scale['fraction_of_static_shear_rate_drift_full']:.4f}, "
            "prediction-only="
            f"{compliance_scale['fraction_of_static_shear_rate_drift_prediction_only']:.4f}\n"
        )
        handle.write(
            "- 0.6 mm equivalent acceleration: "
            f"10 ms={compliance_scale['equivalent_accel_if_released_over_10ms_mps2']:.3g} m/s^2, "
            f"30 ms={compliance_scale['equivalent_accel_if_released_over_30ms_mps2']:.3g} m/s^2, "
            f"100 ms={compliance_scale['equivalent_accel_if_released_over_100ms_mps2']:.3g} m/s^2\n"
        )
    print(OUT_DIR / "physical_detail_diagnostics_summary.md")


if __name__ == "__main__":
    main()
