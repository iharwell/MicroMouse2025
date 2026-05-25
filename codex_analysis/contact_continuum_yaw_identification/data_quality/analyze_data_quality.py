#!/usr/bin/env python3
import csv
import math
from collections import Counter, defaultdict
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent


EXPANDED_RUN_SUMMARY = ROOT / "codex_analysis" / "yaw_torque_expanded" / "expanded_yaw_torque_run_summary.csv"
VALIDATION_RUN_SUMMARY = ROOT / "codex_analysis" / "yaw_torque_expanded_validation" / "run_summary.csv"
VALIDATION_BINS = ROOT / "codex_analysis" / "yaw_torque_expanded_validation" / "nonzero_vf_torque_bins.csv"
VALIDATION_BIN_RUNS = ROOT / "codex_analysis" / "yaw_torque_expanded_validation" / "bin_run_consistency.csv"
COMP_BIN_CONTRIB = ROOT / "codex_analysis" / "yaw_torque_expanded_validation" / "competition_bin_contribution.csv"
DATASET_COMPARISON = ROOT / "codex_analysis" / "yaw_torque_expanded" / "expanded_yaw_torque_dataset_comparison.csv"


CORE_FIELDS = {
    "time": ("master_time_us", "t_us"),
    "dt": ("dt_us",),
    "gyro_raw": ("gyro_raw_radps",),
    "gyro_corrected": ("gyro_radps",),
    "left_velocity": ("left_encoder_velocity_mps", "left_velocity_mps"),
    "right_velocity": ("right_encoder_velocity_mps", "right_velocity_mps"),
    "cmd_linear": ("cmd_linear_mps",),
    "cmd_angular": ("cmd_angular_radps",),
    "left_command": ("left_drive_command", "left_drive_cmd"),
    "right_command": ("right_drive_command", "right_drive_cmd"),
}

TRUST_FIELDS = {
    "saturation": ("saturation_flags",),
    "watchdog": ("watchdog_flags",),
    "clipping": ("clipping_flags",),
    "measurement": ("measurement_flags",),
    "imu_status": ("imu_status", "imu_fr_status", "imu_bl_status"),
    "fan_duty": ("fan_duty_cycle",),
}

PHASE_FIELDS = (
    "section_id",
    "primitive_id",
    "primitive_family",
    "direction_id",
    "phase_id",
    "speed_bin",
    "repeat_index",
    "progress_norm",
    "stationary",
    "front_wall",
    "left_wall",
    "right_wall",
)


def rel(path):
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def read_csv_dicts(path):
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def first_present(fields, names):
    for name in names:
        if name in fields:
            return name
    return ""


def as_float(value):
    try:
        if value is None or value == "":
            return None
        out = float(value)
        if math.isfinite(out):
            return out
    except ValueError:
        pass
    return None


def as_int(value):
    x = as_float(value)
    if x is None:
        return None
    return int(x)


def open_timeseries(path):
    metadata = {}
    header = None
    prefix = None
    header_line = 0
    with path.open("r", encoding="utf-8", errors="replace", newline="") as f:
        for line_no, line in enumerate(f, 1):
            stripped = line.rstrip("\r\n")
            if not stripped:
                continue
            parts = next(csv.reader([stripped]))
            if not parts:
                continue
            if parts[0] == "# meta" and len(parts) >= 3:
                metadata[parts[1]] = parts[2]
                continue
            if parts[0] == "sample":
                header = parts
                prefix = ""
                header_line = line_no
                break
            if parts[0].startswith("#") or parts[0] in ("# event", "event"):
                continue
            if "master_time_us" in parts or "gyro_raw_radps" in parts:
                header = parts
                prefix = ""
                header_line = line_no
                break
    return metadata, header or [], prefix, header_line


def iter_rows(path, header, prefix, header_line):
    with path.open("r", encoding="utf-8", errors="replace", newline="") as f:
        for line_no, line in enumerate(f, 1):
            if line_no <= header_line:
                continue
            stripped = line.rstrip("\r\n")
            if not stripped or stripped.startswith("#"):
                continue
            parts = next(csv.reader([stripped]))
            if prefix == "sample":
                if not parts or parts[0] != "sample":
                    continue
                values = parts[1:]
            else:
                if not parts or parts[0] in ("event", "# event"):
                    continue
                values = parts
            if len(values) < len(header):
                values += [""] * (len(header) - len(values))
            yield dict(zip(header, values))


def discover_logs():
    logs = []
    for path in sorted((ROOT / "TestResults").glob("mmlog_decode_*/open_floor_main.csv")):
        run_id = path.parent.name.replace("mmlog_decode_", "")
        logs.append((run_id, "decoded_open_floor", path))
    comp = ROOT / "TestResults" / "Competition Testing Data"
    for path in sorted(comp.glob("diag*.csv")):
        logs.append((path.stem, "competition_diag", path))
    for path in sorted(comp.glob("aux*.csv")):
        logs.append((path.stem, "competition_aux", path))
    for path in sorted(comp.glob("fwc*.csv")):
        logs.append((path.stem, "competition_fwc", path))
    return logs


def classify_schema(family, fields):
    field_set = set(fields)
    if family == "competition_fwc":
        return "front_wall_characterization"
    if "master_time_us" in field_set:
        full_markers = {"section_id", "primitive_family", "watchdog_flags", "measurement_flags", "fan_duty_cycle"}
        return "decoded_open_floor_full_current" if full_markers.issubset(field_set) else "decoded_open_floor_legacy_or_partial"
    if "t_us" in field_set and "imu_fr_gyro_z" in field_set:
        return "legacy_competition_sample"
    return "unknown_or_non_timeseries"


def scan_log(run_id, family, path, expanded_by_run, validation_by_run):
    metadata, header, prefix, header_line = open_timeseries(path)
    fields = set(header)
    schema = classify_schema(family, fields)

    core_present = {name: first_present(fields, aliases) for name, aliases in CORE_FIELDS.items()}
    trust_present = {name: first_present(fields, aliases) for name, aliases in TRUST_FIELDS.items()}
    missing_core = [name for name, field in core_present.items() if not field]
    missing_trust = [name for name, field in trust_present.items() if not field]
    phase_present = [name for name in PHASE_FIELDS if name in fields]

    meta_fan = as_float(metadata.get("kRacingFanDutyCycle"))
    row_count = 0
    finite_sensor_rows = 0
    moving_yaw_rows = 0
    nonzero_feature_rows = 0
    stationary_sensor_rows = 0
    cmd_ang_zero_moving = 0
    diff_cmd_zero_moving = 0
    command_missing_moving = 0
    saturation_nonzero = 0
    saturation_moving = 0
    watchdog_nonzero = 0
    clipping_nonzero = 0
    imu_bad = 0
    fan_min = None
    fan_max = None
    phase_values = defaultdict(set)
    final_tail_rows = 0
    final_tail_us = 0
    last_time = None
    previous_gyro = None
    previous_time = None
    gyro_derivative_pairs = 0
    gyro_derivative_sum_sq = 0.0
    gyro_derivative_abs_max = 0.0
    gyro_derivative_spikes = 0
    tail_live = True

    time_field = core_present["time"]
    gyro_field = core_present["gyro_raw"]
    left_vel_field = core_present["left_velocity"]
    right_vel_field = core_present["right_velocity"]
    cmd_ang_field = core_present["cmd_angular"]
    left_cmd_field = core_present["left_command"]
    right_cmd_field = core_present["right_command"]

    if header and family != "competition_fwc":
        for row in iter_rows(path, header, prefix, header_line):
            row_count += 1
            t = as_int(row.get(time_field)) if time_field else None
            if t is not None:
                last_time = t
            gyro = as_float(row.get(gyro_field)) if gyro_field else None
            lv = as_float(row.get(left_vel_field)) if left_vel_field else None
            rv = as_float(row.get(right_vel_field)) if right_vel_field else None
            if gyro is None or lv is None or rv is None:
                is_active = False
            else:
                if previous_gyro is not None:
                    dt_s = None
                    if t is not None and previous_time is not None and t > previous_time:
                        dt_s = (t - previous_time) / 1_000_000.0
                    else:
                        dt_us = as_int(row.get("dt_us"))
                        if dt_us and dt_us > 0:
                            dt_s = dt_us / 1_000_000.0
                    if dt_s is not None and 0.0001 <= dt_s <= 0.1:
                        alpha = (gyro - previous_gyro) / dt_s
                        gyro_derivative_pairs += 1
                        gyro_derivative_sum_sq += alpha * alpha
                        gyro_derivative_abs_max = max(gyro_derivative_abs_max, abs(alpha))
                        if abs(alpha) >= 500.0:
                            gyro_derivative_spikes += 1
                previous_gyro = gyro
                previous_time = t
                finite_sensor_rows += 1
                vf = 0.5 * (lv + rv)
                is_active = abs(vf) > 0.02 or abs(gyro) > 0.2
                if abs(vf) > 0.02 and abs(gyro) > 0.2:
                    moving_yaw_rows += 1
                    ca = as_float(row.get(cmd_ang_field)) if cmd_ang_field else None
                    lc = as_float(row.get(left_cmd_field)) if left_cmd_field else None
                    rc = as_float(row.get(right_cmd_field)) if right_cmd_field else None
                    if ca is None:
                        command_missing_moving += 1
                    elif abs(ca) <= 1e-9:
                        cmd_ang_zero_moving += 1
                    if lc is None or rc is None:
                        command_missing_moving += 1
                    elif abs(lc - rc) <= 1e-9:
                        diff_cmd_zero_moving += 1
                    sat_field = trust_present["saturation"]
                    sat = as_int(row.get(sat_field)) if sat_field else None
                    if sat and sat != 0:
                        saturation_moving += 1
                if abs(vf) >= 0.05 and abs(gyro) >= 0.25:
                    nonzero_feature_rows += 1
                stationary_flag = row.get("stationary")
                if stationary_flag not in (None, ""):
                    stationary = as_int(stationary_flag) == 1
                else:
                    stationary = abs(lv) < 0.015 and abs(rv) < 0.015 and abs(gyro) < 0.03
                if stationary:
                    stationary_sensor_rows += 1

            sat_field = trust_present["saturation"]
            if sat_field:
                sat = as_int(row.get(sat_field))
                if sat and sat != 0:
                    saturation_nonzero += 1
            watchdog_field = trust_present["watchdog"]
            if watchdog_field:
                wd = as_int(row.get(watchdog_field))
                if wd and wd != 0:
                    watchdog_nonzero += 1
            clipping_field = trust_present["clipping"]
            if clipping_field:
                cl = as_int(row.get(clipping_field))
                if cl and cl != 0:
                    clipping_nonzero += 1
            imu_status_field = trust_present["imu_status"]
            if imu_status_field:
                status = as_int(row.get(imu_status_field))
                if status is not None and status == 0:
                    imu_bad += 1
            fan_field = trust_present["fan_duty"]
            if fan_field:
                fan = as_float(row.get(fan_field))
                if fan is not None:
                    fan_min = fan if fan_min is None else min(fan_min, fan)
                    fan_max = fan if fan_max is None else max(fan_max, fan)
            for field in phase_present:
                if len(phase_values[field]) < 64:
                    phase_values[field].add(row.get(field, ""))

            if tail_live:
                if is_active:
                    final_tail_rows = 0
                    final_tail_us = 0
                else:
                    final_tail_rows += 1
                    dt = as_int(row.get("dt_us"))
                    if dt is not None:
                        final_tail_us += dt

    expanded = expanded_by_run.get(run_id, {})
    validation = validation_by_run.get(run_id, {})
    expanded_extracted = as_int(expanded.get("extracted_samples"))
    validation_extracted = as_int(validation.get("extracted_samples"))
    if validation and validation_extracted is not None:
        extracted = validation_extracted
    elif expanded_extracted is not None:
        extracted = expanded_extracted
    else:
        extracted = 0
    existing_status = validation.get("status") or expanded.get("limitation") or ""
    existing_cutoff = expanded.get("cutoff_reason", "")

    if trust_present["fan_duty"]:
        fan_source = "per_row"
        load_assumption = "logged per row"
    elif meta_fan is not None:
        fan_source = "metadata"
        fan_min = fan_min if fan_min is not None else meta_fan
        fan_max = fan_max if fan_max is not None else meta_fan
        load_assumption = f"constant metadata fan duty {meta_fan:.3f}"
    else:
        fan_source = "absent"
        load_assumption = "fan/load unobservable in this CSV"

    saturation_fraction = saturation_nonzero / row_count if row_count else 0.0
    saturation_moving_fraction = saturation_moving / moving_yaw_rows if moving_yaw_rows else 0.0
    cmd_zero_fraction = cmd_ang_zero_moving / moving_yaw_rows if moving_yaw_rows else 0.0
    diff_cmd_zero_fraction = diff_cmd_zero_moving / moving_yaw_rows if moving_yaw_rows else 0.0
    imu_bad_fraction = imu_bad / row_count if row_count else 0.0
    tail_seconds = final_tail_us / 1_000_000.0
    gyro_derivative_rms = math.sqrt(gyro_derivative_sum_sq / gyro_derivative_pairs) if gyro_derivative_pairs else 0.0
    gyro_derivative_spike_fraction = gyro_derivative_spikes / gyro_derivative_pairs if gyro_derivative_pairs else 0.0

    bias_feasible = stationary_sensor_rows >= 500
    if not header or family == "competition_fwc":
        recommendation = "exclude"
        reasons = ["not a yaw/encoder time series"]
    elif extracted <= 0 and nonzero_feature_rows < 100:
        recommendation = "exclude"
        reasons = ["no usable nonzero-forward/nonzero-yaw feature samples"]
    elif family == "competition_aux":
        recommendation = "validation_only_downweighted"
        reasons = ["legacy schema", "path/procedure-dependent aux audit runs"]
    elif family == "competition_diag":
        if run_id == "diag002":
            recommendation = "exclude"
            reasons = ["no usable feature samples"]
        else:
            recommendation = "validation_only"
            reasons = ["legacy schema lacks saturation/watchdog/per-row fan duty"]
    else:
        reasons = []
        if not bias_feasible:
            reasons.append("weak independent stationary bias support")
        if not trust_present["saturation"]:
            reasons.append("saturation unavailable")
        elif saturation_moving_fraction >= 0.25:
            reasons.append("high moving-yaw saturation fraction")
        if not trust_present["watchdog"]:
            reasons.append("watchdog unavailable")
        if cmd_zero_fraction >= 0.50:
            reasons.append("angular command mostly zero during moving-yaw rows")
        if tail_seconds >= 0.25 or "dropped final" in existing_cutoff:
            reasons.append("final quiescent/invalid tail detected")
        if extracted < 1000:
            reasons.append("low extracted-sample count")
        if reasons:
            recommendation = "fit_downweighted" if extracted >= 1000 else "validation_only_or_exclude"
        else:
            recommendation = "fit_authoritative"
        if recommendation == "validation_only_or_exclude":
            recommendation = "validation_only"

    phase_cardinality = ";".join(f"{field}:{len(vals)}" for field, vals in sorted(phase_values.items()))
    phase_risk = "low"
    if family == "competition_aux":
        phase_risk = "high_path_dependence"
    elif family == "competition_diag":
        phase_risk = "medium_legacy_phase_labels"
    elif "primitive_family" not in fields or "section_id" not in fields:
        phase_risk = "medium_partial_phase_schema"

    return {
        "run_id": run_id,
        "family": family,
        "path": rel(path),
        "size_bytes": path.stat().st_size if path.exists() else 0,
        "schema_kind": schema,
        "field_count": len(header),
        "input_rows_scanned": row_count,
        "expanded_input_rows": expanded.get("input_rows", ""),
        "expanded_extracted_samples": "" if expanded_extracted is None else expanded_extracted,
        "validation_extracted_samples": "" if validation_extracted is None else validation_extracted,
        "extracted_samples": extracted,
        "existing_status": existing_status,
        "existing_cutoff_reason": existing_cutoff,
        "missing_core_fields": ";".join(missing_core),
        "missing_trust_fields": ";".join(missing_trust),
        "timestamp_field": core_present["time"],
        "gyro_raw_field": gyro_field,
        "left_velocity_field": left_vel_field,
        "right_velocity_field": right_vel_field,
        "cmd_angular_present": "yes" if cmd_ang_field else "no",
        "left_right_command_present": "yes" if left_cmd_field and right_cmd_field else "no",
        "finite_sensor_rows": finite_sensor_rows,
        "stationary_bias_rows_sensor_only": stationary_sensor_rows,
        "bias_estimation_feasible": "yes" if bias_feasible else "no",
        "moving_yaw_sensor_rows_abs_vf_gt_0p02_abs_gyro_gt_0p2": moving_yaw_rows,
        "nonzero_feature_sensor_rows_abs_vf_ge_0p05_abs_gyro_ge_0p25": nonzero_feature_rows,
        "cmd_angular_zero_fraction_moving_yaw": f"{cmd_zero_fraction:.6f}",
        "diff_drive_command_zero_fraction_moving_yaw": f"{diff_cmd_zero_fraction:.6f}",
        "command_missing_moving_yaw_rows": command_missing_moving,
        "saturation_flags_present": "yes" if trust_present["saturation"] else "no",
        "saturation_nonzero_rows": saturation_nonzero,
        "saturation_nonzero_fraction_all": f"{saturation_fraction:.6f}",
        "saturation_nonzero_fraction_moving_yaw": f"{saturation_moving_fraction:.6f}",
        "watchdog_flags_present": "yes" if trust_present["watchdog"] else "no",
        "watchdog_nonzero_rows": watchdog_nonzero,
        "clipping_flags_present": "yes" if trust_present["clipping"] else "no",
        "clipping_nonzero_rows": clipping_nonzero,
        "imu_status_bad_fraction": f"{imu_bad_fraction:.6f}",
        "gyro_derivative_pairs": gyro_derivative_pairs,
        "gyro_derivative_rms_radps2": f"{gyro_derivative_rms:.6f}",
        "gyro_derivative_abs_max_radps2": f"{gyro_derivative_abs_max:.6f}",
        "gyro_derivative_abs_ge_500_radps2_fraction": f"{gyro_derivative_spike_fraction:.6f}",
        "fan_source": fan_source,
        "fan_duty_min": "" if fan_min is None else f"{fan_min:.6f}",
        "fan_duty_max": "" if fan_max is None else f"{fan_max:.6f}",
        "load_assumption": load_assumption,
        "phase_fields_present": ";".join(phase_present),
        "phase_cardinality": phase_cardinality,
        "phase_procedure_contamination_risk": phase_risk,
        "bad_tail_detectable": "yes" if gyro_field and left_vel_field and right_vel_field and core_present["dt"] else "no",
        "final_quiescent_tail_rows": final_tail_rows,
        "final_quiescent_tail_seconds": f"{tail_seconds:.6f}",
        "recommendation": recommendation,
        "recommendation_reasons": "; ".join(reasons),
    }


def write_csv(path, rows, fieldnames=None):
    if fieldnames is None:
        keys = []
        seen = set()
        for row in rows:
            for key in row.keys():
                if key not in seen:
                    keys.append(key)
                    seen.add(key)
        fieldnames = keys
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def aggregate_family(rows):
    out = []
    by_family = defaultdict(list)
    for row in rows:
        by_family[row["family"]].append(row)
    for family, items in sorted(by_family.items()):
        extracted = sum(int(item["extracted_samples"] or 0) for item in items)
        scanned = sum(int(item["input_rows_scanned"] or 0) for item in items)
        moving = sum(int(item["moving_yaw_sensor_rows_abs_vf_gt_0p02_abs_gyro_gt_0p2"] or 0) for item in items)
        nonzero = sum(int(item["nonzero_feature_sensor_rows_abs_vf_ge_0p05_abs_gyro_ge_0p25"] or 0) for item in items)
        saturation_available = sum(1 for item in items if item["saturation_flags_present"] == "yes")
        watchdog_available = sum(1 for item in items if item["watchdog_flags_present"] == "yes")
        per_row_fan = sum(1 for item in items if item["fan_source"] == "per_row")
        meta_fan = sum(1 for item in items if item["fan_source"] == "metadata")
        derivative_pairs = sum(int(item.get("gyro_derivative_pairs") or 0) for item in items)
        derivative_spikes = sum(
            int(round(float(item.get("gyro_derivative_abs_ge_500_radps2_fraction") or 0) * int(item.get("gyro_derivative_pairs") or 0)))
            for item in items
        )
        fit_auth = sum(1 for item in items if item["recommendation"] == "fit_authoritative")
        down = sum(1 for item in items if "downweighted" in item["recommendation"] or item["recommendation"] == "fit_downweighted")
        val = sum(1 for item in items if item["recommendation"].startswith("validation"))
        exc = sum(1 for item in items if item["recommendation"] == "exclude")
        top = max(items, key=lambda item: int(item["extracted_samples"] or 0))
        out.append({
            "family": family,
            "run_count": len(items),
            "rows_scanned": scanned,
            "moving_yaw_sensor_rows": moving,
            "nonzero_feature_sensor_rows": nonzero,
            "extracted_samples_existing_validation": extracted,
            "top_run_by_extracted_samples": top["run_id"],
            "top_run_extracted_samples": top["extracted_samples"],
            "top_run_family_share": f"{(int(top['extracted_samples'] or 0) / extracted if extracted else 0.0):.6f}",
            "saturation_available_runs": saturation_available,
            "watchdog_available_runs": watchdog_available,
            "per_row_fan_runs": per_row_fan,
            "metadata_fan_runs": meta_fan,
            "gyro_derivative_pairs": derivative_pairs,
            "gyro_derivative_abs_ge_500_radps2_fraction": f"{(derivative_spikes / derivative_pairs if derivative_pairs else 0.0):.6f}",
            "fit_authoritative_runs": fit_auth,
            "downweighted_runs": down,
            "validation_only_runs": val,
            "excluded_runs": exc,
        })
    return out


def run_dominance(rows):
    total = sum(int(row["extracted_samples"] or 0) for row in rows)
    fam_totals = Counter()
    for row in rows:
        fam_totals[row["family"]] += int(row["extracted_samples"] or 0)
    out = []
    for row in rows:
        samples = int(row["extracted_samples"] or 0)
        out.append({
            "run_id": row["run_id"],
            "family": row["family"],
            "extracted_samples": samples,
            "overall_share": f"{(samples / total if total else 0.0):.6f}",
            "family_share": f"{(samples / fam_totals[row['family']] if fam_totals[row['family']] else 0.0):.6f}",
            "recommendation": row["recommendation"],
            "dominance_flag": "dominant" if total and samples / total >= 0.10 else ("family_dominant" if fam_totals[row["family"]] and samples / fam_totals[row["family"]] >= 0.25 else ""),
        })
    return sorted(out, key=lambda item: int(item["extracted_samples"]), reverse=True)


def coverage_summary():
    bin_rows = read_csv_dicts(VALIDATION_BINS)
    bin_run_rows = read_csv_dicts(VALIDATION_BIN_RUNS)
    comp_rows = read_csv_dicts(COMP_BIN_CONTRIB)
    dataset_rows = read_csv_dicts(DATASET_COMPARISON)
    total_bins = len(bin_rows)
    strong = [r for r in bin_rows if int(r.get("count") or 0) >= 250 and int(r.get("run_count_ge10") or 0) >= 5 and r.get("consistency") == "cross-run"]
    weak = [r for r in bin_rows if r.get("consistency") != "cross-run" or int(r.get("run_count_ge10") or 0) < 3]
    high_forward = [r for r in bin_rows if abs(float(r.get("forward_velocity_bin_mps") or 0)) >= 0.6]
    high_yaw = [r for r in bin_rows if abs(float(r.get("yaw_rate_bin_radps") or 0)) >= 8.0]
    comp_contrib = [r for r in comp_rows if (r.get("competition_enabled_bin") or "").lower() == "true"]
    run_bin_counts = Counter()
    for row in bin_run_rows:
        run_bin_counts[row["run_id"]] += int(row.get("count") or 0)
    top_bin_run = run_bin_counts.most_common(10)
    out = [{
        "metric": "total_nonzero_vf_yaw_bins",
        "value": total_bins,
        "note": "from yaw_torque_expanded_validation/nonzero_vf_torque_bins.csv",
    }, {
        "metric": "strong_cross_run_bins_count_ge250_runs_ge5",
        "value": len(strong),
        "note": "candidate fit-support bins before family trust weighting",
    }, {
        "metric": "weak_or_single_run_bins",
        "value": len(weak),
        "note": "must be downweighted or targeted for sweeps",
    }, {
        "metric": "high_abs_forward_ge_0p6_bins",
        "value": len(high_forward),
        "note": "coverage exists but is sparse and often run-dominated",
    }, {
        "metric": "high_abs_yaw_ge_8_bins",
        "value": len(high_yaw),
        "note": "mostly low-forward scrub/turn coverage, not broad high-speed arcs",
    }, {
        "metric": "competition_enabled_bins",
        "value": len(comp_contrib),
        "note": "bins crossing support because competition data was present",
    }]
    for row in dataset_rows:
        out.append({
            "metric": f"dataset_{row['dataset']}_samples",
            "value": row.get("samples", ""),
            "note": f"moving_yaw={row.get('moving_yaw_samples_abs_vf_gt_0p02_abs_yaw_gt_0p2','')}, bins={row.get('train_surface_bins','')}",
        })
    for run, count in top_bin_run:
        out.append({
            "metric": f"top_bin_contributor_{run}",
            "value": count,
            "note": "sum of bin_run_consistency counts; highlights run dominance risk",
        })
    return out


def hazard_rows(run_rows, family_rows):
    hazards = []
    def add(scope, hazard, evidence, recommendation):
        hazards.append({
            "scope": scope,
            "hazard": hazard,
            "evidence": evidence,
            "recommendation": recommendation,
        })

    for fam in family_rows:
        family = fam["family"]
        if int(fam["saturation_available_runs"]) < int(fam["run_count"]):
            add(family, "saturation unavailable", f"{fam['saturation_available_runs']}/{fam['run_count']} runs expose saturation flags", "do not use unavailable-saturation runs as fit authority")
        if int(fam["watchdog_available_runs"]) < int(fam["run_count"]):
            add(family, "watchdog unavailable", f"{fam['watchdog_available_runs']}/{fam['run_count']} runs expose watchdog flags", "treat watchdog absence as schema trust loss, not proof of no watchdog activity")
        if int(fam["per_row_fan_runs"]) == 0:
            add(family, "fan/load assumed", f"per-row fan in {fam['per_row_fan_runs']} runs; metadata fan in {fam['metadata_fan_runs']} runs", "separate load/fan inference from contact-continuum yaw fit")
    for row in run_rows:
        scope = f"{row['family']}:{row['run_id']}"
        if row["cmd_angular_present"] == "yes" and float(row["cmd_angular_zero_fraction_moving_yaw"] or 0) >= 0.50 and int(row["moving_yaw_sensor_rows_abs_vf_gt_0p02_abs_gyro_gt_0p2"] or 0) > 0:
            add(scope, "missing/stale angular command evidence", f"cmd_angular zero fraction during moving-yaw rows={row['cmd_angular_zero_fraction_moving_yaw']}", "fit only from wheel commands/sensor timing or downweight")
        if row["saturation_flags_present"] == "yes" and float(row["saturation_nonzero_fraction_moving_yaw"] or 0) >= 0.25:
            add(scope, "saturation can mimic yaw physics", f"moving-yaw saturation fraction={row['saturation_nonzero_fraction_moving_yaw']}", "exclude saturated rows and downweight the run")
        if float(row["final_quiescent_tail_seconds"] or 0) >= 0.25 or "dropped final" in row["existing_cutoff_reason"]:
            add(scope, "final bad tail", f"tail_seconds={row['final_quiescent_tail_seconds']}; existing_cutoff={row['existing_cutoff_reason']}", "trim tails before derivative/ablation work")
        if row["bias_estimation_feasible"] == "no":
            add(scope, "weak gyro bias estimate", f"sensor stationary rows={row['stationary_bias_rows_sensor_only']}", "exclude or validation-only unless paired with external calibration")
        if "legacy" in row["schema_kind"]:
            add(scope, "old schema", row["schema_kind"], "separate family and avoid fit authority")
        if row["phase_procedure_contamination_risk"].startswith("high"):
            add(scope, "phase/procedure contamination", row["phase_procedure_contamination_risk"], "validation only until phase labels and repeats are separated")
        if float(row.get("gyro_derivative_abs_ge_500_radps2_fraction") or 0) >= 0.001:
            add(scope, "gyro derivative noise/spikes", f"abs(dgyro/dt)>=500 rad/s^2 fraction={row['gyro_derivative_abs_ge_500_radps2_fraction']}; rms={row['gyro_derivative_rms_radps2']} rad/s^2", "smooth, bin, or use robust losses before fitting acceleration-like yaw residuals")
    return hazards


def write_report(run_rows, family_rows, dominance_rows, coverage_rows, hazards):
    top_runs = dominance_rows[:8]
    authoritative = [r for r in run_rows if r["recommendation"] == "fit_authoritative"]
    downweighted = [r for r in run_rows if "downweighted" in r["recommendation"] or r["recommendation"] == "fit_downweighted"]
    excluded = [r for r in run_rows if r["recommendation"] == "exclude"]
    lines = []
    lines.append("# Contact-Continuum Yaw Data Quality")
    lines.append("")
    lines.append("Analysis-only output. Production code was not modified. Trust predicates use sensor, command, schema, and metadata fields only; `ukf_state_*` fields are not used.")
    lines.append("")
    lines.append("## Files")
    for name in [
        "data_quality_run_inventory.csv",
        "data_quality_family_summary.csv",
        "data_quality_run_dominance.csv",
        "data_quality_feature_coverage_summary.csv",
        "data_quality_hazards.csv",
        "data_quality_recommendations_by_run.csv",
    ]:
        lines.append(f"- `{name}`")
    lines.append("")
    lines.append("## Family Summary")
    lines.append("| Family | Runs | Extracted | Moving-yaw sensor rows | Saturation runs | Watchdog runs | Gyro d/dt spikes | Fan source | Recommendation shape |")
    lines.append("| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- |")
    for row in family_rows:
        fan = f"per-row {row['per_row_fan_runs']}, metadata {row['metadata_fan_runs']}"
        shape = f"fit {row['fit_authoritative_runs']}, down {row['downweighted_runs']}, val {row['validation_only_runs']}, excl {row['excluded_runs']}"
        lines.append(f"| {row['family']} | {row['run_count']} | {row['extracted_samples_existing_validation']} | {row['moving_yaw_sensor_rows']} | {row['saturation_available_runs']} | {row['watchdog_available_runs']} | {row['gyro_derivative_abs_ge_500_radps2_fraction']} | {fan} | {shape} |")
    lines.append("")
    lines.append("## Recommendations")
    lines.append(f"- Fit-authoritative runs: {len(authoritative)} decoded open-floor runs with adequate sensor coverage and no major schema/trust penalty.")
    lines.append(f"- Downweighted runs: {len(downweighted)} runs, mostly high saturation, missing watchdog/per-row fan, zero angular command evidence, or aux path dependence.")
    lines.append(f"- Excluded runs: {len(excluded)} runs, mostly front-wall characterization, no feature samples, or weak bias/feature support.")
    lines.append("- Competition diag: validation-only; useful maze-turn coverage but legacy schema lacks saturation/watchdog/per-row fan duty.")
    lines.append("- Competition aux: validation-only and downweighted; useful stress coverage but too path/procedure dependent for fit authority.")
    lines.append("")
    lines.append("## Run Dominance")
    lines.append("| Run | Family | Samples | Overall share | Family share | Recommendation |")
    lines.append("| --- | --- | ---: | ---: | ---: | --- |")
    for row in top_runs:
        lines.append(f"| {row['run_id']} | {row['family']} | {row['extracted_samples']} | {row['overall_share']} | {row['family_share']} | {row['recommendation']} |")
    lines.append("")
    lines.append("## Coverage")
    for row in coverage_rows[:12]:
        lines.append(f"- {row['metric']}: {row['value']} ({row['note']})")
    lines.append("")
    lines.append("## Hazards")
    for row in hazards[:40]:
        lines.append(f"- {row['scope']}: {row['hazard']} - {row['evidence']}; {row['recommendation']}.")
    lines.append("")
    lines.append("## Reproduce")
    lines.append("")
    lines.append("```powershell")
    lines.append("python codex_analysis\\contact_continuum_yaw_identification\\data_quality\\analyze_data_quality.py")
    lines.append("```")
    (OUT / "data_quality_report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main():
    expanded_by_run = {row["run_id"]: row for row in read_csv_dicts(EXPANDED_RUN_SUMMARY)}
    validation_by_run = {row["run_id"]: row for row in read_csv_dicts(VALIDATION_RUN_SUMMARY)}
    run_rows = []
    for run_id, family, path in discover_logs():
        run_rows.append(scan_log(run_id, family, path, expanded_by_run, validation_by_run))
    total_extracted = sum(int(row["extracted_samples"] or 0) for row in run_rows)
    family_totals = Counter()
    for row in run_rows:
        family_totals[row["family"]] += int(row["extracted_samples"] or 0)
    for row in run_rows:
        samples = int(row["extracted_samples"] or 0)
        row["overall_extracted_share"] = f"{(samples / total_extracted if total_extracted else 0.0):.6f}"
        row["family_extracted_share"] = f"{(samples / family_totals[row['family']] if family_totals[row['family']] else 0.0):.6f}"

    family_rows = aggregate_family(run_rows)
    dominance_rows = run_dominance(run_rows)
    coverage_rows = coverage_summary()
    hazards = hazard_rows(run_rows, family_rows)
    recommendations = [{
        "run_id": row["run_id"],
        "family": row["family"],
        "recommendation": row["recommendation"],
        "extracted_samples": row["extracted_samples"],
        "moving_yaw_sensor_rows": row["moving_yaw_sensor_rows_abs_vf_gt_0p02_abs_gyro_gt_0p2"],
        "schema_kind": row["schema_kind"],
        "reason": row["recommendation_reasons"],
    } for row in run_rows]

    write_csv(OUT / "data_quality_run_inventory.csv", run_rows)
    write_csv(OUT / "data_quality_family_summary.csv", family_rows)
    write_csv(OUT / "data_quality_run_dominance.csv", dominance_rows)
    write_csv(OUT / "data_quality_feature_coverage_summary.csv", coverage_rows)
    write_csv(OUT / "data_quality_hazards.csv", hazards)
    write_csv(OUT / "data_quality_recommendations_by_run.csv", recommendations)
    write_report(run_rows, family_rows, dominance_rows, coverage_rows, hazards)


if __name__ == "__main__":
    main()
