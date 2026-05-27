#!/usr/bin/env python3
"""Estimate L/R command deltas over a low-speed yaw grid.

Analysis-only worker script. It reads existing yaw-model variant artifacts and
writes outputs only beside this file.
"""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent
CONST_PATH = (
    ROOT
    / "codex_analysis"
    / "contact_continuum_yaw_identification"
    / "features"
    / "plant_mirror_constants.csv"
)


def read_constants() -> dict[str, float]:
    with CONST_PATH.open(newline="", encoding="utf-8") as fh:
        return {row["name"]: float(row["value"]) for row in csv.DictReader(fh)}


def read_key_value_csv(path: Path) -> dict[str, float]:
    with path.open(newline="", encoding="utf-8") as fh:
        return {row["parameter"]: float(row["value"]) for row in csv.DictReader(fh)}


def read_coeff_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as fh:
        return list(csv.DictReader(fh))


def sign(value: float, eps: float = 1.0e-6) -> float:
    return float((value > eps) - (value < -eps))


def signed_direction(preferred: float, fallback: float) -> float:
    preferred_sign = sign(preferred)
    return preferred_sign if preferred_sign != 0.0 else sign(fallback)


def smooth_gate(value: float, scale: float) -> float:
    ratio = abs(value) / scale if scale > 0.0 else 0.0
    return 1.0 / (1.0 + ratio * ratio)


def smooth_positive(value: float, epsilon: float = 1.0e-6) -> float:
    return 0.5 * (value + math.sqrt(value * value + epsilon * epsilon))


def torque_from_command(command: float, wheel_speed_radps: float, constants: dict[str, float]) -> float:
    resistance = constants["drive_resistance_ohms"]
    speed_constant = constants["speed_constant_radps_per_volt"]
    torque_constant = constants["torque_constant_nm_per_a"]
    gear_ratio = constants["gear_ratio"]
    battery = constants["drive_voltage_v"]
    no_load = constants["no_load_current_a"]

    applied_voltage = command * battery
    current = (applied_voltage / resistance) - (
        (wheel_speed_radps * (gear_ratio / speed_constant)) / resistance
    )
    armature_sign = sign(current)
    wheel_sign = sign(wheel_speed_radps)
    no_load_sign = armature_sign if armature_sign else wheel_sign
    load_current = current - no_load_sign * no_load
    if no_load_sign > 0.0 and load_current < 0.0:
        load_current = 0.0
    elif no_load_sign < 0.0 and load_current > 0.0:
        load_current = 0.0
    return torque_constant * gear_ratio * load_current


def command_from_torque(command_torque: float, wheel_speed_radps: float, constants: dict[str, float]) -> float:
    resistance = constants["drive_resistance_ohms"]
    speed_constant = constants["speed_constant_radps_per_volt"]
    torque_constant = constants["torque_constant_nm_per_a"]
    gear_ratio = constants["gear_ratio"]
    battery = constants["drive_voltage_v"]
    no_load = constants["no_load_current_a"]

    motor_torque = command_torque / gear_ratio
    torque_sign = sign(motor_torque)
    wheel_sign = sign(wheel_speed_radps)
    no_load_sign = torque_sign if torque_sign else wheel_sign
    current = (motor_torque / torque_constant) + no_load_sign * no_load
    back_emf = wheel_speed_radps * (gear_ratio / speed_constant)
    return ((current * resistance) + back_emf) / battery


def static_launch_torque(constants: dict[str, float]) -> float:
    return max(0.0, torque_from_command(constants["static_launch_command"], 0.0, constants))


def wheel_speeds(vf_mps: float, yaw_rate: float, constants: dict[str, float]) -> tuple[float, float, float, float]:
    half_track = 0.5 * constants["track_width_m"]
    radius = constants["wheel_radius_m"]
    left_surface_mps = vf_mps + half_track * yaw_rate
    right_surface_mps = vf_mps - half_track * yaw_rate
    return (
        left_surface_mps,
        right_surface_mps,
        left_surface_mps / radius,
        right_surface_mps / radius,
    )


def command_torque_for_applied(
    applied_torque: float,
    wheel_speed_radps: float,
    constants: dict[str, float],
) -> tuple[float, float]:
    surface_speed = constants["wheel_radius_m"] * wheel_speed_radps
    ratio = abs(surface_speed) / constants["static_friction_max_speed_mps"]
    launch = static_launch_torque(constants) * math.exp(-(ratio * ratio))
    launch_dir = signed_direction(applied_torque, wheel_speed_radps)
    loss_dir = signed_direction(wheel_speed_radps, applied_torque)
    rolling = constants["rolling_friction_torque_nm"] * loss_dir
    command_torque = applied_torque
    if signed_direction(applied_torque, wheel_speed_radps) != 0.0:
        command_torque += launch_dir * launch + rolling
    return command_torque, launch


def motor_commands_for_opposing_torque(
    opposing_yaw_torque: float,
    constants: dict[str, float],
    vf_mps: float,
    yaw_rate: float,
) -> dict[str, float]:
    radius = constants["wheel_radius_m"]
    track = constants["track_width_m"]
    left_surface, right_surface, left_speed, right_speed = wheel_speeds(vf_mps, yaw_rate, constants)
    applied_bank_torque = opposing_yaw_torque * radius / track
    left_command_torque, left_launch = command_torque_for_applied(applied_bank_torque, left_speed, constants)
    right_command_torque, right_launch = command_torque_for_applied(-applied_bank_torque, right_speed, constants)
    left_command = command_from_torque(left_command_torque, left_speed, constants)
    right_command = command_from_torque(right_command_torque, right_speed, constants)
    return {
        "applied_bank_torque_nm": applied_bank_torque,
        "left_command_torque_nm": left_command_torque,
        "right_command_torque_nm": right_command_torque,
        "left_command": left_command,
        "right_command": right_command,
        "lr_delta_command": left_command - right_command,
        "left_surface_mps": left_surface,
        "right_surface_mps": right_surface,
        "left_wheel_speed_radps": left_speed,
        "right_wheel_speed_radps": right_speed,
        "left_launch_torque_nm": left_launch,
        "right_launch_torque_nm": right_launch,
    }


def contact_utilization(constants: dict[str, float], yaw_rate: float) -> float:
    longitudinal = constants["drive_wheel_longitudinal_offset_m"]
    front_force = abs(constants["front_right_contact_force_gain_n_per_mps"] * longitudinal * yaw_rate)
    rear_force = abs(constants["rear_right_contact_force_gain_n_per_mps"] * longitudinal * yaw_rate)
    force_limit = constants["mass_kg"] * constants["sustained_lateral_accel_mps2"] / 4.0
    return max(front_force, rear_force) / force_limit


def limiter_activity(constants: dict[str, float], yaw_rate: float) -> float:
    util = contact_utilization(constants, yaw_rate)
    return max(0.0, util - 1.0)


def baseline_opposing_yaw_torque(constants: dict[str, float], yaw_rate: float) -> float:
    longitudinal = constants["drive_wheel_longitudinal_offset_m"]
    front_gain = constants["front_right_contact_force_gain_n_per_mps"]
    rear_gain = constants["rear_right_contact_force_gain_n_per_mps"]
    front_right_velocity = -longitudinal * yaw_rate
    rear_right_velocity = longitudinal * yaw_rate
    front_right_force_total = 2.0 * front_gain * front_right_velocity
    rear_right_force_total = 2.0 * rear_gain * rear_right_velocity
    yaw_moment = longitudinal * (front_right_force_total - rear_right_force_total)
    return -yaw_moment


def variant_a_extra(constants: dict[str, float], vf_mps: float, yaw_rate: float) -> float:
    payload = json.loads(
        (
            ROOT
            / "codex_analysis"
            / "yaw_model_variant_fits"
            / "contact_patch"
            / "variant_a_coefficients.json"
        ).read_text(encoding="utf-8")
    )
    coeffs = payload["coefficients"]
    hp = payload["hyperparameters"]
    longitudinal = constants["drive_wheel_longitudinal_offset_m"]
    q_patch = -(longitudinal * longitudinal) * abs(yaw_rate)
    vbar_rel = longitudinal * abs(yaw_rate)
    low_rel = smooth_gate(vbar_rel, hp["rel_speed_scale_mps"])
    low_forward = smooth_gate(vf_mps, hp["forward_speed_scale_mps"])
    tanh_q = math.tanh(q_patch / hp["q_tanh_scale_m2ps"])
    util = min(max(contact_utilization(constants, yaw_rate), 0.0), 4.0)
    util_squash = util / (1.0 + util)
    abs_patch = abs(q_patch)
    values = {
        "q_patch_yaw_velocity_m2ps": q_patch,
        "q_low_contact_speed": q_patch * low_rel,
        "q_low_forward_speed": q_patch * low_forward,
        "q_low_contact_and_forward": q_patch * low_rel * low_forward,
        "tanh_q_low_contact_and_forward": tanh_q * low_rel * low_forward,
        "tanh_q_low_contact": tanh_q * low_rel,
        "q_force_utilization": q_patch * util_squash,
        "q_force_utilization_low_contact": q_patch * util_squash * low_rel,
        "q_abs_patch_velocity": q_patch * abs_patch,
    }
    additive_yaw_torque = sum(coeffs[name] * values[name] for name in payload["feature_names"])
    return -additive_yaw_torque


def variant_b_extra(base_opposing: float, constants: dict[str, float], vf_mps: float, yaw_rate: float) -> float:
    coeff = read_key_value_csv(
        ROOT / "codex_analysis" / "yaw_model_variant_fits" / "stribeck_scrub" / "stribeck_coefficients.csv"
    )
    longitudinal = constants["drive_wheel_longitudinal_offset_m"]
    vbar_rel = longitudinal * abs(yaw_rate)
    transition_speed = math.hypot(coeff["rel_weight"] * vbar_rel, abs(vf_mps))
    stribeck = math.exp(-((transition_speed / coeff["stribeck_speed_mps"]) ** 2))
    speed_relief = 1.0 / (1.0 + (transition_speed / coeff["speed_fade_mps"]) ** 2)

    extra = 0.0
    for _ in range(50):
        requested = base_opposing + extra
        activation = 1.0 - math.exp(-((smooth_positive(requested) / coeff["req_activation_nm"]) ** 2))
        next_extra = activation * speed_relief * (
            coeff["static_extra_nm"] * stribeck + coeff["sliding_nm"]
        )
        if abs(next_extra - extra) < 1.0e-12:
            return next_extra
        extra = next_extra
    return extra


def variant_c_extra(base_opposing: float, constants: dict[str, float], vf_mps: float, yaw_rate: float) -> float:
    rows = read_coeff_rows(
        ROOT / "codex_analysis" / "yaw_model_variant_fits" / "combined_slip_surface" / "model_coefficients.csv"
    )
    coeff_rows = [row for row in rows if row["candidate"] == "saturation_aware_surface"]
    coeffs = {row["feature"]: float(row["standardized_coefficient_nm"]) for row in coeff_rows}
    scales = {row["feature"]: float(row["feature_scale"]) for row in coeff_rows}
    vrel_knee = float(coeff_rows[0]["vrel_knee_mps"])
    fwd_knee = float(coeff_rows[0]["fwd_knee_mps"])

    longitudinal = constants["drive_wheel_longitudinal_offset_m"]
    v_rel = longitudinal * abs(yaw_rate)
    util = min(max(contact_utilization(constants, yaw_rate), 0.0), 5.0)
    limiter = min(max(limiter_activity(constants, yaw_rate), 0.0), 5.0)
    util_smooth = util / (1.0 + util)
    limiter_smooth = limiter / (1.0 + limiter)
    low_rel = smooth_gate(v_rel, vrel_knee)
    high_forward = 1.0 - smooth_gate(vf_mps, fwd_knee)
    right_front = 2.0 * (longitudinal * longitudinal) * abs(yaw_rate)
    right_rear = 2.0 * (longitudinal * longitudinal) * abs(yaw_rate)
    values_base = {
        "gain_front_right_basis": right_front,
        "gain_rear_right_basis": right_rear,
        "gain_left_long_basis": 0.0,
        "gain_right_long_basis": 0.0,
        "force_gap_opposes_yaw_nm": -base_opposing,
        "req_abs_contact_moment_nm": 0.0,
        "force_abs_contact_moment_nm": 0.0,
    }

    def feature_value(feature: str, extra: float) -> float:
        total_req = base_opposing + extra
        force_moment = extra
        values = dict(values_base)
        values["req_moment_opposes_yaw_nm"] = -total_req
        values["force_moment_opposes_yaw_nm"] = -force_moment
        values["force_gap_opposes_yaw_nm"] = -base_opposing
        values["req_abs_contact_moment_nm"] = abs(total_req)
        values["force_abs_contact_moment_nm"] = abs(force_moment)
        base, suffix = feature.split("__", 1)
        x = values.get(base, 0.0)
        if suffix == "base":
            return x
        if suffix == "low_rel":
            return x * low_rel
        if suffix == "high_forward":
            return x * high_forward
        if suffix == "util":
            return x * util_smooth
        if suffix == "limiter":
            return x * limiter_smooth
        if suffix == "limiter_signed":
            return x * limiter_smooth
        if suffix == "load_delta":
            return 0.0
        return 0.0

    extra = 0.0
    for _ in range(50):
        predicted = 0.0
        for feature, beta in coeffs.items():
            scale = max(scales.get(feature, 1.0), 1.0e-12)
            value = feature_value(feature, extra)
            limit = 8.0 * scale
            value = min(max(value, -limit), limit)
            predicted += beta * (value / scale)
        if abs(predicted - extra) < 1.0e-12:
            return predicted
        extra = predicted
    return extra


def variant_d_extra(base_opposing: float, constants: dict[str, float], vf_mps: float, yaw_rate: float) -> float:
    rows = read_coeff_rows(
        ROOT / "codex_analysis" / "yaw_model_variant_fits" / "residual_surface" / "ridge_coefficients.csv"
    )
    selected = [row for row in rows if row["model"] == "contact_feature_ridge_surface"]
    coeff = {row["feature"]: float(row["coefficient"]) for row in selected}
    center = {row["feature"]: float(row.get("center") or 0.0) for row in selected if row["feature"] != "intercept"}
    scale = {row["feature"]: float(row.get("scale") or 1.0) for row in selected if row["feature"] != "intercept"}
    longitudinal = constants["drive_wheel_longitudinal_offset_m"]
    patch_q = -(longitudinal * longitudinal) * abs(yaw_rate)
    vbar = longitudinal * abs(yaw_rate)

    def predict(extra: float) -> float:
        total_req = base_opposing + extra
        drive_force_delta = 2.0 * total_req / constants["track_width_m"]
        values = {
            "forward_velocity_mps": vf_mps,
            "yaw_rate_radps": yaw_rate,
            "abs_vf_mps": abs(vf_mps),
            "abs_yaw_rate_radps": abs(yaw_rate),
            "vbar_rel_mps": vbar,
            "vbar_lat_mps": vbar,
            "vbar_yaw_mps": vbar,
            "max_force_preprojection_utilization": contact_utilization(constants, yaw_rate),
            "max_force_limiter_activity": limiter_activity(constants, yaw_rate),
            "patch_yaw_velocity_basis_m2ps": patch_q,
            "patch_yaw_abs_velocity_basis_m2ps": abs(patch_q),
            "patch_yaw_req_basis_nm": total_req,
            "patch_yaw_force_basis_nm": extra,
            "front_rear_vrel_f_abs_delta_mps": 0.0,
            "left_right_vrel_f_abs_delta_mps": 0.0,
            "front_rear_vrel_r_abs_delta_mps": 0.0,
            "left_right_vrel_r_abs_delta_mps": 0.0,
            "front_rear_normal_delta_n": 0.0,
            "left_right_normal_delta_n": 0.0,
            "left_right_drive_force_delta_n": drive_force_delta,
        }
        additive = coeff.get("intercept", 0.0)
        for feature, value in values.items():
            additive += coeff.get(feature, 0.0) * (
                (value - center.get(feature, 0.0)) / max(scale.get(feature, 1.0), 1.0e-12)
            )
        return -additive

    extra = 0.0
    for _ in range(50):
        next_extra = predict(extra)
        if abs(next_extra - extra) < 1.0e-12:
            return next_extra
        extra = 0.5 * extra + 0.5 * next_extra
    return extra


def variant_e_extra(constants: dict[str, float], yaw_rate: float) -> float:
    rows = read_coeff_rows(
        ROOT / "codex_analysis" / "yaw_model_variant_fits" / "aggregate_yaw_loss" / "selected_model_coefficients.csv"
    )
    selected = [row for row in rows if row["model"] == "E1_viscous_yaw_rate_nnls"]
    return sum(float(row["coefficient_nm_per_feature_unit"]) * abs(yaw_rate) for row in selected)


def vf_grid() -> list[float]:
    return [round(i * 0.15 / 5.0, 9) for i in range(6)]


def yaw_grid() -> list[float]:
    return [round(0.2 + i * (6.0 - 0.2) / 9.0, 9) for i in range(10)]


def make_variant_rows(constants: dict[str, float], vf_mps: float, yaw_rate: float) -> list[dict[str, object]]:
    base = baseline_opposing_yaw_torque(constants, yaw_rate)
    variants = [
        ("Baseline", 0.0, "current raw right-contact scrub approximation"),
        ("A_contact_patch", variant_a_extra(constants, vf_mps, yaw_rate), "analysis contact-patch correction"),
        ("B_stribeck", variant_b_extra(base, constants, vf_mps, yaw_rate), "request-activated Stribeck scrub"),
        ("C_combined_slip", variant_c_extra(base, constants, vf_mps, yaw_rate), "production-shaped analysis surface"),
        ("D_residual_surface", variant_d_extra(base, constants, vf_mps, yaw_rate), "diagnostic-only contact-feature ridge"),
        ("E_aggregate_selected", variant_e_extra(constants, yaw_rate), "validation-safe selected aggregate model is no-op"),
    ]
    rows: list[dict[str, object]] = []
    util = contact_utilization(constants, yaw_rate)
    limiter = limiter_activity(constants, yaw_rate)
    for name, extra, caveat in variants:
        total = base + extra
        cmd = motor_commands_for_opposing_torque(total, constants, vf_mps, yaw_rate)
        max_abs_cmd = max(abs(cmd["left_command"]), abs(cmd["right_command"]))
        rows.append(
            {
                "vf_mps": vf_mps,
                "yaw_rate_radps": yaw_rate,
                "variant": name,
                "baseline_opposing_yaw_torque_nm": base,
                "extra_opposing_yaw_torque_nm": extra,
                "total_opposing_yaw_torque_nm": total,
                "required_applied_bank_torque_nm": cmd["applied_bank_torque_nm"],
                "left_command": cmd["left_command"],
                "right_command": cmd["right_command"],
                "lr_delta_command": cmd["lr_delta_command"],
                "left_surface_mps": cmd["left_surface_mps"],
                "right_surface_mps": cmd["right_surface_mps"],
                "left_command_torque_nm": cmd["left_command_torque_nm"],
                "right_command_torque_nm": cmd["right_command_torque_nm"],
                "left_launch_torque_nm": cmd["left_launch_torque_nm"],
                "right_launch_torque_nm": cmd["right_launch_torque_nm"],
                "max_abs_command": max_abs_cmd,
                "command_outside_unit": max_abs_cmd > 1.0,
                "contact_utilization_raw": util,
                "limiter_activity_proxy": limiter,
                "contact_projection_sensitive": util > 1.0,
                "low_speed_launch_sensitive": min(abs(cmd["left_surface_mps"]), abs(cmd["right_surface_mps"]))
                < 2.0 * constants["static_friction_max_speed_mps"],
                "caveat": caveat,
            }
        )
    return rows


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    fields: list[str] = []
    for row in rows:
        for key in row:
            if key not in fields:
                fields.append(key)
    with path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def pivot_lines(rows: list[dict[str, object]]) -> list[str]:
    variants = [
        "Baseline",
        "A_contact_patch",
        "B_stribeck",
        "C_combined_slip",
        "D_residual_surface",
        "E_aggregate_selected",
    ]
    yaws = yaw_grid()
    lines = [
        "# L/R Command Delta Grid",
        "",
        "Values are `left_cmd - right_cmd` for positive clockwise yaw. Commands use the same motor inverse and launch/rolling-friction approximation as the prior 1 rad/s worker.",
        "",
        "Yaw-rate columns are rad/s; `Vf` rows are m/s.",
        "",
    ]
    by_key = {(row["variant"], row["vf_mps"], row["yaw_rate_radps"]): row for row in rows}
    for variant in variants:
        lines.append(f"## {variant}")
        lines.append("")
        header = "| Vf \\ yaw | " + " | ".join(f"{yaw:.3g}" for yaw in yaws) + " |"
        lines.append(header)
        lines.append("| ---: | " + " | ".join("---:" for _ in yaws) + " |")
        for vf in vf_grid():
            values = []
            for yaw in yaws:
                row = by_key[(variant, vf, yaw)]
                values.append(f"{float(row['lr_delta_command']):.3f}")
            lines.append(f"| {vf:.3f} | " + " | ".join(values) + " |")
        lines.append("")
    saturated = sum(1 for row in rows if row["command_outside_unit"])
    projection = sum(1 for row in rows if row["contact_projection_sensitive"])
    launch = sum(1 for row in rows if row["low_speed_launch_sensitive"])
    lines.extend(
        [
            "## Flags",
            "",
            f"- Rows with `|cmd| > 1`: {saturated} of {len(rows)}.",
            f"- Rows where raw contact utilization exceeds 1.0: {projection} of {len(rows)}.",
            f"- Rows near a wheel zero-crossing where launch friction matters: {launch} of {len(rows)}.",
            "",
            "Full per-point commands, torques, contact-utilization flags, and caveats are in `lr_delta_grid.csv`.",
        ]
    )
    return lines


def main() -> None:
    constants = read_constants()
    rows: list[dict[str, object]] = []
    for vf in vf_grid():
        for yaw in yaw_grid():
            rows.extend(make_variant_rows(constants, vf, yaw))

    write_csv(OUT / "lr_delta_grid.csv", rows)
    (OUT / "lr_delta_pivot.md").write_text("\n".join(pivot_lines(rows)) + "\n", encoding="utf-8")
    assumptions = {
        "vf_samples_mps": vf_grid(),
        "yaw_rate_samples_radps": yaw_grid(),
        "right_velocity_mps": 0.0,
        "positive_yaw": "clockwise",
        "wheel_surface_model": "left=Vf+halfTrack*yawRate, right=Vf-halfTrack*yawRate",
        "command_mapping": "inverse MotorEncoderDrive torque model plus PlantModel launch and rolling loss approximation",
        "baseline_note": "uses current right-contact scrub basis; high-utilization cells are flagged because full force projection is not replayed after variant correction",
    }
    (OUT / "assumptions.json").write_text(json.dumps(assumptions, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print((OUT / "lr_delta_grid.csv").as_posix())
    print((OUT / "lr_delta_pivot.md").as_posix())


if __name__ == "__main__":
    main()
