#!/usr/bin/env python3
"""Estimate symmetric in-place command for the tuned yaw-resistance variants.

Analysis-only worker script. It reads existing fit artifacts and writes outputs
only beside this file.
"""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent
CONST_PATH = ROOT / "codex_analysis" / "contact_continuum_yaw_identification" / "features" / "plant_mirror_constants.csv"


def read_constants() -> dict[str, float]:
    with CONST_PATH.open(newline="", encoding="utf-8") as fh:
        return {row["name"]: float(row["value"]) for row in csv.DictReader(fh)}


def read_key_value_csv(path: Path) -> dict[str, float]:
    with path.open(newline="", encoding="utf-8") as fh:
        return {row["parameter"]: float(row["value"]) for row in csv.DictReader(fh)}


def read_coeff_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as fh:
        return list(csv.DictReader(fh))


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
    efficiency = 1.0

    applied_voltage = command * battery
    back_emf_per_wheel_radps = gear_ratio / speed_constant
    current = (applied_voltage / resistance) - ((wheel_speed_radps * back_emf_per_wheel_radps) / resistance)
    armature_sign = (current > 1.0e-6) - (current < -1.0e-6)
    wheel_sign = (wheel_speed_radps > 1.0e-6) - (wheel_speed_radps < -1.0e-6)
    no_load_sign = armature_sign if armature_sign else wheel_sign
    load_current = current - no_load_sign * no_load
    if no_load_sign > 0.0 and load_current < 0.0:
        load_current = 0.0
    elif no_load_sign < 0.0 and load_current > 0.0:
        load_current = 0.0
    return torque_constant * gear_ratio * efficiency * load_current


def command_from_torque(command_torque: float, wheel_speed_radps: float, constants: dict[str, float]) -> float:
    resistance = constants["drive_resistance_ohms"]
    speed_constant = constants["speed_constant_radps_per_volt"]
    torque_constant = constants["torque_constant_nm_per_a"]
    gear_ratio = constants["gear_ratio"]
    battery = constants["drive_voltage_v"]
    no_load = constants["no_load_current_a"]
    efficiency = 1.0

    motor_torque = command_torque / (gear_ratio * efficiency)
    torque_sign = (motor_torque > 1.0e-6) - (motor_torque < -1.0e-6)
    wheel_sign = (wheel_speed_radps > 1.0e-6) - (wheel_speed_radps < -1.0e-6)
    no_load_sign = torque_sign if torque_sign else wheel_sign
    current = (motor_torque / torque_constant) + no_load_sign * no_load
    back_emf = wheel_speed_radps * (gear_ratio / speed_constant)
    return ((current * resistance) + back_emf) / battery


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


def motor_commands_for_opposing_torque(opposing_yaw_torque: float, constants: dict[str, float], yaw_rate: float) -> dict[str, float]:
    track = constants["track_width_m"]
    radius = constants["wheel_radius_m"]
    half_track = 0.5 * track
    left_speed = half_track * yaw_rate / radius
    right_speed = -left_speed
    applied_bank_torque = opposing_yaw_torque * radius / track
    rolling = constants["rolling_friction_torque_nm"]

    left_command_torque = applied_bank_torque + rolling
    right_command_torque = -applied_bank_torque - rolling
    return {
        "applied_bank_torque_nm": applied_bank_torque,
        "left_command_torque_nm": left_command_torque,
        "right_command_torque_nm": right_command_torque,
        "left_command": command_from_torque(left_command_torque, left_speed, constants),
        "right_command": command_from_torque(right_command_torque, right_speed, constants),
        "left_wheel_speed_radps": left_speed,
        "right_wheel_speed_radps": right_speed,
    }


def contact_utilization(constants: dict[str, float], yaw_rate: float) -> float:
    longitudinal = constants["drive_wheel_longitudinal_offset_m"]
    front_force = abs(constants["front_right_contact_force_gain_n_per_mps"] * longitudinal * yaw_rate)
    rear_force = abs(constants["rear_right_contact_force_gain_n_per_mps"] * longitudinal * yaw_rate)
    # With the current sustained-mu derivation and a 50/50, left/right symmetric
    # load, each contact's lateral envelope is mass*a_ref/4 independent of fan load.
    force_limit = constants["mass_kg"] * constants["sustained_lateral_accel_mps2"] / 4.0
    return max(front_force, rear_force) / force_limit


def variant_a_extra(constants: dict[str, float], yaw_rate: float) -> float:
    payload = json.loads(
        (ROOT / "codex_analysis" / "yaw_model_variant_fits" / "contact_patch" / "variant_a_coefficients.json").read_text(
            encoding="utf-8"
        )
    )
    coeffs = payload["coefficients"]
    hp = payload["hyperparameters"]
    longitudinal = constants["drive_wheel_longitudinal_offset_m"]
    q_patch = -(longitudinal * longitudinal) * abs(yaw_rate)
    vbar_rel = longitudinal * abs(yaw_rate)
    low_rel = smooth_gate(vbar_rel, hp["rel_speed_scale_mps"])
    low_forward = 1.0
    tanh_q = math.tanh(q_patch / hp["q_tanh_scale_m2ps"])
    util = contact_utilization(constants, yaw_rate)
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


def variant_b_extra(base_opposing: float, constants: dict[str, float], yaw_rate: float) -> float:
    coeff = read_key_value_csv(ROOT / "codex_analysis" / "yaw_model_variant_fits" / "stribeck_scrub" / "stribeck_coefficients.csv")
    longitudinal = constants["drive_wheel_longitudinal_offset_m"]
    vbar_rel = longitudinal * abs(yaw_rate)
    transition_speed = math.hypot(coeff["rel_weight"] * vbar_rel, 0.0)
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


def variant_c_extra(base_opposing: float, constants: dict[str, float], yaw_rate: float) -> float:
    rows = read_coeff_rows(ROOT / "codex_analysis" / "yaw_model_variant_fits" / "combined_slip_surface" / "model_coefficients.csv")
    coeff_rows = [row for row in rows if row["candidate"] == "saturation_aware_surface"]
    coeffs = {row["feature"]: float(row["standardized_coefficient_nm"]) for row in coeff_rows}
    scales = {row["feature"]: float(row["feature_scale"]) for row in coeff_rows}
    vrel_knee = float(coeff_rows[0]["vrel_knee_mps"])
    fwd_knee = float(coeff_rows[0]["fwd_knee_mps"])

    half_track = 0.5 * constants["track_width_m"]
    longitudinal = constants["drive_wheel_longitudinal_offset_m"]
    v_rel = longitudinal * abs(yaw_rate)
    util = contact_utilization(constants, yaw_rate)
    util_smooth = util / (1.0 + util)
    low_rel = smooth_gate(v_rel, vrel_knee)
    high_forward = 0.0
    limiter = 0.0

    # These match the variant-C feature builder for in-place, zero-forward-slip,
    # left/right symmetric contact geometry.
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
        values["req_abs_contact_moment_nm"] = total_req
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
            return x * limiter
        if suffix == "limiter_signed":
            return x * limiter
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


def variant_d_extra(base_opposing: float, constants: dict[str, float], yaw_rate: float, use_total_req: bool = True) -> float:
    rows = read_coeff_rows(ROOT / "codex_analysis" / "yaw_model_variant_fits" / "residual_surface" / "ridge_coefficients.csv")
    selected = [row for row in rows if row["model"] == "contact_feature_ridge_surface"]
    coeff = {row["feature"]: float(row["coefficient"]) for row in selected}
    center = {row["feature"]: float(row.get("center") or 0.0) for row in selected if row["feature"] != "intercept"}
    scale = {row["feature"]: float(row.get("scale") or 1.0) for row in selected if row["feature"] != "intercept"}
    longitudinal = constants["drive_wheel_longitudinal_offset_m"]
    patch_q = -(longitudinal * longitudinal) * abs(yaw_rate)
    vbar = longitudinal * abs(yaw_rate)

    def predict(extra: float) -> float:
        total_req = base_opposing + extra if use_total_req else base_opposing
        drive_force_delta = 2.0 * total_req / constants["track_width_m"]
        values = {
            "forward_velocity_mps": 0.0,
            "yaw_rate_radps": yaw_rate,
            "abs_vf_mps": 0.0,
            "abs_yaw_rate_radps": abs(yaw_rate),
            "vbar_rel_mps": vbar,
            "vbar_lat_mps": vbar,
            "vbar_yaw_mps": vbar,
            "max_force_preprojection_utilization": contact_utilization(constants, yaw_rate),
            "max_force_limiter_activity": 0.0,
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
            additive += coeff.get(feature, 0.0) * ((value - center.get(feature, 0.0)) / max(scale.get(feature, 1.0), 1.0e-12))
        return -additive

    extra = 0.0
    for _ in range(50):
        next_extra = predict(extra)
        if abs(next_extra - extra) < 1.0e-12:
            return next_extra
        # The diagnostic surface is not guaranteed contractive; damp the solve.
        extra = 0.5 * extra + 0.5 * next_extra
    return extra


def variant_e_extra(constants: dict[str, float], yaw_rate: float) -> tuple[float, float]:
    rows = read_coeff_rows(ROOT / "codex_analysis" / "yaw_model_variant_fits" / "aggregate_yaw_loss" / "selected_model_coefficients.csv")
    selected = [row for row in rows if row["model"] == "E1_viscous_yaw_rate_nnls"]
    selected_extra = sum(float(row["coefficient_nm_per_feature_unit"]) * abs(yaw_rate) for row in selected)

    rejected = [row for row in rows if row["model"] == "E5_contact_faded_loss_nnls_yaw0_0.5_vrel0_0.01"]
    coeff = {row["feature"]: float(row["coefficient_nm_per_feature_unit"]) for row in rejected}
    longitudinal = constants["drive_wheel_longitudinal_offset_m"]
    vbar = longitudinal * abs(yaw_rate)
    gate = smooth_gate(vbar, 0.01)
    rejected_extra = (
        coeff.get("vrel_gate_0.01_tanh_yaw_0.5", 0.0) * gate * math.tanh(abs(yaw_rate) / 0.5)
        + coeff.get("vrel_gate_0.01_abs_yaw", 0.0) * gate * abs(yaw_rate)
    )
    return selected_extra, rejected_extra


def make_row(name: str, extra: float, base: float, constants: dict[str, float], yaw_rate: float, caveat: str) -> dict[str, object]:
    total = base + extra
    cmd = motor_commands_for_opposing_torque(total, constants, yaw_rate)
    return {
        "variant": name,
        "extra_opposing_yaw_torque_nm": extra,
        "total_opposing_yaw_torque_nm": total,
        "required_applied_bank_torque_nm": cmd["applied_bank_torque_nm"],
        "left_command": cmd["left_command"],
        "right_command": cmd["right_command"],
        "left_command_torque_nm": cmd["left_command_torque_nm"],
        "right_command_torque_nm": cmd["right_command_torque_nm"],
        "caveat": caveat,
    }


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


def main() -> None:
    constants = read_constants()
    yaw_rate = 1.0
    base = baseline_opposing_yaw_torque(constants, yaw_rate)
    e_selected, e_rejected = variant_e_extra(constants, yaw_rate)

    rows = [
        make_row("Current baseline", 0.0, base, constants, yaw_rate, "Current PlantModel contact-right scrub only."),
        make_row("Variant A contact patch", variant_a_extra(constants, yaw_rate), base, constants, yaw_rate, "Direct algebraic evaluation; no command-dependent terms."),
        make_row("Variant B Stribeck scrub", variant_b_extra(base, constants, yaw_rate), base, constants, yaw_rate, "Fixed point through request-only activation."),
        make_row("Variant C combined slip", variant_c_extra(base, constants, yaw_rate), base, constants, yaw_rate, "Approximate fixed point; production-relevant shape but analysis fit omits full force replay."),
        make_row("Variant D residual surface", variant_d_extra(base, constants, yaw_rate), base, constants, yaw_rate, "Diagnostic-only contact-feature ridge; command mapping is not production meaningful."),
        make_row("Variant E aggregate selected", e_selected, base, constants, yaw_rate, f"Validation-safe selected model is no-op; rejected best nonzero would add {e_rejected:.6f} Nm."),
    ]

    write_csv(OUT / "in_place_1radps_command_estimate.csv", rows)
    assumptions = {
        "yaw_rate_radps": yaw_rate,
        "forward_velocity_mps": 0.0,
        "right_velocity_mps": 0.0,
        "left_wheel_speed_radps": motor_commands_for_opposing_torque(base, constants, yaw_rate)["left_wheel_speed_radps"],
        "right_wheel_speed_radps": motor_commands_for_opposing_torque(base, constants, yaw_rate)["right_wheel_speed_radps"],
        "baseline_opposing_yaw_torque_nm": base,
        "contact_utilization": contact_utilization(constants, yaw_rate),
        "rolling_friction_torque_per_bank_nm": constants["rolling_friction_torque_nm"],
        "static_launch_torque_decay_note": "At 0.0423 m/s wheel surface speed, exp(-(v/0.005)^2) is effectively zero.",
    }
    (OUT / "assumptions.json").write_text(json.dumps(assumptions, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    print("| Variant | Extra opposing yaw torque Nm | Required bank torque Nm | Left cmd | Right cmd | Caveat |")
    print("| --- | ---: | ---: | ---: | ---: | --- |")
    for row in rows:
        print(
            f"| {row['variant']} | {row['extra_opposing_yaw_torque_nm']:.6f} | "
            f"{row['required_applied_bank_torque_nm']:.6f} | {row['left_command']:.3f} | "
            f"{row['right_command']:.3f} | {row['caveat']} |"
        )
    print("")
    print(json.dumps(assumptions, sort_keys=True))


if __name__ == "__main__":
    main()
