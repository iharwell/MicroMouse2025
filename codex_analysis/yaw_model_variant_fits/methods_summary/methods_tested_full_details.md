# Yaw Residual Model Methods Tested - Full Details

Analysis-only summary compiled from existing artifacts under `codex_analysis/yaw_model_variant_fits`. No production code, build metadata, tests, or model tuning were changed.

## Shared Evaluation Basis

Primary feature basis: `codex_analysis/contact_continuum_yaw_identification/ablation/phase_classified_feature_sample.csv`, with selected logs and split policy defined in `codex_analysis/yaw_model_variant_fits/evaluation_contract.md`.

Common residual target convention:

- `residual_additive_yaw_torque_nm` is the raw additive yaw torque target, derived from gyro-differentiated observed yaw moment minus the PlantModel mirror.
- Many fits use `residual_opposes_yaw_nm = -sign(yaw_rate) * residual_additive_yaw_torque_nm`, then convert predictions back with `M_add = -sign(yaw_rate) * M_opp`.
- RMSE values below are residual yaw torque RMSE in Nm unless explicitly labeled as command.

The most repeated baseline split RMSE values were:

| Split | Rows | Baseline RMSE |
| --- | ---: | ---: |
| `primary_open_floor_fit_authoritative` | 47317 | 0.036866 |
| `open_floor_fit_downweighted` | 31165 | 0.043194 |
| `open_floor_validation_only` | 14542 | 0.016931 |
| `diag_validation_only` | 11108 | 0.084621 |
| `aux_downweighted_validation` | 14448 | 0.051266 |
| `validation_non_authoritative` | 71263 | 0.050234 |

The common in-place reference condition is `Vf=0`, `Vr=0`, `yawRate=+1 rad/s`. The project-owner-relevant measured/calculated target command is approximately `+0.646/-0.646`, and round2 hard-gate reports require `min(abs(left), abs(right)) >= 0.600`.

## Consolidated Status Table

| Method | In-place extra opposing torque | In-place left/right command | Primary RMSE | Validation RMSE | Production status |
| --- | ---: | ---: | ---: | ---: | --- |
| Current PlantModel baseline | 0.000000 | +0.261744 / -0.261744 | 0.036866 | 0.050234 | Current reference, insufficient yaw-launch authority |
| First-round A contact patch | -0.001281 | +0.254234 / -0.254234 | 0.029422 primary; 0.032822 weighted fit | 0.041942 validation/competition | Analysis-only patch, not enough in-place authority |
| First-round B Stribeck/request-only | 0.065013 | +0.642750 / -0.642750 | 0.028528 | 0.048885 non-authoritative approx | Rejected for command/request conditioning; useful scale reference |
| First-round C combined-slip/contact surface | 0.018241 | +0.368641 / -0.368641 | 0.024120 | 0.030342 | Best broad first-round physical-shape evidence, but too weak at launch |
| First-round D residual surface diagnostic | 0.012685 | +0.336085 / -0.336085 | 0.027495 best contact ridge | selected-log aggregate 0.029117 | Diagnostic only, not a runtime residual map |
| First-round E aggregate yaw loss | 0.000000 selected; rejected nonzero would add 0.018953 | +0.261744 / -0.261744 selected | 0.036866 selected no-op | 0.050234 selected no-op | Do not add aggregate yaw loss |
| round2_state_minimal_lugre | 0.065013 | +0.642750 / -0.642750 | 0.029042 state; 0.028548 memoryless | 0.049128 state; 0.048891 memoryless | Rejected in fitted request-driven form |
| round2_request_contact_surface | 0.061998 | +0.625079 / -0.625079 | 0.030451 | 0.042721 | Gate pass, not production-ready without force replay |
| round2_hybrid_b_c adhesion partition | 0.064633 | +0.640520 / -0.640520 | 0.029752 | 0.045881 | Gate pass, broad validation worse than C |
| round2_hybrid_b_c dynamic LuGre/Bristle | 0.079622 | +0.728361 / -0.728361 | 0.020907 | 0.033502 | Physically compliant but rejected as production tune; not C-like validation |
| round2_force_domain_stribeck | 0.067115 | +0.655064 / -0.655064 | 0.027908 | 0.048820 aux; 0.084410 diag; 0.011394 open-val | Best force-domain B rewrite; eligible shape, not full replacement for C |
| round2_b_correct_branch_reference | 0.065013 | +0.642750 / -0.642750 | 0.028528 | same as B selected splits | Correct B reference only; production rejected |
| round2_static_yield_contact | 0.065007 | +0.642715 / -0.642715 | 0.028110 | 0.052072 | Static-yield architecture useful; fit not production-ready |

## 1. Baseline / Current PlantModel

Source references:

- `codex_analysis/yaw_model_variant_fits/in_place_1radps_command/estimate_in_place_1radps_command.py`
- `codex_analysis/yaw_model_variant_fits/in_place_1radps_command/in_place_1radps_command_estimate.csv`
- `codex_analysis/yaw_model_variant_fits/lr_delta_grid/estimate_lr_delta_grid.py`
- `codex_analysis/yaw_model_variant_fits/evaluation_contract.md`

Name and purpose: current PlantModel mirror baseline. It is the comparison point for every residual correction and the source of the raw `model_yaw_moment_nm` subtracted from gyro-derived observed yaw moment.

Core form as represented in the analysis helpers:

```text
front_right_velocity = -drive_wheel_longitudinal_offset * yaw_rate
rear_right_velocity  =  drive_wheel_longitudinal_offset * yaw_rate
front_right_force_total = 2 * front_right_contact_force_gain * front_right_velocity
rear_right_force_total  = 2 * rear_right_contact_force_gain * rear_right_velocity
yaw_moment = drive_wheel_longitudinal_offset * (front_right_force_total - rear_right_force_total)
baseline_opposing_yaw_torque = -yaw_moment
```

Physical inputs: PlantModel mirror constants, yaw rate, geometry, right-contact velocity gains, and motor inverse only for command estimate. It does not use UKF fields. It does not use command/request as a traction selector in the in-place helper, although the broader feature target is generated from logs containing commands, encoders, gyro, and PlantModel mirror outputs.

In-place reference: extra opposing torque `0`, total opposing torque `0.01479425`, required applied bank torque `0.00220424`, left/right command `+0.261743981/-0.261743981`, L-R delta `0.523487962`. This is far below the `>=0.600` command gate and the approximately `+0.646/-0.646` target.

RMSE performance: baseline split RMSE values are the common values listed above, notably `0.036866` primary and `0.050234` combined non-authoritative validation.

Strengths: deterministic, current production reference, simple physics/geometry path, no fitted residual table.

Sacrifices/failure modes: underprices low-speed in-place yaw launch by command magnitude; residual reports show systematic yaw-resistance mismatch. It is only the starting point, not an adequate yaw residual model.

Production recommendation: current reference only. Do not treat its existing behavior as sufficient for yaw-launch authority.

## 2. First-Round A - Contact Patch / Contact Velocity Residual

Source references:

- `codex_analysis/yaw_model_variant_fits/contact_patch/variant_a_report.md`
- `codex_analysis/yaw_model_variant_fits/contact_patch/variant_a_coefficients.json`
- `codex_analysis/yaw_model_variant_fits/in_place_1radps_command/in_place_1radps_command_estimate.csv`

Name and purpose: force-level contact patch yaw correction. It tested whether a PlantModel-local contact velocity basis could explain residual yaw torque without a static breakaway branch.

Core form:

```text
q = sum_i load_fraction_i * (f_i * v_rel_r_i - r_i * v_rel_f_i)
g_rel = 1 / (1 + (vbar_rel / s_rel)^2)
g_vf  = 1 / (1 + (Vf / s_vf)^2)
delta_Mz = beta dot [
  q,
  q*g_rel,
  q*g_vf,
  q*g_rel*g_vf,
  tanh(q/q0)*g_rel*g_vf,
  tanh(q/q0)*g_rel,
  q*u,
  q*u*g_rel,
  q*abs(q)
]
```

Selected hyperparameters: `s_rel=0.025 m/s`, `s_vf=0.45 m/s`, `q0=0.0025 m^2/s`, `ridge_lambda=1e-05`.

Physical inputs: per-contact relative velocity, contact geometry, load fraction, `Vf`, `vbar_rel`, and squashed preprojection contact utilization. No UKF fields. Command/request is not a traction selector; utilization is a reconstructed contact-force/projection signal. Lateral body velocity was unavailable, so right-relative velocity assumes the feature extractor's `Vr=0` reconstruction.

In-place reference: extra opposing torque `-0.001281396`, total opposing torque `0.013512854`, left/right command `+0.254234/-0.254234`. It slightly reduces the baseline in-place command and fails the hard gate.

RMSE performance:

- Weighted fit input: `0.038933 -> 0.032822` (-15.70%).
- Primary fit-authoritative split: `0.036866 -> 0.029422` (-20.19%).
- Primary selected-run holdout: `0.042286 -> 0.029916` (-29.25%).
- Open-floor downweighted: `0.043194 -> 0.036381` (-15.77%).
- Validation/competition: `0.055087 -> 0.041942` (-23.86%).
- High-forward yaw sanity: `0.083872 -> 0.066597` (-20.60%).

Strengths: uses contact-local continuous velocity basis; improved selected-run and validation RMSE; does not introduce a runtime residual map.

Sacrifices/failure modes: cannot represent true static yaw breakaway at exactly zero patch velocity; saturation behavior is approximate because the analysis does not replay full force projection after correction; still affected by transient timing and gyro differentiation noise; fails the +1 rad/s in-place command gate.

Production recommendation: not sufficient as the yaw-launch fix. Useful evidence that contact patch velocity matters, but it lacks a static-yield/pre-sliding mechanism.

## 3. First-Round B - Stribeck / Request-Only Branch

Source references:

- `codex_analysis/yaw_model_variant_fits/stribeck_scrub/stribeck_scrub_report.md`
- `codex_analysis/yaw_model_variant_fits/stribeck_scrub/stribeck_coefficients.csv`
- `codex_analysis/yaw_model_variant_fits/round2_b_correct_branch_reference/round2_b_correct_branch_reference_report.md`

Name and purpose: Stribeck static-to-sliding yaw scrub fit. It tested whether a low-speed yaw-opposing resistance term could recover the missing in-place/low-speed launch torque.

Core first-round form:

```text
M_opp = A(v_yaw, M_req) * R(v_transition) *
        (K_slide + K_static * exp(-(v_transition / v_s)^2))
        + K_viscous * v_yaw * R(v_transition)

v_transition = sqrt((rel_weight * vbar_rel)^2 + abs(Vf)^2)
R(v_transition) = 1 / (1 + (v_transition / speed_fade)^2)
M_add = -sign(yaw_rate) * M_opp
```

Selected branch: request-only activation, no yaw-viscous term. Round2 later clarified the exact branch as:

```text
M_extra = A_req(M_base + M_extra) * R(v_transition) *
          (K_slide + K_static * S(v_transition))
A_req(x) = 1 - exp(-(smooth_positive(x) / req_activation_nm)^2)
S(v) = exp(-(v / stribeck_speed_mps)^2)
```

Coefficients: `req_activation_nm=0.035`, `stribeck_speed_mps=0.1`, `speed_fade_mps=0.64`, `rel_weight=0.75`, `static_extra_nm=0.00239126`, `sliding_nm=0.0630316`, `weighted_train_opposes_rmse_nm=0.0243064`.

Physical inputs: first-round B used `Vf`, contact-relative speed, yaw sign, and requested/preprojection yaw moment activation. No UKF fields. Command/request is used as a traction activation input, which is the later production rejection.

In-place reference: extra opposing torque `0.065013359`, total opposing torque `0.079807609`, left/right command `+0.642749606/-0.642749606`, L-R delta `1.285499211`. This passes the `>=0.600` gate and matches the owner-relevant `+0.646/-0.646` scale.

RMSE performance:

- Primary: `0.036866 -> 0.028528` (22.618%).
- Open-floor downweighted: `0.043194 -> 0.041880` (3.042%).
- Open-floor validation: `0.016931 -> 0.011372` (32.833%).
- Diag validation: `0.084621 -> 0.084412` (0.247%).
- Aux validation: `0.051266 -> 0.048945` (4.528%).
- Selected `2026-05-04_20-35-47`: `0.035846 -> 0.021129` (41.057%).

Strengths: hits the in-place command scale; simple low-speed Stribeck intuition; strong on some low-speed/open-floor validation and May 4 launch-like logs.

Sacrifices/failure modes: always opposes yaw, so it cannot correct current-model over-resistance; fades deliberately and is not a full combined-slip model; weak on diag/aux; depends on requested/preprojection yaw moment, violating the command-invariance rule.

Production recommendation: rejected as production traction law. Keep as a diagnostic/reference for the needed yaw-launch command magnitude.

## 4. First-Round C - Combined-Slip / Contact Surface

Source references:

- `codex_analysis/yaw_model_variant_fits/combined_slip_surface/variant_c_combined_slip_surface_report.md`
- `codex_analysis/yaw_model_variant_fits/combined_slip_surface/model_coefficients.csv`
- `codex_analysis/yaw_model_variant_fits/in_place_1radps_command/in_place_1radps_command_estimate.csv`

Name and purpose: production-shaped contact surface that fits yaw-aligned residual torque from contact-relative velocities, projected force state, load, utilization, limiter activity, and smooth schedules.

Core form:

```text
predicted_raw = -sign(yaw_rate) * predicted_opposes
corrected_residual = residual_additive_yaw_torque_nm - predicted_raw

right-force gain basis ~ -sign(yaw) * f_i * v_rel_r_i
longitudinal gain basis ~ sign(yaw) * r_i * v_rel_f_i
low_rel = 1/(1+(vbar_rel/k_rel)^2)
high_forward = 1 - 1/(1+(|Vf|/k_fwd)^2)
```

Selected candidate: `saturation_aware_surface` with `k_rel=0.060 m/s`, `k_fwd=0.700 m/s`, ridge `0.001`, nominal load `1.932931 N`.

Physical inputs: contact-relative velocities, wheel/contact geometry, normal load, projected contact yaw moment, requested-vs-projected contact moment difference, force utilization, limiter activity, total-load delta, `Vf`, yaw sign. No UKF fields. It uses reconstructed request/projected force features as physical-force features, not as a discrete command selector, but production would need full force replay and a continuity-preserving yaw-sign treatment.

In-place reference: extra opposing torque `0.018240534`, total opposing torque `0.033034784`, left/right command `+0.368641/-0.368641`; fails the `>=0.600` gate.

RMSE performance:

- Primary: `0.036866 -> 0.024120` (run-balanced change 35.3%).
- Downweighted: `0.043194 -> 0.033093`.
- Open-floor validation: `0.016931 -> 0.014417`.
- Diag validation: `0.084621 -> 0.038767`.
- Aux validation: `0.051266 -> 0.028529`.
- Combined non-authoritative validation: `0.050234 -> 0.030342` (40.9%).
- Selected `diag003`: `0.085238 -> 0.038970`.

Strengths: strongest broad first-round validation; contact-primitive based; continuous schedules; no runtime residual table; improves straightish, low-speed yaw, high-forward, and limiter risk groups.

Sacrifices/failure modes: underprices the in-place yaw-launch command; near-zero yaw can receive sign-noise corrections; analysis fit does not replay full force projection; still residual-fit evidence, not final production tune.

Production recommendation: best first-round production-shape evidence for moving-contact behavior, but not sufficient alone because it remains C-scale at yaw launch.

## 5. First-Round D - Residual Surface Diagnostic

Source references:

- `codex_analysis/yaw_model_variant_fits/residual_surface/residual_surface_report.md`
- `codex_analysis/yaw_model_variant_fits/residual_surface/ridge_coefficients.csv`
- `codex_analysis/yaw_model_variant_fits/in_place_1radps_command/in_place_1radps_command_estimate.csv`

Name and purpose: diagnostic residual surfaces to compare raw `Vf/yaw` lookup shapes against contact-feature predictors.

Model forms tested:

- `expanded_signed_bin_lookup`: existing `0.10 m/s x 0.50 rad/s` signed-bin median residual table.
- `vf_yaw_ridge_surface`: continuous ridge over forward speed, yaw rate, absolute terms, interactions, `vbar_rel`, utilization.
- `contact_feature_ridge_surface`: continuous ridge over contact-continuum velocity, force, load, and request primitives.
- `vf_yaw_vbar_kernel_cell_surface`: median residual cells keyed by `Vf`, yaw rate, and `vbar_rel`, Gaussian blended.

Physical inputs: depends on the candidate. The best model used contact-continuum feature variables. No UKF fields. These are residual targets/fitted maps, not physical traction laws; request/contact primitives appear as fitted predictors in a diagnostic residual map.

In-place reference for the best diagnostic contact ridge: extra opposing torque `0.012685325`, total opposing torque `0.027479575`, left/right command `+0.336085/-0.336085`; fails the gate. The report also lists one-step yaw-rate signed-bin references, but those are for the existing bin model and not the best contact ridge.

RMSE performance:

- Best primary: contact feature ridge `0.036866 -> 0.027495` (25.4%).
- Best selected-log aggregate: `0.051680 -> 0.029117` (43.7%).
- Leave-selected-run-out mean corrected RMSE: contact feature ridge `0.027902`, mean improvement 31.5%.
- Existing signed-bin table worsened primary (`0.036866 -> 0.041341`) and selected logs (`0.051680 -> 0.054823`) with low coverage.

Strengths: clearly shows contact features beat raw `Vf/yaw` tables; useful as an upper-bound diagnostic and coverage audit.

Sacrifices/failure modes: residual map layered on physics, sparse coverage in high-forward/high-yaw and low-speed launch regions, gyro-differentiated single-sample targets, no lateral velocity measurement, not a runtime physics law.

Production recommendation: do not productionize Variant D. Move the contact-feature signal into PlantModel equations instead.

## 6. First-Round E - Aggregate Yaw Loss

Source references:

- `codex_analysis/yaw_model_variant_fits/aggregate_yaw_loss/aggregate_yaw_loss_report.md`
- `codex_analysis/yaw_model_variant_fits/aggregate_yaw_loss/selected_model_coefficients.csv`
- `codex_analysis/yaw_model_variant_fits/in_place_1radps_command/in_place_1radps_command_estimate.csv`

Name and purpose: tested whether a simple aggregate yaw-loss term, independent of detailed contact geometry, should be added.

Selected validation-guarded form:

```text
E1_viscous_yaw_rate_nnls:
loss = k_yaw * |yaw_rate|
```

The validation-safe selected coefficient is `k_yaw=0`, i.e. no-op. Best nonzero physical candidate:

```text
loss = gate_vrel(0.01) *
       [tau_c * tanh(|yaw|/0.5) + k_yaw * |yaw|]
```

Physical inputs: aggregate yaw rate and optional contact-relative-speed fading. No UKF fields. No command/request traction selector. Signed-relief diagnostics can add yaw-assist, but those are no longer physical aggregate loss.

In-place reference: selected no-op gives extra opposing torque `0`, total `0.01479425`, left/right `+0.261744/-0.261744`. The rejected best nonzero physical model would add about `0.018953 Nm`, still far below B-scale.

RMSE performance:

- Selected no-op: primary `0.036866177 -> 0.036866177`; validation `0.050233832 -> 0.050233832`; validation worsened-sample fraction 0%.
- Best nonzero physical: primary `0.036866177 -> 0.036561548` (-0.826%) but validation `0.050233832 -> 0.051093245` (+1.711% worse), worsened-sample fraction 69.402%.
- Best signed-relief diagnostic validation: `0.050233832 -> 0.053275506` (+6.055% worse).

Strengths: establishes that a generic aggregate yaw-loss term is not supported by validation; avoids adding one-sided resistance where the current plant already over-resists.

Sacrifices/failure modes: cannot express front/rear or left/right patch asymmetry; pure loss worsens negative-target over-resisted rows; signed relief stops being an aggregate loss model.

Production recommendation: do not add aggregate yaw loss from this family.

## 7. round2_state_minimal_lugre

Source references:

- `codex_analysis/yaw_model_variant_fits/round2_state_minimal_lugre/round2_state_minimal_lugre_report.md`
- `codex_analysis/yaw_model_variant_fits/round2_state_minimal_lugre/lugre_coefficients.csv`
- `codex_analysis/yaw_model_variant_fits/round2_state_minimal_lugre/production_reassessment.csv`

Name and purpose: single scalar bristle-fill state for yaw/contact aggregate, testing whether dynamic pre-sliding memory can supply B-scale launch torque.

Fitted request-driven state form:

```text
v_c = sqrt((w_rel * vbar_rel)^2 + |Vf|^2)
A = 1 - exp(-(positive(M_req) / M_act)^2)
q_eq = A / (1 + tau_fill * v_c / x_slip)
dq/dt = (A - q) / tau_fill - (v_c / x_slip) * q
R = 1 / (1 + (v_c / v_fade)^2)
S = exp(-(v_c / v_s)^2)
M_extra = q * R * (K_static * S + K_slide + K_visc * vbar_yaw)
M_add = -sign(yaw) * M_extra
```

Memoryless approximation replaces replayed state with `q=q_eq`.

Physical inputs: `Vf`, `vbar_rel`, `vbar_yaw`, yaw sign, and requested/preprojection yaw moment for activation in the fitted artifacts. No UKF fields. Command/request is used as the bristle fill selector in this fit, which violates the later command-invariance rule.

Production-compliant revision proposed in the report:

```text
dot(z_i) = v_t_i - (|v_t_i| / g_i(|v_t_i|, N_i)) * z_i
g_i = (F_c_i + (F_s_i - F_c_i) * exp(-(|v_t_i|/v_s)^2)) / sigma0_i
F_t_i = clamp(sigma0_i*z_i + sigma1_i*dot(z_i) + sigma2_i*v_t_i,
              -mu_i*N_i, +mu_i*N_i)
M_yaw = sum_i cross(r_i, F_t_i)
```

In-place reference: state steady and memoryless both extra `0.065013`, total `0.079808`, left/right `+0.642750/-0.642750`, pass. Cold 20 ms state: extra `0.063764`, left/right `+0.635427/-0.635427`, pass.

RMSE performance:

- State: primary `0.029042`, validation `0.049128`.
- Memoryless: primary `0.028548`, validation `0.048891`.
- Versus B/C, state is slightly worse than B on most B-like splits and far worse than C on diag/aux. For `validation_non_authoritative`, C is `0.030342`, state `0.049128`, memoryless `0.048891`.

Strengths: introduces the correct missing degree of freedom conceptually: pre-sliding/bristle history can make two identical instantaneous rows differ. It passes the in-place command scale.

Sacrifices/failure modes: fitted implementation is request-driven, not force/slip-driven; broad validation not competitive with C; production state would need explicit reset/decay at yaw reversal, lift/service, or discontinuous session boundaries.

Production recommendation: reject the fitted request-driven state and memoryless forms. Keep as diagnostic magnitude evidence and refit only as a per-contact force/slip/load driven PlantModel state.

## 8. round2_request_contact_surface

Source references:

- `codex_analysis/yaw_model_variant_fits/round2_request_contact_surface/request_contact_surface_report.md`
- `codex_analysis/yaw_model_variant_fits/round2_request_contact_surface/failure_modes.md`
- `codex_analysis/yaw_model_variant_fits/round2_request_contact_surface/split_rmse_vs_b_c.csv`
- `codex_analysis/yaw_model_variant_fits/round2_request_contact_surface/in_place_1radps_command_estimate.csv`
- `codex_analysis/yaw_model_variant_fits/round2_request_contact_surface/fit_request_contact_surface.py`

Name and purpose: despite the directory/script name, the report describes the selected round-2 force-coupled contact-velocity surface. It preserves B-scale launch authority while adding a moving-contact branch based on bounded per-contact slip traction proxies.

Core form:

```text
R_total = R_launch + DeltaR_surface
R_launch = A_force * F_speed *
           (K_slide + K_static * exp(-(v_transition / v_s)^2))
A_force = 1 - exp(-(abs(M_applied_contact_force) / M_force_knee)^2)
v_transition = sqrt((rel_weight * vbar_rel)^2 + abs(Vf)^2)
F_speed = 1 / (1 + (v_transition / v_fade)^2)

F_contact_i = -N_i * [tanh(v_rel_f_i / v_f_k),
                      tanh(v_rel_r_i / v_r_k)]
M_contact_opp = -sign(yaw) * sum_i(f_i * F_contact_r_i - r_i * F_contact_f_i)
DeltaR_surface = G_move * (beta_c*M_contact_opp
                           + beta_f*abs(M_applied_contact_force)
                           + beta_gap*abs(M_force_gap)
                           + beta_v*vbar_rel + ...)
G_move = A_force * (1 - low_rel * low_forward)
M_corr = -sign(yaw_rate_or_applied_force) * R_total
```

Selected hyperparameters: `vrel_knee_mps=0.080`, `fwd_knee_mps=0.350`, ridge `0.01`, fixed launch `K_static=0.002391255`, `K_slide=0.063031582`.

Physical inputs: contact-relative velocities, normal loads, projected/requested contact force moments, force-gap, utilization, limiter/saturation quality, `Vf`, yaw/contact sign. The selected report states command values are not features and do not select traction mode; command is used only downstream to estimate motor command. No UKF fields.

In-place reference: extra opposing torque `0.061998197`, total `0.076792447`, left/right `+0.625079/-0.625079`, pass.

RMSE performance:

- Primary: `0.036866 -> 0.030451` (17.402%).
- Open-floor downweighted: `0.043194 -> 0.040701` (5.772%).
- Open-floor validation: `0.016931 -> 0.019477` (-15.040%, worse).
- Diag: `0.084621 -> 0.063325` (25.167%).
- Aux: `0.051266 -> 0.044315` (13.559%).
- Combined non-authoritative validation: `0.050234 -> 0.042721` (14.956%), still much worse than C's `0.030342`.

Strengths: passes the in-place gate without command as a direct traction selector; introduces bounded `N*tanh(v_rel/v_k)` contact-creep mechanism rather than arbitrary low-order features; materially improves diag/aux versus baseline.

Sacrifices/failure modes: still a residual correction over reconstructed contact features; synthetic grid is approximate and does not replay full PlantModel projection after correction; lateral velocity assumes `Vr=0`; worsens open-floor validation; not competitive with C on broad validation.

Production recommendation: promising physical shape, not production-ready. Needs full force replay and targeted yaw-launch/low-speed-turn validation.

## 9. round2_hybrid_b_c - Adhesion Partition and Dynamic LuGre/Dahl

Source references:

- `codex_analysis/yaw_model_variant_fits/round2_hybrid_b_c/hybrid_b_c_report.md`
- `codex_analysis/yaw_model_variant_fits/round2_hybrid_b_c/dynamic_bristle_lugre_report.md`
- `codex_analysis/yaw_model_variant_fits/round2_hybrid_b_c/hybrid_model_coefficients.csv`
- `codex_analysis/yaw_model_variant_fits/round2_hybrid_b_c/dynamic_bristle_coefficients.csv`

### Adhesion Partition Hybrid

Name and purpose: a coherent hybrid combining a static bristle-displacement reservoir with Variant C-like moving contact features. It attempted to keep B's launch scale while retaining C-like contact-surface breadth.

Core form:

```text
M_opp_pred =
  G_slide * X_contact(v_contact, F_projected, load, utilization_actual) * beta_contact
  + B(v_y, N) * R(v_t) *
    (K_slide + K_static * exp(-(v_t / v_s)^2))

v_t = sqrt((rel_weight * max(vbar_rel, load_weighted_rel))^2 + |Vf|^2)
v_y = rel_weight * max(vbar_yaw, load_weighted_lat)
B(v_y,N) = (N/N_nominal) * (1 - exp(-(v_y / 0.008)^2))
R(v_t) = 1/(1+(v_t/0.640)^2)
G_slide = (1 - clamp((1 - exp(-(v_y/v_bristle)^2))
                    * exp(-(v_t/v_s)^2) * R(v_t), 0, 1))^2
M_raw_pred = -d_yaw * M_opp_pred
```

Physical inputs: contact-relative velocity, load, projected contact force, utilization, and yaw/contact direction. It explicitly rejects requested yaw moment as a static reservoir selector. No UKF fields. B and C are used as calibration/comparison references.

In-place reference: extra `0.064633`, total `0.079427`, left/right `+0.640520/-0.640520`, pass.

RMSE performance:

- Primary: `0.036866 -> 0.029752`.
- Open-floor downweighted: `0.043194 -> 0.042732`.
- Open-floor validation: `0.016931 -> 0.025884` (worse).
- Diag: `0.084621 -> 0.073588`.
- Aux: `0.051266 -> 0.040079`.
- Combined non-authoritative: `0.050234 -> 0.045881`, worse than C's `0.030342`.

Strengths: no command/request traction selector; passes launch gate; articulates the static reservoir as physical bristle displacement.

Sacrifices/failure modes: broad validation weaker than C; moving-contact coefficients are sensitive to force-projection changes; high-yaw grid cells can exceed unit command/contact utilization; near-zero yaw direction fallback needs a continuous physical sign convention.

Production recommendation: not the selected production tune. Useful as architecture evidence for separating static adhesion from sliding surface.

### Dynamic LuGre/Bristle Model

Name and purpose: dynamic LuGre/Dahl-style bristle deflection state replayed from physical yaw/contact relative velocity history, not command/request.

Core form:

```text
z_dot = v_y / L - |v_y| * z / (L * g(v_t))
g(v_t) = mu_c + (1 - mu_c) * exp(-(v_t / v_s)^2)
M_opp = K_z * (N/N0) * max(0, d_yaw*z)
        + X_force_state(v_contact, F_projected, N, utilization_actual) * beta
```

Selected parameters: `L=0.002 m`, `stribeck_speed=0.060 m/s`, `coulomb_fraction=0.650`, ridge `0.030`, `static_gain=0.065783 Nm`, nominal load `1.932931 N`.

Physical inputs: physical yaw/contact relative velocity history, normal load, projected force state, utilization. No command, requested force, requested yaw moment, selector labels, residual lookup table, or UKF fields are traction inputs. B/C are only comparison/calibration references.

In-place reference: extra `0.079622`, total `0.094416`, left/right `+0.728361/-0.728361`, pass.

RMSE performance:

- Primary: `0.036866 -> 0.020907`, better than B `0.028528` and C `0.024120`.
- Open-floor downweighted: `0.043194 -> 0.025782`.
- Open-floor validation: `0.016931 -> 0.010405`.
- Diag: `0.084621 -> 0.057896`, worse than C `0.038767`.
- Aux: `0.051266 -> 0.037632`, worse than C `0.028529`.
- Combined non-authoritative: `0.050234 -> 0.033502`, worse than C `0.030342`.

Strengths: physically compliant with the command-invariance rule; dynamic state is genuinely new and can represent hysteresis/history; passes launch gate; excellent primary/downweighted/open-floor performance.

Sacrifices/failure modes: does not retain C-like broad validation on diag/aux; state was identified from sparse analysis rows rather than full-rate replay; production needs clear integration/reset semantics and stricter physical priors on moving-surface coefficients.

Production recommendation: rejected as a production tune in the report despite physical compliance, because validation did not beat/retain C-like broad performance. Best used as a candidate mechanism for future full-rate force/slip-driven identification.

## 10. round2_force_domain_stribeck

Source references:

- `codex_analysis/yaw_model_variant_fits/round2_force_domain_stribeck/force_domain_stribeck_report.md`
- `codex_analysis/yaw_model_variant_fits/round2_force_domain_stribeck/force_domain_coefficients.csv`
- `codex_analysis/yaw_model_variant_fits/round2_force_domain_stribeck/in_place_1radps_command.csv`

Name and purpose: physically acceptable rewrite of Variant B. It keeps B's static-to-sliding Stribeck torque law but replaces raw request activation with projected/actual contact-force yaw-moment utilization.

Core form:

```text
M_contact = sum_i(f_i * F_r,i - r_i * F_f,i)
M_drive = smooth_positive(sign(yawRate) * M_contact)
M_yield = mu_ref * sum_i(|r_i| * N_i)
u = M_drive / M_yield
A_u = 1 - exp(-(u / u_activation)^2)
v_transition = sqrt((rel_weight * vbar_rel)^2 + |Vf|^2)
S(v) = exp(-(v_transition / stribeck_speed)^2)
R(v) = 1 / (1 + (v_transition / speed_fade)^2)
M_extra = A_u * R(v) * (K_slide + K_static * S(v))
```

Selected parameters: projected-force utilization activation, longitudinal moment support, nominal longitudinal yield `0.1109691045 Nm`, equivalent activation `0.035 Nm`, `u_activation=0.315403104`, `stribeck_speed=0.025 m/s`, `speed_fade=0.64 m/s`, `rel_weight=0.75`, `static_extra=0`, `sliding=0.067416756`, weighted train opposes RMSE `0.02408835`.

Physical inputs: contact-relative speed, normal load, projected/actual contact force vector reduced to yaw-moment utilization, yaw rate sign, `Vf`. No command/request traction input and no UKF fields. Command affects physical contact force upstream, not traction selection.

In-place reference: extra `0.067114720`, total `0.081908970`, left/right `+0.655064465/-0.655064465`, L-R delta `1.310128931`, pass and slightly above B.

RMSE performance:

- Primary: `0.036866 -> 0.027908`, slightly better than B by `0.000620`, worse than C by `0.003788`.
- Open-floor downweighted: `0.043194 -> 0.041507`, slightly better than B, worse than C.
- Open-floor validation: `0.016931 -> 0.011394`, essentially B-scale and better than C on that split.
- Diag: `0.084621 -> 0.084410`, essentially B-scale and far worse than C.
- Aux: `0.051266 -> 0.048820`, essentially B-scale and worse than C.

Strengths: best direct production-shaped replacement for B's prohibited request gate; passes command scale; preserves no-command-conditioning; clear contact force/yield owner concept.

Sacrifices/failure modes: solve-order sensitive because it must use projected/actual contact force, not preprojection demand; one-sided positive contact-force branch needs explicit symmetric sign convention; fades like B and is not a broad combined-slip replacement; synthetic grid is not full force replay.

Production recommendation: physically acceptable B rewrite candidate and useful for yaw-launch scale. It should not be accepted as the whole yaw model because C-like moving-contact validation remains much better.

## 11. round2_b_correct_branch_reference

Source references:

- `codex_analysis/yaw_model_variant_fits/round2_b_correct_branch_reference/round2_b_correct_branch_reference_report.md`
- `codex_analysis/yaw_model_variant_fits/round2_b_correct_branch_reference/coefficients.csv`
- `codex_analysis/yaw_model_variant_fits/round2_b_correct_branch_reference/in_place_1radps_command.csv`

Name and purpose: exact saved Variant B branch reference packet. It resolves ambiguity between the original grid's mixed `request_or_yaw` activation family and the persisted selected `request_only` branch.

Exact branch:

```text
M_extra = A_req(M_base + M_extra) * R(v_transition) *
          (K_slide + K_static * S(v_transition))
A_req(x) = 1 - exp(-(smooth_positive(x) / req_activation_nm)^2)
v_transition = sqrt((rel_weight * drive_wheel_longitudinal_offset
                     * abs(yaw_rate))^2 + abs(Vf)^2)
S(v) = exp(-(v / stribeck_speed_mps)^2)
R(v) = 1 / (1 + (v / speed_fade_mps)^2)
```

Physical inputs: `Vf`, yaw rate/geometry-derived relative speed, and fixed-point requested yaw moment `M_base + M_extra`. No UKF fields. Command/request is the direct activation input, so the same physical contact state can produce different resistance if request differs.

In-place reference: extra `0.065013359207`, total `0.079807609207`, left/right `+0.642749605677/-0.642749605677`, L-R delta `1.285499211353`, pass.

RMSE performance: identical to the correct B selected branch:

- Primary: `0.036866 -> 0.028528`.
- Downweighted: `0.043194 -> 0.041880`.
- Open-floor validation: `0.016931 -> 0.011372`.
- Diag: `0.084621 -> 0.084412`.
- Aux: `0.051266 -> 0.048945`.

Strengths: authoritative reference for the command-scale target; prevents accidentally comparing against the wrong mixed branch.

Sacrifices/failure modes: request-conditioned diagnostic only; not production-eligible.

Production recommendation: keep as reference/diagnostic baseline, not as a PlantModel traction law.

## 12. round2_static_yield_contact

Source references:

- `codex_analysis/yaw_model_variant_fits/round2_static_yield_contact/static_yield_contact_report.md`
- `codex_analysis/yaw_model_variant_fits/round2_static_yield_contact/static_yield_parameters.csv`
- `codex_analysis/yaw_model_variant_fits/round2_static_yield_contact/one_rad_in_place_command.csv`

Name and purpose: explicit memoryless static-yield envelope plus a bounded slide-gated residual surface. It tests whether a contact-state yield envelope can enforce B-scale launch torque while leaving moving-contact residual fitting separate.

Core form:

```text
M_opp = M_yield + M_slide_surface

M_yield =
  A_state(v_yaw_contact, u_force) * R(v_t) * U(u_force) * L(N) *
  [M_slide + (M_static - M_slide) * exp(-(v_t / v_static)^2)]

v_t = sqrt((rel_weight * v_rel)^2 + Vf^2)
R(v_t) = 1/(1+(v_t/v_fade)^2)
U(u) = 0.5 + 0.5*u/(u+u_k)
L(N) = (N/N_nom)^load_exp

M_slide_surface = M_cap * tanh(M_raw/M_cap)
M_add = -sign(yaw) * M_opp
```

Selected parameters: `yaw_activation=0.050 m/s`, `force_activation_util=0.300`, `static_speed=0.035 m/s`, `speed_fade=0.640`, `rel_weight=0.750`, `util_k=0.200`, `slide_ratio=0.200`, `load_exp=1.0`, `static_peak=0.099836 Nm`, `sliding_yield=0.019967 Nm`, surface cap `0.040 Nm`.

Physical inputs: contact-yaw-relative velocity, actual projected-force utilization, transition speed, load, per-contact velocity and actual-force bases, limiter schedules. Command/request is not an input to prediction; command is used only in motor inverse after predicted torque. No UKF fields.

In-place reference: extra `0.065007`, total `0.079802`, left/right `+0.642715/-0.642715`, L-R delta `1.285430`, pass.

RMSE performance:

- Primary: `0.036866 -> 0.028110`.
- Open-floor downweighted: `0.043194 -> 0.036855`.
- Open-floor validation: `0.016931 -> 0.025701` (worse).
- Diag: `0.084621 -> 0.088532` (worse).
- Aux: `0.051266 -> 0.061266` (worse).
- Combined non-authoritative: `0.050234 -> 0.052072` (slightly worse than baseline and much worse than C).

Strengths: explicit static breakaway/yield envelope is architecturally clear and passes yaw-launch gate; separates launch authority from slide-gated residual correction.

Sacrifices/failure modes: memoryless; over-adds resistance in rows where the current plant already over-resists; worsens broad validation; coefficients in rare limiter-scheduled features are weak evidence; load extrapolation is weak.

Production recommendation: not production-ready as fitted. The useful conclusion is architectural: static breakaway should be an explicit contact-state yield envelope, but moving-contact branch fitting remains unresolved.

## 13. UKF Dependency Audit Conclusion

Source reference:

- `codex_analysis/yaw_model_variant_fits/round2_ukf_dependency_audit/ukf_dependency_audit_report.md`

Conclusion: no requested fit is invalidated by direct or indirect dependence on old suspect UKF state-vector data.

Evidence summarized by the audit:

- Greps over `codex_analysis/yaw_model_variant_fits` and `codex_analysis/contact_continuum_yaw_identification` found no fitting-use of `ukf_state`, `ukf`, `state_vector`, `estimator`, `xhat`, or `kalman`.
- Shared feature generation derives `forward_velocity_mps`, `yaw_rate_radps`, contact features, and residual targets from encoder velocities, raw gyro, drive commands, timestamps, and PlantModel mirror constants.
- `yaw_rate_radps` is raw gyro minus stationary sensor-derived bias, not UKF yaw rate.
- Residual target is gyro-differentiated observed yaw moment minus PlantModel mirror moment.
- Lateral/right body velocity is not measured and is set/reconstructed as `Vr=0`.

Important caveat: lack of UKF contamination does not make the fits production-ready. The residuals remain gyro-differentiated single-sample targets; many features are reconstructed; some baselines use command/request diagnostics; and production candidates still need force/slip/load-driven replay and validation.

## Parent-Answer Takeaways

1. The current baseline predicts only `+0.262/-0.262` command at +1 rad/s in-place, far below the approximate `+0.646/-0.646` target.
2. Variant B and B-derived hard-gate models hit the in-place command scale:
   - B reference: `+0.642750/-0.642750`, extra `0.065013 Nm`.
   - force-domain Stribeck: `+0.655064/-0.655064`, extra `0.067115 Nm`.
   - static-yield: `+0.642715/-0.642715`, extra `0.065007 Nm`.
   - request/contact surface: `+0.625079/-0.625079`, extra `0.061998 Nm`.
   - dynamic LuGre/Bristle: `+0.728361/-0.728361`, extra `0.079622 Nm`.
3. Variant C is the broad RMSE winner among first-round production-shaped fits (`validation_non_authoritative 0.050234 -> 0.030342`) but fails the launch command gate (`+0.369/-0.369`).
4. Request-conditioned B is not production-eligible even though it matches launch torque, because command/request cannot independently select traction for identical contact state.
5. The best production direction is not a residual table or aggregate yaw loss. It is a PlantModel-owned contact law combining:
   - explicit static/pre-sliding yield or bristle state driven by contact slip/load/actual force history, and
   - a moving-contact combined-slip surface driven by physical contact primitives.
6. UKF contamination was audited and not found; the remaining data-quality caveats are gyro differentiation, reconstructed lateral/contact velocity, and incomplete force replay.
