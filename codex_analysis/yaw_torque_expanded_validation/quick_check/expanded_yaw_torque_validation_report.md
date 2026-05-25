# Expanded Yaw Torque Validation

Scratch-only independent validation. No production code was modified.

## Method

Inputs are actual sensors and commands only: raw gyro yaw rate minus an independently estimated stationary bias, encoder-derived forward velocity and wheel speeds, logged drive commands, and timestamps. The extraction does not use `ukf_state_*`, pose, estimator yaw-rate targets, or logged gyro bias as targets.

Torque basis follows the prior reconciled Worker E approach: `residual_additive_yaw_torque_nm = observed_yaw_moment_nm - current_model_yaw_moment_nm`, where the current model mirror includes motor torque, static/rolling loss, longitudinal tire stiffness, contact force gains, contact projection, and yaw damping. `opposing_yaw_resistance_nm = -sign(sensor_yaw_rate) * residual_additive_yaw_torque_nm`.

Only nonzero-forward/nonzero-yaw active adjacent samples are binned: `|Vf| >= 0.05 m/s`, `|yaw| >= 0.25 rad/s`, forward bins 0.10 m/s, yaw bins 0.50 rad/s.

## Constants

| Constant | Value |
| --- | ---: |
| mass kg | 0.140000000 |
| track width m | 0.084635000 |
| yaw inertia kg m^2 | 0.000220000 |
| yaw denominator incl wheel spin-up kg m^2 | 0.000246510 |
| wheel radius m | 0.012610000 |

## Log Set

Discovered 71 candidate CSV logs: 50 decoded open-floor logs, 17 competition diagnostic/audit logs, and 4 competition front-wall characterization CSVs.
Used 3 logs with nonzero-Vf yaw samples: 3 decoded open-floor and 0 competition logs.

Full discovered/used/excluded details are in `discovered_logs.csv` and `run_summary.csv`.

## Highest-Coverage Nonzero-Vf Bins

| Vf bin m/s | Yaw bin rad/s | Count | Runs | Median residual Nm | Median opposing Nm | IQR Nm | Run spread Nm | Consistency |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 0.10 | 0.50 | 581 | 3 | -0.107598329 | 0.107598329 | 0.111326056 | 0.113898233 | cross-run |
| -0.20 | -0.50 | 295 | 2 | 0.001466450 | 0.001466450 | 0.012018138 | 0.005005164 | cross-run |
| -0.20 | 0.50 | 289 | 2 | -0.000760852 | 0.000760852 | 0.014543214 | 0.010329951 | cross-run |
| 0.10 | -0.50 | 226 | 1 | 0.000803470 | 0.000803470 | 0.010638930 | 0.000000000 | single-run/weak |
| -0.10 | 0.50 | 219 | 2 | -0.001965129 | 0.001965129 | 0.134370020 | 0.121094983 | cross-run |
| 0.10 | 4.50 | 114 | 1 | 0.001010368 | -0.001010368 | 0.068909001 | 0.000000000 | single-run/weak |
| -0.10 | -0.50 | 105 | 2 | -0.000555050 | -0.000555050 | 0.100327382 | 0.000601650 | cross-run |
| 0.10 | 3.50 | 98 | 1 | -0.034167383 | 0.034167383 | 0.091667456 | 0.000000000 | single-run/weak |
| 0.10 | 4.00 | 97 | 1 | -0.011852283 | 0.011852283 | 0.094995007 | 0.000000000 | single-run/weak |
| 0.20 | 1.00 | 79 | 1 | -0.060787366 | 0.060787366 | 0.061885022 | 0.000000000 | single-run/weak |
| 0.10 | 5.50 | 74 | 1 | 0.028304542 | -0.028304542 | 0.083870126 | 0.000000000 | single-run/weak |
| 0.10 | 3.00 | 70 | 1 | -0.092234109 | 0.092234109 | 0.091905484 | 0.000000000 | single-run/weak |
| 0.20 | 0.50 | 69 | 1 | 0.003334143 | -0.003334143 | 0.030289946 | 0.000000000 | single-run/weak |
| 0.20 | 8.50 | 57 | 1 | 0.042845187 | -0.042845187 | 0.080004711 | 0.000000000 | single-run/weak |
| 0.20 | -0.50 | 55 | 1 | 0.001919819 | 0.001919819 | 0.012017531 | 0.000000000 | single-run/weak |
| 0.20 | 1.50 | 52 | 1 | -0.070936912 | 0.070936912 | 0.059554178 | 0.000000000 | single-run/weak |

## RMSE

Leave-one-run-out correction uses signed-bin median residual torque from all other runs. Units are one-step yaw-rate error in rad/s.

| Holdout | Kind | Samples | Corrected samples | Current RMSE | Corrected RMSE |
| --- | --- | ---: | ---: | ---: | ---: |
| `2026-04-10_18-08-20` | decoded_open_floor | 1407 | 154 | 0.313847862 | 0.278297653 |
| `2026-04-10_18-33-52` | decoded_open_floor | 886 | 516 | 0.305921833 | 0.307530818 |
| `2026-04-11_06-58-25` | decoded_open_floor | 862 | 461 | 0.144449220 | 0.343599765 |

Aggregate: samples=3155, corrected=1131, current RMSE=0.275517332 rad/s, corrected RMSE=0.305539767 rad/s.

## Data Sufficiency Judgment

Supported enough for calibration exploration, but still not production-final:
- Vf=-0.20 m/s, yaw=-0.50 rad/s: n=295, runs=2, medianOpp=0.0015 Nm, runSpread=0.0050
- Vf=-0.20 m/s, yaw=0.50 rad/s: n=289, runs=2, medianOpp=0.0008 Nm, runSpread=0.0103
- Vf=0.10 m/s, yaw=0.50 rad/s: n=581, runs=3, medianOpp=0.1076 Nm, runSpread=0.1139

Weak or targeted-sweep-needed bins that are populated but inconsistent or single-run dominated:
- Vf=-0.10 m/s, yaw=-0.50 rad/s: n=105, runs=2, medianOpp=-0.0006 Nm, runSpread=0.0006
- Vf=-0.10 m/s, yaw=0.50 rad/s: n=219, runs=2, medianOpp=0.0020 Nm, runSpread=0.1211
- Vf=0.10 m/s, yaw=-0.50 rad/s: n=226, runs=1, medianOpp=0.0008 Nm, runSpread=0.0000
- Vf=0.10 m/s, yaw=3.00 rad/s: n=70, runs=1, medianOpp=0.0922 Nm, runSpread=0.0000
- Vf=0.10 m/s, yaw=3.50 rad/s: n=98, runs=1, medianOpp=0.0342 Nm, runSpread=0.0000
- Vf=0.10 m/s, yaw=4.00 rad/s: n=97, runs=1, medianOpp=0.0119 Nm, runSpread=0.0000
- Vf=0.10 m/s, yaw=4.50 rad/s: n=114, runs=1, medianOpp=-0.0010 Nm, runSpread=0.0000
- Vf=0.10 m/s, yaw=5.50 rad/s: n=74, runs=1, medianOpp=-0.0283 Nm, runSpread=0.0000
- Vf=0.20 m/s, yaw=-0.50 rad/s: n=55, runs=1, medianOpp=0.0019 Nm, runSpread=0.0000
- Vf=0.20 m/s, yaw=0.50 rad/s: n=69, runs=1, medianOpp=-0.0033 Nm, runSpread=0.0000
- Vf=0.20 m/s, yaw=1.00 rad/s: n=79, runs=1, medianOpp=0.0608 Nm, runSpread=0.0000
- Vf=0.20 m/s, yaw=1.50 rad/s: n=52, runs=1, medianOpp=0.0709 Nm, runSpread=0.0000
- Vf=0.20 m/s, yaw=8.50 rad/s: n=57, runs=1, medianOpp=-0.0428 Nm, runSpread=0.0000

Regions absent or not reliable require targeted sweeps: high forward speed with high yaw rate, negative forward velocity, and any bins represented by one run only. Competition logs add useful nonzero-forward coverage, but many bins are maneuver/path dependent and not repeated cleanly across runs.

## Discrepancies With Previous Scratch Work

- This validation confirms the prior rejection of the approximately 0.9 Nm motor-only bins; using the fuller current PlantModel mirror keeps typical residuals in the hundredths to low-tenths of Nm range.
- Compared with the previous Worker E run, this extraction is deliberately coverage-first and includes competition diagnostic/audit CSVs plus all discoverable decoded open-floor candidates. It therefore exposes more nonzero-forward bins, but also more single-run and path-dependent bins.
- The sign pattern is not a monotonic counter-yaw table. Several populated bins still have negative opposing resistance, meaning the model sometimes needs yaw-aiding residual torque.

## Reproduce

```powershell
python codex_analysis\yaw_torque_expanded_validation\expanded_yaw_torque_validation.py --out-dir codex_analysis\yaw_torque_expanded_validation
```
