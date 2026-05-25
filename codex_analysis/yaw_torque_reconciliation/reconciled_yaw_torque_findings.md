# Reconciled Yaw Torque Findings

## Scope

This is a scratch reconciliation only. No production code was modified.

The stated goal is extracting counter-yaw torque at discrete forward-velocity and yaw-rate values from actual sensor data. UKF estimates were not used as targets in the accepted analysis path.

## Commands Run

```powershell
Get-Content -LiteralPath AGENTS.md
Get-ChildItem -Force -LiteralPath codex_analysis\yaw_fit, codex_analysis\yaw_fit_validation, codex_analysis\yaw_torque_surface
git status --short
rg -n "inertia|yaw|torque|alpha|accel|RMSE|rmse|dt|gradient|diff|residual|bin|command|counter|abs|signed|train|test|leave|window|trim|tail" codex_analysis\yaw_fit\fit_yaw_plant_params.py
rg -n "inertia|yaw|torque|alpha|accel|RMSE|rmse|dt|gradient|diff|residual|bin|command|counter|abs|signed|train|test|leave|window|trim|tail" codex_analysis\yaw_fit_validation\counter_yaw_torque_surface.py
rg -n "inertia|yaw|torque|alpha|accel|RMSE|rmse|dt|gradient|diff|residual|bin|command|counter|abs|signed|train|test|leave|window|trim|tail" codex_analysis\yaw_torque_surface\extract_yaw_torque_surface.py
python codex_analysis\yaw_fit\fit_yaw_plant_params.py --report codex_analysis\yaw_torque_reconciliation\worker_c_rerun\yaw_fit_report.md
python -c "import pathlib; import codex_analysis.yaw_fit_validation.counter_yaw_torque_surface as m; m.OUT_DIR = pathlib.Path(r'C:\Users\thene\source\repos\MicroMouse2025\codex_analysis\yaw_torque_reconciliation\worker_d_rerun'); raise SystemExit(m.main())"
python codex_analysis\yaw_torque_surface\extract_yaw_torque_surface.py --out-dir codex_analysis\yaw_torque_reconciliation\worker_e_rerun
```

The first Worker D rerun used a relative monkeypatched output path and failed only while formatting `relative_to(REPO_ROOT)` in the report. It had already demonstrated the same sample counts. The absolute-path rerun completed.

## Rerun Results

Worker C reproduced its scalar diagnostic baseline:

| Metric | Current | Fitted |
| --- | ---: | ---: |
| one-step yaw-rate RMSE, rad/s | 0.226296125 | 0.161693351 |
| fitted track width, m | 0.084635000 | 0.104595474 |
| fitted yaw inertia, kg m^2 | 0.000220000 | 0.000603133 |

Worker D reproduced its large RMSE values:

| Subset | Samples | Current RMSE rad/s | Corrected RMSE rad/s |
| --- | ---: | ---: | ---: |
| all samples, global surface | 224933 | 37.869354668 | 28.768104245 |
| aligned samples, aligned surface | 75508 | 34.246127067 | 26.554454978 |

Worker E reproduced its leave-one-run-out surface evaluation:

| Samples | Samples corrected by non-empty bins | Current RMSE rad/s | Corrected RMSE rad/s |
| ---: | ---: | ---: | ---: |
| 429963 | 417993 | 0.369009749 | 0.401910472 |

All RMSE values above are one-step yaw-rate error in rad/s. Worker D's units were not rad/s^2 or Nm, but the model being evaluated was not the same as Worker E's model.

## Formula Reconciliation

The D/E sign conventions are equivalent when the same model moment is used:

| Worker | Stored residual | Opposing torque |
| --- | --- | --- |
| D | `commanded_yaw_moment - yaw_denominator * observed_yaw_accel - damping * yaw_rate` | `residual_counter * sign(yaw_rate)` |
| E | `observed_yaw_moment - current_model_yaw_moment` | `-sign(yaw_rate) * residual_additive` |

Both multiply measured yaw acceleration by the effective yaw denominator that includes wheel spin-up. The conflict is not a Nm/rad/s unit conversion error, not a missing yaw-inertia multiplication, and not a sign inversion.

The root cause is that Worker D's "current model" is motor-only differential yaw moment:

`0.5 * track_width * (left_motor_force - right_motor_force)`

Worker E mirrors the yaw-relevant production `PlantModel` terms: motor torque, static/rolling loss, longitudinal tire stiffness, front/rear right-contact force gains, normal-load and fan-load projection through the sustained-lateral-acceleration contact `mu`, and the front/rear right-force yaw moment. That model is the defensible baseline for extracting a residual torque surface against current production behavior.

Derivative and binning differences are real but secondary:

| Difference | Effect |
| --- | --- |
| D uses 5-control-sample slopes, E uses adjacent valid samples | Changes noise and sample coverage, but does not explain the 10x torque magnitude gap. |
| D reports coarse absolute yaw bins and forward ranges, E uses signed 0.10 m/s and 0.50 rad/s bins | Makes table rows non-identical. |
| D's highlighted 0.98 Nm row is command-aligned only | Selects accelerating rows where a motor-only residual is biased high. |
| E compact rows combine positive/negative signed yaw bins | Good summary view, but implementation should use the signed residual table, not only the compact table. |
| D trims terminal fault/lift-like tails more aggressively | Changes sample count and run coverage, but same-window comparison still collapses D's large bins when the full model is used. |

## Same-Window Check

To isolate formula from sample selection, I replayed Worker D's own near-zero-forward in-place rows and bins, then recomputed opposing torque using Worker E's fuller model formula. The resulting comparison is in `reconciliation_key_bins.csv`.

| Scope | Selection | Forward bin | Abs yaw bin | Count | Median opposing Nm |
| --- | --- | ---: | ---: | ---: | ---: |
| D original motor-only | aligned | -0.1..0.1 | 0.2..1 | 21724 | 0.977698939 |
| Same D windows, full model | aligned | -0.1..0.1 | 0.2..1 | 21724 | 0.110192241 |
| D original motor-only | aligned | -0.1..0.1 | 1..3 | 24656 | 0.894728227 |
| Same D windows, full model | aligned | -0.1..0.1 | 1..3 | 24656 | 0.100968445 |
| D original motor-only | all | -0.1..0.1 | 1..3 | 47698 | 0.047214284 |
| Same D windows, full model | all | -0.1..0.1 | 1..3 | 47698 | -0.009620083 |

This confirms that the large D aligned bins are an artifact of measuring residual torque against an incomplete motor-only plant mirror. They are not a defensible counter-yaw torque surface.

## Consolidated Near-Zero Forward Table

For the user purpose, use Worker E's sensor-only, signed-bin residual extraction as the defensible source. The compact near-zero-forward summary is:

| Forward bin m/s | Abs yaw-rate bin rad/s | Count | Median opposing Nm | Interpretation |
| ---: | ---: | ---: | ---: | --- |
| 0.00 | 0.50 | 28826 | 0.085758753 | small resisting residual |
| 0.00 | 1.00 | 13936 | 0.080160531 | small resisting residual |
| 0.00 | 1.50 | 9913 | 0.078300269 | small resisting residual |
| 0.00 | 2.00 | 10203 | 0.019093792 | near neutral |
| 0.00 | 2.50 | 12089 | -0.039354409 | yaw-aiding residual in all-row summary |
| 0.00 | 3.00 | 10278 | -0.086537855 | yaw-aiding residual in all-row summary |

Do not install a monotonic "more yaw rate means more counter-yaw torque" table from these logs. Several high-count bins are negative in the all-row summary, meaning the current model needs yaw-aiding torque in those bins rather than extra resistance.

## Recommendation

Worker E is the defensible analysis for extracting a discrete yaw-torque residual surface from actual sensor data, with one refinement: use its signed residual-additive torque bins as the calibration artifact, and treat the compact opposing-resistance table only as a human summary.

Worker D should be rejected for magnitude recommendations. Its RMSE is in rad/s, but it evaluates a motor-only simplification rather than the current `PlantModel` yaw behavior, which explains both the implausible 37.87 -> 28.77 rad/s RMSE and the 0.9-1.0 Nm near-zero-forward bins.

Worker C remains useful only as a scalar diagnostic: it fits track/yaw inertia on a narrower in-place/stop dataset and improves one-step RMSE there, but it does not answer the requested velocity/yaw-rate torque-surface extraction.

The current-vs-corrected RMSE claims should be treated as provisional for production tuning. Worker E's same-window leave-one-run-out evaluation is the only consistent RMSE table among the surface analyses, and it worsens aggregate one-step RMSE from 0.369009749 to 0.401910472 rad/s. That does not invalidate the residual structure, but it means the present historical logs are not enough to justify a production correction table without tighter phase selection and validation.

## Created Files

- `codex_analysis\yaw_torque_reconciliation\worker_c_rerun\yaw_fit_report.md`
- `codex_analysis\yaw_torque_reconciliation\worker_d_rerun\counter_yaw_torque_surface_report.md`
- `codex_analysis\yaw_torque_reconciliation\worker_d_rerun\counter_yaw_torque_surface_bins.csv`
- `codex_analysis\yaw_torque_reconciliation\worker_d_rerun\counter_yaw_surface_rmse.csv`
- `codex_analysis\yaw_torque_reconciliation\worker_d_rerun\counter_yaw_run_summary.csv`
- `codex_analysis\yaw_torque_reconciliation\worker_e_rerun\yaw_torque_surface_report.md`
- `codex_analysis\yaw_torque_reconciliation\worker_e_rerun\yaw_torque_surface_bins.csv`
- `codex_analysis\yaw_torque_reconciliation\worker_e_rerun\yaw_torque_surface_rmse.csv`
- `codex_analysis\yaw_torque_reconciliation\worker_e_rerun\yaw_torque_surface_windows.csv`
- `codex_analysis\yaw_torque_reconciliation\reconciliation_key_bins.csv`
- `codex_analysis\yaw_torque_reconciliation\reconciled_yaw_torque_findings.md`
