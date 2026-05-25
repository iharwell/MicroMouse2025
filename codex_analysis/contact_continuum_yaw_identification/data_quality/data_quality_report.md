# Contact-Continuum Yaw Data Quality

Analysis-only output. Production code was not modified. Trust predicates use sensor, command, schema, and metadata fields only; `ukf_state_*` fields are not used.

## Files
- `data_quality_run_inventory.csv`
- `data_quality_family_summary.csv`
- `data_quality_run_dominance.csv`
- `data_quality_feature_coverage_summary.csv`
- `data_quality_hazards.csv`
- `data_quality_recommendations_by_run.csv`

## Family Summary
| Family | Runs | Extracted | Moving-yaw sensor rows | Saturation runs | Watchdog runs | Gyro d/dt spikes | Fan source | Recommendation shape |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- |
| competition_aux | 13 | 45435 | 71179 | 0 | 0 | 0.002017 | per-row 0, metadata 13 | fit 0, down 13, val 13, excl 0 |
| competition_diag | 4 | 46099 | 75617 | 0 | 0 | 0.000408 | per-row 0, metadata 4 | fit 0, down 0, val 3, excl 1 |
| competition_fwc | 4 | 0 | 0 | 0 | 0 | 0.000000 | per-row 0, metadata 0 | fit 0, down 0, val 0, excl 4 |
| decoded_open_floor | 49 | 337792 | 658278 | 49 | 46 | 0.000834 | per-row 47, metadata 0 | fit 15, down 9, val 9, excl 16 |

## Recommendations
- Fit-authoritative runs: 15 decoded open-floor runs with adequate sensor coverage and no major schema/trust penalty.
- Downweighted runs: 22 runs, mostly high saturation, missing watchdog/per-row fan, zero angular command evidence, or aux path dependence.
- Excluded runs: 21 runs, mostly front-wall characterization, no feature samples, or weak bias/feature support.
- Competition diag: validation-only; useful maze-turn coverage but legacy schema lacks saturation/watchdog/per-row fan duty.
- Competition aux: validation-only and downweighted; useful stress coverage but too path/procedure dependent for fit authority.

## Run Dominance
| Run | Family | Samples | Overall share | Family share | Recommendation |
| --- | --- | ---: | ---: | ---: | --- |
| 2026-04-21_00-16-10 | decoded_open_floor | 44880 | 0.104536 | 0.132863 | fit_authoritative |
| 2026-04-20_08-38-39 | decoded_open_floor | 41332 | 0.096272 | 0.122359 | fit_downweighted |
| 2026-04-21_01-09-34 | decoded_open_floor | 25854 | 0.060220 | 0.076538 | fit_authoritative |
| 2026-04-20_12-10-58 | decoded_open_floor | 24297 | 0.056593 | 0.071929 | fit_authoritative |
| diag003 | competition_diag | 23242 | 0.054136 | 0.504176 | validation_only |
| 2026-04-21_05-32-06 | decoded_open_floor | 20317 | 0.047323 | 0.060146 | fit_authoritative |
| 2026-04-20_02-33-07 | decoded_open_floor | 20244 | 0.047153 | 0.059930 | fit_downweighted |
| 2026-04-20_04-54-09 | decoded_open_floor | 19376 | 0.045131 | 0.057361 | fit_downweighted |

## Coverage
- total_nonzero_vf_yaw_bins: 407 (from yaw_torque_expanded_validation/nonzero_vf_torque_bins.csv)
- strong_cross_run_bins_count_ge250_runs_ge5: 199 (candidate fit-support bins before family trust weighting)
- weak_or_single_run_bins: 74 (must be downweighted or targeted for sweeps)
- high_abs_forward_ge_0p6_bins: 28 (coverage exists but is sparse and often run-dominated)
- high_abs_yaw_ge_8_bins: 166 (mostly low-forward scrub/turn coverage, not broad high-speed arcs)
- competition_enabled_bins: 17 (bins crossing support because competition data was present)
- dataset_all_included_samples: 2382049 (moving_yaw=677445, bins=393)
- dataset_open_floor_only_samples: 1870997 (moving_yaw=531861, bins=374)
- dataset_competition_only_samples: 511052 (moving_yaw=145584, bins=127)
- top_bin_contributor_2026-04-21_00-16-10: 43953 (sum of bin_run_consistency counts; highlights run dominance risk)
- top_bin_contributor_2026-04-20_08-38-39: 40844 (sum of bin_run_consistency counts; highlights run dominance risk)
- top_bin_contributor_2026-04-21_01-09-34: 25229 (sum of bin_run_consistency counts; highlights run dominance risk)

## Hazards
- competition_aux: saturation unavailable - 0/13 runs expose saturation flags; do not use unavailable-saturation runs as fit authority.
- competition_aux: watchdog unavailable - 0/13 runs expose watchdog flags; treat watchdog absence as schema trust loss, not proof of no watchdog activity.
- competition_aux: fan/load assumed - per-row fan in 0 runs; metadata fan in 13 runs; separate load/fan inference from contact-continuum yaw fit.
- competition_diag: saturation unavailable - 0/4 runs expose saturation flags; do not use unavailable-saturation runs as fit authority.
- competition_diag: watchdog unavailable - 0/4 runs expose watchdog flags; treat watchdog absence as schema trust loss, not proof of no watchdog activity.
- competition_diag: fan/load assumed - per-row fan in 0 runs; metadata fan in 4 runs; separate load/fan inference from contact-continuum yaw fit.
- competition_fwc: saturation unavailable - 0/4 runs expose saturation flags; do not use unavailable-saturation runs as fit authority.
- competition_fwc: watchdog unavailable - 0/4 runs expose watchdog flags; treat watchdog absence as schema trust loss, not proof of no watchdog activity.
- competition_fwc: fan/load assumed - per-row fan in 0 runs; metadata fan in 0 runs; separate load/fan inference from contact-continuum yaw fit.
- decoded_open_floor: watchdog unavailable - 46/49 runs expose watchdog flags; treat watchdog absence as schema trust loss, not proof of no watchdog activity.
- decoded_open_floor:2026-04-11_06-58-25: missing/stale angular command evidence - cmd_angular zero fraction during moving-yaw rows=0.800239; fit only from wheel commands/sensor timing or downweight.
- decoded_open_floor:2026-04-11_21-03-20: missing/stale angular command evidence - cmd_angular zero fraction during moving-yaw rows=0.648461; fit only from wheel commands/sensor timing or downweight.
- decoded_open_floor:2026-04-12_05-13-55: saturation can mimic yaw physics - moving-yaw saturation fraction=0.530488; exclude saturated rows and downweight the run.
- decoded_open_floor:2026-04-14_02-00-02: saturation can mimic yaw physics - moving-yaw saturation fraction=0.258663; exclude saturated rows and downweight the run.
- decoded_open_floor:2026-04-14_05-40-35: missing/stale angular command evidence - cmd_angular zero fraction during moving-yaw rows=1.000000; fit only from wheel commands/sensor timing or downweight.
- decoded_open_floor:2026-04-14_06-21-48: final bad tail - tail_seconds=0.467997; existing_cutoff=dropped final sensor-quiescent/invalid tail >= 0.25 s; trim tails before derivative/ablation work.
- decoded_open_floor:2026-04-14_06-37-59: final bad tail - tail_seconds=0.451706; existing_cutoff=kept through final sensor-active row; trim tails before derivative/ablation work.
- decoded_open_floor:2026-04-14_07-02-50: final bad tail - tail_seconds=0.432597; existing_cutoff=kept through final sensor-active row; trim tails before derivative/ablation work.
- decoded_open_floor:2026-04-14_07-17-42: final bad tail - tail_seconds=0.451199; existing_cutoff=kept through final sensor-active row; trim tails before derivative/ablation work.
- decoded_open_floor:2026-04-14_07-25-50: missing/stale angular command evidence - cmd_angular zero fraction during moving-yaw rows=1.000000; fit only from wheel commands/sensor timing or downweight.
- decoded_open_floor:2026-04-14_07-25-50: final bad tail - tail_seconds=0.460200; existing_cutoff=dropped final sensor-quiescent/invalid tail >= 0.25 s; trim tails before derivative/ablation work.
- decoded_open_floor:2026-04-14_07-40-05: final bad tail - tail_seconds=0.346514; existing_cutoff=dropped final sensor-quiescent/invalid tail >= 0.25 s; trim tails before derivative/ablation work.
- decoded_open_floor:2026-04-14_07-51-39: missing/stale angular command evidence - cmd_angular zero fraction during moving-yaw rows=1.000000; fit only from wheel commands/sensor timing or downweight.
- decoded_open_floor:2026-04-14_07-51-39: final bad tail - tail_seconds=0.460200; existing_cutoff=dropped final sensor-quiescent/invalid tail >= 0.25 s; trim tails before derivative/ablation work.
- decoded_open_floor:2026-04-14_07-53-57: missing/stale angular command evidence - cmd_angular zero fraction during moving-yaw rows=1.000000; fit only from wheel commands/sensor timing or downweight.
- decoded_open_floor:2026-04-14_07-53-57: final bad tail - tail_seconds=0.357940; existing_cutoff=kept through final sensor-active row; trim tails before derivative/ablation work.
- decoded_open_floor:2026-04-14_08-14-01: missing/stale angular command evidence - cmd_angular zero fraction during moving-yaw rows=1.000000; fit only from wheel commands/sensor timing or downweight.
- decoded_open_floor:2026-04-14_08-14-01: final bad tail - tail_seconds=0.460000; existing_cutoff=dropped final sensor-quiescent/invalid tail >= 0.25 s; trim tails before derivative/ablation work.
- decoded_open_floor:2026-04-14_08-34-57: missing/stale angular command evidence - cmd_angular zero fraction during moving-yaw rows=1.000000; fit only from wheel commands/sensor timing or downweight.
- decoded_open_floor:2026-04-14_08-34-57: final bad tail - tail_seconds=0.468000; existing_cutoff=dropped final sensor-quiescent/invalid tail >= 0.25 s; trim tails before derivative/ablation work.
- decoded_open_floor:2026-04-14_08-44-07: missing/stale angular command evidence - cmd_angular zero fraction during moving-yaw rows=1.000000; fit only from wheel commands/sensor timing or downweight.
- decoded_open_floor:2026-04-14_08-44-07: final bad tail - tail_seconds=0.286000; existing_cutoff=kept through final sensor-active row; trim tails before derivative/ablation work.
- decoded_open_floor:2026-04-14_08-58-04: missing/stale angular command evidence - cmd_angular zero fraction during moving-yaw rows=1.000000; fit only from wheel commands/sensor timing or downweight.
- decoded_open_floor:2026-04-14_08-58-04: final bad tail - tail_seconds=0.328000; existing_cutoff=kept through final sensor-active row; trim tails before derivative/ablation work.
- decoded_open_floor:2026-04-14_09-15-42: missing/stale angular command evidence - cmd_angular zero fraction during moving-yaw rows=1.000000; fit only from wheel commands/sensor timing or downweight.
- decoded_open_floor:2026-04-14_09-15-42: final bad tail - tail_seconds=0.455000; existing_cutoff=dropped final sensor-quiescent/invalid tail >= 0.25 s; trim tails before derivative/ablation work.
- decoded_open_floor:2026-04-14_16-34-17: saturation can mimic yaw physics - moving-yaw saturation fraction=0.306872; exclude saturated rows and downweight the run.
- decoded_open_floor:2026-04-14_16-34-17: gyro derivative noise/spikes - abs(dgyro/dt)>=500 rad/s^2 fraction=0.004431; rms=74.871860 rad/s^2; smooth, bin, or use robust losses before fitting acceleration-like yaw residuals.
- decoded_open_floor:2026-04-15_01-22-20: final bad tail - tail_seconds=0.266000; existing_cutoff=kept through final sensor-active row; trim tails before derivative/ablation work.
- decoded_open_floor:2026-04-15_02-09-58: saturation can mimic yaw physics - moving-yaw saturation fraction=0.269673; exclude saturated rows and downweight the run.

## Reproduce

```powershell
python codex_analysis\contact_continuum_yaw_identification\data_quality\analyze_data_quality.py
```
