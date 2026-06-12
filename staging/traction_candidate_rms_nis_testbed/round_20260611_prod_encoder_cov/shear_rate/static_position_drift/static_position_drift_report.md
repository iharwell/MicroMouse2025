# Static Position Drift

- Candidate config: `C:\Users\thene\source\repos\MicroMouse2025\staging\traction_candidate_rms_nis_testbed\round_20260611_prod_encoder_cov\shear_rate\shear_rate_refined_tuned_only.json`
- Covariance config: `C:\Users\thene\source\repos\MicroMouse2025\staging\traction_candidate_rms_nis_testbed\covariance_conservative.json`
- Static manifest: `C:\Users\thene\source\repos\MicroMouse2025\staging\traction_candidate_rms_nis_testbed\round_20260611\static_stability_analysis\selected_static_segment_manifest.json`
- Segment: `ofnis_001617`, longest prior static segment; zero commands and zero wheel rates.
- No logged UKF state and no encoder NIS are consumed.
- Pass threshold: `0.005` m on both final and max radial drift.

| Estimator | Case | Final x/y (m) | Final radial (m) | Max radial (m) | Yaw drift (rad/deg) | Pass 5 mm |
| --- | --- | ---: | ---: | ---: | ---: | --- |
| `simplex_ukf` | `full_static_replay` | -0.0504707881876/-0.0694956892507 | 0.085889180254 | 0.085889180254 | -0.0179771345043/-1.03001393483 | `false` |
| `simplex_ukf` | `prediction_only_encoder_only` | -0.0216530871408/-0.0138636595718 | 0.0257110334186 | 0.0257110334186 | 0.0198231039959/1.13578019581 | `false` |
| `ekf` | `full_static_replay` | 0.00135578372001/-0.209149086451 | 0.209153480757 | 0.209153480757 | -0.0185446256067/-1.06252877991 | `false` |
| `ekf` | `prediction_only_encoder_only` | 0/0 | 0 | 0 | 0/0 | `true` |

## Artifacts

- Rows: `C:\Users\thene\source\repos\MicroMouse2025\staging\traction_candidate_rms_nis_testbed\round_20260611_prod_encoder_cov\shear_rate\static_position_drift\static_position_drift_rows.csv`
- Metrics: `C:\Users\thene\source\repos\MicroMouse2025\staging\traction_candidate_rms_nis_testbed\round_20260611_prod_encoder_cov\shear_rate\static_position_drift\static_position_drift_metrics.csv`
- Summary: `C:\Users\thene\source\repos\MicroMouse2025\staging\traction_candidate_rms_nis_testbed\round_20260611_prod_encoder_cov\shear_rate\static_position_drift\static_position_drift_summary.json`
- Inputs: `C:\Users\thene\source\repos\MicroMouse2025\staging\traction_candidate_rms_nis_testbed\round_20260611_prod_encoder_cov\shear_rate\static_position_drift\run_inputs.json`
