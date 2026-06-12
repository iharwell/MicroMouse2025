# One-Log Simplex UKF Validation

- Candidate: `shear_rate`
- Status: `pass`
- Segment manifest: `C:\Users\thene\source\repos\MicroMouse2025\staging\traction_candidate_rms_nis_testbed\parallel_replace_20260420_102209_20260610\ukf_one_log_20260610_070622_active_manifest.json`
- Selected log: `mmlog_decode_2026-06-10_07-06-22`
- Segments: `100`
- Samples: `125088`
- Sigma policy: production simplex geometry, 11 active sigma points for 9 states.
- Measurement policy: production yaw-rate and planar-accel streams only; no encoder NIS.
- Uses logged UKF state: `false`
- Candidate config: `C:\Users\thene\source\repos\MicroMouse2025\staging\traction_candidate_rms_nis_testbed\round_20260611_prod_encoder_cov\shear_rate\shear_rate_refined_tuned_only.json`
- Covariance config: `C:\Users\thene\source\repos\MicroMouse2025\staging\traction_candidate_rms_nis_testbed\covariance_conservative.json`

| Metric | Value |
| --- | --- |
| `candidate_id` | `shear_rate` |
| `status` | `pass` |
| `segments` | `100` |
| `samples` | `125088` |
| `active_sigma_points` | `11` |
| `prediction_sigma_points` | `1375968` |
| `measurement_update_count` | `375264` |
| `finite` | `True` |
| `sigma_nonfinite_count` | `0` |
| `factorization_failure_count` | `0` |
| `max_abs_state` | `34.2435649551` |
| `max_abs_prediction` | `380.588451536` |
| `max_abs_sigma_state` | `176.34722298` |
| `max_abs_sigma_prediction` | `400.358396644` |
| `max_covariance_trace` | `141.962887382` |
| `max_yaw_rate_nis` | `4913.69059057` |
| `max_forward_accel_nis` | `658.327103729` |
| `max_right_accel_nis` | `563.529700179` |
| `yaw_rate_rms_nis` | `81.2786261206` |
| `forward_accel_rms_nis` | `21.2718653284` |
| `right_accel_rms_nis` | `19.507545359` |

## Artifacts

- Rows: `C:\Users\thene\source\repos\MicroMouse2025\staging\traction_candidate_rms_nis_testbed\round_20260611_prod_encoder_cov\shear_rate\ukf_one_log\simplex_ukf_rows.csv`
- Metrics CSV: `C:\Users\thene\source\repos\MicroMouse2025\staging\traction_candidate_rms_nis_testbed\round_20260611_prod_encoder_cov\shear_rate\ukf_one_log\simplex_ukf_metrics.csv`
- Summary JSON: `C:\Users\thene\source\repos\MicroMouse2025\staging\traction_candidate_rms_nis_testbed\round_20260611_prod_encoder_cov\shear_rate\ukf_one_log\ukf_one_log_simplex_summary.json`
- Inputs JSON: `C:\Users\thene\source\repos\MicroMouse2025\staging\traction_candidate_rms_nis_testbed\round_20260611_prod_encoder_cov\shear_rate\ukf_one_log\run_inputs.json`
