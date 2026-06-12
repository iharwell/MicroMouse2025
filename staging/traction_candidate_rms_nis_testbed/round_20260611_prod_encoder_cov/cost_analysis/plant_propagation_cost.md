# Plant Propagation Cost

Direct wall-clock timing of `CandidatePlant.propagate()` only. CSV IO, full replay, EKF update, covariance propagation, and NIS scoring are excluded.

| Model | Plant model | us/prop median | Relative vs cheapest | Parameter source |
|---|---:|---:|---:|---|
| baseline/current_holdover | current_holdover_approximation | 72.620 | 1.00x | C:\Users\thene\source\repos\MicroMouse2025\staging\traction_candidate_rms_nis_testbed\candidates.json |
| slip_envelope | algebraic_envelope | 74.873 | 1.03x | C:\Users\thene\source\repos\MicroMouse2025\staging\traction_candidate_rms_nis_testbed\round_20260611_prod_encoder_cov\slip_envelope\tune_expanded\tuned_parameters.json |
| stribeck_fade | stribeck_algebraic | 80.034 | 1.10x | C:\Users\thene\source\repos\MicroMouse2025\staging\traction_candidate_rms_nis_testbed\round_20260611_prod_encoder_cov\stribeck_fade\tune\tuned_parameters.json |
| skew_shear | skew_shear | 84.794 | 1.17x | C:\Users\thene\source\repos\MicroMouse2025\staging\traction_candidate_rms_nis_testbed\round_20260611_prod_encoder_cov\skew_shear\tune_refined\tuned_parameters.json |
| shear_rate | shear_rate | 95.023 | 1.31x | C:\Users\thene\source\repos\MicroMouse2025\staging\traction_candidate_rms_nis_testbed\round_20260611_prod_encoder_cov\shear_rate\refined_tuning\tuned_parameters.json |
| in_shear | in_shear | 82.788 | 1.14x | C:\Users\thene\source\repos\MicroMouse2025\staging\traction_candidate_rms_nis_testbed\round_20260611_prod_encoder_cov\in_shear\tune\tuned_parameters.json |

Caveats: Python interpreter overhead and list/dataclass object traffic dominate the absolute timings. Treat these results as a narrow standalone-testbed relative cost check, not as embedded C++ operation cost.

Iterations: 6000 propagations/repeat x 9 repeats after warmup.
