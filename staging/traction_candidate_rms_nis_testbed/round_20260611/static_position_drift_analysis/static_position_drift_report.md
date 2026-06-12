# Static Position Drift Re-Evaluation

- Segment: `ofnis_001617`
- Duration: `30.522` s
- Criterion: pass only if static position drift stays within `0.005 m`.
- EKF full uses existing `px_m`/`py_m` replay diagnostics. EKF prediction-only and simplex rows did not contain position, so this script integrates `vf/vr/heading/dt` from the row artifacts.
- No production code or model tuning is changed. No launch/open-floor assessment is run. No logged UKF states or encoder NIS are used.

| Model | Case | Final global x/y (mm) | Final body right/forward (mm) | Max global radial (mm) | Max body radial (mm) | Pass 5 mm |
| --- | --- | ---: | ---: | ---: | ---: | --- |
| `stribeck_fade` | `ekf_full` | 0.149/-145.131 | -1.586/-84.417 | 145.131 | 84.432 | `false` |
| `stribeck_fade` | `ekf_prediction_only_zero_encoder` | 0.000/0.000 | 0.000/0.000 | 0.000 | 0.000 | `true` |
| `stribeck_fade` | `simplex_ukf_full` | 16.013/50.081 | 16.850/49.783 | 52.579 | 52.557 | `false` |
| `stribeck_fade` | `simplex_ukf_prediction_only_zero_encoder` | -17.101/-10.040 | -17.021/-10.172 | 19.830 | 19.829 | `false` |
| `slip_envelope` | `ekf_full` | 2.173/-220.840 | -1.430/-137.691 | 220.851 | 137.699 | `false` |
| `slip_envelope` | `ekf_prediction_only_zero_encoder` | 0.000/0.000 | 0.000/0.000 | 0.000 | 0.000 | `true` |
| `slip_envelope` | `simplex_ukf_full` | 24.916/88.527 | 26.172/88.166 | 91.967 | 91.969 | `false` |
| `slip_envelope` | `simplex_ukf_prediction_only_zero_encoder` | -25.319/-15.724 | -25.305/-15.747 | 29.804 | 29.804 | `false` |
| `in_shear` | `ekf_full` | -0.460/-141.774 | -1.583/-82.428 | 141.775 | 82.444 | `false` |
| `in_shear` | `ekf_prediction_only_zero_encoder` | 0.000/0.000 | 0.000/0.000 | 0.000 | 0.000 | `true` |
| `in_shear` | `simplex_ukf_full` | -1.043/60.046 | -0.282/59.978 | 60.055 | 59.978 | `false` |
| `in_shear` | `simplex_ukf_prediction_only_zero_encoder` | -13.710/-11.160 | -13.656/-11.226 | 17.678 | 17.678 | `false` |
| `shear_rate` | `ekf_full` | -0.482/-137.351 | -1.612/-79.423 | 137.352 | 79.439 | `false` |
| `shear_rate` | `ekf_prediction_only_zero_encoder` | 0.000/0.000 | 0.000/0.000 | 0.000 | 0.000 | `true` |
| `shear_rate` | `simplex_ukf_full` | 8.289/47.808 | 8.695/47.731 | 48.521 | 48.517 | `false` |
| `shear_rate` | `simplex_ukf_prediction_only_zero_encoder` | -14.469/-10.452 | -14.398/-10.549 | 17.850 | 17.849 | `false` |
| `skew_shear` | `ekf_full` | -0.007/-144.192 | -1.585/-83.802 | 144.192 | 83.817 | `false` |
| `skew_shear` | `ekf_prediction_only_zero_encoder` | 0.000/0.000 | 0.000/0.000 | 0.000 | 0.000 | `true` |
| `skew_shear` | `simplex_ukf_full` | 15.971/51.760 | 16.633/51.546 | 54.168 | 54.163 | `false` |
| `skew_shear` | `simplex_ukf_prediction_only_zero_encoder` | -16.292/-10.335 | -16.211/-10.460 | 19.293 | 19.292 | `false` |
| `baseline` | `ekf_full` | 121.653/-961.000 | 81.875/-384.909 | 968.729 | 393.520 | `false` |
| `baseline` | `ekf_prediction_only_zero_encoder` | 0.000/0.000 | 0.000/0.000 | 0.000 | 0.000 | `true` |
| `baseline` | `simplex_ukf_full` | -56.776/-2028.752 | -45.027/-2028.925 | 2029.546 | 2029.424 | `false` |
| `baseline` | `simplex_ukf_prediction_only_zero_encoder` | -28.467/7.738 | -8.941/28.512 | 29.500 | 29.881 | `false` |
