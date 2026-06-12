# Static Encoder Ablation

- Segment: `ofnis_001617`
- Duration: `30.522` s
- Inputs forced for ablation: zero left/right commands, zero left/right wheel rates, yaw/accel measurements invalid.
- Propagation: candidate plant mean propagation only; no yaw-rate update, no accel updates, no logged UKF state, no encoder NIS.
- Candidate config: `C:\Users\thene\source\repos\MicroMouse2025\staging\traction_candidate_rms_nis_testbed\round_20260611\static_stability_analysis\combined_static_candidate_config.json`
- Full-EKF reference metrics: `C:\Users\thene\source\repos\MicroMouse2025\staging\traction_candidate_rms_nis_testbed\round_20260611\static_stability_analysis\static_stability_metrics.csv`

| Model | Bounded | Final abs vf/vr/yaw_rate/yaw | Max abs vf/vr/yaw_rate/yaw | Pred accel mean f/r/yaw | Full-EKF final abs vf/vr/yaw_rate/yaw |
| --- | --- | --- | --- | --- | --- |
| `stribeck_fade` | `true` | 0/0/0/0 | 0/0/0/0 | 0/0/0 | 0.00287908473224/6.30051766575e-05/0.000359688364294/0.0176505851569 |
| `slip_envelope` | `true` | 0/0/0/0 | 0/0/0/0 | 0/0/0 | 0.00459246581514/7.21834413686e-05/0.00310855220832/0.0160639918618 |
| `in_shear` | `true` | 0/0/0/0 | 0/0/0/0 | 0/0/0 | 0.00281335123392/6.16930975967e-05/0.000561891240823/0.017597418385 |
| `shear_rate` | `true` | 0/0/0/0 | 0/0/0/0 | 0/0/0 | 0.00271442583868/6.25455351842e-05/0.000788843088589/0.0176044514435 |
| `skew_shear` | `true` | 0/0/0/0 | 0/0/0/0 | 0/0/0 | 0.00285893057249/6.26629467873e-05/0.000398212408818/0.0176375087801 |
| `baseline` | `true` | 0/0/0/0 | 0/0/0/0 | 0/0/0 | 0.0399813197262/0.00261513751405/0.00382478999567/0.0135881220999 |
