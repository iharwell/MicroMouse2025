# Delay-Calibrated Contact Correction Tuning

Primary alignment: +4 samples.
Secondary derivative/onset check: +5 samples.
Current production gain: -6.496619190 N*s/m.
Selected +4 fit-authoritative gain: -8.189143994 N*s/m.

Targets use raw gyro minus independent bias, encoders, drive commands, and timestamps. UKF targets are not used.
Fit authority is fit-authoritative open-floor only. Validation-only open-floor and competition/aux data are reported as stress/off-distribution checks.

## Aggregate

| response_lag_samples | dataset | gain_label | samples | old_rmse_radps | candidate_rmse_radps | relative_delta_pct |
| --- | --- | --- | --- | --- | --- | --- |
| 4 | fit_authoritative_open_floor | old_pre_correction | 1170787 | 0.141863703 | 0.141863703 | 0.000000 |
| 4 | fit_authoritative_open_floor | current_production | 1170787 | 0.141863703 | 0.110393541 | -22.183378 |
| 4 | fit_authoritative_open_floor | delay_calibrated_retuned | 1170787 | 0.141863703 | 0.108777277 | -23.322686 |
| 4 | validation_only_open_floor | old_pre_correction | 359085 | 0.066315731 | 0.066315731 | 0.000000 |
| 4 | validation_only_open_floor | current_production | 359085 | 0.066315731 | 0.066028754 | -0.432744 |
| 4 | validation_only_open_floor | delay_calibrated_retuned | 359085 | 0.066315731 | 0.066171193 | -0.217955 |
| 4 | open_floor_only | old_pre_correction | 2342221 | 0.133164220 | 0.133164220 | 0.000000 |
| 4 | open_floor_only | current_production | 2342221 | 0.133164220 | 0.108459774 | -18.551865 |
| 4 | open_floor_only | delay_calibrated_retuned | 2342221 | 0.133164220 | 0.109043723 | -18.113347 |
| 4 | competition_stress | old_pre_correction | 614159 | 0.357835420 | 0.357835420 | 0.000000 |
| 4 | competition_stress | current_production | 614159 | 0.357835420 | 0.310297382 | -13.284889 |
| 4 | competition_stress | delay_calibrated_retuned | 614159 | 0.357835420 | 0.301644804 | -15.702922 |
| 5 | fit_authoritative_open_floor | old_pre_correction | 1167748 | 0.145087836 | 0.145087836 | 0.000000 |
| 5 | fit_authoritative_open_floor | current_production | 1167748 | 0.145087836 | 0.112598395 | -22.392946 |
| 5 | fit_authoritative_open_floor | delay_calibrated_retuned | 1167748 | 0.145087836 | 0.110500247 | -23.839069 |
| 5 | validation_only_open_floor | old_pre_correction | 358410 | 0.069237960 | 0.069237960 | 0.000000 |
| 5 | validation_only_open_floor | current_production | 358410 | 0.069237960 | 0.068803285 | -0.627798 |
| 5 | validation_only_open_floor | delay_calibrated_retuned | 358410 | 0.069237960 | 0.068898395 | -0.490432 |
| 5 | open_floor_only | old_pre_correction | 2337117 | 0.135130102 | 0.135130102 | 0.000000 |
| 5 | open_floor_only | current_production | 2337117 | 0.135130102 | 0.108203575 | -19.926372 |
| 5 | open_floor_only | delay_calibrated_retuned | 2337117 | 0.135130102 | 0.108086041 | -20.013351 |
| 5 | competition_stress | old_pre_correction | 610015 | 0.357099122 | 0.357099122 | 0.000000 |
| 5 | competition_stress | current_production | 610015 | 0.357099122 | 0.309483493 | -13.334009 |
| 5 | competition_stress | delay_calibrated_retuned | 610015 | 0.357099122 | 0.300804211 | -15.764506 |

## Open-Floor Motion (+4)

| motion_class | gain_label | samples | old_rmse_radps | candidate_rmse_radps | relative_delta_pct |
| --- | --- | --- | --- | --- | --- |
| in_place_yaw | old_pre_correction | 299060 | 0.167288741 | 0.167288741 | 0.000000 |
| in_place_yaw | current_production | 299060 | 0.167288741 | 0.152822307 | -8.647584 |
| in_place_yaw | delay_calibrated_retuned | 299060 | 0.167288741 | 0.155994998 | -6.751048 |
| moving_yaw | old_pre_correction | 491574 | 0.177081948 | 0.177081948 | 0.000000 |
| moving_yaw | current_production | 491574 | 0.177081948 | 0.138517151 | -21.777938 |
| moving_yaw | delay_calibrated_retuned | 491574 | 0.177081948 | 0.141863626 | -19.888149 |
| mostly_forward | old_pre_correction | 1037606 | 0.099772850 | 0.099772850 | 0.000000 |
| mostly_forward | current_production | 1037606 | 0.099772850 | 0.070554280 | -29.285091 |
| mostly_forward | delay_calibrated_retuned | 1037606 | 0.099772850 | 0.068765411 | -31.078033 |

## Yaw Launch

| response_lag_samples | launch_set | gain_label | samples | old_rmse_radps | candidate_rmse_radps | relative_delta_pct |
| --- | --- | --- | --- | --- | --- | --- |
| 4 | yaw_launch_sustained_0p65_0p70 | old_pre_correction | 14000 | 0.139019072 | 0.139019072 | 0.000000 |
| 4 | yaw_launch_sustained_0p65_0p70 | current_production | 14000 | 0.139019072 | 0.149653969 | 7.649955 |
| 4 | yaw_launch_sustained_0p65_0p70 | delay_calibrated_retuned | 14000 | 0.139019072 | 0.152764046 | 9.887114 |
| 4 | yaw_launch_twitch_only_0p50_0p55 | old_pre_correction | 14000 | 0.107601881 | 0.107601881 | 0.000000 |
| 4 | yaw_launch_twitch_only_0p50_0p55 | current_production | 14000 | 0.107601881 | 0.109343837 | 1.618890 |
| 4 | yaw_launch_twitch_only_0p50_0p55 | delay_calibrated_retuned | 14000 | 0.107601881 | 0.109881982 | 2.119016 |
| 5 | yaw_launch_sustained_0p65_0p70 | old_pre_correction | 14000 | 0.141143885 | 0.141143885 | 0.000000 |
| 5 | yaw_launch_sustained_0p65_0p70 | current_production | 14000 | 0.141143885 | 0.151157153 | 7.094369 |
| 5 | yaw_launch_sustained_0p65_0p70 | delay_calibrated_retuned | 14000 | 0.141143885 | 0.154116003 | 9.190705 |
| 5 | yaw_launch_twitch_only_0p50_0p55 | old_pre_correction | 14000 | 0.108668384 | 0.108668384 | 0.000000 |
| 5 | yaw_launch_twitch_only_0p50_0p55 | current_production | 14000 | 0.108668384 | 0.110180608 | 1.391595 |
| 5 | yaw_launch_twitch_only_0p50_0p55 | delay_calibrated_retuned | 14000 | 0.108668384 | 0.110659420 | 1.832213 |
