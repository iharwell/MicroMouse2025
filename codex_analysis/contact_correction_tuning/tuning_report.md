# Contact Correction Tuning

Targets use raw gyro minus independently estimated stationary bias where available, encoders, drive commands, and timestamps. UKF targets are not used.

Base gain: 0.010000000 N*s/m.
Least-squares all-sample gain: -10.088021093 N*s/m.
Least-squares open-floor gain: -6.086872416 N*s/m.
Least-squares fit-authoritative gain: -8.300211993 N*s/m.
Least-squares validation-only gain: -1.992556270 N*s/m.
Selected exact gain: -4.986872416 N*s/m.
Yaw denominator including wheel spin-up: 0.000246510400642 kg*m^2.

## Selected Exact Aggregate

| Dataset | Samples | Old RMSE | Tuned RMSE | Relative delta |
| --- | ---: | ---: | ---: | ---: |
| all_included | 2994884 | 0.212190004 | 0.191075678 | -9.950670% |
| open_floor_only | 2362926 | 0.148593814 | 0.133014323 | -10.484616% |
| competition_only | 631958 | 0.361683931 | 0.326906885 | -9.615314% |
| fit_authoritative_open_floor | 1182997 | 0.150058239 | 0.128486689 | -14.375452% |
| fit_downweighted_open_floor | 779760 | 0.174304915 | 0.161881455 | -7.127430% |
| validation_only_open_floor | 361987 | 0.068655577 | 0.068616233 | -0.057306% |

## Selected Exact Motion Split

| Family | Motion | Samples | Old RMSE | Tuned RMSE | Relative delta |
| --- | --- | ---: | ---: | ---: | ---: |
| all | in_place_yaw | 493183 | 0.358457371 | 0.331973860 | -7.388190% |
| all | moving_yaw | 604633 | 0.259057883 | 0.230506845 | -11.021104% |
| all | mostly_forward | 1195941 | 0.104577079 | 0.082561911 | -21.051619% |
| all | low_motion_commanded | 701127 | 0.159410197 | 0.144859347 | -9.127929% |
| open_floor | in_place_yaw | 302259 | 0.205513948 | 0.196530070 | -4.371420% |
| open_floor | moving_yaw | 497103 | 0.206063378 | 0.184524855 | -10.452378% |
| open_floor | mostly_forward | 1046658 | 0.099300462 | 0.077716014 | -21.736502% |
| open_floor | low_motion_commanded | 516906 | 0.124240335 | 0.115408012 | -7.109062% |
| competition | in_place_yaw | 190924 | 0.514826526 | 0.472791417 | -8.164907% |
| competition | moving_yaw | 107530 | 0.425512443 | 0.375975007 | -11.641830% |
| competition | mostly_forward | 149283 | 0.135937253 | 0.110734033 | -18.540334% |
| competition | low_motion_commanded | 184221 | 0.231091206 | 0.206136089 | -10.798817% |

The linear sweep chooses candidates from one identical sample set; the selected tables above are from an exact replay with the selected coefficient.
