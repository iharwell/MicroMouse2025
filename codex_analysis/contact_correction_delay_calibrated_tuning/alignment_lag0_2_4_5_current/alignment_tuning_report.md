# Contact Correction Alignment Tuning

Targets use raw gyro minus independently estimated stationary bias where available, encoders, drive commands, and timestamps. UKF targets are not used.

Selected response lag: +5 sample(s).
Selected gain: -6.496619190 N*s/m.
Selection reason: forced_gain_exact_replay.

## Split Quality

| Lag | Dataset | Samples | Fit old RMSE | Dataset old RMSE | Offset vs fit | Policy |
| ---: | --- | ---: | ---: | ---: | ---: | --- |
| 0 | fit_downweighted_open_floor | 779760 | 0.150058239 | 0.174304915 | 16.158177% | keep_validation |
| 0 | validation_only_open_floor | 361987 | 0.150058239 | 0.068655577 | -54.247380% | stress_only_off_distribution |
| 0 | competition_stress | 631958 | 0.150058239 | 0.361683931 | 141.029039% | stress_only_off_distribution |
| 2 | fit_downweighted_open_floor | 776989 | 0.144705675 | 0.157111646 | 8.573244% | keep_validation |
| 2 | validation_only_open_floor | 360484 | 0.144705675 | 0.068998992 | -52.317702% | stress_only_off_distribution |
| 2 | competition_stress | 622538 | 0.144705675 | 0.360934074 | 149.426343% | stress_only_off_distribution |
| 4 | fit_downweighted_open_floor | 774212 | 0.141863703 | 0.144079209 | 1.561715% | keep_validation |
| 4 | validation_only_open_floor | 359085 | 0.141863703 | 0.066315731 | -53.253912% | stress_only_off_distribution |
| 4 | competition_stress | 614159 | 0.141863703 | 0.357835420 | 152.238884% | stress_only_off_distribution |
| 5 | fit_downweighted_open_floor | 772826 | 0.145087836 | 0.144659400 | -0.295294% | keep_validation |
| 5 | validation_only_open_floor | 358410 | 0.145087836 | 0.069237960 | -52.278591% | stress_only_off_distribution |
| 5 | competition_stress | 610015 | 0.145087836 | 0.357099122 | 146.126162% | stress_only_off_distribution |

## Selected Gain By Alignment

| Lag | Dataset | Samples | Old RMSE | Tuned RMSE | Relative delta |
| ---: | --- | ---: | ---: | ---: | ---: |
| 0 | competition_stress | 631958 | 0.361683931 | 0.313626841 | -13.287040% |
| 0 | fit_authoritative_open_floor | 1182997 | 0.150058239 | 0.123026261 | -18.014325% |
| 0 | open_floor_only | 2362926 | 0.148593814 | 0.128884803 | -13.263682% |
| 0 | validation_only_open_floor | 361987 | 0.068655577 | 0.068717158 | 0.089697% |
| 2 | competition_stress | 622538 | 0.360934074 | 0.313252733 | -13.210540% |
| 2 | fit_authoritative_open_floor | 1176893 | 0.144705675 | 0.118536295 | -18.084557% |
| 2 | open_floor_only | 2352522 | 0.139536353 | 0.120279694 | -13.800461% |
| 2 | validation_only_open_floor | 360484 | 0.068998992 | 0.070181036 | 1.713133% |
| 4 | competition_stress | 614159 | 0.357835420 | 0.310297382 | -13.284889% |
| 4 | fit_authoritative_open_floor | 1170787 | 0.141863703 | 0.110393541 | -22.183378% |
| 4 | open_floor_only | 2342221 | 0.133164220 | 0.108459774 | -18.551865% |
| 4 | validation_only_open_floor | 359085 | 0.066315731 | 0.066028754 | -0.432744% |
| 5 | competition_stress | 610015 | 0.357099122 | 0.309483493 | -13.334009% |
| 5 | fit_authoritative_open_floor | 1167748 | 0.145087836 | 0.112598395 | -22.392946% |
| 5 | open_floor_only | 2337117 | 0.135130102 | 0.108203575 | -19.926372% |
| 5 | validation_only_open_floor | 358410 | 0.069237960 | 0.068803285 | -0.627798% |
