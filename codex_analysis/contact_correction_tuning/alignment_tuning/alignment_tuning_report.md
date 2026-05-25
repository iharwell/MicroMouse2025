# Contact Correction Alignment Tuning

Targets use raw gyro minus independently estimated stationary bias where available, encoders, drive commands, and timestamps. UKF targets are not used.

Selected response lag: +2 sample(s).
Selected gain: -6.496619190 N*s/m.
Selection reason: fit_primary_validation_delta_pct=1.798094.

## Split Quality

| Lag | Dataset | Samples | Fit old RMSE | Dataset old RMSE | Offset vs fit | Policy |
| ---: | --- | ---: | ---: | ---: | ---: | --- |
| 0 | fit_downweighted_open_floor | 779760 | 0.150058239 | 0.174304915 | 16.158177% | keep_validation |
| 0 | validation_only_open_floor | 361987 | 0.150058239 | 0.068655577 | -54.247380% | stress_only_off_distribution |
| 0 | competition_stress | 631958 | 0.150058239 | 0.361683931 | 141.029039% | stress_only_off_distribution |
| 1 | fit_downweighted_open_floor | 778375 | 0.147584238 | 0.165086008 | 11.858834% | keep_validation |
| 1 | validation_only_open_floor | 361208 | 0.147584238 | 0.069406753 | -52.971432% | stress_only_off_distribution |
| 1 | competition_stress | 626862 | 0.147584238 | 0.361421120 | 144.891409% | stress_only_off_distribution |
| 2 | fit_downweighted_open_floor | 776989 | 0.144705675 | 0.157111646 | 8.573244% | keep_validation |
| 2 | validation_only_open_floor | 360484 | 0.144705675 | 0.068998992 | -52.317702% | stress_only_off_distribution |
| 2 | competition_stress | 622538 | 0.144705675 | 0.360934074 | 149.426343% | stress_only_off_distribution |

## Selected Gain By Alignment

| Lag | Dataset | Samples | Old RMSE | Tuned RMSE | Relative delta |
| ---: | --- | ---: | ---: | ---: | ---: |
| 0 | competition_stress | 631958 | 0.361683931 | 0.318445252 | -11.954824% |
| 0 | fit_authoritative_open_floor | 1182997 | 0.150058239 | 0.125362034 | -16.457747% |
| 0 | open_floor_only | 2362926 | 0.148593814 | 0.131765612 | -11.324968% |
| 0 | validation_only_open_floor | 361987 | 0.068655577 | 0.068753590 | 0.142762% |
| 1 | competition_stress | 626862 | 0.361421120 | 0.318286104 | -11.934835% |
| 1 | fit_authoritative_open_floor | 1179943 | 0.147584238 | 0.124279261 | -15.790966% |
| 1 | open_floor_only | 2357696 | 0.144171501 | 0.128291943 | -11.014353% |
| 1 | validation_only_open_floor | 361208 | 0.069406753 | 0.070531522 | 1.620547% |
| 2 | competition_stress | 622538 | 0.360934074 | 0.318031333 | -11.886587% |
| 2 | fit_authoritative_open_floor | 1176893 | 0.144705675 | 0.120903153 | -16.448921% |
| 2 | open_floor_only | 2352522 | 0.139536353 | 0.123211282 | -11.699511% |
| 2 | validation_only_open_floor | 360484 | 0.068998992 | 0.070239658 | 1.798094% |
