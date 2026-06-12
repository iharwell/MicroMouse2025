# Yaw Launch Signed Residual Bias Analysis

Sign convention: `residual = measurement - prediction`. Positive means the model under-predicted the measured stream; negative means the model over-predicted it.

Inputs: existing `round_20260611` tuned/carry-forward candidate configs, yaw-launch-only manifest derived from the oscillation-filtered assessment manifest, fixed `covariance_conservative.json`, and representative-corpus bias source. Replay mode was standalone residual plant replay; no encoder NIS and no logged UKF state were used.

Authoritative derived outputs:

- `signed_residuals/signed_residual_compact.csv`
- `signed_residuals/signed_residual_summary.csv`
- `signed_residuals/summary.json`

Earlier row-output attempts in `replay_rows/` and `residual_replay_rows/` timed out and are incomplete; they were not used for the results below.

Processed set: 236 yaw-launch segments, 174625 samples, 4 source logs. High-command bin uses `abs((right_command - left_command) / 2) >= 0.65`.

## Overall signed residuals

| Model | Stream | n | mean | median | sign |
| --- | --- | ---: | ---: | ---: | --- |
| `stribeck_fade` | `yaw_rate` | 174625 | 0.0215368 | 0.0161244 | positive |
| `stribeck_fade` | `forward_accel` | 174625 | -1.6465 | -0.139843 | negative |
| `stribeck_fade` | `right_accel` | 174625 | -0.593292 | -0.0766133 | negative |
| `slip_envelope` | `yaw_rate` | 174625 | 0.0289464 | 0.014617 | positive |
| `slip_envelope` | `forward_accel` | 174625 | -1.63907 | -0.135904 | negative |
| `slip_envelope` | `right_accel` | 174625 | -0.645447 | -0.0848529 | negative |
| `in_shear` | `yaw_rate` | 174625 | 0.0312053 | 0.0169898 | positive |
| `in_shear` | `forward_accel` | 174625 | -1.52127 | -0.135519 | negative |
| `in_shear` | `right_accel` | 174625 | -0.520283 | -0.0706883 | negative |
| `shear_rate` | `yaw_rate` | 174625 | 0.0288654 | 0.0169382 | positive |
| `shear_rate` | `forward_accel` | 174625 | -1.55153 | -0.135297 | negative |
| `shear_rate` | `right_accel` | 174625 | -0.518434 | -0.0686191 | negative |
| `skew_shear` | `yaw_rate` | 174625 | -0.048759 | 0.0129603 | mean negative, median positive |
| `skew_shear` | `forward_accel` | 174625 | -1.59145 | -0.127225 | negative |
| `skew_shear` | `right_accel` | 174625 | -0.644577 | -0.0803456 | negative |
| `baseline` | `yaw_rate` | 174625 | -4.48012 | -0.175408 | negative |
| `baseline` | `forward_accel` | 174625 | -901.81 | -149.685 | negative |
| `baseline` | `right_accel` | 174625 | -900.152 | -148.202 | negative |

## Polarity and high-command consistency

| Model | Stream | negative yaw cmd mean/median | positive yaw cmd mean/median | high cmd mean/median | consistent? |
| --- | --- | ---: | ---: | ---: | --- |
| `stribeck_fade` | `yaw_rate` | +0.194907 / -0.285658 | -0.152662 / +0.332793 | +0.0176765 / +0.0193686 | no |
| `stribeck_fade` | `forward_accel` | -1.86601 / -0.157706 | -1.86634 / -0.182063 | -5.2706 / -2.72269 | yes |
| `stribeck_fade` | `right_accel` | -0.935446 / -0.160671 | -0.399304 / -0.0332971 | -1.85165 / -0.851296 | yes |
| `slip_envelope` | `yaw_rate` | -0.00329171 / -0.50523 | +0.0625197 / +0.553353 | +0.0370741 / +0.0573799 | no |
| `slip_envelope` | `forward_accel` | -1.89863 / -0.17786 | -1.81843 / -0.152924 | -5.21746 / -2.3891 | yes |
| `slip_envelope` | `right_accel` | -0.943291 / -0.139699 | -0.510977 / -0.0649028 | -1.99604 / -0.863618 | yes |
| `in_shear` | `yaw_rate` | +0.292459 / -0.254848 | -0.228549 / +0.308132 | +0.0468573 / +0.0309344 | no |
| `in_shear` | `forward_accel` | -1.67169 / -0.116581 | -1.77686 / -0.204183 | -4.8852 / -2.23345 | yes |
| `in_shear` | `right_accel` | -0.898182 / -0.177886 | -0.269269 / -0.00421941 | -1.61948 / -0.703055 | yes |
| `shear_rate` | `yaw_rate` | +0.30224 / -0.253877 | -0.243724 / +0.304965 | +0.0348028 / +0.028606 | no |
| `shear_rate` | `forward_accel` | -1.70563 / -0.120505 | -1.81378 / -0.200223 | -4.98839 / -2.42296 | yes |
| `shear_rate` | `right_accel` | -0.888156 / -0.171984 | -0.279746 / -0.00582297 | -1.61747 / -0.718274 | yes |
| `skew_shear` | `yaw_rate` | +0.111394 / -0.296548 | -0.229063 / +0.338118 | -0.216049 / -0.00436877 | no |
| `skew_shear` | `forward_accel` | -1.75546 / -0.123963 | -1.85863 / -0.18194 | -5.10488 / -2.5448 | yes |
| `skew_shear` | `right_accel` | -1.08301 / -0.185947 | -0.377552 / -0.0286064 | -1.99465 / -0.967109 | yes |
| `baseline` | `yaw_rate` | -107.355 / -68.6446 | +99.8378 / +49.6915 | -1.69401 / -0.00772133 | no |
| `baseline` | `forward_accel` | -750.759 / -162.195 | -688.524 / -77.2818 | -1545.2 / -1176.42 | yes |
| `baseline` | `right_accel` | -736.448 / -150.654 | -698.725 / -86.3952 | -1540.79 / -1172.13 | yes |

Conclusion: forward and right acceleration residuals are consistently negative across overall, command polarity, and high-command bins for every model in this comparison, so those streams are over-predicted on yaw-launch. Yaw-rate residual sign is not consistent across command polarity. The tuned non-baseline models are near zero overall in yaw-rate, but polarity bins split by command direction; `skew_shear` also flips mean versus median overall and is negative in the high-command yaw-rate bin.
