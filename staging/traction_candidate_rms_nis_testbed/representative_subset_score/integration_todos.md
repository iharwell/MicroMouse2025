# Traction RMS NIS Testbed Integration TODOs

The tuning/ranking harness is present. Standalone data/model modules must produce NIS artifacts before rankings are meaningful.

Required integration points:

1. Use a testbed-only data loader for decoded open-floor logs. It must reject or ignore `ukf_state_*` and `logged_ukf_state*` columns.
2. Use a testbed-only traction/estimator replay path that consumes `trial_configs/**.json` and emits NIS CSV rows.
3. Keep estimator covariance fixed. Candidate bounds may tune physical traction/model parameters only.
4. Carry launch observed-command summaries from the segment manifest so itemized reports can separate launch behavior by command bucket.
5. Point the manifest at generated NIS CSV artifacts, then run `score`.

This intentionally does not request production or hardware hooks.
