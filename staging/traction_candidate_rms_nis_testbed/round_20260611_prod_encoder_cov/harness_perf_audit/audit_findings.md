# Standalone Traction/UKF Harness Performance Audit

## Scope

- Profiled the standalone Python testbed only.
- No production code was edited.
- Profiling used one Python process and bounded subsets from the existing one-log manifest.

## Timings

| Path | Median time |
| --- | ---: |
| Direct `CandidatePlant.propagate()` | 92.8 us/propagation |
| Direct `CandidatePlant.plant_result()` | 61.4 us/call |
| Simplex predict + yaw/forward/right updates | 6,233.7 us/sample |
| CSV row write only | 24.5 us/row |
| CSV sample loading through per-segment reader | 15,568.1 us/emitted sample |

The direct propagation number matches the prior 70-95 us range. It is not an embedded algorithm cost estimate; it is dominated by Python object/list/dict/function-call overhead.

## Bottlenecks

1. `CandidatePlant.plant_result()` and contact-force evaluation dominate compute. A single profiled simplex sample made about 65 `plant_result()` calls and 15 `propagate()` calls. The hot functions are `plant_result`, `contact_force`, dataclass construction, `dict.get`, `max`, `sum`, `sqrt`, `hypot`, `tanh`, and finite checks.
2. The Python simplex path stores full covariance and rebuilds simplex points with a Cholesky factorization for predict and for every scalar measurement update. Production stores square-root covariance and generates sigma points from that square root.
3. Measurement updates re-evaluate acceleration measurement functions across sigma points. The two accel updates call `plant_result()` for each active sigma point. This mirrors the conceptual UKF measurement structure, but the Python cost per call is high.
4. CSV loading is pathological for the one-log script shape. Loading only 128 emitted samples through four segment reads advanced 126,644 CSV rows because `read_segment_samples()` opens the file per segment and skips rows until the segment start.
5. CSV writing itself is not the main bottleneck in the bounded profile, but full row artifact creation still adds memory pressure and extra post-update plant prediction work in large runs.

## Production Comparison

The production UKF uses fixed-size `float` Eigen matrices, a simplex sigma policy, cached predicted sigma points between predict and update, square-root covariance, QR square-root covariance propagation, and Cholesky rank updates. The Python harness mirrors the high-level UKF idea, but it does not mirror the production data representation or low-level covariance mechanics.

The largest avoidable structural deviation is the Python full-covariance representation plus repeated Cholesky regeneration. The largest non-algorithmic overhead is Python object traffic in the plant/contact model. The largest IO overhead is repeated per-segment file scanning.

## Recommended Fixes

1. Cache log rows or stream once by log path, then dispatch samples to segments. This is low semantic risk and directly fixes the repeated file-scan cost.
2. Add a no-row-artifact mode for profiling/tuning loops, or emit only aggregate diagnostics unless row artifacts are explicitly requested. Low semantic risk if outputs are gated by an option.
3. Hoist per-candidate constants out of hot loops: vehicle scalars, parameter lookups, contact geometry, bank side masks, and common function locals. Medium-low risk; requires golden-output tests because floating-point operation order can shift slightly.
4. Replace hot dataclasses/dicts in the testbed inner loop with tuples, slot classes, or fixed arrays while keeping boundary APIs unchanged. Medium risk; requires comparison against current harness outputs and production spot checks.
5. Rework the standalone simplex replay to store square-root covariance and generate simplex points from the square root, matching production more closely. Medium-high effort, but this is the best production-equivalent algorithmic rewrite.
6. Consider NumPy or a small compiled extension for fixed-size matrix/contact kernels if the testbed must run full candidate sweeps quickly. Higher integration risk; must be covered by production-equivalence tests.

## Artifacts

- `profile_harness_paths.py`
- `profile_measurements.csv`
- `profile_summary.json`
- `profile_report.md`
- `direct_propagate.pstats`
- `simplex_predict_update.pstats`
- `row_format_after_simplex.pstats`
- `csv_sample_load.pstats`
