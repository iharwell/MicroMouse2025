# Segment Loading Benchmark

- Manifest: `staging\traction_candidate_rms_nis_testbed\parallel_replace_20260420_102209_20260610\ukf_one_log_20260610_070622_active_manifest.json`
- Segments: `4`
- Samples: `128`
- Source logs: `1`
- Old median: `14782.213 us/sample`
- Grouped cold-cache median: `2511.090 us/sample`
- Grouped warm-cache median: `32.683 us/sample`
- Cold-cache speedup: `5.89x`
- Warm-cache speedup: `452.29x`
- Cached index builds: `1`
- Cached file reads: `1`

The grouped cached path was verified against the old per-segment sample sequence before timing.
