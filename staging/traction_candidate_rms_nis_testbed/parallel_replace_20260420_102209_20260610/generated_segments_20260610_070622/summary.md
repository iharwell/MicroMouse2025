# Open-Floor RMS/NIS Segment Manifest Summary

- Sources discovered: 1
- Sources included after duplicate collapse: 1
- Duplicate sources collapsed: 0
- Segments emitted: 102
- Segments ending at reliable stationary points: 101
- Segments ending at corruption boundaries: 1

## Method

- Legacy logs with `section_id` are normalized as `legacy_section_phase`; compact logs without `section_id` are normalized from sidecar `phase_battery_*` metadata.
- Segmentation ignores every `ukf_state_*` column. Boundaries use CSV metadata, drive commands, encoder/gyro stillness, and parsed terminal fault timestamps.
- Reliable stationary runs require at least 25 consecutive rows with zero command and low direct sensor motion.
- Selector-removal, workspace, recovery-timeout, runoff-like, and related terminal faults trim only the terminal segment; earlier non-corrupted segments remain valid.

## Schemas

| schema | sources |
| --- | ---: |
| compact_phase_battery | 1 |

## Stages

| stage | segments |
| --- | ---: |
| launch | 18 |
| mixed_launch | 40 |
| smooth | 1 |
| static | 1 |
| yaw_launch | 42 |

## Families

| family | segments |
| --- | ---: |
| launch | 100 |
| smooth_turn | 1 |
| static_hold | 1 |

## Terminal Faults

| fault_class | sources |
| --- | ---: |
| selector_removed | 1 |

## Artifacts

- `segment_manifest.json`: full machine-readable manifest with nested command summaries.
- `segment_manifest.csv`: flat table for quick filtering.
- `summary.md`: this report.
