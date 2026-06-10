# Open-Floor RMS/NIS Segment Manifest Summary

- Sources discovered: 55
- Sources included after duplicate collapse: 52
- Duplicate sources collapsed: 3
- Segments emitted: 1719
- Segments ending at reliable stationary points: 1690
- Segments ending at corruption boundaries: 29

## Method

- Legacy logs with `section_id` are normalized as `legacy_section_phase`; compact logs without `section_id` are normalized from sidecar `phase_battery_*` metadata.
- Segmentation ignores every `ukf_state_*` column. Boundaries use CSV metadata, drive commands, encoder/gyro stillness, and parsed terminal fault timestamps.
- Reliable stationary runs require at least 25 consecutive rows with zero command and low direct sensor motion.
- Selector-removal, workspace, recovery-timeout, runoff-like, and related terminal faults trim only the terminal segment; earlier non-corrupted segments remain valid.

## Schemas

| schema | sources |
| --- | ---: |
| compact_phase_battery | 7 |
| legacy_section_phase | 45 |

## Stages

| stage | segments |
| --- | ---: |
| SEC_10_STATIC | 44 |
| SEC_20_LAUNCH | 1158 |
| SEC_30_STRAIGHT | 24 |
| SEC_40_YAW | 5 |
| SEC_50_SMOOTH | 15 |
| launch | 72 |
| mixed_launch | 40 |
| smooth | 1 |
| static | 6 |
| straight | 2 |
| yaw_launch | 352 |

## Families

| family | segments |
| --- | ---: |
| in_place_turn | 5 |
| launch | 1619 |
| recovery | 6 |
| smooth_turn | 14 |
| static_hold | 51 |
| straight | 24 |

## Terminal Faults

| fault_class | sources |
| --- | ---: |
| capture_or_setup_failed | 1 |
| fault | 1 |
| selector_removed | 44 |
| workspace_violation | 4 |

## Collapsed Duplicates

| skipped path | duplicate of |
| --- | --- |
| `TestResults/mmlog_decode_2026-04-12_06-44-12/open_floor_main.csv` | `TestResults/mmlog_decode_2026-04-12_06-36-32/open_floor_main.csv` |
| `TestResults/mmlog_decode_2026-04-14_07-51-39/open_floor_main.csv` | `TestResults/mmlog_decode_2026-04-14_07-25-50/open_floor_main.csv` |
| `TestResults/mmlog_decode_2026-04-22_01-06-32/open_floor_main.csv` | `TestResults/mmlog_decode_2026-04-21_23-39-12/open_floor_main.csv` |

## Artifacts

- `segment_manifest.json`: full machine-readable manifest with nested command summaries.
- `segment_manifest.csv`: flat table for quick filtering.
- `summary.md`: this report.
