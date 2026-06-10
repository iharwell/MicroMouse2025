# Representative Traction RMS/NIS Corpus

This directory fixes the source-log corpus for traction candidate tuning/evaluation. It is generated from `staging\traction_candidate_rms_nis_segments\segment_manifest.json` and keeps the source segment boundaries intact.

## Selected Source Logs

| source log | coverage |
| --- | --- |
| `mmlog_decode_2026-06-08_23-37-25` | Preserved latest 2026-06-08 `mixed_launch` log; includes launch, yaw_launch, mixed_launch, and boundary-cropped `smooth`. |
| `mmlog_decode_2026-06-09_23-36-21` | Replacement for `mmlog_decode_2026-05-27_04-44-04`; includes launch, yaw_launch, mixed_launch, and boundary-cropped current `straight`. |
| `mmlog_decode_2026-05-04_16-57-53` | Fan 1.0 compact launch/yaw_launch and lower yaw command sweep. |
| `mmlog_decode_2026-04-22_12-10-34` | Added recent fan 0.8 SEC_40_YAW yaw calibration, boundary-cropped at the fixed selector-removal cutoff. |
| `mmlog_decode_2026-04-20_10-22-09` | Added recent fan 0.8 SEC_40_YAW yaw calibration, boundary-cropped at the fixed selector-removal cutoff. |
| `mmlog_decode_2026-04-20_04-54-09` | Fan 0.0 legacy launch plus SEC_40_YAW yaw calibration. |
| `mmlog_decode_2026-04-15_02-09-58` | Legacy straight and smooth-turn active coverage. |
| `mmlog_decode_2026-04-10_18-33-52` | Old launch threshold sweep from 0.17 through 0.25. |

The original six-log rationale remains: five logs cannot cover the latest mixed-launch log, old launch threshold sweep, fan-off yaw calibration, fan-1.0 yaw launch, recent straight, and legacy smooth-turn active section simultaneously. This fixed corpus now uses eight logs, replaces the old 2026-05-27 compact source with the 2026-06-09 run, and keeps all three valid boundary-cropped SEC_40_YAW yaw calibration samples.

Current corpus counts after the 2026-06-10 replacement:

- 570 total segments.
- 562 primary active segments.
- 80 `mixed_launch` segments, including the preserved 40 from `mmlog_decode_2026-06-08_23-37-25`.
- 3 `SEC_40_YAW` segments, one from each preserved SEC_40 source log.

## Corruption Boundary Rule

Use the manifest's predetermined cutoffs consistently. Recovery segments are excluded. Boundary-ended active calibration segments remain part of the primary active corpus only for their `active_start_row_index..active_end_row_index` window before the terminal cutoff. Consumers that skip boundary-ended segments by default must opt into boundary-cropped active rows, then evaluate only active rows for the primary objective.

Yaw launch and SEC_40_YAW yaw calibration sections are valid calibration data unless the source manifest marks a terminal pickup/runoff/external-force boundary; those rows are cropped by the fixed boundary instead of being ad hoc excluded.

Do not use logged UKF/replay state columns. The source segmentation and this corpus use command, sensor, sidecar, and terminal-fault metadata only.

Yaw/gyro NIS gating is disabled for this testbed corpus: rejected-rate policy for yaw/gyro is forced zero/ignored, while accelerometer validity remains the NIS rejection gate. Active-only validation manifests must use this full representative corpus as their bias source so static/stationary accelerometer bias assessment is still available.

## Files

- `segment_manifest.json`: tool-compatible selected segment manifest plus corpus policy metadata.
- `selected_segments.csv`: flat audit of included segment IDs and boundaries.
