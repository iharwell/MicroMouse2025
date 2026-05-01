# Sub-3 Logical Line Functions: Tooling, Scripts, and Root Python

Definition used by this report:
- C/C++/Arduino/PowerShell: fewer than 3 non-empty, non-comment body lines after removing brace-only lines.
- Python: fewer than 3 executable body statements after ignoring an initial docstring.

- Files scanned: 23
- Matching functions: 80

## [analyze_all_logs_opening_stationary_allan.py](C:/Users/thene/source/repos/MicroMouse2025/analyze_all_logs_opening_stationary_allan.py)

- `load_csv` at [line 107](C:/Users/thene/source/repos/MicroMouse2025/analyze_all_logs_opening_stationary_allan.py:107) (1 logical line; python)
- `detect_opening_stationary_prefix.bounded` at [line 139](C:/Users/thene/source/repos/MicroMouse2025/analyze_all_logs_opening_stationary_allan.py:139) (2 logical lines; python)
- `adjacent_log_slopes` at [line 205](C:/Users/thene/source/repos/MicroMouse2025/analyze_all_logs_opening_stationary_allan.py:205) (2 logical lines; python)

## [decode_mmlog_to_csv.py](C:/Users/thene/source/repos/MicroMouse2025/decode_mmlog_to_csv.py)

- `FieldSpec.is_string_hash` at [line 62](C:/Users/thene/source/repos/MicroMouse2025/decode_mmlog_to_csv.py:62) (1 logical line; python)
- `SidecarSpec.struct_format` at [line 84](C:/Users/thene/source/repos/MicroMouse2025/decode_mmlog_to_csv.py:84) (1 logical line; python)
- `truncate_hash` at [line 96](C:/Users/thene/source/repos/MicroMouse2025/decode_mmlog_to_csv.py:96) (2 logical lines; python)
- `normalize_string_hash_metadata` at [line 101](C:/Users/thene/source/repos/MicroMouse2025/decode_mmlog_to_csv.py:101) (1 logical line; python)
- `read_mmlog_header_and_payload` at [line 254](C:/Users/thene/source/repos/MicroMouse2025/decode_mmlog_to_csv.py:254) (2 logical lines; python)
- `build_default_output_dir` at [line 387](C:/Users/thene/source/repos/MicroMouse2025/decode_mmlog_to_csv.py:387) (2 logical lines; python)

## [tooling/analyze_encoder_imu_disagreement.py](C:/Users/thene/source/repos/MicroMouse2025/tooling/analyze_encoder_imu_disagreement.py)

- `has_derived_fragment` at [line 57](C:/Users/thene/source/repos/MicroMouse2025/tooling/analyze_encoder_imu_disagreement.py:57) (2 logical lines; python)
- `format_float` at [line 320](C:/Users/thene/source/repos/MicroMouse2025/tooling/analyze_encoder_imu_disagreement.py:320) (1 logical line; python)
- `write_summary_csv` at [line 324](C:/Users/thene/source/repos/MicroMouse2025/tooling/analyze_encoder_imu_disagreement.py:324) (2 logical lines; python)

## [tooling/analyze_open_floor.py](C:/Users/thene/source/repos/MicroMouse2025/tooling/analyze_open_floor.py)

- `print_scalar_summary` at [line 1356](C:/Users/thene/source/repos/MicroMouse2025/tooling/analyze_open_floor.py:1356) (1 logical line; python)
- `print_distribution_summary` at [line 1363](C:/Users/thene/source/repos/MicroMouse2025/tooling/analyze_open_floor.py:1363) (1 logical line; python)
- `print_tail_stats` at [line 1372](C:/Users/thene/source/repos/MicroMouse2025/tooling/analyze_open_floor.py:1372) (1 logical line; python)
- `format_optional_float` at [line 1380](C:/Users/thene/source/repos/MicroMouse2025/tooling/analyze_open_floor.py:1380) (2 logical lines; python)

## [tooling/analyze_open_floor_turn_bias.py](C:/Users/thene/source/repos/MicroMouse2025/tooling/analyze_open_floor_turn_bias.py)

- `strip_cpp_comments` at [line 224](C:/Users/thene/source/repos/MicroMouse2025/tooling/analyze_open_floor_turn_bias.py:224) (2 logical lines; python)
- `load_csv_rows` at [line 329](C:/Users/thene/source/repos/MicroMouse2025/tooling/analyze_open_floor_turn_bias.py:329) (1 logical line; python)
- `_median_finite` at [line 355](C:/Users/thene/source/repos/MicroMouse2025/tooling/analyze_open_floor_turn_bias.py:355) (2 logical lines; python)
- `_counter_to_plain_dict` at [line 360](C:/Users/thene/source/repos/MicroMouse2025/tooling/analyze_open_floor_turn_bias.py:360) (1 logical line; python)
- `cor_x_from_average_velocity` at [line 396](C:/Users/thene/source/repos/MicroMouse2025/tooling/analyze_open_floor_turn_bias.py:396) (1 logical line; python)
- `cor_x_from_center_accel` at [line 404](C:/Users/thene/source/repos/MicroMouse2025/tooling/analyze_open_floor_turn_bias.py:404) (1 logical line; python)
- `_format_optional_mm` at [line 799](C:/Users/thene/source/repos/MicroMouse2025/tooling/analyze_open_floor_turn_bias.py:799) (1 logical line; python)
- `_format_optional_float` at [line 803](C:/Users/thene/source/repos/MicroMouse2025/tooling/analyze_open_floor_turn_bias.py:803) (1 logical line; python)

## [tooling/build_feedforward_tensor.py](C:/Users/thene/source/repos/MicroMouse2025/tooling/build_feedforward_tensor.py)

- `TensorAxis.clamp_value` at [line 51](C:/Users/thene/source/repos/MicroMouse2025/tooling/build_feedforward_tensor.py:51) (2 logical lines; python)
- `TensorAxis.boundary_values` at [line 56](C:/Users/thene/source/repos/MicroMouse2025/tooling/build_feedforward_tensor.py:56) (2 logical lines; python)
- `TensorAxis.locate_bin` at [line 64](C:/Users/thene/source/repos/MicroMouse2025/tooling/build_feedforward_tensor.py:64) (2 logical lines; python)
- `FeedforwardTensor.shape` at [line 106](C:/Users/thene/source/repos/MicroMouse2025/tooling/build_feedforward_tensor.py:106) (1 logical line; python)
- `FeedforwardTensor.to_json_dict` at [line 209](C:/Users/thene/source/repos/MicroMouse2025/tooling/build_feedforward_tensor.py:209) (1 logical line; python)
- `ReservoirSampler.__post_init__` at [line 299](C:/Users/thene/source/repos/MicroMouse2025/tooling/build_feedforward_tensor.py:299) (1 logical line; python)
- `parse_float` at [line 318](C:/Users/thene/source/repos/MicroMouse2025/tooling/build_feedforward_tensor.py:318) (2 logical lines; python)
- `parse_int` at [line 332](C:/Users/thene/source/repos/MicroMouse2025/tooling/build_feedforward_tensor.py:332) (2 logical lines; python)
- `is_comparable_open_floor_transition` at [line 366](C:/Users/thene/source/repos/MicroMouse2025/tooling/build_feedforward_tensor.py:366) (1 logical line; python)
- `is_comparable_diag_transition` at [line 374](C:/Users/thene/source/repos/MicroMouse2025/tooling/build_feedforward_tensor.py:374) (1 logical line; python)
- `commands_are_valid` at [line 387](C:/Users/thene/source/repos/MicroMouse2025/tooling/build_feedforward_tensor.py:387) (1 logical line; python)
- `iter_open_floor_main_samples` at [line 447](C:/Users/thene/source/repos/MicroMouse2025/tooling/build_feedforward_tensor.py:447) (1 logical line; python)
- `iter_competition_diag_samples` at [line 481](C:/Users/thene/source/repos/MicroMouse2025/tooling/build_feedforward_tensor.py:481) (1 logical line; python)
- `iter_sensor_feedforward_samples` at [line 522](C:/Users/thene/source/repos/MicroMouse2025/tooling/build_feedforward_tensor.py:522) (1 logical line; python)
- `iter_feedforward_path_samples` at [line 554](C:/Users/thene/source/repos/MicroMouse2025/tooling/build_feedforward_tensor.py:554) (1 logical line; python)
- `axis_index` at [line 640](C:/Users/thene/source/repos/MicroMouse2025/tooling/build_feedforward_tensor.py:640) (1 logical line; python)
- `discover_source_file_counts` at [line 656](C:/Users/thene/source/repos/MicroMouse2025/tooling/build_feedforward_tensor.py:656) (1 logical line; python)
- `write_tensor_json` at [line 741](C:/Users/thene/source/repos/MicroMouse2025/tooling/build_feedforward_tensor.py:741) (1 logical line; python)
- `write_tensor_cell_csv` at [line 748](C:/Users/thene/source/repos/MicroMouse2025/tooling/build_feedforward_tensor.py:748) (1 logical line; python)
- `format_axis_summary` at [line 789](C:/Users/thene/source/repos/MicroMouse2025/tooling/build_feedforward_tensor.py:789) (1 logical line; python)
- `write_summary_file` at [line 815](C:/Users/thene/source/repos/MicroMouse2025/tooling/build_feedforward_tensor.py:815) (1 logical line; python)
- `load_tensor_json` at [line 819](C:/Users/thene/source/repos/MicroMouse2025/tooling/build_feedforward_tensor.py:819) (1 logical line; python)

## [tooling/competition_feedforward.py](C:/Users/thene/source/repos/MicroMouse2025/tooling/competition_feedforward.py)

- `strip_cpp_comments` at [line 96](C:/Users/thene/source/repos/MicroMouse2025/tooling/competition_feedforward.py:96) (2 logical lines; python)
- `milli_amps_to_amps` at [line 151](C:/Users/thene/source/repos/MicroMouse2025/tooling/competition_feedforward.py:151) (1 logical line; python)
- `milli_newton_meters_to_newton_meters` at [line 155](C:/Users/thene/source/repos/MicroMouse2025/tooling/competition_feedforward.py:155) (1 logical line; python)
- `rpm_to_rad_per_second` at [line 159](C:/Users/thene/source/repos/MicroMouse2025/tooling/competition_feedforward.py:159) (1 logical line; python)
- `compute_motor_speed_constant_radps_per_volt` at [line 163](C:/Users/thene/source/repos/MicroMouse2025/tooling/competition_feedforward.py:163) (2 logical lines; python)
- `summarize_competition_sweep` at [line 531](C:/Users/thene/source/repos/MicroMouse2025/tooling/competition_feedforward.py:531) (2 logical lines; python)
- `analyze_competition_feedforward` at [line 550](C:/Users/thene/source/repos/MicroMouse2025/tooling/competition_feedforward.py:550) (2 logical lines; python)
- `discover_competition_diag_csvs` at [line 705](C:/Users/thene/source/repos/MicroMouse2025/tooling/competition_feedforward.py:705) (1 logical line; python)

## [tooling/open_floor_launch_floor.py](C:/Users/thene/source/repos/MicroMouse2025/tooling/open_floor_launch_floor.py)

- `percentile_l95` at [line 82](C:/Users/thene/source/repos/MicroMouse2025/tooling/open_floor_launch_floor.py:82) (1 logical line; python)
- `median_of` at [line 86](C:/Users/thene/source/repos/MicroMouse2025/tooling/open_floor_launch_floor.py:86) (2 logical lines; python)

## [tooling/open_floor_plant_fit.py](C:/Users/thene/source/repos/MicroMouse2025/tooling/open_floor_plant_fit.py)

- `mean_trace_point_from_rows` at [line 215](C:/Users/thene/source/repos/MicroMouse2025/tooling/open_floor_plant_fit.py:215) (1 logical line; python)
- `central_derivative` at [line 276](C:/Users/thene/source/repos/MicroMouse2025/tooling/open_floor_plant_fit.py:276) (2 logical lines; python)
- `static_friction_speed_threshold_radps` at [line 350](C:/Users/thene/source/repos/MicroMouse2025/tooling/open_floor_plant_fit.py:350) (2 logical lines; python)

## [tooling/open_floor_recovery.py](C:/Users/thene/source/repos/MicroMouse2025/tooling/open_floor_recovery.py)

- `parse_float` at [line 113](C:/Users/thene/source/repos/MicroMouse2025/tooling/open_floor_recovery.py:113) (1 logical line; python)
- `parse_int` at [line 117](C:/Users/thene/source/repos/MicroMouse2025/tooling/open_floor_recovery.py:117) (1 logical line; python)
- `row_dt_seconds` at [line 121](C:/Users/thene/source/repos/MicroMouse2025/tooling/open_floor_recovery.py:121) (1 logical line; python)
- `row_watchdog_flags` at [line 125](C:/Users/thene/source/repos/MicroMouse2025/tooling/open_floor_recovery.py:125) (1 logical line; python)
- `encoder_diff_speed_mps` at [line 129](C:/Users/thene/source/repos/MicroMouse2025/tooling/open_floor_recovery.py:129) (1 logical line; python)
- `independent_gyro_radps` at [line 133](C:/Users/thene/source/repos/MicroMouse2025/tooling/open_floor_recovery.py:133) (1 logical line; python)
- `corrected_accel_body` at [line 181](C:/Users/thene/source/repos/MicroMouse2025/tooling/open_floor_recovery.py:181) (1 logical line; python)

## [tooling/open_floor_yaw_fft.py](C:/Users/thene/source/repos/MicroMouse2025/tooling/open_floor_yaw_fft.py)

- `_hann_window` at [line 146](C:/Users/thene/source/repos/MicroMouse2025/tooling/open_floor_yaw_fft.py:146) (2 logical lines; python)
- `_finite_median` at [line 155](C:/Users/thene/source/repos/MicroMouse2025/tooling/open_floor_yaw_fft.py:155) (2 logical lines; python)

## [tooling/search_open_floor_ukf_tuning.py](C:/Users/thene/source/repos/MicroMouse2025/tooling/search_open_floor_ukf_tuning.py)

- `timestamp` at [line 84](C:/Users/thene/source/repos/MicroMouse2025/tooling/search_open_floor_ukf_tuning.py:84) (1 logical line; python)
- `write_tuning_file` at [line 88](C:/Users/thene/source/repos/MicroMouse2025/tooling/search_open_floor_ukf_tuning.py:88) (2 logical lines; python)
- `load_json` at [line 93](C:/Users/thene/source/repos/MicroMouse2025/tooling/search_open_floor_ukf_tuning.py:93) (1 logical line; python)
- `tuning_signature` at [line 117](C:/Users/thene/source/repos/MicroMouse2025/tooling/search_open_floor_ukf_tuning.py:117) (1 logical line; python)
- `current_value` at [line 121](C:/Users/thene/source/repos/MicroMouse2025/tooling/search_open_floor_ukf_tuning.py:121) (1 logical line; python)

## [tooling/test_build_feedforward_tensor.py](C:/Users/thene/source/repos/MicroMouse2025/tooling/test_build_feedforward_tensor.py)

- `BuildFeedforwardTensorTest.write_csv` at [line 14](C:/Users/thene/source/repos/MicroMouse2025/tooling/test_build_feedforward_tensor.py:14) (2 logical lines; python)
- `BuildFeedforwardTensorTest.test_iter_open_floor_main_samples_uses_sensor_transition` at [line 21](C:/Users/thene/source/repos/MicroMouse2025/tooling/test_build_feedforward_tensor.py:21) (1 logical line; python)
- `BuildFeedforwardTensorTest.test_build_feedforward_tensor_aggregates_recognized_sources` at [line 57](C:/Users/thene/source/repos/MicroMouse2025/tooling/test_build_feedforward_tensor.py:57) (1 logical line; python)

## [tooling/test_competition_feedforward.py](C:/Users/thene/source/repos/MicroMouse2025/tooling/test_competition_feedforward.py)

- `CompetitionFeedforwardTest.make_setup` at [line 16](C:/Users/thene/source/repos/MicroMouse2025/tooling/test_competition_feedforward.py:16) (2 logical lines; python)
- `CompetitionFeedforwardTest.test_analyze_competition_feedforward_uses_hold_segment_and_outcomes.sample_row` at [line 75](C:/Users/thene/source/repos/MicroMouse2025/tooling/test_competition_feedforward.py:75) (1 logical line; python)

## [tooling/test_open_floor_launch_floor.py](C:/Users/thene/source/repos/MicroMouse2025/tooling/test_open_floor_launch_floor.py)

- `OpenFloorLaunchFloorTest.make_row` at [line 7](C:/Users/thene/source/repos/MicroMouse2025/tooling/test_open_floor_launch_floor.py:7) (1 logical line; python)

## [tooling/test_open_floor_plant_fit.py](C:/Users/thene/source/repos/MicroMouse2025/tooling/test_open_floor_plant_fit.py)

- `OpenFloorPlantFitTest.make_launch_row` at [line 11](C:/Users/thene/source/repos/MicroMouse2025/tooling/test_open_floor_plant_fit.py:11) (1 logical line; python)
