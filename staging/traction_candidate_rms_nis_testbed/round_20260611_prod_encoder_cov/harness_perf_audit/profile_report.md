# Harness Performance Profile

| Path | Median us/unit | Mean us/unit | Units | Median total s |
| --- | ---: | ---: | ---: | ---: |
| `direct_propagate` | 92.835 | 94.721 | 2400 | 0.222805 |
| `plant_result_only` | 61.385 | 57.110 | 4800 | 0.294647 |
| `simplex_predict_update` | 6233.688 | 6211.766 | 32 | 0.199478 |
| `row_format_after_simplex` | 5124.703 | 5068.737 | 32 | 0.163991 |
| `csv_write_32_rows` | 24.547 | 26.407 | 32 | 0.000786 |
| `csv_sample_load` | 15568.127 | 15983.115 | 128 | 1.992720 |

## cProfile Hot Spots

### direct_propagate
```
         1313197 function calls in 0.792 seconds

   Ordered by: cumulative time
   List reduced from 36 to 35 due to restriction <35>

   ncalls  tottime  percall  cumtime  percall filename:lineno(function)
        1    0.000    0.000    0.834    0.834 profile_harness_paths.py:220(<lambda>)
        1    0.004    0.004    0.834    0.834 profile_harness_paths.py:119(run_propagate_loop)
     2400    0.018    0.000    0.830    0.000 estimator_core.py:1008(propagate)
     4800    0.014    0.000    0.807    0.000 estimator_core.py:1021(derivative)
     4800    0.100    0.000    0.789    0.000 estimator_core.py:1039(plant_result)
    19200    0.186    0.000    0.416    0.000 estimator_core.py:1260(contact_force)
     4800    0.085    0.000    0.177    0.000 estimator_core.py:1144(make_plant_result)
   386597    0.085    0.000    0.096    0.000 {built-in method builtins.max}
    19200    0.032    0.000    0.062    0.000 estimator_core.py:1349(replayable_contact_velocity_rates)
    38400    0.029    0.000    0.053    0.000 estimator_core.py:415(smooth_sign)
    14400    0.021    0.000    0.039    0.000 {built-in method builtins.sum}
    76800    0.026    0.000    0.038    0.000 estimator_core.py:350(finite)
   153600    0.034    0.000    0.034    0.000 {method 'get' of 'dict' objects}
    91397    0.020    0.000    0.020    0.000 {built-in method math.sqrt}
     4800    0.019    0.000    0.019    0.000 <string>:2(__init__)
    19200    0.009    0.000    0.019    0.000 estimator_core.py:420(smooth_scale_to_unit)
    96000    0.017    0.000    0.017    0.000 {built-in method builtins.abs}
    57600    0.015    0.000    0.015    0.000 {built-in method math.hypot}
    79200    0.013    0.000    0.013    0.000 {built-in method math.isfinite}
    19200    0.011    0.000    0.011    0.000 estimator_core.py:1246(relative_velocity)
    24000    0.007    0.000    0.010    0.000 estimator_core.py:1172(<genexpr>)
    38400    0.010    0.000    0.010    0.000 {built-in method math.tanh}
     4800    0.005    0.000    0.006    0.000 estimator_core.py:1202(normal_loads)
    14400    0.005    0.000    0.005    0.000 estimator_core.py:1050(<genexpr>)
    19200    0.004    0.000    0.004    0.000 {method 'append' of 'list' objects}
    24000    0.004    0.000    0.004    0.000 estimator_core.py:1084(<genexpr>)
    14400    0.004    0.000    0.004    0.000 estimator_core.py:1051(<genexpr>)
    24000    0.004    0.000    0.004    0.000 estimator_core.py:1196(<genexpr>)
    24000    0.003    0.000    0.003    0.000 estimator_core.py:1197(<genexpr>)
     9600    0.002    0.000    0.002    0.000 {built-in method math.sin}
     9600    0.002    0.000    0.002    0.000 {built-in method math.cos}
     2400    0.001    0.000    0.002    0.000 estimator_core.py:354(finite_or)
     4800    0.002    0.000    0.002    0.000 estimator_core.py:407(normalize_angle)
     4800    0.001    0.000    0.001    0.000 estimator_core.py:1426(yaw_loss_damping)
     2400    0.001    0.000    0.001    0.000 {built-in method builtins.min}

```

### simplex_predict_update
```
         826130 function calls in 0.467 seconds

   Ordered by: cumulative time
   List reduced from 83 to 35 due to restriction <35>

   ncalls  tottime  percall  cumtime  percall filename:lineno(function)
        1    0.000    0.000    0.485    0.485 profile_harness_paths.py:221(<lambda>)
        1    0.001    0.001    0.485    0.485 profile_harness_paths.py:139(run_simplex_loop)
     2080    0.039    0.000    0.340    0.000 estimator_core.py:1039(plant_result)
       32    0.003    0.000    0.285    0.009 run_static_simplex_ukf_analysis.py:318(predict)
       32    0.001    0.000    0.195    0.006 run_static_simplex_ukf_analysis.py:356(update_production_measurements)
       96    0.006    0.000    0.194    0.002 run_static_simplex_ukf_analysis.py:272(measurement_update)
     8320    0.079    0.000    0.180    0.000 estimator_core.py:1260(contact_force)
      480    0.004    0.000    0.167    0.000 estimator_core.py:1008(propagate)
      960    0.003    0.000    0.162    0.000 estimator_core.py:1021(derivative)
      128    0.008    0.000    0.089    0.001 run_static_simplex_ukf_analysis.py:219(simplex_points)
     2080    0.036    0.000    0.078    0.000 estimator_core.py:1144(make_plant_result)
    23680    0.042    0.000    0.072    0.000 {built-in method builtins.sum}
      352    0.001    0.000    0.059    0.000 run_static_simplex_ukf_analysis.py:385(<lambda>)
      352    0.001    0.000    0.058    0.000 run_static_simplex_ukf_analysis.py:374(<lambda>)
     1280    0.012    0.000    0.056    0.000 run_static_simplex_ukf_analysis.py:143(matvec)
   169074    0.042    0.000    0.050    0.000 {built-in method builtins.max}
       32    0.000    0.000    0.049    0.002 profile_harness_paths.py:63(process_noise_like)
       32    0.002    0.000    0.049    0.002 estimator_core.py:1485(process_noise)
       32    0.000    0.000    0.047    0.001 estimator_core.py:1563(encoder_input_noise)
     8320    0.017    0.000    0.032    0.000 estimator_core.py:1349(replayable_contact_velocity_rates)
      128    0.009    0.000    0.024    0.000 run_static_simplex_ukf_analysis.py:169(regularized_cholesky)
    16640    0.012    0.000    0.021    0.000 estimator_core.py:415(smooth_sign)
    40560    0.012    0.000    0.018    0.000 estimator_core.py:350(finite)
   115200    0.017    0.000    0.017    0.000 run_static_simplex_ukf_analysis.py:144(<genexpr>)
    66560    0.013    0.000    0.013    0.000 {method 'get' of 'dict' objects}
       64    0.002    0.000    0.012    0.000 run_static_simplex_ukf_analysis.py:124(max_abs)
    43346    0.009    0.000    0.009    0.000 {built-in method math.sqrt}
    50144    0.008    0.000    0.008    0.000 {built-in method builtins.abs}
     8320    0.004    0.000    0.008    0.000 estimator_core.py:420(smooth_scale_to_unit)
    55088    0.008    0.000    0.008    0.000 {built-in method math.isfinite}
     2080    0.008    0.000    0.008    0.000 <string>:2(__init__)
       32    0.002    0.000    0.007    0.000 run_static_simplex_ukf_analysis.py:258(simplex_covariance)
    24960    0.006    0.000    0.006    0.000 {built-in method math.hypot}
    14016    0.004    0.000    0.006    0.000 run_static_simplex_ukf_analysis.py:116(finite)
     8320    0.005    0.000    0.005    0.000 estimator_core.py:1246(relative_velocity)

```

### row_format_after_simplex
```
         829650 function calls in 0.500 seconds

   Ordered by: cumulative time
   List reduced from 86 to 35 due to restriction <35>

   ncalls  tottime  percall  cumtime  percall filename:lineno(function)
        1    0.000    0.000    0.520    0.520 profile_harness_paths.py:222(<lambda>)
        1    0.001    0.001    0.520    0.520 profile_harness_paths.py:157(run_row_format_loop)
     2080    0.042    0.000    0.363    0.000 estimator_core.py:1039(plant_result)
       32    0.003    0.000    0.308    0.010 run_static_simplex_ukf_analysis.py:318(predict)
       32    0.001    0.000    0.203    0.006 run_static_simplex_ukf_analysis.py:356(update_production_measurements)
       96    0.007    0.000    0.202    0.002 run_static_simplex_ukf_analysis.py:272(measurement_update)
     8320    0.087    0.000    0.197    0.000 estimator_core.py:1260(contact_force)
      480    0.004    0.000    0.180    0.000 estimator_core.py:1008(propagate)
      960    0.003    0.000    0.175    0.000 estimator_core.py:1021(derivative)
      128    0.009    0.000    0.094    0.001 run_static_simplex_ukf_analysis.py:219(simplex_points)
     2080    0.038    0.000    0.079    0.000 estimator_core.py:1144(make_plant_result)
    23648    0.043    0.000    0.075    0.000 {built-in method builtins.sum}
      352    0.001    0.000    0.062    0.000 run_static_simplex_ukf_analysis.py:385(<lambda>)
      352    0.001    0.000    0.060    0.000 run_static_simplex_ukf_analysis.py:374(<lambda>)
     1280    0.012    0.000    0.059    0.000 run_static_simplex_ukf_analysis.py:143(matvec)
       32    0.000    0.000    0.056    0.002 profile_harness_paths.py:63(process_noise_like)
       32    0.002    0.000    0.056    0.002 estimator_core.py:1485(process_noise)
       32    0.001    0.000    0.053    0.002 estimator_core.py:1563(encoder_input_noise)
   169106    0.043    0.000    0.052    0.000 {built-in method builtins.max}
     8320    0.017    0.000    0.034    0.000 estimator_core.py:1349(replayable_contact_velocity_rates)
      128    0.010    0.000    0.025    0.000 run_static_simplex_ukf_analysis.py:169(regularized_cholesky)
    16640    0.013    0.000    0.023    0.000 estimator_core.py:415(smooth_sign)
    40560    0.014    0.000    0.021    0.000 estimator_core.py:350(finite)
   115200    0.019    0.000    0.019    0.000 run_static_simplex_ukf_analysis.py:144(<genexpr>)
       96    0.002    0.000    0.015    0.000 run_static_simplex_ukf_analysis.py:124(max_abs)
    66656    0.014    0.000    0.014    0.000 {method 'get' of 'dict' objects}
     8320    0.004    0.000    0.010    0.000 estimator_core.py:420(smooth_scale_to_unit)
    43346    0.010    0.000    0.010    0.000 {built-in method math.sqrt}
    55376    0.010    0.000    0.010    0.000 {built-in method math.isfinite}
    50432    0.009    0.000    0.009    0.000 {built-in method builtins.abs}
     2080    0.009    0.000    0.009    0.000 <string>:2(__init__)
       32    0.002    0.000    0.007    0.000 run_static_simplex_ukf_analysis.py:258(simplex_covariance)
    14304    0.004    0.000    0.007    0.000 run_static_simplex_ukf_analysis.py:116(finite)
    24960    0.007    0.000    0.007    0.000 {built-in method math.hypot}
     7776    0.003    0.000    0.007    0.000 run_static_simplex_ukf_analysis.py:128(<genexpr>)

```

### csv_sample_load
```
         910402 function calls in 2.588 seconds

   Ordered by: cumulative time
   List reduced from 82 to 35 due to restriction <35>

   ncalls  tottime  percall  cumtime  percall filename:lineno(function)
        1    0.000    0.000    2.588    2.588 profile_harness_paths.py:223(<lambda>)
        1    0.000    0.000    2.588    2.588 profile_harness_paths.py:194(run_sample_load)
      132    0.228    0.002    2.577    0.020 estimator_core.py:2111(read_segment_samples)
   126644    0.749    0.000    2.289    0.000 csv.py:173(__next__)
   126648    1.408    0.000    1.441    0.000 {built-in method builtins.next}
   253292    0.059    0.000    0.059    0.000 csv.py:159(fieldnames)
      128    0.004    0.000    0.057    0.000 estimator_core.py:2333(sample_from_row)
     1408    0.029    0.000    0.049    0.000 estimator_core.py:395(csv_value)
   253401    0.041    0.000    0.041    0.000 {built-in method builtins.len}
     9135    0.012    0.000    0.032    0.000 <frozen codecs>:322(decode)
     9134    0.006    0.000    0.021    0.000 utf_8_sig.py:54(_buffer_decode)
   101989    0.018    0.000    0.018    0.000 {method 'lower' of 'str' objects}
     9135    0.015    0.000    0.015    0.000 {built-in method _codecs.utf_8_decode}
        1    0.001    0.001    0.011    0.011 estimator_core.py:2056(segment_specs_from_manifest)
        1    0.000    0.000    0.005    0.005 estimator_core.py:1913(load_json)
        1    0.000    0.000    0.004    0.004 __init__.py:304(loads)
        1    0.000    0.000    0.004    0.004 decoder.py:340(decode)
        1    0.004    0.004    0.004    0.004 decoder.py:351(raw_decode)
      100    0.000    0.000    0.002    0.000 __init__.py:505(is_absolute)
      100    0.000    0.000    0.002    0.000 <frozen ntpath>:80(isabs)
      252    0.002    0.000    0.002    0.000 <string>:2(__init__)
      109    0.000    0.000    0.001    0.000 __init__.py:190(__fspath__)
     1152    0.001    0.000    0.001    0.000 estimator_core.py:371(parse_float)
      204    0.000    0.000    0.001    0.000 {built-in method nt.fspath}
      109    0.000    0.000    0.001    0.000 __init__.py:251(__str__)
        4    0.000    0.000    0.001    0.000 __init__.py:663(exists)
        4    0.001    0.000    0.001    0.000 {built-in method nt._path_exists}
        5    0.000    0.000    0.001    0.000 __init__.py:763(open)
        5    0.001    0.000    0.001    0.000 {built-in method _io.open}
      128    0.000    0.000    0.001    0.000 estimator_core.py:2489(sample_with_previous_inputs)
     4773    0.001    0.000    0.001    0.000 {method 'get' of 'dict' objects}
        1    0.000    0.000    0.001    0.001 __init__.py:780(read_text)
        1    0.001    0.001    0.001    0.001 {method 'read' of '_io.TextIOWrapper' objects}
      104    0.000    0.000    0.001    0.000 __init__.py:341(drive)
     4284    0.001    0.000    0.001    0.000 {method 'strip' of 'str' objects}

```
