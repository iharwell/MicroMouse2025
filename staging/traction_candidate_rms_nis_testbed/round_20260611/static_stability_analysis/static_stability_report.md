# Static Stability Analysis

- Selected manifest: `C:\Users\thene\source\repos\MicroMouse2025\staging\traction_candidate_rms_nis_testbed\round_20260611\static_stability_analysis\selected_static_segment_manifest.json`
- Candidate config: `C:\Users\thene\source\repos\MicroMouse2025\staging\traction_candidate_rms_nis_testbed\round_20260611\static_stability_analysis\combined_static_candidate_config.json`
- Row diagnostics: `C:\Users\thene\source\repos\MicroMouse2025\staging\traction_candidate_rms_nis_testbed\round_20260611\static_stability_analysis\replay_rows\residual_diagnostics.csv`
- NIS rows: `C:\Users\thene\source\repos\MicroMouse2025\staging\traction_candidate_rms_nis_testbed\round_20260611\static_stability_analysis\replay_rows\nis_samples.csv`
- Fixed covariance: `C:\Users\thene\source\repos\MicroMouse2025\staging\traction_candidate_rms_nis_testbed\covariance_conservative.json`
- Logged UKF states used: `false`

| Model | Samples | Final | Max abs | Pred accel mean f/r/yaw | Pred accel max abs f/r/yaw | Residual mean yaw/f/r | RMS NIS all/yaw/f/r | Command max abs L/R wheel L/R |
| --- | ---: | --- | --- | --- | --- | --- | --- | --- |
| `baseline` | 30523 | vf=0.0399813197, vr=0.00261513751, yr=0.00382479, yaw=0.0135881221 | vf=0.0405712087, vr=0.00691786616, yr=0.0266287943, yaw=0.0136091718 | -0.000179229142/-0.0112369517/-0.0110150232 | 0.130088296/0.362367811/6.10744656 | 4.64535465e-05/0.000179229142/0.0112369517 | 0.479907107/0.828500648/0.0612620205/0.0276791368 | 0/0 0/0 |
| `in_shear` | 30523 | vf=0.00281335123, vr=6.16930976e-05, yr=0.000561891241, yaw=0.0175974184 | vf=0.00524593934, vr=0.00169963528, yr=0.036283817, yaw=0.0176072395 | -0.0310769274/0.00308338821/-0.112115059 | 0.516129805/0.2451996/6.10760061 | 6.17696834e-05/0.0310769274/-0.00308338821 | 0.260640581/0.450095482/0.0320167394/0.0137667209 | 0/0 0/0 |
| `shear_rate` | 30523 | vf=0.00271442584, vr=6.25455352e-05, yr=0.000788843089, yaw=0.0176044514 | vf=0.00501180463, vr=0.00181955778, yr=0.0364030634, yaw=0.017614129 | -0.0330250268/0.00301192844/-0.109181466 | 0.493628193/0.249662925/6.20828337 | 6.38623268e-05/0.0330250268/-0.00301192844 | 0.255579559/0.441330186/0.031767806/0.0134612512 | 0/0 0/0 |
| `skew_shear` | 30523 | vf=0.00285893057, vr=6.26629468e-05, yr=0.000398212409, yaw=0.0176375088 | vf=0.00534716256, vr=0.00199787139, yr=0.0362068298, yaw=0.0176474199 | -0.0297835514/0.00311651463/-0.114557105 | 0.52376939/0.245459483/5.96292164 | 5.96686056e-05/0.0297835514/-0.00311651463 | 0.264216767/0.456268051/0.0324443396/0.014082949 | 0/0 0/0 |
| `slip_envelope` | 30523 | vf=0.00459246582, vr=7.21834414e-05, yr=0.00310855221, yaw=0.0160639919 | vf=0.00916288349, vr=0.00300251635, yr=0.0319722501, yaw=0.0160780216 | 0.00475215509/0.00280163653/-0.111418804 | 0.668646334/0.208364995/3.42201 | 5.71529508e-05/-0.00475215509/-0.00280163653 | 0.36654795/0.632992751/0.0441120405/0.0211306378 | 0/0 0/0 |
| `stribeck_fade` | 30523 | vf=0.00287908473, vr=6.30051767e-05, yr=0.000359688364, yaw=0.0176505852 | vf=0.00540091352, vr=0.00210685686, yr=0.0361628588, yaw=0.0176605198 | -0.0295613408/0.00312943826/-0.115772344 | 0.530488849/0.245079246/5.94146133 | 5.85822674e-05/0.0295613408/-0.00312943826 | 0.265122084/0.457828887/0.0325749649/0.0141683009 | 0/0 0/0 |
