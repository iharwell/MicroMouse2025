# Round-2 Force/Contact Surface Failure Modes

- The fixed launch core preserves the B-scale in-place breakaway point, but the fitted surface is still a residual correction. It should not be treated as a final physical tire law without force-replay validation.
- The model is rejected as a viable candidate if the +1 rad/s in-place command is below `0.60` in absolute left/right command, regardless of broad RMSE.
- The moving-contact term is materially different from the first-round low-order ridge/surface attempts because it projects bounded per-contact `N*tanh(v_rel/v_k)` traction proxies into yaw moment before fitting branch gains. If future work collapses this back to arbitrary low-order features, it should be treated as a different and weaker model.
- Synthetic grid evaluation uses the same approximate contact replay as the prior L/R grid: no full PlantModel projection is replayed after the residual correction changes applied tire-force demand.
- Lateral body velocity is unavailable in the source logs; right-relative contact velocity assumes `Vr=0`, so lateral scrub terms are reconstruction features.
- Rows with limiter or hardware-saturation evidence are downweighted, not excluded. The surface may still learn some projection artifacts.
- The contact surface is launch-gated to avoid erasing the B breakaway authority. That means it may underfit real low-speed sliding regimes that are neither static launch nor forward-speed-dominated.
- The target is differentiated gyro yaw acceleration, so single-row residuals contain timing jitter and derivative noise. Aggregate split/run metrics are more meaningful than individual samples.
- The model uses force activation and continuous schedules, not command values or maneuver labels. It can still be biased by the open-floor coverage distribution because selected maneuvers dominate the available data.
