# Top Speed Mode Audit Against Boot Mode Infrastructure Spec

## Scope

This audit compares `TopSpeedMeasurementMode` against the current target architecture in `boot_mode_infrastructure_spec.md`.

## Findings

1. Descriptor and startup text are not authoritative today.
   The mode runs with `3.0f` forward acceleration for `6000000U` us, but the operator-facing startup text and descriptor still describe an `8.5 m/s^2` launch for up to `1 second`. The spec says the descriptor should be the authoritative human-facing summary, so this drift makes the metadata unreliable.

2. The mode still owns lifecycle machinery that the spec assigns to shared infrastructure.
   `TopSpeedMeasurementMode` still directly registers the fault handler, brings hardware up, begins sensors, flushes `logging.txt`, opens and closes the `.mmlog`, and manages selector monitoring. The spec's target shape is for those shared utility-mode mechanics to converge into shared runtime ownership or `BootUtilityModeFramework`, not stay mode-local.

3. Launch was only partially registry-driven before the current registry cleanup.
   The registry entry and descriptor existed, but `MazeMapApplication.cpp` still kept a manual startup-mode switch and dedicated controller storage. That meant the registry was not yet the sole authority for launch routing.

4. Run state leaks through file-scope state.
   The file-scope `accelcount` is outside the authoritative mode owner, is not reset in `ResetRunState()`, and is used in the timeout path even though elapsed microseconds are already computed locally. That is both an ownership problem and a re-entry correctness risk.

## Notes

- This audit treats `boot_mode_infrastructure_spec.md` as the target architecture, not as proof that the rest of the repository is already fully migrated.
- Some gaps are therefore broader repository cleanup items, but they still show up concretely in top speed mode.
