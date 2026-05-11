# Reviewer Plato Handoff

Role: read-only architecture reviewer over the dirty patch before Worker E/F corrections.

Status: completed. No files edited.

## Findings At Time Of Review

- Blocker: `DriveBase` still owned motor hardware and encoder capture.
  - It constructed `_leftMotor` and `_rightMotor`.
  - It exposed `ConsumeEncoderObservation`.
  - It called command application paths such as `SetOpenLoopRaw`.
- Blocker: `DriveBase` had a public caller-supplied `PlantParams` constructor path and stored prepared params.
  - This allowed divergence from runtime `PlantModel(speedVehicle)`.
- Blocker: encoder capture had moved into `RuntimeSensorSuite`, but `DriveBase` still read stale local sensor/encoder caches.
- Blocker: `Estimator::ukf()` removal was incomplete at that point.
  - Production callers still used it in `ShowcasingDonutController` and `OpenFloorMeasurementController`.
  - `DriveBaseTest` still used it.
- Blocker: migration was explicitly partial.
  - `Estimator.h` said call-site migration was pending.
  - Tests still called removed DriveBase measurement/pose paths.

## Notes

These findings predate Workers E, F, G, and H. They are useful as a rejection checklist, not as a current-state report.

