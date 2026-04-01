#pragma once
#include "Defines.h"
#include "Maze.h"
#include "Vehicle.h"
namespace Config
{
    // Supported mission tuning parameters live here. Treat hard-coded literals elsewhere as implementation
    // details unless they are intentionally promoted into this section with documentation.
    // Likelihood tags for this high-spec robot:
    // [High] commonly adjusted while chasing race pace and consistency.
    // [Medium] adjusted when logs show model mismatch, sensing issues, or workflow friction.
    // [Low] normally fixed after hardware bring-up, wiring, and rule handling are settled.

    // [Low] Maze cell size in meters. This is derived from the maze model and normally should not be edited here;
    // only change it if the robot is being adapted to a different maze standard.
    const float kCellSizeM = MazeMap::Maze::GetCellDimension() / 100.0f;
    // [Low] Physical wall thickness for the maze standard. Each neighboring cell contributes half of this thickness to
    // the occupied wall, so the open span inside one cell is smaller than the cell pitch.
    constexpr float kMazeWallThicknessM = 0.012f;
    // [Low] Clear open span inside one cell after subtracting the 12 mm shared wall thickness from the 180 mm pitch.
    const float kCellClearSpanM = kCellSizeM - kMazeWallThicknessM;
    // [High] Vehicle is the sole authority for the robot's physical dimensions. Track width and its wear-bounded
    // envelope are sourced from the shared vehicle model rather than duplicated here.
    constexpr float kTrackWidthPhysicalMinM = MazeMap::Vehicle::GetPhysicalModel().trackWidthPhysicalMinM;
    constexpr float kTrackWidthPhysicalMaxM = MazeMap::Vehicle::GetPhysicalModel().trackWidthPhysicalMaxM;
    constexpr float kTrackWidthM = MazeMap::Vehicle::GetPhysicalModel().trackWidthM;
    // The contact-patch envelope remains useful as a geometric reference, but the shared kinematic fit may exceed it
    // until wheel-diameter/compliance calibration is fully closed out.
    // [Low] Main mission control period. Shorten it only if the loop still has timing margin and you need faster
    // correction; lengthen it if added work causes deadline misses or sensor/SD contention.
    constexpr unsigned long kControlPeriodUs = 1000UL;
    // [Medium] Startup gyro bias averaging window for normal mission mode. Increase if stationary heading bias is noisy
    // between runs; decrease if startup latency matters and the IMU bias is already repeatable.
    constexpr uint16_t kGyroBiasSamples = 300U;
    // [High] Minimum wall-clock window used for each startup gyro bias measurement. Keep this at or above 500 ms so
    // encoder-safe retries still produce a stable stationary average even if the configured sample count is reduced later.
    constexpr uint16_t kGyroBiasMinimumAveragingWindowMs = 500U;
    // [Medium] Maximum absolute raw yaw rate still treated as stationary while adapting the gyro bias estimate. Raise
    // it only if startup bias convergence is clearly too slow; lower it if short settle windows keep pulling motion
    // transients into the bias estimate.
    constexpr float kGyroBiasUpdateMaxAbsRateRadps = 0.03f;
    // [Medium] Time to sit still before wall observations. Increase if wall sensors or chassis motion need longer to
    // settle after stopping; decrease if mapping feels sluggish and readings are already stable.
    constexpr uint16_t kObservationSettleMs = 25U;
    // [Medium] Continuous stationary dwell required before the startup wall-calibration routine may move. Increase if
    // the robot can still begin while being placed by hand; decrease only if startup latency matters more than this gate.
    constexpr uint16_t kMissionStartupStationaryHoldMs = 2000U;
    // [Medium] Maximum absolute chassis and wheel speed still counted as stationary for the startup dwell. Increase only
    // if encoder noise keeps resetting the timer; decrease if the robot can still creep while being treated as settled.
    constexpr float kMissionStartupStationarySpeedThresholdMps = 0.01f;
    // [Medium] Maximum absolute yaw rate still counted as stationary for the startup dwell. Keep this close to the gyro
    // bias-adaptation gate unless logs show harmless IMU noise is delaying mission start.
    constexpr float kMissionStartupStationaryMaxAbsYawRateRadps = 0.03f;
    // [Medium] Stop-to-stop mission and calibration moves should not hand off to the next segment until the chassis is
    // genuinely settled. These thresholds are tighter than the old profile exit tolerances so short moves do not carry
    // residual wheel motion into the next turn or pose snap.
    constexpr float kMotionSettleSpeedThresholdMps = 0.010f;
    constexpr float kMotionSettleAngularSpeedThresholdRadps = 0.05f;
    constexpr uint16_t kMotionSettleHoldMs = 20U;
    constexpr uint16_t kMotionSettleTimeoutMs = 500U;
    // [Medium] Stationary hold before each startup wall-calibration sample. Increase if the chassis or IR readings
    // need more time to settle after the calibration motions; decrease if startup takes too long and the samples are clean.
    constexpr uint16_t kStartupWallCalibrationSettleMs = 40U;
    // [Medium] Cruise speed for the startup wall-calibration translation moves. Raise it only after confirming the
    // short calibration moves stop repeatably; lower it if the robot overshoots the sample points.
    constexpr float kStartupWallCalibrationSpeedMps = 0.24f;
    // [Medium] Acceleration limit for startup wall calibration. Increase if the routine feels unnecessarily slow;
    // decrease if the initial ramp disturbs the sampled wall readings.
    constexpr float kStartupWallCalibrationAccelMps2 = 1.50f;
    // [Medium] Braking limit for startup wall calibration. Increase if the robot drifts past the reference poses;
    // decrease if it skids or rocks at the end of the calibration moves.
    constexpr float kStartupWallCalibrationDecelMps2 = 1.60f;
    // [Medium] Turn-rate limit for startup wall calibration. Keep this in the same regime as mission search turns so
    // startup calibration exercises the same alignment authority the robot will rely on once the run begins.
    constexpr float kStartupWallCalibrationTurnMaxOmegaRadps = 8.0f;
    // [Medium] Angular-acceleration limit for startup wall calibration. Keep this close to the mission search turn ramp
    // so the calibration turns break static friction reliably without jumping all the way to the more aggressive test envelope.
    constexpr float kStartupWallCalibrationTurnAccelRadps2 = 45.0f;
    // [High] Shared wheel-speed proportional gain multiplier used by all motion modes. This is the profile that
    // startup calibration proved out, and it is now the nominal wheel controller everywhere so turn behavior stays
    // consistent between calibration, corridor tests, mapping, and racing transitions.
    constexpr float kNominalWheelVelocityKpScale = 2.10f;
    // [High] Shared wheel-speed integral gain multiplier used by all motion modes.
    constexpr float kNominalWheelVelocityKiScale = 1.70f;
    // [High] Shared wheel-speed integral limit multiplier used by all motion modes.
    constexpr float kNominalWheelIntegralLimitScale = 2.10f;
    // [Medium] Mapping-only transient wheel-response multiplier. This is kept out of racing so mapping can feel more
    // decisive without changing the high-speed run behavior.
    constexpr float kMappingWheelAccelerationResponseScale = 1.0f;
    // Startup calibration now reuses the common wheel profile instead of carrying a separate controller regime.
    constexpr float kStartupWallCalibrationWheelVelocityKpScale = kNominalWheelVelocityKpScale;
    constexpr float kStartupWallCalibrationWheelVelocityKiScale = kNominalWheelVelocityKiScale;
    constexpr float kStartupWallCalibrationWheelIntegralLimitScale = kNominalWheelIntegralLimitScale;
    // [Medium] Cruise speed for the short centering moves during startup calibration. Keep this below the main startup
    // calibration cruise so the robot can pull cleanly off touched walls without running past the intended center pose.
    constexpr float kStartupWallCalibrationCenteringSpeedMps = 0.16f;
    // [Medium] Acceleration used for startup-calibration centering moves. Raise it only if these short pull-offs still
    // hesitate; lower it if they jump far enough to disturb the calibration geometry.
    constexpr float kStartupWallCalibrationCenteringAccelMps2 = 0.90f;
    // [Medium] Braking used for startup-calibration centering moves. Raise it if those pull-offs coast long; lower it
    // if they become abrupt enough to shake the wall-sensor readings.
    constexpr float kStartupWallCalibrationCenteringDecelMps2 = 1.20f;
    // [Medium] Number of full in-place turns used for startup front calibration. This is intentionally a single sweep so
    // the wall/open buckets reflect one consistent ambient scene instead of mixing multiple turns together.
    constexpr uint8_t kStartupWallCalibrationFrontSpinTurnCount = 1U;
    // [Medium] Angular spacing between stored front-calibration samples during the startup front sweep. Reduce for denser
    // coverage if the sweep still leaves sparse buckets; increase if startup time or storage becomes tight.
    constexpr float kStartupWallCalibrationFrontSpinCaptureStepRad = 2.0f * DEG_TO_RAD_F;
    // [Medium] Maximum stored samples per sensor bucket for the startup front sweep. This only bounds retained samples;
    // the sweep itself still completes even if the buffers fill.
    constexpr uint16_t kStartupWallCalibrationFrontSpinMaxSamples = 256U;
    // [Medium] Headings within this angle of north are treated as the open-start-cell front scene during the startup
    // sweep. Keep this comfortably inside the true opening so wall/post geometry does not contaminate the baseline.
    constexpr float kStartupWallCalibrationFrontNorthOpenHalfWidthRad = 25.0f * DEG_TO_RAD_F;
    // [Medium] Treat only the east-of-north arc that sees the known east wall of cell (0,1) as the front wall-reference
    // bucket. West-of-north headings are ignored because that scene is not guaranteed by the maze topology.
    constexpr float kStartupWallCalibrationFrontWallMinEastOfNorthRad = 30.0f * DEG_TO_RAD_F;
    constexpr float kStartupWallCalibrationFrontWallMaxEastOfNorthRad = 90.0f * DEG_TO_RAD_F;
    // [Medium] Front startup sweep should be materially slower than the normal calibration turns so the front sensors
    // sample a cleaner angular sweep through the start-cell wall geometry.
    constexpr float kStartupWallCalibrationFrontSweepMaxOmegaRadps = 2.5f;
    constexpr float kStartupWallCalibrationFrontSweepAccelRadps2 = 12.0f;
    constexpr uint8_t kStartupWallCalibrationFrontSweepMatchedWallSampleCount = 8U;
    constexpr uint8_t kStartupWallCalibrationFrontSweepMatchedWallMinSamples = 4U;
    constexpr float kStartupWallCalibrationFrontSweepMatchedWallMaxDistanceErrorM = 0.020f;
    // [Medium] Vehicle is the shared source for the mission-start wall-contact geometry.
    constexpr float kRobotFrontWallContactOffsetM = MazeMap::Vehicle::GetPhysicalModel().frontWallContactOffsetM;
    constexpr float kRobotWidthM = MazeMap::Vehicle::GetPhysicalModel().widthM;
    constexpr float kRobotHalfWidthM = 0.5f * kRobotWidthM;
    constexpr float kRobotLengthM = MazeMap::Vehicle::GetPhysicalModel().lengthM;
    // [Medium] Measured rear-to-origin distance when the chassis is backed squarely against a wall.
    constexpr float kRobotRearWallContactOffsetM = kRobotLengthM - kRobotFrontWallContactOffsetM;
    // [Medium] Measured rear-to-origin distance when the chassis is backed squarely against a wall at mission start.
    constexpr float kMissionStartRearWallStandoffM = kRobotRearWallContactOffsetM;
    // [Low] Actual mission-start center y when the robot is backed against the south inner wall face of a 12 mm wall.
    constexpr float kMissionStartRearWallInsetM = (0.5f * kMazeWallThicknessM) + kMissionStartRearWallStandoffM;
    // [Medium] Measured center-to-wall distance when the chassis is gently touched squarely against a wall. Increase if
    // the robot stops farther from the wall than assumed; decrease if touch-off leaves the pose too deep into the wall.
    constexpr float kWallTouchContactStandoffM = kRobotFrontWallContactOffsetM;
    // [Low] Minimum clearance maintained against non-target walls during startup calibration motion planning.
    constexpr float kWallCalibrationWallClearanceM = 0.003f;
    // [Medium] Open-loop drive command used during the early portion of wall touch-off. Increase only if the robot
    // fails to reach the wall in time; decrease if the approach is still too harsh before the slow final section.
    constexpr float kWallTouchDriveCommand = 0.26f;
    // [Medium] Reduced open-loop drive command used over the last few centimeters of a predicted wall-touch approach.
    // Lower it if impact still carries through without latching; raise it if the robot hesitates and never finishes.
    constexpr float kWallTouchFinalApproachDriveCommand = 0.12f;
    // [Medium] Distance ahead of the predicted first-contact window where wall touch-off should drop into the slower
    // final approach command so contact happens at a speed the encoders and velocity watchdog can actually observe.
    constexpr float kWallTouchFinalApproachWindowM = 0.060f;
    // [Medium] Maximum encoder-observed wheel speed allowed during the wall-touch approach. Keep this low enough that
    // a real strike shows up as a velocity collapse instead of hiding behind wheelspin or rebound.
    constexpr float kWallTouchMaxApproachEncoderSpeedMps = 0.25f;
    // [Medium] Maximum encoder-observed wheel speed allowed once the robot is already in the wall-touch seating/skid
    // phase. This remains looser than the approach cap because the chassis is already against the wall.
    constexpr float kWallTouchMaxSeatEncoderSpeedMps = 0.40f;
    // [Medium] Minimum approach distance before wall contact is allowed to latch. Increase if the robot sometimes latches
    // before reaching the wall; decrease if the touch-off starts too close to the wall in practice.
    constexpr float kWallTouchMinApproachDistanceM = 0.030f;
    // [Medium] Allowed shortfall between the geometry-predicted wall-touch travel and the first accepted contact latch.
    // Keep this small enough to reject glancing hits, but not so small that a real first-corner touch cannot settle in.
    constexpr float kWallTouchExpectedTravelSlackM = 0.008f;
    // [Medium] Minimum time spent actively driving into the wall before the short-approach latch can trigger. Raise it
    // if false positives appear before wall contact; lower it if real short bumps still push fruitlessly before stopping.
    constexpr uint16_t kWallTouchMinCommandTimeMs = 120U;
    // [Medium] If wall-touch encoder progress has not increased by this much for the full stall window after the
    // minimum command time, treat the chassis as having found a hard stop and move into the seating phase.
    constexpr float kWallTouchProgressStallDistanceM = 0.0005f;
    // [Medium] Progress-stall dwell used to recognize a real wall hit when the robot reaches the wall but does not meet
    // the very low speed latch threshold because it is still pushing into the wall.
    constexpr uint16_t kWallTouchProgressStallWindowMs = 150U;
    // [High] Contact indicators must agree continuously for this long before the touch-off may leave the contact-seek
    // state. This rejects brief bumps and false first-contact spikes.
    constexpr uint16_t kWallTouchContactConfirmationMs = 12U;
    // [High] Peak open-loop drive command used while forcing the chassis to seat on the wall after encoder stall is
    // detected. This intentionally allows full drive so the wall-touch can scrub into square on this chassis.
    constexpr float kWallTouchSeatRampMaxDriveCommand = 0.72f;
    // [Medium] Short ramp from the low-speed approach command up to the seating preload. This should be fast enough to
    // preserve contact continuity while still avoiding a full-step shove into the wall.
    constexpr uint16_t kWallTouchSeatRampMs = 30U;
    // [High] Initial seated dwell before any square-up success evaluation is allowed. This deliberately outlasts the
    // first compliance transient and false stick-slip latch.
    constexpr uint16_t kWallTouchInitialSeatDwellMs = 120U;
    // [High] Minimum wall-touch seating command that must be reached before resumed encoder motion is accepted as the
    // tires breaking free into the wall-scrub phase. This keeps a perfectly square hit from releasing on a soft bounce.
    constexpr float kWallTouchSeatReleaseMinDriveCommand = 0.58f;
    constexpr float kWallTouchSeatReleaseDistanceM = 0.003f;
    constexpr uint16_t kWallTouchSeatReleaseMinSkidMs = 100U;
    // [High] Half-cycle duration for the alternating square-up yaw dither. Keep it long enough that each polarity has
    // time to produce real chassis settling instead of only exciting wall/frame compliance.
    constexpr uint16_t kWallTouchSeatWiggleHalfPeriodMs = 80U;
    // [High] Blend duration used when the square-up dither swaps polarity. A soft reversal reduces rebound compared with
    // a hard sign step.
    constexpr uint16_t kWallTouchSeatWiggleBlendMs = 12U;
    // [High] Minimum time a single dither polarity must persist before the half-cycle metrics may be evaluated.
    constexpr uint16_t kWallTouchSeatWiggleMinimumBiasPhaseMs = 68U;
    // [High] Initial differential-drive fraction applied during the wall-touch square-up dither.
    constexpr float kWallTouchSeatWiggleTurnFraction = 0.16f;
    // [High] Increment applied to the square-up dither turn fraction when additional effort is needed.
    constexpr float kWallTouchSeatWiggleTurnFractionStep = 0.04f;
    // [High] Maximum differential-drive fraction allowed during the square-up dither.
    constexpr float kWallTouchSeatWiggleMaxTurnFraction = 0.28f;
    // [High] Minimum fraction of the requested forward seating command that must be retained on both wheels while
    // yaw-dithering. This keeps the dither from unloading one side enough to look like a back-off.
    constexpr float kWallTouchSeatWiggleRetainedForwardFraction = 0.85f;
    // [High] Minimum number of full alternating dither cycles that must complete before success becomes eligible.
    constexpr uint8_t kWallTouchSeatMinimumFullCycles = 3U;
    // [High] Number of consecutive full cycles that must satisfy the square-up metrics before the primitive may release.
    constexpr uint8_t kWallTouchSeatRequiredGoodFullCycles = 2U;
    // [High] Minimum confirmed-contact time required before the touch-off may declare success.
    constexpr uint16_t kWallTouchMinimumConfirmedContactMs = 450U;
    // [High] Hard timeout for the square-up dither after contact has been confirmed.
    constexpr uint16_t kWallTouchSquareUpTimeoutMs = 1500U;
    // [Medium] Post-square seated hold where the estimator reset should be applied.
    constexpr uint16_t kWallTouchPostSquareHoldMs = 100U;
    // [Medium] Time used to ramp the seating preload down during release.
    constexpr uint16_t kWallTouchReleaseRampMs = 30U;
    // [Medium] Reverse-torque overlap during release so the wall-touch does not rebound when the seating preload unloads.
    constexpr uint16_t kWallTouchReleaseReverseOverlapMs = 15U;
    // [Medium] Reverse drive command reached at the end of the controlled release.
    constexpr float kWallTouchReleaseReverseDriveCommand = 0.16f;
    // [Medium] Minimum reverse clearance that must be established before the next maneuver may begin.
    constexpr float kWallTouchFrontClearanceDistanceM = 0.008f;
    // [Medium] Reverse speed used while clearing the wall after the controlled release.
    constexpr float kWallTouchReverseSpeedMps = 0.08f;
    // [Medium] Angular-command clamp used by the reverse yaw hold after the estimator reset.
    constexpr float kWallTouchReverseMaxAngularCommandRadps = 2.0f;
    // [Medium] Front-sensor asymmetry threshold treated as square at the end of a full dither cycle.
    constexpr float kWallTouchSquareFrontSkewThresholdM = 0.0025f;
    // [Medium] Residual yaw-rate threshold required at the end of each dither half-cycle.
    constexpr float kWallTouchSquareResidualYawRateThresholdRadps = 0.05f;
    // [Medium] Net yaw change threshold allowed across a full alternating dither cycle.
    constexpr float kWallTouchSquareNetYawChangeThresholdRad = 1.0f * DEG_TO_RAD_F;
    // [Low] Improvement floor used when deciding whether the square-up trend has saturated.
    constexpr float kWallTouchSquareImprovementSaturationThresholdM = 0.0005f;
    // [Low] Base maximum travel allowed while searching for a wall during touch-off. Nearby one-cell touches clamp to
    // this clear-span budget, while longer known-wall moves expand from the geometry-predicted travel plus slack.
    const float kWallTouchBaseMaxApproachDistanceM = kCellClearSpanM;
    constexpr auto kMissionRuntimeAccelFilterFreq = MazeMap::Vehicle::ImuBackLeft::ACCEL_FILTER_FREQ::FRAC_1_020;
    // [Low] Minimum south-wall clearance required before the startup routine may rotate back to the mission heading.
    // Increase only if chassis geometry changes and the rear still clips the wall during that recovery turn.
    constexpr float kMissionStartTurnClearanceM = 0.026f;
    // [Low] Search-mode pause when the goal is first reached. Increase if you need a more obvious confirmation;
    // decrease if you want the mapper to resume faster after goal detection.
    constexpr uint16_t kGoalPauseMs = 2000U;
    // [Low] Mandatory stop at the end of each speed run before driving back to the start. Increase for more margin
    // or judge visibility; decrease only if race rules and stopping accuracy allow a shorter dwell.
    constexpr uint16_t kSpeedRunFinishPauseMs = 3000U;
    // [Medium] Active-high vacuum-fan duty during each racing run, including the finish hold before returning to
    // start. Raise only if additional downforce is worth the power draw and thermal cost; lower if traction is already
    // sufficient or the fan system runs too hot.
    constexpr float kRacingFanDutyCycle = 0.80f;
    // [Medium] Soft-start time for the vacuum fan in diagnostic and racing modes. Increase if the drivetrain or power
    // rail reacts poorly to a full-step fan start; decrease if you need full downforce sooner.
    constexpr uint16_t kRacingFanRampMs = 2000U;
    // [Low] Service jumper pin A for the between-run tire-cleaning workflow. Change only if these pins conflict
    // with hardware; move both service-jumper pins together.
    constexpr uint8_t kInterRunServicePinA = 34U;
    // [Low] Service jumper pin B for the between-run tire-cleaning workflow. Change only with pin A.
    constexpr uint8_t kInterRunServicePinB = 35U;
    // [Low] Poll period while waiting for the service jumper sequence. Lower for faster response to the operator;
    // raise slightly if you want less busy-wait activity while the robot is idle.
    constexpr uint16_t kInterRunServicePollMs = 20U;

    // [High] Straight-line speed cap during exploration and return-to-start mapping. Raise it to shorten mapping once
    // wall sensing stays reliable at speed; lower it if exploration overshoots cells or loses corridor control.
    constexpr float kSearchMaxSpeedMps = 0.50f;
    // [High] Forward acceleration for mapping straights. Raise it to reduce ramp time; lower it if launches cause
    // wheelspin, pitch, or noisy wall readings right after starts.
    constexpr float kSearchAccelMps2 = 2.50f;
    // [High] Braking deceleration for mapping straights. Raise it if the robot overruns cell centers; lower it if
    // stops become unstable or the tires slide when entering observations and turns.
    constexpr float kSearchDecelMps2 = 2.50f;
    // [High] Executed in-place turn speed cap during search and homing. Raise it for faster alignment when turns are
    // clean; lower it if mapping turns overshoot, chatter, or scrub the tires.
    constexpr float kSearchTurnMaxOmegaRadps = 8.5f;
    // [High] Executed in-place turn acceleration during search and homing. Raise it for snappier turn entry; lower it
    // if the robot jerks into turns or becomes harder to stop on heading.
    constexpr float kSearchTurnAccelRadps2 = 100.0f;
    // [High] Mapping-mode lateral acceleration limit with the fan off. This preserves the old search cornering
    // envelope implied by the search speed cap and executed in-place turn rate cap, without reintroducing a
    // separate smooth-turn command clamp.
    constexpr float kSearchMaxLateralAccelerationMps2 = kSearchMaxSpeedMps * kSearchTurnMaxOmegaRadps;
    // [High] Lowest sustained forward speed observed in the latest diagnostic forward sweep. This came from the
    // `forward_010` hold segment and anchors the minimum nonzero cruise speed used during mapping.
    constexpr float kObservedDiagnosticMinimumSustainableSpeedMps = 0.052f;
    // [High] Minimum nonzero cruise speed allowed during mapping. This is set to 1.3x the observed minimum
    // sustainable diagnostic speed so the robot stays out of the weak low-speed regime.
    constexpr float kMinimumAllowedCruiseSpeedMps = 0.0676f;

    // [High] Global multiplier applied to the speed-run vehicle limits. Increase it carefully toward the chassis limit
    // after logging shows clean tracking; decrease it immediately after crashes, clipping walls, or late braking.
    constexpr float kSpeedRunScale = 0.90f;

    // [High] Desired side-wall spacing when centered in a straight corridor. Adjust from logs taken while the robot is
    // visibly centered: increase if it hugs the right wall, decrease if it hugs the left wall.
    constexpr float kExpectedSideWallDistanceM = 0.049f;
    // [High] Front-wall detection threshold fallback for latching "wall present". The front sensors now derive their
    // preferred latch point from the calibrated differential-light curve and the half-cell-into-adjacent geometry.
    // This fixed value is retained only as a fallback if front calibration data is unavailable.
    constexpr float kFrontWallOnThresholdM = 0.110f;
    // [High] Front-wall release threshold fallback. See the ON-threshold note above.
    constexpr float kFrontWallOffThresholdM = 0.130f;
    // [High] Additional release margin used when converting the derived front-wall latch point into a hysteresis band.
    constexpr float kFrontWallReleaseHysteresisM = kFrontWallOffThresholdM - kFrontWallOnThresholdM;
    // [High] Scale applied to front measured-signal thresholds derived from the startup fit. The log-amp front sensors
    // read weaker in the far tail than the extrapolated model predicts, so lower values latch sooner on real walls.
    constexpr float kFrontWallMeasuredSignalThresholdScale = 0.70f;
    // [High] Front-wall detection is normalized to the measured span from the north-facing start-scene baseline to the
    // weakest in-place wall-facing front calibration band captured at start-center. This fraction is tuned against the
    // real 9-sample rolling observation vote so it remains conservative under maze-to-maze ambient changes.
    constexpr float kFrontWallSignalLatchFractionOfCalibratedSpan = 0.22f;
    // [High] Release fraction for the normalized front-wall span. Keep it below the latch fraction to preserve
    // hysteresis while still releasing once most of the calibrated wall rise disappears.
    constexpr float kFrontWallSignalReleaseFractionOfCalibratedSpan = 0.15f;
    // [High] Minimum number of rolling-observation samples that must land inside the persisted front characterization
    // curve before the mission front detector will trust the curve fit.
    constexpr uint8_t kFrontWallCharacterizationMinMatchSamples = 5U;
    // [High] Minimum fitted wall-template amplitude, relative to the stored dark-room template, required before the
    // persisted front curve may assert a wall during rolling observation.
    constexpr float kFrontWallCharacterizationMinScale = 0.20f;
    // [High] Minimum normalized correlation required for the rolling front observation to match the persisted wall
    // approach curve. Raise this to reject more false positives; lower it to tolerate more shape distortion.
    constexpr float kFrontWallCharacterizationMinCorrelation = 0.85f;
    // [High] Maximum residual energy, normalized by measured front-rise energy, allowed for a persisted-curve wall
    // match. Lower this for stricter shape agreement.
    constexpr float kFrontWallCharacterizationMaxRelativeResidual = 0.50f;
    // [High] Side-wall measured-signal latch threshold in normalized corrected-signal units. The side thresholds now
    // use Decimus-style one-point run normalization against the in-cell wall reference rather than an inverse-square
    // projection of the live signal itself.
    constexpr float kSideWallMeasuredSignalLatchThreshold = 0.45f;
    // [High] Side-wall measured-signal release threshold in normalized corrected-signal units.
    constexpr float kSideWallMeasuredSignalReleaseThreshold = 0.40f;
    // [High] Only trust side-wall detections while the receiver is aimed at the center third of a wall segment.
    constexpr float kSideWallSegmentCenterFraction = (1.0f / 3.0f);
    // [High] Reset side-wall temporal state once the side receivers are one-quarter of the way into a new cell so
    // the center-third observation window is measured without carry-over from the previous cell.
    constexpr float kSideWallStateResetCellEntryFraction = 0.25f;
    // WARNING: Search-mode mapping observation on cautious straights is a single constant-velocity traversal.
    // Do not introduce parallel mapping traversal mechanisms, extra observation passes, stop-and-peek detours, or any
    // other motion-shape changes here. If mapping quality needs work, change only the observation timing, target
    // region, or vote logic while the straight itself remains constant velocity through the cell.
    constexpr uint8_t kSearchRollingObservationSampleCount = 9U;
    constexpr uint8_t kSearchRollingObservationMajorityCount =
        static_cast<uint8_t>((kSearchRollingObservationSampleCount / 2U) + 1U);
    // [High] Distance-domain side-wall threshold fallback geometry. This remains the "wall versus next-cell wall"
    // ratio used only when the measured-signal threshold path is unavailable.
    constexpr float kSideWallDistanceLatchFractionOfCalibration = (1.0f / 6.0f);
    // [High] Distance-domain side-wall release fallback fraction.
    constexpr float kSideWallDistanceReleaseFractionOfCalibration = (1.0f / 8.0f);
    // [High] Side-wall detection threshold fallback for latching "wall present". The preferred threshold is now derived
    // from the calibrated side-wall distance and the linear inverse-square signal model.
    constexpr float kSideWallOnThresholdM = 0.120f;
    // [High] Side-wall release threshold fallback. See the ON-threshold note above.
    constexpr float kSideWallOffThresholdM = 0.140f;
    // [High] Number of raw samples averaged for each startup wall-calibration capture. Raise it if calibration still
    // jitters; lower it only if startup time matters more than wall-fit quality.
    constexpr uint16_t kWallCalibrationAverageSampleCount = 50U;
    // [High] Robust calibration-band half-width in scaled-MAD units. Raise it if startup calibration samples still
    // contain outliers; lower it only if the reference bands are obviously over-conservative.
    constexpr float kWallCalibrationScaledMadMultiplier = 3.0f;
    // [High] Runtime wall-detection averaging window in control-loop cycles. Raise it if false negatives remain under
    // noise; lower it only if detection latency becomes a real control problem.
    constexpr uint8_t kWallDetectionAverageWindowCycles = 10U;
    // [High] Side-wall signal delta, expressed as a fraction of the calibrated latch threshold, that indicates the
    // receiver is crossing an opening or post edge instead of seeing a stable corridor wall.
    constexpr float kSideWallTransitionSignalFractionOfLatch = 0.60f;
    // [High] Fraction of the latch threshold below which a gated sample is considered a confident "open" miss for the
    // short-horizon map evidence accumulator.
    constexpr float kWallMapMissSignalFractionOfLatch = 0.35f;
    // [High] Per-sample weight for confident wall-hit classifications inside a map decision window.
    constexpr float kWallMapEvidenceHitWeight = 0.55f;
    // [High] Per-sample weight for confident wall-miss classifications inside a map decision window.
    constexpr float kWallMapEvidenceMissWeight = 0.55f;
    // [Medium] Extra miss impulse applied when a strong side-wall transition is observed during a map decision window.
    constexpr float kWallMapEvidenceTransitionMissWeight = 0.35f;
    // [Medium] Amount by which unknown samples pull the short-horizon evidence accumulator back toward zero.
    constexpr float kWallMapEvidenceUnknownDecay = 0.08f;
    // [High] Minimum evidence magnitude required before a gated decision window is allowed to touch the wall-belief
    // map.
    constexpr float kWallMapEvidenceCommitThreshold = 1.00f;
    // [High] Log-odds increment applied to a confident wall-hit observation.
    constexpr float kWallBeliefHitLogOdds = 1.20f;
    // [High] Log-odds decrement applied to a confident wall-miss observation when the wall is still unknown or open.
    constexpr float kWallBeliefMissLogOdds = 1.20f;
    // [High] Reduced log-odds decrement applied to a contradictory wall-miss observation so one weak miss cannot erase
    // a previously confirmed wall.
    constexpr float kWallBeliefContradictoryMissLogOdds = 0.55f;
    // [High] Positive log-odds threshold for confirming a wall segment.
    constexpr float kWallBeliefSetThreshold = 1.00f;
    // [High] Negative log-odds threshold for confirming an opening.
    constexpr float kWallBeliefClearThreshold = -1.00f;
    // [Medium] Saturation limit applied to wall-segment log odds to prevent runaway certainty from redundant samples.
    constexpr float kWallBeliefSaturationMagnitude = 3.50f;

    // [High] Residual static trim on top of the motor-model wheel feedforward. Keep this at zero unless the shared
    // motor model still leaves a repeatable low-speed bias after launch assist has already done its job.
    constexpr float kWheelStaticFeedforward = 0.0f;
    // [High] Minimum motor command used only on the first control update of a move from rest. Raise it if launches still
    // stall against static friction; decrease it if the robot kicks too hard when starting a move.
    constexpr float kWheelRestLaunchDriveCommand = 0.30f;
    // [High] Maximum motor command the launch assist may reach if the encoders still report no wheel motion after the
    // initial launch floor. Raise it if starts still hang on static friction; decrease it if launches hit too hard.
    constexpr float kWheelRestLaunchMaxDriveCommand = 0.55f;
    // [Medium] Time for the launch assist to ramp from the initial floor to the max launch command while the encoders
    // still show the wheel at rest. Shorten it for a harder snap off the line; lengthen it for a softer launch.
    constexpr unsigned long kWheelRestLaunchRampMs = 250UL;
    // [Medium] Wheel-speed threshold for considering the robot stationary for the one-shot launch assist. Increase it if
    // encoder quantization keeps the assist from triggering at rest; decrease it if the assist retriggers after motion.
    constexpr float kWheelRestLaunchSpeedThresholdMps = 0.02f;
    // [Medium] Prior-drive threshold for treating the previous cycle as effectively stopped before applying the launch
    // assist. Increase it if very small commands should still count as "at rest"; decrease it if the assist retriggers.
    constexpr float kWheelRestLaunchDriveThreshold = 0.05f;
    // [High] Residual speed-proportional trim on top of the motor-model wheel feedforward. Keep this at zero unless
    // the physical motor model still leaves a repeatable speed-dependent bias under measured load.
    constexpr float kWheelVelocityFeedforward = 0.0f;
    // [Medium] Extra drive-command trim per wheel-target acceleration used only when the wheel is still chasing the
    // commanded speed. Raise it if mapping still feels lazy; lower it if mapping starts to snap too hard.
    constexpr float kWheelAccelerationResponseGainPerMps2 = 0.20f;
    // [Medium] Speed-delta window over which the acceleration-response trim fades out near target. Increase it if the
    // trim falls away too early; decrease it if the approach to target speed gets too punchy.
    constexpr float kWheelAccelerationResponseDeltaWindowMps = 0.08f;
    // [High] Wheel-speed proportional gain. Increase for tighter speed tracking; decrease if motor commands chatter
    // or the chassis oscillates in speed on straight segments.
    constexpr float kWheelVelocityKp = 1.10f;
    // [High] Wheel-speed integral gain. Increase if steady-state speed error remains under load; decrease if the loop
    // winds up and produces slow surging or long recovery after a stop or saturation.
    constexpr float kWheelVelocityKi = 1.50f;
    // [Medium] Clamp on the wheel-speed integrator. Increase only if the loop needs more integral authority to overcome
    // repeatable bias; decrease if recovery from saturation is sluggish or overshoots badly.
    constexpr float kWheelIntegralLimit = 0.25f;
    // [High] Straight-line heading proportional gain. Increase if the robot drifts off heading in open corridors;
    // decrease if it snakes left-right while trying to stay on course.
    constexpr float kStraightHeadingKp = 11.5f;
    // [High] Straight-line yaw damping. Increase if heading correction oscillates; decrease if heading correction feels
    // lazy and the robot lets errors build before responding.
    constexpr float kStraightYawD = 0.20f;
    // [High] Wall-centering gain used when side walls are available. Increase if the robot does not recenter in a
    // corridor; decrease if wall following hunts or bounces between walls.
    constexpr float kWallCenterGain = 135.0f;
    // [High] Wall-centering derivative gain used in mapping/search straights. Increase it for a more damped, less
    // oscillatory lateral response; decrease it if the wall-centering correction feels too reluctant to engage.
    constexpr float kWallCenterD = 3.0f;
    // [Medium] First-order filter time constant for the wall-centering derivative term. Increase it if side-sensor
    // jitter still leaks into steering; decrease it if centering feels too syrupy to settle cleanly.
    constexpr float kWallCenterDerivativeFilterTauSeconds = 0.030f;
    // [Medium] Maximum lateral closure that wall-centering may demand over one cell of travel. This is converted into
    // a curvature cap for the centering term only, so mapping stays disciplined instead of yanking sharply off line.
    constexpr float kWallCenterMaxClosurePerCellM = 0.020f;
    // [High] Diagonal wall-balance gain used on diagonal straight segments. This uses the live left/right signal
    // balance rather than orthogonal side-wall distance, matching the sensor-balance approach common on top-end mice.
    constexpr float kDiagonalWallCenterGain = 15.0f;
    // [Medium] Minimum side-sensor signal, normalized to the calibration wall sample, required before diagonal
    // centering will trust that side as a valid wall cue.
    constexpr float kDiagonalWallMinNormalizedSignal = kSideWallMeasuredSignalReleaseThreshold;
    // [Medium] Front-wall skew correction near the end of a straight. Increase if the robot stops angled at front walls;
    // decrease if it twitches too aggressively when approaching a front wall.
    constexpr float kFrontSkewGain = 12.0f;
    // [High] Arc heading proportional gain. Increase if speed-run arcs cut corners or drift wide; decrease if the
    // robot oscillates while trying to hold an arc.
    constexpr float kArcHeadingKp = 14.0f;
    // [High] Arc yaw damping. Increase if arc tracking oscillates in yaw; decrease if the robot lags behind the
    // desired curvature and feels overdamped through smooth turns.
    constexpr float kArcYawD = 0.08f;
    // [High] Smooth-turn yaw-rate proportional gain. Smooth maneuvers treat the prescribed yaw-rate trace as the
    // authority, so this closes the measured turn rate onto that trace without changing maneuver geometry.
    constexpr float kSmoothTurnYawRateKp = 1.10f;
    // [High] Smooth-turn yaw-rate derivative gain. This damps yaw-rate error directly so the robot follows the
    // maneuver's sample-by-sample turn-rate target rather than lagging wide through the corner.
    constexpr float kSmoothTurnYawRateKd = 0.0001f;
    // [High] In-place turn proportional gain. Increase if the robot consistently stops short of target heading;
    // decrease if it overshoots or rings at the end of turns.
    constexpr float kTurnHeadingKp = 7.5f;
    // [High] In-place turn damping. Increase if turn-stop oscillation remains; decrease if in-place turns feel too
    // sluggish to settle on target heading.
    constexpr float kTurnYawD = 0.12f;

    // [Medium] Position tolerance used to declare straight and arc profiles complete. Tighten it if stop error is too
    // large and the robot can settle cleanly; loosen it if profiles dither near the endpoint.
    constexpr float kDistanceToleranceM = 0.003f;
    // [Medium] Heading tolerance used to declare turns complete. Tighten it for stricter final heading accuracy; loosen
    // it if turns hunt around the target because the requested precision exceeds sensor/control quality.
    constexpr float kAngleToleranceRad = 0.75f * DEG_TO_RAD_F;
    // [Medium] Linear-speed threshold used to declare motion settled. Lower it if the robot is still rolling when a
    // profile completes; raise it if noisy velocity estimates keep motions from ever finishing.
    constexpr float kSpeedToleranceMps = 0.05f;
    // [Medium] Angular-speed threshold used to declare turns settled. Lower it if the robot is still rotating when turns
    // finish; raise it if gyro noise keeps the turn controller waiting too long.
    constexpr float kAngularSpeedToleranceRadps = 0.10f;
    // [Medium] Mapping/search in-place turns should settle more tightly than the generic default because they anchor
    // the maze solution. Keep this below one degree.
    constexpr float kMappingAngleToleranceRad = 0.50f * DEG_TO_RAD_F;
    // [Medium] Mapping/search in-place turns should also be nearly stopped before they declare completion.
    constexpr float kMappingAngularSpeedToleranceRadps = 0.03f;
    // [Medium] Smallest encoder distance change treated as real translation progress for the motion watchdog. Increase
    // if encoder quantization or noise causes false progress; decrease if low-speed moves stall without tripping it.
    constexpr float kEncoderProgressEpsilonM = 0.0015f;
    // [Medium] Minimum commanded translation speed that enables the encoder-progress watchdog. Increase if very slow
    // creeping maneuvers false-trigger; decrease if moderate-speed runaways are not caught quickly enough.
    constexpr float kEncoderStallCommandThresholdMps = 0.06f;
    // [Medium] Maximum time to allow a translation profile to command real motion without encoder progress. Increase if
    // launch latency is genuinely longer; decrease if you want faster containment when odometry is lost.
    constexpr unsigned long kEncoderStallTimeoutMs = 250UL;
    // [Medium] Minimum time a translation profile must spend commanding real motion before the encoder-progress
    // watchdog can trip. This should cover the launch-assist ramp so high-strung starts do not false-fault.
    constexpr unsigned long kEncoderStallStartupGraceMs = 250UL;
}
