#pragma once
// Private runtime core definitions shared by MazeMap application subsystems.

inline bool AppendStartupTrace(const char* line);

#if !defined(ARDUINO_TEENSY41)
namespace Pins
{
    constexpr uint8_t R_MotorA = 5;
    constexpr uint8_t R_MotorB = 6;
    constexpr uint8_t R_EncA = 7;
    constexpr uint8_t R_EncB = 8;
    constexpr uint8_t L_MotorA = 24;
    constexpr uint8_t L_MotorB = 25;
    constexpr uint8_t L_EncA = 2;
    constexpr uint8_t L_EncB = 3;
    constexpr uint8_t Fan_CTRL = 4;
    constexpr uint8_t IMU_INT_1A = 32;
    constexpr uint8_t IMU_INT_1B = 33;
    constexpr uint8_t LED_Ctrl_Forward_Right = 19;
    constexpr uint8_t LED_Ctrl_Forward_Left = 18;
    constexpr uint8_t LED_Ctrl_Side_Right = 17;
    constexpr uint8_t LED_Ctrl_Side_Left = 16;
}

namespace HardwareConfig
{
    constexpr uint32_t kFrontWallSensorSwitchSettleTime_us = 60U;
    constexpr uint32_t kSideWallSensorSwitchSettleTime_us = 30U;
}

inline bool SetupHardware()
{
    return true;
}
#endif

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
    constexpr float kGyroBiasUpdateMaxAbsRateRadps = 0.02f;
    // [Medium] Time to sit still before wall observations. Increase if wall sensors or chassis motion need longer to
    // settle after stopping; decrease if mapping feels sluggish and readings are already stable.
    constexpr uint16_t kObservationSettleMs = 25U;
    // [Medium] Continuous stationary dwell required before the startup wall-calibration routine may move. Increase if
    // the robot can still begin while being placed by hand; decrease only if startup latency matters more than this gate.
    constexpr uint16_t kMissionStartupStationaryHoldMs = 2000U;
    // [Medium] Maximum absolute chassis and wheel speed still counted as stationary for the startup dwell. Increase only
    // if encoder noise keeps resetting the timer; decrease if the robot can still creep while being treated as settled.
    constexpr float kMissionStartupStationarySpeedThresholdMps = 0.008f;
    // [Medium] Maximum absolute yaw rate still counted as stationary for the startup dwell. Keep this close to the gyro
    // bias-adaptation gate unless logs show harmless IMU noise is delaying mission start.
    constexpr float kMissionStartupStationaryMaxAbsYawRateRadps = 0.02f;
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
    constexpr float kSearchMaxSpeedMps = 0.40f;
    // [High] Forward acceleration for mapping straights. Raise it to reduce ramp time; lower it if launches cause
    // wheelspin, pitch, or noisy wall readings right after starts.
    constexpr float kSearchAccelMps2 = 0.30f;
    // [High] Braking deceleration for mapping straights. Raise it if the robot overruns cell centers; lower it if
    // stops become unstable or the tires slide when entering observations and turns.
    constexpr float kSearchDecelMps2 = 0.30f;
    // [High] Executed in-place turn speed cap during search and homing. Raise it for faster alignment when turns are
    // clean; lower it if mapping turns overshoot, chatter, or scrub the tires.
    constexpr float kSearchTurnMaxOmegaRadps = 8.5f;
    // [High] Executed in-place turn acceleration during search and homing. Raise it for snappier turn entry; lower it
    // if the robot jerks into turns or becomes harder to stop on heading.
    constexpr float kSearchTurnAccelRadps2 = 30.0f;
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
    constexpr float kMappingAngularSpeedToleranceRadps = 0.07f;
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

static_assert(
    (Config::kSearchRollingObservationSampleCount % 2U) == 1U,
    "Search rolling observation majority vote requires an odd sample count.");

namespace DiagnosticConfig
{
    // Likelihood tags for the diagnostic-only config:
    // [High] commonly adjusted to improve data quality or stress level during test sessions.
    // [Medium] sometimes adjusted as the diagnostic workflow matures.
    // [Low] usually fixed by wiring, safety limits, or operator workflow.

    // [Low] Dedicated strap pins for the full diagnostic battery mode. Change them only if the wiring harness needs
    // different spare pins; keep them distinct from the service-jumper and maneuver-test jumpers.
    constexpr uint8_t kModeSelectPinA = 30U;
    // [Low] Dedicated strap pins for the full diagnostic battery mode. Change with pin A.
    constexpr uint8_t kModeSelectPinB = 31U;
    // [Medium] Diagnostic control/log period. Shorten it only if SD logging, sensor reads, and control math still meet
    // the deadline; lengthen it if you see dropped samples or write stalls.
    constexpr unsigned long kControlPeriodUs = 500UL;
    // [Medium] Initial stationary settle before the diagnostic battery starts. Increase if the robot is still moving
    // from placement when logging begins; decrease if startup idle time is unnecessary.
    constexpr uint16_t kStartupSettleMs = 250U;
    // [Medium] Idle capture window used for baseline noise and bias logging. Increase if you want better stationary
    // statistics; decrease if the diagnostic routine spends too long collecting idle data.
    constexpr uint16_t kBaselineHoldMs = 2500U;
    // [Medium] Pause between diagnostic phases. This is set from measured post-motion settling data so the robot has
    // time to re-enter a genuinely stationary state before the next maneuver starts.
    constexpr uint16_t kInterTestHoldMs = 350U;
    // [Medium] SD flush cadence during diagnostics. Decrease it if you want less data loss risk on power interruption;
    // increase it if flush overhead limits logging throughput.
    constexpr uint32_t kLogFlushPeriodMs = 250U;
    // [Low] Half-width of the allowed diagnostic operating square. Increase only if the test area is larger and you
    // intentionally want wider trajectories; decrease for a tighter safety fence around the start pose.
    constexpr float kBoundaryHalfSpanM = 0.34f;
    // [High] Short straight distance used in the diagnostic battery. Increase for more steady-state straight data;
    // decrease if you need to stay well inside the safety box or focus on launch/braking behavior.
    constexpr float kShortStraightDistanceM = 0.18f;
    // [High] Long straight distance used in the diagnostic battery. Increase if you need more data at higher speed;
    // decrease if the robot approaches the boundary or cannot complete the profile cleanly.
    constexpr float kLongStraightDistanceM = 0.27f;
    // [High] Side length of the diagnostic square-loop test. Increase for more coupled straight/turn data; decrease
    // if the loop approaches the boundary or you want to isolate turn behavior.
    constexpr float kSquareLegDistanceM = 0.15f;
    // [High] Arc length for each half-circle arc test. Increase for longer arc-tracking data; decrease if the circle
    // grows too large for the available floor space or you want tighter curvature.
    constexpr float kArcHalfCircleDistanceM = 0.20f;
    // [High] Cruise speed for conservative diagnostic straights and square loops. Increase once low-speed data is
    // boring and stable; decrease if you need cleaner low-dynamics identification data.
    constexpr float kSlowStraightSpeedMps = 0.25f;
    // [High] Mid-speed cruise used by the diagnostic circle sweep. Increase once the slower circle data is clean;
    // decrease if the circle comparison already exposes the smooth-turn mismatch you need to correct.
    constexpr float kCircleMediumSpeedMps = 0.35f;
    // [High] Cruise speed for the longer diagnostic straight test. Increase to probe higher-speed behavior; decrease
    // if braking distance, tracking error, or the safety boundary becomes problematic.
    constexpr float kFastStraightSpeedMps = 0.45f;
    // [High] Straight-profile acceleration during diagnostics. Increase if you want stronger feedforward/traction data;
    // decrease if launches spin the tires or make the test less repeatable.
    constexpr float kStraightAccelMps2 = 1.50f;
    // [High] Straight-profile braking during diagnostics. Increase if you want more braking data or tighter stops;
    // decrease if braking becomes noisy, slides the tires, or destabilizes the chassis.
    constexpr float kStraightDecelMps2 = 1.80f;
    // [High] Turn-rate limit for diagnostic turn sweeps. Increase to stress higher-yaw-rate behavior; decrease if the
    // turn data is dominated by overshoot or wheel scrub instead of clean rotational dynamics.
    constexpr float kTurnMaxOmegaRadps = 9.0f;
    // [High] Turn acceleration for diagnostic turn sweeps. Increase to excite sharper turn entry/exit dynamics;
    // decrease if the robot cannot reach those ramps repeatably without slipping.
    constexpr float kTurnAccelRadps2 = 35.0f;
    // Diagnostics now reuse the same wheel profile as mission and startup calibration so any turn or straight behavior
    // seen here matches the controller the robot will use elsewhere.
    constexpr float kDiagnosticWheelVelocityKpScale = Config::kNominalWheelVelocityKpScale;
    constexpr float kDiagnosticWheelVelocityKiScale = Config::kNominalWheelVelocityKiScale;
    constexpr float kDiagnosticWheelIntegralLimitScale = Config::kNominalWheelIntegralLimitScale;
    // [High] Lowest raw drive command included in the kickoff sweep. Decrease to probe weaker launches; increase if
    // the robot clearly does not move at very low values and you want a shorter sweep.
    constexpr float kKickoffSweepMinDriveCommand = 0.15f;
    // [High] Highest raw drive command included in the kickoff sweep. Increase to probe more aggressive launches;
    // decrease if the sweep already reaches reliable breakaway or the robot moves too far per sample.
    constexpr float kKickoffSweepMaxDriveCommand = 0.70f;
    // [Medium] Step size between kickoff sweep commands. Decrease for finer resolution; increase for a faster sweep.
    constexpr float kKickoffSweepStepDriveCommand = 0.05f;
    // [High] Pulse length for each kickoff sample. Increase if static friction needs a longer shove to reveal the
    // threshold; decrease if the robot moves too far before the recovery segment.
    constexpr uint16_t kKickoffSweepPulseMs = 120U;
    // [Medium] Minimum distance that counts as "moved" during the kickoff sweep. Increase if encoder noise causes false
    // positives; decrease if real breakaway moves are being missed.
    constexpr float kKickoffSweepMoveThresholdM = 0.004f;
    // [Medium] Minimum peak speed that counts as "moved" during the kickoff sweep. Increase if noise spikes look like
    // launches; decrease if the robot creeps but does not cross the distance threshold.
    constexpr float kKickoffSweepMoveThresholdMps = 0.03f;
    // [High] Raw kickoff command used ahead of each forward-hold sample. Raise it if the forward sweep still stalls at
    // the start; lower it if the kickoff itself dominates the measurement too much.
    constexpr float kForwardSweepKickoffDriveCommand = 0.35f;
    // [Medium] Kickoff pulse length used before each forward-hold sample. Increase if the robot needs more time to
    // break away; decrease if the kickoff contributes too much of the measured travel.
    constexpr uint16_t kForwardSweepKickoffMs = 80U;
    // [High] Lowest raw hold command included in the forward sweep. Decrease to probe weaker sustaining commands;
    // increase if very low commands are obviously useless.
    constexpr float kForwardSweepMinDriveCommand = 0.10f;
    // [High] Highest raw hold command included in the forward sweep. Increase to extend the command-to-speed map;
    // decrease if the sweep gets too aggressive for the available space.
    constexpr float kForwardSweepMaxDriveCommand = 0.40f;
    // [Medium] Step size between forward sweep commands. Decrease for finer resolution; increase for a shorter test.
    constexpr float kForwardSweepStepDriveCommand = 0.05f;
    // [High] Hold time for each forward sweep sample after the kickoff pulse. Increase for better steady-speed data;
    // decrease if the robot travels too far before the recovery segment.
    constexpr uint16_t kForwardSweepHoldMs = 220U;
    // [Medium] Average hold speed that counts as "carried" during the forward sweep. Increase if tiny creeping speeds
    // are not useful; decrease if the desired sustaining command is very gentle.
    constexpr float kForwardSweepCarryThresholdMps = 0.05f;
    // [Medium] Distance accumulated during the hold segment that counts as a meaningful carry. Increase to ignore tiny
    // nudges; decrease if low-speed sustained motion is the target.
    constexpr float kForwardSweepCarryThresholdM = 0.010f;
    // [Medium] Brake-and-settle window after each characterization sample. This now tracks the measured diagnostic
    // settling time instead of the earlier optimistic estimate.
    constexpr uint16_t kCharacterizationSettleMs = 350U;
    // [Medium] Distance reserve kept inside the diagnostic boundary while open-loop characterization probes are still
    // driving. Increase if probes still reach the edge before braking; decrease only if the sweep no longer reaches
    // useful speeds in the available space.
    constexpr float kCharacterizationBoundaryReserveM = 0.14f;
    // [Medium] Recovery cruise speed used to bring the robot back to the start between characterization samples after
    // a turnaround. Increase if recovery legs are too slow; decrease if the recovery path overshoots or turns get
    // sloppy between samples.
    constexpr float kCharacterizationRecoverySpeedMps = 0.18f;
}

namespace AuxMeasurementConfig
{
    enum class Routine : uint8_t
    {
        FanStaticSurvey = 0U,
        TurningTractionSweep = 1U,
        CorridorRepeatabilitySweep = 2U,
        PositionAccuracyAudit = 3U,
    };

    // Likelihood tags for the auxiliary one-off measurement mode:
    // [High] commonly adjusted when instrumenting a new internal measurement routine.
    // [Medium] sometimes adjusted to trade logging density against run time.
    // [Low] usually fixed by spare-pin availability and operator workflow.

    // [Low] Short pins 28 and 29 at startup to enter auxiliary measurement mode. These pins are selector-only; they
    // are not sampled as measurement inputs after boot.
    constexpr uint8_t kModeSelectPinA = 28U;
    // [Low] Dedicated strap partner for the auxiliary pin-measurement mode. Change with pin A.
    constexpr uint8_t kModeSelectPinB = 29U;
    // [Medium] Selected one-off routine. Change this when you need a different internal measurement script without
    // disturbing the main characterization battery.
    constexpr Routine kRoutine = Routine::PositionAccuracyAudit;
    // [Medium] Control/log period for auxiliary capture. Shorten it only if SD logging still keeps up and the extra
    // temporal resolution is actually useful.
    constexpr unsigned long kControlPeriodUs = 1000UL;
    // [Medium] Initial stationary settle before the first auxiliary phase. Increase if the chassis or fan still rings
    // after placement; decrease if the idle lead-in is longer than needed.
    constexpr uint16_t kStartupSettleMs = 500U;
    // [Medium] Fan-off idle capture at the start of the default auxiliary routine. Increase if you need a stronger
    // baseline for vibration or bias comparisons.
    constexpr uint16_t kBaselineHoldMs = 2000U;
    // [Medium] Fan-on idle capture used by the default auxiliary routine. Increase if you need more statistics on
    // fan-induced vibration, gyro shift, or wall-sensor noise.
    constexpr uint16_t kFanHoldMs = 3000U;
    // [Medium] Fan-off recovery capture after the fan phase. Increase if you want to see how quickly the sensors and
    // chassis settle back to baseline.
    constexpr uint16_t kRecoveryHoldMs = 2000U;
    // [Medium] Direction used by the turning-traction sweep. Flip this only if you specifically want the opposite
    // circle direction for a one-off asymmetry check.
    constexpr bool kTurningTractionSweepClockwise = true;
    // [Medium] Circle radius commanded by the turning-traction sweep. Increase if you need a gentler sweep with more
    // floor margin; decrease only if you have verified the smaller radius still matches your smooth-turn geometry.
    constexpr float kTurningTractionSweepRadiusM = 0.180f;
    // [Medium] Initial speed for the turning-traction sweep. Increase only if the early portion of the sweep is
    // uninformative and well below any plausible traction threshold.
    constexpr float kTurningTractionSweepStartSpeedMps = 0.60f;
    // [Medium] Linear acceleration used to ramp the turning-traction circle. Increase if the sweep takes too long;
    // decrease if longitudinal acceleration is contaminating the lateral-traction measurement.
    constexpr float kTurningTractionSweepAccelMps2 = 0.03f;
    // [Medium] Optional commanded-speed ceiling for the turning-traction sweep. Set to `0` to remove the software
    // speed limit and let the run continue toward the hardware boundary instead.
    constexpr float kTurningTractionSweepMaxSpeedMps = 0.0f;
    // [Medium] Fan-on stationary lead-in before the traction sweep. Keep this at or above the mission fan ramp time so
    // the circle begins with steady downforce instead of during fan spin-up.
    constexpr uint16_t kTurningTractionSweepFanSettleMs = 2500U;
    // [Medium] Open-loop launch window used to preserve the commanded circle radius while the drivetrain breaks static
    // friction. Increase if the sweep still begins nearly straight; decrease only if the launch segment feels too long.
    constexpr uint16_t kTurningTractionLaunchMs = 250U;
    // [Medium] Optional angular-command ceiling for the turning-traction sweep. Set to `0` to remove the software
    // yaw-rate cap and let actuator saturation, slip detection, and timeout define the limit instead.
    constexpr float kTurningTractionSweepMaxAngularCommandRadps = 0.0f;
    // [Medium] Wheel-speed proportional gain scale used only by the turning-traction sweep. Raise it if the robot
    // still settles into a much wider arc than commanded; lower it if the wheel commands become noisy or oscillatory.
    constexpr float kTurningTractionWheelVelocityKpScale = DiagnosticConfig::kDiagnosticWheelVelocityKpScale;
    // [Medium] Wheel-speed integral gain scale used only by the turning-traction sweep. Raise it if steady-state
    // curvature still stays biased after the proportional term is no longer enough.
    constexpr float kTurningTractionWheelVelocityKiScale = DiagnosticConfig::kDiagnosticWheelVelocityKiScale;
    // [Medium] Wheel integrator limit scale used only by the turning-traction sweep. Raise it only if the sweep still
    // runs out of corrective authority during the sustained circle.
    constexpr float kTurningTractionWheelIntegralLimitScale = DiagnosticConfig::kDiagnosticWheelIntegralLimitScale;
    // [Medium] Minimum measured speed required before the sweep is allowed to conclude that straight-line acceleration
    // has plateaued and it should start tightening the turn instead.
    constexpr float kTurningTractionPlateauMinSpeedMps = 0.70f;
    // [Medium] Speed gain required within the plateau window to stay in the speed-ramp stage once the outer wheel is
    // already near saturation. If the measured speed does not improve by at least this much, the sweep starts
    // increasing curvature to force a traction breakaway.
    constexpr float kTurningTractionPlateauDeltaMps = 0.02f;
    // [Medium] Observation window used to decide whether the sweep has stopped making forward-speed progress after the
    // outer wheel is already near saturation.
    constexpr uint16_t kTurningTractionPlateauWindowMs = 2500U;
    // [Medium] Outer-wheel command magnitude that counts as actuator-limited for the plateau detector.
    constexpr float kTurningTractionActuatorCeilingCommand = 0.92f;
    // [Medium] Curvature ramp applied after the speed stage plateaus. Raise it if the tightened-turn stage still takes
    // too long to reach slip; lower it if the transition becomes too abrupt to interpret cleanly.
    constexpr float kTurningTractionCurvatureRampMInvPerSec = 2.0f;
    // [Medium] Minimum speed before the slip detector is allowed to trip. Increase if low-speed sensor noise produces
    // false positives; decrease only if traction is breaking much earlier than expected.
    constexpr float kTurningTractionSlipMinSpeedMps = 0.25f;
    // [Medium] Minimum encoder-derived lateral acceleration before the slip detector is allowed to trip. Increase if
    // low-g data is noisy; decrease if traction loss begins sooner on low-friction surfaces.
    constexpr float kTurningTractionSlipMinLatAccelMps2 = 1.00f;
    // [Medium] Minimum gyro-to-encoder yaw coherence expected while the robot is still gripping. Lower this only if
    // clean runs show a persistent model mismatch; raise it if slip is being detected too late.
    constexpr float kTurningTractionSlipYawCoherenceFloor = 0.70f;
    // [Medium] Minimum planar-acceleration coherence expected while the robot is still gripping. Lower this only if
    // accel bias/noise clearly depresses clean data; raise it if the sweep overruns after visible sliding starts.
    constexpr float kTurningTractionSlipPlanarCoherenceFloor = 0.65f;
    // [Medium] Sustained mismatch time required before the traction detector trips. Increase if brief bumps trigger
    // false positives; decrease if obvious sliding is taking too long to stop the test.
    constexpr uint16_t kTurningTractionSlipConfirmMs = 150U;
    // [Low] Hard stop for the turning-traction sweep. Increase only if the configured acceleration profile can no
    // longer reach max speed within this window.
    constexpr uint32_t kTurningTractionSweepTimeoutMs = 90000U;
    // [Low] Number of cells in the enclosed corridor used by the auxiliary mapping-repeatability sweep. This includes
    // the start cell.
    constexpr uint8_t kCorridorRepeatabilityRowCellCount = 5U;
    // [Medium] Number of speed points exercised by the corridor repeatability sweep.
    constexpr uint8_t kCorridorRepeatabilitySpeedCount = 4U;
    // [Medium] Cruise speeds used by the corridor repeatability sweep. Increase upper entries if the run remains
    // clearly reliable; reduce any entry that obviously overruns the corridor or fails to settle at turn-around.
    constexpr float kCorridorRepeatabilitySpeedsMps[kCorridorRepeatabilitySpeedCount] = { 0.30f, 0.45f, 0.60f, 0.75f };
    // [Medium] Acceleration used for the straight corridor sweeps. Keep this high enough that the robot reaches the
    // test speed within the available corridor length.
    constexpr float kCorridorRepeatabilityAccelMps2 = 1.00f;
    // [Medium] Deceleration used for the corridor sweeps and end-of-leg stopping.
    constexpr float kCorridorRepeatabilityDecelMps2 = 1.20f;
    // [Medium] Turn-rate ceiling used for the 180-degree turnarounds in the corridor sweep.
    constexpr float kCorridorRepeatabilityTurnMaxOmegaRadps = Config::kSearchTurnMaxOmegaRadps;
    // [Medium] Angular acceleration used for the corridor turnarounds.
    constexpr float kCorridorRepeatabilityTurnAccelRadps2 = Config::kSearchTurnAccelRadps2;
    // [Medium] Short hold at the calibrated start pose before each speed pass.
    constexpr uint16_t kCorridorRepeatabilityStartSettleMs = 150U;
    // [Low] Number of cells in the northbound enclosed corridor used by the position-accuracy audit. This count
    // includes the start cell and the corner cell at the far end.
    constexpr uint8_t kPositionAuditNorthCorridorCellCount = 5U;
    // [Low] Number of cells extending east beyond the corner cell in the position-accuracy audit. With the short 90
    // ending on the half-step east of the corner, a value of four leaves seven clear half-steps before the east wall.
    constexpr uint8_t kPositionAuditEastBranchCellCount = 4U;
    // [Medium] Straight-speed points used by the position-accuracy audit.
    constexpr uint8_t kPositionAuditStraightSpeedCount = 3U;
    constexpr float kPositionAuditStraightSpeedsMps[kPositionAuditStraightSpeedCount] = { 0.30f, 0.55f, 0.80f };
    // [Medium] Corner-entry speed caps used by the position-accuracy audit.
    constexpr uint8_t kPositionAuditCornerSpeedCount = 3U;
    constexpr float kPositionAuditCornerSpeedsMps[kPositionAuditCornerSpeedCount] = { 0.30f, 0.55f, 0.80f };
    // Measurement runs must not inherit the mapping/search turn-rate ceiling. Keep this effectively unbounded so the
    // smooth-turn audit can ask for whatever angular command the maneuver-tracking law requires.
    constexpr float kPositionAuditCornerMaxOmegaRadps = 1000.0f;
    // [Medium] The fixed-fixture smooth-turn audit is intended to characterize high-speed cornering, so it runs with
    // the mission fan profile enabled through the entire phase, including the mirrored return path.
    constexpr bool kPositionAuditSmoothTurnFanEnabled = true;
    // [Medium] Shared linear limits used by the position-accuracy audit straight and corner trials.
    constexpr float kPositionAuditAccelMps2 = 1.00f;
    constexpr float kPositionAuditDecelMps2 = 1.20f;
    // [Medium] In-place turn directions exercised by the position-accuracy audit. Both directions should use the same
    // shared turn profile, so asymmetry here points at drivetrain or geometry error rather than a separate controller.
    constexpr uint8_t kPositionAuditInPlaceTurnCount = 2U;
    constexpr MazeMap::Direction kPositionAuditInPlaceTurnTargets[kPositionAuditInPlaceTurnCount] = {
        MazeMap::Right,
        MazeMap::Left,
    };
    // [Medium] Smooth-turn maneuvers exercised by the position-accuracy audit. These are the radius-sensitive mapping
    // turns that most directly expose effective-track-width drift versus nominal maneuver radius.
    constexpr uint8_t kPositionAuditSmoothTurnCodeCount = 2U;
    constexpr MazeMap::ManeuverCode kPositionAuditSmoothTurnCodes[kPositionAuditSmoothTurnCodeCount] = {
        MazeMap::S90SS,
        MazeMap::S90LS,
    };
    // [Low] Phase 1 runs a straight out-and-back from the start-cell center to the corner-cell center.
    constexpr uint8_t kPositionAuditPhase1ForwardHalfSteps = 8U;
    // [Low] Phase 2 runs the short smooth 90 on the fixed fixture using the exact half-step launch and runout that
    // fit the 5-cell north corridor and 4-cell east leg.
    constexpr uint8_t kPositionAuditPhase2PreTurnHalfSteps = 7U;
    constexpr uint8_t kPositionAuditPhase2PostTurnHalfSteps = 7U;
    // [Low] Phase 3 runs the long smooth 90 on the same fixed fixture.
    constexpr uint8_t kPositionAuditPhase3PreTurnHalfSteps = 6U;
    constexpr uint8_t kPositionAuditPhase3PostTurnHalfSteps = 6U;
    // [Medium] Short settle used at the mission start pose and between anchored audit phases.
    constexpr uint16_t kPositionAuditStartSettleMs = 150U;
    // [Medium] SD flush cadence during auxiliary capture. Decrease it if you want less risk of losing data on power
    // interruption; increase it if flush overhead becomes the limiting factor.
    constexpr uint32_t kLogFlushPeriodMs = 250U;
}

namespace FrontWallCharacterizationConfig
{
    // [Low] Short pins 39 and 40 at startup to enter the persistent front-wall characterization routine. These pins
    // are selector-only and are not sampled as part of the stored curve.
    constexpr uint8_t kModeSelectPinA = 39U;
    constexpr uint8_t kModeSelectPinB = 40U;
    // [Medium] Control period for the reverse characterization run. Keep this fast enough for smooth wheel control and
    // fine encoder-based spacing along the captured curve.
    constexpr unsigned long kControlPeriodUs = 1000UL;
    // [Medium] Initial settle before the reverse sweep starts. Increase if placement against the wall still rings in
    // the chassis or if the operator needs more time after letting go.
    constexpr uint16_t kStartupSettleMs = 400U;
    // [Medium] Once the SD card is present, wait this long before starting the characterization flow so the operator
    // can finish installing the card and clear their hands from the robot.
    constexpr uint16_t kPostSdReadyDelayMs = 15000U;
    // [Medium] Reverse speed used while backing away from the wall. Keep this slow so the stored curve is dominated by
    // the front-sensor response rather than drivetrain transients.
    constexpr float kReverseSpeedMps = 0.08f;
    // [Medium] Acceleration limit while ramping into the reverse characterization speed.
    constexpr float kReverseAccelMps2 = 0.20f;
    // [Medium] Angular-rate limit used by the heading hold during the reverse sweep.
    constexpr float kMaxAngularCommandRadps = 2.0f;
    // [Medium] Distance spacing between stored curve samples. Lower for denser templates; raise if EEPROM space is
    // needed elsewhere.
    constexpr float kStoredDistanceStepM = 0.001f;
    // [Medium] Maximum reverse travel allowed while searching for the collapse-to-zero region.
    constexpr float kMaxReverseTravelM = 0.14f;
    // [Medium] Differential-light threshold treated as collapsed-to-zero during this dark-room characterization mode.
    constexpr float kCollapsedDifferentialLightThreshold = 0.0005f;
    // [Medium] Require the collapsed condition for several consecutive control samples before ending the sweep so a
    // single noisy read does not truncate the stored curve.
    constexpr uint8_t kCollapsedConsecutiveSamples = 12U;
    // [Low] Ignore collapse-to-zero until the robot has backed off by at least this much, so a bad starting placement
    // or a transient first read cannot terminate the run immediately.
    constexpr float kMinimumTravelBeforeCollapseCheckM = 0.01f;
    // [Medium] Post-capture stationary hold before reporting success so the stored endpoint corresponds to a settled
    // robot state.
    constexpr uint16_t kPostCaptureSettleMs = 300U;
    // [Low] EEPROM address used for the persisted front-wall curve. Keep this at zero until other persistent data
    // needs to coexist in the same Teensy EEPROM region.
    constexpr int kStorageAddress = 0;
}

namespace LedCalibrationConfig
{
    constexpr uint8_t kModeSelectPinA = 38U;
    constexpr uint8_t kModeSelectPinB = 39U;
}

struct MotionLimits
{
    float maxSpeedMps;
    float accelMps2;
    float decelMps2;
    float maxAngularSpeedRadps;
    float angularAccelRadps2;
    float angleToleranceRad = Config::kAngleToleranceRad;
    float angularSpeedToleranceRadps = Config::kAngularSpeedToleranceRadps;
};

struct PoseEstimate
{
    float xMeters;
    float yMeters;
    MazeMap::Vectorf<2> headingUnit;
    float yawRad;
    float linearSpeedMps;
    float angularSpeedRadps;
};

struct SensorSnapshot
{
    float frontLeftDistanceM;
    float frontRightDistanceM;
    float frontLeftDifferentialLight;
    float frontRightDifferentialLight;
    float sideLeftDistanceM;
    float sideRightDistanceM;
    float sideLeftDifferentialLight;
    float sideRightDifferentialLight;
    float corridorErrorM;
    float frontSkewM;
    float accelBodyXMps2;
    float accelBodyYMps2;
    float planarAccelMps2;
    float gyroRadps;
    bool frontWall;
    bool frontLeftWall;
    bool frontRightWall;
    bool frontWallObservationValid;
    bool frontWallUsesFallbackDetection;
    bool frontWallUsesCharacterizationDetection;
    bool leftWall;
    bool rightWall;
    bool leftDistanceValidForControl;
    bool rightDistanceValidForControl;
    bool leftWallObservation;
    bool rightWallObservation;
    bool leftWallObservationWindowValid;
    bool rightWallObservationWindowValid;
    bool leftTransitionDetected;
    bool rightTransitionDetected;
};

struct DriveTelemetry
{
    float leftDriveCommand = 0.0f;
    float rightDriveCommand = 0.0f;
    int32_t leftEncoderCount = 0;
    int32_t rightEncoderCount = 0;
    float leftDistanceM = 0.0f;
    float rightDistanceM = 0.0f;
    float leftVelocityMps = 0.0f;
    float rightVelocityMps = 0.0f;
};

struct EncoderProgressWatchdog
{
    void Reset(float traveledM, unsigned long nowMs) noexcept
    {
        _lastProgressM = traveledM;
        _lastProgressMs = nowMs;
        _activeMotionStartMs = nowMs;
        _activeMotionCommand = false;
    }

    bool Stalled(float traveledM, float commandedSpeedMps, float remainingM, unsigned long nowMs) noexcept
    {
        if ((traveledM - _lastProgressM) >= Config::kEncoderProgressEpsilonM)
        {
            _lastProgressM = traveledM;
            _lastProgressMs = nowMs;
        }

        if (!MazeMap::IsEncoderProgressWatchdogArmed(
                commandedSpeedMps,
                remainingM,
                _activeMotionCommand ? (nowMs - _activeMotionStartMs) : 0UL,
                Config::kEncoderStallCommandThresholdMps,
                Config::kDistanceToleranceM,
                Config::kEncoderStallStartupGraceMs))
        {
            if ((commandedSpeedMps >= Config::kEncoderStallCommandThresholdMps) && (remainingM > Config::kDistanceToleranceM))
            {
                if (!_activeMotionCommand)
                {
                    _activeMotionStartMs = nowMs;
                    _activeMotionCommand = true;
                    _lastProgressMs = nowMs;
                }
            }
            else
            {
                _activeMotionStartMs = nowMs;
                _activeMotionCommand = false;
            }

            _lastProgressMs = nowMs;
            return false;
        }

        return static_cast<unsigned long>(nowMs - _lastProgressMs) >= Config::kEncoderStallTimeoutMs;
    }

private:
    float _lastProgressM = 0.0f;
    unsigned long _lastProgressMs = 0UL;
    unsigned long _activeMotionStartMs = 0UL;
    bool _activeMotionCommand = false;
};

struct WallSensorTelemetry
{
    float ambientLight = 0.0f;
    float litLight = 0.0f;
    float differentialLight = 0.0f;
    float rawDistanceM = 0.20f;
    float distanceM = 0.20f;
    bool wall = false;
};

struct ImuTelemetry
{
    uint8_t status = 0U;
    int16_t gyroX = 0;
    int16_t gyroY = 0;
    int16_t gyroZ = 0;
    int16_t accelX = 0;
    int16_t accelY = 0;
    int16_t accelZ = 0;
    int16_t temp = 0;
    bool interruptHigh = false;
};

struct DiagnosticSensorSnapshot
{
    WallSensorTelemetry frontLeft;
    WallSensorTelemetry frontRight;
    WallSensorTelemetry sideLeft;
    WallSensorTelemetry sideRight;
    ImuTelemetry imuFrontRight;
    ImuTelemetry imuBackLeft;
    float corridorErrorM = 0.0f;
    float frontSkewM = 0.0f;
    float accelBodyXMps2 = 0.0f;
    float accelBodyYMps2 = 0.0f;
    bool frontWall = false;
    bool leftWall = false;
    bool rightWall = false;
    bool leftDistanceValidForControl = false;
    bool rightDistanceValidForControl = false;
    float gyroBiasRadps = 0.0f;
    float gyroRawRadps = 0.0f;
    float gyroRadps = 0.0f;
};

enum class WallSensorId : uint8_t
{
    FrontLeft = 0U,
    FrontRight = 1U,
    SideLeft = 2U,
    SideRight = 3U,
    Count = 4U
};

enum class CalibrationWall : uint8_t
{
    West,
    East,
    South,
    North
};

enum class WallTouchOutcome : uint8_t
{
    SeatedContact,
    PassedThroughNoWall
};

struct RawWallSensorSample
{
    float ambientLight = 0.0f;
    float litLight = 0.0f;
    float differentialLight = 0.0f;
    float rawDistanceM = 0.20f;
};

struct WallSensorCalibrationInput
{
    float measuredValue = 0.0f;
    float fallbackDistanceM = 0.20f;
    float differentialLight = 0.0f;
    float ambientLight = 0.0f;
    float litLight = 0.0f;
};

struct RobustSignalBand
{
    float median = 0.0f;
    float low = 0.0f;
    float high = 0.0f;
};

struct WallSensorCalibrationCapture
{
    WallSensorCalibrationInput input{};
    RobustSignalBand differentialLightBand{};
    bool haveDifferentialLightBand = false;
};

template <size_t MaxSamples>
struct FrontCalibrationSpinSampleSet
{
    std::array<float, MaxSamples> frontLeftOpenSamples{};
    std::array<float, MaxSamples> frontLeftWallSamples{};
    std::array<float, MaxSamples> frontLeftWallDistanceSamples{};
    std::array<float, MaxSamples> frontRightOpenSamples{};
    std::array<float, MaxSamples> frontRightWallSamples{};
    std::array<float, MaxSamples> frontRightWallDistanceSamples{};
    uint16_t frontLeftOpenCount = 0U;
    uint16_t frontLeftWallCount = 0U;
    uint16_t frontRightOpenCount = 0U;
    uint16_t frontRightWallCount = 0U;

    void Push(
        MazeMap::FrontCalibrationSpinHeadingClass headingClass,
        float frontLeftDifferentialLight,
        float frontRightDifferentialLight,
        float frontLeftWallDistanceM,
        float frontRightWallDistanceM) noexcept
    {
        if (headingClass == MazeMap::FrontCalibrationSpinHeadingClass::OpenNorth)
        {
            if (std::isfinite(frontLeftDifferentialLight) &&
                frontLeftDifferentialLight >= 0.0f &&
                frontLeftOpenCount < static_cast<uint16_t>(MaxSamples))
            {
                frontLeftOpenSamples[frontLeftOpenCount] = frontLeftDifferentialLight;
                ++frontLeftOpenCount;
            }
            if (std::isfinite(frontRightDifferentialLight) &&
                frontRightDifferentialLight >= 0.0f &&
                frontRightOpenCount < static_cast<uint16_t>(MaxSamples))
            {
                frontRightOpenSamples[frontRightOpenCount] = frontRightDifferentialLight;
                ++frontRightOpenCount;
            }
            return;
        }

        if (headingClass == MazeMap::FrontCalibrationSpinHeadingClass::Wall)
        {
            if (std::isfinite(frontLeftDifferentialLight) &&
                frontLeftDifferentialLight > 0.0f &&
                std::isfinite(frontLeftWallDistanceM) &&
                frontLeftWallDistanceM > 0.0f &&
                frontLeftWallCount < static_cast<uint16_t>(MaxSamples))
            {
                frontLeftWallSamples[frontLeftWallCount] = frontLeftDifferentialLight;
                frontLeftWallDistanceSamples[frontLeftWallCount] = frontLeftWallDistanceM;
                ++frontLeftWallCount;
            }
            if (std::isfinite(frontRightDifferentialLight) &&
                frontRightDifferentialLight > 0.0f &&
                std::isfinite(frontRightWallDistanceM) &&
                frontRightWallDistanceM > 0.0f &&
                frontRightWallCount < static_cast<uint16_t>(MaxSamples))
            {
                frontRightWallSamples[frontRightWallCount] = frontRightDifferentialLight;
                frontRightWallDistanceSamples[frontRightWallCount] = frontRightWallDistanceM;
                ++frontRightWallCount;
            }
        }
    }
};

template <uint8_t WindowCycles>
struct AveragedWallSensorInputWindow
{
    void Clear() noexcept
    {
        measuredValue.Clear();
        fallbackDistanceM.Clear();
        differentialLight.Clear();
        ambientLight.Clear();
        litLight.Clear();
    }

    WallSensorCalibrationInput Average() const noexcept
    {
        WallSensorCalibrationInput averaged{};
        averaged.measuredValue = measuredValue.Average();
        averaged.fallbackDistanceM = fallbackDistanceM.Average();
        averaged.differentialLight = differentialLight.Average();
        averaged.ambientLight = ambientLight.Average();
        averaged.litLight = litLight.Average();
        return averaged;
    }

    WallSensorCalibrationInput PushAndAverage(const WallSensorCalibrationInput& input) noexcept
    {
        measuredValue.Push(input.measuredValue);
        fallbackDistanceM.Push(input.fallbackDistanceM);
        differentialLight.Push(input.differentialLight);
        ambientLight.Push(input.ambientLight);
        litLight.Push(input.litLight);
        return Average();
    }

    MazeMap::RollingAverageWindow<WindowCycles> measuredValue;
    MazeMap::RollingAverageWindow<WindowCycles> fallbackDistanceM;
    MazeMap::RollingAverageWindow<WindowCycles> differentialLight;
    MazeMap::RollingAverageWindow<WindowCycles> ambientLight;
    MazeMap::RollingAverageWindow<WindowCycles> litLight;
};

#if defined(ARDUINO_TEENSY41)
inline bool ConfigureLoopMatchedBackLeftImu(
    MazeMap::Vehicle::ImuBackLeft& imu,
    unsigned long controlPeriodUs,
    bool enableAccel)
{
    using Imu = MazeMap::Vehicle::ImuBackLeft;

    switch (MazeMap::SelectUiImuSamplingProfile(controlPeriodUs))
    {
    case MazeMap::UiImuSamplingProfile::Exact1000Hz:
        imu.ConfigureUiHighAccuracyOdr(
            Imu::HAODR_SELECTION::EXACT_1000_2000_4000_8000,
            enableAccel ? Imu::ODR_SETTING::ODR_0960HZ_HP_N_LP : Imu::ODR_SETTING::DISABLE,
            Imu::ODR_SETTING::ODR_0960HZ_HP_N_LP);
        return true;
    case MazeMap::UiImuSamplingProfile::Exact2000Hz:
        imu.ConfigureUiHighAccuracyOdr(
            Imu::HAODR_SELECTION::EXACT_1000_2000_4000_8000,
            enableAccel ? Imu::ODR_SETTING::ODR_1920HZ_HP_N_LP : Imu::ODR_SETTING::DISABLE,
            Imu::ODR_SETTING::ODR_1920HZ_HP_N_LP);
        return true;
    default:
        Serial.print("Unsupported IMU control period us: ");
        Serial.println(controlPeriodUs);
        return false;
    }
}

inline uint8_t ReadDrivenLowPinWithPullup(uint8_t pin)
{
    pinMode(pin, INPUT_PULLUP);
    delayMicroseconds(20);
    const uint8_t level = static_cast<uint8_t>(digitalRead(pin));
    pinMode(pin, INPUT);
    return level;
}

struct AveragedBackLeftImuSample
{
    float accelMgX = 0.0f;
    float accelMgY = 0.0f;
    float accelMgZ = 0.0f;
    float gyroDpsX = 0.0f;
    float gyroDpsY = 0.0f;
    float gyroDpsZ = 0.0f;
};

enum class StationaryImuCalibrationResult : uint8_t
{
    Success = 0U,
    RestartEncoderMotion,
    Failure,
};

inline constexpr unsigned long kImuCalibrationSampleIntervalMs = 2UL;
inline constexpr uint16_t kImuSelfTestAverageSamples = 64U;
inline constexpr uint16_t kImuSelfTestSettleMs = 50U;
inline constexpr float kImuSelfTestGyroFullScaleDps = 2000.0f;

inline MazeMap::EncoderCountPair CaptureDriveEncoderCounts()
{
    MazeMap::EncoderCountPair counts{};
    const auto& leftDriveHardware = MazeMap::MotorEncoderDrive::GetLeftHardwareConfig();
    const auto& rightDriveHardware = MazeMap::MotorEncoderDrive::GetRightHardwareConfig();
    counts.left = MazeMap::Platform::ReadEncoderCount(leftDriveHardware.encoderChannel);
    counts.right = MazeMap::Platform::ReadEncoderCount(rightDriveHardware.encoderChannel);
    return counts;
}

inline bool HaveDriveEncodersMovedSince(const MazeMap::EncoderCountPair& startCounts)
{
    return MazeMap::HaveEncoderCountsChanged(startCounts, CaptureDriveEncoderCounts());
}

inline StationaryImuCalibrationResult WaitForImuCalibrationSettle(
    const MazeMap::EncoderCountPair& startCounts,
    unsigned long settleMs)
{
    const unsigned long settleStartMs = millis();
    while ((millis() - settleStartMs) < settleMs)
    {
        if (HaveDriveEncodersMovedSince(startCounts))
        {
            return StationaryImuCalibrationResult::RestartEncoderMotion;
        }

        delay(1);
    }

    return StationaryImuCalibrationResult::Success;
}

inline bool ConfigureBackLeftImuForRuntime(
    MazeMap::Vehicle::ImuBackLeft& imu,
    unsigned long controlPeriodUs,
    bool enableAccel,
    MazeMap::Vehicle::ImuBackLeft::ACCEL_FILTER_FREQ accelFilterFreq =
        MazeMap::Vehicle::ImuBackLeft::ACCEL_FILTER_FREQ::FRAC_1_400)
{
    const bool imuConfigured = ConfigureLoopMatchedBackLeftImu(imu, controlPeriodUs, enableAccel);
    if (!imuConfigured)
    {
        return false;
    }

    imu.SetSelfTest(
        MazeMap::Vehicle::ImuBackLeft::SELF_TEST_MODE::DISABLED,
        MazeMap::Vehicle::ImuBackLeft::SELF_TEST_MODE::DISABLED);
    if (enableAccel)
    {
        imu.SetAccelRange(
            accelFilterFreq,
            MazeMap::Vehicle::ImuBackLeft::ACCEL_FULLSCALE::G8);
    }

    imu.SetGyroRange(
        MazeMap::Vehicle::ImuBackLeft::GYRO_LPF1_MODE::CUT_213,
        MazeMap::Vehicle::ImuBackLeft::GYRO_FULLSCALE_RANGE::DPS2000);
    return true;
}

inline StationaryImuCalibrationResult AverageBackLeftImuSelfTestSample(
    MazeMap::Vehicle::ImuBackLeft& imu,
    uint16_t sampleCount,
    const MazeMap::EncoderCountPair& startCounts,
    AveragedBackLeftImuSample& averagedSample)
{
    if (sampleCount == 0U)
    {
        return StationaryImuCalibrationResult::Failure;
    }

    const float accelMgPerLsb = imu.AccelSensitivityMgPerLsb();
    const float gyroDpsPerLsb = imu.GyroSensitivityMdpsPerLsb() / 1000.0f;
    double accelMgSumX = 0.0;
    double accelMgSumY = 0.0;
    double accelMgSumZ = 0.0;
    double gyroDpsSumX = 0.0;
    double gyroDpsSumY = 0.0;
    double gyroDpsSumZ = 0.0;

    for (uint16_t sampleIndex = 0U; sampleIndex < sampleCount; ++sampleIndex)
    {
        if (HaveDriveEncodersMovedSince(startCounts))
        {
            return StationaryImuCalibrationResult::RestartEncoderMotion;
        }

        const MazeMap::Vehicle::ImuBackLeft::Axes accel = imu.ReadAccel();
        const MazeMap::Vehicle::ImuBackLeft::Axes gyro = imu.ReadGyro();
        accelMgSumX += static_cast<double>(accel.x) * accelMgPerLsb;
        accelMgSumY += static_cast<double>(accel.y) * accelMgPerLsb;
        accelMgSumZ += static_cast<double>(accel.z) * accelMgPerLsb;
        gyroDpsSumX += static_cast<double>(gyro.x) * gyroDpsPerLsb;
        gyroDpsSumY += static_cast<double>(gyro.y) * gyroDpsPerLsb;
        gyroDpsSumZ += static_cast<double>(gyro.z) * gyroDpsPerLsb;
        delay(kImuCalibrationSampleIntervalMs);
    }

    if (HaveDriveEncodersMovedSince(startCounts))
    {
        return StationaryImuCalibrationResult::RestartEncoderMotion;
    }

    const double normalization = 1.0 / static_cast<double>(sampleCount);
    averagedSample.accelMgX = static_cast<float>(accelMgSumX * normalization);
    averagedSample.accelMgY = static_cast<float>(accelMgSumY * normalization);
    averagedSample.accelMgZ = static_cast<float>(accelMgSumZ * normalization);
    averagedSample.gyroDpsX = static_cast<float>(gyroDpsSumX * normalization);
    averagedSample.gyroDpsY = static_cast<float>(gyroDpsSumY * normalization);
    averagedSample.gyroDpsZ = static_cast<float>(gyroDpsSumZ * normalization);
    return StationaryImuCalibrationResult::Success;
}

inline StationaryImuCalibrationResult RunStationaryBackLeftImuSelfTest(
    MazeMap::Vehicle::ImuBackLeft& imu,
    unsigned long controlPeriodUs,
    const MazeMap::EncoderCountPair& startCounts,
    MazeMap::Vehicle::ImuBackLeft::ACCEL_FILTER_FREQ accelFilterFreq =
        MazeMap::Vehicle::ImuBackLeft::ACCEL_FILTER_FREQ::FRAC_1_400)
{
    if (!ConfigureBackLeftImuForRuntime(imu, controlPeriodUs, true, accelFilterFreq))
    {
        return StationaryImuCalibrationResult::Failure;
    }

    using SelfTestMode = MazeMap::Vehicle::ImuBackLeft::SELF_TEST_MODE;

    imu.SetSelfTest(SelfTestMode::DISABLED, SelfTestMode::DISABLED);
    StationaryImuCalibrationResult settleResult = WaitForImuCalibrationSettle(startCounts, kImuSelfTestSettleMs);
    if (settleResult != StationaryImuCalibrationResult::Success)
    {
        return settleResult;
    }

    AveragedBackLeftImuSample baseline{};
    StationaryImuCalibrationResult sampleResult =
        AverageBackLeftImuSelfTestSample(imu, kImuSelfTestAverageSamples, startCounts, baseline);
    if (sampleResult != StationaryImuCalibrationResult::Success)
    {
        return sampleResult;
    }

    imu.SetSelfTest(SelfTestMode::POSITIVE, SelfTestMode::POSITIVE);
    settleResult = WaitForImuCalibrationSettle(startCounts, kImuSelfTestSettleMs);
    if (settleResult != StationaryImuCalibrationResult::Success)
    {
        imu.SetSelfTest(SelfTestMode::DISABLED, SelfTestMode::DISABLED);
        return settleResult;
    }

    AveragedBackLeftImuSample stimulated{};
    sampleResult = AverageBackLeftImuSelfTestSample(imu, kImuSelfTestAverageSamples, startCounts, stimulated);
    imu.SetSelfTest(SelfTestMode::DISABLED, SelfTestMode::DISABLED);
    if (sampleResult != StationaryImuCalibrationResult::Success)
    {
        return sampleResult;
    }

    settleResult = WaitForImuCalibrationSettle(startCounts, kImuSelfTestSettleMs);
    if (settleResult != StationaryImuCalibrationResult::Success)
    {
        return settleResult;
    }

    const float accelDeltaMgX = std::fabs(stimulated.accelMgX - baseline.accelMgX);
    const float accelDeltaMgY = std::fabs(stimulated.accelMgY - baseline.accelMgY);
    const float accelDeltaMgZ = std::fabs(stimulated.accelMgZ - baseline.accelMgZ);
    const float gyroDeltaDpsX = std::fabs(stimulated.gyroDpsX - baseline.gyroDpsX);
    const float gyroDeltaDpsY = std::fabs(stimulated.gyroDpsY - baseline.gyroDpsY);
    const float gyroDeltaDpsZ = std::fabs(stimulated.gyroDpsZ - baseline.gyroDpsZ);
    const bool accelOk =
        MazeMap::IsAccelSelfTestDeltaValidMg(accelDeltaMgX) &&
        MazeMap::IsAccelSelfTestDeltaValidMg(accelDeltaMgY) &&
        MazeMap::IsAccelSelfTestDeltaValidMg(accelDeltaMgZ);
    const bool gyroOk =
        MazeMap::IsGyroSelfTestDeltaValidDps(gyroDeltaDpsX, kImuSelfTestGyroFullScaleDps) &&
        MazeMap::IsGyroSelfTestDeltaValidDps(gyroDeltaDpsY, kImuSelfTestGyroFullScaleDps) &&
        MazeMap::IsGyroSelfTestDeltaValidDps(gyroDeltaDpsZ, kImuSelfTestGyroFullScaleDps);
    if (accelOk && gyroOk)
    {
        return StationaryImuCalibrationResult::Success;
    }

    Serial.print("IMU stationary self-test failed; accel_delta_mg=[");
    Serial.print(accelDeltaMgX, 1);
    Serial.print(',');
    Serial.print(accelDeltaMgY, 1);
    Serial.print(',');
    Serial.print(accelDeltaMgZ, 1);
    Serial.print("], gyro_delta_dps=[");
    Serial.print(gyroDeltaDpsX, 1);
    Serial.print(',');
    Serial.print(gyroDeltaDpsY, 1);
    Serial.print(',');
    Serial.print(gyroDeltaDpsZ, 1);
    Serial.println(']');
    return StationaryImuCalibrationResult::Failure;
}

inline void PrintHexByte(uint8_t value)
{
    if (value < 0x10U)
    {
        Serial.print('0');
    }
    Serial.print(static_cast<unsigned>(value), HEX);
}
#endif

inline const char* WallSensorIdName(WallSensorId sensorId)
{
    switch (sensorId)
    {
    case WallSensorId::FrontLeft:
        return "front_left";
    case WallSensorId::FrontRight:
        return "front_right";
    case WallSensorId::SideLeft:
        return "side_left";
    case WallSensorId::SideRight:
        return "side_right";
    default:
        return "unknown";
    }
}

inline const char* CalibrationWallName(CalibrationWall wall)
{
    switch (wall)
    {
    case CalibrationWall::West:
        return "west";
    case CalibrationWall::East:
        return "east";
    case CalibrationWall::South:
        return "south";
    case CalibrationWall::North:
        return "north";
    default:
        return "unknown";
    }
}

inline const char* WallTouchOutcomeName(WallTouchOutcome outcome)
{
    switch (outcome)
    {
    case WallTouchOutcome::SeatedContact:
        return "seated_contact";
    case WallTouchOutcome::PassedThroughNoWall:
        return "passed_through";
    default:
        return "unknown";
    }
}

inline const char* DirectionName(MazeMap::Direction direction)
{
    switch (direction)
    {
    case MazeMap::Up:
        return "up";
    case MazeMap::UpRight:
        return "up_right";
    case MazeMap::Right:
        return "right";
    case MazeMap::DownRight:
        return "down_right";
    case MazeMap::Down:
        return "down";
    case MazeMap::DownLeft:
        return "down_left";
    case MazeMap::Left:
        return "left";
    case MazeMap::UpLeft:
        return "up_left";
    default:
        return "unknown";
    }
}

inline const char* WallStateName(MazeMap::WallState state)
{
    switch (state)
    {
    case MazeMap::NoWall:
        return "no_wall";
    case MazeMap::Wall:
        return "wall";
    case MazeMap::Unknown:
    default:
        return "unknown";
    }
}

inline bool IsFrontWallSensor(WallSensorId sensorId)
{
    return sensorId == WallSensorId::FrontLeft || sensorId == WallSensorId::FrontRight;
}

inline MazeMap::WallSensorCalibrationMode WallSensorCalibrationModeFor(WallSensorId sensorId)
{
    return IsFrontWallSensor(sensorId) ?
        MazeMap::WallSensorCalibrationMode::DirectInterpolation :
        MazeMap::WallSensorCalibrationMode::DistanceOffset;
}

inline const char* WallSensorCalibrationMeasurementName(WallSensorId sensorId)
{
    return IsFrontWallSensor(sensorId) ? "differential_light" : "raw_distance_m";
}

class WallDistanceCalibration
{
public:
    WallDistanceCalibration()
        : _curves{}
        , _frontSignalModelCache{}
        , _expectedSideWallDistanceM(Config::kExpectedSideWallDistanceM)
    {
    }

    void Clear()
    {
        for (uint8_t i = 0U; i < static_cast<uint8_t>(WallSensorId::Count); ++i)
        {
            _curves[i].Clear();
        }
        InvalidateFrontSignalModelCache();
        _frontWallBaselineDifferentialLight[0] = 0.0f;
        _frontWallBaselineDifferentialLight[1] = 0.0f;
        _frontWallBaselineValid[0] = false;
        _frontWallBaselineValid[1] = false;
        _frontWallBaselineDifferentialLightLow[0] = 0.0f;
        _frontWallBaselineDifferentialLightLow[1] = 0.0f;
        _frontWallBaselineDifferentialLightHigh[0] = 0.0f;
        _frontWallBaselineDifferentialLightHigh[1] = 0.0f;
        _frontWallBaselineBandValid[0] = false;
        _frontWallBaselineBandValid[1] = false;
        _frontWallWeakestCalibrationMeasuredValue[0] = 0.0f;
        _frontWallWeakestCalibrationMeasuredValue[1] = 0.0f;
        _frontWallWeakestCalibrationDifferentialLightLow[0] = 0.0f;
        _frontWallWeakestCalibrationDifferentialLightLow[1] = 0.0f;
        _frontWallWeakestCalibrationDifferentialLightHigh[0] = 0.0f;
        _frontWallWeakestCalibrationDifferentialLightHigh[1] = 0.0f;
        _frontWallWeakestCalibrationBandValid[0] = false;
        _frontWallWeakestCalibrationBandValid[1] = false;
        _frontWallDirectOnRiseThreshold[0] = 0.0f;
        _frontWallDirectOnRiseThreshold[1] = 0.0f;
        _frontWallDirectOffRiseThreshold[0] = 0.0f;
        _frontWallDirectOffRiseThreshold[1] = 0.0f;
        _frontWallDirectSignalBaseline[0] = 0.0f;
        _frontWallDirectSignalBaseline[1] = 0.0f;
        _frontWallDirectThresholdValid[0] = false;
        _frontWallDirectThresholdValid[1] = false;
        _sideWallBaselineDifferentialLight[0] = 0.0f;
        _sideWallBaselineDifferentialLight[1] = 0.0f;
        _sideWallBaselineValid[0] = false;
        _sideWallBaselineValid[1] = false;
        _sideWallBaselineDifferentialLightLow[0] = 0.0f;
        _sideWallBaselineDifferentialLightLow[1] = 0.0f;
        _sideWallBaselineDifferentialLightHigh[0] = 0.0f;
        _sideWallBaselineDifferentialLightHigh[1] = 0.0f;
        _sideWallBaselineBandValid[0] = false;
        _sideWallBaselineBandValid[1] = false;
        _sideWallReferenceDifferentialLight[0] = 0.0f;
        _sideWallReferenceDifferentialLight[1] = 0.0f;
        _sideWallReferenceValid[0] = false;
        _sideWallReferenceValid[1] = false;
        _sideWallReferenceDifferentialLightLow[0] = 0.0f;
        _sideWallReferenceDifferentialLightLow[1] = 0.0f;
        _sideWallReferenceDifferentialLightHigh[0] = 0.0f;
        _sideWallReferenceDifferentialLightHigh[1] = 0.0f;
        _sideWallReferenceBandValid[0] = false;
        _sideWallReferenceBandValid[1] = false;
        _sideWallReferenceDistanceM[0] = 0.0f;
        _sideWallReferenceDistanceM[1] = 0.0f;
        _sideWallReferenceDistanceValid[0] = false;
        _sideWallReferenceDistanceValid[1] = false;
        _expectedSideWallDistanceM = Config::kExpectedSideWallDistanceM;
    }

    bool AddPoint(WallSensorId sensorId, float measuredValue, float actualDistanceM, float ambientLight = 0.0f)
    {
        const bool stored = _curves[static_cast<uint8_t>(sensorId)].AddPoint(measuredValue, actualDistanceM, ambientLight);
        if (stored && IsFrontWallSensor(sensorId))
        {
            InvalidateFrontSignalModelCache(sensorId);
        }

        return stored;
    }

    float Apply(WallSensorId sensorId, float measuredValue, float fallbackDistanceM) const
    {
        if (!std::isfinite(fallbackDistanceM) || fallbackDistanceM <= 0.0f)
        {
            fallbackDistanceM = measuredValue;
        }
        if (!std::isfinite(measuredValue) || measuredValue <= 0.0f)
        {
            return fallbackDistanceM;
        }

        const MazeMap::WallSensorCalibrationCurve& curve = _curves[static_cast<uint8_t>(sensorId)];
        if (curve.GetCount() == 0U)
        {
            return fallbackDistanceM;
        }

        const MazeMap::WallSensorCalibrationMode mode = WallSensorCalibrationModeFor(sensorId);
        if (mode == MazeMap::WallSensorCalibrationMode::DirectInterpolation && curve.GetCount() < 2U)
        {
            return fallbackDistanceM;
        }

        return curve.Apply(measuredValue, mode);
    }

    void SetExpectedSideWallDistanceM(float expectedDistanceM)
    {
        if (std::isfinite(expectedDistanceM) && expectedDistanceM > 0.0f)
        {
            _expectedSideWallDistanceM = expectedDistanceM;
        }
    }

    float GetExpectedSideWallDistanceM() const
    {
        return _expectedSideWallDistanceM;
    }

    void SetFrontWallBaselineDifferentialLight(WallSensorId sensorId, float differentialLight)
    {
        if (!IsFrontWallSensor(sensorId) ||
            !std::isfinite(differentialLight) ||
            differentialLight < 0.0f)
        {
            return;
        }

        const uint8_t index = FrontWallIndex(sensorId);
        _frontWallBaselineDifferentialLight[index] = differentialLight;
        _frontWallBaselineValid[index] = true;
    }

    void SetFrontWallBaselineDifferentialLightBand(WallSensorId sensorId, float lowDifferentialLight, float highDifferentialLight)
    {
        if (!IsFrontWallSensor(sensorId) ||
            !std::isfinite(lowDifferentialLight) ||
            !std::isfinite(highDifferentialLight) ||
            lowDifferentialLight < 0.0f ||
            highDifferentialLight < lowDifferentialLight)
        {
            return;
        }

        const uint8_t index = FrontWallIndex(sensorId);
        _frontWallBaselineDifferentialLightLow[index] = lowDifferentialLight;
        _frontWallBaselineDifferentialLightHigh[index] = highDifferentialLight;
        _frontWallBaselineBandValid[index] = true;
    }

    bool TryGetFrontWallBaselineDifferentialLight(WallSensorId sensorId, float& differentialLight) const
    {
        differentialLight = 0.0f;
        if (!IsFrontWallSensor(sensorId))
        {
            return false;
        }

        const uint8_t index = FrontWallIndex(sensorId);
        if (!_frontWallBaselineValid[index])
        {
            return false;
        }

        differentialLight = _frontWallBaselineDifferentialLight[index];
        return std::isfinite(differentialLight) && differentialLight >= 0.0f;
    }

    bool TryGetFrontWallBaselineDifferentialLightBand(
        WallSensorId sensorId,
        float& lowDifferentialLight,
        float& highDifferentialLight) const
    {
        lowDifferentialLight = 0.0f;
        highDifferentialLight = 0.0f;
        if (!IsFrontWallSensor(sensorId))
        {
            return false;
        }

        const uint8_t index = FrontWallIndex(sensorId);
        if (!_frontWallBaselineBandValid[index])
        {
            return false;
        }

        lowDifferentialLight = _frontWallBaselineDifferentialLightLow[index];
        highDifferentialLight = _frontWallBaselineDifferentialLightHigh[index];
        return
            std::isfinite(lowDifferentialLight) &&
            std::isfinite(highDifferentialLight) &&
            lowDifferentialLight >= 0.0f &&
            highDifferentialLight >= lowDifferentialLight;
    }

    void SetFrontWeakestCalibrationDifferentialLightBand(
        WallSensorId sensorId,
        float measuredValue,
        float lowDifferentialLight,
        float highDifferentialLight)
    {
        if (!IsFrontWallSensor(sensorId) ||
            !std::isfinite(measuredValue) ||
            !std::isfinite(lowDifferentialLight) ||
            !std::isfinite(highDifferentialLight) ||
            measuredValue <= 0.0f ||
            lowDifferentialLight <= 0.0f ||
            highDifferentialLight < lowDifferentialLight)
        {
            return;
        }

        const uint8_t index = FrontWallIndex(sensorId);
        if (_frontWallWeakestCalibrationBandValid[index] &&
            (measuredValue > (_frontWallWeakestCalibrationMeasuredValue[index] + 0.001f)))
        {
            return;
        }

        _frontWallWeakestCalibrationMeasuredValue[index] = measuredValue;
        _frontWallWeakestCalibrationDifferentialLightLow[index] = lowDifferentialLight;
        _frontWallWeakestCalibrationDifferentialLightHigh[index] = highDifferentialLight;
        _frontWallWeakestCalibrationBandValid[index] = true;
    }

    bool TryGetFrontWeakestCalibrationDifferentialLightBand(
        WallSensorId sensorId,
        float& lowDifferentialLight,
        float& highDifferentialLight) const
    {
        lowDifferentialLight = 0.0f;
        highDifferentialLight = 0.0f;
        if (!IsFrontWallSensor(sensorId))
        {
            return false;
        }

        const uint8_t index = FrontWallIndex(sensorId);
        if (!_frontWallWeakestCalibrationBandValid[index])
        {
            return false;
        }

        lowDifferentialLight = _frontWallWeakestCalibrationDifferentialLightLow[index];
        highDifferentialLight = _frontWallWeakestCalibrationDifferentialLightHigh[index];
        return
            std::isfinite(lowDifferentialLight) &&
            std::isfinite(highDifferentialLight) &&
            lowDifferentialLight > 0.0f &&
            highDifferentialLight >= lowDifferentialLight;
    }

    void SetFrontDirectRiseThresholds(
        WallSensorId sensorId,
        float signalBaseline,
        float onRiseThreshold,
        float offRiseThreshold)
    {
        if (!IsFrontWallSensor(sensorId) ||
            !std::isfinite(signalBaseline) ||
            signalBaseline < 0.0f ||
            !std::isfinite(onRiseThreshold) ||
            !std::isfinite(offRiseThreshold) ||
            onRiseThreshold <= 0.0f ||
            offRiseThreshold <= 0.0f ||
            offRiseThreshold >= onRiseThreshold)
        {
            return;
        }

        const uint8_t index = FrontWallIndex(sensorId);
        _frontWallDirectSignalBaseline[index] = signalBaseline;
        _frontWallDirectOnRiseThreshold[index] = onRiseThreshold;
        _frontWallDirectOffRiseThreshold[index] = offRiseThreshold;
        _frontWallDirectThresholdValid[index] = true;
    }

    bool TryGetFrontDirectRiseThresholds(
        WallSensorId sensorId,
        float& signalBaseline,
        float& onRiseThreshold,
        float& offRiseThreshold) const
    {
        signalBaseline = 0.0f;
        onRiseThreshold = 0.0f;
        offRiseThreshold = 0.0f;
        if (!IsFrontWallSensor(sensorId))
        {
            return false;
        }

        const uint8_t index = FrontWallIndex(sensorId);
        if (!_frontWallDirectThresholdValid[index])
        {
            return false;
        }

        signalBaseline = _frontWallDirectSignalBaseline[index];
        onRiseThreshold = _frontWallDirectOnRiseThreshold[index];
        offRiseThreshold = _frontWallDirectOffRiseThreshold[index];
        return
            std::isfinite(signalBaseline) &&
            signalBaseline >= 0.0f &&
            std::isfinite(onRiseThreshold) &&
            std::isfinite(offRiseThreshold) &&
            onRiseThreshold > 0.0f &&
            offRiseThreshold > 0.0f &&
            offRiseThreshold < onRiseThreshold;
    }

    bool TryComputeSideWallDistanceThresholds(float latchSignalFraction, float releaseSignalFraction, float& onThresholdM, float& offThresholdM) const
    {
        onThresholdM = Config::kSideWallOnThresholdM;
        offThresholdM = Config::kSideWallOffThresholdM;

        float derivedOnThresholdM = 0.0f;
        float derivedOffThresholdM = 0.0f;
        if (!MazeMap::TryComputeLinearWallSignalDistanceThresholdM(
                _expectedSideWallDistanceM,
                latchSignalFraction,
                derivedOnThresholdM) ||
            !MazeMap::TryComputeLinearWallSignalDistanceThresholdM(
                _expectedSideWallDistanceM,
                releaseSignalFraction,
                derivedOffThresholdM))
        {
            return false;
        }

        if (!(std::isfinite(derivedOnThresholdM) &&
            std::isfinite(derivedOffThresholdM) &&
            derivedOnThresholdM > 0.0f &&
            derivedOffThresholdM >= derivedOnThresholdM))
        {
            return false;
        }

        onThresholdM = derivedOnThresholdM;
        offThresholdM = derivedOffThresholdM;
        return true;
    }

    void SetSideWallReferenceDifferentialLight(WallSensorId sensorId, float differentialLight)
    {
        if (!IsSideWallSensor(sensorId) ||
            !std::isfinite(differentialLight) ||
            differentialLight <= 0.0f)
        {
            return;
        }

        const uint8_t index = SideWallIndex(sensorId);
        _sideWallReferenceDifferentialLight[index] = differentialLight;
        _sideWallReferenceValid[index] = true;
    }

    void SetSideWallReferenceDifferentialLightBand(WallSensorId sensorId, float lowDifferentialLight, float highDifferentialLight)
    {
        if (!IsSideWallSensor(sensorId) ||
            !std::isfinite(lowDifferentialLight) ||
            !std::isfinite(highDifferentialLight) ||
            lowDifferentialLight <= 0.0f ||
            highDifferentialLight < lowDifferentialLight)
        {
            return;
        }

        const uint8_t index = SideWallIndex(sensorId);
        _sideWallReferenceDifferentialLightLow[index] = lowDifferentialLight;
        _sideWallReferenceDifferentialLightHigh[index] = highDifferentialLight;
        _sideWallReferenceBandValid[index] = true;
    }

    void SetSideWallReferenceDistanceM(WallSensorId sensorId, float distanceM)
    {
        if (!IsSideWallSensor(sensorId) ||
            !std::isfinite(distanceM) ||
            distanceM <= 0.0f)
        {
            return;
        }

        const uint8_t index = SideWallIndex(sensorId);
        _sideWallReferenceDistanceM[index] = distanceM;
        _sideWallReferenceDistanceValid[index] = true;
    }

    void SetSideWallBaselineDifferentialLight(WallSensorId sensorId, float differentialLight)
    {
        if (!IsSideWallSensor(sensorId) ||
            !std::isfinite(differentialLight) ||
            differentialLight < 0.0f)
        {
            return;
        }

        const uint8_t index = SideWallIndex(sensorId);
        _sideWallBaselineDifferentialLight[index] = differentialLight;
        _sideWallBaselineValid[index] = true;
    }

    void SetSideWallBaselineDifferentialLightBand(WallSensorId sensorId, float lowDifferentialLight, float highDifferentialLight)
    {
        if (!IsSideWallSensor(sensorId) ||
            !std::isfinite(lowDifferentialLight) ||
            !std::isfinite(highDifferentialLight) ||
            lowDifferentialLight < 0.0f ||
            highDifferentialLight < lowDifferentialLight)
        {
            return;
        }

        const uint8_t index = SideWallIndex(sensorId);
        _sideWallBaselineDifferentialLightLow[index] = lowDifferentialLight;
        _sideWallBaselineDifferentialLightHigh[index] = highDifferentialLight;
        _sideWallBaselineBandValid[index] = true;
    }

    bool TryGetSideWallBaselineDifferentialLight(WallSensorId sensorId, float& differentialLight) const
    {
        differentialLight = 0.0f;
        if (!IsSideWallSensor(sensorId))
        {
            return false;
        }

        const uint8_t index = SideWallIndex(sensorId);
        if (!_sideWallBaselineValid[index])
        {
            return false;
        }

        differentialLight = _sideWallBaselineDifferentialLight[index];
        return std::isfinite(differentialLight) && differentialLight >= 0.0f;
    }

    bool TryGetSideWallReferenceDifferentialLightBand(WallSensorId sensorId, float& lowDifferentialLight, float& highDifferentialLight) const
    {
        lowDifferentialLight = 0.0f;
        highDifferentialLight = 0.0f;
        if (!IsSideWallSensor(sensorId))
        {
            return false;
        }

        const uint8_t index = SideWallIndex(sensorId);
        if (!_sideWallReferenceBandValid[index])
        {
            return false;
        }

        lowDifferentialLight = _sideWallReferenceDifferentialLightLow[index];
        highDifferentialLight = _sideWallReferenceDifferentialLightHigh[index];
        return
            std::isfinite(lowDifferentialLight) &&
            std::isfinite(highDifferentialLight) &&
            lowDifferentialLight > 0.0f &&
            highDifferentialLight >= lowDifferentialLight;
    }

    bool TryGetSideWallBaselineDifferentialLightBand(WallSensorId sensorId, float& lowDifferentialLight, float& highDifferentialLight) const
    {
        lowDifferentialLight = 0.0f;
        highDifferentialLight = 0.0f;
        if (!IsSideWallSensor(sensorId))
        {
            return false;
        }

        const uint8_t index = SideWallIndex(sensorId);
        if (!_sideWallBaselineBandValid[index])
        {
            return false;
        }

        lowDifferentialLight = _sideWallBaselineDifferentialLightLow[index];
        highDifferentialLight = _sideWallBaselineDifferentialLightHigh[index];
        return
            std::isfinite(lowDifferentialLight) &&
            std::isfinite(highDifferentialLight) &&
            lowDifferentialLight >= 0.0f &&
            highDifferentialLight >= lowDifferentialLight;
    }

    bool TryComputeSideWallNormalizedReferenceDifferentialLight(
        WallSensorId sensorId,
        float& differentialLight) const
    {
        differentialLight = 0.0f;
        return TryGetSideWallReferenceDifferentialLight(sensorId, differentialLight);
    }

    bool TryComputeSideWallMeasuredThresholds(
        WallSensorId sensorId,
        float latchSignalFraction,
        float releaseSignalFraction,
        float& onMeasuredThreshold,
        float& offMeasuredThreshold,
        float& signalBaseline) const
    {
        onMeasuredThreshold = 0.0f;
        offMeasuredThreshold = 0.0f;
        signalBaseline = 0.0f;
        if (!IsSideWallSensor(sensorId))
        {
            return false;
        }

        float referenceDifferentialLight = 0.0f;
        if (!TryGetSideWallReferenceDifferentialLight(
                sensorId,
                referenceDifferentialLight))
        {
            return false;
        }

        float baselineDifferentialLight = 0.0f;
        float baselineDifferentialLightLow = 0.0f;
        float baselineDifferentialLightHigh = 0.0f;
        float referenceDifferentialLightLow = 0.0f;
        float referenceDifferentialLightHigh = 0.0f;
        if (TryGetSideWallReferenceDifferentialLightBand(
                sensorId,
                referenceDifferentialLightLow,
                referenceDifferentialLightHigh))
        {
            referenceDifferentialLight = referenceDifferentialLightLow;
        }
        else
        {
            referenceDifferentialLightLow = referenceDifferentialLight;
            referenceDifferentialLightHigh = referenceDifferentialLight;
        }

        if (TryGetSideWallBaselineDifferentialLightBand(
                sensorId,
                baselineDifferentialLightLow,
                baselineDifferentialLightHigh))
        {
        }
        else if (TryGetSideWallBaselineDifferentialLight(sensorId, baselineDifferentialLight))
        {
            baselineDifferentialLightLow = baselineDifferentialLight;
            baselineDifferentialLightHigh = baselineDifferentialLight;
        }

        if (MazeMap::TryComputeConservativeSignalRiseThresholdsFromBands(
                baselineDifferentialLightLow,
                baselineDifferentialLightHigh,
                referenceDifferentialLightLow,
                referenceDifferentialLightHigh,
                latchSignalFraction,
                releaseSignalFraction,
                onMeasuredThreshold,
                offMeasuredThreshold,
                signalBaseline))
        {
            return true;
        }

        if (TryGetSideWallBaselineDifferentialLight(sensorId, baselineDifferentialLight) &&
            MazeMap::TryComputeSignalRiseThresholds(
                baselineDifferentialLight,
                referenceDifferentialLight,
                latchSignalFraction,
                releaseSignalFraction,
                onMeasuredThreshold,
                offMeasuredThreshold))
        {
            signalBaseline = baselineDifferentialLight;
            return true;
        }

        return MazeMap::TryComputeSignalHighThresholds(
            referenceDifferentialLight,
            latchSignalFraction,
            releaseSignalFraction,
            onMeasuredThreshold,
            offMeasuredThreshold);
    }

    bool TryGetSideWallReferenceDifferentialLight(WallSensorId sensorId, float& differentialLight) const
    {
        differentialLight = 0.0f;
        if (!IsSideWallSensor(sensorId))
        {
            return false;
        }

        const uint8_t index = SideWallIndex(sensorId);
        if (!_sideWallReferenceValid[index])
        {
            return false;
        }

        differentialLight = _sideWallReferenceDifferentialLight[index];
        return std::isfinite(differentialLight) && differentialLight > 0.0f;
    }

    bool TryGetSideWallReferenceDistanceM(WallSensorId sensorId, float& distanceM) const
    {
        distanceM = 0.0f;
        if (!IsSideWallSensor(sensorId))
        {
            return false;
        }

        const uint8_t index = SideWallIndex(sensorId);
        if (_sideWallReferenceDistanceValid[index])
        {
            distanceM = _sideWallReferenceDistanceM[index];
            return std::isfinite(distanceM) && distanceM > 0.0f;
        }

        const MazeMap::WallSensorCalibrationCurve& curve = _curves[static_cast<uint8_t>(sensorId)];
        if (curve.GetCount() == 0U)
        {
            return false;
        }

        float distanceSumM = 0.0f;
        uint8_t validCount = 0U;
        for (uint8_t i = 0U; i < curve.GetCount(); ++i)
        {
            const float pointDistanceM = curve.GetPoint(i).actualDistanceM;
            if (!std::isfinite(pointDistanceM) || pointDistanceM <= 0.0f)
            {
                continue;
            }

            distanceSumM += pointDistanceM;
            ++validCount;
        }

        if (validCount == 0U)
        {
            return false;
        }

        distanceM = distanceSumM / static_cast<float>(validCount);
        return std::isfinite(distanceM) && distanceM > 0.0f;
    }

    bool TryGetWeakestFrontCalibrationMeasuredValue(
        WallSensorId sensorId,
        float& measuredValue) const
    {
        measuredValue = 0.0f;
        if (!IsFrontWallSensor(sensorId))
        {
            return false;
        }

        const MazeMap::WallSensorCalibrationCurve& curve = _curves[static_cast<uint8_t>(sensorId)];
        if (curve.GetCount() == 0U)
        {
            return false;
        }

        measuredValue = curve.GetPoint(0U).measuredValue;
        return std::isfinite(measuredValue) && measuredValue > 0.0f;
    }

    bool TryComputeFrontWallDistanceThresholds(
        const MazeMap::Vehicle& vehicle,
        float releaseHysteresisDistanceM,
        float& onDistanceThresholdM,
        float& offDistanceThresholdM) const
    {
        const float forwardSensorOffsetM = (std::min)(
            vehicle.FrontLeft.GetPosition().GetX(),
            vehicle.FrontRight.GetPosition().GetX());
        if (!MazeMap::TryComputeFrontWallHalfwayIntoAdjacentDistanceM(
                Config::kCellSizeM,
                Config::kMazeWallThicknessM,
                forwardSensorOffsetM,
                onDistanceThresholdM) ||
            !MazeMap::TryExpandWallThresholdDistanceM(
                onDistanceThresholdM,
                releaseHysteresisDistanceM,
                offDistanceThresholdM))
        {
            return false;
        }

        return MazeMap::TryClampWallThresholdDistanceRangeM(
            onDistanceThresholdM,
            offDistanceThresholdM,
            Config::kFrontWallOnThresholdM,
            Config::kFrontWallOffThresholdM,
            onDistanceThresholdM,
            offDistanceThresholdM);
    }

    bool TryComputeFrontSensorMeasuredThresholds(
        WallSensorId sensorId,
        const MazeMap::Vehicle& vehicle,
        float releaseHysteresisDistanceM,
        float ambientLight,
        float& onMeasuredThreshold,
        float& offMeasuredThreshold,
        float& signalBaseline) const
    {
        onMeasuredThreshold = 0.0f;
        offMeasuredThreshold = 0.0f;
        signalBaseline = 0.0f;

        if (!IsFrontWallSensor(sensorId))
        {
            return false;
        }

        if (TryGetFrontDirectRiseThresholds(
                sensorId,
                signalBaseline,
                onMeasuredThreshold,
                offMeasuredThreshold))
        {
            return true;
        }

        float weakestCalibrationDifferentialLightLow = 0.0f;
        float weakestCalibrationDifferentialLightHigh = 0.0f;
        float baselineDifferentialLightLow = 0.0f;
        float baselineDifferentialLightHigh = 0.0f;
        if (TryGetFrontWeakestCalibrationDifferentialLightBand(
                sensorId,
                weakestCalibrationDifferentialLightLow,
                weakestCalibrationDifferentialLightHigh) &&
            TryGetFrontWallBaselineDifferentialLightBand(
                sensorId,
                baselineDifferentialLightLow,
                baselineDifferentialLightHigh) &&
            MazeMap::TryComputeConservativeSignalRiseThresholdsFromBands(
                baselineDifferentialLightLow,
                baselineDifferentialLightHigh,
                weakestCalibrationDifferentialLightLow,
                weakestCalibrationDifferentialLightHigh,
                Config::kFrontWallSignalLatchFractionOfCalibratedSpan,
                Config::kFrontWallSignalReleaseFractionOfCalibratedSpan,
                onMeasuredThreshold,
                offMeasuredThreshold,
                signalBaseline))
        {
            return true;
        }

        float weakestCalibrationSignal = 0.0f;
        float baselineDifferentialLight = 0.0f;
        if (TryGetWeakestFrontCalibrationMeasuredValue(sensorId, weakestCalibrationSignal) &&
            TryGetFrontWallBaselineDifferentialLight(sensorId, baselineDifferentialLight) &&
            MazeMap::TryComputeSignalRiseThresholds(
                baselineDifferentialLight,
                weakestCalibrationSignal,
                Config::kFrontWallSignalLatchFractionOfCalibratedSpan,
                Config::kFrontWallSignalReleaseFractionOfCalibratedSpan,
                onMeasuredThreshold,
                offMeasuredThreshold))
        {
            signalBaseline = baselineDifferentialLight;
            return true;
        }

        float onDistanceThresholdM = 0.0f;
        float offDistanceThresholdM = 0.0f;
        if (!TryComputeFrontWallDistanceThresholds(
                vehicle,
                releaseHysteresisDistanceM,
                onDistanceThresholdM,
                offDistanceThresholdM))
        {
            return false;
        }

        float effectiveAmbientLight = ambientLight;
        if (!(std::isfinite(effectiveAmbientLight) && effectiveAmbientLight >= 0.0f) &&
            !TryComputeFrontSensorRepresentativeAmbientLight(sensorId, effectiveAmbientLight))
        {
            return false;
        }

        float signalGain = 0.0f;
        float signalLightScale = 0.0f;
        if (!TryGetFrontSignalModel(sensorId, signalGain, signalLightScale) ||
            !MazeMap::TryComputeMeasuredValueForActualDistanceUsingAmbientAwareLogSignalModel(
                signalGain,
                signalLightScale,
                effectiveAmbientLight,
                onDistanceThresholdM,
                onMeasuredThreshold) ||
            !MazeMap::TryComputeMeasuredValueForActualDistanceUsingAmbientAwareLogSignalModel(
                signalGain,
                signalLightScale,
                effectiveAmbientLight,
                offDistanceThresholdM,
                offMeasuredThreshold))
        {
            return false;
        }

        if (!MazeMap::TryScaleSignalHighThresholds(
                Config::kFrontWallMeasuredSignalThresholdScale,
                onMeasuredThreshold,
                offMeasuredThreshold))
        {
            onMeasuredThreshold = 0.0f;
            offMeasuredThreshold = 0.0f;
            return false;
        }

        return
            std::isfinite(onMeasuredThreshold) &&
            std::isfinite(offMeasuredThreshold) &&
            onMeasuredThreshold > 0.0f &&
            offMeasuredThreshold > 0.0f &&
            offMeasuredThreshold < onMeasuredThreshold;
    }

    bool TryComputeFrontSensorRepresentativeAmbientLight(WallSensorId sensorId, float& ambientLight) const
    {
        ambientLight = 0.0f;
        if (!IsFrontWallSensor(sensorId))
        {
            return false;
        }

        const MazeMap::WallSensorCalibrationCurve& curve = _curves[static_cast<uint8_t>(sensorId)];
        if (curve.GetCount() < 2U)
        {
            return false;
        }

        double ambientLightSum = 0.0;
        uint8_t validPointCount = 0U;
        for (uint8_t index = 0U; index < curve.GetCount(); ++index)
        {
            const MazeMap::WallSensorCalibrationCurve::Point& point = curve.GetPoint(index);
            if (!std::isfinite(point.ambientLight) || point.ambientLight < 0.0f)
            {
                return false;
            }

            ambientLightSum += static_cast<double>(point.ambientLight);
            ++validPointCount;
        }

        if (validPointCount == 0U)
        {
            return false;
        }

        ambientLight = static_cast<float>(ambientLightSum / static_cast<double>(validPointCount));
        return std::isfinite(ambientLight) && ambientLight >= 0.0f;
    }

    const MazeMap::WallSensorCalibrationCurve& GetCurve(WallSensorId sensorId) const
    {
        return _curves[static_cast<uint8_t>(sensorId)];
    }

private:
    struct FrontSignalModelCache
    {
        bool valid = false;
        float gain = 0.0f;
        float lightScale = 0.0f;
    };

    MazeMap::WallSensorCalibrationCurve _curves[static_cast<uint8_t>(WallSensorId::Count)];
    mutable FrontSignalModelCache _frontSignalModelCache[2];
    float _frontWallBaselineDifferentialLight[2] = {};
    bool _frontWallBaselineValid[2] = {};
    float _frontWallBaselineDifferentialLightLow[2] = {};
    float _frontWallBaselineDifferentialLightHigh[2] = {};
    bool _frontWallBaselineBandValid[2] = {};
    float _frontWallWeakestCalibrationMeasuredValue[2] = {};
    float _frontWallWeakestCalibrationDifferentialLightLow[2] = {};
    float _frontWallWeakestCalibrationDifferentialLightHigh[2] = {};
    bool _frontWallWeakestCalibrationBandValid[2] = {};
    float _frontWallDirectOnRiseThreshold[2] = {};
    float _frontWallDirectOffRiseThreshold[2] = {};
    float _frontWallDirectSignalBaseline[2] = {};
    bool _frontWallDirectThresholdValid[2] = {};
    float _sideWallBaselineDifferentialLight[2] = {};
    bool _sideWallBaselineValid[2] = {};
    float _sideWallBaselineDifferentialLightLow[2] = {};
    float _sideWallBaselineDifferentialLightHigh[2] = {};
    bool _sideWallBaselineBandValid[2] = {};
    float _sideWallReferenceDifferentialLight[2] = {};
    bool _sideWallReferenceValid[2] = {};
    float _sideWallReferenceDifferentialLightLow[2] = {};
    float _sideWallReferenceDifferentialLightHigh[2] = {};
    bool _sideWallReferenceBandValid[2] = {};
    float _sideWallReferenceDistanceM[2] = {};
    bool _sideWallReferenceDistanceValid[2] = {};
    float _expectedSideWallDistanceM;

    static bool IsSideWallSensor(WallSensorId sensorId)
    {
        return sensorId == WallSensorId::SideLeft || sensorId == WallSensorId::SideRight;
    }

    static uint8_t SideWallIndex(WallSensorId sensorId)
    {
        return (sensorId == WallSensorId::SideRight) ? 1U : 0U;
    }

    static uint8_t FrontWallIndex(WallSensorId sensorId)
    {
        return (sensorId == WallSensorId::FrontRight) ? 1U : 0U;
    }

    void InvalidateFrontSignalModelCache()
    {
        for (uint8_t index = 0U; index < 2U; ++index)
        {
            _frontSignalModelCache[index] = FrontSignalModelCache{};
        }
    }

    void InvalidateFrontSignalModelCache(WallSensorId sensorId)
    {
        if (!IsFrontWallSensor(sensorId))
        {
            return;
        }

        _frontSignalModelCache[FrontWallIndex(sensorId)] = FrontSignalModelCache{};
    }

    bool TryGetFrontSignalModel(WallSensorId sensorId, float& gain, float& lightScale) const
    {
        gain = 0.0f;
        lightScale = 0.0f;
        if (!IsFrontWallSensor(sensorId))
        {
            return false;
        }

        FrontSignalModelCache& cache = _frontSignalModelCache[FrontWallIndex(sensorId)];
        if (!cache.valid)
        {
            const MazeMap::WallSensorCalibrationCurve& curve = _curves[static_cast<uint8_t>(sensorId)];
            if (!MazeMap::TryFitAmbientAwareLogDifferentialSignalModel(curve, cache.gain, cache.lightScale))
            {
                return false;
            }

            cache.valid = true;
        }

        gain = cache.gain;
        lightScale = cache.lightScale;
        return std::isfinite(gain) && std::isfinite(lightScale) && gain > 0.0f && lightScale > 0.0f;
    }
};

inline WallDistanceCalibration gWallDistanceCalibration;

inline float ComputeDiagonalWallCenterOmegaRadps(
    const WallDistanceCalibration& wallCalibration,
    float leftMeasuredSignal,
    float rightMeasuredSignal)
{
    float leftReferenceSignal = 0.0f;
    float rightReferenceSignal = 0.0f;
    if (!wallCalibration.TryComputeSideWallNormalizedReferenceDifferentialLight(WallSensorId::SideLeft, leftReferenceSignal) ||
        !wallCalibration.TryComputeSideWallNormalizedReferenceDifferentialLight(WallSensorId::SideRight, rightReferenceSignal))
    {
        return 0.0f;
    }

    float balanceError = 0.0f;
    if (!MazeMap::TryComputeNormalizedWallSignalBalanceError(
            leftMeasuredSignal,
            leftReferenceSignal,
            rightMeasuredSignal,
            rightReferenceSignal,
            Config::kDiagonalWallMinNormalizedSignal,
            balanceError))
    {
        return 0.0f;
    }

    return Config::kDiagonalWallCenterGain * balanceError;
}

inline bool TryComputeSideWallSignalDistanceM(
    const WallDistanceCalibration& wallCalibration,
    WallSensorId sensorId,
    float measuredSignal,
    float& distanceM)
{
    distanceM = 0.0f;

    float referenceSignal = 0.0f;
    float referenceDistanceM = 0.0f;
    if (!wallCalibration.TryGetSideWallReferenceDifferentialLight(sensorId, referenceSignal) ||
        !wallCalibration.TryGetSideWallReferenceDistanceM(sensorId, referenceDistanceM))
    {
        return false;
    }

    return MazeMap::TryComputeInverseSquareDistanceFromReferenceSignal(
        measuredSignal,
        referenceSignal,
        referenceDistanceM,
        distanceM);
}

inline float ComputeSignalRiseAboveBaselineValue(
    float measuredDifferentialLight,
    float signalBaseline)
{
    // Exclusively for the purpose of centering.
    if (!std::isfinite(measuredDifferentialLight) ||
        !std::isfinite(signalBaseline))
    {
        return 0.0f;
    }

    return (measuredDifferentialLight > signalBaseline) ?
        (measuredDifferentialLight - signalBaseline) :
        0.0f;
}

inline bool IsCalibratedSideDistanceValidForControl(
    const WallDistanceCalibration& wallCalibration,
    WallSensorId sensorId,
    float measuredDifferentialLight)
{
    // Exclusively for the purpose of deciding whether a side distance estimate is trustworthy for control.
    float onMeasuredThreshold = 0.0f;
    float offMeasuredThreshold = 0.0f;
    float signalBaseline = 0.0f;
    if (!wallCalibration.TryComputeSideWallMeasuredThresholds(
            sensorId,
            Config::kSideWallMeasuredSignalLatchThreshold,
            Config::kSideWallMeasuredSignalReleaseThreshold,
            onMeasuredThreshold,
            offMeasuredThreshold,
            signalBaseline))
    {
        return false;
    }

    return ComputeSignalRiseAboveBaselineValue(measuredDifferentialLight, signalBaseline) >= onMeasuredThreshold;
}

inline bool TryComputeStraightWallCenterErrorM(
    const WallDistanceCalibration& wallCalibration,
    float leftMeasuredSignal,
    bool leftWall,
    float rightMeasuredSignal,
    bool rightWall,
    float& corridorErrorM)
{
    corridorErrorM = 0.0f;

    const float expectedDistanceM = wallCalibration.GetExpectedSideWallDistanceM();
    if (!(std::isfinite(expectedDistanceM) && expectedDistanceM > 0.0f))
    {
        return false;
    }

    if (leftWall && rightWall)
    {
        float leftReferenceSignal = 0.0f;
        float rightReferenceSignal = 0.0f;
        float balanceError = 0.0f;
        if (wallCalibration.TryComputeSideWallNormalizedReferenceDifferentialLight(WallSensorId::SideLeft, leftReferenceSignal) &&
            wallCalibration.TryComputeSideWallNormalizedReferenceDifferentialLight(WallSensorId::SideRight, rightReferenceSignal) &&
            MazeMap::TryComputeNormalizedWallSignalBalanceError(
                leftMeasuredSignal,
                leftReferenceSignal,
                rightMeasuredSignal,
                rightReferenceSignal,
                Config::kSideWallMeasuredSignalReleaseThreshold,
                balanceError))
        {
            corridorErrorM = -0.5f * expectedDistanceM * balanceError;
            return std::isfinite(corridorErrorM);
        }
    }

    if (leftWall)
    {
        float leftDistanceM = 0.0f;
        if (TryComputeSideWallSignalDistanceM(
                wallCalibration,
                WallSensorId::SideLeft,
                leftMeasuredSignal,
                leftDistanceM))
        {
            corridorErrorM = leftDistanceM - expectedDistanceM;
            return std::isfinite(corridorErrorM);
        }
    }

    if (rightWall)
    {
        float rightDistanceM = 0.0f;
        if (TryComputeSideWallSignalDistanceM(
                wallCalibration,
                WallSensorId::SideRight,
                rightMeasuredSignal,
                rightDistanceM))
        {
            corridorErrorM = expectedDistanceM - rightDistanceM;
            return std::isfinite(corridorErrorM);
        }
    }

    return false;
}

inline float SignF(float value)
{
    return static_cast<float>((value > 0.0f) - (value < 0.0f));
}

inline float WrapAngleRad(float angle)
{
    return std::remainder(angle, TWO_PI_F);
}

inline float AngleErrorRad(float target, float measured)
{
    return WrapAngleRad(target - measured);
}

inline MazeMap::Vectorf<2> DirectionToUnitVector(MazeMap::Direction direction)
{
    float dx = 0.0f;
    float dy = 0.0f;
    MazeMap::GetHeading(direction, dx, dy);
    return MazeMap::Vectorf<2>(dx, dy);
}

inline float DirectionToYawRad(MazeMap::Direction direction)
{
    switch (direction)
    {
    case MazeMap::Up:
        return HALF_PI_F;
    case MazeMap::UpRight:
        return 0.25f * PI_F;
    case MazeMap::Right:
        return 0.0f;
    case MazeMap::DownRight:
        return -0.25f * PI_F;
    case MazeMap::Down:
        return -HALF_PI_F;
    case MazeMap::DownLeft:
        return -0.75f * PI_F;
    case MazeMap::Left:
        return PI_F;
    case MazeMap::UpLeft:
        return 0.75f * PI_F;
    default:
        return 0.0f;
    }
}

inline MazeMap::Vectorf<2> HeadingUnitFromYawRad(float yawRad)
{
    return MazeMap::Vectorf<2>(cosf(yawRad), sinf(yawRad));
}

inline float HeadingErrorRad(const MazeMap::Vectorf<2>& targetHeading, const MazeMap::Vectorf<2>& measuredHeading)
{
    const float dot = (std::clamp)((targetHeading * measuredHeading), -1.0f, 1.0f);
    const float cross = (measuredHeading.GetX() * targetHeading.GetY()) - (measuredHeading.GetY() * targetHeading.GetX());
    return atan2f(cross, dot);
}

inline constexpr float kStandardGravityMps2 = 9.80665f;
inline constexpr unsigned long kFanRampStepMs = 20UL;
inline float gMissionFanDutyCycle = 0.0f;

inline uint16_t FanPwmCode(float dutyCycle)
{
#if defined(ARDUINO_TEENSY41)
    const float clampedDutyCycle = (std::clamp)(dutyCycle, 0.0f, 1.0f);
    const uint32_t maxPwmCode = (1UL << HardwareConfig::kPwmBits) - 1UL;
    return static_cast<uint16_t>(clampedDutyCycle * static_cast<float>(maxPwmCode) + 0.5f);
#else
    (void)dutyCycle;
    return 0U;
#endif
}

inline void WriteFanDutyCycle(float dutyCycle)
{
    gMissionFanDutyCycle = (std::clamp)(dutyCycle, 0.0f, 1.0f);
#if defined(ARDUINO_TEENSY41)
    analogWrite(Pins::Fan_CTRL, FanPwmCode(dutyCycle));
#else
    (void)dutyCycle;
#endif
}

inline float GetMissionFanDutyCycle()
{
    return gMissionFanDutyCycle;
}

inline void RampFanDutyCycle(float targetDutyCycle)
{
#if defined(ARDUINO_TEENSY41)
    const unsigned long rampDurationMs = static_cast<unsigned long>(Config::kRacingFanRampMs);
    const float clampedTargetDutyCycle = MazeMap::ComputeFanRampDutyCycle(targetDutyCycle, rampDurationMs, rampDurationMs);
    if (clampedTargetDutyCycle <= 0.0f)
    {
        WriteFanDutyCycle(0.0f);
        return;
    }

    if (rampDurationMs == 0UL)
    {
        WriteFanDutyCycle(clampedTargetDutyCycle);
        return;
    }

    WriteFanDutyCycle(0.0f);
    const unsigned long startMs = millis();
    while (true)
    {
        const unsigned long elapsedMs = millis() - startMs;
        if (elapsedMs >= rampDurationMs)
        {
            break;
        }

        WriteFanDutyCycle(MazeMap::ComputeFanRampDutyCycle(clampedTargetDutyCycle, elapsedMs, rampDurationMs));
        delay((std::min)(kFanRampStepMs, rampDurationMs - elapsedMs));
    }

    WriteFanDutyCycle(clampedTargetDutyCycle);
#else
    (void)targetDutyCycle;
#endif
}

inline void SetMissionLevelFanEnabled(bool enabled)
{
    if (enabled)
    {
        RampFanDutyCycle(Config::kRacingFanDutyCycle);
        return;
    }

    WriteFanDutyCycle(0.0f);
}

inline float ReachableSpeedWithBoundary(float boundarySpeed, float distance, float accel)
{
    if (accel <= 0.0f)
    {
        return (std::max)(boundarySpeed, 0.0f);
    }

    return MazeMap::LinearKinematics::V1IgnoringT((std::max)(distance, 0.0f), boundarySpeed, accel);
}

inline MazeMap::Vectorf<2> LeftUnitFromHeading(const MazeMap::Vectorf<2>& headingUnit)
{
    return MazeMap::Vectorf<2>(-headingUnit.GetY(), headingUnit.GetX());
}

inline MazeMap::Vectorf<2> RotateBodyVectorToWorld(const PoseEstimate& pose, const MazeMap::Vectorf<2>& bodyVector)
{
    const MazeMap::Vectorf<2> leftUnit = LeftUnitFromHeading(pose.headingUnit);
    return MazeMap::Vectorf<2>(
        (pose.headingUnit.GetX() * bodyVector.GetX()) + (leftUnit.GetX() * bodyVector.GetY()),
        (pose.headingUnit.GetY() * bodyVector.GetX()) + (leftUnit.GetY() * bodyVector.GetY()));
}

inline MazeMap::Vectorf<2> SensorWorldPosition(const PoseEstimate& pose, const MazeMap::WallSensor& sensor)
{
    const MazeMap::Vectorf<2> worldOffset = RotateBodyVectorToWorld(pose, sensor.GetPosition());
    return MazeMap::Vectorf<2>(pose.xMeters + worldOffset.GetX(), pose.yMeters + worldOffset.GetY());
}

inline MazeMap::Vectorf<2> SensorWorldFacing(const PoseEstimate& pose, const MazeMap::WallSensor& sensor)
{
    return RotateBodyVectorToWorld(pose, sensor.GetFacingDirection());
}

inline bool TryDistanceToWestWall(const PoseEstimate& pose, const MazeMap::WallSensor& sensor, float& distanceM)
{
    const MazeMap::Vectorf<2> sensorPosition = SensorWorldPosition(pose, sensor);
    const MazeMap::Vectorf<2> sensorFacing = SensorWorldFacing(pose, sensor);
    const float westWallXM = MazeMap::ComputeCellInnerMinCoordinateM(Config::kMazeWallThicknessM);
    const float southWallYM = MazeMap::ComputeCellInnerMinCoordinateM(Config::kMazeWallThicknessM);
    const float northWallYM = MazeMap::ComputeCellInnerMaxCoordinateM(Config::kCellSizeM, Config::kMazeWallThicknessM);
    if (sensorFacing.GetX() >= -0.1f)
    {
        return false;
    }

    const float candidateDistanceM = (westWallXM - sensorPosition.GetX()) / sensorFacing.GetX();
    const float intersectionY = sensorPosition.GetY() + (candidateDistanceM * sensorFacing.GetY());
    if (candidateDistanceM <= 0.0f || intersectionY < (southWallYM - 0.005f) || intersectionY > (northWallYM + 0.005f))
    {
        return false;
    }

    distanceM = candidateDistanceM;
    return true;
}

inline bool TryDistanceToEastWall(const PoseEstimate& pose, const MazeMap::WallSensor& sensor, float& distanceM)
{
    const MazeMap::Vectorf<2> sensorPosition = SensorWorldPosition(pose, sensor);
    const MazeMap::Vectorf<2> sensorFacing = SensorWorldFacing(pose, sensor);
    const float eastWallXM = MazeMap::ComputeCellInnerMaxCoordinateM(Config::kCellSizeM, Config::kMazeWallThicknessM);
    const float southWallYM = MazeMap::ComputeCellInnerMinCoordinateM(Config::kMazeWallThicknessM);
    const float northWallYM = MazeMap::ComputeCellInnerMaxCoordinateM(Config::kCellSizeM, Config::kMazeWallThicknessM);
    if (sensorFacing.GetX() <= 0.1f)
    {
        return false;
    }

    const float candidateDistanceM = (eastWallXM - sensorPosition.GetX()) / sensorFacing.GetX();
    const float intersectionY = sensorPosition.GetY() + (candidateDistanceM * sensorFacing.GetY());
    if (candidateDistanceM <= 0.0f || intersectionY < (southWallYM - 0.005f) || intersectionY > (northWallYM + 0.005f))
    {
        return false;
    }

    distanceM = candidateDistanceM;
    return true;
}

inline bool TryDistanceToSouthWall(const PoseEstimate& pose, const MazeMap::WallSensor& sensor, float& distanceM)
{
    const MazeMap::Vectorf<2> sensorPosition = SensorWorldPosition(pose, sensor);
    const MazeMap::Vectorf<2> sensorFacing = SensorWorldFacing(pose, sensor);
    const float southWallYM = MazeMap::ComputeCellInnerMinCoordinateM(Config::kMazeWallThicknessM);
    const float westWallXM = MazeMap::ComputeCellInnerMinCoordinateM(Config::kMazeWallThicknessM);
    const float eastWallXM = MazeMap::ComputeCellInnerMaxCoordinateM(Config::kCellSizeM, Config::kMazeWallThicknessM);
    if (sensorFacing.GetY() >= -0.1f)
    {
        return false;
    }

    const float candidateDistanceM = (southWallYM - sensorPosition.GetY()) / sensorFacing.GetY();
    const float intersectionX = sensorPosition.GetX() + (candidateDistanceM * sensorFacing.GetX());
    if (candidateDistanceM <= 0.0f || intersectionX < (westWallXM - 0.005f) || intersectionX > (eastWallXM + 0.005f))
    {
        return false;
    }

    distanceM = candidateDistanceM;
    return true;
}

inline bool TryDistanceToNorthWall(const PoseEstimate& pose, const MazeMap::WallSensor& sensor, float& distanceM)
{
    const MazeMap::Vectorf<2> sensorPosition = SensorWorldPosition(pose, sensor);
    const MazeMap::Vectorf<2> sensorFacing = SensorWorldFacing(pose, sensor);
    const float northWallYM = MazeMap::ComputeCellInnerMaxCoordinateM(Config::kCellSizeM, Config::kMazeWallThicknessM);
    const float westWallXM = MazeMap::ComputeCellInnerMinCoordinateM(Config::kMazeWallThicknessM);
    const float eastWallXM = MazeMap::ComputeCellInnerMaxCoordinateM(Config::kCellSizeM, Config::kMazeWallThicknessM);
    if (sensorFacing.GetY() <= 0.1f)
    {
        return false;
    }

    const float candidateDistanceM = (northWallYM - sensorPosition.GetY()) / sensorFacing.GetY();
    const float intersectionX = sensorPosition.GetX() + (candidateDistanceM * sensorFacing.GetX());
    if (candidateDistanceM <= 0.0f || intersectionX < (westWallXM - 0.005f) || intersectionX > (eastWallXM + 0.005f))
    {
        return false;
    }

    distanceM = candidateDistanceM;
    return true;
}

inline bool TryComputeNearestStartCellWallDistanceM(const PoseEstimate& pose, const MazeMap::WallSensor& sensor, float& distanceM)
{
    distanceM = 0.0f;
    float bestDistanceM = INFINITY;
    float candidateDistanceM = 0.0f;
    if (TryDistanceToWestWall(pose, sensor, candidateDistanceM) && candidateDistanceM < bestDistanceM)
    {
        bestDistanceM = candidateDistanceM;
    }
    if (TryDistanceToEastWall(pose, sensor, candidateDistanceM) && candidateDistanceM < bestDistanceM)
    {
        bestDistanceM = candidateDistanceM;
    }
    if (TryDistanceToSouthWall(pose, sensor, candidateDistanceM) && candidateDistanceM < bestDistanceM)
    {
        bestDistanceM = candidateDistanceM;
    }

    if (!std::isfinite(bestDistanceM) || !(bestDistanceM > 0.0f))
    {
        return false;
    }

    distanceM = bestDistanceM;
    return true;
}

inline bool TryComputeEffectiveTurnRadiusM(
    float leftDistanceM,
    float rightDistanceM,
    float yawChangeRad,
    float& turnRadiusM)
{
    if (!std::isfinite(leftDistanceM) ||
        !std::isfinite(rightDistanceM) ||
        !std::isfinite(yawChangeRad) ||
        std::fabs(yawChangeRad) < 1.0e-4f)
    {
        turnRadiusM = 0.0f;
        return false;
    }

    turnRadiusM = std::fabs(0.5f * (leftDistanceM + rightDistanceM) / yawChangeRad);
    return std::isfinite(turnRadiusM) && (turnRadiusM > 0.0f);
}

inline bool TryGetCellCenterMeters(const MazeMap::CellCoordinates& cell, float& xMeters, float& yMeters)
{
    MazeMap::MazeLocation::CellCenter(cell).GetPhysicalLocation(Config::kCellSizeM, xMeters, yMeters);
    return std::isfinite(xMeters) && std::isfinite(yMeters);
}

inline bool TryGetCellWallFaceCoordinateM(
    const MazeMap::CellCoordinates& cell,
    MazeMap::Direction wallDirection,
    float& coordinateM)
{
    coordinateM = 0.0f;
    const float cellBaseXM = static_cast<float>(cell.GetX()) * Config::kCellSizeM;
    const float cellBaseYM = static_cast<float>(cell.GetY()) * Config::kCellSizeM;
    switch (wallDirection)
    {
    case MazeMap::Left:
        coordinateM = cellBaseXM + MazeMap::ComputeCellInnerMinCoordinateM(Config::kMazeWallThicknessM);
        return true;
    case MazeMap::Right:
        coordinateM = cellBaseXM + MazeMap::ComputeCellInnerMaxCoordinateM(Config::kCellSizeM, Config::kMazeWallThicknessM);
        return true;
    case MazeMap::Down:
        coordinateM = cellBaseYM + MazeMap::ComputeCellInnerMinCoordinateM(Config::kMazeWallThicknessM);
        return true;
    case MazeMap::Up:
        coordinateM = cellBaseYM + MazeMap::ComputeCellInnerMaxCoordinateM(Config::kCellSizeM, Config::kMazeWallThicknessM);
        return true;
    default:
        return false;
    }
}

inline bool TryComputeDistanceToCellWallM(
    const PoseEstimate& pose,
    const MazeMap::WallSensor& sensor,
    const MazeMap::CellCoordinates& cell,
    MazeMap::Direction wallDirection,
    float& distanceM)
{
    distanceM = 0.0f;

    float wallCoordinateM = 0.0f;
    if (!TryGetCellWallFaceCoordinateM(cell, wallDirection, wallCoordinateM))
    {
        return false;
    }

    const MazeMap::Vectorf<2> sensorPosition = SensorWorldPosition(pose, sensor);
    const MazeMap::Vectorf<2> sensorFacing = SensorWorldFacing(pose, sensor);
    const float cellBaseXM = static_cast<float>(cell.GetX()) * Config::kCellSizeM;
    const float cellBaseYM = static_cast<float>(cell.GetY()) * Config::kCellSizeM;
    const float cellInnerMinXM = cellBaseXM + MazeMap::ComputeCellInnerMinCoordinateM(Config::kMazeWallThicknessM);
    const float cellInnerMaxXM = cellBaseXM + MazeMap::ComputeCellInnerMaxCoordinateM(Config::kCellSizeM, Config::kMazeWallThicknessM);
    const float cellInnerMinYM = cellBaseYM + MazeMap::ComputeCellInnerMinCoordinateM(Config::kMazeWallThicknessM);
    const float cellInnerMaxYM = cellBaseYM + MazeMap::ComputeCellInnerMaxCoordinateM(Config::kCellSizeM, Config::kMazeWallThicknessM);

    switch (wallDirection)
    {
    case MazeMap::Left:
    case MazeMap::Right:
    {
        if (((wallDirection == MazeMap::Left) && sensorFacing.GetX() >= -0.1f) ||
            ((wallDirection == MazeMap::Right) && sensorFacing.GetX() <= 0.1f))
        {
            return false;
        }

        const float candidateDistanceM = (wallCoordinateM - sensorPosition.GetX()) / sensorFacing.GetX();
        const float intersectionYM = sensorPosition.GetY() + (candidateDistanceM * sensorFacing.GetY());
        if (candidateDistanceM <= 0.0f ||
            intersectionYM < (cellInnerMinYM - 0.005f) ||
            intersectionYM > (cellInnerMaxYM + 0.005f))
        {
            return false;
        }

        distanceM = candidateDistanceM;
        return std::isfinite(distanceM) && distanceM > 0.0f;
    }
    case MazeMap::Down:
    case MazeMap::Up:
    {
        if (((wallDirection == MazeMap::Down) && sensorFacing.GetY() >= -0.1f) ||
            ((wallDirection == MazeMap::Up) && sensorFacing.GetY() <= 0.1f))
        {
            return false;
        }

        const float candidateDistanceM = (wallCoordinateM - sensorPosition.GetY()) / sensorFacing.GetY();
        const float intersectionXM = sensorPosition.GetX() + (candidateDistanceM * sensorFacing.GetX());
        if (candidateDistanceM <= 0.0f ||
            intersectionXM < (cellInnerMinXM - 0.005f) ||
            intersectionXM > (cellInnerMaxXM + 0.005f))
        {
            return false;
        }

        distanceM = candidateDistanceM;
        return std::isfinite(distanceM) && distanceM > 0.0f;
    }
    default:
        return false;
    }
}

inline bool TryComputeFrontWallCandidateDistancesForPose(
    const PoseEstimate& pose,
    const MazeMap::Vehicle& vehicle,
    const MazeMap::CellCoordinates& observedCell,
    MazeMap::Direction observedDirection,
    float& frontLeftDistanceM,
    float& frontRightDistanceM)
{
    frontLeftDistanceM = NAN;
    frontRightDistanceM = NAN;
    const MazeMap::Direction forwardDirection = observedDirection + MazeMap::Forward;
    const bool haveFrontLeftDistance =
        TryComputeDistanceToCellWallM(
            pose,
            vehicle.FrontLeft,
            observedCell,
            forwardDirection,
            frontLeftDistanceM);
    const bool haveFrontRightDistance =
        TryComputeDistanceToCellWallM(
            pose,
            vehicle.FrontRight,
            observedCell,
            forwardDirection,
            frontRightDistanceM);
    return haveFrontLeftDistance || haveFrontRightDistance;
}

inline void ClearFrontWallObservationDecision(SensorSnapshot& snapshot)
{
    snapshot.frontWall = false;
    snapshot.frontLeftWall = false;
    snapshot.frontRightWall = false;
    snapshot.frontWallObservationValid = false;
    snapshot.frontWallUsesFallbackDetection = false;
    snapshot.frontWallUsesCharacterizationDetection = false;
}

inline bool TryComputeFrontWallObservationSampleDistanceM(
    const MazeMap::Vehicle& vehicle,
    const MazeMap::WallSensor& sensor,
    uint8_t sampleIndex,
    float& distanceM)
{
    distanceM = 0.0f;

    float poseXM = 0.0f;
    float poseYM = 0.0f;
    const float sideSensorForwardOffsetM =
        (std::max)(vehicle.SideLeft.GetPosition().GetX(), vehicle.SideRight.GetPosition().GetX());
    const MazeMap::CellCoordinates observedCell(0, 0);
    if (!MazeMap::TryComputeSideWallObservationSamplePoseM(
            observedCell,
            MazeMap::Up,
            Config::kCellSizeM,
            Config::kMazeWallThicknessM,
            sideSensorForwardOffsetM,
            Config::kSideWallSegmentCenterFraction,
            sampleIndex,
            Config::kSearchRollingObservationSampleCount,
            poseXM,
            poseYM))
    {
        return false;
    }

    PoseEstimate pose{};
    pose.xMeters = poseXM;
    pose.yMeters = poseYM;
    pose.headingUnit = DirectionToUnitVector(MazeMap::Up);
    pose.yawRad = DirectionToYawRad(MazeMap::Up);
    pose.linearSpeedMps = 0.0f;
    pose.angularSpeedRadps = 0.0f;
    return TryComputeDistanceToCellWallM(pose, sensor, observedCell, MazeMap::Up, distanceM);
}

inline bool TryComputeFrontWallObservationThresholdDistancesM(
    const MazeMap::Vehicle& vehicle,
    WallSensorId sensorId,
    float releaseHysteresisDistanceM,
    float& onThresholdM,
    float& offThresholdM)
{
    onThresholdM = 0.0f;
    offThresholdM = 0.0f;
    if (!IsFrontWallSensor(sensorId))
    {
        return false;
    }

    const MazeMap::WallSensor& sensor =
        (sensorId == WallSensorId::FrontLeft) ?
        vehicle.FrontLeft :
        vehicle.FrontRight;
    constexpr uint8_t kLatchSampleIndex =
        Config::kSearchRollingObservationSampleCount - Config::kSearchRollingObservationMajorityCount;
    float preferredOnThresholdM = 0.0f;
    float farthestObservationThresholdM = 0.0f;
    if (!TryComputeFrontWallObservationSampleDistanceM(
            vehicle,
            sensor,
            kLatchSampleIndex,
            preferredOnThresholdM) ||
        !TryComputeFrontWallObservationSampleDistanceM(
            vehicle,
            sensor,
            0U,
            farthestObservationThresholdM))
    {
        return false;
    }

    float preferredOffThresholdM = 0.0f;
    if (!MazeMap::TryExpandWallThresholdDistanceM(
            preferredOnThresholdM,
            releaseHysteresisDistanceM,
            preferredOffThresholdM))
    {
        return false;
    }

    if (std::isfinite(farthestObservationThresholdM) &&
        farthestObservationThresholdM > preferredOnThresholdM &&
        preferredOffThresholdM > farthestObservationThresholdM)
    {
        preferredOffThresholdM = farthestObservationThresholdM;
    }

    return MazeMap::TryClampWallThresholdDistanceRangeM(
        preferredOnThresholdM,
        preferredOffThresholdM,
        Config::kFrontWallOnThresholdM,
        Config::kFrontWallOffThresholdM,
        onThresholdM,
        offThresholdM);
}

inline bool TryComputeWallTouchTargetCoordinateForCellWall(
    const MazeMap::CellCoordinates& cell,
    MazeMap::Direction wallDirection,
    float& targetCoordinateM,
    CalibrationWall& calibrationWall)
{
    targetCoordinateM = 0.0f;
    float wallFaceCoordinateM = 0.0f;
    if (!TryGetCellWallFaceCoordinateM(cell, wallDirection, wallFaceCoordinateM))
    {
        return false;
    }

    switch (wallDirection)
    {
    case MazeMap::Left:
        calibrationWall = CalibrationWall::West;
        targetCoordinateM = wallFaceCoordinateM + Config::kWallTouchContactStandoffM;
        return true;
    case MazeMap::Right:
        calibrationWall = CalibrationWall::East;
        targetCoordinateM = wallFaceCoordinateM - Config::kWallTouchContactStandoffM;
        return true;
    case MazeMap::Down:
        calibrationWall = CalibrationWall::South;
        targetCoordinateM = wallFaceCoordinateM + Config::kWallTouchContactStandoffM;
        return true;
    case MazeMap::Up:
        calibrationWall = CalibrationWall::North;
        targetCoordinateM = wallFaceCoordinateM - Config::kWallTouchContactStandoffM;
        return true;
    default:
        return false;
    }
}

inline bool TryComputePoseAxisFromObservedWall(
    const PoseEstimate& pose,
    const MazeMap::WallSensor& sensor,
    float measuredDistanceM,
    const MazeMap::CellCoordinates& cell,
    MazeMap::Direction wallDirection,
    float& coordinateM)
{
    coordinateM = 0.0f;
    if (!std::isfinite(measuredDistanceM) || measuredDistanceM <= 0.0f)
    {
        return false;
    }

    float wallCoordinateM = 0.0f;
    if (!TryGetCellWallFaceCoordinateM(cell, wallDirection, wallCoordinateM))
    {
        return false;
    }

    const MazeMap::Vectorf<2> worldOffset = RotateBodyVectorToWorld(pose, sensor.GetPosition());
    const MazeMap::Vectorf<2> sensorFacing = SensorWorldFacing(pose, sensor);
    if (wallDirection == MazeMap::Left || wallDirection == MazeMap::Right)
    {
        if (!std::isfinite(worldOffset.GetX()) ||
            !std::isfinite(sensorFacing.GetX()) ||
            ((wallDirection == MazeMap::Left) && sensorFacing.GetX() >= -0.1f) ||
            ((wallDirection == MazeMap::Right) && sensorFacing.GetX() <= 0.1f))
        {
            return false;
        }

        coordinateM = wallCoordinateM - worldOffset.GetX() - (measuredDistanceM * sensorFacing.GetX());
        return std::isfinite(coordinateM);
    }

    if (!std::isfinite(worldOffset.GetY()) ||
        !std::isfinite(sensorFacing.GetY()) ||
        ((wallDirection == MazeMap::Down) && sensorFacing.GetY() >= -0.1f) ||
        ((wallDirection == MazeMap::Up) && sensorFacing.GetY() <= 0.1f))
    {
        return false;
    }

    coordinateM = wallCoordinateM - worldOffset.GetY() - (measuredDistanceM * sensorFacing.GetY());
    return std::isfinite(coordinateM);
}

inline uint32_t WallSensorAmbientSettleTimeUs(WallSensorId sensorId)
{
    return IsFrontWallSensor(sensorId) ? HardwareConfig::kFrontWallSensorSwitchSettleTime_us : HardwareConfig::kSideWallSensorSwitchSettleTime_us;
}

inline uint32_t WallSensorLitSettleTimeUs(WallSensorId sensorId)
{
    return IsFrontWallSensor(sensorId) ? HardwareConfig::kFrontWallSensorSwitchSettleTime_us : HardwareConfig::kSideWallSensorSwitchSettleTime_us;
}

inline uint32_t WallSensorLedCalibrationHalfPeriodUs(WallSensorId sensorId)
{
    return (std::max)(WallSensorAmbientSettleTimeUs(sensorId), WallSensorLitSettleTimeUs(sensorId));
}

inline float WallSensorMeasuredValueForCalibration(WallSensorId sensorId, const RawWallSensorSample& sample)
{
    return IsFrontWallSensor(sensorId) ? sample.differentialLight : sample.rawDistanceM;
}

inline WallSensorCalibrationInput BuildWallSensorCalibrationInput(WallSensorId sensorId, const RawWallSensorSample& sample)
{
    WallSensorCalibrationInput input{};
    input.measuredValue = WallSensorMeasuredValueForCalibration(sensorId, sample);
    input.fallbackDistanceM = sample.rawDistanceM;
    input.differentialLight = sample.differentialLight;
    input.ambientLight = sample.ambientLight;
    input.litLight = sample.litLight;
    return input;
}

inline RawWallSensorSample SampleWallSensorRaw(WallSensorId sensorId, const MazeMap::WallSensor& sensor)
{
    RawWallSensorSample sample{};
    sensor.SetLedEnabled(false);
    delayMicroseconds(WallSensorAmbientSettleTimeUs(sensorId));
    sample.ambientLight = sensor.ReadLightLevel();

    sensor.SetLedEnabled(true);
    delayMicroseconds(WallSensorLitSettleTimeUs(sensorId));
    sample.litLight = sensor.ReadLightLevel();
    sample.differentialLight = MazeMap::WallSensor::DifferentialLightLevel(sample.ambientLight, sample.litLight);
    sample.rawDistanceM = sensor.DistanceFromDifferentialLight(sample.differentialLight);
    sensor.SetLedEnabled(false);
    return sample;
}

inline void SampleWallSensorPairRaw(
    WallSensorId firstSensorId,
    const MazeMap::WallSensor& firstSensor,
    WallSensorId secondSensorId,
    const MazeMap::WallSensor& secondSensor,
    RawWallSensorSample& firstSample,
    RawWallSensorSample& secondSample)
{
    firstSensor.SetLedEnabled(false);
    secondSensor.SetLedEnabled(false);
    delayMicroseconds((std::max)(WallSensorAmbientSettleTimeUs(firstSensorId), WallSensorAmbientSettleTimeUs(secondSensorId)));
    firstSample.ambientLight = firstSensor.ReadLightLevel();
    secondSample.ambientLight = secondSensor.ReadLightLevel();

    firstSensor.SetLedEnabled(true);
    secondSensor.SetLedEnabled(true);
    delayMicroseconds((std::max)(WallSensorLitSettleTimeUs(firstSensorId), WallSensorLitSettleTimeUs(secondSensorId)));
    firstSample.litLight = firstSensor.ReadLightLevel();
    firstSample.differentialLight = MazeMap::WallSensor::DifferentialLightLevel(firstSample.ambientLight, firstSample.litLight);
    firstSample.rawDistanceM = firstSensor.DistanceFromDifferentialLight(firstSample.differentialLight);
    secondSample.litLight = secondSensor.ReadLightLevel();
    secondSample.differentialLight = MazeMap::WallSensor::DifferentialLightLevel(secondSample.ambientLight, secondSample.litLight);
    secondSample.rawDistanceM = secondSensor.DistanceFromDifferentialLight(secondSample.differentialLight);
    firstSensor.SetLedEnabled(false);
    secondSensor.SetLedEnabled(false);
}

inline WallSensorCalibrationInput SampleWallCalibrationInputRaw(WallSensorId sensorId, const MazeMap::WallSensor& sensor)
{
    return BuildWallSensorCalibrationInput(sensorId, SampleWallSensorRaw(sensorId, sensor));
}

inline void SampleWallCalibrationInputRawPair(
    WallSensorId firstSensorId,
    const MazeMap::WallSensor& firstSensor,
    WallSensorId secondSensorId,
    const MazeMap::WallSensor& secondSensor,
    WallSensorCalibrationInput& firstInput,
    WallSensorCalibrationInput& secondInput)
{
    RawWallSensorSample firstSample{};
    RawWallSensorSample secondSample{};
    SampleWallSensorPairRaw(firstSensorId, firstSensor, secondSensorId, secondSensor, firstSample, secondSample);
    firstInput = BuildWallSensorCalibrationInput(firstSensorId, firstSample);
    secondInput = BuildWallSensorCalibrationInput(secondSensorId, secondSample);
}

inline WallSensorCalibrationCapture SampleWallCalibrationCaptureAverageRaw(WallSensorId sensorId, const MazeMap::WallSensor& sensor)
{
    AveragedWallSensorInputWindow<static_cast<uint8_t>(Config::kWallCalibrationAverageSampleCount)> averageWindow{};
    std::array<float, Config::kWallCalibrationAverageSampleCount> differentialLightSamples{};
    uint16_t differentialLightCount = 0U;
    for (uint16_t i = 0U; i < Config::kWallCalibrationAverageSampleCount; ++i)
    {
        const WallSensorCalibrationInput input = SampleWallCalibrationInputRaw(sensorId, sensor);
        averageWindow.PushAndAverage(input);
        differentialLightSamples[differentialLightCount] = input.differentialLight;
        ++differentialLightCount;
    }

    WallSensorCalibrationCapture capture{};
    capture.input = averageWindow.Average();
    capture.haveDifferentialLightBand = MazeMap::TryComputeRobustSignalBandFromSamples(
        differentialLightSamples,
        differentialLightCount,
        Config::kWallCalibrationScaledMadMultiplier,
        capture.differentialLightBand.median,
        capture.differentialLightBand.low,
        capture.differentialLightBand.high);
    return capture;
}

inline void SampleWallCalibrationCaptureAverageRawPair(
    WallSensorId firstSensorId,
    const MazeMap::WallSensor& firstSensor,
    WallSensorId secondSensorId,
    const MazeMap::WallSensor& secondSensor,
    WallSensorCalibrationCapture& firstCapture,
    WallSensorCalibrationCapture& secondCapture)
{
    AveragedWallSensorInputWindow<static_cast<uint8_t>(Config::kWallCalibrationAverageSampleCount)> firstAverageWindow{};
    AveragedWallSensorInputWindow<static_cast<uint8_t>(Config::kWallCalibrationAverageSampleCount)> secondAverageWindow{};
    std::array<float, Config::kWallCalibrationAverageSampleCount> firstDifferentialLightSamples{};
    std::array<float, Config::kWallCalibrationAverageSampleCount> secondDifferentialLightSamples{};
    uint16_t differentialLightCount = 0U;
    for (uint16_t i = 0U; i < Config::kWallCalibrationAverageSampleCount; ++i)
    {
        WallSensorCalibrationInput firstInput{};
        WallSensorCalibrationInput secondInput{};
        SampleWallCalibrationInputRawPair(
            firstSensorId,
            firstSensor,
            secondSensorId,
            secondSensor,
            firstInput,
            secondInput);
        firstAverageWindow.PushAndAverage(firstInput);
        secondAverageWindow.PushAndAverage(secondInput);
        firstDifferentialLightSamples[differentialLightCount] = firstInput.differentialLight;
        secondDifferentialLightSamples[differentialLightCount] = secondInput.differentialLight;
        ++differentialLightCount;
    }

    firstCapture = {};
    firstCapture.input = firstAverageWindow.Average();
    firstCapture.haveDifferentialLightBand = MazeMap::TryComputeRobustSignalBandFromSamples(
        firstDifferentialLightSamples,
        differentialLightCount,
        Config::kWallCalibrationScaledMadMultiplier,
        firstCapture.differentialLightBand.median,
        firstCapture.differentialLightBand.low,
        firstCapture.differentialLightBand.high);

    secondCapture = {};
    secondCapture.input = secondAverageWindow.Average();
    secondCapture.haveDifferentialLightBand = MazeMap::TryComputeRobustSignalBandFromSamples(
        secondDifferentialLightSamples,
        differentialLightCount,
        Config::kWallCalibrationScaledMadMultiplier,
        secondCapture.differentialLightBand.median,
        secondCapture.differentialLightBand.low,
        secondCapture.differentialLightBand.high);
}

inline bool HysteresisWall(bool currentState, float distanceM, float onThresholdM, float offThresholdM)
{
    if (currentState)
    {
        return distanceM < offThresholdM;
    }
    return distanceM < onThresholdM;
}

inline bool IsApproximatelyDiagonalHeadingUnit(const MazeMap::Vectorf<2>& headingUnit)
{
    const float absX = std::fabs(headingUnit.GetX());
    const float absY = std::fabs(headingUnit.GetY());
    return absX > 0.5f && absY > 0.5f && std::fabs(absX - absY) <= 0.15f;
}

inline MazeMap::ManeuverCode RelativeToInPlaceCode(MazeMap::RelativeDirection rel)
{
    switch (rel)
    {
    case MazeMap::Left45:
        return MazeMap::IP45_M;
    case MazeMap::Right45:
        return MazeMap::IP45;
    case MazeMap::Left90:
        return MazeMap::IP90_M;
    case MazeMap::Right90:
        return MazeMap::IP90;
    case MazeMap::Left135:
        return MazeMap::IP135_M;
    case MazeMap::Right135:
        return MazeMap::IP135;
    case MazeMap::Reverse:
        return MazeMap::IP180;
    default:
        return MazeMap::MC_NONE;
    }
}

inline bool IsStraightCode(MazeMap::ManeuverCode code)
{
    return code != MazeMap::MC_NONE && code <= MazeMap::S31;
}

inline void TrimAsciiWhitespace(char* text)
{
    if (text == nullptr)
    {
        return;
    }

    char* start = text;
    while (*start != '\0' && isspace(static_cast<unsigned char>(*start)) != 0)
    {
        ++start;
    }

    if (start != text)
    {
        memmove(text, start, strlen(start) + 1U);
    }

    size_t length = strlen(text);
    while (length > 0U && isspace(static_cast<unsigned char>(text[length - 1U])) != 0)
    {
        text[length - 1U] = '\0';
        --length;
    }
}

inline void NormalizeToken(char* text)
{
    TrimAsciiWhitespace(text);
    if (text == nullptr)
    {
        return;
    }

    for (char* cursor = text; *cursor != '\0'; ++cursor)
    {
        *cursor = static_cast<char>(toupper(static_cast<unsigned char>(*cursor)));
    }
}

inline bool TryParseBaseManeuverCodeName(const char* token, MazeMap::ManeuverCode& code)
{
    if (token == nullptr || token[0] == '\0')
    {
        return false;
    }

    if (token[0] == 'S' && isdigit(static_cast<unsigned char>(token[1])) != 0)
    {
        char* end = nullptr;
        const long value = strtol(token + 1, &end, 10);
        if (end != nullptr && *end == '\0' && value >= 1L && value <= 31L)
        {
            code = static_cast<MazeMap::ManeuverCode>(value);
            return true;
        }
    }

    struct NamedCode
    {
        const char* name;
        MazeMap::ManeuverCode code;
    };

    static const NamedCode kNamedCodes[] = {
        { "IP45", MazeMap::IP45 },
        { "IP90", MazeMap::IP90 },
        { "IP135", MazeMap::IP135 },
        { "IP180", MazeMap::IP180 },
        { "S45SS", MazeMap::S45SS },
        { "S45SD", MazeMap::S45SD },
        { "S45LS", MazeMap::S45LS },
        { "S45LD", MazeMap::S45LD },
        { "S90SS", MazeMap::S90SS },
        { "S90SD", MazeMap::S90SD },
        { "S90LS", MazeMap::S90LS },
        { "S90LD", MazeMap::S90LD },
        { "S135SS", MazeMap::S135SS },
        { "S135SD", MazeMap::S135SD },
        { "S135LS", MazeMap::S135LS },
        { "S135LD", MazeMap::S135LD },
        { "S180SS", MazeMap::S180SS },
        { "S180LS", MazeMap::S180LS },
        { "S90ELD", MazeMap::S90ELD },
        { "S180ELS", MazeMap::S180ELS },
    };

    for (const NamedCode& entry : kNamedCodes)
    {
        if (strcmp(token, entry.name) == 0)
        {
            code = entry.code;
            return true;
        }
    }

    return false;
}

inline bool TryParseManeuverCodeToken(const char* token, MazeMap::ManeuverCode& code)
{
    if (token == nullptr)
    {
        return false;
    }

    char normalized[24] = {};
    snprintf(normalized, sizeof(normalized), "%s", token);
    NormalizeToken(normalized);
    if (normalized[0] == '\0')
    {
        return false;
    }

    char* numericEnd = nullptr;
    const long numericValue = strtol(normalized, &numericEnd, 0);
    if (numericEnd != nullptr && *numericEnd == '\0' && numericValue >= 0L && numericValue <= 255L)
    {
        code = static_cast<MazeMap::ManeuverCode>(numericValue);
        return true;
    }

    const size_t length = strlen(normalized);
    if (length > 2U && strcmp(normalized + length - 2U, "_M") == 0)
    {
        normalized[length - 2U] = '\0';
        MazeMap::ManeuverCode baseCode = MazeMap::MC_NONE;
        if (!TryParseBaseManeuverCodeName(normalized, baseCode))
        {
            return false;
        }

        code = static_cast<MazeMap::ManeuverCode>(
            static_cast<uint8_t>(baseCode) |
            static_cast<uint8_t>(MazeMap::MIRRORED_MANEUVER_FLAG));
        return true;
    }

    return TryParseBaseManeuverCodeName(normalized, code);
}

inline void FormatManeuverCodeName(MazeMap::ManeuverCode code, char* buffer, size_t bufferSize)
{
    if (buffer == nullptr || bufferSize == 0U)
    {
        return;
    }

    const bool mirrored = (code & MazeMap::MIRRORED_MANEUVER_FLAG) == MazeMap::MIRRORED_MANEUVER_FLAG;
    const MazeMap::ManeuverCode baseCode = code & MazeMap::INVERTED_MIRRORED_MANEUVER_FLAG;

    if (baseCode >= MazeMap::S1 && baseCode <= MazeMap::S31)
    {
        snprintf(
            buffer,
            bufferSize,
            "S%u%s",
            static_cast<unsigned>(static_cast<uint8_t>(baseCode)),
            mirrored ? "_M" : "");
        return;
    }

    const char* name = "UNKNOWN";
    switch (baseCode)
    {
    case MazeMap::IP45: name = "IP45"; break;
    case MazeMap::IP90: name = "IP90"; break;
    case MazeMap::IP135: name = "IP135"; break;
    case MazeMap::IP180: name = "IP180"; break;
    case MazeMap::S45SS: name = "S45SS"; break;
    case MazeMap::S45SD: name = "S45SD"; break;
    case MazeMap::S45LS: name = "S45LS"; break;
    case MazeMap::S45LD: name = "S45LD"; break;
    case MazeMap::S90SS: name = "S90SS"; break;
    case MazeMap::S90SD: name = "S90SD"; break;
    case MazeMap::S90LS: name = "S90LS"; break;
    case MazeMap::S90LD: name = "S90LD"; break;
    case MazeMap::S135SS: name = "S135SS"; break;
    case MazeMap::S135SD: name = "S135SD"; break;
    case MazeMap::S135LS: name = "S135LS"; break;
    case MazeMap::S135LD: name = "S135LD"; break;
    case MazeMap::S180SS: name = "S180SS"; break;
    case MazeMap::S180LS: name = "S180LS"; break;
    case MazeMap::S90ELD: name = "S90ELD"; break;
    case MazeMap::S180ELS: name = "S180ELS"; break;
    default: break;
    }

    snprintf(buffer, bufferSize, "%s%s", name, mirrored ? "_M" : "");
}

inline float ReadBackLeftGyroZRadpsRaw(MazeMap::Vehicle& vehicle)
{
#if defined(ARDUINO_TEENSY41)
    const float blDps = vehicle.IMU_BL.GyroRawToDps(vehicle.IMU_BL.ReadGyroZ());
    return blDps * DEG_TO_RAD_F;
#else
    (void)vehicle;
    return 0.0f;
#endif
}

inline float EstimateMissionGyroBiasRadps(MazeMap::Vehicle& vehicle)
{
    float accumulatedRadps = 0.0f;
    constexpr unsigned long kBiasSampleIntervalMs = 2UL;
    const unsigned long requiredSamples = MazeMap::ComputeGyroBiasSampleCount(
        static_cast<unsigned long>(Config::kGyroBiasSamples),
        kBiasSampleIntervalMs,
        static_cast<unsigned long>(Config::kGyroBiasMinimumAveragingWindowMs));
    for (unsigned long i = 0UL; i < requiredSamples; ++i)
    {
        accumulatedRadps += ReadBackLeftGyroZRadpsRaw(vehicle);
        delay(kBiasSampleIntervalMs);
    }
    return accumulatedRadps / static_cast<float>(requiredSamples);
}

inline bool CalibrateStationaryBackLeftGyroBias(
    MazeMap::Vehicle& vehicle,
    unsigned long controlPeriodUs,
    bool enableAccelRuntime,
    float& gyroBiasRadps,
    MazeMap::Vehicle::ImuBackLeft::ACCEL_FILTER_FREQ accelFilterFreq =
        MazeMap::Vehicle::ImuBackLeft::ACCEL_FILTER_FREQ::FRAC_1_400)
{
#if defined(ARDUINO_TEENSY41)
    while (true)
    {
        const MazeMap::EncoderCountPair startCounts = CaptureDriveEncoderCounts();
        const StationaryImuCalibrationResult selfTestResult =
            RunStationaryBackLeftImuSelfTest(vehicle.IMU_BL, controlPeriodUs, startCounts, accelFilterFreq);
        if (selfTestResult == StationaryImuCalibrationResult::RestartEncoderMotion)
        {
            Serial.println("Encoder motion detected during stationary IMU self-test; restarting bias calibration");
            continue;
        }
        if (selfTestResult != StationaryImuCalibrationResult::Success)
        {
            return false;
        }

        if (!ConfigureBackLeftImuForRuntime(vehicle.IMU_BL, controlPeriodUs, enableAccelRuntime, accelFilterFreq))
        {
            return false;
        }

        const unsigned long requiredSamples = MazeMap::ComputeGyroBiasSampleCount(
            static_cast<unsigned long>(Config::kGyroBiasSamples),
            kImuCalibrationSampleIntervalMs,
            static_cast<unsigned long>(Config::kGyroBiasMinimumAveragingWindowMs));
        const unsigned long measurementStartMs = millis();
        double accumulatedRadps = 0.0;
        unsigned long collectedSamples = 0UL;
        while ((collectedSamples < requiredSamples) ||
            ((millis() - measurementStartMs) < static_cast<unsigned long>(Config::kGyroBiasMinimumAveragingWindowMs)))
        {
            if (HaveDriveEncodersMovedSince(startCounts))
            {
                Serial.println("Encoder motion detected during gyro bias measurement; restarting IMU self-test");
                accumulatedRadps = 0.0;
                collectedSamples = 0UL;
                break;
            }

            accumulatedRadps += static_cast<double>(ReadBackLeftGyroZRadpsRaw(vehicle));
            ++collectedSamples;
            delay(kImuCalibrationSampleIntervalMs);
        }

        if (collectedSamples == 0UL)
        {
            continue;
        }

        if (HaveDriveEncodersMovedSince(startCounts))
        {
            Serial.println("Encoder motion detected after gyro bias capture; restarting IMU self-test");
            continue;
        }

        gyroBiasRadps = static_cast<float>(accumulatedRadps / static_cast<double>(collectedSamples));
        return true;
    }
#else
    (void)controlPeriodUs;
    (void)enableAccelRuntime;
    (void)accelFilterFreq;
    gyroBiasRadps = EstimateMissionGyroBiasRadps(vehicle);
    return true;
#endif
}

inline bool TryComputeSideWallAimCoordinateM(
    const PoseEstimate& pose,
    const MazeMap::WallSensor& sensor,
    float& alongWallCoordinateM)
{
    alongWallCoordinateM = 0.0f;
    if (!std::isfinite(pose.xMeters) ||
        !std::isfinite(pose.yMeters) ||
        !std::isfinite(pose.yawRad))
    {
        return false;
    }

    const float yawCos = std::cos(pose.yawRad);
    const float yawSin = std::sin(pose.yawRad);
    const MazeMap::Vectorf<2>& sensorPosition = sensor.GetPosition();
    const MazeMap::Vectorf<2>& sensorFacing = sensor.GetFacingDirection();
    const float sensorXM =
        pose.xMeters +
        (yawCos * sensorPosition.GetX()) -
        (yawSin * sensorPosition.GetY());
    const float sensorYM =
        pose.yMeters +
        (yawSin * sensorPosition.GetX()) +
        (yawCos * sensorPosition.GetY());
    const float facingXM =
        (yawCos * sensorFacing.GetX()) -
        (yawSin * sensorFacing.GetY());
    const float facingYM =
        (yawSin * sensorFacing.GetX()) +
        (yawCos * sensorFacing.GetY());
    const float innerMinCoordinateM =
        MazeMap::ComputeCellInnerMinCoordinateM(Config::kMazeWallThicknessM);
    const float innerMaxCoordinateM =
        MazeMap::ComputeCellInnerMaxCoordinateM(
            Config::kCellSizeM,
            Config::kMazeWallThicknessM);

    if (std::fabs(facingXM) >= std::fabs(facingYM))
    {
        if (!(std::fabs(facingXM) > 1.0e-4f))
        {
            return false;
        }

        const float cellBaseXM =
            std::floor(sensorXM / Config::kCellSizeM) * Config::kCellSizeM;
        const float wallFaceXM =
            (facingXM >= 0.0f) ?
            (cellBaseXM + innerMaxCoordinateM) :
            (cellBaseXM + innerMinCoordinateM);
        const float rayScale = (wallFaceXM - sensorXM) / facingXM;
        if (!(rayScale >= 0.0f) || !std::isfinite(rayScale))
        {
            return false;
        }

        alongWallCoordinateM = sensorYM + (rayScale * facingYM);
        return std::isfinite(alongWallCoordinateM);
    }

    if (!(std::fabs(facingYM) > 1.0e-4f))
    {
        return false;
    }

    const float cellBaseYM =
        std::floor(sensorYM / Config::kCellSizeM) * Config::kCellSizeM;
    const float wallFaceYM =
        (facingYM >= 0.0f) ?
        (cellBaseYM + innerMaxCoordinateM) :
        (cellBaseYM + innerMinCoordinateM);
    const float rayScale = (wallFaceYM - sensorYM) / facingYM;
    if (!(rayScale >= 0.0f) || !std::isfinite(rayScale))
    {
        return false;
    }

    alongWallCoordinateM = sensorXM + (rayScale * facingXM);
    return std::isfinite(alongWallCoordinateM);
}

inline bool IsSideWallDetectionWindowValid(
    const PoseEstimate& pose,
    const MazeMap::WallSensor& sensor)
{
    float alongWallCoordinateM = 0.0f;
    return
        TryComputeSideWallAimCoordinateM(
            pose,
            sensor,
            alongWallCoordinateM) &&
        MazeMap::IsWithinWallSegmentCenterWindowM(
            alongWallCoordinateM,
            Config::kCellSizeM,
            Config::kMazeWallThicknessM,
            Config::kSideWallSegmentCenterFraction);
}

struct RollingObservationVoteSummary
{
    uint8_t sampleCount = 0U;
    uint8_t frontWallVotes = 0U;
    uint8_t frontLeftWallVotes = 0U;
    uint8_t frontRightWallVotes = 0U;
    uint8_t frontFallbackVotes = 0U;
    uint8_t leftWallVotes = 0U;
    uint8_t rightWallVotes = 0U;
    uint8_t leftWindowValidVotes = 0U;
    uint8_t rightWindowValidVotes = 0U;
};

inline bool ObservationVoteWinsMajority(uint8_t votes, uint8_t sampleCount)
{
    return sampleCount > 0U && votes >= static_cast<uint8_t>((sampleCount / 2U) + 1U);
}

inline float AverageFiniteObservationValue(float sum, uint8_t count, float fallbackValue)
{
    return (count > 0U) ? (sum / static_cast<float>(count)) : fallbackValue;
}

inline bool BuildEvidenceObservationSnapshot(
    const SensorSnapshot* samples,
    uint8_t sampleCount,
    SensorSnapshot& combinedSnapshot,
    RollingObservationVoteSummary& voteSummary)
{
    if (samples == nullptr || sampleCount == 0U)
    {
        return false;
    }

    voteSummary = RollingObservationVoteSummary{};
    voteSummary.sampleCount = sampleCount;
    memset(&combinedSnapshot, 0, sizeof(combinedSnapshot));

    float frontLeftDistanceSum = 0.0f;
    float frontRightDistanceSum = 0.0f;
    float frontLeftDifferentialLightSum = 0.0f;
    float frontRightDifferentialLightSum = 0.0f;
    float sideLeftDistanceSum = 0.0f;
    float sideRightDistanceSum = 0.0f;
    float sideLeftDifferentialLightSum = 0.0f;
    float sideRightDifferentialLightSum = 0.0f;
    float corridorErrorSum = 0.0f;
    float frontSkewSum = 0.0f;
    float planarAccelSum = 0.0f;
    float gyroSum = 0.0f;
    uint8_t frontLeftDistanceCount = 0U;
    uint8_t frontRightDistanceCount = 0U;
    uint8_t frontLeftDifferentialLightCount = 0U;
    uint8_t frontRightDifferentialLightCount = 0U;
    uint8_t sideLeftDistanceCount = 0U;
    uint8_t sideRightDistanceCount = 0U;
    uint8_t sideLeftDifferentialLightCount = 0U;
    uint8_t sideRightDifferentialLightCount = 0U;
    uint8_t corridorErrorCount = 0U;
    uint8_t frontSkewCount = 0U;
    uint8_t planarAccelCount = 0U;
    uint8_t gyroCount = 0U;
    MazeMap::WallDecisionAccumulator frontEvidence{};
    MazeMap::WallDecisionAccumulator leftEvidence{};
    MazeMap::WallDecisionAccumulator rightEvidence{};
    bool leftTransitionDetected = false;
    bool rightTransitionDetected = false;

    for (uint8_t sampleIndex = 0U; sampleIndex < sampleCount; ++sampleIndex)
    {
        const SensorSnapshot& sample = samples[sampleIndex];
        if (sample.frontWall)
        {
            ++voteSummary.frontWallVotes;
        }
        if (sample.frontLeftWall)
        {
            ++voteSummary.frontLeftWallVotes;
        }
        if (sample.frontRightWall)
        {
            ++voteSummary.frontRightWallVotes;
        }
        if (sample.frontWallUsesFallbackDetection)
        {
            ++voteSummary.frontFallbackVotes;
        }
        if (sample.leftWallObservation)
        {
            ++voteSummary.leftWallVotes;
        }
        if (sample.rightWallObservation)
        {
            ++voteSummary.rightWallVotes;
        }
        if (sample.leftWallObservationWindowValid)
        {
            ++voteSummary.leftWindowValidVotes;
        }
        if (sample.rightWallObservationWindowValid)
        {
            ++voteSummary.rightWindowValidVotes;
        }

        frontEvidence.Update(
            sample.frontWall ?
                MazeMap::WallSampleClassification::WallHit :
                MazeMap::WallSampleClassification::WallMiss,
            Config::kWallMapEvidenceHitWeight,
            Config::kWallMapEvidenceMissWeight,
            Config::kWallMapEvidenceUnknownDecay);

        leftEvidence.Update(
            sample.leftWallObservationWindowValid ?
                (sample.leftWallObservation ?
                    MazeMap::WallSampleClassification::WallHit :
                    MazeMap::WallSampleClassification::WallMiss) :
                MazeMap::WallSampleClassification::Unknown,
            Config::kWallMapEvidenceHitWeight,
            Config::kWallMapEvidenceMissWeight,
            Config::kWallMapEvidenceUnknownDecay);
        if (sample.leftTransitionDetected)
        {
            leftTransitionDetected = true;
            leftEvidence.InjectMissImpulse(Config::kWallMapEvidenceTransitionMissWeight);
        }

        rightEvidence.Update(
            sample.rightWallObservationWindowValid ?
                (sample.rightWallObservation ?
                    MazeMap::WallSampleClassification::WallHit :
                    MazeMap::WallSampleClassification::WallMiss) :
                MazeMap::WallSampleClassification::Unknown,
            Config::kWallMapEvidenceHitWeight,
            Config::kWallMapEvidenceMissWeight,
            Config::kWallMapEvidenceUnknownDecay);
        if (sample.rightTransitionDetected)
        {
            rightTransitionDetected = true;
            rightEvidence.InjectMissImpulse(Config::kWallMapEvidenceTransitionMissWeight);
        }

        if (std::isfinite(sample.frontLeftDistanceM))
        {
            frontLeftDistanceSum += sample.frontLeftDistanceM;
            ++frontLeftDistanceCount;
        }
        if (std::isfinite(sample.frontRightDistanceM))
        {
            frontRightDistanceSum += sample.frontRightDistanceM;
            ++frontRightDistanceCount;
        }
        if (std::isfinite(sample.frontLeftDifferentialLight))
        {
            frontLeftDifferentialLightSum += sample.frontLeftDifferentialLight;
            ++frontLeftDifferentialLightCount;
        }
        if (std::isfinite(sample.frontRightDifferentialLight))
        {
            frontRightDifferentialLightSum += sample.frontRightDifferentialLight;
            ++frontRightDifferentialLightCount;
        }
        if (std::isfinite(sample.sideLeftDistanceM))
        {
            sideLeftDistanceSum += sample.sideLeftDistanceM;
            ++sideLeftDistanceCount;
        }
        if (std::isfinite(sample.sideRightDistanceM))
        {
            sideRightDistanceSum += sample.sideRightDistanceM;
            ++sideRightDistanceCount;
        }
        if (std::isfinite(sample.sideLeftDifferentialLight))
        {
            sideLeftDifferentialLightSum += sample.sideLeftDifferentialLight;
            ++sideLeftDifferentialLightCount;
        }
        if (std::isfinite(sample.sideRightDifferentialLight))
        {
            sideRightDifferentialLightSum += sample.sideRightDifferentialLight;
            ++sideRightDifferentialLightCount;
        }
        if (std::isfinite(sample.corridorErrorM))
        {
            corridorErrorSum += sample.corridorErrorM;
            ++corridorErrorCount;
        }
        if (std::isfinite(sample.frontSkewM))
        {
            frontSkewSum += sample.frontSkewM;
            ++frontSkewCount;
        }
        if (std::isfinite(sample.planarAccelMps2))
        {
            planarAccelSum += sample.planarAccelMps2;
            ++planarAccelCount;
        }
        if (std::isfinite(sample.gyroRadps))
        {
            gyroSum += sample.gyroRadps;
            ++gyroCount;
        }
    }

    const SensorSnapshot& lastSample = samples[sampleCount - 1U];
    combinedSnapshot.frontLeftDistanceM =
        AverageFiniteObservationValue(frontLeftDistanceSum, frontLeftDistanceCount, lastSample.frontLeftDistanceM);
    combinedSnapshot.frontRightDistanceM =
        AverageFiniteObservationValue(frontRightDistanceSum, frontRightDistanceCount, lastSample.frontRightDistanceM);
    combinedSnapshot.frontLeftDifferentialLight =
        AverageFiniteObservationValue(frontLeftDifferentialLightSum, frontLeftDifferentialLightCount, lastSample.frontLeftDifferentialLight);
    combinedSnapshot.frontRightDifferentialLight =
        AverageFiniteObservationValue(frontRightDifferentialLightSum, frontRightDifferentialLightCount, lastSample.frontRightDifferentialLight);
    combinedSnapshot.sideLeftDistanceM =
        AverageFiniteObservationValue(sideLeftDistanceSum, sideLeftDistanceCount, lastSample.sideLeftDistanceM);
    combinedSnapshot.sideRightDistanceM =
        AverageFiniteObservationValue(sideRightDistanceSum, sideRightDistanceCount, lastSample.sideRightDistanceM);
    combinedSnapshot.sideLeftDifferentialLight =
        AverageFiniteObservationValue(sideLeftDifferentialLightSum, sideLeftDifferentialLightCount, lastSample.sideLeftDifferentialLight);
    combinedSnapshot.sideRightDifferentialLight =
        AverageFiniteObservationValue(sideRightDifferentialLightSum, sideRightDifferentialLightCount, lastSample.sideRightDifferentialLight);
    combinedSnapshot.corridorErrorM =
        AverageFiniteObservationValue(corridorErrorSum, corridorErrorCount, lastSample.corridorErrorM);
    combinedSnapshot.frontSkewM =
        AverageFiniteObservationValue(frontSkewSum, frontSkewCount, lastSample.frontSkewM);
    combinedSnapshot.planarAccelMps2 =
        AverageFiniteObservationValue(planarAccelSum, planarAccelCount, lastSample.planarAccelMps2);
    combinedSnapshot.gyroRadps =
        AverageFiniteObservationValue(gyroSum, gyroCount, lastSample.gyroRadps);
    const MazeMap::WallSampleClassification frontDecision =
        frontEvidence.FinalClassification(Config::kWallMapEvidenceCommitThreshold);
    const MazeMap::WallSampleClassification leftDecision =
        leftEvidence.FinalClassification(Config::kWallMapEvidenceCommitThreshold);
    const MazeMap::WallSampleClassification rightDecision =
        rightEvidence.FinalClassification(Config::kWallMapEvidenceCommitThreshold);
    combinedSnapshot.frontWall = frontDecision == MazeMap::WallSampleClassification::WallHit;
    combinedSnapshot.frontLeftWall = ObservationVoteWinsMajority(voteSummary.frontLeftWallVotes, sampleCount);
    combinedSnapshot.frontRightWall = ObservationVoteWinsMajority(voteSummary.frontRightWallVotes, sampleCount);
    combinedSnapshot.frontWallObservationValid = frontDecision != MazeMap::WallSampleClassification::Unknown;
    combinedSnapshot.frontWallUsesFallbackDetection = combinedSnapshot.frontWallObservationValid;
    combinedSnapshot.frontWallUsesCharacterizationDetection = false;
    combinedSnapshot.leftWallObservation = leftDecision == MazeMap::WallSampleClassification::WallHit;
    combinedSnapshot.rightWallObservation = rightDecision == MazeMap::WallSampleClassification::WallHit;
    combinedSnapshot.leftWall = combinedSnapshot.leftWallObservation;
    combinedSnapshot.rightWall = combinedSnapshot.rightWallObservation;
    combinedSnapshot.leftWallObservationWindowValid = leftDecision != MazeMap::WallSampleClassification::Unknown;
    combinedSnapshot.rightWallObservationWindowValid = rightDecision != MazeMap::WallSampleClassification::Unknown;
    combinedSnapshot.leftTransitionDetected = leftTransitionDetected;
    combinedSnapshot.rightTransitionDetected = rightTransitionDetected;
    return true;
}



