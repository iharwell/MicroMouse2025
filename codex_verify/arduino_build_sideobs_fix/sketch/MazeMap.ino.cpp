#include <Arduino.h>
#line 1 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
#include "Defines.h"
#include "TeensyLayout.h"
#include "Maze.h"
#include "PathFinder.h"
#include "Maneuver.h"
#include "ManeuverQueue.h"
#include "ManeuverPathFinder.h"
#include "MotorEncoderDrive.h"
#include "SearchRunPlanner.h"
#include "Vehicle.h"
#include "WallSensorCalibration.h"
#include "DirectionalLocation.h"
#include "Kinematics.h"
#include "CoreFileExport.h"
#include "DiagnosticCoverage.h"
#include "DiagnosticLogBudget.h"
#include "DiagnosticMotionPlan.h"
#include "EncoderStallPolicy.h"
#include "FanRampProfile.h"
#include "GyroBiasUpdatePolicy.h"
#include "ImuCalibrationPolicy.h"
#include "ImuSamplingProfile.h"
#include "InPlaceTurnProfile.h"
#include "LaunchAssistProfile.h"
#include "CruiseSpeedFloor.h"
#include "MotionTargetProjection.h"
#include "MissionStartPolicy.h"
#include "MissionMazeExport.h"
#include "MotorModelUnits.h"
#include "OpenLoopDriveCommand.h"
#include "RollingAverageWindow.h"
#include "TrackWidthEstimate.h"
#include "TurnCommandGeometry.h"
#include "TurnWallEdgeTracker.h"
#include "DiagonalWallCentering.h"
#include "TractionLimitSweep.h"
#include "WallDetectionThresholds.h"
#include "WallContactDetection.h"
#include "WheelControlProfile.h"
#include <ctype.h>
#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

class WallDistanceCalibration;
struct SensorSnapshot;
struct RollingObservationVoteSummary;
struct AveragedBackLeftImuSample;
enum class StationaryImuCalibrationResult : uint8_t;

static bool ObservationVoteWinsMajority(uint8_t votes, uint8_t sampleCount);
static float AverageFiniteObservationValue(float sum, uint8_t count, float fallbackValue);
static bool BuildMajorityObservationSnapshot(
    const SensorSnapshot* samples,
    uint8_t sampleCount,
    SensorSnapshot& combinedSnapshot,
    RollingObservationVoteSummary& voteSummary);

namespace MazeMap
{
    namespace Platform
    {
        int32_t ReadEncoderCount(uint8_t channel);
    }
}

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

static bool SetupHardware()
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
    // [Medium] Initial eastward offset before the front-sensor sweep begins. Increase to give the log-amp front sensors
    // a farther calibration point; decrease if the sweep wastes time or approaches the east wall of the start cell.
    constexpr float kStartupWallCalibrationFrontFarOffsetM = 0.000f;
    // [Medium] Spacing between front-sensor calibration sample poses. Increase to cover a wider distance range with fewer
    // points; decrease if you need a denser fit across the front-sensor log-amp response.
    constexpr float kStartupWallCalibrationFrontStepM = 0.020f;
    // [Medium] Number of front-sensor calibration points collected against the west wall. Increase if the front-sensor
    // curve still shows shape error after calibration; decrease if startup time matters more than curve fidelity.
    constexpr uint8_t kStartupWallCalibrationFrontPointCount = 16U;
    // [Medium] Minimum front-sensor clearance from the west wall during the moving startup calibration sweep. If the
    // nominal sweep would violate this, the planner should shift or skip the sweep rather than fault the entire run.
    constexpr float kStartupWallCalibrationFrontSensorMinWallDistanceM = 0.010f;
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
    // [Medium] Open-loop drive command used during wall touch-off. Increase only if the robot fails to make contact;
    // decrease if the wheels spin, the impact is harsher than desired, or contact repeatability gets worse.
    constexpr float kWallTouchDriveCommand = 0.28f;
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
    // [High] Peak open-loop drive command used while forcing the chassis to seat on the wall after encoder stall is
    // detected. This intentionally allows full drive so the wall-touch can scrub into square on this chassis.
    constexpr float kWallTouchSeatRampMaxDriveCommand = 1.0f;
    // [Medium] Ramp time from the normal wall-touch drive command up to the seating command ceiling after encoder stall.
    constexpr uint16_t kWallTouchSeatRampMs = 350U;
    // [High] Minimum wall-touch seating command that must be reached before resumed encoder motion is accepted as the
    // tires breaking free into the wall-scrub phase. This keeps a perfectly square hit from releasing on a soft bounce.
    constexpr float kWallTouchSeatReleaseMinDriveCommand = 0.80f;
    constexpr float kWallTouchSeatReleaseDistanceM = 0.003f;
    constexpr uint16_t kWallTouchSeatReleaseMinSkidMs = 100U;
    // [High] Half-period for the wall-touch seat wiggle once the push is strong enough to try to break the tires free.
    constexpr uint16_t kWallTouchSeatWiggleHalfPeriodMs = 120U;
    // [High] Differential-drive fraction applied during the wall-touch seat wiggle. The wiggle stays forward-driving;
    // this only biases one side harder than the other to help the chassis scrub square on the wall.
    constexpr float kWallTouchSeatWiggleTurnFraction = 0.20f;
    // [High] Minimum fraction of the requested forward seating command that must be retained on both wheels while
    // yaw-wiggling. This keeps the wiggle from unloading one side enough to look like a back-off.
    constexpr float kWallTouchSeatWiggleRetainedForwardFraction = 0.90f;
    // [Low] Base maximum travel allowed while searching for a wall during touch-off. Nearby one-cell touches clamp to
    // this clear-span budget, while longer known-wall moves expand from the geometry-predicted travel plus slack.
    const float kWallTouchBaseMaxApproachDistanceM = kCellClearSpanM;
    // [Medium] Mission boundary-impact wall discovery is currently disabled. These thresholds are retained only so the
    // contact detector can be re-enabled later without re-deriving them.
    constexpr bool kEnableMissionBoundaryImpactWallDetection = false;
    constexpr float kFrontWallTapMinimumMotionSpeedMps = 0.018f;
    constexpr float kFrontWallTapMinimumMotionDistanceM = 0.003f;
    // [Medium] Contact is recognized primarily as a sharp encoder-speed collapse after motion is established. Keep the
    // minimum peak low enough for short taps, but require a meaningful absolute drop and ratio collapse to reject noise.
    constexpr float kFrontWallTapMinimumPeakEncoderSpeedMps = 0.020f;
    constexpr float kFrontWallTapMaximumCurrentPeakRatio = 0.45f;
    constexpr float kFrontWallTapMinimumEncoderDropMps = 0.0075f;
    // [Medium] The mission contact tap uses planar-acceleration spikes as secondary evidence. Run the mission accel LPF2
    // at ODR/20 (~50 Hz at the 1 kHz mission loop) instead of ODR/400 (~2.5 Hz), which is too slow for impact events.
    constexpr auto kMissionRuntimeAccelFilterFreq = MazeMap::Vehicle::ImuBackLeft::ACCEL_FILTER_FREQ::FRAC_1_020;
    constexpr uint16_t kFrontWallTapAccelArmDelayMs = 40U;
    constexpr float kFrontWallTapPlanarAccelSpikeMps2 = 0.375f;
    constexpr float kSearchBoundaryImpactArmDistanceM = 0.020f;
    constexpr float kSearchBoundaryOpenConfirmMarginM = 0.010f;
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
    constexpr float kSearchTurnMaxOmegaRadps = 6.5f;
    // [High] Executed in-place turn acceleration during search and homing. Raise it for snappier turn entry; lower it
    // if the robot jerks into turns or becomes harder to stop on heading.
    constexpr float kSearchTurnAccelRadps2 = 30.0f;
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
    // weakest front calibration point. When robust calibration bands are available, use the conservative
    // baseline-high to wall-low span before falling back to the averaged span.
    constexpr float kFrontWallSignalLatchFractionOfCalibratedSpan = 0.50f;
    // [High] Release fraction for the normalized front-wall span. Keep it below the latch fraction to preserve
    // hysteresis while still releasing when the scene-specific rise mostly disappears.
    constexpr float kFrontWallSignalReleaseFractionOfCalibratedSpan = 0.35f;
    // [High] Side-wall measured-signal latch threshold in normalized corrected-signal units. The side thresholds now
    // use Decimus-style one-point run normalization against the in-cell wall reference rather than an inverse-square
    // projection of the live signal itself.
    constexpr float kSideWallMeasuredSignalLatchThreshold = 0.10f;
    // [High] Side-wall measured-signal release threshold in normalized corrected-signal units.
    constexpr float kSideWallMeasuredSignalReleaseThreshold = 0.07f;
    // [High] Only trust side-wall detections while the receiver is aimed at the center third of a wall segment.
    constexpr float kSideWallSegmentCenterFraction = (1.0f / 3.0f);
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

    // [High] Static wheel feedforward used to break stiction. Increase if low-speed commands do not move the robot;
    // decrease if the robot twitches or creeps around zero command.
    constexpr float kWheelStaticFeedforward = 0.14f;
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
    // [High] Velocity feedforward slope. Increase if steady cruise speed falls short with little feedback action;
    // decrease if the robot runs fast before the feedback loop has time to correct.
    constexpr float kWheelVelocityFeedforward = 1.05f;
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
    constexpr float kAngleToleranceRad = 0.75f * DEG_TO_RAD;
    // [Medium] Linear-speed threshold used to declare motion settled. Lower it if the robot is still rolling when a
    // profile completes; raise it if noisy velocity estimates keep motions from ever finishing.
    constexpr float kSpeedToleranceMps = 0.05f;
    // [Medium] Angular-speed threshold used to declare turns settled. Lower it if the robot is still rotating when turns
    // finish; raise it if gyro noise keeps the turn controller waiting too long.
    constexpr float kAngularSpeedToleranceRadps = 0.10f;
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
    // [Medium] Raw stationary gyro-bias averaging window captured at the start of the position audit for comparison
    // against the normal IMU runtime calibration.
    constexpr uint16_t kPositionAuditGyroBiasAverageWindowMs = 5000U;
    constexpr uint8_t kPositionAuditGyroBiasAverageSampleIntervalMs = 2U;
    // [Medium] SD flush cadence during auxiliary capture. Decrease it if you want less risk of losing data on power
    // interruption; increase it if flush overhead becomes the limiting factor.
    constexpr uint32_t kLogFlushPeriodMs = 250U;
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
    float sideLeftDistanceM;
    float sideRightDistanceM;
    float sideLeftDifferentialLight;
    float sideRightDifferentialLight;
    float corridorErrorM;
    float frontSkewM;
    float planarAccelMps2;
    float gyroRadps;
    bool frontWall;
    bool frontLeftWall;
    bool frontRightWall;
    bool frontWallUsesFallbackDetection;
    bool leftWall;
    bool rightWall;
    bool leftWallObservation;
    bool rightWallObservation;
    bool leftWallObservationWindowValid;
    bool rightWallObservationWindowValid;
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
    bool frontWall = false;
    bool leftWall = false;
    bool rightWall = false;
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
#line 1024 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static bool ConfigureLoopMatchedBackLeftImu( MazeMap::Vehicle::ImuBackLeft& imu, unsigned long controlPeriodUs, bool enableAccel);
#line 1052 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static uint8_t ReadDrivenLowPinWithPullup(uint8_t pin);
#line 1083 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static MazeMap::EncoderCountPair CaptureDriveEncoderCounts();
#line 1093 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static bool HaveDriveEncodersMovedSince(const MazeMap::EncoderCountPair& startCounts);
#line 1098 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static StationaryImuCalibrationResult WaitForImuCalibrationSettle( const MazeMap::EncoderCountPair& startCounts, unsigned long settleMs);
#line 1145 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static StationaryImuCalibrationResult AverageBackLeftImuSelfTestSample( MazeMap::Vehicle::ImuBackLeft& imu, uint16_t sampleCount, const MazeMap::EncoderCountPair& startCounts, AveragedBackLeftImuSample& averagedSample);
#line 1284 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static void PrintHexByte(uint8_t value);
#line 1294 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static const char * WallSensorIdName(WallSensorId sensorId);
#line 1311 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static const char * CalibrationWallName(CalibrationWall wall);
#line 1328 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static const char * WallTouchOutcomeName(WallTouchOutcome outcome);
#line 1341 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static const char * DirectionName(MazeMap::Direction direction);
#line 1366 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static const char * WallStateName(MazeMap::WallState state);
#line 1380 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static bool IsFrontWallSensor(WallSensorId sensorId);
#line 1385 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static MazeMap::WallSensorCalibrationMode WallSensorCalibrationModeFor(WallSensorId sensorId);
#line 1392 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static const char * WallSensorCalibrationMeasurementName(WallSensorId sensorId);
#line 2278 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static float ComputeDiagonalWallCenterOmegaRadps( const WallDistanceCalibration& wallCalibration, float leftMeasuredSignal, float rightMeasuredSignal);
#line 2306 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static bool TryComputeSideWallSignalDistanceM( const WallDistanceCalibration& wallCalibration, WallSensorId sensorId, float measuredSignal, float& distanceM);
#line 2329 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static bool TryComputeStraightWallCenterErrorM( const WallDistanceCalibration& wallCalibration, float leftMeasuredSignal, bool leftWall, float rightMeasuredSignal, bool rightWall, float& corridorErrorM);
#line 2396 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static float SignF(float value);
#line 2401 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static float WrapAngleRad(float angle);
#line 2406 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static float AngleErrorRad(float target, float measured);
#line 2419 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static float DirectionToYawRad(MazeMap::Direction direction);
#line 2449 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static float HeadingErrorRad(const MazeMap::Vectorf<2>& targetHeading, const MazeMap::Vectorf<2>& measuredHeading);
#line 2459 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static uint16_t FanPwmCode(float dutyCycle);
#line 2471 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static void WriteFanDutyCycle(float dutyCycle);
#line 2480 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static void RampFanDutyCycle(float targetDutyCycle);
#line 2517 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static void SetMissionLevelFanEnabled(bool enabled);
#line 2528 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static float ReachableSpeedWithBoundary(float boundarySpeed, float distance, float accel);
#line 2562 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static bool TryDistanceToWestWall(const PoseEstimate& pose, const MazeMap::WallSensor& sensor, float& distanceM);
#line 2585 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static bool TryDistanceToEastWall(const PoseEstimate& pose, const MazeMap::WallSensor& sensor, float& distanceM);
#line 2608 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static bool TryDistanceToSouthWall(const PoseEstimate& pose, const MazeMap::WallSensor& sensor, float& distanceM);
#line 2631 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static bool TryComputeEffectiveTurnRadiusM( float leftDistanceM, float rightDistanceM, float yawChangeRad, float& turnRadiusM);
#line 2650 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static bool TryGetCellCenterMeters(const MazeMap::CellCoordinates& cell, float& xMeters, float& yMeters);
#line 2656 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static bool TryGetCellWallFaceCoordinateM( const MazeMap::CellCoordinates& cell, MazeMap::Direction wallDirection, float& coordinateM);
#line 2683 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static bool TryComputeWallTouchTargetCoordinateForCellWall( const MazeMap::CellCoordinates& cell, MazeMap::Direction wallDirection, float& targetCoordinateM, CalibrationWall& calibrationWall);
#line 2719 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static bool TryComputePoseAxisFromObservedWall( const PoseEstimate& pose, const MazeMap::WallSensor& sensor, float measuredDistanceM, const MazeMap::CellCoordinates& cell, MazeMap::Direction wallDirection, float& coordinateM);
#line 2767 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static uint32_t WallSensorAmbientSettleTimeUs(WallSensorId sensorId);
#line 2772 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static uint32_t WallSensorLitSettleTimeUs(WallSensorId sensorId);
#line 2777 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static uint32_t WallSensorLedCalibrationHalfPeriodUs(WallSensorId sensorId);
#line 2782 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static float WallSensorMeasuredValueForCalibration(WallSensorId sensorId, const RawWallSensorSample& sample);
#line 2787 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static WallSensorCalibrationInput BuildWallSensorCalibrationInput(WallSensorId sensorId, const RawWallSensorSample& sample);
#line 2798 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static RawWallSensorSample SampleWallSensorRaw(WallSensorId sensorId, const MazeMap::WallSensor& sensor);
#line 2814 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static void SampleWallSensorPairRaw( WallSensorId firstSensorId, const MazeMap::WallSensor& firstSensor, WallSensorId secondSensorId, const MazeMap::WallSensor& secondSensor, RawWallSensorSample& firstSample, RawWallSensorSample& secondSample);
#line 2841 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static WallSensorCalibrationInput SampleWallCalibrationInputRaw(WallSensorId sensorId, const MazeMap::WallSensor& sensor);
#line 2846 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static void SampleWallCalibrationInputRawPair( WallSensorId firstSensorId, const MazeMap::WallSensor& firstSensor, WallSensorId secondSensorId, const MazeMap::WallSensor& secondSensor, WallSensorCalibrationInput& firstInput, WallSensorCalibrationInput& secondInput);
#line 2861 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static WallSensorCalibrationCapture SampleWallCalibrationCaptureAverageRaw(WallSensorId sensorId, const MazeMap::WallSensor& sensor);
#line 2886 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static void SampleWallCalibrationCaptureAverageRawPair( WallSensorId firstSensorId, const MazeMap::WallSensor& firstSensor, WallSensorId secondSensorId, const MazeMap::WallSensor& secondSensor, WallSensorCalibrationCapture& firstCapture, WallSensorCalibrationCapture& secondCapture);
#line 2940 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static bool HysteresisWall(bool currentState, float distanceM, float onThresholdM, float offThresholdM);
#line 2949 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static bool IsApproximatelyDiagonalHeadingUnit(const MazeMap::Vectorf<2>& headingUnit);
#line 2956 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static MazeMap::ManeuverCode RelativeToInPlaceCode(MazeMap::RelativeDirection rel);
#line 2979 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static bool IsStraightCode(MazeMap::ManeuverCode code);
#line 2984 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static void TrimAsciiWhitespace(char* text);
#line 3010 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static void NormalizeToken(char* text);
#line 3024 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static bool TryParseBaseManeuverCodeName(const char* token, MazeMap::ManeuverCode& code);
#line 3083 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static bool TryParseManeuverCodeToken(const char* token, MazeMap::ManeuverCode& code);
#line 3125 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static void FormatManeuverCodeName(MazeMap::ManeuverCode code, char* buffer, size_t bufferSize);
#line 3175 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static float ReadBackLeftGyroZRadpsRaw(MazeMap::Vehicle& vehicle);
#line 3186 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static float EstimateMissionGyroBiasRadps(MazeMap::Vehicle& vehicle);
#line 3202 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static bool TryAverageRawGyroBiasRadpsOverWindow( MazeMap::Vehicle& vehicle, unsigned long durationMs, unsigned long sampleIntervalMs, float& averageRadps, unsigned long& sampleCount);
#line 3312 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static bool TryComputeSideWallAimCoordinateM( const PoseEstimate& pose, const MazeMap::WallSensor& sensor, float& alongWallCoordinateM);
#line 3394 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static bool IsSideWallDetectionWindowValid( const PoseEstimate& pose, const MazeMap::WallSensor& sensor);
#line 4095 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static MazeMap::WheelControlProfile BuildNominalWheelControlProfile();
#line 4104 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static MazeMap::InPlaceTurnProfile BuildSharedInPlaceTurnProfile(float maxAngularSpeedRadps, float angularAccelRadps2);
#line 4116 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static MazeMap::InPlaceTurnProfile BuildSharedInPlaceTurnProfile(const MazeMap::Vehicle& vehicle);
#line 4968 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static bool SelectSequentialCsvFileName( char* buffer, size_t bufferSize, const char* explicitFileName, const char* teensyFormat, const char* hostFallback);
#line 5468 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static bool IsPinPairStrapped(uint8_t pinA, uint8_t pinB);
#line 5493 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static bool IsPrimaryDiagnosticModeRequested();
#line 5498 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static bool IsManeuverTestModeRequested();
#line 5503 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static bool IsAuxiliaryMeasurementModeRequested();
#line 5508 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static bool IsInterRunServiceJumperInstalled();
#line 5513 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static bool IsWallSensorLedCalibrationModeRequested();
#line 5518 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static bool ResetStartupTrace(const char* firstLine);
#line 5545 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static bool AppendStartupTrace(const char* line);
#line 5569 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static const char * AuxMeasurementRoutineName(AuxMeasurementConfig::Routine routine);
#line 14779 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
void setup();
#line 14837 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
void loop();
#line 1024 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
static bool ConfigureLoopMatchedBackLeftImu(
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

static uint8_t ReadDrivenLowPinWithPullup(uint8_t pin)
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

static constexpr unsigned long kImuCalibrationSampleIntervalMs = 2UL;
static constexpr uint16_t kImuSelfTestAverageSamples = 64U;
static constexpr uint16_t kImuSelfTestSettleMs = 50U;
static constexpr float kImuSelfTestGyroFullScaleDps = 2000.0f;

static MazeMap::EncoderCountPair CaptureDriveEncoderCounts()
{
    MazeMap::EncoderCountPair counts{};
    const auto& leftDriveHardware = MazeMap::MotorEncoderDrive::GetLeftHardwareConfig();
    const auto& rightDriveHardware = MazeMap::MotorEncoderDrive::GetRightHardwareConfig();
    counts.left = MazeMap::Platform::ReadEncoderCount(leftDriveHardware.encoderChannel);
    counts.right = MazeMap::Platform::ReadEncoderCount(rightDriveHardware.encoderChannel);
    return counts;
}

static bool HaveDriveEncodersMovedSince(const MazeMap::EncoderCountPair& startCounts)
{
    return MazeMap::HaveEncoderCountsChanged(startCounts, CaptureDriveEncoderCounts());
}

static StationaryImuCalibrationResult WaitForImuCalibrationSettle(
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

static bool ConfigureBackLeftImuForRuntime(
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

static StationaryImuCalibrationResult AverageBackLeftImuSelfTestSample(
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

static StationaryImuCalibrationResult RunStationaryBackLeftImuSelfTest(
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

static void PrintHexByte(uint8_t value)
{
    if (value < 0x10U)
    {
        Serial.print('0');
    }
    Serial.print(static_cast<unsigned>(value), HEX);
}
#endif

static const char* WallSensorIdName(WallSensorId sensorId)
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

static const char* CalibrationWallName(CalibrationWall wall)
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

static const char* WallTouchOutcomeName(WallTouchOutcome outcome)
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

static const char* DirectionName(MazeMap::Direction direction)
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

static const char* WallStateName(MazeMap::WallState state)
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

static bool IsFrontWallSensor(WallSensorId sensorId)
{
    return sensorId == WallSensorId::FrontLeft || sensorId == WallSensorId::FrontRight;
}

static MazeMap::WallSensorCalibrationMode WallSensorCalibrationModeFor(WallSensorId sensorId)
{
    return IsFrontWallSensor(sensorId) ?
        MazeMap::WallSensorCalibrationMode::DirectInterpolation :
        MazeMap::WallSensorCalibrationMode::DistanceOffset;
}

static const char* WallSensorCalibrationMeasurementName(WallSensorId sensorId)
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

static WallDistanceCalibration gWallDistanceCalibration;

static float ComputeDiagonalWallCenterOmegaRadps(
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

static bool TryComputeSideWallSignalDistanceM(
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

static bool TryComputeStraightWallCenterErrorM(
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

static float SignF(float value)
{
    return static_cast<float>((value > 0.0f) - (value < 0.0f));
}

static float WrapAngleRad(float angle)
{
    return std::remainder(angle, TWO_PI);
}

static float AngleErrorRad(float target, float measured)
{
    return WrapAngleRad(target - measured);
}

static MazeMap::Vectorf<2> DirectionToUnitVector(MazeMap::Direction direction)
{
    float dx = 0.0f;
    float dy = 0.0f;
    MazeMap::GetHeading(direction, dx, dy);
    return MazeMap::Vectorf<2>(dx, dy);
}

static float DirectionToYawRad(MazeMap::Direction direction)
{
    switch (direction)
    {
    case MazeMap::Up:
        return 0.5f * PI;
    case MazeMap::UpRight:
        return 0.25f * PI;
    case MazeMap::Right:
        return 0.0f;
    case MazeMap::DownRight:
        return -0.25f * PI;
    case MazeMap::Down:
        return -0.5f * PI;
    case MazeMap::DownLeft:
        return -0.75f * PI;
    case MazeMap::Left:
        return PI;
    case MazeMap::UpLeft:
        return 0.75f * PI;
    default:
        return 0.0f;
    }
}

static MazeMap::Vectorf<2> HeadingUnitFromYawRad(float yawRad)
{
    return MazeMap::Vectorf<2>(cosf(yawRad), sinf(yawRad));
}

static float HeadingErrorRad(const MazeMap::Vectorf<2>& targetHeading, const MazeMap::Vectorf<2>& measuredHeading)
{
    const float dot = (std::clamp)((targetHeading * measuredHeading), -1.0f, 1.0f);
    const float cross = (measuredHeading.GetX() * targetHeading.GetY()) - (measuredHeading.GetY() * targetHeading.GetX());
    return atan2f(cross, dot);
}

static constexpr float kStandardGravityMps2 = 9.80665f;
static constexpr unsigned long kFanRampStepMs = 20UL;

static uint16_t FanPwmCode(float dutyCycle)
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

static void WriteFanDutyCycle(float dutyCycle)
{
#if defined(ARDUINO_TEENSY41)
    analogWrite(Pins::Fan_CTRL, FanPwmCode(dutyCycle));
#else
    (void)dutyCycle;
#endif
}

static void RampFanDutyCycle(float targetDutyCycle)
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

static void SetMissionLevelFanEnabled(bool enabled)
{
    if (enabled)
    {
        RampFanDutyCycle(Config::kRacingFanDutyCycle);
        return;
    }

    WriteFanDutyCycle(0.0f);
}

static float ReachableSpeedWithBoundary(float boundarySpeed, float distance, float accel)
{
    if (accel <= 0.0f)
    {
        return (std::max)(boundarySpeed, 0.0f);
    }

    return MazeMap::LinearKinematics::V1IgnoringT((std::max)(distance, 0.0f), boundarySpeed, accel);
}

static MazeMap::Vectorf<2> LeftUnitFromHeading(const MazeMap::Vectorf<2>& headingUnit)
{
    return MazeMap::Vectorf<2>(-headingUnit.GetY(), headingUnit.GetX());
}

static MazeMap::Vectorf<2> RotateBodyVectorToWorld(const PoseEstimate& pose, const MazeMap::Vectorf<2>& bodyVector)
{
    const MazeMap::Vectorf<2> leftUnit = LeftUnitFromHeading(pose.headingUnit);
    return MazeMap::Vectorf<2>(
        (pose.headingUnit.GetX() * bodyVector.GetX()) + (leftUnit.GetX() * bodyVector.GetY()),
        (pose.headingUnit.GetY() * bodyVector.GetX()) + (leftUnit.GetY() * bodyVector.GetY()));
}

static MazeMap::Vectorf<2> SensorWorldPosition(const PoseEstimate& pose, const MazeMap::WallSensor& sensor)
{
    const MazeMap::Vectorf<2> worldOffset = RotateBodyVectorToWorld(pose, sensor.GetPosition());
    return MazeMap::Vectorf<2>(pose.xMeters + worldOffset.GetX(), pose.yMeters + worldOffset.GetY());
}

static MazeMap::Vectorf<2> SensorWorldFacing(const PoseEstimate& pose, const MazeMap::WallSensor& sensor)
{
    return RotateBodyVectorToWorld(pose, sensor.GetFacingDirection());
}

static bool TryDistanceToWestWall(const PoseEstimate& pose, const MazeMap::WallSensor& sensor, float& distanceM)
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

static bool TryDistanceToEastWall(const PoseEstimate& pose, const MazeMap::WallSensor& sensor, float& distanceM)
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

static bool TryDistanceToSouthWall(const PoseEstimate& pose, const MazeMap::WallSensor& sensor, float& distanceM)
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

static bool TryComputeEffectiveTurnRadiusM(
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

static bool TryGetCellCenterMeters(const MazeMap::CellCoordinates& cell, float& xMeters, float& yMeters)
{
    MazeMap::MazeLocation::CellCenter(cell).GetPhysicalLocation(Config::kCellSizeM, xMeters, yMeters);
    return std::isfinite(xMeters) && std::isfinite(yMeters);
}

static bool TryGetCellWallFaceCoordinateM(
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

static bool TryComputeWallTouchTargetCoordinateForCellWall(
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

static bool TryComputePoseAxisFromObservedWall(
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

static uint32_t WallSensorAmbientSettleTimeUs(WallSensorId sensorId)
{
    return IsFrontWallSensor(sensorId) ? HardwareConfig::kFrontWallSensorSwitchSettleTime_us : HardwareConfig::kSideWallSensorSwitchSettleTime_us;
}

static uint32_t WallSensorLitSettleTimeUs(WallSensorId sensorId)
{
    return IsFrontWallSensor(sensorId) ? HardwareConfig::kFrontWallSensorSwitchSettleTime_us : HardwareConfig::kSideWallSensorSwitchSettleTime_us;
}

static uint32_t WallSensorLedCalibrationHalfPeriodUs(WallSensorId sensorId)
{
    return (std::max)(WallSensorAmbientSettleTimeUs(sensorId), WallSensorLitSettleTimeUs(sensorId));
}

static float WallSensorMeasuredValueForCalibration(WallSensorId sensorId, const RawWallSensorSample& sample)
{
    return IsFrontWallSensor(sensorId) ? sample.differentialLight : sample.rawDistanceM;
}

static WallSensorCalibrationInput BuildWallSensorCalibrationInput(WallSensorId sensorId, const RawWallSensorSample& sample)
{
    WallSensorCalibrationInput input{};
    input.measuredValue = WallSensorMeasuredValueForCalibration(sensorId, sample);
    input.fallbackDistanceM = sample.rawDistanceM;
    input.differentialLight = sample.differentialLight;
    input.ambientLight = sample.ambientLight;
    input.litLight = sample.litLight;
    return input;
}

static RawWallSensorSample SampleWallSensorRaw(WallSensorId sensorId, const MazeMap::WallSensor& sensor)
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

static void SampleWallSensorPairRaw(
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

static WallSensorCalibrationInput SampleWallCalibrationInputRaw(WallSensorId sensorId, const MazeMap::WallSensor& sensor)
{
    return BuildWallSensorCalibrationInput(sensorId, SampleWallSensorRaw(sensorId, sensor));
}

static void SampleWallCalibrationInputRawPair(
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

static WallSensorCalibrationCapture SampleWallCalibrationCaptureAverageRaw(WallSensorId sensorId, const MazeMap::WallSensor& sensor)
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

static void SampleWallCalibrationCaptureAverageRawPair(
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
    uint16_t firstDifferentialLightCount = 0U;
    uint16_t secondDifferentialLightCount = 0U;
    for (uint16_t i = 0U; i < Config::kWallCalibrationAverageSampleCount; ++i)
    {
        WallSensorCalibrationInput rawFirstInput{};
        WallSensorCalibrationInput rawSecondInput{};
        SampleWallCalibrationInputRawPair(
            firstSensorId,
            firstSensor,
            secondSensorId,
            secondSensor,
            rawFirstInput,
            rawSecondInput);
        firstAverageWindow.PushAndAverage(rawFirstInput);
        secondAverageWindow.PushAndAverage(rawSecondInput);
        firstDifferentialLightSamples[firstDifferentialLightCount] = rawFirstInput.differentialLight;
        ++firstDifferentialLightCount;
        secondDifferentialLightSamples[secondDifferentialLightCount] = rawSecondInput.differentialLight;
        ++secondDifferentialLightCount;
    }

    firstCapture = {};
    firstCapture.input = firstAverageWindow.Average();
    firstCapture.haveDifferentialLightBand = MazeMap::TryComputeRobustSignalBandFromSamples(
        firstDifferentialLightSamples,
        firstDifferentialLightCount,
        Config::kWallCalibrationScaledMadMultiplier,
        firstCapture.differentialLightBand.median,
        firstCapture.differentialLightBand.low,
        firstCapture.differentialLightBand.high);

    secondCapture = {};
    secondCapture.input = secondAverageWindow.Average();
    secondCapture.haveDifferentialLightBand = MazeMap::TryComputeRobustSignalBandFromSamples(
        secondDifferentialLightSamples,
        secondDifferentialLightCount,
        Config::kWallCalibrationScaledMadMultiplier,
        secondCapture.differentialLightBand.median,
        secondCapture.differentialLightBand.low,
        secondCapture.differentialLightBand.high);
}

static bool HysteresisWall(bool currentState, float distanceM, float onThresholdM, float offThresholdM)
{
    if (currentState)
    {
        return distanceM < offThresholdM;
    }
    return distanceM < onThresholdM;
}

static bool IsApproximatelyDiagonalHeadingUnit(const MazeMap::Vectorf<2>& headingUnit)
{
    const float absX = std::fabs(headingUnit.GetX());
    const float absY = std::fabs(headingUnit.GetY());
    return absX > 0.5f && absY > 0.5f && std::fabs(absX - absY) <= 0.15f;
}

static MazeMap::ManeuverCode RelativeToInPlaceCode(MazeMap::RelativeDirection rel)
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

static bool IsStraightCode(MazeMap::ManeuverCode code)
{
    return code != MazeMap::MC_NONE && code <= MazeMap::S31;
}

static void TrimAsciiWhitespace(char* text)
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

static void NormalizeToken(char* text)
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

static bool TryParseBaseManeuverCodeName(const char* token, MazeMap::ManeuverCode& code)
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

static bool TryParseManeuverCodeToken(const char* token, MazeMap::ManeuverCode& code)
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

static void FormatManeuverCodeName(MazeMap::ManeuverCode code, char* buffer, size_t bufferSize)
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

static float ReadBackLeftGyroZRadpsRaw(MazeMap::Vehicle& vehicle)
{
#if defined(ARDUINO_TEENSY41)
    const float blDps = vehicle.IMU_BL.GyroRawToDps(vehicle.IMU_BL.ReadGyroZ());
    return blDps * DEG_TO_RAD;
#else
    (void)vehicle;
    return 0.0f;
#endif
}

static float EstimateMissionGyroBiasRadps(MazeMap::Vehicle& vehicle)
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

static bool TryAverageRawGyroBiasRadpsOverWindow(
    MazeMap::Vehicle& vehicle,
    unsigned long durationMs,
    unsigned long sampleIntervalMs,
    float& averageRadps,
    unsigned long& sampleCount)
{
    averageRadps = 0.0f;
    sampleCount = 0UL;
    if (durationMs == 0UL || sampleIntervalMs == 0UL)
    {
        return false;
    }

    const unsigned long requiredSamples =
        (std::max)(1UL, (durationMs + sampleIntervalMs - 1UL) / sampleIntervalMs);
    double accumulatedRadps = 0.0;
    for (unsigned long i = 0UL; i < requiredSamples; ++i)
    {
        accumulatedRadps += static_cast<double>(ReadBackLeftGyroZRadpsRaw(vehicle));
        ++sampleCount;
        if ((i + 1UL) < requiredSamples)
        {
            delay(sampleIntervalMs);
        }
    }

    if (sampleCount == 0UL)
    {
        return false;
    }

    averageRadps = static_cast<float>(accumulatedRadps / static_cast<double>(sampleCount));
    return true;
}

static bool CalibrateStationaryBackLeftGyroBias(
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
    gyroBiasRadps = EstimateMissionGyroBiasRadps(vehicle);
    return true;
#endif
}

static bool TryComputeSideWallAimCoordinateM(
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

static bool IsSideWallDetectionWindowValid(
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

static bool ObservationVoteWinsMajority(uint8_t votes, uint8_t sampleCount)
{
    return sampleCount > 0U && votes >= static_cast<uint8_t>((sampleCount / 2U) + 1U);
}

static float AverageFiniteObservationValue(float sum, uint8_t count, float fallbackValue)
{
    return (count > 0U) ? (sum / static_cast<float>(count)) : fallbackValue;
}

static bool BuildMajorityObservationSnapshot(
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
    uint8_t sideLeftDistanceCount = 0U;
    uint8_t sideRightDistanceCount = 0U;
    uint8_t sideLeftDifferentialLightCount = 0U;
    uint8_t sideRightDifferentialLightCount = 0U;
    uint8_t corridorErrorCount = 0U;
    uint8_t frontSkewCount = 0U;
    uint8_t planarAccelCount = 0U;
    uint8_t gyroCount = 0U;

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
    combinedSnapshot.frontWall = ObservationVoteWinsMajority(voteSummary.frontWallVotes, sampleCount);
    combinedSnapshot.frontLeftWall = ObservationVoteWinsMajority(voteSummary.frontLeftWallVotes, sampleCount);
    combinedSnapshot.frontRightWall = ObservationVoteWinsMajority(voteSummary.frontRightWallVotes, sampleCount);
    combinedSnapshot.frontWallUsesFallbackDetection = ObservationVoteWinsMajority(voteSummary.frontFallbackVotes, sampleCount);
    combinedSnapshot.leftWallObservation = ObservationVoteWinsMajority(voteSummary.leftWallVotes, sampleCount);
    combinedSnapshot.rightWallObservation = ObservationVoteWinsMajority(voteSummary.rightWallVotes, sampleCount);
    combinedSnapshot.leftWall = combinedSnapshot.leftWallObservation;
    combinedSnapshot.rightWall = combinedSnapshot.rightWallObservation;
    combinedSnapshot.leftWallObservationWindowValid = ObservationVoteWinsMajority(voteSummary.leftWindowValidVotes, sampleCount);
    combinedSnapshot.rightWallObservationWindowValid = ObservationVoteWinsMajority(voteSummary.rightWindowValidVotes, sampleCount);
    return true;
}

class SensorSuite
{
public:
    SensorSuite(MazeMap::Vehicle& vehicle, WallDistanceCalibration& wallCalibration)
        : _vehicle(vehicle)
        , _wallCalibration(wallCalibration)
        , _gyroBiasRadps(0.0f)
        , _frontLeft{}
        , _frontRight{}
        , _sideLeft{}
        , _sideRight{}
        , _frontLeftWallSignalFiltered(0.0f)
        , _frontRightWallSignalFiltered(0.0f)
        , _sideLeftWallSignalFiltered(0.0f)
        , _sideRightWallSignalFiltered(0.0f)
        , _accelBiasXG(0.0f)
        , _accelBiasYG(0.0f)
        , _frontLeftWallState(false)
        , _frontRightWallState(false)
        , _frontWallUsesFallbackDetection(false)
        , _frontLeftWallSignalInitialized(false)
        , _frontRightWallSignalInitialized(false)
        , _sideLeftWallSignalInitialized(false)
        , _sideRightWallSignalInitialized(false)
        , _accelBiasInitialized(false)
    {
    }

    bool Begin()
    {
        _frontLeft = {};
        _frontRight = {};
        _sideLeft = {};
        _sideRight = {};
        _frontLeftWallSignalFiltered = 0.0f;
        _frontRightWallSignalFiltered = 0.0f;
        _sideLeftWallSignalFiltered = 0.0f;
        _sideRightWallSignalFiltered = 0.0f;
        _frontLeftInputAverage.Clear();
        _frontRightInputAverage.Clear();
        _sideLeftInputAverage.Clear();
        _sideRightInputAverage.Clear();
        _frontLeftWallState = false;
        _frontRightWallState = false;
        _frontWallUsesFallbackDetection = false;
        _frontLeftWallSignalInitialized = false;
        _frontRightWallSignalInitialized = false;
        _sideLeftWallSignalInitialized = false;
        _sideRightWallSignalInitialized = false;
        _accelBiasXG = 0.0f;
        _accelBiasYG = 0.0f;
        _accelBiasInitialized = false;
        bool ok = true;
#if defined(ARDUINO_TEENSY41)
        Serial.println("IMU_FR disabled; using IMU_BL only");

        const bool imuBackLeftOk = _vehicle.IMU_BL.Begin();
        if (!imuBackLeftOk)
        {
            Serial.print("IMU_BL init failed (");
            Serial.print(_vehicle.IMU_BL.GetLastBeginFailureReasonName());
            Serial.print("), WHO_AM_I=0x");
            const uint8_t whoAmI = _vehicle.IMU_BL.GetLastWhoAmI();
            PrintHexByte(whoAmI);
            Serial.print(", INT1_pullup=");
            Serial.print(ReadDrivenLowPinWithPullup(Pins::IMU_INT_1B) == LOW ? "low" : "high");
            Serial.print(", WHO_AM_I@mode3/400kHz=0x");
            PrintHexByte(_vehicle.IMU_BL.ReadWhoAmIWithSettings(400000UL, SPI_MODE3));
            Serial.print(", WHO_AM_I@mode0/400kHz=0x");
            PrintHexByte(_vehicle.IMU_BL.ReadWhoAmIWithSettings(400000UL, SPI_MODE0));
            Serial.println();
        }
        ok = imuBackLeftOk && ok;
#endif
        if (ok)
        {
            ok = CalibrateGyroBias(Config::kControlPeriodUs, true) && ok;
        }
        return ok;
    }

    bool CalibrateGyroBias(unsigned long controlPeriodUs, bool enableAccelRuntime)
    {
        return CalibrateStationaryBackLeftGyroBias(
            _vehicle,
            controlPeriodUs,
            enableAccelRuntime,
            _gyroBiasRadps,
            Config::kMissionRuntimeAccelFilterFreq);
    }

    SensorSnapshot Capture(bool stationary, const PoseEstimate& pose)
    {
        SensorSnapshot snapshot{};
        WallSensorCalibrationInput frontLeftRawInput{};
        WallSensorCalibrationInput frontRightRawInput{};
        WallSensorCalibrationInput sideLeftRawInput{};
        WallSensorCalibrationInput sideRightRawInput{};
        SampleWallCalibrationInputRawPair(
            WallSensorId::FrontLeft,
            _vehicle.FrontLeft,
            WallSensorId::FrontRight,
            _vehicle.FrontRight,
            frontLeftRawInput,
            frontRightRawInput);
        sideLeftRawInput = SampleWallCalibrationInputRaw(WallSensorId::SideLeft, _vehicle.SideLeft);
        sideRightRawInput = SampleWallCalibrationInputRaw(WallSensorId::SideRight, _vehicle.SideRight);
        const WallSensorCalibrationInput frontLeftInput = _frontLeftInputAverage.PushAndAverage(frontLeftRawInput);
        const WallSensorCalibrationInput frontRightInput = _frontRightInputAverage.PushAndAverage(frontRightRawInput);
        const WallSensorCalibrationInput sideLeftInput = _sideLeftInputAverage.PushAndAverage(sideLeftRawInput);
        const WallSensorCalibrationInput sideRightInput = _sideRightInputAverage.PushAndAverage(sideRightRawInput);
        float sideWallOnThresholdM = Config::kSideWallOnThresholdM;
        float sideWallOffThresholdM = Config::kSideWallOffThresholdM;
        _wallCalibration.TryComputeSideWallDistanceThresholds(
            Config::kSideWallDistanceLatchFractionOfCalibration,
            Config::kSideWallDistanceReleaseFractionOfCalibration,
            sideWallOnThresholdM,
            sideWallOffThresholdM);
        snapshot.frontLeftDistanceM = UpdateChannelFromMeasuredDistance(
            _frontLeft,
            _wallCalibration.Apply(WallSensorId::FrontLeft, frontLeftInput.measuredValue, frontLeftInput.fallbackDistanceM));
        snapshot.frontRightDistanceM = UpdateChannelFromMeasuredDistance(
            _frontRight,
            _wallCalibration.Apply(WallSensorId::FrontRight, frontRightInput.measuredValue, frontRightInput.fallbackDistanceM));
        float sideLeftDistanceM = _wallCalibration.Apply(
            WallSensorId::SideLeft,
            sideLeftInput.measuredValue,
            sideLeftInput.fallbackDistanceM);
        float sideRightDistanceM = _wallCalibration.Apply(
            WallSensorId::SideRight,
            sideRightInput.measuredValue,
            sideRightInput.fallbackDistanceM);
        (void)TryComputeSideWallSignalDistanceM(
            _wallCalibration,
            WallSensorId::SideLeft,
            sideLeftInput.differentialLight,
            sideLeftDistanceM);
        (void)TryComputeSideWallSignalDistanceM(
            _wallCalibration,
            WallSensorId::SideRight,
            sideRightInput.differentialLight,
            sideRightDistanceM);
        snapshot.sideLeftDistanceM = UpdateChannelFromMeasuredDistance(
            _sideLeft,
            sideLeftDistanceM);
        snapshot.sideRightDistanceM = UpdateChannelFromMeasuredDistance(
            _sideRight,
            sideRightDistanceM);
        snapshot.sideLeftDifferentialLight = sideLeftInput.differentialLight;
        snapshot.sideRightDifferentialLight = sideRightInput.differentialLight;

        snapshot.frontWall = UpdateFrontWallState(
            frontLeftInput.ambientLight,
            frontLeftInput.measuredValue,
            frontRightInput.ambientLight,
            frontRightInput.measuredValue,
            (std::min)(snapshot.frontLeftDistanceM, snapshot.frontRightDistanceM));
        snapshot.frontLeftWall = _frontLeftWallState;
        snapshot.frontRightWall = _frontRightWallState;
        snapshot.frontWallUsesFallbackDetection = _frontWallUsesFallbackDetection;
        const bool sideLeftWindowValid = IsSideWallDetectionWindowValid(pose, _vehicle.SideLeft);
        const bool sideRightWindowValid = IsSideWallDetectionWindowValid(pose, _vehicle.SideRight);
        snapshot.leftWallObservationWindowValid = sideLeftWindowValid;
        snapshot.rightWallObservationWindowValid = sideRightWindowValid;
        snapshot.leftWall = UpdateSideWallState(
            WallSensorId::SideLeft,
            sideLeftInput.differentialLight,
            sideLeftInput.fallbackDistanceM,
            sideWallOnThresholdM,
            sideWallOffThresholdM,
            sideLeftWindowValid,
            _sideLeftWallSignalFiltered,
            _sideLeftWallSignalInitialized,
            _sideLeft.wall);
        snapshot.rightWall = UpdateSideWallState(
            WallSensorId::SideRight,
            sideRightInput.differentialLight,
            sideRightInput.fallbackDistanceM,
            sideWallOnThresholdM,
            sideWallOffThresholdM,
            sideRightWindowValid,
            _sideRightWallSignalFiltered,
            _sideRightWallSignalInitialized,
            _sideRight.wall);
        snapshot.leftWallObservation = ComputeSideWallObservationHit(
            WallSensorId::SideLeft,
            sideLeftInput.differentialLight,
            sideLeftInput.fallbackDistanceM,
            sideWallOnThresholdM,
            sideLeftWindowValid);
        snapshot.rightWallObservation = ComputeSideWallObservationHit(
            WallSensorId::SideRight,
            sideRightInput.differentialLight,
            sideRightInput.fallbackDistanceM,
            sideWallOnThresholdM,
            sideRightWindowValid);
        snapshot.frontSkewM = snapshot.frontLeftDistanceM - snapshot.frontRightDistanceM;
        snapshot.corridorErrorM = ComputeCorridorError(snapshot);

#if defined(ARDUINO_TEENSY41)
        const MazeMap::Vehicle::ImuBackLeft::Axes accel = _vehicle.IMU_BL.ReadAccel();
        const float accelXG = _vehicle.IMU_BL.AccelRawToG(accel.x);
        const float accelYG = _vehicle.IMU_BL.AccelRawToG(accel.y);
        if (!_accelBiasInitialized)
        {
            _accelBiasXG = accelXG;
            _accelBiasYG = accelYG;
            _accelBiasInitialized = true;
        }
        if (stationary)
        {
            _accelBiasXG = (0.998f * _accelBiasXG) + (0.002f * accelXG);
            _accelBiasYG = (0.998f * _accelBiasYG) + (0.002f * accelYG);
        }
        const float accelDeltaXG = accelXG - _accelBiasXG;
        const float accelDeltaYG = accelYG - _accelBiasYG;
        snapshot.planarAccelMps2 = kStandardGravityMps2 * std::sqrt((accelDeltaXG * accelDeltaXG) + (accelDeltaYG * accelDeltaYG));
#else
        snapshot.planarAccelMps2 = 0.0f;
#endif

        const float rawGyroRadps = ReadGyroZRadpsRaw();
        if (stationary && MazeMap::ShouldUpdateGyroBiasFromStationarySample(rawGyroRadps, Config::kGyroBiasUpdateMaxAbsRateRadps))
        {
            _gyroBiasRadps = (0.995f * _gyroBiasRadps) + (0.005f * rawGyroRadps);
        }
        snapshot.gyroRadps = rawGyroRadps - _gyroBiasRadps;
        return snapshot;
    }

private:
    struct FilteredIrChannel
    {
        float filteredDistanceM = 0.20f;
        bool wall = false;
        bool initialized = false;
    };

    MazeMap::Vehicle& _vehicle;
    WallDistanceCalibration& _wallCalibration;
    float _gyroBiasRadps;
    FilteredIrChannel _frontLeft;
    FilteredIrChannel _frontRight;
    FilteredIrChannel _sideLeft;
    FilteredIrChannel _sideRight;
    float _frontLeftWallSignalFiltered;
    float _frontRightWallSignalFiltered;
    float _sideLeftWallSignalFiltered;
    float _sideRightWallSignalFiltered;
    float _accelBiasXG;
    float _accelBiasYG;
    AveragedWallSensorInputWindow<Config::kWallDetectionAverageWindowCycles> _frontLeftInputAverage;
    AveragedWallSensorInputWindow<Config::kWallDetectionAverageWindowCycles> _frontRightInputAverage;
    AveragedWallSensorInputWindow<Config::kWallDetectionAverageWindowCycles> _sideLeftInputAverage;
    AveragedWallSensorInputWindow<Config::kWallDetectionAverageWindowCycles> _sideRightInputAverage;
    bool _frontLeftWallState;
    bool _frontRightWallState;
    bool _frontWallUsesFallbackDetection;
    bool _frontLeftWallSignalInitialized;
    bool _frontRightWallSignalInitialized;
    bool _sideLeftWallSignalInitialized;
    bool _sideRightWallSignalInitialized;
    bool _accelBiasInitialized;

    float ComputeCorridorError(const SensorSnapshot& snapshot) const
    {
        if (snapshot.leftWall && snapshot.rightWall)
        {
            return 0.5f * (snapshot.sideLeftDistanceM - snapshot.sideRightDistanceM);
        }
        if (snapshot.leftWall)
        {
            return snapshot.sideLeftDistanceM - _wallCalibration.GetExpectedSideWallDistanceM();
        }
        if (snapshot.rightWall)
        {
            return _wallCalibration.GetExpectedSideWallDistanceM() - snapshot.sideRightDistanceM;
        }
        return 0.0f;
    }

    static float UpdateChannelFromMeasuredDistance(FilteredIrChannel& channel, float measuredDistanceM)
    {
        channel.filteredDistanceM = measuredDistanceM;
        channel.initialized = true;
        return measuredDistanceM;
    }

    static float UpdateChannelFromMeasuredDistance(FilteredIrChannel& channel, float measuredDistanceM, float onThresholdM, float offThresholdM)
    {
        (void)offThresholdM;
        const float currentDistanceM = UpdateChannelFromMeasuredDistance(channel, measuredDistanceM);
        channel.wall = currentDistanceM < onThresholdM;
        return currentDistanceM;
    }

    static bool UpdateFilteredSignalState(
        float measuredDifferentialLight,
        float onMeasuredThreshold,
        float offMeasuredThreshold,
        float& filteredSignal,
        bool& currentState,
        bool& initialized)
    {
        filteredSignal = measuredDifferentialLight;
        initialized = true;
        currentState = MazeMap::HysteresisSignalHigh(
            currentState,
            measuredDifferentialLight,
            onMeasuredThreshold,
            offMeasuredThreshold);
        return currentState;
    }

    static float ComputeSignalRiseAboveBaseline(
        float measuredDifferentialLight,
        float signalBaseline)
    {
        if (!std::isfinite(measuredDifferentialLight) ||
            !std::isfinite(signalBaseline))
        {
            return 0.0f;
        }

        return (measuredDifferentialLight > signalBaseline) ?
            (measuredDifferentialLight - signalBaseline) :
            0.0f;
    }

    bool ComputeSideWallObservationHit(
        WallSensorId sensorId,
        float measuredDifferentialLight,
        float fallbackDistanceM,
        float onThresholdM,
        bool detectionWindowValid) const
    {
        if (!detectionWindowValid)
        {
            return false;
        }

        float onMeasuredThreshold = 0.0f;
        float offMeasuredThreshold = 0.0f;
        float signalBaseline = 0.0f;
        if (_wallCalibration.TryComputeSideWallMeasuredThresholds(
                sensorId,
                Config::kSideWallMeasuredSignalLatchThreshold,
                Config::kSideWallMeasuredSignalReleaseThreshold,
                onMeasuredThreshold,
                offMeasuredThreshold,
                signalBaseline))
        {
            return ComputeSignalRiseAboveBaseline(
                       measuredDifferentialLight,
                       signalBaseline) >= onMeasuredThreshold;
        }

        return std::isfinite(fallbackDistanceM) && fallbackDistanceM < onThresholdM;
    }

    bool UpdateSideWallState(
        WallSensorId sensorId,
        float measuredDifferentialLight,
        float fallbackDistanceM,
        float onThresholdM,
        float offThresholdM,
        bool detectionWindowValid,
        float& filteredSignal,
        bool& signalInitialized,
        bool& currentState)
    {
        if (!detectionWindowValid)
        {
            filteredSignal = 0.0f;
            signalInitialized = false;
            currentState = false;
            return false;
        }

        float onMeasuredThreshold = 0.0f;
        float offMeasuredThreshold = 0.0f;
        float signalBaseline = 0.0f;
        if (_wallCalibration.TryComputeSideWallMeasuredThresholds(
                sensorId,
                Config::kSideWallMeasuredSignalLatchThreshold,
                Config::kSideWallMeasuredSignalReleaseThreshold,
                onMeasuredThreshold,
                offMeasuredThreshold,
                signalBaseline))
        {
            return UpdateFilteredSignalState(
                ComputeSignalRiseAboveBaseline(
                    measuredDifferentialLight,
                    signalBaseline),
                onMeasuredThreshold,
                offMeasuredThreshold,
                filteredSignal,
                currentState,
                signalInitialized);
        }

        signalInitialized = false;
        currentState = HysteresisWall(
            currentState,
            fallbackDistanceM,
            onThresholdM,
            offThresholdM);
        return currentState;
    }

    bool UpdateFrontWallState(
        float leftAmbientLight,
        float leftMeasuredDifferentialLight,
        float rightAmbientLight,
        float rightMeasuredDifferentialLight,
        float fallbackDistanceM)
    {
        float leftOnMeasuredThreshold = 0.0f;
        float leftOffMeasuredThreshold = 0.0f;
        float leftSignalBaseline = 0.0f;
        float rightOnMeasuredThreshold = 0.0f;
        float rightOffMeasuredThreshold = 0.0f;
        float rightSignalBaseline = 0.0f;
        const bool haveLeftThreshold =
            _wallCalibration.TryComputeFrontSensorMeasuredThresholds(
                WallSensorId::FrontLeft,
                _vehicle,
                Config::kFrontWallReleaseHysteresisM,
                leftAmbientLight,
                leftOnMeasuredThreshold,
                leftOffMeasuredThreshold,
                leftSignalBaseline);
        const bool haveRightThreshold =
            _wallCalibration.TryComputeFrontSensorMeasuredThresholds(
                WallSensorId::FrontRight,
                _vehicle,
                Config::kFrontWallReleaseHysteresisM,
                rightAmbientLight,
                rightOnMeasuredThreshold,
                rightOffMeasuredThreshold,
                rightSignalBaseline);

        if (haveLeftThreshold || haveRightThreshold)
        {
            _frontWallUsesFallbackDetection = false;
            if (haveLeftThreshold)
            {
                UpdateFilteredSignalState(
                    ComputeSignalRiseAboveBaseline(
                        leftMeasuredDifferentialLight,
                        leftSignalBaseline),
                    leftOnMeasuredThreshold,
                    leftOffMeasuredThreshold,
                    _frontLeftWallSignalFiltered,
                    _frontLeftWallState,
                    _frontLeftWallSignalInitialized);
            }
            else
            {
                _frontLeftWallState = false;
                _frontLeftWallSignalInitialized = false;
            }

            if (haveRightThreshold)
            {
                UpdateFilteredSignalState(
                    ComputeSignalRiseAboveBaseline(
                        rightMeasuredDifferentialLight,
                        rightSignalBaseline),
                    rightOnMeasuredThreshold,
                    rightOffMeasuredThreshold,
                    _frontRightWallSignalFiltered,
                    _frontRightWallState,
                    _frontRightWallSignalInitialized);
            }
            else
            {
                _frontRightWallState = false;
                _frontRightWallSignalInitialized = false;
            }

            return _frontLeftWallState || _frontRightWallState;
        }

        const bool fallbackState = HysteresisWall(
            _frontLeftWallState || _frontRightWallState,
            fallbackDistanceM,
            Config::kFrontWallOnThresholdM,
            Config::kFrontWallOffThresholdM);
        _frontWallUsesFallbackDetection = true;
        _frontLeftWallState = fallbackState;
        _frontRightWallState = fallbackState;
        _frontLeftWallSignalInitialized = false;
        _frontRightWallSignalInitialized = false;
        return fallbackState;
    }

    float ReadGyroZRadpsRaw()
    {
        return ReadBackLeftGyroZRadpsRaw(_vehicle);
    }
};

static MazeMap::WheelControlProfile BuildNominalWheelControlProfile()
{
    MazeMap::WheelControlProfile profile{};
    profile.velocityKpScale = Config::kNominalWheelVelocityKpScale;
    profile.velocityKiScale = Config::kNominalWheelVelocityKiScale;
    profile.integralLimitScale = Config::kNominalWheelIntegralLimitScale;
    return profile;
}

static MazeMap::InPlaceTurnProfile BuildSharedInPlaceTurnProfile(float maxAngularSpeedRadps, float angularAccelRadps2)
{
    MazeMap::InPlaceTurnProfile profile{};
    profile.maxAngularSpeedRadps = maxAngularSpeedRadps;
    profile.angularAccelRadps2 = angularAccelRadps2;
    profile.headingKp = Config::kTurnHeadingKp;
    profile.yawD = Config::kTurnYawD;
    profile.angleToleranceRad = Config::kAngleToleranceRad;
    profile.angularSpeedToleranceRadps = Config::kAngularSpeedToleranceRadps;
    return profile;
}

static MazeMap::InPlaceTurnProfile BuildSharedInPlaceTurnProfile(const MazeMap::Vehicle& vehicle)
{
    return BuildSharedInPlaceTurnProfile(
        vehicle.GetMaxRotationalVelocity(),
        vehicle.GetMaxAngularAcceleration());
}

class DriveBase
{
public:
    DriveBase()
        : _leftMotor(MazeMap::MotorEncoderDrive::CreateDefaultLeftDrive())
        , _rightMotor(MazeMap::MotorEncoderDrive::CreateDefaultRightDrive())
        , _pose{}
        , _lastLeftDistanceM(0.0f)
        , _lastRightDistanceM(0.0f)
        , _leftIntegral(0.0f)
        , _rightIntegral(0.0f)
        , _lastLinearCommandMps(0.0f)
        , _lastAngularCommandRadps(0.0f)
        , _wheelControlProfile(BuildNominalWheelControlProfile())
    {
    }

    bool Begin()
    {
        const bool leftOk = _leftMotor.begin();
        const bool rightOk = _rightMotor.begin();
        _leftMotor.resetEncoderDistanceMeters();
        _rightMotor.resetEncoderDistanceMeters();
        _lastLeftDistanceM = _leftMotor.getEncoderDistanceMeters();
        _lastRightDistanceM = _rightMotor.getEncoderDistanceMeters();
        _pose = PoseEstimate{ 0.0f, 0.0f, DirectionToUnitVector(MazeMap::Up), DirectionToYawRad(MazeMap::Up), 0.0f, 0.0f };
        ResetControllers();
        Brake();
        return leftOk && rightOk;
    }

    void ResetControllers()
    {
        _leftIntegral = 0.0f;
        _rightIntegral = 0.0f;
        ResetLaunchAssist();
    }

    void SetWheelControlProfile(const MazeMap::WheelControlProfile& profile)
    {
        _wheelControlProfile = MazeMap::NormalizeWheelControlProfile(profile);
        const float integralLimit = GetWheelIntegralLimit();
        _leftIntegral = (std::clamp)(_leftIntegral, -integralLimit, integralLimit);
        _rightIntegral = (std::clamp)(_rightIntegral, -integralLimit, integralLimit);
    }

    void UseNominalWheelControlProfile()
    {
        SetWheelControlProfile(BuildNominalWheelControlProfile());
    }

    void SnapTo(MazeMap::DirectionalLocation logical)
    {
        logical.GetLocation().GetPhysicalLocation(Config::kCellSizeM, _pose.xMeters, _pose.yMeters);
        _pose.headingUnit = DirectionToUnitVector(logical.GetDirection());
        _pose.yawRad = DirectionToYawRad(logical.GetDirection());
        _pose.linearSpeedMps = 0.0f;
        _pose.angularSpeedRadps = 0.0f;
        _lastLeftDistanceM = _leftMotor.getEncoderDistanceMeters();
        _lastRightDistanceM = _rightMotor.getEncoderDistanceMeters();
        ResetControllers();
    }

    void SetPose(float xMeters, float yMeters, float yawRad)
    {
        _pose.xMeters = xMeters;
        _pose.yMeters = yMeters;
        _pose.yawRad = WrapAngleRad(yawRad);
        _pose.headingUnit = HeadingUnitFromYawRad(_pose.yawRad);
        _pose.linearSpeedMps = 0.0f;
        _pose.angularSpeedRadps = 0.0f;
        _lastLeftDistanceM = _leftMotor.getEncoderDistanceMeters();
        _lastRightDistanceM = _rightMotor.getEncoderDistanceMeters();
        ResetControllers();
    }

    void SetPoseXMeters(float xMeters)
    {
        if (std::isfinite(xMeters))
        {
            _pose.xMeters = xMeters;
        }
    }

    void SetPoseYMeters(float yMeters)
    {
        if (std::isfinite(yMeters))
        {
            _pose.yMeters = yMeters;
        }
    }

    void UpdateOdometry(float dtSeconds, float gyroRadps)
    {
        const float leftDistanceM = _leftMotor.getEncoderDistanceMeters();
        const float rightDistanceM = _rightMotor.getEncoderDistanceMeters();
        const float leftDeltaM = leftDistanceM - _lastLeftDistanceM;
        const float rightDeltaM = rightDistanceM - _lastRightDistanceM;
        _lastLeftDistanceM = leftDistanceM;
        _lastRightDistanceM = rightDistanceM;

        const float centerDeltaM = 0.5f * (leftDeltaM + rightDeltaM);
        const float measuredLinearSpeedMps = (dtSeconds > 0.0f) ? (centerDeltaM / dtSeconds) : 0.0f;
        const float effectiveTrackWidthM = MazeMap::Vehicle::GetEffectiveTrackWidthForMotion(measuredLinearSpeedMps, gyroRadps);
        const float encoderYawDeltaRad =
            (effectiveTrackWidthM > 0.0f) ? ((rightDeltaM - leftDeltaM) / effectiveTrackWidthM) : 0.0f;
        const float predictedYawRad = _pose.yawRad + (gyroRadps * dtSeconds);
        const float encoderYawRad = _pose.yawRad + encoderYawDeltaRad;
        _pose.yawRad = WrapAngleRad((0.98f * predictedYawRad) + (0.02f * encoderYawRad));
        _pose.headingUnit = HeadingUnitFromYawRad(_pose.yawRad);
        _pose.xMeters += centerDeltaM * _pose.headingUnit.GetX();
        _pose.yMeters += centerDeltaM * _pose.headingUnit.GetY();
        _pose.linearSpeedMps = measuredLinearSpeedMps;
        _pose.angularSpeedRadps = gyroRadps;
    }

    void CommandVelocity(float linearSpeedMps, float angularSpeedRadps, float dtSeconds)
    {
        _lastLinearCommandMps = linearSpeedMps;
        _lastAngularCommandRadps = angularSpeedRadps;
        const float effectiveTrackWidthM = MazeMap::Vehicle::GetEffectiveTrackWidthForMotion(linearSpeedMps, angularSpeedRadps);
        const float leftTargetMps = linearSpeedMps - (0.5f * effectiveTrackWidthM * angularSpeedRadps);
        const float rightTargetMps = linearSpeedMps + (0.5f * effectiveTrackWidthM * angularSpeedRadps);
        const float leftMeasuredMps = _leftMotor.getEncoderVelocityMetersPerSecond();
        const float rightMeasuredMps = _rightMotor.getEncoderVelocityMetersPerSecond();
        const unsigned long nowMs = millis();
        const bool leftLaunchAssistActive = UpdateWheelLaunchAssistState(_leftLaunchAssist, leftMeasuredMps, leftTargetMps, _leftMotor.getDriveCommand(), nowMs);
        const bool rightLaunchAssistActive = UpdateWheelLaunchAssistState(_rightLaunchAssist, rightMeasuredMps, rightTargetMps, _rightMotor.getDriveCommand(), nowMs);

        const float leftErrorMps = leftTargetMps - leftMeasuredMps;
        const float rightErrorMps = rightTargetMps - rightMeasuredMps;
        const float integralLimit = GetWheelIntegralLimit();

        _leftIntegral = (std::clamp)(_leftIntegral + (leftErrorMps * dtSeconds), -integralLimit, integralLimit);
        _rightIntegral = (std::clamp)(_rightIntegral + (rightErrorMps * dtSeconds), -integralLimit, integralLimit);

        float leftCommand = VelocityCommandFromError(leftTargetMps, leftErrorMps, _leftIntegral);
        float rightCommand = VelocityCommandFromError(rightTargetMps, rightErrorMps, _rightIntegral);
        if (leftLaunchAssistActive)
        {
            leftCommand = ApplyLaunchAssistFloor(
                leftCommand,
                leftTargetMps,
                GetWheelLaunchAssistFloor(_leftLaunchAssist, nowMs));
        }
        if (rightLaunchAssistActive)
        {
            rightCommand = ApplyLaunchAssistFloor(
                rightCommand,
                rightTargetMps,
                GetWheelLaunchAssistFloor(_rightLaunchAssist, nowMs));
        }

        _leftMotor.setDriveCommand(leftCommand);
        _rightMotor.setDriveCommand(rightCommand);
    }

    void CommandOpenLoop(float leftDriveCommand, float rightDriveCommand)
    {
        const float leftMeasuredMps = _leftMotor.getEncoderVelocityMetersPerSecond();
        const float rightMeasuredMps = _rightMotor.getEncoderVelocityMetersPerSecond();
        const unsigned long nowMs = millis();
        if (UpdateWheelLaunchAssistState(_leftLaunchAssist, leftMeasuredMps, leftDriveCommand, _leftMotor.getDriveCommand(), nowMs))
        {
            leftDriveCommand = ApplyLaunchAssistFloor(
                leftDriveCommand,
                leftDriveCommand,
                GetWheelLaunchAssistFloor(_leftLaunchAssist, nowMs));
        }
        if (UpdateWheelLaunchAssistState(_rightLaunchAssist, rightMeasuredMps, rightDriveCommand, _rightMotor.getDriveCommand(), nowMs))
        {
            rightDriveCommand = ApplyLaunchAssistFloor(
                rightDriveCommand,
                rightDriveCommand,
                GetWheelLaunchAssistFloor(_rightLaunchAssist, nowMs));
        }

        SetOpenLoopRaw(leftDriveCommand, rightDriveCommand);
    }

    void CommandOpenLoop(const MazeMap::OpenLoopDriveCommand& command)
    {
        CommandOpenLoop(command.leftDriveCommand, command.rightDriveCommand);
    }

    void CommandOpenLoopRaw(float leftDriveCommand, float rightDriveCommand)
    {
        _lastLinearCommandMps = 0.0f;
        _lastAngularCommandRadps = 0.0f;
        ResetLaunchAssist();
        SetOpenLoopRaw(leftDriveCommand, rightDriveCommand);
    }

    void CommandOpenLoopRaw(const MazeMap::OpenLoopDriveCommand& command)
    {
        CommandOpenLoopRaw(command.leftDriveCommand, command.rightDriveCommand);
    }

    void Brake()
    {
        _lastLinearCommandMps = 0.0f;
        _lastAngularCommandRadps = 0.0f;
        ResetLaunchAssist();
        _leftMotor.brake();
        _rightMotor.brake();
    }

    float GetAverageDistanceMeters() const
    {
        return 0.5f * (_leftMotor.getEncoderDistanceMeters() + _rightMotor.getEncoderDistanceMeters());
    }

    const PoseEstimate& GetPose() const
    {
        return _pose;
    }

    float GetLastLinearCommandMps() const
    {
        return _lastLinearCommandMps;
    }

    float GetLastAngularCommandRadps() const
    {
        return _lastAngularCommandRadps;
    }

    DriveTelemetry GetTelemetry() const
    {
        DriveTelemetry telemetry{};
        telemetry.leftDriveCommand = _leftMotor.getDriveCommand();
        telemetry.rightDriveCommand = _rightMotor.getDriveCommand();
        telemetry.leftEncoderCount = _leftMotor.getEncoderCount();
        telemetry.rightEncoderCount = _rightMotor.getEncoderCount();
        telemetry.leftDistanceM = _leftMotor.getEncoderDistanceMeters();
        telemetry.rightDistanceM = _rightMotor.getEncoderDistanceMeters();
        telemetry.leftVelocityMps = _leftMotor.getEncoderVelocityMetersPerSecond();
        telemetry.rightVelocityMps = _rightMotor.getEncoderVelocityMetersPerSecond();
        return telemetry;
    }

private:
    void SetOpenLoopRaw(float leftDriveCommand, float rightDriveCommand)
    {
        _leftMotor.setDriveCommand((std::clamp)(leftDriveCommand, -1.0f, 1.0f));
        _rightMotor.setDriveCommand((std::clamp)(rightDriveCommand, -1.0f, 1.0f));
    }
    MazeMap::MotorEncoderDrive _leftMotor;
    MazeMap::MotorEncoderDrive _rightMotor;
    PoseEstimate _pose;
    float _lastLeftDistanceM;
    float _lastRightDistanceM;
    float _leftIntegral;
    float _rightIntegral;
    float _lastLinearCommandMps;
    float _lastAngularCommandRadps;
    MazeMap::WheelControlProfile _wheelControlProfile;
    struct WheelLaunchAssistState
    {
        bool active = false;
        unsigned long startMs = 0UL;
        float requestedDirection = 0.0f;
    };
    WheelLaunchAssistState _leftLaunchAssist;
    WheelLaunchAssistState _rightLaunchAssist;

    float VelocityCommandFromError(float targetSpeedMps, float errorMps, float integral) const
    {
        float command = 0.0f;
        if (std::fabs(targetSpeedMps) > 0.01f)
        {
            command += SignF(targetSpeedMps) * Config::kWheelStaticFeedforward;
        }
        command += Config::kWheelVelocityFeedforward * targetSpeedMps;
        command += GetWheelVelocityKp() * errorMps;
        command += GetWheelVelocityKi() * integral;
        return (std::clamp)(command, -1.0f, 1.0f);
    }

    float GetWheelVelocityKp() const
    {
        return MazeMap::ScaleWheelControlValue(Config::kWheelVelocityKp, _wheelControlProfile.velocityKpScale);
    }

    float GetWheelVelocityKi() const
    {
        return MazeMap::ScaleWheelControlValue(Config::kWheelVelocityKi, _wheelControlProfile.velocityKiScale);
    }

    float GetWheelIntegralLimit() const
    {
        return MazeMap::ScaleWheelControlValue(Config::kWheelIntegralLimit, _wheelControlProfile.integralLimitScale);
    }

    void ResetLaunchAssist()
    {
        _leftLaunchAssist = WheelLaunchAssistState{};
        _rightLaunchAssist = WheelLaunchAssistState{};
    }

    static void ResetWheelLaunchAssistState(WheelLaunchAssistState& state, unsigned long nowMs)
    {
        state.active = false;
        state.startMs = nowMs;
        state.requestedDirection = 0.0f;
    }

    static bool UpdateWheelLaunchAssistState(
        WheelLaunchAssistState& state,
        float measuredMps,
        float requestedDirection,
        float priorDriveCommand,
        unsigned long nowMs)
    {
        if ((std::fabs(requestedDirection) <= 0.01f) ||
            (std::fabs(measuredMps) > Config::kWheelRestLaunchSpeedThresholdMps))
        {
            ResetWheelLaunchAssistState(state, nowMs);
            return false;
        }

        if (!state.active)
        {
            if (std::fabs(priorDriveCommand) > Config::kWheelRestLaunchDriveThreshold)
            {
                return false;
            }

            state.active = true;
            state.startMs = nowMs;
            state.requestedDirection = SignF(requestedDirection);
            return true;
        }

        const float requestedSign = SignF(requestedDirection);
        if ((requestedSign != 0.0f) &&
            (state.requestedDirection != 0.0f) &&
            (requestedSign != state.requestedDirection))
        {
            state.startMs = nowMs;
            state.requestedDirection = requestedSign;
        }
        else if ((state.requestedDirection == 0.0f) && (requestedSign != 0.0f))
        {
            state.startMs = nowMs;
            state.requestedDirection = requestedSign;
        }

        return true;
    }

    static float GetWheelLaunchAssistFloor(const WheelLaunchAssistState& state, unsigned long nowMs)
    {
        if (!state.active)
        {
            return 0.0f;
        }

        return MazeMap::ComputeLaunchAssistDriveFloor(
            Config::kWheelRestLaunchDriveCommand,
            Config::kWheelRestLaunchMaxDriveCommand,
            nowMs - state.startMs,
            Config::kWheelRestLaunchRampMs);
    }

    static float ApplyLaunchAssistFloor(float command, float requestedDirection, float launchFloor)
    {
        if ((std::fabs(requestedDirection) <= 0.01f) || !(launchFloor > 0.0f))
        {
            return command;
        }

        const float magnitude = std::fabs(command);
        if (magnitude >= launchFloor)
        {
            return command;
        }

        return SignF(requestedDirection) * launchFloor;
    }

};

class DiagnosticSensorSuite
{
public:
    DiagnosticSensorSuite(MazeMap::Vehicle& vehicle, WallDistanceCalibration& wallCalibration)
        : _vehicle(vehicle)
        , _wallCalibration(wallCalibration)
        , _gyroBiasRadps(0.0f)
        , _accelBiasXG(0.0f)
        , _accelBiasYG(0.0f)
        , _frontLeftWallSignalFiltered(0.0f)
        , _frontRightWallSignalFiltered(0.0f)
        , _sideLeftWallSignalFiltered(0.0f)
        , _sideRightWallSignalFiltered(0.0f)
        , _frontLeftWallState(false)
        , _frontRightWallState(false)
        , _leftWallState(false)
        , _rightWallState(false)
        , _frontLeftWallSignalInitialized(false)
        , _frontRightWallSignalInitialized(false)
        , _sideLeftWallSignalInitialized(false)
        , _sideRightWallSignalInitialized(false)
        , _accelBiasInitialized(false)
    {
    }

    bool Begin(unsigned long controlPeriodUs = DiagnosticConfig::kControlPeriodUs)
    {
        bool ok = true;
        _frontLeftWallSignalFiltered = 0.0f;
        _frontRightWallSignalFiltered = 0.0f;
        _sideLeftWallSignalFiltered = 0.0f;
        _sideRightWallSignalFiltered = 0.0f;
        _frontLeftInputAverage.Clear();
        _frontRightInputAverage.Clear();
        _sideLeftInputAverage.Clear();
        _sideRightInputAverage.Clear();
        _frontLeftWallState = false;
        _frontRightWallState = false;
        _leftWallState = false;
        _rightWallState = false;
        _frontLeftWallSignalInitialized = false;
        _frontRightWallSignalInitialized = false;
        _sideLeftWallSignalInitialized = false;
        _sideRightWallSignalInitialized = false;
        _accelBiasInitialized = false;
#if defined(ARDUINO_TEENSY41)
        Serial.println("IMU_FR disabled; using IMU_BL only");

        const bool imuBackLeftOk = _vehicle.IMU_BL.Begin();
        if (!imuBackLeftOk)
        {
            Serial.print("IMU_BL init failed (");
            Serial.print(_vehicle.IMU_BL.GetLastBeginFailureReasonName());
            Serial.print("), WHO_AM_I=0x");
            const uint8_t whoAmI = _vehicle.IMU_BL.GetLastWhoAmI();
            PrintHexByte(whoAmI);
            Serial.print(", INT1_pullup=");
            Serial.print(ReadDrivenLowPinWithPullup(Pins::IMU_INT_1B) == LOW ? "low" : "high");
            Serial.print(", WHO_AM_I@mode3/400kHz=0x");
            PrintHexByte(_vehicle.IMU_BL.ReadWhoAmIWithSettings(400000UL, SPI_MODE3));
            Serial.print(", WHO_AM_I@mode0/400kHz=0x");
            PrintHexByte(_vehicle.IMU_BL.ReadWhoAmIWithSettings(400000UL, SPI_MODE0));
            Serial.println();
        }
        ok = imuBackLeftOk && ok;
#endif
        if (ok)
        {
            ok = CalibrateGyroBias(controlPeriodUs, true) && ok;
        }
        return ok;
    }

    bool CalibrateGyroBias(unsigned long controlPeriodUs, bool enableAccelRuntime)
    {
        return CalibrateStationaryBackLeftGyroBias(
            _vehicle,
            controlPeriodUs,
            enableAccelRuntime,
            _gyroBiasRadps);
    }

    DiagnosticSensorSnapshot Capture(bool stationary, const PoseEstimate& pose)
    {
        DiagnosticSensorSnapshot snapshot{};
        WallSensorCalibrationInput frontLeftRawInput{};
        WallSensorCalibrationInput frontRightRawInput{};
        SampleWallCalibrationInputRawPair(
            WallSensorId::FrontLeft,
            _vehicle.FrontLeft,
            WallSensorId::FrontRight,
            _vehicle.FrontRight,
            frontLeftRawInput,
            frontRightRawInput);
        const WallSensorCalibrationInput frontLeftInput = _frontLeftInputAverage.PushAndAverage(frontLeftRawInput);
        const WallSensorCalibrationInput frontRightInput = _frontRightInputAverage.PushAndAverage(frontRightRawInput);
        const WallSensorCalibrationInput sideLeftInput = _sideLeftInputAverage.PushAndAverage(
            SampleWallCalibrationInputRaw(WallSensorId::SideLeft, _vehicle.SideLeft));
        const WallSensorCalibrationInput sideRightInput = _sideRightInputAverage.PushAndAverage(
            SampleWallCalibrationInputRaw(WallSensorId::SideRight, _vehicle.SideRight));
        snapshot.frontLeft = BuildWallSensorTelemetry(WallSensorId::FrontLeft, frontLeftInput);
        snapshot.frontRight = BuildWallSensorTelemetry(WallSensorId::FrontRight, frontRightInput);
        snapshot.sideLeft = BuildWallSensorTelemetry(WallSensorId::SideLeft, sideLeftInput);
        snapshot.sideRight = BuildWallSensorTelemetry(WallSensorId::SideRight, sideRightInput);
        (void)TryComputeSideWallSignalDistanceM(
            _wallCalibration,
            WallSensorId::SideLeft,
            sideLeftInput.differentialLight,
            snapshot.sideLeft.distanceM);
        (void)TryComputeSideWallSignalDistanceM(
            _wallCalibration,
            WallSensorId::SideRight,
            sideRightInput.differentialLight,
            snapshot.sideRight.distanceM);
        snapshot.imuFrontRight = {};
        snapshot.imuBackLeft = CaptureImu(_vehicle.IMU_BL, Pins::IMU_INT_1B);
        float sideWallOnThresholdM = Config::kSideWallOnThresholdM;
        float sideWallOffThresholdM = Config::kSideWallOffThresholdM;
        _wallCalibration.TryComputeSideWallDistanceThresholds(
            Config::kSideWallDistanceLatchFractionOfCalibration,
            Config::kSideWallDistanceReleaseFractionOfCalibration,
            sideWallOnThresholdM,
            sideWallOffThresholdM);

        snapshot.frontWall = UpdateFrontWallState(
            frontLeftInput.ambientLight,
            frontLeftInput.measuredValue,
            frontRightInput.ambientLight,
            frontRightInput.measuredValue,
            (std::min)(snapshot.frontLeft.distanceM, snapshot.frontRight.distanceM));
        const bool sideLeftWindowValid = IsSideWallDetectionWindowValid(pose, _vehicle.SideLeft);
        const bool sideRightWindowValid = IsSideWallDetectionWindowValid(pose, _vehicle.SideRight);
        snapshot.leftWall = UpdateSideWallState(
            WallSensorId::SideLeft,
            sideLeftInput.differentialLight,
            sideLeftInput.fallbackDistanceM,
            sideWallOnThresholdM,
            sideWallOffThresholdM,
            sideLeftWindowValid,
            _sideLeftWallSignalFiltered,
            _sideLeftWallSignalInitialized,
            _leftWallState);
        snapshot.rightWall = UpdateSideWallState(
            WallSensorId::SideRight,
            sideRightInput.differentialLight,
            sideRightInput.fallbackDistanceM,
            sideWallOnThresholdM,
            sideWallOffThresholdM,
            sideRightWindowValid,
            _sideRightWallSignalFiltered,
            _sideRightWallSignalInitialized,
            _rightWallState);

        snapshot.frontLeft.wall = snapshot.frontWall && (snapshot.frontLeft.distanceM <= snapshot.frontRight.distanceM);
        snapshot.frontRight.wall = snapshot.frontWall && (snapshot.frontRight.distanceM <= snapshot.frontLeft.distanceM);
        snapshot.sideLeft.wall = snapshot.leftWall;
        snapshot.sideRight.wall = snapshot.rightWall;
        snapshot.frontSkewM = snapshot.frontLeft.distanceM - snapshot.frontRight.distanceM;
        snapshot.corridorErrorM = ComputeCorridorError(snapshot);

        const float blGyroZRadps = _vehicle.IMU_BL.GyroRawToDps(snapshot.imuBackLeft.gyroZ) * DEG_TO_RAD;
        const float accelXG = _vehicle.IMU_BL.AccelRawToG(snapshot.imuBackLeft.accelX);
        const float accelYG = _vehicle.IMU_BL.AccelRawToG(snapshot.imuBackLeft.accelY);
        if (!_accelBiasInitialized)
        {
            _accelBiasXG = accelXG;
            _accelBiasYG = accelYG;
            _accelBiasInitialized = true;
        }
        if (stationary)
        {
            _accelBiasXG = (0.998f * _accelBiasXG) + (0.002f * accelXG);
            _accelBiasYG = (0.998f * _accelBiasYG) + (0.002f * accelYG);
        }
        snapshot.gyroRawRadps = blGyroZRadps;
        if (stationary && MazeMap::ShouldUpdateGyroBiasFromStationarySample(snapshot.gyroRawRadps, Config::kGyroBiasUpdateMaxAbsRateRadps))
        {
            _gyroBiasRadps = (0.998f * _gyroBiasRadps) + (0.002f * snapshot.gyroRawRadps);
        }
        snapshot.gyroBiasRadps = _gyroBiasRadps;
        snapshot.gyroRadps = snapshot.gyroRawRadps - snapshot.gyroBiasRadps;
        return snapshot;
    }

    float GetGyroSensitivityMdpsPerLsb() const
    {
        return _vehicle.IMU_BL.GyroSensitivityMdpsPerLsb();
    }

    float GetAccelSensitivityMgPerLsb() const
    {
        return _vehicle.IMU_BL.AccelSensitivityMgPerLsb();
    }

    float GetGyroBiasRadps() const
    {
        return _gyroBiasRadps;
    }

    float GetPlanarAccelMps2(const DiagnosticSensorSnapshot& snapshot) const
    {
        const float accelXG = _vehicle.IMU_BL.AccelRawToG(snapshot.imuBackLeft.accelX) - _accelBiasXG;
        const float accelYG = _vehicle.IMU_BL.AccelRawToG(snapshot.imuBackLeft.accelY) - _accelBiasYG;
        return kStandardGravityMps2 * std::sqrt((accelXG * accelXG) + (accelYG * accelYG));
    }

private:
    MazeMap::Vehicle& _vehicle;
    WallDistanceCalibration& _wallCalibration;
    float _gyroBiasRadps;
    float _accelBiasXG;
    float _accelBiasYG;
    float _frontLeftWallSignalFiltered;
    float _frontRightWallSignalFiltered;
    float _sideLeftWallSignalFiltered;
    float _sideRightWallSignalFiltered;
    AveragedWallSensorInputWindow<Config::kWallDetectionAverageWindowCycles> _frontLeftInputAverage;
    AveragedWallSensorInputWindow<Config::kWallDetectionAverageWindowCycles> _frontRightInputAverage;
    AveragedWallSensorInputWindow<Config::kWallDetectionAverageWindowCycles> _sideLeftInputAverage;
    AveragedWallSensorInputWindow<Config::kWallDetectionAverageWindowCycles> _sideRightInputAverage;
    bool _frontLeftWallState;
    bool _frontRightWallState;
    bool _leftWallState;
    bool _rightWallState;
    bool _frontLeftWallSignalInitialized;
    bool _frontRightWallSignalInitialized;
    bool _sideLeftWallSignalInitialized;
    bool _sideRightWallSignalInitialized;
    bool _accelBiasInitialized;

    float ComputeCorridorError(const DiagnosticSensorSnapshot& snapshot) const
    {
        if (snapshot.leftWall && snapshot.rightWall)
        {
            return 0.5f * (snapshot.sideLeft.distanceM - snapshot.sideRight.distanceM);
        }
        if (snapshot.leftWall)
        {
            return snapshot.sideLeft.distanceM - _wallCalibration.GetExpectedSideWallDistanceM();
        }
        if (snapshot.rightWall)
        {
            return _wallCalibration.GetExpectedSideWallDistanceM() - snapshot.sideRight.distanceM;
        }
        return 0.0f;
    }

    static bool UpdateFilteredSignalState(
        float measuredDifferentialLight,
        float onMeasuredThreshold,
        float offMeasuredThreshold,
        float& filteredSignal,
        bool& currentState,
        bool& initialized)
    {
        filteredSignal = measuredDifferentialLight;
        initialized = true;
        currentState = MazeMap::HysteresisSignalHigh(
            currentState,
            measuredDifferentialLight,
            onMeasuredThreshold,
            offMeasuredThreshold);
        return currentState;
    }

    static float ComputeSignalRiseAboveBaseline(
        float measuredDifferentialLight,
        float signalBaseline)
    {
        if (!std::isfinite(measuredDifferentialLight) ||
            !std::isfinite(signalBaseline))
        {
            return 0.0f;
        }

        return (measuredDifferentialLight > signalBaseline) ?
            (measuredDifferentialLight - signalBaseline) :
            0.0f;
    }

    bool UpdateSideWallState(
        WallSensorId sensorId,
        float measuredDifferentialLight,
        float fallbackDistanceM,
        float onThresholdM,
        float offThresholdM,
        bool detectionWindowValid,
        float& filteredSignal,
        bool& signalInitialized,
        bool& currentState)
    {
        if (!detectionWindowValid)
        {
            filteredSignal = 0.0f;
            signalInitialized = false;
            currentState = false;
            return false;
        }

        float onMeasuredThreshold = 0.0f;
        float offMeasuredThreshold = 0.0f;
        float signalBaseline = 0.0f;
        if (_wallCalibration.TryComputeSideWallMeasuredThresholds(
                sensorId,
                Config::kSideWallMeasuredSignalLatchThreshold,
                Config::kSideWallMeasuredSignalReleaseThreshold,
                onMeasuredThreshold,
                offMeasuredThreshold,
                signalBaseline))
        {
            return UpdateFilteredSignalState(
                ComputeSignalRiseAboveBaseline(
                    measuredDifferentialLight,
                    signalBaseline),
                onMeasuredThreshold,
                offMeasuredThreshold,
                filteredSignal,
                currentState,
                signalInitialized);
        }

        signalInitialized = false;
        currentState = HysteresisWall(
            currentState,
            fallbackDistanceM,
            onThresholdM,
            offThresholdM);
        return currentState;
    }

    bool UpdateFrontWallState(
        float leftAmbientLight,
        float leftMeasuredDifferentialLight,
        float rightAmbientLight,
        float rightMeasuredDifferentialLight,
        float fallbackDistanceM)
    {
        float leftOnMeasuredThreshold = 0.0f;
        float leftOffMeasuredThreshold = 0.0f;
        float leftSignalBaseline = 0.0f;
        float rightOnMeasuredThreshold = 0.0f;
        float rightOffMeasuredThreshold = 0.0f;
        float rightSignalBaseline = 0.0f;
        const bool haveLeftThreshold =
            _wallCalibration.TryComputeFrontSensorMeasuredThresholds(
                WallSensorId::FrontLeft,
                _vehicle,
                Config::kFrontWallReleaseHysteresisM,
                leftAmbientLight,
                leftOnMeasuredThreshold,
                leftOffMeasuredThreshold,
                leftSignalBaseline);
        const bool haveRightThreshold =
            _wallCalibration.TryComputeFrontSensorMeasuredThresholds(
                WallSensorId::FrontRight,
                _vehicle,
                Config::kFrontWallReleaseHysteresisM,
                rightAmbientLight,
                rightOnMeasuredThreshold,
                rightOffMeasuredThreshold,
                rightSignalBaseline);

        if (haveLeftThreshold || haveRightThreshold)
        {
            if (haveLeftThreshold)
            {
                UpdateFilteredSignalState(
                    ComputeSignalRiseAboveBaseline(
                        leftMeasuredDifferentialLight,
                        leftSignalBaseline),
                    leftOnMeasuredThreshold,
                    leftOffMeasuredThreshold,
                    _frontLeftWallSignalFiltered,
                    _frontLeftWallState,
                    _frontLeftWallSignalInitialized);
            }
            else
            {
                _frontLeftWallState = false;
                _frontLeftWallSignalInitialized = false;
            }

            if (haveRightThreshold)
            {
                UpdateFilteredSignalState(
                    ComputeSignalRiseAboveBaseline(
                        rightMeasuredDifferentialLight,
                        rightSignalBaseline),
                    rightOnMeasuredThreshold,
                    rightOffMeasuredThreshold,
                    _frontRightWallSignalFiltered,
                    _frontRightWallState,
                    _frontRightWallSignalInitialized);
            }
            else
            {
                _frontRightWallState = false;
                _frontRightWallSignalInitialized = false;
            }

            return _frontLeftWallState || _frontRightWallState;
        }

        const bool fallbackState = HysteresisWall(
            _frontLeftWallState || _frontRightWallState,
            fallbackDistanceM,
            Config::kFrontWallOnThresholdM,
            Config::kFrontWallOffThresholdM);
        _frontLeftWallState = fallbackState;
        _frontRightWallState = fallbackState;
        _frontLeftWallSignalInitialized = false;
        _frontRightWallSignalInitialized = false;
        return fallbackState;
    }

    WallSensorTelemetry BuildWallSensorTelemetry(WallSensorId sensorId, const WallSensorCalibrationInput& input) const
    {
        WallSensorTelemetry telemetry{};
        telemetry.ambientLight = input.ambientLight;
        telemetry.litLight = input.litLight;
        telemetry.differentialLight = input.differentialLight;
        telemetry.rawDistanceM = input.fallbackDistanceM;
        telemetry.distanceM = _wallCalibration.Apply(sensorId, input.measuredValue, input.fallbackDistanceM);
        return telemetry;
    }

    WallSensorTelemetry SampleWallSensor(WallSensorId sensorId, const MazeMap::WallSensor& sensor) const
    {
        return BuildWallSensorTelemetry(sensorId, SampleWallCalibrationInputRaw(sensorId, sensor));
    }

    template <typename TImu>
    static ImuTelemetry CaptureImu(TImu& imu, uint8_t interruptPin)
    {
        ImuTelemetry telemetry{};
#if defined(ARDUINO_TEENSY41)
        const typename TImu::StatusReg status = imu.ReadStatus();
        const typename TImu::Axes gyro = imu.ReadGyro();
        const typename TImu::Axes accel = imu.ReadAccel();

        telemetry.status = status.Raw();
        telemetry.gyroX = gyro.x;
        telemetry.gyroY = gyro.y;
        telemetry.gyroZ = gyro.z;
        telemetry.accelX = accel.x;
        telemetry.accelY = accel.y;
        telemetry.accelZ = accel.z;
        telemetry.temp = imu.ReadTemp();
        telemetry.interruptHigh = (digitalRead(interruptPin) == HIGH);
#else
        (void)imu;
        (void)interruptPin;
#endif
        return telemetry;
    }

    float ReadAverageGyroZRadpsRaw()
    {
        return ReadBackLeftGyroZRadpsRaw(_vehicle);
    }
};

static bool SelectSequentialCsvFileName(
    char* buffer,
    size_t bufferSize,
    const char* explicitFileName,
    const char* teensyFormat,
    const char* hostFallback)
{
    if (buffer == nullptr || bufferSize == 0U)
    {
        return false;
    }

    if (explicitFileName != nullptr && explicitFileName[0] != '\0')
    {
        snprintf(buffer, bufferSize, "%s", explicitFileName);
        return true;
    }

#if defined(ARDUINO_TEENSY41)
    if (teensyFormat == nullptr || teensyFormat[0] == '\0')
    {
        return false;
    }

    for (uint16_t index = 0U; index < 1000U; ++index)
    {
        snprintf(buffer, bufferSize, teensyFormat, static_cast<unsigned>(index));
        if (!SD.exists(buffer))
        {
            return true;
        }
    }

    snprintf(buffer, bufferSize, teensyFormat, 999U);
    return true;
#else
    snprintf(buffer, bufferSize, "%s", (hostFallback != nullptr) ? hostFallback : "measurement_log.csv");
    return true;
#endif
}

class DiagnosticLogger
{
public:
    DiagnosticLogger()
        : _file()
        , _phaseId(0UL)
        , _sampleCount(0UL)
        , _lastFlushMs(0UL)
    {
        _fileName[0] = '\0';
    }

    bool Begin(
        const DiagnosticSensorSuite& sensors,
        const char* fileName = nullptr,
        unsigned long controlPeriodUs = DiagnosticConfig::kControlPeriodUs,
        const char* modeName = nullptr)
    {
        if (!SelectFileName(_fileName, sizeof(_fileName), fileName))
        {
            return false;
        }
        if (!_file.Open(_fileName))
        {
            return false;
        }

        _lastFlushMs = millis();
        if (!WriteMetadata("file", _fileName))
        {
            return false;
        }
        if (modeName != nullptr && modeName[0] != '\0' && !WriteMetadata("mode", modeName))
        {
            return false;
        }
        if (!WriteMetadataUL("control_period_us", controlPeriodUs))
        {
            return false;
        }
        const unsigned long imuSampleRateHz = MazeMap::GetUiImuSampleRateHzForControlPeriodUs(controlPeriodUs);
        if (imuSampleRateHz > 0UL && !WriteMetadataUL("imu_sample_rate_hz", imuSampleRateHz))
        {
            return false;
        }
        const float imuAccelLpf2CutoffHz = MazeMap::GetUiAccelLpf2CutoffHzForControlPeriodUs(controlPeriodUs);
        if (imuAccelLpf2CutoffHz > 0.0f && !WriteMetadataFloat("imu_accel_lpf2_cutoff_hz", imuAccelLpf2CutoffHz, 3))
        {
            return false;
        }
        const float imuGyroLpf1ReferenceHz = MazeMap::GetUiGyroCut213DatasheetReferenceHzForControlPeriodUs(controlPeriodUs);
        if (imuGyroLpf1ReferenceHz > 0.0f && !WriteMetadataFloat("imu_gyro_lpf1_cut213_datasheet_ref_hz", imuGyroLpf1ReferenceHz, 3))
        {
            return false;
        }
        if (!WriteMetadataFloat("boundary_half_span_m", DiagnosticConfig::kBoundaryHalfSpanM, 3))
        {
            return false;
        }
        if (!WriteMetadataFloat("imu_gyro_mdps_per_lsb", sensors.GetGyroSensitivityMdpsPerLsb(), 3))
        {
            return false;
        }
        if (!WriteMetadataFloat("imu_accel_mg_per_lsb", sensors.GetAccelSensitivityMgPerLsb(), 3))
        {
            return false;
        }
        if (!WriteMetadataFloat("mission_gyro_bias_estimate_radps", sensors.GetGyroBiasRadps(), 6))
        {
            return false;
        }
        if (!WriteDiagnosticTuningMetadata())
        {
            return false;
        }
        if (!WriteSummaryInstructions())
        {
            return false;
        }

        return _file.Write(
            "sample,phase_id,t_us,dt_us,stationary,"
            "pose_x_m,pose_y_m,yaw_rad,linear_speed_mps,angular_speed_radps,"
            "cmd_linear_mps,cmd_angular_radps,left_drive_cmd,right_drive_cmd,"
            "left_encoder_count,right_encoder_count,left_distance_m,right_distance_m,left_velocity_mps,right_velocity_mps,"
            "imu_fr_status,imu_fr_gyro_x,imu_fr_gyro_y,imu_fr_gyro_z,imu_fr_accel_x,imu_fr_accel_y,imu_fr_accel_z,imu_fr_temp,imu_fr_int,"
            "imu_bl_status,imu_bl_gyro_x,imu_bl_gyro_y,imu_bl_gyro_z,imu_bl_accel_x,imu_bl_accel_y,imu_bl_accel_z,imu_bl_temp,imu_bl_int,"
            "ws_fl_ambient,ws_fl_lit,ws_fl_delta,ws_fl_raw_distance_m,ws_fl_distance_m,ws_fr_ambient,ws_fr_lit,ws_fr_delta,ws_fr_raw_distance_m,ws_fr_distance_m,"
            "ws_sl_ambient,ws_sl_lit,ws_sl_delta,ws_sl_raw_distance_m,ws_sl_distance_m,ws_sr_ambient,ws_sr_lit,ws_sr_delta,ws_sr_raw_distance_m,ws_sr_distance_m,"
            "front_wall,left_wall,right_wall,corridor_error_m,front_skew_m,gyro_bias_radps,gyro_raw_radps,gyro_radps\n");
    }

    bool BeginPhase(const char* name)
    {
        ++_phaseId;

        char line[160] = {};
        const int length = snprintf(
            line,
            sizeof(line),
            "# phase,%lu,%lu,%s\n",
            _phaseId,
            micros(),
            (name != nullptr) ? name : "");

        if (length <= 0 || length >= static_cast<int>(sizeof(line)))
        {
            return false;
        }

        const bool ok = _file.Write(line);
        FlushIfNeeded(true);
        return ok;
    }

    bool WriteEvent(const char* type, const char* message)
    {
        char line[MazeMap::kDiagnosticEventLineCapacity] = {};
        const int length = snprintf(
            line,
            sizeof(line),
            "# event,%lu,%s,%s\n",
            micros(),
            (type != nullptr) ? type : "",
            (message != nullptr) ? message : "");

        if (length <= 0 || length >= static_cast<int>(sizeof(line)))
        {
            return false;
        }

        const bool ok = _file.Write(line);
        FlushIfNeeded(true);
        return ok;
    }

    bool LogSample(
        bool stationary,
        uint32_t timestampUs,
        uint32_t dtUs,
        const PoseEstimate& pose,
        const DriveBase& drive,
        const DriveTelemetry& driveTelemetry,
        const DiagnosticSensorSnapshot& sensorSnapshot)
    {
        char line[2048] = {};
        const int length = snprintf(
            line,
            sizeof(line),
            "%lu,%lu,%lu,%lu,%u,"
            "%.6f,%.6f,%.6f,%.6f,%.6f,"
            "%.6f,%.6f,%.4f,%.4f,"
            "%ld,%ld,%.6f,%.6f,%.6f,%.6f,"
            "%u,%d,%d,%d,%d,%d,%d,%d,%u,"
            "%u,%d,%d,%d,%d,%d,%d,%d,%u,"
            "%.3f,%.3f,%.3f,%.6f,%.6f,%.3f,%.3f,%.3f,%.6f,%.6f,"
            "%.3f,%.3f,%.3f,%.6f,%.6f,%.3f,%.3f,%.3f,%.6f,%.6f,"
            "%u,%u,%u,%.6f,%.6f,%.6f,%.6f,%.6f\n",
            _sampleCount,
            _phaseId,
            timestampUs,
            dtUs,
            stationary ? 1U : 0U,
            pose.xMeters,
            pose.yMeters,
            pose.yawRad,
            pose.linearSpeedMps,
            pose.angularSpeedRadps,
            drive.GetLastLinearCommandMps(),
            drive.GetLastAngularCommandRadps(),
            driveTelemetry.leftDriveCommand,
            driveTelemetry.rightDriveCommand,
            static_cast<long>(driveTelemetry.leftEncoderCount),
            static_cast<long>(driveTelemetry.rightEncoderCount),
            driveTelemetry.leftDistanceM,
            driveTelemetry.rightDistanceM,
            driveTelemetry.leftVelocityMps,
            driveTelemetry.rightVelocityMps,
            sensorSnapshot.imuFrontRight.status,
            sensorSnapshot.imuFrontRight.gyroX,
            sensorSnapshot.imuFrontRight.gyroY,
            sensorSnapshot.imuFrontRight.gyroZ,
            sensorSnapshot.imuFrontRight.accelX,
            sensorSnapshot.imuFrontRight.accelY,
            sensorSnapshot.imuFrontRight.accelZ,
            sensorSnapshot.imuFrontRight.temp,
            sensorSnapshot.imuFrontRight.interruptHigh ? 1U : 0U,
            sensorSnapshot.imuBackLeft.status,
            sensorSnapshot.imuBackLeft.gyroX,
            sensorSnapshot.imuBackLeft.gyroY,
            sensorSnapshot.imuBackLeft.gyroZ,
            sensorSnapshot.imuBackLeft.accelX,
            sensorSnapshot.imuBackLeft.accelY,
            sensorSnapshot.imuBackLeft.accelZ,
            sensorSnapshot.imuBackLeft.temp,
            sensorSnapshot.imuBackLeft.interruptHigh ? 1U : 0U,
            sensorSnapshot.frontLeft.ambientLight,
            sensorSnapshot.frontLeft.litLight,
            sensorSnapshot.frontLeft.differentialLight,
            sensorSnapshot.frontLeft.rawDistanceM,
            sensorSnapshot.frontLeft.distanceM,
            sensorSnapshot.frontRight.ambientLight,
            sensorSnapshot.frontRight.litLight,
            sensorSnapshot.frontRight.differentialLight,
            sensorSnapshot.frontRight.rawDistanceM,
            sensorSnapshot.frontRight.distanceM,
            sensorSnapshot.sideLeft.ambientLight,
            sensorSnapshot.sideLeft.litLight,
            sensorSnapshot.sideLeft.differentialLight,
            sensorSnapshot.sideLeft.rawDistanceM,
            sensorSnapshot.sideLeft.distanceM,
            sensorSnapshot.sideRight.ambientLight,
            sensorSnapshot.sideRight.litLight,
            sensorSnapshot.sideRight.differentialLight,
            sensorSnapshot.sideRight.rawDistanceM,
            sensorSnapshot.sideRight.distanceM,
            sensorSnapshot.frontWall ? 1U : 0U,
            sensorSnapshot.leftWall ? 1U : 0U,
            sensorSnapshot.rightWall ? 1U : 0U,
            sensorSnapshot.corridorErrorM,
            sensorSnapshot.frontSkewM,
            sensorSnapshot.gyroBiasRadps,
            sensorSnapshot.gyroRawRadps,
            sensorSnapshot.gyroRadps);

        if (length <= 0 || length >= static_cast<int>(sizeof(line)))
        {
            return false;
        }

        if (!_file.Write(line))
        {
            return false;
        }

        ++_sampleCount;
        FlushIfNeeded(false);
        return true;
    }

    void Flush()
    {
        _file.Flush();
        _lastFlushMs = millis();
    }

    void Close()
    {
        _file.Close();
    }

    const char* GetFileName() const
    {
        return _fileName;
    }

    bool WriteMetadataUnsigned(const char* key, unsigned long value)
    {
        return WriteMetadataUL(key, value);
    }

    bool WriteMetadataValueFloat(const char* key, float value, uint8_t precision)
    {
        return WriteMetadataFloat(key, value, precision);
    }

private:
    MazeMap::CoreFileExport _file;
    char _fileName[24];
    unsigned long _phaseId;
    unsigned long _sampleCount;
    unsigned long _lastFlushMs;

    static bool SelectFileName(char* buffer, size_t bufferSize, const char* explicitFileName)
    {
        return SelectSequentialCsvFileName(buffer, bufferSize, explicitFileName, "diag%03u.csv", "diagnostic_log.csv");
    }

    bool WriteMetadata(const char* key, const char* value)
    {
        char line[128] = {};
        const int length = snprintf(
            line,
            sizeof(line),
            "# meta,%s,%s\n",
            (key != nullptr) ? key : "",
            (value != nullptr) ? value : "");

        if (length <= 0 || length >= static_cast<int>(sizeof(line)))
        {
            return false;
        }
        return _file.Write(line);
    }

    bool WriteMetadataUL(const char* key, unsigned long value)
    {
        char line[128] = {};
        const int length = snprintf(
            line,
            sizeof(line),
            "# meta,%s,%lu\n",
            (key != nullptr) ? key : "",
            value);

        if (length <= 0 || length >= static_cast<int>(sizeof(line)))
        {
            return false;
        }
        return _file.Write(line);
    }

    bool WriteMetadataFloat(const char* key, float value, uint8_t precision)
    {
        char format[32] = {};
        char line[128] = {};
        const int formatLength = snprintf(format, sizeof(format), "# meta,%%s,%%.%uf\n", static_cast<unsigned>(precision));
        if (formatLength <= 0 || formatLength >= static_cast<int>(sizeof(format)))
        {
            return false;
        }

        const int length = snprintf(line, sizeof(line), format, (key != nullptr) ? key : "", value);
        if (length <= 0 || length >= static_cast<int>(sizeof(line)))
        {
            return false;
        }
        return _file.Write(line);
    }

    bool WriteDiagnosticTuningMetadata()
    {
        const auto& driveModel = MazeMap::MotorEncoderDrive::GetSharedPhysicalModel();
        const auto& vehicleModel = MazeMap::Vehicle::GetPhysicalModel();
        auto writeUL = [this](const char* key, unsigned long value) -> bool
        {
            return WriteMetadataUL(key, value);
        };
        auto writeFloat = [this](const char* key, float value) -> bool
        {
            return WriteMetadataFloat(key, value, 6);
        };

        if (!writeUL("kGyroBiasSamples", static_cast<unsigned long>(Config::kGyroBiasSamples))) return false;
        if (!writeUL("kGyroBiasMinimumAveragingWindowMs", static_cast<unsigned long>(Config::kGyroBiasMinimumAveragingWindowMs))) return false;
        if (!writeFloat("kGyroBiasUpdateMaxAbsRateRadps", Config::kGyroBiasUpdateMaxAbsRateRadps)) return false;
        if (!writeFloat("kTrackWidthM", Config::kTrackWidthM)) return false;
        if (!writeFloat("kArcTrackWidthTightRadiusM", vehicleModel.arcTrackWidthInterpolation.tightRadiusM)) return false;
        if (!writeFloat("kArcTrackWidthTightM", vehicleModel.arcTrackWidthInterpolation.tightTrackWidthM)) return false;
        if (!writeFloat("kArcTrackWidthWideRadiusM", vehicleModel.arcTrackWidthInterpolation.wideRadiusM)) return false;
        if (!writeFloat("kArcTrackWidthWideM", vehicleModel.arcTrackWidthInterpolation.wideTrackWidthM)) return false;
        if (!writeFloat("kWheelDiameterM", driveModel.wheelDiameterM)) return false;
        if (!writeUL("kEncoderCountsPerRev", static_cast<unsigned long>(driveModel.pulsesPerRev))) return false;
        if (!writeFloat("kMotorToWheelGearRatio", driveModel.gearRatio)) return false;
        if (!writeFloat("kMotorNominalVoltageV", driveModel.nominalVoltageV)) return false;
        if (!writeFloat("kMotorNominalNoLoadSpeedRpm", driveModel.nominalNoLoadSpeedRpm)) return false;
        if (!writeFloat("kMotorSupplyVoltageV", driveModel.supplyVoltageV)) return false;
        if (!writeFloat("kMotorTerminalResistanceOhms", driveModel.resistanceOhms)) return false;
        if (!writeFloat("kMotorTorqueConstantNmPerA", driveModel.torqueConstantNmPerA)) return false;
        if (!writeFloat("kMotorSpeedConstantRadpsPerVolt", driveModel.speedConstantRadpsPerVolt)) return false;
        if (!writeFloat("kMotorNoLoadCurrentA", driveModel.noLoadCurrentA)) return false;
        if (!writeFloat("kRacingFanDutyCycle", Config::kRacingFanDutyCycle)) return false;
        if (!writeUL("kRacingFanRampMs", static_cast<unsigned long>(Config::kRacingFanRampMs))) return false;
        if (!writeFloat("kWheelStaticFeedforward", Config::kWheelStaticFeedforward)) return false;
        if (!writeFloat("kWheelRestLaunchDriveCommand", Config::kWheelRestLaunchDriveCommand)) return false;
        if (!writeFloat("kWheelRestLaunchMaxDriveCommand", Config::kWheelRestLaunchMaxDriveCommand)) return false;
        if (!writeUL("kWheelRestLaunchRampMs", Config::kWheelRestLaunchRampMs)) return false;
        if (!writeFloat("kWheelRestLaunchSpeedThresholdMps", Config::kWheelRestLaunchSpeedThresholdMps)) return false;
        if (!writeFloat("kWheelRestLaunchDriveThreshold", Config::kWheelRestLaunchDriveThreshold)) return false;
        if (!writeFloat("kWheelVelocityFeedforward", Config::kWheelVelocityFeedforward)) return false;
        if (!writeFloat("kWheelVelocityKp", Config::kWheelVelocityKp)) return false;
        if (!writeFloat("kWheelVelocityKi", Config::kWheelVelocityKi)) return false;
        if (!writeFloat("kWheelIntegralLimit", Config::kWheelIntegralLimit)) return false;
        if (!writeFloat("kDiagnosticWheelVelocityKpScale", DiagnosticConfig::kDiagnosticWheelVelocityKpScale)) return false;
        if (!writeFloat("kDiagnosticWheelVelocityKiScale", DiagnosticConfig::kDiagnosticWheelVelocityKiScale)) return false;
        if (!writeFloat("kDiagnosticWheelIntegralLimitScale", DiagnosticConfig::kDiagnosticWheelIntegralLimitScale)) return false;
        if (!writeFloat("kDiagnosticWheelVelocityKpEffective", MazeMap::ScaleWheelControlValue(Config::kWheelVelocityKp, DiagnosticConfig::kDiagnosticWheelVelocityKpScale))) return false;
        if (!writeFloat("kDiagnosticWheelVelocityKiEffective", MazeMap::ScaleWheelControlValue(Config::kWheelVelocityKi, DiagnosticConfig::kDiagnosticWheelVelocityKiScale))) return false;
        if (!writeFloat("kDiagnosticWheelIntegralLimitEffective", MazeMap::ScaleWheelControlValue(Config::kWheelIntegralLimit, DiagnosticConfig::kDiagnosticWheelIntegralLimitScale))) return false;
        if (!writeFloat("kStraightHeadingKp", Config::kStraightHeadingKp)) return false;
        if (!writeFloat("kStraightYawD", Config::kStraightYawD)) return false;
        if (!writeFloat("kArcHeadingKp", Config::kArcHeadingKp)) return false;
        if (!writeFloat("kArcYawD", Config::kArcYawD)) return false;
        if (!writeFloat("kTurnHeadingKp", Config::kTurnHeadingKp)) return false;
        if (!writeFloat("kTurnYawD", Config::kTurnYawD)) return false;
        if (!writeFloat("kDistanceToleranceM", Config::kDistanceToleranceM)) return false;
        if (!writeFloat("kAngleToleranceRad", Config::kAngleToleranceRad)) return false;
        if (!writeFloat("kSpeedToleranceMps", Config::kSpeedToleranceMps)) return false;
        if (!writeFloat("kAngularSpeedToleranceRadps", Config::kAngularSpeedToleranceRadps)) return false;
        if (!writeFloat("kObservedDiagnosticMinimumSustainableSpeedMps", Config::kObservedDiagnosticMinimumSustainableSpeedMps)) return false;
        if (!writeFloat("kMinimumAllowedCruiseSpeedMps", Config::kMinimumAllowedCruiseSpeedMps)) return false;
        if (!writeFloat("kEncoderProgressEpsilonM", Config::kEncoderProgressEpsilonM)) return false;
        if (!writeFloat("kEncoderStallCommandThresholdMps", Config::kEncoderStallCommandThresholdMps)) return false;
        if (!writeUL("kEncoderStallTimeoutMs", Config::kEncoderStallTimeoutMs)) return false;
        if (!writeUL("kEncoderStallStartupGraceMs", Config::kEncoderStallStartupGraceMs)) return false;

        if (!writeUL("kModeSelectPinA", static_cast<unsigned long>(DiagnosticConfig::kModeSelectPinA))) return false;
        if (!writeUL("kModeSelectPinB", static_cast<unsigned long>(DiagnosticConfig::kModeSelectPinB))) return false;
        if (!writeUL("kControlPeriodUs", DiagnosticConfig::kControlPeriodUs)) return false;
        if (!writeUL("kStartupSettleMs", static_cast<unsigned long>(DiagnosticConfig::kStartupSettleMs))) return false;
        if (!writeUL("kBaselineHoldMs", static_cast<unsigned long>(DiagnosticConfig::kBaselineHoldMs))) return false;
        if (!writeUL("kInterTestHoldMs", static_cast<unsigned long>(DiagnosticConfig::kInterTestHoldMs))) return false;
        if (!writeUL("kLogFlushPeriodMs", static_cast<unsigned long>(DiagnosticConfig::kLogFlushPeriodMs))) return false;
        if (!writeFloat("kBoundaryHalfSpanM", DiagnosticConfig::kBoundaryHalfSpanM)) return false;
        if (!writeFloat("kShortStraightDistanceM", DiagnosticConfig::kShortStraightDistanceM)) return false;
        if (!writeFloat("kLongStraightDistanceM", DiagnosticConfig::kLongStraightDistanceM)) return false;
        if (!writeFloat("kSquareLegDistanceM", DiagnosticConfig::kSquareLegDistanceM)) return false;
        if (!writeFloat("kArcHalfCircleDistanceM", DiagnosticConfig::kArcHalfCircleDistanceM)) return false;
        if (!writeFloat("kSlowStraightSpeedMps", DiagnosticConfig::kSlowStraightSpeedMps)) return false;
        if (!writeFloat("kCircleMediumSpeedMps", DiagnosticConfig::kCircleMediumSpeedMps)) return false;
        if (!writeFloat("kFastStraightSpeedMps", DiagnosticConfig::kFastStraightSpeedMps)) return false;
        if (!writeFloat("kStraightAccelMps2", DiagnosticConfig::kStraightAccelMps2)) return false;
        if (!writeFloat("kStraightDecelMps2", DiagnosticConfig::kStraightDecelMps2)) return false;
        static const MazeMap::Vehicle sharedVehicle{};
        const MazeMap::InPlaceTurnProfile inPlaceTurnProfile = BuildSharedInPlaceTurnProfile(sharedVehicle);
        if (!writeFloat("kInPlaceTurnMaxOmegaRadps", inPlaceTurnProfile.maxAngularSpeedRadps)) return false;
        if (!writeFloat("kInPlaceTurnAccelRadps2", inPlaceTurnProfile.angularAccelRadps2)) return false;
        if (!writeFloat("kKickoffSweepMinDriveCommand", DiagnosticConfig::kKickoffSweepMinDriveCommand)) return false;
        if (!writeFloat("kKickoffSweepMaxDriveCommand", DiagnosticConfig::kKickoffSweepMaxDriveCommand)) return false;
        if (!writeFloat("kKickoffSweepStepDriveCommand", DiagnosticConfig::kKickoffSweepStepDriveCommand)) return false;
        if (!writeUL("kKickoffSweepPulseMs", static_cast<unsigned long>(DiagnosticConfig::kKickoffSweepPulseMs))) return false;
        if (!writeFloat("kKickoffSweepMoveThresholdM", DiagnosticConfig::kKickoffSweepMoveThresholdM)) return false;
        if (!writeFloat("kKickoffSweepMoveThresholdMps", DiagnosticConfig::kKickoffSweepMoveThresholdMps)) return false;
        if (!writeFloat("kForwardSweepKickoffDriveCommand", DiagnosticConfig::kForwardSweepKickoffDriveCommand)) return false;
        if (!writeUL("kForwardSweepKickoffMs", static_cast<unsigned long>(DiagnosticConfig::kForwardSweepKickoffMs))) return false;
        if (!writeFloat("kForwardSweepMinDriveCommand", DiagnosticConfig::kForwardSweepMinDriveCommand)) return false;
        if (!writeFloat("kForwardSweepMaxDriveCommand", DiagnosticConfig::kForwardSweepMaxDriveCommand)) return false;
        if (!writeFloat("kForwardSweepStepDriveCommand", DiagnosticConfig::kForwardSweepStepDriveCommand)) return false;
        if (!writeUL("kForwardSweepHoldMs", static_cast<unsigned long>(DiagnosticConfig::kForwardSweepHoldMs))) return false;
        if (!writeFloat("kForwardSweepCarryThresholdMps", DiagnosticConfig::kForwardSweepCarryThresholdMps)) return false;
        if (!writeFloat("kForwardSweepCarryThresholdM", DiagnosticConfig::kForwardSweepCarryThresholdM)) return false;
        if (!writeFloat("kCharacterizationBoundaryReserveM", DiagnosticConfig::kCharacterizationBoundaryReserveM)) return false;
        if (!writeUL("kCharacterizationSettleMs", static_cast<unsigned long>(DiagnosticConfig::kCharacterizationSettleMs))) return false;
        if (!writeFloat("kCharacterizationRecoverySpeedMps", DiagnosticConfig::kCharacterizationRecoverySpeedMps)) return false;

        return true;
    }

    bool WriteSummaryInstructions()
    {
        for (size_t index = 0U; index < MazeMap::GetDiagnosticSummaryInstructionCount(); ++index)
        {
            if (!WriteEvent("summary", MazeMap::GetDiagnosticSummaryInstruction(index).message))
            {
                return false;
            }
        }
        return true;
    }

    void FlushIfNeeded(bool force)
    {
        if (force || ((millis() - _lastFlushMs) >= DiagnosticConfig::kLogFlushPeriodMs))
        {
            Flush();
        }
    }
};

static bool IsPinPairStrapped(uint8_t pinA, uint8_t pinB)
{
#if defined(ARDUINO_TEENSY41)
    pinMode(pinA, OUTPUT);
    digitalWrite(pinA, LOW);
    pinMode(pinB, INPUT_PULLUP);
    delay(2);
    const bool forwardSense = (digitalRead(pinB) == LOW);

    pinMode(pinA, INPUT_PULLUP);
    pinMode(pinB, OUTPUT);
    digitalWrite(pinB, LOW);
    delay(2);
    const bool reverseSense = (digitalRead(pinA) == LOW);

    pinMode(pinA, INPUT_PULLUP);
    pinMode(pinB, INPUT_PULLUP);
    return forwardSense && reverseSense;
#else
    (void)pinA;
    (void)pinB;
    return false;
#endif
}

static bool IsPrimaryDiagnosticModeRequested()
{
    return IsPinPairStrapped(DiagnosticConfig::kModeSelectPinA, DiagnosticConfig::kModeSelectPinB);
}

static bool IsManeuverTestModeRequested()
{
    return IsPinPairStrapped(29U, DiagnosticConfig::kModeSelectPinA);
}

static bool IsAuxiliaryMeasurementModeRequested()
{
    return IsPinPairStrapped(AuxMeasurementConfig::kModeSelectPinA, AuxMeasurementConfig::kModeSelectPinB);
}

static bool IsInterRunServiceJumperInstalled()
{
    return IsPinPairStrapped(Config::kInterRunServicePinA, Config::kInterRunServicePinB);
}

static bool IsWallSensorLedCalibrationModeRequested()
{
    return IsPinPairStrapped(LedCalibrationConfig::kModeSelectPinA, LedCalibrationConfig::kModeSelectPinB);
}

static bool ResetStartupTrace(const char* firstLine)
{
#if defined(ARDUINO_TEENSY41)
    if (firstLine == nullptr || firstLine[0] == '\0')
    {
        return false;
    }

    MazeMap::CoreFileExport file;
    if (!file.Open("startup_trace.txt"))
    {
        return false;
    }

    if (!file.Write(firstLine) || !file.WriteChar('\n'))
    {
        return false;
    }

    file.Flush();
    return true;
#else
    (void)firstLine;
    return false;
#endif
}

static bool AppendStartupTrace(const char* line)
{
#if defined(ARDUINO_TEENSY41)
    if (line == nullptr || line[0] == '\0')
    {
        return false;
    }

    File file = SD.open("startup_trace.txt", FILE_WRITE);
    if (!file)
    {
        return false;
    }

    const bool ok = (file.print(line) > 0U) && (file.write('\n') == 1U);
    file.flush();
    file.close();
    return ok;
#else
    (void)line;
    return false;
#endif
}

static const char* AuxMeasurementRoutineName(AuxMeasurementConfig::Routine routine)
{
    switch (routine)
    {
    case AuxMeasurementConfig::Routine::FanStaticSurvey:
        return "fan_static_survey";
    case AuxMeasurementConfig::Routine::TurningTractionSweep:
        return "turning_traction_sweep";
    case AuxMeasurementConfig::Routine::CorridorRepeatabilitySweep:
        return "corridor_repeatability_sweep";
    case AuxMeasurementConfig::Routine::PositionAccuracyAudit:
        return "position_accuracy_audit";
    default:
        return "unknown";
    }
}

class AuxMeasurementLogger
{
public:
    AuxMeasurementLogger()
        : _file()
        , _phaseId(0UL)
        , _sampleCount(0UL)
        , _lastFlushMs(0UL)
    {
        _fileName[0] = '\0';
    }

    bool Begin(
        const DiagnosticSensorSuite& sensors,
        AuxMeasurementConfig::Routine routine,
        const char* fileName = nullptr)
    {
        const auto& vehicleModel = MazeMap::Vehicle::GetPhysicalModel();
        if (!SelectSequentialCsvFileName(_fileName, sizeof(_fileName), fileName, "aux%03u.csv", "aux_measurement_log.csv"))
        {
            return false;
        }
        if (!_file.Open(_fileName))
        {
            return false;
        }

        _lastFlushMs = millis();
        if (!WriteMetadata("file", _fileName)) return false;
        if (!WriteMetadata("mode", "aux_measurement")) return false;
        if (!WriteMetadata("routine", AuxMeasurementRoutineName(routine))) return false;
        if (!WriteMetadataUL("control_period_us", AuxMeasurementConfig::kControlPeriodUs)) return false;
        {
            const unsigned long imuSampleRateHz = MazeMap::GetUiImuSampleRateHzForControlPeriodUs(AuxMeasurementConfig::kControlPeriodUs);
            if (imuSampleRateHz > 0UL && !WriteMetadataUL("imu_sample_rate_hz", imuSampleRateHz)) return false;
        }
        {
            const float imuAccelLpf2CutoffHz = MazeMap::GetUiAccelLpf2CutoffHzForControlPeriodUs(AuxMeasurementConfig::kControlPeriodUs);
            if (imuAccelLpf2CutoffHz > 0.0f && !WriteMetadataFloat("imu_accel_lpf2_cutoff_hz", imuAccelLpf2CutoffHz, 3)) return false;
        }
        {
            const float imuGyroLpf1ReferenceHz = MazeMap::GetUiGyroCut213DatasheetReferenceHzForControlPeriodUs(AuxMeasurementConfig::kControlPeriodUs);
            if (imuGyroLpf1ReferenceHz > 0.0f && !WriteMetadataFloat("imu_gyro_lpf1_cut213_datasheet_ref_hz", imuGyroLpf1ReferenceHz, 3)) return false;
        }
        if (!WriteMetadataUL("startup_settle_ms", static_cast<unsigned long>(AuxMeasurementConfig::kStartupSettleMs))) return false;
        if (!WriteMetadataUL("log_flush_period_ms", static_cast<unsigned long>(AuxMeasurementConfig::kLogFlushPeriodMs))) return false;
        if (!WriteMetadataUL("mode_select_pin_a", static_cast<unsigned long>(AuxMeasurementConfig::kModeSelectPinA))) return false;
        if (!WriteMetadataUL("mode_select_pin_b", static_cast<unsigned long>(AuxMeasurementConfig::kModeSelectPinB))) return false;
        if (!WriteMetadataFloat("imu_gyro_mdps_per_lsb", sensors.GetGyroSensitivityMdpsPerLsb(), 3)) return false;
        if (!WriteMetadataFloat("imu_accel_mg_per_lsb", sensors.GetAccelSensitivityMgPerLsb(), 3)) return false;
        if (!WriteMetadataFloat("mission_gyro_bias_estimate_radps", sensors.GetGyroBiasRadps(), 6)) return false;
        if (!WriteMetadataFloat("kTrackWidthM", Config::kTrackWidthM, 6)) return false;
        if (!WriteMetadataFloat("kArcTrackWidthTightRadiusM", vehicleModel.arcTrackWidthInterpolation.tightRadiusM, 6)) return false;
        if (!WriteMetadataFloat("kArcTrackWidthTightM", vehicleModel.arcTrackWidthInterpolation.tightTrackWidthM, 6)) return false;
        if (!WriteMetadataFloat("kArcTrackWidthWideRadiusM", vehicleModel.arcTrackWidthInterpolation.wideRadiusM, 6)) return false;
        if (!WriteMetadataFloat("kArcTrackWidthWideM", vehicleModel.arcTrackWidthInterpolation.wideTrackWidthM, 6)) return false;
        if (routine == AuxMeasurementConfig::Routine::FanStaticSurvey)
        {
            if (!WriteMetadataUL("baseline_hold_ms", static_cast<unsigned long>(AuxMeasurementConfig::kBaselineHoldMs))) return false;
            if (!WriteMetadataUL("fan_hold_ms", static_cast<unsigned long>(AuxMeasurementConfig::kFanHoldMs))) return false;
            if (!WriteMetadataUL("recovery_hold_ms", static_cast<unsigned long>(AuxMeasurementConfig::kRecoveryHoldMs))) return false;
            if (!WriteMetadataFloat("kRacingFanDutyCycle", Config::kRacingFanDutyCycle, 6)) return false;
            if (!WriteMetadataUL("kRacingFanRampMs", static_cast<unsigned long>(Config::kRacingFanRampMs))) return false;
        }
        if (routine == AuxMeasurementConfig::Routine::TurningTractionSweep)
        {
            if (!WriteMetadata("turn_direction", AuxMeasurementConfig::kTurningTractionSweepClockwise ? "cw" : "ccw")) return false;
            if (!WriteMetadataFloat("turning_traction_radius_m", AuxMeasurementConfig::kTurningTractionSweepRadiusM, 6)) return false;
            if (!WriteMetadataFloat("turning_traction_start_speed_mps", AuxMeasurementConfig::kTurningTractionSweepStartSpeedMps, 6)) return false;
            if (!WriteMetadataFloat("turning_traction_accel_mps2", AuxMeasurementConfig::kTurningTractionSweepAccelMps2, 6)) return false;
            if (!WriteMetadataFloat("turning_traction_max_speed_mps", AuxMeasurementConfig::kTurningTractionSweepMaxSpeedMps, 6)) return false;
            if (!WriteMetadataUL("turning_traction_fan_settle_ms", static_cast<unsigned long>(AuxMeasurementConfig::kTurningTractionSweepFanSettleMs))) return false;
            if (!WriteMetadataUL("turning_traction_launch_ms", static_cast<unsigned long>(AuxMeasurementConfig::kTurningTractionLaunchMs))) return false;
            if (!WriteMetadataFloat("turning_traction_max_omega_radps", AuxMeasurementConfig::kTurningTractionSweepMaxAngularCommandRadps, 6)) return false;
            if (!WriteMetadataFloat("turning_traction_wheel_kp_scale", AuxMeasurementConfig::kTurningTractionWheelVelocityKpScale, 6)) return false;
            if (!WriteMetadataFloat("turning_traction_wheel_ki_scale", AuxMeasurementConfig::kTurningTractionWheelVelocityKiScale, 6)) return false;
            if (!WriteMetadataFloat("turning_traction_wheel_integral_limit_scale", AuxMeasurementConfig::kTurningTractionWheelIntegralLimitScale, 6)) return false;
            if (!WriteMetadataFloat("turning_traction_plateau_min_speed_mps", AuxMeasurementConfig::kTurningTractionPlateauMinSpeedMps, 6)) return false;
            if (!WriteMetadataFloat("turning_traction_plateau_delta_mps", AuxMeasurementConfig::kTurningTractionPlateauDeltaMps, 6)) return false;
            if (!WriteMetadataUL("turning_traction_plateau_window_ms", static_cast<unsigned long>(AuxMeasurementConfig::kTurningTractionPlateauWindowMs))) return false;
            if (!WriteMetadataFloat("turning_traction_actuator_ceiling_cmd", AuxMeasurementConfig::kTurningTractionActuatorCeilingCommand, 6)) return false;
            if (!WriteMetadataFloat("turning_traction_curvature_ramp_m_inv_per_s", AuxMeasurementConfig::kTurningTractionCurvatureRampMInvPerSec, 6)) return false;
            if (!WriteMetadataFloat("turning_traction_slip_min_speed_mps", AuxMeasurementConfig::kTurningTractionSlipMinSpeedMps, 6)) return false;
            if (!WriteMetadataFloat("turning_traction_slip_min_lat_accel_mps2", AuxMeasurementConfig::kTurningTractionSlipMinLatAccelMps2, 6)) return false;
            if (!WriteMetadataFloat("turning_traction_slip_yaw_coherence_floor", AuxMeasurementConfig::kTurningTractionSlipYawCoherenceFloor, 6)) return false;
            if (!WriteMetadataFloat("turning_traction_slip_planar_coherence_floor", AuxMeasurementConfig::kTurningTractionSlipPlanarCoherenceFloor, 6)) return false;
            if (!WriteMetadataUL("turning_traction_slip_confirm_ms", static_cast<unsigned long>(AuxMeasurementConfig::kTurningTractionSlipConfirmMs))) return false;
            if (!WriteMetadataUL("turning_traction_timeout_ms", static_cast<unsigned long>(AuxMeasurementConfig::kTurningTractionSweepTimeoutMs))) return false;
            if (!WriteMetadataFloat("kRacingFanDutyCycle", Config::kRacingFanDutyCycle, 6)) return false;
            if (!WriteMetadataUL("kRacingFanRampMs", static_cast<unsigned long>(Config::kRacingFanRampMs))) return false;
        }
        if (!WriteEvent("summary", "Enter by shorting pins 28 and 29 at boot. Those pins only select this mode; they are not measurement inputs.")) return false;
        if (!WriteEvent("summary", "Edit AuxMeasurementConfig::kRoutine and RunSelectedRoutine() to repurpose this mode for one-off internal measurements.")) return false;
        if (routine == AuxMeasurementConfig::Routine::TurningTractionSweep)
        {
            if (!WriteEvent("summary", "The default routine enables the mission fan, ramps circle speed without a software ceiling, and if speed plateaus before slip it tightens curvature until sustained encoder-vs-gyro/IMU mismatch indicates traction loss.")) return false;
            if (!WriteEvent("summary", "Use traction_limit_result and the last steady samples before it to estimate the maximum sustainable circle speed, yaw rate, and lateral acceleration.")) return false;
        }
        else
        {
            if (!WriteEvent("summary", "The default routine logs stationary fan-off, fan-on, and recovery phases so you can quantify fan-induced sensor and vibration shifts.")) return false;
        }

        return _file.Write(
            "sample,phase_id,t_us,dt_us,stationary,fan_enabled,"
            "pose_x_m,pose_y_m,yaw_rad,linear_speed_mps,angular_speed_radps,planar_accel_mps2,"
            "cmd_linear_mps,cmd_angular_radps,left_drive_cmd,right_drive_cmd,"
            "left_encoder_count,right_encoder_count,left_distance_m,right_distance_m,left_velocity_mps,right_velocity_mps,"
            "imu_fr_status,imu_fr_gyro_x,imu_fr_gyro_y,imu_fr_gyro_z,imu_fr_accel_x,imu_fr_accel_y,imu_fr_accel_z,imu_fr_temp,imu_fr_int,"
            "imu_bl_status,imu_bl_gyro_x,imu_bl_gyro_y,imu_bl_gyro_z,imu_bl_accel_x,imu_bl_accel_y,imu_bl_accel_z,imu_bl_temp,imu_bl_int,"
            "ws_fl_ambient,ws_fl_lit,ws_fl_delta,ws_fl_raw_distance_m,ws_fl_distance_m,ws_fr_ambient,ws_fr_lit,ws_fr_delta,ws_fr_raw_distance_m,ws_fr_distance_m,"
            "ws_sl_ambient,ws_sl_lit,ws_sl_delta,ws_sl_raw_distance_m,ws_sl_distance_m,ws_sr_ambient,ws_sr_lit,ws_sr_delta,ws_sr_raw_distance_m,ws_sr_distance_m,"
            "front_wall,left_wall,right_wall,corridor_error_m,front_skew_m,gyro_bias_radps,gyro_raw_radps,gyro_radps\n");
    }

    bool BeginPhase(const char* name)
    {
        ++_phaseId;

        char line[160] = {};
        const int length = snprintf(
            line,
            sizeof(line),
            "# phase,%lu,%lu,%s\n",
            _phaseId,
            micros(),
            (name != nullptr) ? name : "");

        if (length <= 0 || length >= static_cast<int>(sizeof(line)))
        {
            return false;
        }

        const bool ok = _file.Write(line);
        FlushIfNeeded(true);
        return ok;
    }

    bool WriteEvent(const char* type, const char* message)
    {
        char line[MazeMap::kDiagnosticEventLineCapacity] = {};
        const int length = snprintf(
            line,
            sizeof(line),
            "# event,%lu,%s,%s\n",
            micros(),
            (type != nullptr) ? type : "",
            (message != nullptr) ? message : "");

        if (length <= 0 || length >= static_cast<int>(sizeof(line)))
        {
            return false;
        }

        const bool ok = _file.Write(line);
        FlushIfNeeded(true);
        return ok;
    }

    bool LogSample(
        bool stationary,
        bool fanEnabled,
        uint32_t timestampUs,
        uint32_t dtUs,
        const PoseEstimate& pose,
        const DriveBase& drive,
        const DriveTelemetry& driveTelemetry,
        const DiagnosticSensorSnapshot& sensorSnapshot,
        float planarAccelMps2)
    {
        char line[2048] = {};
        const int length = snprintf(
            line,
            sizeof(line),
            "%lu,%lu,%lu,%lu,%u,%u,"
            "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
            "%.6f,%.6f,%.4f,%.4f,"
            "%ld,%ld,%.6f,%.6f,%.6f,%.6f,"
            "%u,%d,%d,%d,%d,%d,%d,%d,%u,"
            "%u,%d,%d,%d,%d,%d,%d,%d,%u,"
            "%.3f,%.3f,%.3f,%.6f,%.6f,%.3f,%.3f,%.3f,%.6f,%.6f,"
            "%.3f,%.3f,%.3f,%.6f,%.6f,%.3f,%.3f,%.3f,%.6f,%.6f,"
            "%u,%u,%u,%.6f,%.6f,%.6f,%.6f,%.6f\n",
            _sampleCount,
            _phaseId,
            static_cast<unsigned long>(timestampUs),
            static_cast<unsigned long>(dtUs),
            stationary ? 1U : 0U,
            fanEnabled ? 1U : 0U,
            pose.xMeters,
            pose.yMeters,
            pose.yawRad,
            pose.linearSpeedMps,
            pose.angularSpeedRadps,
            planarAccelMps2,
            drive.GetLastLinearCommandMps(),
            drive.GetLastAngularCommandRadps(),
            driveTelemetry.leftDriveCommand,
            driveTelemetry.rightDriveCommand,
            static_cast<long>(driveTelemetry.leftEncoderCount),
            static_cast<long>(driveTelemetry.rightEncoderCount),
            driveTelemetry.leftDistanceM,
            driveTelemetry.rightDistanceM,
            driveTelemetry.leftVelocityMps,
            driveTelemetry.rightVelocityMps,
            sensorSnapshot.imuFrontRight.status,
            sensorSnapshot.imuFrontRight.gyroX,
            sensorSnapshot.imuFrontRight.gyroY,
            sensorSnapshot.imuFrontRight.gyroZ,
            sensorSnapshot.imuFrontRight.accelX,
            sensorSnapshot.imuFrontRight.accelY,
            sensorSnapshot.imuFrontRight.accelZ,
            sensorSnapshot.imuFrontRight.temp,
            sensorSnapshot.imuFrontRight.interruptHigh ? 1U : 0U,
            sensorSnapshot.imuBackLeft.status,
            sensorSnapshot.imuBackLeft.gyroX,
            sensorSnapshot.imuBackLeft.gyroY,
            sensorSnapshot.imuBackLeft.gyroZ,
            sensorSnapshot.imuBackLeft.accelX,
            sensorSnapshot.imuBackLeft.accelY,
            sensorSnapshot.imuBackLeft.accelZ,
            sensorSnapshot.imuBackLeft.temp,
            sensorSnapshot.imuBackLeft.interruptHigh ? 1U : 0U,
            sensorSnapshot.frontLeft.ambientLight,
            sensorSnapshot.frontLeft.litLight,
            sensorSnapshot.frontLeft.differentialLight,
            sensorSnapshot.frontLeft.rawDistanceM,
            sensorSnapshot.frontLeft.distanceM,
            sensorSnapshot.frontRight.ambientLight,
            sensorSnapshot.frontRight.litLight,
            sensorSnapshot.frontRight.differentialLight,
            sensorSnapshot.frontRight.rawDistanceM,
            sensorSnapshot.frontRight.distanceM,
            sensorSnapshot.sideLeft.ambientLight,
            sensorSnapshot.sideLeft.litLight,
            sensorSnapshot.sideLeft.differentialLight,
            sensorSnapshot.sideLeft.rawDistanceM,
            sensorSnapshot.sideLeft.distanceM,
            sensorSnapshot.sideRight.ambientLight,
            sensorSnapshot.sideRight.litLight,
            sensorSnapshot.sideRight.differentialLight,
            sensorSnapshot.sideRight.rawDistanceM,
            sensorSnapshot.sideRight.distanceM,
            sensorSnapshot.frontWall ? 1U : 0U,
            sensorSnapshot.leftWall ? 1U : 0U,
            sensorSnapshot.rightWall ? 1U : 0U,
            sensorSnapshot.corridorErrorM,
            sensorSnapshot.frontSkewM,
            sensorSnapshot.gyroBiasRadps,
            sensorSnapshot.gyroRawRadps,
            sensorSnapshot.gyroRadps);

        if (length <= 0 || length >= static_cast<int>(sizeof(line)))
        {
            return false;
        }

        if (!_file.Write(line))
        {
            return false;
        }

        ++_sampleCount;
        FlushIfNeeded(false);
        return true;
    }

    void Flush()
    {
        _file.Flush();
        _lastFlushMs = millis();
    }

    void Close()
    {
        _file.Close();
    }

    const char* GetFileName() const
    {
        return _fileName;
    }

private:
    MazeMap::CoreFileExport _file;
    char _fileName[24];
    unsigned long _phaseId;
    unsigned long _sampleCount;
    unsigned long _lastFlushMs;

    bool WriteMetadata(const char* key, const char* value)
    {
        char line[128] = {};
        const int length = snprintf(
            line,
            sizeof(line),
            "# meta,%s,%s\n",
            (key != nullptr) ? key : "",
            (value != nullptr) ? value : "");

        if (length <= 0 || length >= static_cast<int>(sizeof(line)))
        {
            return false;
        }
        return _file.Write(line);
    }

    bool WriteMetadataUL(const char* key, unsigned long value)
    {
        char line[128] = {};
        const int length = snprintf(
            line,
            sizeof(line),
            "# meta,%s,%lu\n",
            (key != nullptr) ? key : "",
            value);

        if (length <= 0 || length >= static_cast<int>(sizeof(line)))
        {
            return false;
        }
        return _file.Write(line);
    }

    bool WriteMetadataFloat(const char* key, float value, uint8_t precision)
    {
        char format[32] = {};
        char line[128] = {};
        const int formatLength = snprintf(format, sizeof(format), "# meta,%%s,%%.%uf\n", static_cast<unsigned>(precision));
        if (formatLength <= 0 || formatLength >= static_cast<int>(sizeof(format)))
        {
            return false;
        }

        const int length = snprintf(line, sizeof(line), format, (key != nullptr) ? key : "", value);
        if (length <= 0 || length >= static_cast<int>(sizeof(line)))
        {
            return false;
        }
        return _file.Write(line);
    }

    void FlushIfNeeded(bool force)
    {
        const unsigned long nowMs = millis();
        if (force || static_cast<unsigned long>(nowMs - _lastFlushMs) >= AuxMeasurementConfig::kLogFlushPeriodMs)
        {
            _file.Flush();
            _lastFlushMs = nowMs;
        }
    }
};

class AuxMeasurementController
{
public:
    AuxMeasurementController()
        : _vehicle()
        , _sensors(_vehicle, gWallDistanceCalibration)
        , _drive()
        , _logger()
        , _faulted(false)
        , _fanEnabled(false)
        , _lastControlMicros(0UL)
    {
    }

    bool Begin()
    {
        Serial.begin(115200);
        delay(1000);
        Serial.println("Auxiliary measurement mode");

        if (!SetupHardware())
        {
            return Fail("Hardware setup failed");
        }
        ResetStartupTrace("mode:aux_measurement");
        if (!_drive.Begin())
        {
            return Fail("Drive base init failed");
        }
        if (AuxMeasurementConfig::kRoutine == AuxMeasurementConfig::Routine::TurningTractionSweep)
        {
            _drive.SetWheelControlProfile(BuildTurningTractionWheelControlProfile());
        }
        else
        {
            _drive.UseNominalWheelControlProfile();
        }
        SetFanEnabled(false);
        gWallDistanceCalibration.Clear();
        if (!_sensors.Begin(AuxMeasurementConfig::kControlPeriodUs))
        {
            return Fail("Auxiliary sensor init failed");
        }
        if (!_logger.Begin(_sensors, AuxMeasurementConfig::kRoutine))
        {
            return Fail("Auxiliary measurement log open failed");
        }

        _drive.SnapTo(MazeMap::DirectionalLocation(MazeMap::MazeLocation::CellCenter(MazeMap::CellCoordinates(0, 0)), MazeMap::Up));
        _lastControlMicros = micros();
        return true;
    }

    void Run()
    {
        if (_faulted)
        {
            return;
        }

        Serial.println("Entered by shorting pins 28-29 at boot.");
        Serial.println("This mode uses internal sensors only; edit AuxMeasurementConfig::kRoutine for other one-off measurements.");

        const bool ok = RunSelectedRoutine();

        _drive.Brake();
        _drive.UseNominalWheelControlProfile();
        SetFanEnabled(false);
        _logger.Flush();
        if (ok)
        {
            Serial.print("Auxiliary measurement complete, log saved to ");
            Serial.println(_logger.GetFileName());
        }
        _logger.Close();
    }

private:
    MazeMap::Vehicle _vehicle;
    DiagnosticSensorSuite _sensors;
    DriveBase _drive;
    AuxMeasurementLogger _logger;
    bool _faulted;
    bool _fanEnabled;
    unsigned long _lastControlMicros;

    bool RunSelectedRoutine()
    {
        switch (AuxMeasurementConfig::kRoutine)
        {
        case AuxMeasurementConfig::Routine::FanStaticSurvey:
            return RunFanStaticSurvey();
        case AuxMeasurementConfig::Routine::TurningTractionSweep:
            return RunTurningTractionSweep();
        default:
            return Fail("Unknown auxiliary measurement routine");
        }
    }

    bool RunFanStaticSurvey()
    {
        bool ok = true;
        ok = ok && HoldPhase("startup_settle", AuxMeasurementConfig::kStartupSettleMs, true, false);
        ok = ok && HoldPhase("fan_off_baseline", AuxMeasurementConfig::kBaselineHoldMs, true, false);
        ok = ok && HoldPhase("fan_on_hold", AuxMeasurementConfig::kFanHoldMs, true, true);
        ok = ok && HoldPhase("fan_off_recovery", AuxMeasurementConfig::kRecoveryHoldMs, true, false);
        return ok;
    }

    bool RunTurningTractionSweep()
    {
        bool ok = true;
        ok = ok && HoldPhase("startup_settle", AuxMeasurementConfig::kStartupSettleMs, true, false);
        ok = ok && HoldPhase("fan_spinup", AuxMeasurementConfig::kTurningTractionSweepFanSettleMs, true, true);
        if (!ok)
        {
            return false;
        }

        if (!_logger.BeginPhase("turning_traction_sweep"))
        {
            return Fail("Failed to begin turning traction sweep phase");
        }

        SetFanEnabled(true);
        _drive.SetWheelControlProfile(BuildTurningTractionWheelControlProfile());
        const float directionSign = AuxMeasurementConfig::kTurningTractionSweepClockwise ? -1.0f : 1.0f;
        const float circleRadiusM = AuxMeasurementConfig::kTurningTractionSweepRadiusM;
        float commandedSpeedMps = AuxMeasurementConfig::kTurningTractionSweepStartSpeedMps;
        float heldSpeedMps = commandedSpeedMps;
        float commandedCurvatureMInv = (circleRadiusM > 1.0e-6f) ? (1.0f / circleRadiusM) : 0.0f;
        float targetYawRad = _drive.GetPose().yawRad;
        const unsigned long phaseStartMs = millis();
        unsigned long saturationPlateauStartMs = 0UL;
        unsigned long slipCandidateStartMs = 0UL;
        bool slipCandidateActive = false;
        bool tighteningTurn = false;
        MazeMap::TurningTractionMetrics lastMetrics{};
        float lastPlanarAccelMps2 = 0.0f;
        float lastCommandedOmegaRadps = 0.0f;
        float saturationReferenceSpeedMps = 0.0f;

        while (!_faulted)
        {
            const unsigned long nowMs = millis();
            if (static_cast<unsigned long>(nowMs - phaseStartMs) >= AuxMeasurementConfig::kTurningTractionSweepTimeoutMs)
            {
                _drive.Brake();
                return WriteTurningTractionResult(
                    "timeout",
                    false,
                    static_cast<unsigned long>(nowMs - phaseStartMs),
                    commandedSpeedMps,
                    lastCommandedOmegaRadps,
                    lastMetrics,
                    lastPlanarAccelMps2);
            }

            uint32_t timestampUs = 0U;
            uint32_t dtUs = 0U;
            WaitForNextSample(timestampUs, dtUs);

            const DiagnosticSensorSnapshot sensorSnapshot = _sensors.Capture(false, _drive.GetPose());
            const float dtSeconds = static_cast<float>(dtUs) * 1.0e-6f;
            _drive.UpdateOdometry(dtSeconds, sensorSnapshot.gyroRadps);
            if (!tighteningTurn)
            {
                commandedSpeedMps += AuxMeasurementConfig::kTurningTractionSweepAccelMps2 * dtSeconds;
                if (AuxMeasurementConfig::kTurningTractionSweepMaxSpeedMps > 0.0f)
                {
                    commandedSpeedMps = (std::min)(AuxMeasurementConfig::kTurningTractionSweepMaxSpeedMps, commandedSpeedMps);
                }
                heldSpeedMps = commandedSpeedMps;
            }
            else
            {
                commandedSpeedMps = heldSpeedMps;
                commandedCurvatureMInv += AuxMeasurementConfig::kTurningTractionCurvatureRampMInvPerSec * dtSeconds;
            }

            const float nominalOmegaRadps = directionSign * (commandedSpeedMps * commandedCurvatureMInv);
            targetYawRad = WrapAngleRad(targetYawRad + (nominalOmegaRadps * dtSeconds));
            lastCommandedOmegaRadps = MazeMap::ComputeTurningTractionAngularCommand(
                nominalOmegaRadps,
                targetYawRad,
                _drive.GetPose().yawRad,
                _drive.GetPose().angularSpeedRadps,
                Config::kArcHeadingKp,
                Config::kArcYawD,
                AuxMeasurementConfig::kTurningTractionSweepMaxAngularCommandRadps);
            const float effectiveTrackWidthM =
                MazeMap::Vehicle::GetEffectiveTrackWidthForMotion(commandedSpeedMps, lastCommandedOmegaRadps);
            if (static_cast<unsigned long>(nowMs - phaseStartMs) < AuxMeasurementConfig::kTurningTractionLaunchMs)
            {
                const MazeMap::TurningLaunchCommands launchCommands = MazeMap::ComputeTurningLaunchCommands(
                    commandedSpeedMps,
                    lastCommandedOmegaRadps,
                    effectiveTrackWidthM,
                    Config::kWheelRestLaunchDriveCommand);
                _drive.CommandOpenLoopRaw(launchCommands.leftCommand, launchCommands.rightCommand);
            }
            else
            {
                _drive.CommandVelocity(commandedSpeedMps, lastCommandedOmegaRadps, dtSeconds);
            }

            const DriveTelemetry driveTelemetry = _drive.GetTelemetry();
            const float planarAccelMps2 = _sensors.GetPlanarAccelMps2(sensorSnapshot);
            const MazeMap::TurningTractionMetrics metrics = MazeMap::ComputeTurningTractionMetrics(
                driveTelemetry.leftVelocityMps,
                driveTelemetry.rightVelocityMps,
                effectiveTrackWidthM,
                sensorSnapshot.gyroRadps,
                planarAccelMps2);
            lastMetrics = metrics;
            lastPlanarAccelMps2 = planarAccelMps2;

            if (!_logger.LogSample(
                false,
                _fanEnabled,
                timestampUs,
                dtUs,
                _drive.GetPose(),
                _drive,
                driveTelemetry,
                sensorSnapshot,
                planarAccelMps2))
            {
                return Fail("Failed to write turning traction sample");
            }

            const bool slipDetected = MazeMap::IsTurningTractionLossDetected(
                metrics,
                AuxMeasurementConfig::kTurningTractionSlipMinSpeedMps,
                AuxMeasurementConfig::kTurningTractionSlipMinLatAccelMps2,
                AuxMeasurementConfig::kTurningTractionSlipYawCoherenceFloor,
                AuxMeasurementConfig::kTurningTractionSlipPlanarCoherenceFloor);
            if (slipDetected)
            {
                if (!slipCandidateActive)
                {
                    slipCandidateStartMs = nowMs;
                    slipCandidateActive = true;
                }
                else if (static_cast<unsigned long>(nowMs - slipCandidateStartMs) >= AuxMeasurementConfig::kTurningTractionSlipConfirmMs)
                {
                    _drive.Brake();
                    return WriteTurningTractionResult(
                        "traction_loss",
                        true,
                        static_cast<unsigned long>(nowMs - phaseStartMs),
                        commandedSpeedMps,
                        lastCommandedOmegaRadps,
                        metrics,
                        planarAccelMps2);
                }
            }
            else
            {
                slipCandidateActive = false;
            }

            const float maxWheelCommandMagnitude = (std::max)(
                std::fabs(driveTelemetry.leftDriveCommand),
                std::fabs(driveTelemetry.rightDriveCommand));
            if (!tighteningTurn)
            {
                const bool actuatorLimited =
                    (metrics.encoderLinearSpeedMps >= AuxMeasurementConfig::kTurningTractionPlateauMinSpeedMps) &&
                    (maxWheelCommandMagnitude >= AuxMeasurementConfig::kTurningTractionActuatorCeilingCommand);

                if (!actuatorLimited)
                {
                    saturationPlateauStartMs = 0UL;
                    saturationReferenceSpeedMps = metrics.encoderLinearSpeedMps;
                }
                else if (saturationPlateauStartMs == 0UL)
                {
                    saturationPlateauStartMs = nowMs;
                    saturationReferenceSpeedMps = metrics.encoderLinearSpeedMps;
                }
                else if (metrics.encoderLinearSpeedMps >= (saturationReferenceSpeedMps + AuxMeasurementConfig::kTurningTractionPlateauDeltaMps))
                {
                    saturationPlateauStartMs = nowMs;
                    saturationReferenceSpeedMps = metrics.encoderLinearSpeedMps;
                }
                else if (static_cast<unsigned long>(nowMs - saturationPlateauStartMs) >= AuxMeasurementConfig::kTurningTractionPlateauWindowMs)
                {
                    tighteningTurn = true;
                    heldSpeedMps = (std::max)(commandedSpeedMps, metrics.encoderLinearSpeedMps);

                    char message[160] = {};
                    const int messageLength = snprintf(
                        message,
                        sizeof(message),
                        "reason=speed_plateau;hold_v_mps=%.3f;curvature_m_inv=%.3f;outer_cmd=%.3f",
                        heldSpeedMps,
                        commandedCurvatureMInv,
                        maxWheelCommandMagnitude);
                    if (messageLength <= 0 || messageLength >= static_cast<int>(sizeof(message)))
                    {
                        return Fail("Failed to format turning traction mode event");
                    }
                    if (!_logger.WriteEvent("turning_traction_mode", message))
                    {
                        return Fail("Failed to write turning traction mode event");
                    }
                }
            }
        }

        return false;
    }

    bool HoldPhase(const char* phaseName, uint16_t durationMs, bool stationary, bool fanEnabled)
    {
        if (!_logger.BeginPhase(phaseName))
        {
            return Fail("Failed to begin auxiliary measurement phase");
        }

        SetFanEnabled(fanEnabled);
        const unsigned long startMs = millis();
        while (!_faulted && static_cast<unsigned long>(millis() - startMs) < durationMs)
        {
            uint32_t timestampUs = 0U;
            uint32_t dtUs = 0U;
            WaitForNextSample(timestampUs, dtUs);

            _drive.Brake();
            const DiagnosticSensorSnapshot sensorSnapshot = _sensors.Capture(stationary, _drive.GetPose());
            const float dtSeconds = static_cast<float>(dtUs) * 1.0e-6f;
            _drive.UpdateOdometry(dtSeconds, sensorSnapshot.gyroRadps);
            const DriveTelemetry driveTelemetry = _drive.GetTelemetry();
            const float planarAccelMps2 = _sensors.GetPlanarAccelMps2(sensorSnapshot);
            if (!_logger.LogSample(
                stationary,
                _fanEnabled,
                timestampUs,
                dtUs,
                _drive.GetPose(),
                _drive,
                driveTelemetry,
                sensorSnapshot,
                planarAccelMps2))
            {
                return Fail("Failed to write auxiliary measurement sample");
            }
        }

        return !_faulted;
    }

    void SetFanEnabled(bool enabled)
    {
        if (_fanEnabled == enabled)
        {
            return;
        }

        _fanEnabled = enabled;
        SetMissionLevelFanEnabled(enabled);
    }

    void WaitForNextSample(uint32_t& timestampUs, uint32_t& dtUs)
    {
        while ((micros() - _lastControlMicros) < AuxMeasurementConfig::kControlPeriodUs)
        {
            delayMicroseconds(50);
        }

        timestampUs = micros();
        dtUs = static_cast<uint32_t>(timestampUs - _lastControlMicros);
        _lastControlMicros = timestampUs;
    }

    bool WriteTurningTractionResult(
        const char* reason,
        bool slipDetected,
        unsigned long elapsedMs,
        float commandedSpeedMps,
        float commandedOmegaRadps,
        const MazeMap::TurningTractionMetrics& metrics,
        float planarAccelMps2)
    {
        char message[256] = {};
        const int length = snprintf(
            message,
            sizeof(message),
            "reason=%s;slip=%u;elapsed_ms=%lu;cmd_v_mps=%.3f;cmd_w_radps=%.3f;enc_v_mps=%.3f;enc_w_radps=%.3f;pred_lat_mps2=%.3f;planar_accel_mps2=%.3f;yaw_ratio=%.3f;planar_ratio=%.3f",
            (reason != nullptr) ? reason : "unknown",
            slipDetected ? 1U : 0U,
            elapsedMs,
            commandedSpeedMps,
            commandedOmegaRadps,
            metrics.encoderLinearSpeedMps,
            metrics.encoderOmegaRadps,
            metrics.predictedLateralAccelMps2,
            planarAccelMps2,
            metrics.yawCoherence,
            metrics.planarCoherence);

        if (length <= 0 || length >= static_cast<int>(sizeof(message)))
        {
            return Fail("Turning traction result event overflowed");
        }
        if (_logger.WriteEvent("traction_limit_result", message))
        {
            return true;
        }
        return Fail("Failed to write turning traction result");
    }

    bool Fail(const char* reason)
    {
        _faulted = true;
        _drive.Brake();
        SetFanEnabled(false);
        Serial.print("Auxiliary measurement fault: ");
        Serial.println((reason != nullptr) ? reason : "unknown");
        if (reason != nullptr && reason[0] != '\0')
        {
            _logger.WriteEvent("fault", reason);
        }
        AppendStartupTrace((reason != nullptr) ? reason : "aux_measurement_fault");
        return false;
    }

    static MazeMap::WheelControlProfile BuildTurningTractionWheelControlProfile()
    {
        return BuildNominalWheelControlProfile();
    }
};

class WallSensorLedCalibrationController
{
public:
    bool Begin()
    {
        Serial.begin(115200);
        delay(1000);
        Serial.println("Wall sensor LED calibration mode");
        ResetStartupTrace("mode:wall_sensor_led_calibration");

        pinMode(Pins::LED_Ctrl_Forward_Left, OUTPUT);
        pinMode(Pins::LED_Ctrl_Forward_Right, OUTPUT);
        pinMode(Pins::LED_Ctrl_Side_Left, OUTPUT);
        pinMode(Pins::LED_Ctrl_Side_Right, OUTPUT);
        SetFrontLeds(false);
        SetSideLeds(false);
        BeginJumperMonitor();

        Serial.println("Front calibration active; side LEDs held off");
        PrintFrequency("Front LED square wave (Hz): ", WallSensorLedCalibrationHalfPeriodUs(WallSensorId::FrontLeft));
        Serial.println("Remove jumper on pins 38-39 to switch to side calibration");
        return true;
    }

    void Run()
    {
        const uint32_t frontHalfPeriodUs = WallSensorLedCalibrationHalfPeriodUs(WallSensorId::FrontLeft);
        const uint32_t sideHalfPeriodUs = WallSensorLedCalibrationHalfPeriodUs(WallSensorId::SideLeft);
        RunFrontCalibration(frontHalfPeriodUs);

        SetFrontLeds(false);
        SetSideLeds(false);
        Serial.println("Side calibration active; front LEDs held off");
        PrintFrequency("Side LED square wave (Hz): ", sideHalfPeriodUs);
        RunSideCalibration(sideHalfPeriodUs);
    }

private:
    static void BeginJumperMonitor()
    {
        pinMode(LedCalibrationConfig::kModeSelectPinA, OUTPUT);
        digitalWriteFast(LedCalibrationConfig::kModeSelectPinA, LOW);
        pinMode(LedCalibrationConfig::kModeSelectPinB, INPUT_PULLUP);
    }

    static bool IsCalibrationJumperInstalled()
    {
        return digitalReadFast(LedCalibrationConfig::kModeSelectPinB) == LOW;
    }

    static void SetFrontLeds(bool enabled)
    {
        digitalWriteFast(Pins::LED_Ctrl_Forward_Left, enabled ? HIGH : LOW);
        digitalWriteFast(Pins::LED_Ctrl_Forward_Right, enabled ? HIGH : LOW);
    }

    static void SetSideLeds(bool enabled)
    {
        digitalWriteFast(Pins::LED_Ctrl_Side_Left, enabled ? HIGH : LOW);
        digitalWriteFast(Pins::LED_Ctrl_Side_Right, enabled ? HIGH : LOW);
    }

    static void RunFrontCalibration(uint32_t halfPeriodUs)
    {
        bool enabled = false;
        unsigned long lastToggleUs = micros();

        while (IsCalibrationJumperInstalled())
        {
            const unsigned long nowUs = micros();
            if (static_cast<uint32_t>(nowUs - lastToggleUs) >= halfPeriodUs)
            {
                enabled = !enabled;
                SetFrontLeds(enabled);
                lastToggleUs = nowUs;
            }
        }
    }

    static void RunSideCalibration(uint32_t halfPeriodUs)
    {
        bool enabled = false;
        unsigned long lastToggleUs = micros();

        while (true)
        {
            const unsigned long nowUs = micros();
            if (static_cast<uint32_t>(nowUs - lastToggleUs) >= halfPeriodUs)
            {
                enabled = !enabled;
                SetSideLeds(enabled);
                lastToggleUs = nowUs;
            }
        }
    }

    static void PrintFrequency(const char* label, uint32_t halfPeriodUs)
    {
        Serial.print(label);
        if (halfPeriodUs == 0U)
        {
            Serial.println(0.0f, 3);
            return;
        }

        Serial.println(1000000.0f / (2.0f * static_cast<float>(halfPeriodUs)), 3);
    }
};

class DiagnosticController
{
public:
    DiagnosticController()
        : _vehicle()
        , _sensors(_vehicle, gWallDistanceCalibration)
        , _drive()
        , _logger()
        , _startX(0.0f)
        , _startY(0.0f)
        , _faulted(false)
        , _lastControlMicros(0UL)
    {
    }

    bool Begin()
    {
        Serial.begin(115200);
        delay(1000);
        Serial.println("Micromouse diagnostic setup");

        if (!SetupHardware())
        {
            return Fail("Hardware setup failed");
        }
        ResetStartupTrace("mode:primary_diagnostic");
        if (!_drive.Begin())
        {
            return Fail("Drive base init failed");
        }
        _drive.SetWheelControlProfile(BuildDiagnosticWheelControlProfile());
        SetMissionLevelFanEnabled(true);
        gWallDistanceCalibration.Clear();
        if (!_sensors.Begin(DiagnosticConfig::kControlPeriodUs))
        {
            return Fail("Diagnostic sensor init failed");
        }
        if (!_logger.Begin(_sensors))
        {
            return Fail("Diagnostic log open failed");
        }

        _drive.SnapTo(MazeMap::DirectionalLocation(MazeMap::MazeLocation::CellCenter(MazeMap::CellCoordinates(0, 0)), MazeMap::Up));
        _startX = _drive.GetPose().xMeters;
        _startY = _drive.GetPose().yMeters;
        _lastControlMicros = micros();
        return true;
    }

    void Run()
    {
        if (_faulted)
        {
            return;
        }

        bool ok = true;
        ok = ok && HoldPhase("startup_settle", DiagnosticConfig::kStartupSettleMs, true);
        ok = ok && HoldPhase("baseline_idle", DiagnosticConfig::kBaselineHoldMs, true);
        ok = ok && ExecuteKickoffSweep();
        ok = ok && ExecuteForwardSweep();
        ok = ok && HoldPhase("characterization_settle", DiagnosticConfig::kInterTestHoldMs, true);
        ok = ok && ExecuteTurnPhase("turn_cw_90_1", -0.5f * PI);
        ok = ok && ExecuteTurnPhase("turn_ccw_90_1", 0.5f * PI);
        ok = ok && ExecuteTurnPhase("turn_cw_90_2", -0.5f * PI);
        ok = ok && ExecuteTurnPhase("turn_ccw_90_2", 0.5f * PI);
        ok = ok && ExecuteTurnPhase("turn_cw_180", -PI);
        ok = ok && ExecuteTurnPhase("turn_ccw_180", PI);
        ok = ok && HoldPhase("turn_sweep_settle", DiagnosticConfig::kInterTestHoldMs, true);
        float shortReturnDistanceM = DiagnosticConfig::kShortStraightDistanceM;
        ok = ok && ExecuteStraightPhase("straight_short_forward", DiagnosticConfig::kShortStraightDistanceM, DiagnosticConfig::kSlowStraightSpeedMps, &shortReturnDistanceM);
        shortReturnDistanceM = MazeMap::SelectDiagnosticReturnDistanceM(DiagnosticConfig::kShortStraightDistanceM, shortReturnDistanceM);
        ok = ok && ExecuteTurnPhase("straight_short_turnaround", PI);
        ok = ok && ExecuteStraightPhase("straight_short_return", shortReturnDistanceM, DiagnosticConfig::kSlowStraightSpeedMps);
        ok = ok && ExecuteTurnPhase("straight_short_reset_heading", PI);
        ok = ok && HoldPhase("straight_short_settle", DiagnosticConfig::kInterTestHoldMs, true);
        float longReturnDistanceM = DiagnosticConfig::kLongStraightDistanceM;
        ok = ok && ExecuteStraightPhase("straight_long_forward", DiagnosticConfig::kLongStraightDistanceM, DiagnosticConfig::kFastStraightSpeedMps, &longReturnDistanceM);
        longReturnDistanceM = MazeMap::SelectDiagnosticReturnDistanceM(DiagnosticConfig::kLongStraightDistanceM, longReturnDistanceM);
        ok = ok && ExecuteTurnPhase("straight_long_turnaround", PI);
        ok = ok && ExecuteStraightPhase("straight_long_return", longReturnDistanceM, DiagnosticConfig::kFastStraightSpeedMps);
        ok = ok && ExecuteTurnPhase("straight_long_reset_heading", PI);
        ok = ok && HoldPhase("straight_long_settle", DiagnosticConfig::kInterTestHoldMs, true);
        ok = ok && ExecuteCircleSpeedSweep("slow", DiagnosticConfig::kSlowStraightSpeedMps);
        ok = ok && ExecuteCircleSpeedSweep("medium", DiagnosticConfig::kCircleMediumSpeedMps);
        ok = ok && ExecuteCircleSpeedSweep("fast", DiagnosticConfig::kFastStraightSpeedMps);
        ok = ok && ExecuteSquareLoop("square_cw", -0.5f * PI);
        ok = ok && HoldPhase("square_cw_settle", DiagnosticConfig::kInterTestHoldMs, true);
        ok = ok && ExecuteSquareLoop("square_ccw", 0.5f * PI);
        ok = ok && HoldPhase("final_idle", DiagnosticConfig::kBaselineHoldMs / 2U, true);

        _drive.Brake();
        _drive.UseNominalWheelControlProfile();
        _logger.Flush();

        if (ok)
        {
            Serial.print("Diagnostic complete, log saved to ");
            Serial.println(_logger.GetFileName());
            Serial.println("Use the # event,summary lines in the log header to map phases to tunables.");
        }

        _logger.Close();
        SetMissionLevelFanEnabled(false);
    }

private:
    struct StraightPhaseMetrics
    {
        float peakSpeedMps = 0.0f;
        float maxHeadingErrorRad = 0.0f;
    };

    struct TurnPhaseMetrics
    {
        float peakOmegaRadps = 0.0f;
        float maxYawErrorRad = 0.0f;
    };

    struct ArcPhaseMetrics
    {
        float peakSpeedMps = 0.0f;
        float peakOmegaRadps = 0.0f;
        float maxHeadingErrorRad = 0.0f;
        float durationSeconds = 0.0f;
        float omegaIntegralRad = 0.0f;
        float speedIntegralMpsSeconds = 0.0f;
        float planarAccelIntegralMps2Seconds = 0.0f;
        float peakPlanarAccelMps2 = 0.0f;
    };

    MazeMap::Vehicle _vehicle;
    DiagnosticSensorSuite _sensors;
    DriveBase _drive;
    DiagnosticLogger _logger;
    float _startX;
    float _startY;
    bool _faulted;
    unsigned long _lastControlMicros;

    bool WriteStraightResult(
        const char* phaseName,
        float distanceM,
        float cruiseSpeedMps,
        float traveledM,
        const MazeMap::Vectorf<2>& targetHeading,
        const StraightPhaseMetrics& metrics)
    {
        char message[192] = {};
        const float stopErrorM = traveledM - distanceM;
        const float finalYawErrorDeg = RAD_TO_DEG * HeadingErrorRad(targetHeading, _drive.GetPose().headingUnit);
        const int length = snprintf(
            message,
            sizeof(message),
            "%s;distance_m=%.3f;cruise_mps=%.3f;peak_speed_mps=%.3f;max_heading_err_deg=%.2f;stop_err_m=%.4f;final_yaw_err_deg=%.2f",
            (phaseName != nullptr) ? phaseName : "",
            distanceM,
            cruiseSpeedMps,
            metrics.peakSpeedMps,
            RAD_TO_DEG * metrics.maxHeadingErrorRad,
            stopErrorM,
            finalYawErrorDeg);
        if (length <= 0 || length >= static_cast<int>(sizeof(message)))
        {
            return Fail("Failed to format straight diagnostic result");
        }
        return WriteEventOrFail("straight_result", message, "Failed to write straight diagnostic result");
    }

    bool WriteTurnResult(const char* phaseName, float angleRad, const TurnPhaseMetrics& metrics, float targetYawRad)
    {
        char message[176] = {};
        const float finalYawErrorDeg = RAD_TO_DEG * AngleErrorRad(targetYawRad, _drive.GetPose().yawRad);
        const int length = snprintf(
            message,
            sizeof(message),
            "%s;angle_deg=%.1f;peak_omega_radps=%.3f;peak_yaw_err_deg=%.2f;final_yaw_err_deg=%.2f",
            (phaseName != nullptr) ? phaseName : "",
            RAD_TO_DEG * angleRad,
            metrics.peakOmegaRadps,
            RAD_TO_DEG * metrics.maxYawErrorRad,
            finalYawErrorDeg);
        if (length <= 0 || length >= static_cast<int>(sizeof(message)))
        {
            return Fail("Failed to format turn diagnostic result");
        }
        return WriteEventOrFail("turn_result", message, "Failed to write turn diagnostic result");
    }

    bool WriteArcResult(
        const char* phaseName,
        float distanceM,
        float angleRad,
        float traveledM,
        float targetYawRad,
        const ArcPhaseMetrics& metrics)
    {
        char message[192] = {};
        const float distanceErrorM = traveledM - distanceM;
        const float finalYawErrorDeg = RAD_TO_DEG * AngleErrorRad(targetYawRad, _drive.GetPose().yawRad);
        const int length = snprintf(
            message,
            sizeof(message),
            "%s;dist_m=%.3f;ang_deg=%.1f;peak_w_radps=%.3f;max_head_err_deg=%.2f;dist_err_m=%.4f;final_yaw_err_deg=%.2f",
            (phaseName != nullptr) ? phaseName : "",
            distanceM,
            RAD_TO_DEG * angleRad,
            metrics.peakOmegaRadps,
            RAD_TO_DEG * metrics.maxHeadingErrorRad,
            distanceErrorM,
            finalYawErrorDeg);
        if (length <= 0 || length >= static_cast<int>(sizeof(message)))
        {
            return Fail("Failed to format arc diagnostic result");
        }
        return WriteEventOrFail("arc_result", message, "Failed to write arc diagnostic result");
    }

    bool WriteCircleResult(
        const char* phaseName,
        float cruiseSpeedMps,
        const DriveTelemetry& startTelemetry,
        const ArcPhaseMetrics& metrics)
    {
        const DriveTelemetry endTelemetry = _drive.GetTelemetry();
        const long leftCountDelta = static_cast<long>(endTelemetry.leftEncoderCount - startTelemetry.leftEncoderCount);
        const long rightCountDelta = static_cast<long>(endTelemetry.rightEncoderCount - startTelemetry.rightEncoderCount);
        const float leftDistanceDeltaM = endTelemetry.leftDistanceM - startTelemetry.leftDistanceM;
        const float rightDistanceDeltaM = endTelemetry.rightDistanceM - startTelemetry.rightDistanceM;
        const float averageOmegaRadps = (metrics.durationSeconds > 0.0f) ? (metrics.omegaIntegralRad / metrics.durationSeconds) : 0.0f;
        const float averageSpeedMps = (metrics.durationSeconds > 0.0f) ? (metrics.speedIntegralMpsSeconds / metrics.durationSeconds) : 0.0f;
        const float effectiveTrackWidthM =
            MazeMap::Vehicle::GetEffectiveTrackWidthForMotion(averageSpeedMps, averageOmegaRadps);
        const float encoderYawDeg =
            (effectiveTrackWidthM > 0.0f)
            ? (RAD_TO_DEG * ((rightDistanceDeltaM - leftDistanceDeltaM) / effectiveTrackWidthM))
            : 0.0f;
        const float estimatedLateralAccelMps2 = std::fabs(averageSpeedMps * averageOmegaRadps);
        const float averageLateralAccelMps2 = (metrics.durationSeconds > 0.0f) ? (metrics.planarAccelIntegralMps2Seconds / metrics.durationSeconds) : 0.0f;

        char message[256] = {};
        const int length = snprintf(
            message,
            sizeof(message),
            "%s;cruise_mps=%.3f;l_cnt=%ld;r_cnt=%ld;enc_yaw_deg=%.1f;avg_speed_mps=%.3f;avg_omega_radps=%.3f;est_lat_mps2=%.3f;avg_lat_mps2=%.3f;peak_lat_mps2=%.3f",
            (phaseName != nullptr) ? phaseName : "",
            cruiseSpeedMps,
            leftCountDelta,
            rightCountDelta,
            encoderYawDeg,
            averageSpeedMps,
            averageOmegaRadps,
            estimatedLateralAccelMps2,
            averageLateralAccelMps2,
            metrics.peakPlanarAccelMps2);
        if (length <= 0 || length >= static_cast<int>(sizeof(message)))
        {
            return Fail("Failed to format circle diagnostic result");
        }
        return WriteEventOrFail("circle_result", message, "Failed to write circle diagnostic result");
    }

    bool WriteClosureResult(const char* type, const char* phaseName, const PoseEstimate& startPose, const char* failMessage)
    {
        char message[160] = {};
        const PoseEstimate& pose = _drive.GetPose();
        const float deltaXM = pose.xMeters - startPose.xMeters;
        const float deltaYM = pose.yMeters - startPose.yMeters;
        const float closureErrorM = std::sqrt((deltaXM * deltaXM) + (deltaYM * deltaYM));
        const float finalYawErrorDeg = RAD_TO_DEG * AngleErrorRad(startPose.yawRad, pose.yawRad);
        const int length = snprintf(
            message,
            sizeof(message),
            "%s;closure_err_m=%.4f;final_yaw_err_deg=%.2f",
            (phaseName != nullptr) ? phaseName : "",
            closureErrorM,
            finalYawErrorDeg);
        if (length <= 0 || length >= static_cast<int>(sizeof(message)))
        {
            return Fail("Failed to format diagnostic closure result");
        }
        return WriteEventOrFail(type, message, failMessage);
    }

    static void AccumulateArcMetrics(ArcPhaseMetrics& total, const ArcPhaseMetrics& segment)
    {
        total.peakSpeedMps = (std::max)(total.peakSpeedMps, segment.peakSpeedMps);
        total.peakOmegaRadps = (std::max)(total.peakOmegaRadps, segment.peakOmegaRadps);
        total.maxHeadingErrorRad = (std::max)(total.maxHeadingErrorRad, segment.maxHeadingErrorRad);
        total.durationSeconds += segment.durationSeconds;
        total.omegaIntegralRad += segment.omegaIntegralRad;
        total.speedIntegralMpsSeconds += segment.speedIntegralMpsSeconds;
        total.planarAccelIntegralMps2Seconds += segment.planarAccelIntegralMps2Seconds;
        total.peakPlanarAccelMps2 = (std::max)(total.peakPlanarAccelMps2, segment.peakPlanarAccelMps2);
    }

    static MotionLimits DiagnosticLimits(float maxSpeedMps)
    {
        MotionLimits limits{};
        limits.maxSpeedMps = maxSpeedMps;
        limits.accelMps2 = DiagnosticConfig::kStraightAccelMps2;
        limits.decelMps2 = DiagnosticConfig::kStraightDecelMps2;
        limits.maxAngularSpeedRadps = DiagnosticConfig::kTurnMaxOmegaRadps;
        limits.angularAccelRadps2 = DiagnosticConfig::kTurnAccelRadps2;
        return limits;
    }

    bool WriteEventOrFail(const char* type, const char* message, const char* failMessage)
    {
        if (_logger.WriteEvent(type, message))
        {
            return true;
        }

        return Fail(failMessage);
    }

    static void BuildDriveCommandLabel(const char* prefix, float driveCommand, char* buffer, size_t bufferSize)
    {
        const unsigned drivePercent = static_cast<unsigned>((100.0f * driveCommand) + 0.5f);
        snprintf(buffer, bufferSize, "%s_%03u", (prefix != nullptr) ? prefix : "cmd", drivePercent);
    }

    static MazeMap::WheelControlProfile BuildDiagnosticWheelControlProfile()
    {
        return BuildNominalWheelControlProfile();
    }

    bool Fail(const char* message)
    {
        _faulted = true;
        SetMissionLevelFanEnabled(false);
        _drive.Brake();
        _drive.UseNominalWheelControlProfile();
        Serial.print("DIAGNOSTIC FAULT: ");
        Serial.println(message);
        _logger.WriteEvent("fault", message);
        _logger.Flush();
        return false;
    }

    bool StartPhase(const char* name)
    {
        Serial.print("Diagnostic phase: ");
        Serial.println(name);
        if (_logger.BeginPhase(name))
        {
            return true;
        }
        return Fail("Failed to write diagnostic phase marker");
    }

    bool IsWithinBoundary() const
    {
        const PoseEstimate& pose = _drive.GetPose();
        return (std::fabs(pose.xMeters - _startX) <= DiagnosticConfig::kBoundaryHalfSpanM) &&
            (std::fabs(pose.yMeters - _startY) <= DiagnosticConfig::kBoundaryHalfSpanM);
    }

    bool TickControl(bool stationary, float& dtSeconds, uint32_t& timestampUs, DiagnosticSensorSnapshot& snapshot)
    {
        while ((micros() - _lastControlMicros) < DiagnosticConfig::kControlPeriodUs)
        {
            delayMicroseconds(20);
        }

        timestampUs = micros();
        dtSeconds = static_cast<float>(timestampUs - _lastControlMicros) * 1.0e-6f;
        _lastControlMicros = timestampUs;

        snapshot = _sensors.Capture(stationary, _drive.GetPose());
        _drive.UpdateOdometry(dtSeconds, snapshot.gyroRadps);

        if (!IsWithinBoundary())
        {
            return Fail("Diagnostic boundary exceeded");
        }

        return true;
    }

    bool LogSample(bool stationary, uint32_t timestampUs, float dtSeconds, const DiagnosticSensorSnapshot& snapshot)
    {
        const DriveTelemetry telemetry = _drive.GetTelemetry();
        const uint32_t dtUs = static_cast<uint32_t>(dtSeconds * 1.0e6f);
        if (_logger.LogSample(stationary, timestampUs, dtUs, _drive.GetPose(), _drive, telemetry, snapshot))
        {
            return true;
        }
        return Fail("Failed to write diagnostic sample");
    }

    bool HoldPhase(const char* phaseName, uint16_t durationMs, bool stationary)
    {
        if (!StartPhase(phaseName))
        {
            return false;
        }

        const unsigned long deadline = millis() + durationMs;
        while (static_cast<long>(deadline - millis()) > 0)
        {
            float dtSeconds = 0.0f;
            uint32_t timestampUs = 0UL;
            DiagnosticSensorSnapshot snapshot{};
            if (!TickControl(stationary, dtSeconds, timestampUs, snapshot))
            {
                return false;
            }

            _drive.Brake();
            if (!LogSample(true, timestampUs, dtSeconds, snapshot))
            {
                return false;
            }
        }

        return true;
    }

    bool ExecuteStraightPhase(const char* phaseName, float distanceM, float cruiseSpeedMps, float* outTraveledDistanceM = nullptr)
    {
        if (!StartPhase(phaseName))
        {
            return false;
        }

        const MotionLimits limits = DiagnosticLimits(cruiseSpeedMps);
        const float startDistanceM = _drive.GetAverageDistanceMeters();
        const MazeMap::Vectorf<2> targetHeading = _drive.GetPose().headingUnit;
        float commandedSpeedMps = 0.0f;
        float traveledM = 0.0f;
        const unsigned long timeoutMs = millis() + static_cast<unsigned long>(2500.0f + (6000.0f * distanceM));
        EncoderProgressWatchdog translationWatchdog{};
        translationWatchdog.Reset(0.0f, millis());
        StraightPhaseMetrics metrics{};

        while (true)
        {
            float dtSeconds = 0.0f;
            uint32_t timestampUs = 0UL;
            DiagnosticSensorSnapshot snapshot{};
            if (!TickControl(false, dtSeconds, timestampUs, snapshot))
            {
                return false;
            }

            traveledM = std::fabs(_drive.GetAverageDistanceMeters() - startDistanceM);
            const float remainingM = (std::max)(0.0f, distanceM - traveledM);
            metrics.peakSpeedMps = (std::max)(metrics.peakSpeedMps, std::fabs(_drive.GetPose().linearSpeedMps));
            if ((remainingM <= Config::kDistanceToleranceM) && (std::fabs(_drive.GetPose().linearSpeedMps) <= Config::kSpeedToleranceMps))
            {
                _drive.Brake();
                if (!LogSample(false, timestampUs, dtSeconds, snapshot))
                {
                    return false;
                }
                break;
            }
            if (translationWatchdog.Stalled(traveledM, commandedSpeedMps, remainingM, millis()))
            {
                _drive.Brake();
                return Fail("Straight diagnostic encoder progress stalled");
            }
            if (static_cast<long>(timeoutMs - millis()) <= 0)
            {
                return Fail("Straight diagnostic phase timed out");
            }

            const float accelLimitedSpeedMps = (std::min)(limits.maxSpeedMps, commandedSpeedMps + (limits.accelMps2 * dtSeconds));
            const float decelLimitedSpeedMps = ReachableSpeedWithBoundary(0.0f, remainingM, limits.decelMps2);
            commandedSpeedMps = (std::min)(accelLimitedSpeedMps, decelLimitedSpeedMps);

            const float headingErrorRad = HeadingErrorRad(targetHeading, _drive.GetPose().headingUnit);
            metrics.maxHeadingErrorRad = (std::max)(metrics.maxHeadingErrorRad, std::fabs(headingErrorRad));
            float angularCommandRadps = (Config::kStraightHeadingKp * headingErrorRad) - (Config::kStraightYawD * _drive.GetPose().angularSpeedRadps);
            angularCommandRadps = (std::clamp)(angularCommandRadps, -limits.maxAngularSpeedRadps, limits.maxAngularSpeedRadps);
            _drive.CommandVelocity(commandedSpeedMps, angularCommandRadps, dtSeconds);

            if (!LogSample(false, timestampUs, dtSeconds, snapshot))
            {
                return false;
            }
        }

        if (outTraveledDistanceM != nullptr)
        {
            *outTraveledDistanceM = traveledM;
        }

        return WriteStraightResult(phaseName, distanceM, cruiseSpeedMps, traveledM, targetHeading, metrics);
    }

    bool RecoverCharacterizationSample(const char* label, float traveledDistanceM)
    {
        char phaseName[48] = {};
        if (traveledDistanceM <= DiagnosticConfig::kKickoffSweepMoveThresholdM)
        {
            snprintf(phaseName, sizeof(phaseName), "%s_settle", label);
            return HoldPhase(phaseName, DiagnosticConfig::kCharacterizationSettleMs, true);
        }

        snprintf(phaseName, sizeof(phaseName), "%s_turnaround", label);
        if (!ExecuteTurnPhase(phaseName, PI))
        {
            return false;
        }

        // Recover characterization samples with the same forward-drive path used elsewhere in diagnostics.
        // This avoids the poorly controlled reverse leg that can drift far past the available space.
        snprintf(phaseName, sizeof(phaseName), "%s_return", label);
        if (!ExecuteStraightPhase(phaseName, traveledDistanceM, DiagnosticConfig::kCharacterizationRecoverySpeedMps))
        {
            return false;
        }

        snprintf(phaseName, sizeof(phaseName), "%s_reset_heading", label);
        if (!ExecuteTurnPhase(phaseName, PI))
        {
            return false;
        }

        const PoseEstimate& pose = _drive.GetPose();
        _drive.SetPose(pose.xMeters, pose.yMeters, DirectionToYawRad(MazeMap::Up));
        _lastControlMicros = micros();

        snprintf(phaseName, sizeof(phaseName), "%s_settle", label);
        return HoldPhase(phaseName, DiagnosticConfig::kCharacterizationSettleMs, true);
    }

    bool ExecuteKickoffCharacterizationSample(float driveCommand)
    {
        char label[24] = {};
        char phaseName[48] = {};
        BuildDriveCommandLabel("kickoff", driveCommand, label, sizeof(label));
        snprintf(phaseName, sizeof(phaseName), "%s_probe", label);
        if (!StartPhase(phaseName))
        {
            return false;
        }

        const float startDistanceM = _drive.GetAverageDistanceMeters();
        const unsigned long pulseDeadlineMs = millis() + DiagnosticConfig::kKickoffSweepPulseMs;
        const unsigned long settleDeadlineMs = pulseDeadlineMs + DiagnosticConfig::kCharacterizationSettleMs;
        const float travelLimitM = MazeMap::ComputeDiagnosticCharacterizationTravelLimitM(
            DiagnosticConfig::kBoundaryHalfSpanM,
            DiagnosticConfig::kCharacterizationBoundaryReserveM);
        float maxSpeedMps = 0.0f;
        bool travelLimited = false;
        unsigned long travelLimitSettleDeadlineMs = 0UL;

        while (true)
        {
            float dtSeconds = 0.0f;
            uint32_t timestampUs = 0UL;
            DiagnosticSensorSnapshot snapshot{};
            if (!TickControl(false, dtSeconds, timestampUs, snapshot))
            {
                return false;
            }

            const unsigned long nowMs = millis();
            const float traveledDistanceM = std::fabs(_drive.GetAverageDistanceMeters() - startDistanceM);
            if (!travelLimited && travelLimitM > 0.0f && traveledDistanceM >= travelLimitM)
            {
                travelLimited = true;
                travelLimitSettleDeadlineMs = nowMs + DiagnosticConfig::kCharacterizationSettleMs;
            }

            const bool pulseActive = !travelLimited && static_cast<long>(pulseDeadlineMs - nowMs) > 0;
            if (travelLimited)
            {
                _drive.Brake();
            }
            else if (pulseActive)
            {
                _drive.CommandOpenLoopRaw(driveCommand, driveCommand);
            }
            else
            {
                _drive.Brake();
            }

            maxSpeedMps = (std::max)(maxSpeedMps, std::fabs(_drive.GetPose().linearSpeedMps));
            if (!LogSample(false, timestampUs, dtSeconds, snapshot))
            {
                return false;
            }

            if (travelLimited &&
                static_cast<long>(travelLimitSettleDeadlineMs - nowMs) <= 0 &&
                (std::fabs(_drive.GetPose().linearSpeedMps) <= Config::kSpeedToleranceMps))
            {
                break;
            }

            if (!travelLimited &&
                !pulseActive &&
                static_cast<long>(settleDeadlineMs - nowMs) <= 0 &&
                (std::fabs(_drive.GetPose().linearSpeedMps) <= Config::kSpeedToleranceMps))
            {
                break;
            }
        }

        _drive.Brake();
        const float traveledDistanceM = std::fabs(_drive.GetAverageDistanceMeters() - startDistanceM);
        const bool moved =
            (traveledDistanceM >= DiagnosticConfig::kKickoffSweepMoveThresholdM) ||
            (maxSpeedMps >= DiagnosticConfig::kKickoffSweepMoveThresholdMps);

        char message[192] = {};
        const int length = snprintf(
            message,
            sizeof(message),
            "%s;cmd=%.2f;dist_m=%.4f;max_speed_mps=%.3f;moved=%u;travel_limited=%u",
            label,
            driveCommand,
            traveledDistanceM,
            maxSpeedMps,
            moved ? 1U : 0U,
            travelLimited ? 1U : 0U);
        if (length <= 0 || length >= static_cast<int>(sizeof(message)))
        {
            return Fail("Failed to format kickoff characterization result");
        }
        if (!WriteEventOrFail("kickoff_result", message, "Failed to write kickoff characterization result"))
        {
            return false;
        }

        return RecoverCharacterizationSample(label, traveledDistanceM);
    }

    bool ExecuteForwardCharacterizationSample(float forwardDriveCommand)
    {
        char label[24] = {};
        char phaseName[48] = {};
        BuildDriveCommandLabel("forward", forwardDriveCommand, label, sizeof(label));
        snprintf(phaseName, sizeof(phaseName), "%s_probe", label);
        if (!StartPhase(phaseName))
        {
            return false;
        }

        const float startDistanceM = _drive.GetAverageDistanceMeters();
        const unsigned long kickoffDeadlineMs = millis() + DiagnosticConfig::kForwardSweepKickoffMs;
        const unsigned long holdDeadlineMs = kickoffDeadlineMs + DiagnosticConfig::kForwardSweepHoldMs;
        const unsigned long settleDeadlineMs = holdDeadlineMs + DiagnosticConfig::kCharacterizationSettleMs;
        const float travelLimitM = MazeMap::ComputeDiagnosticCharacterizationTravelLimitM(
            DiagnosticConfig::kBoundaryHalfSpanM,
            DiagnosticConfig::kCharacterizationBoundaryReserveM);
        float maxSpeedMps = 0.0f;
        float holdStartDistanceM = 0.0f;
        float holdEndDistanceM = 0.0f;
        float holdElapsedSeconds = 0.0f;
        bool holdStarted = false;
        bool holdComplete = false;
        bool travelLimited = false;
        unsigned long travelLimitSettleDeadlineMs = 0UL;

        while (true)
        {
            float dtSeconds = 0.0f;
            uint32_t timestampUs = 0UL;
            DiagnosticSensorSnapshot snapshot{};
            if (!TickControl(false, dtSeconds, timestampUs, snapshot))
            {
                return false;
            }

            const unsigned long nowMs = millis();
            const float traveledDistanceM = std::fabs(_drive.GetAverageDistanceMeters() - startDistanceM);
            if (!travelLimited && travelLimitM > 0.0f && traveledDistanceM >= travelLimitM)
            {
                travelLimited = true;
                travelLimitSettleDeadlineMs = nowMs + DiagnosticConfig::kCharacterizationSettleMs;
                if (holdStarted && !holdComplete)
                {
                    holdComplete = true;
                    holdEndDistanceM = _drive.GetAverageDistanceMeters();
                }
            }

            if (travelLimited)
            {
                _drive.Brake();
            }
            else if (static_cast<long>(kickoffDeadlineMs - nowMs) > 0)
            {
                _drive.CommandOpenLoopRaw(
                    DiagnosticConfig::kForwardSweepKickoffDriveCommand,
                    DiagnosticConfig::kForwardSweepKickoffDriveCommand);
            }
            else if (static_cast<long>(holdDeadlineMs - nowMs) > 0)
            {
                if (!holdStarted)
                {
                    holdStarted = true;
                    holdStartDistanceM = _drive.GetAverageDistanceMeters();
                }
                holdElapsedSeconds += dtSeconds;
                _drive.CommandOpenLoopRaw(forwardDriveCommand, forwardDriveCommand);
            }
            else
            {
                if (!holdComplete)
                {
                    holdComplete = true;
                    holdEndDistanceM = _drive.GetAverageDistanceMeters();
                }
                _drive.Brake();
            }

            maxSpeedMps = (std::max)(maxSpeedMps, std::fabs(_drive.GetPose().linearSpeedMps));
            if (!LogSample(false, timestampUs, dtSeconds, snapshot))
            {
                return false;
            }

            if (travelLimited &&
                static_cast<long>(travelLimitSettleDeadlineMs - nowMs) <= 0 &&
                (std::fabs(_drive.GetPose().linearSpeedMps) <= Config::kSpeedToleranceMps))
            {
                break;
            }

            if (!travelLimited &&
                holdComplete &&
                static_cast<long>(settleDeadlineMs - nowMs) <= 0 &&
                (std::fabs(_drive.GetPose().linearSpeedMps) <= Config::kSpeedToleranceMps))
            {
                break;
            }
        }

        _drive.Brake();
        if (!holdComplete)
        {
            holdEndDistanceM = _drive.GetAverageDistanceMeters();
        }

        const float totalDistanceM = std::fabs(_drive.GetAverageDistanceMeters() - startDistanceM);
        const float holdDistanceM = holdStarted ? std::fabs(holdEndDistanceM - holdStartDistanceM) : 0.0f;
        const float averageHoldSpeedMps = (holdElapsedSeconds > 0.0f) ? (holdDistanceM / holdElapsedSeconds) : 0.0f;
        const bool carried =
            (averageHoldSpeedMps >= DiagnosticConfig::kForwardSweepCarryThresholdMps) ||
            (holdDistanceM >= DiagnosticConfig::kForwardSweepCarryThresholdM);

        char message[224] = {};
        const int length = snprintf(
            message,
            sizeof(message),
            "%s;kickoff=%.2f;hold=%.2f;hold_dist_m=%.4f;hold_avg_speed_mps=%.3f;total_dist_m=%.4f;max_speed_mps=%.3f;carried=%u;travel_limited=%u",
            label,
            DiagnosticConfig::kForwardSweepKickoffDriveCommand,
            forwardDriveCommand,
            holdDistanceM,
            averageHoldSpeedMps,
            totalDistanceM,
            maxSpeedMps,
            carried ? 1U : 0U,
            travelLimited ? 1U : 0U);
        if (length <= 0 || length >= static_cast<int>(sizeof(message)))
        {
            return Fail("Failed to format forward characterization result");
        }
        if (!WriteEventOrFail("forward_result", message, "Failed to write forward characterization result"))
        {
            return false;
        }

        return RecoverCharacterizationSample(label, totalDistanceM);
    }

    bool ExecuteKickoffSweep()
    {
        if (!HoldPhase("kickoff_sweep_prep", DiagnosticConfig::kCharacterizationSettleMs, true))
        {
            return false;
        }

        for (float driveCommand = DiagnosticConfig::kKickoffSweepMinDriveCommand;
            driveCommand <= (DiagnosticConfig::kKickoffSweepMaxDriveCommand + 0.0001f);
            driveCommand += DiagnosticConfig::kKickoffSweepStepDriveCommand)
        {
            if (!ExecuteKickoffCharacterizationSample(driveCommand))
            {
                return false;
            }
        }

        return true;
    }

    bool ExecuteForwardSweep()
    {
        if (!HoldPhase("forward_sweep_prep", DiagnosticConfig::kCharacterizationSettleMs, true))
        {
            return false;
        }

        for (float driveCommand = DiagnosticConfig::kForwardSweepMinDriveCommand;
            driveCommand <= (DiagnosticConfig::kForwardSweepMaxDriveCommand + 0.0001f);
            driveCommand += DiagnosticConfig::kForwardSweepStepDriveCommand)
        {
            if (!ExecuteForwardCharacterizationSample(driveCommand))
            {
                return false;
            }
        }

        return true;
    }

    bool ExecuteTurnPhase(const char* phaseName, float angleRad)
    {
        if (!StartPhase(phaseName))
        {
            return false;
        }

        const float targetYawRad = WrapAngleRad(_drive.GetPose().yawRad + angleRad);
        const MazeMap::InPlaceTurnProfile turnProfile = BuildSharedInPlaceTurnProfile(_vehicle);
        float commandedOmegaRadps = 0.0f;
        const unsigned long timeoutMs = millis() + 3000UL;
        TurnPhaseMetrics metrics{};

        while (true)
        {
            float dtSeconds = 0.0f;
            uint32_t timestampUs = 0UL;
            DiagnosticSensorSnapshot snapshot{};
            if (!TickControl(false, dtSeconds, timestampUs, snapshot))
            {
                return false;
            }

            const float errorRad = AngleErrorRad(targetYawRad, _drive.GetPose().yawRad);
            const float remainingRad = std::fabs(errorRad);
            metrics.maxYawErrorRad = (std::max)(metrics.maxYawErrorRad, remainingRad);
            metrics.peakOmegaRadps = (std::max)(metrics.peakOmegaRadps, std::fabs(_drive.GetPose().angularSpeedRadps));
            if (MazeMap::IsInPlaceTurnComplete(errorRad, _drive.GetPose().angularSpeedRadps, turnProfile))
            {
                _drive.Brake();
                if (!LogSample(false, timestampUs, dtSeconds, snapshot))
                {
                    return false;
                }
                break;
            }
            if (static_cast<long>(timeoutMs - millis()) <= 0)
            {
                return Fail("Turn diagnostic phase timed out");
            }

            float angularCommandRadps = 0.0f;
            if (!MazeMap::TryComputeInPlaceTurnCommandRadps(
                    errorRad,
                    _drive.GetPose().angularSpeedRadps,
                    dtSeconds,
                    turnProfile,
                    commandedOmegaRadps,
                    angularCommandRadps))
            {
                return Fail("Turn diagnostic phase profile became invalid");
            }
            _drive.CommandVelocity(0.0f, angularCommandRadps, dtSeconds);

            if (!LogSample(false, timestampUs, dtSeconds, snapshot))
            {
                return false;
            }
        }

        return WriteTurnResult(phaseName, angleRad, metrics, targetYawRad);
    }

    bool ExecuteArcPhase(const char* phaseName, float distanceM, float angleRad, float cruiseSpeedMps, ArcPhaseMetrics* outMetrics = nullptr)
    {
        if (distanceM <= 0.0f)
        {
            return Fail("Diagnostic arc distance must be positive");
        }
        if (!StartPhase(phaseName))
        {
            return false;
        }

        const MotionLimits limits = DiagnosticLimits(cruiseSpeedMps);
        const float startDistanceM = _drive.GetAverageDistanceMeters();
        const float startYawRad = _drive.GetPose().yawRad;
        const float targetYawRad = WrapAngleRad(startYawRad + angleRad);
        const float curvature = angleRad / distanceM;
        float commandedSpeedMps = 0.0f;
        float traveledM = 0.0f;
        const unsigned long timeoutMs = millis() + static_cast<unsigned long>(2500.0f + (5000.0f * distanceM));
        EncoderProgressWatchdog translationWatchdog{};
        translationWatchdog.Reset(0.0f, millis());
        ArcPhaseMetrics metrics{};

        while (true)
        {
            float dtSeconds = 0.0f;
            uint32_t timestampUs = 0UL;
            DiagnosticSensorSnapshot snapshot{};
            if (!TickControl(false, dtSeconds, timestampUs, snapshot))
            {
                return false;
            }

            traveledM = std::fabs(_drive.GetAverageDistanceMeters() - startDistanceM);
            const float remainingM = (std::max)(0.0f, distanceM - traveledM);
            metrics.peakSpeedMps = (std::max)(metrics.peakSpeedMps, std::fabs(_drive.GetPose().linearSpeedMps));
            metrics.peakOmegaRadps = (std::max)(metrics.peakOmegaRadps, std::fabs(_drive.GetPose().angularSpeedRadps));
            metrics.durationSeconds += dtSeconds;
            metrics.omegaIntegralRad += _drive.GetPose().angularSpeedRadps * dtSeconds;
            metrics.speedIntegralMpsSeconds += std::fabs(_drive.GetPose().linearSpeedMps) * dtSeconds;
            const float planarAccelMps2 = _sensors.GetPlanarAccelMps2(snapshot);
            metrics.planarAccelIntegralMps2Seconds += planarAccelMps2 * dtSeconds;
            metrics.peakPlanarAccelMps2 = (std::max)(metrics.peakPlanarAccelMps2, planarAccelMps2);
            if ((remainingM <= Config::kDistanceToleranceM) && (std::fabs(_drive.GetPose().linearSpeedMps) <= Config::kSpeedToleranceMps))
            {
                _drive.Brake();
                if (!LogSample(false, timestampUs, dtSeconds, snapshot))
                {
                    return false;
                }
                break;
            }
            if (translationWatchdog.Stalled(traveledM, commandedSpeedMps, remainingM, millis()))
            {
                _drive.Brake();
                return Fail("Arc diagnostic encoder progress stalled");
            }
            if (static_cast<long>(timeoutMs - millis()) <= 0)
            {
                return Fail("Arc diagnostic phase timed out");
            }

            const float accelLimitedSpeedMps = (std::min)(cruiseSpeedMps, commandedSpeedMps + (limits.accelMps2 * dtSeconds));
            const float decelLimitedSpeedMps = ReachableSpeedWithBoundary(0.0f, remainingM, limits.decelMps2);
            commandedSpeedMps = (std::min)(accelLimitedSpeedMps, decelLimitedSpeedMps);

            const float progress = (std::clamp)(traveledM / distanceM, 0.0f, 1.0f);
            const float phaseTargetYawRad = WrapAngleRad(startYawRad + (angleRad * progress));
            const float headingErrorRad = AngleErrorRad(phaseTargetYawRad, _drive.GetPose().yawRad);
            metrics.maxHeadingErrorRad = (std::max)(metrics.maxHeadingErrorRad, std::fabs(headingErrorRad));
            float angularCommandRadps = (curvature * commandedSpeedMps) + (Config::kArcHeadingKp * headingErrorRad) - (Config::kArcYawD * _drive.GetPose().angularSpeedRadps);
            angularCommandRadps = (std::clamp)(angularCommandRadps, -limits.maxAngularSpeedRadps, limits.maxAngularSpeedRadps);
            _drive.CommandVelocity(commandedSpeedMps, angularCommandRadps, dtSeconds);

            if (!LogSample(false, timestampUs, dtSeconds, snapshot))
            {
                return false;
            }
        }

        if (outMetrics != nullptr)
        {
            *outMetrics = metrics;
        }
        return WriteArcResult(phaseName, distanceM, angleRad, traveledM, targetYawRad, metrics);
    }

    bool ExecuteArcCircle(const char* namePrefix, float halfCircleAngleRad, float halfCircleDistanceM, float cruiseSpeedMps)
    {
        char phaseName[48] = {};
        const PoseEstimate startPose = _drive.GetPose();
        const DriveTelemetry startTelemetry = _drive.GetTelemetry();
        ArcPhaseMetrics totalMetrics{};
        ArcPhaseMetrics segmentMetrics{};

        snprintf(phaseName, sizeof(phaseName), "%s_half_1", (namePrefix != nullptr) ? namePrefix : "arc_circle");
        if (!ExecuteArcPhase(phaseName, halfCircleDistanceM, halfCircleAngleRad, cruiseSpeedMps, &segmentMetrics))
        {
            return false;
        }
        AccumulateArcMetrics(totalMetrics, segmentMetrics);

        snprintf(phaseName, sizeof(phaseName), "%s_half_2", (namePrefix != nullptr) ? namePrefix : "arc_circle");
        if (!ExecuteArcPhase(phaseName, halfCircleDistanceM, halfCircleAngleRad, cruiseSpeedMps, &segmentMetrics))
        {
            return false;
        }
        AccumulateArcMetrics(totalMetrics, segmentMetrics);

        if (!WriteCircleResult(namePrefix, cruiseSpeedMps, startTelemetry, totalMetrics))
        {
            return false;
        }

        return WriteClosureResult("arc_circle_result", namePrefix, startPose, "Failed to write arc circle diagnostic result");
    }

    bool ExecuteCircleSpeedSweep(const char* speedLabel, float cruiseSpeedMps)
    {
        char phaseName[48] = {};

        snprintf(phaseName, sizeof(phaseName), "circle_cw_%s", (speedLabel != nullptr) ? speedLabel : "speed");
        if (!ExecuteArcCircle(phaseName, -PI, DiagnosticConfig::kArcHalfCircleDistanceM, cruiseSpeedMps))
        {
            return false;
        }

        snprintf(phaseName, sizeof(phaseName), "circle_cw_%s_settle", (speedLabel != nullptr) ? speedLabel : "speed");
        if (!HoldPhase(phaseName, DiagnosticConfig::kInterTestHoldMs, true))
        {
            return false;
        }

        snprintf(phaseName, sizeof(phaseName), "circle_ccw_%s", (speedLabel != nullptr) ? speedLabel : "speed");
        if (!ExecuteArcCircle(phaseName, PI, DiagnosticConfig::kArcHalfCircleDistanceM, cruiseSpeedMps))
        {
            return false;
        }

        snprintf(phaseName, sizeof(phaseName), "circle_%s_settle", (speedLabel != nullptr) ? speedLabel : "speed");
        return HoldPhase(phaseName, DiagnosticConfig::kInterTestHoldMs, true);
    }

    bool ExecuteSquareLoop(const char* namePrefix, float turnAngleRad)
    {
        char phaseName[48] = {};
        const PoseEstimate startPose = _drive.GetPose();
        for (uint8_t leg = 0; leg < 4U; ++leg)
        {
            snprintf(phaseName, sizeof(phaseName), "%s_leg_%u", namePrefix, static_cast<unsigned>(leg + 1U));
            if (!ExecuteStraightPhase(phaseName, DiagnosticConfig::kSquareLegDistanceM, DiagnosticConfig::kSlowStraightSpeedMps))
            {
                return false;
            }

            snprintf(phaseName, sizeof(phaseName), "%s_turn_%u", namePrefix, static_cast<unsigned>(leg + 1U));
            if (!ExecuteTurnPhase(phaseName, turnAngleRad))
            {
                return false;
            }
        }

        return WriteClosureResult("square_result", namePrefix, startPose, "Failed to write square diagnostic result");
    }
};

#if 0
class ManeuverFileTestController
{
public:
    ManeuverFileTestController()
        : _vehicle()
        , _sensors(_vehicle)
        , _drive()
        , _logger()
        , _path()
        , _queue()
        , _expectedDirectionalLocation(MazeMap::MazeLocation::CellCenter(MazeMap::CellCoordinates(0, 0)), MazeMap::Up)
        , _faulted(false)
        , _lastControlMicros(0UL)
    {
    }

    bool Begin()
    {
        Serial.begin(115200);
        delay(1000);
        Serial.println("Micromouse maneuver file test setup");

        if (!SetupHardware())
        {
            return Fail("Hardware setup failed");
        }
        if (!_drive.Begin())
        {
            return Fail("Drive base init failed");
        }
        SetRacingFanEnabled(false);
        if (!_sensors.Begin(Config::kControlPeriodUs))
        {
            return Fail("Sensor init failed");
        }
        if (!_logger.Begin(_sensors, "maneuver_test.csv", Config::kControlPeriodUs, "maneuver_test"))
        {
            Serial.println("MANEUVER FILE LOGGING ISSUE: maneuver_test.csv unavailable");
        }

        _expectedDirectionalLocation = MazeMap::DirectionalLocation(MazeMap::MazeLocation::CellCenter(MazeMap::CellCoordinates(0, 0)), MazeMap::Up);
        _drive.SnapTo(_expectedDirectionalLocation);
        _lastControlMicros = micros();

        if (!_logger.WriteEvent("source", "test.txt"))
        {
            Serial.println("MANEUVER FILE LOGGING ISSUE: Source metadata write failed");
        }

        return LoadQueueFromSdFile("test.txt");
    }

    void Run()
    {
        if (_faulted)
        {
            return;
        }

        bool ok = true;
        ok = ok && HoldPhase("startup_settle", Config::kObservationSettleMs, true);
        ok = ok && ExecuteQueue();
        ok = ok && HoldPhase("final_hold", Config::kObservationSettleMs, true);

        _drive.Brake();
        _logger.Flush();

        if (ok)
        {
            Serial.println("Maneuver file test complete");
        }

        _logger.Close();
    }

private:
    MazeMap::Vehicle _vehicle;
    DiagnosticSensorSuite _sensors;
    DriveBase _drive;
    DiagnosticLogger _logger;
    MazeMap::ManeuverPath _path;
    MazeMap::ManeuverQueue _queue;
    MazeMap::DirectionalLocation _expectedDirectionalLocation;
    bool _faulted;
    unsigned long _lastControlMicros;

    MotionLimits SpeedRunLimits() const
    {
        MotionLimits limits{};
        limits.maxSpeedMps = _vehicle.GetMaxSpeed() * Config::kSpeedRunScale;
        limits.accelMps2 = _vehicle.GetMaxForwardAcceleration() * Config::kSpeedRunScale;
        limits.decelMps2 = _vehicle.GetMaxForwardAcceleration() * Config::kSpeedRunScale;
        limits.maxAngularSpeedRadps = _vehicle.GetMaxRotationalVelocity() * Config::kSpeedRunScale;
        limits.angularAccelRadps2 = _vehicle.GetMaxAngularAcceleration() * Config::kSpeedRunScale;
        return limits;
    }

    static bool TryGetSmoothTurnExecutionProfileMeters(MazeMap::ManeuverCode code, MazeMap::SmoothTurnExecutionProfile& profile)
    {
        profile = MazeMap::SmoothTurnExecutionProfile{};
        if ((code == MazeMap::MC_NONE) || IsStraightCode(code))
        {
            return false;
        }

        MazeMap::SmoothTurnExecutionProfile profileInCells{};
        if (!MazeMap::ManeuverSet::GetSet()[code].TryGetSmoothTurnExecutionProfile(profileInCells))
        {
            return false;
        }

        profile = MazeMap::ScaleSmoothTurnExecutionProfile(profileInCells, Config::kCellSizeM);
        profile.radians = static_cast<float>(MazeMap::CodeDegrees(code)) * DEG_TO_RAD;
        return profile.IsValid();
    }

    static float ManeuverDistanceMeters(MazeMap::ManeuverCode code)
    {
        MazeMap::SmoothTurnExecutionProfile smoothTurnProfile{};
        if (TryGetSmoothTurnExecutionProfileMeters(code, smoothTurnProfile))
        {
            return smoothTurnProfile.totalDistance;
        }
        return 0.5f * Config::kCellSizeM * static_cast<float>(MazeMap::ManeuverSet::GetSet().DistanceTravelled(code));
    }

    float ManeuverSpeedLimit(MazeMap::ManeuverCode code) const
    {
        if (code == MazeMap::MC_NONE)
        {
            return 0.0f;
        }
        if (IsStraightCode(code))
        {
            return SpeedRunLimits().maxSpeedMps;
        }
        return (std::min)(SpeedRunLimits().maxSpeedMps, MazeMap::ManeuverSet::GetSet()[code].GetVMax(_vehicle) * Config::kSpeedRunScale);
    }

    bool Fail(const char* message)
    {
        _faulted = true;
        _drive.Brake();
        Serial.print("MANEUVER TEST FAULT: ");
        Serial.println(message);
        _logger.WriteEvent("fault", message);
        _logger.Flush();
        return false;
    }

    bool ReportManeuverFileIssue(const char* message)
    {
        _drive.Brake();
        Serial.print("MANEUVER FILE ISSUE: ");
        Serial.println((message != nullptr) ? message : "unknown");
        return false;
    }

    bool StartPhase(const char* name)
    {
        Serial.print("Maneuver test phase: ");
        Serial.println(name);
        if (_logger.BeginPhase(name))
        {
            return true;
        }
        return Fail("Failed to write maneuver test phase marker");
    }

    bool TickControl(bool stationary, float& dtSeconds, uint32_t& timestampUs, DiagnosticSensorSnapshot& snapshot)
    {
        while ((micros() - _lastControlMicros) < Config::kControlPeriodUs)
        {
            delayMicroseconds(50);
        }

        timestampUs = micros();
        dtSeconds = static_cast<float>(timestampUs - _lastControlMicros) * 1.0e-6f;
        _lastControlMicros = timestampUs;
        snapshot = _sensors.Capture(stationary, _drive.GetPose());
        _drive.UpdateOdometry(dtSeconds, snapshot.gyroRadps);
        return true;
    }

    bool LogSample(bool stationary, uint32_t timestampUs, float dtSeconds, const DiagnosticSensorSnapshot& snapshot)
    {
        const DriveTelemetry telemetry = _drive.GetTelemetry();
        const uint32_t dtUs = static_cast<uint32_t>(dtSeconds * 1.0e6f);
        if (_logger.LogSample(stationary, timestampUs, dtUs, _drive.GetPose(), _drive, telemetry, snapshot))
        {
            return true;
        }
        return Fail("Failed to write maneuver test sample");
    }

    bool HoldPhase(const char* phaseName, uint16_t durationMs, bool stationary)
    {
        if (!StartPhase(phaseName))
        {
            return false;
        }

        const unsigned long deadline = millis() + durationMs;
        while (static_cast<long>(deadline - millis()) > 0)
        {
            float dtSeconds = 0.0f;
            uint32_t timestampUs = 0UL;
            DiagnosticSensorSnapshot snapshot{};
            if (!TickControl(stationary, dtSeconds, timestampUs, snapshot))
            {
                return false;
            }

            _drive.Brake();
            if (!LogSample(stationary, timestampUs, dtSeconds, snapshot))
            {
                return false;
            }
        }

        return true;
    }

    bool LoadQueueFromSdFile(const char* fileName)
    {
#if defined(ARDUINO_TEENSY41)
        File file = SD.open(fileName, FILE_READ);
        if (!file)
        {
            return ReportManeuverFileIssue("Maneuver file unavailable");
        }

        _path.clear();
        _queue.clear();

        char line[128] = {};
        uint16_t lineNumber = 0U;
        while (file.available())
        {
            const size_t lineLength = file.readBytesUntil('\n', line, sizeof(line) - 1U);
            line[lineLength] = '\0';
            ++lineNumber;

            char* hashComment = strchr(line, '#');
            if (hashComment != nullptr)
            {
                *hashComment = '\0';
            }

            char* slashComment = strstr(line, "//");
            if (slashComment != nullptr)
            {
                *slashComment = '\0';
            }

            for (char* token = strtok(line, ", \t\r;"); token != nullptr; token = strtok(nullptr, ", \t\r;"))
            {
                MazeMap::ManeuverCode code = MazeMap::MC_NONE;
                if (!TryParseManeuverCodeToken(token, code))
                {
                    char errorMessage[96] = {};
                    snprintf(errorMessage, sizeof(errorMessage), "Maneuver file token issue on line %u: %s", lineNumber, token);
                    file.close();
                    return ReportManeuverFileIssue(errorMessage);
                }
                if (!_path.push_back(code))
                {
                    file.close();
                    return ReportManeuverFileIssue("Maneuver file exceeded path capacity");
                }
            }
        }

        file.close();

        if (_path.GetSize() == 0)
        {
            return ReportManeuverFileIssue("Maneuver file did not contain any maneuvers");
        }
        if (!_queue.push_back(_path, _expectedDirectionalLocation))
        {
            return ReportManeuverFileIssue("Unable to build maneuver queue from maneuver file");
        }

        _queue.ComputeSpeeds(_vehicle, 0.0f, 0.0f);
        ApplyAsymmetricQueueLimits(_queue, 0.0f, 0.0f);
        return LogQueueDefinition();
#else
        (void)fileName;
        return ReportManeuverFileIssue("Maneuver-file test mode requires the Teensy target");
#endif
    }

    bool LogQueueDefinition()
    {
        char message[128] = {};
        snprintf(message, sizeof(message), "count,%u", static_cast<unsigned>(_queue.size()));
        if (!_logger.WriteEvent("queue", message))
        {
            Serial.println("MANEUVER FILE LOGGING ISSUE: Queue size logging failed");
            return true;
        }

        for (uint16_t i = 0; i < _queue.size(); ++i)
        {
            char codeName[24] = {};
            char queueLine[160] = {};
            FormatManeuverCodeName(_queue[i].GetCode(), codeName, sizeof(codeName));
            snprintf(
                queueLine,
                sizeof(queueLine),
                "%u,%s,%.6f,%.6f",
                static_cast<unsigned>(i),
                codeName,
                _queue[i].GetEntrySpeed(),
                _queue[i].GetExitSpeed());

            if (!_logger.WriteEvent("queue_entry", queueLine))
            {
                Serial.println("MANEUVER FILE LOGGING ISSUE: Queue entry logging failed");
                return true;
            }
        }

        return true;
    }

    bool ExecuteQueue()
    {
        if (_queue.empty())
        {
            return ReportManeuverFileIssue("Maneuver queue is empty");
        }

        const MotionLimits limits = SpeedRunLimits();
        for (uint16_t i = 0; i < _queue.size(); ++i)
        {
            const MazeMap::ManeuverInstance& entry = _queue[i];
            const MazeMap::ManeuverCode code = entry.GetCode();
            const float entrySpeed = entry.GetEntrySpeed();
            const float exitSpeed = entry.GetExitSpeed();

            char codeName[24] = {};
            char phaseName[48] = {};
            FormatManeuverCodeName(code, codeName, sizeof(codeName));
            snprintf(phaseName, sizeof(phaseName), "maneuver_%u_%s", static_cast<unsigned>(i), codeName);

            bool ok = false;
            if (IsStraightCode(code))
            {
                ok = ExecuteStraightProfile(
                    phaseName,
                    0.5f * Config::kCellSizeM * static_cast<float>(static_cast<uint8_t>(code)),
                    entrySpeed,
                    limits.maxSpeedMps,
                    exitSpeed,
                    limits,
                    true);
            }
            else
            {
                const float angleRad = static_cast<float>(MazeMap::CodeDegrees(code)) * DEG_TO_RAD;
                MazeMap::SmoothTurnExecutionProfile smoothTurnProfile{};
                if (TryGetSmoothTurnExecutionProfileMeters(code, smoothTurnProfile))
                {
                    const float maneuverSpeedLimit = ManeuverSpeedLimit(code);
                    ok = ExecuteSmoothTurnProfile(phaseName, code, entrySpeed, exitSpeed, maneuverSpeedLimit, limits);
                }
                else
                {
                    const float distanceM = ManeuverDistanceMeters(code);
                    if (distanceM <= 0.0f)
                    {
                        ok = ExecuteTurnProfile(phaseName, angleRad, limits);
                    }
                    else
                    {
                        const float maneuverSpeedLimit = ManeuverSpeedLimit(code);
                        ok = ExecuteArcProfile(phaseName, distanceM, angleRad, entrySpeed, exitSpeed, maneuverSpeedLimit, limits);
                    }
                }
            }

            if (!ok)
            {
                return false;
            }

            _expectedDirectionalLocation = entry.GetEnd();
        }

        return true;
    }

    bool ExecuteStraightProfile(
        const char* phaseName,
        float distanceM,
        float entrySpeed,
        float cruiseSpeed,
        float exitSpeed,
        const MotionLimits& limits,
        bool useWallCentering)
    {
        if (!StartPhase(phaseName))
        {
            return false;
        }

        const float startDistanceM = _drive.GetAverageDistanceMeters();
        const MazeMap::Vectorf<2> targetHeading = _drive.GetPose().headingUnit;
        const bool diagonalHeading = IsApproximatelyDiagonalHeadingUnit(targetHeading);
        float commandedSpeedMps = (std::max)(entrySpeed, 0.0f);
        const unsigned long timeoutMs = millis() + static_cast<unsigned long>(2000.0f + (4000.0f * distanceM));
        EncoderProgressWatchdog translationWatchdog{};
        translationWatchdog.Reset(0.0f, millis());

        while (true)
        {
            float dtSeconds = 0.0f;
            uint32_t timestampUs = 0UL;
            DiagnosticSensorSnapshot snapshot{};
            if (!TickControl(false, dtSeconds, timestampUs, snapshot))
            {
                return false;
            }

            const float traveledM = std::fabs(_drive.GetAverageDistanceMeters() - startDistanceM);
            const float remainingM = (std::max)(0.0f, distanceM - traveledM);
            if ((remainingM <= Config::kDistanceToleranceM) && (std::fabs(_drive.GetPose().linearSpeedMps - exitSpeed) <= Config::kSpeedToleranceMps))
            {
                _drive.Brake();
                if (!LogSample(false, timestampUs, dtSeconds, snapshot))
                {
                    return false;
                }
                break;
            }
            if (translationWatchdog.Stalled(traveledM, commandedSpeedMps, remainingM, millis()))
            {
                _drive.Brake();
                return Fail("Maneuver straight encoder progress stalled");
            }
            if (static_cast<long>(timeoutMs - millis()) <= 0)
            {
                return Fail("Maneuver straight profile timed out");
            }

            const float accelLimitedSpeedMps = (std::min)(cruiseSpeed, commandedSpeedMps + (limits.accelMps2 * dtSeconds));
            const float decelLimitedSpeedMps = ReachableSpeedWithBoundary(exitSpeed, remainingM, limits.decelMps2);
            commandedSpeedMps = (std::min)(accelLimitedSpeedMps, decelLimitedSpeedMps);

            float wallOmegaRadps = 0.0f;
            if (useWallCentering)
            {
                if (diagonalHeading)
                {
                    wallOmegaRadps += ComputeDiagonalWallCenterOmegaRadps(
                        gWallDistanceCalibration,
                        snapshot.sideLeft.differentialLight,
                        snapshot.sideRight.differentialLight);
                }
                else
                {
                    float signalCorridorErrorM = 0.0f;
                    if (TryComputeStraightWallCenterErrorM(
                            gWallDistanceCalibration,
                            snapshot.sideLeft.differentialLight,
                            snapshot.leftWall,
                            snapshot.sideRight.differentialLight,
                            snapshot.rightWall,
                            signalCorridorErrorM))
                    {
                        wallOmegaRadps += Config::kWallCenterGain * signalCorridorErrorM;
                    }
                    else
                    {
                        wallOmegaRadps += Config::kWallCenterGain * snapshot.corridorErrorM;
                    }
                    if (snapshot.frontWall && remainingM < 0.07f)
                    {
                        wallOmegaRadps += Config::kFrontSkewGain * snapshot.frontSkewM;
                    }
                }
            }

            const float headingErrorRad = HeadingErrorRad(targetHeading, _drive.GetPose().headingUnit);
            float angularCommandRadps = (Config::kStraightHeadingKp * headingErrorRad) - (Config::kStraightYawD * _drive.GetPose().angularSpeedRadps) + wallOmegaRadps;
            angularCommandRadps = (std::clamp)(angularCommandRadps, -limits.maxAngularSpeedRadps, limits.maxAngularSpeedRadps);
            _drive.CommandVelocity(commandedSpeedMps, angularCommandRadps, dtSeconds);

            if (!LogSample(false, timestampUs, dtSeconds, snapshot))
            {
                return false;
            }
        }

        return true;
    }

    bool ExecuteTurnProfile(const char* phaseName, float angleRad, const MotionLimits& limits)
    {
        (void)limits;
        if (!StartPhase(phaseName))
        {
            return false;
        }

        const float targetYawRad = WrapAngleRad(_drive.GetPose().yawRad + angleRad);
        const MazeMap::InPlaceTurnProfile turnProfile = BuildSharedInPlaceTurnProfile(_vehicle);
        float commandedOmegaRadps = 0.0f;
        const unsigned long timeoutMs = millis() + 2500UL;

        while (true)
        {
            float dtSeconds = 0.0f;
            uint32_t timestampUs = 0UL;
            DiagnosticSensorSnapshot snapshot{};
            if (!TickControl(false, dtSeconds, timestampUs, snapshot))
            {
                return false;
            }

            const float errorRad = AngleErrorRad(targetYawRad, _drive.GetPose().yawRad);
            if (MazeMap::IsInPlaceTurnComplete(errorRad, _drive.GetPose().angularSpeedRadps, turnProfile))
            {
                _drive.Brake();
                if (!LogSample(false, timestampUs, dtSeconds, snapshot))
                {
                    return false;
                }
                break;
            }
            if (static_cast<long>(timeoutMs - millis()) <= 0)
            {
                return Fail("Maneuver turn profile timed out");
            }

            float angularCommandRadps = 0.0f;
            if (!MazeMap::TryComputeInPlaceTurnCommandRadps(
                    errorRad,
                    _drive.GetPose().angularSpeedRadps,
                    dtSeconds,
                    turnProfile,
                    commandedOmegaRadps,
                    angularCommandRadps))
            {
                return Fail("Maneuver turn profile became invalid");
            }
            _drive.CommandVelocity(0.0f, angularCommandRadps, dtSeconds);

            if (!LogSample(false, timestampUs, dtSeconds, snapshot))
            {
                return false;
            }
        }

        return true;
    }

    bool ExecuteArcProfile(
        const char* phaseName,
        float distanceM,
        float angleRad,
        float entrySpeed,
        float exitSpeed,
        float cruiseSpeed,
        const MotionLimits& limits)
    {
        if (!StartPhase(phaseName))
        {
            return false;
        }

        const float startDistanceM = _drive.GetAverageDistanceMeters();
        const float startYawRad = _drive.GetPose().yawRad;
        const float curvature = angleRad / distanceM;
        float commandedSpeedMps = (std::max)(entrySpeed, 0.0f);
        const unsigned long timeoutMs = millis() + static_cast<unsigned long>(2500.0f + (5000.0f * distanceM));
        EncoderProgressWatchdog translationWatchdog{};
        translationWatchdog.Reset(0.0f, millis());

        while (true)
        {
            float dtSeconds = 0.0f;
            uint32_t timestampUs = 0UL;
            DiagnosticSensorSnapshot snapshot{};
            if (!TickControl(false, dtSeconds, timestampUs, snapshot))
            {
                return false;
            }

            const float traveledM = std::fabs(_drive.GetAverageDistanceMeters() - startDistanceM);
            const float remainingM = (std::max)(0.0f, distanceM - traveledM);
            if ((remainingM <= Config::kDistanceToleranceM) && (std::fabs(_drive.GetPose().linearSpeedMps - exitSpeed) <= Config::kSpeedToleranceMps))
            {
                _drive.Brake();
                if (!LogSample(false, timestampUs, dtSeconds, snapshot))
                {
                    return false;
                }
                break;
            }
            if (translationWatchdog.Stalled(traveledM, commandedSpeedMps, remainingM, millis()))
            {
                _drive.Brake();
                return Fail("Maneuver arc encoder progress stalled");
            }
            if (static_cast<long>(timeoutMs - millis()) <= 0)
            {
                return Fail("Maneuver arc profile timed out");
            }

            const float accelLimitedSpeedMps = (std::min)(cruiseSpeed, commandedSpeedMps + (limits.accelMps2 * dtSeconds));
            const float decelLimitedSpeedMps = ReachableSpeedWithBoundary(exitSpeed, remainingM, limits.decelMps2);
            commandedSpeedMps = (std::min)(accelLimitedSpeedMps, decelLimitedSpeedMps);

            const float progress = (std::clamp)(traveledM / distanceM, 0.0f, 1.0f);
            const float targetYawRad = WrapAngleRad(startYawRad + (angleRad * progress));
            const float headingErrorRad = AngleErrorRad(targetYawRad, _drive.GetPose().yawRad);
            float angularCommandRadps = (curvature * commandedSpeedMps) + (Config::kArcHeadingKp * headingErrorRad) - (Config::kArcYawD * _drive.GetPose().angularSpeedRadps);
            angularCommandRadps = (std::clamp)(angularCommandRadps, -limits.maxAngularSpeedRadps, limits.maxAngularSpeedRadps);
            _drive.CommandVelocity(commandedSpeedMps, angularCommandRadps, dtSeconds);

            if (!LogSample(false, timestampUs, dtSeconds, snapshot))
            {
                return false;
            }
        }

        return true;
    }

    bool ExecuteSmoothTurnProfile(
        const char* phaseName,
        MazeMap::ManeuverCode code,
        float entrySpeed,
        float exitSpeed,
        float cruiseSpeed,
        const MotionLimits& limits)
    {
        if (!StartPhase(phaseName))
        {
            return false;
        }

        MazeMap::SmoothTurnExecutionProfile profile{};
        if (!TryGetSmoothTurnExecutionProfileMeters(code, profile))
        {
            return Fail("Maneuver smooth turn geometry is unavailable");
        }

        float maneuverSpeedMps = cruiseSpeed;
        if (!(maneuverSpeedMps > 0.0f))
        {
            maneuverSpeedMps = (std::max)(entrySpeed, exitSpeed);
        }
        if (!(maneuverSpeedMps > 0.0f))
        {
            return Fail("Maneuver smooth turn speed is invalid");
        }

        const float startDistanceM = _drive.GetAverageDistanceMeters();
        const float startYawRad = _drive.GetPose().yawRad;
        const bool stoppingAtEndpoint = exitSpeed <= 0.05f;
        const unsigned long timeoutMs = millis() + static_cast<unsigned long>(2500.0f + (5000.0f * profile.totalDistance));
        EncoderProgressWatchdog translationWatchdog{};
        translationWatchdog.Reset(0.0f, millis());

        while (true)
        {
            float dtSeconds = 0.0f;
            uint32_t timestampUs = 0UL;
            DiagnosticSensorSnapshot snapshot{};
            if (!TickControl(false, dtSeconds, timestampUs, snapshot))
            {
                return false;
            }

            const float traveledM = std::fabs(_drive.GetAverageDistanceMeters() - startDistanceM);
            const float remainingM = (std::max)(0.0f, profile.totalDistance - traveledM);
            if (remainingM <= Config::kDistanceToleranceM)
            {
                if (stoppingAtEndpoint)
                {
                    _drive.Brake();
                }
                if (!LogSample(false, timestampUs, dtSeconds, snapshot))
                {
                    return false;
                }
                break;
            }
            if (translationWatchdog.Stalled(traveledM, maneuverSpeedMps, remainingM, millis()))
            {
                _drive.Brake();
                return Fail("Maneuver smooth turn encoder progress stalled");
            }
            if (static_cast<long>(timeoutMs - millis()) <= 0)
            {
                return Fail("Maneuver smooth turn profile timed out");
            }

            float yawOffsetRad = 0.0f;
            float nominalOmegaRadps = 0.0f;
            if (!MazeMap::TryComputeSmoothTurnTarget(profile, traveledM, maneuverSpeedMps, yawOffsetRad, nominalOmegaRadps))
            {
                return Fail("Maneuver smooth turn target became invalid");
            }

            const float targetYawRad = WrapAngleRad(startYawRad + yawOffsetRad);
            const float headingErrorRad = AngleErrorRad(targetYawRad, _drive.GetPose().yawRad);
            float angularCommandRadps = nominalOmegaRadps + (Config::kArcHeadingKp * headingErrorRad) - (Config::kArcYawD * _drive.GetPose().angularSpeedRadps);
            angularCommandRadps = (std::clamp)(angularCommandRadps, -limits.maxAngularSpeedRadps, limits.maxAngularSpeedRadps);
            _drive.CommandVelocity(maneuverSpeedMps, angularCommandRadps, dtSeconds);

            if (!LogSample(false, timestampUs, dtSeconds, snapshot))
            {
                return false;
            }
        }

        return true;
    }

    void ApplyAsymmetricQueueLimits(MazeMap::ManeuverQueue& queue, float initialEntrySpeed, float finalExitSpeed)
    {
        if (queue.empty())
        {
            return;
        }

        float boundarySpeed = (std::max)(initialEntrySpeed, 0.0f);
        for (uint16_t i = 0; i < queue.size(); ++i)
        {
            MazeMap::ManeuverInstance& entry = queue[i];
            const float speedLimit = ManeuverSpeedLimit(entry.GetCode());
            if (IsStraightCode(entry.GetCode()))
            {
                const float distanceM = ManeuverDistanceMeters(entry.GetCode());
                const float entrySpeed = (std::min)(boundarySpeed, speedLimit);
                const float exitSpeed = (std::min)(entry.GetExitSpeed(), (std::min)(speedLimit, ReachableSpeedWithBoundary(entrySpeed, distanceM, SpeedRunLimits().accelMps2)));
                entry.SetEntrySpeed(entrySpeed);
                entry.SetExitSpeed(exitSpeed);
                boundarySpeed = exitSpeed;
            }
            else
            {
                const float maneuverSpeed = (std::min)((std::min)(entry.GetEntrySpeed(), boundarySpeed), speedLimit);
                entry.SetEntrySpeed(maneuverSpeed);
                entry.SetExitSpeed(maneuverSpeed);
                boundarySpeed = maneuverSpeed;
            }
        }

        float requiredExitSpeed = (std::max)(finalExitSpeed, 0.0f);
        for (int i = static_cast<int>(queue.size()) - 1; i >= 0; --i)
        {
            MazeMap::ManeuverInstance& entry = queue[static_cast<uint16_t>(i)];
            const float speedLimit = ManeuverSpeedLimit(entry.GetCode());
            if (IsStraightCode(entry.GetCode()))
            {
                const float distanceM = ManeuverDistanceMeters(entry.GetCode());
                const float exitSpeed = (std::min)(entry.GetExitSpeed(), (std::min)(requiredExitSpeed, speedLimit));
                const float entrySpeed = (std::min)(entry.GetEntrySpeed(), (std::min)(speedLimit, ReachableSpeedWithBoundary(exitSpeed, distanceM, SpeedRunLimits().decelMps2)));
                entry.SetEntrySpeed(entrySpeed);
                entry.SetExitSpeed(exitSpeed);
                requiredExitSpeed = entrySpeed;
            }
            else
            {
                const float maneuverSpeed = (std::min)(entry.GetEntrySpeed(), (std::min)(requiredExitSpeed, speedLimit));
                entry.SetEntrySpeed(maneuverSpeed);
                entry.SetExitSpeed(maneuverSpeed);
                requiredExitSpeed = maneuverSpeed;
            }
        }

        boundarySpeed = (std::max)(initialEntrySpeed, 0.0f);
        for (uint16_t i = 0; i < queue.size(); ++i)
        {
            MazeMap::ManeuverInstance& entry = queue[i];
            const float speedLimit = ManeuverSpeedLimit(entry.GetCode());
            if (IsStraightCode(entry.GetCode()))
            {
                const float distanceM = ManeuverDistanceMeters(entry.GetCode());
                const float entrySpeed = (std::min)(entry.GetEntrySpeed(), (std::min)(boundarySpeed, speedLimit));
                const float exitSpeed = (std::min)(entry.GetExitSpeed(), (std::min)(speedLimit, ReachableSpeedWithBoundary(entrySpeed, distanceM, SpeedRunLimits().accelMps2)));
                entry.SetEntrySpeed(entrySpeed);
                entry.SetExitSpeed(exitSpeed);
                boundarySpeed = exitSpeed;
            }
            else
            {
                const float maneuverSpeed = (std::min)(entry.GetEntrySpeed(), (std::min)(boundarySpeed, speedLimit));
                entry.SetEntrySpeed(maneuverSpeed);
                entry.SetExitSpeed(maneuverSpeed);
                boundarySpeed = maneuverSpeed;
            }
        }
    }
};
#endif

struct MissionPlannerCore
{
    MazeMap::Vehicle vehicle;
    MazeMap::Maze maze;
    MazeMap::FloodFillPathFinder searchPathFinder;
    MazeMap::ManeuverPathFinder speedPathFinder;

    MissionPlannerCore()
        : vehicle()
        , maze()
        , searchPathFinder(maze, vehicle)
        , speedPathFinder(maze, vehicle)
    {
    }
};

// Keep the large planner objects at file scope so controller instances do not duplicate them by default.
static MissionPlannerCore gMissionPlannerCore;

class MissionController
{
public:
    MissionController()
        : _speedVehicle(gMissionPlannerCore.vehicle)
        , _maze(gMissionPlannerCore.maze)
        , _searchPathFinder(gMissionPlannerCore.searchPathFinder)
        , _speedPathFinder(gMissionPlannerCore.speedPathFinder)
        , _sensors(_speedVehicle, gWallDistanceCalibration)
        , _telemetrySensors(_speedVehicle, gWallDistanceCalibration)
        , _drive()
        , _currentCell(0, 0)
        , _currentDirection(MazeMap::Up)
        , _currentDirectionalLocation(MazeMap::MazeLocation::CellCenter(MazeMap::CellCoordinates(0, 0)), MazeMap::Up)
        , _goalPauseComplete(false)
        , _missionComplete(false)
        , _faulted(false)
        , _maneuverTestMode(false)
        , _telemetryLoggingEnabled(false)
        , _missionTextLoggingEnabled(false)
        , _missionMazeSnapshotWritten(false)
        , _lastWallTouchStandoffEstimateM(0.0f)
        , _hasWallTouchStandoffEstimate(false)
        , _lastControlMicros(0UL)
    {
    }

    MissionController(const MissionController&) = delete;
    MissionController& operator=(const MissionController&) = delete;
    MissionController(MissionController&&) = delete;
    MissionController& operator=(MissionController&&) = delete;

    bool Begin()
    {
        _maneuverTestMode = false;
        _telemetryLoggingEnabled = false;
        _missionTextLoggingEnabled = true;
        _missionMazeSnapshotWritten = false;
        _hasWallTouchStandoffEstimate = false;
        if (!Initialize("Micromouse mission setup", false))
        {
            return false;
        }

        PrimeKnownMissionStartCell();
        AppendStartupTrace("initialize:seeded_known_start_cell");
        return true;
    }

    bool BeginManeuverFileTest()
    {
        _maneuverTestMode = true;
        _telemetryLoggingEnabled = false;
        _missionTextLoggingEnabled = false;
        _missionMazeSnapshotWritten = false;
        _missionTextLogFile.Close();
        _hasWallTouchStandoffEstimate = false;
        if (!Initialize("Micromouse maneuver test setup", false))
        {
            return false;
        }
        if (!_telemetrySensors.Begin())
        {
            return Fail("Telemetry sensor init failed");
        }
        if (!_telemetryLogger.Begin(_telemetrySensors, "maneuver_test.csv", Config::kControlPeriodUs, "maneuver_test"))
        {
            AppendStartupTrace("maneuver_test:telemetry_logger_open_failed");
            Serial.println("Maneuver test telemetry log unavailable; continuing without telemetry file");
            _telemetryLoggingEnabled = false;
            return true;
        }
        _telemetryLoggingEnabled = true;
        AppendStartupTrace("maneuver_test:telemetry_logger_opened");
        if (!_telemetryLogger.WriteEvent("source", "test.txt"))
        {
            AppendStartupTrace("maneuver_test:source_metadata_write_failed");
            Serial.println("Maneuver test source metadata write failed; disabling telemetry file logging");
            _telemetryLogger.Flush();
            _telemetryLogger.Close();
            _telemetryLoggingEnabled = false;
            return true;
        }
        AppendStartupTrace("maneuver_test:source_metadata_written");
        if (!LogWallCalibrationMetadata())
        {
            return false;
        }
        AppendStartupTrace("maneuver_test:wall_calibration_logged");
        return true;
    }

    bool BeginCorridorRepeatabilitySweep()
    {
        _maneuverTestMode = false;
        _telemetryLoggingEnabled = false;
        _missionTextLoggingEnabled = false;
        _missionMazeSnapshotWritten = false;
        _missionTextLogFile.Close();
        _goalPauseComplete = false;
        _missionComplete = false;
        _faulted = false;
        _hasWallTouchStandoffEstimate = false;
        if (!Initialize("Corridor repeatability setup", false))
        {
            return false;
        }

        PrimeKnownMissionStartCell();
        AppendStartupTrace("initialize:seeded_known_start_cell");
        if (!_telemetrySensors.Begin())
        {
            return Fail("Telemetry sensor init failed");
        }

        char fileName[32] = {};
        if (!SelectSequentialCsvFileName(fileName, sizeof(fileName), nullptr, "aux%03u.csv", "corridor_repeatability.csv"))
        {
            return Fail("Unable to choose corridor repeatability log file");
        }
        if (!_telemetryLogger.Begin(_telemetrySensors, fileName, Config::kControlPeriodUs, "corridor_repeatability"))
        {
            return Fail("Unable to open corridor repeatability log");
        }

        _telemetryLoggingEnabled = true;
        AppendStartupTrace("corridor_repeatability:telemetry_logger_opened");
        if (!LogWallCalibrationMetadata())
        {
            return false;
        }
        if (!LogCorridorRepeatabilityMetadata())
        {
            return false;
        }
        AppendStartupTrace("corridor_repeatability:metadata_written");
        return true;
    }

    bool BeginPositionAccuracyAudit()
    {
        _maneuverTestMode = false;
        _telemetryLoggingEnabled = false;
        _missionTextLoggingEnabled = false;
        _missionMazeSnapshotWritten = false;
        _missionTextLogFile.Close();
        _goalPauseComplete = false;
        _missionComplete = false;
        _faulted = false;
        _hasWallTouchStandoffEstimate = false;
        if (!Initialize("Position accuracy audit setup", false))
        {
            return false;
        }

        PrimeKnownMissionStartCell();
        AppendStartupTrace("initialize:seeded_known_start_cell");
        if (!_telemetrySensors.Begin())
        {
            return Fail("Telemetry sensor init failed");
        }

        float positionAuditGyroBiasAverageRadps = 0.0f;
        unsigned long positionAuditGyroBiasAverageSamples = 0UL;
        if (!TryAverageRawGyroBiasRadpsOverWindow(
                _speedVehicle,
                static_cast<unsigned long>(AuxMeasurementConfig::kPositionAuditGyroBiasAverageWindowMs),
                static_cast<unsigned long>(AuxMeasurementConfig::kPositionAuditGyroBiasAverageSampleIntervalMs),
                positionAuditGyroBiasAverageRadps,
                positionAuditGyroBiasAverageSamples))
        {
            return Fail("Unable to average position audit gyro bias");
        }

        char fileName[32] = {};
        if (!SelectSequentialCsvFileName(fileName, sizeof(fileName), nullptr, "aux%03u.csv", "position_accuracy_audit.csv"))
        {
            return Fail("Unable to choose position accuracy audit log file");
        }
        if (!_telemetryLogger.Begin(_telemetrySensors, fileName, Config::kControlPeriodUs, "position_accuracy_audit"))
        {
            return Fail("Unable to open position accuracy audit log");
        }

        _telemetryLoggingEnabled = true;
        AppendStartupTrace("position_accuracy_audit:telemetry_logger_opened");
        if (!_telemetryLogger.WriteMetadataUnsigned(
                "position_audit_gyro_bias_average_window_ms",
                static_cast<unsigned long>(AuxMeasurementConfig::kPositionAuditGyroBiasAverageWindowMs)))
        {
            return Fail("Unable to write position audit gyro bias metadata");
        }
        if (!_telemetryLogger.WriteMetadataUnsigned(
                "position_audit_gyro_bias_average_interval_ms",
                static_cast<unsigned long>(AuxMeasurementConfig::kPositionAuditGyroBiasAverageSampleIntervalMs)))
        {
            return Fail("Unable to write position audit gyro bias metadata");
        }
        if (!_telemetryLogger.WriteMetadataUnsigned(
                "position_audit_gyro_bias_average_samples",
                positionAuditGyroBiasAverageSamples))
        {
            return Fail("Unable to write position audit gyro bias metadata");
        }
        if (!_telemetryLogger.WriteMetadataValueFloat(
                "position_audit_gyro_bias_average_radps",
                positionAuditGyroBiasAverageRadps,
                6))
        {
            return Fail("Unable to write position audit gyro bias metadata");
        }
        if (!_telemetryLogger.WriteMetadataValueFloat(
                "position_audit_gyro_bias_delta_vs_mission_radps",
                positionAuditGyroBiasAverageRadps - _telemetrySensors.GetGyroBiasRadps(),
                6))
        {
            return Fail("Unable to write position audit gyro bias metadata");
        }
        if (!LogWallCalibrationMetadata())
        {
            return false;
        }
        const PositionAuditFixtureGeometry positionAuditGeometry = BuildPositionAuditFixtureGeometry();
        if (!LogPositionAccuracyAuditMetadata(positionAuditGeometry))
        {
            return false;
        }
        AppendStartupTrace("position_accuracy_audit:metadata_written");
        return true;
    }

    void CommandOpenLoop(const MazeMap::OpenLoopDriveCommand& command)
    {
        _drive.CommandOpenLoop(command);
    }

    void CommandOpenLoopRaw(const MazeMap::OpenLoopDriveCommand& command)
    {
        _drive.CommandOpenLoopRaw(command);
    }

    void Run()
    {
        if (_missionComplete || _faulted)
        {
            return;
        }

        if (!EmitMissionControllerLineOrFail("Exploration start"))
        {
            return;
        }
        if (!ExploreFullMaze())
        {
            return;
        }

        if (!EmitMissionControllerLineOrFail("Returning to start"))
        {
            return;
        }
        if (!ReturnToStart())
        {
            return;
        }

        _maze.PreCalculate();

        if (!EmitMissionControllerLineOrFail("Speed run 1 start"))
        {
            return;
        }
        if (!ExecuteRacingRunCycle())
        {
            return;
        }

        if (!HandleInterRunServiceCycle())
        {
            return;
        }

        if (!EmitMissionControllerLineOrFail("Speed run 2 start"))
        {
            return;
        }
        if (!ExecuteRacingRunCycle())
        {
            return;
        }

        (void)WriteMissionMazeSnapshot("mission_complete");

        SetRacingFanEnabled(false);
        _drive.Brake();
        _missionComplete = true;
        (void)EmitMissionControllerLine("Mission complete");
        CloseMissionTextLog();
    }

    void RunManeuverFileTest()
    {
        if (_faulted)
        {
            return;
        }

        AppendStartupTrace("maneuver_test:run_entered");
        MazeMap::ManeuverQueue queue;
        if (!LoadManeuverQueueFromSd("test.txt", queue))
        {
            return;
        }
        AppendStartupTrace("maneuver_test:queue_loaded");

        Serial.print("Loaded maneuver test queue with ");
        Serial.print(queue.size());
        Serial.println(" maneuvers");

        if (!HoldPosition(Config::kObservationSettleMs, "startup_settle"))
        {
            _telemetryLoggingEnabled = false;
            _telemetryLogger.Close();
            return;
        }

        if (!ExecuteQueuedManeuvers(queue, false))
        {
            _telemetryLoggingEnabled = false;
            _telemetryLogger.Close();
            return;
        }

        if (!HoldPosition(50, "final_hold"))
        {
            _telemetryLoggingEnabled = false;
            _telemetryLogger.Close();
            return;
        }
        _drive.Brake();
        _telemetryLogger.Flush();
        _telemetryLoggingEnabled = false;
        _telemetryLogger.Close();
        AppendStartupTrace("maneuver_test:complete");
        Serial.println("Maneuver file test complete");
    }

    void RunCorridorRepeatabilitySweep()
    {
        if (_faulted)
        {
            return;
        }

        AppendStartupTrace("corridor_repeatability:run_entered");
        const bool ok = RunCorridorRepeatabilityPasses();

        _drive.Brake();
        _telemetryLogger.Flush();
        _telemetryLoggingEnabled = false;
        _telemetryLogger.Close();
        if (ok)
        {
            AppendStartupTrace("corridor_repeatability:complete");
            Serial.println("Corridor repeatability sweep complete");
        }
    }

    void RunPositionAccuracyAudit()
    {
        if (_faulted)
        {
            return;
        }

        AppendStartupTrace("position_accuracy_audit:run_entered");
        const bool ok = RunPositionAccuracyAuditPasses();

        _drive.Brake();
        _telemetryLogger.Flush();
        _telemetryLoggingEnabled = false;
        _telemetryLogger.Close();
        if (ok)
        {
            AppendStartupTrace("position_accuracy_audit:complete");
            Serial.println("Position accuracy audit complete");
        }
    }

private:
    static void SetKnownMazeCellWalls(
        MazeMap::Maze& maze,
        const MazeMap::CellCoordinates& cellCoordinates,
        MazeMap::WallState up,
        MazeMap::WallState down,
        MazeMap::WallState left,
        MazeMap::WallState right)
    {
        MazeMap::Cell& cell = maze[cellCoordinates];
        maze.SetWall(cell, MazeMap::Up, up);
        maze.SetWall(cell, MazeMap::Down, down);
        maze.SetWall(cell, MazeMap::Left, left);
        maze.SetWall(cell, MazeMap::Right, right);
    }

    struct PositionAuditFixtureGeometry
    {
        MazeMap::Maze maze;
        uint8_t northCorridorCellCount = 0U;
        uint8_t eastExtensionCellCount = 0U;
        uint8_t eastTotalCellCount = 0U;
        float northCorridorSpanYM = 0.0f;
        float eastBranchSpanXM = 0.0f;
        float outDistanceM = 0.0f;
        float farCellCenterYM = 0.0f;
        float farWallTouchYM = 0.0f;
        float eastWallTouchXM = 0.0f;
    };

    static MazeMap::Maze BuildPositionAuditMazeFixture(uint8_t northCorridorCellCount, uint8_t eastExtensionCellCount)
    {
        MazeMap::Maze maze;

        for (uint8_t y = 0U; y < northCorridorCellCount; ++y)
        {
            const MazeMap::CellCoordinates cell(0U, y);
            SetKnownMazeCellWalls(
                maze,
                cell,
                (y + 1U < northCorridorCellCount) ? MazeMap::NoWall : MazeMap::Wall,
                (y > 0U) ? MazeMap::NoWall : MazeMap::Wall,
                MazeMap::Wall,
                (y + 1U == northCorridorCellCount) ? MazeMap::NoWall : MazeMap::Wall);
        }

        for (uint8_t x = 1U; x <= eastExtensionCellCount; ++x)
        {
            const MazeMap::CellCoordinates cell(
                x,
                static_cast<uint8_t>(northCorridorCellCount - 1U));
            SetKnownMazeCellWalls(
                maze,
                cell,
                MazeMap::Wall,
                MazeMap::Wall,
                MazeMap::NoWall,
                (x < eastExtensionCellCount) ? MazeMap::NoWall : MazeMap::Wall);
        }

        return maze;
    }

    static PositionAuditFixtureGeometry BuildPositionAuditFixtureGeometry()
    {
        PositionAuditFixtureGeometry geometry{};
        geometry.northCorridorCellCount = AuxMeasurementConfig::kPositionAuditNorthCorridorCellCount;
        geometry.eastExtensionCellCount = AuxMeasurementConfig::kPositionAuditEastBranchCellCount;
        geometry.eastTotalCellCount = static_cast<uint8_t>(geometry.eastExtensionCellCount + 1U);
        geometry.northCorridorSpanYM = Config::kCellSizeM * static_cast<float>(geometry.northCorridorCellCount);
        geometry.eastBranchSpanXM = Config::kCellSizeM * static_cast<float>(geometry.eastTotalCellCount);
        geometry.outDistanceM =
            Config::kCellSizeM *
            static_cast<float>(geometry.northCorridorCellCount - 1U);
        geometry.farCellCenterYM =
            (static_cast<float>(geometry.northCorridorCellCount) - 0.5f) *
            Config::kCellSizeM;
        geometry.farWallTouchYM = MazeMap::ComputeWallTouchPoseFromNorthWallM(
            geometry.northCorridorSpanYM,
            Config::kMazeWallThicknessM,
            Config::kWallTouchContactStandoffM);
        geometry.eastWallTouchXM = MazeMap::ComputeWallTouchPoseFromEastWallM(
            geometry.eastBranchSpanXM,
            Config::kMazeWallThicknessM,
            Config::kWallTouchContactStandoffM);
        geometry.maze = BuildPositionAuditMazeFixture(
            geometry.northCorridorCellCount,
            geometry.eastExtensionCellCount);
        return geometry;
    }

    static uint8_t CountClearForwardHalfStepsUntilBlocked(
        const MazeMap::Maze& maze,
        const MazeMap::DirectionalLocation& start,
        uint8_t maxHalfSteps = 31U)
    {
        MazeMap::DirectionalLocation cursor = start;
        uint8_t clearHalfSteps = 0U;
        while (clearHalfSteps < maxHalfSteps)
        {
            cursor = cursor.MoveForward(1U);
            if (!maze.IsAccessibleLocation(cursor.GetLocation()))
            {
                break;
            }

            ++clearHalfSteps;
        }

        return clearHalfSteps;
    }

    static bool TryResolvePositionAuditSmoothTurnLaunchLocation(
        const PositionAuditFixtureGeometry& geometry,
        MazeMap::ManeuverCode code,
        MazeMap::DirectionalLocation& launchLocation)
    {
        const uint8_t corridorCenterHalfX = 1U;
        const uint8_t maxHalfY = static_cast<uint8_t>(
            (geometry.northCorridorCellCount << 1U) - 1U);
        const MazeMap::Maze& auditMaze = geometry.maze;

        for (uint8_t halfY = maxHalfY; halfY > 0U; --halfY)
        {
            const MazeMap::DirectionalLocation candidate(
                MazeMap::MazeLocation(corridorCenterHalfX, halfY),
                MazeMap::Up);
            if (!auditMaze.IsAccessibleLocation(candidate.GetLocation()))
            {
                continue;
            }
            if (!MazeMap::ManeuverSet::GetSet().IsValidMove(code, candidate, auditMaze))
            {
                continue;
            }

            const MazeMap::DirectionalLocation maneuverEnd = MazeMap::ManeuverSet::GetSet().Move(code, candidate);
            if (!auditMaze.IsAccessibleLocation(maneuverEnd.GetLocation()))
            {
                continue;
            }
            if (CountClearForwardHalfStepsUntilBlocked(auditMaze, maneuverEnd) == 0U)
            {
                continue;
            }

            launchLocation = candidate;
            return true;
        }

        launchLocation = MazeMap::DirectionalLocation();
        return false;
    }

    static bool TryBuildReverseManeuverPath(
        const MazeMap::ManeuverPath& forwardPath,
        MazeMap::ManeuverPath& reversePath)
    {
        reversePath.clear();
        const MazeMap::ManeuverSet& maneuverSet = MazeMap::ManeuverSet::GetSet();
        for (int index = static_cast<int>(forwardPath.GetSize()) - 1; index >= 0; --index)
        {
            if (!reversePath.push_back(maneuverSet.GetReverseCode(forwardPath[static_cast<uint16_t>(index)])))
            {
                reversePath.clear();
                return false;
            }
        }

        return true;
    }

    static bool TryResolvePositionAuditSmoothTurnHalfSteps(
        MazeMap::ManeuverCode code,
        uint8_t& preTurnHalfSteps,
        uint8_t& postTurnHalfSteps)
    {
        switch (code)
        {
        case MazeMap::S90SS:
            preTurnHalfSteps = AuxMeasurementConfig::kPositionAuditPhase2PreTurnHalfSteps;
            postTurnHalfSteps = AuxMeasurementConfig::kPositionAuditPhase2PostTurnHalfSteps;
            return true;
        case MazeMap::S90LS:
            preTurnHalfSteps = AuxMeasurementConfig::kPositionAuditPhase3PreTurnHalfSteps;
            postTurnHalfSteps = AuxMeasurementConfig::kPositionAuditPhase3PostTurnHalfSteps;
            return true;
        default:
            preTurnHalfSteps = 0U;
            postTurnHalfSteps = 0U;
            return false;
        }
    }

    static bool TryBuildPositionAuditSmoothTurnPaths(
        MazeMap::ManeuverCode code,
        MazeMap::ManeuverPath& forwardPath,
        MazeMap::ManeuverPath& reversePath,
        uint8_t& preTurnHalfSteps,
        uint8_t& postTurnHalfSteps)
    {
        forwardPath.clear();
        reversePath.clear();
        if (!TryResolvePositionAuditSmoothTurnHalfSteps(code, preTurnHalfSteps, postTurnHalfSteps))
        {
            return false;
        }

        if (!forwardPath.push_back(static_cast<MazeMap::ManeuverCode>(preTurnHalfSteps)) ||
            !forwardPath.push_back(code) ||
            !forwardPath.push_back(static_cast<MazeMap::ManeuverCode>(postTurnHalfSteps)))
        {
            forwardPath.clear();
            return false;
        }

        return TryBuildReverseManeuverPath(forwardPath, reversePath);
    }

    static bool TryValidatePositionAuditPath(
        const MazeMap::Maze& maze,
        const MazeMap::ManeuverPath& path,
        MazeMap::DirectionalLocation start,
        MazeMap::DirectionalLocation& end)
    {
        MazeMap::DirectionalLocation current = start;
        const MazeMap::ManeuverSet& maneuverSet = MazeMap::ManeuverSet::GetSet();
        for (uint16_t index = 0U; index < path.GetSize(); ++index)
        {
            const MazeMap::ManeuverCode code = path[index];
            if (!maneuverSet.IsValidMove(code, current, maze))
            {
                end = MazeMap::DirectionalLocation();
                return false;
            }

            current = maneuverSet.Move(code, current);
            if (!maze.IsAccessibleLocation(current.GetLocation()))
            {
                end = MazeMap::DirectionalLocation();
                return false;
            }
        }

        end = current;
        return true;
    }

    MazeMap::Vehicle& _speedVehicle;
    MazeMap::Maze& _maze;
    MazeMap::FloodFillPathFinder& _searchPathFinder;
    MazeMap::ManeuverPathFinder& _speedPathFinder;
    SensorSuite _sensors;
    DiagnosticSensorSuite _telemetrySensors;
    DiagnosticLogger _telemetryLogger;
    MazeMap::CoreFileExport _missionTextLogFile;
    DriveBase _drive;
    MazeMap::CellCoordinates _currentCell;
    MazeMap::Direction _currentDirection;
    MazeMap::DirectionalLocation _currentDirectionalLocation;
    bool _goalPauseComplete;
    bool _missionComplete;
    bool _faulted;
    bool _maneuverTestMode;
    bool _telemetryLoggingEnabled;
    bool _missionTextLoggingEnabled;
    bool _missionMazeSnapshotWritten;
    float _lastWallTouchStandoffEstimateM;
    bool _hasWallTouchStandoffEstimate;
    unsigned long _lastControlMicros;

    static void SetRacingFanEnabled(bool enabled)
    {
        if (enabled)
        {
            RampFanDutyCycle(Config::kRacingFanDutyCycle);
            return;
        }

        WriteFanDutyCycle(0.0f);
    }

    static MotionLimits SearchLimits()
    {
        MotionLimits limits{};
        limits.maxSpeedMps = Config::kSearchMaxSpeedMps;
        limits.accelMps2 = Config::kSearchAccelMps2;
        limits.decelMps2 = Config::kSearchDecelMps2;
        limits.maxAngularSpeedRadps = Config::kSearchTurnMaxOmegaRadps;
        limits.angularAccelRadps2 = Config::kSearchTurnAccelRadps2;
        return limits;
    }

    MotionLimits FinalLimits() const
    {
        MotionLimits limits{};
        limits.maxSpeedMps = _speedVehicle.GetMaxSpeed() * Config::kSpeedRunScale;
        limits.accelMps2 = _speedVehicle.GetMaxForwardAcceleration() * Config::kSpeedRunScale;
        limits.decelMps2 = _speedVehicle.GetMaxForwardAcceleration() * Config::kSpeedRunScale;
        limits.maxAngularSpeedRadps = _speedVehicle.GetMaxRotationalVelocity() * Config::kSpeedRunScale;
        limits.angularAccelRadps2 = _speedVehicle.GetMaxAngularAcceleration() * Config::kSpeedRunScale;
        return limits;
    }

    static MotionLimits StartupWallCalibrationLimits()
    {
        MotionLimits limits{};
        limits.maxSpeedMps = Config::kStartupWallCalibrationSpeedMps;
        limits.accelMps2 = Config::kStartupWallCalibrationAccelMps2;
        limits.decelMps2 = Config::kStartupWallCalibrationDecelMps2;
        limits.maxAngularSpeedRadps = Config::kStartupWallCalibrationTurnMaxOmegaRadps;
        limits.angularAccelRadps2 = Config::kStartupWallCalibrationTurnAccelRadps2;
        return limits;
    }

    static MotionLimits StartupWallCalibrationCenteringLimits()
    {
        MotionLimits limits = StartupWallCalibrationLimits();
        limits.maxSpeedMps = Config::kStartupWallCalibrationCenteringSpeedMps;
        limits.accelMps2 = Config::kStartupWallCalibrationCenteringAccelMps2;
        limits.decelMps2 = Config::kStartupWallCalibrationCenteringDecelMps2;
        return limits;
    }

    float SearchUnmappedCruiseSpeedMps() const
    {
        const float frontSensorForwardOffsetM = (std::min)(
            _speedVehicle.FrontLeft.GetPosition().GetX(),
            _speedVehicle.FrontRight.GetPosition().GetX());
        float frontWallOnThresholdM = Config::kFrontWallOnThresholdM;
        float frontWallOffThresholdM = Config::kFrontWallOffThresholdM;
        gWallDistanceCalibration.TryComputeFrontWallDistanceThresholds(
            _speedVehicle,
            Config::kFrontWallReleaseHysteresisM,
            frontWallOnThresholdM,
            frontWallOffThresholdM);

        const float safeCruiseSpeedMps = (std::min)(
            SearchLimits().maxSpeedMps,
            MazeMap::ComputeSafeUnmappedCruiseSpeed(
                SearchLimits().decelMps2,
                frontWallOnThresholdM,
                frontSensorForwardOffsetM,
                Config::kWallTouchContactStandoffM,
                Config::kDistanceToleranceM));
        return MazeMap::ApplyMinimumCruiseSpeedFloor(
            safeCruiseSpeedMps,
            Config::kMinimumAllowedCruiseSpeedMps,
            SearchLimits().maxSpeedMps);
    }

    void SnapToStartPose()
    {
        _currentCell = MazeMap::CellCoordinates(0, 0);
        _currentDirection = MazeMap::Up;
        _currentDirectionalLocation = MazeMap::DirectionalLocation(MazeMap::MazeLocation::CellCenter(_currentCell), _currentDirection);
        _drive.SnapTo(_currentDirectionalLocation);
        _lastControlMicros = micros();
    }

    void PrimeKnownMissionStartCell()
    {
        MazeMap::Cell& cell = _maze[MazeMap::CellCoordinates(0, 0)];
        _maze.SetWall(cell, MazeMap::Up, MazeMap::NoWall);
        _maze.SetWall(cell, MazeMap::Down, MazeMap::Wall);
        _maze.SetWall(cell, MazeMap::Left, MazeMap::Wall);
        _maze.SetWall(cell, MazeMap::Right, MazeMap::Wall);
    }

    bool OpenMissionTextLog()
    {
        if (!_missionTextLoggingEnabled)
        {
            return true;
        }

        return _missionTextLogFile.Open("logging.txt");
    }

    void FlushMissionTextLog()
    {
        if (_missionTextLoggingEnabled)
        {
            _missionTextLogFile.Flush();
        }
    }

    void CloseMissionTextLog()
    {
        _missionTextLogFile.Close();
    }

    bool WriteMissionTextLineIfEnabled(const char* message)
    {
        if (!_missionTextLoggingEnabled)
        {
            return true;
        }

        if (message == nullptr || !_missionTextLogFile.IsOpen())
        {
            return false;
        }

        if (!_missionTextLogFile.Write(message) || !_missionTextLogFile.WriteChar('\n'))
        {
            return false;
        }

        _missionTextLogFile.Flush();
        return true;
    }

    void DisableMissionTextLogging(const char* traceLabel)
    {
        if (!_missionTextLoggingEnabled)
        {
            return;
        }

        if (traceLabel != nullptr && traceLabel[0] != '\0')
        {
            AppendStartupTrace(traceLabel);
            Serial.print("Mission text logging disabled: ");
            Serial.println(traceLabel);
        }

        FlushMissionTextLog();
        CloseMissionTextLog();
        _missionTextLoggingEnabled = false;
    }

    bool WriteMissionTraceLineBestEffort(const char* message, const char* traceLabel)
    {
        if (!_missionTextLoggingEnabled)
        {
            return true;
        }

        if (WriteMissionTextLineIfEnabled(message))
        {
            return true;
        }

        DisableMissionTextLogging(traceLabel);
        return true;
    }

    static const char* FrontObservationSourceName(const SensorSnapshot& snapshot)
    {
        if (snapshot.frontWallUsesFallbackDetection)
        {
            return "front_pair_fallback";
        }
        if (snapshot.frontLeftWall && snapshot.frontRightWall)
        {
            return "front_left+front_right";
        }
        if (snapshot.frontLeftWall)
        {
            return "front_left";
        }
        if (snapshot.frontRightWall)
        {
            return "front_right";
        }
        return "front_left+front_right";
    }

    bool LogWallObservationDecision(
        const MazeMap::CellCoordinates& observedCell,
        const char* relativeDirectionName,
        MazeMap::Direction absoluteDirection,
        MazeMap::WallState observedState,
        const char* sensorSource,
        const char* sensorMode,
        float primaryDistanceM,
        float secondaryDistanceM,
        bool primaryDetected,
        bool secondaryDetected,
        const SensorSnapshot& snapshot)
    {
        char line[256] = {};
        const bool haveSecondaryDistance = std::isfinite(secondaryDistanceM);
        const int written =
            haveSecondaryDistance ?
            snprintf(
                line,
                sizeof(line),
                "wall_obs,cell=(%u,%u),rel=%s,abs=%s,state=%s,sensor=%s,mode=%s,primary_hit=%u,secondary_hit=%u,primary_m=%.4f,secondary_m=%.4f",
                static_cast<unsigned>(observedCell.GetX()),
                static_cast<unsigned>(observedCell.GetY()),
                (relativeDirectionName != nullptr) ? relativeDirectionName : "unknown",
                DirectionName(absoluteDirection),
                WallStateName(observedState),
                (sensorSource != nullptr) ? sensorSource : "unknown",
                (sensorMode != nullptr) ? sensorMode : "unknown",
                primaryDetected ? 1U : 0U,
                secondaryDetected ? 1U : 0U,
                primaryDistanceM,
                secondaryDistanceM) :
            snprintf(
                line,
                sizeof(line),
                "wall_obs,cell=(%u,%u),rel=%s,abs=%s,state=%s,sensor=%s,primary_hit=%u,primary_m=%.4f",
                static_cast<unsigned>(observedCell.GetX()),
                static_cast<unsigned>(observedCell.GetY()),
                (relativeDirectionName != nullptr) ? relativeDirectionName : "unknown",
                DirectionName(absoluteDirection),
                WallStateName(observedState),
                (sensorSource != nullptr) ? sensorSource : "unknown",
                primaryDetected ? 1U : 0U,
                primaryDistanceM);
        if (written <= 0 || written >= static_cast<int>(sizeof(line)))
        {
            return Fail("Unable to format wall observation log");
        }

        AppendStartupTrace(line);
        (void)WriteMissionTraceLineBestEffort(line, "mission_text_logging:wall_observation_write_failed");
        if (_telemetryLoggingEnabled && !_telemetryLogger.WriteEvent("wall_observation", line))
        {
            return Fail("Unable to write wall observation log");
        }
        (void)snapshot;
        return true;
    }

    bool EmitMissionControllerLine(const char* message)
    {
        if (message == nullptr)
        {
            return false;
        }

        if (_missionTextLoggingEnabled)
        {
            if (WriteMissionTextLineIfEnabled(message))
            {
                return true;
            }

            DisableMissionTextLogging("mission_text_logging:controller_write_failed");
        }

        Serial.println(message);
        return true;
    }

    bool EmitMissionControllerFormatted(const char* format, ...)
    {
        if (format == nullptr)
        {
            return false;
        }

        char line[192] = {};
        va_list args;
        va_start(args, format);
        const int written = vsnprintf(line, sizeof(line), format, args);
        va_end(args);
        if (written <= 0 || written >= static_cast<int>(sizeof(line)))
        {
            return false;
        }

        return EmitMissionControllerLine(line);
    }

    bool EmitMissionControllerLineOrFail(const char* message)
    {
        return EmitMissionControllerLine(message);
    }

    bool EmitMissionControllerFormattedOrFail(const char* format, ...)
    {
        if (format == nullptr)
        {
            return false;
        }

        char line[192] = {};
        va_list args;
        va_start(args, format);
        const int written = vsnprintf(line, sizeof(line), format, args);
        va_end(args);
        if (written <= 0 || written >= static_cast<int>(sizeof(line)))
        {
            return false;
        }

        return EmitMissionControllerLineOrFail(line);
    }

    void AppendMissionTraceLine(const char* message)
    {
        if (message == nullptr)
        {
            return;
        }

        AppendStartupTrace(message);
        (void)WriteMissionTraceLineBestEffort(message, "mission_text_logging:trace_write_failed");
    }

    void AppendMissionTraceFormatted(const char* format, ...)
    {
        if (format == nullptr)
        {
            return;
        }

        char line[320] = {};
        va_list args;
        va_start(args, format);
        const int written = vsnprintf(line, sizeof(line), format, args);
        va_end(args);
        if (written <= 0 || written >= static_cast<int>(sizeof(line)))
        {
            return;
        }

        AppendMissionTraceLine(line);
    }

    bool WriteMissionMazeSnapshot(const char* trigger)
    {
        if (!_missionTextLoggingEnabled || _missionMazeSnapshotWritten)
        {
            return true;
        }

        const bool ok = MazeMap::ExportMazeSnapshot(_maze, "maze.txt");
        AppendStartupTrace(ok ? "mission_maze_snapshot:maze.txt" : "mission_maze_snapshot:write_failed");
        if (ok)
        {
            _missionMazeSnapshotWritten = true;
            (void)EmitMissionControllerFormatted("Maze snapshot written to maze.txt after %s", (trigger != nullptr) ? trigger : "unknown");
        }
        else
        {
            (void)EmitMissionControllerFormatted("Maze snapshot write failed after %s", (trigger != nullptr) ? trigger : "unknown");
        }

        return ok;
    }

    void AppendStartupCalibrationStateTrace(const char* label)
    {
        const PoseEstimate& pose = _drive.GetPose();
        const DriveTelemetry telemetry = _drive.GetTelemetry();
        char line[320] = {};
        snprintf(
            line,
            sizeof(line),
            "startup_cal_state:%s,x=%.4f,y=%.4f,yaw_deg=%.2f,v=%.4f,w=%.4f,left_v=%.4f,right_v=%.4f",
            (label != nullptr) ? label : "unknown",
            pose.xMeters,
            pose.yMeters,
            pose.yawRad * (180.0f / PI),
            pose.linearSpeedMps,
            pose.angularSpeedRadps,
            telemetry.leftVelocityMps,
            telemetry.rightVelocityMps);
        AppendStartupTrace(line);
    }

    void AppendStartupCalibrationMoveTrace(
        const char* axis,
        float startMeters,
        float targetMeters,
        float signedTravelMeters)
    {
        char line[192] = {};
        snprintf(
            line,
            sizeof(line),
            "startup_cal_move:%s,start=%.4f,target=%.4f,signed=%.4f",
            (axis != nullptr) ? axis : "unknown",
            startMeters,
            targetMeters,
            signedTravelMeters);
        AppendStartupTrace(line);
    }

    void AppendStartupCalibrationTurnTrace(const char* label, float currentYawRad, float targetYawRad, float angleRad)
    {
        char line[192] = {};
        snprintf(
            line,
            sizeof(line),
            "startup_cal_turn:%s,current_deg=%.2f,target_deg=%.2f,angle_deg=%.2f",
            (label != nullptr) ? label : "unknown",
            currentYawRad * (180.0f / PI),
            targetYawRad * (180.0f / PI),
            angleRad * (180.0f / PI));
        AppendStartupTrace(line);
    }

    void AppendStartupCalibrationTouchPlanTrace(
        CalibrationWall wall,
        float expectedTravelM,
        float minLatchTravelM,
        float maxApproachTravelM,
        float targetYawRad)
    {
        char line[256] = {};
        snprintf(
            line,
            sizeof(line),
            "startup_cal_touch_plan:wall=%s,expected=%.4f,min_latch=%.4f,max_travel=%.4f,target_yaw_deg=%.2f",
            CalibrationWallName(wall),
            expectedTravelM,
            minLatchTravelM,
            maxApproachTravelM,
            targetYawRad * (180.0f / PI));
        AppendStartupTrace(line);
    }

    void AppendStartupCalibrationTouchTrace(
        CalibrationWall wall,
        float traveledDistanceM,
        float expectedTravelM,
        float minLatchTravelM,
        float finalYawErrorRad)
    {
        const DriveTelemetry telemetry = _drive.GetTelemetry();
        char line[256] = {};
        snprintf(
            line,
            sizeof(line),
            "startup_cal_touch:wall=%s,travel=%.4f,expected=%.4f,min_latch=%.4f,final_yaw_err_deg=%.2f,left_v=%.4f,right_v=%.4f",
            CalibrationWallName(wall),
            traveledDistanceM,
            expectedTravelM,
            minLatchTravelM,
            finalYawErrorRad * (180.0f / PI),
            telemetry.leftVelocityMps,
            telemetry.rightVelocityMps);
        AppendStartupTrace(line);
    }

    void AppendStartupCalibrationSampleTrace(
        WallSensorId sensorId,
        CalibrationWall wall,
        float measuredValue,
        float fallbackDistanceM,
        float actualDistanceM)
    {
        char line[224] = {};
        snprintf(
            line,
            sizeof(line),
            "startup_cal_sample:sensor=%s,wall=%s,measured=%.6f,fallback=%.4f,actual=%.4f",
            WallSensorIdName(sensorId),
            CalibrationWallName(wall),
            measuredValue,
            fallbackDistanceM,
            actualDistanceM);
        AppendStartupTrace(line);
    }

    bool LogCorridorRepeatabilityMetadata()
    {
        char line[160] = {};
        if (!_telemetryLogger.WriteEvent(
            "summary",
            "Place the robot in a 5-cell enclosed row like a mission start. This routine runs startup wall calibration, drives to the far end and back at several speeds, and logs closure error at the start cell."))
        {
            return Fail("Unable to write corridor repeatability summary");
        }

        snprintf(line, sizeof(line), "row_cell_count,%u", static_cast<unsigned>(AuxMeasurementConfig::kCorridorRepeatabilityRowCellCount));
        if (!_telemetryLogger.WriteEvent("corridor_repeatability", line))
        {
            return Fail("Unable to write corridor repeatability metadata");
        }

        const float outDistanceM =
            (AuxMeasurementConfig::kCorridorRepeatabilityRowCellCount > 0U) ?
            (Config::kCellSizeM * static_cast<float>(AuxMeasurementConfig::kCorridorRepeatabilityRowCellCount - 1U)) :
            0.0f;
        snprintf(line, sizeof(line), "out_distance_m,%.6f", outDistanceM);
        if (!_telemetryLogger.WriteEvent("corridor_repeatability", line))
        {
            return Fail("Unable to write corridor repeatability metadata");
        }

        snprintf(
            line,
            sizeof(line),
            "accel_mps2,%.6f;decel_mps2,%.6f;turn_max_omega_radps,%.6f;turn_accel_radps2,%.6f",
            AuxMeasurementConfig::kCorridorRepeatabilityAccelMps2,
            AuxMeasurementConfig::kCorridorRepeatabilityDecelMps2,
            AuxMeasurementConfig::kCorridorRepeatabilityTurnMaxOmegaRadps,
            AuxMeasurementConfig::kCorridorRepeatabilityTurnAccelRadps2);
        if (!_telemetryLogger.WriteEvent("corridor_repeatability", line))
        {
            return Fail("Unable to write corridor repeatability metadata");
        }

        for (uint8_t speedIndex = 0U; speedIndex < AuxMeasurementConfig::kCorridorRepeatabilitySpeedCount; ++speedIndex)
        {
            snprintf(
                line,
                sizeof(line),
                "speed_%u_mps,%.6f",
                static_cast<unsigned>(speedIndex),
                AuxMeasurementConfig::kCorridorRepeatabilitySpeedsMps[speedIndex]);
            if (!_telemetryLogger.WriteEvent("corridor_repeatability_speed", line))
            {
                return Fail("Unable to write corridor repeatability speed metadata");
            }
        }

        return true;
    }

    bool LogPositionAccuracyAuditMetadata(const PositionAuditFixtureGeometry& geometry)
    {
        char line[320] = {};
        snprintf(
            line,
            sizeof(line),
            "Build a one-cell-wide fixture: normal mission start, a %u-cell north corridor including the start and corner cells, and a %u-cell east extension beyond that corner with solid side walls. All following phases reuse this same fixed geometry.",
            static_cast<unsigned>(geometry.northCorridorCellCount),
            static_cast<unsigned>(geometry.eastExtensionCellCount));
        if (!_telemetryLogger.WriteEvent("summary", line))
        {
            return Fail("Unable to write position accuracy audit summary");
        }
        if (!_telemetryLogger.WriteEvent(
                "summary",
                "position_straight_result isolates wheel-diameter, straight feedforward, and stop-distance error through north_touch_correction_m, enc_out_err_m, closure_m, and yaw_err_deg."))
        {
            return Fail("Unable to write position accuracy audit summary");
        }
        if (!_telemetryLogger.WriteEvent(
                "summary",
                "position_in_place_turn_result isolates the shared in-place turn profile through yaw_err_deg, effective_track_width_m, and wall_touch_correction_m."))
        {
            return Fail("Unable to write position accuracy audit summary");
        }
        if (!_telemetryLogger.WriteEvent(
                "summary",
                "position_smooth_turn_result compares S90SS and S90LS against nominal_radius_m, measured_radius_m, effective_track_width_m, corridor_err_m, and east_touch_correction_m to expose radius-dependent feedforward error."))
        {
            return Fail("Unable to write position accuracy audit summary");
        }
        if (!_telemetryLogger.WriteEvent(
                "summary",
                "position_audit_gyro_bias_average_radps is a 5 s raw stationary average; compare it against mission_gyro_bias_estimate_radps and position_audit_gyro_bias_delta_vs_mission_radps to judge startup gyro-bias repeatability."))
        {
            return Fail("Unable to write position accuracy audit summary");
        }
        if (!_telemetryLogger.WriteEvent(
                "summary",
                "Phase 1 runs S8, centers in the north corner, turns in place to face down, and runs S8 back to start."))
        {
            return Fail("Unable to write position accuracy audit summary");
        }
        if (!_telemetryLogger.WriteEvent(
                "summary",
                "Phase 2 reseats at start, runs S7 + S90SS + S7, centers at the east end, turns to face left, and returns on the reversed maneuver path."))
        {
            return Fail("Unable to write position accuracy audit summary");
        }
        if (!_telemetryLogger.WriteEvent(
                "summary",
                "Phase 3 reseats at start, runs S6 + S90LS + S6, recenters at the east end, and returns on the reversed maneuver path."))
        {
            return Fail("Unable to write position accuracy audit summary");
        }

        snprintf(
            line,
            sizeof(line),
            "north_corridor_cells,%u;east_extension_cells,%u;east_total_cells,%u",
            static_cast<unsigned>(geometry.northCorridorCellCount),
            static_cast<unsigned>(geometry.eastExtensionCellCount),
            static_cast<unsigned>(geometry.eastTotalCellCount));
        if (!_telemetryLogger.WriteEvent("position_audit", line))
        {
            return Fail("Unable to write position accuracy audit metadata");
        }

        snprintf(
            line,
            sizeof(line),
            "accel_mps2,%.6f;decel_mps2,%.6f;start_settle_ms,%u",
            AuxMeasurementConfig::kPositionAuditAccelMps2,
            AuxMeasurementConfig::kPositionAuditDecelMps2,
            static_cast<unsigned>(AuxMeasurementConfig::kPositionAuditStartSettleMs));
        if (!_telemetryLogger.WriteEvent("position_audit", line))
        {
            return Fail("Unable to write position accuracy audit metadata");
        }

        snprintf(
            line,
            sizeof(line),
            "phase=1;forward_half_steps=%u;turn=IP180;return_half_steps=%u",
            static_cast<unsigned>(AuxMeasurementConfig::kPositionAuditPhase1ForwardHalfSteps),
            static_cast<unsigned>(AuxMeasurementConfig::kPositionAuditPhase1ForwardHalfSteps));
        if (!_telemetryLogger.WriteEvent("position_audit_phase", line))
        {
            return Fail("Unable to write position accuracy audit metadata");
        }

        snprintf(
            line,
            sizeof(line),
            "phase=2;forward=%u,S90SS,%u;return=reverse(forward)",
            static_cast<unsigned>(AuxMeasurementConfig::kPositionAuditPhase2PreTurnHalfSteps),
            static_cast<unsigned>(AuxMeasurementConfig::kPositionAuditPhase2PostTurnHalfSteps));
        if (!_telemetryLogger.WriteEvent("position_audit_phase", line))
        {
            return Fail("Unable to write position accuracy audit metadata");
        }

        snprintf(
            line,
            sizeof(line),
            "phase=3;forward=%u,S90LS,%u;return=reverse(forward)",
            static_cast<unsigned>(AuxMeasurementConfig::kPositionAuditPhase3PreTurnHalfSteps),
            static_cast<unsigned>(AuxMeasurementConfig::kPositionAuditPhase3PostTurnHalfSteps));
        if (!_telemetryLogger.WriteEvent("position_audit_phase", line))
        {
            return Fail("Unable to write position accuracy audit metadata");
        }

        for (uint8_t speedIndex = 0U; speedIndex < AuxMeasurementConfig::kPositionAuditStraightSpeedCount; ++speedIndex)
        {
            snprintf(
                line,
                sizeof(line),
                "speed_%u_mps,%.6f",
                static_cast<unsigned>(speedIndex),
                AuxMeasurementConfig::kPositionAuditStraightSpeedsMps[speedIndex]);
            if (!_telemetryLogger.WriteEvent("position_audit_straight_speed", line))
            {
                return Fail("Unable to write position accuracy audit speed metadata");
            }
        }

        for (uint8_t speedIndex = 0U; speedIndex < AuxMeasurementConfig::kPositionAuditCornerSpeedCount; ++speedIndex)
        {
            snprintf(
                line,
                sizeof(line),
                "speed_%u_mps,%.6f",
                static_cast<unsigned>(speedIndex),
                AuxMeasurementConfig::kPositionAuditCornerSpeedsMps[speedIndex]);
            if (!_telemetryLogger.WriteEvent("position_audit_corner_speed", line))
            {
                return Fail("Unable to write position accuracy audit speed metadata");
            }
        }

        for (uint8_t codeIndex = 0U; codeIndex < AuxMeasurementConfig::kPositionAuditSmoothTurnCodeCount; ++codeIndex)
        {
            const MazeMap::ManeuverCode code = AuxMeasurementConfig::kPositionAuditSmoothTurnCodes[codeIndex];
            char codeName[24] = {};
            FormatManeuverCodeName(code, codeName, sizeof(codeName));
            snprintf(
                line,
                sizeof(line),
                "code=%s;nominal_radius_m=%.6f;distance_m=%.6f",
                codeName,
                MazeMap::ManeuverSet::GetSet()[code].GetNominalTurnRadiusInCells() * Config::kCellSizeM,
                ManeuverDistanceMeters(code));
            if (!_telemetryLogger.WriteEvent("position_audit_turn_code", line))
            {
                return Fail("Unable to write position accuracy audit turn metadata");
            }
        }

        return true;
    }

    MotionLimits CorridorRepeatabilityLimits(float cruiseSpeedMps) const
    {
        MotionLimits limits{};
        limits.maxSpeedMps = (std::max)(0.0f, cruiseSpeedMps);
        limits.accelMps2 = AuxMeasurementConfig::kCorridorRepeatabilityAccelMps2;
        limits.decelMps2 = AuxMeasurementConfig::kCorridorRepeatabilityDecelMps2;
        limits.maxAngularSpeedRadps = AuxMeasurementConfig::kCorridorRepeatabilityTurnMaxOmegaRadps;
        limits.angularAccelRadps2 = AuxMeasurementConfig::kCorridorRepeatabilityTurnAccelRadps2;
        return limits;
    }

    MotionLimits PositionAccuracyAuditStraightLimits(float cruiseSpeedMps) const
    {
        MotionLimits limits{};
        limits.maxSpeedMps = (std::max)(0.0f, cruiseSpeedMps);
        limits.accelMps2 = AuxMeasurementConfig::kPositionAuditAccelMps2;
        limits.decelMps2 = AuxMeasurementConfig::kPositionAuditDecelMps2;
        limits.maxAngularSpeedRadps = Config::kSearchTurnMaxOmegaRadps;
        limits.angularAccelRadps2 = Config::kSearchTurnAccelRadps2;
        return limits;
    }

    MotionLimits PositionAccuracyAuditCornerLimits(float cruiseSpeedMps, float nominalRadiusM) const
    {
        MotionLimits limits = PositionAccuracyAuditStraightLimits(cruiseSpeedMps);
        if (nominalRadiusM > 0.0f)
        {
            const float requiredOmegaRadps = cruiseSpeedMps / nominalRadiusM;
            limits.maxAngularSpeedRadps = (std::max)(limits.maxAngularSpeedRadps, 1.30f * requiredOmegaRadps);
        }
        return limits;
    }

    bool WriteCorridorRepeatabilityResult(
        uint8_t speedIndex,
        float cruiseSpeedMps,
        const PoseEstimate& startPose,
        const DriveTelemetry& startTelemetry)
    {
        const PoseEstimate& finalPose = _drive.GetPose();
        const DriveTelemetry& finalTelemetry = _drive.GetTelemetry();
        const float deltaXM = finalPose.xMeters - startPose.xMeters;
        const float deltaYM = finalPose.yMeters - startPose.yMeters;
        const float closureErrorM = std::sqrt((deltaXM * deltaXM) + (deltaYM * deltaYM));
        const float yawErrorDeg = RAD_TO_DEG * AngleErrorRad(startPose.yawRad, finalPose.yawRad);

        char message[224] = {};
        const int length = snprintf(
            message,
            sizeof(message),
            "speed_index=%u;cruise_mps=%.3f;dx_m=%.6f;dy_m=%.6f;closure_m=%.6f;yaw_err_deg=%.3f;"
            "left_delta_m=%.6f;right_delta_m=%.6f;left_delta_cnt=%ld;right_delta_cnt=%ld",
            static_cast<unsigned>(speedIndex),
            cruiseSpeedMps,
            deltaXM,
            deltaYM,
            closureErrorM,
            yawErrorDeg,
            finalTelemetry.leftDistanceM - startTelemetry.leftDistanceM,
            finalTelemetry.rightDistanceM - startTelemetry.rightDistanceM,
            static_cast<long>(finalTelemetry.leftEncoderCount - startTelemetry.leftEncoderCount),
            static_cast<long>(finalTelemetry.rightEncoderCount - startTelemetry.rightEncoderCount));

        if (length <= 0 || length >= static_cast<int>(sizeof(message)))
        {
            return Fail("Corridor repeatability result event overflowed");
        }
        if (_telemetryLogger.WriteEvent("corridor_repeatability_result", message))
        {
            return true;
        }
        return Fail("Unable to write corridor repeatability result");
    }

    bool WritePositionStraightAuditResult(
        uint8_t speedIndex,
        float cruiseSpeedMps,
        float northStopErrorM,
        float northTouchCorrectionM,
        float encoderOutErrorM,
        const PoseEstimate& startPose,
        const DriveTelemetry& startTelemetry)
    {
        const PoseEstimate& finalPose = _drive.GetPose();
        const DriveTelemetry finalTelemetry = _drive.GetTelemetry();
        const float deltaXM = finalPose.xMeters - startPose.xMeters;
        const float deltaYM = finalPose.yMeters - startPose.yMeters;
        const float closureErrorM = std::sqrt((deltaXM * deltaXM) + (deltaYM * deltaYM));
        const float yawErrorDeg = RAD_TO_DEG * AngleErrorRad(startPose.yawRad, finalPose.yawRad);

        char message[224] = {};
        const int length = snprintf(
            message,
            sizeof(message),
            "speed_idx=%u;v=%.3f;stop_err_m=%.6f;touch_correction_m=%.6f;enc_out_err_m=%.6f;"
            "dx_m=%.6f;dy_m=%.6f;closure_m=%.6f;yaw_err_deg=%.3f",
            static_cast<unsigned>(speedIndex),
            cruiseSpeedMps,
            northStopErrorM,
            northTouchCorrectionM,
            encoderOutErrorM,
            deltaXM,
            deltaYM,
            closureErrorM,
            yawErrorDeg);
        if (length <= 0 || length >= static_cast<int>(sizeof(message)))
        {
            return Fail("Position straight result event overflowed");
        }
        if (_telemetryLogger.WriteEvent("position_straight_result", message))
        {
            return true;
        }
        return Fail("Unable to write position straight result");
    }

    bool WritePositionInPlaceTurnAuditResult(
        MazeMap::Direction targetDirection,
        float touchCorrectionM,
        float leftDeltaM,
        float rightDeltaM,
        float yawChangeRad)
    {
        float effectiveTrackWidthM = 0.0f;
        const bool haveTrackWidth = MazeMap::TryComputeEffectiveTrackWidthM(
            leftDeltaM,
            rightDeltaM,
            yawChangeRad,
            effectiveTrackWidthM);
        const float yawErrorDeg = RAD_TO_DEG * AngleErrorRad(DirectionToYawRad(targetDirection), _drive.GetPose().yawRad);

        char message[224] = {};
        const int length =
            haveTrackWidth ?
            snprintf(
                message,
                sizeof(message),
                "target=%s;yaw_err_deg=%.3f;touch_correction_m=%.6f;left_delta_m=%.6f;right_delta_m=%.6f;"
                "yaw_change_deg=%.3f;effective_track_width_m=%.6f",
                DirectionName(targetDirection),
                yawErrorDeg,
                touchCorrectionM,
                leftDeltaM,
                rightDeltaM,
                RAD_TO_DEG * yawChangeRad,
                effectiveTrackWidthM) :
            snprintf(
                message,
                sizeof(message),
                "target=%s;yaw_err_deg=%.3f;touch_correction_m=%.6f;left_delta_m=%.6f;right_delta_m=%.6f;"
                "yaw_change_deg=%.3f;effective_track_width_m=nan",
                DirectionName(targetDirection),
                yawErrorDeg,
                touchCorrectionM,
                leftDeltaM,
                rightDeltaM,
                RAD_TO_DEG * yawChangeRad);
        if (length <= 0 || length >= static_cast<int>(sizeof(message)))
        {
            return Fail("Position in-place turn result event overflowed");
        }
        if (_telemetryLogger.WriteEvent("position_in_place_turn_result", message))
        {
            return true;
        }
        return Fail("Unable to write position in-place turn result");
    }

    bool WritePositionSmoothTurnAuditResult(
        MazeMap::ManeuverCode code,
        uint8_t speedIndex,
        float cruiseSpeedMps,
        float nominalRadiusM,
        float corridorErrorM,
        float eastTouchCorrectionM,
        float leftArcDeltaM,
        float rightArcDeltaM,
        float yawChangeRad,
        float yawErrorDeg)
    {
        float effectiveTrackWidthM = 0.0f;
        const bool haveTrackWidth = MazeMap::TryComputeEffectiveTrackWidthM(
            leftArcDeltaM,
            rightArcDeltaM,
            yawChangeRad,
            effectiveTrackWidthM);
        float measuredRadiusM = 0.0f;
        const bool haveMeasuredRadius = TryComputeEffectiveTurnRadiusM(
            leftArcDeltaM,
            rightArcDeltaM,
            yawChangeRad,
            measuredRadiusM);

        char codeName[24] = {};
        FormatManeuverCodeName(code, codeName, sizeof(codeName));
        char measuredRadiusText[24] = {};
        char effectiveTrackWidthText[24] = {};
        if (haveMeasuredRadius)
        {
            snprintf(measuredRadiusText, sizeof(measuredRadiusText), "%.6f", measuredRadiusM);
        }
        else
        {
            snprintf(measuredRadiusText, sizeof(measuredRadiusText), "nan");
        }
        if (haveTrackWidth)
        {
            snprintf(effectiveTrackWidthText, sizeof(effectiveTrackWidthText), "%.6f", effectiveTrackWidthM);
        }
        else
        {
            snprintf(effectiveTrackWidthText, sizeof(effectiveTrackWidthText), "nan");
        }

        char message[256] = {};
        const int length = snprintf(
            message,
            sizeof(message),
            "code=%s;speed_idx=%u;v=%.3f;nominal_radius_m=%.6f;measured_radius_m=%s;"
            "effective_track_width_m=%s;yaw_err_deg=%.3f;corridor_err_m=%.6f;east_touch_correction_m=%.6f",
            codeName,
            static_cast<unsigned>(speedIndex),
            cruiseSpeedMps,
            nominalRadiusM,
            measuredRadiusText,
            effectiveTrackWidthText,
            yawErrorDeg,
            corridorErrorM,
            eastTouchCorrectionM);
        if (length <= 0 || length >= static_cast<int>(sizeof(message)))
        {
            return Fail("Position smooth turn result event overflowed");
        }
        if (_telemetryLogger.WriteEvent("position_smooth_turn_result", message))
        {
            return true;
        }
        return Fail("Unable to write position smooth turn result");
    }

    bool ReseatMissionStartPoseWithPhasePrefix(const char* phasePrefix, uint16_t settleMs)
    {
        const MotionLimits limits = StartupWallCalibrationLimits();
        const MotionLimits centeringLimits = StartupWallCalibrationCenteringLimits();
        char phaseName[64] = {};

        if (!RetreatCalibrationPoseFromSideWallForSafeRotation(Config::kCellSizeM, centeringLimits, phasePrefix))
        {
            return false;
        }

        snprintf(phaseName, sizeof(phaseName), "%s_touch_south", (phasePrefix != nullptr) ? phasePrefix : "reseat");
        if (!BeginTelemetryPhase(phaseName))
        {
            return false;
        }
        if (!TouchWallAndSetPose(MazeMap::Down, CalibrationWall::South))
        {
            return false;
        }

        snprintf(phaseName, sizeof(phaseName), "%s_center", (phasePrefix != nullptr) ? phasePrefix : "reseat");
        if (!BeginTelemetryPhase(phaseName))
        {
            return false;
        }
        if (!DriveCalibrationPoseToKnownY(0.5f * Config::kCellSizeM, centeringLimits))
        {
            return false;
        }

        snprintf(phaseName, sizeof(phaseName), "%s_touch_west", (phasePrefix != nullptr) ? phasePrefix : "reseat");
        if (!BeginTelemetryPhase(phaseName))
        {
            return false;
        }
        if (!TouchWallAndSetPose(MazeMap::Left, CalibrationWall::West))
        {
            return false;
        }

        snprintf(phaseName, sizeof(phaseName), "%s_center_x", (phasePrefix != nullptr) ? phasePrefix : "reseat");
        if (!BeginTelemetryPhase(phaseName))
        {
            return false;
        }
        if (!DriveCalibrationPoseToKnownX(0.5f * Config::kCellSizeM, centeringLimits))
        {
            return false;
        }

        snprintf(phaseName, sizeof(phaseName), "%s_rotate_up", (phasePrefix != nullptr) ? phasePrefix : "reseat");
        if (!BeginTelemetryPhase(phaseName))
        {
            return false;
        }
        if (!RotateCalibrationTo(MazeMap::Up, limits))
        {
            return false;
        }

        SnapToStartPose();
        PrimeKnownMissionStartCell();
        snprintf(phaseName, sizeof(phaseName), "%s_settle", (phasePrefix != nullptr) ? phasePrefix : "reseat");
        return HoldPosition(settleMs, phaseName);
    }

    bool ReseatCorridorRepeatabilityStartPose(uint8_t speedIndex, float centerOffsetFromTouchM)
    {
        (void)centerOffsetFromTouchM;
        char phasePrefix[48] = {};
        snprintf(phasePrefix, sizeof(phasePrefix), "corridor_%u_reseat", static_cast<unsigned>(speedIndex));
        return ReseatMissionStartPoseWithPhasePrefix(
            phasePrefix,
            AuxMeasurementConfig::kCorridorRepeatabilityStartSettleMs);
    }

    bool RunSingleCorridorRepeatabilityPass(uint8_t speedIndex, float cruiseSpeedMps, float outDistanceM, float centerOffsetFromTouchM)
    {
        const MotionLimits limits = CorridorRepeatabilityLimits(cruiseSpeedMps);
        const MotionLimits touchLimits = StartupWallCalibrationLimits();
        const MotionLimits centeringLimits = StartupWallCalibrationCenteringLimits();
        char phaseName[48] = {};
        const MazeMap::Vectorf<2> northHeading = DirectionToUnitVector(MazeMap::Up);
        const MazeMap::Vectorf<2> southHeading = DirectionToUnitVector(MazeMap::Down);
        const float farCellCenterYM = (0.5f * Config::kCellSizeM) + outDistanceM;
        const float corridorSpanYM =
            Config::kCellSizeM *
            static_cast<float>(AuxMeasurementConfig::kCorridorRepeatabilityRowCellCount);
        const float farWallTouchYM = MazeMap::ComputeWallTouchPoseFromNorthWallM(
            corridorSpanYM,
            Config::kMazeWallThicknessM,
            Config::kWallTouchContactStandoffM);
        const MazeMap::Vectorf<2> farCellCenter(0.5f * Config::kCellSizeM, farCellCenterYM);
        const MazeMap::Vectorf<2> startCellCenter(0.5f * Config::kCellSizeM, 0.5f * Config::kCellSizeM);

        snprintf(phaseName, sizeof(phaseName), "corridor_%u_start", static_cast<unsigned>(speedIndex));
        if (!HoldPosition(AuxMeasurementConfig::kCorridorRepeatabilityStartSettleMs, phaseName))
        {
            return false;
        }

        const PoseEstimate startPose = _drive.GetPose();
        const DriveTelemetry startTelemetry = _drive.GetTelemetry();

        snprintf(phaseName, sizeof(phaseName), "corridor_%u_out", static_cast<unsigned>(speedIndex));
        if (!BeginTelemetryPhase(phaseName))
        {
            return false;
        }
        if (!ExecuteStraightProfile(outDistanceM, 0.0f, cruiseSpeedMps, 0.0f, limits, true, &northHeading, &farCellCenter))
        {
            return false;
        }

        snprintf(phaseName, sizeof(phaseName), "corridor_%u_touch_far", static_cast<unsigned>(speedIndex));
        if (!BeginTelemetryPhase(phaseName))
        {
            return false;
        }
        if (!TouchWallAndSetKnownWallCoordinate(MazeMap::Up, CalibrationWall::North, farWallTouchYM))
        {
            return false;
        }

        snprintf(phaseName, sizeof(phaseName), "corridor_%u_center_far", static_cast<unsigned>(speedIndex));
        if (!BeginTelemetryPhase(phaseName))
        {
            return false;
        }
        if (!DriveCalibrationPoseToKnownY(farCellCenterYM, centeringLimits, corridorSpanYM))
        {
            return false;
        }

        snprintf(phaseName, sizeof(phaseName), "corridor_%u_turn_far", static_cast<unsigned>(speedIndex));
        if (!BeginTelemetryPhase(phaseName))
        {
            return false;
        }
        if (!RotateCalibrationTo(MazeMap::Down, touchLimits))
        {
            return false;
        }
        if (!HoldPosition(AuxMeasurementConfig::kCorridorRepeatabilityStartSettleMs))
        {
            return false;
        }

        snprintf(phaseName, sizeof(phaseName), "corridor_%u_back", static_cast<unsigned>(speedIndex));
        if (!BeginTelemetryPhase(phaseName))
        {
            return false;
        }
        if (!ExecuteStraightProfile(outDistanceM, 0.0f, cruiseSpeedMps, 0.0f, limits, true, &southHeading, &startCellCenter))
        {
            return false;
        }

        snprintf(phaseName, sizeof(phaseName), "corridor_%u_turn_home", static_cast<unsigned>(speedIndex));
        if (!BeginTelemetryPhase(phaseName))
        {
            return false;
        }
        if (!RotateCalibrationTo(MazeMap::Up, touchLimits))
        {
            return false;
        }
        if (!HoldPosition(AuxMeasurementConfig::kCorridorRepeatabilityStartSettleMs))
        {
            return false;
        }

        if (!WriteCorridorRepeatabilityResult(speedIndex, cruiseSpeedMps, startPose, startTelemetry))
        {
            return false;
        }

        return ReseatCorridorRepeatabilityStartPose(speedIndex, centerOffsetFromTouchM);
    }

    bool RunCorridorRepeatabilityPasses()
    {
        if (AuxMeasurementConfig::kCorridorRepeatabilityRowCellCount < 2U)
        {
            return Fail("Corridor repeatability row must be at least two cells long");
        }

        const float outDistanceM =
            Config::kCellSizeM *
            static_cast<float>(AuxMeasurementConfig::kCorridorRepeatabilityRowCellCount - 1U);
        const float centerOffsetFromTouchM = MazeMap::ComputeMissionStartCenterAdvanceM(
            Config::kCellSizeM,
            Config::kMissionStartRearWallInsetM);
        if (centerOffsetFromTouchM <= 0.0f)
        {
            return Fail("Invalid corridor repeatability start-cell center offset");
        }

        for (uint8_t speedIndex = 0U; speedIndex < AuxMeasurementConfig::kCorridorRepeatabilitySpeedCount; ++speedIndex)
        {
            if (!RunSingleCorridorRepeatabilityPass(
                    speedIndex,
                    AuxMeasurementConfig::kCorridorRepeatabilitySpeedsMps[speedIndex],
                    outDistanceM,
                    centerOffsetFromTouchM))
            {
                return false;
            }
        }

        return true;
    }

    bool RunSinglePositionStraightAuditPass(
        const PositionAuditFixtureGeometry& geometry,
        uint8_t speedIndex,
        float cruiseSpeedMps)
    {
        const MotionLimits limits = PositionAccuracyAuditStraightLimits(cruiseSpeedMps);
        const MotionLimits touchLimits = StartupWallCalibrationLimits();
        const MotionLimits centeringLimits = StartupWallCalibrationCenteringLimits();
        const MazeMap::Vectorf<2> northHeading = DirectionToUnitVector(MazeMap::Up);
        const MazeMap::Vectorf<2> southHeading = DirectionToUnitVector(MazeMap::Down);
        const MazeMap::Vectorf<2> farCellCenter(0.5f * Config::kCellSizeM, geometry.farCellCenterYM);
        const MazeMap::Vectorf<2> startCellCenter(0.5f * Config::kCellSizeM, 0.5f * Config::kCellSizeM);
        char phaseName[64] = {};

        snprintf(phaseName, sizeof(phaseName), "position_straight_%u_start", static_cast<unsigned>(speedIndex));
        if (!HoldPosition(AuxMeasurementConfig::kPositionAuditStartSettleMs, phaseName))
        {
            return false;
        }

        const PoseEstimate startPose = _drive.GetPose();
        const DriveTelemetry startTelemetry = _drive.GetTelemetry();

        snprintf(phaseName, sizeof(phaseName), "position_straight_%u_out", static_cast<unsigned>(speedIndex));
        if (!BeginTelemetryPhase(phaseName))
        {
            return false;
        }
        if (!ExecuteStraightProfile(geometry.outDistanceM, 0.0f, cruiseSpeedMps, 0.0f, limits, true, &northHeading, &farCellCenter))
        {
            return false;
        }

        const PoseEstimate poseBeforeTouch = _drive.GetPose();
        const DriveTelemetry outTelemetry = _drive.GetTelemetry();
        const float northStopErrorM = geometry.farCellCenterYM - poseBeforeTouch.yMeters;
        const float encoderOutDistanceM =
            0.5f *
            ((outTelemetry.leftDistanceM - startTelemetry.leftDistanceM) +
                (outTelemetry.rightDistanceM - startTelemetry.rightDistanceM));
        const float encoderOutErrorM = encoderOutDistanceM - geometry.outDistanceM;

        snprintf(phaseName, sizeof(phaseName), "position_straight_%u_touch_far", static_cast<unsigned>(speedIndex));
        if (!BeginTelemetryPhase(phaseName))
        {
            return false;
        }
        float northTouchCorrectionM = 0.0f;
        if (!TouchWallAndSetKnownWallCoordinate(MazeMap::Up, CalibrationWall::North, geometry.farWallTouchYM, &northTouchCorrectionM))
        {
            return false;
        }

        snprintf(phaseName, sizeof(phaseName), "position_straight_%u_center_far", static_cast<unsigned>(speedIndex));
        if (!BeginTelemetryPhase(phaseName))
        {
            return false;
        }
        if (!DriveCalibrationPoseToKnownY(geometry.farCellCenterYM, centeringLimits, geometry.northCorridorSpanYM))
        {
            return false;
        }

        const DriveTelemetry turnStartTelemetry = _drive.GetTelemetry();
        const float turnStartYawRad = _drive.GetPose().yawRad;
        snprintf(phaseName, sizeof(phaseName), "position_straight_%u_turn_far", static_cast<unsigned>(speedIndex));
        if (!BeginTelemetryPhase(phaseName))
        {
            return false;
        }
        if (!RotateCalibrationTo(MazeMap::Down, touchLimits))
        {
            return false;
        }
        if (!HoldPosition(AuxMeasurementConfig::kPositionAuditStartSettleMs))
        {
            return false;
        }
        const DriveTelemetry turnEndTelemetry = _drive.GetTelemetry();
        const float yawChangeRad = WrapAngleRad(_drive.GetPose().yawRad - turnStartYawRad);
        const float leftTurnDeltaM = turnEndTelemetry.leftDistanceM - turnStartTelemetry.leftDistanceM;
        const float rightTurnDeltaM = turnEndTelemetry.rightDistanceM - turnStartTelemetry.rightDistanceM;
        if (!WritePositionInPlaceTurnAuditResult(
                MazeMap::Down,
                northTouchCorrectionM,
                leftTurnDeltaM,
                rightTurnDeltaM,
                yawChangeRad))
        {
            return false;
        }

        snprintf(phaseName, sizeof(phaseName), "position_straight_%u_back", static_cast<unsigned>(speedIndex));
        if (!BeginTelemetryPhase(phaseName))
        {
            return false;
        }
        if (!ExecuteStraightProfile(geometry.outDistanceM, 0.0f, cruiseSpeedMps, 0.0f, limits, true, &southHeading, &startCellCenter))
        {
            return false;
        }

        if (!WritePositionStraightAuditResult(
                speedIndex,
                cruiseSpeedMps,
                northStopErrorM,
                northTouchCorrectionM,
                encoderOutErrorM,
                startPose,
                startTelemetry))
        {
            return false;
        }

        return true;
    }

    bool RunSinglePositionInPlaceTurnAuditPass(uint8_t turnIndex, MazeMap::Direction targetDirection)
    {
        if (!(targetDirection == MazeMap::Right || targetDirection == MazeMap::Left))
        {
            return Fail("Position audit in-place turn direction is invalid");
        }

        const MotionLimits touchLimits = StartupWallCalibrationLimits();
        char phaseName[64] = {};

        snprintf(phaseName, sizeof(phaseName), "position_ip_turn_%u_start", static_cast<unsigned>(turnIndex));
        if (!HoldPosition(AuxMeasurementConfig::kPositionAuditStartSettleMs, phaseName))
        {
            return false;
        }

        const DriveTelemetry startTelemetry = _drive.GetTelemetry();
        const float startYawRad = _drive.GetPose().yawRad;
        float angleRad = 0.0f;
        if (!MazeMap::TryComputeSignedTurnAngleRad(startYawRad, DirectionToYawRad(targetDirection), angleRad))
        {
            return Fail("Position audit in-place turn angle is invalid");
        }

        snprintf(phaseName, sizeof(phaseName), "position_ip_turn_%u_turn", static_cast<unsigned>(turnIndex));
        if (!BeginTelemetryPhase(phaseName))
        {
            return false;
        }
        if (!ExecuteTurnProfile(angleRad, touchLimits))
        {
            return false;
        }

        const DriveTelemetry endTelemetry = _drive.GetTelemetry();
        const float yawChangeRad = WrapAngleRad(_drive.GetPose().yawRad - startYawRad);
        const float leftDeltaM = endTelemetry.leftDistanceM - startTelemetry.leftDistanceM;
        const float rightDeltaM = endTelemetry.rightDistanceM - startTelemetry.rightDistanceM;

        float touchCoordinateM = 0.0f;
        CalibrationWall touchWall = CalibrationWall::West;
        if (targetDirection == MazeMap::Right)
        {
            touchCoordinateM = MazeMap::ComputeWallTouchPoseFromEastWallM(
                Config::kCellSizeM,
                Config::kMazeWallThicknessM,
                Config::kWallTouchContactStandoffM);
            touchWall = CalibrationWall::East;
        }
        else
        {
            touchCoordinateM = MazeMap::ComputeWallTouchPoseFromWestWallM(
                Config::kMazeWallThicknessM,
                Config::kWallTouchContactStandoffM);
            touchWall = CalibrationWall::West;
        }

        snprintf(phaseName, sizeof(phaseName), "position_ip_turn_%u_touch", static_cast<unsigned>(turnIndex));
        if (!BeginTelemetryPhase(phaseName))
        {
            return false;
        }
        float touchCorrectionM = 0.0f;
        if (!TouchWallAndSetKnownWallCoordinate(targetDirection, touchWall, touchCoordinateM, &touchCorrectionM))
        {
            return false;
        }

        if (!WritePositionInPlaceTurnAuditResult(targetDirection, touchCorrectionM, leftDeltaM, rightDeltaM, yawChangeRad))
        {
            return false;
        }

        char phasePrefix[48] = {};
        snprintf(phasePrefix, sizeof(phasePrefix), "position_ip_turn_%u_reseat", static_cast<unsigned>(turnIndex));
        return ReseatMissionStartPoseWithPhasePrefix(phasePrefix, AuxMeasurementConfig::kPositionAuditStartSettleMs);
    }

    bool RunSinglePositionSmoothTurnAuditPass(
        const PositionAuditFixtureGeometry& geometry,
        uint8_t codeIndex,
        MazeMap::ManeuverCode code,
        uint8_t speedIndex,
        float requestedCruiseSpeedMps)
    {
        MazeMap::ManeuverPath forwardPath;
        MazeMap::ManeuverPath reversePath;
        uint8_t launchHalfSteps = 0U;
        uint8_t postStraightHalfSteps = 0U;
        if (!TryBuildPositionAuditSmoothTurnPaths(code, forwardPath, reversePath, launchHalfSteps, postStraightHalfSteps))
        {
            return Fail("Position audit smooth turn path is invalid");
        }

        const MazeMap::DirectionalLocation auditStart(
            MazeMap::MazeLocation::CellCenter(MazeMap::CellCoordinates(0U, 0U)),
            MazeMap::Up);
        MazeMap::DirectionalLocation finalLocation;
        if (!TryValidatePositionAuditPath(geometry.maze, forwardPath, auditStart, finalLocation))
        {
            return Fail("Position audit smooth turn path does not fit fixture");
        }
        const MazeMap::DirectionalLocation returnStart(finalLocation.GetLocation(), -finalLocation.GetDirection());
        MazeMap::DirectionalLocation returnEnd;
        if (!TryValidatePositionAuditPath(geometry.maze, reversePath, returnStart, returnEnd))
        {
            return Fail("Position audit smooth turn reverse path does not fit fixture");
        }
        if (!(returnEnd.GetLocation() == auditStart.GetLocation()) || returnEnd.GetDirection() != MazeMap::Down)
        {
            return Fail("Position audit smooth turn reverse path does not return to start");
        }

        const float nominalRadiusM = MazeMap::ManeuverSet::GetSet()[code].GetNominalTurnRadiusInCells() * Config::kCellSizeM;
        const MotionLimits straightLimits = PositionAccuracyAuditStraightLimits(requestedCruiseSpeedMps);
        const MotionLimits cornerLimits = PositionAccuracyAuditCornerLimits(requestedCruiseSpeedMps, nominalRadiusM);
        const MotionLimits calibrationLimits = StartupWallCalibrationLimits();
        const MotionLimits centeringLimits = StartupWallCalibrationCenteringLimits();
        const float turnCruiseSpeedMps = ManeuverSpeedLimit(code, cornerLimits);
        if (!(turnCruiseSpeedMps > 0.0f))
        {
            return Fail("Position audit smooth turn speed is invalid");
        }

        const MazeMap::DirectionalLocation launchLocation = auditStart.MoveForward(launchHalfSteps);
        const MazeMap::DirectionalLocation maneuverEnd = MazeMap::ManeuverSet::GetSet().Move(code, launchLocation);
        const float postStraightDistanceM = 0.5f * Config::kCellSizeM * static_cast<float>(postStraightHalfSteps);

        float finalTargetXM = 0.0f;
        float finalTargetYM = 0.0f;
        finalLocation.GetLocation().GetPhysicalLocation(Config::kCellSizeM, finalTargetXM, finalTargetYM);
        const MazeMap::Vectorf<2> northHeading = DirectionToUnitVector(MazeMap::Up);
        const MazeMap::Vectorf<2> finalHeading = DirectionToUnitVector(finalLocation.GetDirection());
        float launchXM = 0.0f;
        float launchYM = 0.0f;
        launchLocation.GetLocation().GetPhysicalLocation(Config::kCellSizeM, launchXM, launchYM);
        const MazeMap::Vectorf<2> launchPosition(launchXM, launchYM);
        const MazeMap::Vectorf<2> finalPosition(finalTargetXM, finalTargetYM);
        const float launchDistanceM = launchYM - (0.5f * Config::kCellSizeM);
        const float maneuverDistanceM = ManeuverDistanceMeters(code);
        const float maneuverAngleRad = static_cast<float>(MazeMap::CodeDegrees(code)) * DEG_TO_RAD;
        MazeMap::SmoothTurnExecutionProfile smoothTurnProfile{};
        const bool hasSmoothTurnProfile = TryGetSmoothTurnExecutionProfileMeters(code, smoothTurnProfile);
        char phaseName[64] = {};

        snprintf(
            phaseName,
            sizeof(phaseName),
            "position_turn_%u_%u_start",
            static_cast<unsigned>(codeIndex),
            static_cast<unsigned>(speedIndex));
        if (!HoldPosition(AuxMeasurementConfig::kPositionAuditStartSettleMs, phaseName))
        {
            return false;
        }

        snprintf(
            phaseName,
            sizeof(phaseName),
            "position_turn_%u_%u_launch",
            static_cast<unsigned>(codeIndex),
            static_cast<unsigned>(speedIndex));
        if (!BeginTelemetryPhase(phaseName))
        {
            return false;
        }
        if (!ExecuteStraightProfile(
                launchDistanceM,
                0.0f,
                requestedCruiseSpeedMps,
                turnCruiseSpeedMps,
                straightLimits,
                true,
                &northHeading,
                &launchPosition))
        {
            return false;
        }

        const DriveTelemetry arcStartTelemetry = _drive.GetTelemetry();
        const float arcStartYawRad = _drive.GetPose().yawRad;
        snprintf(
            phaseName,
            sizeof(phaseName),
            "position_turn_%u_%u_arc",
            static_cast<unsigned>(codeIndex),
            static_cast<unsigned>(speedIndex));
        if (!BeginTelemetryPhase(phaseName))
        {
            return false;
        }
        const float maneuverExitSpeedMps = (postStraightDistanceM > 0.0f) ? turnCruiseSpeedMps : 0.0f;
        if (hasSmoothTurnProfile)
        {
            if (!ExecuteSmoothTurnProfile(
                    code,
                    turnCruiseSpeedMps,
                    maneuverExitSpeedMps,
                    turnCruiseSpeedMps,
                    cornerLimits))
            {
                return false;
            }
        }
        else if (!ExecuteArcProfile(
                maneuverDistanceM,
                maneuverAngleRad,
                turnCruiseSpeedMps,
                maneuverExitSpeedMps,
                turnCruiseSpeedMps,
                cornerLimits))
        {
            return false;
        }
        const DriveTelemetry arcEndTelemetry = _drive.GetTelemetry();
        const float arcEndYawRad = _drive.GetPose().yawRad;
        const float leftArcDeltaM = arcEndTelemetry.leftDistanceM - arcStartTelemetry.leftDistanceM;
        const float rightArcDeltaM = arcEndTelemetry.rightDistanceM - arcStartTelemetry.rightDistanceM;
        const float yawChangeRad = WrapAngleRad(arcEndYawRad - arcStartYawRad);

        if (postStraightDistanceM > 0.0f)
        {
            snprintf(
                phaseName,
                sizeof(phaseName),
                "position_turn_%u_%u_post",
                static_cast<unsigned>(codeIndex),
                static_cast<unsigned>(speedIndex));
            if (!BeginTelemetryPhase(phaseName))
            {
                return false;
            }
            if (!ExecuteStraightProfile(
                    postStraightDistanceM,
                    turnCruiseSpeedMps,
                    requestedCruiseSpeedMps,
                    0.0f,
                    straightLimits,
                    true,
                    &finalHeading,
                    &finalPosition))
            {
                return false;
            }
        }

        const PoseEstimate poseBeforeTouch = _drive.GetPose();
        const SensorSnapshot snapshotBeforeTouch = _sensors.Capture(true, _drive.GetPose());
        const float yawErrorDeg = RAD_TO_DEG * AngleErrorRad(DirectionToYawRad(finalLocation.GetDirection()), poseBeforeTouch.yawRad);

        snprintf(
            phaseName,
            sizeof(phaseName),
            "position_turn_%u_%u_touch_east",
            static_cast<unsigned>(codeIndex),
            static_cast<unsigned>(speedIndex));
        if (!BeginTelemetryPhase(phaseName))
        {
            return false;
        }
        float eastTouchCorrectionM = 0.0f;
        if (!TouchWallAndSetKnownWallCoordinate(MazeMap::Right, CalibrationWall::East, geometry.eastWallTouchXM, &eastTouchCorrectionM))
        {
            return false;
        }

        if (!WritePositionSmoothTurnAuditResult(
                code,
                speedIndex,
                turnCruiseSpeedMps,
                nominalRadiusM,
                snapshotBeforeTouch.corridorErrorM,
                eastTouchCorrectionM,
                leftArcDeltaM,
                rightArcDeltaM,
                yawChangeRad,
                yawErrorDeg))
        {
            return false;
        }

        snprintf(
            phaseName,
            sizeof(phaseName),
            "position_turn_%u_%u_center_east",
            static_cast<unsigned>(codeIndex),
            static_cast<unsigned>(speedIndex));
        if (!BeginTelemetryPhase(phaseName))
        {
            return false;
        }
        if (!DriveCalibrationPoseToKnownX(finalTargetXM, centeringLimits))
        {
            return false;
        }

        snprintf(
            phaseName,
            sizeof(phaseName),
            "position_turn_%u_%u_face_left",
            static_cast<unsigned>(codeIndex),
            static_cast<unsigned>(speedIndex));
        if (!BeginTelemetryPhase(phaseName))
        {
            return false;
        }
        if (!RotateCalibrationTo(MazeMap::Left, calibrationLimits))
        {
            return false;
        }
        if (!HoldPosition(AuxMeasurementConfig::kPositionAuditStartSettleMs))
        {
            return false;
        }
        snprintf(
            phaseName,
            sizeof(phaseName),
            "position_turn_%u_%u_return",
            static_cast<unsigned>(codeIndex),
            static_cast<unsigned>(speedIndex));
        if (!BeginTelemetryPhase(phaseName))
        {
            return false;
        }

        _currentDirectionalLocation = returnStart;
        _currentDirection = _currentDirectionalLocation.GetDirection();
        _currentCell = static_cast<MazeMap::CellCoordinates>(_currentDirectionalLocation.GetLocation());
        MazeMap::ManeuverQueue queue(reversePath, _currentDirectionalLocation);
        ApplyAsymmetricQueueLimits(queue, cornerLimits, 0.0f, 0.0f);
        return ExecuteQueuedManeuvers(queue, cornerLimits, false);
    }

    bool RunPositionAccuracyAuditPasses()
    {
        if (AuxMeasurementConfig::kPositionAuditNorthCorridorCellCount < 3U)
        {
            return Fail("Position accuracy audit north corridor must be at least three cells");
        }
        if (AuxMeasurementConfig::kPositionAuditEastBranchCellCount < 1U)
        {
            return Fail("Position accuracy audit east extension must be at least one cell");
        }

        const PositionAuditFixtureGeometry geometry = BuildPositionAuditFixtureGeometry();

        for (uint8_t speedIndex = 0U; speedIndex < AuxMeasurementConfig::kPositionAuditStraightSpeedCount; ++speedIndex)
        {
            char phasePrefix[56] = {};
            snprintf(phasePrefix, sizeof(phasePrefix), "position_pass_%u_phase1", static_cast<unsigned>(speedIndex));
            if (!ReseatMissionStartPoseWithPhasePrefix(phasePrefix, AuxMeasurementConfig::kPositionAuditStartSettleMs))
            {
                return false;
            }
            if (!RunSinglePositionStraightAuditPass(
                    geometry,
                    speedIndex,
                    AuxMeasurementConfig::kPositionAuditStraightSpeedsMps[speedIndex]))
            {
                return false;
            }

            snprintf(phasePrefix, sizeof(phasePrefix), "position_pass_%u_phase2", static_cast<unsigned>(speedIndex));
            if (!ReseatMissionStartPoseWithPhasePrefix(phasePrefix, AuxMeasurementConfig::kPositionAuditStartSettleMs))
            {
                return false;
            }
            if (!RunSinglePositionSmoothTurnAuditPass(
                    geometry,
                    0U,
                    MazeMap::S90SS,
                    speedIndex,
                    AuxMeasurementConfig::kPositionAuditCornerSpeedsMps[speedIndex]))
            {
                return false;
            }

            snprintf(phasePrefix, sizeof(phasePrefix), "position_pass_%u_phase3", static_cast<unsigned>(speedIndex));
            if (!ReseatMissionStartPoseWithPhasePrefix(phasePrefix, AuxMeasurementConfig::kPositionAuditStartSettleMs))
            {
                return false;
            }
            if (!RunSinglePositionSmoothTurnAuditPass(
                    geometry,
                    1U,
                    MazeMap::S90LS,
                    speedIndex,
                    AuxMeasurementConfig::kPositionAuditCornerSpeedsMps[speedIndex]))
            {
                return false;
            }
        }

        return true;
    }

    void SeedStartupWallCalibrationPoseFromSouthWall()
    {
        _currentCell = MazeMap::CellCoordinates(0, 0);
        _currentDirection = MazeMap::Up;
        _currentDirectionalLocation = MazeMap::DirectionalLocation(MazeMap::MazeLocation::CellCenter(_currentCell), _currentDirection);
        _drive.SetPose(0.5f * Config::kCellSizeM, Config::kMissionStartRearWallInsetM, DirectionToYawRad(MazeMap::Up));
        _lastControlMicros = micros();
        AppendStartupCalibrationStateTrace("seed_south_wall_start");
    }

    bool RotateCalibrationTo(MazeMap::Direction targetDirection, const MotionLimits& limits)
    {
        const float targetYawRad = DirectionToYawRad(targetDirection);
        float angleRad = 0.0f;
        if (!MazeMap::TryComputeSignedTurnAngleRad(_drive.GetPose().yawRad, targetYawRad, angleRad))
        {
            return Fail("Startup calibration turn angle is invalid");
        }
        AppendStartupCalibrationTurnTrace("rotate_begin", _drive.GetPose().yawRad, targetYawRad, angleRad);
        if (!ExecuteTurnProfile(angleRad, limits))
        {
            return false;
        }

        _lastControlMicros = micros();
        AppendStartupCalibrationStateTrace("rotate_end");
        return true;
    }

    bool DriveCalibrationPoseToKnownX(float targetXMeters, const MotionLimits& limits)
    {
        // Audit reseat can target a global fixture x beyond one cell after an east-wall touch.
        if (!MazeMap::IsValidCalibrationCenterCoordinateM(targetXMeters))
        {
            return Fail("Startup calibration target x is invalid");
        }

        const PoseEstimate& startPose = _drive.GetPose();
        const float headingX = startPose.headingUnit.GetX();
        if (std::fabs(headingX) < 0.5f)
        {
            return Fail("Startup calibration x reposition requires east-west heading");
        }

        const float deltaXMeters = targetXMeters - startPose.xMeters;
        const float signedTravelMeters = deltaXMeters / headingX;
        const MazeMap::Vectorf<2> targetHeading = startPose.headingUnit;
        const MazeMap::Vectorf<2> targetPosition(targetXMeters, startPose.yMeters);
        AppendStartupCalibrationMoveTrace("x", startPose.xMeters, targetXMeters, signedTravelMeters);
        if (signedTravelMeters > Config::kDistanceToleranceM)
        {
            if (!ExecuteStraightProfile(signedTravelMeters, 0.0f, limits.maxSpeedMps, 0.0f, limits, false, &targetHeading, &targetPosition))
            {
                return false;
            }
        }
        else if (signedTravelMeters < -Config::kDistanceToleranceM)
        {
            if (!ExecuteReverseStraightProfile(-signedTravelMeters, limits, &targetHeading, &targetPosition))
            {
                return false;
            }
        }

        _lastControlMicros = micros();
        AppendStartupCalibrationStateTrace("x_move_end");
        return true;
    }

    bool DriveCalibrationPoseToKnownY(float targetYMeters, const MotionLimits& limits, float maxAllowedYMeters)
    {
        if (!(std::isfinite(targetYMeters) &&
            std::isfinite(maxAllowedYMeters) &&
            targetYMeters >= 0.0f &&
            maxAllowedYMeters >= 0.0f &&
            targetYMeters <= maxAllowedYMeters))
        {
            return Fail("Startup calibration target y is invalid");
        }

        const PoseEstimate& startPose = _drive.GetPose();
        const float headingY = startPose.headingUnit.GetY();
        if (std::fabs(headingY) < 0.5f)
        {
            return Fail("Startup calibration y reposition requires north-south heading");
        }

        const float deltaYMeters = targetYMeters - startPose.yMeters;
        const float signedTravelMeters = deltaYMeters / headingY;
        const MazeMap::Vectorf<2> targetHeading = startPose.headingUnit;
        const MazeMap::Vectorf<2> targetPosition(startPose.xMeters, targetYMeters);
        AppendStartupCalibrationMoveTrace("y", startPose.yMeters, targetYMeters, signedTravelMeters);
        if (signedTravelMeters > Config::kDistanceToleranceM)
        {
            if (!ExecuteStraightProfile(signedTravelMeters, 0.0f, limits.maxSpeedMps, 0.0f, limits, false, &targetHeading, &targetPosition))
            {
                return false;
            }
        }
        else if (signedTravelMeters < -Config::kDistanceToleranceM)
        {
            if (!ExecuteReverseStraightProfile(-signedTravelMeters, limits, &targetHeading, &targetPosition))
            {
                return false;
            }
        }

        _lastControlMicros = micros();
        AppendStartupCalibrationStateTrace("y_move_end");
        return true;
    }

    bool DriveCalibrationPoseToKnownY(float targetYMeters, const MotionLimits& limits)
    {
        return DriveCalibrationPoseToKnownY(targetYMeters, limits, Config::kCellSizeM);
    }

    float ComputeCalibrationSideRotationClearanceM() const
    {
        return Config::kWallCalibrationWallClearanceM + Config::kDistanceToleranceM;
    }

    float ComputeCalibrationSafeMinCenterXForWestWallRotationM() const
    {
        return MazeMap::ComputeCalibrationSafeMinCenterXFromWestWallForRearCornerM(
            Config::kMazeWallThicknessM,
            Config::kRobotRearWallContactOffsetM,
            Config::kRobotHalfWidthM,
            ComputeCalibrationSideRotationClearanceM());
    }

    float ComputeCalibrationSafeMaxCenterXForEastWallRotationM(float spanXMeters) const
    {
        return MazeMap::ComputeCalibrationSafeMaxCenterXFromEastWallForRearCornerM(
            spanXMeters,
            Config::kMazeWallThicknessM,
            Config::kRobotRearWallContactOffsetM,
            Config::kRobotHalfWidthM,
            ComputeCalibrationSideRotationClearanceM());
    }

    bool RetreatCalibrationPoseFromSideWallForSafeRotation(
        float spanXMeters,
        const MotionLimits& limits,
        const char* phasePrefix = nullptr)
    {
        if (!(std::isfinite(spanXMeters) && spanXMeters > 0.0f))
        {
            return Fail("Startup calibration side-clear span is invalid");
        }

        const PoseEstimate& pose = _drive.GetPose();
        const float headingX = pose.headingUnit.GetX();
        if (std::fabs(headingX) < 0.5f)
        {
            return true;
        }

        const float safeMinCenterXM = ComputeCalibrationSafeMinCenterXForWestWallRotationM();
        const float safeMaxCenterXM = ComputeCalibrationSafeMaxCenterXForEastWallRotationM(spanXMeters);
        if (!(safeMinCenterXM > 0.0f &&
            safeMaxCenterXM > safeMinCenterXM &&
            safeMaxCenterXM < spanXMeters))
        {
            return Fail("Startup calibration side-clear target is invalid");
        }

        float targetXMeters = pose.xMeters;
        if (headingX > 0.5f)
        {
            targetXMeters = (std::min)(pose.xMeters, safeMaxCenterXM);
        }
        else
        {
            targetXMeters = (std::max)(pose.xMeters, safeMinCenterXM);
        }

        if (std::fabs(targetXMeters - pose.xMeters) <= Config::kDistanceToleranceM)
        {
            return true;
        }

        if (phasePrefix != nullptr)
        {
            char phaseName[64] = {};
            snprintf(phaseName, sizeof(phaseName), "%s_clear_side", phasePrefix);
            if (!BeginTelemetryPhase(phaseName))
            {
                return false;
            }
        }

        return DriveCalibrationPoseToKnownX(targetXMeters, limits);
    }

    static bool HasWallTouchEncoderMotion(
        const DriveTelemetry& reference,
        const DriveTelemetry& current,
        float minimumDistanceDeltaM)
    {
        if (!std::isfinite(reference.leftDistanceM) ||
            !std::isfinite(reference.rightDistanceM) ||
            !std::isfinite(current.leftDistanceM) ||
            !std::isfinite(current.rightDistanceM) ||
            !std::isfinite(minimumDistanceDeltaM) ||
            minimumDistanceDeltaM < 0.0f)
        {
            return false;
        }

        return (std::fabs(current.leftDistanceM - reference.leftDistanceM) >= minimumDistanceDeltaM) ||
            (std::fabs(current.rightDistanceM - reference.rightDistanceM) >= minimumDistanceDeltaM);
    }

    static bool HasWallTouchSeatReleaseMotion(
        const DriveTelemetry& reference,
        const DriveTelemetry& current,
        float minimumPerWheelDistanceDeltaM)
    {
        if (!std::isfinite(reference.leftDistanceM) ||
            !std::isfinite(reference.rightDistanceM) ||
            !std::isfinite(current.leftDistanceM) ||
            !std::isfinite(current.rightDistanceM) ||
            !std::isfinite(minimumPerWheelDistanceDeltaM) ||
            minimumPerWheelDistanceDeltaM < 0.0f)
        {
            return false;
        }

        return (std::fabs(current.leftDistanceM - reference.leftDistanceM) >= minimumPerWheelDistanceDeltaM) &&
            (std::fabs(current.rightDistanceM - reference.rightDistanceM) >= minimumPerWheelDistanceDeltaM);
    }

    bool ExecuteWallTouchOff(
        float targetYawRad,
        float minLatchTravelM,
        float maxApproachTravelM,
        bool allowPassThroughNoWall,
        WallTouchOutcome& outcome,
        float& traveledDistanceM)
    {
        outcome = WallTouchOutcome::SeatedContact;
        const float startDistanceM = _drive.GetAverageDistanceMeters();
        const unsigned long touchStartMs = millis();
        const float clampedMinLatchTravelM = (std::max)(0.0f, minLatchTravelM);
        const float clampedMaxApproachTravelM = (std::max)(clampedMinLatchTravelM, maxApproachTravelM);
        if (!(std::isfinite(clampedMaxApproachTravelM) && clampedMaxApproachTravelM > 0.0f))
        {
            return Fail("Wall touch-off max travel is invalid");
        }
        const float motionEpsilonM = Config::kWallTouchProgressStallDistanceM;
        DriveTelemetry lastMotionTelemetry = _drive.GetTelemetry();
        unsigned long lastMotionMs = touchStartMs;
        (void)targetYawRad;
        const auto finishWallTouch = [this](const char* timeoutMessage) -> bool
        {
            _drive.Brake();
            return HoldBrakedUntilDriveSettles(timeoutMessage, Config::kStartupWallCalibrationSettleMs, 0U);
        };

        while (true)
        {
            float dtSeconds = 0.0f;
            SensorSnapshot snapshot{};
            if (!TickControl(false, dtSeconds, snapshot))
            {
                return false;
            }
            (void)snapshot;

            traveledDistanceM = std::fabs(_drive.GetAverageDistanceMeters() - startDistanceM);
            if (traveledDistanceM >= clampedMaxApproachTravelM)
            {
                char traceLine[192] = {};
                snprintf(
                    traceLine,
                    sizeof(traceLine),
                    "startup_cal_touch:max_travel_during_approach,travel=%.4f,expected=%.4f,max=%.4f",
                    traveledDistanceM,
                    clampedMinLatchTravelM,
                    clampedMaxApproachTravelM);
                AppendStartupTrace(traceLine);
                if (allowPassThroughNoWall)
                {
                    outcome = WallTouchOutcome::PassedThroughNoWall;
                    return finishWallTouch("Wall touch-off failed to settle after pass-through");
                }
                return Fail("Wall touch-off exceeded max travel");
            }

            CommandOpenLoop(MazeMap::MakeSymmetricOpenLoopDriveCommand(Config::kWallTouchDriveCommand));

            const unsigned long nowMs = millis();
            const unsigned long elapsedMs = nowMs - touchStartMs;
            const PoseEstimate& pose = _drive.GetPose();
            const DriveTelemetry telemetry = _drive.GetTelemetry();
            if (MazeMap::IsWallTouchContactSample(
                    traveledDistanceM,
                    pose.linearSpeedMps,
                    Config::kWallTouchMinApproachDistanceM,
                    clampedMinLatchTravelM,
                    Config::kMotionSettleSpeedThresholdMps,
                    elapsedMs,
                    Config::kWallTouchMinCommandTimeMs))
            {
                char traceLine[224] = {};
                snprintf(
                    traceLine,
                    sizeof(traceLine),
                    "startup_cal_touch:contact,travel=%.4f,expected=%.4f,elapsed_ms=%lu,v=%.4f",
                    traveledDistanceM,
                    clampedMinLatchTravelM,
                    elapsedMs,
                    pose.linearSpeedMps);
                AppendStartupTrace(traceLine);
                outcome = WallTouchOutcome::SeatedContact;
                return finishWallTouch("Wall touch-off failed to settle at contact");
            }

            if (HasWallTouchEncoderMotion(lastMotionTelemetry, telemetry, motionEpsilonM))
            {
                lastMotionMs = nowMs;
                lastMotionTelemetry = telemetry;
                continue;
            }

            if (elapsedMs < Config::kWallTouchMinCommandTimeMs)
            {
                continue;
            }

            if ((nowMs - lastMotionMs) < Config::kWallTouchProgressStallWindowMs)
            {
                continue;
            }

            char traceLine[192] = {};
            snprintf(
                traceLine,
                sizeof(traceLine),
                "startup_cal_touch:stalled,travel=%.4f,expected=%.4f,elapsed_ms=%lu",
                traveledDistanceM,
                clampedMinLatchTravelM,
                nowMs - touchStartMs);
            AppendStartupTrace(traceLine);

            if (MazeMap::IsWallTouchSeatedSample(
                    traveledDistanceM,
                    clampedMinLatchTravelM,
                    pose.linearSpeedMps,
                    pose.angularSpeedRadps,
                    telemetry.leftVelocityMps,
                    telemetry.rightVelocityMps,
                    Config::kMotionSettleSpeedThresholdMps,
                    Config::kMotionSettleAngularSpeedThresholdRadps,
                    Config::kMotionSettleSpeedThresholdMps))
            {
                char seatedTraceLine[256] = {};
                snprintf(
                    seatedTraceLine,
                    sizeof(seatedTraceLine),
                    "startup_cal_touch:seated,travel=%.4f,expected=%.4f,v=%.4f,w=%.4f,left_v=%.4f,right_v=%.4f",
                    traveledDistanceM,
                    clampedMinLatchTravelM,
                    pose.linearSpeedMps,
                    pose.angularSpeedRadps,
                    telemetry.leftVelocityMps,
                    telemetry.rightVelocityMps);
                AppendStartupTrace(seatedTraceLine);
                outcome = WallTouchOutcome::SeatedContact;
                return finishWallTouch("Wall touch-off failed to settle after seating");
            }

            const DriveTelemetry stallTelemetry = telemetry;
            const unsigned long seatStartMs = nowMs;
            while (true)
            {
                float seatDtSeconds = 0.0f;
                SensorSnapshot seatSnapshot{};
                if (!TickControl(false, seatDtSeconds, seatSnapshot))
                {
                    return false;
                }
                (void)seatSnapshot;

                traveledDistanceM = std::fabs(_drive.GetAverageDistanceMeters() - startDistanceM);
                if (traveledDistanceM >= clampedMaxApproachTravelM)
                {
                    char seatTraceLine[192] = {};
                    snprintf(
                        seatTraceLine,
                        sizeof(seatTraceLine),
                        "startup_cal_touch:max_travel_during_seat,travel=%.4f,expected=%.4f,max=%.4f",
                        traveledDistanceM,
                        clampedMinLatchTravelM,
                        clampedMaxApproachTravelM);
                    AppendStartupTrace(seatTraceLine);
                    if (allowPassThroughNoWall)
                    {
                        outcome = WallTouchOutcome::PassedThroughNoWall;
                        return finishWallTouch("Wall touch-off failed to settle after pass-through");
                    }
                    return Fail("Wall touch-off exceeded max travel");
                }

                const unsigned long seatNowMs = millis();
                const unsigned long seatElapsedMs = seatNowMs - seatStartMs;
                const float seatDriveCommand = MazeMap::ComputeLaunchAssistDriveFloor(
                    Config::kWallTouchDriveCommand,
                    Config::kWallTouchSeatRampMaxDriveCommand,
                    seatElapsedMs,
                    Config::kWallTouchSeatRampMs);
                const MazeMap::OpenLoopDriveCommand seatCommand =
                    (seatDriveCommand >= Config::kWallTouchSeatReleaseMinDriveCommand) ?
                    MazeMap::ComputeOpenLoopYawWiggleCommand(
                        seatDriveCommand,
                        seatElapsedMs,
                        Config::kWallTouchSeatWiggleHalfPeriodMs,
                        Config::kWallTouchSeatWiggleTurnFraction,
                        Config::kWallTouchSeatWiggleRetainedForwardFraction) :
                    MazeMap::MakeSymmetricOpenLoopDriveCommand(seatDriveCommand);
                CommandOpenLoop(seatCommand);

                const DriveTelemetry seatTelemetry = _drive.GetTelemetry();
                const PoseEstimate& seatPose = _drive.GetPose();
                const float leftSeatDeltaM = std::fabs(seatTelemetry.leftDistanceM - stallTelemetry.leftDistanceM);
                const float rightSeatDeltaM = std::fabs(seatTelemetry.rightDistanceM - stallTelemetry.rightDistanceM);
                if (MazeMap::IsWallTouchSeatedSample(
                        traveledDistanceM,
                        clampedMinLatchTravelM,
                        seatPose.linearSpeedMps,
                        seatPose.angularSpeedRadps,
                        seatTelemetry.leftVelocityMps,
                        seatTelemetry.rightVelocityMps,
                        Config::kMotionSettleSpeedThresholdMps,
                        Config::kMotionSettleAngularSpeedThresholdRadps,
                        Config::kMotionSettleSpeedThresholdMps))
                {
                    char seatedTraceLine[320] = {};
                    snprintf(
                        seatedTraceLine,
                        sizeof(seatedTraceLine),
                        "startup_cal_touch:seat_contact,drive=%.3f,left_cmd=%.3f,right_cmd=%.3f,travel=%.4f,elapsed_ms=%lu,left_v=%.4f,right_v=%.4f,left_delta=%.4f,right_delta=%.4f",
                        seatDriveCommand,
                        seatCommand.leftDriveCommand,
                        seatCommand.rightDriveCommand,
                        traveledDistanceM,
                        seatElapsedMs,
                        seatTelemetry.leftVelocityMps,
                        seatTelemetry.rightVelocityMps,
                        leftSeatDeltaM,
                        rightSeatDeltaM);
                    AppendStartupTrace(seatedTraceLine);
                    outcome = WallTouchOutcome::SeatedContact;
                    return finishWallTouch("Wall touch-off failed to settle after seating");
                }
                const bool seatReleaseMotionDetected =
                    HasWallTouchSeatReleaseMotion(stallTelemetry, seatTelemetry, Config::kWallTouchSeatReleaseDistanceM);
                if (MazeMap::ShouldReleaseWallTouchSeat(
                        seatDriveCommand,
                        Config::kWallTouchSeatReleaseMinDriveCommand,
                        seatElapsedMs,
                        Config::kWallTouchSeatReleaseMinSkidMs,
                        seatReleaseMotionDetected))
                {
                    char seatTraceLine[288] = {};
                    snprintf(
                        seatTraceLine,
                        sizeof(seatTraceLine),
                        "startup_cal_touch:seat_skid,drive=%.3f,left_cmd=%.3f,right_cmd=%.3f,travel=%.4f,elapsed_ms=%lu,left_v=%.4f,right_v=%.4f,left_delta=%.4f,right_delta=%.4f",
                        seatDriveCommand,
                        seatCommand.leftDriveCommand,
                        seatCommand.rightDriveCommand,
                        traveledDistanceM,
                        seatElapsedMs,
                        seatTelemetry.leftVelocityMps,
                        seatTelemetry.rightVelocityMps,
                        leftSeatDeltaM,
                        rightSeatDeltaM);
                    AppendStartupTrace(seatTraceLine);
                    outcome = WallTouchOutcome::SeatedContact;
                    return finishWallTouch("Wall touch-off failed to settle after seating");
                }
            }
        }
    }

    bool TryTouchWallAndMaybeSetKnownWallCoordinate(
        MazeMap::Direction facingDirection,
        CalibrationWall wall,
        float targetCoordinateM,
        bool allowPassThroughNoWall,
        WallTouchOutcome& outcome,
        float* traveledDistanceM = nullptr)
    {
        outcome = WallTouchOutcome::SeatedContact;
        if (!(std::isfinite(targetCoordinateM) && targetCoordinateM >= 0.0f))
        {
            return Fail("Startup calibration touch coordinate is invalid");
        }

        const MotionLimits limits = StartupWallCalibrationLimits();
        if (!RotateCalibrationTo(facingDirection, limits))
        {
            return false;
        }
        if (!HoldPosition(Config::kStartupWallCalibrationSettleMs))
        {
            return false;
        }

        const PoseEstimate& pose = _drive.GetPose();
        float xMeters = pose.xMeters;
        float yMeters = pose.yMeters;
        float expectedTravelM = 0.0f;
        switch (wall)
        {
        case CalibrationWall::West:
            xMeters = targetCoordinateM;
            expectedTravelM = std::fabs(pose.xMeters - xMeters);
            break;
        case CalibrationWall::East:
            xMeters = targetCoordinateM;
            expectedTravelM = std::fabs(xMeters - pose.xMeters);
            break;
        case CalibrationWall::South:
            yMeters = targetCoordinateM;
            expectedTravelM = std::fabs(pose.yMeters - yMeters);
            break;
        case CalibrationWall::North:
            yMeters = targetCoordinateM;
            expectedTravelM = std::fabs(yMeters - pose.yMeters);
            break;
        default:
            break;
        }

        const float targetYawRad = DirectionToYawRad(facingDirection);
        const float minLatchTravelM = MazeMap::ComputeWallTouchMinimumLatchTravelM(
            expectedTravelM,
            Config::kWallTouchMinApproachDistanceM,
            Config::kWallTouchExpectedTravelSlackM);
        const float maxApproachTravelM = MazeMap::ComputeWallTouchMaximumApproachDistanceM(
            expectedTravelM,
            Config::kWallTouchBaseMaxApproachDistanceM,
            Config::kWallTouchExpectedTravelSlackM);
        AppendStartupCalibrationTouchPlanTrace(wall, expectedTravelM, minLatchTravelM, maxApproachTravelM, targetYawRad);

        float localTravelM = 0.0f;
        if (!ExecuteWallTouchOff(
                targetYawRad,
                minLatchTravelM,
                maxApproachTravelM,
                allowPassThroughNoWall,
                outcome,
                localTravelM))
        {
            return false;
        }

        if (outcome == WallTouchOutcome::SeatedContact)
        {
            const float finalYawErrorRad = AngleErrorRad(targetYawRad, _drive.GetPose().yawRad);
            _drive.SetPose(xMeters, yMeters, DirectionToYawRad(facingDirection));
            _lastControlMicros = micros();
            AppendStartupCalibrationTouchTrace(wall, localTravelM, expectedTravelM, minLatchTravelM, finalYawErrorRad);
            AppendStartupCalibrationStateTrace("touch_pose_set");
        }

        if (traveledDistanceM != nullptr)
        {
            *traveledDistanceM = localTravelM;
        }
        return true;
    }

    bool TouchWallAndSetKnownWallCoordinate(
        MazeMap::Direction facingDirection,
        CalibrationWall wall,
        float targetCoordinateM,
        float* traveledDistanceM = nullptr)
    {
        WallTouchOutcome outcome = WallTouchOutcome::SeatedContact;
        if (!TryTouchWallAndMaybeSetKnownWallCoordinate(
                facingDirection,
                wall,
                targetCoordinateM,
                false,
                outcome,
                traveledDistanceM))
        {
            return false;
        }

        return true;
    }

    bool TouchWallAndSetPose(MazeMap::Direction facingDirection, CalibrationWall wall, float* traveledDistanceM = nullptr)
    {
        float targetCoordinateM = 0.0f;
        switch (wall)
        {
        case CalibrationWall::West:
            targetCoordinateM = MazeMap::ComputeWallTouchPoseFromWestWallM(
                Config::kMazeWallThicknessM,
                Config::kWallTouchContactStandoffM);
            break;
        case CalibrationWall::East:
            targetCoordinateM = MazeMap::ComputeWallTouchPoseFromEastWallM(
                Config::kCellSizeM,
                Config::kMazeWallThicknessM,
                Config::kWallTouchContactStandoffM);
            break;
        case CalibrationWall::South:
            targetCoordinateM = MazeMap::ComputeWallTouchPoseFromSouthWallM(
                Config::kMazeWallThicknessM,
                Config::kWallTouchContactStandoffM);
            break;
        case CalibrationWall::North:
            targetCoordinateM = MazeMap::ComputeWallTouchPoseFromNorthWallM(
                Config::kCellSizeM,
                Config::kMazeWallThicknessM,
                Config::kWallTouchContactStandoffM);
            break;
        default:
            return Fail("Startup calibration wall touch is invalid");
        }

        return TouchWallAndSetKnownWallCoordinate(
            facingDirection,
            wall,
            targetCoordinateM,
            traveledDistanceM);
    }

    bool TryComputeCalibrationReferenceDistanceM(const MazeMap::WallSensor& sensor, CalibrationWall wall, float& actualDistanceM) const
    {
        const PoseEstimate& pose = _drive.GetPose();
        switch (wall)
        {
        case CalibrationWall::West:
            return TryDistanceToWestWall(pose, sensor, actualDistanceM);
        case CalibrationWall::East:
            return TryDistanceToEastWall(pose, sensor, actualDistanceM);
        case CalibrationWall::South:
            return TryDistanceToSouthWall(pose, sensor, actualDistanceM);
        case CalibrationWall::North:
            return false;
        default:
            return false;
        }
    }

    bool StoreWallCalibrationPoint(
        WallSensorId sensorId,
        CalibrationWall wall,
        float actualDistanceM,
        const WallSensorCalibrationInput& input,
        const RobustSignalBand* differentialLightBand = nullptr)
    {
        if (!(std::isfinite(actualDistanceM) && actualDistanceM > 0.0f))
        {
            return Fail("Unable to compute startup wall calibration reference");
        }

        if (!gWallDistanceCalibration.AddPoint(sensorId, input.measuredValue, actualDistanceM, input.ambientLight))
        {
            return Fail("Unable to store startup wall calibration point");
        }
        if ((sensorId == WallSensorId::SideLeft) || (sensorId == WallSensorId::SideRight))
        {
            if (std::isfinite(input.differentialLight) && input.differentialLight > 0.0f)
            {
                gWallDistanceCalibration.SetSideWallReferenceDifferentialLight(sensorId, input.differentialLight);
                gWallDistanceCalibration.SetSideWallReferenceDistanceM(sensorId, actualDistanceM);
                if ((differentialLightBand != nullptr) &&
                    std::isfinite(differentialLightBand->low) &&
                    std::isfinite(differentialLightBand->high) &&
                    differentialLightBand->low > 0.0f &&
                    differentialLightBand->high >= differentialLightBand->low)
                {
                    gWallDistanceCalibration.SetSideWallReferenceDifferentialLightBand(
                        sensorId,
                        differentialLightBand->low,
                        differentialLightBand->high);
                }
            }
        }
        else if (IsFrontWallSensor(sensorId) &&
            (differentialLightBand != nullptr) &&
            std::isfinite(input.measuredValue) &&
            input.measuredValue > 0.0f &&
            std::isfinite(differentialLightBand->low) &&
            std::isfinite(differentialLightBand->high) &&
            differentialLightBand->low > 0.0f &&
            differentialLightBand->high >= differentialLightBand->low)
        {
            gWallDistanceCalibration.SetFrontWeakestCalibrationDifferentialLightBand(
                sensorId,
                input.measuredValue,
                differentialLightBand->low,
                differentialLightBand->high);
        }
        AppendStartupCalibrationSampleTrace(sensorId, wall, input.measuredValue, input.fallbackDistanceM, actualDistanceM);

        return true;
    }

    bool AddWallCalibrationPoint(WallSensorId sensorId, const MazeMap::WallSensor& sensor, CalibrationWall wall, float& actualDistanceM)
    {
        if (!TryComputeCalibrationReferenceDistanceM(sensor, wall, actualDistanceM))
        {
            return Fail("Unable to compute startup wall calibration reference");
        }

        const WallSensorCalibrationCapture capture = SampleWallCalibrationCaptureAverageRaw(sensorId, sensor);
        return StoreWallCalibrationPoint(
            sensorId,
            wall,
            actualDistanceM,
            capture.input,
            capture.haveDifferentialLightBand ? &capture.differentialLightBand : nullptr);
    }

    bool CollectMovingFrontCalibrationSweep(
        float startCenterXM,
        float farthestCenterXM,
        uint8_t pointCount,
        CalibrationWall wall,
        const MotionLimits& limits)
    {
        if (pointCount == 0U)
        {
            return true;
        }
        if (wall != CalibrationWall::West && wall != CalibrationWall::East)
        {
            return Fail("Startup front calibration sweep requires side wall reference");
        }

        const PoseEstimate& startPose = _drive.GetPose();
        const float headingX = startPose.headingUnit.GetX();
        if (std::fabs(headingX) < 0.5f)
        {
            return Fail("Startup front calibration sweep requires east-west heading");
        }

        const float deltaXMeters = farthestCenterXM - startPose.xMeters;
        const float signedTravelMeters = deltaXMeters / headingX;
        const bool drivingForward = signedTravelMeters >= 0.0f;
        const bool movingEast = deltaXMeters >= 0.0f;
        const MazeMap::Vectorf<2> targetHeading = startPose.headingUnit;
        const MazeMap::Vectorf<2> targetPosition(farthestCenterXM, startPose.yMeters);
        const float startDistanceM = _drive.GetAverageDistanceMeters();
        float commandedSpeedMps = 0.0f;
        EncoderProgressWatchdog translationWatchdog{};
        translationWatchdog.Reset(0.0f, millis());
        const unsigned long timeoutMs = millis() + static_cast<unsigned long>(2000.0f + (4000.0f * std::fabs(signedTravelMeters)));
        uint8_t nextPointIndex = 0U;

        while (true)
        {
            float dtSeconds = 0.0f;
            SensorSnapshot snapshot{};
            if (!TickControl(false, dtSeconds, snapshot))
            {
                return false;
            }
            (void)snapshot;

            const PoseEstimate& pose = _drive.GetPose();
            while (nextPointIndex < pointCount)
            {
                const float sampleCenterXM = MazeMap::ComputeStartupWallCalibrationFrontSampleCenterWithinSpanXM(
                    startCenterXM,
                    farthestCenterXM,
                    pointCount,
                    nextPointIndex);
                const bool crossedSample =
                    movingEast ?
                    (pose.xMeters >= (sampleCenterXM - Config::kDistanceToleranceM)) :
                    (pose.xMeters <= (sampleCenterXM + Config::kDistanceToleranceM));
                if (!crossedSample)
                {
                    break;
                }

                WallSensorCalibrationCapture frontLeftCapture{};
                WallSensorCalibrationCapture frontRightCapture{};
                SampleWallCalibrationCaptureAverageRawPair(
                    WallSensorId::FrontLeft,
                    _speedVehicle.FrontLeft,
                    WallSensorId::FrontRight,
                    _speedVehicle.FrontRight,
                    frontLeftCapture,
                    frontRightCapture);

                float frontLeftActualDistanceM = 0.0f;
                if (!TryComputeCalibrationReferenceDistanceM(_speedVehicle.FrontLeft, wall, frontLeftActualDistanceM) ||
                    !StoreWallCalibrationPoint(
                        WallSensorId::FrontLeft,
                        wall,
                        frontLeftActualDistanceM,
                        frontLeftCapture.input,
                        frontLeftCapture.haveDifferentialLightBand ? &frontLeftCapture.differentialLightBand : nullptr))
                {
                    return false;
                }

                float frontRightActualDistanceM = 0.0f;
                if (!TryComputeCalibrationReferenceDistanceM(_speedVehicle.FrontRight, wall, frontRightActualDistanceM) ||
                    !StoreWallCalibrationPoint(
                        WallSensorId::FrontRight,
                        wall,
                        frontRightActualDistanceM,
                        frontRightCapture.input,
                        frontRightCapture.haveDifferentialLightBand ? &frontRightCapture.differentialLightBand : nullptr))
                {
                    return false;
                }

                ++nextPointIndex;
            }

            float projectedRemainingM = 0.0f;
            if (!MazeMap::TryComputeProjectedDistanceToTargetM(
                    pose.xMeters,
                    pose.yMeters,
                    targetPosition.GetX(),
                    targetPosition.GetY(),
                    drivingForward ? targetHeading.GetX() : -targetHeading.GetX(),
                    drivingForward ? targetHeading.GetY() : -targetHeading.GetY(),
                    projectedRemainingM))
            {
                return Fail("Startup front calibration target projection is invalid");
            }

            const float traveledM = std::fabs(_drive.GetAverageDistanceMeters() - startDistanceM);
            const float remainingM = (std::max)(0.0f, projectedRemainingM);
            if ((remainingM <= Config::kDistanceToleranceM) && IsDriveMotionSettled())
            {
                _drive.Brake();
                break;
            }
            if (translationWatchdog.Stalled(traveledM, commandedSpeedMps, remainingM, millis()))
            {
                _drive.Brake();
                return Fail("Startup front calibration encoder progress stalled");
            }
            if (static_cast<long>(timeoutMs - millis()) <= 0)
            {
                return Fail("Startup front calibration sweep timed out");
            }

            const float accelLimitedSpeedMps = (std::min)(limits.maxSpeedMps, commandedSpeedMps + (limits.accelMps2 * dtSeconds));
            const float decelLimitedSpeedMps = ReachableSpeedWithBoundary(0.0f, remainingM, limits.decelMps2);
            commandedSpeedMps = (std::min)(accelLimitedSpeedMps, decelLimitedSpeedMps);

            const float headingErrorRad = HeadingErrorRad(targetHeading, pose.headingUnit);
            float angularCommandRadps = (Config::kStraightHeadingKp * headingErrorRad) - (Config::kStraightYawD * pose.angularSpeedRadps);
            angularCommandRadps = (std::clamp)(angularCommandRadps, -limits.maxAngularSpeedRadps, limits.maxAngularSpeedRadps);
            _drive.CommandVelocity(drivingForward ? commandedSpeedMps : -commandedSpeedMps, angularCommandRadps, dtSeconds);
        }

        if (nextPointIndex < pointCount)
        {
            return Fail("Startup front calibration sweep missed sample points");
        }

        return HoldPosition(Config::kMotionSettleHoldMs);
    }

    // WARNING: Keep this procedure aligned with the validated hardware sequence unless it is re-proven on the robot.
    // The startup wall calibration must remain:
    // 1. start north with the rear touching the south wall
    // 2. complete the stationary settle
    // 3. drive forward to the start-cell center
    // 4. face west and calibrate the left side sensor against the south-wall-referenced distance
    // 5. face east and calibrate the right side sensor against the south-wall-referenced distance
    // 6. touch the east wall, then back away while calibrating the front sensors against that east wall
    // 7. return to cell center, snap the centered pose, rotate north, and exit
    bool RunStartupWallCalibration()
    {
        if (!EmitMissionControllerLineOrFail("Startup wall calibration"))
        {
            return false;
        }
        gWallDistanceCalibration.Clear();
        _hasWallTouchStandoffEstimate = false;
        SeedStartupWallCalibrationPoseFromSouthWall();
        if (!WaitForMissionStartupStationaryHold())
        {
            return false;
        }
        AppendStartupTrace("startup_wall_calibration:begin");

        const uint8_t configuredFrontPointCount = Config::kStartupWallCalibrationFrontPointCount;
        const float targetCenterXM = 0.5f * Config::kCellSizeM;
        const float targetCenterYM = 0.5f * Config::kCellSizeM;
        const float eastWallTouchCenterXM = MazeMap::ComputeWallTouchPoseFromEastWallM(
            Config::kCellSizeM,
            Config::kMazeWallThicknessM,
            Config::kWallTouchContactStandoffM);
        const float requestedFarthestFrontCalibrationCenterXM =
            MazeMap::ComputeStartupWallCalibrationFarthestFrontSampleCenterFromEastWallXM(
                eastWallTouchCenterXM,
                Config::kStartupWallCalibrationFrontFarOffsetM,
                Config::kStartupWallCalibrationFrontStepM,
                configuredFrontPointCount);
        const float safeFrontCalibrationMinCenterXM = MazeMap::ComputeCalibrationSafeMinCenterXFromWestWallM(
            Config::kMazeWallThicknessM,
            Config::kRobotRearWallContactOffsetM,
            Config::kWallCalibrationWallClearanceM + Config::kDistanceToleranceM);
        const float frontSweepStartCenterXM = eastWallTouchCenterXM;
        uint8_t frontPointCount = configuredFrontPointCount;
        float farthestFrontCalibrationCenterXM = (std::max)(
            safeFrontCalibrationMinCenterXM,
            (std::min)(frontSweepStartCenterXM, requestedFarthestFrontCalibrationCenterXM));
        const float nearestFrontSensorDistanceM = MazeMap::ComputeCellInnerMaxCoordinateM(
            Config::kCellSizeM,
            Config::kMazeWallThicknessM) -
            frontSweepStartCenterXM -
            (std::max)(_speedVehicle.FrontLeft.GetPosition().GetX(), _speedVehicle.FrontRight.GetPosition().GetX());

        char geometryTraceLine[256] = {};
        snprintf(
            geometryTraceLine,
            sizeof(geometryTraceLine),
            "startup_cal_geometry:cell_pitch=%.4f,cell_clear=%.4f,center_y=%.4f,east_touch_x=%.4f,front_start_x=%.4f,front_farthest_x=%.4f,front_safe_min_x=%.4f,front_sensor_wall_dist=%.4f",
            Config::kCellSizeM,
            Config::kCellClearSpanM,
            targetCenterYM,
            eastWallTouchCenterXM,
            frontSweepStartCenterXM,
            farthestFrontCalibrationCenterXM,
            safeFrontCalibrationMinCenterXM,
            nearestFrontSensorDistanceM);
        AppendStartupTrace(geometryTraceLine);

        if (farthestFrontCalibrationCenterXM > (requestedFarthestFrontCalibrationCenterXM + Config::kDistanceToleranceM))
        {
            char traceLine[192] = {};
            snprintf(
                traceLine,
                sizeof(traceLine),
                "startup_wall_calibration:recover_front_sweep_end,requested=%.4f,adjusted=%.4f,safe_min_x=%.4f",
                requestedFarthestFrontCalibrationCenterXM,
                farthestFrontCalibrationCenterXM,
                safeFrontCalibrationMinCenterXM);
            AppendStartupTrace(traceLine);
        }
        if ((frontSweepStartCenterXM - farthestFrontCalibrationCenterXM) <= Config::kDistanceToleranceM)
        {
            frontPointCount = 0U;
            farthestFrontCalibrationCenterXM = frontSweepStartCenterXM;
            char traceLine[192] = {};
            snprintf(
                traceLine,
                sizeof(traceLine),
                "startup_wall_calibration:skip_front_sweep_no_safe_span,start_x=%.4f,safe_min_x=%.4f",
                frontSweepStartCenterXM,
                safeFrontCalibrationMinCenterXM);
            AppendStartupTrace(traceLine);
        }

        const MotionLimits limits = StartupWallCalibrationLimits();
        const MotionLimits centeringLimits = StartupWallCalibrationCenteringLimits();
        AppendStartupTrace("startup_wall_calibration:settle_at_south_wall_start");
        if (!HoldPosition(Config::kStartupWallCalibrationSettleMs))
        {
            return false;
        }
        AppendStartupTrace("startup_wall_calibration:move_forward_to_center");
        if (!DriveCalibrationPoseToKnownY(targetCenterYM, centeringLimits))
        {
            return false;
        }
        AppendStartupTrace("startup_wall_calibration:settle_after_forward_move");
        if (!HoldPosition(Config::kStartupWallCalibrationSettleMs))
        {
            return false;
        }

        float actualDistanceM = 0.0f;
        float sideDistanceSumM = 0.0f;
        uint8_t sideDistanceCount = 0U;

        AppendStartupTrace("startup_wall_calibration:rotate_west_for_left_side_sample");
        if (!RotateCalibrationTo(MazeMap::Left, limits))
        {
            return false;
        }
        AppendStartupTrace("startup_wall_calibration:settle_for_left_side_sample");
        if (!HoldPosition(Config::kStartupWallCalibrationSettleMs))
        {
            return false;
        }
        AppendStartupTrace("startup_wall_calibration:sample_left_side");
        if (!AddWallCalibrationPoint(WallSensorId::SideLeft, _speedVehicle.SideLeft, CalibrationWall::South, actualDistanceM))
        {
            return false;
        }
        const WallSensorCalibrationCapture rightSideBaselineCapture =
            SampleWallCalibrationCaptureAverageRaw(WallSensorId::SideRight, _speedVehicle.SideRight);
        gWallDistanceCalibration.SetSideWallBaselineDifferentialLight(
            WallSensorId::SideRight,
            rightSideBaselineCapture.input.differentialLight);
        if (rightSideBaselineCapture.haveDifferentialLightBand)
        {
            gWallDistanceCalibration.SetSideWallBaselineDifferentialLightBand(
                WallSensorId::SideRight,
                rightSideBaselineCapture.differentialLightBand.low,
                rightSideBaselineCapture.differentialLightBand.high);
        }
        sideDistanceSumM += actualDistanceM;
        ++sideDistanceCount;

        AppendStartupTrace("startup_wall_calibration:rotate_east_for_right_side_sample");
        if (!RotateCalibrationTo(MazeMap::Right, limits))
        {
            return false;
        }
        AppendStartupTrace("startup_wall_calibration:settle_for_right_side_sample");
        if (!HoldPosition(Config::kStartupWallCalibrationSettleMs))
        {
            return false;
        }
        AppendStartupTrace("startup_wall_calibration:sample_right_side");
        if (!AddWallCalibrationPoint(WallSensorId::SideRight, _speedVehicle.SideRight, CalibrationWall::South, actualDistanceM))
        {
            return false;
        }
        const WallSensorCalibrationCapture leftSideBaselineCapture =
            SampleWallCalibrationCaptureAverageRaw(WallSensorId::SideLeft, _speedVehicle.SideLeft);
        gWallDistanceCalibration.SetSideWallBaselineDifferentialLight(
            WallSensorId::SideLeft,
            leftSideBaselineCapture.input.differentialLight);
        if (leftSideBaselineCapture.haveDifferentialLightBand)
        {
            gWallDistanceCalibration.SetSideWallBaselineDifferentialLightBand(
                WallSensorId::SideLeft,
                leftSideBaselineCapture.differentialLightBand.low,
                leftSideBaselineCapture.differentialLightBand.high);
        }
        sideDistanceSumM += actualDistanceM;
        ++sideDistanceCount;

        AppendStartupTrace("startup_wall_calibration:touch_east");
        if (!TouchWallAndSetPose(MazeMap::Right, CalibrationWall::East))
        {
            return false;
        }
        if (frontPointCount > 0U)
        {
            AppendStartupTrace("startup_wall_calibration:moving_front_sweep_begin");
            if (!CollectMovingFrontCalibrationSweep(
                    frontSweepStartCenterXM,
                    farthestFrontCalibrationCenterXM,
                    frontPointCount,
                    CalibrationWall::East,
                    centeringLimits))
            {
                return false;
            }
        }
        else
        {
            AppendStartupTrace("startup_wall_calibration:moving_front_sweep_skipped");
        }

        AppendStartupTrace("startup_wall_calibration:return_to_center_from_front_sweep");
        if (!DriveCalibrationPoseToKnownX(targetCenterXM, centeringLimits))
        {
            return false;
        }
        AppendStartupTrace("startup_wall_calibration:set_pose_at_start_center");
        _drive.SetPose(targetCenterXM, targetCenterYM, DirectionToYawRad(MazeMap::Right));
        _lastControlMicros = micros();
        AppendStartupCalibrationStateTrace("center_pose_set");
        AppendStartupTrace("startup_wall_calibration:rotate_up_at_start_center");
        if (!RotateCalibrationTo(MazeMap::Up, limits))
        {
            return false;
        }

        AppendStartupTrace("startup_wall_calibration:settle_at_start_center");
        if (!HoldPosition(Config::kStartupWallCalibrationSettleMs))
        {
            return false;
        }

        WallSensorCalibrationCapture frontLeftBaselineCapture{};
        WallSensorCalibrationCapture frontRightBaselineCapture{};
        SampleWallCalibrationCaptureAverageRawPair(
            WallSensorId::FrontLeft,
            _speedVehicle.FrontLeft,
            WallSensorId::FrontRight,
            _speedVehicle.FrontRight,
            frontLeftBaselineCapture,
            frontRightBaselineCapture);
        gWallDistanceCalibration.SetFrontWallBaselineDifferentialLight(
            WallSensorId::FrontLeft,
            frontLeftBaselineCapture.input.differentialLight);
        gWallDistanceCalibration.SetFrontWallBaselineDifferentialLight(
            WallSensorId::FrontRight,
            frontRightBaselineCapture.input.differentialLight);
        if (frontLeftBaselineCapture.haveDifferentialLightBand)
        {
            gWallDistanceCalibration.SetFrontWallBaselineDifferentialLightBand(
                WallSensorId::FrontLeft,
                frontLeftBaselineCapture.differentialLightBand.low,
                frontLeftBaselineCapture.differentialLightBand.high);
        }
        if (frontRightBaselineCapture.haveDifferentialLightBand)
        {
            gWallDistanceCalibration.SetFrontWallBaselineDifferentialLightBand(
                WallSensorId::FrontRight,
                frontRightBaselineCapture.differentialLightBand.low,
                frontRightBaselineCapture.differentialLightBand.high);
        }

        if (sideDistanceCount > 0U)
        {
            gWallDistanceCalibration.SetExpectedSideWallDistanceM(sideDistanceSumM / static_cast<float>(sideDistanceCount));
        }

        AppendStartupTrace("startup_wall_calibration:complete");
        AppendStartupCalibrationStateTrace("startup_complete");
        SnapToStartPose();
        return HoldPosition(Config::kStartupWallCalibrationSettleMs);
    }

    bool Initialize(const char* banner, bool observeCurrentCellAfterInit)
    {
        Serial.begin(115200);
        delay(1000);

        if (!SetupHardware())
        {
            return Fail("Hardware setup failed");
        }
        ResetStartupTrace(_maneuverTestMode ? "mode:maneuver_file_test" : "mode:mission");
        if (!OpenMissionTextLog())
        {
            AppendStartupTrace("initialize:logging_txt_open_failed");
            DisableMissionTextLogging("initialize:mission_text_log_unavailable");
        }
        if (!EmitMissionControllerLine(banner))
        {
            AppendStartupTrace("initialize:banner_log_failed");
            return false;
        }
        AppendStartupTrace("initialize:setup_hardware_ok");
        if (!_drive.Begin())
        {
            return Fail("Drive base init failed");
        }
        AppendStartupTrace("initialize:drive_ok");
        if (!_sensors.Begin())
        {
            return Fail("Sensor init failed");
        }
        AppendStartupTrace("initialize:sensors_ok");

        SnapToStartPose();
        if (!RunStartupWallCalibration())
        {
            return false;
        }
        AppendStartupTrace("initialize:startup_wall_calibration_ok");

        if (observeCurrentCellAfterInit && !ObserveCurrentCell())
        {
            return false;
        }
        if (observeCurrentCellAfterInit)
        {
            AppendStartupTrace("initialize:observed_current_cell");
        }

        return true;
    }

    bool BeginTelemetryPhase(const char* name)
    {
        if (!_telemetryLoggingEnabled)
        {
            return true;
        }
        if (_telemetryLogger.BeginPhase(name))
        {
            return true;
        }
        return Fail("Failed to write maneuver test phase marker");
    }

    bool LogWallCalibrationMetadata()
    {
        if (!_telemetryLoggingEnabled)
        {
            return true;
        }

        char line[128] = {};
        snprintf(
            line,
            sizeof(line),
            "calibration_average_samples,%u",
            static_cast<unsigned>(Config::kWallCalibrationAverageSampleCount));
        if (!_telemetryLogger.WriteEvent("wall_calibration", line))
        {
            return Fail("Unable to write wall calibration metadata");
        }
        snprintf(
            line,
            sizeof(line),
            "detection_window_cycles,%u",
            static_cast<unsigned>(Config::kWallDetectionAverageWindowCycles));
        if (!_telemetryLogger.WriteEvent("wall_calibration", line))
        {
            return Fail("Unable to write wall calibration metadata");
        }
        snprintf(
            line,
            sizeof(line),
            "front_point_count,%u",
            static_cast<unsigned>(Config::kStartupWallCalibrationFrontPointCount));
        if (!_telemetryLogger.WriteEvent("wall_calibration", line))
        {
            return Fail("Unable to write wall calibration metadata");
        }
        snprintf(
            line,
            sizeof(line),
            "expected_side_distance_m,%.6f",
            gWallDistanceCalibration.GetExpectedSideWallDistanceM());
        if (!_telemetryLogger.WriteEvent("wall_calibration", line))
        {
            return Fail("Unable to write wall calibration metadata");
        }
        snprintf(
            line,
            sizeof(line),
            "configured_touch_standoff_m,%.6f",
            Config::kWallTouchContactStandoffM);
        if (!_telemetryLogger.WriteEvent("wall_calibration", line))
        {
            return Fail("Unable to write wall calibration metadata");
        }
        if (_hasWallTouchStandoffEstimate)
        {
            snprintf(
                line,
                sizeof(line),
                "estimated_touch_standoff_m,%.6f",
                _lastWallTouchStandoffEstimateM);
            if (!_telemetryLogger.WriteEvent("wall_calibration", line))
            {
                return Fail("Unable to write wall calibration metadata");
            }
        }

        float sideWallOnThresholdM = Config::kSideWallOnThresholdM;
        float sideWallOffThresholdM = Config::kSideWallOffThresholdM;
        if (gWallDistanceCalibration.TryComputeSideWallDistanceThresholds(
                Config::kSideWallDistanceLatchFractionOfCalibration,
                Config::kSideWallDistanceReleaseFractionOfCalibration,
                sideWallOnThresholdM,
                sideWallOffThresholdM))
        {
            snprintf(
                line,
                sizeof(line),
                "derived_side_wall_thresholds_m,%.6f,%.6f",
                sideWallOnThresholdM,
                sideWallOffThresholdM);
            if (!_telemetryLogger.WriteEvent("wall_calibration", line))
            {
                return Fail("Unable to write wall calibration metadata");
            }
        }

        float sideLeftReferenceDifferentialLight = 0.0f;
        float sideRightReferenceDifferentialLight = 0.0f;
        const bool haveSideLeftReferenceDifferentialLight = gWallDistanceCalibration.TryGetSideWallReferenceDifferentialLight(
            WallSensorId::SideLeft,
            sideLeftReferenceDifferentialLight);
        const bool haveSideRightReferenceDifferentialLight = gWallDistanceCalibration.TryGetSideWallReferenceDifferentialLight(
            WallSensorId::SideRight,
            sideRightReferenceDifferentialLight);
        if (haveSideLeftReferenceDifferentialLight || haveSideRightReferenceDifferentialLight)
        {
            snprintf(
                line,
                sizeof(line),
                "side_wall_reference_diff,%.6f,%.6f",
                sideLeftReferenceDifferentialLight,
                sideRightReferenceDifferentialLight);
            if (!_telemetryLogger.WriteEvent("wall_calibration", line))
            {
                return Fail("Unable to write wall calibration metadata");
            }
        }

        float sideLeftReferenceDifferentialLightLow = 0.0f;
        float sideLeftReferenceDifferentialLightHigh = 0.0f;
        float sideRightReferenceDifferentialLightLow = 0.0f;
        float sideRightReferenceDifferentialLightHigh = 0.0f;
        const bool haveSideLeftReferenceDifferentialLightBand = gWallDistanceCalibration.TryGetSideWallReferenceDifferentialLightBand(
            WallSensorId::SideLeft,
            sideLeftReferenceDifferentialLightLow,
            sideLeftReferenceDifferentialLightHigh);
        const bool haveSideRightReferenceDifferentialLightBand = gWallDistanceCalibration.TryGetSideWallReferenceDifferentialLightBand(
            WallSensorId::SideRight,
            sideRightReferenceDifferentialLightLow,
            sideRightReferenceDifferentialLightHigh);
        if (haveSideLeftReferenceDifferentialLightBand || haveSideRightReferenceDifferentialLightBand)
        {
            snprintf(
                line,
                sizeof(line),
                "side_wall_reference_diff_band,%.6f,%.6f,%.6f,%.6f",
                sideLeftReferenceDifferentialLightLow,
                sideLeftReferenceDifferentialLightHigh,
                sideRightReferenceDifferentialLightLow,
                sideRightReferenceDifferentialLightHigh);
            if (!_telemetryLogger.WriteEvent("wall_calibration", line))
            {
                return Fail("Unable to write wall calibration metadata");
            }
        }

        float sideLeftReferenceDistanceM = 0.0f;
        float sideRightReferenceDistanceM = 0.0f;
        const bool haveSideLeftReferenceDistanceM = gWallDistanceCalibration.TryGetSideWallReferenceDistanceM(
            WallSensorId::SideLeft,
            sideLeftReferenceDistanceM);
        const bool haveSideRightReferenceDistanceM = gWallDistanceCalibration.TryGetSideWallReferenceDistanceM(
            WallSensorId::SideRight,
            sideRightReferenceDistanceM);
        if (haveSideLeftReferenceDistanceM || haveSideRightReferenceDistanceM)
        {
            snprintf(
                line,
                sizeof(line),
                "side_wall_reference_distance_m,%.6f,%.6f",
                sideLeftReferenceDistanceM,
                sideRightReferenceDistanceM);
            if (!_telemetryLogger.WriteEvent("wall_calibration", line))
            {
                return Fail("Unable to write wall calibration metadata");
            }
        }

        float sideLeftBaselineDifferentialLight = 0.0f;
        float sideRightBaselineDifferentialLight = 0.0f;
        const bool haveSideLeftBaselineDifferentialLight = gWallDistanceCalibration.TryGetSideWallBaselineDifferentialLight(
            WallSensorId::SideLeft,
            sideLeftBaselineDifferentialLight);
        const bool haveSideRightBaselineDifferentialLight = gWallDistanceCalibration.TryGetSideWallBaselineDifferentialLight(
            WallSensorId::SideRight,
            sideRightBaselineDifferentialLight);
        if (haveSideLeftBaselineDifferentialLight || haveSideRightBaselineDifferentialLight)
        {
            snprintf(
                line,
                sizeof(line),
                "side_scene_baseline_diff,%.6f,%.6f",
                sideLeftBaselineDifferentialLight,
                sideRightBaselineDifferentialLight);
            if (!_telemetryLogger.WriteEvent("wall_calibration", line))
            {
                return Fail("Unable to write wall calibration metadata");
            }
        }

        float sideLeftBaselineDifferentialLightLow = 0.0f;
        float sideLeftBaselineDifferentialLightHigh = 0.0f;
        float sideRightBaselineDifferentialLightLow = 0.0f;
        float sideRightBaselineDifferentialLightHigh = 0.0f;
        const bool haveSideLeftBaselineDifferentialLightBand = gWallDistanceCalibration.TryGetSideWallBaselineDifferentialLightBand(
            WallSensorId::SideLeft,
            sideLeftBaselineDifferentialLightLow,
            sideLeftBaselineDifferentialLightHigh);
        const bool haveSideRightBaselineDifferentialLightBand = gWallDistanceCalibration.TryGetSideWallBaselineDifferentialLightBand(
            WallSensorId::SideRight,
            sideRightBaselineDifferentialLightLow,
            sideRightBaselineDifferentialLightHigh);
        if (haveSideLeftBaselineDifferentialLightBand || haveSideRightBaselineDifferentialLightBand)
        {
            snprintf(
                line,
                sizeof(line),
                "side_scene_baseline_diff_band,%.6f,%.6f,%.6f,%.6f",
                sideLeftBaselineDifferentialLightLow,
                sideLeftBaselineDifferentialLightHigh,
                sideRightBaselineDifferentialLightLow,
                sideRightBaselineDifferentialLightHigh);
            if (!_telemetryLogger.WriteEvent("wall_calibration", line))
            {
                return Fail("Unable to write wall calibration metadata");
            }
        }

        float sideLeftOnMeasuredThreshold = 0.0f;
        float sideLeftOffMeasuredThreshold = 0.0f;
        float sideRightOnMeasuredThreshold = 0.0f;
        float sideRightOffMeasuredThreshold = 0.0f;
        float sideLeftSignalBaseline = 0.0f;
        float sideRightSignalBaseline = 0.0f;
        const bool haveSideLeftMeasuredThreshold = gWallDistanceCalibration.TryComputeSideWallMeasuredThresholds(
            WallSensorId::SideLeft,
            Config::kSideWallMeasuredSignalLatchThreshold,
            Config::kSideWallMeasuredSignalReleaseThreshold,
            sideLeftOnMeasuredThreshold,
            sideLeftOffMeasuredThreshold,
            sideLeftSignalBaseline);
        const bool haveSideRightMeasuredThreshold = gWallDistanceCalibration.TryComputeSideWallMeasuredThresholds(
            WallSensorId::SideRight,
            Config::kSideWallMeasuredSignalLatchThreshold,
            Config::kSideWallMeasuredSignalReleaseThreshold,
            sideRightOnMeasuredThreshold,
            sideRightOffMeasuredThreshold,
            sideRightSignalBaseline);
        if (haveSideLeftMeasuredThreshold || haveSideRightMeasuredThreshold)
        {
            snprintf(
                line,
                sizeof(line),
                "derived_side_wall_diff_thresholds,%.6f,%.6f,%.6f,%.6f",
                sideLeftOnMeasuredThreshold,
                sideLeftOffMeasuredThreshold,
                sideRightOnMeasuredThreshold,
                sideRightOffMeasuredThreshold);
            if (!_telemetryLogger.WriteEvent("wall_calibration", line))
            {
                return Fail("Unable to write wall calibration metadata");
            }
        }

        float frontWallOnThresholdM = 0.0f;
        float frontWallOffThresholdM = 0.0f;
        if (gWallDistanceCalibration.TryComputeFrontWallDistanceThresholds(
                _speedVehicle,
                Config::kFrontWallReleaseHysteresisM,
                frontWallOnThresholdM,
                frontWallOffThresholdM))
        {
            snprintf(
                line,
                sizeof(line),
                "derived_front_wall_thresholds_m,%.6f,%.6f",
                frontWallOnThresholdM,
                frontWallOffThresholdM);
            if (!_telemetryLogger.WriteEvent("wall_calibration", line))
            {
                return Fail("Unable to write wall calibration metadata");
            }
        }

        float frontLeftOnMeasuredThreshold = 0.0f;
        float frontLeftOffMeasuredThreshold = 0.0f;
        float frontLeftSignalBaseline = 0.0f;
        float frontRightOnMeasuredThreshold = 0.0f;
        float frontRightOffMeasuredThreshold = 0.0f;
        float frontRightSignalBaseline = 0.0f;
        float frontLeftReferenceAmbientLight = 0.0f;
        float frontRightReferenceAmbientLight = 0.0f;
        const bool haveFrontLeftAmbient = gWallDistanceCalibration.TryComputeFrontSensorRepresentativeAmbientLight(
            WallSensorId::FrontLeft,
            frontLeftReferenceAmbientLight);
        const bool haveFrontRightAmbient = gWallDistanceCalibration.TryComputeFrontSensorRepresentativeAmbientLight(
            WallSensorId::FrontRight,
            frontRightReferenceAmbientLight);
        const bool haveFrontLeftThreshold = gWallDistanceCalibration.TryComputeFrontSensorMeasuredThresholds(
            WallSensorId::FrontLeft,
            _speedVehicle,
            Config::kFrontWallReleaseHysteresisM,
            haveFrontLeftAmbient ? frontLeftReferenceAmbientLight : NAN,
            frontLeftOnMeasuredThreshold,
            frontLeftOffMeasuredThreshold,
            frontLeftSignalBaseline);
        const bool haveFrontRightThreshold = gWallDistanceCalibration.TryComputeFrontSensorMeasuredThresholds(
            WallSensorId::FrontRight,
            _speedVehicle,
            Config::kFrontWallReleaseHysteresisM,
            haveFrontRightAmbient ? frontRightReferenceAmbientLight : NAN,
            frontRightOnMeasuredThreshold,
            frontRightOffMeasuredThreshold,
            frontRightSignalBaseline);
        if (haveFrontLeftAmbient || haveFrontRightAmbient)
        {
            snprintf(
                line,
                sizeof(line),
                "derived_front_wall_diff_reference_ambient,%.6f,%.6f",
                frontLeftReferenceAmbientLight,
                frontRightReferenceAmbientLight);
            if (!_telemetryLogger.WriteEvent("wall_calibration", line))
            {
                return Fail("Unable to write wall calibration metadata");
            }
        }
        if (haveFrontLeftThreshold || haveFrontRightThreshold)
        {
            snprintf(
                line,
                sizeof(line),
                "derived_front_wall_diff_thresholds,%.6f,%.6f,%.6f,%.6f",
                frontLeftOnMeasuredThreshold,
                frontLeftOffMeasuredThreshold,
                frontRightOnMeasuredThreshold,
                frontRightOffMeasuredThreshold);
            if (!_telemetryLogger.WriteEvent("wall_calibration", line))
            {
                return Fail("Unable to write wall calibration metadata");
            }
        }
        const bool haveFrontLeftBaselineDifferentialLight = gWallDistanceCalibration.TryGetFrontWallBaselineDifferentialLight(
            WallSensorId::FrontLeft,
            frontLeftSignalBaseline);
        const bool haveFrontRightBaselineDifferentialLight = gWallDistanceCalibration.TryGetFrontWallBaselineDifferentialLight(
            WallSensorId::FrontRight,
            frontRightSignalBaseline);
        if (haveFrontLeftBaselineDifferentialLight || haveFrontRightBaselineDifferentialLight)
        {
            snprintf(
                line,
                sizeof(line),
                "front_scene_baseline_diff,%.6f,%.6f",
                frontLeftSignalBaseline,
                frontRightSignalBaseline);
            if (!_telemetryLogger.WriteEvent("wall_calibration", line))
            {
                return Fail("Unable to write wall calibration metadata");
            }
        }

        float frontLeftBaselineDifferentialLightLow = 0.0f;
        float frontLeftBaselineDifferentialLightHigh = 0.0f;
        float frontRightBaselineDifferentialLightLow = 0.0f;
        float frontRightBaselineDifferentialLightHigh = 0.0f;
        const bool haveFrontLeftBaselineDifferentialLightBand = gWallDistanceCalibration.TryGetFrontWallBaselineDifferentialLightBand(
            WallSensorId::FrontLeft,
            frontLeftBaselineDifferentialLightLow,
            frontLeftBaselineDifferentialLightHigh);
        const bool haveFrontRightBaselineDifferentialLightBand = gWallDistanceCalibration.TryGetFrontWallBaselineDifferentialLightBand(
            WallSensorId::FrontRight,
            frontRightBaselineDifferentialLightLow,
            frontRightBaselineDifferentialLightHigh);
        if (haveFrontLeftBaselineDifferentialLightBand || haveFrontRightBaselineDifferentialLightBand)
        {
            snprintf(
                line,
                sizeof(line),
                "front_scene_baseline_diff_band,%.6f,%.6f,%.6f,%.6f",
                frontLeftBaselineDifferentialLightLow,
                frontLeftBaselineDifferentialLightHigh,
                frontRightBaselineDifferentialLightLow,
                frontRightBaselineDifferentialLightHigh);
            if (!_telemetryLogger.WriteEvent("wall_calibration", line))
            {
                return Fail("Unable to write wall calibration metadata");
            }
        }

        float frontLeftWeakestDifferentialLightLow = 0.0f;
        float frontLeftWeakestDifferentialLightHigh = 0.0f;
        float frontRightWeakestDifferentialLightLow = 0.0f;
        float frontRightWeakestDifferentialLightHigh = 0.0f;
        const bool haveFrontLeftWeakestDifferentialLightBand = gWallDistanceCalibration.TryGetFrontWeakestCalibrationDifferentialLightBand(
            WallSensorId::FrontLeft,
            frontLeftWeakestDifferentialLightLow,
            frontLeftWeakestDifferentialLightHigh);
        const bool haveFrontRightWeakestDifferentialLightBand = gWallDistanceCalibration.TryGetFrontWeakestCalibrationDifferentialLightBand(
            WallSensorId::FrontRight,
            frontRightWeakestDifferentialLightLow,
            frontRightWeakestDifferentialLightHigh);
        if (haveFrontLeftWeakestDifferentialLightBand || haveFrontRightWeakestDifferentialLightBand)
        {
            snprintf(
                line,
                sizeof(line),
                "front_weakest_calibration_diff_band,%.6f,%.6f,%.6f,%.6f",
                frontLeftWeakestDifferentialLightLow,
                frontLeftWeakestDifferentialLightHigh,
                frontRightWeakestDifferentialLightLow,
                frontRightWeakestDifferentialLightHigh);
            if (!_telemetryLogger.WriteEvent("wall_calibration", line))
            {
                return Fail("Unable to write wall calibration metadata");
            }
        }

        for (uint8_t sensorIndex = 0U; sensorIndex < static_cast<uint8_t>(WallSensorId::Count); ++sensorIndex)
        {
            const WallSensorId sensorId = static_cast<WallSensorId>(sensorIndex);
            const MazeMap::WallSensorCalibrationCurve& curve = gWallDistanceCalibration.GetCurve(sensorId);
            snprintf(
                line,
                sizeof(line),
                "%s,measurement,%s,count,%u",
                WallSensorIdName(sensorId),
                WallSensorCalibrationMeasurementName(sensorId),
                static_cast<unsigned>(curve.GetCount()));
            if (!_telemetryLogger.WriteEvent("wall_calibration_curve", line))
            {
                return Fail("Unable to write wall calibration curve metadata");
            }

            for (uint8_t pointIndex = 0U; pointIndex < curve.GetCount(); ++pointIndex)
            {
                const MazeMap::WallSensorCalibrationCurve::Point& point = curve.GetPoint(pointIndex);
                snprintf(
                    line,
                    sizeof(line),
                    "%s,%s,%u,%.6f,%.6f",
                    WallSensorIdName(sensorId),
                    WallSensorCalibrationMeasurementName(sensorId),
                    static_cast<unsigned>(pointIndex),
                    point.measuredValue,
                    point.actualDistanceM);
                if (!_telemetryLogger.WriteEvent("wall_calibration_point", line))
                {
                    return Fail("Unable to write wall calibration point metadata");
                }
            }
        }

        return true;
    }

    bool LogTelemetrySample(bool stationary, uint32_t timestampUs, uint32_t dtUs)
    {
        if (!_telemetryLoggingEnabled)
        {
            return true;
        }

        const DiagnosticSensorSnapshot telemetrySnapshot = _telemetrySensors.Capture(stationary, _drive.GetPose());
        const DriveTelemetry telemetry = _drive.GetTelemetry();
        if (_telemetryLogger.LogSample(stationary, timestampUs, dtUs, _drive.GetPose(), _drive, telemetry, telemetrySnapshot))
        {
            return true;
        }
        return Fail("Failed to write maneuver test sample");
    }

    bool Fail(const char* message)
    {
        _faulted = true;
        SetRacingFanEnabled(false);
        _drive.Brake();
        AppendStartupCalibrationStateTrace("fault_state");
        char traceMessage[128] = {};
        snprintf(traceMessage, sizeof(traceMessage), "fault:%s", (message != nullptr) ? message : "unknown");
        AppendStartupTrace(traceMessage);
        (void)EmitMissionControllerFormatted("FAULT: %s", (message != nullptr) ? message : "unknown");
        if (_missionTextLoggingEnabled && !_missionMazeSnapshotWritten)
        {
            (void)WriteMissionMazeSnapshot("mission_fault");
        }
        if (_telemetryLoggingEnabled)
        {
            _telemetryLogger.WriteEvent("fault", message);
            _telemetryLogger.Flush();
        }
        FlushMissionTextLog();
        if (_missionTextLoggingEnabled)
        {
            CloseMissionTextLog();
        }
        return false;
    }

    bool TickControl(bool stationary, float& dtSeconds, SensorSnapshot& snapshot)
    {
        while ((micros() - _lastControlMicros) < Config::kControlPeriodUs)
        {
            delayMicroseconds(50);
        }

        const unsigned long now = micros();
        dtSeconds = static_cast<float>(now - _lastControlMicros) * 1.0e-6f;
        _lastControlMicros = now;
        snapshot = _sensors.Capture(stationary, _drive.GetPose());
        _drive.UpdateOdometry(dtSeconds, snapshot.gyroRadps);
        return LogTelemetrySample(stationary, now, static_cast<uint32_t>(dtSeconds * 1.0e6f));
    }

    bool HoldPosition(uint16_t durationMs, const char* phaseName = nullptr)
    {
        if (phaseName != nullptr && !BeginTelemetryPhase(phaseName))
        {
            return false;
        }

        const unsigned long deadline = millis() + durationMs;
        _drive.Brake();
        while (static_cast<long>(deadline - millis()) > 0)
        {
            float dtSeconds = 0.0f;
            SensorSnapshot snapshot{};
            if (!TickControl(true, dtSeconds, snapshot))
            {
                return false;
            }
            _drive.Brake();
        }

        return true;
    }

    bool IsDriveMotionSettled() const
    {
        const DriveTelemetry telemetry = _drive.GetTelemetry();
        return MazeMap::IsMissionStartupStationarySample(
            _drive.GetPose().linearSpeedMps,
            _drive.GetPose().angularSpeedRadps,
            telemetry.leftVelocityMps,
            telemetry.rightVelocityMps,
            Config::kMotionSettleSpeedThresholdMps,
            Config::kMotionSettleAngularSpeedThresholdRadps);
    }

    bool HoldBrakedUntilDriveSettles(const char* timeoutMessage, uint16_t stationaryHoldMs = Config::kMotionSettleHoldMs, uint16_t timeoutMs = Config::kMotionSettleTimeoutMs)
    {
        if (timeoutMs > 0U && timeoutMessage == nullptr)
        {
            timeoutMessage = "Drive settle timed out";
        }

        const unsigned long startMs = millis();
        unsigned long stationaryStartMs = 0UL;
        bool stationaryWindowActive = false;
        _drive.Brake();
        while (true)
        {
            float dtSeconds = 0.0f;
            SensorSnapshot snapshot{};
            if (!TickControl(true, dtSeconds, snapshot))
            {
                return false;
            }
            (void)snapshot;

            _drive.Brake();
            const bool settled = IsDriveMotionSettled();
            const unsigned long nowMs = millis();
            if (!settled)
            {
                stationaryWindowActive = false;
            }
            else
            {
                if (!stationaryWindowActive)
                {
                    stationaryStartMs = nowMs;
                    stationaryWindowActive = true;
                }
                if ((nowMs - stationaryStartMs) >= stationaryHoldMs)
                {
                    return true;
                }
            }

            if ((timeoutMs > 0U) && ((nowMs - startMs) >= timeoutMs))
            {
                return Fail(timeoutMessage);
            }
        }
    }

    bool WaitForMissionStartupStationaryHold()
    {
        if (!EmitMissionControllerLineOrFail("Waiting for 2 s stationary start"))
        {
            return false;
        }
        AppendStartupTrace("startup_stationary_hold:waiting");

        unsigned long stationaryStartMs = 0UL;
        bool stationaryWindowActive = false;
        while (true)
        {
            float dtSeconds = 0.0f;
            SensorSnapshot snapshot{};
            if (!TickControl(true, dtSeconds, snapshot))
            {
                return false;
            }
            (void)snapshot;

            const DriveTelemetry telemetry = _drive.GetTelemetry();
            const bool stationary = MazeMap::IsMissionStartupStationarySample(
                _drive.GetPose().linearSpeedMps,
                _drive.GetPose().angularSpeedRadps,
                telemetry.leftVelocityMps,
                telemetry.rightVelocityMps,
                Config::kMissionStartupStationarySpeedThresholdMps,
                Config::kMissionStartupStationaryMaxAbsYawRateRadps);

            _drive.Brake();
            if (!stationary)
            {
                stationaryWindowActive = false;
                continue;
            }

            const unsigned long nowMs = millis();
            if (!stationaryWindowActive)
            {
                stationaryStartMs = nowMs;
                stationaryWindowActive = true;
            }

            if ((nowMs - stationaryStartMs) >= Config::kMissionStartupStationaryHoldMs)
            {
                AppendStartupTrace("startup_stationary_hold:complete");
                return true;
            }
        }
    }

    bool ExecuteReverseStraightProfile(
        float distanceM,
        const MotionLimits& limits,
        const MazeMap::Vectorf<2>* targetHeadingOverride = nullptr,
        const MazeMap::Vectorf<2>* targetPositionOverride = nullptr)
    {
        if (!(std::isfinite(distanceM) && distanceM > 0.0f))
        {
            return true;
        }

        const float startDistanceM = _drive.GetAverageDistanceMeters();
        const MazeMap::Vectorf<2> targetHeading =
            (targetHeadingOverride != nullptr) ?
            *targetHeadingOverride :
            _drive.GetPose().headingUnit;
        float commandedSpeedMps = 0.0f;
        EncoderProgressWatchdog translationWatchdog{};
        translationWatchdog.Reset(0.0f, millis());
        const unsigned long timeoutMs = millis() + static_cast<unsigned long>(2000.0f + (4000.0f * distanceM));
        bool projectionFallbackLogged = false;

        while (true)
        {
            float dtSeconds = 0.0f;
            SensorSnapshot snapshot{};
            if (!TickControl(false, dtSeconds, snapshot))
            {
                return false;
            }
            (void)snapshot;

            const float traveledM = std::fabs(_drive.GetAverageDistanceMeters() - startDistanceM);
            float remainingM = (std::max)(0.0f, distanceM - traveledM);
            if (targetPositionOverride != nullptr)
            {
                const PoseEstimate& pose = _drive.GetPose();
                float projectedRemainingM = 0.0f;
                if (!MazeMap::TryComputeProjectedDistanceToTargetM(
                    pose.xMeters,
                    pose.yMeters,
                    targetPositionOverride->GetX(),
                    targetPositionOverride->GetY(),
                    -targetHeading.GetX(),
                    -targetHeading.GetY(),
                    projectedRemainingM))
                {
                    if (!projectionFallbackLogged)
                    {
                        projectionFallbackLogged = true;
                        AppendStartupTrace("reverse_profile:projection_fallback_to_encoder_distance");
                    }
                }
                else
                {
                    remainingM = (std::max)(0.0f, projectedRemainingM);
                }
            }
            if ((remainingM <= Config::kDistanceToleranceM) && IsDriveMotionSettled())
            {
                _drive.Brake();
                return HoldPosition(Config::kMotionSettleHoldMs);
            }
            if (translationWatchdog.Stalled(traveledM, commandedSpeedMps, remainingM, millis()))
            {
                _drive.Brake();
                AppendStartupTrace("reverse_profile:encoder_progress_stalled_holding_position");
                return HoldPosition(Config::kMotionSettleHoldMs);
            }
            if (static_cast<long>(timeoutMs - millis()) <= 0)
            {
                _drive.Brake();
                AppendStartupTrace("reverse_profile:elapsed_budget_reached_holding_position");
                return HoldPosition(Config::kMotionSettleHoldMs);
            }

            const float accelLimitedSpeedMps = (std::min)(limits.maxSpeedMps, commandedSpeedMps + (limits.accelMps2 * dtSeconds));
            const float decelLimitedSpeedMps = ReachableSpeedWithBoundary(0.0f, remainingM, limits.decelMps2);
            commandedSpeedMps = (std::min)(accelLimitedSpeedMps, decelLimitedSpeedMps);

            const float headingErrorRad = HeadingErrorRad(targetHeading, _drive.GetPose().headingUnit);
            float angularCommandRadps = (Config::kStraightHeadingKp * headingErrorRad) - (Config::kStraightYawD * _drive.GetPose().angularSpeedRadps);
            angularCommandRadps = (std::clamp)(angularCommandRadps, -limits.maxAngularSpeedRadps, limits.maxAngularSpeedRadps);
            _drive.CommandVelocity(-commandedSpeedMps, angularCommandRadps, dtSeconds);
        }
    }

    bool LoadManeuverQueueFromSd(const char* fileName, MazeMap::ManeuverQueue& queue)
    {
#if defined(ARDUINO_TEENSY41)
        File file = SD.open(fileName, FILE_READ);
        if (!file)
        {
            AppendStartupTrace("maneuver_test:test_file_unavailable");
            Serial.println("Maneuver file unavailable; skipping maneuver-file test");
            return false;
        }
        AppendStartupTrace("maneuver_test:test_txt_opened");

        MazeMap::ManeuverPath path;
        char line[128] = {};
        uint16_t lineNumber = 0U;
        while (file.available())
        {
            const size_t lineLength = file.readBytesUntil('\n', line, sizeof(line) - 1U);
            line[lineLength] = '\0';
            ++lineNumber;

            char* hashComment = strchr(line, '#');
            if (hashComment != nullptr)
            {
                *hashComment = '\0';
            }

            char* slashComment = strstr(line, "//");
            if (slashComment != nullptr)
            {
                *slashComment = '\0';
            }

            for (char* token = strtok(line, ", \t\r;"); token != nullptr; token = strtok(nullptr, ", \t\r;"))
            {
                MazeMap::ManeuverCode code = MazeMap::MC_NONE;
                if (!TryParseManeuverCodeToken(token, code))
                {
                    char message[96] = {};
                    snprintf(message, sizeof(message), "Maneuver file token issue on line %u: %s", lineNumber, token);
                    file.close();
                    AppendStartupTrace("maneuver_test:test_file_parse_issue");
                    Serial.println(message);
                    return false;
                }
                if (!path.push_back(code))
                {
                    file.close();
                    AppendStartupTrace("maneuver_test:test_file_path_capacity_reached");
                    Serial.println("Maneuver file exceeded path capacity; skipping maneuver-file test");
                    return false;
                }
            }
        }

        file.close();

        if (path.GetSize() == 0)
        {
            AppendStartupTrace("maneuver_test:test_file_empty");
            Serial.println("Maneuver file did not contain any maneuvers");
            return false;
        }
        AppendStartupTrace("maneuver_test:path_parsed");

        queue.clear();
        if (!queue.push_back(path, _currentDirectionalLocation))
        {
            AppendStartupTrace("maneuver_test:queue_build_issue");
            Serial.println("Maneuver file could not be converted into a queue");
            return false;
        }
        AppendStartupTrace("maneuver_test:queue_built");

        queue.ComputeSpeeds(_speedVehicle, 0.0f, 0.0f);
        ApplyAsymmetricQueueLimits(queue, 0.0f, 0.0f);
        AppendStartupTrace("maneuver_test:speeds_computed");
        return LogLoadedManeuverQueue(queue);
#else
        (void)fileName;
        (void)queue;
        AppendStartupTrace("maneuver_test:teensy_target_required");
        Serial.println("Maneuver-file test mode requires the Teensy target");
        return false;
#endif
    }

    bool LogLoadedManeuverQueue(const MazeMap::ManeuverQueue& queue)
    {
        if (!_telemetryLoggingEnabled)
        {
            return true;
        }

        char message[128] = {};
        snprintf(message, sizeof(message), "count,%u", static_cast<unsigned>(queue.size()));
        if (!_telemetryLogger.WriteEvent("queue", message))
        {
            AppendStartupTrace("maneuver_test:queue_logging_disabled");
            Serial.println("Maneuver queue logging unavailable; continuing without queue metadata");
            _telemetryLogger.Flush();
            _telemetryLogger.Close();
            _telemetryLoggingEnabled = false;
            return true;
        }

        for (uint16_t i = 0; i < queue.size(); ++i)
        {
            char codeName[24] = {};
            char queueLine[160] = {};
            FormatManeuverCodeName(queue[i].GetCode(), codeName, sizeof(codeName));
            snprintf(
                queueLine,
                sizeof(queueLine),
                "%u,%s,%.6f,%.6f",
                static_cast<unsigned>(i),
                codeName,
                queue[i].GetEntrySpeed(),
                queue[i].GetExitSpeed());

            if (!_telemetryLogger.WriteEvent("queue_entry", queueLine))
            {
                AppendStartupTrace("maneuver_test:queue_logging_disabled");
                Serial.println("Maneuver queue entry logging unavailable; continuing without queue metadata");
                _telemetryLogger.Flush();
                _telemetryLogger.Close();
                _telemetryLoggingEnabled = false;
                return true;
            }
        }

        return true;
    }

    bool ExecuteQueuedManeuvers(MazeMap::ManeuverQueue& queue, const MotionLimits& limits, bool snapToExpectedLocation)
    {
        if (!EmitMissionControllerFormattedOrFail("Queued maneuvers: %u", static_cast<unsigned>(queue.size())))
        {
            return false;
        }

        for (uint16_t i = 0; i < queue.size(); ++i)
        {
            const MazeMap::ManeuverInstance& entry = queue[i];
            const MazeMap::ManeuverCode code = entry.GetCode();
            const float entrySpeed = entry.GetEntrySpeed();
            const float exitSpeed = entry.GetExitSpeed();
            char codeName[24] = {};
            FormatManeuverCodeName(code, codeName, sizeof(codeName));

            AppendMissionTraceFormatted(
                "mission_maneuver:begin,index=%u,code=%s,cell=(%d,%d),dir=%s,entry_v=%.4f,exit_v=%.4f",
                static_cast<unsigned>(i),
                codeName,
                _currentCell.GetX(),
                _currentCell.GetY(),
                DirectionName(_currentDirection),
                entrySpeed,
                exitSpeed);

            if (_maneuverTestMode)
            {
                char phaseName[48] = {};
                snprintf(phaseName, sizeof(phaseName), "maneuver_%u_%s", static_cast<unsigned>(i), codeName);
                if (!BeginTelemetryPhase(phaseName))
                {
                    return false;
                }
            }

            bool ok = false;
            if (IsStraightCode(code))
            {
                ok = ExecuteStraightProfile(
                    0.5f * Config::kCellSizeM * static_cast<float>(static_cast<uint8_t>(code)),
                    entrySpeed,
                    limits.maxSpeedMps,
                    exitSpeed,
                    limits,
                    true);
            }
            else
            {
                const float angleRad = static_cast<float>(MazeMap::CodeDegrees(code)) * DEG_TO_RAD;
                MazeMap::SmoothTurnExecutionProfile smoothTurnProfile{};
                if (TryGetSmoothTurnExecutionProfileMeters(code, smoothTurnProfile))
                {
                    const float maneuverSpeedLimit = ManeuverSpeedLimit(code, limits);
                    ok = ExecuteSmoothTurnProfile(code, entrySpeed, exitSpeed, maneuverSpeedLimit, limits);
                }
                else
                {
                    const float distanceM = ManeuverDistanceMeters(code);
                    if (distanceM <= 0.0f)
                    {
                        ok = ExecuteTurnProfile(angleRad, limits);
                    }
                    else
                    {
                        const float maneuverSpeedLimit = ManeuverSpeedLimit(code, limits);
                        ok = ExecuteArcProfile(distanceM, angleRad, entrySpeed, exitSpeed, maneuverSpeedLimit, limits);
                    }
                }
            }

            if (!ok)
            {
                return false;
            }

            _currentDirectionalLocation = entry.GetEnd();
            _currentDirection = _currentDirectionalLocation.GetDirection();
            _currentCell = static_cast<MazeMap::CellCoordinates>(_currentDirectionalLocation.GetLocation());

            AppendMissionTraceFormatted(
                "mission_maneuver:end,index=%u,code=%s,cell=(%d,%d),dir=%s,x=%.4f,y=%.4f,yaw_deg=%.2f",
                static_cast<unsigned>(i),
                codeName,
                _currentCell.GetX(),
                _currentCell.GetY(),
                DirectionName(_currentDirection),
                _drive.GetPose().xMeters,
                _drive.GetPose().yMeters,
                RAD_TO_DEG * _drive.GetPose().yawRad);

            if (snapToExpectedLocation)
            {
                _drive.SnapTo(_currentDirectionalLocation);
            }
        }

        return HoldPosition(50);
    }

    bool ExecuteQueuedManeuvers(MazeMap::ManeuverQueue& queue, bool snapToExpectedLocation)
    {
        return ExecuteQueuedManeuvers(queue, FinalLimits(), snapToExpectedLocation);
    }

    bool SnapPoseCoordinateOnConfirmedWallContact(CalibrationWall touchWall, float targetCoordinateM)
    {
        if (!(std::isfinite(targetCoordinateM) && targetCoordinateM >= 0.0f))
        {
            return Fail("Front wall contact snap coordinate is invalid");
        }

        const PoseEstimate& pose = _drive.GetPose();
        float xMeters = pose.xMeters;
        float yMeters = pose.yMeters;
        switch (touchWall)
        {
        case CalibrationWall::West:
        case CalibrationWall::East:
            xMeters = targetCoordinateM;
            break;
        case CalibrationWall::South:
        case CalibrationWall::North:
            yMeters = targetCoordinateM;
            break;
        default:
            return Fail("Front wall contact snap wall is invalid");
        }

        _drive.SetPose(xMeters, yMeters, pose.yawRad);
        _lastControlMicros = micros();
        AppendMissionTraceFormatted(
            "mission_front_touch_snap,cell=(%d,%d),wall=%s,target=%.4f,x=%.4f,y=%.4f,yaw_deg=%.2f",
            _currentCell.GetX(),
            _currentCell.GetY(),
            CalibrationWallName(touchWall),
            targetCoordinateM,
            xMeters,
            yMeters,
            RAD_TO_DEG * pose.yawRad);
        return true;
    }

    bool BackOffToCurrentCellCenterAlongCurrentHeading()
    {
        float centerXMeters = 0.0f;
        float centerYMeters = 0.0f;
        if (!TryGetCellCenterMeters(_currentCell, centerXMeters, centerYMeters))
        {
            return Fail("Unable to compute current cell center for front-wall backoff");
        }

        const PoseEstimate& pose = _drive.GetPose();
        const MazeMap::Vectorf<2> targetHeading = pose.headingUnit;
        const MazeMap::Vectorf<2> targetPosition(centerXMeters, centerYMeters);
        float reverseDistanceM = 0.0f;
        if (!MazeMap::TryComputeProjectedDistanceToTargetM(
                pose.xMeters,
                pose.yMeters,
                centerXMeters,
                centerYMeters,
                -targetHeading.GetX(),
                -targetHeading.GetY(),
                reverseDistanceM))
        {
            return Fail("Front-wall backoff target projection is invalid");
        }

        reverseDistanceM = (std::max)(0.0f, reverseDistanceM);
        AppendMissionTraceFormatted(
            "mission_front_touch_backoff,cell=(%d,%d),distance_m=%.4f,target_x=%.4f,target_y=%.4f",
            _currentCell.GetX(),
            _currentCell.GetY(),
            reverseDistanceM,
            centerXMeters,
            centerYMeters);
        if (reverseDistanceM <= Config::kDistanceToleranceM)
        {
            return HoldPosition(Config::kMotionSettleHoldMs);
        }

        return ExecuteReverseStraightProfile(
            reverseDistanceM,
            SearchLimits(),
            &targetHeading,
            &targetPosition);
    }

    bool ObserveCellFromSnapshot(
        const MazeMap::CellCoordinates& observedCell,
        MazeMap::Direction observedDirection,
        const SensorSnapshot& snapshot)
    {
        MazeMap::WallState knownWallState = MazeMap::WallState::Unknown;
        if (MazeMap::TryGetKnownMissionStartWallState(observedCell, MazeMap::Up, knownWallState))
        {
            PrimeKnownMissionStartCell();
            AppendStartupTrace("observe_current_cell:used_known_start_cell_topology");
            return true;
        }

        MazeMap::Cell& cell = _maze[observedCell];
        const MazeMap::Direction forwardDirection = observedDirection + MazeMap::Forward;
        const MazeMap::Direction leftDirection = observedDirection + MazeMap::Left90;
        const MazeMap::Direction rightDirection = observedDirection + MazeMap::Right90;
        const bool forwardUnknown = cell.GetWall(forwardDirection) == MazeMap::WallState::Unknown;
        const bool leftUnknown = cell.GetWall(leftDirection) == MazeMap::WallState::Unknown;
        const bool rightUnknown = cell.GetWall(rightDirection) == MazeMap::WallState::Unknown;
        if (!(forwardUnknown || leftUnknown || rightUnknown))
        {
            AppendStartupTrace("observe_current_cell:skipped_known_walls");
            return true;
        }

        if (forwardUnknown)
        {
            MazeMap::WallState observedState = snapshot.frontWall ? MazeMap::Wall : MazeMap::NoWall;
            const char* sensorSource = FrontObservationSourceName(snapshot);
            const char* sensorMode = snapshot.frontWallUsesFallbackDetection ? "fallback" : "differential";
            _maze.SetWall(cell, forwardDirection, observedState);
            if (!LogWallObservationDecision(
                    observedCell,
                    "forward",
                    forwardDirection,
                    observedState,
                    sensorSource,
                    sensorMode,
                    snapshot.frontLeftDistanceM,
                    snapshot.frontRightDistanceM,
                    snapshot.frontLeftWall,
                    snapshot.frontRightWall,
                    snapshot))
            {
                return false;
            }
        }
        if (leftUnknown)
        {
            if (snapshot.leftWallObservationWindowValid)
            {
                const MazeMap::WallState observedState = snapshot.leftWallObservation ? MazeMap::Wall : MazeMap::NoWall;
                _maze.SetWall(cell, leftDirection, observedState);
                if (!LogWallObservationDecision(
                        observedCell,
                        "left",
                        leftDirection,
                        observedState,
                        WallSensorIdName(WallSensorId::SideLeft),
                        nullptr,
                        snapshot.sideLeftDistanceM,
                        NAN,
                        snapshot.leftWallObservation,
                        false,
                        snapshot))
                {
                    return false;
                }
            }
        }
        if (rightUnknown)
        {
            if (snapshot.rightWallObservationWindowValid)
            {
                const MazeMap::WallState observedState = snapshot.rightWallObservation ? MazeMap::Wall : MazeMap::NoWall;
                _maze.SetWall(cell, rightDirection, observedState);
                if (!LogWallObservationDecision(
                        observedCell,
                        "right",
                        rightDirection,
                        observedState,
                        WallSensorIdName(WallSensorId::SideRight),
                        nullptr,
                        snapshot.sideRightDistanceM,
                        NAN,
                        snapshot.rightWallObservation,
                        false,
                        snapshot))
                {
                    return false;
                }
            }
        }
        return true;
    }

    bool ObserveCurrentCellFromSnapshot(const SensorSnapshot& snapshot)
    {
        return ObserveCellFromSnapshot(_currentCell, _currentDirection, snapshot);
    }

    bool ObserveCurrentCell()
    {
        if (!HoldPosition(Config::kObservationSettleMs))
        {
            return false;
        }

        float dtSeconds = 0.0f;
        SensorSnapshot snapshot{};
        if (!TickControl(true, dtSeconds, snapshot))
        {
            return false;
        }
        _drive.Brake();
        return ObserveCurrentCellFromSnapshot(snapshot);
    }

    bool ExploreFullMaze()
    {
        while (!_maze.IsComplete())
        {
            if (_maze.HasFoundGoal() && !_goalPauseComplete)
            {
                if (!EmitMissionControllerLineOrFail("Goal discovered"))
                {
                    return false;
                }
                if (!DriveToGoalAndPause())
                {
                    return false;
                }
                continue;
            }

            MazeMap::Path<PATH_SIZE> path;
            _searchPathFinder.PathToNearestUnknown(_currentCell, _currentDirection, path);

            if (path.GetSize() < 2)
            {
                if (!ObserveCurrentCell())
                {
                    return false;
                }
                _searchPathFinder.PathToNearestUnknown(_currentCell, _currentDirection, path);
                if (path.GetSize() < 2 && !_maze.IsComplete())
                {
                    return Fail("Search path stalled before maze completion");
                }
                continue;
            }

            if (!ExecuteSearchPath(path, true))
            {
                return false;
            }
        }

        return true;
    }

    bool DriveToGoalAndPause()
    {
        while (!IsInGoalCell(_currentCell))
        {
            MazeMap::Path<PATH_SIZE> goalPath;
            _searchPathFinder.PathToGoal(_currentCell, _currentDirection, goalPath);
            if (goalPath.GetSize() < 2)
            {
                return Fail("Unable to drive to goal after detection");
            }
            if (!ExecuteSearchPath(goalPath, false))
            {
                return false;
            }
        }

        if (!EmitMissionControllerLineOrFail("Holding in goal for 2 seconds"))
        {
            return false;
        }
        if (!HoldPosition(Config::kGoalPauseMs))
        {
            return false;
        }
        _goalPauseComplete = true;
        return ObserveCurrentCell();
    }

    bool ReturnToStart()
    {
        const MazeMap::CellCoordinates start(0, 0);
        while (_currentCell != start)
        {
            MazeMap::HalfStepPath<PATH_SIZE * 2> returnHalfStepPath;
            _searchPathFinder.HalfStepPathFromTo(_currentCell, _currentDirection, start, returnHalfStepPath);

            MazeMap::ManeuverPath maneuverPath;
            if (returnHalfStepPath.GetSize() > 1U
                && MazeMap::ManeuverPath::FromHalfStep(returnHalfStepPath, _currentDirectionalLocation, maneuverPath)
                && maneuverPath.GetSize() > 0U)
            {
                char traceLine[112] = {};
                snprintf(
                    traceLine,
                    sizeof(traceLine),
                    "return_to_start:using_search_halfstep_maneuvers,halfsteps=%u,maneuvers=%u",
                    static_cast<unsigned>(returnHalfStepPath.GetSize()),
                    static_cast<unsigned>(maneuverPath.GetSize()));
                AppendStartupTrace(traceLine);

                MazeMap::ManeuverQueue queue(maneuverPath, _currentDirectionalLocation);
                queue.ComputeSpeeds(_speedVehicle, 0.0f, 0.0f);
                const MotionLimits returnLimits = SearchLimits();
                ApplyAsymmetricQueueLimits(queue, returnLimits, 0.0f, 0.0f);
                if (!ExecuteQueuedManeuvers(queue, returnLimits, false))
                {
                    return false;
                }
            }
            else
            {
                MazeMap::Path<PATH_SIZE> path;
                _searchPathFinder.PathFromTo(_currentCell, _currentDirection, start, path);
                if (path.GetSize() < 2)
                {
                    return Fail("Unable to return to start");
                }
                AppendStartupTrace("return_to_start:fallback_search_path");
                if (!ExecuteSearchPath(path, false))
                {
                    return false;
                }
            }
        }

        if (!OrientTo(MazeMap::Up, SearchLimits()))
        {
            return false;
        }

        _currentDirectionalLocation = MazeMap::DirectionalLocation(MazeMap::MazeLocation::CellCenter(_currentCell), _currentDirection);
        return ObserveCurrentCell();
    }

    bool HandleInterRunServiceCycle()
    {
        if (!EmitMissionControllerLineOrFail("Install 34-35 jumper before lifting for tire service"))
        {
            return false;
        }
        _drive.Brake();
        while (!IsInterRunServiceJumperInstalled())
        {
            delay(Config::kInterRunServicePollMs);
        }

        if (!EmitMissionControllerLineOrFail("Service jumper detected; place robot back at start facing up and remove jumper"))
        {
            return false;
        }
        while (IsInterRunServiceJumperInstalled())
        {
            _drive.Brake();
            delay(Config::kInterRunServicePollMs);
        }

        return PrepareForSecondSpeedRun();
    }

    bool PrepareForSecondSpeedRun()
    {
        if (!_sensors.Begin())
        {
            return Fail("Sensor reset failed after inter-run service");
        }

        SnapToStartPose();
        return RunStartupWallCalibration();
    }

    bool FinishSpeedRunAndReturnToStart()
    {
        if (!EmitMissionControllerLineOrFail("Holding at finish for 3 seconds"))
        {
            return false;
        }
        if (!HoldPosition(Config::kSpeedRunFinishPauseMs))
        {
            return false;
        }

        if (!EmitMissionControllerLineOrFail("Returning to start"))
        {
            return false;
        }
        return ReturnToStart();
    }

    bool ExecuteRacingRunCycle()
    {
        SetRacingFanEnabled(true);
        const bool ok = RunSpeedRun() && FinishSpeedRunAndReturnToStart();
        SetRacingFanEnabled(false);
        return ok;
    }

    bool RunSpeedRun()
    {
        MazeMap::ManeuverPath path;
        _speedPathFinder.ManeuverPathToGoal(_currentCell, _currentDirection, path);
        if (path.GetSize() == 0)
        {
            return Fail("ManeuverPathFinder returned an empty path");
        }

        MazeMap::ManeuverQueue queue(path, _currentDirectionalLocation);
        queue.ComputeSpeeds(_speedVehicle, 0.0f, 0.0f);
        ApplyAsymmetricQueueLimits(queue, 0.0f, 0.0f);
        return ExecuteQueuedManeuvers(queue, true);
    }

    bool ExecuteSearchStraightCells(
        MazeMap::Direction direction,
        uint16_t cellCount,
        float entrySpeedMps,
        float cruiseSpeedMps,
        float exitSpeedMps,
        bool snapAtEnd,
        bool observeWhileRolling = false,
        bool* outInterruptedByBoundaryImpact = nullptr)
    {
        (void)snapAtEnd;
        if (outInterruptedByBoundaryImpact != nullptr)
        {
            *outInterruptedByBoundaryImpact = false;
        }

        if (cellCount == 0U)
        {
            return true;
        }

        const MazeMap::CellCoordinates startCell = _currentCell;
        MazeMap::CellCoordinates destination = startCell;
        for (uint16_t i = 0U; i < cellCount; ++i)
        {
            destination = destination >> direction;
        }

        const MazeMap::Vectorf<2> targetHeading = DirectionToUnitVector(direction);
        float targetXMeters = 0.0f;
        float targetYMeters = 0.0f;
        MazeMap::MazeLocation::CellCenter(destination).GetPhysicalLocation(Config::kCellSizeM, targetXMeters, targetYMeters);

        const PoseEstimate startPose = _drive.GetPose();
        float distanceToTargetM = 0.0f;
        if (!MazeMap::TryComputeProjectedDistanceToTargetM(
            startPose.xMeters,
            startPose.yMeters,
            targetXMeters,
            targetYMeters,
            targetHeading.GetX(),
            targetHeading.GetY(),
            distanceToTargetM))
        {
            return Fail("Search straight target distance is invalid");
        }
        if (distanceToTargetM < -Config::kDistanceToleranceM)
        {
            return Fail("Search straight target fell behind the current pose");
        }

        struct BoundaryImpactWatch
        {
            bool armed = false;
            float peakEncoderSpeedMps = 0.0f;
            float baselinePlanarAccelMps2 = 0.0f;
            bool baselinePlanarAccelValid = false;
            bool motionEstablished = false;
            unsigned long motionEstablishedMs = 0UL;
        };

        BoundaryImpactWatch boundaryImpactWatch{};
        MazeMap::CellCoordinates nextBoundaryCell = startCell;
        uint16_t unresolvedBoundaryCount = cellCount;
        const float sideSensorForwardOffsetM =
            (std::max)(_speedVehicle.SideLeft.GetPosition().GetX(), _speedVehicle.SideRight.GetPosition().GetX());
        uint16_t rollingObservationCount = 0U;
        MazeMap::CellCoordinates nextRollingObservationCell = startCell;
        float rollingObservationTriggerTravelM[Config::kSearchRollingObservationSampleCount] = {};
        SensorSnapshot rollingObservationSamples[Config::kSearchRollingObservationSampleCount] = {};
        uint8_t rollingObservationNextSampleIndex = 0U;
        bool rollingObservationPlanInitialized = false;
        if (observeWhileRolling)
        {
            nextRollingObservationCell = nextRollingObservationCell >> direction;
        }

        const auto resetBoundaryImpactWatch = [&boundaryImpactWatch]()
        {
            boundaryImpactWatch = BoundaryImpactWatch{};
        };

        const auto resetRollingObservationPlan = [&]()
        {
            rollingObservationNextSampleIndex = 0U;
            rollingObservationPlanInitialized = false;
            memset(rollingObservationTriggerTravelM, 0, sizeof(rollingObservationTriggerTravelM));
            memset(rollingObservationSamples, 0, sizeof(rollingObservationSamples));
        };

        const auto initializeRollingObservationPlan = [&]() -> bool
        {
            if (!observeWhileRolling || rollingObservationCount >= cellCount)
            {
                return true;
            }
            if (rollingObservationPlanInitialized)
            {
                return true;
            }

            // WARNING: Mapping observation here is intentionally one constant-velocity traversal through the cell.
            // Do not add any parallel traversal mechanism, extra pass, or speed-shape change here. The only permitted
            // refinement is to move the observation timing inside the target region while the chassis keeps the same
            // steady mapping straight.
            for (uint8_t sampleIndex = 0U; sampleIndex < Config::kSearchRollingObservationSampleCount; ++sampleIndex)
            {
                float targetObservationXMeters = 0.0f;
                float targetObservationYMeters = 0.0f;
                if (!MazeMap::TryComputeSideWallObservationSamplePoseM(
                        nextRollingObservationCell,
                        direction,
                        Config::kCellSizeM,
                        Config::kMazeWallThicknessM,
                        sideSensorForwardOffsetM,
                        Config::kSideWallSegmentCenterFraction,
                        sampleIndex,
                        Config::kSearchRollingObservationSampleCount,
                        targetObservationXMeters,
                        targetObservationYMeters))
                {
                    return Fail("Search straight rolling observation sample pose is invalid");
                }

                float triggerTravelM = 0.0f;
                if (!MazeMap::TryComputeProjectedDistanceToTargetM(
                        startPose.xMeters,
                        startPose.yMeters,
                        targetObservationXMeters,
                        targetObservationYMeters,
                        targetHeading.GetX(),
                        targetHeading.GetY(),
                        triggerTravelM))
                {
                    return Fail("Search straight rolling observation sample trigger is invalid");
                }
                if (sampleIndex > 0U &&
                    triggerTravelM < (rollingObservationTriggerTravelM[sampleIndex - 1U] - Config::kDistanceToleranceM))
                {
                    AppendMissionTraceFormatted(
                        "mission_observation_trigger_recovered,cell=(%d,%d),abs=%s,sample=%u,prev_m=%.4f,raw_m=%.4f",
                        nextRollingObservationCell.GetX(),
                        nextRollingObservationCell.GetY(),
                        DirectionName(direction),
                        static_cast<unsigned>(sampleIndex),
                        rollingObservationTriggerTravelM[sampleIndex - 1U],
                        triggerTravelM);
                    triggerTravelM = rollingObservationTriggerTravelM[sampleIndex - 1U];
                }

                rollingObservationTriggerTravelM[sampleIndex] = triggerTravelM;
            }

            rollingObservationPlanInitialized = true;
            return true;
        };

        const auto tryComputeDistanceToBoundaryTouch = [&](
                                                      const MazeMap::CellCoordinates& boundaryCell,
                                                      const PoseEstimate& poseEstimate,
                                                      float& distanceToBoundaryTouchM,
                                                      float& targetCoordinateM,
                                                      CalibrationWall& touchWall) -> bool
        {
            distanceToBoundaryTouchM = 0.0f;
            targetCoordinateM = 0.0f;
            if (!TryComputeWallTouchTargetCoordinateForCellWall(boundaryCell, direction, targetCoordinateM, touchWall))
            {
                return false;
            }

            float targetXM = poseEstimate.xMeters;
            float targetYM = poseEstimate.yMeters;
            switch (touchWall)
            {
            case CalibrationWall::West:
            case CalibrationWall::East:
                targetXM = targetCoordinateM;
                break;
            case CalibrationWall::South:
            case CalibrationWall::North:
                targetYM = targetCoordinateM;
                break;
            default:
                return false;
            }

            return MazeMap::TryComputeProjectedDistanceToTargetM(
                poseEstimate.xMeters,
                poseEstimate.yMeters,
                targetXM,
                targetYM,
                targetHeading.GetX(),
                targetHeading.GetY(),
                distanceToBoundaryTouchM);
        };

        const auto tryObserveRollingCells = [&](float projectedTravelM, const SensorSnapshot& liveSnapshot) -> bool
        {
            if (!observeWhileRolling)
            {
                return true;
            }

            while (rollingObservationCount < cellCount)
            {
                if (!initializeRollingObservationPlan())
                {
                    return false;
                }

                while (rollingObservationNextSampleIndex < Config::kSearchRollingObservationSampleCount &&
                    (projectedTravelM + Config::kDistanceToleranceM) >= rollingObservationTriggerTravelM[rollingObservationNextSampleIndex])
                {
                    rollingObservationSamples[rollingObservationNextSampleIndex] = liveSnapshot;
                    ++rollingObservationNextSampleIndex;
                }

                if (rollingObservationNextSampleIndex < Config::kSearchRollingObservationSampleCount)
                {
                    break;
                }

                SensorSnapshot majoritySnapshot{};
                RollingObservationVoteSummary voteSummary{};
                if (!BuildMajorityObservationSnapshot(
                        rollingObservationSamples,
                        Config::kSearchRollingObservationSampleCount,
                        majoritySnapshot,
                        voteSummary))
                {
                    return Fail("Search straight rolling observation majority snapshot is invalid");
                }

                AppendMissionTraceFormatted(
                    "mission_observation_timed,cell=(%d,%d),abs=%s,samples=%u,start_m=%.4f,end_m=%.4f,travel_m=%.4f,front_votes=%u,left_valid=%u,left_votes=%u,right_valid=%u,right_votes=%u",
                    nextRollingObservationCell.GetX(),
                    nextRollingObservationCell.GetY(),
                    DirectionName(direction),
                    static_cast<unsigned>(voteSummary.sampleCount),
                    rollingObservationTriggerTravelM[0],
                    rollingObservationTriggerTravelM[Config::kSearchRollingObservationSampleCount - 1U],
                    projectedTravelM,
                    static_cast<unsigned>(voteSummary.frontWallVotes),
                    static_cast<unsigned>(voteSummary.leftWindowValidVotes),
                    static_cast<unsigned>(voteSummary.leftWallVotes),
                    static_cast<unsigned>(voteSummary.rightWindowValidVotes),
                    static_cast<unsigned>(voteSummary.rightWallVotes));
                if (!ObserveCellFromSnapshot(nextRollingObservationCell, direction, majoritySnapshot))
                {
                    return false;
                }

                ++rollingObservationCount;
                if (rollingObservationCount < cellCount)
                {
                    nextRollingObservationCell = nextRollingObservationCell >> direction;
                }
                resetRollingObservationPlan();
            }

            return true;
        };

        const MotionLimits searchLimits = SearchLimits();
        const float startDistanceM = _drive.GetAverageDistanceMeters();
        float commandedSpeedMps = (std::max)(entrySpeedMps, 0.0f);
        const unsigned long expectedCompletionDeadlineMs = millis() + static_cast<unsigned long>(2000.0f + (4000.0f * distanceToTargetM));
        EncoderProgressWatchdog translationWatchdog{};
        translationWatchdog.Reset(0.0f, millis());
        bool stallLogged = false;
        bool durationLogged = false;

        while (true)
        {
            float dtSeconds = 0.0f;
            SensorSnapshot snapshot{};
            if (!TickControl(false, dtSeconds, snapshot))
            {
                return false;
            }

            const PoseEstimate& livePose = _drive.GetPose();
            const DriveTelemetry telemetry = _drive.GetTelemetry();
            const float encoderSpeedMps = MazeMap::ComputeAverageEncoderAbsSpeedMps(
                telemetry.leftVelocityMps,
                telemetry.rightVelocityMps);
            const float traveledM = std::fabs(_drive.GetAverageDistanceMeters() - startDistanceM);

            float projectedRemainingM = 0.0f;
            if (!MazeMap::TryComputeProjectedDistanceToTargetM(
                    livePose.xMeters,
                    livePose.yMeters,
                    targetXMeters,
                    targetYMeters,
                    targetHeading.GetX(),
                    targetHeading.GetY(),
                    projectedRemainingM))
            {
                return Fail("Search straight target projection is invalid");
            }
            const float remainingM = (std::max)(0.0f, projectedRemainingM);
            const float projectedTravelM = (std::clamp)(distanceToTargetM - projectedRemainingM, 0.0f, distanceToTargetM);

            if (Config::kEnableMissionBoundaryImpactWallDetection && unresolvedBoundaryCount > 0U)
            {
                float distanceToBoundaryTouchM = 0.0f;
                float boundaryTargetCoordinateM = 0.0f;
                CalibrationWall boundaryTouchWall = CalibrationWall::North;
                if (!tryComputeDistanceToBoundaryTouch(
                        nextBoundaryCell,
                        livePose,
                        distanceToBoundaryTouchM,
                        boundaryTargetCoordinateM,
                        boundaryTouchWall))
                {
                    AppendMissionTraceFormatted(
                        "mission_boundary_watch_disable,cell=(%d,%d),abs=%s,reason=target_unavailable",
                        nextBoundaryCell.GetX(),
                        nextBoundaryCell.GetY(),
                        DirectionName(direction));
                    unresolvedBoundaryCount = 0U;
                    resetBoundaryImpactWatch();
                }
                else
                {
                    if (!boundaryImpactWatch.armed &&
                        MazeMap::ShouldArmBoundaryImpactWatch(
                            distanceToBoundaryTouchM,
                            Config::kSearchBoundaryImpactArmDistanceM))
                    {
                        boundaryImpactWatch.armed = true;
                        boundaryImpactWatch.baselinePlanarAccelMps2 = snapshot.planarAccelMps2;
                        boundaryImpactWatch.baselinePlanarAccelValid = true;
                        AppendMissionTraceFormatted(
                            "mission_boundary_watch_arm,cell=(%d,%d),abs=%s,target=%.4f,distance_to_touch_m=%.4f",
                            nextBoundaryCell.GetX(),
                            nextBoundaryCell.GetY(),
                            DirectionName(direction),
                            boundaryTargetCoordinateM,
                            distanceToBoundaryTouchM);
                    }

                    if (boundaryImpactWatch.armed)
                    {
                        boundaryImpactWatch.peakEncoderSpeedMps =
                            (std::max)(boundaryImpactWatch.peakEncoderSpeedMps, encoderSpeedMps);
                        if (!boundaryImpactWatch.motionEstablished &&
                            MazeMap::IsWallTapMotionEstablished(
                                boundaryImpactWatch.peakEncoderSpeedMps,
                                traveledM,
                                Config::kFrontWallTapMinimumMotionSpeedMps,
                                Config::kFrontWallTapMinimumMotionDistanceM))
                        {
                            boundaryImpactWatch.motionEstablished = true;
                            boundaryImpactWatch.motionEstablishedMs = millis();
                        }

                        const bool encoderImpactDetected =
                            boundaryImpactWatch.motionEstablished &&
                            MazeMap::HasSharpEncoderVelocityDecline(
                                boundaryImpactWatch.peakEncoderSpeedMps,
                                encoderSpeedMps,
                                Config::kFrontWallTapMinimumPeakEncoderSpeedMps,
                                Config::kFrontWallTapMaximumCurrentPeakRatio,
                                Config::kFrontWallTapMinimumEncoderDropMps);
                        const bool accelImpactDetected =
                            boundaryImpactWatch.motionEstablished &&
                            ((millis() - boundaryImpactWatch.motionEstablishedMs) >= Config::kFrontWallTapAccelArmDelayMs) &&
                            MazeMap::HasPlanarAccelContactSpike(
                                boundaryImpactWatch.baselinePlanarAccelMps2,
                                snapshot.planarAccelMps2,
                                Config::kFrontWallTapPlanarAccelSpikeMps2);
                        if (encoderImpactDetected || accelImpactDetected)
                        {
                            _drive.Brake();
                            if (!HoldBrakedUntilDriveSettles(
                                    nullptr,
                                    Config::kMotionSettleHoldMs,
                                    0U))
                            {
                                return false;
                            }

                            _maze.SetWall(_maze[nextBoundaryCell], direction, MazeMap::Wall);
                            _currentCell = nextBoundaryCell;
                            _currentDirection = direction;
                            _currentDirectionalLocation = MazeMap::DirectionalLocation(
                                MazeMap::MazeLocation::CellCenter(_currentCell),
                                _currentDirection);
                            AppendMissionTraceFormatted(
                                "mission_boundary_impact,cell=(%d,%d),abs=%s,target=%.4f,enc_v_mps=%.4f,peak_enc_v_mps=%.4f,planar_accel_mps2=%.4f",
                                _currentCell.GetX(),
                                _currentCell.GetY(),
                                DirectionName(direction),
                                boundaryTargetCoordinateM,
                                encoderSpeedMps,
                                boundaryImpactWatch.peakEncoderSpeedMps,
                                snapshot.planarAccelMps2);
                            if (!SnapPoseCoordinateOnConfirmedWallContact(boundaryTouchWall, boundaryTargetCoordinateM))
                            {
                                return false;
                            }
                            if (!BackOffToCurrentCellCenterAlongCurrentHeading())
                            {
                                return false;
                            }

                            if (outInterruptedByBoundaryImpact != nullptr)
                            {
                                *outInterruptedByBoundaryImpact = true;
                            }
                            return true;
                        }
                    }

                    if (MazeMap::HasClearedBoundaryWithoutImpact(
                            distanceToBoundaryTouchM,
                            Config::kSearchBoundaryOpenConfirmMarginM))
                    {
                        _maze.SetWall(_maze[nextBoundaryCell], direction, MazeMap::NoWall);
                        AppendMissionTraceFormatted(
                            "mission_boundary_clear,cell=(%d,%d),abs=%s,target=%.4f,distance_to_touch_m=%.4f",
                            nextBoundaryCell.GetX(),
                            nextBoundaryCell.GetY(),
                            DirectionName(direction),
                            boundaryTargetCoordinateM,
                            distanceToBoundaryTouchM);
                        nextBoundaryCell = nextBoundaryCell >> direction;
                        --unresolvedBoundaryCount;
                        resetBoundaryImpactWatch();
                    }
                }
            }

            if (!tryObserveRollingCells(projectedTravelM, snapshot))
            {
                return false;
            }

            const bool stoppingAtEndpoint = exitSpeedMps <= 0.05f;
            const bool terminalReached =
                stoppingAtEndpoint ?
                ((remainingM <= Config::kDistanceToleranceM) && IsDriveMotionSettled()) :
                ((remainingM <= Config::kDistanceToleranceM) && (std::fabs(_drive.GetPose().linearSpeedMps - exitSpeedMps) <= Config::kSpeedToleranceMps));
            if (terminalReached)
            {
                _drive.Brake();
                break;
            }

            const unsigned long nowMs = millis();
            if (!stallLogged && translationWatchdog.Stalled(traveledM, commandedSpeedMps, remainingM, nowMs))
            {
                stallLogged = true;
                AppendMissionTraceFormatted(
                    "mission_motion_watchdog,mode=search_straight,reason=encoder_stall,cell=(%d,%d),traveled_m=%.4f,remaining_m=%.4f,cmd_v_mps=%.4f",
                    _currentCell.GetX(),
                    _currentCell.GetY(),
                    traveledM,
                    remainingM,
                    commandedSpeedMps);
            }
            if (!durationLogged && static_cast<long>(expectedCompletionDeadlineMs - nowMs) <= 0)
            {
                durationLogged = true;
                AppendMissionTraceFormatted(
                    "mission_motion_watchdog,mode=search_straight,reason=elapsed_budget_exceeded,cell=(%d,%d),traveled_m=%.4f,remaining_m=%.4f,cmd_v_mps=%.4f",
                    _currentCell.GetX(),
                    _currentCell.GetY(),
                    traveledM,
                    remainingM,
                    commandedSpeedMps);
            }

            const float accelLimitedSpeedMps = (std::min)(cruiseSpeedMps, commandedSpeedMps + (searchLimits.accelMps2 * dtSeconds));
            const float decelLimitedSpeedMps = ReachableSpeedWithBoundary(exitSpeedMps, remainingM, searchLimits.decelMps2);
            commandedSpeedMps = (std::min)(accelLimitedSpeedMps, decelLimitedSpeedMps);

            float wallOmegaRadps = 0.0f;
            float signalCorridorErrorM = 0.0f;
            if (TryComputeStraightWallCenterErrorM(
                    gWallDistanceCalibration,
                    snapshot.sideLeftDifferentialLight,
                    snapshot.leftWall,
                    snapshot.sideRightDifferentialLight,
                    snapshot.rightWall,
                    signalCorridorErrorM))
            {
                wallOmegaRadps += Config::kWallCenterGain * signalCorridorErrorM;
            }
            else
            {
                wallOmegaRadps += Config::kWallCenterGain * snapshot.corridorErrorM;
            }
            if (snapshot.frontWall && remainingM < 0.07f)
            {
                wallOmegaRadps += Config::kFrontSkewGain * snapshot.frontSkewM;
            }

            const float headingErrorRad = HeadingErrorRad(targetHeading, _drive.GetPose().headingUnit);
            float angularCommandRadps = (Config::kStraightHeadingKp * headingErrorRad) - (Config::kStraightYawD * _drive.GetPose().angularSpeedRadps) + wallOmegaRadps;
            angularCommandRadps = (std::clamp)(angularCommandRadps, -searchLimits.maxAngularSpeedRadps, searchLimits.maxAngularSpeedRadps);
            _drive.CommandVelocity(commandedSpeedMps, angularCommandRadps, dtSeconds);
        }

        if (exitSpeedMps <= 0.05f)
        {
            if (!HoldBrakedUntilDriveSettles(nullptr, Config::kMotionSettleHoldMs, 0U))
            {
                return false;
            }
        }

        _currentCell = destination;
        _currentDirectionalLocation = MazeMap::DirectionalLocation(MazeMap::MazeLocation::CellCenter(_currentCell), _currentDirection);
        return true;
    }

    bool ExecuteSearchPath(const MazeMap::Path<PATH_SIZE>& path, bool observeFinalCell)
    {
        if (path.GetSize() < 2)
        {
            return observeFinalCell ? ObserveCurrentCell() : true;
        }

        const MotionLimits searchLimits = SearchLimits();
        const float cautiousCruiseSpeedMps = SearchUnmappedCruiseSpeedMps();
        uint16_t pathIndex = 1U;

        while (pathIndex < path.GetSize())
        {
            const MazeMap::SearchStraightPlan plan = MazeMap::PlanSearchStraightSegment(_maze, path, pathIndex);
            if (plan.direction == MazeMap::None || plan.TotalCellCount() == 0U)
            {
                return Fail("Search path contained an invalid segment");
            }

            if (!OrientTo(plan.direction, searchLimits))
            {
                return false;
            }

            float rollingEntrySpeedMps = 0.0f;
            if (plan.fullSpeedCellCount > 0U)
            {
                const float exitSpeedMps = (plan.cautiousCellCount > 0U) ? cautiousCruiseSpeedMps : 0.0f;
                bool interruptedByBoundaryImpact = false;
                if (!ExecuteSearchStraightCells(
                    plan.direction,
                    plan.fullSpeedCellCount,
                    0.0f,
                    searchLimits.maxSpeedMps,
                    exitSpeedMps,
                    plan.cautiousCellCount == 0U,
                    false,
                    &interruptedByBoundaryImpact))
                {
                    return false;
                }
                if (interruptedByBoundaryImpact)
                {
                    return ObserveCurrentCell();
                }
                rollingEntrySpeedMps = exitSpeedMps;
            }

            if (plan.cautiousCellCount > 0U)
            {
                if (!observeFinalCell)
                {
                    const float cautiousEntrySpeedMps = (plan.fullSpeedCellCount > 0U) ? cautiousCruiseSpeedMps : 0.0f;
                    bool interruptedByBoundaryImpact = false;
                    if (!ExecuteSearchStraightCells(
                            plan.direction,
                            plan.cautiousCellCount,
                            cautiousEntrySpeedMps,
                            cautiousCruiseSpeedMps,
                            0.0f,
                            true,
                            false,
                            &interruptedByBoundaryImpact))
                    {
                        return false;
                    }
                    if (interruptedByBoundaryImpact)
                    {
                        return ObserveCurrentCell();
                    }
                    pathIndex = static_cast<uint16_t>(plan.segmentEndIndex + 1U);
                    continue;
                }

                float cautiousEntrySpeedMps = (plan.fullSpeedCellCount > 0U) ? rollingEntrySpeedMps : 0.0f;
                while (true)
                {
                    bool interruptedByBoundaryImpact = false;
                    if (!ExecuteSearchStraightCells(
                            plan.direction,
                            1U,
                            cautiousEntrySpeedMps,
                            cautiousCruiseSpeedMps,
                            cautiousCruiseSpeedMps,
                            false,
                            true,
                            &interruptedByBoundaryImpact))
                    {
                        return false;
                    }
                    if (interruptedByBoundaryImpact)
                    {
                        return ObserveCurrentCell();
                    }

                    cautiousEntrySpeedMps = cautiousCruiseSpeedMps;
                    MazeMap::Path<PATH_SIZE> continuingPath;
                    _searchPathFinder.PathToNearestUnknown(_currentCell, _currentDirection, continuingPath);
                    if (continuingPath.GetSize() < 2U)
                    {
                        _drive.Brake();
                        if (!HoldBrakedUntilDriveSettles(nullptr, Config::kMotionSettleHoldMs, 0U))
                        {
                            return false;
                        }
                        return true;
                    }

                    const MazeMap::SearchStraightPlan nextPlan = MazeMap::PlanSearchStraightSegment(_maze, continuingPath, 1U);
                    if (nextPlan.direction == MazeMap::None || nextPlan.TotalCellCount() == 0U)
                    {
                        return Fail("Search path contained an invalid continuation");
                    }

                    if (nextPlan.direction != plan.direction)
                    {
                        _drive.Brake();
                        if (!HoldBrakedUntilDriveSettles(nullptr, Config::kMotionSettleHoldMs, 0U))
                        {
                            return false;
                        }
                        return true;
                    }

                    if (nextPlan.fullSpeedCellCount > 0U)
                    {
                        const float exitSpeedMps = (nextPlan.cautiousCellCount > 0U) ? cautiousCruiseSpeedMps : 0.0f;
                        bool interruptedByBoundaryImpact = false;
                        if (!ExecuteSearchStraightCells(
                                plan.direction,
                                nextPlan.fullSpeedCellCount,
                                cautiousEntrySpeedMps,
                                searchLimits.maxSpeedMps,
                                exitSpeedMps,
                                nextPlan.cautiousCellCount == 0U,
                                false,
                                &interruptedByBoundaryImpact))
                        {
                            return false;
                        }
                        if (interruptedByBoundaryImpact)
                        {
                            return ObserveCurrentCell();
                        }

                        if (nextPlan.cautiousCellCount == 0U)
                        {
                            return observeFinalCell ? ObserveCurrentCell() : true;
                        }

                        cautiousEntrySpeedMps = exitSpeedMps;
                    }
                }
            }

            pathIndex = static_cast<uint16_t>(plan.segmentEndIndex + 1U);
        }

        return observeFinalCell ? ObserveCurrentCell() : true;
    }

    bool TryComputeWallGroundedCorridorCoordinateM(const SensorSnapshot& snapshot, float& coordinateM, bool& correctsXAxis) const
    {
        coordinateM = 0.0f;
        correctsXAxis = false;
        switch (_currentDirection)
        {
        case MazeMap::Up:
        case MazeMap::Down:
            correctsXAxis = true;
            break;
        case MazeMap::Left:
        case MazeMap::Right:
            correctsXAxis = false;
            break;
        default:
            return false;
        }

        const MazeMap::Direction leftWallDirection = _currentDirection + MazeMap::Left90;
        const MazeMap::Direction rightWallDirection = _currentDirection + MazeMap::Right90;
        float leftCoordinateM = 0.0f;
        float rightCoordinateM = 0.0f;
        bool haveLeftCoordinate = false;
        bool haveRightCoordinate = false;

        if (snapshot.leftWall)
        {
            haveLeftCoordinate = TryComputePoseAxisFromObservedWall(
                _drive.GetPose(),
                _speedVehicle.SideLeft,
                snapshot.sideLeftDistanceM,
                _currentCell,
                leftWallDirection,
                leftCoordinateM);
        }

        if (snapshot.rightWall)
        {
            haveRightCoordinate = TryComputePoseAxisFromObservedWall(
                _drive.GetPose(),
                _speedVehicle.SideRight,
                snapshot.sideRightDistanceM,
                _currentCell,
                rightWallDirection,
                rightCoordinateM);
        }

        if (!haveLeftCoordinate && !haveRightCoordinate)
        {
            return false;
        }

        coordinateM = haveLeftCoordinate && haveRightCoordinate ?
            (0.5f * (leftCoordinateM + rightCoordinateM)) :
            (haveLeftCoordinate ? leftCoordinateM : rightCoordinateM);
        return std::isfinite(coordinateM);
    }

    bool TryComputeWallGroundedCorridorErrorM(const SensorSnapshot& snapshot, float& corridorErrorM) const
    {
        corridorErrorM = 0.0f;

        float corridorCoordinateM = 0.0f;
        bool correctsXAxis = false;
        if (!TryComputeWallGroundedCorridorCoordinateM(snapshot, corridorCoordinateM, correctsXAxis))
        {
            return false;
        }

        float centerXM = 0.0f;
        float centerYM = 0.0f;
        if (!TryGetCellCenterMeters(_currentCell, centerXM, centerYM))
        {
            return false;
        }

        const float errorXM = correctsXAxis ? (corridorCoordinateM - centerXM) : 0.0f;
        const float errorYM = correctsXAxis ? 0.0f : (corridorCoordinateM - centerYM);
        const MazeMap::Vectorf<2> heading = DirectionToUnitVector(_currentDirection);
        corridorErrorM = (heading.GetY() * errorXM) - (heading.GetX() * errorYM);
        return std::isfinite(corridorErrorM);
    }

    bool ApplyWallGroundedCorridorPoseCorrection(const SensorSnapshot& snapshot)
    {
        float corridorCoordinateM = 0.0f;
        bool correctsXAxis = false;
        if (!TryComputeWallGroundedCorridorCoordinateM(snapshot, corridorCoordinateM, correctsXAxis))
        {
            return false;
        }

        const PoseEstimate& pose = _drive.GetPose();
        const float priorCoordinateM = correctsXAxis ? pose.xMeters : pose.yMeters;
        if (correctsXAxis)
        {
            _drive.SetPoseXMeters(corridorCoordinateM);
        }
        else
        {
            _drive.SetPoseYMeters(corridorCoordinateM);
        }

        if (std::fabs(corridorCoordinateM - priorCoordinateM) >= 0.001f)
        {
            char traceLine[160] = {};
            snprintf(
                traceLine,
                sizeof(traceLine),
                "mission_pose_snap,axis=%s,from=%.4f,to=%.4f,cell=(%d,%d)",
                correctsXAxis ? "x" : "y",
                priorCoordinateM,
                corridorCoordinateM,
                _currentCell.GetX(),
                _currentCell.GetY());
            AppendStartupTrace(traceLine);
        }
        return true;
    }

    bool TryComputeTurnWallEdgeCoordinateM(
        MazeMap::Direction targetDirection,
        const SensorSnapshot& snapshot,
        const MazeMap::TurnWallEdgeTracker& edgeTracker,
        float& coordinateM,
        bool& correctsXAxis,
        const char*& sourceName) const
    {
        coordinateM = 0.0f;
        correctsXAxis = false;
        sourceName = "none";

        // The center post in the 2x2 goal can create side-sensor rising edges that do not correspond to a usable
        // corridor wall boundary, so suppress turn-edge grounding inside the goal area.
        if (IsInGoalCell(_currentCell))
        {
            sourceName = "goal_suppressed";
            return false;
        }

        switch (targetDirection)
        {
        case MazeMap::Up:
        case MazeMap::Down:
            correctsXAxis = true;
            break;
        case MazeMap::Left:
        case MazeMap::Right:
            correctsXAxis = false;
            break;
        default:
            return false;
        }

        const MazeMap::Cell& cell = _maze[_currentCell];
        const MazeMap::Direction leftWallDirection = targetDirection + MazeMap::Left90;
        const MazeMap::Direction rightWallDirection = targetDirection + MazeMap::Right90;
        const PoseEstimate& pose = _drive.GetPose();
        float leftCoordinateM = 0.0f;
        float rightCoordinateM = 0.0f;
        bool haveLeftCoordinate =
            edgeTracker.leftWallRose &&
            snapshot.leftWall &&
            (cell.GetWall(leftWallDirection) == MazeMap::Wall) &&
            TryComputePoseAxisFromObservedWall(
                pose,
                _speedVehicle.SideLeft,
                snapshot.sideLeftDistanceM,
                _currentCell,
                leftWallDirection,
                leftCoordinateM);
        bool haveRightCoordinate =
            edgeTracker.rightWallRose &&
            snapshot.rightWall &&
            (cell.GetWall(rightWallDirection) == MazeMap::Wall) &&
            TryComputePoseAxisFromObservedWall(
                pose,
                _speedVehicle.SideRight,
                snapshot.sideRightDistanceM,
                _currentCell,
                rightWallDirection,
                rightCoordinateM);

        if (!haveLeftCoordinate && !haveRightCoordinate)
        {
            return false;
        }

        if (haveLeftCoordinate && haveRightCoordinate)
        {
            coordinateM = 0.5f * (leftCoordinateM + rightCoordinateM);
            sourceName = "left+right";
        }
        else if (haveLeftCoordinate)
        {
            coordinateM = leftCoordinateM;
            sourceName = "left";
        }
        else
        {
            coordinateM = rightCoordinateM;
            sourceName = "right";
        }

        return std::isfinite(coordinateM);
    }

    bool ApplyTurnWallEdgePoseCorrection(
        MazeMap::Direction targetDirection,
        const SensorSnapshot& snapshot,
        const MazeMap::TurnWallEdgeTracker& edgeTracker)
    {
        float correctedCoordinateM = 0.0f;
        bool correctsXAxis = false;
        const char* sourceName = "none";
        if (!TryComputeTurnWallEdgeCoordinateM(
                targetDirection,
                snapshot,
                edgeTracker,
                correctedCoordinateM,
                correctsXAxis,
                sourceName))
        {
            if (IsInGoalCell(_currentCell) && (edgeTracker.leftWallRose || edgeTracker.rightWallRose))
            {
                AppendMissionTraceFormatted(
                    "mission_turn_edge_snap:suppressed,cell=(%d,%d),dir=%s,left_rose=%u,right_rose=%u",
                    _currentCell.GetX(),
                    _currentCell.GetY(),
                    DirectionName(targetDirection),
                    edgeTracker.leftWallRose ? 1U : 0U,
                    edgeTracker.rightWallRose ? 1U : 0U);
            }
            return true;
        }

        const PoseEstimate& pose = _drive.GetPose();
        const float priorCoordinateM = correctsXAxis ? pose.xMeters : pose.yMeters;
        if (correctsXAxis)
        {
            _drive.SetPoseXMeters(correctedCoordinateM);
        }
        else
        {
            _drive.SetPoseYMeters(correctedCoordinateM);
        }

        char traceLine[192] = {};
        snprintf(
            traceLine,
            sizeof(traceLine),
            "mission_turn_edge_snap,axis=%s,from=%.4f,to=%.4f,dir=%s,source=%s,cell=(%d,%d)",
            correctsXAxis ? "x" : "y",
            priorCoordinateM,
            correctedCoordinateM,
            DirectionName(targetDirection),
            sourceName,
            _currentCell.GetX(),
            _currentCell.GetY());
        AppendStartupTrace(traceLine);
        (void)WriteMissionTraceLineBestEffort(traceLine, "mission_text_logging:turn_edge_snap_write_failed");

        return true;
    }

    bool OrientTo(MazeMap::Direction targetDirection, const MotionLimits& limits)
    {
        const MazeMap::RelativeDirection relative = targetDirection - _currentDirection;
        if (relative == MazeMap::Forward)
        {
            return true;
        }

        if (RelativeToInPlaceCode(relative) == MazeMap::MC_NONE)
        {
            return Fail("Unsupported in-place turn requested");
        }

        const float targetYawRad = DirectionToYawRad(targetDirection);
        float angleRad = 0.0f;
        if (!MazeMap::TryComputeSignedTurnAngleRad(_drive.GetPose().yawRad, targetYawRad, angleRad))
        {
            return Fail("Mission turn angle is invalid");
        }
        char traceLine[160] = {};
        snprintf(
            traceLine,
            sizeof(traceLine),
            "mission_turn:begin,current_deg=%.2f,target_deg=%.2f,angle_deg=%.2f,cell=(%d,%d)",
            RAD_TO_DEG * _drive.GetPose().yawRad,
            RAD_TO_DEG * targetYawRad,
            RAD_TO_DEG * angleRad,
            _currentCell.GetX(),
            _currentCell.GetY());
        AppendStartupTrace(traceLine);
        MazeMap::TurnWallEdgeTracker wallEdgeTracker{};
        if (!ExecuteTurnProfile(angleRad, limits, &wallEdgeTracker))
        {
            return false;
        }

        const SensorSnapshot settledSnapshot = _sensors.Capture(true, _drive.GetPose());
        if (!ApplyTurnWallEdgePoseCorrection(targetDirection, settledSnapshot, wallEdgeTracker))
        {
            return false;
        }

        snprintf(
            traceLine,
            sizeof(traceLine),
            "mission_turn:end,x=%.4f,y=%.4f,yaw_deg=%.2f,v=%.4f,w=%.4f",
            _drive.GetPose().xMeters,
            _drive.GetPose().yMeters,
            RAD_TO_DEG * _drive.GetPose().yawRad,
            _drive.GetPose().linearSpeedMps,
            _drive.GetPose().angularSpeedRadps);
        AppendStartupTrace(traceLine);
        _currentDirection = targetDirection;
        _currentDirectionalLocation = MazeMap::DirectionalLocation(MazeMap::MazeLocation::CellCenter(_currentCell), _currentDirection);
        return true;
    }

    bool ExecuteStraightProfile(
        float distanceM,
        float entrySpeed,
        float cruiseSpeed,
        float exitSpeed,
        const MotionLimits& limits,
        bool useWallCentering,
        const MazeMap::Vectorf<2>* targetHeadingOverride = nullptr,
        const MazeMap::Vectorf<2>* targetPositionOverride = nullptr)
    {
        const float startDistanceM = _drive.GetAverageDistanceMeters();
        const MazeMap::Vectorf<2> targetHeading =
            (targetHeadingOverride != nullptr) ?
            *targetHeadingOverride :
            _drive.GetPose().headingUnit;
        const bool diagonalHeading = IsApproximatelyDiagonalHeadingUnit(targetHeading);
        float commandedSpeedMps = (std::max)(entrySpeed, 0.0f);
        const unsigned long expectedCompletionDeadlineMs = millis() + static_cast<unsigned long>(2000.0f + (4000.0f * distanceM));
        EncoderProgressWatchdog translationWatchdog{};
        translationWatchdog.Reset(0.0f, millis());
        bool stallLogged = false;
        bool durationLogged = false;

        while (true)
        {
            float dtSeconds = 0.0f;
            SensorSnapshot snapshot{};
            if (!TickControl(false, dtSeconds, snapshot))
            {
                return false;
            }

            const float traveledM = std::fabs(_drive.GetAverageDistanceMeters() - startDistanceM);
            float remainingM = (std::max)(0.0f, distanceM - traveledM);
            if (targetPositionOverride != nullptr)
            {
                const PoseEstimate& pose = _drive.GetPose();
                float projectedRemainingM = 0.0f;
                if (!MazeMap::TryComputeProjectedDistanceToTargetM(
                    pose.xMeters,
                    pose.yMeters,
                    targetPositionOverride->GetX(),
                    targetPositionOverride->GetY(),
                    targetHeading.GetX(),
                    targetHeading.GetY(),
                    projectedRemainingM))
                {
                    return Fail("Straight target projection is invalid");
                }
                remainingM = (std::max)(0.0f, projectedRemainingM);
            }
            const bool stoppingAtEndpoint = exitSpeed <= 0.05f;
            const bool terminalReached =
                stoppingAtEndpoint ?
                ((remainingM <= Config::kDistanceToleranceM) && IsDriveMotionSettled()) :
                ((remainingM <= Config::kDistanceToleranceM) && (std::fabs(_drive.GetPose().linearSpeedMps - exitSpeed) <= Config::kSpeedToleranceMps));
            if (terminalReached)
            {
                _drive.Brake();
                break;
            }
            const unsigned long nowMs = millis();
            if (!stallLogged && translationWatchdog.Stalled(traveledM, commandedSpeedMps, remainingM, nowMs))
            {
                stallLogged = true;
                AppendMissionTraceFormatted(
                    "mission_motion_watchdog,mode=straight,reason=encoder_stall,cell=(%d,%d),traveled_m=%.4f,remaining_m=%.4f,cmd_v_mps=%.4f",
                    _currentCell.GetX(),
                    _currentCell.GetY(),
                    traveledM,
                    remainingM,
                    commandedSpeedMps);
            }
            if (!durationLogged && static_cast<long>(expectedCompletionDeadlineMs - nowMs) <= 0)
            {
                durationLogged = true;
                AppendMissionTraceFormatted(
                    "mission_motion_watchdog,mode=straight,reason=elapsed_budget_exceeded,cell=(%d,%d),traveled_m=%.4f,remaining_m=%.4f,cmd_v_mps=%.4f",
                    _currentCell.GetX(),
                    _currentCell.GetY(),
                    traveledM,
                    remainingM,
                    commandedSpeedMps);
            }

            const float accelLimitedSpeedMps = (std::min)(cruiseSpeed, commandedSpeedMps + (limits.accelMps2 * dtSeconds));
            const float decelLimitedSpeedMps = ReachableSpeedWithBoundary(exitSpeed, remainingM, limits.decelMps2);
            commandedSpeedMps = (std::min)(accelLimitedSpeedMps, decelLimitedSpeedMps);

            float wallOmegaRadps = 0.0f;
            if (useWallCentering)
            {
                if (diagonalHeading)
                {
                    wallOmegaRadps += ComputeDiagonalWallCenterOmegaRadps(
                        gWallDistanceCalibration,
                        snapshot.sideLeftDifferentialLight,
                        snapshot.sideRightDifferentialLight);
                }
                else
                {
                    float signalCorridorErrorM = 0.0f;
                    if (TryComputeStraightWallCenterErrorM(
                            gWallDistanceCalibration,
                            snapshot.sideLeftDifferentialLight,
                            snapshot.leftWall,
                            snapshot.sideRightDifferentialLight,
                            snapshot.rightWall,
                            signalCorridorErrorM))
                    {
                        wallOmegaRadps += Config::kWallCenterGain * signalCorridorErrorM;
                    }
                    else
                    {
                        wallOmegaRadps += Config::kWallCenterGain * snapshot.corridorErrorM;
                    }
                    if (snapshot.frontWall && remainingM < 0.07f)
                    {
                        wallOmegaRadps += Config::kFrontSkewGain * snapshot.frontSkewM;
                    }
                }
            }

            const float headingErrorRad = HeadingErrorRad(targetHeading, _drive.GetPose().headingUnit);
            float angularCommandRadps = (Config::kStraightHeadingKp * headingErrorRad) - (Config::kStraightYawD * _drive.GetPose().angularSpeedRadps) + wallOmegaRadps;
            angularCommandRadps = (std::clamp)(angularCommandRadps, -limits.maxAngularSpeedRadps, limits.maxAngularSpeedRadps);
            _drive.CommandVelocity(commandedSpeedMps, angularCommandRadps, dtSeconds);
        }

        if (exitSpeed <= 0.05f)
        {
            if (!HoldBrakedUntilDriveSettles(nullptr, Config::kMotionSettleHoldMs, 0U))
            {
                return false;
            }
        }
        return true;
    }

    bool ExecuteTurnProfile(
        float angleRad,
        const MotionLimits& limits,
        MazeMap::TurnWallEdgeTracker* wallEdgeTracker = nullptr)
    {
        (void)limits;
        const float targetYawRad = WrapAngleRad(_drive.GetPose().yawRad + angleRad);
        const MazeMap::InPlaceTurnProfile turnProfile = BuildSharedInPlaceTurnProfile(_speedVehicle);
        float commandedOmegaRadps = 0.0f;
        const unsigned long expectedCompletionDeadlineMs = millis() + 2500UL;
        bool durationLogged = false;

        while (true)
        {
            float dtSeconds = 0.0f;
            SensorSnapshot snapshot{};
            if (!TickControl(false, dtSeconds, snapshot))
            {
                return false;
            }
            if (wallEdgeTracker != nullptr)
            {
                MazeMap::ObserveTurnWallStates(*wallEdgeTracker, snapshot.leftWall, snapshot.rightWall);
            }

            const float errorRad = AngleErrorRad(targetYawRad, _drive.GetPose().yawRad);
            if (MazeMap::IsInPlaceTurnComplete(errorRad, _drive.GetPose().angularSpeedRadps, turnProfile))
            {
                _drive.Brake();
                break;
            }
            if (!durationLogged && static_cast<long>(expectedCompletionDeadlineMs - millis()) <= 0)
            {
                durationLogged = true;
                AppendMissionTraceFormatted(
                    "mission_motion_watchdog,mode=turn,reason=elapsed_budget_exceeded,cell=(%d,%d),yaw_err_deg=%.2f,w_radps=%.4f",
                    _currentCell.GetX(),
                    _currentCell.GetY(),
                    RAD_TO_DEG * errorRad,
                    _drive.GetPose().angularSpeedRadps);
            }

            float angularCommandRadps = 0.0f;
            if (!MazeMap::TryComputeInPlaceTurnCommandRadps(
                    errorRad,
                    _drive.GetPose().angularSpeedRadps,
                    dtSeconds,
                    turnProfile,
                    commandedOmegaRadps,
                    angularCommandRadps))
            {
                return Fail("Turn profile became invalid");
            }
            _drive.CommandVelocity(0.0f, angularCommandRadps, dtSeconds);
        }

        if (!HoldBrakedUntilDriveSettles(nullptr, Config::kMotionSettleHoldMs, 0U))
        {
            return false;
        }
        return true;
    }

    bool ExecuteArcProfile(float distanceM, float angleRad, float entrySpeed, float exitSpeed, float cruiseSpeed, const MotionLimits& limits)
    {
        if (distanceM <= 0.0f)
        {
            return ExecuteTurnProfile(angleRad, limits);
        }

        const float startDistanceM = _drive.GetAverageDistanceMeters();
        const float startYawRad = _drive.GetPose().yawRad;
        const float curvature = angleRad / distanceM;
        float commandedSpeedMps = (std::max)(entrySpeed, 0.0f);
        EncoderProgressWatchdog translationWatchdog{};
        translationWatchdog.Reset(0.0f, millis());
        const unsigned long expectedCompletionDeadlineMs = millis() + static_cast<unsigned long>(2500.0f + (5000.0f * distanceM));
        bool stallLogged = false;
        bool durationLogged = false;

        while (true)
        {
            float dtSeconds = 0.0f;
            SensorSnapshot snapshot{};
            if (!TickControl(false, dtSeconds, snapshot))
            {
                return false;
            }
            (void)snapshot;

            const float traveledM = std::fabs(_drive.GetAverageDistanceMeters() - startDistanceM);
            const float remainingM = (std::max)(0.0f, distanceM - traveledM);
            const bool stoppingAtEndpoint = exitSpeed <= 0.05f;
            const bool terminalReached =
                stoppingAtEndpoint ?
                ((remainingM <= Config::kDistanceToleranceM) && IsDriveMotionSettled()) :
                ((remainingM <= Config::kDistanceToleranceM) && (std::fabs(_drive.GetPose().linearSpeedMps - exitSpeed) <= Config::kSpeedToleranceMps));
            if (terminalReached)
            {
                _drive.Brake();
                break;
            }
            const unsigned long nowMs = millis();
            if (!stallLogged && translationWatchdog.Stalled(traveledM, commandedSpeedMps, remainingM, nowMs))
            {
                stallLogged = true;
                AppendMissionTraceFormatted(
                    "mission_motion_watchdog,mode=arc,reason=encoder_stall,cell=(%d,%d),traveled_m=%.4f,remaining_m=%.4f,cmd_v_mps=%.4f",
                    _currentCell.GetX(),
                    _currentCell.GetY(),
                    traveledM,
                    remainingM,
                    commandedSpeedMps);
            }
            if (!durationLogged && static_cast<long>(expectedCompletionDeadlineMs - nowMs) <= 0)
            {
                durationLogged = true;
                AppendMissionTraceFormatted(
                    "mission_motion_watchdog,mode=arc,reason=elapsed_budget_exceeded,cell=(%d,%d),traveled_m=%.4f,remaining_m=%.4f,cmd_v_mps=%.4f",
                    _currentCell.GetX(),
                    _currentCell.GetY(),
                    traveledM,
                    remainingM,
                    commandedSpeedMps);
            }

            const float accelLimitedSpeedMps = (std::min)(cruiseSpeed, commandedSpeedMps + (limits.accelMps2 * dtSeconds));
            const float decelLimitedSpeedMps = ReachableSpeedWithBoundary(exitSpeed, remainingM, limits.decelMps2);
            commandedSpeedMps = (std::min)(accelLimitedSpeedMps, decelLimitedSpeedMps);

            const float progress = (std::clamp)(traveledM / distanceM, 0.0f, 1.0f);
            const float targetYawRad = WrapAngleRad(startYawRad + (angleRad * progress));
            const float headingErrorRad = AngleErrorRad(targetYawRad, _drive.GetPose().yawRad);
            float angularCommandRadps = (curvature * commandedSpeedMps) + (Config::kArcHeadingKp * headingErrorRad) - (Config::kArcYawD * _drive.GetPose().angularSpeedRadps);
            angularCommandRadps = (std::clamp)(angularCommandRadps, -limits.maxAngularSpeedRadps, limits.maxAngularSpeedRadps);
            _drive.CommandVelocity(commandedSpeedMps, angularCommandRadps, dtSeconds);
        }

        if (exitSpeed <= 0.05f)
        {
            if (!HoldBrakedUntilDriveSettles(nullptr, Config::kMotionSettleHoldMs, 0U))
            {
                return false;
            }
        }
        return true;
    }

    bool ExecuteSmoothTurnProfile(
        MazeMap::ManeuverCode code,
        float entrySpeed,
        float exitSpeed,
        float cruiseSpeed,
        const MotionLimits& limits)
    {
        MazeMap::SmoothTurnExecutionProfile profile{};
        if (!TryGetSmoothTurnExecutionProfileMeters(code, profile))
        {
            return Fail("Smooth turn geometry is unavailable");
        }

        float maneuverSpeedMps = cruiseSpeed;
        if (!(maneuverSpeedMps > 0.0f))
        {
            maneuverSpeedMps = (std::max)(entrySpeed, exitSpeed);
        }
        if (!(maneuverSpeedMps > 0.0f))
        {
            return Fail("Smooth turn speed is invalid");
        }

        const float startDistanceM = _drive.GetAverageDistanceMeters();
        const float startYawRad = _drive.GetPose().yawRad;
        const bool stoppingAtEndpoint = exitSpeed <= 0.05f;
        EncoderProgressWatchdog translationWatchdog{};
        translationWatchdog.Reset(0.0f, millis());
        const unsigned long expectedCompletionDeadlineMs = millis() + static_cast<unsigned long>(2500.0f + (5000.0f * profile.totalDistance));
        bool stallLogged = false;
        bool durationLogged = false;

        while (true)
        {
            float dtSeconds = 0.0f;
            SensorSnapshot snapshot{};
            if (!TickControl(false, dtSeconds, snapshot))
            {
                return false;
            }
            (void)snapshot;

            const float traveledM = std::fabs(_drive.GetAverageDistanceMeters() - startDistanceM);
            const float remainingM = (std::max)(0.0f, profile.totalDistance - traveledM);
            if (remainingM <= Config::kDistanceToleranceM)
            {
                if (stoppingAtEndpoint)
                {
                    _drive.Brake();
                }
                break;
            }

            const unsigned long nowMs = millis();
            if (!stallLogged && translationWatchdog.Stalled(traveledM, maneuverSpeedMps, remainingM, nowMs))
            {
                stallLogged = true;
                AppendMissionTraceFormatted(
                    "mission_motion_watchdog,mode=smooth_turn,reason=encoder_stall,cell=(%d,%d),traveled_m=%.4f,remaining_m=%.4f,cmd_v_mps=%.4f",
                    _currentCell.GetX(),
                    _currentCell.GetY(),
                    traveledM,
                    remainingM,
                    maneuverSpeedMps);
            }
            if (!durationLogged && static_cast<long>(expectedCompletionDeadlineMs - nowMs) <= 0)
            {
                durationLogged = true;
                AppendMissionTraceFormatted(
                    "mission_motion_watchdog,mode=smooth_turn,reason=elapsed_budget_exceeded,cell=(%d,%d),traveled_m=%.4f,remaining_m=%.4f,cmd_v_mps=%.4f",
                    _currentCell.GetX(),
                    _currentCell.GetY(),
                    traveledM,
                    remainingM,
                    maneuverSpeedMps);
            }

            float yawOffsetRad = 0.0f;
            float nominalOmegaRadps = 0.0f;
            if (!MazeMap::TryComputeSmoothTurnTarget(profile, traveledM, maneuverSpeedMps, yawOffsetRad, nominalOmegaRadps))
            {
                return Fail("Smooth turn target became invalid");
            }

            const float targetYawRad = WrapAngleRad(startYawRad + yawOffsetRad);
            const float headingErrorRad = AngleErrorRad(targetYawRad, _drive.GetPose().yawRad);
            float angularCommandRadps = nominalOmegaRadps + (Config::kArcHeadingKp * headingErrorRad) - (Config::kArcYawD * _drive.GetPose().angularSpeedRadps);
            angularCommandRadps = (std::clamp)(angularCommandRadps, -limits.maxAngularSpeedRadps, limits.maxAngularSpeedRadps);
            _drive.CommandVelocity(maneuverSpeedMps, angularCommandRadps, dtSeconds);
        }

        if (stoppingAtEndpoint)
        {
            if (!HoldBrakedUntilDriveSettles(nullptr, Config::kMotionSettleHoldMs, 0U))
            {
                return false;
            }
        }
        return true;
    }

    bool IsInGoalCell(MazeMap::CellCoordinates coords) const
    {
        if (!_maze.HasFoundGoal())
        {
            return false;
        }

        const MazeMap::CellCoordinates goal = _maze.GetGoalLowerLeft();
        const bool xMatch = (coords.GetX() == goal.GetX()) || (coords.GetX() == static_cast<uint8_t>(goal.GetX() + 1));
        const bool yMatch = (coords.GetY() == goal.GetY()) || (coords.GetY() == static_cast<uint8_t>(goal.GetY() + 1));
        return xMatch && yMatch;
    }

    static bool TryGetSmoothTurnExecutionProfileMeters(MazeMap::ManeuverCode code, MazeMap::SmoothTurnExecutionProfile& profile)
    {
        profile = MazeMap::SmoothTurnExecutionProfile{};
        if ((code == MazeMap::MC_NONE) || IsStraightCode(code))
        {
            return false;
        }

        MazeMap::SmoothTurnExecutionProfile profileInCells{};
        if (!MazeMap::ManeuverSet::GetSet()[code].TryGetSmoothTurnExecutionProfile(profileInCells))
        {
            return false;
        }

        profile = MazeMap::ScaleSmoothTurnExecutionProfile(profileInCells, Config::kCellSizeM);
        profile.radians = static_cast<float>(MazeMap::CodeDegrees(code)) * DEG_TO_RAD;
        return profile.IsValid();
    }

    static float ManeuverDistanceMeters(MazeMap::ManeuverCode code)
    {
        MazeMap::SmoothTurnExecutionProfile smoothTurnProfile{};
        if (TryGetSmoothTurnExecutionProfileMeters(code, smoothTurnProfile))
        {
            return smoothTurnProfile.totalDistance;
        }
        return 0.5f * Config::kCellSizeM * static_cast<float>(MazeMap::ManeuverSet::GetSet().DistanceTravelled(code));
    }

    float ManeuverSpeedLimit(MazeMap::ManeuverCode code, const MotionLimits& limits) const
    {
        if (code == MazeMap::MC_NONE)
        {
            return 0.0f;
        }
        if (IsStraightCode(code))
        {
            return limits.maxSpeedMps;
        }
        return (std::min)(limits.maxSpeedMps, MazeMap::ManeuverSet::GetSet()[code].GetVMax(_speedVehicle));
    }

    float ManeuverSpeedLimit(MazeMap::ManeuverCode code) const
    {
        return ManeuverSpeedLimit(code, FinalLimits());
    }

    void ApplyAsymmetricQueueLimits(MazeMap::ManeuverQueue& queue, const MotionLimits& limits, float initialEntrySpeed, float finalExitSpeed)
    {
        if (queue.empty())
        {
            return;
        }

        float boundarySpeed = (std::max)(initialEntrySpeed, 0.0f);
        for (uint16_t i = 0; i < queue.size(); ++i)
        {
            MazeMap::ManeuverInstance& entry = queue[i];
            const float speedLimit = ManeuverSpeedLimit(entry.GetCode(), limits);
            if (IsStraightCode(entry.GetCode()))
            {
                const float distanceM = ManeuverDistanceMeters(entry.GetCode());
                const float entrySpeed = (std::min)(boundarySpeed, speedLimit);
                const float exitSpeed = (std::min)(entry.GetExitSpeed(), (std::min)(speedLimit, ReachableSpeedWithBoundary(entrySpeed, distanceM, limits.accelMps2)));
                entry.SetEntrySpeed(entrySpeed);
                entry.SetExitSpeed(exitSpeed);
                boundarySpeed = exitSpeed;
            }
            else
            {
                const float maneuverSpeed = (std::min)((std::min)(entry.GetEntrySpeed(), boundarySpeed), speedLimit);
                entry.SetEntrySpeed(maneuverSpeed);
                entry.SetExitSpeed(maneuverSpeed);
                boundarySpeed = maneuverSpeed;
            }
        }

        float requiredExitSpeed = (std::max)(finalExitSpeed, 0.0f);
        for (int i = static_cast<int>(queue.size()) - 1; i >= 0; --i)
        {
            MazeMap::ManeuverInstance& entry = queue[static_cast<uint16_t>(i)];
            const float speedLimit = ManeuverSpeedLimit(entry.GetCode(), limits);
            if (IsStraightCode(entry.GetCode()))
            {
                const float distanceM = ManeuverDistanceMeters(entry.GetCode());
                const float exitSpeed = (std::min)(entry.GetExitSpeed(), (std::min)(requiredExitSpeed, speedLimit));
                const float entrySpeed = (std::min)(entry.GetEntrySpeed(), (std::min)(speedLimit, ReachableSpeedWithBoundary(exitSpeed, distanceM, limits.decelMps2)));
                entry.SetEntrySpeed(entrySpeed);
                entry.SetExitSpeed(exitSpeed);
                requiredExitSpeed = entrySpeed;
            }
            else
            {
                const float maneuverSpeed = (std::min)(entry.GetEntrySpeed(), (std::min)(requiredExitSpeed, speedLimit));
                entry.SetEntrySpeed(maneuverSpeed);
                entry.SetExitSpeed(maneuverSpeed);
                requiredExitSpeed = maneuverSpeed;
            }
        }

        boundarySpeed = (std::max)(initialEntrySpeed, 0.0f);
        for (uint16_t i = 0; i < queue.size(); ++i)
        {
            MazeMap::ManeuverInstance& entry = queue[i];
            const float speedLimit = ManeuverSpeedLimit(entry.GetCode(), limits);
            if (IsStraightCode(entry.GetCode()))
            {
                const float distanceM = ManeuverDistanceMeters(entry.GetCode());
                const float entrySpeed = (std::min)(entry.GetEntrySpeed(), (std::min)(boundarySpeed, speedLimit));
                const float exitSpeed = (std::min)(entry.GetExitSpeed(), (std::min)(speedLimit, ReachableSpeedWithBoundary(entrySpeed, distanceM, limits.accelMps2)));
                entry.SetEntrySpeed(entrySpeed);
                entry.SetExitSpeed(exitSpeed);
                boundarySpeed = exitSpeed;
            }
            else
            {
                const float maneuverSpeed = (std::min)(entry.GetEntrySpeed(), (std::min)(boundarySpeed, speedLimit));
                entry.SetEntrySpeed(maneuverSpeed);
                entry.SetExitSpeed(maneuverSpeed);
                boundarySpeed = maneuverSpeed;
            }
        }
    }

    void ApplyAsymmetricQueueLimits(MazeMap::ManeuverQueue& queue, float initialEntrySpeed, float finalExitSpeed)
    {
        ApplyAsymmetricQueueLimits(queue, FinalLimits(), initialEntrySpeed, finalExitSpeed);
    }
};

WallSensorLedCalibrationController gLedCalibration;
AuxMeasurementController gAuxMeasurement;
DiagnosticController gDiagnostic;
MissionController gMission;

void setup()
{
    if (IsWallSensorLedCalibrationModeRequested())
    {
        if (gLedCalibration.Begin())
        {
            gLedCalibration.Run();
        }
        return;
    }

    if (IsAuxiliaryMeasurementModeRequested())
    {
        if (AuxMeasurementConfig::kRoutine == AuxMeasurementConfig::Routine::CorridorRepeatabilitySweep)
        {
            if (gMission.BeginCorridorRepeatabilitySweep())
            {
                gMission.RunCorridorRepeatabilitySweep();
            }
        }
        else if (AuxMeasurementConfig::kRoutine == AuxMeasurementConfig::Routine::PositionAccuracyAudit)
        {
            if (gMission.BeginPositionAccuracyAudit())
            {
                gMission.RunPositionAccuracyAudit();
            }
        }
        else if (gAuxMeasurement.Begin())
        {
            gAuxMeasurement.Run();
        }
        return;
    }

    if (IsManeuverTestModeRequested())
    {
        if (gMission.BeginManeuverFileTest())
        {
            gMission.RunManeuverFileTest();
        }
        return;
    }

    if (IsPrimaryDiagnosticModeRequested())
    {
        if (gDiagnostic.Begin())
        {
            gDiagnostic.Run();
        }
        return;
    }

    if (gMission.Begin())
    {
        gMission.Run();
    }
}

void loop()
{
    delay(100);
}


