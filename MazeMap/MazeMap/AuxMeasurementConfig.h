#pragma once

#include "Defines.h"
#include "DirectionalLocation.h"
#include "Maneuver.h"
namespace MazeMap::AuxMeasurementConfig
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

namespace AuxMeasurementConfig = MazeMap::AuxMeasurementConfig;
