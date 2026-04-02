#pragma once
#include "Defines.h"
namespace MazeMap::DiagnosticConfig
{
    // Likelihood tags for the diagnostic-only config:
    // [High] commonly adjusted to improve data quality or stress level during test sessions.
    // [Medium] sometimes adjusted as the diagnostic workflow matures.
    // [Low] usually fixed by wiring, safety limits, or operator workflow.

    // [Low] Short pins 27 and 28 at boot to enter the open-floor measurement routine.
    constexpr uint8_t kModeSelectPinA = 27U;
    // [Low] Dedicated strap partner for open-floor measurement mode.
    constexpr uint8_t kModeSelectPinB = 28U;
    // [Low] One half-step in the open-floor measurement workspace, in millimeters.
    constexpr float kHalfStepMm = 90.0f;
    // [Low] The open-floor workspace is a 5 x 5 half-step box in local coordinates.
    constexpr uint8_t kWorkspaceSizeHalfSteps = 5U;
    // [Medium] Diagnostic control/log period. Shorten it only if SD logging, sensor reads, and control math still meet
    // the deadline; lengthen it if you see dropped samples or write stalls.
    constexpr unsigned long kControlPeriodUs = 1000UL;
    // [Medium] Timing-block repetitions required before the main run may begin.
    constexpr uint16_t kTimingCaptureCycles = 200U;
    // [Medium] Initial stationary settle before the diagnostic battery starts. Increase if the robot is still moving
    // from placement when logging begins; decrease if startup idle time is unnecessary.
    constexpr uint16_t kStartupSettleMs = 250U;
    // [Medium] Static-hold duration used by SEC_10_STATIC.
    constexpr uint16_t kStaticHoldMs = 15000U;
    // [Medium] Idle capture window used for baseline noise and bias logging. Increase if you want better stationary
    // statistics; decrease if the diagnostic routine spends too long collecting idle data.
    constexpr uint16_t kBaselineHoldMs = 2500U;
    // [Medium] Pause between diagnostic phases. This is set from measured post-motion settling data so the robot has
    // time to re-enter a genuinely stationary state before the next maneuver starts.
    constexpr uint16_t kInterTestHoldMs = 350U;
    // [Medium] SD flush cadence during diagnostics. Decrease it if you want less data loss risk on power interruption;
    // increase it if flush overhead limits logging throughput.
    constexpr uint32_t kLogFlushPeriodMs = 250U;
    // [Medium] Launch repeats per command magnitude and sign in SEC_20_LAUNCH.
    constexpr uint8_t kLaunchRepeatsPerMagnitude = 5U;
    // [Medium] Straight repeats per speed bin and direction in SEC_30_STRAIGHT.
    constexpr uint8_t kStraightRepeatsPerSpeed = 3U;
    // [Medium] Yaw repeats per primitive-speed pair in SEC_40_YAW.
    constexpr uint8_t kYawRepeatsPerPrimitiveSpeed = 3U;
    // [Medium] Smooth-turn repeats per primitive-speed pair in SEC_50_SMOOTH.
    constexpr uint8_t kSmoothRepeatsPerPrimitiveSpeed = 5U;
    // [Medium] Closed-loop repeats for SEC_60_LOOP_CW and SEC_70_LOOP_CCW.
    constexpr uint8_t kLoopRepeats = 5U;
    // [Low] Half-width of the legacy diagnostic operating square. Retained for compatibility with shared helpers.
    constexpr float kBoundaryHalfSpanM = 0.34f;
    // [High] Short straight distance used in the diagnostic battery. Increase for more steady-state straight data;
    // decrease if you need to stay well inside the safety box or focus on launch/braking behavior.
    constexpr float kShortStraightDistanceM = 0.18f;
    // [High] Long straight distance used in the diagnostic battery. Increase if you need more data at higher speed;
    // decrease if the robot approaches the boundary or cannot complete the profile cleanly.
    constexpr float kLongStraightDistanceM = 0.27f;
    // [High] Side length of the diagnostic square-loop test. Increase for more coupled straight/turn data; decrease
    // if the loop approaches the boundary or you want to isolate turn behavior.
    constexpr float kSquareLegDistanceM = 0.18f;
    // [High] Arc length for each half-circle arc test. Increase for longer arc-tracking data; decrease if the circle
    // grows too large for the available floor space or you want tighter curvature.
    constexpr float kArcHalfCircleDistanceM = 0.20f;
    // [High] Cruise speed for conservative diagnostic straights and square loops. Increase once low-speed data is
    // boring and stable; decrease if you need cleaner low-dynamics identification data.
    constexpr float kSlowStraightSpeedMps = 0.4f;
    // [High] Mid-speed cruise used by the diagnostic circle sweep. Increase once the slower circle data is clean;
    // decrease if the circle comparison already exposes the smooth-turn mismatch you need to correct.
    constexpr float kCircleMediumSpeedMps = 0.6f;
    // [High] Cruise speed for the longer diagnostic straight test. Increase to probe higher-speed behavior; decrease
    // if braking distance, tracking error, or the safety boundary becomes problematic.
    constexpr float kFastStraightSpeedMps = 0.8f;
    // [High] Straight-profile acceleration during diagnostics. Increase if you want stronger feedforward/traction data;
    // decrease if launches spin the tires or make the test less repeatable.
    constexpr float kStraightAccelMps2 = 6.50f;
    // [High] Straight-profile braking during diagnostics. Increase if you want more braking data or tighter stops;
    // decrease if braking becomes noisy, slides the tires, or destabilizes the chassis.
    constexpr float kStraightDecelMps2 = 5.80f;
    // [High] Turn-rate limit for diagnostic turn sweeps. Increase to stress higher-yaw-rate behavior; decrease if the
    // turn data is dominated by overshoot or wheel scrub instead of clean rotational dynamics.
    constexpr float kTurnMaxOmegaRadps = 15.0f;
    // [High] Turn acceleration for diagnostic turn sweeps. Increase to excite sharper turn entry/exit dynamics;
    // decrease if the robot cannot reach those ramps repeatably without slipping.
    constexpr float kTurnAccelRadps2 = 350.0f;
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
    constexpr float kKickoffSweepStepDriveCommand = 0.01f;
    // [High] Pulse length for each kickoff sample. Increase if static friction needs a longer shove to reveal the
    // threshold; decrease if the robot moves too far before the recovery segment.
    constexpr uint16_t kKickoffSweepPulseMs = 120U;
    // [Medium] Minimum distance that counts as "moved" during the kickoff sweep. Increase if encoder noise causes false
    // positives; decrease if real breakaway moves are being missed.
    constexpr float kKickoffSweepMoveThresholdM = 0.02f;
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
    constexpr float kForwardSweepStepDriveCommand = 0.01f;
    // [High] Hold time for each forward sweep sample after the kickoff pulse. Increase for better steady-speed data;
    // decrease if the robot travels too far before the recovery segment.
    constexpr uint16_t kForwardSweepHoldMs = 220U;
    // [Medium] Average hold speed that counts as "carried" during the forward sweep. Increase if tiny creeping speeds
    // are not useful; decrease if the desired sustaining command is very gentle.
    constexpr float kForwardSweepCarryThresholdMps = 0.05f;
    // [Medium] Distance accumulated during the hold segment that counts as a meaningful carry. Increase to ignore tiny
    // nudges; decrease if low-speed sustained motion is the target.
    constexpr float kForwardSweepCarryThresholdM = 0.180f;
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

namespace DiagnosticConfig = MazeMap::DiagnosticConfig;
