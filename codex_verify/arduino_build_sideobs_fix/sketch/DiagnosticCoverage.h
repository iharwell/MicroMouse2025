#line 1 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\DiagnosticCoverage.h"
#pragma once

#include "Defines.h"

namespace MazeMap
{
    struct DiagnosticSummaryInstruction
    {
        const char* id;
        const char* message;
    };

    inline constexpr DiagnosticSummaryInstruction kDiagnosticSummaryInstructions[] = {
        { "meta", "All active diagnostic tunables are written as # meta lines near the top of this log." },
        { "gyro", "baseline_idle/final_idle -> kGyroBiasSamples and kGyroBiasUpdateMaxAbsRateRadps. Use stationary gyro_raw_radps drift and bias convergence." },
        { "kickoff_ff", "kickoff_* -> kWheelStaticFeedforward and kWheelRestLaunchDriveCommand. Use moved/max_speed_mps for breakaway." },
        { "kickoff_thresholds", "kickoff_* -> kWheelRestLaunchSpeedThresholdMps, kWheelRestLaunchDriveThreshold, kWheelRestLaunchMaxDriveCommand, and kWheelRestLaunchRampMs." },
        { "forward", "forward_* -> kWheelVelocityFeedforward, kWheelVelocityKp, kWheelVelocityKi, kWheelIntegralLimit. Use hold_avg_speed_mps/carried/travel_limited." },
        { "diag_feedback", "closed-loop diag phases also use kDiagnosticWheelVelocityKpScale, kDiagnosticWheelVelocityKiScale, and kDiagnosticWheelIntegralLimitScale." },
        { "turn", "turn_* -> kTurnHeadingKp, kTurnYawD, kAngleToleranceRad, and kAngularSpeedToleranceRadps. Use turn_result peak/final yaw error." },
        { "straight", "straight_* -> kStraightHeadingKp, kStraightYawD, kDistanceToleranceM, and kSpeedToleranceMps. Use straight_result heading and stop error." },
        { "watchdog", "straight_* faults -> kEncoderProgressEpsilonM, kEncoderStallCommandThresholdMps, kEncoderStallTimeoutMs, and kEncoderStallStartupGraceMs." },
        { "arc", "arc_* -> kArcHeadingKp and kArcYawD. Use arc_result and arc_circle_result heading and closure error." },
        { "arc_track_width", "arc_* also uses kArcTrackWidthTightM, kArcTrackWidthTightRadiusM, kArcTrackWidthWideM, and kArcTrackWidthWideRadiusM." },
        { "circle", "circle_* -> kArcHeadingKp, kArcYawD, and the arc track-width model. Use circle_result counts, avg_omega_radps, est_lat_mps2, and avg_lat_mps2." },
        { "square", "square_* -> kTrackWidthM with kStraightHeadingKp/kStraightYawD and kTurnHeadingKp/kTurnYawD. Use square_result closure and final heading error." },
    };

    inline constexpr size_t GetDiagnosticSummaryInstructionCount()
    {
        return sizeof(kDiagnosticSummaryInstructions) / sizeof(kDiagnosticSummaryInstructions[0]);
    }

    inline constexpr const DiagnosticSummaryInstruction& GetDiagnosticSummaryInstruction(size_t index)
    {
        return kDiagnosticSummaryInstructions[index];
    }
}
