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
        { "meta", "Active diagnostic tunables are emitted as # meta lines near the top of the log." },
        { "gyro", "baseline_idle/final_idle -> kGyroBiasSamples, kGyroBiasUpdateMaxAbsRateRadps; inspect gyro_raw_radps drift and bias convergence." },
        { "kickoff_ff", "kickoff_* -> motor-model wheel FF, kWheelStaticFeedforward trim, kWheelRestLaunchDriveCommand." },
        { "kickoff_thresholds", "kickoff_* -> kWheelRestLaunchSpeedThresholdMps, kWheelRestLaunchDriveThreshold, kWheelRestLaunchMaxDriveCommand, kWheelRestLaunchRampMs." },
        { "forward", "forward_* -> motor-model wheel FF, kWheelVelocityFeedforward, kWheelVelocityKp, kWheelVelocityKi, kWheelIntegralLimit." },
        { "diag_feedback", "diag closed-loop -> kDiagnosticWheelVelocityKpScale, kDiagnosticWheelVelocityKiScale, kDiagnosticWheelIntegralLimitScale." },
        { "turn", "turn_* -> kTurnHeadingKp, kTurnYawD, kAngleToleranceRad, kAngularSpeedToleranceRadps; use turn_result peak/final yaw error." },
        { "straight", "straight_* -> kStraightHeadingKp, kStraightYawD, kDistanceToleranceM, kSpeedToleranceMps; use straight_result heading and stop error." },
        { "watchdog", "straight_* faults -> kEncoderProgressEpsilonM, kEncoderStallCommandThresholdMps, kEncoderStallTimeoutMs, kEncoderStallStartupGraceMs." },
        { "arc", "arc_* -> kArcHeadingKp, kArcYawD; use arc_result and arc_circle_result heading/closure error." },
        { "arc_track_width", "arc_* also uses kArcTrackWidthTightM, kArcTrackWidthTightRadiusM, kArcTrackWidthWideM, kArcTrackWidthWideRadiusM." },
        { "circle", "circle_* -> kArcHeadingKp, kArcYawD, arc track-width model; use circle_result counts, avg_omega_radps, est_lat_mps2, avg_lat_mps2." },
        { "square", "square_* -> kTrackWidthM with kStraightHeadingKp/kStraightYawD and kTurnHeadingKp/kTurnYawD; use square_result closure and final heading error." },
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
