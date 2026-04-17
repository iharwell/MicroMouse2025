#include "pch.h"
#include "MazeMapRuntimeInfrastructure.h"

#include "MazeMapSharedRuntime.h"
#include "RuntimeSensorSuite.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace MazeMap::App::Internal::Runtime
{
    bool BeginDiagnosticUtilityTelemetryLog(
        MazeMap::App::Internal::SharedRobotRuntime& runtime,
        RuntimeSensorSuite& sensors,
        DiagnosticLogRow& row,
        const char* fileName,
        const char* modeName,
        unsigned long& phaseId,
        unsigned long& sampleCount)
    {
        const char* const resolvedFileName =
            (fileName != nullptr && fileName[0] != '\0') ? fileName : "telemetry.mmlog";
        const char* const resolvedModeName =
            (modeName != nullptr && modeName[0] != '\0') ? modeName : "telemetry";

        phaseId = 0UL;
        sampleCount = 0UL;
        row = {};
        (void)runtime.CloseUtilityDataLog();

        auto fail = [&runtime, &row]() -> bool
        {
            row = {};
            (void)runtime.CloseUtilityDataLog();
            return false;
        };

        if (!runtime.OpenUtilityDataLogFile(resolvedFileName))
        {
            return false;
        }
        if (!runtime.WriteUtilityDataLogMetadata("mode", resolvedModeName))
        {
            return fail();
        }
        if (!runtime.WriteUtilityDataLogMetadataUnsigned("control_period_us", Config::kControlPeriodUs))
        {
            return fail();
        }
        {
            const unsigned long imuSampleRateHz = MazeMap::GetUiImuSampleRateHzForControlPeriodUs(Config::kControlPeriodUs);
            if (imuSampleRateHz > 0UL && !runtime.WriteUtilityDataLogMetadataUnsigned("imu_sample_rate_hz", imuSampleRateHz))
            {
                return fail();
            }
        }
        {
            const float imuAccelLpf2CutoffHz = MazeMap::GetUiAccelLpf2CutoffHzForControlPeriodUs(
                Config::kControlPeriodUs,
                Config::kMissionRuntimeAccelFilterFreq);
            if (imuAccelLpf2CutoffHz > 0.0f &&
                !runtime.WriteUtilityDataLogMetadataFloat("imu_accel_lpf2_cutoff_hz", imuAccelLpf2CutoffHz, 3))
            {
                return fail();
            }
        }
        {
            const float imuGyroLpf1ReferenceHz =
                MazeMap::GetUiGyroCut213DatasheetReferenceHzForControlPeriodUs(Config::kControlPeriodUs);
            if (imuGyroLpf1ReferenceHz > 0.0f &&
                !runtime.WriteUtilityDataLogMetadataFloat("imu_gyro_lpf1_cut213_datasheet_ref_hz", imuGyroLpf1ReferenceHz, 3))
            {
                return fail();
            }
        }
        if (!runtime.WriteUtilityDataLogMetadataFloat("boundary_half_span_m", DiagnosticConfig::kBoundaryHalfSpanM, 3))
        {
            return fail();
        }
        if (!runtime.WriteUtilityDataLogMetadataFloat("imu_gyro_mdps_per_lsb", sensors.GetGyroSensitivityMdpsPerLsb(), 3))
        {
            return fail();
        }
        if (!runtime.WriteUtilityDataLogMetadataFloat("imu_accel_mg_per_lsb", sensors.GetAccelSensitivityMgPerLsb(), 3))
        {
            return fail();
        }
        if (!runtime.WriteUtilityDataLogMetadataFloat("mission_gyro_bias_estimate_radps", sensors.GetGyroBiasRadps(), 6))
        {
            return fail();
        }
        if (!runtime.WriteUtilityDataLogAccelBiasMetadata(sensors))
        {
            return fail();
        }
        if (!runtime.WriteUtilityDataLogMetadata("format_spec", "micromouse_logging_spec_rev_g"))
        {
            return fail();
        }
        if (!runtime.WriteUtilityDataLogMetadata("endianness", "little"))
        {
            return fail();
        }

        if (!runtime.BeginUtilityDataLogSchema(row))
        {
            return fail();
        }
        if (!runtime.WriteTextLogMetadata("file", runtime.TextLogFileName()))
        {
            return fail();
        }
        if (!runtime.WriteTextLogMetadata("data_file", resolvedFileName))
        {
            return fail();
        }
        if (!runtime.WriteTextLogMetadata("mode", resolvedModeName))
        {
            return fail();
        }
        if (!WriteDiagnosticTuningEvents(
                [&runtime](const char* type, const char* message) -> bool
                {
                    return runtime.WriteTextLogEntry(micros(), type, message);
                }))
        {
            return fail();
        }
        if (!WriteDiagnosticSummaryInstructions(
                [&runtime](const char* type, const char* message) -> bool
                {
                    return runtime.WriteTextLogEntry(micros(), type, message);
                }))
        {
            return fail();
        }

        return true;
    }

    LoopController::ControlVector DriveSharedWallTouchLoopTick(
        DriveBase& drive,
        void* rawState,
        WallTouchLoopState& wallTouch,
        const LoopController::ModeState& state,
        LoopController::TickServices& services,
        const WallTouchLoopHooks& hooks,
        const MazeMap::CommandPD trackingCommandPd)
    {
        auto appendTraceLine = [&hooks](const char* line) noexcept
        {
            if (hooks.appendTraceLine != nullptr && line != nullptr)
            {
                hooks.appendTraceLine(hooks.context, line);
            }
        };
        auto fail = [&hooks, &services](const char* reason) noexcept -> LoopController::ControlVector
        {
            if (hooks.fault != nullptr)
            {
                return hooks.fault(hooks.context, services, reason);
            }

            services.Fault(reason);
            return LoopController::ControlVector::Brake;
        };
        auto complete = [&hooks, &services]() -> LoopController::ControlVector
        {
            if (hooks.complete != nullptr)
            {
                return hooks.complete(hooks.context, services);
            }

            services.RequestEndLoop();
            return LoopController::ControlVector::Brake;
        };

        const float clampedMinLatchTravelM = (std::max)(0.0f, wallTouch.minLatchTravelM);
        const float clampedMaxApproachTravelM =
            (std::max)(clampedMinLatchTravelM, wallTouch.maxApproachTravelM);
        if (!(std::isfinite(clampedMaxApproachTravelM) && (clampedMaxApproachTravelM > 0.0f)))
        {
            return fail("Wall touch-off max travel is invalid");
        }

        auto traceStateTransition =
            [&wallTouch, &appendTraceLine](const WallTouchState fromState,
                                           const WallTouchState toState,
                                           const float traveledDistanceM)
        {
            char line[192] = {};
            snprintf(
                line,
                sizeof(line),
                "startup_cal_touch:state,from=%s,to=%s,elapsed_ms=%lu,travel=%.4f",
                WallTouchStateName(fromState),
                WallTouchStateName(toState),
                static_cast<unsigned long>(millis() - wallTouch.touchStartMs),
                traveledDistanceM);
            appendTraceLine(line);
        };

        const WallTouchObservation observation = MakeWallTouchObservation(state.sensors);
        const unsigned long nowMs = millis();
        const unsigned long elapsedMs = nowMs - wallTouch.touchStartMs;
        const unsigned long stateElapsedMs = nowMs - wallTouch.stateStartMs;
        const PoseEstimate& pose = drive.GetPose();
        const DriveTelemetry& telemetry = state.driveTelemetry;
        const float traveledDistanceM = std::fabs(drive.GetAverageDistanceMeters() - wallTouch.startDistanceM);
        const bool frontSignalActive =
            observation.frontWall ||
            observation.frontLeftWall ||
            observation.frontRightWall;
        wallTouch.result.finalTravelM = traveledDistanceM;

        if ((wallTouch.runtimeState != WallTouchState::ControlledRelease) &&
            (wallTouch.runtimeState != WallTouchState::ReverseToClearance) &&
            (traveledDistanceM >= clampedMaxApproachTravelM))
        {
            char line[192] = {};
            snprintf(
                line,
                sizeof(line),
                "startup_cal_touch:max_travel,state=%s,travel=%.4f,expected=%.4f,max=%.4f",
                WallTouchStateName(wallTouch.runtimeState),
                traveledDistanceM,
                clampedMinLatchTravelM,
                clampedMaxApproachTravelM);
            appendTraceLine(line);
            if (wallTouch.allowPassThroughNoWall && (wallTouch.contactConfirmedStartMs == 0UL))
            {
                wallTouch.result.outcome = WallTouchOutcome::PassedThroughNoWall;
                if (hooks.beginPassThroughSettle == nullptr)
                {
                    return fail("Wall touch-off pass-through settle hook was not installed");
                }

                return hooks.beginPassThroughSettle(hooks.context, rawState, services);
            }

            return fail("Wall touch-off exceeded max travel");
        }

        if (HasWallTouchEncoderMotion(
                wallTouch.lastMotionTelemetry,
                telemetry,
                Config::kWallTouchProgressStallDistanceM))
        {
            wallTouch.lastMotionMs = nowMs;
            wallTouch.lastMotionTelemetry = telemetry;
        }

        if (wallTouch.runtimeState == WallTouchState::ContactSeek)
        {
            wallTouch.approachDriveCommand =
                ComputeWallTouchApproachDriveCommand(
                    traveledDistanceM,
                    clampedMinLatchTravelM);
            LoopController::ControlVector command = LoopController::ControlVector::Brake;
            if (ShouldBrakeWallTouchApproachForEncoderSpeed(telemetry))
            {
                command = LoopController::ControlVector::Brake;
            }
            else
            {
                wallTouch.approachDriveCommand =
                    LimitWallTouchApproachDriveCommandByEncoderSpeed(
                        wallTouch.approachDriveCommand,
                        telemetry);
                command = LoopController::ControlVector::RawMotorPwm(
                    wallTouch.approachDriveCommand,
                    wallTouch.approachDriveCommand);
            }

            const bool motionCollapseIndicator = MazeMap::IsWallTouchContactSample(
                traveledDistanceM,
                pose.linearSpeedMps,
                Config::kWallTouchMinApproachDistanceM,
                clampedMinLatchTravelM,
                Config::kMotionSettleSpeedThresholdMps,
                elapsedMs,
                Config::kWallTouchMinCommandTimeMs);
            const bool progressStallIndicator =
                (elapsedMs >= Config::kWallTouchMinCommandTimeMs) &&
                ((nowMs - wallTouch.lastMotionMs) >= Config::kWallTouchProgressStallWindowMs);
            const std::uint8_t indicatorCount = MazeMap::CountWallTouchContactIndicators(
                frontSignalActive,
                motionCollapseIndicator,
                progressStallIndicator);
            if (indicatorCount >= 2U)
            {
                if (!wallTouch.contactCandidateActive)
                {
                    wallTouch.contactCandidateStartMs = nowMs;
                    wallTouch.contactCandidateActive = true;
                }
                else if (MazeMap::HasWallTouchConfirmedContact(
                    nowMs - wallTouch.contactCandidateStartMs,
                    Config::kWallTouchContactConfirmationMs,
                    indicatorCount))
                {
                    wallTouch.contactConfirmedStartMs = wallTouch.contactCandidateStartMs;
                    wallTouch.runtimeState = WallTouchState::SeatingPreloadRamp;
                    wallTouch.stateStartMs = nowMs;
                    char line[224] = {};
                    snprintf(
                        line,
                        sizeof(line),
                        "startup_cal_touch:contact_confirmed,travel=%.4f,elapsed_ms=%lu,front=%u,collapse=%u,stall=%u",
                        traveledDistanceM,
                        elapsedMs,
                        frontSignalActive ? 1U : 0U,
                        motionCollapseIndicator ? 1U : 0U,
                        progressStallIndicator ? 1U : 0U);
                    appendTraceLine(line);
                    traceStateTransition(
                        WallTouchState::ContactSeek,
                        wallTouch.runtimeState,
                        traveledDistanceM);
                }
            }
            else
            {
                wallTouch.contactCandidateActive = false;
            }

            return command;
        }

        if (wallTouch.runtimeState == WallTouchState::SeatingPreloadRamp)
        {
            const float rampAlpha =
                static_cast<float>((std::min)(stateElapsedMs, static_cast<unsigned long>(Config::kWallTouchSeatRampMs))) /
                static_cast<float>((std::max)(Config::kWallTouchSeatRampMs, static_cast<std::uint16_t>(1U)));
            const float seatDriveCommand =
                wallTouch.approachDriveCommand +
                ((Config::kWallTouchSeatRampMaxDriveCommand - wallTouch.approachDriveCommand) * rampAlpha);
            if (stateElapsedMs >= Config::kWallTouchSeatRampMs)
            {
                wallTouch.runtimeState = WallTouchState::InitialSeatingDwell;
                wallTouch.stateStartMs = nowMs;
                traceStateTransition(
                    WallTouchState::SeatingPreloadRamp,
                    wallTouch.runtimeState,
                    traveledDistanceM);
            }
            return LoopController::ControlVector::RawMotorPwm(seatDriveCommand, seatDriveCommand);
        }

        if (wallTouch.runtimeState == WallTouchState::InitialSeatingDwell)
        {
            if (stateElapsedMs >= Config::kWallTouchInitialSeatDwellMs)
            {
                wallTouch.runtimeState = WallTouchState::SquareUpDither;
                wallTouch.stateStartMs = nowMs;
                wallTouch.currentCycleStartYawRad = pose.yawRad;
                wallTouch.currentCycleMaxFrontSkewMagnitudeM = 0.0f;
                wallTouch.currentCycleMaxResidualYawRateRadps = 0.0f;
                wallTouch.currentCycleFrontSignalValid = true;
                wallTouch.haveSquareSample = false;
                wallTouch.completedHalfCycles = 0U;
                wallTouch.result.completedFullCycles = 0U;
                wallTouch.consecutiveGoodFullCycles = 0U;
                wallTouch.ditherTurnFraction = Config::kWallTouchSeatWiggleTurnFraction;
                wallTouch.frontSignalMissingStartMs = 0UL;
                traceStateTransition(
                    WallTouchState::InitialSeatingDwell,
                    wallTouch.runtimeState,
                    traveledDistanceM);
            }
            return LoopController::ControlVector::RawMotorPwm(
                Config::kWallTouchSeatRampMaxDriveCommand,
                Config::kWallTouchSeatRampMaxDriveCommand);
        }

        if (wallTouch.runtimeState == WallTouchState::SquareUpDither)
        {
            const unsigned long contactDurationMs =
                (wallTouch.contactConfirmedStartMs > 0UL) ?
                (nowMs - wallTouch.contactConfirmedStartMs) :
                0UL;
            wallTouch.result.confirmedContactMs = contactDurationMs;
            if (!frontSignalActive)
            {
                if (wallTouch.frontSignalMissingStartMs == 0UL)
                {
                    wallTouch.frontSignalMissingStartMs = nowMs;
                }
                else if ((nowMs - wallTouch.frontSignalMissingStartMs) >= Config::kWallTouchContactConfirmationMs)
                {
                    char line[192] = {};
                    snprintf(
                        line,
                        sizeof(line),
                        "startup_cal_touch:front_signal_invalid,elapsed_ms=%lu,travel=%.4f",
                        contactDurationMs,
                        traveledDistanceM);
                    appendTraceLine(line);
                    return fail("Wall touch-off front sensors invalid during square-up");
                }
            }
            else
            {
                wallTouch.frontSignalMissingStartMs = 0UL;
            }

            const MazeMap::OpenLoopDriveCommand ditherCommand = MazeMap::ComputeOpenLoopYawDitherCommand(
                Config::kWallTouchSeatRampMaxDriveCommand,
                stateElapsedMs,
                Config::kWallTouchSeatWiggleHalfPeriodMs,
                Config::kWallTouchSeatWiggleBlendMs,
                wallTouch.ditherTurnFraction,
                Config::kWallTouchSeatWiggleRetainedForwardFraction);

            const unsigned long halfCycleIndex =
                stateElapsedMs /
                (std::max)(Config::kWallTouchSeatWiggleHalfPeriodMs, static_cast<std::uint16_t>(1U));
            if (wallTouch.haveSquareSample && (halfCycleIndex != wallTouch.lastHalfCycleIndex))
            {
                ++wallTouch.completedHalfCycles;
                wallTouch.currentCycleMaxFrontSkewMagnitudeM = (std::max)(
                    wallTouch.currentCycleMaxFrontSkewMagnitudeM,
                    std::fabs(wallTouch.lastSquareFrontSkewM));
                wallTouch.currentCycleMaxResidualYawRateRadps = (std::max)(
                    wallTouch.currentCycleMaxResidualYawRateRadps,
                    std::fabs(wallTouch.lastSquareYawRateRadps));
                wallTouch.currentCycleFrontSignalValid =
                    wallTouch.currentCycleFrontSignalValid && wallTouch.lastSquareFrontSignalValid;

                char halfCycleLine[256] = {};
                snprintf(
                    halfCycleLine,
                    sizeof(halfCycleLine),
                    "startup_cal_touch:half_cycle,index=%u,front_skew_m=%.4f,residual_yaw_rate_radps=%.4f,turn_fraction=%.3f",
                    static_cast<unsigned>(wallTouch.completedHalfCycles),
                    std::fabs(wallTouch.lastSquareFrontSkewM),
                    std::fabs(wallTouch.lastSquareYawRateRadps),
                    wallTouch.ditherTurnFraction);
                appendTraceLine(halfCycleLine);

                if ((wallTouch.completedHalfCycles & 1U) == 0U)
                {
                    ++wallTouch.result.completedFullCycles;
                    const float netYawChangeMagnitudeRad =
                        std::fabs(AngleErrorRad(wallTouch.currentCycleStartYawRad, wallTouch.lastSquareYawRad));
                    const bool cycleGood = MazeMap::IsWallTouchSquareCycleGood(
                        wallTouch.currentCycleMaxFrontSkewMagnitudeM,
                        Config::kWallTouchSquareFrontSkewThresholdM,
                        wallTouch.currentCycleMaxResidualYawRateRadps,
                        Config::kWallTouchSquareResidualYawRateThresholdRadps,
                        netYawChangeMagnitudeRad,
                        Config::kWallTouchSquareNetYawChangeThresholdRad,
                        wallTouch.currentCycleFrontSignalValid);
                    wallTouch.consecutiveGoodFullCycles =
                        cycleGood ? static_cast<std::uint8_t>(wallTouch.consecutiveGoodFullCycles + 1U) : 0U;

                    char cycleLine[320] = {};
                    snprintf(
                        cycleLine,
                        sizeof(cycleLine),
                        "startup_cal_touch:full_cycle,index=%u,good=%u,front_skew_m=%.4f,residual_yaw_rate_radps=%.4f,net_yaw_deg=%.2f,contact_ms=%lu,turn_fraction=%.3f",
                        static_cast<unsigned>(wallTouch.result.completedFullCycles),
                        cycleGood ? 1U : 0U,
                        wallTouch.currentCycleMaxFrontSkewMagnitudeM,
                        wallTouch.currentCycleMaxResidualYawRateRadps,
                        RAD_TO_DEG_F * netYawChangeMagnitudeRad,
                        contactDurationMs,
                        wallTouch.ditherTurnFraction);
                    appendTraceLine(cycleLine);

                    if (!cycleGood &&
                        (wallTouch.ditherTurnFraction < Config::kWallTouchSeatWiggleMaxTurnFraction) &&
                        (MazeMap::HasWallTouchSquareUpSaturated(
                            wallTouch.previousCycleFrontSkewMagnitudeM,
                            wallTouch.currentCycleMaxFrontSkewMagnitudeM,
                            Config::kWallTouchSquareImprovementSaturationThresholdM) ||
                            (wallTouch.result.completedFullCycles >= Config::kWallTouchSeatMinimumFullCycles)))
                    {
                        wallTouch.ditherTurnFraction = MazeMap::ComputeWallTouchSeatWiggleTurnFraction(
                            wallTouch.result.completedFullCycles,
                            Config::kWallTouchSeatWiggleTurnFraction,
                            Config::kWallTouchSeatWiggleTurnFractionStep,
                            Config::kWallTouchSeatWiggleMaxTurnFraction);
                    }

                    wallTouch.previousCycleFrontSkewMagnitudeM = wallTouch.currentCycleMaxFrontSkewMagnitudeM;
                    wallTouch.currentCycleStartYawRad = wallTouch.lastSquareYawRad;
                    wallTouch.currentCycleMaxFrontSkewMagnitudeM = 0.0f;
                    wallTouch.currentCycleMaxResidualYawRateRadps = 0.0f;
                    wallTouch.currentCycleFrontSignalValid = true;

                    if (MazeMap::IsWallTouchSquareSuccessEligible(
                            contactDurationMs,
                            Config::kWallTouchMinimumConfirmedContactMs,
                            wallTouch.result.completedFullCycles,
                            Config::kWallTouchSeatMinimumFullCycles,
                            wallTouch.consecutiveGoodFullCycles,
                            Config::kWallTouchSeatRequiredGoodFullCycles))
                    {
                        wallTouch.runtimeState = WallTouchState::PostSquareSeatedHold;
                        wallTouch.stateStartMs = nowMs;
                        wallTouch.result.seatedTravelM = traveledDistanceM;
                        wallTouch.result.seatedYawErrorRad = AngleErrorRad(wallTouch.targetYawRad, pose.yawRad);
                        traceStateTransition(
                            WallTouchState::SquareUpDither,
                            wallTouch.runtimeState,
                            traveledDistanceM);
                    }
                }
            }

            if (contactDurationMs >= Config::kWallTouchSquareUpTimeoutMs)
            {
                char line[192] = {};
                snprintf(
                    line,
                    sizeof(line),
                    "startup_cal_touch:square_timeout,contact_ms=%lu,turn_fraction=%.3f,cycles=%u",
                    contactDurationMs,
                    wallTouch.ditherTurnFraction,
                    static_cast<unsigned>(wallTouch.result.completedFullCycles));
                appendTraceLine(line);
                return fail("Wall touch-off square-up timed out");
            }

            wallTouch.haveSquareSample = true;
            wallTouch.lastHalfCycleIndex = halfCycleIndex;
            wallTouch.lastSquareYawRad = pose.yawRad;
            wallTouch.lastSquareFrontSkewM = observation.frontSkewM;
            wallTouch.lastSquareYawRateRadps = pose.angularSpeedRadps;
            wallTouch.lastSquareFrontSignalValid = frontSignalActive;
            return LoopController::ControlVector::RawMotorPwm(
                ditherCommand.leftDriveCommand,
                ditherCommand.rightDriveCommand);
        }

        if (wallTouch.runtimeState == WallTouchState::PostSquareSeatedHold)
        {
            if (!wallTouch.seatedResetApplied &&
                (stateElapsedMs >= (Config::kWallTouchPostSquareHoldMs / 2U)))
            {
                wallTouch.seatedResetApplied = true;
                if (wallTouch.poseResetTarget != nullptr && wallTouch.poseResetTarget->enabled)
                {
                    drive.SetPose(
                        wallTouch.poseResetTarget->xMeters,
                        wallTouch.poseResetTarget->yMeters,
                        wallTouch.poseResetTarget->yawRad);
                    if (hooks.onPoseReset != nullptr)
                    {
                        hooks.onPoseReset(hooks.context);
                    }
                }
            }
            if (stateElapsedMs >= Config::kWallTouchPostSquareHoldMs)
            {
                char line[224] = {};
                snprintf(
                    line,
                    sizeof(line),
                    "startup_cal_touch:reset_pose,x=%.4f,y=%.4f,yaw_deg=%.2f,travel=%.4f",
                    drive.GetPose().xMeters,
                    drive.GetPose().yMeters,
                    RAD_TO_DEG_F * drive.GetPose().yawRad,
                    wallTouch.result.seatedTravelM);
                appendTraceLine(line);
                wallTouch.runtimeState = WallTouchState::ControlledRelease;
                wallTouch.stateStartMs = nowMs;
                traceStateTransition(
                    WallTouchState::PostSquareSeatedHold,
                    wallTouch.runtimeState,
                    traveledDistanceM);
            }
            return LoopController::ControlVector::RawMotorPwm(
                Config::kWallTouchSeatRampMaxDriveCommand,
                Config::kWallTouchSeatRampMaxDriveCommand);
        }

        if (wallTouch.runtimeState == WallTouchState::ControlledRelease)
        {
            const float releaseAlpha =
                static_cast<float>((std::min)(stateElapsedMs, static_cast<unsigned long>(Config::kWallTouchReleaseRampMs))) /
                static_cast<float>((std::max)(Config::kWallTouchReleaseRampMs, static_cast<std::uint16_t>(1U)));
            const float forwardPreloadCommand =
                Config::kWallTouchSeatRampMaxDriveCommand * (1.0f - releaseAlpha);
            float reverseCommand = 0.0f;
            if (Config::kWallTouchReleaseReverseOverlapMs >= Config::kWallTouchReleaseRampMs)
            {
                reverseCommand = Config::kWallTouchReleaseReverseDriveCommand * releaseAlpha;
            }
            else if (stateElapsedMs >= (Config::kWallTouchReleaseRampMs - Config::kWallTouchReleaseReverseOverlapMs))
            {
                const unsigned long reverseElapsedMs =
                    stateElapsedMs - (Config::kWallTouchReleaseRampMs - Config::kWallTouchReleaseReverseOverlapMs);
                const float reverseAlpha =
                    static_cast<float>((std::min)(reverseElapsedMs, static_cast<unsigned long>(Config::kWallTouchReleaseReverseOverlapMs))) /
                    static_cast<float>((std::max)(Config::kWallTouchReleaseReverseOverlapMs, static_cast<std::uint16_t>(1U)));
                reverseCommand = Config::kWallTouchReleaseReverseDriveCommand * reverseAlpha;
            }

            wallTouch.result.reverseDistanceM = (std::max)(0.0f, wallTouch.result.seatedTravelM - traveledDistanceM);
            if ((wallTouch.result.reverseDistanceM >= Config::kDistanceToleranceM) && !frontSignalActive)
            {
                char line[224] = {};
                snprintf(
                    line,
                    sizeof(line),
                    "startup_cal_touch:release_clear,reverse_m=%.4f,elapsed_ms=%lu",
                    wallTouch.result.reverseDistanceM,
                    stateElapsedMs);
                appendTraceLine(line);
                wallTouch.runtimeState = WallTouchState::ReverseToClearance;
                wallTouch.stateStartMs = nowMs;
                traceStateTransition(
                    WallTouchState::ControlledRelease,
                    wallTouch.runtimeState,
                    traveledDistanceM);
            }
            else if (stateElapsedMs >= Config::kWallTouchReleaseRampMs)
            {
                wallTouch.runtimeState = WallTouchState::ReverseToClearance;
                wallTouch.stateStartMs = nowMs;
                traceStateTransition(
                    WallTouchState::ControlledRelease,
                    wallTouch.runtimeState,
                    traveledDistanceM);
            }
            return LoopController::ControlVector::RawMotorPwm(
                forwardPreloadCommand - reverseCommand,
                forwardPreloadCommand - reverseCommand);
        }

        if (wallTouch.runtimeState == WallTouchState::ReverseToClearance)
        {
            wallTouch.result.reverseDistanceM = (std::max)(0.0f, wallTouch.result.seatedTravelM - traveledDistanceM);
            const float headingErrorRad = AngleErrorRad(wallTouch.targetYawRad, pose.yawRad);
            float angularCommandRadps = Config::kStraightHeadingKp * headingErrorRad;
            angularCommandRadps = (std::clamp)(
                angularCommandRadps,
                -Config::kWallTouchReverseMaxAngularCommandRadps,
                Config::kWallTouchReverseMaxAngularCommandRadps);

            if (wallTouch.result.reverseDistanceM >= Config::kWallTouchFrontClearanceDistanceM)
            {
                char line[224] = {};
                snprintf(
                    line,
                    sizeof(line),
                    "startup_cal_touch:clearance_reached,reverse_m=%.4f,target_m=%.4f",
                    wallTouch.result.reverseDistanceM,
                    Config::kWallTouchFrontClearanceDistanceM);
                appendTraceLine(line);
                return complete();
            }

            if ((stateElapsedMs >= Config::kMotionSettleTimeoutMs) && frontSignalActive)
            {
                char line[192] = {};
                snprintf(
                    line,
                    sizeof(line),
                    "startup_cal_touch:clearance_failed,reverse_m=%.4f,elapsed_ms=%lu",
                    wallTouch.result.reverseDistanceM,
                    stateElapsedMs);
                appendTraceLine(line);
                return fail("Wall touch-off failed to establish front-wall clearance");
            }

            return drive.PointControlVector(
                -Config::kWallTouchReverseSpeedMps,
                angularCommandRadps,
                trackingCommandPd);
        }

        return complete();
    }
}
