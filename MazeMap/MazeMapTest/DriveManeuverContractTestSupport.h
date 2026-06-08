#pragma once

#include "EstimatorTestSupport.h"
#include "..\MazeMap\CoreConfig.h"
#include "..\MazeMap\Defines.h"
#include "..\MazeMap\DirectionalLocation.h"
#include "..\MazeMap\Drive.h"
#include "..\MazeMap\DriveBase.h"
#include "..\MazeMap\DriveTelemetry.h"
#include "..\MazeMap\Estimator.h"
#include "..\MazeMap\Maneuver.h"
#include "..\MazeMap\ManeuverInstance.h"
#include "..\MazeMap\ManeuverSet.h"
#include "..\MazeMap\MazeMapRuntimeCore.h"
#include "..\MazeMap\MotionLimits.h"
#include "..\MazeMap\PlantModel.h"
#include "..\MazeMap\SensorSnapshot.h"
#include "..\MazeMap\SharedRobotRuntime.h"
#include "..\MazeMap\Vehicle.h"
#include "..\MazeMap\VehicleState.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numeric>
#include <string>
#include <vector>

#include "CppUnitTest.h"
namespace MazeMap::App::DriveManeuverContractTestSupport
{
    using CommandVector = Internal::CommandVector;

    constexpr float kSimulationDtSeconds = 0.001f;
    constexpr int kMaxSimulationSteps = 20000;
    constexpr float kSmoothEntrySpeedMps = 0.5f;
    constexpr float kInPlacePositionToleranceM = 0.020f;
    constexpr float kSmoothPositionToleranceM = 0.030f;
    constexpr float kHeadingToleranceRad = 3.0f * DEG_TO_RAD_F;
    constexpr float kTimeToleranceFraction = 0.40f;
    constexpr float kVelocityVariationLimit = 0.05f;
    constexpr float kYawAccelerationVariationLimit = 0.20f;
    constexpr float kYawRateVariationLimit = 0.08f;
    constexpr float kYawRateMagnitudeEpsilonRadps = 1.0e-4f;
    constexpr float kTurnPlateauFraction = 0.95f;
    constexpr std::size_t kRampDeltaTrimSamples = 5U;

    struct ScopedMissionFanDuty final
    {
        explicit ScopedMissionFanDuty(Vehicle& vehicle, const float dutyCycle) noexcept
            : targetVehicle(vehicle)
            , previousDutyCycle(vehicle.GetFanDuty())
        {
            targetVehicle.SetFanDuty(dutyCycle);
        }

        ~ScopedMissionFanDuty() noexcept
        {
            targetVehicle.SetFanDuty(previousDutyCycle);
        }

        Vehicle& targetVehicle;
        float previousDutyCycle = 0.0f;
    };

    struct CommandSample final
    {
        float timeSeconds = 0.0f;
        float linearCommandMps = 0.0f;
        float angularCommandRadps = 0.0f;
    };

    struct ManeuverExecutionTrace final
    {
        bool initialPoseReset = false;
        bool entryPoseReset = false;
        bool started = false;
        bool completed = false;
        bool estimatorFault = false;
        bool leftReturnedCommandFinite = false;
        bool rightReturnedCommandFinite = false;
        bool bodyProposalEvidenceSet = false;
        bool commandTelemetryEvidenceSet = false;
        bool leftCommandEvidenceMatchesReturnedCommand = false;
        bool rightCommandEvidenceMatchesReturnedCommand = false;
        bool requestedForwardMpsFinite = false;
        bool requestedYawRateRadpsFinite = false;
        bool requestedForwardAccelMps2Finite = false;
        bool requestedYawAccelRadps2Finite = false;
        bool requestedYawRadFinite = false;
        float elapsedSeconds = 0.0f;
        CommandVector lastReturnedCommand;
        DriveTelemetry lastTelemetry{};
        VehicleState runtimeState;
        VehicleState truthState;
        std::vector<CommandSample> samples;
    };

    inline const wchar_t* BoolText(const bool value) noexcept
    {
        return value ? L"true" : L"false";
    }

    inline std::wstring BuildTraceStatusMessage(const ManeuverExecutionTrace& trace)
    {
        return
            std::wstring(L" initial_pose_reset=") + BoolText(trace.initialPoseReset) +
            L" entry_pose_reset=" + BoolText(trace.entryPoseReset) +
            L" started=" + BoolText(trace.started) +
            L" completed=" + BoolText(trace.completed) +
            L" estimator_fault=" + BoolText(trace.estimatorFault) +
            L" samples=" + std::to_wstring(trace.samples.size()) +
            L" elapsed_s=" + std::to_wstring(trace.elapsedSeconds) +
            L" encoder_avg_m=" + std::to_wstring(trace.runtimeState.GetSensorSnapshot().AverageEncoderDistanceM()) +
            L" runtime_vf_mps=" + std::to_wstring(trace.runtimeState.GetForwardVelocity()) +
            L" runtime_yaw_rate_radps=" + std::to_wstring(trace.runtimeState.GetYawRate()) +
            L" truth_vf_mps=" + std::to_wstring(trace.truthState.GetForwardVelocity()) +
            L" truth_yaw_rate_radps=" + std::to_wstring(trace.truthState.GetYawRate());
    }

    inline bool AccumulateSampleFlag(
        const bool hasPriorSample,
        const bool previousValue,
        const bool currentValue) noexcept
    {
        return hasPriorSample ? (previousValue && currentValue) : currentValue;
    }

    inline SensorSnapshot BuildDriveManeuverSensorSnapshot(const float yawRateRadps = 0.0f) noexcept
    {
        SensorSnapshot snapshot{};
        snapshot.SetRawYawRateRadps(yawRateRadps);
        snapshot.SetYawRateRadps(yawRateRadps);
        return snapshot;
    }

    inline std::int32_t ConsumeWholeEncoderCounts(
        const float deltaCounts,
        float& remainderCounts) noexcept
    {
        remainderCounts += deltaCounts;
        const std::int32_t wholeCounts =
            (remainderCounts >= 0.0f) ?
            static_cast<std::int32_t>(std::floor(remainderCounts)) :
            static_cast<std::int32_t>(std::ceil(remainderCounts));
        remainderCounts -= static_cast<float>(wholeCounts);
        return wholeCounts;
    }

    inline std::int32_t EncoderDeltaCountsFromWheelSpeedRadps(
        const float wheelSpeedRadps,
        const float dtSeconds) noexcept
    {
        const float distancePerCountM = MazeMap::Vehicle::DriveEncoderDistanceFromCounts(1);
        if (!std::isfinite(wheelSpeedRadps) ||
            !std::isfinite(dtSeconds) ||
            !(dtSeconds > 0.0f) ||
            !(distancePerCountM > 0.0f))
        {
            return 0;
        }

        return static_cast<std::int32_t>(
            std::lround(
                (MazeMap::Vehicle::WheelLinearVelocityFromWheelSpeed(wheelSpeedRadps) * dtSeconds) /
                distancePerCountM));
    }

    inline void PublishInitialEncoderObservationForWheelSpeedsRadps(
        VehicleState& state,
        const float leftWheelSpeedRadps,
        const float rightWheelSpeedRadps) noexcept
    {
        const std::int32_t leftCounts =
            EncoderDeltaCountsFromWheelSpeedRadps(leftWheelSpeedRadps, kSimulationDtSeconds);
        const std::int32_t rightCounts =
            EncoderDeltaCountsFromWheelSpeedRadps(rightWheelSpeedRadps, kSimulationDtSeconds);
        const float leftDistanceDeltaM =
            MazeMap::Vehicle::DriveEncoderDistanceFromCounts(leftCounts);
        const float rightDistanceDeltaM =
            MazeMap::Vehicle::DriveEncoderDistanceFromCounts(rightCounts);

        SensorSnapshot::EncoderObs encoderObservation = SensorSnapshot{}.EncoderObservation();
        encoderObservation.SetTotalLeftCounts(leftCounts);
        encoderObservation.SetTotalRightCounts(rightCounts);
        encoderObservation.SetLeftDistanceDeltaM(leftDistanceDeltaM);
        encoderObservation.SetRightDistanceDeltaM(rightDistanceDeltaM);
        encoderObservation.SetLeftVelocityMps(leftDistanceDeltaM / kSimulationDtSeconds);
        encoderObservation.SetRightVelocityMps(rightDistanceDeltaM / kSimulationDtSeconds);
        encoderObservation.SetLeftWheelSpeedRadps(
            MazeMap::Vehicle::WheelSpeedFromLinearVelocity(encoderObservation.LeftVelocityMps()));
        encoderObservation.SetRightWheelSpeedRadps(
            MazeMap::Vehicle::WheelSpeedFromLinearVelocity(encoderObservation.RightVelocityMps()));

        SensorSnapshot snapshot = state.GetSensorSnapshot();
        snapshot.PublishEncoderObservation(
            encoderObservation,
            true,
            leftCounts,
            rightCounts,
            leftDistanceDeltaM,
            rightDistanceDeltaM);
        state.SetSensorSnapshot(snapshot);
    }

    inline void ApplyEncoderObservation(
        Estimator& estimator,
        VehicleState& runtimeState,
        const float leftDistanceDeltaM,
        const float rightDistanceDeltaM,
        const float yawRateRadps,
        float& leftEncoderRemainderCounts,
        float& rightEncoderRemainderCounts,
        const float dtSeconds,
        const CommandVector& appliedControl = MakeControlVector())
    {
        const float distancePerCountM = MazeMap::Vehicle::DriveEncoderDistanceFromCounts(1);
        const std::int32_t leftCounts =
            ConsumeWholeEncoderCounts(leftDistanceDeltaM / distancePerCountM, leftEncoderRemainderCounts);
        const std::int32_t rightCounts =
            ConsumeWholeEncoderCounts(rightDistanceDeltaM / distancePerCountM, rightEncoderRemainderCounts);

        SensorSnapshot snapshot = BuildDriveManeuverSensorSnapshot(yawRateRadps);
        SensorSnapshot::EncoderObs encoderObservation = SensorSnapshot{}.EncoderObservation();
        encoderObservation.SetTotalLeftCounts(leftCounts);
        encoderObservation.SetTotalRightCounts(rightCounts);
        encoderObservation.SetLeftDistanceDeltaM(static_cast<float>(leftCounts) * distancePerCountM);
        encoderObservation.SetRightDistanceDeltaM(static_cast<float>(rightCounts) * distancePerCountM);
        if ((dtSeconds > 0.0f) && std::isfinite(dtSeconds) && (MazeMap::Vehicle::GetDriveWheelRadiusM() > 0.0f))
        {
            const float invWheelRadiusM = 1.0f / MazeMap::Vehicle::GetDriveWheelRadiusM();
            const float invDtSeconds = 1.0f / dtSeconds;
            encoderObservation.SetLeftVelocityMps(encoderObservation.LeftDistanceDeltaM() * invDtSeconds);
            encoderObservation.SetRightVelocityMps(encoderObservation.RightDistanceDeltaM() * invDtSeconds);
            encoderObservation.SetLeftWheelSpeedRadps(encoderObservation.LeftVelocityMps() * invWheelRadiusM);
            encoderObservation.SetRightWheelSpeedRadps(encoderObservation.RightVelocityMps() * invWheelRadiusM);
        }
        const std::int64_t leftTotalCounts =
            runtimeState.GetSensorSnapshot().LeftEncoderTotalCounts() + static_cast<std::int64_t>(leftCounts);
        const std::int64_t rightTotalCounts =
            runtimeState.GetSensorSnapshot().RightEncoderTotalCounts() + static_cast<std::int64_t>(rightCounts);
        snapshot.PublishEncoderObservation(
            encoderObservation,
            true,
            leftTotalCounts,
            rightTotalCounts,
            MazeMap::Vehicle::DriveEncoderDistanceFromCounts(leftTotalCounts),
            MazeMap::Vehicle::DriveEncoderDistanceFromCounts(rightTotalCounts));
        UpdateDriveEstimator(
            estimator,
            runtimeState,
            dtSeconds,
            snapshot,
            appliedControl);
    }


    inline VehicleState BuildTruthState(const float linearSpeedMps) noexcept
    {
        const float wheelSpeedRadps = Vehicle::WheelSpeedFromLinearVelocity(linearSpeedMps);
        VehicleState state;
        state.SetForwardVelocity(linearSpeedMps);
        PublishInitialEncoderObservationForWheelSpeedsRadps(state, wheelSpeedRadps, wheelSpeedRadps);
        return state;
    }

    inline void ComputeWheelSpeedsFromBodyState(
        const VehicleState& state,
        float& leftWheelSpeedRadps,
        float& rightWheelSpeedRadps) noexcept
    {
        Vehicle::WheelSpeedsFromBodyVelocity(
            state.GetForwardVelocity(),
            state.GetYawRate(),
            leftWheelSpeedRadps,
            rightWheelSpeedRadps);
    }

    inline bool PrimeDriveForSmoothEntry(
        Internal::SharedRobotRuntime& runtime,
        VehicleState& truthState,
        float& leftEncoderRemainderCounts,
        float& rightEncoderRemainderCounts)
    {
        truthState = BuildTruthState(kSmoothEntrySpeedMps);
        const float distancePerCountM = MazeMap::Vehicle::DriveEncoderDistanceFromCounts(1);
        float projectedLeftEncoderRemainderCounts = leftEncoderRemainderCounts;
        float projectedRightEncoderRemainderCounts = rightEncoderRemainderCounts;
        const std::int32_t projectedLeftCounts = ConsumeWholeEncoderCounts(
            (kSmoothEntrySpeedMps * kSimulationDtSeconds) / distancePerCountM,
            projectedLeftEncoderRemainderCounts);
        const std::int32_t projectedRightCounts = ConsumeWholeEncoderCounts(
            (kSmoothEntrySpeedMps * kSimulationDtSeconds) / distancePerCountM,
            projectedRightEncoderRemainderCounts);
        const float projectedForwardDistanceM =
            0.5f * static_cast<float>(projectedLeftCounts + projectedRightCounts) * distancePerCountM;
        if (!runtime.Estimator().ResetPose(0.0f, -projectedForwardDistanceM, 0.0f))
        {
            return false;
        }

        ApplyEncoderObservation(
            runtime.Estimator(),
            runtime.RuntimeState(),
            kSmoothEntrySpeedMps * kSimulationDtSeconds,
            kSmoothEntrySpeedMps * kSimulationDtSeconds,
            0.0f,
            leftEncoderRemainderCounts,
            rightEncoderRemainderCounts,
            kSimulationDtSeconds);
        truthState.SetSensorSnapshot(runtime.RuntimeState().GetSensorSnapshot());
        return true;
    }

    inline void SimulateRuntimeDriveCycle(
        Internal::SharedRobotRuntime& runtime,
        PlantModel& truthPlant,
        VehicleState& truthState,
        float& leftEncoderRemainderCounts,
        float& rightEncoderRemainderCounts,
        const float dtSeconds,
        const CommandVector& control)
    {
		Microsoft::VisualStudio::CppUnitTestFramework::Assert::AreEqual(dtSeconds, 0.001f, 1e-9f, L"SimulateRuntimeDriveCycle should be called with the canonical dt");
        float previousLeftWheelSpeedRadps = 0.0f;
        float previousRightWheelSpeedRadps = 0.0f;
        ComputeWheelSpeedsFromBodyState(
            truthState,
            previousLeftWheelSpeedRadps,
            previousRightWheelSpeedRadps);
        truthPlant.integrate(control, dtSeconds);
        float currentLeftWheelSpeedRadps = 0.0f;
        float currentRightWheelSpeedRadps = 0.0f;
        ComputeWheelSpeedsFromBodyState(
            truthState,
            currentLeftWheelSpeedRadps,
            currentRightWheelSpeedRadps);

        const float leftDistanceDeltaM =
            0.5f *
            (previousLeftWheelSpeedRadps + currentLeftWheelSpeedRadps) *
            Vehicle::GetDriveWheelRadiusM() *
            dtSeconds;
        const float rightDistanceDeltaM =
            0.5f *
            (previousRightWheelSpeedRadps + currentRightWheelSpeedRadps) *
            Vehicle::GetDriveWheelRadiusM() *
            dtSeconds;

        ApplyEncoderObservation(
            runtime.Estimator(),
            runtime.RuntimeState(),
            leftDistanceDeltaM,
            rightDistanceDeltaM,
            truthState.GetYawRate(),
            leftEncoderRemainderCounts,
            rightEncoderRemainderCounts,
            dtSeconds,
            control);
        truthState.SetSensorSnapshot(runtime.RuntimeState().GetSensorSnapshot());
    }

    inline DirectionalLocation BuildManeuverStart() noexcept
    {
        return DirectionalLocation(MazeLocation(0U, 0U), Up);
    }

    inline DirectionalLocation BuildNominalEndLocation(const ManeuverCode code)
    {
        return ManeuverSet::GetSet().Move(code, BuildManeuverStart());
    }

    inline float BuildNominalEndXMeters(const ManeuverCode code) noexcept
    {
        const DirectionalLocation nominalEnd = BuildNominalEndLocation(code);
        return 0.5f * Config::kCellSizeM * static_cast<float>(nominalEnd.GetLocation().GetX());
    }

    inline float BuildNominalEndYMeters(const ManeuverCode code) noexcept
    {
        const DirectionalLocation nominalEnd = BuildNominalEndLocation(code);
        return 0.5f * Config::kCellSizeM * static_cast<float>(nominalEnd.GetLocation().GetY());
    }

    inline float BuildNominalEndYawRad(const ManeuverCode code) noexcept
    {
        return DirectionToYawRad(BuildNominalEndLocation(code).GetDirection());
    }

    inline float ComputeInPlaceTurnKinematicTimeSeconds(
        const float angleRad,
        const MotionLimits& limits) noexcept
    {
        return limits.ComputeMinimumTurnDurationSeconds(angleRad);
    }

    inline std::wstring CodeLabel(const ManeuverCode code)
    {
        switch (code)
        {
        case IP45: return L"IP45";
        case IP90: return L"IP90";
        case IP135: return L"IP135";
        case IP180: return L"IP180";
        case S45LS: return L"S45LS";
        case S45LD: return L"S45LD";
        case S45SS: return L"S45SS";
        case S45SD: return L"S45SD";
        case S90LS: return L"S90LS";
        case S90SS: return L"S90SS";
        case S90SD: return L"S90SD";
        case S135LS: return L"S135LS";
        case S135LD: return L"S135LD";
        case S135SS: return L"S135SS";
        case S135SD: return L"S135SD";
        case S180LS: return L"S180LS";
        case S180SS: return L"S180SS";
        default: return L"UNKNOWN";
        }
    }

    inline ManeuverExecutionTrace SimulateDriveManeuver(
        const ManeuverCode code,
        const bool smoothTurn)
    {
        ManeuverExecutionTrace trace{};
        Internal::SharedRobotRuntime runtime(kSimulationDtSeconds);
        ScopedMissionFanDuty fanDuty(runtime.Vehicle(), 0.80f);
        float leftEncoderRemainderCounts = 0.0f;
        float rightEncoderRemainderCounts = 0.0f;

        runtime.DriveBase().ClearCommandEvidence();

        trace.initialPoseReset = runtime.Estimator().ResetPose(0.0f, 0.0f, 0.0f);
        if (!trace.initialPoseReset)
        {
            return trace;
        }

        if (smoothTurn)
        {
            trace.entryPoseReset = PrimeDriveForSmoothEntry(
                runtime,
                trace.truthState,
                leftEncoderRemainderCounts,
                rightEncoderRemainderCounts);
            if (!trace.entryPoseReset)
            {
                return trace;
            }
            trace.runtimeState = runtime.RuntimeState();
        }
        else
        {
            trace.entryPoseReset = true;
            trace.truthState = BuildTruthState(0.0f);
            trace.runtimeState = runtime.RuntimeState();
        }
        PlantModel truthPlant(runtime.Vehicle(), trace.truthState);

        ManeuverInstance maneuver(
            code,
            BuildManeuverStart(),
            smoothTurn ? kSmoothEntrySpeedMps : 0.0f,
            smoothTurn ? kSmoothEntrySpeedMps : 0.0f);

        Internal::Drive& driveService = runtime.DriveService();
        driveService.StartManeuver(maneuver);
        trace.started = true;

        for (int stepIndex = 0; stepIndex < kMaxSimulationSteps; ++stepIndex)
        {
            if (runtime.Estimator().HasFault())
            {
                trace.estimatorFault = true;
                return trace;
            }

            bool done = false;
            const CommandVector control = driveService.GetNextControls(done);
            if (done)
            {
                trace.completed = true;
                trace.runtimeState = runtime.RuntimeState();
                return trace;
            }

            const DriveTelemetry& telemetry = runtime.DriveBase().LastTelemetry();
            const bool hasPriorSample = !trace.samples.empty();
            const bool leftReturnedCommandFinite = std::isfinite(control.LeftCommand());
            const bool rightReturnedCommandFinite = std::isfinite(control.RightCommand());
            const bool bodyProposalEvidenceSet =
                (telemetry.commandKindFlags & DriveTelemetry::kCommandKindBodyProposal) != 0U;
            const bool commandTelemetryEvidenceSet =
                (telemetry.telemetryValidFlags & DriveTelemetry::kTelemetryCommandEvidenceValid) != 0U;
            const bool leftCommandEvidenceMatchesReturnedCommand =
                std::fabs(control.LeftCommand() - telemetry.leftDriveCommand) <= 1.0e-6f;
            const bool rightCommandEvidenceMatchesReturnedCommand =
                std::fabs(control.RightCommand() - telemetry.rightDriveCommand) <= 1.0e-6f;
            const bool requestedForwardMpsFinite = std::isfinite(telemetry.requestedForwardMps);
            const bool requestedYawRateRadpsFinite = std::isfinite(telemetry.requestedYawRateRadps);
            const bool requestedForwardAccelMps2Finite = std::isfinite(telemetry.requestedForwardAccelMps2);
            const bool requestedYawAccelRadps2Finite = std::isfinite(telemetry.requestedYawAccelRadps2);
            const bool requestedYawRadFinite = std::isfinite(telemetry.requestedYawRad);
            trace.leftReturnedCommandFinite =
                AccumulateSampleFlag(hasPriorSample, trace.leftReturnedCommandFinite, leftReturnedCommandFinite);
            trace.rightReturnedCommandFinite =
                AccumulateSampleFlag(hasPriorSample, trace.rightReturnedCommandFinite, rightReturnedCommandFinite);
            trace.bodyProposalEvidenceSet =
                AccumulateSampleFlag(hasPriorSample, trace.bodyProposalEvidenceSet, bodyProposalEvidenceSet);
            trace.commandTelemetryEvidenceSet =
                AccumulateSampleFlag(hasPriorSample, trace.commandTelemetryEvidenceSet, commandTelemetryEvidenceSet);
            trace.leftCommandEvidenceMatchesReturnedCommand =
                AccumulateSampleFlag(
                    hasPriorSample,
                    trace.leftCommandEvidenceMatchesReturnedCommand,
                    leftCommandEvidenceMatchesReturnedCommand);
            trace.rightCommandEvidenceMatchesReturnedCommand =
                AccumulateSampleFlag(
                    hasPriorSample,
                    trace.rightCommandEvidenceMatchesReturnedCommand,
                    rightCommandEvidenceMatchesReturnedCommand);
            trace.requestedForwardMpsFinite =
                AccumulateSampleFlag(hasPriorSample, trace.requestedForwardMpsFinite, requestedForwardMpsFinite);
            trace.requestedYawRateRadpsFinite =
                AccumulateSampleFlag(hasPriorSample, trace.requestedYawRateRadpsFinite, requestedYawRateRadpsFinite);
            trace.requestedForwardAccelMps2Finite =
                AccumulateSampleFlag(
                    hasPriorSample,
                    trace.requestedForwardAccelMps2Finite,
                    requestedForwardAccelMps2Finite);
            trace.requestedYawAccelRadps2Finite =
                AccumulateSampleFlag(
                    hasPriorSample,
                    trace.requestedYawAccelRadps2Finite,
                    requestedYawAccelRadps2Finite);
            trace.requestedYawRadFinite =
                AccumulateSampleFlag(hasPriorSample, trace.requestedYawRadFinite, requestedYawRadFinite);
            trace.lastReturnedCommand = control;
            trace.lastTelemetry = telemetry;

            trace.samples.push_back(
                CommandSample{
                    trace.elapsedSeconds,
                    telemetry.requestedForwardMps,
                    telemetry.requestedYawRateRadps
                });

            SimulateRuntimeDriveCycle(
                runtime,
                truthPlant,
                trace.truthState,
                leftEncoderRemainderCounts,
                rightEncoderRemainderCounts,
                kSimulationDtSeconds,
                control);
            trace.runtimeState = runtime.RuntimeState();
            trace.elapsedSeconds += kSimulationDtSeconds;
        }

        return trace;
    }

    inline std::wstring BuildManeuverMessage(
        const wchar_t* const field,
        const ManeuverCode code,
        const ManeuverExecutionTrace& trace)
    {
        return std::wstring(field) + L" code=" + CodeLabel(code) + BuildTraceStatusMessage(trace);
    }

    inline float ComputeNormalizedSpan(const std::vector<float>& values) noexcept
    {
        if (values.size() < 2U)
        {
            return (std::numeric_limits<float>::infinity)();
        }

        const auto minmax = std::minmax_element(values.begin(), values.end());
        const float average =
            std::accumulate(values.begin(), values.end(), 0.0f) /
            static_cast<float>(values.size());
        if (!(average > 0.0f) || !std::isfinite(average))
        {
            return (std::numeric_limits<float>::infinity)();
        }

        return (*minmax.second - *minmax.first) / average;
    }

    inline std::vector<float> CollectLinearCommandMagnitudes(const ManeuverExecutionTrace& trace)
    {
        std::vector<float> magnitudes;
        magnitudes.reserve(trace.samples.size());
        for (const CommandSample& sample : trace.samples)
        {
            magnitudes.push_back(std::fabs(sample.linearCommandMps));
        }
        return magnitudes;
    }

    inline std::vector<float> CollectTurnYawRateMagnitudes(const ManeuverExecutionTrace& trace)
    {
        std::vector<float> magnitudes;
        if (trace.samples.empty())
        {
            return magnitudes;
        }

        float maxYawRateMagnitudeRadps = 0.0f;
        for (const CommandSample& sample : trace.samples)
        {
            maxYawRateMagnitudeRadps =
                (std::max)(maxYawRateMagnitudeRadps, std::fabs(sample.angularCommandRadps));
        }

        const float plateauThresholdRadps = kTurnPlateauFraction * maxYawRateMagnitudeRadps;
        for (const CommandSample& sample : trace.samples)
        {
            const float magnitudeRadps = std::fabs(sample.angularCommandRadps);
            if (magnitudeRadps >= plateauThresholdRadps)
            {
                magnitudes.push_back(magnitudeRadps);
            }
        }

        return magnitudes;
    }

    inline std::vector<float> CollectRampYawAccelMagnitudes(const ManeuverExecutionTrace& trace)
    {
        std::vector<float> magnitudes;
        if (trace.samples.size() < 3U)
        {
            return magnitudes;
        }

        float maxYawRateMagnitudeRadps = 0.0f;
        std::size_t plateauBeginIndex = trace.samples.size();
        std::size_t plateauEndIndex = 0U;
        for (std::size_t index = 0U; index < trace.samples.size(); ++index)
        {
            maxYawRateMagnitudeRadps =
                (std::max)(
                    maxYawRateMagnitudeRadps,
                    std::fabs(trace.samples[index].angularCommandRadps));
        }

        const float plateauThresholdRadps = kTurnPlateauFraction * maxYawRateMagnitudeRadps;
        for (std::size_t index = 0U; index < trace.samples.size(); ++index)
        {
            if (std::fabs(trace.samples[index].angularCommandRadps) >= plateauThresholdRadps)
            {
                plateauBeginIndex = (std::min)(plateauBeginIndex, index);
                plateauEndIndex = index;
            }
        }

        if (plateauBeginIndex == trace.samples.size())
        {
            return magnitudes;
        }

        const auto appendTrimmedRegionMagnitudes =
            [&](const std::size_t deltaBeginIndex, const std::size_t deltaEndIndexExclusive)
        {
            if (deltaBeginIndex >= deltaEndIndexExclusive)
            {
                return;
            }

            const std::size_t regionLength = deltaEndIndexExclusive - deltaBeginIndex;
            if (regionLength <= (2U * kRampDeltaTrimSamples))
            {
                return;
            }

            const std::size_t trimmedBeginIndex = deltaBeginIndex + kRampDeltaTrimSamples;
            const std::size_t trimmedEndIndexExclusive = deltaEndIndexExclusive - kRampDeltaTrimSamples;
            for (std::size_t index = trimmedBeginIndex; index < trimmedEndIndexExclusive; ++index)
            {
                const float previousYawRateMagnitudeRadps =
                    std::fabs(trace.samples[index - 1U].angularCommandRadps);
                const float currentYawRateMagnitudeRadps =
                    std::fabs(trace.samples[index].angularCommandRadps);
                if ((previousYawRateMagnitudeRadps <= kYawRateMagnitudeEpsilonRadps) ||
                    (currentYawRateMagnitudeRadps <= kYawRateMagnitudeEpsilonRadps))
                {
                    continue;
                }

                const float dtSeconds = trace.samples[index].timeSeconds - trace.samples[index - 1U].timeSeconds;
                if (!(dtSeconds > 0.0f))
                {
                    continue;
                }

                magnitudes.push_back(
                    std::fabs(
                        (trace.samples[index].angularCommandRadps - trace.samples[index - 1U].angularCommandRadps) /
                        dtSeconds));
            }
        };

        appendTrimmedRegionMagnitudes(1U, plateauBeginIndex + 1U);
        appendTrimmedRegionMagnitudes(plateauEndIndex + 1U, trace.samples.size());

        return magnitudes;
    }

    inline float InvalidManeuverMeasurement() noexcept
    {
        return (std::numeric_limits<float>::infinity)();
    }

    inline float ComputeInPlaceShiftMeters(const ManeuverExecutionTrace& trace) noexcept
    {
        if (!trace.completed)
        {
            return InvalidManeuverMeasurement();
        }

        return
            std::hypot(
                trace.truthState.GetPositionX(),
                trace.truthState.GetPositionY());
    }

    inline float ComputeInPlaceHeadingErrorRad(
        const ManeuverCode code,
        const ManeuverExecutionTrace& trace) noexcept
    {
        if (!trace.completed)
        {
            return InvalidManeuverMeasurement();
        }

        return std::fabs(AngleErrorRad(BuildNominalEndYawRad(code), trace.truthState.GetHeading()));
    }

    inline float ComputeInPlaceExpectedTimeSeconds(const ManeuverCode code)
    {
        Internal::SharedRobotRuntime runtime(kSimulationDtSeconds);
        return
            ComputeInPlaceTurnKinematicTimeSeconds(
                std::fabs(BuildNominalEndYawRad(code)),
                runtime.DriveService().GetLimits());
    }

    inline float ComputeInPlaceRelativeTimeError(
        const ManeuverCode code,
        const ManeuverExecutionTrace& trace)
    {
        if (!trace.completed)
        {
            return InvalidManeuverMeasurement();
        }

        const float expectedTimeSeconds = ComputeInPlaceExpectedTimeSeconds(code);
        return
            (expectedTimeSeconds > 0.0f) ?
            (std::fabs(trace.elapsedSeconds - expectedTimeSeconds) / expectedTimeSeconds) :
            (std::numeric_limits<float>::infinity)();
    }

    inline float ComputeSmoothVelocityNormalizedSpan(const ManeuverExecutionTrace& trace)
    {
        return trace.completed ?
            ComputeNormalizedSpan(CollectLinearCommandMagnitudes(trace)) :
            InvalidManeuverMeasurement();
    }

    inline float ComputeSmoothYawAccelerationNormalizedSpan(const ManeuverExecutionTrace& trace)
    {
        return trace.completed ?
            ComputeNormalizedSpan(CollectRampYawAccelMagnitudes(trace)) :
            InvalidManeuverMeasurement();
    }

    inline float ComputeSmoothYawRateNormalizedSpan(const ManeuverExecutionTrace& trace)
    {
        return trace.completed ?
            ComputeNormalizedSpan(CollectTurnYawRateMagnitudes(trace)) :
            InvalidManeuverMeasurement();
    }

    inline float ComputeSmoothFinalPositionErrorMeters(
        const ManeuverCode code,
        const ManeuverExecutionTrace& trace) noexcept
    {
        if (!trace.completed)
        {
            return InvalidManeuverMeasurement();
        }

        return
            std::hypot(
                trace.truthState.GetPositionX() - BuildNominalEndXMeters(code),
                trace.truthState.GetPositionY() - BuildNominalEndYMeters(code));
    }

    inline float ComputeSmoothFinalHeadingErrorRad(
        const ManeuverCode code,
        const ManeuverExecutionTrace& trace) noexcept
    {
        if (!trace.completed)
        {
            return InvalidManeuverMeasurement();
        }

        return std::fabs(AngleErrorRad(BuildNominalEndYawRad(code), trace.truthState.GetHeading()));
    }
}
