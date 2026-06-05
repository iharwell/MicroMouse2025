#include "pch.h"
#include "CppUnitTest.h"

#include "EstimatorTestSupport.h"
#include "..\MazeMap\Drive.h"
#include "..\MazeMap\DriveBase.h"
#include "..\MazeMap\ManeuverInstance.h"
#include "..\MazeMap\ManeuverSet.h"
#include "..\MazeMap\PlantModel.h"
#include "..\MazeMap\SharedRobotRuntime.h"
#include "..\MazeMap\Vehicle.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <string>
#include <vector>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap::App
{
    namespace
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
            VehicleState truthState;
            std::vector<CommandSample> samples;
        };

        const wchar_t* BoolText(const bool value) noexcept
        {
            return value ? L"true" : L"false";
        }

        std::wstring BuildTraceStatusMessage(const ManeuverExecutionTrace& trace)
        {
            return
                std::wstring(L" initial_pose_reset=") + BoolText(trace.initialPoseReset) +
                L" entry_pose_reset=" + BoolText(trace.entryPoseReset) +
                L" started=" + BoolText(trace.started) +
                L" completed=" + BoolText(trace.completed) +
                L" estimator_fault=" + BoolText(trace.estimatorFault) +
                L" samples=" + std::to_wstring(trace.samples.size()) +
                L" elapsed_s=" + std::to_wstring(trace.elapsedSeconds);
        }

        bool AccumulateSampleFlag(
            const bool hasPriorSample,
            const bool previousValue,
            const bool currentValue) noexcept
        {
            return hasPriorSample ? (previousValue && currentValue) : currentValue;
        }

        SensorSnapshot BuildDriveManeuverSensorSnapshot(const float yawRateRadps = 0.0f) noexcept
        {
            SensorSnapshot snapshot{};
            snapshot.SetRawYawRateRadps(yawRateRadps);
            snapshot.SetYawRateRadps(yawRateRadps);
            return snapshot;
        }

        int32_t ConsumeWholeEncoderCounts(
            const float deltaCounts,
            float& remainderCounts) noexcept
        {
            remainderCounts += deltaCounts;
            const int32_t wholeCounts =
                (remainderCounts >= 0.0f) ?
                static_cast<int32_t>(std::floor(remainderCounts)) :
                static_cast<int32_t>(std::ceil(remainderCounts));
            remainderCounts -= static_cast<float>(wholeCounts);
            return wholeCounts;
        }

        void ApplyEncoderObservation(
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
            const int32_t leftCounts =
                ConsumeWholeEncoderCounts(leftDistanceDeltaM / distancePerCountM, leftEncoderRemainderCounts);
            const int32_t rightCounts =
                ConsumeWholeEncoderCounts(rightDistanceDeltaM / distancePerCountM, rightEncoderRemainderCounts);

            SensorSnapshot snapshot = BuildDriveManeuverSensorSnapshot(yawRateRadps);
            MazeMap::EncoderObs encoderObservation{};
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
            snapshot.SetEncoderObservation(encoderObservation, true);
            snapshot.SetEncoderTotals(
                runtimeState.GetSensorSnapshot().LeftEncoderTotalCounts() + static_cast<std::int64_t>(leftCounts),
                runtimeState.GetSensorSnapshot().RightEncoderTotalCounts() + static_cast<std::int64_t>(rightCounts));
            snapshot.SetEncoderDistancesM(
                MazeMap::Vehicle::DriveEncoderDistanceFromCounts(snapshot.LeftEncoderTotalCounts()),
                MazeMap::Vehicle::DriveEncoderDistanceFromCounts(snapshot.RightEncoderTotalCounts()));
            UpdateDriveEstimator(
                estimator,
                runtimeState,
                dtSeconds,
                snapshot,
                appliedControl);
        }


        VehicleState BuildTruthState(const float linearSpeedMps) noexcept
        {
            const float wheelSpeedRadps = Vehicle::WheelSpeedFromLinearVelocity(linearSpeedMps);
            VehicleState state;
            state.SetForwardVelocity(linearSpeedMps);
            state.SetWheelSpeedLeft(wheelSpeedRadps);
            state.SetWheelSpeedRight(wheelSpeedRadps);
            return state;
        }

        bool PrimeDriveForSmoothEntry(
            Internal::SharedRobotRuntime& runtime,
            VehicleState& truthState,
            float& leftEncoderRemainderCounts,
            float& rightEncoderRemainderCounts)
        {
            truthState = BuildTruthState(kSmoothEntrySpeedMps);
            const float distancePerCountM = MazeMap::Vehicle::DriveEncoderDistanceFromCounts(1);
            float projectedLeftEncoderRemainderCounts = leftEncoderRemainderCounts;
            float projectedRightEncoderRemainderCounts = rightEncoderRemainderCounts;
            const int32_t projectedLeftCounts = ConsumeWholeEncoderCounts(
                (kSmoothEntrySpeedMps * kSimulationDtSeconds) / distancePerCountM,
                projectedLeftEncoderRemainderCounts);
            const int32_t projectedRightCounts = ConsumeWholeEncoderCounts(
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
            return true;
        }

        void SimulateRuntimeDriveCycle(
            Internal::SharedRobotRuntime& runtime,
            PlantModel& truthPlant,
            VehicleState& truthState,
            float& leftEncoderRemainderCounts,
            float& rightEncoderRemainderCounts,
            const float dtSeconds,
            const CommandVector& control)
        {
            const float previousLeftWheelSpeedRadps = truthState.GetWheelSpeedLeft();
            const float previousRightWheelSpeedRadps = truthState.GetWheelSpeedRight();
            truthPlant.integrate(control, dtSeconds);

            const float leftDistanceDeltaM =
                0.5f *
                (previousLeftWheelSpeedRadps + truthState.GetWheelSpeedLeft()) *
                Vehicle::GetDriveWheelRadiusM() *
                dtSeconds;
            const float rightDistanceDeltaM =
                0.5f *
                (previousRightWheelSpeedRadps + truthState.GetWheelSpeedRight()) *
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
        }

        DirectionalLocation BuildManeuverStart() noexcept
        {
            return DirectionalLocation(MazeLocation(0U, 0U), Up);
        }

        DirectionalLocation BuildNominalEndLocation(const ManeuverCode code)
        {
            return ManeuverSet::GetSet().Move(code, BuildManeuverStart());
        }

        float BuildNominalEndXMeters(const ManeuverCode code) noexcept
        {
            const DirectionalLocation nominalEnd = BuildNominalEndLocation(code);
            return 0.5f * Config::kCellSizeM * static_cast<float>(nominalEnd.GetLocation().GetX());
        }

        float BuildNominalEndYMeters(const ManeuverCode code) noexcept
        {
            const DirectionalLocation nominalEnd = BuildNominalEndLocation(code);
            return 0.5f * Config::kCellSizeM * static_cast<float>(nominalEnd.GetLocation().GetY());
        }

        float BuildNominalEndYawRad(const ManeuverCode code) noexcept
        {
            return DirectionToYawRad(BuildNominalEndLocation(code).GetDirection());
        }

        float ComputeInPlaceTurnKinematicTimeSeconds(
            const float angleRad,
            const MotionLimits& limits) noexcept
        {
            return limits.ComputeMinimumTurnDurationSeconds(angleRad);
        }

        std::wstring CodeLabel(const ManeuverCode code)
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

        ManeuverExecutionTrace SimulateDriveManeuver(
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
            }
            else
            {
                trace.entryPoseReset = true;
                trace.truthState = BuildTruthState(0.0f);
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
                trace.elapsedSeconds += kSimulationDtSeconds;
            }

            return trace;
        }

        std::wstring BuildManeuverMessage(
            const wchar_t* const field,
            const ManeuverCode code,
            const ManeuverExecutionTrace& trace)
        {
            return std::wstring(field) + L" code=" + CodeLabel(code) + BuildTraceStatusMessage(trace);
        }

        float ComputeNormalizedSpan(const std::vector<float>& values) noexcept
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

        std::vector<float> CollectLinearCommandMagnitudes(const ManeuverExecutionTrace& trace)
        {
            std::vector<float> magnitudes;
            magnitudes.reserve(trace.samples.size());
            for (const CommandSample& sample : trace.samples)
            {
                magnitudes.push_back(std::fabs(sample.linearCommandMps));
            }
            return magnitudes;
        }

        std::vector<float> CollectTurnYawRateMagnitudes(const ManeuverExecutionTrace& trace)
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

        std::vector<float> CollectRampYawAccelMagnitudes(const ManeuverExecutionTrace& trace)
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

        float InvalidManeuverMeasurement() noexcept
        {
            return (std::numeric_limits<float>::infinity)();
        }

        float ComputeInPlaceShiftMeters(const ManeuverExecutionTrace& trace) noexcept
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

        float ComputeInPlaceHeadingErrorRad(
            const ManeuverCode code,
            const ManeuverExecutionTrace& trace) noexcept
        {
            if (!trace.completed)
            {
                return InvalidManeuverMeasurement();
            }

            return std::fabs(AngleErrorRad(BuildNominalEndYawRad(code), trace.truthState.GetHeading()));
        }

        float ComputeInPlaceExpectedTimeSeconds(const ManeuverCode code)
        {
            Internal::SharedRobotRuntime runtime(kSimulationDtSeconds);
            return
                ComputeInPlaceTurnKinematicTimeSeconds(
                    std::fabs(BuildNominalEndYawRad(code)),
                    runtime.DriveService().GetLimits());
        }

        float ComputeInPlaceRelativeTimeError(
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

        float ComputeSmoothVelocityNormalizedSpan(const ManeuverExecutionTrace& trace)
        {
            return trace.completed ?
                ComputeNormalizedSpan(CollectLinearCommandMagnitudes(trace)) :
                InvalidManeuverMeasurement();
        }

        float ComputeSmoothYawAccelerationNormalizedSpan(const ManeuverExecutionTrace& trace)
        {
            return trace.completed ?
                ComputeNormalizedSpan(CollectRampYawAccelMagnitudes(trace)) :
                InvalidManeuverMeasurement();
        }

        float ComputeSmoothYawRateNormalizedSpan(const ManeuverExecutionTrace& trace)
        {
            return trace.completed ?
                ComputeNormalizedSpan(CollectTurnYawRateMagnitudes(trace)) :
                InvalidManeuverMeasurement();
        }

        float ComputeSmoothFinalPositionErrorMeters(
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

        float ComputeSmoothFinalHeadingErrorRad(
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
    TEST_CLASS(DriveManeuverIP45ContractTest)
    {
        static constexpr ManeuverCode kCode = IP45;
        static constexpr bool kSmoothTurn = false;

    public:
        TEST_METHOD(Completes)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"completion", kCode, trace);
            Assert::IsTrue(trace.completed, message.c_str());
        }

        TEST_METHOD(CommandSamplesCaptured)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"command_samples", kCode, trace);
            Assert::IsTrue(!trace.samples.empty(), message.c_str());
        }

        TEST_METHOD(LeftReturnedCommandIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"left_returned_command", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastReturnedCommand.LeftCommand()) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.leftReturnedCommandFinite, message.c_str());
        }

        TEST_METHOD(RightReturnedCommandIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"right_returned_command", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastReturnedCommand.RightCommand()) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.rightReturnedCommandFinite, message.c_str());
        }

        TEST_METHOD(BodyProposalEvidenceIsSet)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"body_proposal_evidence", kCode, trace) +
                L" actual_flags=" + std::to_wstring(trace.lastTelemetry.commandKindFlags) +
                L" required_mask=" + std::to_wstring(DriveTelemetry::kCommandKindBodyProposal);
            Assert::IsTrue(trace.bodyProposalEvidenceSet, message.c_str());
        }

        TEST_METHOD(CommandTelemetryEvidenceIsSet)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"command_telemetry_evidence", kCode, trace) +
                L" actual_flags=" + std::to_wstring(trace.lastTelemetry.telemetryValidFlags) +
                L" required_mask=" + std::to_wstring(DriveTelemetry::kTelemetryCommandEvidenceValid);
            Assert::IsTrue(trace.commandTelemetryEvidenceSet, message.c_str());
        }

        TEST_METHOD(LeftDriveEvidenceMatchesCommand)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"left_drive_evidence", kCode, trace) +
                L" expected=" + std::to_wstring(trace.lastReturnedCommand.LeftCommand()) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.leftDriveCommand) +
                L" tolerance=1e-6";
            Assert::IsTrue(trace.leftCommandEvidenceMatchesReturnedCommand, message.c_str());
        }

        TEST_METHOD(RightDriveEvidenceMatchesCommand)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"right_drive_evidence", kCode, trace) +
                L" expected=" + std::to_wstring(trace.lastReturnedCommand.RightCommand()) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.rightDriveCommand) +
                L" tolerance=1e-6";
            Assert::IsTrue(trace.rightCommandEvidenceMatchesReturnedCommand, message.c_str());
        }

        TEST_METHOD(RequestedForwardIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_forward_mps", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedForwardMps) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedForwardMpsFinite, message.c_str());
        }

        TEST_METHOD(RequestedYawRateIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_yaw_rate_radps", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedYawRateRadps) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedYawRateRadpsFinite, message.c_str());
        }

        TEST_METHOD(RequestedForwardAccelIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_forward_accel_mps2", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedForwardAccelMps2) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedForwardAccelMps2Finite, message.c_str());
        }

        TEST_METHOD(RequestedYawAccelIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_yaw_accel_radps2", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedYawAccelRadps2) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedYawAccelRadps2Finite, message.c_str());
        }

        TEST_METHOD(RequestedYawIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_yaw_rad", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedYawRad) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedYawRadFinite, message.c_str());
        }

        TEST_METHOD(TruthPositionXIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetPositionX();
            const std::wstring message = BuildManeuverMessage(L"truth_position_x", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthPositionYIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetPositionY();
            const std::wstring message = BuildManeuverMessage(L"truth_position_y", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthHeadingIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetHeading();
            const std::wstring message = BuildManeuverMessage(L"truth_heading", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthForwardVelocityIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetForwardVelocity();
            const std::wstring message = BuildManeuverMessage(L"truth_forward_velocity", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthRightwardVelocityIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetRightwardVelocity();
            const std::wstring message = BuildManeuverMessage(L"truth_rightward_velocity", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthYawRateIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetYawRate();
            const std::wstring message = BuildManeuverMessage(L"truth_yaw_rate", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthLeftWheelSpeedIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetWheelSpeedLeft();
            const std::wstring message = BuildManeuverMessage(L"truth_left_wheel_speed", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthRightWheelSpeedIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetWheelSpeedRight();
            const std::wstring message = BuildManeuverMessage(L"truth_right_wheel_speed", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }


        TEST_METHOD(ShiftWithinTolerance)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float shiftMeters = ComputeInPlaceShiftMeters(trace);
            const std::wstring message = BuildManeuverMessage(L"shift", kCode, trace) +
                L" actual_m=" + std::to_wstring(shiftMeters) +
                L" limit_m=" + std::to_wstring(kInPlacePositionToleranceM);
            Assert::IsTrue(shiftMeters < kInPlacePositionToleranceM, message.c_str());
        }

        TEST_METHOD(HeadingWithinTolerance)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float headingErrorRad = ComputeInPlaceHeadingErrorRad(kCode, trace);
            const std::wstring message = BuildManeuverMessage(L"heading", kCode, trace) +
                L" error_deg=" + std::to_wstring(headingErrorRad * RAD_TO_DEG_F) +
                L" limit_deg=" + std::to_wstring(kHeadingToleranceRad * RAD_TO_DEG_F);
            Assert::IsTrue(headingErrorRad <= kHeadingToleranceRad, message.c_str());
        }

        TEST_METHOD(DurationWithinTolerance)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float expectedTimeSeconds = ComputeInPlaceExpectedTimeSeconds(kCode);
            const float relativeError = ComputeInPlaceRelativeTimeError(kCode, trace);
            const std::wstring message = BuildManeuverMessage(L"time", kCode, trace) +
                L" elapsed_s=" + std::to_wstring(trace.elapsedSeconds) +
                L" expected_s=" + std::to_wstring(expectedTimeSeconds) +
                L" rel_err=" + std::to_wstring(relativeError) +
                L" limit=" + std::to_wstring(kTimeToleranceFraction);
            Assert::IsTrue(relativeError <= kTimeToleranceFraction, message.c_str());
        }

    };

    TEST_CLASS(DriveManeuverIP90ContractTest)
    {
        static constexpr ManeuverCode kCode = IP90;
        static constexpr bool kSmoothTurn = false;

    public:
        TEST_METHOD(Completes)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"completion", kCode, trace);
            Assert::IsTrue(trace.completed, message.c_str());
        }

        TEST_METHOD(CommandSamplesCaptured)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"command_samples", kCode, trace);
            Assert::IsTrue(!trace.samples.empty(), message.c_str());
        }

        TEST_METHOD(LeftReturnedCommandIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"left_returned_command", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastReturnedCommand.LeftCommand()) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.leftReturnedCommandFinite, message.c_str());
        }

        TEST_METHOD(RightReturnedCommandIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"right_returned_command", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastReturnedCommand.RightCommand()) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.rightReturnedCommandFinite, message.c_str());
        }

        TEST_METHOD(BodyProposalEvidenceIsSet)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"body_proposal_evidence", kCode, trace) +
                L" actual_flags=" + std::to_wstring(trace.lastTelemetry.commandKindFlags) +
                L" required_mask=" + std::to_wstring(DriveTelemetry::kCommandKindBodyProposal);
            Assert::IsTrue(trace.bodyProposalEvidenceSet, message.c_str());
        }

        TEST_METHOD(CommandTelemetryEvidenceIsSet)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"command_telemetry_evidence", kCode, trace) +
                L" actual_flags=" + std::to_wstring(trace.lastTelemetry.telemetryValidFlags) +
                L" required_mask=" + std::to_wstring(DriveTelemetry::kTelemetryCommandEvidenceValid);
            Assert::IsTrue(trace.commandTelemetryEvidenceSet, message.c_str());
        }

        TEST_METHOD(LeftDriveEvidenceMatchesCommand)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"left_drive_evidence", kCode, trace) +
                L" expected=" + std::to_wstring(trace.lastReturnedCommand.LeftCommand()) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.leftDriveCommand) +
                L" tolerance=1e-6";
            Assert::IsTrue(trace.leftCommandEvidenceMatchesReturnedCommand, message.c_str());
        }

        TEST_METHOD(RightDriveEvidenceMatchesCommand)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"right_drive_evidence", kCode, trace) +
                L" expected=" + std::to_wstring(trace.lastReturnedCommand.RightCommand()) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.rightDriveCommand) +
                L" tolerance=1e-6";
            Assert::IsTrue(trace.rightCommandEvidenceMatchesReturnedCommand, message.c_str());
        }

        TEST_METHOD(RequestedForwardIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_forward_mps", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedForwardMps) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedForwardMpsFinite, message.c_str());
        }

        TEST_METHOD(RequestedYawRateIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_yaw_rate_radps", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedYawRateRadps) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedYawRateRadpsFinite, message.c_str());
        }

        TEST_METHOD(RequestedForwardAccelIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_forward_accel_mps2", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedForwardAccelMps2) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedForwardAccelMps2Finite, message.c_str());
        }

        TEST_METHOD(RequestedYawAccelIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_yaw_accel_radps2", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedYawAccelRadps2) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedYawAccelRadps2Finite, message.c_str());
        }

        TEST_METHOD(RequestedYawIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_yaw_rad", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedYawRad) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedYawRadFinite, message.c_str());
        }

        TEST_METHOD(TruthPositionXIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetPositionX();
            const std::wstring message = BuildManeuverMessage(L"truth_position_x", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthPositionYIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetPositionY();
            const std::wstring message = BuildManeuverMessage(L"truth_position_y", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthHeadingIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetHeading();
            const std::wstring message = BuildManeuverMessage(L"truth_heading", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthForwardVelocityIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetForwardVelocity();
            const std::wstring message = BuildManeuverMessage(L"truth_forward_velocity", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthRightwardVelocityIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetRightwardVelocity();
            const std::wstring message = BuildManeuverMessage(L"truth_rightward_velocity", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthYawRateIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetYawRate();
            const std::wstring message = BuildManeuverMessage(L"truth_yaw_rate", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthLeftWheelSpeedIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetWheelSpeedLeft();
            const std::wstring message = BuildManeuverMessage(L"truth_left_wheel_speed", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthRightWheelSpeedIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetWheelSpeedRight();
            const std::wstring message = BuildManeuverMessage(L"truth_right_wheel_speed", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }


        TEST_METHOD(ShiftWithinTolerance)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float shiftMeters = ComputeInPlaceShiftMeters(trace);
            const std::wstring message = BuildManeuverMessage(L"shift", kCode, trace) +
                L" actual_m=" + std::to_wstring(shiftMeters) +
                L" limit_m=" + std::to_wstring(kInPlacePositionToleranceM);
            Assert::IsTrue(shiftMeters < kInPlacePositionToleranceM, message.c_str());
        }

        TEST_METHOD(HeadingWithinTolerance)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float headingErrorRad = ComputeInPlaceHeadingErrorRad(kCode, trace);
            const std::wstring message = BuildManeuverMessage(L"heading", kCode, trace) +
                L" error_deg=" + std::to_wstring(headingErrorRad * RAD_TO_DEG_F) +
                L" limit_deg=" + std::to_wstring(kHeadingToleranceRad * RAD_TO_DEG_F);
            Assert::IsTrue(headingErrorRad <= kHeadingToleranceRad, message.c_str());
        }

        TEST_METHOD(DurationWithinTolerance)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float expectedTimeSeconds = ComputeInPlaceExpectedTimeSeconds(kCode);
            const float relativeError = ComputeInPlaceRelativeTimeError(kCode, trace);
            const std::wstring message = BuildManeuverMessage(L"time", kCode, trace) +
                L" elapsed_s=" + std::to_wstring(trace.elapsedSeconds) +
                L" expected_s=" + std::to_wstring(expectedTimeSeconds) +
                L" rel_err=" + std::to_wstring(relativeError) +
                L" limit=" + std::to_wstring(kTimeToleranceFraction);
            Assert::IsTrue(relativeError <= kTimeToleranceFraction, message.c_str());
        }

    };

    TEST_CLASS(DriveManeuverIP135ContractTest)
    {
        static constexpr ManeuverCode kCode = IP135;
        static constexpr bool kSmoothTurn = false;

    public:
        TEST_METHOD(Completes)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"completion", kCode, trace);
            Assert::IsTrue(trace.completed, message.c_str());
        }

        TEST_METHOD(CommandSamplesCaptured)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"command_samples", kCode, trace);
            Assert::IsTrue(!trace.samples.empty(), message.c_str());
        }

        TEST_METHOD(LeftReturnedCommandIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"left_returned_command", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastReturnedCommand.LeftCommand()) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.leftReturnedCommandFinite, message.c_str());
        }

        TEST_METHOD(RightReturnedCommandIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"right_returned_command", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastReturnedCommand.RightCommand()) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.rightReturnedCommandFinite, message.c_str());
        }

        TEST_METHOD(BodyProposalEvidenceIsSet)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"body_proposal_evidence", kCode, trace) +
                L" actual_flags=" + std::to_wstring(trace.lastTelemetry.commandKindFlags) +
                L" required_mask=" + std::to_wstring(DriveTelemetry::kCommandKindBodyProposal);
            Assert::IsTrue(trace.bodyProposalEvidenceSet, message.c_str());
        }

        TEST_METHOD(CommandTelemetryEvidenceIsSet)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"command_telemetry_evidence", kCode, trace) +
                L" actual_flags=" + std::to_wstring(trace.lastTelemetry.telemetryValidFlags) +
                L" required_mask=" + std::to_wstring(DriveTelemetry::kTelemetryCommandEvidenceValid);
            Assert::IsTrue(trace.commandTelemetryEvidenceSet, message.c_str());
        }

        TEST_METHOD(LeftDriveEvidenceMatchesCommand)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"left_drive_evidence", kCode, trace) +
                L" expected=" + std::to_wstring(trace.lastReturnedCommand.LeftCommand()) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.leftDriveCommand) +
                L" tolerance=1e-6";
            Assert::IsTrue(trace.leftCommandEvidenceMatchesReturnedCommand, message.c_str());
        }

        TEST_METHOD(RightDriveEvidenceMatchesCommand)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"right_drive_evidence", kCode, trace) +
                L" expected=" + std::to_wstring(trace.lastReturnedCommand.RightCommand()) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.rightDriveCommand) +
                L" tolerance=1e-6";
            Assert::IsTrue(trace.rightCommandEvidenceMatchesReturnedCommand, message.c_str());
        }

        TEST_METHOD(RequestedForwardIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_forward_mps", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedForwardMps) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedForwardMpsFinite, message.c_str());
        }

        TEST_METHOD(RequestedYawRateIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_yaw_rate_radps", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedYawRateRadps) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedYawRateRadpsFinite, message.c_str());
        }

        TEST_METHOD(RequestedForwardAccelIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_forward_accel_mps2", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedForwardAccelMps2) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedForwardAccelMps2Finite, message.c_str());
        }

        TEST_METHOD(RequestedYawAccelIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_yaw_accel_radps2", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedYawAccelRadps2) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedYawAccelRadps2Finite, message.c_str());
        }

        TEST_METHOD(RequestedYawIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_yaw_rad", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedYawRad) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedYawRadFinite, message.c_str());
        }

        TEST_METHOD(TruthPositionXIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetPositionX();
            const std::wstring message = BuildManeuverMessage(L"truth_position_x", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthPositionYIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetPositionY();
            const std::wstring message = BuildManeuverMessage(L"truth_position_y", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthHeadingIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetHeading();
            const std::wstring message = BuildManeuverMessage(L"truth_heading", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthForwardVelocityIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetForwardVelocity();
            const std::wstring message = BuildManeuverMessage(L"truth_forward_velocity", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthRightwardVelocityIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetRightwardVelocity();
            const std::wstring message = BuildManeuverMessage(L"truth_rightward_velocity", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthYawRateIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetYawRate();
            const std::wstring message = BuildManeuverMessage(L"truth_yaw_rate", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthLeftWheelSpeedIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetWheelSpeedLeft();
            const std::wstring message = BuildManeuverMessage(L"truth_left_wheel_speed", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthRightWheelSpeedIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetWheelSpeedRight();
            const std::wstring message = BuildManeuverMessage(L"truth_right_wheel_speed", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }


        TEST_METHOD(ShiftWithinTolerance)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float shiftMeters = ComputeInPlaceShiftMeters(trace);
            const std::wstring message = BuildManeuverMessage(L"shift", kCode, trace) +
                L" actual_m=" + std::to_wstring(shiftMeters) +
                L" limit_m=" + std::to_wstring(kInPlacePositionToleranceM);
            Assert::IsTrue(shiftMeters < kInPlacePositionToleranceM, message.c_str());
        }

        TEST_METHOD(HeadingWithinTolerance)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float headingErrorRad = ComputeInPlaceHeadingErrorRad(kCode, trace);
            const std::wstring message = BuildManeuverMessage(L"heading", kCode, trace) +
                L" error_deg=" + std::to_wstring(headingErrorRad * RAD_TO_DEG_F) +
                L" limit_deg=" + std::to_wstring(kHeadingToleranceRad * RAD_TO_DEG_F);
            Assert::IsTrue(headingErrorRad <= kHeadingToleranceRad, message.c_str());
        }

        TEST_METHOD(DurationWithinTolerance)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float expectedTimeSeconds = ComputeInPlaceExpectedTimeSeconds(kCode);
            const float relativeError = ComputeInPlaceRelativeTimeError(kCode, trace);
            const std::wstring message = BuildManeuverMessage(L"time", kCode, trace) +
                L" elapsed_s=" + std::to_wstring(trace.elapsedSeconds) +
                L" expected_s=" + std::to_wstring(expectedTimeSeconds) +
                L" rel_err=" + std::to_wstring(relativeError) +
                L" limit=" + std::to_wstring(kTimeToleranceFraction);
            Assert::IsTrue(relativeError <= kTimeToleranceFraction, message.c_str());
        }

    };

    TEST_CLASS(DriveManeuverIP180ContractTest)
    {
        static constexpr ManeuverCode kCode = IP180;
        static constexpr bool kSmoothTurn = false;

    public:
        TEST_METHOD(Completes)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"completion", kCode, trace);
            Assert::IsTrue(trace.completed, message.c_str());
        }

        TEST_METHOD(CommandSamplesCaptured)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"command_samples", kCode, trace);
            Assert::IsTrue(!trace.samples.empty(), message.c_str());
        }

        TEST_METHOD(LeftReturnedCommandIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"left_returned_command", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastReturnedCommand.LeftCommand()) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.leftReturnedCommandFinite, message.c_str());
        }

        TEST_METHOD(RightReturnedCommandIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"right_returned_command", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastReturnedCommand.RightCommand()) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.rightReturnedCommandFinite, message.c_str());
        }

        TEST_METHOD(BodyProposalEvidenceIsSet)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"body_proposal_evidence", kCode, trace) +
                L" actual_flags=" + std::to_wstring(trace.lastTelemetry.commandKindFlags) +
                L" required_mask=" + std::to_wstring(DriveTelemetry::kCommandKindBodyProposal);
            Assert::IsTrue(trace.bodyProposalEvidenceSet, message.c_str());
        }

        TEST_METHOD(CommandTelemetryEvidenceIsSet)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"command_telemetry_evidence", kCode, trace) +
                L" actual_flags=" + std::to_wstring(trace.lastTelemetry.telemetryValidFlags) +
                L" required_mask=" + std::to_wstring(DriveTelemetry::kTelemetryCommandEvidenceValid);
            Assert::IsTrue(trace.commandTelemetryEvidenceSet, message.c_str());
        }

        TEST_METHOD(LeftDriveEvidenceMatchesCommand)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"left_drive_evidence", kCode, trace) +
                L" expected=" + std::to_wstring(trace.lastReturnedCommand.LeftCommand()) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.leftDriveCommand) +
                L" tolerance=1e-6";
            Assert::IsTrue(trace.leftCommandEvidenceMatchesReturnedCommand, message.c_str());
        }

        TEST_METHOD(RightDriveEvidenceMatchesCommand)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"right_drive_evidence", kCode, trace) +
                L" expected=" + std::to_wstring(trace.lastReturnedCommand.RightCommand()) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.rightDriveCommand) +
                L" tolerance=1e-6";
            Assert::IsTrue(trace.rightCommandEvidenceMatchesReturnedCommand, message.c_str());
        }

        TEST_METHOD(RequestedForwardIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_forward_mps", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedForwardMps) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedForwardMpsFinite, message.c_str());
        }

        TEST_METHOD(RequestedYawRateIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_yaw_rate_radps", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedYawRateRadps) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedYawRateRadpsFinite, message.c_str());
        }

        TEST_METHOD(RequestedForwardAccelIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_forward_accel_mps2", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedForwardAccelMps2) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedForwardAccelMps2Finite, message.c_str());
        }

        TEST_METHOD(RequestedYawAccelIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_yaw_accel_radps2", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedYawAccelRadps2) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedYawAccelRadps2Finite, message.c_str());
        }

        TEST_METHOD(RequestedYawIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_yaw_rad", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedYawRad) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedYawRadFinite, message.c_str());
        }

        TEST_METHOD(TruthPositionXIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetPositionX();
            const std::wstring message = BuildManeuverMessage(L"truth_position_x", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthPositionYIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetPositionY();
            const std::wstring message = BuildManeuverMessage(L"truth_position_y", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthHeadingIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetHeading();
            const std::wstring message = BuildManeuverMessage(L"truth_heading", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthForwardVelocityIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetForwardVelocity();
            const std::wstring message = BuildManeuverMessage(L"truth_forward_velocity", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthRightwardVelocityIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetRightwardVelocity();
            const std::wstring message = BuildManeuverMessage(L"truth_rightward_velocity", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthYawRateIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetYawRate();
            const std::wstring message = BuildManeuverMessage(L"truth_yaw_rate", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthLeftWheelSpeedIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetWheelSpeedLeft();
            const std::wstring message = BuildManeuverMessage(L"truth_left_wheel_speed", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthRightWheelSpeedIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetWheelSpeedRight();
            const std::wstring message = BuildManeuverMessage(L"truth_right_wheel_speed", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }


        TEST_METHOD(ShiftWithinTolerance)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float shiftMeters = ComputeInPlaceShiftMeters(trace);
            const std::wstring message = BuildManeuverMessage(L"shift", kCode, trace) +
                L" actual_m=" + std::to_wstring(shiftMeters) +
                L" limit_m=" + std::to_wstring(kInPlacePositionToleranceM);
            Assert::IsTrue(shiftMeters < kInPlacePositionToleranceM, message.c_str());
        }

        TEST_METHOD(HeadingWithinTolerance)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float headingErrorRad = ComputeInPlaceHeadingErrorRad(kCode, trace);
            const std::wstring message = BuildManeuverMessage(L"heading", kCode, trace) +
                L" error_deg=" + std::to_wstring(headingErrorRad * RAD_TO_DEG_F) +
                L" limit_deg=" + std::to_wstring(kHeadingToleranceRad * RAD_TO_DEG_F);
            Assert::IsTrue(headingErrorRad <= kHeadingToleranceRad, message.c_str());
        }

        TEST_METHOD(DurationWithinTolerance)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float expectedTimeSeconds = ComputeInPlaceExpectedTimeSeconds(kCode);
            const float relativeError = ComputeInPlaceRelativeTimeError(kCode, trace);
            const std::wstring message = BuildManeuverMessage(L"time", kCode, trace) +
                L" elapsed_s=" + std::to_wstring(trace.elapsedSeconds) +
                L" expected_s=" + std::to_wstring(expectedTimeSeconds) +
                L" rel_err=" + std::to_wstring(relativeError) +
                L" limit=" + std::to_wstring(kTimeToleranceFraction);
            Assert::IsTrue(relativeError <= kTimeToleranceFraction, message.c_str());
        }

    };

    TEST_CLASS(DriveManeuverS45LSContractTest)
    {
        static constexpr ManeuverCode kCode = S45LS;
        static constexpr bool kSmoothTurn = true;

    public:
        TEST_METHOD(Completes)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"completion", kCode, trace);
            Assert::IsTrue(trace.completed, message.c_str());
        }

        TEST_METHOD(CommandSamplesCaptured)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"command_samples", kCode, trace);
            Assert::IsTrue(!trace.samples.empty(), message.c_str());
        }

        TEST_METHOD(LeftReturnedCommandIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"left_returned_command", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastReturnedCommand.LeftCommand()) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.leftReturnedCommandFinite, message.c_str());
        }

        TEST_METHOD(RightReturnedCommandIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"right_returned_command", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastReturnedCommand.RightCommand()) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.rightReturnedCommandFinite, message.c_str());
        }

        TEST_METHOD(BodyProposalEvidenceIsSet)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"body_proposal_evidence", kCode, trace) +
                L" actual_flags=" + std::to_wstring(trace.lastTelemetry.commandKindFlags) +
                L" required_mask=" + std::to_wstring(DriveTelemetry::kCommandKindBodyProposal);
            Assert::IsTrue(trace.bodyProposalEvidenceSet, message.c_str());
        }

        TEST_METHOD(CommandTelemetryEvidenceIsSet)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"command_telemetry_evidence", kCode, trace) +
                L" actual_flags=" + std::to_wstring(trace.lastTelemetry.telemetryValidFlags) +
                L" required_mask=" + std::to_wstring(DriveTelemetry::kTelemetryCommandEvidenceValid);
            Assert::IsTrue(trace.commandTelemetryEvidenceSet, message.c_str());
        }

        TEST_METHOD(LeftDriveEvidenceMatchesCommand)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"left_drive_evidence", kCode, trace) +
                L" expected=" + std::to_wstring(trace.lastReturnedCommand.LeftCommand()) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.leftDriveCommand) +
                L" tolerance=1e-6";
            Assert::IsTrue(trace.leftCommandEvidenceMatchesReturnedCommand, message.c_str());
        }

        TEST_METHOD(RightDriveEvidenceMatchesCommand)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"right_drive_evidence", kCode, trace) +
                L" expected=" + std::to_wstring(trace.lastReturnedCommand.RightCommand()) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.rightDriveCommand) +
                L" tolerance=1e-6";
            Assert::IsTrue(trace.rightCommandEvidenceMatchesReturnedCommand, message.c_str());
        }

        TEST_METHOD(RequestedForwardIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_forward_mps", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedForwardMps) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedForwardMpsFinite, message.c_str());
        }

        TEST_METHOD(RequestedYawRateIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_yaw_rate_radps", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedYawRateRadps) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedYawRateRadpsFinite, message.c_str());
        }

        TEST_METHOD(RequestedForwardAccelIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_forward_accel_mps2", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedForwardAccelMps2) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedForwardAccelMps2Finite, message.c_str());
        }

        TEST_METHOD(RequestedYawAccelIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_yaw_accel_radps2", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedYawAccelRadps2) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedYawAccelRadps2Finite, message.c_str());
        }

        TEST_METHOD(RequestedYawIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_yaw_rad", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedYawRad) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedYawRadFinite, message.c_str());
        }

        TEST_METHOD(TruthPositionXIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetPositionX();
            const std::wstring message = BuildManeuverMessage(L"truth_position_x", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthPositionYIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetPositionY();
            const std::wstring message = BuildManeuverMessage(L"truth_position_y", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthHeadingIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetHeading();
            const std::wstring message = BuildManeuverMessage(L"truth_heading", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthForwardVelocityIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetForwardVelocity();
            const std::wstring message = BuildManeuverMessage(L"truth_forward_velocity", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthRightwardVelocityIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetRightwardVelocity();
            const std::wstring message = BuildManeuverMessage(L"truth_rightward_velocity", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthYawRateIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetYawRate();
            const std::wstring message = BuildManeuverMessage(L"truth_yaw_rate", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthLeftWheelSpeedIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetWheelSpeedLeft();
            const std::wstring message = BuildManeuverMessage(L"truth_left_wheel_speed", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthRightWheelSpeedIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetWheelSpeedRight();
            const std::wstring message = BuildManeuverMessage(L"truth_right_wheel_speed", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }


        TEST_METHOD(VelocityStable)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float normalizedSpan = ComputeSmoothVelocityNormalizedSpan(trace);
            const std::wstring message = BuildManeuverMessage(L"velocity", kCode, trace) +
                L" span=" + std::to_wstring(normalizedSpan) +
                L" limit=" + std::to_wstring(kVelocityVariationLimit);
            Assert::IsTrue(normalizedSpan < kVelocityVariationLimit, message.c_str());
        }

        TEST_METHOD(YawAccelerationStable)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float normalizedSpan = ComputeSmoothYawAccelerationNormalizedSpan(trace);
            const std::wstring message = BuildManeuverMessage(L"yaw_accel", kCode, trace) +
                L" span=" + std::to_wstring(normalizedSpan) +
                L" limit=" + std::to_wstring(kYawAccelerationVariationLimit);
            Assert::IsTrue(normalizedSpan < kYawAccelerationVariationLimit, message.c_str());
        }

        TEST_METHOD(YawRateStable)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float normalizedSpan = ComputeSmoothYawRateNormalizedSpan(trace);
            const std::wstring message = BuildManeuverMessage(L"yaw_rate", kCode, trace) +
                L" span=" + std::to_wstring(normalizedSpan) +
                L" limit=" + std::to_wstring(kYawRateVariationLimit);
            Assert::IsTrue(normalizedSpan < kYawRateVariationLimit, message.c_str());
        }

        TEST_METHOD(FinalPositionWithinTolerance)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float positionErrorMeters = ComputeSmoothFinalPositionErrorMeters(kCode, trace);
            const std::wstring message = BuildManeuverMessage(L"position", kCode, trace) +
                L" error_m=" + std::to_wstring(positionErrorMeters) +
                L" limit_m=" + std::to_wstring(kSmoothPositionToleranceM);
            Assert::IsTrue(positionErrorMeters <= kSmoothPositionToleranceM, message.c_str());
        }

        TEST_METHOD(FinalHeadingWithinTolerance)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float headingErrorRad = ComputeSmoothFinalHeadingErrorRad(kCode, trace);
            const std::wstring message = BuildManeuverMessage(L"heading", kCode, trace) +
                L" error_deg=" + std::to_wstring(headingErrorRad * RAD_TO_DEG_F) +
                L" limit_deg=" + std::to_wstring(kHeadingToleranceRad * RAD_TO_DEG_F);
            Assert::IsTrue(headingErrorRad <= kHeadingToleranceRad, message.c_str());
        }

    };

    TEST_CLASS(DriveManeuverS45LDContractTest)
    {
        static constexpr ManeuverCode kCode = S45LD;
        static constexpr bool kSmoothTurn = true;

    public:
        TEST_METHOD(Completes)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"completion", kCode, trace);
            Assert::IsTrue(trace.completed, message.c_str());
        }

        TEST_METHOD(CommandSamplesCaptured)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"command_samples", kCode, trace);
            Assert::IsTrue(!trace.samples.empty(), message.c_str());
        }

        TEST_METHOD(LeftReturnedCommandIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"left_returned_command", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastReturnedCommand.LeftCommand()) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.leftReturnedCommandFinite, message.c_str());
        }

        TEST_METHOD(RightReturnedCommandIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"right_returned_command", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastReturnedCommand.RightCommand()) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.rightReturnedCommandFinite, message.c_str());
        }

        TEST_METHOD(BodyProposalEvidenceIsSet)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"body_proposal_evidence", kCode, trace) +
                L" actual_flags=" + std::to_wstring(trace.lastTelemetry.commandKindFlags) +
                L" required_mask=" + std::to_wstring(DriveTelemetry::kCommandKindBodyProposal);
            Assert::IsTrue(trace.bodyProposalEvidenceSet, message.c_str());
        }

        TEST_METHOD(CommandTelemetryEvidenceIsSet)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"command_telemetry_evidence", kCode, trace) +
                L" actual_flags=" + std::to_wstring(trace.lastTelemetry.telemetryValidFlags) +
                L" required_mask=" + std::to_wstring(DriveTelemetry::kTelemetryCommandEvidenceValid);
            Assert::IsTrue(trace.commandTelemetryEvidenceSet, message.c_str());
        }

        TEST_METHOD(LeftDriveEvidenceMatchesCommand)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"left_drive_evidence", kCode, trace) +
                L" expected=" + std::to_wstring(trace.lastReturnedCommand.LeftCommand()) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.leftDriveCommand) +
                L" tolerance=1e-6";
            Assert::IsTrue(trace.leftCommandEvidenceMatchesReturnedCommand, message.c_str());
        }

        TEST_METHOD(RightDriveEvidenceMatchesCommand)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"right_drive_evidence", kCode, trace) +
                L" expected=" + std::to_wstring(trace.lastReturnedCommand.RightCommand()) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.rightDriveCommand) +
                L" tolerance=1e-6";
            Assert::IsTrue(trace.rightCommandEvidenceMatchesReturnedCommand, message.c_str());
        }

        TEST_METHOD(RequestedForwardIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_forward_mps", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedForwardMps) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedForwardMpsFinite, message.c_str());
        }

        TEST_METHOD(RequestedYawRateIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_yaw_rate_radps", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedYawRateRadps) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedYawRateRadpsFinite, message.c_str());
        }

        TEST_METHOD(RequestedForwardAccelIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_forward_accel_mps2", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedForwardAccelMps2) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedForwardAccelMps2Finite, message.c_str());
        }

        TEST_METHOD(RequestedYawAccelIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_yaw_accel_radps2", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedYawAccelRadps2) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedYawAccelRadps2Finite, message.c_str());
        }

        TEST_METHOD(RequestedYawIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_yaw_rad", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedYawRad) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedYawRadFinite, message.c_str());
        }

        TEST_METHOD(TruthPositionXIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetPositionX();
            const std::wstring message = BuildManeuverMessage(L"truth_position_x", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthPositionYIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetPositionY();
            const std::wstring message = BuildManeuverMessage(L"truth_position_y", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthHeadingIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetHeading();
            const std::wstring message = BuildManeuverMessage(L"truth_heading", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthForwardVelocityIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetForwardVelocity();
            const std::wstring message = BuildManeuverMessage(L"truth_forward_velocity", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthRightwardVelocityIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetRightwardVelocity();
            const std::wstring message = BuildManeuverMessage(L"truth_rightward_velocity", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthYawRateIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetYawRate();
            const std::wstring message = BuildManeuverMessage(L"truth_yaw_rate", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthLeftWheelSpeedIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetWheelSpeedLeft();
            const std::wstring message = BuildManeuverMessage(L"truth_left_wheel_speed", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthRightWheelSpeedIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetWheelSpeedRight();
            const std::wstring message = BuildManeuverMessage(L"truth_right_wheel_speed", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }


        TEST_METHOD(VelocityStable)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float normalizedSpan = ComputeSmoothVelocityNormalizedSpan(trace);
            const std::wstring message = BuildManeuverMessage(L"velocity", kCode, trace) +
                L" span=" + std::to_wstring(normalizedSpan) +
                L" limit=" + std::to_wstring(kVelocityVariationLimit);
            Assert::IsTrue(normalizedSpan < kVelocityVariationLimit, message.c_str());
        }

        TEST_METHOD(YawAccelerationStable)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float normalizedSpan = ComputeSmoothYawAccelerationNormalizedSpan(trace);
            const std::wstring message = BuildManeuverMessage(L"yaw_accel", kCode, trace) +
                L" span=" + std::to_wstring(normalizedSpan) +
                L" limit=" + std::to_wstring(kYawAccelerationVariationLimit);
            Assert::IsTrue(normalizedSpan < kYawAccelerationVariationLimit, message.c_str());
        }

        TEST_METHOD(YawRateStable)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float normalizedSpan = ComputeSmoothYawRateNormalizedSpan(trace);
            const std::wstring message = BuildManeuverMessage(L"yaw_rate", kCode, trace) +
                L" span=" + std::to_wstring(normalizedSpan) +
                L" limit=" + std::to_wstring(kYawRateVariationLimit);
            Assert::IsTrue(normalizedSpan < kYawRateVariationLimit, message.c_str());
        }

        TEST_METHOD(FinalPositionWithinTolerance)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float positionErrorMeters = ComputeSmoothFinalPositionErrorMeters(kCode, trace);
            const std::wstring message = BuildManeuverMessage(L"position", kCode, trace) +
                L" error_m=" + std::to_wstring(positionErrorMeters) +
                L" limit_m=" + std::to_wstring(kSmoothPositionToleranceM);
            Assert::IsTrue(positionErrorMeters <= kSmoothPositionToleranceM, message.c_str());
        }

        TEST_METHOD(FinalHeadingWithinTolerance)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float headingErrorRad = ComputeSmoothFinalHeadingErrorRad(kCode, trace);
            const std::wstring message = BuildManeuverMessage(L"heading", kCode, trace) +
                L" error_deg=" + std::to_wstring(headingErrorRad * RAD_TO_DEG_F) +
                L" limit_deg=" + std::to_wstring(kHeadingToleranceRad * RAD_TO_DEG_F);
            Assert::IsTrue(headingErrorRad <= kHeadingToleranceRad, message.c_str());
        }

    };

    TEST_CLASS(DriveManeuverS45SSContractTest)
    {
        static constexpr ManeuverCode kCode = S45SS;
        static constexpr bool kSmoothTurn = true;

    public:
        TEST_METHOD(Completes)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"completion", kCode, trace);
            Assert::IsTrue(trace.completed, message.c_str());
        }

        TEST_METHOD(CommandSamplesCaptured)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"command_samples", kCode, trace);
            Assert::IsTrue(!trace.samples.empty(), message.c_str());
        }

        TEST_METHOD(LeftReturnedCommandIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"left_returned_command", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastReturnedCommand.LeftCommand()) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.leftReturnedCommandFinite, message.c_str());
        }

        TEST_METHOD(RightReturnedCommandIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"right_returned_command", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastReturnedCommand.RightCommand()) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.rightReturnedCommandFinite, message.c_str());
        }

        TEST_METHOD(BodyProposalEvidenceIsSet)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"body_proposal_evidence", kCode, trace) +
                L" actual_flags=" + std::to_wstring(trace.lastTelemetry.commandKindFlags) +
                L" required_mask=" + std::to_wstring(DriveTelemetry::kCommandKindBodyProposal);
            Assert::IsTrue(trace.bodyProposalEvidenceSet, message.c_str());
        }

        TEST_METHOD(CommandTelemetryEvidenceIsSet)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"command_telemetry_evidence", kCode, trace) +
                L" actual_flags=" + std::to_wstring(trace.lastTelemetry.telemetryValidFlags) +
                L" required_mask=" + std::to_wstring(DriveTelemetry::kTelemetryCommandEvidenceValid);
            Assert::IsTrue(trace.commandTelemetryEvidenceSet, message.c_str());
        }

        TEST_METHOD(LeftDriveEvidenceMatchesCommand)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"left_drive_evidence", kCode, trace) +
                L" expected=" + std::to_wstring(trace.lastReturnedCommand.LeftCommand()) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.leftDriveCommand) +
                L" tolerance=1e-6";
            Assert::IsTrue(trace.leftCommandEvidenceMatchesReturnedCommand, message.c_str());
        }

        TEST_METHOD(RightDriveEvidenceMatchesCommand)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"right_drive_evidence", kCode, trace) +
                L" expected=" + std::to_wstring(trace.lastReturnedCommand.RightCommand()) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.rightDriveCommand) +
                L" tolerance=1e-6";
            Assert::IsTrue(trace.rightCommandEvidenceMatchesReturnedCommand, message.c_str());
        }

        TEST_METHOD(RequestedForwardIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_forward_mps", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedForwardMps) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedForwardMpsFinite, message.c_str());
        }

        TEST_METHOD(RequestedYawRateIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_yaw_rate_radps", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedYawRateRadps) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedYawRateRadpsFinite, message.c_str());
        }

        TEST_METHOD(RequestedForwardAccelIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_forward_accel_mps2", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedForwardAccelMps2) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedForwardAccelMps2Finite, message.c_str());
        }

        TEST_METHOD(RequestedYawAccelIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_yaw_accel_radps2", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedYawAccelRadps2) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedYawAccelRadps2Finite, message.c_str());
        }

        TEST_METHOD(RequestedYawIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_yaw_rad", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedYawRad) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedYawRadFinite, message.c_str());
        }

        TEST_METHOD(TruthPositionXIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetPositionX();
            const std::wstring message = BuildManeuverMessage(L"truth_position_x", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthPositionYIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetPositionY();
            const std::wstring message = BuildManeuverMessage(L"truth_position_y", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthHeadingIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetHeading();
            const std::wstring message = BuildManeuverMessage(L"truth_heading", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthForwardVelocityIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetForwardVelocity();
            const std::wstring message = BuildManeuverMessage(L"truth_forward_velocity", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthRightwardVelocityIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetRightwardVelocity();
            const std::wstring message = BuildManeuverMessage(L"truth_rightward_velocity", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthYawRateIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetYawRate();
            const std::wstring message = BuildManeuverMessage(L"truth_yaw_rate", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthLeftWheelSpeedIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetWheelSpeedLeft();
            const std::wstring message = BuildManeuverMessage(L"truth_left_wheel_speed", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthRightWheelSpeedIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetWheelSpeedRight();
            const std::wstring message = BuildManeuverMessage(L"truth_right_wheel_speed", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }


        TEST_METHOD(VelocityStable)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float normalizedSpan = ComputeSmoothVelocityNormalizedSpan(trace);
            const std::wstring message = BuildManeuverMessage(L"velocity", kCode, trace) +
                L" span=" + std::to_wstring(normalizedSpan) +
                L" limit=" + std::to_wstring(kVelocityVariationLimit);
            Assert::IsTrue(normalizedSpan < kVelocityVariationLimit, message.c_str());
        }

        TEST_METHOD(YawAccelerationStable)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float normalizedSpan = ComputeSmoothYawAccelerationNormalizedSpan(trace);
            const std::wstring message = BuildManeuverMessage(L"yaw_accel", kCode, trace) +
                L" span=" + std::to_wstring(normalizedSpan) +
                L" limit=" + std::to_wstring(kYawAccelerationVariationLimit);
            Assert::IsTrue(normalizedSpan < kYawAccelerationVariationLimit, message.c_str());
        }

        TEST_METHOD(YawRateStable)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float normalizedSpan = ComputeSmoothYawRateNormalizedSpan(trace);
            const std::wstring message = BuildManeuverMessage(L"yaw_rate", kCode, trace) +
                L" span=" + std::to_wstring(normalizedSpan) +
                L" limit=" + std::to_wstring(kYawRateVariationLimit);
            Assert::IsTrue(normalizedSpan < kYawRateVariationLimit, message.c_str());
        }

        TEST_METHOD(FinalPositionWithinTolerance)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float positionErrorMeters = ComputeSmoothFinalPositionErrorMeters(kCode, trace);
            const std::wstring message = BuildManeuverMessage(L"position", kCode, trace) +
                L" error_m=" + std::to_wstring(positionErrorMeters) +
                L" limit_m=" + std::to_wstring(kSmoothPositionToleranceM);
            Assert::IsTrue(positionErrorMeters <= kSmoothPositionToleranceM, message.c_str());
        }

        TEST_METHOD(FinalHeadingWithinTolerance)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float headingErrorRad = ComputeSmoothFinalHeadingErrorRad(kCode, trace);
            const std::wstring message = BuildManeuverMessage(L"heading", kCode, trace) +
                L" error_deg=" + std::to_wstring(headingErrorRad * RAD_TO_DEG_F) +
                L" limit_deg=" + std::to_wstring(kHeadingToleranceRad * RAD_TO_DEG_F);
            Assert::IsTrue(headingErrorRad <= kHeadingToleranceRad, message.c_str());
        }

    };

    TEST_CLASS(DriveManeuverS45SDContractTest)
    {
        static constexpr ManeuverCode kCode = S45SD;
        static constexpr bool kSmoothTurn = true;

    public:
        TEST_METHOD(Completes)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"completion", kCode, trace);
            Assert::IsTrue(trace.completed, message.c_str());
        }

        TEST_METHOD(CommandSamplesCaptured)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"command_samples", kCode, trace);
            Assert::IsTrue(!trace.samples.empty(), message.c_str());
        }

        TEST_METHOD(LeftReturnedCommandIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"left_returned_command", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastReturnedCommand.LeftCommand()) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.leftReturnedCommandFinite, message.c_str());
        }

        TEST_METHOD(RightReturnedCommandIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"right_returned_command", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastReturnedCommand.RightCommand()) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.rightReturnedCommandFinite, message.c_str());
        }

        TEST_METHOD(BodyProposalEvidenceIsSet)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"body_proposal_evidence", kCode, trace) +
                L" actual_flags=" + std::to_wstring(trace.lastTelemetry.commandKindFlags) +
                L" required_mask=" + std::to_wstring(DriveTelemetry::kCommandKindBodyProposal);
            Assert::IsTrue(trace.bodyProposalEvidenceSet, message.c_str());
        }

        TEST_METHOD(CommandTelemetryEvidenceIsSet)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"command_telemetry_evidence", kCode, trace) +
                L" actual_flags=" + std::to_wstring(trace.lastTelemetry.telemetryValidFlags) +
                L" required_mask=" + std::to_wstring(DriveTelemetry::kTelemetryCommandEvidenceValid);
            Assert::IsTrue(trace.commandTelemetryEvidenceSet, message.c_str());
        }

        TEST_METHOD(LeftDriveEvidenceMatchesCommand)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"left_drive_evidence", kCode, trace) +
                L" expected=" + std::to_wstring(trace.lastReturnedCommand.LeftCommand()) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.leftDriveCommand) +
                L" tolerance=1e-6";
            Assert::IsTrue(trace.leftCommandEvidenceMatchesReturnedCommand, message.c_str());
        }

        TEST_METHOD(RightDriveEvidenceMatchesCommand)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"right_drive_evidence", kCode, trace) +
                L" expected=" + std::to_wstring(trace.lastReturnedCommand.RightCommand()) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.rightDriveCommand) +
                L" tolerance=1e-6";
            Assert::IsTrue(trace.rightCommandEvidenceMatchesReturnedCommand, message.c_str());
        }

        TEST_METHOD(RequestedForwardIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_forward_mps", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedForwardMps) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedForwardMpsFinite, message.c_str());
        }

        TEST_METHOD(RequestedYawRateIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_yaw_rate_radps", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedYawRateRadps) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedYawRateRadpsFinite, message.c_str());
        }

        TEST_METHOD(RequestedForwardAccelIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_forward_accel_mps2", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedForwardAccelMps2) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedForwardAccelMps2Finite, message.c_str());
        }

        TEST_METHOD(RequestedYawAccelIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_yaw_accel_radps2", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedYawAccelRadps2) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedYawAccelRadps2Finite, message.c_str());
        }

        TEST_METHOD(RequestedYawIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_yaw_rad", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedYawRad) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedYawRadFinite, message.c_str());
        }

        TEST_METHOD(TruthPositionXIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetPositionX();
            const std::wstring message = BuildManeuverMessage(L"truth_position_x", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthPositionYIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetPositionY();
            const std::wstring message = BuildManeuverMessage(L"truth_position_y", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthHeadingIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetHeading();
            const std::wstring message = BuildManeuverMessage(L"truth_heading", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthForwardVelocityIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetForwardVelocity();
            const std::wstring message = BuildManeuverMessage(L"truth_forward_velocity", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthRightwardVelocityIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetRightwardVelocity();
            const std::wstring message = BuildManeuverMessage(L"truth_rightward_velocity", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthYawRateIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetYawRate();
            const std::wstring message = BuildManeuverMessage(L"truth_yaw_rate", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthLeftWheelSpeedIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetWheelSpeedLeft();
            const std::wstring message = BuildManeuverMessage(L"truth_left_wheel_speed", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthRightWheelSpeedIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetWheelSpeedRight();
            const std::wstring message = BuildManeuverMessage(L"truth_right_wheel_speed", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }


        TEST_METHOD(VelocityStable)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float normalizedSpan = ComputeSmoothVelocityNormalizedSpan(trace);
            const std::wstring message = BuildManeuverMessage(L"velocity", kCode, trace) +
                L" span=" + std::to_wstring(normalizedSpan) +
                L" limit=" + std::to_wstring(kVelocityVariationLimit);
            Assert::IsTrue(normalizedSpan < kVelocityVariationLimit, message.c_str());
        }

        TEST_METHOD(YawAccelerationStable)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float normalizedSpan = ComputeSmoothYawAccelerationNormalizedSpan(trace);
            const std::wstring message = BuildManeuverMessage(L"yaw_accel", kCode, trace) +
                L" span=" + std::to_wstring(normalizedSpan) +
                L" limit=" + std::to_wstring(kYawAccelerationVariationLimit);
            Assert::IsTrue(normalizedSpan < kYawAccelerationVariationLimit, message.c_str());
        }

        TEST_METHOD(YawRateStable)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float normalizedSpan = ComputeSmoothYawRateNormalizedSpan(trace);
            const std::wstring message = BuildManeuverMessage(L"yaw_rate", kCode, trace) +
                L" span=" + std::to_wstring(normalizedSpan) +
                L" limit=" + std::to_wstring(kYawRateVariationLimit);
            Assert::IsTrue(normalizedSpan < kYawRateVariationLimit, message.c_str());
        }

        TEST_METHOD(FinalPositionWithinTolerance)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float positionErrorMeters = ComputeSmoothFinalPositionErrorMeters(kCode, trace);
            const std::wstring message = BuildManeuverMessage(L"position", kCode, trace) +
                L" error_m=" + std::to_wstring(positionErrorMeters) +
                L" limit_m=" + std::to_wstring(kSmoothPositionToleranceM);
            Assert::IsTrue(positionErrorMeters <= kSmoothPositionToleranceM, message.c_str());
        }

        TEST_METHOD(FinalHeadingWithinTolerance)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float headingErrorRad = ComputeSmoothFinalHeadingErrorRad(kCode, trace);
            const std::wstring message = BuildManeuverMessage(L"heading", kCode, trace) +
                L" error_deg=" + std::to_wstring(headingErrorRad * RAD_TO_DEG_F) +
                L" limit_deg=" + std::to_wstring(kHeadingToleranceRad * RAD_TO_DEG_F);
            Assert::IsTrue(headingErrorRad <= kHeadingToleranceRad, message.c_str());
        }

    };

    TEST_CLASS(DriveManeuverS90LSContractTest)
    {
        static constexpr ManeuverCode kCode = S90LS;
        static constexpr bool kSmoothTurn = true;

    public:
        TEST_METHOD(Completes)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"completion", kCode, trace);
            Assert::IsTrue(trace.completed, message.c_str());
        }

        TEST_METHOD(CommandSamplesCaptured)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"command_samples", kCode, trace);
            Assert::IsTrue(!trace.samples.empty(), message.c_str());
        }

        TEST_METHOD(LeftReturnedCommandIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"left_returned_command", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastReturnedCommand.LeftCommand()) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.leftReturnedCommandFinite, message.c_str());
        }

        TEST_METHOD(RightReturnedCommandIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"right_returned_command", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastReturnedCommand.RightCommand()) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.rightReturnedCommandFinite, message.c_str());
        }

        TEST_METHOD(BodyProposalEvidenceIsSet)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"body_proposal_evidence", kCode, trace) +
                L" actual_flags=" + std::to_wstring(trace.lastTelemetry.commandKindFlags) +
                L" required_mask=" + std::to_wstring(DriveTelemetry::kCommandKindBodyProposal);
            Assert::IsTrue(trace.bodyProposalEvidenceSet, message.c_str());
        }

        TEST_METHOD(CommandTelemetryEvidenceIsSet)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"command_telemetry_evidence", kCode, trace) +
                L" actual_flags=" + std::to_wstring(trace.lastTelemetry.telemetryValidFlags) +
                L" required_mask=" + std::to_wstring(DriveTelemetry::kTelemetryCommandEvidenceValid);
            Assert::IsTrue(trace.commandTelemetryEvidenceSet, message.c_str());
        }

        TEST_METHOD(LeftDriveEvidenceMatchesCommand)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"left_drive_evidence", kCode, trace) +
                L" expected=" + std::to_wstring(trace.lastReturnedCommand.LeftCommand()) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.leftDriveCommand) +
                L" tolerance=1e-6";
            Assert::IsTrue(trace.leftCommandEvidenceMatchesReturnedCommand, message.c_str());
        }

        TEST_METHOD(RightDriveEvidenceMatchesCommand)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"right_drive_evidence", kCode, trace) +
                L" expected=" + std::to_wstring(trace.lastReturnedCommand.RightCommand()) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.rightDriveCommand) +
                L" tolerance=1e-6";
            Assert::IsTrue(trace.rightCommandEvidenceMatchesReturnedCommand, message.c_str());
        }

        TEST_METHOD(RequestedForwardIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_forward_mps", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedForwardMps) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedForwardMpsFinite, message.c_str());
        }

        TEST_METHOD(RequestedYawRateIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_yaw_rate_radps", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedYawRateRadps) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedYawRateRadpsFinite, message.c_str());
        }

        TEST_METHOD(RequestedForwardAccelIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_forward_accel_mps2", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedForwardAccelMps2) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedForwardAccelMps2Finite, message.c_str());
        }

        TEST_METHOD(RequestedYawAccelIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_yaw_accel_radps2", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedYawAccelRadps2) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedYawAccelRadps2Finite, message.c_str());
        }

        TEST_METHOD(RequestedYawIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_yaw_rad", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedYawRad) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedYawRadFinite, message.c_str());
        }

        TEST_METHOD(TruthPositionXIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetPositionX();
            const std::wstring message = BuildManeuverMessage(L"truth_position_x", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthPositionYIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetPositionY();
            const std::wstring message = BuildManeuverMessage(L"truth_position_y", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthHeadingIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetHeading();
            const std::wstring message = BuildManeuverMessage(L"truth_heading", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthForwardVelocityIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetForwardVelocity();
            const std::wstring message = BuildManeuverMessage(L"truth_forward_velocity", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthRightwardVelocityIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetRightwardVelocity();
            const std::wstring message = BuildManeuverMessage(L"truth_rightward_velocity", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthYawRateIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetYawRate();
            const std::wstring message = BuildManeuverMessage(L"truth_yaw_rate", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthLeftWheelSpeedIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetWheelSpeedLeft();
            const std::wstring message = BuildManeuverMessage(L"truth_left_wheel_speed", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthRightWheelSpeedIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetWheelSpeedRight();
            const std::wstring message = BuildManeuverMessage(L"truth_right_wheel_speed", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }


        TEST_METHOD(VelocityStable)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float normalizedSpan = ComputeSmoothVelocityNormalizedSpan(trace);
            const std::wstring message = BuildManeuverMessage(L"velocity", kCode, trace) +
                L" span=" + std::to_wstring(normalizedSpan) +
                L" limit=" + std::to_wstring(kVelocityVariationLimit);
            Assert::IsTrue(normalizedSpan < kVelocityVariationLimit, message.c_str());
        }

        TEST_METHOD(YawAccelerationStable)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float normalizedSpan = ComputeSmoothYawAccelerationNormalizedSpan(trace);
            const std::wstring message = BuildManeuverMessage(L"yaw_accel", kCode, trace) +
                L" span=" + std::to_wstring(normalizedSpan) +
                L" limit=" + std::to_wstring(kYawAccelerationVariationLimit);
            Assert::IsTrue(normalizedSpan < kYawAccelerationVariationLimit, message.c_str());
        }

        TEST_METHOD(YawRateStable)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float normalizedSpan = ComputeSmoothYawRateNormalizedSpan(trace);
            const std::wstring message = BuildManeuverMessage(L"yaw_rate", kCode, trace) +
                L" span=" + std::to_wstring(normalizedSpan) +
                L" limit=" + std::to_wstring(kYawRateVariationLimit);
            Assert::IsTrue(normalizedSpan < kYawRateVariationLimit, message.c_str());
        }

        TEST_METHOD(FinalPositionWithinTolerance)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float positionErrorMeters = ComputeSmoothFinalPositionErrorMeters(kCode, trace);
            const std::wstring message = BuildManeuverMessage(L"position", kCode, trace) +
                L" error_m=" + std::to_wstring(positionErrorMeters) +
                L" limit_m=" + std::to_wstring(kSmoothPositionToleranceM);
            Assert::IsTrue(positionErrorMeters <= kSmoothPositionToleranceM, message.c_str());
        }

        TEST_METHOD(FinalHeadingWithinTolerance)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float headingErrorRad = ComputeSmoothFinalHeadingErrorRad(kCode, trace);
            const std::wstring message = BuildManeuverMessage(L"heading", kCode, trace) +
                L" error_deg=" + std::to_wstring(headingErrorRad * RAD_TO_DEG_F) +
                L" limit_deg=" + std::to_wstring(kHeadingToleranceRad * RAD_TO_DEG_F);
            Assert::IsTrue(headingErrorRad <= kHeadingToleranceRad, message.c_str());
        }

    };

    TEST_CLASS(DriveManeuverS90SSContractTest)
    {
        static constexpr ManeuverCode kCode = S90SS;
        static constexpr bool kSmoothTurn = true;

    public:
        TEST_METHOD(Completes)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"completion", kCode, trace);
            Assert::IsTrue(trace.completed, message.c_str());
        }

        TEST_METHOD(CommandSamplesCaptured)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"command_samples", kCode, trace);
            Assert::IsTrue(!trace.samples.empty(), message.c_str());
        }

        TEST_METHOD(LeftReturnedCommandIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"left_returned_command", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastReturnedCommand.LeftCommand()) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.leftReturnedCommandFinite, message.c_str());
        }

        TEST_METHOD(RightReturnedCommandIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"right_returned_command", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastReturnedCommand.RightCommand()) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.rightReturnedCommandFinite, message.c_str());
        }

        TEST_METHOD(BodyProposalEvidenceIsSet)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"body_proposal_evidence", kCode, trace) +
                L" actual_flags=" + std::to_wstring(trace.lastTelemetry.commandKindFlags) +
                L" required_mask=" + std::to_wstring(DriveTelemetry::kCommandKindBodyProposal);
            Assert::IsTrue(trace.bodyProposalEvidenceSet, message.c_str());
        }

        TEST_METHOD(CommandTelemetryEvidenceIsSet)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"command_telemetry_evidence", kCode, trace) +
                L" actual_flags=" + std::to_wstring(trace.lastTelemetry.telemetryValidFlags) +
                L" required_mask=" + std::to_wstring(DriveTelemetry::kTelemetryCommandEvidenceValid);
            Assert::IsTrue(trace.commandTelemetryEvidenceSet, message.c_str());
        }

        TEST_METHOD(LeftDriveEvidenceMatchesCommand)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"left_drive_evidence", kCode, trace) +
                L" expected=" + std::to_wstring(trace.lastReturnedCommand.LeftCommand()) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.leftDriveCommand) +
                L" tolerance=1e-6";
            Assert::IsTrue(trace.leftCommandEvidenceMatchesReturnedCommand, message.c_str());
        }

        TEST_METHOD(RightDriveEvidenceMatchesCommand)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"right_drive_evidence", kCode, trace) +
                L" expected=" + std::to_wstring(trace.lastReturnedCommand.RightCommand()) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.rightDriveCommand) +
                L" tolerance=1e-6";
            Assert::IsTrue(trace.rightCommandEvidenceMatchesReturnedCommand, message.c_str());
        }

        TEST_METHOD(RequestedForwardIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_forward_mps", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedForwardMps) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedForwardMpsFinite, message.c_str());
        }

        TEST_METHOD(RequestedYawRateIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_yaw_rate_radps", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedYawRateRadps) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedYawRateRadpsFinite, message.c_str());
        }

        TEST_METHOD(RequestedForwardAccelIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_forward_accel_mps2", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedForwardAccelMps2) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedForwardAccelMps2Finite, message.c_str());
        }

        TEST_METHOD(RequestedYawAccelIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_yaw_accel_radps2", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedYawAccelRadps2) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedYawAccelRadps2Finite, message.c_str());
        }

        TEST_METHOD(RequestedYawIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_yaw_rad", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedYawRad) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedYawRadFinite, message.c_str());
        }

        TEST_METHOD(TruthPositionXIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetPositionX();
            const std::wstring message = BuildManeuverMessage(L"truth_position_x", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthPositionYIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetPositionY();
            const std::wstring message = BuildManeuverMessage(L"truth_position_y", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthHeadingIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetHeading();
            const std::wstring message = BuildManeuverMessage(L"truth_heading", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthForwardVelocityIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetForwardVelocity();
            const std::wstring message = BuildManeuverMessage(L"truth_forward_velocity", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthRightwardVelocityIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetRightwardVelocity();
            const std::wstring message = BuildManeuverMessage(L"truth_rightward_velocity", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthYawRateIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetYawRate();
            const std::wstring message = BuildManeuverMessage(L"truth_yaw_rate", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthLeftWheelSpeedIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetWheelSpeedLeft();
            const std::wstring message = BuildManeuverMessage(L"truth_left_wheel_speed", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthRightWheelSpeedIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetWheelSpeedRight();
            const std::wstring message = BuildManeuverMessage(L"truth_right_wheel_speed", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }


        TEST_METHOD(VelocityStable)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float normalizedSpan = ComputeSmoothVelocityNormalizedSpan(trace);
            const std::wstring message = BuildManeuverMessage(L"velocity", kCode, trace) +
                L" span=" + std::to_wstring(normalizedSpan) +
                L" limit=" + std::to_wstring(kVelocityVariationLimit);
            Assert::IsTrue(normalizedSpan < kVelocityVariationLimit, message.c_str());
        }

        TEST_METHOD(YawAccelerationStable)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float normalizedSpan = ComputeSmoothYawAccelerationNormalizedSpan(trace);
            const std::wstring message = BuildManeuverMessage(L"yaw_accel", kCode, trace) +
                L" span=" + std::to_wstring(normalizedSpan) +
                L" limit=" + std::to_wstring(kYawAccelerationVariationLimit);
            Assert::IsTrue(normalizedSpan < kYawAccelerationVariationLimit, message.c_str());
        }

        TEST_METHOD(YawRateStable)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float normalizedSpan = ComputeSmoothYawRateNormalizedSpan(trace);
            const std::wstring message = BuildManeuverMessage(L"yaw_rate", kCode, trace) +
                L" span=" + std::to_wstring(normalizedSpan) +
                L" limit=" + std::to_wstring(kYawRateVariationLimit);
            Assert::IsTrue(normalizedSpan < kYawRateVariationLimit, message.c_str());
        }

        TEST_METHOD(FinalPositionWithinTolerance)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float positionErrorMeters = ComputeSmoothFinalPositionErrorMeters(kCode, trace);
            const std::wstring message = BuildManeuverMessage(L"position", kCode, trace) +
                L" error_m=" + std::to_wstring(positionErrorMeters) +
                L" limit_m=" + std::to_wstring(kSmoothPositionToleranceM);
            Assert::IsTrue(positionErrorMeters <= kSmoothPositionToleranceM, message.c_str());
        }

        TEST_METHOD(FinalHeadingWithinTolerance)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float headingErrorRad = ComputeSmoothFinalHeadingErrorRad(kCode, trace);
            const std::wstring message = BuildManeuverMessage(L"heading", kCode, trace) +
                L" error_deg=" + std::to_wstring(headingErrorRad * RAD_TO_DEG_F) +
                L" limit_deg=" + std::to_wstring(kHeadingToleranceRad * RAD_TO_DEG_F);
            Assert::IsTrue(headingErrorRad <= kHeadingToleranceRad, message.c_str());
        }

    };

    TEST_CLASS(DriveManeuverS90SDContractTest)
    {
        static constexpr ManeuverCode kCode = S90SD;
        static constexpr bool kSmoothTurn = true;

    public:
        TEST_METHOD(Completes)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"completion", kCode, trace);
            Assert::IsTrue(trace.completed, message.c_str());
        }

        TEST_METHOD(CommandSamplesCaptured)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"command_samples", kCode, trace);
            Assert::IsTrue(!trace.samples.empty(), message.c_str());
        }

        TEST_METHOD(LeftReturnedCommandIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"left_returned_command", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastReturnedCommand.LeftCommand()) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.leftReturnedCommandFinite, message.c_str());
        }

        TEST_METHOD(RightReturnedCommandIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"right_returned_command", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastReturnedCommand.RightCommand()) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.rightReturnedCommandFinite, message.c_str());
        }

        TEST_METHOD(BodyProposalEvidenceIsSet)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"body_proposal_evidence", kCode, trace) +
                L" actual_flags=" + std::to_wstring(trace.lastTelemetry.commandKindFlags) +
                L" required_mask=" + std::to_wstring(DriveTelemetry::kCommandKindBodyProposal);
            Assert::IsTrue(trace.bodyProposalEvidenceSet, message.c_str());
        }

        TEST_METHOD(CommandTelemetryEvidenceIsSet)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"command_telemetry_evidence", kCode, trace) +
                L" actual_flags=" + std::to_wstring(trace.lastTelemetry.telemetryValidFlags) +
                L" required_mask=" + std::to_wstring(DriveTelemetry::kTelemetryCommandEvidenceValid);
            Assert::IsTrue(trace.commandTelemetryEvidenceSet, message.c_str());
        }

        TEST_METHOD(LeftDriveEvidenceMatchesCommand)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"left_drive_evidence", kCode, trace) +
                L" expected=" + std::to_wstring(trace.lastReturnedCommand.LeftCommand()) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.leftDriveCommand) +
                L" tolerance=1e-6";
            Assert::IsTrue(trace.leftCommandEvidenceMatchesReturnedCommand, message.c_str());
        }

        TEST_METHOD(RightDriveEvidenceMatchesCommand)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"right_drive_evidence", kCode, trace) +
                L" expected=" + std::to_wstring(trace.lastReturnedCommand.RightCommand()) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.rightDriveCommand) +
                L" tolerance=1e-6";
            Assert::IsTrue(trace.rightCommandEvidenceMatchesReturnedCommand, message.c_str());
        }

        TEST_METHOD(RequestedForwardIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_forward_mps", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedForwardMps) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedForwardMpsFinite, message.c_str());
        }

        TEST_METHOD(RequestedYawRateIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_yaw_rate_radps", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedYawRateRadps) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedYawRateRadpsFinite, message.c_str());
        }

        TEST_METHOD(RequestedForwardAccelIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_forward_accel_mps2", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedForwardAccelMps2) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedForwardAccelMps2Finite, message.c_str());
        }

        TEST_METHOD(RequestedYawAccelIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_yaw_accel_radps2", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedYawAccelRadps2) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedYawAccelRadps2Finite, message.c_str());
        }

        TEST_METHOD(RequestedYawIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_yaw_rad", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedYawRad) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedYawRadFinite, message.c_str());
        }

        TEST_METHOD(TruthPositionXIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetPositionX();
            const std::wstring message = BuildManeuverMessage(L"truth_position_x", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthPositionYIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetPositionY();
            const std::wstring message = BuildManeuverMessage(L"truth_position_y", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthHeadingIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetHeading();
            const std::wstring message = BuildManeuverMessage(L"truth_heading", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthForwardVelocityIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetForwardVelocity();
            const std::wstring message = BuildManeuverMessage(L"truth_forward_velocity", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthRightwardVelocityIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetRightwardVelocity();
            const std::wstring message = BuildManeuverMessage(L"truth_rightward_velocity", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthYawRateIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetYawRate();
            const std::wstring message = BuildManeuverMessage(L"truth_yaw_rate", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthLeftWheelSpeedIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetWheelSpeedLeft();
            const std::wstring message = BuildManeuverMessage(L"truth_left_wheel_speed", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthRightWheelSpeedIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetWheelSpeedRight();
            const std::wstring message = BuildManeuverMessage(L"truth_right_wheel_speed", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }


        TEST_METHOD(VelocityStable)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float normalizedSpan = ComputeSmoothVelocityNormalizedSpan(trace);
            const std::wstring message = BuildManeuverMessage(L"velocity", kCode, trace) +
                L" span=" + std::to_wstring(normalizedSpan) +
                L" limit=" + std::to_wstring(kVelocityVariationLimit);
            Assert::IsTrue(normalizedSpan < kVelocityVariationLimit, message.c_str());
        }

        TEST_METHOD(YawAccelerationStable)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float normalizedSpan = ComputeSmoothYawAccelerationNormalizedSpan(trace);
            const std::wstring message = BuildManeuverMessage(L"yaw_accel", kCode, trace) +
                L" span=" + std::to_wstring(normalizedSpan) +
                L" limit=" + std::to_wstring(kYawAccelerationVariationLimit);
            Assert::IsTrue(normalizedSpan < kYawAccelerationVariationLimit, message.c_str());
        }

        TEST_METHOD(YawRateStable)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float normalizedSpan = ComputeSmoothYawRateNormalizedSpan(trace);
            const std::wstring message = BuildManeuverMessage(L"yaw_rate", kCode, trace) +
                L" span=" + std::to_wstring(normalizedSpan) +
                L" limit=" + std::to_wstring(kYawRateVariationLimit);
            Assert::IsTrue(normalizedSpan < kYawRateVariationLimit, message.c_str());
        }

        TEST_METHOD(FinalPositionWithinTolerance)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float positionErrorMeters = ComputeSmoothFinalPositionErrorMeters(kCode, trace);
            const std::wstring message = BuildManeuverMessage(L"position", kCode, trace) +
                L" error_m=" + std::to_wstring(positionErrorMeters) +
                L" limit_m=" + std::to_wstring(kSmoothPositionToleranceM);
            Assert::IsTrue(positionErrorMeters <= kSmoothPositionToleranceM, message.c_str());
        }

        TEST_METHOD(FinalHeadingWithinTolerance)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float headingErrorRad = ComputeSmoothFinalHeadingErrorRad(kCode, trace);
            const std::wstring message = BuildManeuverMessage(L"heading", kCode, trace) +
                L" error_deg=" + std::to_wstring(headingErrorRad * RAD_TO_DEG_F) +
                L" limit_deg=" + std::to_wstring(kHeadingToleranceRad * RAD_TO_DEG_F);
            Assert::IsTrue(headingErrorRad <= kHeadingToleranceRad, message.c_str());
        }

    };

    TEST_CLASS(DriveManeuverS135LSContractTest)
    {
        static constexpr ManeuverCode kCode = S135LS;
        static constexpr bool kSmoothTurn = true;

    public:
        TEST_METHOD(Completes)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"completion", kCode, trace);
            Assert::IsTrue(trace.completed, message.c_str());
        }

        TEST_METHOD(CommandSamplesCaptured)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"command_samples", kCode, trace);
            Assert::IsTrue(!trace.samples.empty(), message.c_str());
        }

        TEST_METHOD(LeftReturnedCommandIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"left_returned_command", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastReturnedCommand.LeftCommand()) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.leftReturnedCommandFinite, message.c_str());
        }

        TEST_METHOD(RightReturnedCommandIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"right_returned_command", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastReturnedCommand.RightCommand()) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.rightReturnedCommandFinite, message.c_str());
        }

        TEST_METHOD(BodyProposalEvidenceIsSet)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"body_proposal_evidence", kCode, trace) +
                L" actual_flags=" + std::to_wstring(trace.lastTelemetry.commandKindFlags) +
                L" required_mask=" + std::to_wstring(DriveTelemetry::kCommandKindBodyProposal);
            Assert::IsTrue(trace.bodyProposalEvidenceSet, message.c_str());
        }

        TEST_METHOD(CommandTelemetryEvidenceIsSet)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"command_telemetry_evidence", kCode, trace) +
                L" actual_flags=" + std::to_wstring(trace.lastTelemetry.telemetryValidFlags) +
                L" required_mask=" + std::to_wstring(DriveTelemetry::kTelemetryCommandEvidenceValid);
            Assert::IsTrue(trace.commandTelemetryEvidenceSet, message.c_str());
        }

        TEST_METHOD(LeftDriveEvidenceMatchesCommand)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"left_drive_evidence", kCode, trace) +
                L" expected=" + std::to_wstring(trace.lastReturnedCommand.LeftCommand()) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.leftDriveCommand) +
                L" tolerance=1e-6";
            Assert::IsTrue(trace.leftCommandEvidenceMatchesReturnedCommand, message.c_str());
        }

        TEST_METHOD(RightDriveEvidenceMatchesCommand)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"right_drive_evidence", kCode, trace) +
                L" expected=" + std::to_wstring(trace.lastReturnedCommand.RightCommand()) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.rightDriveCommand) +
                L" tolerance=1e-6";
            Assert::IsTrue(trace.rightCommandEvidenceMatchesReturnedCommand, message.c_str());
        }

        TEST_METHOD(RequestedForwardIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_forward_mps", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedForwardMps) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedForwardMpsFinite, message.c_str());
        }

        TEST_METHOD(RequestedYawRateIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_yaw_rate_radps", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedYawRateRadps) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedYawRateRadpsFinite, message.c_str());
        }

        TEST_METHOD(RequestedForwardAccelIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_forward_accel_mps2", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedForwardAccelMps2) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedForwardAccelMps2Finite, message.c_str());
        }

        TEST_METHOD(RequestedYawAccelIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_yaw_accel_radps2", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedYawAccelRadps2) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedYawAccelRadps2Finite, message.c_str());
        }

        TEST_METHOD(RequestedYawIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_yaw_rad", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedYawRad) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedYawRadFinite, message.c_str());
        }

        TEST_METHOD(TruthPositionXIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetPositionX();
            const std::wstring message = BuildManeuverMessage(L"truth_position_x", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthPositionYIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetPositionY();
            const std::wstring message = BuildManeuverMessage(L"truth_position_y", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthHeadingIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetHeading();
            const std::wstring message = BuildManeuverMessage(L"truth_heading", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthForwardVelocityIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetForwardVelocity();
            const std::wstring message = BuildManeuverMessage(L"truth_forward_velocity", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthRightwardVelocityIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetRightwardVelocity();
            const std::wstring message = BuildManeuverMessage(L"truth_rightward_velocity", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthYawRateIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetYawRate();
            const std::wstring message = BuildManeuverMessage(L"truth_yaw_rate", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthLeftWheelSpeedIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetWheelSpeedLeft();
            const std::wstring message = BuildManeuverMessage(L"truth_left_wheel_speed", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthRightWheelSpeedIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetWheelSpeedRight();
            const std::wstring message = BuildManeuverMessage(L"truth_right_wheel_speed", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }


        TEST_METHOD(VelocityStable)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float normalizedSpan = ComputeSmoothVelocityNormalizedSpan(trace);
            const std::wstring message = BuildManeuverMessage(L"velocity", kCode, trace) +
                L" span=" + std::to_wstring(normalizedSpan) +
                L" limit=" + std::to_wstring(kVelocityVariationLimit);
            Assert::IsTrue(normalizedSpan < kVelocityVariationLimit, message.c_str());
        }

        TEST_METHOD(YawAccelerationStable)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float normalizedSpan = ComputeSmoothYawAccelerationNormalizedSpan(trace);
            const std::wstring message = BuildManeuverMessage(L"yaw_accel", kCode, trace) +
                L" span=" + std::to_wstring(normalizedSpan) +
                L" limit=" + std::to_wstring(kYawAccelerationVariationLimit);
            Assert::IsTrue(normalizedSpan < kYawAccelerationVariationLimit, message.c_str());
        }

        TEST_METHOD(YawRateStable)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float normalizedSpan = ComputeSmoothYawRateNormalizedSpan(trace);
            const std::wstring message = BuildManeuverMessage(L"yaw_rate", kCode, trace) +
                L" span=" + std::to_wstring(normalizedSpan) +
                L" limit=" + std::to_wstring(kYawRateVariationLimit);
            Assert::IsTrue(normalizedSpan < kYawRateVariationLimit, message.c_str());
        }

        TEST_METHOD(FinalPositionWithinTolerance)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float positionErrorMeters = ComputeSmoothFinalPositionErrorMeters(kCode, trace);
            const std::wstring message = BuildManeuverMessage(L"position", kCode, trace) +
                L" error_m=" + std::to_wstring(positionErrorMeters) +
                L" limit_m=" + std::to_wstring(kSmoothPositionToleranceM);
            Assert::IsTrue(positionErrorMeters <= kSmoothPositionToleranceM, message.c_str());
        }

        TEST_METHOD(FinalHeadingWithinTolerance)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float headingErrorRad = ComputeSmoothFinalHeadingErrorRad(kCode, trace);
            const std::wstring message = BuildManeuverMessage(L"heading", kCode, trace) +
                L" error_deg=" + std::to_wstring(headingErrorRad * RAD_TO_DEG_F) +
                L" limit_deg=" + std::to_wstring(kHeadingToleranceRad * RAD_TO_DEG_F);
            Assert::IsTrue(headingErrorRad <= kHeadingToleranceRad, message.c_str());
        }

    };

    TEST_CLASS(DriveManeuverS135LDContractTest)
    {
        static constexpr ManeuverCode kCode = S135LD;
        static constexpr bool kSmoothTurn = true;

    public:
        TEST_METHOD(Completes)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"completion", kCode, trace);
            Assert::IsTrue(trace.completed, message.c_str());
        }

        TEST_METHOD(CommandSamplesCaptured)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"command_samples", kCode, trace);
            Assert::IsTrue(!trace.samples.empty(), message.c_str());
        }

        TEST_METHOD(LeftReturnedCommandIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"left_returned_command", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastReturnedCommand.LeftCommand()) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.leftReturnedCommandFinite, message.c_str());
        }

        TEST_METHOD(RightReturnedCommandIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"right_returned_command", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastReturnedCommand.RightCommand()) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.rightReturnedCommandFinite, message.c_str());
        }

        TEST_METHOD(BodyProposalEvidenceIsSet)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"body_proposal_evidence", kCode, trace) +
                L" actual_flags=" + std::to_wstring(trace.lastTelemetry.commandKindFlags) +
                L" required_mask=" + std::to_wstring(DriveTelemetry::kCommandKindBodyProposal);
            Assert::IsTrue(trace.bodyProposalEvidenceSet, message.c_str());
        }

        TEST_METHOD(CommandTelemetryEvidenceIsSet)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"command_telemetry_evidence", kCode, trace) +
                L" actual_flags=" + std::to_wstring(trace.lastTelemetry.telemetryValidFlags) +
                L" required_mask=" + std::to_wstring(DriveTelemetry::kTelemetryCommandEvidenceValid);
            Assert::IsTrue(trace.commandTelemetryEvidenceSet, message.c_str());
        }

        TEST_METHOD(LeftDriveEvidenceMatchesCommand)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"left_drive_evidence", kCode, trace) +
                L" expected=" + std::to_wstring(trace.lastReturnedCommand.LeftCommand()) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.leftDriveCommand) +
                L" tolerance=1e-6";
            Assert::IsTrue(trace.leftCommandEvidenceMatchesReturnedCommand, message.c_str());
        }

        TEST_METHOD(RightDriveEvidenceMatchesCommand)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"right_drive_evidence", kCode, trace) +
                L" expected=" + std::to_wstring(trace.lastReturnedCommand.RightCommand()) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.rightDriveCommand) +
                L" tolerance=1e-6";
            Assert::IsTrue(trace.rightCommandEvidenceMatchesReturnedCommand, message.c_str());
        }

        TEST_METHOD(RequestedForwardIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_forward_mps", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedForwardMps) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedForwardMpsFinite, message.c_str());
        }

        TEST_METHOD(RequestedYawRateIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_yaw_rate_radps", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedYawRateRadps) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedYawRateRadpsFinite, message.c_str());
        }

        TEST_METHOD(RequestedForwardAccelIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_forward_accel_mps2", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedForwardAccelMps2) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedForwardAccelMps2Finite, message.c_str());
        }

        TEST_METHOD(RequestedYawAccelIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_yaw_accel_radps2", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedYawAccelRadps2) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedYawAccelRadps2Finite, message.c_str());
        }

        TEST_METHOD(RequestedYawIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_yaw_rad", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedYawRad) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedYawRadFinite, message.c_str());
        }

        TEST_METHOD(TruthPositionXIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetPositionX();
            const std::wstring message = BuildManeuverMessage(L"truth_position_x", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthPositionYIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetPositionY();
            const std::wstring message = BuildManeuverMessage(L"truth_position_y", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthHeadingIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetHeading();
            const std::wstring message = BuildManeuverMessage(L"truth_heading", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthForwardVelocityIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetForwardVelocity();
            const std::wstring message = BuildManeuverMessage(L"truth_forward_velocity", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthRightwardVelocityIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetRightwardVelocity();
            const std::wstring message = BuildManeuverMessage(L"truth_rightward_velocity", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthYawRateIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetYawRate();
            const std::wstring message = BuildManeuverMessage(L"truth_yaw_rate", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthLeftWheelSpeedIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetWheelSpeedLeft();
            const std::wstring message = BuildManeuverMessage(L"truth_left_wheel_speed", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthRightWheelSpeedIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetWheelSpeedRight();
            const std::wstring message = BuildManeuverMessage(L"truth_right_wheel_speed", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }


        TEST_METHOD(VelocityStable)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float normalizedSpan = ComputeSmoothVelocityNormalizedSpan(trace);
            const std::wstring message = BuildManeuverMessage(L"velocity", kCode, trace) +
                L" span=" + std::to_wstring(normalizedSpan) +
                L" limit=" + std::to_wstring(kVelocityVariationLimit);
            Assert::IsTrue(normalizedSpan < kVelocityVariationLimit, message.c_str());
        }

        TEST_METHOD(YawAccelerationStable)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float normalizedSpan = ComputeSmoothYawAccelerationNormalizedSpan(trace);
            const std::wstring message = BuildManeuverMessage(L"yaw_accel", kCode, trace) +
                L" span=" + std::to_wstring(normalizedSpan) +
                L" limit=" + std::to_wstring(kYawAccelerationVariationLimit);
            Assert::IsTrue(normalizedSpan < kYawAccelerationVariationLimit, message.c_str());
        }

        TEST_METHOD(YawRateStable)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float normalizedSpan = ComputeSmoothYawRateNormalizedSpan(trace);
            const std::wstring message = BuildManeuverMessage(L"yaw_rate", kCode, trace) +
                L" span=" + std::to_wstring(normalizedSpan) +
                L" limit=" + std::to_wstring(kYawRateVariationLimit);
            Assert::IsTrue(normalizedSpan < kYawRateVariationLimit, message.c_str());
        }

        TEST_METHOD(FinalPositionWithinTolerance)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float positionErrorMeters = ComputeSmoothFinalPositionErrorMeters(kCode, trace);
            const std::wstring message = BuildManeuverMessage(L"position", kCode, trace) +
                L" error_m=" + std::to_wstring(positionErrorMeters) +
                L" limit_m=" + std::to_wstring(kSmoothPositionToleranceM);
            Assert::IsTrue(positionErrorMeters <= kSmoothPositionToleranceM, message.c_str());
        }

        TEST_METHOD(FinalHeadingWithinTolerance)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float headingErrorRad = ComputeSmoothFinalHeadingErrorRad(kCode, trace);
            const std::wstring message = BuildManeuverMessage(L"heading", kCode, trace) +
                L" error_deg=" + std::to_wstring(headingErrorRad * RAD_TO_DEG_F) +
                L" limit_deg=" + std::to_wstring(kHeadingToleranceRad * RAD_TO_DEG_F);
            Assert::IsTrue(headingErrorRad <= kHeadingToleranceRad, message.c_str());
        }

    };

    TEST_CLASS(DriveManeuverS135SSContractTest)
    {
        static constexpr ManeuverCode kCode = S135SS;
        static constexpr bool kSmoothTurn = true;

    public:
        TEST_METHOD(Completes)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"completion", kCode, trace);
            Assert::IsTrue(trace.completed, message.c_str());
        }

        TEST_METHOD(CommandSamplesCaptured)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"command_samples", kCode, trace);
            Assert::IsTrue(!trace.samples.empty(), message.c_str());
        }

        TEST_METHOD(LeftReturnedCommandIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"left_returned_command", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastReturnedCommand.LeftCommand()) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.leftReturnedCommandFinite, message.c_str());
        }

        TEST_METHOD(RightReturnedCommandIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"right_returned_command", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastReturnedCommand.RightCommand()) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.rightReturnedCommandFinite, message.c_str());
        }

        TEST_METHOD(BodyProposalEvidenceIsSet)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"body_proposal_evidence", kCode, trace) +
                L" actual_flags=" + std::to_wstring(trace.lastTelemetry.commandKindFlags) +
                L" required_mask=" + std::to_wstring(DriveTelemetry::kCommandKindBodyProposal);
            Assert::IsTrue(trace.bodyProposalEvidenceSet, message.c_str());
        }

        TEST_METHOD(CommandTelemetryEvidenceIsSet)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"command_telemetry_evidence", kCode, trace) +
                L" actual_flags=" + std::to_wstring(trace.lastTelemetry.telemetryValidFlags) +
                L" required_mask=" + std::to_wstring(DriveTelemetry::kTelemetryCommandEvidenceValid);
            Assert::IsTrue(trace.commandTelemetryEvidenceSet, message.c_str());
        }

        TEST_METHOD(LeftDriveEvidenceMatchesCommand)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"left_drive_evidence", kCode, trace) +
                L" expected=" + std::to_wstring(trace.lastReturnedCommand.LeftCommand()) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.leftDriveCommand) +
                L" tolerance=1e-6";
            Assert::IsTrue(trace.leftCommandEvidenceMatchesReturnedCommand, message.c_str());
        }

        TEST_METHOD(RightDriveEvidenceMatchesCommand)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"right_drive_evidence", kCode, trace) +
                L" expected=" + std::to_wstring(trace.lastReturnedCommand.RightCommand()) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.rightDriveCommand) +
                L" tolerance=1e-6";
            Assert::IsTrue(trace.rightCommandEvidenceMatchesReturnedCommand, message.c_str());
        }

        TEST_METHOD(RequestedForwardIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_forward_mps", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedForwardMps) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedForwardMpsFinite, message.c_str());
        }

        TEST_METHOD(RequestedYawRateIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_yaw_rate_radps", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedYawRateRadps) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedYawRateRadpsFinite, message.c_str());
        }

        TEST_METHOD(RequestedForwardAccelIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_forward_accel_mps2", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedForwardAccelMps2) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedForwardAccelMps2Finite, message.c_str());
        }

        TEST_METHOD(RequestedYawAccelIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_yaw_accel_radps2", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedYawAccelRadps2) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedYawAccelRadps2Finite, message.c_str());
        }

        TEST_METHOD(RequestedYawIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_yaw_rad", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedYawRad) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedYawRadFinite, message.c_str());
        }

        TEST_METHOD(TruthPositionXIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetPositionX();
            const std::wstring message = BuildManeuverMessage(L"truth_position_x", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthPositionYIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetPositionY();
            const std::wstring message = BuildManeuverMessage(L"truth_position_y", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthHeadingIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetHeading();
            const std::wstring message = BuildManeuverMessage(L"truth_heading", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthForwardVelocityIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetForwardVelocity();
            const std::wstring message = BuildManeuverMessage(L"truth_forward_velocity", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthRightwardVelocityIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetRightwardVelocity();
            const std::wstring message = BuildManeuverMessage(L"truth_rightward_velocity", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthYawRateIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetYawRate();
            const std::wstring message = BuildManeuverMessage(L"truth_yaw_rate", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthLeftWheelSpeedIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetWheelSpeedLeft();
            const std::wstring message = BuildManeuverMessage(L"truth_left_wheel_speed", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthRightWheelSpeedIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetWheelSpeedRight();
            const std::wstring message = BuildManeuverMessage(L"truth_right_wheel_speed", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }


        TEST_METHOD(VelocityStable)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float normalizedSpan = ComputeSmoothVelocityNormalizedSpan(trace);
            const std::wstring message = BuildManeuverMessage(L"velocity", kCode, trace) +
                L" span=" + std::to_wstring(normalizedSpan) +
                L" limit=" + std::to_wstring(kVelocityVariationLimit);
            Assert::IsTrue(normalizedSpan < kVelocityVariationLimit, message.c_str());
        }

        TEST_METHOD(YawAccelerationStable)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float normalizedSpan = ComputeSmoothYawAccelerationNormalizedSpan(trace);
            const std::wstring message = BuildManeuverMessage(L"yaw_accel", kCode, trace) +
                L" span=" + std::to_wstring(normalizedSpan) +
                L" limit=" + std::to_wstring(kYawAccelerationVariationLimit);
            Assert::IsTrue(normalizedSpan < kYawAccelerationVariationLimit, message.c_str());
        }

        TEST_METHOD(YawRateStable)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float normalizedSpan = ComputeSmoothYawRateNormalizedSpan(trace);
            const std::wstring message = BuildManeuverMessage(L"yaw_rate", kCode, trace) +
                L" span=" + std::to_wstring(normalizedSpan) +
                L" limit=" + std::to_wstring(kYawRateVariationLimit);
            Assert::IsTrue(normalizedSpan < kYawRateVariationLimit, message.c_str());
        }

        TEST_METHOD(FinalPositionWithinTolerance)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float positionErrorMeters = ComputeSmoothFinalPositionErrorMeters(kCode, trace);
            const std::wstring message = BuildManeuverMessage(L"position", kCode, trace) +
                L" error_m=" + std::to_wstring(positionErrorMeters) +
                L" limit_m=" + std::to_wstring(kSmoothPositionToleranceM);
            Assert::IsTrue(positionErrorMeters <= kSmoothPositionToleranceM, message.c_str());
        }

        TEST_METHOD(FinalHeadingWithinTolerance)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float headingErrorRad = ComputeSmoothFinalHeadingErrorRad(kCode, trace);
            const std::wstring message = BuildManeuverMessage(L"heading", kCode, trace) +
                L" error_deg=" + std::to_wstring(headingErrorRad * RAD_TO_DEG_F) +
                L" limit_deg=" + std::to_wstring(kHeadingToleranceRad * RAD_TO_DEG_F);
            Assert::IsTrue(headingErrorRad <= kHeadingToleranceRad, message.c_str());
        }

    };

    TEST_CLASS(DriveManeuverS135SDContractTest)
    {
        static constexpr ManeuverCode kCode = S135SD;
        static constexpr bool kSmoothTurn = true;

    public:
        TEST_METHOD(Completes)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"completion", kCode, trace);
            Assert::IsTrue(trace.completed, message.c_str());
        }

        TEST_METHOD(CommandSamplesCaptured)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"command_samples", kCode, trace);
            Assert::IsTrue(!trace.samples.empty(), message.c_str());
        }

        TEST_METHOD(LeftReturnedCommandIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"left_returned_command", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastReturnedCommand.LeftCommand()) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.leftReturnedCommandFinite, message.c_str());
        }

        TEST_METHOD(RightReturnedCommandIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"right_returned_command", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastReturnedCommand.RightCommand()) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.rightReturnedCommandFinite, message.c_str());
        }

        TEST_METHOD(BodyProposalEvidenceIsSet)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"body_proposal_evidence", kCode, trace) +
                L" actual_flags=" + std::to_wstring(trace.lastTelemetry.commandKindFlags) +
                L" required_mask=" + std::to_wstring(DriveTelemetry::kCommandKindBodyProposal);
            Assert::IsTrue(trace.bodyProposalEvidenceSet, message.c_str());
        }

        TEST_METHOD(CommandTelemetryEvidenceIsSet)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"command_telemetry_evidence", kCode, trace) +
                L" actual_flags=" + std::to_wstring(trace.lastTelemetry.telemetryValidFlags) +
                L" required_mask=" + std::to_wstring(DriveTelemetry::kTelemetryCommandEvidenceValid);
            Assert::IsTrue(trace.commandTelemetryEvidenceSet, message.c_str());
        }

        TEST_METHOD(LeftDriveEvidenceMatchesCommand)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"left_drive_evidence", kCode, trace) +
                L" expected=" + std::to_wstring(trace.lastReturnedCommand.LeftCommand()) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.leftDriveCommand) +
                L" tolerance=1e-6";
            Assert::IsTrue(trace.leftCommandEvidenceMatchesReturnedCommand, message.c_str());
        }

        TEST_METHOD(RightDriveEvidenceMatchesCommand)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"right_drive_evidence", kCode, trace) +
                L" expected=" + std::to_wstring(trace.lastReturnedCommand.RightCommand()) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.rightDriveCommand) +
                L" tolerance=1e-6";
            Assert::IsTrue(trace.rightCommandEvidenceMatchesReturnedCommand, message.c_str());
        }

        TEST_METHOD(RequestedForwardIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_forward_mps", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedForwardMps) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedForwardMpsFinite, message.c_str());
        }

        TEST_METHOD(RequestedYawRateIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_yaw_rate_radps", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedYawRateRadps) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedYawRateRadpsFinite, message.c_str());
        }

        TEST_METHOD(RequestedForwardAccelIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_forward_accel_mps2", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedForwardAccelMps2) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedForwardAccelMps2Finite, message.c_str());
        }

        TEST_METHOD(RequestedYawAccelIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_yaw_accel_radps2", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedYawAccelRadps2) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedYawAccelRadps2Finite, message.c_str());
        }

        TEST_METHOD(RequestedYawIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_yaw_rad", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedYawRad) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedYawRadFinite, message.c_str());
        }

        TEST_METHOD(TruthPositionXIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetPositionX();
            const std::wstring message = BuildManeuverMessage(L"truth_position_x", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthPositionYIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetPositionY();
            const std::wstring message = BuildManeuverMessage(L"truth_position_y", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthHeadingIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetHeading();
            const std::wstring message = BuildManeuverMessage(L"truth_heading", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthForwardVelocityIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetForwardVelocity();
            const std::wstring message = BuildManeuverMessage(L"truth_forward_velocity", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthRightwardVelocityIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetRightwardVelocity();
            const std::wstring message = BuildManeuverMessage(L"truth_rightward_velocity", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthYawRateIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetYawRate();
            const std::wstring message = BuildManeuverMessage(L"truth_yaw_rate", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthLeftWheelSpeedIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetWheelSpeedLeft();
            const std::wstring message = BuildManeuverMessage(L"truth_left_wheel_speed", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthRightWheelSpeedIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetWheelSpeedRight();
            const std::wstring message = BuildManeuverMessage(L"truth_right_wheel_speed", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }


        TEST_METHOD(VelocityStable)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float normalizedSpan = ComputeSmoothVelocityNormalizedSpan(trace);
            const std::wstring message = BuildManeuverMessage(L"velocity", kCode, trace) +
                L" span=" + std::to_wstring(normalizedSpan) +
                L" limit=" + std::to_wstring(kVelocityVariationLimit);
            Assert::IsTrue(normalizedSpan < kVelocityVariationLimit, message.c_str());
        }

        TEST_METHOD(YawAccelerationStable)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float normalizedSpan = ComputeSmoothYawAccelerationNormalizedSpan(trace);
            const std::wstring message = BuildManeuverMessage(L"yaw_accel", kCode, trace) +
                L" span=" + std::to_wstring(normalizedSpan) +
                L" limit=" + std::to_wstring(kYawAccelerationVariationLimit);
            Assert::IsTrue(normalizedSpan < kYawAccelerationVariationLimit, message.c_str());
        }

        TEST_METHOD(YawRateStable)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float normalizedSpan = ComputeSmoothYawRateNormalizedSpan(trace);
            const std::wstring message = BuildManeuverMessage(L"yaw_rate", kCode, trace) +
                L" span=" + std::to_wstring(normalizedSpan) +
                L" limit=" + std::to_wstring(kYawRateVariationLimit);
            Assert::IsTrue(normalizedSpan < kYawRateVariationLimit, message.c_str());
        }

        TEST_METHOD(FinalPositionWithinTolerance)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float positionErrorMeters = ComputeSmoothFinalPositionErrorMeters(kCode, trace);
            const std::wstring message = BuildManeuverMessage(L"position", kCode, trace) +
                L" error_m=" + std::to_wstring(positionErrorMeters) +
                L" limit_m=" + std::to_wstring(kSmoothPositionToleranceM);
            Assert::IsTrue(positionErrorMeters <= kSmoothPositionToleranceM, message.c_str());
        }

        TEST_METHOD(FinalHeadingWithinTolerance)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float headingErrorRad = ComputeSmoothFinalHeadingErrorRad(kCode, trace);
            const std::wstring message = BuildManeuverMessage(L"heading", kCode, trace) +
                L" error_deg=" + std::to_wstring(headingErrorRad * RAD_TO_DEG_F) +
                L" limit_deg=" + std::to_wstring(kHeadingToleranceRad * RAD_TO_DEG_F);
            Assert::IsTrue(headingErrorRad <= kHeadingToleranceRad, message.c_str());
        }

    };

    TEST_CLASS(DriveManeuverS180LSContractTest)
    {
        static constexpr ManeuverCode kCode = S180LS;
        static constexpr bool kSmoothTurn = true;

    public:
        TEST_METHOD(Completes)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"completion", kCode, trace);
            Assert::IsTrue(trace.completed, message.c_str());
        }

        TEST_METHOD(CommandSamplesCaptured)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"command_samples", kCode, trace);
            Assert::IsTrue(!trace.samples.empty(), message.c_str());
        }

        TEST_METHOD(LeftReturnedCommandIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"left_returned_command", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastReturnedCommand.LeftCommand()) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.leftReturnedCommandFinite, message.c_str());
        }

        TEST_METHOD(RightReturnedCommandIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"right_returned_command", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastReturnedCommand.RightCommand()) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.rightReturnedCommandFinite, message.c_str());
        }

        TEST_METHOD(BodyProposalEvidenceIsSet)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"body_proposal_evidence", kCode, trace) +
                L" actual_flags=" + std::to_wstring(trace.lastTelemetry.commandKindFlags) +
                L" required_mask=" + std::to_wstring(DriveTelemetry::kCommandKindBodyProposal);
            Assert::IsTrue(trace.bodyProposalEvidenceSet, message.c_str());
        }

        TEST_METHOD(CommandTelemetryEvidenceIsSet)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"command_telemetry_evidence", kCode, trace) +
                L" actual_flags=" + std::to_wstring(trace.lastTelemetry.telemetryValidFlags) +
                L" required_mask=" + std::to_wstring(DriveTelemetry::kTelemetryCommandEvidenceValid);
            Assert::IsTrue(trace.commandTelemetryEvidenceSet, message.c_str());
        }

        TEST_METHOD(LeftDriveEvidenceMatchesCommand)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"left_drive_evidence", kCode, trace) +
                L" expected=" + std::to_wstring(trace.lastReturnedCommand.LeftCommand()) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.leftDriveCommand) +
                L" tolerance=1e-6";
            Assert::IsTrue(trace.leftCommandEvidenceMatchesReturnedCommand, message.c_str());
        }

        TEST_METHOD(RightDriveEvidenceMatchesCommand)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"right_drive_evidence", kCode, trace) +
                L" expected=" + std::to_wstring(trace.lastReturnedCommand.RightCommand()) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.rightDriveCommand) +
                L" tolerance=1e-6";
            Assert::IsTrue(trace.rightCommandEvidenceMatchesReturnedCommand, message.c_str());
        }

        TEST_METHOD(RequestedForwardIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_forward_mps", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedForwardMps) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedForwardMpsFinite, message.c_str());
        }

        TEST_METHOD(RequestedYawRateIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_yaw_rate_radps", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedYawRateRadps) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedYawRateRadpsFinite, message.c_str());
        }

        TEST_METHOD(RequestedForwardAccelIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_forward_accel_mps2", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedForwardAccelMps2) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedForwardAccelMps2Finite, message.c_str());
        }

        TEST_METHOD(RequestedYawAccelIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_yaw_accel_radps2", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedYawAccelRadps2) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedYawAccelRadps2Finite, message.c_str());
        }

        TEST_METHOD(RequestedYawIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_yaw_rad", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedYawRad) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedYawRadFinite, message.c_str());
        }

        TEST_METHOD(TruthPositionXIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetPositionX();
            const std::wstring message = BuildManeuverMessage(L"truth_position_x", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthPositionYIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetPositionY();
            const std::wstring message = BuildManeuverMessage(L"truth_position_y", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthHeadingIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetHeading();
            const std::wstring message = BuildManeuverMessage(L"truth_heading", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthForwardVelocityIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetForwardVelocity();
            const std::wstring message = BuildManeuverMessage(L"truth_forward_velocity", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthRightwardVelocityIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetRightwardVelocity();
            const std::wstring message = BuildManeuverMessage(L"truth_rightward_velocity", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthYawRateIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetYawRate();
            const std::wstring message = BuildManeuverMessage(L"truth_yaw_rate", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthLeftWheelSpeedIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetWheelSpeedLeft();
            const std::wstring message = BuildManeuverMessage(L"truth_left_wheel_speed", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthRightWheelSpeedIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetWheelSpeedRight();
            const std::wstring message = BuildManeuverMessage(L"truth_right_wheel_speed", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }


        TEST_METHOD(VelocityStable)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float normalizedSpan = ComputeSmoothVelocityNormalizedSpan(trace);
            const std::wstring message = BuildManeuverMessage(L"velocity", kCode, trace) +
                L" span=" + std::to_wstring(normalizedSpan) +
                L" limit=" + std::to_wstring(kVelocityVariationLimit);
            Assert::IsTrue(normalizedSpan < kVelocityVariationLimit, message.c_str());
        }

        TEST_METHOD(YawAccelerationStable)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float normalizedSpan = ComputeSmoothYawAccelerationNormalizedSpan(trace);
            const std::wstring message = BuildManeuverMessage(L"yaw_accel", kCode, trace) +
                L" span=" + std::to_wstring(normalizedSpan) +
                L" limit=" + std::to_wstring(kYawAccelerationVariationLimit);
            Assert::IsTrue(normalizedSpan < kYawAccelerationVariationLimit, message.c_str());
        }

        TEST_METHOD(YawRateStable)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float normalizedSpan = ComputeSmoothYawRateNormalizedSpan(trace);
            const std::wstring message = BuildManeuverMessage(L"yaw_rate", kCode, trace) +
                L" span=" + std::to_wstring(normalizedSpan) +
                L" limit=" + std::to_wstring(kYawRateVariationLimit);
            Assert::IsTrue(normalizedSpan < kYawRateVariationLimit, message.c_str());
        }

        TEST_METHOD(FinalPositionWithinTolerance)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float positionErrorMeters = ComputeSmoothFinalPositionErrorMeters(kCode, trace);
            const std::wstring message = BuildManeuverMessage(L"position", kCode, trace) +
                L" error_m=" + std::to_wstring(positionErrorMeters) +
                L" limit_m=" + std::to_wstring(kSmoothPositionToleranceM);
            Assert::IsTrue(positionErrorMeters <= kSmoothPositionToleranceM, message.c_str());
        }

        TEST_METHOD(FinalHeadingWithinTolerance)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float headingErrorRad = ComputeSmoothFinalHeadingErrorRad(kCode, trace);
            const std::wstring message = BuildManeuverMessage(L"heading", kCode, trace) +
                L" error_deg=" + std::to_wstring(headingErrorRad * RAD_TO_DEG_F) +
                L" limit_deg=" + std::to_wstring(kHeadingToleranceRad * RAD_TO_DEG_F);
            Assert::IsTrue(headingErrorRad <= kHeadingToleranceRad, message.c_str());
        }

    };

    TEST_CLASS(DriveManeuverS180SSContractTest)
    {
        static constexpr ManeuverCode kCode = S180SS;
        static constexpr bool kSmoothTurn = true;

    public:
        TEST_METHOD(Completes)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"completion", kCode, trace);
            Assert::IsTrue(trace.completed, message.c_str());
        }

        TEST_METHOD(CommandSamplesCaptured)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"command_samples", kCode, trace);
            Assert::IsTrue(!trace.samples.empty(), message.c_str());
        }

        TEST_METHOD(LeftReturnedCommandIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"left_returned_command", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastReturnedCommand.LeftCommand()) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.leftReturnedCommandFinite, message.c_str());
        }

        TEST_METHOD(RightReturnedCommandIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"right_returned_command", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastReturnedCommand.RightCommand()) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.rightReturnedCommandFinite, message.c_str());
        }

        TEST_METHOD(BodyProposalEvidenceIsSet)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"body_proposal_evidence", kCode, trace) +
                L" actual_flags=" + std::to_wstring(trace.lastTelemetry.commandKindFlags) +
                L" required_mask=" + std::to_wstring(DriveTelemetry::kCommandKindBodyProposal);
            Assert::IsTrue(trace.bodyProposalEvidenceSet, message.c_str());
        }

        TEST_METHOD(CommandTelemetryEvidenceIsSet)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"command_telemetry_evidence", kCode, trace) +
                L" actual_flags=" + std::to_wstring(trace.lastTelemetry.telemetryValidFlags) +
                L" required_mask=" + std::to_wstring(DriveTelemetry::kTelemetryCommandEvidenceValid);
            Assert::IsTrue(trace.commandTelemetryEvidenceSet, message.c_str());
        }

        TEST_METHOD(LeftDriveEvidenceMatchesCommand)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"left_drive_evidence", kCode, trace) +
                L" expected=" + std::to_wstring(trace.lastReturnedCommand.LeftCommand()) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.leftDriveCommand) +
                L" tolerance=1e-6";
            Assert::IsTrue(trace.leftCommandEvidenceMatchesReturnedCommand, message.c_str());
        }

        TEST_METHOD(RightDriveEvidenceMatchesCommand)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"right_drive_evidence", kCode, trace) +
                L" expected=" + std::to_wstring(trace.lastReturnedCommand.RightCommand()) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.rightDriveCommand) +
                L" tolerance=1e-6";
            Assert::IsTrue(trace.rightCommandEvidenceMatchesReturnedCommand, message.c_str());
        }

        TEST_METHOD(RequestedForwardIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_forward_mps", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedForwardMps) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedForwardMpsFinite, message.c_str());
        }

        TEST_METHOD(RequestedYawRateIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_yaw_rate_radps", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedYawRateRadps) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedYawRateRadpsFinite, message.c_str());
        }

        TEST_METHOD(RequestedForwardAccelIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_forward_accel_mps2", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedForwardAccelMps2) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedForwardAccelMps2Finite, message.c_str());
        }

        TEST_METHOD(RequestedYawAccelIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_yaw_accel_radps2", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedYawAccelRadps2) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedYawAccelRadps2Finite, message.c_str());
        }

        TEST_METHOD(RequestedYawIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_yaw_rad", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedYawRad) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedYawRadFinite, message.c_str());
        }

        TEST_METHOD(TruthPositionXIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetPositionX();
            const std::wstring message = BuildManeuverMessage(L"truth_position_x", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthPositionYIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetPositionY();
            const std::wstring message = BuildManeuverMessage(L"truth_position_y", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthHeadingIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetHeading();
            const std::wstring message = BuildManeuverMessage(L"truth_heading", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthForwardVelocityIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetForwardVelocity();
            const std::wstring message = BuildManeuverMessage(L"truth_forward_velocity", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthRightwardVelocityIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetRightwardVelocity();
            const std::wstring message = BuildManeuverMessage(L"truth_rightward_velocity", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthYawRateIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetYawRate();
            const std::wstring message = BuildManeuverMessage(L"truth_yaw_rate", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthLeftWheelSpeedIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetWheelSpeedLeft();
            const std::wstring message = BuildManeuverMessage(L"truth_left_wheel_speed", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthRightWheelSpeedIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetWheelSpeedRight();
            const std::wstring message = BuildManeuverMessage(L"truth_right_wheel_speed", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }


        TEST_METHOD(VelocityStable)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float normalizedSpan = ComputeSmoothVelocityNormalizedSpan(trace);
            const std::wstring message = BuildManeuverMessage(L"velocity", kCode, trace) +
                L" span=" + std::to_wstring(normalizedSpan) +
                L" limit=" + std::to_wstring(kVelocityVariationLimit);
            Assert::IsTrue(normalizedSpan < kVelocityVariationLimit, message.c_str());
        }

        TEST_METHOD(YawAccelerationStable)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float normalizedSpan = ComputeSmoothYawAccelerationNormalizedSpan(trace);
            const std::wstring message = BuildManeuverMessage(L"yaw_accel", kCode, trace) +
                L" span=" + std::to_wstring(normalizedSpan) +
                L" limit=" + std::to_wstring(kYawAccelerationVariationLimit);
            Assert::IsTrue(normalizedSpan < kYawAccelerationVariationLimit, message.c_str());
        }

        TEST_METHOD(YawRateStable)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float normalizedSpan = ComputeSmoothYawRateNormalizedSpan(trace);
            const std::wstring message = BuildManeuverMessage(L"yaw_rate", kCode, trace) +
                L" span=" + std::to_wstring(normalizedSpan) +
                L" limit=" + std::to_wstring(kYawRateVariationLimit);
            Assert::IsTrue(normalizedSpan < kYawRateVariationLimit, message.c_str());
        }

        TEST_METHOD(FinalPositionWithinTolerance)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float positionErrorMeters = ComputeSmoothFinalPositionErrorMeters(kCode, trace);
            const std::wstring message = BuildManeuverMessage(L"position", kCode, trace) +
                L" error_m=" + std::to_wstring(positionErrorMeters) +
                L" limit_m=" + std::to_wstring(kSmoothPositionToleranceM);
            Assert::IsTrue(positionErrorMeters <= kSmoothPositionToleranceM, message.c_str());
        }

        TEST_METHOD(FinalHeadingWithinTolerance)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float headingErrorRad = ComputeSmoothFinalHeadingErrorRad(kCode, trace);
            const std::wstring message = BuildManeuverMessage(L"heading", kCode, trace) +
                L" error_deg=" + std::to_wstring(headingErrorRad * RAD_TO_DEG_F) +
                L" limit_deg=" + std::to_wstring(kHeadingToleranceRad * RAD_TO_DEG_F);
            Assert::IsTrue(headingErrorRad <= kHeadingToleranceRad, message.c_str());
        }

    };
}

