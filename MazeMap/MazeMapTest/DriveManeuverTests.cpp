#include "pch.h"
#include "CppUnitTest.h"

#include "EstimatorTestSupport.h"
#include "..\MazeMap\Drive.h"
#include "..\MazeMap\DriveBase.h"
#include "..\MazeMap\ManeuverInstance.h"
#include "..\MazeMap\ManeuverSet.h"
#include "..\MazeMap\SharedRobotRuntime.h"

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
        constexpr float kOmegaMagnitudeEpsilonRadps = 1.0e-4f;
        constexpr float kTurnPlateauFraction = 0.95f;
        constexpr std::size_t kRampDeltaTrimSamples = 5U;

        struct ScopedMissionFanDuty final
        {
            explicit ScopedMissionFanDuty(const float dutyCycle) noexcept
                : previousDutyCycle(GetMissionFanDutyCycle())
            {
                WriteFanDutyCycle(dutyCycle);
            }

            ~ScopedMissionFanDuty() noexcept
            {
                WriteFanDutyCycle(previousDutyCycle);
            }

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
            bool started = false;
            bool completed = false;
            float elapsedSeconds = 0.0f;
            VehicleState::StateVector truthState = VehicleState::StateVector::Zero();
            std::vector<CommandSample> samples;
        };

        struct CheckResult final
        {
            bool passed = false;
            std::wstring message;
        };

        SensorSnapshot BuildDriveManeuverSensorSnapshot(const float yawRateRadps = 0.0f) noexcept
        {
            SensorSnapshot snapshot{};
            snapshot.gyroRawRadps = yawRateRadps;
            snapshot.gyroRadps = yawRateRadps;
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
            DriveBase& drive,
            Estimator& estimator,
            VehicleState& runtimeState,
            const float leftDistanceDeltaM,
            const float rightDistanceDeltaM,
            const float yawRateRadps,
            float& leftEncoderRemainderCounts,
            float& rightEncoderRemainderCounts,
            const float dtSeconds)
        {
            const PlantParams params = PlantParams::Default();
            const float distancePerCountM = DistancePerEncoderCountMeters(params);
            const int32_t leftCounts =
                ConsumeWholeEncoderCounts(leftDistanceDeltaM / distancePerCountM, leftEncoderRemainderCounts);
            const int32_t rightCounts =
                ConsumeWholeEncoderCounts(rightDistanceDeltaM / distancePerCountM, rightEncoderRemainderCounts);

            SensorSnapshot snapshot = BuildDriveManeuverSensorSnapshot(yawRateRadps);
            snapshot.encoderObservation.totalLeftCounts = leftCounts;
            snapshot.encoderObservation.totalRightCounts = rightCounts;
            snapshot.encoderObservation.leftDistanceDeltaM = static_cast<float>(leftCounts) * distancePerCountM;
            snapshot.encoderObservation.rightDistanceDeltaM = static_cast<float>(rightCounts) * distancePerCountM;
            if ((dtSeconds > 0.0f) && std::isfinite(dtSeconds) && (params.wheelRadiusM > 0.0f))
            {
                const float invWheelRadiusM = 1.0f / params.wheelRadiusM;
                const float invDtSeconds = 1.0f / dtSeconds;
                snapshot.encoderObservation.leftVelocityMps =
                    snapshot.encoderObservation.leftDistanceDeltaM * invDtSeconds;
                snapshot.encoderObservation.rightVelocityMps =
                    snapshot.encoderObservation.rightDistanceDeltaM * invDtSeconds;
                snapshot.encoderObservation.omegaLeftRadps =
                    snapshot.encoderObservation.leftVelocityMps * invWheelRadiusM;
                snapshot.encoderObservation.omegaRightRadps =
                    snapshot.encoderObservation.rightVelocityMps * invWheelRadiusM;
            }
            snapshot.encoderObservationValid = true;
            UpdateDriveEstimator(
                drive,
                estimator,
                runtimeState,
                dtSeconds,
                snapshot);
        }

        void ApplyControlVector(DriveBase& drive, const CommandVector& control) noexcept
        {
            if (std::isfinite(control.LeftMotorPwm()) && std::isfinite(control.RightMotorPwm()))
            {
                drive.CommandOpenLoopRaw(control);
            }
            else
            {
                drive.Brake();
            }
        }

        VehicleState::StateVector BuildTruthState(const float linearSpeedMps) noexcept
        {
            const PlantParams params = PlantParams::Default();
            const float wheelOmegaRadps = linearSpeedMps / params.wheelRadiusM;
            return BuildUkfState(
                0.0f,
                0.0f,
                0.0f,
                linearSpeedMps,
                0.0f,
                0.0f,
                wheelOmegaRadps,
                wheelOmegaRadps,
                0.0f);
        }

        void PrimeDriveForSmoothEntry(
            Internal::SharedRobotRuntime& runtime,
            VehicleState::StateVector& truthState,
            float& leftEncoderRemainderCounts,
            float& rightEncoderRemainderCounts)
        {
            truthState = BuildTruthState(kSmoothEntrySpeedMps);
            const PlantParams params = PlantParams::Default();
            const float distancePerCountM = DistancePerEncoderCountMeters(params);
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
            Assert::IsTrue(runtime.Estimator().ResetPose(0.0f, -projectedForwardDistanceM, 0.0f));
            ApplyEncoderObservation(
                runtime.Drive(),
                runtime.Estimator(),
                runtime.RuntimeState(),
                kSmoothEntrySpeedMps * kSimulationDtSeconds,
                kSmoothEntrySpeedMps * kSimulationDtSeconds,
                0.0f,
                leftEncoderRemainderCounts,
                rightEncoderRemainderCounts,
                kSimulationDtSeconds);
        }

        void SimulateRuntimeDriveCycle(
            Internal::SharedRobotRuntime& runtime,
            PlantModel& plant,
            VehicleState::StateVector& truthState,
            float& leftEncoderRemainderCounts,
            float& rightEncoderRemainderCounts,
            const float dtSeconds)
        {
            const PlantParams params = PlantParams::Default();
            const DriveTelemetry telemetry = runtime.Drive().GetTelemetry();

            const CommandVector control = CommandVector(
                telemetry.leftDriveCommand,
                telemetry.rightDriveCommand);

            const VehicleState::StateVector previousTruthState = truthState;
            truthState = plant.integrate(truthState, control, 0.80f, params.supplyVoltageV, dtSeconds, params);

            const float leftDistanceDeltaM =
                0.5f *
                (previousTruthState(VehicleState::kOmegaL) + truthState(VehicleState::kOmegaL)) *
                params.wheelRadiusM *
                dtSeconds;
            const float rightDistanceDeltaM =
                0.5f *
                (previousTruthState(VehicleState::kOmegaR) + truthState(VehicleState::kOmegaR)) *
                params.wheelRadiusM *
                dtSeconds;

            ApplyEncoderObservation(
                runtime.Drive(),
                runtime.Estimator(),
                runtime.RuntimeState(),
                leftDistanceDeltaM,
                rightDistanceDeltaM,
                truthState(VehicleState::kR),
                leftEncoderRemainderCounts,
                rightEncoderRemainderCounts,
                dtSeconds);
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
            const float absoluteAngleRad = std::fabs(angleRad);
            const float maxAngularSpeedRadps = limits.maxAngularSpeedRadps;
            const float angularAccelRadps2 = limits.angularAccelRadps2;
            if (!(absoluteAngleRad > 0.0f) || !(angularAccelRadps2 > 0.0f))
            {
                return 0.0f;
            }

            if (!(maxAngularSpeedRadps > 0.0f))
            {
                return 2.0f * std::sqrt(absoluteAngleRad / angularAccelRadps2);
            }

            const float accelAndDecelAngleRad =
                (maxAngularSpeedRadps * maxAngularSpeedRadps) / angularAccelRadps2;
            if (absoluteAngleRad <= accelAndDecelAngleRad)
            {
                return 2.0f * std::sqrt(absoluteAngleRad / angularAccelRadps2);
            }

            return
                (2.0f * maxAngularSpeedRadps / angularAccelRadps2) +
                ((absoluteAngleRad - accelAndDecelAngleRad) / maxAngularSpeedRadps);
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
            ScopedMissionFanDuty fanDuty(0.80f);
            ManeuverExecutionTrace trace{};
            Internal::SharedRobotRuntime runtime;
            PlantModel& plant = runtime.Plant();
            float leftEncoderRemainderCounts = 0.0f;
            float rightEncoderRemainderCounts = 0.0f;

            if (!runtime.Drive().Begin())
            {
                return trace;
            }

            Assert::IsTrue(runtime.Estimator().ResetPose(0.0f, 0.0f, 0.0f));
            if (smoothTurn)
            {
                PrimeDriveForSmoothEntry(
                    runtime,
                    trace.truthState,
                    leftEncoderRemainderCounts,
                    rightEncoderRemainderCounts);
            }
            else
            {
                trace.truthState = BuildTruthState(0.0f);
            }

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
                    return trace;
                }

                bool done = false;
                const CommandVector control = driveService.GetNextControls(done);
                if (done)
                {
                    trace.completed = true;
                    return trace;
                }

                trace.samples.push_back(
                    CommandSample{
                        trace.elapsedSeconds,
                        runtime.Drive().GetLastLinearCommandMps(),
                        runtime.Drive().GetLastAngularCommandRadps()
                    });

                ApplyControlVector(runtime.Drive(), control);
                SimulateRuntimeDriveCycle(
                    runtime,
                    plant,
                    trace.truthState,
                    leftEncoderRemainderCounts,
                    rightEncoderRemainderCounts,
                    kSimulationDtSeconds);
                trace.elapsedSeconds += kSimulationDtSeconds;
            }

            return trace;
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

            float maxOmegaMagnitudeRadps = 0.0f;
            for (const CommandSample& sample : trace.samples)
            {
                maxOmegaMagnitudeRadps =
                    (std::max)(maxOmegaMagnitudeRadps, std::fabs(sample.angularCommandRadps));
            }

            const float plateauThresholdRadps = kTurnPlateauFraction * maxOmegaMagnitudeRadps;
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

            float maxOmegaMagnitudeRadps = 0.0f;
            std::size_t plateauBeginIndex = trace.samples.size();
            std::size_t plateauEndIndex = 0U;
            for (std::size_t index = 0U; index < trace.samples.size(); ++index)
            {
                maxOmegaMagnitudeRadps =
                    (std::max)(
                        maxOmegaMagnitudeRadps,
                        std::fabs(trace.samples[index].angularCommandRadps));
            }

            const float plateauThresholdRadps = kTurnPlateauFraction * maxOmegaMagnitudeRadps;
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
                    const float previousOmegaMagnitudeRadps =
                        std::fabs(trace.samples[index - 1U].angularCommandRadps);
                    const float currentOmegaMagnitudeRadps =
                        std::fabs(trace.samples[index].angularCommandRadps);
                    if ((previousOmegaMagnitudeRadps <= kOmegaMagnitudeEpsilonRadps) ||
                        (currentOmegaMagnitudeRadps <= kOmegaMagnitudeEpsilonRadps))
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

        CheckResult EvaluateInPlaceShift(const ManeuverCode code)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(code, false);
            const float shiftMeters =
                std::hypot(
                    trace.truthState(VehicleState::kPx),
                    trace.truthState(VehicleState::kPy));
            CheckResult result{};
            result.passed = trace.started && trace.completed && (shiftMeters < kInPlacePositionToleranceM);
            result.message =
                L"shift code=" + CodeLabel(code) +
                L" actual_m=" + std::to_wstring(shiftMeters) +
                L" limit_m=" + std::to_wstring(kInPlacePositionToleranceM) +
                L" completed=" + (trace.completed ? L"true" : L"false");
            return result;
        }

        CheckResult EvaluateInPlaceHeading(const ManeuverCode code)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(code, false);
            const float headingErrorRad =
                std::fabs(AngleErrorRad(BuildNominalEndYawRad(code), trace.truthState(VehicleState::kPsi)));
            CheckResult result{};
            result.passed = trace.started && trace.completed && (headingErrorRad <= kHeadingToleranceRad);
            result.message =
                L"heading code=" + CodeLabel(code) +
                L" error_deg=" + std::to_wstring(headingErrorRad * RAD_TO_DEG_F) +
                L" limit_deg=" + std::to_wstring(kHeadingToleranceRad * RAD_TO_DEG_F) +
                L" completed=" + (trace.completed ? L"true" : L"false");
            return result;
        }

        CheckResult EvaluateInPlaceTime(const ManeuverCode code)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(code, false);
            Internal::SharedRobotRuntime runtime;
            const float expectedTimeSeconds =
                ComputeInPlaceTurnKinematicTimeSeconds(
                    std::fabs(BuildNominalEndYawRad(code)),
                    runtime.DriveService().GetLimits());
            const float relativeError =
                (expectedTimeSeconds > 0.0f) ?
                (std::fabs(trace.elapsedSeconds - expectedTimeSeconds) / expectedTimeSeconds) :
                (std::numeric_limits<float>::infinity)();
            CheckResult result{};
            result.passed = trace.started && trace.completed && (relativeError <= kTimeToleranceFraction);
            result.message =
                L"time code=" + CodeLabel(code) +
                L" elapsed_s=" + std::to_wstring(trace.elapsedSeconds) +
                L" expected_s=" + std::to_wstring(expectedTimeSeconds) +
                L" rel_err=" + std::to_wstring(relativeError) +
                L" limit=" + std::to_wstring(kTimeToleranceFraction) +
                L" completed=" + (trace.completed ? L"true" : L"false");
            return result;
        }

        CheckResult EvaluateSmoothVelocityConstancy(const ManeuverCode code)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(code, true);
            const float normalizedSpan = ComputeNormalizedSpan(CollectLinearCommandMagnitudes(trace));
            CheckResult result{};
            result.passed = trace.started && trace.completed && (normalizedSpan < kVelocityVariationLimit);
            result.message =
                L"velocity code=" + CodeLabel(code) +
                L" span=" + std::to_wstring(normalizedSpan) +
                L" limit=" + std::to_wstring(kVelocityVariationLimit) +
                L" completed=" + (trace.completed ? L"true" : L"false");
            return result;
        }

        CheckResult EvaluateSmoothYawAccelerationConstancy(const ManeuverCode code)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(code, true);
            const float normalizedSpan = ComputeNormalizedSpan(CollectRampYawAccelMagnitudes(trace));
            CheckResult result{};
            result.passed = trace.started && trace.completed && (normalizedSpan < kYawAccelerationVariationLimit);
            result.message =
                L"yaw_accel code=" + CodeLabel(code) +
                L" span=" + std::to_wstring(normalizedSpan) +
                L" limit=" + std::to_wstring(kYawAccelerationVariationLimit) +
                L" completed=" + (trace.completed ? L"true" : L"false");
            return result;
        }

        CheckResult EvaluateSmoothYawRateConstancy(const ManeuverCode code)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(code, true);
            const float normalizedSpan = ComputeNormalizedSpan(CollectTurnYawRateMagnitudes(trace));
            CheckResult result{};
            result.passed = trace.started && trace.completed && (normalizedSpan < kYawRateVariationLimit);
            result.message =
                L"yaw_rate code=" + CodeLabel(code) +
                L" span=" + std::to_wstring(normalizedSpan) +
                L" limit=" + std::to_wstring(kYawRateVariationLimit) +
                L" completed=" + (trace.completed ? L"true" : L"false");
            return result;
        }

        CheckResult EvaluateSmoothFinalPosition(const ManeuverCode code)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(code, true);
            const float positionErrorMeters =
                std::hypot(
                    trace.truthState(VehicleState::kPx) - BuildNominalEndXMeters(code),
                    trace.truthState(VehicleState::kPy) - BuildNominalEndYMeters(code));
            CheckResult result{};
            result.passed = trace.started && trace.completed && (positionErrorMeters <= kSmoothPositionToleranceM);
            result.message =
                L"position code=" + CodeLabel(code) +
                L" error_m=" + std::to_wstring(positionErrorMeters) +
                L" limit_m=" + std::to_wstring(kSmoothPositionToleranceM) +
                L" completed=" + (trace.completed ? L"true" : L"false");
            return result;
        }

        CheckResult EvaluateSmoothFinalHeading(const ManeuverCode code)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(code, true);
            const float headingErrorRad =
                std::fabs(AngleErrorRad(BuildNominalEndYawRad(code), trace.truthState(VehicleState::kPsi)));
            CheckResult result{};
            result.passed = trace.started && trace.completed && (headingErrorRad <= kHeadingToleranceRad);
            result.message =
                L"heading code=" + CodeLabel(code) +
                L" error_deg=" + std::to_wstring(headingErrorRad * RAD_TO_DEG_F) +
                L" limit_deg=" + std::to_wstring(kHeadingToleranceRad * RAD_TO_DEG_F) +
                L" completed=" + (trace.completed ? L"true" : L"false");
            return result;
        }
    }

#define DRIVE_IN_PLACE_CONTRACT_TESTS(NAME, CODE) \
    TEST_METHOD(NAME##_ShiftAcceptable) \
    { \
        const CheckResult result = EvaluateInPlaceShift(CODE); \
        Assert::IsTrue(result.passed, result.message.c_str()); \
    } \
    TEST_METHOD(NAME##_HeadingAcceptable) \
    { \
        const CheckResult result = EvaluateInPlaceHeading(CODE); \
        Assert::IsTrue(result.passed, result.message.c_str()); \
    } \
    TEST_METHOD(NAME##_TimeAcceptable) \
    { \
        const CheckResult result = EvaluateInPlaceTime(CODE); \
        Assert::IsTrue(result.passed, result.message.c_str()); \
    }

#define DRIVE_SMOOTH_CONTRACT_TESTS(NAME, CODE) \
    TEST_METHOD(NAME##_VelocityVariationAcceptable) \
    { \
        const CheckResult result = EvaluateSmoothVelocityConstancy(CODE); \
        Assert::IsTrue(result.passed, result.message.c_str()); \
    } \
    TEST_METHOD(NAME##_YawAccelerationVariationAcceptable) \
    { \
        const CheckResult result = EvaluateSmoothYawAccelerationConstancy(CODE); \
        Assert::IsTrue(result.passed, result.message.c_str()); \
    } \
    TEST_METHOD(NAME##_YawRateVariationAcceptable) \
    { \
        const CheckResult result = EvaluateSmoothYawRateConstancy(CODE); \
        Assert::IsTrue(result.passed, result.message.c_str()); \
    } \
    TEST_METHOD(NAME##_FinalPositionAcceptable) \
    { \
        const CheckResult result = EvaluateSmoothFinalPosition(CODE); \
        Assert::IsTrue(result.passed, result.message.c_str()); \
    } \
    TEST_METHOD(NAME##_FinalHeadingAcceptable) \
    { \
        const CheckResult result = EvaluateSmoothFinalHeading(CODE); \
        Assert::IsTrue(result.passed, result.message.c_str()); \
    }

    TEST_CLASS(DriveManeuverTests)
    {
    public:
        DRIVE_IN_PLACE_CONTRACT_TESTS(DriveManeuver_IP45, IP45)
        DRIVE_IN_PLACE_CONTRACT_TESTS(DriveManeuver_IP90, IP90)
        DRIVE_IN_PLACE_CONTRACT_TESTS(DriveManeuver_IP135, IP135)
        DRIVE_IN_PLACE_CONTRACT_TESTS(DriveManeuver_IP180, IP180)

        DRIVE_SMOOTH_CONTRACT_TESTS(DriveManeuver_S45LS, S45LS)
        DRIVE_SMOOTH_CONTRACT_TESTS(DriveManeuver_S45LD, S45LD)
        DRIVE_SMOOTH_CONTRACT_TESTS(DriveManeuver_S45SS, S45SS)
        DRIVE_SMOOTH_CONTRACT_TESTS(DriveManeuver_S45SD, S45SD)
        DRIVE_SMOOTH_CONTRACT_TESTS(DriveManeuver_S90LS, S90LS)
        DRIVE_SMOOTH_CONTRACT_TESTS(DriveManeuver_S90SS, S90SS)
        DRIVE_SMOOTH_CONTRACT_TESTS(DriveManeuver_S90SD, S90SD)
        DRIVE_SMOOTH_CONTRACT_TESTS(DriveManeuver_S135LS, S135LS)
        DRIVE_SMOOTH_CONTRACT_TESTS(DriveManeuver_S135LD, S135LD)
        DRIVE_SMOOTH_CONTRACT_TESTS(DriveManeuver_S135SS, S135SS)
        DRIVE_SMOOTH_CONTRACT_TESTS(DriveManeuver_S135SD, S135SD)
        DRIVE_SMOOTH_CONTRACT_TESTS(DriveManeuver_S180LS, S180LS)
        DRIVE_SMOOTH_CONTRACT_TESTS(DriveManeuver_S180SS, S180SS)
    };

#undef DRIVE_SMOOTH_CONTRACT_TESTS
#undef DRIVE_IN_PLACE_CONTRACT_TESTS
}




