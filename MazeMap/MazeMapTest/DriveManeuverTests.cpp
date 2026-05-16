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
            bool started = false;
            bool completed = false;
            bool allReturnedCommandsFinite = true;
            bool commandEvidenceMatchesReturnedCommand = true;
            bool bodyObjectivesFinite = true;
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
            snapshot.leftEncoderTotalCounts =
                runtimeState.GetSensorSnapshot().leftEncoderTotalCounts +
                static_cast<std::int64_t>(leftCounts);
            snapshot.rightEncoderTotalCounts =
                runtimeState.GetSensorSnapshot().rightEncoderTotalCounts +
                static_cast<std::int64_t>(rightCounts);
            snapshot.leftEncoderDistanceM =
                MazeMap::Vehicle::DriveEncoderDistanceFromCounts(snapshot.leftEncoderTotalCounts);
            snapshot.rightEncoderDistanceM =
                MazeMap::Vehicle::DriveEncoderDistanceFromCounts(snapshot.rightEncoderTotalCounts);
            UpdateDriveEstimator(
                estimator,
                runtimeState,
                dtSeconds,
                snapshot,
                appliedControl);
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
            const float dtSeconds,
            const CommandVector& control)
        {
            const PlantParams params = PlantParams::Default();

            const VehicleState::StateVector previousTruthState = truthState;
            truthState = plant.integrate(truthState, control, dtSeconds, params);

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
                runtime.Estimator(),
                runtime.RuntimeState(),
                leftDistanceDeltaM,
                rightDistanceDeltaM,
                truthState(VehicleState::kR),
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
            PlantModel& plant = runtime.Plant();
            float leftEncoderRemainderCounts = 0.0f;
            float rightEncoderRemainderCounts = 0.0f;

            runtime.DriveBase().ClearCommandEvidence();

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

                const DriveTelemetry& telemetry = runtime.DriveBase().LastTelemetry();
                trace.allReturnedCommandsFinite =
                    trace.allReturnedCommandsFinite && control.IsFinite();
                trace.commandEvidenceMatchesReturnedCommand =
                    trace.commandEvidenceMatchesReturnedCommand &&
                    ((telemetry.commandKindFlags & DriveTelemetry::kCommandKindBodyProposal) != 0U) &&
                    ((telemetry.telemetryValidFlags & DriveTelemetry::kTelemetryCommandEvidenceValid) != 0U) &&
                    (std::fabs(control.LeftCommand() - telemetry.leftDriveCommand) <= 1.0e-6f) &&
                    (std::fabs(control.RightCommand() - telemetry.rightDriveCommand) <= 1.0e-6f);
                trace.bodyObjectivesFinite =
                    trace.bodyObjectivesFinite &&
                    std::isfinite(telemetry.requestedForwardMps) &&
                    std::isfinite(telemetry.requestedYawRateRadps) &&
                    std::isfinite(telemetry.requestedForwardAccelMps2) &&
                    std::isfinite(telemetry.requestedYawAccelRadps2) &&
                    std::isfinite(telemetry.requestedYawRad);

                trace.samples.push_back(
                    CommandSample{
                        trace.elapsedSeconds,
                        telemetry.requestedForwardMps,
                        telemetry.requestedYawRateRadps
                    });

                SimulateRuntimeDriveCycle(
                    runtime,
                    plant,
                    trace.truthState,
                    leftEncoderRemainderCounts,
                    rightEncoderRemainderCounts,
                    kSimulationDtSeconds,
                    control);
                trace.elapsedSeconds += kSimulationDtSeconds;
            }

            return trace;
        }

        CheckResult EvaluateManeuverCompletes(const ManeuverCode code, const bool smoothTurn)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(code, smoothTurn);
            CheckResult result{};
            result.passed = trace.started && trace.completed && !trace.samples.empty();
            result.message =
                L"completion code=" + CodeLabel(code) +
                L" started=" + (trace.started ? L"true" : L"false") +
                L" completed=" + (trace.completed ? L"true" : L"false") +
                L" samples=" + std::to_wstring(trace.samples.size()) +
                L" elapsed_s=" + std::to_wstring(trace.elapsedSeconds);
            return result;
        }

        CheckResult EvaluateManeuverCommandEvidence(const ManeuverCode code, const bool smoothTurn)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(code, smoothTurn);
            CheckResult result{};
            result.passed =
                trace.started &&
                trace.completed &&
                !trace.samples.empty() &&
                trace.allReturnedCommandsFinite &&
                trace.commandEvidenceMatchesReturnedCommand &&
                trace.bodyObjectivesFinite &&
                trace.truthState.allFinite();
            result.message =
                L"command evidence code=" + CodeLabel(code) +
                L" completed=" + (trace.completed ? L"true" : L"false") +
                L" finite_commands=" + (trace.allReturnedCommandsFinite ? L"true" : L"false") +
                L" evidence_matches=" + (trace.commandEvidenceMatchesReturnedCommand ? L"true" : L"false") +
                L" finite_objectives=" + (trace.bodyObjectivesFinite ? L"true" : L"false") +
                L" truth_finite=" + (trace.truthState.allFinite() ? L"true" : L"false");
            return result;
        }
    }

#define DRIVE_IN_PLACE_CONTRACT_TESTS(NAME, CODE) \
    TEST_METHOD(NAME##_Completes) \
    { \
        const CheckResult result = EvaluateManeuverCompletes(CODE, false); \
        Assert::IsTrue(result.passed, result.message.c_str()); \
    } \
    TEST_METHOD(NAME##_CommandEvidenceMatchesReturnedCommand) \
    { \
        const CheckResult result = EvaluateManeuverCommandEvidence(CODE, false); \
        Assert::IsTrue(result.passed, result.message.c_str()); \
    }

#define DRIVE_SMOOTH_CONTRACT_TESTS(NAME, CODE) \
    TEST_METHOD(NAME##_Completes) \
    { \
        const CheckResult result = EvaluateManeuverCompletes(CODE, true); \
        Assert::IsTrue(result.passed, result.message.c_str()); \
    } \
    TEST_METHOD(NAME##_CommandEvidenceMatchesReturnedCommand) \
    { \
        const CheckResult result = EvaluateManeuverCommandEvidence(CODE, true); \
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




