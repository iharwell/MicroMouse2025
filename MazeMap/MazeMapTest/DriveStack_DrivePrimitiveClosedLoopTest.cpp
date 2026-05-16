#include "pch.h"
#include "CppUnitTest.h"

#include "..\MazeMap\CoreConfig.h"
#include "..\MazeMap\Drive.h"
#include "..\MazeMap\DriveBase.h"
#include "..\MazeMap\DriveTelemetry.h"
#include "..\MazeMap\MotionLimits.h"
#include "..\MazeMap\PlantModel.h"
#include "..\MazeMap\SensorSnapshot.h"
#include "..\MazeMap\SharedRobotRuntime.h"
#include "..\MazeMap\Vehicle.h"
#include "..\MazeMap\VehicleState.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap::App
{
    namespace
    {
        using CommandVector = Internal::CommandVector;

        constexpr float kPrimitiveDtSeconds = 0.001f;
        constexpr int kStraightMaxTicks = 6000;
        constexpr int kTurnMaxTicks = 6000;
        constexpr int kCurvedMaxTicks = 4000;
        constexpr float kRollingEntrySpeedMps = 0.30f;
        constexpr float kStraightDistanceM = 0.30f;
        constexpr float kStraightCruiseMps = 0.30f;
        constexpr float kTransitionDistanceM = 0.30f;
        constexpr float kTransitionCurvatureRatePerM = 5.0f;
        constexpr float kArcDistanceM = 0.35f;
        constexpr float kArcCurvaturePerM = 2.0f;

        MotionLimits MakePrimitiveLimits() noexcept
        {
            MotionLimits limits{};
            limits.SetMaxSpeedMps(0.35f);
            limits.SetAccelMps2(3.0f);
            limits.SetDecelMps2(3.0f);
            limits.SetMaxAngularSpeedRadps(6.0f);
            limits.SetAngularAccelRadps2(30.0f);
            limits.SetAngleToleranceRad(Config::kAngleToleranceRad);
            return limits;
        }

        struct ScopedFanDuty final
        {
            ScopedFanDuty(Vehicle& vehicle, const float duty) noexcept
                : target(vehicle)
                , previous(vehicle.GetFanDuty())
            {
                target.SetFanDuty(duty);
            }

            ~ScopedFanDuty() noexcept
            {
                target.SetFanDuty(previous);
            }

            Vehicle& target;
            float previous = 0.0f;
        };

        struct WheelObservationState final
        {
            float leftDistanceM = 0.0f;
            float rightDistanceM = 0.0f;
        };

        struct PrimitiveTrace final
        {
            bool completed = false;
            bool allControlsFinite = true;
            bool truthFinite = true;
            bool commandEvidenceValid = true;
            bool requestedObjectivesFinite = true;
            bool solverClean = true;
            int appliedTicks = 0;
            float elapsedSeconds = 0.0f;
            float minX = 0.0f;
            float maxX = 0.0f;
            float minY = 0.0f;
            float maxY = 0.0f;
            float minYawRad = 0.0f;
            float maxYawRad = 0.0f;
            float minRequestedForwardMps = (std::numeric_limits<float>::infinity)();
            float maxRequestedForwardMps = -(std::numeric_limits<float>::infinity)();
            float minRequestedYawRateRadps = (std::numeric_limits<float>::infinity)();
            float maxRequestedYawRateRadps = -(std::numeric_limits<float>::infinity)();
            WheelObservationState wheels{};
            DriveTelemetry lastTelemetry{};
            VehicleState::StateVector truth = VehicleState::StateVector::Zero();
        };

        VehicleState::StateVector BuildTruthState(
            const float forwardMps,
            const float yawRad,
            const PlantParams& params) noexcept
        {
            VehicleState::StateVector state = VehicleState::StateVector::Zero();
            state(VehicleState::kPsi) = yawRad;
            state(VehicleState::kU) = forwardMps;
            state(VehicleState::kOmegaL) = forwardMps / params.wheelRadiusM;
            state(VehicleState::kOmegaR) = forwardMps / params.wheelRadiusM;
            VehicleState::NormalizeStateVector(state);
            return state;
        }

        void PublishTruthToRuntime(
            Internal::SharedRobotRuntime& runtime,
            const VehicleState::StateVector& truth,
            const WheelObservationState& wheels,
            const float leftDistanceDeltaM,
            const float rightDistanceDeltaM,
            const PlantParams& params)
        {
            SensorSnapshot snapshot{};
            snapshot.gyroRawRadps = truth(VehicleState::kR);
            snapshot.gyroRadps = truth(VehicleState::kR);
            snapshot.encoderObservationValid = true;
            snapshot.leftEncoderDistanceM = wheels.leftDistanceM;
            snapshot.rightEncoderDistanceM = wheels.rightDistanceM;
            snapshot.encoderObservation.leftDistanceDeltaM = leftDistanceDeltaM;
            snapshot.encoderObservation.rightDistanceDeltaM = rightDistanceDeltaM;
            snapshot.encoderObservation.leftVelocityMps = truth(VehicleState::kOmegaL) * params.wheelRadiusM;
            snapshot.encoderObservation.rightVelocityMps = truth(VehicleState::kOmegaR) * params.wheelRadiusM;
            snapshot.encoderObservation.omegaLeftRadps = truth(VehicleState::kOmegaL);
            snapshot.encoderObservation.omegaRightRadps = truth(VehicleState::kOmegaR);

            VehicleState& state = runtime.RuntimeState();
            state.SetPosition(Eigen::Vector2f(truth(VehicleState::kPx), truth(VehicleState::kPy)));
            state.SetOrientation(truth(VehicleState::kPsi));
            state.SetVelocity(truth(VehicleState::kU));
            state.SetLateralVelocity(truth(VehicleState::kV));
            state.SetRotationalVelocity(truth(VehicleState::kR));
            state.SetWheelSpeedLeft(truth(VehicleState::kOmegaL));
            state.SetWheelSpeedRight(truth(VehicleState::kOmegaR));
            state.SetGyroBiasZ(truth(VehicleState::kBgz));
            state.SetSensorSnapshot(snapshot);
        }

        void RecordTraceState(PrimitiveTrace& trace) noexcept
        {
            trace.truthFinite = trace.truthFinite && trace.truth.allFinite();
            trace.minX = (std::min)(trace.minX, trace.truth(VehicleState::kPx));
            trace.maxX = (std::max)(trace.maxX, trace.truth(VehicleState::kPx));
            trace.minY = (std::min)(trace.minY, trace.truth(VehicleState::kPy));
            trace.maxY = (std::max)(trace.maxY, trace.truth(VehicleState::kPy));
            trace.minYawRad = (std::min)(trace.minYawRad, trace.truth(VehicleState::kPsi));
            trace.maxYawRad = (std::max)(trace.maxYawRad, trace.truth(VehicleState::kPsi));
        }

        void RecordTelemetry(
            PrimitiveTrace& trace,
            const CommandVector& control,
            const DriveTelemetry& telemetry) noexcept
        {
            trace.allControlsFinite = trace.allControlsFinite && control.IsFinite();
            trace.commandEvidenceValid =
                trace.commandEvidenceValid &&
                ((telemetry.commandKindFlags & DriveTelemetry::kCommandKindBodyProposal) != 0U) &&
                ((telemetry.telemetryValidFlags & DriveTelemetry::kTelemetryCommandEvidenceValid) != 0U) &&
                ((telemetry.telemetryValidFlags & DriveTelemetry::kTelemetryPlantCommandValid) != 0U) &&
                (std::fabs(control.LeftCommand() - telemetry.leftDriveCommand) <= 1.0e-5f) &&
                (std::fabs(control.RightCommand() - telemetry.rightDriveCommand) <= 1.0e-5f);
            trace.requestedObjectivesFinite =
                trace.requestedObjectivesFinite &&
                std::isfinite(telemetry.requestedForwardMps) &&
                std::isfinite(telemetry.requestedYawRateRadps) &&
                std::isfinite(telemetry.requestedForwardAccelMps2) &&
                std::isfinite(telemetry.requestedYawAccelRadps2) &&
                std::isfinite(telemetry.requestedYawRad);
            trace.solverClean = trace.solverClean && (telemetry.solverFailureFlags == 0U);
            trace.minRequestedForwardMps =
                (std::min)(trace.minRequestedForwardMps, telemetry.requestedForwardMps);
            trace.maxRequestedForwardMps =
                (std::max)(trace.maxRequestedForwardMps, telemetry.requestedForwardMps);
            trace.minRequestedYawRateRadps =
                (std::min)(trace.minRequestedYawRateRadps, telemetry.requestedYawRateRadps);
            trace.maxRequestedYawRateRadps =
                (std::max)(trace.maxRequestedYawRateRadps, telemetry.requestedYawRateRadps);
            trace.lastTelemetry = telemetry;
        }

        void AdvancePlantTruth(
            Internal::SharedRobotRuntime& runtime,
            PrimitiveTrace& trace,
            const CommandVector& control,
            const PlantParams& params)
        {
            const VehicleState::StateVector previous = trace.truth;
            trace.truth = runtime.Plant().integrate(previous, control, kPrimitiveDtSeconds, params);

            const float leftDeltaM =
                0.5f *
                (previous(VehicleState::kOmegaL) + trace.truth(VehicleState::kOmegaL)) *
                params.wheelRadiusM *
                kPrimitiveDtSeconds;
            const float rightDeltaM =
                0.5f *
                (previous(VehicleState::kOmegaR) + trace.truth(VehicleState::kOmegaR)) *
                params.wheelRadiusM *
                kPrimitiveDtSeconds;
            trace.wheels.leftDistanceM += leftDeltaM;
            trace.wheels.rightDistanceM += rightDeltaM;
            PublishTruthToRuntime(runtime, trace.truth, trace.wheels, leftDeltaM, rightDeltaM, params);
            ++trace.appliedTicks;
            trace.elapsedSeconds = static_cast<float>(trace.appliedTicks) * kPrimitiveDtSeconds;
            RecordTraceState(trace);
        }

        template <typename ArmPrimitive>
        PrimitiveTrace SimulatePrimitive(
            const float initialForwardMps,
            const int maxTicks,
            ArmPrimitive armPrimitive)
        {
            const PlantParams params = PlantParams::Default();
            PrimitiveTrace trace{};
            Internal::SharedRobotRuntime runtime(kPrimitiveDtSeconds);
            ScopedFanDuty fanDuty(runtime.Vehicle(), 0.80f);

            trace.truth = BuildTruthState(initialForwardMps, 0.0f, params);
            PublishTruthToRuntime(runtime, trace.truth, trace.wheels, 0.0f, 0.0f, params);
            runtime.DriveBase().ClearCommandEvidence();

            Internal::Drive& drive = runtime.DriveService();
            drive.SetOperationMode(Internal::Drive::OperationMode::OpenFloor);
            drive.SetLimits(MakePrimitiveLimits());
            armPrimitive(drive);

            for (int tick = 0; tick < maxTicks; ++tick)
            {
                bool done = false;
                const CommandVector control = drive.GetNextControls(done);
                if (done)
                {
                    trace.completed = true;
                    break;
                }

                RecordTelemetry(trace, control, runtime.DriveBase().LastTelemetry());
                AdvancePlantTruth(runtime, trace, control, params);
            }

            return trace;
        }

        float AverageEncoderDistanceM(const PrimitiveTrace& trace) noexcept
        {
            return 0.5f * (trace.wheels.leftDistanceM + trace.wheels.rightDistanceM);
        }

        float AbsAngleErrorRad(const float expectedRad, const float actualRad) noexcept
        {
            return std::fabs(VehicleState::NormalizeAngle(expectedRad - actualRad));
        }

        std::wstring TraceMessage(
            const wchar_t* label,
            const wchar_t* primitive,
            const wchar_t* field,
            const PrimitiveTrace& trace,
            const float expected,
            const float actual,
            const float limit)
        {
            return
                std::wstring(L"DRV50_PRIMITIVE_CLOSED_LOOP ") + label +
                L" primitive=" + primitive +
                L" field=" + field +
                L" expected=" + std::to_wstring(expected) +
                L" actual=" + std::to_wstring(actual) +
                L" limit=" + std::to_wstring(limit) +
                L" completed=" + (trace.completed ? L"true" : L"false") +
                L" ticks=" + std::to_wstring(trace.appliedTicks) +
                L" elapsed_s=" + std::to_wstring(trace.elapsedSeconds) +
                L" x_m=" + std::to_wstring(trace.truth(VehicleState::kPx)) +
                L" y_m=" + std::to_wstring(trace.truth(VehicleState::kPy)) +
                L" yaw_deg=" + std::to_wstring(trace.truth(VehicleState::kPsi) * RAD_TO_DEG_F) +
                L" encoder_m=" + std::to_wstring(AverageEncoderDistanceM(trace));
        }

        PrimitiveTrace RunStartStraight()
        {
            return SimulatePrimitive(
                0.0f,
                kStraightMaxTicks,
                [](Internal::Drive& drive)
            {
                drive.StartStraight(kStraightDistanceM, kStraightCruiseMps, 0.0f);
            });
        }

        PrimitiveTrace RunStartTurn()
        {
            return SimulatePrimitive(
                0.0f,
                kTurnMaxTicks,
                [](Internal::Drive& drive)
            {
                drive.StartTurn(HALF_PI_F);
            });
        }

        PrimitiveTrace RunStartTurnTransition()
        {
            return SimulatePrimitive(
                kRollingEntrySpeedMps,
                kCurvedMaxTicks,
                [](Internal::Drive& drive)
            {
                drive.StartTurnTransition(kTransitionDistanceM, kTransitionCurvatureRatePerM);
            });
        }

        PrimitiveTrace RunStartArc()
        {
            return SimulatePrimitive(
                kRollingEntrySpeedMps,
                kCurvedMaxTicks,
                [](Internal::Drive& drive)
            {
                drive.StartArc(kArcDistanceM, kArcCurvaturePerM);
            });
        }
    }

    TEST_CLASS(DriveStack_DrivePrimitiveClosedLoopTest)
    {
    public:
        TEST_METHOD(StartStraight_Completes)
        {
            const PrimitiveTrace trace = RunStartStraight();
            Assert::IsTrue(trace.completed, TraceMessage(L"DRV50_STRAIGHT", L"StartStraight", L"completed", trace, 1.0f, 0.0f, 0.0f).c_str());
        }

        TEST_METHOD(StartStraight_UsesMultiTickHorizon)
        {
            const PrimitiveTrace trace = RunStartStraight();
            Assert::IsTrue(trace.appliedTicks >= 20, TraceMessage(L"DRV50_STRAIGHT", L"StartStraight", L"multi_tick_horizon", trace, 20.0f, static_cast<float>(trace.appliedTicks), 0.0f).c_str());
        }

        TEST_METHOD(StartStraight_ControlsStayFinite)
        {
            const PrimitiveTrace trace = RunStartStraight();
            Assert::IsTrue(trace.allControlsFinite, TraceMessage(L"DRV50_STRAIGHT", L"StartStraight", L"finite_controls", trace, 1.0f, 0.0f, 0.0f).c_str());
        }

        TEST_METHOD(StartStraight_TruthStaysFinite)
        {
            const PrimitiveTrace trace = RunStartStraight();
            Assert::IsTrue(trace.truthFinite, TraceMessage(L"DRV50_STRAIGHT", L"StartStraight", L"finite_truth", trace, 1.0f, 0.0f, 0.0f).c_str());
        }

        TEST_METHOD(StartStraight_CommandEvidenceMatchesControls)
        {
            const PrimitiveTrace trace = RunStartStraight();
            Assert::IsTrue(trace.commandEvidenceValid, TraceMessage(L"DRV50_STRAIGHT", L"StartStraight", L"command_evidence", trace, 1.0f, 0.0f, 0.0f).c_str());
        }

        TEST_METHOD(StartStraight_RequestedObjectivesStayFinite)
        {
            const PrimitiveTrace trace = RunStartStraight();
            Assert::IsTrue(trace.requestedObjectivesFinite, TraceMessage(L"DRV50_STRAIGHT", L"StartStraight", L"finite_objectives", trace, 1.0f, 0.0f, 0.0f).c_str());
        }

        TEST_METHOD(StartStraight_SolverFlagsStayClear)
        {
            const PrimitiveTrace trace = RunStartStraight();
            Assert::IsTrue(trace.solverClean, TraceMessage(L"DRV50_STRAIGHT", L"StartStraight", L"solver_flags", trace, 0.0f, static_cast<float>(trace.lastTelemetry.solverFailureFlags), 0.0f).c_str());
        }

        TEST_METHOD(StartStraight_DoesNotMoveBackwardY)
        {
            const PrimitiveTrace trace = RunStartStraight();
            Assert::IsTrue(
                trace.minY >= -0.010f,
                TraceMessage(L"DRV50_STRAIGHT", L"StartStraight", L"wrong_way_y", trace, 0.0f, trace.minY, 0.010f).c_str());
        }

        TEST_METHOD(StartStraight_FinalYMatchesDistance)
        {
            const PrimitiveTrace trace = RunStartStraight();
            Assert::AreEqual(
                kStraightDistanceM,
                trace.truth(VehicleState::kPy),
                0.080f,
                TraceMessage(L"DRV50_STRAIGHT", L"StartStraight", L"final_y_m", trace, kStraightDistanceM, trace.truth(VehicleState::kPy), 0.080f).c_str());
        }

        TEST_METHOD(StartStraight_FinalXStaysCentered)
        {
            const PrimitiveTrace trace = RunStartStraight();
            Assert::AreEqual(
                0.0f,
                trace.truth(VehicleState::kPx),
                0.030f,
                TraceMessage(L"DRV50_STRAIGHT", L"StartStraight", L"final_x_m", trace, 0.0f, trace.truth(VehicleState::kPx), 0.030f).c_str());
        }

        TEST_METHOD(StartStraight_FinalYawStaysZero)
        {
            const PrimitiveTrace trace = RunStartStraight();
            Assert::AreEqual(
                0.0f,
                trace.truth(VehicleState::kPsi),
                0.080f,
                TraceMessage(L"DRV50_STRAIGHT", L"StartStraight", L"final_yaw_rad", trace, 0.0f, trace.truth(VehicleState::kPsi), 0.080f).c_str());
        }

        TEST_METHOD(StartTurn_Completes)
        {
            const PrimitiveTrace trace = RunStartTurn();
            Assert::IsTrue(trace.completed, TraceMessage(L"DRV50_TURN", L"StartTurn", L"completed", trace, 1.0f, 0.0f, 0.0f).c_str());
        }

        TEST_METHOD(StartTurn_UsesMultiTickHorizon)
        {
            const PrimitiveTrace trace = RunStartTurn();
            Assert::IsTrue(trace.appliedTicks >= 20, TraceMessage(L"DRV50_TURN", L"StartTurn", L"multi_tick_horizon", trace, 20.0f, static_cast<float>(trace.appliedTicks), 0.0f).c_str());
        }

        TEST_METHOD(StartTurn_ControlsStayFinite)
        {
            const PrimitiveTrace trace = RunStartTurn();
            Assert::IsTrue(trace.allControlsFinite, TraceMessage(L"DRV50_TURN", L"StartTurn", L"finite_controls", trace, 1.0f, 0.0f, 0.0f).c_str());
        }

        TEST_METHOD(StartTurn_TruthStaysFinite)
        {
            const PrimitiveTrace trace = RunStartTurn();
            Assert::IsTrue(trace.truthFinite, TraceMessage(L"DRV50_TURN", L"StartTurn", L"finite_truth", trace, 1.0f, 0.0f, 0.0f).c_str());
        }

        TEST_METHOD(StartTurn_CommandEvidenceMatchesControls)
        {
            const PrimitiveTrace trace = RunStartTurn();
            Assert::IsTrue(trace.commandEvidenceValid, TraceMessage(L"DRV50_TURN", L"StartTurn", L"command_evidence", trace, 1.0f, 0.0f, 0.0f).c_str());
        }

        TEST_METHOD(StartTurn_RequestedObjectivesStayFinite)
        {
            const PrimitiveTrace trace = RunStartTurn();
            Assert::IsTrue(trace.requestedObjectivesFinite, TraceMessage(L"DRV50_TURN", L"StartTurn", L"finite_objectives", trace, 1.0f, 0.0f, 0.0f).c_str());
        }

        TEST_METHOD(StartTurn_SolverFlagsStayClear)
        {
            const PrimitiveTrace trace = RunStartTurn();
            Assert::IsTrue(trace.solverClean, TraceMessage(L"DRV50_TURN", L"StartTurn", L"solver_flags", trace, 0.0f, static_cast<float>(trace.lastTelemetry.solverFailureFlags), 0.0f).c_str());
        }

        TEST_METHOD(StartTurn_BuildsClockwiseYaw)
        {
            const PrimitiveTrace trace = RunStartTurn();
            Assert::IsTrue(
                trace.maxYawRad > 0.50f,
                TraceMessage(L"DRV50_TURN", L"StartTurn", L"clockwise_yaw_progress", trace, 0.50f, trace.maxYawRad, 0.0f).c_str());
        }

        TEST_METHOD(StartTurn_DoesNotYawWrongWay)
        {
            const PrimitiveTrace trace = RunStartTurn();
            Assert::IsTrue(
                trace.minYawRad >= -0.030f,
                TraceMessage(L"DRV50_TURN", L"StartTurn", L"wrong_way_yaw", trace, 0.0f, trace.minYawRad, 0.030f).c_str());
        }

        TEST_METHOD(StartTurn_FinalHeadingMatchesRequest)
        {
            const PrimitiveTrace trace = RunStartTurn();
            const float headingErrorRad = AbsAngleErrorRad(HALF_PI_F, trace.truth(VehicleState::kPsi));
            Assert::AreEqual(
                0.0f,
                headingErrorRad,
                0.120f,
                TraceMessage(L"DRV50_TURN", L"StartTurn", L"heading_error_rad", trace, 0.0f, headingErrorRad, 0.120f).c_str());
        }

        TEST_METHOD(StartTurn_PositionShiftStaysBounded)
        {
            const PrimitiveTrace trace = RunStartTurn();
            const float shiftM = std::hypot(trace.truth(VehicleState::kPx), trace.truth(VehicleState::kPy));
            Assert::AreEqual(
                0.0f,
                shiftM,
                0.080f,
                TraceMessage(L"DRV50_TURN", L"StartTurn", L"position_shift_m", trace, 0.0f, shiftM, 0.080f).c_str());
        }

        TEST_METHOD(StartTurnTransition_Completes)
        {
            const PrimitiveTrace trace = RunStartTurnTransition();
            Assert::IsTrue(trace.completed, TraceMessage(L"DRV50_TRANSITION", L"StartTurnTransition", L"completed", trace, 1.0f, 0.0f, 0.0f).c_str());
        }

        TEST_METHOD(StartTurnTransition_UsesMultiTickHorizon)
        {
            const PrimitiveTrace trace = RunStartTurnTransition();
            Assert::IsTrue(trace.appliedTicks >= 20, TraceMessage(L"DRV50_TRANSITION", L"StartTurnTransition", L"multi_tick_horizon", trace, 20.0f, static_cast<float>(trace.appliedTicks), 0.0f).c_str());
        }

        TEST_METHOD(StartTurnTransition_ControlsStayFinite)
        {
            const PrimitiveTrace trace = RunStartTurnTransition();
            Assert::IsTrue(trace.allControlsFinite, TraceMessage(L"DRV50_TRANSITION", L"StartTurnTransition", L"finite_controls", trace, 1.0f, 0.0f, 0.0f).c_str());
        }

        TEST_METHOD(StartTurnTransition_TruthStaysFinite)
        {
            const PrimitiveTrace trace = RunStartTurnTransition();
            Assert::IsTrue(trace.truthFinite, TraceMessage(L"DRV50_TRANSITION", L"StartTurnTransition", L"finite_truth", trace, 1.0f, 0.0f, 0.0f).c_str());
        }

        TEST_METHOD(StartTurnTransition_CommandEvidenceMatchesControls)
        {
            const PrimitiveTrace trace = RunStartTurnTransition();
            Assert::IsTrue(trace.commandEvidenceValid, TraceMessage(L"DRV50_TRANSITION", L"StartTurnTransition", L"command_evidence", trace, 1.0f, 0.0f, 0.0f).c_str());
        }

        TEST_METHOD(StartTurnTransition_RequestedObjectivesStayFinite)
        {
            const PrimitiveTrace trace = RunStartTurnTransition();
            Assert::IsTrue(trace.requestedObjectivesFinite, TraceMessage(L"DRV50_TRANSITION", L"StartTurnTransition", L"finite_objectives", trace, 1.0f, 0.0f, 0.0f).c_str());
        }

        TEST_METHOD(StartTurnTransition_SolverFlagsStayClear)
        {
            const PrimitiveTrace trace = RunStartTurnTransition();
            Assert::IsTrue(trace.solverClean, TraceMessage(L"DRV50_TRANSITION", L"StartTurnTransition", L"solver_flags", trace, 0.0f, static_cast<float>(trace.lastTelemetry.solverFailureFlags), 0.0f).c_str());
        }

        TEST_METHOD(StartTurnTransition_RequestsPositiveYawRate)
        {
            const PrimitiveTrace trace = RunStartTurnTransition();
            Assert::IsTrue(
                trace.maxRequestedYawRateRadps > 0.05f,
                TraceMessage(L"DRV50_TRANSITION", L"StartTurnTransition", L"positive_yaw_rate_request", trace, 0.05f, trace.maxRequestedYawRateRadps, 0.0f).c_str());
        }

        TEST_METHOD(StartTurnTransition_DoesNotRequestWrongWayYawRate)
        {
            const PrimitiveTrace trace = RunStartTurnTransition();
            Assert::IsTrue(
                trace.minRequestedYawRateRadps >= -0.020f,
                TraceMessage(L"DRV50_TRANSITION", L"StartTurnTransition", L"wrong_way_yaw_rate_request", trace, 0.0f, trace.minRequestedYawRateRadps, 0.020f).c_str());
        }

        TEST_METHOD(StartTurnTransition_EncoderDistanceMatchesRequest)
        {
            const PrimitiveTrace trace = RunStartTurnTransition();
            Assert::AreEqual(
                kTransitionDistanceM,
                AverageEncoderDistanceM(trace),
                0.050f,
                TraceMessage(L"DRV50_TRANSITION", L"StartTurnTransition", L"encoder_distance_m", trace, kTransitionDistanceM, AverageEncoderDistanceM(trace), 0.050f).c_str());
        }

        TEST_METHOD(StartTurnTransition_FinalHeadingMatchesIntegratedCurvature)
        {
            const PrimitiveTrace trace = RunStartTurnTransition();
            const float expectedYawRad =
                0.5f * kTransitionCurvatureRatePerM * kTransitionDistanceM * kTransitionDistanceM;
            const float headingErrorRad = AbsAngleErrorRad(expectedYawRad, trace.truth(VehicleState::kPsi));
            Assert::AreEqual(
                0.0f,
                headingErrorRad,
                0.200f,
                TraceMessage(L"DRV50_TRANSITION", L"StartTurnTransition", L"heading_error_rad", trace, 0.0f, headingErrorRad, 0.200f).c_str());
        }

        TEST_METHOD(StartTurnTransition_MakesForwardProgress)
        {
            const PrimitiveTrace trace = RunStartTurnTransition();
            Assert::IsTrue(
                trace.truth(VehicleState::kPy) > 0.20f,
                TraceMessage(L"DRV50_TRANSITION", L"StartTurnTransition", L"forward_progress_y_m", trace, 0.20f, trace.truth(VehicleState::kPy), 0.0f).c_str());
        }

        TEST_METHOD(StartArc_Completes)
        {
            const PrimitiveTrace trace = RunStartArc();
            Assert::IsTrue(trace.completed, TraceMessage(L"DRV50_ARC", L"StartArc", L"completed", trace, 1.0f, 0.0f, 0.0f).c_str());
        }

        TEST_METHOD(StartArc_UsesMultiTickHorizon)
        {
            const PrimitiveTrace trace = RunStartArc();
            Assert::IsTrue(trace.appliedTicks >= 20, TraceMessage(L"DRV50_ARC", L"StartArc", L"multi_tick_horizon", trace, 20.0f, static_cast<float>(trace.appliedTicks), 0.0f).c_str());
        }

        TEST_METHOD(StartArc_ControlsStayFinite)
        {
            const PrimitiveTrace trace = RunStartArc();
            Assert::IsTrue(trace.allControlsFinite, TraceMessage(L"DRV50_ARC", L"StartArc", L"finite_controls", trace, 1.0f, 0.0f, 0.0f).c_str());
        }

        TEST_METHOD(StartArc_TruthStaysFinite)
        {
            const PrimitiveTrace trace = RunStartArc();
            Assert::IsTrue(trace.truthFinite, TraceMessage(L"DRV50_ARC", L"StartArc", L"finite_truth", trace, 1.0f, 0.0f, 0.0f).c_str());
        }

        TEST_METHOD(StartArc_CommandEvidenceMatchesControls)
        {
            const PrimitiveTrace trace = RunStartArc();
            Assert::IsTrue(trace.commandEvidenceValid, TraceMessage(L"DRV50_ARC", L"StartArc", L"command_evidence", trace, 1.0f, 0.0f, 0.0f).c_str());
        }

        TEST_METHOD(StartArc_RequestedObjectivesStayFinite)
        {
            const PrimitiveTrace trace = RunStartArc();
            Assert::IsTrue(trace.requestedObjectivesFinite, TraceMessage(L"DRV50_ARC", L"StartArc", L"finite_objectives", trace, 1.0f, 0.0f, 0.0f).c_str());
        }

        TEST_METHOD(StartArc_SolverFlagsStayClear)
        {
            const PrimitiveTrace trace = RunStartArc();
            Assert::IsTrue(trace.solverClean, TraceMessage(L"DRV50_ARC", L"StartArc", L"solver_flags", trace, 0.0f, static_cast<float>(trace.lastTelemetry.solverFailureFlags), 0.0f).c_str());
        }

        TEST_METHOD(StartArc_RequestsPositiveYawRate)
        {
            const PrimitiveTrace trace = RunStartArc();
            Assert::IsTrue(
                trace.maxRequestedYawRateRadps > 0.10f,
                TraceMessage(L"DRV50_ARC", L"StartArc", L"positive_yaw_rate_request", trace, 0.10f, trace.maxRequestedYawRateRadps, 0.0f).c_str());
        }

        TEST_METHOD(StartArc_DoesNotYawWrongWay)
        {
            const PrimitiveTrace trace = RunStartArc();
            Assert::IsTrue(
                trace.minYawRad >= -0.030f,
                TraceMessage(L"DRV50_ARC", L"StartArc", L"wrong_way_yaw", trace, 0.0f, trace.minYawRad, 0.030f).c_str());
        }

        TEST_METHOD(StartArc_EncoderDistanceMatchesRequest)
        {
            const PrimitiveTrace trace = RunStartArc();
            Assert::AreEqual(
                kArcDistanceM,
                AverageEncoderDistanceM(trace),
                0.060f,
                TraceMessage(L"DRV50_ARC", L"StartArc", L"encoder_distance_m", trace, kArcDistanceM, AverageEncoderDistanceM(trace), 0.060f).c_str());
        }

        TEST_METHOD(StartArc_FinalHeadingMatchesCurvature)
        {
            const PrimitiveTrace trace = RunStartArc();
            const float expectedYawRad = kArcDistanceM * kArcCurvaturePerM;
            const float headingErrorRad = AbsAngleErrorRad(expectedYawRad, trace.truth(VehicleState::kPsi));
            Assert::AreEqual(
                0.0f,
                headingErrorRad,
                0.250f,
                TraceMessage(L"DRV50_ARC", L"StartArc", L"heading_error_rad", trace, 0.0f, headingErrorRad, 0.250f).c_str());
        }

        TEST_METHOD(StartArc_MakesRightwardProgress)
        {
            const PrimitiveTrace trace = RunStartArc();
            Assert::IsTrue(
                trace.truth(VehicleState::kPx) > 0.030f,
                TraceMessage(L"DRV50_ARC", L"StartArc", L"rightward_progress_x_m", trace, 0.030f, trace.truth(VehicleState::kPx), 0.0f).c_str());
        }

        TEST_METHOD(StartArc_MakesForwardProgress)
        {
            const PrimitiveTrace trace = RunStartArc();
            Assert::IsTrue(
                trace.truth(VehicleState::kPy) > 0.20f,
                TraceMessage(L"DRV50_ARC", L"StartArc", L"forward_progress_y_m", trace, 0.20f, trace.truth(VehicleState::kPy), 0.0f).c_str());
        }
    };
}
