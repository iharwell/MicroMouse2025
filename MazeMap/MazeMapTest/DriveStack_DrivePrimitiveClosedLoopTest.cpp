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
#include <sstream>
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
            VehicleState truth;
        };

        VehicleState BuildTruthState(
            const float forwardMps,
            const float yawRad) noexcept
        {
            VehicleState state;
            state.SetOrientation(yawRad);
            state.SetVelocity(forwardMps);
            state.SetWheelSpeedLeft(Vehicle::WheelOmegaFromLinearVelocity(forwardMps));
            state.SetWheelSpeedRight(Vehicle::WheelOmegaFromLinearVelocity(forwardMps));
            return state;
        }

        bool IsVehicleStateFinite(const VehicleState& state) noexcept
        {
            return
                std::isfinite(state.GetPositionX()) &&
                std::isfinite(state.GetPositionY()) &&
                std::isfinite(state.GetOrientation()) &&
                std::isfinite(state.GetVelocity()) &&
                std::isfinite(state.GetLateralVelocity()) &&
                std::isfinite(state.GetRotationalVelocity()) &&
                std::isfinite(state.GetWheelSpeedLeft()) &&
                std::isfinite(state.GetWheelSpeedRight()) &&
                std::isfinite(state.GetGyroBiasZ());
        }

        void PublishTruthToRuntime(
            Internal::SharedRobotRuntime& runtime,
            const VehicleState& truth,
            const WheelObservationState& wheels,
            const float leftDistanceDeltaM,
            const float rightDistanceDeltaM)
        {
            SensorSnapshot snapshot{};
            snapshot.gyroRawRadps = truth.GetRotationalVelocity();
            snapshot.gyroRadps = truth.GetRotationalVelocity();
            snapshot.encoderObservationValid = true;
            snapshot.leftEncoderDistanceM = wheels.leftDistanceM;
            snapshot.rightEncoderDistanceM = wheels.rightDistanceM;
            snapshot.encoderObservation.leftDistanceDeltaM = leftDistanceDeltaM;
            snapshot.encoderObservation.rightDistanceDeltaM = rightDistanceDeltaM;
            snapshot.encoderObservation.leftVelocityMps =
                Vehicle::WheelLinearVelocityFromOmega(truth.GetWheelSpeedLeft());
            snapshot.encoderObservation.rightVelocityMps =
                Vehicle::WheelLinearVelocityFromOmega(truth.GetWheelSpeedRight());
            snapshot.encoderObservation.omegaLeftRadps = truth.GetWheelSpeedLeft();
            snapshot.encoderObservation.omegaRightRadps = truth.GetWheelSpeedRight();

            VehicleState& state = runtime.RuntimeState();
            state.SetPosition(truth.GetPosition());
            state.SetOrientation(truth.GetOrientation());
            state.SetVelocity(truth.GetVelocity());
            state.SetLateralVelocity(truth.GetLateralVelocity());
            state.SetRotationalVelocity(truth.GetRotationalVelocity());
            state.SetWheelSpeedLeft(truth.GetWheelSpeedLeft());
            state.SetWheelSpeedRight(truth.GetWheelSpeedRight());
            state.SetGyroBiasZ(truth.GetGyroBiasZ());
            state.SetSensorSnapshot(snapshot);
        }

        void RecordTraceState(PrimitiveTrace& trace) noexcept
        {
            trace.truthFinite = trace.truthFinite && IsVehicleStateFinite(trace.truth);
            trace.minX = (std::min)(trace.minX, trace.truth.GetPositionX());
            trace.maxX = (std::max)(trace.maxX, trace.truth.GetPositionX());
            trace.minY = (std::min)(trace.minY, trace.truth.GetPositionY());
            trace.maxY = (std::max)(trace.maxY, trace.truth.GetPositionY());
            trace.minYawRad = (std::min)(trace.minYawRad, trace.truth.GetOrientation());
            trace.maxYawRad = (std::max)(trace.maxYawRad, trace.truth.GetOrientation());
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
            trace.solverClean = trace.solverClean;
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
            const CommandVector& control)
        {
            const VehicleState previous = trace.truth;
            runtime.Plant().integrate(control, kPrimitiveDtSeconds);
            trace.truth = runtime.RuntimeState();

            const float leftDeltaM =
                0.5f *
                (previous.GetWheelSpeedLeft() + trace.truth.GetWheelSpeedLeft()) *
                Vehicle::GetDriveWheelRadiusM() *
                kPrimitiveDtSeconds;
            const float rightDeltaM =
                0.5f *
                (previous.GetWheelSpeedRight() + trace.truth.GetWheelSpeedRight()) *
                Vehicle::GetDriveWheelRadiusM() *
                kPrimitiveDtSeconds;
            trace.wheels.leftDistanceM += leftDeltaM;
            trace.wheels.rightDistanceM += rightDeltaM;
            PublishTruthToRuntime(runtime, trace.truth, trace.wheels, leftDeltaM, rightDeltaM);
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
            PrimitiveTrace trace{};
            Internal::SharedRobotRuntime runtime(kPrimitiveDtSeconds);
            ScopedFanDuty fanDuty(runtime.Vehicle(), 0.80f);

            trace.truth = BuildTruthState(initialForwardMps, 0.0f);
            PublishTruthToRuntime(runtime, trace.truth, trace.wheels, 0.0f, 0.0f);
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
                AdvancePlantTruth(runtime, trace, control);
            }

            return trace;
        }

        float AverageEncoderDistanceM(const PrimitiveTrace& trace) noexcept
        {
            return 0.5f * (trace.wheels.leftDistanceM + trace.wheels.rightDistanceM);
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
            Assert::IsTrue(
                trace.completed,
                ([&]()
                {
                    std::wstringstream message;
                    message << L"DRV50_PRIMITIVE_CLOSED_LOOP"
                        << L"\nlabel=" << L"DRV50_STRAIGHT"
                        << L"\nprimitive=" << L"StartStraight"
                        << L"\nfield=" << L"completed"
                        << L"\nexpected=" << 1.0f
                        << L"\nactual=" << 0.0f
                        << L"\nlimit=" << 0.0f
                        << L"\ncompleted=" << (trace.completed ? L"true" : L"false")
                        << L"\nall_controls_finite=" << (trace.allControlsFinite ? L"true" : L"false")
                        << L"\ntruth_finite=" << (trace.truthFinite ? L"true" : L"false")
                        << L"\ncommand_evidence_valid=" << (trace.commandEvidenceValid ? L"true" : L"false")
                        << L"\nrequested_objectives_finite=" << (trace.requestedObjectivesFinite ? L"true" : L"false")
                        << L"\nsolver_clean=" << (trace.solverClean ? L"true" : L"false")
                        << L"\nticks=" << trace.appliedTicks
                        << L"\nelapsed_s=" << trace.elapsedSeconds
                        << L"\nx_m=" << trace.truth.GetPositionX()
                        << L"\ny_m=" << trace.truth.GetPositionY()
                        << L"\nyaw_deg=" << (trace.truth.GetOrientation() * RAD_TO_DEG_F)
                        << L"\nencoder_m=" << AverageEncoderDistanceM(trace)
                        << L"\nmin_y_m=" << trace.minY
                        << L"\nmin_yaw_rad=" << trace.minYawRad
                        << L"\nmax_yaw_rad=" << trace.maxYawRad
                        << L"\nmin_requested_yaw_rate_radps=" << trace.minRequestedYawRateRadps
                        << L"\nmax_requested_yaw_rate_radps=" << trace.maxRequestedYawRateRadps;
                    return message.str();
                }()).c_str());
        }

        TEST_METHOD(StartStraight_UsesMultiTickHorizon)
        {
            const PrimitiveTrace trace = RunStartStraight();
            Assert::IsTrue(
                trace.appliedTicks >= 20,
                ([&]()
                {
                    std::wstringstream message;
                    message << L"DRV50_PRIMITIVE_CLOSED_LOOP"
                        << L"\nlabel=" << L"DRV50_STRAIGHT"
                        << L"\nprimitive=" << L"StartStraight"
                        << L"\nfield=" << L"multi_tick_horizon"
                        << L"\nexpected=" << 20.0f
                        << L"\nactual=" << static_cast<float>(trace.appliedTicks)
                        << L"\nlimit=" << 0.0f
                        << L"\ncompleted=" << (trace.completed ? L"true" : L"false")
                        << L"\nall_controls_finite=" << (trace.allControlsFinite ? L"true" : L"false")
                        << L"\ntruth_finite=" << (trace.truthFinite ? L"true" : L"false")
                        << L"\ncommand_evidence_valid=" << (trace.commandEvidenceValid ? L"true" : L"false")
                        << L"\nrequested_objectives_finite=" << (trace.requestedObjectivesFinite ? L"true" : L"false")
                        << L"\nsolver_clean=" << (trace.solverClean ? L"true" : L"false")
                        << L"\nticks=" << trace.appliedTicks
                        << L"\nelapsed_s=" << trace.elapsedSeconds
                        << L"\nx_m=" << trace.truth.GetPositionX()
                        << L"\ny_m=" << trace.truth.GetPositionY()
                        << L"\nyaw_deg=" << (trace.truth.GetOrientation() * RAD_TO_DEG_F)
                        << L"\nencoder_m=" << AverageEncoderDistanceM(trace)
                        << L"\nmin_y_m=" << trace.minY
                        << L"\nmin_yaw_rad=" << trace.minYawRad
                        << L"\nmax_yaw_rad=" << trace.maxYawRad
                        << L"\nmin_requested_yaw_rate_radps=" << trace.minRequestedYawRateRadps
                        << L"\nmax_requested_yaw_rate_radps=" << trace.maxRequestedYawRateRadps;
                    return message.str();
                }()).c_str());
        }

        TEST_METHOD(StartStraight_ControlsStayFinite)
        {
            const PrimitiveTrace trace = RunStartStraight();
            Assert::IsTrue(
                trace.allControlsFinite,
                ([&]()
                {
                    std::wstringstream message;
                    message << L"DRV50_PRIMITIVE_CLOSED_LOOP"
                        << L"\nlabel=" << L"DRV50_STRAIGHT"
                        << L"\nprimitive=" << L"StartStraight"
                        << L"\nfield=" << L"finite_controls"
                        << L"\nexpected=" << 1.0f
                        << L"\nactual=" << 0.0f
                        << L"\nlimit=" << 0.0f
                        << L"\ncompleted=" << (trace.completed ? L"true" : L"false")
                        << L"\nall_controls_finite=" << (trace.allControlsFinite ? L"true" : L"false")
                        << L"\ntruth_finite=" << (trace.truthFinite ? L"true" : L"false")
                        << L"\ncommand_evidence_valid=" << (trace.commandEvidenceValid ? L"true" : L"false")
                        << L"\nrequested_objectives_finite=" << (trace.requestedObjectivesFinite ? L"true" : L"false")
                        << L"\nsolver_clean=" << (trace.solverClean ? L"true" : L"false")
                        << L"\nticks=" << trace.appliedTicks
                        << L"\nelapsed_s=" << trace.elapsedSeconds
                        << L"\nx_m=" << trace.truth.GetPositionX()
                        << L"\ny_m=" << trace.truth.GetPositionY()
                        << L"\nyaw_deg=" << (trace.truth.GetOrientation() * RAD_TO_DEG_F)
                        << L"\nencoder_m=" << AverageEncoderDistanceM(trace)
                        << L"\nmin_y_m=" << trace.minY
                        << L"\nmin_yaw_rad=" << trace.minYawRad
                        << L"\nmax_yaw_rad=" << trace.maxYawRad
                        << L"\nmin_requested_yaw_rate_radps=" << trace.minRequestedYawRateRadps
                        << L"\nmax_requested_yaw_rate_radps=" << trace.maxRequestedYawRateRadps;
                    return message.str();
                }()).c_str());
        }

        TEST_METHOD(StartStraight_TruthStaysFinite)
        {
            const PrimitiveTrace trace = RunStartStraight();
            Assert::IsTrue(
                trace.truthFinite,
                ([&]()
                {
                    std::wstringstream message;
                    message << L"DRV50_PRIMITIVE_CLOSED_LOOP"
                        << L"\nlabel=" << L"DRV50_STRAIGHT"
                        << L"\nprimitive=" << L"StartStraight"
                        << L"\nfield=" << L"finite_truth"
                        << L"\nexpected=" << 1.0f
                        << L"\nactual=" << 0.0f
                        << L"\nlimit=" << 0.0f
                        << L"\ncompleted=" << (trace.completed ? L"true" : L"false")
                        << L"\nall_controls_finite=" << (trace.allControlsFinite ? L"true" : L"false")
                        << L"\ntruth_finite=" << (trace.truthFinite ? L"true" : L"false")
                        << L"\ncommand_evidence_valid=" << (trace.commandEvidenceValid ? L"true" : L"false")
                        << L"\nrequested_objectives_finite=" << (trace.requestedObjectivesFinite ? L"true" : L"false")
                        << L"\nsolver_clean=" << (trace.solverClean ? L"true" : L"false")
                        << L"\nticks=" << trace.appliedTicks
                        << L"\nelapsed_s=" << trace.elapsedSeconds
                        << L"\nx_m=" << trace.truth.GetPositionX()
                        << L"\ny_m=" << trace.truth.GetPositionY()
                        << L"\nyaw_deg=" << (trace.truth.GetOrientation() * RAD_TO_DEG_F)
                        << L"\nencoder_m=" << AverageEncoderDistanceM(trace)
                        << L"\nmin_y_m=" << trace.minY
                        << L"\nmin_yaw_rad=" << trace.minYawRad
                        << L"\nmax_yaw_rad=" << trace.maxYawRad
                        << L"\nmin_requested_yaw_rate_radps=" << trace.minRequestedYawRateRadps
                        << L"\nmax_requested_yaw_rate_radps=" << trace.maxRequestedYawRateRadps;
                    return message.str();
                }()).c_str());
        }

        TEST_METHOD(StartStraight_CommandEvidenceMatchesControls)
        {
            const PrimitiveTrace trace = RunStartStraight();
            Assert::IsTrue(
                trace.commandEvidenceValid,
                ([&]()
                {
                    std::wstringstream message;
                    message << L"DRV50_PRIMITIVE_CLOSED_LOOP"
                        << L"\nlabel=" << L"DRV50_STRAIGHT"
                        << L"\nprimitive=" << L"StartStraight"
                        << L"\nfield=" << L"command_evidence"
                        << L"\nexpected=" << 1.0f
                        << L"\nactual=" << 0.0f
                        << L"\nlimit=" << 0.0f
                        << L"\ncompleted=" << (trace.completed ? L"true" : L"false")
                        << L"\nall_controls_finite=" << (trace.allControlsFinite ? L"true" : L"false")
                        << L"\ntruth_finite=" << (trace.truthFinite ? L"true" : L"false")
                        << L"\ncommand_evidence_valid=" << (trace.commandEvidenceValid ? L"true" : L"false")
                        << L"\nrequested_objectives_finite=" << (trace.requestedObjectivesFinite ? L"true" : L"false")
                        << L"\nsolver_clean=" << (trace.solverClean ? L"true" : L"false")
                        << L"\nticks=" << trace.appliedTicks
                        << L"\nelapsed_s=" << trace.elapsedSeconds
                        << L"\nx_m=" << trace.truth.GetPositionX()
                        << L"\ny_m=" << trace.truth.GetPositionY()
                        << L"\nyaw_deg=" << (trace.truth.GetOrientation() * RAD_TO_DEG_F)
                        << L"\nencoder_m=" << AverageEncoderDistanceM(trace)
                        << L"\nmin_y_m=" << trace.minY
                        << L"\nmin_yaw_rad=" << trace.minYawRad
                        << L"\nmax_yaw_rad=" << trace.maxYawRad
                        << L"\nmin_requested_yaw_rate_radps=" << trace.minRequestedYawRateRadps
                        << L"\nmax_requested_yaw_rate_radps=" << trace.maxRequestedYawRateRadps;
                    return message.str();
                }()).c_str());
        }

        TEST_METHOD(StartStraight_RequestedObjectivesStayFinite)
        {
            const PrimitiveTrace trace = RunStartStraight();
            Assert::IsTrue(
                trace.requestedObjectivesFinite,
                ([&]()
                {
                    std::wstringstream message;
                    message << L"DRV50_PRIMITIVE_CLOSED_LOOP"
                        << L"\nlabel=" << L"DRV50_STRAIGHT"
                        << L"\nprimitive=" << L"StartStraight"
                        << L"\nfield=" << L"finite_objectives"
                        << L"\nexpected=" << 1.0f
                        << L"\nactual=" << 0.0f
                        << L"\nlimit=" << 0.0f
                        << L"\ncompleted=" << (trace.completed ? L"true" : L"false")
                        << L"\nall_controls_finite=" << (trace.allControlsFinite ? L"true" : L"false")
                        << L"\ntruth_finite=" << (trace.truthFinite ? L"true" : L"false")
                        << L"\ncommand_evidence_valid=" << (trace.commandEvidenceValid ? L"true" : L"false")
                        << L"\nrequested_objectives_finite=" << (trace.requestedObjectivesFinite ? L"true" : L"false")
                        << L"\nsolver_clean=" << (trace.solverClean ? L"true" : L"false")
                        << L"\nticks=" << trace.appliedTicks
                        << L"\nelapsed_s=" << trace.elapsedSeconds
                        << L"\nx_m=" << trace.truth.GetPositionX()
                        << L"\ny_m=" << trace.truth.GetPositionY()
                        << L"\nyaw_deg=" << (trace.truth.GetOrientation() * RAD_TO_DEG_F)
                        << L"\nencoder_m=" << AverageEncoderDistanceM(trace)
                        << L"\nmin_y_m=" << trace.minY
                        << L"\nmin_yaw_rad=" << trace.minYawRad
                        << L"\nmax_yaw_rad=" << trace.maxYawRad
                        << L"\nmin_requested_yaw_rate_radps=" << trace.minRequestedYawRateRadps
                        << L"\nmax_requested_yaw_rate_radps=" << trace.maxRequestedYawRateRadps;
                    return message.str();
                }()).c_str());
        }

        TEST_METHOD(StartStraight_SolverFlagsStayClear)
        {
            const PrimitiveTrace trace = RunStartStraight();
            Assert::IsTrue(
                trace.solverClean,
                ([&]()
                {
                    std::wstringstream message;
                    message << L"DRV50_PRIMITIVE_CLOSED_LOOP"
                        << L"\nlabel=" << L"DRV50_STRAIGHT"
                        << L"\nprimitive=" << L"StartStraight"
                        << L"\nfield=" << L"solver_flags"
                        << L"\nexpected=" << 0.0f
                        << L"\nlimit=" << 0.0f
                        << L"\ncompleted=" << (trace.completed ? L"true" : L"false")
                        << L"\nall_controls_finite=" << (trace.allControlsFinite ? L"true" : L"false")
                        << L"\ntruth_finite=" << (trace.truthFinite ? L"true" : L"false")
                        << L"\ncommand_evidence_valid=" << (trace.commandEvidenceValid ? L"true" : L"false")
                        << L"\nrequested_objectives_finite=" << (trace.requestedObjectivesFinite ? L"true" : L"false")
                        << L"\nsolver_clean=" << (trace.solverClean ? L"true" : L"false")
                        << L"\nticks=" << trace.appliedTicks
                        << L"\nelapsed_s=" << trace.elapsedSeconds
                        << L"\nx_m=" << trace.truth.GetPositionX()
                        << L"\ny_m=" << trace.truth.GetPositionY()
                        << L"\nyaw_deg=" << (trace.truth.GetOrientation() * RAD_TO_DEG_F)
                        << L"\nencoder_m=" << AverageEncoderDistanceM(trace)
                        << L"\nmin_y_m=" << trace.minY
                        << L"\nmin_yaw_rad=" << trace.minYawRad
                        << L"\nmax_yaw_rad=" << trace.maxYawRad
                        << L"\nmin_requested_yaw_rate_radps=" << trace.minRequestedYawRateRadps
                        << L"\nmax_requested_yaw_rate_radps=" << trace.maxRequestedYawRateRadps;
                    return message.str();
                }()).c_str());
        }

        TEST_METHOD(StartStraight_DoesNotMoveBackwardY)
        {
            const PrimitiveTrace trace = RunStartStraight();
            Assert::IsTrue(
                trace.minY >= -0.010f,
                ([&]()
                {
                    std::wstringstream message;
                    message << L"DRV50_PRIMITIVE_CLOSED_LOOP"
                        << L"\nlabel=" << L"DRV50_STRAIGHT"
                        << L"\nprimitive=" << L"StartStraight"
                        << L"\nfield=" << L"wrong_way_y"
                        << L"\nexpected=" << 0.0f
                        << L"\nactual=" << trace.minY
                        << L"\nlimit=" << 0.010f
                        << L"\ncompleted=" << (trace.completed ? L"true" : L"false")
                        << L"\nall_controls_finite=" << (trace.allControlsFinite ? L"true" : L"false")
                        << L"\ntruth_finite=" << (trace.truthFinite ? L"true" : L"false")
                        << L"\ncommand_evidence_valid=" << (trace.commandEvidenceValid ? L"true" : L"false")
                        << L"\nrequested_objectives_finite=" << (trace.requestedObjectivesFinite ? L"true" : L"false")
                        << L"\nsolver_clean=" << (trace.solverClean ? L"true" : L"false")
                        << L"\nticks=" << trace.appliedTicks
                        << L"\nelapsed_s=" << trace.elapsedSeconds
                        << L"\nx_m=" << trace.truth.GetPositionX()
                        << L"\ny_m=" << trace.truth.GetPositionY()
                        << L"\nyaw_deg=" << (trace.truth.GetOrientation() * RAD_TO_DEG_F)
                        << L"\nencoder_m=" << AverageEncoderDistanceM(trace)
                        << L"\nmin_y_m=" << trace.minY
                        << L"\nmin_yaw_rad=" << trace.minYawRad
                        << L"\nmax_yaw_rad=" << trace.maxYawRad
                        << L"\nmin_requested_yaw_rate_radps=" << trace.minRequestedYawRateRadps
                        << L"\nmax_requested_yaw_rate_radps=" << trace.maxRequestedYawRateRadps;
                    return message.str();
                }()).c_str());
        }

        TEST_METHOD(StartStraight_FinalYMatchesDistance)
        {
            const PrimitiveTrace trace = RunStartStraight();
            Assert::AreEqual(
                kStraightDistanceM,
                trace.truth.GetPositionY(),
                0.080f,
                ([&]()
                {
                    std::wstringstream message;
                    message << L"DRV50_PRIMITIVE_CLOSED_LOOP"
                        << L"\nlabel=" << L"DRV50_STRAIGHT"
                        << L"\nprimitive=" << L"StartStraight"
                        << L"\nfield=" << L"final_y_m"
                        << L"\nexpected=" << kStraightDistanceM
                        << L"\nactual=" << trace.truth.GetPositionY()
                        << L"\nlimit=" << 0.080f
                        << L"\ncompleted=" << (trace.completed ? L"true" : L"false")
                        << L"\nall_controls_finite=" << (trace.allControlsFinite ? L"true" : L"false")
                        << L"\ntruth_finite=" << (trace.truthFinite ? L"true" : L"false")
                        << L"\ncommand_evidence_valid=" << (trace.commandEvidenceValid ? L"true" : L"false")
                        << L"\nrequested_objectives_finite=" << (trace.requestedObjectivesFinite ? L"true" : L"false")
                        << L"\nsolver_clean=" << (trace.solverClean ? L"true" : L"false")
                        << L"\nticks=" << trace.appliedTicks
                        << L"\nelapsed_s=" << trace.elapsedSeconds
                        << L"\nx_m=" << trace.truth.GetPositionX()
                        << L"\ny_m=" << trace.truth.GetPositionY()
                        << L"\nyaw_deg=" << (trace.truth.GetOrientation() * RAD_TO_DEG_F)
                        << L"\nencoder_m=" << AverageEncoderDistanceM(trace)
                        << L"\nmin_y_m=" << trace.minY
                        << L"\nmin_yaw_rad=" << trace.minYawRad
                        << L"\nmax_yaw_rad=" << trace.maxYawRad
                        << L"\nmin_requested_yaw_rate_radps=" << trace.minRequestedYawRateRadps
                        << L"\nmax_requested_yaw_rate_radps=" << trace.maxRequestedYawRateRadps;
                    return message.str();
                }()).c_str());
        }

        TEST_METHOD(StartStraight_FinalXStaysCentered)
        {
            const PrimitiveTrace trace = RunStartStraight();
            Assert::AreEqual(
                0.0f,
                trace.truth.GetPositionX(),
                0.030f,
                ([&]()
                {
                    std::wstringstream message;
                    message << L"DRV50_PRIMITIVE_CLOSED_LOOP"
                        << L"\nlabel=" << L"DRV50_STRAIGHT"
                        << L"\nprimitive=" << L"StartStraight"
                        << L"\nfield=" << L"final_x_m"
                        << L"\nexpected=" << 0.0f
                        << L"\nactual=" << trace.truth.GetPositionX()
                        << L"\nlimit=" << 0.030f
                        << L"\ncompleted=" << (trace.completed ? L"true" : L"false")
                        << L"\nall_controls_finite=" << (trace.allControlsFinite ? L"true" : L"false")
                        << L"\ntruth_finite=" << (trace.truthFinite ? L"true" : L"false")
                        << L"\ncommand_evidence_valid=" << (trace.commandEvidenceValid ? L"true" : L"false")
                        << L"\nrequested_objectives_finite=" << (trace.requestedObjectivesFinite ? L"true" : L"false")
                        << L"\nsolver_clean=" << (trace.solverClean ? L"true" : L"false")
                        << L"\nticks=" << trace.appliedTicks
                        << L"\nelapsed_s=" << trace.elapsedSeconds
                        << L"\nx_m=" << trace.truth.GetPositionX()
                        << L"\ny_m=" << trace.truth.GetPositionY()
                        << L"\nyaw_deg=" << (trace.truth.GetOrientation() * RAD_TO_DEG_F)
                        << L"\nencoder_m=" << AverageEncoderDistanceM(trace)
                        << L"\nmin_y_m=" << trace.minY
                        << L"\nmin_yaw_rad=" << trace.minYawRad
                        << L"\nmax_yaw_rad=" << trace.maxYawRad
                        << L"\nmin_requested_yaw_rate_radps=" << trace.minRequestedYawRateRadps
                        << L"\nmax_requested_yaw_rate_radps=" << trace.maxRequestedYawRateRadps;
                    return message.str();
                }()).c_str());
        }

        TEST_METHOD(StartStraight_FinalYawStaysZero)
        {
            const PrimitiveTrace trace = RunStartStraight();
            Assert::AreEqual(
                0.0f,
                trace.truth.GetOrientation(),
                0.080f,
                ([&]()
                {
                    std::wstringstream message;
                    message << L"DRV50_PRIMITIVE_CLOSED_LOOP"
                        << L"\nlabel=" << L"DRV50_STRAIGHT"
                        << L"\nprimitive=" << L"StartStraight"
                        << L"\nfield=" << L"final_yaw_rad"
                        << L"\nexpected=" << 0.0f
                        << L"\nactual=" << trace.truth.GetOrientation()
                        << L"\nlimit=" << 0.080f
                        << L"\ncompleted=" << (trace.completed ? L"true" : L"false")
                        << L"\nall_controls_finite=" << (trace.allControlsFinite ? L"true" : L"false")
                        << L"\ntruth_finite=" << (trace.truthFinite ? L"true" : L"false")
                        << L"\ncommand_evidence_valid=" << (trace.commandEvidenceValid ? L"true" : L"false")
                        << L"\nrequested_objectives_finite=" << (trace.requestedObjectivesFinite ? L"true" : L"false")
                        << L"\nsolver_clean=" << (trace.solverClean ? L"true" : L"false")
                        << L"\nticks=" << trace.appliedTicks
                        << L"\nelapsed_s=" << trace.elapsedSeconds
                        << L"\nx_m=" << trace.truth.GetPositionX()
                        << L"\ny_m=" << trace.truth.GetPositionY()
                        << L"\nyaw_deg=" << (trace.truth.GetOrientation() * RAD_TO_DEG_F)
                        << L"\nencoder_m=" << AverageEncoderDistanceM(trace)
                        << L"\nmin_y_m=" << trace.minY
                        << L"\nmin_yaw_rad=" << trace.minYawRad
                        << L"\nmax_yaw_rad=" << trace.maxYawRad
                        << L"\nmin_requested_yaw_rate_radps=" << trace.minRequestedYawRateRadps
                        << L"\nmax_requested_yaw_rate_radps=" << trace.maxRequestedYawRateRadps;
                    return message.str();
                }()).c_str());
        }

        TEST_METHOD(StartTurn_Completes)
        {
            const PrimitiveTrace trace = RunStartTurn();
            Assert::IsTrue(
                trace.completed,
                ([&]()
                {
                    std::wstringstream message;
                    message << L"DRV50_PRIMITIVE_CLOSED_LOOP"
                        << L"\nlabel=" << L"DRV50_TURN"
                        << L"\nprimitive=" << L"StartTurn"
                        << L"\nfield=" << L"completed"
                        << L"\nexpected=" << 1.0f
                        << L"\nactual=" << 0.0f
                        << L"\nlimit=" << 0.0f
                        << L"\ncompleted=" << (trace.completed ? L"true" : L"false")
                        << L"\nall_controls_finite=" << (trace.allControlsFinite ? L"true" : L"false")
                        << L"\ntruth_finite=" << (trace.truthFinite ? L"true" : L"false")
                        << L"\ncommand_evidence_valid=" << (trace.commandEvidenceValid ? L"true" : L"false")
                        << L"\nrequested_objectives_finite=" << (trace.requestedObjectivesFinite ? L"true" : L"false")
                        << L"\nsolver_clean=" << (trace.solverClean ? L"true" : L"false")
                        << L"\nticks=" << trace.appliedTicks
                        << L"\nelapsed_s=" << trace.elapsedSeconds
                        << L"\nx_m=" << trace.truth.GetPositionX()
                        << L"\ny_m=" << trace.truth.GetPositionY()
                        << L"\nyaw_deg=" << (trace.truth.GetOrientation() * RAD_TO_DEG_F)
                        << L"\nencoder_m=" << AverageEncoderDistanceM(trace)
                        << L"\nmin_y_m=" << trace.minY
                        << L"\nmin_yaw_rad=" << trace.minYawRad
                        << L"\nmax_yaw_rad=" << trace.maxYawRad
                        << L"\nmin_requested_yaw_rate_radps=" << trace.minRequestedYawRateRadps
                        << L"\nmax_requested_yaw_rate_radps=" << trace.maxRequestedYawRateRadps;
                    return message.str();
                }()).c_str());
        }

        TEST_METHOD(StartTurn_UsesMultiTickHorizon)
        {
            const PrimitiveTrace trace = RunStartTurn();
            Assert::IsTrue(
                trace.appliedTicks >= 20,
                ([&]()
                {
                    std::wstringstream message;
                    message << L"DRV50_PRIMITIVE_CLOSED_LOOP"
                        << L"\nlabel=" << L"DRV50_TURN"
                        << L"\nprimitive=" << L"StartTurn"
                        << L"\nfield=" << L"multi_tick_horizon"
                        << L"\nexpected=" << 20.0f
                        << L"\nactual=" << static_cast<float>(trace.appliedTicks)
                        << L"\nlimit=" << 0.0f
                        << L"\ncompleted=" << (trace.completed ? L"true" : L"false")
                        << L"\nall_controls_finite=" << (trace.allControlsFinite ? L"true" : L"false")
                        << L"\ntruth_finite=" << (trace.truthFinite ? L"true" : L"false")
                        << L"\ncommand_evidence_valid=" << (trace.commandEvidenceValid ? L"true" : L"false")
                        << L"\nrequested_objectives_finite=" << (trace.requestedObjectivesFinite ? L"true" : L"false")
                        << L"\nsolver_clean=" << (trace.solverClean ? L"true" : L"false")
                        << L"\nticks=" << trace.appliedTicks
                        << L"\nelapsed_s=" << trace.elapsedSeconds
                        << L"\nx_m=" << trace.truth.GetPositionX()
                        << L"\ny_m=" << trace.truth.GetPositionY()
                        << L"\nyaw_deg=" << (trace.truth.GetOrientation() * RAD_TO_DEG_F)
                        << L"\nencoder_m=" << AverageEncoderDistanceM(trace)
                        << L"\nmin_y_m=" << trace.minY
                        << L"\nmin_yaw_rad=" << trace.minYawRad
                        << L"\nmax_yaw_rad=" << trace.maxYawRad
                        << L"\nmin_requested_yaw_rate_radps=" << trace.minRequestedYawRateRadps
                        << L"\nmax_requested_yaw_rate_radps=" << trace.maxRequestedYawRateRadps;
                    return message.str();
                }()).c_str());
        }

        TEST_METHOD(StartTurn_ControlsStayFinite)
        {
            const PrimitiveTrace trace = RunStartTurn();
            Assert::IsTrue(
                trace.allControlsFinite,
                ([&]()
                {
                    std::wstringstream message;
                    message << L"DRV50_PRIMITIVE_CLOSED_LOOP"
                        << L"\nlabel=" << L"DRV50_TURN"
                        << L"\nprimitive=" << L"StartTurn"
                        << L"\nfield=" << L"finite_controls"
                        << L"\nexpected=" << 1.0f
                        << L"\nactual=" << 0.0f
                        << L"\nlimit=" << 0.0f
                        << L"\ncompleted=" << (trace.completed ? L"true" : L"false")
                        << L"\nall_controls_finite=" << (trace.allControlsFinite ? L"true" : L"false")
                        << L"\ntruth_finite=" << (trace.truthFinite ? L"true" : L"false")
                        << L"\ncommand_evidence_valid=" << (trace.commandEvidenceValid ? L"true" : L"false")
                        << L"\nrequested_objectives_finite=" << (trace.requestedObjectivesFinite ? L"true" : L"false")
                        << L"\nsolver_clean=" << (trace.solverClean ? L"true" : L"false")
                        << L"\nticks=" << trace.appliedTicks
                        << L"\nelapsed_s=" << trace.elapsedSeconds
                        << L"\nx_m=" << trace.truth.GetPositionX()
                        << L"\ny_m=" << trace.truth.GetPositionY()
                        << L"\nyaw_deg=" << (trace.truth.GetOrientation() * RAD_TO_DEG_F)
                        << L"\nencoder_m=" << AverageEncoderDistanceM(trace)
                        << L"\nmin_y_m=" << trace.minY
                        << L"\nmin_yaw_rad=" << trace.minYawRad
                        << L"\nmax_yaw_rad=" << trace.maxYawRad
                        << L"\nmin_requested_yaw_rate_radps=" << trace.minRequestedYawRateRadps
                        << L"\nmax_requested_yaw_rate_radps=" << trace.maxRequestedYawRateRadps;
                    return message.str();
                }()).c_str());
        }

        TEST_METHOD(StartTurn_TruthStaysFinite)
        {
            const PrimitiveTrace trace = RunStartTurn();
            Assert::IsTrue(
                trace.truthFinite,
                ([&]()
                {
                    std::wstringstream message;
                    message << L"DRV50_PRIMITIVE_CLOSED_LOOP"
                        << L"\nlabel=" << L"DRV50_TURN"
                        << L"\nprimitive=" << L"StartTurn"
                        << L"\nfield=" << L"finite_truth"
                        << L"\nexpected=" << 1.0f
                        << L"\nactual=" << 0.0f
                        << L"\nlimit=" << 0.0f
                        << L"\ncompleted=" << (trace.completed ? L"true" : L"false")
                        << L"\nall_controls_finite=" << (trace.allControlsFinite ? L"true" : L"false")
                        << L"\ntruth_finite=" << (trace.truthFinite ? L"true" : L"false")
                        << L"\ncommand_evidence_valid=" << (trace.commandEvidenceValid ? L"true" : L"false")
                        << L"\nrequested_objectives_finite=" << (trace.requestedObjectivesFinite ? L"true" : L"false")
                        << L"\nsolver_clean=" << (trace.solverClean ? L"true" : L"false")
                        << L"\nticks=" << trace.appliedTicks
                        << L"\nelapsed_s=" << trace.elapsedSeconds
                        << L"\nx_m=" << trace.truth.GetPositionX()
                        << L"\ny_m=" << trace.truth.GetPositionY()
                        << L"\nyaw_deg=" << (trace.truth.GetOrientation() * RAD_TO_DEG_F)
                        << L"\nencoder_m=" << AverageEncoderDistanceM(trace)
                        << L"\nmin_y_m=" << trace.minY
                        << L"\nmin_yaw_rad=" << trace.minYawRad
                        << L"\nmax_yaw_rad=" << trace.maxYawRad
                        << L"\nmin_requested_yaw_rate_radps=" << trace.minRequestedYawRateRadps
                        << L"\nmax_requested_yaw_rate_radps=" << trace.maxRequestedYawRateRadps;
                    return message.str();
                }()).c_str());
        }

        TEST_METHOD(StartTurn_CommandEvidenceMatchesControls)
        {
            const PrimitiveTrace trace = RunStartTurn();
            Assert::IsTrue(
                trace.commandEvidenceValid,
                ([&]()
                {
                    std::wstringstream message;
                    message << L"DRV50_PRIMITIVE_CLOSED_LOOP"
                        << L"\nlabel=" << L"DRV50_TURN"
                        << L"\nprimitive=" << L"StartTurn"
                        << L"\nfield=" << L"command_evidence"
                        << L"\nexpected=" << 1.0f
                        << L"\nactual=" << 0.0f
                        << L"\nlimit=" << 0.0f
                        << L"\ncompleted=" << (trace.completed ? L"true" : L"false")
                        << L"\nall_controls_finite=" << (trace.allControlsFinite ? L"true" : L"false")
                        << L"\ntruth_finite=" << (trace.truthFinite ? L"true" : L"false")
                        << L"\ncommand_evidence_valid=" << (trace.commandEvidenceValid ? L"true" : L"false")
                        << L"\nrequested_objectives_finite=" << (trace.requestedObjectivesFinite ? L"true" : L"false")
                        << L"\nsolver_clean=" << (trace.solverClean ? L"true" : L"false")
                        << L"\nticks=" << trace.appliedTicks
                        << L"\nelapsed_s=" << trace.elapsedSeconds
                        << L"\nx_m=" << trace.truth.GetPositionX()
                        << L"\ny_m=" << trace.truth.GetPositionY()
                        << L"\nyaw_deg=" << (trace.truth.GetOrientation() * RAD_TO_DEG_F)
                        << L"\nencoder_m=" << AverageEncoderDistanceM(trace)
                        << L"\nmin_y_m=" << trace.minY
                        << L"\nmin_yaw_rad=" << trace.minYawRad
                        << L"\nmax_yaw_rad=" << trace.maxYawRad
                        << L"\nmin_requested_yaw_rate_radps=" << trace.minRequestedYawRateRadps
                        << L"\nmax_requested_yaw_rate_radps=" << trace.maxRequestedYawRateRadps;
                    return message.str();
                }()).c_str());
        }

        TEST_METHOD(StartTurn_RequestedObjectivesStayFinite)
        {
            const PrimitiveTrace trace = RunStartTurn();
            Assert::IsTrue(
                trace.requestedObjectivesFinite,
                ([&]()
                {
                    std::wstringstream message;
                    message << L"DRV50_PRIMITIVE_CLOSED_LOOP"
                        << L"\nlabel=" << L"DRV50_TURN"
                        << L"\nprimitive=" << L"StartTurn"
                        << L"\nfield=" << L"finite_objectives"
                        << L"\nexpected=" << 1.0f
                        << L"\nactual=" << 0.0f
                        << L"\nlimit=" << 0.0f
                        << L"\ncompleted=" << (trace.completed ? L"true" : L"false")
                        << L"\nall_controls_finite=" << (trace.allControlsFinite ? L"true" : L"false")
                        << L"\ntruth_finite=" << (trace.truthFinite ? L"true" : L"false")
                        << L"\ncommand_evidence_valid=" << (trace.commandEvidenceValid ? L"true" : L"false")
                        << L"\nrequested_objectives_finite=" << (trace.requestedObjectivesFinite ? L"true" : L"false")
                        << L"\nsolver_clean=" << (trace.solverClean ? L"true" : L"false")
                        << L"\nticks=" << trace.appliedTicks
                        << L"\nelapsed_s=" << trace.elapsedSeconds
                        << L"\nx_m=" << trace.truth.GetPositionX()
                        << L"\ny_m=" << trace.truth.GetPositionY()
                        << L"\nyaw_deg=" << (trace.truth.GetOrientation() * RAD_TO_DEG_F)
                        << L"\nencoder_m=" << AverageEncoderDistanceM(trace)
                        << L"\nmin_y_m=" << trace.minY
                        << L"\nmin_yaw_rad=" << trace.minYawRad
                        << L"\nmax_yaw_rad=" << trace.maxYawRad
                        << L"\nmin_requested_yaw_rate_radps=" << trace.minRequestedYawRateRadps
                        << L"\nmax_requested_yaw_rate_radps=" << trace.maxRequestedYawRateRadps;
                    return message.str();
                }()).c_str());
        }

        TEST_METHOD(StartTurn_BuildsClockwiseYaw)
        {
            const PrimitiveTrace trace = RunStartTurn();
            Assert::IsTrue(
                trace.maxYawRad > 0.50f,
                ([&]()
                {
                    std::wstringstream message;
                    message << L"DRV50_PRIMITIVE_CLOSED_LOOP"
                        << L"\nlabel=" << L"DRV50_TURN"
                        << L"\nprimitive=" << L"StartTurn"
                        << L"\nfield=" << L"clockwise_yaw_progress"
                        << L"\nexpected=" << 0.50f
                        << L"\nactual=" << trace.maxYawRad
                        << L"\nlimit=" << 0.0f
                        << L"\ncompleted=" << (trace.completed ? L"true" : L"false")
                        << L"\nall_controls_finite=" << (trace.allControlsFinite ? L"true" : L"false")
                        << L"\ntruth_finite=" << (trace.truthFinite ? L"true" : L"false")
                        << L"\ncommand_evidence_valid=" << (trace.commandEvidenceValid ? L"true" : L"false")
                        << L"\nrequested_objectives_finite=" << (trace.requestedObjectivesFinite ? L"true" : L"false")
                        << L"\nsolver_clean=" << (trace.solverClean ? L"true" : L"false")
                        << L"\nticks=" << trace.appliedTicks
                        << L"\nelapsed_s=" << trace.elapsedSeconds
                        << L"\nx_m=" << trace.truth.GetPositionX()
                        << L"\ny_m=" << trace.truth.GetPositionY()
                        << L"\nyaw_deg=" << (trace.truth.GetOrientation() * RAD_TO_DEG_F)
                        << L"\nencoder_m=" << AverageEncoderDistanceM(trace)
                        << L"\nmin_y_m=" << trace.minY
                        << L"\nmin_yaw_rad=" << trace.minYawRad
                        << L"\nmax_yaw_rad=" << trace.maxYawRad
                        << L"\nmin_requested_yaw_rate_radps=" << trace.minRequestedYawRateRadps
                        << L"\nmax_requested_yaw_rate_radps=" << trace.maxRequestedYawRateRadps;
                    return message.str();
                }()).c_str());
        }

        TEST_METHOD(StartTurn_DoesNotYawWrongWay)
        {
            const PrimitiveTrace trace = RunStartTurn();
            Assert::IsTrue(
                trace.minYawRad >= -0.030f,
                ([&]()
                {
                    std::wstringstream message;
                    message << L"DRV50_PRIMITIVE_CLOSED_LOOP"
                        << L"\nlabel=" << L"DRV50_TURN"
                        << L"\nprimitive=" << L"StartTurn"
                        << L"\nfield=" << L"wrong_way_yaw"
                        << L"\nexpected=" << 0.0f
                        << L"\nactual=" << trace.minYawRad
                        << L"\nlimit=" << 0.030f
                        << L"\ncompleted=" << (trace.completed ? L"true" : L"false")
                        << L"\nall_controls_finite=" << (trace.allControlsFinite ? L"true" : L"false")
                        << L"\ntruth_finite=" << (trace.truthFinite ? L"true" : L"false")
                        << L"\ncommand_evidence_valid=" << (trace.commandEvidenceValid ? L"true" : L"false")
                        << L"\nrequested_objectives_finite=" << (trace.requestedObjectivesFinite ? L"true" : L"false")
                        << L"\nsolver_clean=" << (trace.solverClean ? L"true" : L"false")
                        << L"\nticks=" << trace.appliedTicks
                        << L"\nelapsed_s=" << trace.elapsedSeconds
                        << L"\nx_m=" << trace.truth.GetPositionX()
                        << L"\ny_m=" << trace.truth.GetPositionY()
                        << L"\nyaw_deg=" << (trace.truth.GetOrientation() * RAD_TO_DEG_F)
                        << L"\nencoder_m=" << AverageEncoderDistanceM(trace)
                        << L"\nmin_y_m=" << trace.minY
                        << L"\nmin_yaw_rad=" << trace.minYawRad
                        << L"\nmax_yaw_rad=" << trace.maxYawRad
                        << L"\nmin_requested_yaw_rate_radps=" << trace.minRequestedYawRateRadps
                        << L"\nmax_requested_yaw_rate_radps=" << trace.maxRequestedYawRateRadps;
                    return message.str();
                }()).c_str());
        }

        TEST_METHOD(StartTurn_FinalHeadingMatchesRequest)
        {
            const PrimitiveTrace trace = RunStartTurn();
            const float headingErrorRad = std::fabs(AngleDifference(HALF_PI_F, trace.truth.GetOrientation()));
            Assert::AreEqual(
                0.0f,
                headingErrorRad,
                0.120f,
                ([&]()
                {
                    std::wstringstream message;
                    message << L"DRV50_PRIMITIVE_CLOSED_LOOP"
                        << L"\nlabel=" << L"DRV50_TURN"
                        << L"\nprimitive=" << L"StartTurn"
                        << L"\nfield=" << L"heading_error_rad"
                        << L"\nexpected=" << 0.0f
                        << L"\nactual=" << headingErrorRad
                        << L"\nlimit=" << 0.120f
                        << L"\ncompleted=" << (trace.completed ? L"true" : L"false")
                        << L"\nall_controls_finite=" << (trace.allControlsFinite ? L"true" : L"false")
                        << L"\ntruth_finite=" << (trace.truthFinite ? L"true" : L"false")
                        << L"\ncommand_evidence_valid=" << (trace.commandEvidenceValid ? L"true" : L"false")
                        << L"\nrequested_objectives_finite=" << (trace.requestedObjectivesFinite ? L"true" : L"false")
                        << L"\nsolver_clean=" << (trace.solverClean ? L"true" : L"false")
                        << L"\nticks=" << trace.appliedTicks
                        << L"\nelapsed_s=" << trace.elapsedSeconds
                        << L"\nx_m=" << trace.truth.GetPositionX()
                        << L"\ny_m=" << trace.truth.GetPositionY()
                        << L"\nyaw_deg=" << (trace.truth.GetOrientation() * RAD_TO_DEG_F)
                        << L"\nencoder_m=" << AverageEncoderDistanceM(trace)
                        << L"\nmin_y_m=" << trace.minY
                        << L"\nmin_yaw_rad=" << trace.minYawRad
                        << L"\nmax_yaw_rad=" << trace.maxYawRad
                        << L"\nmin_requested_yaw_rate_radps=" << trace.minRequestedYawRateRadps
                        << L"\nmax_requested_yaw_rate_radps=" << trace.maxRequestedYawRateRadps;
                    return message.str();
                }()).c_str());
        }

        TEST_METHOD(StartTurn_PositionShiftStaysBounded)
        {
            const PrimitiveTrace trace = RunStartTurn();
            const float shiftM = std::hypot(trace.truth.GetPositionX(), trace.truth.GetPositionY());
            Assert::AreEqual(
                0.0f,
                shiftM,
                0.080f,
                ([&]()
                {
                    std::wstringstream message;
                    message << L"DRV50_PRIMITIVE_CLOSED_LOOP"
                        << L"\nlabel=" << L"DRV50_TURN"
                        << L"\nprimitive=" << L"StartTurn"
                        << L"\nfield=" << L"position_shift_m"
                        << L"\nexpected=" << 0.0f
                        << L"\nactual=" << shiftM
                        << L"\nlimit=" << 0.080f
                        << L"\ncompleted=" << (trace.completed ? L"true" : L"false")
                        << L"\nall_controls_finite=" << (trace.allControlsFinite ? L"true" : L"false")
                        << L"\ntruth_finite=" << (trace.truthFinite ? L"true" : L"false")
                        << L"\ncommand_evidence_valid=" << (trace.commandEvidenceValid ? L"true" : L"false")
                        << L"\nrequested_objectives_finite=" << (trace.requestedObjectivesFinite ? L"true" : L"false")
                        << L"\nsolver_clean=" << (trace.solverClean ? L"true" : L"false")
                        << L"\nticks=" << trace.appliedTicks
                        << L"\nelapsed_s=" << trace.elapsedSeconds
                        << L"\nx_m=" << trace.truth.GetPositionX()
                        << L"\ny_m=" << trace.truth.GetPositionY()
                        << L"\nyaw_deg=" << (trace.truth.GetOrientation() * RAD_TO_DEG_F)
                        << L"\nencoder_m=" << AverageEncoderDistanceM(trace)
                        << L"\nmin_y_m=" << trace.minY
                        << L"\nmin_yaw_rad=" << trace.minYawRad
                        << L"\nmax_yaw_rad=" << trace.maxYawRad
                        << L"\nmin_requested_yaw_rate_radps=" << trace.minRequestedYawRateRadps
                        << L"\nmax_requested_yaw_rate_radps=" << trace.maxRequestedYawRateRadps;
                    return message.str();
                }()).c_str());
        }

        TEST_METHOD(StartTurnTransition_Completes)
        {
            const PrimitiveTrace trace = RunStartTurnTransition();
            Assert::IsTrue(
                trace.completed,
                ([&]()
                {
                    std::wstringstream message;
                    message << L"DRV50_PRIMITIVE_CLOSED_LOOP"
                        << L"\nlabel=" << L"DRV50_TRANSITION"
                        << L"\nprimitive=" << L"StartTurnTransition"
                        << L"\nfield=" << L"completed"
                        << L"\nexpected=" << 1.0f
                        << L"\nactual=" << 0.0f
                        << L"\nlimit=" << 0.0f
                        << L"\ncompleted=" << (trace.completed ? L"true" : L"false")
                        << L"\nall_controls_finite=" << (trace.allControlsFinite ? L"true" : L"false")
                        << L"\ntruth_finite=" << (trace.truthFinite ? L"true" : L"false")
                        << L"\ncommand_evidence_valid=" << (trace.commandEvidenceValid ? L"true" : L"false")
                        << L"\nrequested_objectives_finite=" << (trace.requestedObjectivesFinite ? L"true" : L"false")
                        << L"\nsolver_clean=" << (trace.solverClean ? L"true" : L"false")
                        << L"\nticks=" << trace.appliedTicks
                        << L"\nelapsed_s=" << trace.elapsedSeconds
                        << L"\nx_m=" << trace.truth.GetPositionX()
                        << L"\ny_m=" << trace.truth.GetPositionY()
                        << L"\nyaw_deg=" << (trace.truth.GetOrientation() * RAD_TO_DEG_F)
                        << L"\nencoder_m=" << AverageEncoderDistanceM(trace)
                        << L"\nmin_y_m=" << trace.minY
                        << L"\nmin_yaw_rad=" << trace.minYawRad
                        << L"\nmax_yaw_rad=" << trace.maxYawRad
                        << L"\nmin_requested_yaw_rate_radps=" << trace.minRequestedYawRateRadps
                        << L"\nmax_requested_yaw_rate_radps=" << trace.maxRequestedYawRateRadps;
                    return message.str();
                }()).c_str());
        }

        TEST_METHOD(StartTurnTransition_UsesMultiTickHorizon)
        {
            const PrimitiveTrace trace = RunStartTurnTransition();
            Assert::IsTrue(
                trace.appliedTicks >= 20,
                ([&]()
                {
                    std::wstringstream message;
                    message << L"DRV50_PRIMITIVE_CLOSED_LOOP"
                        << L"\nlabel=" << L"DRV50_TRANSITION"
                        << L"\nprimitive=" << L"StartTurnTransition"
                        << L"\nfield=" << L"multi_tick_horizon"
                        << L"\nexpected=" << 20.0f
                        << L"\nactual=" << static_cast<float>(trace.appliedTicks)
                        << L"\nlimit=" << 0.0f
                        << L"\ncompleted=" << (trace.completed ? L"true" : L"false")
                        << L"\nall_controls_finite=" << (trace.allControlsFinite ? L"true" : L"false")
                        << L"\ntruth_finite=" << (trace.truthFinite ? L"true" : L"false")
                        << L"\ncommand_evidence_valid=" << (trace.commandEvidenceValid ? L"true" : L"false")
                        << L"\nrequested_objectives_finite=" << (trace.requestedObjectivesFinite ? L"true" : L"false")
                        << L"\nsolver_clean=" << (trace.solverClean ? L"true" : L"false")
                        << L"\nticks=" << trace.appliedTicks
                        << L"\nelapsed_s=" << trace.elapsedSeconds
                        << L"\nx_m=" << trace.truth.GetPositionX()
                        << L"\ny_m=" << trace.truth.GetPositionY()
                        << L"\nyaw_deg=" << (trace.truth.GetOrientation() * RAD_TO_DEG_F)
                        << L"\nencoder_m=" << AverageEncoderDistanceM(trace)
                        << L"\nmin_y_m=" << trace.minY
                        << L"\nmin_yaw_rad=" << trace.minYawRad
                        << L"\nmax_yaw_rad=" << trace.maxYawRad
                        << L"\nmin_requested_yaw_rate_radps=" << trace.minRequestedYawRateRadps
                        << L"\nmax_requested_yaw_rate_radps=" << trace.maxRequestedYawRateRadps;
                    return message.str();
                }()).c_str());
        }

        TEST_METHOD(StartTurnTransition_ControlsStayFinite)
        {
            const PrimitiveTrace trace = RunStartTurnTransition();
            Assert::IsTrue(
                trace.allControlsFinite,
                ([&]()
                {
                    std::wstringstream message;
                    message << L"DRV50_PRIMITIVE_CLOSED_LOOP"
                        << L"\nlabel=" << L"DRV50_TRANSITION"
                        << L"\nprimitive=" << L"StartTurnTransition"
                        << L"\nfield=" << L"finite_controls"
                        << L"\nexpected=" << 1.0f
                        << L"\nactual=" << 0.0f
                        << L"\nlimit=" << 0.0f
                        << L"\ncompleted=" << (trace.completed ? L"true" : L"false")
                        << L"\nall_controls_finite=" << (trace.allControlsFinite ? L"true" : L"false")
                        << L"\ntruth_finite=" << (trace.truthFinite ? L"true" : L"false")
                        << L"\ncommand_evidence_valid=" << (trace.commandEvidenceValid ? L"true" : L"false")
                        << L"\nrequested_objectives_finite=" << (trace.requestedObjectivesFinite ? L"true" : L"false")
                        << L"\nsolver_clean=" << (trace.solverClean ? L"true" : L"false")
                        << L"\nticks=" << trace.appliedTicks
                        << L"\nelapsed_s=" << trace.elapsedSeconds
                        << L"\nx_m=" << trace.truth.GetPositionX()
                        << L"\ny_m=" << trace.truth.GetPositionY()
                        << L"\nyaw_deg=" << (trace.truth.GetOrientation() * RAD_TO_DEG_F)
                        << L"\nencoder_m=" << AverageEncoderDistanceM(trace)
                        << L"\nmin_y_m=" << trace.minY
                        << L"\nmin_yaw_rad=" << trace.minYawRad
                        << L"\nmax_yaw_rad=" << trace.maxYawRad
                        << L"\nmin_requested_yaw_rate_radps=" << trace.minRequestedYawRateRadps
                        << L"\nmax_requested_yaw_rate_radps=" << trace.maxRequestedYawRateRadps;
                    return message.str();
                }()).c_str());
        }

        TEST_METHOD(StartTurnTransition_TruthStaysFinite)
        {
            const PrimitiveTrace trace = RunStartTurnTransition();
            Assert::IsTrue(
                trace.truthFinite,
                ([&]()
                {
                    std::wstringstream message;
                    message << L"DRV50_PRIMITIVE_CLOSED_LOOP"
                        << L"\nlabel=" << L"DRV50_TRANSITION"
                        << L"\nprimitive=" << L"StartTurnTransition"
                        << L"\nfield=" << L"finite_truth"
                        << L"\nexpected=" << 1.0f
                        << L"\nactual=" << 0.0f
                        << L"\nlimit=" << 0.0f
                        << L"\ncompleted=" << (trace.completed ? L"true" : L"false")
                        << L"\nall_controls_finite=" << (trace.allControlsFinite ? L"true" : L"false")
                        << L"\ntruth_finite=" << (trace.truthFinite ? L"true" : L"false")
                        << L"\ncommand_evidence_valid=" << (trace.commandEvidenceValid ? L"true" : L"false")
                        << L"\nrequested_objectives_finite=" << (trace.requestedObjectivesFinite ? L"true" : L"false")
                        << L"\nsolver_clean=" << (trace.solverClean ? L"true" : L"false")
                        << L"\nticks=" << trace.appliedTicks
                        << L"\nelapsed_s=" << trace.elapsedSeconds
                        << L"\nx_m=" << trace.truth.GetPositionX()
                        << L"\ny_m=" << trace.truth.GetPositionY()
                        << L"\nyaw_deg=" << (trace.truth.GetOrientation() * RAD_TO_DEG_F)
                        << L"\nencoder_m=" << AverageEncoderDistanceM(trace)
                        << L"\nmin_y_m=" << trace.minY
                        << L"\nmin_yaw_rad=" << trace.minYawRad
                        << L"\nmax_yaw_rad=" << trace.maxYawRad
                        << L"\nmin_requested_yaw_rate_radps=" << trace.minRequestedYawRateRadps
                        << L"\nmax_requested_yaw_rate_radps=" << trace.maxRequestedYawRateRadps;
                    return message.str();
                }()).c_str());
        }

        TEST_METHOD(StartTurnTransition_CommandEvidenceMatchesControls)
        {
            const PrimitiveTrace trace = RunStartTurnTransition();
            Assert::IsTrue(
                trace.commandEvidenceValid,
                ([&]()
                {
                    std::wstringstream message;
                    message << L"DRV50_PRIMITIVE_CLOSED_LOOP"
                        << L"\nlabel=" << L"DRV50_TRANSITION"
                        << L"\nprimitive=" << L"StartTurnTransition"
                        << L"\nfield=" << L"command_evidence"
                        << L"\nexpected=" << 1.0f
                        << L"\nactual=" << 0.0f
                        << L"\nlimit=" << 0.0f
                        << L"\ncompleted=" << (trace.completed ? L"true" : L"false")
                        << L"\nall_controls_finite=" << (trace.allControlsFinite ? L"true" : L"false")
                        << L"\ntruth_finite=" << (trace.truthFinite ? L"true" : L"false")
                        << L"\ncommand_evidence_valid=" << (trace.commandEvidenceValid ? L"true" : L"false")
                        << L"\nrequested_objectives_finite=" << (trace.requestedObjectivesFinite ? L"true" : L"false")
                        << L"\nsolver_clean=" << (trace.solverClean ? L"true" : L"false")
                        << L"\nticks=" << trace.appliedTicks
                        << L"\nelapsed_s=" << trace.elapsedSeconds
                        << L"\nx_m=" << trace.truth.GetPositionX()
                        << L"\ny_m=" << trace.truth.GetPositionY()
                        << L"\nyaw_deg=" << (trace.truth.GetOrientation() * RAD_TO_DEG_F)
                        << L"\nencoder_m=" << AverageEncoderDistanceM(trace)
                        << L"\nmin_y_m=" << trace.minY
                        << L"\nmin_yaw_rad=" << trace.minYawRad
                        << L"\nmax_yaw_rad=" << trace.maxYawRad
                        << L"\nmin_requested_yaw_rate_radps=" << trace.minRequestedYawRateRadps
                        << L"\nmax_requested_yaw_rate_radps=" << trace.maxRequestedYawRateRadps;
                    return message.str();
                }()).c_str());
        }

        TEST_METHOD(StartTurnTransition_RequestedObjectivesStayFinite)
        {
            const PrimitiveTrace trace = RunStartTurnTransition();
            Assert::IsTrue(
                trace.requestedObjectivesFinite,
                ([&]()
                {
                    std::wstringstream message;
                    message << L"DRV50_PRIMITIVE_CLOSED_LOOP"
                        << L"\nlabel=" << L"DRV50_TRANSITION"
                        << L"\nprimitive=" << L"StartTurnTransition"
                        << L"\nfield=" << L"finite_objectives"
                        << L"\nexpected=" << 1.0f
                        << L"\nactual=" << 0.0f
                        << L"\nlimit=" << 0.0f
                        << L"\ncompleted=" << (trace.completed ? L"true" : L"false")
                        << L"\nall_controls_finite=" << (trace.allControlsFinite ? L"true" : L"false")
                        << L"\ntruth_finite=" << (trace.truthFinite ? L"true" : L"false")
                        << L"\ncommand_evidence_valid=" << (trace.commandEvidenceValid ? L"true" : L"false")
                        << L"\nrequested_objectives_finite=" << (trace.requestedObjectivesFinite ? L"true" : L"false")
                        << L"\nsolver_clean=" << (trace.solverClean ? L"true" : L"false")
                        << L"\nticks=" << trace.appliedTicks
                        << L"\nelapsed_s=" << trace.elapsedSeconds
                        << L"\nx_m=" << trace.truth.GetPositionX()
                        << L"\ny_m=" << trace.truth.GetPositionY()
                        << L"\nyaw_deg=" << (trace.truth.GetOrientation() * RAD_TO_DEG_F)
                        << L"\nencoder_m=" << AverageEncoderDistanceM(trace)
                        << L"\nmin_y_m=" << trace.minY
                        << L"\nmin_yaw_rad=" << trace.minYawRad
                        << L"\nmax_yaw_rad=" << trace.maxYawRad
                        << L"\nmin_requested_yaw_rate_radps=" << trace.minRequestedYawRateRadps
                        << L"\nmax_requested_yaw_rate_radps=" << trace.maxRequestedYawRateRadps;
                    return message.str();
                }()).c_str());
        }

        TEST_METHOD(StartTurnTransition_RequestsPositiveYawRate)
        {
            const PrimitiveTrace trace = RunStartTurnTransition();
            Assert::IsTrue(
                trace.maxRequestedYawRateRadps > 0.05f,
                ([&]()
                {
                    std::wstringstream message;
                    message << L"DRV50_PRIMITIVE_CLOSED_LOOP"
                        << L"\nlabel=" << L"DRV50_TRANSITION"
                        << L"\nprimitive=" << L"StartTurnTransition"
                        << L"\nfield=" << L"positive_yaw_rate_request"
                        << L"\nexpected=" << 0.05f
                        << L"\nactual=" << trace.maxRequestedYawRateRadps
                        << L"\nlimit=" << 0.0f
                        << L"\ncompleted=" << (trace.completed ? L"true" : L"false")
                        << L"\nall_controls_finite=" << (trace.allControlsFinite ? L"true" : L"false")
                        << L"\ntruth_finite=" << (trace.truthFinite ? L"true" : L"false")
                        << L"\ncommand_evidence_valid=" << (trace.commandEvidenceValid ? L"true" : L"false")
                        << L"\nrequested_objectives_finite=" << (trace.requestedObjectivesFinite ? L"true" : L"false")
                        << L"\nsolver_clean=" << (trace.solverClean ? L"true" : L"false")
                        << L"\nticks=" << trace.appliedTicks
                        << L"\nelapsed_s=" << trace.elapsedSeconds
                        << L"\nx_m=" << trace.truth.GetPositionX()
                        << L"\ny_m=" << trace.truth.GetPositionY()
                        << L"\nyaw_deg=" << (trace.truth.GetOrientation() * RAD_TO_DEG_F)
                        << L"\nencoder_m=" << AverageEncoderDistanceM(trace)
                        << L"\nmin_y_m=" << trace.minY
                        << L"\nmin_yaw_rad=" << trace.minYawRad
                        << L"\nmax_yaw_rad=" << trace.maxYawRad
                        << L"\nmin_requested_yaw_rate_radps=" << trace.minRequestedYawRateRadps
                        << L"\nmax_requested_yaw_rate_radps=" << trace.maxRequestedYawRateRadps;
                    return message.str();
                }()).c_str());
        }

        TEST_METHOD(StartTurnTransition_DoesNotRequestWrongWayYawRate)
        {
            const PrimitiveTrace trace = RunStartTurnTransition();
            Assert::IsTrue(
                trace.minRequestedYawRateRadps >= -0.020f,
                ([&]()
                {
                    std::wstringstream message;
                    message << L"DRV50_PRIMITIVE_CLOSED_LOOP"
                        << L"\nlabel=" << L"DRV50_TRANSITION"
                        << L"\nprimitive=" << L"StartTurnTransition"
                        << L"\nfield=" << L"wrong_way_yaw_rate_request"
                        << L"\nexpected=" << 0.0f
                        << L"\nactual=" << trace.minRequestedYawRateRadps
                        << L"\nlimit=" << 0.020f
                        << L"\ncompleted=" << (trace.completed ? L"true" : L"false")
                        << L"\nall_controls_finite=" << (trace.allControlsFinite ? L"true" : L"false")
                        << L"\ntruth_finite=" << (trace.truthFinite ? L"true" : L"false")
                        << L"\ncommand_evidence_valid=" << (trace.commandEvidenceValid ? L"true" : L"false")
                        << L"\nrequested_objectives_finite=" << (trace.requestedObjectivesFinite ? L"true" : L"false")
                        << L"\nsolver_clean=" << (trace.solverClean ? L"true" : L"false")
                        << L"\nticks=" << trace.appliedTicks
                        << L"\nelapsed_s=" << trace.elapsedSeconds
                        << L"\nx_m=" << trace.truth.GetPositionX()
                        << L"\ny_m=" << trace.truth.GetPositionY()
                        << L"\nyaw_deg=" << (trace.truth.GetOrientation() * RAD_TO_DEG_F)
                        << L"\nencoder_m=" << AverageEncoderDistanceM(trace)
                        << L"\nmin_y_m=" << trace.minY
                        << L"\nmin_yaw_rad=" << trace.minYawRad
                        << L"\nmax_yaw_rad=" << trace.maxYawRad
                        << L"\nmin_requested_yaw_rate_radps=" << trace.minRequestedYawRateRadps
                        << L"\nmax_requested_yaw_rate_radps=" << trace.maxRequestedYawRateRadps;
                    return message.str();
                }()).c_str());
        }

        TEST_METHOD(StartTurnTransition_EncoderDistanceMatchesRequest)
        {
            const PrimitiveTrace trace = RunStartTurnTransition();
            Assert::AreEqual(
                kTransitionDistanceM,
                AverageEncoderDistanceM(trace),
                0.050f,
                ([&]()
                {
                    std::wstringstream message;
                    message << L"DRV50_PRIMITIVE_CLOSED_LOOP"
                        << L"\nlabel=" << L"DRV50_TRANSITION"
                        << L"\nprimitive=" << L"StartTurnTransition"
                        << L"\nfield=" << L"encoder_distance_m"
                        << L"\nexpected=" << kTransitionDistanceM
                        << L"\nactual=" << AverageEncoderDistanceM(trace)
                        << L"\nlimit=" << 0.050f
                        << L"\ncompleted=" << (trace.completed ? L"true" : L"false")
                        << L"\nall_controls_finite=" << (trace.allControlsFinite ? L"true" : L"false")
                        << L"\ntruth_finite=" << (trace.truthFinite ? L"true" : L"false")
                        << L"\ncommand_evidence_valid=" << (trace.commandEvidenceValid ? L"true" : L"false")
                        << L"\nrequested_objectives_finite=" << (trace.requestedObjectivesFinite ? L"true" : L"false")
                        << L"\nsolver_clean=" << (trace.solverClean ? L"true" : L"false")
                        << L"\nticks=" << trace.appliedTicks
                        << L"\nelapsed_s=" << trace.elapsedSeconds
                        << L"\nx_m=" << trace.truth.GetPositionX()
                        << L"\ny_m=" << trace.truth.GetPositionY()
                        << L"\nyaw_deg=" << (trace.truth.GetOrientation() * RAD_TO_DEG_F)
                        << L"\nencoder_m=" << AverageEncoderDistanceM(trace)
                        << L"\nmin_y_m=" << trace.minY
                        << L"\nmin_yaw_rad=" << trace.minYawRad
                        << L"\nmax_yaw_rad=" << trace.maxYawRad
                        << L"\nmin_requested_yaw_rate_radps=" << trace.minRequestedYawRateRadps
                        << L"\nmax_requested_yaw_rate_radps=" << trace.maxRequestedYawRateRadps;
                    return message.str();
                }()).c_str());
        }

        TEST_METHOD(StartTurnTransition_FinalHeadingMatchesIntegratedCurvature)
        {
            const PrimitiveTrace trace = RunStartTurnTransition();
            const float expectedYawRad =
                0.5f * kTransitionCurvatureRatePerM * kTransitionDistanceM * kTransitionDistanceM;
            const float headingErrorRad = std::fabs(AngleDifference(expectedYawRad, trace.truth.GetOrientation()));
            Assert::AreEqual(
                0.0f,
                headingErrorRad,
                0.200f,
                ([&]()
                {
                    std::wstringstream message;
                    message << L"DRV50_PRIMITIVE_CLOSED_LOOP"
                        << L"\nlabel=" << L"DRV50_TRANSITION"
                        << L"\nprimitive=" << L"StartTurnTransition"
                        << L"\nfield=" << L"heading_error_rad"
                        << L"\nexpected=" << 0.0f
                        << L"\nactual=" << headingErrorRad
                        << L"\nlimit=" << 0.200f
                        << L"\ncompleted=" << (trace.completed ? L"true" : L"false")
                        << L"\nall_controls_finite=" << (trace.allControlsFinite ? L"true" : L"false")
                        << L"\ntruth_finite=" << (trace.truthFinite ? L"true" : L"false")
                        << L"\ncommand_evidence_valid=" << (trace.commandEvidenceValid ? L"true" : L"false")
                        << L"\nrequested_objectives_finite=" << (trace.requestedObjectivesFinite ? L"true" : L"false")
                        << L"\nsolver_clean=" << (trace.solverClean ? L"true" : L"false")
                        << L"\nticks=" << trace.appliedTicks
                        << L"\nelapsed_s=" << trace.elapsedSeconds
                        << L"\nx_m=" << trace.truth.GetPositionX()
                        << L"\ny_m=" << trace.truth.GetPositionY()
                        << L"\nyaw_deg=" << (trace.truth.GetOrientation() * RAD_TO_DEG_F)
                        << L"\nencoder_m=" << AverageEncoderDistanceM(trace)
                        << L"\nmin_y_m=" << trace.minY
                        << L"\nmin_yaw_rad=" << trace.minYawRad
                        << L"\nmax_yaw_rad=" << trace.maxYawRad
                        << L"\nmin_requested_yaw_rate_radps=" << trace.minRequestedYawRateRadps
                        << L"\nmax_requested_yaw_rate_radps=" << trace.maxRequestedYawRateRadps;
                    return message.str();
                }()).c_str());
        }

        TEST_METHOD(StartTurnTransition_MakesForwardProgress)
        {
            const PrimitiveTrace trace = RunStartTurnTransition();
            Assert::IsTrue(
                trace.truth.GetPositionY() > 0.20f,
                ([&]()
                {
                    std::wstringstream message;
                    message << L"DRV50_PRIMITIVE_CLOSED_LOOP"
                        << L"\nlabel=" << L"DRV50_TRANSITION"
                        << L"\nprimitive=" << L"StartTurnTransition"
                        << L"\nfield=" << L"forward_progress_y_m"
                        << L"\nexpected=" << 0.20f
                        << L"\nactual=" << trace.truth.GetPositionY()
                        << L"\nlimit=" << 0.0f
                        << L"\ncompleted=" << (trace.completed ? L"true" : L"false")
                        << L"\nall_controls_finite=" << (trace.allControlsFinite ? L"true" : L"false")
                        << L"\ntruth_finite=" << (trace.truthFinite ? L"true" : L"false")
                        << L"\ncommand_evidence_valid=" << (trace.commandEvidenceValid ? L"true" : L"false")
                        << L"\nrequested_objectives_finite=" << (trace.requestedObjectivesFinite ? L"true" : L"false")
                        << L"\nsolver_clean=" << (trace.solverClean ? L"true" : L"false")
                        << L"\nticks=" << trace.appliedTicks
                        << L"\nelapsed_s=" << trace.elapsedSeconds
                        << L"\nx_m=" << trace.truth.GetPositionX()
                        << L"\ny_m=" << trace.truth.GetPositionY()
                        << L"\nyaw_deg=" << (trace.truth.GetOrientation() * RAD_TO_DEG_F)
                        << L"\nencoder_m=" << AverageEncoderDistanceM(trace)
                        << L"\nmin_y_m=" << trace.minY
                        << L"\nmin_yaw_rad=" << trace.minYawRad
                        << L"\nmax_yaw_rad=" << trace.maxYawRad
                        << L"\nmin_requested_yaw_rate_radps=" << trace.minRequestedYawRateRadps
                        << L"\nmax_requested_yaw_rate_radps=" << trace.maxRequestedYawRateRadps;
                    return message.str();
                }()).c_str());
        }

        TEST_METHOD(StartArc_Completes)
        {
            const PrimitiveTrace trace = RunStartArc();
            Assert::IsTrue(
                trace.completed,
                ([&]()
                {
                    std::wstringstream message;
                    message << L"DRV50_PRIMITIVE_CLOSED_LOOP"
                        << L"\nlabel=" << L"DRV50_ARC"
                        << L"\nprimitive=" << L"StartArc"
                        << L"\nfield=" << L"completed"
                        << L"\nexpected=" << 1.0f
                        << L"\nactual=" << 0.0f
                        << L"\nlimit=" << 0.0f
                        << L"\ncompleted=" << (trace.completed ? L"true" : L"false")
                        << L"\nall_controls_finite=" << (trace.allControlsFinite ? L"true" : L"false")
                        << L"\ntruth_finite=" << (trace.truthFinite ? L"true" : L"false")
                        << L"\ncommand_evidence_valid=" << (trace.commandEvidenceValid ? L"true" : L"false")
                        << L"\nrequested_objectives_finite=" << (trace.requestedObjectivesFinite ? L"true" : L"false")
                        << L"\nsolver_clean=" << (trace.solverClean ? L"true" : L"false")
                        << L"\nticks=" << trace.appliedTicks
                        << L"\nelapsed_s=" << trace.elapsedSeconds
                        << L"\nx_m=" << trace.truth.GetPositionX()
                        << L"\ny_m=" << trace.truth.GetPositionY()
                        << L"\nyaw_deg=" << (trace.truth.GetOrientation() * RAD_TO_DEG_F)
                        << L"\nencoder_m=" << AverageEncoderDistanceM(trace)
                        << L"\nmin_y_m=" << trace.minY
                        << L"\nmin_yaw_rad=" << trace.minYawRad
                        << L"\nmax_yaw_rad=" << trace.maxYawRad
                        << L"\nmin_requested_yaw_rate_radps=" << trace.minRequestedYawRateRadps
                        << L"\nmax_requested_yaw_rate_radps=" << trace.maxRequestedYawRateRadps;
                    return message.str();
                }()).c_str());
        }

        TEST_METHOD(StartArc_UsesMultiTickHorizon)
        {
            const PrimitiveTrace trace = RunStartArc();
            Assert::IsTrue(
                trace.appliedTicks >= 20,
                ([&]()
                {
                    std::wstringstream message;
                    message << L"DRV50_PRIMITIVE_CLOSED_LOOP"
                        << L"\nlabel=" << L"DRV50_ARC"
                        << L"\nprimitive=" << L"StartArc"
                        << L"\nfield=" << L"multi_tick_horizon"
                        << L"\nexpected=" << 20.0f
                        << L"\nactual=" << static_cast<float>(trace.appliedTicks)
                        << L"\nlimit=" << 0.0f
                        << L"\ncompleted=" << (trace.completed ? L"true" : L"false")
                        << L"\nall_controls_finite=" << (trace.allControlsFinite ? L"true" : L"false")
                        << L"\ntruth_finite=" << (trace.truthFinite ? L"true" : L"false")
                        << L"\ncommand_evidence_valid=" << (trace.commandEvidenceValid ? L"true" : L"false")
                        << L"\nrequested_objectives_finite=" << (trace.requestedObjectivesFinite ? L"true" : L"false")
                        << L"\nsolver_clean=" << (trace.solverClean ? L"true" : L"false")
                        << L"\nticks=" << trace.appliedTicks
                        << L"\nelapsed_s=" << trace.elapsedSeconds
                        << L"\nx_m=" << trace.truth.GetPositionX()
                        << L"\ny_m=" << trace.truth.GetPositionY()
                        << L"\nyaw_deg=" << (trace.truth.GetOrientation() * RAD_TO_DEG_F)
                        << L"\nencoder_m=" << AverageEncoderDistanceM(trace)
                        << L"\nmin_y_m=" << trace.minY
                        << L"\nmin_yaw_rad=" << trace.minYawRad
                        << L"\nmax_yaw_rad=" << trace.maxYawRad
                        << L"\nmin_requested_yaw_rate_radps=" << trace.minRequestedYawRateRadps
                        << L"\nmax_requested_yaw_rate_radps=" << trace.maxRequestedYawRateRadps;
                    return message.str();
                }()).c_str());
        }

        TEST_METHOD(StartArc_ControlsStayFinite)
        {
            const PrimitiveTrace trace = RunStartArc();
            Assert::IsTrue(
                trace.allControlsFinite,
                ([&]()
                {
                    std::wstringstream message;
                    message << L"DRV50_PRIMITIVE_CLOSED_LOOP"
                        << L"\nlabel=" << L"DRV50_ARC"
                        << L"\nprimitive=" << L"StartArc"
                        << L"\nfield=" << L"finite_controls"
                        << L"\nexpected=" << 1.0f
                        << L"\nactual=" << 0.0f
                        << L"\nlimit=" << 0.0f
                        << L"\ncompleted=" << (trace.completed ? L"true" : L"false")
                        << L"\nall_controls_finite=" << (trace.allControlsFinite ? L"true" : L"false")
                        << L"\ntruth_finite=" << (trace.truthFinite ? L"true" : L"false")
                        << L"\ncommand_evidence_valid=" << (trace.commandEvidenceValid ? L"true" : L"false")
                        << L"\nrequested_objectives_finite=" << (trace.requestedObjectivesFinite ? L"true" : L"false")
                        << L"\nsolver_clean=" << (trace.solverClean ? L"true" : L"false")
                        << L"\nticks=" << trace.appliedTicks
                        << L"\nelapsed_s=" << trace.elapsedSeconds
                        << L"\nx_m=" << trace.truth.GetPositionX()
                        << L"\ny_m=" << trace.truth.GetPositionY()
                        << L"\nyaw_deg=" << (trace.truth.GetOrientation() * RAD_TO_DEG_F)
                        << L"\nencoder_m=" << AverageEncoderDistanceM(trace)
                        << L"\nmin_y_m=" << trace.minY
                        << L"\nmin_yaw_rad=" << trace.minYawRad
                        << L"\nmax_yaw_rad=" << trace.maxYawRad
                        << L"\nmin_requested_yaw_rate_radps=" << trace.minRequestedYawRateRadps
                        << L"\nmax_requested_yaw_rate_radps=" << trace.maxRequestedYawRateRadps;
                    return message.str();
                }()).c_str());
        }

        TEST_METHOD(StartArc_TruthStaysFinite)
        {
            const PrimitiveTrace trace = RunStartArc();
            Assert::IsTrue(
                trace.truthFinite,
                ([&]()
                {
                    std::wstringstream message;
                    message << L"DRV50_PRIMITIVE_CLOSED_LOOP"
                        << L"\nlabel=" << L"DRV50_ARC"
                        << L"\nprimitive=" << L"StartArc"
                        << L"\nfield=" << L"finite_truth"
                        << L"\nexpected=" << 1.0f
                        << L"\nactual=" << 0.0f
                        << L"\nlimit=" << 0.0f
                        << L"\ncompleted=" << (trace.completed ? L"true" : L"false")
                        << L"\nall_controls_finite=" << (trace.allControlsFinite ? L"true" : L"false")
                        << L"\ntruth_finite=" << (trace.truthFinite ? L"true" : L"false")
                        << L"\ncommand_evidence_valid=" << (trace.commandEvidenceValid ? L"true" : L"false")
                        << L"\nrequested_objectives_finite=" << (trace.requestedObjectivesFinite ? L"true" : L"false")
                        << L"\nsolver_clean=" << (trace.solverClean ? L"true" : L"false")
                        << L"\nticks=" << trace.appliedTicks
                        << L"\nelapsed_s=" << trace.elapsedSeconds
                        << L"\nx_m=" << trace.truth.GetPositionX()
                        << L"\ny_m=" << trace.truth.GetPositionY()
                        << L"\nyaw_deg=" << (trace.truth.GetOrientation() * RAD_TO_DEG_F)
                        << L"\nencoder_m=" << AverageEncoderDistanceM(trace)
                        << L"\nmin_y_m=" << trace.minY
                        << L"\nmin_yaw_rad=" << trace.minYawRad
                        << L"\nmax_yaw_rad=" << trace.maxYawRad
                        << L"\nmin_requested_yaw_rate_radps=" << trace.minRequestedYawRateRadps
                        << L"\nmax_requested_yaw_rate_radps=" << trace.maxRequestedYawRateRadps;
                    return message.str();
                }()).c_str());
        }

        TEST_METHOD(StartArc_CommandEvidenceMatchesControls)
        {
            const PrimitiveTrace trace = RunStartArc();
            Assert::IsTrue(
                trace.commandEvidenceValid,
                ([&]()
                {
                    std::wstringstream message;
                    message << L"DRV50_PRIMITIVE_CLOSED_LOOP"
                        << L"\nlabel=" << L"DRV50_ARC"
                        << L"\nprimitive=" << L"StartArc"
                        << L"\nfield=" << L"command_evidence"
                        << L"\nexpected=" << 1.0f
                        << L"\nactual=" << 0.0f
                        << L"\nlimit=" << 0.0f
                        << L"\ncompleted=" << (trace.completed ? L"true" : L"false")
                        << L"\nall_controls_finite=" << (trace.allControlsFinite ? L"true" : L"false")
                        << L"\ntruth_finite=" << (trace.truthFinite ? L"true" : L"false")
                        << L"\ncommand_evidence_valid=" << (trace.commandEvidenceValid ? L"true" : L"false")
                        << L"\nrequested_objectives_finite=" << (trace.requestedObjectivesFinite ? L"true" : L"false")
                        << L"\nsolver_clean=" << (trace.solverClean ? L"true" : L"false")
                        << L"\nticks=" << trace.appliedTicks
                        << L"\nelapsed_s=" << trace.elapsedSeconds
                        << L"\nx_m=" << trace.truth.GetPositionX()
                        << L"\ny_m=" << trace.truth.GetPositionY()
                        << L"\nyaw_deg=" << (trace.truth.GetOrientation() * RAD_TO_DEG_F)
                        << L"\nencoder_m=" << AverageEncoderDistanceM(trace)
                        << L"\nmin_y_m=" << trace.minY
                        << L"\nmin_yaw_rad=" << trace.minYawRad
                        << L"\nmax_yaw_rad=" << trace.maxYawRad
                        << L"\nmin_requested_yaw_rate_radps=" << trace.minRequestedYawRateRadps
                        << L"\nmax_requested_yaw_rate_radps=" << trace.maxRequestedYawRateRadps;
                    return message.str();
                }()).c_str());
        }

        TEST_METHOD(StartArc_RequestedObjectivesStayFinite)
        {
            const PrimitiveTrace trace = RunStartArc();
            Assert::IsTrue(
                trace.requestedObjectivesFinite,
                ([&]()
                {
                    std::wstringstream message;
                    message << L"DRV50_PRIMITIVE_CLOSED_LOOP"
                        << L"\nlabel=" << L"DRV50_ARC"
                        << L"\nprimitive=" << L"StartArc"
                        << L"\nfield=" << L"finite_objectives"
                        << L"\nexpected=" << 1.0f
                        << L"\nactual=" << 0.0f
                        << L"\nlimit=" << 0.0f
                        << L"\ncompleted=" << (trace.completed ? L"true" : L"false")
                        << L"\nall_controls_finite=" << (trace.allControlsFinite ? L"true" : L"false")
                        << L"\ntruth_finite=" << (trace.truthFinite ? L"true" : L"false")
                        << L"\ncommand_evidence_valid=" << (trace.commandEvidenceValid ? L"true" : L"false")
                        << L"\nrequested_objectives_finite=" << (trace.requestedObjectivesFinite ? L"true" : L"false")
                        << L"\nsolver_clean=" << (trace.solverClean ? L"true" : L"false")
                        << L"\nticks=" << trace.appliedTicks
                        << L"\nelapsed_s=" << trace.elapsedSeconds
                        << L"\nx_m=" << trace.truth.GetPositionX()
                        << L"\ny_m=" << trace.truth.GetPositionY()
                        << L"\nyaw_deg=" << (trace.truth.GetOrientation() * RAD_TO_DEG_F)
                        << L"\nencoder_m=" << AverageEncoderDistanceM(trace)
                        << L"\nmin_y_m=" << trace.minY
                        << L"\nmin_yaw_rad=" << trace.minYawRad
                        << L"\nmax_yaw_rad=" << trace.maxYawRad
                        << L"\nmin_requested_yaw_rate_radps=" << trace.minRequestedYawRateRadps
                        << L"\nmax_requested_yaw_rate_radps=" << trace.maxRequestedYawRateRadps;
                    return message.str();
                }()).c_str());
        }

        TEST_METHOD(StartArc_RequestsPositiveYawRate)
        {
            const PrimitiveTrace trace = RunStartArc();
            Assert::IsTrue(
                trace.maxRequestedYawRateRadps > 0.10f,
                ([&]()
                {
                    std::wstringstream message;
                    message << L"DRV50_PRIMITIVE_CLOSED_LOOP"
                        << L"\nlabel=" << L"DRV50_ARC"
                        << L"\nprimitive=" << L"StartArc"
                        << L"\nfield=" << L"positive_yaw_rate_request"
                        << L"\nexpected=" << 0.10f
                        << L"\nactual=" << trace.maxRequestedYawRateRadps
                        << L"\nlimit=" << 0.0f
                        << L"\ncompleted=" << (trace.completed ? L"true" : L"false")
                        << L"\nall_controls_finite=" << (trace.allControlsFinite ? L"true" : L"false")
                        << L"\ntruth_finite=" << (trace.truthFinite ? L"true" : L"false")
                        << L"\ncommand_evidence_valid=" << (trace.commandEvidenceValid ? L"true" : L"false")
                        << L"\nrequested_objectives_finite=" << (trace.requestedObjectivesFinite ? L"true" : L"false")
                        << L"\nsolver_clean=" << (trace.solverClean ? L"true" : L"false")
                        << L"\nticks=" << trace.appliedTicks
                        << L"\nelapsed_s=" << trace.elapsedSeconds
                        << L"\nx_m=" << trace.truth.GetPositionX()
                        << L"\ny_m=" << trace.truth.GetPositionY()
                        << L"\nyaw_deg=" << (trace.truth.GetOrientation() * RAD_TO_DEG_F)
                        << L"\nencoder_m=" << AverageEncoderDistanceM(trace)
                        << L"\nmin_y_m=" << trace.minY
                        << L"\nmin_yaw_rad=" << trace.minYawRad
                        << L"\nmax_yaw_rad=" << trace.maxYawRad
                        << L"\nmin_requested_yaw_rate_radps=" << trace.minRequestedYawRateRadps
                        << L"\nmax_requested_yaw_rate_radps=" << trace.maxRequestedYawRateRadps;
                    return message.str();
                }()).c_str());
        }

        TEST_METHOD(StartArc_DoesNotYawWrongWay)
        {
            const PrimitiveTrace trace = RunStartArc();
            Assert::IsTrue(
                trace.minYawRad >= -0.030f,
                ([&]()
                {
                    std::wstringstream message;
                    message << L"DRV50_PRIMITIVE_CLOSED_LOOP"
                        << L"\nlabel=" << L"DRV50_ARC"
                        << L"\nprimitive=" << L"StartArc"
                        << L"\nfield=" << L"wrong_way_yaw"
                        << L"\nexpected=" << 0.0f
                        << L"\nactual=" << trace.minYawRad
                        << L"\nlimit=" << 0.030f
                        << L"\ncompleted=" << (trace.completed ? L"true" : L"false")
                        << L"\nall_controls_finite=" << (trace.allControlsFinite ? L"true" : L"false")
                        << L"\ntruth_finite=" << (trace.truthFinite ? L"true" : L"false")
                        << L"\ncommand_evidence_valid=" << (trace.commandEvidenceValid ? L"true" : L"false")
                        << L"\nrequested_objectives_finite=" << (trace.requestedObjectivesFinite ? L"true" : L"false")
                        << L"\nsolver_clean=" << (trace.solverClean ? L"true" : L"false")
                        << L"\nticks=" << trace.appliedTicks
                        << L"\nelapsed_s=" << trace.elapsedSeconds
                        << L"\nx_m=" << trace.truth.GetPositionX()
                        << L"\ny_m=" << trace.truth.GetPositionY()
                        << L"\nyaw_deg=" << (trace.truth.GetOrientation() * RAD_TO_DEG_F)
                        << L"\nencoder_m=" << AverageEncoderDistanceM(trace)
                        << L"\nmin_y_m=" << trace.minY
                        << L"\nmin_yaw_rad=" << trace.minYawRad
                        << L"\nmax_yaw_rad=" << trace.maxYawRad
                        << L"\nmin_requested_yaw_rate_radps=" << trace.minRequestedYawRateRadps
                        << L"\nmax_requested_yaw_rate_radps=" << trace.maxRequestedYawRateRadps;
                    return message.str();
                }()).c_str());
        }

        TEST_METHOD(StartArc_EncoderDistanceMatchesRequest)
        {
            const PrimitiveTrace trace = RunStartArc();
            Assert::AreEqual(
                kArcDistanceM,
                AverageEncoderDistanceM(trace),
                0.060f,
                ([&]()
                {
                    std::wstringstream message;
                    message << L"DRV50_PRIMITIVE_CLOSED_LOOP"
                        << L"\nlabel=" << L"DRV50_ARC"
                        << L"\nprimitive=" << L"StartArc"
                        << L"\nfield=" << L"encoder_distance_m"
                        << L"\nexpected=" << kArcDistanceM
                        << L"\nactual=" << AverageEncoderDistanceM(trace)
                        << L"\nlimit=" << 0.060f
                        << L"\ncompleted=" << (trace.completed ? L"true" : L"false")
                        << L"\nall_controls_finite=" << (trace.allControlsFinite ? L"true" : L"false")
                        << L"\ntruth_finite=" << (trace.truthFinite ? L"true" : L"false")
                        << L"\ncommand_evidence_valid=" << (trace.commandEvidenceValid ? L"true" : L"false")
                        << L"\nrequested_objectives_finite=" << (trace.requestedObjectivesFinite ? L"true" : L"false")
                        << L"\nsolver_clean=" << (trace.solverClean ? L"true" : L"false")
                        << L"\nticks=" << trace.appliedTicks
                        << L"\nelapsed_s=" << trace.elapsedSeconds
                        << L"\nx_m=" << trace.truth.GetPositionX()
                        << L"\ny_m=" << trace.truth.GetPositionY()
                        << L"\nyaw_deg=" << (trace.truth.GetOrientation() * RAD_TO_DEG_F)
                        << L"\nencoder_m=" << AverageEncoderDistanceM(trace)
                        << L"\nmin_y_m=" << trace.minY
                        << L"\nmin_yaw_rad=" << trace.minYawRad
                        << L"\nmax_yaw_rad=" << trace.maxYawRad
                        << L"\nmin_requested_yaw_rate_radps=" << trace.minRequestedYawRateRadps
                        << L"\nmax_requested_yaw_rate_radps=" << trace.maxRequestedYawRateRadps;
                    return message.str();
                }()).c_str());
        }

        TEST_METHOD(StartArc_FinalHeadingMatchesCurvature)
        {
            const PrimitiveTrace trace = RunStartArc();
            const float expectedYawRad = kArcDistanceM * kArcCurvaturePerM;
            const float headingErrorRad = std::fabs(AngleDifference(expectedYawRad, trace.truth.GetOrientation()));
            Assert::AreEqual(
                0.0f,
                headingErrorRad,
                0.250f,
                ([&]()
                {
                    std::wstringstream message;
                    message << L"DRV50_PRIMITIVE_CLOSED_LOOP"
                        << L"\nlabel=" << L"DRV50_ARC"
                        << L"\nprimitive=" << L"StartArc"
                        << L"\nfield=" << L"heading_error_rad"
                        << L"\nexpected=" << 0.0f
                        << L"\nactual=" << headingErrorRad
                        << L"\nlimit=" << 0.250f
                        << L"\ncompleted=" << (trace.completed ? L"true" : L"false")
                        << L"\nall_controls_finite=" << (trace.allControlsFinite ? L"true" : L"false")
                        << L"\ntruth_finite=" << (trace.truthFinite ? L"true" : L"false")
                        << L"\ncommand_evidence_valid=" << (trace.commandEvidenceValid ? L"true" : L"false")
                        << L"\nrequested_objectives_finite=" << (trace.requestedObjectivesFinite ? L"true" : L"false")
                        << L"\nsolver_clean=" << (trace.solverClean ? L"true" : L"false")
                        << L"\nticks=" << trace.appliedTicks
                        << L"\nelapsed_s=" << trace.elapsedSeconds
                        << L"\nx_m=" << trace.truth.GetPositionX()
                        << L"\ny_m=" << trace.truth.GetPositionY()
                        << L"\nyaw_deg=" << (trace.truth.GetOrientation() * RAD_TO_DEG_F)
                        << L"\nencoder_m=" << AverageEncoderDistanceM(trace)
                        << L"\nmin_y_m=" << trace.minY
                        << L"\nmin_yaw_rad=" << trace.minYawRad
                        << L"\nmax_yaw_rad=" << trace.maxYawRad
                        << L"\nmin_requested_yaw_rate_radps=" << trace.minRequestedYawRateRadps
                        << L"\nmax_requested_yaw_rate_radps=" << trace.maxRequestedYawRateRadps;
                    return message.str();
                }()).c_str());
        }

        TEST_METHOD(StartArc_MakesRightwardProgress)
        {
            const PrimitiveTrace trace = RunStartArc();
            Assert::IsTrue(
                trace.truth.GetPositionX() > 0.030f,
                ([&]()
                {
                    std::wstringstream message;
                    message << L"DRV50_PRIMITIVE_CLOSED_LOOP"
                        << L"\nlabel=" << L"DRV50_ARC"
                        << L"\nprimitive=" << L"StartArc"
                        << L"\nfield=" << L"rightward_progress_x_m"
                        << L"\nexpected=" << 0.030f
                        << L"\nactual=" << trace.truth.GetPositionX()
                        << L"\nlimit=" << 0.0f
                        << L"\ncompleted=" << (trace.completed ? L"true" : L"false")
                        << L"\nall_controls_finite=" << (trace.allControlsFinite ? L"true" : L"false")
                        << L"\ntruth_finite=" << (trace.truthFinite ? L"true" : L"false")
                        << L"\ncommand_evidence_valid=" << (trace.commandEvidenceValid ? L"true" : L"false")
                        << L"\nrequested_objectives_finite=" << (trace.requestedObjectivesFinite ? L"true" : L"false")
                        << L"\nsolver_clean=" << (trace.solverClean ? L"true" : L"false")
                        << L"\nticks=" << trace.appliedTicks
                        << L"\nelapsed_s=" << trace.elapsedSeconds
                        << L"\nx_m=" << trace.truth.GetPositionX()
                        << L"\ny_m=" << trace.truth.GetPositionY()
                        << L"\nyaw_deg=" << (trace.truth.GetOrientation() * RAD_TO_DEG_F)
                        << L"\nencoder_m=" << AverageEncoderDistanceM(trace)
                        << L"\nmin_y_m=" << trace.minY
                        << L"\nmin_yaw_rad=" << trace.minYawRad
                        << L"\nmax_yaw_rad=" << trace.maxYawRad
                        << L"\nmin_requested_yaw_rate_radps=" << trace.minRequestedYawRateRadps
                        << L"\nmax_requested_yaw_rate_radps=" << trace.maxRequestedYawRateRadps;
                    return message.str();
                }()).c_str());
        }

        TEST_METHOD(StartArc_MakesForwardProgress)
        {
            const PrimitiveTrace trace = RunStartArc();
            Assert::IsTrue(
                trace.truth.GetPositionY() > 0.20f,
                ([&]()
                {
                    std::wstringstream message;
                    message << L"DRV50_PRIMITIVE_CLOSED_LOOP"
                        << L"\nlabel=" << L"DRV50_ARC"
                        << L"\nprimitive=" << L"StartArc"
                        << L"\nfield=" << L"forward_progress_y_m"
                        << L"\nexpected=" << 0.20f
                        << L"\nactual=" << trace.truth.GetPositionY()
                        << L"\nlimit=" << 0.0f
                        << L"\ncompleted=" << (trace.completed ? L"true" : L"false")
                        << L"\nall_controls_finite=" << (trace.allControlsFinite ? L"true" : L"false")
                        << L"\ntruth_finite=" << (trace.truthFinite ? L"true" : L"false")
                        << L"\ncommand_evidence_valid=" << (trace.commandEvidenceValid ? L"true" : L"false")
                        << L"\nrequested_objectives_finite=" << (trace.requestedObjectivesFinite ? L"true" : L"false")
                        << L"\nsolver_clean=" << (trace.solverClean ? L"true" : L"false")
                        << L"\nticks=" << trace.appliedTicks
                        << L"\nelapsed_s=" << trace.elapsedSeconds
                        << L"\nx_m=" << trace.truth.GetPositionX()
                        << L"\ny_m=" << trace.truth.GetPositionY()
                        << L"\nyaw_deg=" << (trace.truth.GetOrientation() * RAD_TO_DEG_F)
                        << L"\nencoder_m=" << AverageEncoderDistanceM(trace)
                        << L"\nmin_y_m=" << trace.minY
                        << L"\nmin_yaw_rad=" << trace.minYawRad
                        << L"\nmax_yaw_rad=" << trace.maxYawRad
                        << L"\nmin_requested_yaw_rate_radps=" << trace.minRequestedYawRateRadps
                        << L"\nmax_requested_yaw_rate_radps=" << trace.maxRequestedYawRateRadps;
                    return message.str();
                }()).c_str());
        }
    };
}
