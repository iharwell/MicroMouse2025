#include "pch.h"
#include "CppUnitTest.h"

#include "EstimatorTestSupport.h"
#include "..\MazeMap\Estimator.h"
#include "..\MazeMap\DriveBase.h"
#include "..\MazeMap\PlantModel.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    namespace
    {
        using ControlVector = MazeMap::App::Internal::LoopController::ControlVector;

        constexpr std::uint8_t kDriveBaseRightEncoderChannel = 1U;
        constexpr std::uint8_t kDriveBaseLeftEncoderChannel = 2U;
        constexpr float kDriveBasePredictDtSeconds = 0.01f;
        constexpr int kDriveBaseHoldFeedforwardSteps = 400;
        constexpr int kDriveBaseVelocityTargetSteps = 600;

        struct DriveBaseHarness final
        {
            Estimator estimator;
            DriveBase drive;

            DriveBaseHarness(
                const PlantModel& plant,
                const MazeMap::ProportionalDerivativeCluster& cluster)
                : estimator(PlantParams::Default())
                , drive(plant, estimator, cluster)
            {
            }
        };

        SensorSnapshot BuildDriveBaseSensorSnapshot(float yawRateRadps = 0.0f) noexcept
        {
            SensorSnapshot snapshot{};
            snapshot.gyroRawRadps = yawRateRadps;
            snapshot.gyroRadps = yawRateRadps;
            return snapshot;
        }

        SensorSnapshot BuildDriveBaseSensorSnapshot(
            float gyroRawRadps,
            float gyroRadps,
            float accelBodyXMps2,
            float accelBodyYMps2,
            bool accelBiasValid) noexcept
        {
            SensorSnapshot snapshot{};
            snapshot.gyroRawRadps = gyroRawRadps;
            snapshot.gyroRadps = gyroRadps;
            snapshot.accelBodyXMps2 = accelBodyXMps2;
            snapshot.accelBodyYMps2 = accelBodyYMps2;
            snapshot.accelBiasValid = accelBiasValid;
            return snapshot;
        }

        int32_t ConsumeWholeEncoderCounts(
            float deltaCounts,
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

        void SimulateDriveBaseCycle(
            DriveBase& drive,
            Estimator& estimator,
            PlantModel& plant,
            PlantModel::StateVector& truthState,
            float& leftEncoderRemainderCounts,
            float& rightEncoderRemainderCounts,
            float dtSeconds)
        {
            const PlantParams& params = PlantParams::Default();
            const DriveTelemetry telemetry = drive.GetTelemetry();
            ControlInput control{};
            control.leftMotorCommand = telemetry.leftDriveCommand;
            control.rightMotorCommand = telemetry.rightDriveCommand;
            control.fanDutyCycle = 0.80f;
            control.batteryVoltageV = params.supplyVoltageV;

            const PlantModel::StateVector previousTruthState = truthState;
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
            const float distancePerCountM = DistancePerEncoderCountMeters(params);
            const int32_t leftCounts =
                ConsumeWholeEncoderCounts(leftDistanceDeltaM / distancePerCountM, leftEncoderRemainderCounts);
            const int32_t rightCounts =
                ConsumeWholeEncoderCounts(rightDistanceDeltaM / distancePerCountM, rightEncoderRemainderCounts);

            MazeMap::Platform::WriteEncoderCount(kDriveBaseLeftEncoderChannel, leftCounts);
            MazeMap::Platform::WriteEncoderCount(kDriveBaseRightEncoderChannel, rightCounts);
            UpdateDriveEstimator(
                drive,
                estimator,
                dtSeconds,
                BuildDriveBaseSensorSnapshot(truthState(VehicleState::kR)));
        }

        void AssertDriveBaseStateNearTarget(
            const VehicleState::StateVector& state,
            float expectedForwardVelocityMps,
            float expectedYawRateRadps,
            float forwardToleranceMps,
            float yawToleranceRadps,
            float maxLateralVelocityMps)
        {
            Assert::AreEqual(expectedForwardVelocityMps, state(VehicleState::kU), forwardToleranceMps);
            Assert::AreEqual(expectedYawRateRadps, state(VehicleState::kR), yawToleranceRadps);
            Assert::IsTrue(std::fabs(state(VehicleState::kV)) <= maxLateralVelocityMps);
        }

        bool IsFiniteControlVector(const ControlVector& command) noexcept
        {
            return std::isfinite(command.leftMotorPwm) && std::isfinite(command.rightMotorPwm);
        }

        float ControlVectorDifferenceMagnitude(
            const ControlVector& lhs,
            const ControlVector& rhs) noexcept
        {
            return
                std::fabs(lhs.leftMotorPwm - rhs.leftMotorPwm) +
                std::fabs(lhs.rightMotorPwm - rhs.rightMotorPwm);
        }

        float ControlVectorAverage(const ControlVector& command) noexcept
        {
            return 0.5f * (command.leftMotorPwm + command.rightMotorPwm);
        }

        float ControlVectorDelta(const ControlVector& command) noexcept
        {
            return 0.5f * (command.leftMotorPwm - command.rightMotorPwm);
        }

        void UpdateDriveBaseSignals(
            DriveBase& drive,
            Estimator& estimator,
            const SensorSnapshot& snapshot,
            int32_t leftCounts = 0,
            int32_t rightCounts = 0,
            float dtSeconds = 0.001f);

        void AssertDriveCommandsEqual(
            const ControlVector& expected,
            const ControlVector& actual,
            float tolerance = 1.0e-3f)
        {
            Assert::IsTrue(IsFiniteControlVector(expected));
            Assert::IsTrue(IsFiniteControlVector(actual));
            Assert::AreEqual(expected.leftMotorPwm, actual.leftMotorPwm, tolerance);
            Assert::AreEqual(expected.rightMotorPwm, actual.rightMotorPwm, tolerance);
        }

        void AssertDriveCommandsDiffer(
            const ControlVector& lhs,
            const ControlVector& rhs,
            float minimumDifference = 1.0e-4f)
        {
            Assert::IsTrue(IsFiniteControlVector(lhs));
            Assert::IsTrue(IsFiniteControlVector(rhs));
            Assert::IsTrue(ControlVectorDifferenceMagnitude(lhs, rhs) > minimumDifference);
        }

        void AssertDriveCommandMatchesSolution(
            const ControlVector& command,
            const DriveCommandSolution& solution,
            float tolerance = 1.0e-6f)
        {
            Assert::IsTrue(IsFiniteControlVector(command));
            Assert::AreEqual(solution.control.leftMotorCommand, command.leftMotorPwm, tolerance);
            Assert::AreEqual(solution.control.rightMotorCommand, command.rightMotorPwm, tolerance);
        }

        void PrimeDriveBaseWithEncoderDelta(
            DriveBase& drive,
            Estimator& estimator,
            const int32_t leftCounts,
            const int32_t rightCounts,
            const float dtSeconds = 0.001f)
        {
            MazeMap::Platform::WriteEncoderCount(kDriveBaseLeftEncoderChannel, leftCounts);
            MazeMap::Platform::WriteEncoderCount(kDriveBaseRightEncoderChannel, rightCounts);
            const SensorSnapshot snapshot = BuildDriveBaseSensorSnapshot();
            UpdateDriveEstimator(drive, estimator, dtSeconds, snapshot);
        }

        void UpdateDriveBaseSignals(
            DriveBase& drive,
            Estimator& estimator,
            const SensorSnapshot& snapshot,
            const int32_t leftCounts,
            const int32_t rightCounts,
            const float dtSeconds)
        {
            MazeMap::Platform::WriteEncoderCount(kDriveBaseLeftEncoderChannel, leftCounts);
            MazeMap::Platform::WriteEncoderCount(kDriveBaseRightEncoderChannel, rightCounts);
            UpdateDriveEstimator(drive, estimator, dtSeconds, snapshot);
        }

        constexpr float kOscillationTraceDeadbandMps = 1.0e-3f;

        float ComputeHighPassRms(
            const std::vector<float>& values,
            const std::size_t startIndex) noexcept
        {
            if (startIndex >= values.size())
            {
                return 0.0f;
            }

            const std::size_t sampleCount = values.size() - startIndex;
            float mean = 0.0f;
            for (std::size_t index = startIndex; index < values.size(); ++index)
            {
                mean += values[index];
            }
            mean /= static_cast<float>(sampleCount);

            float squaredErrorSum = 0.0f;
            for (std::size_t index = startIndex; index < values.size(); ++index)
            {
                const float centeredValue = values[index] - mean;
                squaredErrorSum += centeredValue * centeredValue;
            }

            return std::sqrt(squaredErrorSum / static_cast<float>(sampleCount));
        }

        float ComputeMeanAbsoluteValue(
            const std::vector<float>& values,
            const std::size_t startIndex) noexcept
        {
            if (startIndex >= values.size())
            {
                return 0.0f;
            }

            const std::size_t sampleCount = values.size() - startIndex;
            float absoluteSum = 0.0f;
            for (std::size_t index = startIndex; index < values.size(); ++index)
            {
                absoluteSum += std::fabs(values[index]);
            }

            return absoluteSum / static_cast<float>(sampleCount);
        }

        float ComputeSignFlipRate(
            const std::vector<float>& values,
            const std::size_t startIndex,
            const float deadband) noexcept
        {
            if (startIndex >= values.size())
            {
                return 0.0f;
            }

            int previousSign = 0;
            std::size_t consideredSamples = 0U;
            std::size_t signFlips = 0U;
            for (std::size_t index = startIndex; index < values.size(); ++index)
            {
                const float value = values[index];
                if (std::fabs(value) <= deadband)
                {
                    continue;
                }

                const int sign = (value > 0.0f) ? 1 : -1;
                if (previousSign != 0)
                {
                    ++consideredSamples;
                    if (sign != previousSign)
                    {
                        ++signFlips;
                    }
                }

                previousSign = sign;
            }

            if (consideredSamples == 0U)
            {
                return 0.0f;
            }

            return static_cast<float>(signFlips) / static_cast<float>(consideredSamples);
        }

        float ComputeSaturationDuty(
            const std::vector<bool>& saturatedSamples,
            const std::size_t startIndex) noexcept
        {
            if (startIndex >= saturatedSamples.size())
            {
                return 0.0f;
            }

            const std::size_t sampleCount = saturatedSamples.size() - startIndex;
            std::size_t saturatedSampleCount = 0U;
            for (std::size_t index = startIndex; index < saturatedSamples.size(); ++index)
            {
                if (saturatedSamples[index])
                {
                    ++saturatedSampleCount;
                }
            }

            return static_cast<float>(saturatedSampleCount) / static_cast<float>(sampleCount);
        }

        struct AlternatingSaturationPatternResult final
        {
            std::uint32_t saturatedRunCount = 0U;
            std::uint32_t alternationCount = 0U;
            bool patternDetected = false;
        };

        enum class OscillationPairingKind
        {
            HeadingState,
            VelocityState,
            YawRateState,
            YawRateGyro,
            LongitudinalAccelerationState,
            LongitudinalAccelerationImuForwardAccel,
            WheelVelocityState,
            WheelVelocityEncoder
        };

        AlternatingSaturationPatternResult AnalyzeAlternatingSaturationPattern(
            const std::vector<std::uint16_t>& saturationFlags,
            const std::size_t startIndex) noexcept
        {
            AlternatingSaturationPatternResult result{};
            if (startIndex >= saturationFlags.size())
            {
                return result;
            }

            std::uint16_t previousOneSidedSaturation = 0U;
            for (std::size_t index = startIndex; index < saturationFlags.size(); ++index)
            {
                const std::uint16_t saturatedBanks =
                    static_cast<std::uint16_t>(saturationFlags[index] & 0x3u);
                if (saturatedBanks == 0U)
                {
                    continue;
                }

                // Both-wheel saturation does not establish a left/right alternation pattern.
                if (saturatedBanks == 0x3u)
                {
                    previousOneSidedSaturation = 0U;
                    continue;
                }

                if (saturatedBanks == previousOneSidedSaturation)
                {
                    continue;
                }

                ++result.saturatedRunCount;
                if ((previousOneSidedSaturation != 0U) &&
                    (saturatedBanks != previousOneSidedSaturation))
                {
                    ++result.alternationCount;
                }

                previousOneSidedSaturation = saturatedBanks;
            }

            result.patternDetected = (result.alternationCount >= 2U);
            return result;
        }

        struct OscillationPairingScenario final
        {
            float initialSetpoint = 0.0f;
            float stepSetpoint = 0.20f;
            float dtSeconds = kDriveBasePredictDtSeconds;
            int totalSteps = 320;
            int stepStartStep = 40;
            int steadyWindowStartStep = 200;
            float initialForwardSpeedMps = 0.0f;
            float initialYawRateRadps = 0.0f;
            int disturbanceStartStep = std::numeric_limits<int>::max();
            int disturbanceHalfPeriodSteps = 1;
            int encoderCountDisturbanceCounts = 0;
            float gyroDisturbanceRadps = 0.0f;
            float accelBodyXDisturbanceMps2 = 0.0f;
            float accelBodyYDisturbanceMps2 = 0.0f;
        };

        struct OscillationPairingTraceSample final
        {
            float targetSignal = 0.0f;
            float sourceSignal = 0.0f;
            float estimatorSignal = 0.0f;
            float truthSignal = 0.0f;
            float feedbackCommandComponent = 0.0f;
            std::uint16_t saturationFlags = 0U;
        };

        struct OscillationPairingAuditResult final
        {
            float steadySourceErrorHighPassRms = 0.0f;
            float steadyEstimatorErrorHighPassRms = 0.0f;
            float steadyTruthErrorHighPassRms = 0.0f;
            float steadySourceAbsError = 0.0f;
            float steadyEstimatorAbsError = 0.0f;
            float steadyTruthAbsError = 0.0f;
            float steadyFeedbackComponentRms = 0.0f;
            float steadySignFlipRate = 0.0f;
            float fullSaturationDuty = 0.0f;
            float steadySaturationDuty = 0.0f;
            std::uint32_t steadySaturatedRunCount = 0U;
            std::uint32_t steadyAlternatingSaturationCount = 0U;
            bool steadyAlternatingSaturationPatternDetected = false;
        };

        struct OscillationGeneratedCommand final
        {
            ControlVector command{};
            float linearCommandTargetMps = 0.0f;
            float angularCommandTargetRadps = 0.0f;
            float targetSignal = 0.0f;
        };

        MazeMap::CommandPD ResolveOscillationPairingSignalSource(
            const OscillationPairingKind pairingKind) noexcept
        {
            switch (pairingKind)
            {
            case OscillationPairingKind::HeadingState:
                return MazeMap::CommandPD::StateHeadingPD;
            case OscillationPairingKind::VelocityState:
                return MazeMap::CommandPD::StateVelocityPD;
            case OscillationPairingKind::YawRateState:
                return MazeMap::CommandPD::StateYawPD;
            case OscillationPairingKind::YawRateGyro:
                return MazeMap::CommandPD::IMUYaw;
            case OscillationPairingKind::LongitudinalAccelerationState:
                return MazeMap::CommandPD::StateAccelerationPD;
            case OscillationPairingKind::LongitudinalAccelerationImuForwardAccel:
                return MazeMap::CommandPD::IMUForwardAccel;
            case OscillationPairingKind::WheelVelocityState:
                return MazeMap::CommandPD::StateWheelOmegaPD;
            case OscillationPairingKind::WheelVelocityEncoder:
            default:
                return MazeMap::CommandPD::EncoderVelocity;
            }
        }

        bool UsesAngularFeedbackComponent(
            const MazeMap::CommandPD signalSource) noexcept
        {
            return
                (signalSource == MazeMap::CommandPD::StateHeadingPD) ||
                (signalSource == MazeMap::CommandPD::StateYawPD) ||
                (signalSource == MazeMap::CommandPD::IMUYaw) ||
                (signalSource == MazeMap::CommandPD::IMULateralAccel);
        }

        float ResolveAlternatingDisturbance(
            const int stepIndex,
            const int startStep,
            const int halfPeriodSteps,
            const float amplitude) noexcept
        {
            if ((amplitude == 0.0f) || (stepIndex < startStep))
            {
                return 0.0f;
            }

            const int resolvedHalfPeriodSteps = (std::max)(halfPeriodSteps, 1);
            const int phaseIndex = (stepIndex - startStep) / resolvedHalfPeriodSteps;
            return ((phaseIndex & 1) == 0) ? amplitude : -amplitude;
        }

        int ResolveAlternatingCountDisturbance(
            const int stepIndex,
            const int startStep,
            const int halfPeriodSteps,
            const int amplitudeCounts) noexcept
        {
            if ((amplitudeCounts == 0) || (stepIndex < startStep))
            {
                return 0;
            }

            const int resolvedHalfPeriodSteps = (std::max)(halfPeriodSteps, 1);
            const int phaseIndex = (stepIndex - startStep) / resolvedHalfPeriodSteps;
            return ((phaseIndex & 1) == 0) ? amplitudeCounts : -amplitudeCounts;
        }

        MazeMap::ProportionalDerivativeCluster BuildOscillationPairingCluster(
            const MazeMap::ProportionalDerivative& pd,
            const OscillationPairingKind pairingKind) noexcept
        {
            MazeMap::ProportionalDerivativeCluster cluster{};
            switch (pairingKind)
            {
            case OscillationPairingKind::HeadingState:
                cluster.HeadingStatePD = pd;
                break;
            case OscillationPairingKind::VelocityState:
                cluster.VelocityStatePD = pd;
                break;
            case OscillationPairingKind::YawRateState:
                cluster.YawRateStatePD = pd;
                break;
            case OscillationPairingKind::YawRateGyro:
                cluster.YawRateGyroPD = pd;
                break;
            case OscillationPairingKind::LongitudinalAccelerationState:
                cluster.LongitudinalAccelerationStatePD = pd;
                break;
            case OscillationPairingKind::LongitudinalAccelerationImuForwardAccel:
                cluster.LongitudinalAccelerationIMUForwardAccelPD = pd;
                break;
            case OscillationPairingKind::WheelVelocityState:
                cluster.WheelVelocityStatePD = pd;
                break;
            case OscillationPairingKind::WheelVelocityEncoder:
                cluster.WheelVelocityEncoderPD = pd;
                break;
            default:
                break;
            }

            return cluster;
        }

        OscillationGeneratedCommand GenerateOscillationPairingCommand(
            DriveBase& drive,
            const Estimator& estimator,
            const OscillationPairingKind pairingKind,
            const float setpoint)
        {
            OscillationGeneratedCommand generated{};
            const PlantModel::StateVector state = estimator.StateVector();
            const float presentLinearSpeedMps = state(VehicleState::kU);
            const float presentYawRateRadps = state(VehicleState::kR);
            const float wheelRadiusM = estimator.ukf().preparedParams().wheelRadiusM;

            switch (pairingKind)
            {
            case OscillationPairingKind::HeadingState:
                generated.command =
                    drive.PointCommandWithHeadingTarget(
                        0.0f,
                        0.0f,
                        setpoint,
                        MazeMap::CommandPD::RawCommand,
                        MazeMap::CommandPD::StateHeadingPD);
                generated.targetSignal = setpoint;
                break;
            case OscillationPairingKind::VelocityState:
                generated.command =
                    drive.PointCommand(
                        setpoint,
                        MazeMap::CommandPD::StateVelocityPD);
                generated.linearCommandTargetMps = setpoint;
                generated.targetSignal = setpoint;
                break;
            case OscillationPairingKind::YawRateState:
                generated.command =
                    drive.PointYawRateCommand(
                        setpoint,
                        MazeMap::CommandPD::StateYawPD);
                generated.linearCommandTargetMps = presentLinearSpeedMps;
                generated.angularCommandTargetRadps = setpoint;
                generated.targetSignal = setpoint;
                break;
            case OscillationPairingKind::YawRateGyro:
                generated.command =
                    drive.PointYawRateCommand(
                        setpoint,
                        MazeMap::CommandPD::IMUYaw);
                generated.linearCommandTargetMps = presentLinearSpeedMps;
                generated.angularCommandTargetRadps = setpoint;
                generated.targetSignal = setpoint;
                break;
            case OscillationPairingKind::LongitudinalAccelerationState:
                generated.command =
                    drive.DeltaCommand(
                        presentLinearSpeedMps,
                        setpoint,
                        MazeMap::CommandPD::StateAccelerationPD);
                generated.linearCommandTargetMps = presentLinearSpeedMps;
                generated.angularCommandTargetRadps = presentYawRateRadps;
                generated.targetSignal = setpoint;
                break;
            case OscillationPairingKind::LongitudinalAccelerationImuForwardAccel:
                generated.command =
                    drive.DeltaCommand(
                        presentLinearSpeedMps,
                        setpoint,
                        MazeMap::CommandPD::IMUForwardAccel);
                generated.linearCommandTargetMps = presentLinearSpeedMps;
                generated.angularCommandTargetRadps = presentYawRateRadps;
                generated.targetSignal = setpoint;
                break;
            case OscillationPairingKind::WheelVelocityState:
            {
                const float linearSpeedTargetMps = setpoint * wheelRadiusM;
                generated.command =
                    drive.PointCommand(
                        linearSpeedTargetMps,
                        MazeMap::CommandPD::StateWheelOmegaPD);
                generated.linearCommandTargetMps = linearSpeedTargetMps;
                generated.targetSignal = setpoint;
                break;
            }
            case OscillationPairingKind::WheelVelocityEncoder:
                generated.command =
                    drive.PointCommand(
                        setpoint,
                        MazeMap::CommandPD::EncoderVelocity);
                generated.linearCommandTargetMps = setpoint;
                generated.targetSignal = setpoint;
                break;
            default:
                break;
            }

            return generated;
        }

        float ResolveOscillationTruthSignal(
            const MazeMap::CommandPD signalSource,
            const PlantModel::StateVector& truthState,
            const MazeMap::PlantDerivatives& truthDerivatives) noexcept
        {
            switch (signalSource)
            {
            case MazeMap::CommandPD::StateHeadingPD:
                return WrapAngleRad(truthState(VehicleState::kPsi));
            case MazeMap::CommandPD::StateYawPD:
            case MazeMap::CommandPD::IMUYaw:
                return truthState(VehicleState::kR);
            case MazeMap::CommandPD::StateWheelOmegaPD:
                return 0.5f * (truthState(VehicleState::kOmegaL) + truthState(VehicleState::kOmegaR));
            case MazeMap::CommandPD::StateAccelerationPD:
                return truthDerivatives.longitudinalAccelMps2;
            case MazeMap::CommandPD::IMUForwardAccel:
                return truthDerivatives.imuAccelBodyMps2.y();
            case MazeMap::CommandPD::IMULateralAccel:
                return truthDerivatives.imuAccelBodyMps2.x();
            case MazeMap::CommandPD::StateVelocityPD:
            case MazeMap::CommandPD::EncoderVelocity:
            default:
                return truthState(VehicleState::kU);
            }
        }

        float ResolveOscillationEstimatorSignal(
            const MazeMap::CommandPD signalSource,
            const PlantModel::StateVector& estimatorState,
            const MazeMap::PlantDerivatives& estimatorDerivatives) noexcept
        {
            switch (signalSource)
            {
            case MazeMap::CommandPD::StateHeadingPD:
                return WrapAngleRad(estimatorState(VehicleState::kPsi));
            case MazeMap::CommandPD::StateYawPD:
            case MazeMap::CommandPD::IMUYaw:
                return estimatorState(VehicleState::kR);
            case MazeMap::CommandPD::StateWheelOmegaPD:
                return 0.5f * (estimatorState(VehicleState::kOmegaL) + estimatorState(VehicleState::kOmegaR));
            case MazeMap::CommandPD::StateAccelerationPD:
                return estimatorDerivatives.longitudinalAccelMps2;
            case MazeMap::CommandPD::IMUForwardAccel:
                return estimatorDerivatives.imuAccelBodyMps2.y();
            case MazeMap::CommandPD::IMULateralAccel:
                return estimatorDerivatives.imuAccelBodyMps2.x();
            case MazeMap::CommandPD::StateVelocityPD:
            case MazeMap::CommandPD::EncoderVelocity:
            default:
                return estimatorState(VehicleState::kU);
            }
        }

        float ResolveOscillationSourceSignal(
            const MazeMap::CommandPD signalSource,
            const SensorSnapshot& snapshot,
            const DriveTelemetry& telemetry,
            const PlantModel::StateVector& estimatorState,
            const MazeMap::PlantDerivatives& estimatorDerivatives) noexcept
        {
            switch (signalSource)
            {
            case MazeMap::CommandPD::StateHeadingPD:
                return WrapAngleRad(estimatorState(VehicleState::kPsi));
            case MazeMap::CommandPD::StateYawPD:
                return estimatorState(VehicleState::kR);
            case MazeMap::CommandPD::StateWheelOmegaPD:
                return 0.5f * (telemetry.leftEncoderOmegaRadps + telemetry.rightEncoderOmegaRadps);
            case MazeMap::CommandPD::StateAccelerationPD:
                return estimatorDerivatives.longitudinalAccelMps2;
            case MazeMap::CommandPD::EncoderVelocity:
                return 0.5f * (telemetry.leftVelocityMps + telemetry.rightVelocityMps);
            case MazeMap::CommandPD::IMUYaw:
                return snapshot.gyroRawRadps;
            case MazeMap::CommandPD::IMUForwardAccel:
                return snapshot.accelBodyYMps2;
            case MazeMap::CommandPD::IMULateralAccel:
                return snapshot.accelBodyXMps2;
            case MazeMap::CommandPD::StateVelocityPD:
            default:
                return estimatorState(VehicleState::kU);
            }
        }

        float ResolveOscillationFeedbackComponent(
            const MazeMap::CommandPD signalSource,
            const DriveTelemetry& telemetry) noexcept
        {
            return
                UsesAngularFeedbackComponent(signalSource) ?
                (0.5f * (telemetry.leftFeedbackCommand - telemetry.rightFeedbackCommand)) :
                (0.5f * (telemetry.leftFeedbackCommand + telemetry.rightFeedbackCommand));
        }

        float ComputeOscillationTrackingError(
            const MazeMap::CommandPD signalSource,
            const float targetSignal,
            const float actualSignal) noexcept
        {
            if (signalSource == MazeMap::CommandPD::StateHeadingPD)
            {
                return
                    HeadingErrorRad(
                        HeadingUnitFromYawRad(targetSignal),
                        HeadingUnitFromYawRad(actualSignal));
            }

            return targetSignal - actualSignal;
        }

        float ResolveOscillationScenarioSetpoint(
            const OscillationPairingScenario& scenario,
            const int stepIndex) noexcept
        {
            return
                (stepIndex >= scenario.stepStartStep) ?
                scenario.stepSetpoint :
                scenario.initialSetpoint;
        }

        float ResolveOscillationScenarioReferenceMagnitude(
            const OscillationPairingScenario& scenario) noexcept
        {
            const float initialMagnitude = std::fabs(scenario.initialSetpoint);
            const float stepMagnitude = std::fabs(scenario.stepSetpoint);
            const float stepDeltaMagnitude =
                std::fabs(scenario.stepSetpoint - scenario.initialSetpoint);
            return
                (std::max)(
                    (std::max)(initialMagnitude, stepMagnitude),
                    stepDeltaMagnitude);
        }

        float ResolveOscillationJitterThreshold(
            const MazeMap::CommandPD signalSource) noexcept
        {
            switch (signalSource)
            {
            case MazeMap::CommandPD::StateHeadingPD:
                return 0.0020f;
            case MazeMap::CommandPD::StateYawPD:
            case MazeMap::CommandPD::IMUYaw:
                return 0.0020f;
            case MazeMap::CommandPD::StateWheelOmegaPD:
                return 0.0200f;
            case MazeMap::CommandPD::StateAccelerationPD:
            case MazeMap::CommandPD::IMUForwardAccel:
            case MazeMap::CommandPD::IMULateralAccel:
                return 0.0200f;
            case MazeMap::CommandPD::StateVelocityPD:
            case MazeMap::CommandPD::EncoderVelocity:
            default:
                return 0.0004f;
            }
        }

        float ResolveOscillationSteadyErrorThreshold(
            const MazeMap::CommandPD signalSource,
            const float setpoint) noexcept
        {
            switch (signalSource)
            {
            case MazeMap::CommandPD::StateHeadingPD:
                return (std::max)(0.15f, 0.50f * std::fabs(setpoint));
            case MazeMap::CommandPD::StateYawPD:
            case MazeMap::CommandPD::IMUYaw:
                return (std::max)(0.15f, 0.25f * std::fabs(setpoint));
            case MazeMap::CommandPD::StateWheelOmegaPD:
                return (std::max)(0.50f, 0.25f * std::fabs(setpoint));
            case MazeMap::CommandPD::StateAccelerationPD:
            case MazeMap::CommandPD::IMUForwardAccel:
            case MazeMap::CommandPD::IMULateralAccel:
                return (std::max)(0.15f, 0.50f * std::fabs(setpoint));
            case MazeMap::CommandPD::StateVelocityPD:
            case MazeMap::CommandPD::EncoderVelocity:
            default:
                return (std::max)(0.15f, 0.25f * std::fabs(setpoint));
            }
        }

        OscillationPairingScenario BuildStepOscillationPairingScenario(
            const MazeMap::CommandPD signalSource) noexcept
        {
            OscillationPairingScenario scenario{};
            switch (signalSource)
            {
            case MazeMap::CommandPD::StateHeadingPD:
                scenario.stepSetpoint = 0.15f;
                scenario.totalSteps = 520;
                scenario.steadyWindowStartStep = 360;
                break;
            case MazeMap::CommandPD::StateYawPD:
            case MazeMap::CommandPD::IMUYaw:
                scenario.stepSetpoint = 0.35f;
                scenario.totalSteps = 520;
                scenario.steadyWindowStartStep = 360;
                break;
            case MazeMap::CommandPD::StateWheelOmegaPD:
                scenario.stepSetpoint = 4.0f;
                scenario.totalSteps = 520;
                scenario.steadyWindowStartStep = 360;
                break;
            case MazeMap::CommandPD::StateAccelerationPD:
            case MazeMap::CommandPD::IMUForwardAccel:
                scenario.stepSetpoint = 0.75f;
                scenario.stepStartStep = 30;
                scenario.totalSteps = 460;
                scenario.steadyWindowStartStep = 300;
                break;
            case MazeMap::CommandPD::IMULateralAccel:
                scenario.stepSetpoint = 0.30f;
                scenario.initialForwardSpeedMps = 0.20f;
                scenario.totalSteps = 520;
                scenario.steadyWindowStartStep = 360;
                break;
            case MazeMap::CommandPD::StateVelocityPD:
            case MazeMap::CommandPD::EncoderVelocity:
            default:
                scenario.stepSetpoint = 0.20f;
                scenario.totalSteps = 840;
                scenario.steadyWindowStartStep = 640;
                break;
            }

            return scenario;
        }

        void PrimeOscillationPairingInitialState(
            DriveBase& drive,
            Estimator& estimator,
            PlantModel& plant,
            const OscillationPairingScenario& scenario,
            PlantModel::StateVector& truthState,
            float& leftEncoderRemainderCounts,
            float& rightEncoderRemainderCounts)
        {
            if ((std::fabs(scenario.initialForwardSpeedMps) <= 1.0e-6f) &&
                (std::fabs(scenario.initialYawRateRadps) <= 1.0e-6f))
            {
                return;
            }

            const PlantParams& params = PlantParams::Default();
            const PlantModel::PreparedParams prepared = PlantModel::Prepare(params);
            float leftTargetVelocityMps = 0.0f;
            float rightTargetVelocityMps = 0.0f;
            float unusedLeftTargetAccelMps2 = 0.0f;
            float unusedRightTargetAccelMps2 = 0.0f;
            float leftTargetOmegaRadps = 0.0f;
            float rightTargetOmegaRadps = 0.0f;
            plant.resolveWheelMotionTargets(
                scenario.initialForwardSpeedMps,
                scenario.initialYawRateRadps,
                0.0f,
                0.0f,
                prepared,
                leftTargetVelocityMps,
                rightTargetVelocityMps,
                unusedLeftTargetAccelMps2,
                unusedRightTargetAccelMps2,
                leftTargetOmegaRadps,
                rightTargetOmegaRadps);

            truthState(VehicleState::kU) = scenario.initialForwardSpeedMps;
            truthState(VehicleState::kR) = scenario.initialYawRateRadps;
            truthState(VehicleState::kOmegaL) = leftTargetOmegaRadps;
            truthState(VehicleState::kOmegaR) = rightTargetOmegaRadps;
            VehicleState::NormalizeStateVector(truthState);

            const float distancePerCountM = DistancePerEncoderCountMeters(params);
            const int32_t leftCounts =
                ConsumeWholeEncoderCounts(
                    (leftTargetVelocityMps * scenario.dtSeconds) / distancePerCountM,
                    leftEncoderRemainderCounts);
            const int32_t rightCounts =
                ConsumeWholeEncoderCounts(
                    (rightTargetVelocityMps * scenario.dtSeconds) / distancePerCountM,
                    rightEncoderRemainderCounts);
            UpdateDriveBaseSignals(
                drive,
                estimator,
                BuildDriveBaseSensorSnapshot(
                    scenario.initialYawRateRadps,
                    scenario.initialYawRateRadps,
                    0.0f,
                    0.0f,
                    false),
                leftCounts,
                rightCounts,
                scenario.dtSeconds);
            drive.ProjectMeasuredKinematics(0.0f, scenario.initialYawRateRadps);
        }

        std::vector<OscillationPairingTraceSample> RunOscillationPairingTrace(
            const MazeMap::ProportionalDerivative& pd,
            const OscillationPairingKind pairingKind,
            const OscillationPairingScenario& scenario)
        {
            const MazeMap::CommandPD signalSource =
                ResolveOscillationPairingSignalSource(pairingKind);
            PlantModel plant;
            const MazeMap::ProportionalDerivativeCluster pairingCluster =
                BuildOscillationPairingCluster(pd, pairingKind);
            DriveBaseHarness driveHarness(plant, pairingCluster);
            DriveBase& drive = driveHarness.drive;
            Assert::IsTrue(drive.Begin());
            drive.SetPose(0.0f, 0.0f, 0.0f);
            UpdateDriveBaseSignals(drive, driveHarness.estimator, BuildDriveBaseSensorSnapshot(0.0f));

            PlantModel::StateVector truthState = PlantModel::StateVector::Zero();
            float leftEncoderRemainderCounts = 0.0f;
            float rightEncoderRemainderCounts = 0.0f;
            const PlantParams& params = PlantParams::Default();
            PrimeOscillationPairingInitialState(
                drive,
                driveHarness.estimator,
                plant,
                scenario,
                truthState,
                leftEncoderRemainderCounts,
                rightEncoderRemainderCounts);

            std::vector<OscillationPairingTraceSample> samples;
            samples.reserve(static_cast<std::size_t>((std::max)(scenario.totalSteps, 0)));

            for (int stepIndex = 0; stepIndex < scenario.totalSteps; ++stepIndex)
            {
                const float setpoint =
                    ResolveOscillationScenarioSetpoint(scenario, stepIndex);
                const OscillationGeneratedCommand generatedCommand =
                    GenerateOscillationPairingCommand(
                        drive,
                        driveHarness.estimator,
                        pairingKind,
                        setpoint);
                drive.CommandGenerated(
                    generatedCommand.command,
                    generatedCommand.linearCommandTargetMps,
                    generatedCommand.angularCommandTargetRadps,
                    true);

                const DriveTelemetry appliedTelemetry = drive.GetTelemetry();
                ControlInput control{};
                control.leftMotorCommand = appliedTelemetry.leftDriveCommand;
                control.rightMotorCommand = appliedTelemetry.rightDriveCommand;
                control.fanDutyCycle = 0.80f;
                control.batteryVoltageV = params.supplyVoltageV;

                const PlantModel::StateVector previousTruthState = truthState;
                truthState = plant.integrate(truthState, control, scenario.dtSeconds, params);
                const MazeMap::PlantDerivatives truthDerivatives =
                    plant.forwardStep(truthState, control, params);

                const float leftDistanceDeltaM =
                    0.5f *
                    (previousTruthState(VehicleState::kOmegaL) + truthState(VehicleState::kOmegaL)) *
                    params.wheelRadiusM *
                    scenario.dtSeconds;
                const float rightDistanceDeltaM =
                    0.5f *
                    (previousTruthState(VehicleState::kOmegaR) + truthState(VehicleState::kOmegaR)) *
                    params.wheelRadiusM *
                    scenario.dtSeconds;
                const float distancePerCountM = DistancePerEncoderCountMeters(params);
                const int32_t leftCounts =
                    ConsumeWholeEncoderCounts(leftDistanceDeltaM / distancePerCountM, leftEncoderRemainderCounts);
                const int32_t rightCounts =
                    ConsumeWholeEncoderCounts(rightDistanceDeltaM / distancePerCountM, rightEncoderRemainderCounts);
                const int encoderCountDisturbance =
                    ResolveAlternatingCountDisturbance(
                        stepIndex + 1,
                        scenario.disturbanceStartStep,
                        scenario.disturbanceHalfPeriodSteps,
                        scenario.encoderCountDisturbanceCounts);
                const float gyroDisturbanceRadps =
                    ResolveAlternatingDisturbance(
                        stepIndex + 1,
                        scenario.disturbanceStartStep,
                        scenario.disturbanceHalfPeriodSteps,
                        scenario.gyroDisturbanceRadps);
                const float accelBodyXDisturbanceMps2 =
                    ResolveAlternatingDisturbance(
                        stepIndex + 1,
                        scenario.disturbanceStartStep,
                        scenario.disturbanceHalfPeriodSteps,
                        scenario.accelBodyXDisturbanceMps2);
                const float accelBodyYDisturbanceMps2 =
                    ResolveAlternatingDisturbance(
                        stepIndex + 1,
                        scenario.disturbanceStartStep,
                        scenario.disturbanceHalfPeriodSteps,
                        scenario.accelBodyYDisturbanceMps2);

                const SensorSnapshot snapshot =
                    BuildDriveBaseSensorSnapshot(
                        truthState(VehicleState::kR) + gyroDisturbanceRadps,
                        truthState(VehicleState::kR) + gyroDisturbanceRadps,
                        truthDerivatives.imuAccelBodyMps2.x() + accelBodyXDisturbanceMps2,
                        truthDerivatives.imuAccelBodyMps2.y() + accelBodyYDisturbanceMps2,
                        true);
                UpdateDriveBaseSignals(
                    drive,
                    driveHarness.estimator,
                    snapshot,
                    leftCounts + encoderCountDisturbance,
                    rightCounts + encoderCountDisturbance,
                    scenario.dtSeconds);

                const DriveTelemetry updatedTelemetry = drive.GetTelemetry();
                const PlantModel::StateVector estimatorState = driveHarness.estimator.StateVector();
                const MazeMap::PlantDerivatives estimatorDerivatives =
                    plant.forwardStep(estimatorState, control, params);

                OscillationPairingTraceSample sample{};
                sample.targetSignal = generatedCommand.targetSignal;
                sample.sourceSignal =
                    ResolveOscillationSourceSignal(
                        signalSource,
                        snapshot,
                        updatedTelemetry,
                        estimatorState,
                        estimatorDerivatives);
                sample.estimatorSignal =
                    ResolveOscillationEstimatorSignal(
                        signalSource,
                        estimatorState,
                        estimatorDerivatives);
                sample.truthSignal =
                    ResolveOscillationTruthSignal(
                        signalSource,
                        truthState,
                        truthDerivatives);
                sample.feedbackCommandComponent =
                    ResolveOscillationFeedbackComponent(signalSource, updatedTelemetry);
                sample.saturationFlags = updatedTelemetry.saturationFlags;
                samples.push_back(sample);
            }

            return samples;
        }

        OscillationPairingAuditResult AuditOscillationPairingAgainstPlantAndSrUkf(
            const MazeMap::ProportionalDerivative& pd,
            const OscillationPairingKind pairingKind,
            const OscillationPairingScenario& scenario)
        {
            const MazeMap::CommandPD signalSource =
                ResolveOscillationPairingSignalSource(pairingKind);
            const std::vector<OscillationPairingTraceSample> samples =
                RunOscillationPairingTrace(pd, pairingKind, scenario);

            OscillationPairingAuditResult result{};
            if (samples.empty())
            {
                return result;
            }

            const std::size_t steadyStartIndex =
                static_cast<std::size_t>((std::clamp)(
                    scenario.steadyWindowStartStep,
                    0,
                    static_cast<int>(samples.size())));

            std::vector<float> sourceErrors;
            std::vector<float> estimatorErrors;
            std::vector<float> truthErrors;
            std::vector<float> feedbackComponents;
            std::vector<std::uint16_t> saturationFlags;
            sourceErrors.reserve(samples.size());
            estimatorErrors.reserve(samples.size());
            truthErrors.reserve(samples.size());
            feedbackComponents.reserve(samples.size());
            saturationFlags.reserve(samples.size());

            for (const OscillationPairingTraceSample& sample : samples)
            {
                const float sourceError =
                    ComputeOscillationTrackingError(
                        signalSource,
                        sample.targetSignal,
                        sample.sourceSignal);
                const float estimatorError =
                    ComputeOscillationTrackingError(
                        signalSource,
                        sample.targetSignal,
                        sample.estimatorSignal);
                const float truthError =
                    ComputeOscillationTrackingError(
                        signalSource,
                        sample.targetSignal,
                        sample.truthSignal);
                sourceErrors.push_back(sourceError);
                estimatorErrors.push_back(estimatorError);
                truthErrors.push_back(truthError);
                feedbackComponents.push_back(sample.feedbackCommandComponent);
                saturationFlags.push_back(sample.saturationFlags);
            }

            result.steadySourceErrorHighPassRms =
                ComputeHighPassRms(sourceErrors, steadyStartIndex);
            result.steadyEstimatorErrorHighPassRms =
                ComputeHighPassRms(estimatorErrors, steadyStartIndex);
            result.steadyTruthErrorHighPassRms =
                ComputeHighPassRms(truthErrors, steadyStartIndex);
            result.steadySourceAbsError =
                ComputeMeanAbsoluteValue(sourceErrors, steadyStartIndex);
            result.steadyEstimatorAbsError =
                ComputeMeanAbsoluteValue(estimatorErrors, steadyStartIndex);
            result.steadyTruthAbsError =
                ComputeMeanAbsoluteValue(truthErrors, steadyStartIndex);
            result.steadyFeedbackComponentRms =
                ComputeHighPassRms(feedbackComponents, steadyStartIndex);
            result.steadySignFlipRate =
                ComputeSignFlipRate(sourceErrors, steadyStartIndex, kOscillationTraceDeadbandMps);
            {
                std::vector<bool> saturationSamples;
                saturationSamples.reserve(samples.size());
                for (const std::uint16_t flags : saturationFlags)
                {
                    saturationSamples.push_back(flags != 0U);
                }
                result.fullSaturationDuty = ComputeSaturationDuty(saturationSamples, 0U);
                result.steadySaturationDuty = ComputeSaturationDuty(saturationSamples, steadyStartIndex);
            }

            const AlternatingSaturationPatternResult saturationPattern =
                AnalyzeAlternatingSaturationPattern(saturationFlags, steadyStartIndex);
            result.steadySaturatedRunCount = saturationPattern.saturatedRunCount;
            result.steadyAlternatingSaturationCount = saturationPattern.alternationCount;
            result.steadyAlternatingSaturationPatternDetected = saturationPattern.patternDetected;

            return result;
        }

        std::wstring BuildOscillationPairingAuditMessage(
            const OscillationPairingAuditResult& result)
        {
            return
                std::wstring(L"steady_source_hp_rms=") + std::to_wstring(result.steadySourceErrorHighPassRms) +
                L" steady_estimator_hp_rms=" + std::to_wstring(result.steadyEstimatorErrorHighPassRms) +
                L" steady_truth_hp_rms=" + std::to_wstring(result.steadyTruthErrorHighPassRms) +
                L" steady_source_abs_err=" + std::to_wstring(result.steadySourceAbsError) +
                L" steady_estimator_abs_err=" + std::to_wstring(result.steadyEstimatorAbsError) +
                L" steady_truth_abs_err=" + std::to_wstring(result.steadyTruthAbsError) +
                L" steady_feedback_component_rms=" + std::to_wstring(result.steadyFeedbackComponentRms) +
                L" steady_sign_flip_rate=" + std::to_wstring(result.steadySignFlipRate) +
                L" full_sat_duty=" + std::to_wstring(result.fullSaturationDuty) +
                L" steady_sat_duty=" + std::to_wstring(result.steadySaturationDuty) +
                L" steady_sat_runs=" + std::to_wstring(static_cast<unsigned long long>(result.steadySaturatedRunCount)) +
                L" steady_sat_alternations=" + std::to_wstring(static_cast<unsigned long long>(result.steadyAlternatingSaturationCount)) +
                L" steady_alt_sat_pattern=" + std::to_wstring(result.steadyAlternatingSaturationPatternDetected ? 1 : 0);
        }

        std::wstring BuildOscillationPairingStepMessage(
            const OscillationPairingScenario& scenario,
            const OscillationPairingAuditResult& audit)
        {
            return
                BuildOscillationPairingAuditMessage(audit) +
                L" initial_setpoint=" + std::to_wstring(scenario.initialSetpoint) +
                L" step_setpoint=" + std::to_wstring(scenario.stepSetpoint) +
                L" step_start=" + std::to_wstring(scenario.stepStartStep) +
                L" steady_start=" + std::to_wstring(scenario.steadyWindowStartStep);
        }

        bool OscillationPairingAuditHasFiniteMetrics(
            const OscillationPairingAuditResult& audit) noexcept
        {
            return
                std::isfinite(audit.steadySourceErrorHighPassRms) &&
                std::isfinite(audit.steadyEstimatorErrorHighPassRms) &&
                std::isfinite(audit.steadyTruthErrorHighPassRms) &&
                std::isfinite(audit.steadySourceAbsError) &&
                std::isfinite(audit.steadyEstimatorAbsError) &&
                std::isfinite(audit.steadyTruthAbsError) &&
                std::isfinite(audit.steadyFeedbackComponentRms) &&
                std::isfinite(audit.steadySignFlipRate) &&
                std::isfinite(audit.fullSaturationDuty) &&
                std::isfinite(audit.steadySaturationDuty);
        }

        void AssertOscillationPairingMetricsFinite(
            const MazeMap::ProportionalDerivative& pd,
            const OscillationPairingKind pairingKind)
        {
            const MazeMap::CommandPD signalSource =
                ResolveOscillationPairingSignalSource(pairingKind);
            const OscillationPairingScenario scenario =
                BuildStepOscillationPairingScenario(signalSource);
            const OscillationPairingAuditResult audit =
                AuditOscillationPairingAgainstPlantAndSrUkf(
                    pd,
                    pairingKind,
                    scenario);
            const std::wstring message =
                BuildOscillationPairingStepMessage(scenario, audit);

            Assert::IsTrue(
                OscillationPairingAuditHasFiniteMetrics(audit),
                message.c_str());
        }

        void AssertOscillationPairingSourceJitterLow(
            const MazeMap::ProportionalDerivative& pd,
            const OscillationPairingKind pairingKind)
        {
            const MazeMap::CommandPD signalSource =
                ResolveOscillationPairingSignalSource(pairingKind);
            const OscillationPairingScenario scenario =
                BuildStepOscillationPairingScenario(signalSource);
            const OscillationPairingAuditResult audit =
                AuditOscillationPairingAgainstPlantAndSrUkf(
                    pd,
                    pairingKind,
                    scenario);
            const float jitterThreshold = ResolveOscillationJitterThreshold(signalSource);
            const std::wstring message =
                BuildOscillationPairingStepMessage(scenario, audit) +
                L" jitter_threshold=" + std::to_wstring(jitterThreshold);

            if (!std::isfinite(audit.steadySourceErrorHighPassRms))
            {
                return;
            }

            Assert::IsTrue(
                audit.steadySourceErrorHighPassRms <= jitterThreshold,
                message.c_str());
        }

        void AssertOscillationPairingEstimatorTrackingAccurate(
            const MazeMap::ProportionalDerivative& pd,
            const OscillationPairingKind pairingKind)
        {
            const MazeMap::CommandPD signalSource =
                ResolveOscillationPairingSignalSource(pairingKind);
            const OscillationPairingScenario scenario =
                BuildStepOscillationPairingScenario(signalSource);
            const OscillationPairingAuditResult audit =
                AuditOscillationPairingAgainstPlantAndSrUkf(
                    pd,
                    pairingKind,
                    scenario);
            const float steadyErrorThreshold =
                ResolveOscillationSteadyErrorThreshold(
                    signalSource,
                    ResolveOscillationScenarioReferenceMagnitude(scenario));
            const std::wstring message =
                BuildOscillationPairingStepMessage(scenario, audit) +
                L" steady_error_threshold=" + std::to_wstring(steadyErrorThreshold);

            if (!std::isfinite(audit.steadyEstimatorAbsError))
            {
                return;
            }

            Assert::IsTrue(audit.steadyEstimatorAbsError <= steadyErrorThreshold, message.c_str());
        }

        void AssertOscillationPairingPlantTrackingAccurate(
            const MazeMap::ProportionalDerivative& pd,
            const OscillationPairingKind pairingKind)
        {
            const MazeMap::CommandPD signalSource =
                ResolveOscillationPairingSignalSource(pairingKind);
            const OscillationPairingScenario scenario =
                BuildStepOscillationPairingScenario(signalSource);
            const OscillationPairingAuditResult audit =
                AuditOscillationPairingAgainstPlantAndSrUkf(
                    pd,
                    pairingKind,
                    scenario);
            const float steadyErrorThreshold =
                ResolveOscillationSteadyErrorThreshold(
                    signalSource,
                    ResolveOscillationScenarioReferenceMagnitude(scenario));
            const std::wstring message =
                BuildOscillationPairingStepMessage(scenario, audit) +
                L" steady_error_threshold=" + std::to_wstring(steadyErrorThreshold);

            if (!std::isfinite(audit.steadyTruthAbsError))
            {
                return;
            }

            Assert::IsTrue(audit.steadyTruthAbsError <= steadyErrorThreshold, message.c_str());
        }

        void AssertOscillationPairingHasNoAlternatingSaturation(
            const MazeMap::ProportionalDerivative& pd,
            const OscillationPairingKind pairingKind)
        {
            const MazeMap::CommandPD signalSource =
                ResolveOscillationPairingSignalSource(pairingKind);
            const OscillationPairingScenario scenario =
                BuildStepOscillationPairingScenario(signalSource);
            const OscillationPairingAuditResult audit =
                AuditOscillationPairingAgainstPlantAndSrUkf(
                    pd,
                    pairingKind,
                    scenario);
            const std::wstring message =
                BuildOscillationPairingStepMessage(scenario, audit);

            Assert::IsFalse(audit.steadyAlternatingSaturationPatternDetected, message.c_str());
        }

    }

    TEST_CLASS(DriveBaseTest)
    {
    public:
        TEST_METHOD(MotionLimitsProvideDefaultTurnEnvelopeAndBoundaryReachability)
        {
            MotionLimits limits{};
            limits.maxSpeedMps = 1.0f;
            limits.accelMps2 = 2.0f;
            limits.decelMps2 = 3.0f;
            limits.maxAngularSpeedRadps = 9.0f;
            limits.angularAccelRadps2 = 45.0f;

            Assert::AreEqual(0.75f * DEG_TO_RAD_F, limits.angleToleranceRad, 1.0e-6f);
            Assert::AreEqual(0.10f, limits.angularSpeedToleranceRadps, 1.0e-6f);

            const float brakingRateRadps = ReachableSpeedWithBoundary(0.0f, 0.01f, limits.angularAccelRadps2);
            Assert::AreEqual(
                sqrtf(2.0f * limits.angularAccelRadps2 * 0.01f),
                brakingRateRadps,
                1.0e-6f);
        }

        TEST_METHOD(DriveBaseDeltaCommandStaysSymmetricAcrossWheelSpeedMismatch)
        {
            PlantModel plant;
            DriveBaseHarness driveHarness(plant, Config::kDriveBasePDCluster);
            DriveBase& drive = driveHarness.drive;
            Assert::IsTrue(drive.Begin());

            PrimeDriveBaseWithEncoderDelta(drive, driveHarness.estimator, 6, 42);

            const ControlVector command =
                drive.DeltaCommand(
                    0.20f,
                    8.5f);

            Assert::IsTrue(IsFiniteControlVector(command));
            Assert::IsTrue(command.leftMotorPwm > 0.0f);
            Assert::AreEqual(command.leftMotorPwm, command.rightMotorPwm, 1.0e-6f);
        }

        TEST_METHOD(DriveBaseDeltaCommandHeadingHoldStaysSymmetricWhenAlreadyAligned)
        {
            PlantModel plant;
            DriveBaseHarness driveHarness(plant, Config::kDriveBasePDCluster);
            DriveBase& drive = driveHarness.drive;
            Assert::IsTrue(drive.Begin());

            PrimeDriveBaseWithEncoderDelta(drive, driveHarness.estimator, 6, 42);

            const ControlVector command =
                drive.DeltaCommand(
                    0.20f,
                    8.5f,
                    MazeMap::CommandPD::StateHeadingPD);

            Assert::IsTrue(IsFiniteControlVector(command));
            Assert::AreEqual(command.leftMotorPwm, command.rightMotorPwm, 1.0e-3f);
        }

        TEST_METHOD(DriveBaseDeltaCommandRawMatchesPlantFeedforwardAtSteadyForwardTarget)
        {
            PlantModel plant;
            DriveBaseHarness driveHarness(plant, Config::kDriveBasePDCluster);
            DriveBase& drive = driveHarness.drive;
            Assert::IsTrue(drive.Begin());
            drive.SetPose(0.0f, 0.0f, 0.0f);
            const PlantParams params = PlantParams::Default();

            const ControlVector command =
                drive.DeltaCommand(
                    0.20f,
                    0.0f,
                    MazeMap::CommandPD::RawCommand);
            const DriveCommandSolution solution =
                plant.solveDriveCommandsForVelocityTarget(
                    0.20f,
                    0.20f,
                    0.0f,
                    0.0f,
                    params,
                    0.80f,
                    params.supplyVoltageV,
                    PlantModel::kDefaultVelocityTargetResponseTimeS);

            AssertDriveCommandMatchesSolution(command, solution);
        }

        TEST_METHOD(DriveBaseDeltaCommandCombinedRawMatchesPlantFeedforwardAtSteadyTarget)
        {
            PlantModel plant;
            DriveBaseHarness driveHarness(plant, Config::kDriveBasePDCluster);
            DriveBase& drive = driveHarness.drive;
            Assert::IsTrue(drive.Begin());
            drive.SetPose(0.0f, 0.0f, 0.0f);
            const PlantParams params = PlantParams::Default();

            const ControlVector command =
                drive.DeltaCommand(
                    0.20f,
                    0.0f,
                    0.40f,
                    0.0f,
                    MazeMap::CommandPD::RawCommand);
            const DriveCommandSolution solution =
                plant.solveDriveCommandsForVelocityTarget(
                    0.20f,
                    0.20f,
                    0.40f,
                    0.40f,
                    params,
                    0.80f,
                    params.supplyVoltageV,
                    PlantModel::kDefaultVelocityTargetResponseTimeS);

            AssertDriveCommandMatchesSolution(command, solution);
        }

        TEST_METHOD(DriveBaseDeltaYawRateCommandRawMatchesPlantFeedforwardAtSteadyYawRateTarget)
        {
            PlantModel plant;
            DriveBaseHarness driveHarness(plant, Config::kDriveBasePDCluster);
            DriveBase& drive = driveHarness.drive;
            Assert::IsTrue(drive.Begin());
            drive.SetPose(0.0f, 0.0f, 0.0f);
            const PlantParams params = PlantParams::Default();

            const ControlVector command =
                drive.DeltaYawRateCommand(
                    0.40f,
                    0.0f,
                    MazeMap::CommandPD::RawCommand);
            const DriveCommandSolution solution =
                plant.solveDriveCommandsForVelocityTarget(
                    0.0f,
                    0.0f,
                    0.40f,
                    0.40f,
                    params,
                    0.80f,
                    params.supplyVoltageV,
                    PlantModel::kDefaultVelocityTargetResponseTimeS);

            AssertDriveCommandMatchesSolution(command, solution);
        }

        TEST_METHOD(DriveBasePointCommandRawMatchesPlantVelocityTargetFeedforwardAtForwardTarget)
        {
            PlantModel plant;
            DriveBaseHarness driveHarness(plant, Config::kDriveBasePDCluster);
            DriveBase& drive = driveHarness.drive;
            Assert::IsTrue(drive.Begin());
            drive.SetPose(0.0f, 0.0f, 0.0f);
            const PlantParams params = PlantParams::Default();

            const ControlVector command =
                drive.PointCommand(
                    0.20f,
                    MazeMap::CommandPD::RawCommand);
            const DriveCommandSolution solution =
                plant.solveDriveCommandsForVelocityTarget(
                    driveHarness.estimator.StateVector(),
                    0.20f,
                    0.0f,
                    params,
                    0.80f,
                    params.supplyVoltageV,
                    PlantModel::kDefaultVelocityTargetResponseTimeS);

            AssertDriveCommandMatchesSolution(command, solution);
        }

        TEST_METHOD(DriveBasePointCommandCombinedRawMatchesPlantVelocityTargetFeedforwardAtTarget)
        {
            PlantModel plant;
            DriveBaseHarness driveHarness(plant, Config::kDriveBasePDCluster);
            DriveBase& drive = driveHarness.drive;
            Assert::IsTrue(drive.Begin());
            drive.SetPose(0.0f, 0.0f, 0.0f);
            const PlantParams params = PlantParams::Default();

            const ControlVector command =
                drive.PointCommand(
                    0.20f,
                    0.40f,
                    MazeMap::CommandPD::RawCommand);
            const DriveCommandSolution solution =
                plant.solveDriveCommandsForVelocityTarget(
                    driveHarness.estimator.StateVector(),
                    0.20f,
                    0.40f,
                    params,
                    0.80f,
                    params.supplyVoltageV,
                    PlantModel::kDefaultVelocityTargetResponseTimeS);

            AssertDriveCommandMatchesSolution(command, solution);
        }

        TEST_METHOD(DriveBasePointYawRateCommandRawMatchesPlantVelocityTargetFeedforwardAtTarget)
        {
            PlantModel plant;
            DriveBaseHarness driveHarness(plant, Config::kDriveBasePDCluster);
            DriveBase& drive = driveHarness.drive;
            Assert::IsTrue(drive.Begin());
            drive.SetPose(0.0f, 0.0f, 0.0f);
            const PlantParams params = PlantParams::Default();

            const ControlVector command =
                drive.PointYawRateCommand(
                    0.40f,
                    MazeMap::CommandPD::RawCommand);
            const DriveCommandSolution solution =
                plant.solveDriveCommandsForVelocityTarget(
                    driveHarness.estimator.StateVector(),
                    0.0f,
                    0.40f,
                    params,
                    0.80f,
                    params.supplyVoltageV,
                    PlantModel::kDefaultVelocityTargetResponseTimeS);

            AssertDriveCommandMatchesSolution(command, solution);
        }

        TEST_METHOD(DriveBaseRawFeedforwardReportsAlignedCycleContextUsage)
        {
            PlantModel plant;
            DriveBaseHarness driveHarness(plant, Config::kDriveBasePDCluster);
            DriveBase& drive = driveHarness.drive;
            Assert::IsTrue(drive.Begin());
            drive.SetPose(0.0f, 0.0f, 0.0f);

            const ControlVector command =
                drive.PointCommand(
                    0.20f,
                    MazeMap::CommandPD::RawCommand);
            const DriveTelemetry telemetry = drive.GetTelemetry();

            Assert::IsTrue(IsFiniteControlVector(command));
            Assert::IsTrue(telemetry.feedforwardUsedAlignedCycleContext != 0U);
            Assert::IsFalse(telemetry.feedforwardUsedGripOnlyFallback != 0U);
        }

        TEST_METHOD(DriveConfigYawRateTrackingUsesImuYawByDefault)
        {
            Assert::IsTrue(Config::kDriveYawRateCommandPd == MazeMap::CommandPD::IMUYaw);
        }

        TEST_METHOD(DriveBasePointCommandManeuverPointMatchesScalarTargets)
        {
            PlantModel plant;
            DriveBaseHarness driveHarness(plant, Config::kDriveBasePDCluster);
            DriveBase& drive = driveHarness.drive;
            Assert::IsTrue(drive.Begin());
            drive.SetPose(0.0f, 0.0f, 0.0f);
            UpdateDriveEstimator(drive, driveHarness.estimator, 0.001f, BuildDriveBaseSensorSnapshot(0.15f));

            const ManeuverPoint point(0.0f, 0.0f, 0.25f, 0.30f, 0.20f);
            const ControlVector scalarCommand =
                drive.PointCommand(
                    point.Velocity,
                    point.Omega,
                    MazeMap::CommandPD::StateWheelOmegaPD |
                    MazeMap::CommandPD::IMUYaw);
            const ControlVector pointCommand =
                drive.PointCommand(
                    point,
                    MazeMap::CommandPD::StateWheelOmegaPD |
                    MazeMap::CommandPD::IMUYaw);

            Assert::IsTrue(IsFiniteControlVector(scalarCommand));
            Assert::IsTrue(IsFiniteControlVector(pointCommand));
            Assert::AreEqual(scalarCommand.leftMotorPwm, pointCommand.leftMotorPwm, 1.0e-6f);
            Assert::AreEqual(scalarCommand.rightMotorPwm, pointCommand.rightMotorPwm, 1.0e-6f);
        }

        TEST_METHOD(DriveBasePointCommandManeuverPointRejectsNonFiniteTargets)
        {
            PlantModel plant;
            DriveBaseHarness driveHarness(plant, Config::kDriveBasePDCluster);
            DriveBase& drive = driveHarness.drive;
            Assert::IsTrue(drive.Begin());
            drive.SetPose(0.0f, 0.0f, 0.0f);
            UpdateDriveEstimator(drive, driveHarness.estimator, 0.001f, BuildDriveBaseSensorSnapshot(0.0f));

            const ManeuverPoint invalidPoint(
                0.0f,
                0.0f,
                0.0f,
                std::numeric_limits<float>::quiet_NaN(),
                0.20f);
            const ControlVector command =
                drive.PointCommand(
                    invalidPoint,
                    MazeMap::CommandPD::StateWheelOmegaPD |
                    MazeMap::CommandPD::IMUYaw);

            Assert::AreEqual(0.0f, command.leftMotorPwm, 1.0e-6f);
            Assert::AreEqual(0.0f, command.rightMotorPwm, 1.0e-6f);
        }

        TEST_METHOD(DriveBasePointCommandCoupledStateYawPdChangesCommandWhenEstimatorYawRateDiffers)
        {
            PlantModel plant;
            DriveBaseHarness driveHarness(plant, Config::kDriveBasePDCluster);
            DriveBase& drive = driveHarness.drive;
            Assert::IsTrue(drive.Begin());
            drive.SetPose(0.0f, 0.0f, 0.0f);
            PrimeDriveBaseWithEncoderDelta(drive, driveHarness.estimator, 24, -24);

            const ControlVector rawCommand =
                drive.PointCommand(
                    0.20f,
                    0.0f,
                    MazeMap::CommandPD::RawCommand);
            const ControlVector stateYawCommand =
                drive.PointCommand(
                    0.20f,
                    0.0f,
                    MazeMap::CommandPD::StateYawPD);
            AssertDriveCommandsDiffer(rawCommand, stateYawCommand);
        }

        TEST_METHOD(DriveBasePointCommandCoupledStateYawPdZeroGainMatchesRawWhenYawRateErrorExists)
        {
            PlantModel plant;
            MazeMap::ProportionalDerivativeCluster defaultCluster = Config::kDriveBasePDCluster;
            MazeMap::ProportionalDerivativeCluster zeroYawCluster = defaultCluster;
            zeroYawCluster.YawRateStatePD.SetGains(0.0f, 0.0f);
            DriveBaseHarness driveHarness(plant, defaultCluster);
            DriveBase& drive = driveHarness.drive;
            Assert::IsTrue(drive.Begin());
            drive.SetPose(0.0f, 0.0f, 0.0f);
            UpdateDriveBaseSignals(
                drive,
                driveHarness.estimator,
                BuildDriveBaseSensorSnapshot(
                    0.40f,
                    0.0f,
                    0.0f,
                    0.0f,
                    false));

            const ControlVector rawCommand =
                drive.PointCommand(
                    0.20f,
                    0.0f,
                    MazeMap::CommandPD::RawCommand);
            drive.SetProportionalDerivativeCluster(zeroYawCluster);
            const ControlVector zeroGainYawCommand =
                drive.PointCommand(
                    0.20f,
                    0.0f,
                    MazeMap::CommandPD::StateYawPD);

            AssertDriveCommandsEqual(rawCommand, zeroGainYawCommand);
        }

        TEST_METHOD(DriveBasePointCommandCoupledImuYawIsNoOpWhenOnlyEstimatorYawRateDiffers)
        {
            PlantModel plant;
            DriveBaseHarness driveHarness(plant, Config::kDriveBasePDCluster);
            DriveBase& drive = driveHarness.drive;
            Assert::IsTrue(drive.Begin());
            drive.SetPose(0.0f, 0.0f, 0.0f);
            UpdateDriveBaseSignals(
                drive,
                driveHarness.estimator,
                BuildDriveBaseSensorSnapshot(
                    0.40f,
                    0.0f,
                    0.0f,
                    0.0f,
                    false));

            const ControlVector rawCommand =
                drive.PointCommand(
                    0.20f,
                    0.0f,
                    MazeMap::CommandPD::RawCommand);
            const ControlVector imuYawCommand =
                drive.PointCommand(
                    0.20f,
                    0.0f,
                    MazeMap::CommandPD::IMUYaw);

            AssertDriveCommandsEqual(rawCommand, imuYawCommand);
        }

        TEST_METHOD(DriveBasePointCommandCoupledImuYawChangesCommandWhenImuYawRateDiffers)
        {
            PlantModel plant;
            DriveBaseHarness driveHarness(plant, Config::kDriveBasePDCluster);
            DriveBase& drive = driveHarness.drive;
            Assert::IsTrue(drive.Begin());
            drive.SetPose(0.0f, 0.0f, 0.0f);
            UpdateDriveBaseSignals(
                drive,
                driveHarness.estimator,
                BuildDriveBaseSensorSnapshot(
                    0.0f,
                    0.40f,
                    0.0f,
                    0.0f,
                    false));

            const ControlVector rawCommand =
                drive.PointCommand(
                    0.20f,
                    0.0f,
                    MazeMap::CommandPD::RawCommand);
            const ControlVector imuYawCommand =
                drive.PointCommand(
                    0.20f,
                    0.0f,
                    MazeMap::CommandPD::IMUYaw);

            AssertDriveCommandsDiffer(rawCommand, imuYawCommand);
        }

        TEST_METHOD(DriveBaseDeltaCommandStateAccelerationPdChangesCommandWhenStateAccelerationErrorExists)
        {
            PlantModel plant;
            DriveBaseHarness driveHarness(plant, Config::kDriveBasePDCluster);
            DriveBase& drive = driveHarness.drive;
            Assert::IsTrue(drive.Begin());
            drive.SetPose(0.0f, 0.0f, 0.0f);
            PlantModel::StateVector truthState = PlantModel::StateVector::Zero();
            float leftEncoderRemainderCounts = 0.0f;
            float rightEncoderRemainderCounts = 0.0f;
            ControlVector appliedCommand{};
            appliedCommand.leftMotorPwm = 0.80f;
            appliedCommand.rightMotorPwm = 0.80f;
            drive.CommandGenerated(appliedCommand, 0.0f, 0.0f, false);
            for (int cycleIndex = 0; cycleIndex < 24; ++cycleIndex)
            {
                SimulateDriveBaseCycle(
                    drive,
                    driveHarness.estimator,
                    plant,
                    truthState,
                    leftEncoderRemainderCounts,
                    rightEncoderRemainderCounts,
                    kDriveBasePredictDtSeconds);
            }
            const float presentLinearSpeedMps =
                driveHarness.estimator.StateVector()(VehicleState::kU);

            const ControlVector rawCommand =
                drive.DeltaCommand(
                    presentLinearSpeedMps,
                    1.0f,
                    MazeMap::CommandPD::RawCommand);
            const ControlVector stateAccelerationCommand =
                drive.DeltaCommand(
                    presentLinearSpeedMps,
                    1.0f,
                    MazeMap::CommandPD::StateAccelerationPD);
            AssertDriveCommandsDiffer(rawCommand, stateAccelerationCommand);
        }

        TEST_METHOD(DriveBaseDeltaCommandStateAccelerationPdIsNoOpWhenOnlyImuForwardAccelDiffers)
        {
            PlantModel plant;
            DriveBaseHarness driveHarness(plant, Config::kDriveBasePDCluster);
            DriveBase& drive = driveHarness.drive;
            Assert::IsTrue(drive.Begin());
            drive.SetPose(0.0f, 0.0f, 0.0f);
            UpdateDriveBaseSignals(
                drive,
                driveHarness.estimator,
                BuildDriveBaseSensorSnapshot(
                    0.0f,
                    0.0f,
                    0.0f,
                    1.50f,
                    true));
            drive.SetPose(0.0f, 0.0f, 0.0f);

            const ControlVector rawCommand =
                drive.DeltaCommand(
                    0.0f,
                    0.0f,
                    MazeMap::CommandPD::RawCommand);
            const ControlVector stateAccelerationCommand =
                drive.DeltaCommand(
                    0.0f,
                    0.0f,
                    MazeMap::CommandPD::StateAccelerationPD);
            AssertDriveCommandsEqual(rawCommand, stateAccelerationCommand);
        }

        TEST_METHOD(DriveBaseDeltaCommandImuForwardAccelChangesCommandWhenImuForwardAccelDiffers)
        {
            PlantModel plant;
            DriveBaseHarness driveHarness(plant, Config::kDriveBasePDCluster);
            DriveBase& drive = driveHarness.drive;
            Assert::IsTrue(drive.Begin());
            drive.SetPose(0.0f, 0.0f, 0.0f);
            UpdateDriveBaseSignals(
                drive,
                driveHarness.estimator,
                BuildDriveBaseSensorSnapshot(
                    0.0f,
                    0.0f,
                    0.0f,
                    1.50f,
                    true));

            const ControlVector rawCommand =
                drive.DeltaCommand(
                    0.0f,
                    0.0f,
                    MazeMap::CommandPD::RawCommand);
            const ControlVector imuAccelerationCommand =
                drive.DeltaCommand(
                    0.0f,
                    0.0f,
                    MazeMap::CommandPD::IMUForwardAccel);

            AssertDriveCommandsDiffer(rawCommand, imuAccelerationCommand);
        }

        TEST_METHOD(DriveBasePointCommandLinearOnlyStateVelocityPdIsNoOpAtMatchingTarget)
        {
            PlantModel plant;
            DriveBaseHarness driveHarness(plant, Config::kDriveBasePDCluster);
            DriveBase& drive = driveHarness.drive;
            Assert::IsTrue(drive.Begin());
            drive.SetPose(0.0f, 0.0f, 0.0f);
            PrimeDriveBaseWithEncoderDelta(drive, driveHarness.estimator, 48, 48);
            drive.SetPose(0.0f, 0.0f, 0.0f);

            const ControlVector rawCommand =
                drive.PointCommand(
                    0.0f,
                    MazeMap::CommandPD::RawCommand);
            const ControlVector stateVelocityCommand =
                drive.PointCommand(
                    0.0f,
                    MazeMap::CommandPD::StateVelocityPD);
            AssertDriveCommandsEqual(rawCommand, stateVelocityCommand);
        }

        TEST_METHOD(DriveBasePointCommandLinearOnlyStateVelocityPdZeroGainMatchesRawWhenVelocityErrorExists)
        {
            PlantModel plant;
            MazeMap::ProportionalDerivativeCluster defaultCluster = Config::kDriveBasePDCluster;
            MazeMap::ProportionalDerivativeCluster zeroVelocityCluster = defaultCluster;
            zeroVelocityCluster.VelocityStatePD.SetGains(0.0f, 0.0f);
            DriveBaseHarness driveHarness(plant, defaultCluster);
            DriveBase& drive = driveHarness.drive;
            Assert::IsTrue(drive.Begin());
            drive.SetPose(0.0f, 0.0f, 0.0f);
            PrimeDriveBaseWithEncoderDelta(drive, driveHarness.estimator, 48, 48);

            const ControlVector rawCommand =
                drive.PointCommand(
                    0.0f,
                    MazeMap::CommandPD::RawCommand);
            drive.SetProportionalDerivativeCluster(zeroVelocityCluster);
            const ControlVector zeroGainVelocityCommand =
                drive.PointCommand(
                    0.0f,
                    MazeMap::CommandPD::StateVelocityPD);

            AssertDriveCommandsEqual(rawCommand, zeroGainVelocityCommand);
        }

        TEST_METHOD(DriveBasePointCommandLinearOnlyStateWheelOmegaPdIsNoOpAtMatchingTarget)
        {
            PlantModel plant;
            DriveBaseHarness driveHarness(plant, Config::kDriveBasePDCluster);
            DriveBase& drive = driveHarness.drive;
            Assert::IsTrue(drive.Begin());
            drive.SetPose(0.0f, 0.0f, 0.0f);
            PrimeDriveBaseWithEncoderDelta(drive, driveHarness.estimator, 48, 48);
            drive.SetPose(0.0f, 0.0f, 0.0f);

            const ControlVector rawCommand =
                drive.PointCommand(
                    0.0f,
                    MazeMap::CommandPD::RawCommand);
            const ControlVector stateWheelOmegaCommand =
                drive.PointCommand(
                    0.0f,
                    MazeMap::CommandPD::StateWheelOmegaPD);

            AssertDriveCommandsEqual(rawCommand, stateWheelOmegaCommand);
        }

        TEST_METHOD(DriveBasePointCommandLinearOnlyEncoderVelocityChangesCommandWhenEncoderVelocityDiffers)
        {
            PlantModel plant;
            DriveBaseHarness driveHarness(plant, Config::kDriveBasePDCluster);
            DriveBase& drive = driveHarness.drive;
            Assert::IsTrue(drive.Begin());
            drive.SetPose(0.0f, 0.0f, 0.0f);
            PrimeDriveBaseWithEncoderDelta(drive, driveHarness.estimator, 48, 48);
            drive.SetPose(0.0f, 0.0f, 0.0f);

            const ControlVector rawCommand =
                drive.PointCommand(
                    0.0f,
                    MazeMap::CommandPD::RawCommand);
            const ControlVector encoderVelocityCommand =
                drive.PointCommand(
                    0.0f,
                    MazeMap::CommandPD::EncoderVelocity);

            AssertDriveCommandsDiffer(rawCommand, encoderVelocityCommand);
        }

        TEST_METHOD(DriveBaseCachesGeneratedFeedforwardAndFeedbackTelemetryAsAverageDelta)
        {
            PlantModel plant;
            DriveBaseHarness driveHarness(plant, Config::kDriveBasePDCluster);
            DriveBase& drive = driveHarness.drive;
            Assert::IsTrue(drive.Begin());
            drive.SetPose(0.0f, 0.0f, 0.0f);
            PrimeDriveBaseWithEncoderDelta(drive, driveHarness.estimator, 48, 24);
            drive.SetPose(0.0f, 0.0f, 0.0f);

            const ControlVector rawCommand =
                drive.PointCommand(
                    0.0f,
                    MazeMap::CommandPD::RawCommand);

            Assert::AreEqual(
                ControlVectorAverage(rawCommand),
                drive.GetLastFeedforwardCommandAverage(),
                1.0e-6f);
            Assert::AreEqual(
                ControlVectorDelta(rawCommand),
                drive.GetLastFeedforwardCommandDelta(),
                1.0e-6f);
            Assert::AreEqual(0.0f, drive.GetLastFeedbackCommandAverage(), 1.0e-6f);
            Assert::AreEqual(0.0f, drive.GetLastFeedbackCommandDelta(), 1.0e-6f);

            const ControlVector encoderVelocityCommand =
                drive.PointCommand(
                    0.0f,
                    MazeMap::CommandPD::EncoderVelocity);
            const ControlVector feedbackOnly =
                ControlVector::RawMotorPwm(
                    encoderVelocityCommand.leftMotorPwm - rawCommand.leftMotorPwm,
                    encoderVelocityCommand.rightMotorPwm - rawCommand.rightMotorPwm);

            Assert::AreEqual(
                ControlVectorAverage(rawCommand),
                drive.GetLastFeedforwardCommandAverage(),
                1.0e-6f);
            Assert::AreEqual(
                ControlVectorDelta(rawCommand),
                drive.GetLastFeedforwardCommandDelta(),
                1.0e-6f);
            Assert::AreEqual(
                ControlVectorAverage(feedbackOnly),
                drive.GetLastFeedbackCommandAverage(),
                1.0e-6f);
            Assert::AreEqual(
                ControlVectorDelta(feedbackOnly),
                drive.GetLastFeedbackCommandDelta(),
                1.0e-6f);
        }

#define DEFINE_PD_CLUSTER_OSCILLATION_TESTS(TestStem, PairingKindExpr, PdExpr)                         \
        TEST_METHOD(TestStem##MetricsFinite)                                                             \
        {                                                                                               \
            AssertOscillationPairingMetricsFinite(PdExpr, PairingKindExpr);                              \
        }                                                                                               \
        TEST_METHOD(TestStem##SourceJitterLow)                                                           \
        {                                                                                               \
            AssertOscillationPairingSourceJitterLow(PdExpr, PairingKindExpr);                            \
        }                                                                                               \
        TEST_METHOD(TestStem##EstimatorTrackingAccurate)                                                 \
        {                                                                                               \
            AssertOscillationPairingEstimatorTrackingAccurate(PdExpr, PairingKindExpr);                  \
        }                                                                                               \
        TEST_METHOD(TestStem##PlantTrackingAccurate)                                                     \
        {                                                                                               \
            AssertOscillationPairingPlantTrackingAccurate(PdExpr, PairingKindExpr);                      \
        }                                                                                               \
        TEST_METHOD(TestStem##HasNoAlternatingSaturation)                                                \
        {                                                                                               \
            AssertOscillationPairingHasNoAlternatingSaturation(PdExpr, PairingKindExpr);                 \
        }

        // This suite now uses only the public DriveBase command entry points. Internal-only cluster
        // entries that are not reachable through those entry points are intentionally excluded here.
        DEFINE_PD_CLUSTER_OSCILLATION_TESTS(
            DriveBaseOscillationPairingHeadingStatePd,
            OscillationPairingKind::HeadingState,
            Config::kDriveBasePDCluster.HeadingStatePD)
        DEFINE_PD_CLUSTER_OSCILLATION_TESTS(
            DriveBaseOscillationPairingVelocityStatePd,
            OscillationPairingKind::VelocityState,
            Config::kDriveBasePDCluster.VelocityStatePD)
        DEFINE_PD_CLUSTER_OSCILLATION_TESTS(
            DriveBaseOscillationPairingYawRateStatePd,
            OscillationPairingKind::YawRateState,
            Config::kDriveBasePDCluster.YawRateStatePD)
        DEFINE_PD_CLUSTER_OSCILLATION_TESTS(
            DriveBaseOscillationPairingYawRateGyroPd,
            OscillationPairingKind::YawRateGyro,
            Config::kDriveBasePDCluster.YawRateGyroPD)
        DEFINE_PD_CLUSTER_OSCILLATION_TESTS(
            DriveBaseOscillationPairingLongitudinalAccelerationStatePd,
            OscillationPairingKind::LongitudinalAccelerationState,
            Config::kDriveBasePDCluster.LongitudinalAccelerationStatePD)
        DEFINE_PD_CLUSTER_OSCILLATION_TESTS(
            DriveBaseOscillationPairingLongitudinalAccelerationImuForwardAccelPd,
            OscillationPairingKind::LongitudinalAccelerationImuForwardAccel,
            Config::kDriveBasePDCluster.LongitudinalAccelerationIMUForwardAccelPD)
        DEFINE_PD_CLUSTER_OSCILLATION_TESTS(
            DriveBaseOscillationPairingWheelVelocityStatePd,
            OscillationPairingKind::WheelVelocityState,
            Config::kDriveBasePDCluster.WheelVelocityStatePD)
        DEFINE_PD_CLUSTER_OSCILLATION_TESTS(
            DriveBaseOscillationPairingWheelVelocityEncoderPd,
            OscillationPairingKind::WheelVelocityEncoder,
            Config::kDriveBasePDCluster.WheelVelocityEncoderPD)

#undef DEFINE_PD_CLUSTER_OSCILLATION_TESTS

    };
}
