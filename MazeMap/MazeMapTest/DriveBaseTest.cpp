#include "pch.h"
#include "CppUnitTest.h"

#include "..\MazeMap\Estimator.h"
#include "..\MazeMap\DriveBase.h"
#include "..\MazeMap\PlantModel.h"
#include "..\MazeMap\Vehicle.h"
#include "..\MazeMap\VehicleState.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    namespace
    {
        using CommandVector = MazeMap::App::Internal::CommandVector;

        constexpr float kDriveBaseLoopDtSeconds = 0.001f;
        constexpr float kDriveBaseLegacyLoopDtSeconds = 0.01f;
        constexpr float kDriveBaseInPlaceTurnMinimumDriveCommand = 0.60f;
        constexpr int kDriveBaseHoldFeedforwardSteps = 400;
        constexpr int kDriveBaseVelocityTargetSteps = 600;

        constexpr int kDriveBaseLegacyLoopStepScale =
            static_cast<int>((kDriveBaseLegacyLoopDtSeconds / kDriveBaseLoopDtSeconds) + 0.5f);

        constexpr int ScaleLegacyLoopSteps(const int legacyStepCount) noexcept
        {
            return legacyStepCount * kDriveBaseLegacyLoopStepScale;
        }

        struct DriveBaseHarness final
        {
            VehicleState& runtimeState;
            Estimator estimator;
            DriveBase drive;

            DriveBaseHarness(
                PlantModel& plant,
                VehicleState& sharedRuntimeState,
                const MazeMap::ProportionalDerivativeCluster& cluster)
                : runtimeState(sharedRuntimeState)
                , estimator(plant, runtimeState)
                , drive(plant, runtimeState, cluster)
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

        float DriveBaseDistancePerEncoderCountMeters(const PlantParams& params) noexcept
        {
            return
                (2.0f * PI_F * params.wheelRadiusM) /
                (params.gearRatio * static_cast<float>(params.encoderCountsPerMotorRev));
        }

        VehicleState::StateVector BuildDriveBaseStateVector(const VehicleState& state)
        {
            VehicleState::StateVector result = VehicleState::StateVector::Zero();
            result(VehicleState::kPx) = state.GetPositionX();
            result(VehicleState::kPy) = state.GetPositionY();
            result(VehicleState::kPsi) = state.GetOrientation();
            result(VehicleState::kU) = state.GetVelocity();
            result(VehicleState::kV) = state.GetLateralVelocity();
            result(VehicleState::kR) = state.GetRotationalVelocity();
            result(VehicleState::kOmegaL) = state.GetWheelSpeedLeft();
            result(VehicleState::kOmegaR) = state.GetWheelSpeedRight();
            result(VehicleState::kBgz) = state.GetGyroBiasZ();
            VehicleState::NormalizeStateVector(result);
            return result;
        }

        EncoderObs BuildDriveBaseEncoderObservation(
            const int32_t leftCounts,
            const int32_t rightCounts,
            const float dtSeconds,
            const PlantParams& params) noexcept
        {
            EncoderObs observation{};
            observation.totalLeftCounts = leftCounts;
            observation.totalRightCounts = rightCounts;
            observation.leftDistanceDeltaM =
                DriveBaseDistancePerEncoderCountMeters(params) * static_cast<float>(leftCounts);
            observation.rightDistanceDeltaM =
                DriveBaseDistancePerEncoderCountMeters(params) * static_cast<float>(rightCounts);
            if ((dtSeconds > 0.0f) && std::isfinite(dtSeconds) &&
                (params.wheelRadiusM > 0.0f) && std::isfinite(params.wheelRadiusM))
            {
                const float invWheelRadiusM = 1.0f / params.wheelRadiusM;
                const float invDtSeconds = 1.0f / dtSeconds;
                observation.leftVelocityMps = observation.leftDistanceDeltaM * invDtSeconds;
                observation.rightVelocityMps = observation.rightDistanceDeltaM * invDtSeconds;
                observation.omegaLeftRadps = observation.leftVelocityMps * invWheelRadiusM;
                observation.omegaRightRadps = observation.rightVelocityMps * invWheelRadiusM;
            }
            return observation;
        }

        void UpdateDriveEstimator(
            DriveBase& drive,
            Estimator& estimator,
            VehicleState& runtimeState,
            const float dtSeconds,
            const SensorSnapshot& snapshot)
        {
            runtimeState.SetSensorSnapshot(snapshot);
            if (std::isfinite(dtSeconds) && (dtSeconds > 0.0f))
            {
                runtimeState.SetTime(runtimeState.GetTime() + dtSeconds);
            }

            if (estimator.HasFault())
            {
                estimator.SyncRuntimeState();
                runtimeState.SetSensorSnapshot(snapshot);
                return;
            }

            if (std::isfinite(dtSeconds) && (dtSeconds > 0.0f))
            {
                if (!estimator.predict(
                    dtSeconds,
                    drive.CurrentControlVector(),
                    GetMissionFanDutyCycle(),
                    drive.CurrentBatteryVoltageV()))
                {
                    estimator.SyncRuntimeState();
                    runtimeState.SetSensorSnapshot(snapshot);
                    return;
                }
            }

            if (snapshot.encoderObservationValid)
            {
                const bool updateYawFromEncoder = !std::isfinite(snapshot.gyroRawRadps);
                (void)estimator.updateEncoderPair(snapshot.encoderObservation, dtSeconds, updateYawFromEncoder);
            }

            if (std::isfinite(snapshot.gyroRawRadps))
            {
                const MeasurementUpdateResult yawUpdate = estimator.updateYawRate(snapshot.gyroRawRadps);
                if (!yawUpdate.accepted)
                {
                    estimator.SyncRuntimeState();
                    runtimeState.SetSensorSnapshot(snapshot);
                    return;
                }
            }

            ImuAccelObs accelObservation{};
            accelObservation.valid =
                snapshot.accelBiasValid &&
                std::isfinite(snapshot.accelBodyXMps2) &&
                std::isfinite(snapshot.accelBodyYMps2);
            accelObservation.accelBodyXMps2 = snapshot.accelBodyXMps2;
            accelObservation.accelBodyYMps2 = snapshot.accelBodyYMps2;
            (void)estimator.updatePlanarAccel(accelObservation);

            estimator.SyncRuntimeState();
            runtimeState.SetSensorSnapshot(snapshot);
        }

        void UpdateDriveBaseSignals(
            DriveBase& drive,
            Estimator& estimator,
            VehicleState& runtimeState,
            const SensorSnapshot& snapshot,
            const int32_t leftCounts = 0,
            const int32_t rightCounts = 0,
            const float dtSeconds = 0.001f)
        {
            const PlantParams& params = PlantParams::Default();
            SensorSnapshot updatedSnapshot = snapshot;
            updatedSnapshot.encoderObservation =
                BuildDriveBaseEncoderObservation(leftCounts, rightCounts, dtSeconds, params);
            updatedSnapshot.encoderObservationValid = true;
            UpdateDriveEstimator(drive, estimator, runtimeState, dtSeconds, updatedSnapshot);
        }

        void SimulateDriveBaseCycle(
            DriveBase& drive,
            Estimator& estimator,
            VehicleState& runtimeState,
            PlantModel& plant,
            PlantModel::StateVector& truthState,
            float& leftEncoderRemainderCounts,
            float& rightEncoderRemainderCounts,
            float dtSeconds)
        {
            const PlantParams& params = PlantParams::Default();
            const DriveTelemetry telemetry = drive.GetTelemetry();
            const CommandVector control = CommandVector(
                telemetry.leftDriveCommand,
                telemetry.rightDriveCommand);

            const PlantModel::StateVector previousTruthState = truthState;
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
            const float distancePerCountM = DriveBaseDistancePerEncoderCountMeters(params);
            const int32_t leftCounts =
                ConsumeWholeEncoderCounts(leftDistanceDeltaM / distancePerCountM, leftEncoderRemainderCounts);
            const int32_t rightCounts =
                ConsumeWholeEncoderCounts(rightDistanceDeltaM / distancePerCountM, rightEncoderRemainderCounts);

            UpdateDriveBaseSignals(
                drive,
                estimator,
                runtimeState,
                BuildDriveBaseSensorSnapshot(truthState(VehicleState::kR)),
                leftCounts,
                rightCounts,
                dtSeconds);
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

        bool IsFiniteControlVector(const CommandVector& command) noexcept
        {
            return std::isfinite(command.LeftMotorPwm()) && std::isfinite(command.RightMotorPwm());
        }

        float ControlVectorDifferenceMagnitude(
            const CommandVector& lhs,
            const CommandVector& rhs) noexcept
        {
            return
                std::fabs(lhs.LeftMotorPwm() - rhs.LeftMotorPwm()) +
                std::fabs(lhs.RightMotorPwm() - rhs.RightMotorPwm());
        }

        float ControlVectorAverage(const CommandVector& command) noexcept
        {
            return 0.5f * (command.LeftMotorPwm() + command.RightMotorPwm());
        }

        float ControlVectorDelta(const CommandVector& command) noexcept
        {
            return 0.5f * (command.LeftMotorPwm() - command.RightMotorPwm());
        }

        void AssertDriveCommandsEqual(
            const CommandVector& expected,
            const CommandVector& actual,
            float tolerance = 1.0e-3f)
        {
            Assert::IsTrue(IsFiniteControlVector(expected));
            Assert::IsTrue(IsFiniteControlVector(actual));
            Assert::AreEqual(expected.LeftMotorPwm(), actual.LeftMotorPwm(), tolerance);
            Assert::AreEqual(expected.RightMotorPwm(), actual.RightMotorPwm(), tolerance);
        }

        void AssertDriveCommandsDiffer(
            const CommandVector& lhs,
            const CommandVector& rhs,
            float minimumDifference = 1.0e-4f)
        {
            Assert::IsTrue(IsFiniteControlVector(lhs));
            Assert::IsTrue(IsFiniteControlVector(rhs));
            Assert::IsTrue(ControlVectorDifferenceMagnitude(lhs, rhs) > minimumDifference);
        }

        void AssertPositiveInPlaceTurnCommandMeetsMinimumDrive(
            const CommandVector& command,
            float minimumDriveCommand = kDriveBaseInPlaceTurnMinimumDriveCommand)
        {
            Assert::IsTrue(IsFiniteControlVector(command));
            Assert::IsTrue(command.LeftMotorPwm() >= minimumDriveCommand);
            Assert::IsTrue(command.RightMotorPwm() <= -minimumDriveCommand);
        }

        void PrimeDriveBaseWithEncoderDelta(
            DriveBase& drive,
            Estimator& estimator,
            VehicleState& runtimeState,
            const int32_t leftCounts,
            const int32_t rightCounts,
            const float dtSeconds = 0.001f)
        {
            const SensorSnapshot snapshot = BuildDriveBaseSensorSnapshot();
            UpdateDriveBaseSignals(drive, estimator, runtimeState, snapshot, leftCounts, rightCounts, dtSeconds);
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
            float dtSeconds = kDriveBaseLoopDtSeconds;
            int totalSteps = ScaleLegacyLoopSteps(320);
            int stepStartStep = ScaleLegacyLoopSteps(40);
            int steadyWindowStartStep = ScaleLegacyLoopSteps(200);
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
            float feedbackSignal = 0.0f;
            float feedbackCommandComponent = 0.0f;
            std::uint16_t saturationFlags = 0U;
        };

        struct OscillationPairingAuditResult final
        {
            float steadyErrorHighPassRms = 0.0f;
            float steadyAbsError = 0.0f;
            float steadyFeedbackComponentRms = 0.0f;
            float steadySignFlipRate = 0.0f;
            float fullSaturationDuty = 0.0f;
            float steadySaturationDuty = 0.0f;
            std::uint32_t steadySaturatedRunCount = 0U;
            std::uint32_t steadyAlternatingSaturationCount = 0U;
            bool steadyAlternatingSaturationPatternDetected = false;
            int collectionStartStep = 0;
            int collectionEndStep = 0;
        };

        struct OscillationPairingAuditWindow final
        {
            std::size_t startIndex = 0U;
            std::size_t endIndex = 0U;
        };

        struct OscillationGeneratedCommand final
        {
            CommandVector command{};
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
            const VehicleState& runtimeState,
            const OscillationPairingKind pairingKind,
            const float setpoint)
        {
            OscillationGeneratedCommand generated{};
            const float presentLinearSpeedMps = runtimeState.GetVelocity();
            const float presentYawRateRadps = runtimeState.GetRotationalVelocity();

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

        float ResolveOscillationFeedbackSignal(
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

        bool IsAccelerationLikeOscillationSignal(
            const MazeMap::CommandPD signalSource) noexcept
        {
            return
                (signalSource == MazeMap::CommandPD::StateAccelerationPD) ||
                (signalSource == MazeMap::CommandPD::IMUForwardAccel) ||
                (signalSource == MazeMap::CommandPD::IMULateralAccel);
        }

        bool IsVelocityOrYawRateOscillationSignal(
            const MazeMap::CommandPD signalSource) noexcept
        {
            return
                (signalSource == MazeMap::CommandPD::StateVelocityPD) ||
                (signalSource == MazeMap::CommandPD::EncoderVelocity) ||
                (signalSource == MazeMap::CommandPD::StateYawPD) ||
                (signalSource == MazeMap::CommandPD::IMUYaw);
        }

        bool HasReachedOscillationTarget(
            const float initialSetpoint,
            const float targetSignal,
            const float actualSignal) noexcept
        {
            const float targetDelta = targetSignal - initialSetpoint;
            if (std::fabs(targetDelta) <= 1.0e-6f)
            {
                return true;
            }

            return
                (targetDelta > 0.0f) ?
                (actualSignal >= targetSignal) :
                (actualSignal <= targetSignal);
        }

        OscillationPairingAuditWindow ResolveOscillationAuditWindow(
            const MazeMap::CommandPD signalSource,
            const OscillationPairingScenario& scenario,
            const std::vector<OscillationPairingTraceSample>& samples) noexcept
        {
            const int sampleCount = static_cast<int>(samples.size());
            const int lastSampleIndex = (std::max)(sampleCount - 1, 0);
            int startStep =
                (std::clamp)(
                    scenario.steadyWindowStartStep,
                    0,
                    lastSampleIndex);
            int endStep = sampleCount;

            if (IsAccelerationLikeOscillationSignal(signalSource))
            {
                startStep =
                    (std::clamp)(
                        scenario.stepStartStep + 10,
                        0,
                        lastSampleIndex);
                endStep = (std::min)(sampleCount, startStep + 3000);
                for (int sampleIndex = startStep; sampleIndex < endStep; ++sampleIndex)
                {
                    if (samples[static_cast<std::size_t>(sampleIndex)].saturationFlags != 0U)
                    {
                        endStep = sampleIndex;
                        break;
                    }
                }
            }
            else if (IsVelocityOrYawRateOscillationSignal(signalSource))
            {
                const int timeBasedStartStep =
                    (std::clamp)(
                        scenario.stepStartStep + 1000,
                        0,
                        lastSampleIndex);
                int firstTargetHitStep = sampleCount;
                for (int sampleIndex = scenario.stepStartStep; sampleIndex < sampleCount; ++sampleIndex)
                {
                    const OscillationPairingTraceSample& sample =
                        samples[static_cast<std::size_t>(sampleIndex)];
                    if (HasReachedOscillationTarget(
                        scenario.initialSetpoint,
                        sample.targetSignal,
                        sample.feedbackSignal))
                    {
                        firstTargetHitStep = sampleIndex;
                        break;
                    }
                }

                startStep = (std::min)(timeBasedStartStep, firstTargetHitStep);
                startStep = (std::clamp)(startStep, 0, lastSampleIndex);
                endStep = (std::min)(sampleCount, startStep + 2000);
            }

            endStep = (std::clamp)(endStep, startStep + 1, sampleCount);
            return
                OscillationPairingAuditWindow{
                    static_cast<std::size_t>(startStep),
                    static_cast<std::size_t>(endStep)
                };
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

        OscillationPairingScenario BuildStepOscillationPairingScenario(
            const MazeMap::CommandPD signalSource) noexcept
        {
            OscillationPairingScenario scenario{};
            switch (signalSource)
            {
            case MazeMap::CommandPD::StateHeadingPD:
                scenario.stepSetpoint = 0.15f;
                scenario.totalSteps = ScaleLegacyLoopSteps(520);
                scenario.steadyWindowStartStep = ScaleLegacyLoopSteps(360);
                break;
            case MazeMap::CommandPD::StateYawPD:
            case MazeMap::CommandPD::IMUYaw:
                scenario.stepSetpoint = 10.0f;
                scenario.stepStartStep = 1;
                scenario.totalSteps = 3001;
                scenario.steadyWindowStartStep = 1001;
                break;
            case MazeMap::CommandPD::StateAccelerationPD:
            case MazeMap::CommandPD::IMUForwardAccel:
                scenario.stepSetpoint = 0.5f * 9.80665f;
                scenario.initialForwardSpeedMps = 0.60f;
                scenario.stepStartStep = 1;
                scenario.totalSteps = 3011;
                scenario.steadyWindowStartStep = 11;
                break;
            case MazeMap::CommandPD::IMULateralAccel:
                scenario.stepSetpoint = 0.30f;
                scenario.initialForwardSpeedMps = 0.20f;
                scenario.totalSteps = ScaleLegacyLoopSteps(520);
                scenario.steadyWindowStartStep = ScaleLegacyLoopSteps(360);
                break;
            case MazeMap::CommandPD::StateVelocityPD:
            case MazeMap::CommandPD::EncoderVelocity:
            default:
                scenario.stepSetpoint = 0.60f;
                scenario.stepStartStep = 1;
                scenario.totalSteps = 3001;
                scenario.steadyWindowStartStep = 1001;
                break;
            }

            return scenario;
        }

        void PrimeOscillationPairingInitialState(
            DriveBase& drive,
            Estimator& estimator,
            VehicleState& runtimeState,
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

            const float distancePerCountM = DriveBaseDistancePerEncoderCountMeters(params);
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
                runtimeState,
                BuildDriveBaseSensorSnapshot(
                    scenario.initialYawRateRadps,
                    scenario.initialYawRateRadps,
                    0.0f,
                    0.0f,
                    false),
                leftCounts,
                rightCounts,
                scenario.dtSeconds);
            estimator.ProjectMeasuredKinematics(
                0.0f,
                BuildDriveBaseEncoderObservation(leftCounts, rightCounts, scenario.dtSeconds, params),
                scenario.initialYawRateRadps);
        }

        std::vector<OscillationPairingTraceSample> RunOscillationPairingTrace(
            const MazeMap::ProportionalDerivative& pd,
            const OscillationPairingKind pairingKind,
            const OscillationPairingScenario& scenario)
        {
            const MazeMap::CommandPD signalSource =
                ResolveOscillationPairingSignalSource(pairingKind);
            Vehicle vehicle;
            VehicleState runtimeState;
            PlantModel plant(vehicle, runtimeState);
            const MazeMap::ProportionalDerivativeCluster pairingCluster =
                BuildOscillationPairingCluster(pd, pairingKind);
            DriveBaseHarness driveHarness(plant, runtimeState, pairingCluster);
            DriveBase& drive = driveHarness.drive;
            Assert::IsTrue(drive.Begin());
            Assert::IsTrue(driveHarness.estimator.ResetPose(0.0f, 0.0f, 0.0f));
            UpdateDriveBaseSignals(
                drive,
                driveHarness.estimator,
                driveHarness.runtimeState,
                BuildDriveBaseSensorSnapshot(0.0f));

            PlantModel::StateVector truthState = PlantModel::StateVector::Zero();
            float leftEncoderRemainderCounts = 0.0f;
            float rightEncoderRemainderCounts = 0.0f;
            const PlantParams& params = PlantParams::Default();
            PrimeOscillationPairingInitialState(
                drive,
                driveHarness.estimator,
                driveHarness.runtimeState,
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
                        driveHarness.runtimeState,
                        pairingKind,
                        setpoint);
                drive.CommandGenerated(
                    generatedCommand.command,
                    generatedCommand.linearCommandTargetMps,
                    generatedCommand.angularCommandTargetRadps,
                    true);

                const DriveTelemetry appliedTelemetry = drive.GetTelemetry();
                const CommandVector control = CommandVector(
                    appliedTelemetry.leftDriveCommand,
                    appliedTelemetry.rightDriveCommand);

                const PlantModel::StateVector previousTruthState = truthState;
                truthState = plant.integrate(
                    truthState,
                    control,
                    0.80f,
                    params.supplyVoltageV,
                    scenario.dtSeconds,
                    params);
                const MazeMap::PlantDerivatives truthDerivatives =
                    plant.forwardStep(truthState, control, 0.80f, params.supplyVoltageV, params);

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
                const float distancePerCountM = DriveBaseDistancePerEncoderCountMeters(params);
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
                    driveHarness.runtimeState,
                    snapshot,
                    leftCounts + encoderCountDisturbance,
                    rightCounts + encoderCountDisturbance,
                    scenario.dtSeconds);

                const DriveTelemetry updatedTelemetry = drive.GetTelemetry();
                const PlantModel::StateVector estimatorState =
                    BuildDriveBaseStateVector(driveHarness.runtimeState);
                const MazeMap::PlantDerivatives estimatorDerivatives =
                    plant.forwardStep(estimatorState, control, 0.80f, params.supplyVoltageV, params);

                OscillationPairingTraceSample sample{};
                sample.targetSignal = generatedCommand.targetSignal;
                sample.feedbackSignal =
                    ResolveOscillationFeedbackSignal(
                        signalSource,
                        snapshot,
                        updatedTelemetry,
                        estimatorState,
                        estimatorDerivatives);
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

            std::vector<float> errors;
            std::vector<float> feedbackComponents;
            std::vector<std::uint16_t> saturationFlags;
            errors.reserve(samples.size());
            feedbackComponents.reserve(samples.size());
            saturationFlags.reserve(samples.size());

            for (const OscillationPairingTraceSample& sample : samples)
            {
                const float error =
                    ComputeOscillationTrackingError(
                        signalSource,
                        sample.targetSignal,
                        sample.feedbackSignal);
                errors.push_back(error);
                feedbackComponents.push_back(sample.feedbackCommandComponent);
                saturationFlags.push_back(sample.saturationFlags);
            }

            const OscillationPairingAuditWindow auditWindow =
                ResolveOscillationAuditWindow(signalSource, scenario, samples);
            result.collectionStartStep = static_cast<int>(auditWindow.startIndex);
            result.collectionEndStep = static_cast<int>(auditWindow.endIndex);

            const auto errorsWindowBegin = errors.begin() + static_cast<std::ptrdiff_t>(auditWindow.startIndex);
            const auto errorsWindowEnd = errors.begin() + static_cast<std::ptrdiff_t>(auditWindow.endIndex);
            const auto feedbackWindowBegin =
                feedbackComponents.begin() + static_cast<std::ptrdiff_t>(auditWindow.startIndex);
            const auto feedbackWindowEnd =
                feedbackComponents.begin() + static_cast<std::ptrdiff_t>(auditWindow.endIndex);
            const auto saturationWindowBegin =
                saturationFlags.begin() + static_cast<std::ptrdiff_t>(auditWindow.startIndex);
            const auto saturationWindowEnd =
                saturationFlags.begin() + static_cast<std::ptrdiff_t>(auditWindow.endIndex);

            const std::vector<float> windowErrors(errorsWindowBegin, errorsWindowEnd);
            const std::vector<float> windowFeedbackComponents(feedbackWindowBegin, feedbackWindowEnd);
            const std::vector<std::uint16_t> windowSaturationFlags(saturationWindowBegin, saturationWindowEnd);

            result.steadyErrorHighPassRms =
                ComputeHighPassRms(windowErrors, 0U);
            result.steadyAbsError =
                ComputeMeanAbsoluteValue(windowErrors, 0U);
            result.steadyFeedbackComponentRms =
                ComputeHighPassRms(windowFeedbackComponents, 0U);
            result.steadySignFlipRate =
                ComputeSignFlipRate(windowErrors, 0U, kOscillationTraceDeadbandMps);
            {
                std::vector<bool> saturationSamples;
                saturationSamples.reserve(samples.size());
                for (const std::uint16_t flags : saturationFlags)
                {
                    saturationSamples.push_back(flags != 0U);
                }
                result.fullSaturationDuty = ComputeSaturationDuty(saturationSamples, 0U);

                std::vector<bool> windowSaturationSamples;
                windowSaturationSamples.reserve(windowSaturationFlags.size());
                for (const std::uint16_t flags : windowSaturationFlags)
                {
                    windowSaturationSamples.push_back(flags != 0U);
                }
                result.steadySaturationDuty = ComputeSaturationDuty(windowSaturationSamples, 0U);
            }

            const AlternatingSaturationPatternResult saturationPattern =
                AnalyzeAlternatingSaturationPattern(windowSaturationFlags, 0U);
            result.steadySaturatedRunCount = saturationPattern.saturatedRunCount;
            result.steadyAlternatingSaturationCount = saturationPattern.alternationCount;
            result.steadyAlternatingSaturationPatternDetected = saturationPattern.patternDetected;

            return result;
        }

        std::wstring BuildOscillationPairingAuditMessage(
            const OscillationPairingAuditResult& result)
        {
            return
                std::wstring(L"steady_signal_hp_rms=") + std::to_wstring(result.steadyErrorHighPassRms) +
                L" steady_signal_abs_err=" + std::to_wstring(result.steadyAbsError) +
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
                L" collect_start=" + std::to_wstring(audit.collectionStartStep) +
                L" collect_end=" + std::to_wstring(audit.collectionEndStep);
        }

        bool OscillationPairingAuditHasFiniteMetrics(
            const OscillationPairingAuditResult& audit) noexcept
        {
            return
                std::isfinite(audit.steadyErrorHighPassRms) &&
                std::isfinite(audit.steadyAbsError) &&
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

        void AssertOscillationPairingSignalJitterLow(
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

            if (!std::isfinite(audit.steadyErrorHighPassRms))
            {
                return;
            }

            Assert::IsTrue(
                audit.steadyErrorHighPassRms <= jitterThreshold,
                message.c_str());
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
            Vehicle vehicle;
            VehicleState runtimeState;
            PlantModel plant(vehicle, runtimeState);
            DriveBaseHarness driveHarness(plant, runtimeState, Config::kDriveBasePDCluster);
            DriveBase& drive = driveHarness.drive;
            Assert::IsTrue(drive.Begin());

            PrimeDriveBaseWithEncoderDelta(drive, driveHarness.estimator, driveHarness.runtimeState, 6, 42);

            const CommandVector command =
                drive.DeltaCommand(
                    0.20f,
                    8.5f);

            Assert::IsTrue(IsFiniteControlVector(command));
            Assert::IsTrue(command.LeftMotorPwm() > 0.0f);
            Assert::AreEqual(command.LeftMotorPwm(), command.RightMotorPwm(), 1.0e-6f);
        }

        TEST_METHOD(DriveBaseDeltaCommandHeadingHoldStaysSymmetricWhenAlreadyAligned)
        {
            Vehicle vehicle;
            VehicleState runtimeState;
            PlantModel plant(vehicle, runtimeState);
            DriveBaseHarness driveHarness(plant, runtimeState, Config::kDriveBasePDCluster);
            DriveBase& drive = driveHarness.drive;
            Assert::IsTrue(drive.Begin());

            PrimeDriveBaseWithEncoderDelta(drive, driveHarness.estimator, driveHarness.runtimeState, 6, 42);

            const CommandVector command =
                drive.DeltaCommand(
                    0.20f,
                    8.5f,
                    MazeMap::CommandPD::StateHeadingPD);

            Assert::IsTrue(IsFiniteControlVector(command));
            Assert::AreEqual(command.LeftMotorPwm(), command.RightMotorPwm(), 1.0e-3f);
        }

        TEST_METHOD(DriveBaseDeltaCommandRawMatchesPlantFeedforwardAtSteadyForwardTarget)
        {
            Vehicle vehicle;
            VehicleState runtimeState;
            PlantModel plant(vehicle, runtimeState);
            DriveBaseHarness driveHarness(plant, runtimeState, Config::kDriveBasePDCluster);
            DriveBase& drive = driveHarness.drive;
            Assert::IsTrue(drive.Begin());
            Assert::IsTrue(driveHarness.estimator.ResetPose(0.0f, 0.0f, 0.0f));

            const CommandVector command =
                drive.DeltaCommand(
                    0.20f,
                    0.0f,
                    MazeMap::CommandPD::RawCommand);
            const CommandVector feedforwardCommand =
                plant.solveSteadyStateFeedforward(0.20f, 0.0f);

            AssertDriveCommandsEqual(feedforwardCommand, command, 1.0e-6f);
        }

        TEST_METHOD(DriveBaseDeltaCommandCombinedRawMatchesPlantFeedforwardAtSteadyTarget)
        {
            Vehicle vehicle;
            VehicleState runtimeState;
            PlantModel plant(vehicle, runtimeState);
            DriveBaseHarness driveHarness(plant, runtimeState, Config::kDriveBasePDCluster);
            DriveBase& drive = driveHarness.drive;
            Assert::IsTrue(drive.Begin());
            Assert::IsTrue(driveHarness.estimator.ResetPose(0.0f, 0.0f, 0.0f));

            const CommandVector command =
                drive.DeltaCommand(
                    0.20f,
                    0.0f,
                    0.40f,
                    0.0f,
                    MazeMap::CommandPD::RawCommand);
            const CommandVector feedforwardCommand =
                plant.solveSteadyStateFeedforward(0.20f, 0.40f);

            AssertDriveCommandsEqual(feedforwardCommand, command, 1.0e-6f);
        }

        TEST_METHOD(DriveBaseDeltaYawRateCommandRawMatchesPlantFeedforwardAtSteadyYawRateTarget)
        {
            Vehicle vehicle;
            VehicleState runtimeState;
            PlantModel plant(vehicle, runtimeState);
            DriveBaseHarness driveHarness(plant, runtimeState, Config::kDriveBasePDCluster);
            DriveBase& drive = driveHarness.drive;
            Assert::IsTrue(drive.Begin());
            Assert::IsTrue(driveHarness.estimator.ResetPose(0.0f, 0.0f, 0.0f));

            const CommandVector command =
                drive.DeltaYawRateCommand(
                    0.40f,
                    0.0f,
                    MazeMap::CommandPD::RawCommand);
            const CommandVector feedforwardCommand =
                plant.solveSteadyStateFeedforward(0.0f, 0.40f);

            AssertDriveCommandsEqual(feedforwardCommand, command, 1.0e-6f);
        }

        TEST_METHOD(DriveBasePointCommandRawMatchesPlantVelocityTargetFeedforwardAtForwardTarget)
        {
            Vehicle vehicle;
            VehicleState runtimeState;
            PlantModel plant(vehicle, runtimeState);
            DriveBaseHarness driveHarness(plant, runtimeState, Config::kDriveBasePDCluster);
            DriveBase& drive = driveHarness.drive;
            Assert::IsTrue(drive.Begin());
            Assert::IsTrue(driveHarness.estimator.ResetPose(0.0f, 0.0f, 0.0f));

            const CommandVector command =
                drive.PointCommand(
                    0.20f,
                    MazeMap::CommandPD::RawCommand);
            const CommandVector feedforwardCommand =
                plant.solveSteadyStateFeedforward(0.20f, 0.0f);

            AssertDriveCommandsEqual(feedforwardCommand, command, 1.0e-6f);
        }

        TEST_METHOD(DriveBasePointCommandCombinedRawMatchesPlantVelocityTargetFeedforwardAtTarget)
        {
            Vehicle vehicle;
            VehicleState runtimeState;
            PlantModel plant(vehicle, runtimeState);
            DriveBaseHarness driveHarness(plant, runtimeState, Config::kDriveBasePDCluster);
            DriveBase& drive = driveHarness.drive;
            Assert::IsTrue(drive.Begin());
            Assert::IsTrue(driveHarness.estimator.ResetPose(0.0f, 0.0f, 0.0f));

            const CommandVector command =
                drive.PointCommand(
                    0.20f,
                    0.40f,
                    MazeMap::CommandPD::RawCommand);
            const CommandVector feedforwardCommand =
                plant.solveSteadyStateFeedforward(0.20f, 0.40f);

            AssertDriveCommandsEqual(feedforwardCommand, command, 1.0e-6f);
        }

        TEST_METHOD(DriveBasePointYawRateCommandRawMatchesPlantVelocityTargetFeedforwardAtTarget)
        {
            Vehicle vehicle;
            VehicleState runtimeState;
            PlantModel plant(vehicle, runtimeState);
            DriveBaseHarness driveHarness(plant, runtimeState, Config::kDriveBasePDCluster);
            DriveBase& drive = driveHarness.drive;
            Assert::IsTrue(drive.Begin());
            Assert::IsTrue(driveHarness.estimator.ResetPose(0.0f, 0.0f, 0.0f));

            const CommandVector command =
                drive.PointYawRateCommand(
                    0.40f,
                    MazeMap::CommandPD::RawCommand);
            const CommandVector feedforwardCommand =
                plant.solveSteadyStateFeedforward(0.0f, 0.40f);

            AssertDriveCommandsEqual(feedforwardCommand, command, 1.0e-6f);
        }

        TEST_METHOD(DriveBasePointYawRateCommandStateYawPdAtRestMeetsInPlaceTurnMinimumDrive)
        {
            Vehicle vehicle;
            VehicleState runtimeState;
            PlantModel plant(vehicle, runtimeState);
            DriveBaseHarness driveHarness(plant, runtimeState, Config::kDriveBasePDCluster);
            DriveBase& drive = driveHarness.drive;
            Assert::IsTrue(drive.Begin());
            Assert::IsTrue(driveHarness.estimator.ResetPose(0.0f, 0.0f, 0.0f));
            UpdateDriveBaseSignals(
                drive,
                driveHarness.estimator,
                driveHarness.runtimeState,
                BuildDriveBaseSensorSnapshot(0.0f));

            const CommandVector command =
                drive.PointYawRateCommand(
                    3.0f,
                    MazeMap::CommandPD::StateYawPD);

            AssertPositiveInPlaceTurnCommandMeetsMinimumDrive(command);
        }

        TEST_METHOD(DriveBasePointYawRateCommandImuYawPdAtRestMeetsInPlaceTurnMinimumDrive)
        {
            Vehicle vehicle;
            VehicleState runtimeState;
            PlantModel plant(vehicle, runtimeState);
            DriveBaseHarness driveHarness(plant, runtimeState, Config::kDriveBasePDCluster);
            DriveBase& drive = driveHarness.drive;
            Assert::IsTrue(drive.Begin());
            Assert::IsTrue(driveHarness.estimator.ResetPose(0.0f, 0.0f, 0.0f));
            UpdateDriveBaseSignals(
                drive,
                driveHarness.estimator,
                driveHarness.runtimeState,
                BuildDriveBaseSensorSnapshot(0.0f));

            const CommandVector command =
                drive.PointYawRateCommand(
                    3.0f,
                    MazeMap::CommandPD::IMUYaw);

            AssertPositiveInPlaceTurnCommandMeetsMinimumDrive(command);
        }

        TEST_METHOD(DriveBasePointCommandWithHeadingTargetAtRestMeetsInPlaceTurnMinimumDrive)
        {
            Vehicle vehicle;
            VehicleState runtimeState;
            PlantModel plant(vehicle, runtimeState);
            DriveBaseHarness driveHarness(plant, runtimeState, Config::kDriveBasePDCluster);
            DriveBase& drive = driveHarness.drive;
            Assert::IsTrue(drive.Begin());
            Assert::IsTrue(driveHarness.estimator.ResetPose(0.0f, 0.0f, 0.0f));
            UpdateDriveBaseSignals(
                drive,
                driveHarness.estimator,
                driveHarness.runtimeState,
                BuildDriveBaseSensorSnapshot(0.0f));

            const CommandVector command =
                drive.PointCommandWithHeadingTarget(
                    0.0f,
                    0.0f,
                    0.15f,
                    MazeMap::CommandPD::RawCommand,
                    MazeMap::CommandPD::StateHeadingPD);

            AssertPositiveInPlaceTurnCommandMeetsMinimumDrive(command);
        }

        TEST_METHOD(DriveBaseRawFeedforwardMatchesPlantVelocityTargetSolve)
        {
            Vehicle vehicle;
            VehicleState runtimeState;
            PlantModel plant(vehicle, runtimeState);
            DriveBaseHarness driveHarness(plant, runtimeState, Config::kDriveBasePDCluster);
            DriveBase& drive = driveHarness.drive;
            Assert::IsTrue(drive.Begin());
            Assert::IsTrue(driveHarness.estimator.ResetPose(0.0f, 0.0f, 0.0f));

            const CommandVector feedforwardCommand =
                plant.solveSteadyStateFeedforward(
                    0.20f,
                    driveHarness.runtimeState.GetRotationalVelocity());
            const CommandVector command =
                drive.PointCommand(
                    0.20f,
                    MazeMap::CommandPD::RawCommand);

            AssertDriveCommandsEqual(feedforwardCommand, command, 1.0e-6f);
        }

        TEST_METHOD(DriveConfigYawRateTrackingUsesImuYawByDefault)
        {
            Assert::IsTrue(Config::kDriveYawRateCommandPd == MazeMap::CommandPD::IMUYaw);
        }

        TEST_METHOD(DriveBasePointCommandManeuverPointMatchesScalarTargets)
        {
            Vehicle vehicle;
            VehicleState runtimeState;
            PlantModel plant(vehicle, runtimeState);
            DriveBaseHarness driveHarness(plant, runtimeState, Config::kDriveBasePDCluster);
            DriveBase& drive = driveHarness.drive;
            Assert::IsTrue(drive.Begin());
            Assert::IsTrue(driveHarness.estimator.ResetPose(0.0f, 0.0f, 0.0f));
            UpdateDriveEstimator(
                drive,
                driveHarness.estimator,
                driveHarness.runtimeState,
                0.001f,
                BuildDriveBaseSensorSnapshot(0.15f));

            const ManeuverPoint point(0.0f, 0.0f, 0.25f, 0.30f, 0.20f);
            const CommandVector scalarCommand =
                drive.PointCommand(
                    point.Velocity,
                    point.Omega,
                    MazeMap::CommandPD::EncoderVelocity |
                    MazeMap::CommandPD::IMUYaw);
            const CommandVector pointCommand =
                drive.PointCommand(
                    point,
                    MazeMap::CommandPD::EncoderVelocity |
                    MazeMap::CommandPD::IMUYaw);

            Assert::IsTrue(IsFiniteControlVector(scalarCommand));
            Assert::IsTrue(IsFiniteControlVector(pointCommand));
            Assert::AreEqual(scalarCommand.LeftMotorPwm(), pointCommand.LeftMotorPwm(), 1.0e-6f);
            Assert::AreEqual(scalarCommand.RightMotorPwm(), pointCommand.RightMotorPwm(), 1.0e-6f);
        }

        TEST_METHOD(DriveBasePointCommandManeuverPointRejectsNonFiniteTargets)
        {
            Vehicle vehicle;
            VehicleState runtimeState;
            PlantModel plant(vehicle, runtimeState);
            DriveBaseHarness driveHarness(plant, runtimeState, Config::kDriveBasePDCluster);
            DriveBase& drive = driveHarness.drive;
            Assert::IsTrue(drive.Begin());
            Assert::IsTrue(driveHarness.estimator.ResetPose(0.0f, 0.0f, 0.0f));
            UpdateDriveEstimator(
                drive,
                driveHarness.estimator,
                driveHarness.runtimeState,
                0.001f,
                BuildDriveBaseSensorSnapshot(0.0f));

            const ManeuverPoint invalidPoint(
                0.0f,
                0.0f,
                0.0f,
                std::numeric_limits<float>::quiet_NaN(),
                0.20f);
            const CommandVector command =
                drive.PointCommand(
                    invalidPoint,
                    MazeMap::CommandPD::EncoderVelocity |
                    MazeMap::CommandPD::IMUYaw);

            Assert::AreEqual(0.0f, command.LeftMotorPwm(), 1.0e-6f);
            Assert::AreEqual(0.0f, command.RightMotorPwm(), 1.0e-6f);
        }

        TEST_METHOD(DriveBasePointCommandCoupledStateYawPdChangesCommandWhenEstimatorYawRateDiffers)
        {
            Vehicle vehicle;
            VehicleState runtimeState;
            PlantModel plant(vehicle, runtimeState);
            DriveBaseHarness driveHarness(plant, runtimeState, Config::kDriveBasePDCluster);
            DriveBase& drive = driveHarness.drive;
            Assert::IsTrue(drive.Begin());
            Assert::IsTrue(driveHarness.estimator.ResetPose(0.0f, 0.0f, 0.0f));
            PrimeDriveBaseWithEncoderDelta(drive, driveHarness.estimator, driveHarness.runtimeState, 24, -24);

            const CommandVector rawCommand =
                drive.PointCommand(
                    0.20f,
                    0.0f,
                    MazeMap::CommandPD::RawCommand);
            const CommandVector stateYawCommand =
                drive.PointCommand(
                    0.20f,
                    0.0f,
                    MazeMap::CommandPD::StateYawPD);
            AssertDriveCommandsDiffer(rawCommand, stateYawCommand);
        }

        TEST_METHOD(DriveBasePointCommandCoupledStateYawPdZeroGainMatchesRawWhenYawRateErrorExists)
        {
            Vehicle vehicle;
            VehicleState runtimeState;
            PlantModel plant(vehicle, runtimeState);
            MazeMap::ProportionalDerivativeCluster defaultCluster = Config::kDriveBasePDCluster;
            MazeMap::ProportionalDerivativeCluster zeroYawCluster = defaultCluster;
            zeroYawCluster.YawRateStatePD.SetGains(0.0f, 0.0f);
            DriveBaseHarness driveHarness(plant, runtimeState, defaultCluster);
            DriveBase& drive = driveHarness.drive;
            Assert::IsTrue(drive.Begin());
            Assert::IsTrue(driveHarness.estimator.ResetPose(0.0f, 0.0f, 0.0f));
            UpdateDriveBaseSignals(
                drive,
                driveHarness.estimator,
                driveHarness.runtimeState,
                BuildDriveBaseSensorSnapshot(
                    0.40f,
                    0.0f,
                    0.0f,
                    0.0f,
                    false));

            const CommandVector rawCommand =
                drive.PointCommand(
                    0.20f,
                    0.0f,
                    MazeMap::CommandPD::RawCommand);
            drive.SetProportionalDerivativeCluster(zeroYawCluster);
            const CommandVector zeroGainYawCommand =
                drive.PointCommand(
                    0.20f,
                    0.0f,
                    MazeMap::CommandPD::StateYawPD);

            AssertDriveCommandsEqual(rawCommand, zeroGainYawCommand);
        }

        TEST_METHOD(DriveBasePointCommandCoupledImuYawIsNoOpWhenOnlyEstimatorYawRateDiffers)
        {
            Vehicle vehicle;
            VehicleState runtimeState;
            PlantModel plant(vehicle, runtimeState);
            DriveBaseHarness driveHarness(plant, runtimeState, Config::kDriveBasePDCluster);
            DriveBase& drive = driveHarness.drive;
            Assert::IsTrue(drive.Begin());
            Assert::IsTrue(driveHarness.estimator.ResetPose(0.0f, 0.0f, 0.0f));
            UpdateDriveBaseSignals(
                drive,
                driveHarness.estimator,
                driveHarness.runtimeState,
                BuildDriveBaseSensorSnapshot(
                    0.40f,
                    0.0f,
                    0.0f,
                    0.0f,
                    false));

            const CommandVector rawCommand =
                drive.PointCommand(
                    0.20f,
                    0.0f,
                    MazeMap::CommandPD::RawCommand);
            const CommandVector imuYawCommand =
                drive.PointCommand(
                    0.20f,
                    0.0f,
                    MazeMap::CommandPD::IMUYaw);

            AssertDriveCommandsEqual(rawCommand, imuYawCommand);
        }

        TEST_METHOD(DriveBasePointCommandCoupledImuYawChangesCommandWhenImuYawRateDiffers)
        {
            Vehicle vehicle;
            VehicleState runtimeState;
            PlantModel plant(vehicle, runtimeState);
            DriveBaseHarness driveHarness(plant, runtimeState, Config::kDriveBasePDCluster);
            DriveBase& drive = driveHarness.drive;
            Assert::IsTrue(drive.Begin());
            Assert::IsTrue(driveHarness.estimator.ResetPose(0.0f, 0.0f, 0.0f));
            UpdateDriveBaseSignals(
                drive,
                driveHarness.estimator,
                driveHarness.runtimeState,
                BuildDriveBaseSensorSnapshot(
                    0.0f,
                    0.40f,
                    0.0f,
                    0.0f,
                    false));

            const CommandVector rawCommand =
                drive.PointCommand(
                    0.20f,
                    0.0f,
                    MazeMap::CommandPD::RawCommand);
            const CommandVector imuYawCommand =
                drive.PointCommand(
                    0.20f,
                    0.0f,
                    MazeMap::CommandPD::IMUYaw);

            AssertDriveCommandsDiffer(rawCommand, imuYawCommand);
        }

        TEST_METHOD(DriveBasePointCommandCoupledImuYawUsesGyroPdIndependentOfStateYawPd)
        {
            Vehicle vehicle;
            VehicleState runtimeState;
            PlantModel plant(vehicle, runtimeState);
            MazeMap::ProportionalDerivativeCluster cluster = Config::kDriveBasePDCluster;
            cluster.YawRateStatePD.SetGains(100.0f, 100.0f);
            cluster.YawRateGyroPD.SetGains(0.0f, 0.0f);
            DriveBaseHarness driveHarness(plant, runtimeState, cluster);
            DriveBase& drive = driveHarness.drive;
            Assert::IsTrue(drive.Begin());
            Assert::IsTrue(driveHarness.estimator.ResetPose(0.0f, 0.0f, 0.0f));
            UpdateDriveBaseSignals(
                drive,
                driveHarness.estimator,
                driveHarness.runtimeState,
                BuildDriveBaseSensorSnapshot(
                    0.0f,
                    0.40f,
                    0.0f,
                    0.0f,
                    false));

            const CommandVector rawCommand =
                drive.PointCommand(
                    0.20f,
                    0.0f,
                    MazeMap::CommandPD::RawCommand);
            const CommandVector imuYawCommand =
                drive.PointCommand(
                    0.20f,
                    0.0f,
                    MazeMap::CommandPD::IMUYaw);

            AssertDriveCommandsEqual(rawCommand, imuYawCommand);
        }

        TEST_METHOD(DriveBaseDeltaCommandStateAccelerationPdChangesCommandWhenStateAccelerationErrorExists)
        {
            Vehicle vehicle;
            VehicleState runtimeState;
            PlantModel plant(vehicle, runtimeState);
            DriveBaseHarness driveHarness(plant, runtimeState, Config::kDriveBasePDCluster);
            DriveBase& drive = driveHarness.drive;
            Assert::IsTrue(drive.Begin());
            Assert::IsTrue(driveHarness.estimator.ResetPose(0.0f, 0.0f, 0.0f));
            PlantModel::StateVector truthState = PlantModel::StateVector::Zero();
            float leftEncoderRemainderCounts = 0.0f;
            float rightEncoderRemainderCounts = 0.0f;
            CommandVector appliedCommand{};
            appliedCommand.SetLeftMotorPwm(0.80f);
            appliedCommand.SetRightMotorPwm(0.80f);
            drive.CommandGenerated(appliedCommand, 0.0f, 0.0f, false);
            for (int cycleIndex = 0; cycleIndex < ScaleLegacyLoopSteps(24); ++cycleIndex)
            {
                SimulateDriveBaseCycle(
                    drive,
                    driveHarness.estimator,
                    driveHarness.runtimeState,
                    plant,
                    truthState,
                    leftEncoderRemainderCounts,
                    rightEncoderRemainderCounts,
                    kDriveBaseLoopDtSeconds);
            }
            const float presentLinearSpeedMps = driveHarness.runtimeState.GetVelocity();

            const CommandVector rawCommand =
                drive.DeltaCommand(
                    presentLinearSpeedMps,
                    1.0f,
                    MazeMap::CommandPD::RawCommand);
            const CommandVector stateAccelerationCommand =
                drive.DeltaCommand(
                    presentLinearSpeedMps,
                    1.0f,
                    MazeMap::CommandPD::StateAccelerationPD);
            AssertDriveCommandsDiffer(rawCommand, stateAccelerationCommand);
        }

        TEST_METHOD(DriveBaseDeltaCommandStateAccelerationPdIsNoOpWhenOnlyImuForwardAccelDiffers)
        {
            Vehicle vehicle;
            VehicleState runtimeState;
            PlantModel plant(vehicle, runtimeState);
            DriveBaseHarness driveHarness(plant, runtimeState, Config::kDriveBasePDCluster);
            DriveBase& drive = driveHarness.drive;
            Assert::IsTrue(drive.Begin());
            Assert::IsTrue(driveHarness.estimator.ResetPose(0.0f, 0.0f, 0.0f));
            UpdateDriveBaseSignals(
                drive,
                driveHarness.estimator,
                driveHarness.runtimeState,
                BuildDriveBaseSensorSnapshot(
                    0.0f,
                    0.0f,
                    0.0f,
                    1.50f,
                    true));
            Assert::IsTrue(driveHarness.estimator.ResetPose(0.0f, 0.0f, 0.0f));

            const CommandVector rawCommand =
                drive.DeltaCommand(
                    0.0f,
                    0.0f,
                    MazeMap::CommandPD::RawCommand);
            const CommandVector stateAccelerationCommand =
                drive.DeltaCommand(
                    0.0f,
                    0.0f,
                    MazeMap::CommandPD::StateAccelerationPD);
            AssertDriveCommandsEqual(rawCommand, stateAccelerationCommand);
        }

        TEST_METHOD(DriveBaseDeltaCommandImuForwardAccelChangesCommandWhenImuForwardAccelDiffers)
        {
            Vehicle vehicle;
            VehicleState runtimeState;
            PlantModel plant(vehicle, runtimeState);
            DriveBaseHarness driveHarness(plant, runtimeState, Config::kDriveBasePDCluster);
            DriveBase& drive = driveHarness.drive;
            Assert::IsTrue(drive.Begin());
            Assert::IsTrue(driveHarness.estimator.ResetPose(0.0f, 0.0f, 0.0f));
            UpdateDriveBaseSignals(
                drive,
                driveHarness.estimator,
                driveHarness.runtimeState,
                BuildDriveBaseSensorSnapshot(
                    0.0f,
                    0.0f,
                    0.0f,
                    1.50f,
                    true));

            const CommandVector rawCommand =
                drive.DeltaCommand(
                    0.0f,
                    0.0f,
                    MazeMap::CommandPD::RawCommand);
            const CommandVector imuAccelerationCommand =
                drive.DeltaCommand(
                    0.0f,
                    0.0f,
                    MazeMap::CommandPD::IMUForwardAccel);

            AssertDriveCommandsDiffer(rawCommand, imuAccelerationCommand);
        }

        TEST_METHOD(DriveBaseDeltaCommandImuForwardAccelUsesImuAccelPdIndependentOfStateAccelerationPd)
        {
            Vehicle vehicle;
            VehicleState runtimeState;
            PlantModel plant(vehicle, runtimeState);
            MazeMap::ProportionalDerivativeCluster cluster = Config::kDriveBasePDCluster;
            cluster.LongitudinalAccelerationStatePD.SetGains(100.0f, 100.0f);
            cluster.LongitudinalAccelerationIMUForwardAccelPD.SetGains(0.0f, 0.0f);
            DriveBaseHarness driveHarness(plant, runtimeState, cluster);
            DriveBase& drive = driveHarness.drive;
            Assert::IsTrue(drive.Begin());
            Assert::IsTrue(driveHarness.estimator.ResetPose(0.0f, 0.0f, 0.0f));
            UpdateDriveBaseSignals(
                drive,
                driveHarness.estimator,
                driveHarness.runtimeState,
                BuildDriveBaseSensorSnapshot(
                    0.0f,
                    0.0f,
                    0.0f,
                    1.50f,
                    true));

            const CommandVector rawCommand =
                drive.DeltaCommand(
                    0.0f,
                    0.0f,
                    MazeMap::CommandPD::RawCommand);
            const CommandVector imuAccelerationCommand =
                drive.DeltaCommand(
                    0.0f,
                    0.0f,
                    MazeMap::CommandPD::IMUForwardAccel);

            AssertDriveCommandsEqual(rawCommand, imuAccelerationCommand);
        }

        TEST_METHOD(DriveBasePointCommandLinearOnlyStateVelocityPdIsNoOpAtMatchingTarget)
        {
            Vehicle vehicle;
            VehicleState runtimeState;
            PlantModel plant(vehicle, runtimeState);
            DriveBaseHarness driveHarness(plant, runtimeState, Config::kDriveBasePDCluster);
            DriveBase& drive = driveHarness.drive;
            Assert::IsTrue(drive.Begin());
            Assert::IsTrue(driveHarness.estimator.ResetPose(0.0f, 0.0f, 0.0f));
            PrimeDriveBaseWithEncoderDelta(drive, driveHarness.estimator, driveHarness.runtimeState, 48, 48);
            Assert::IsTrue(driveHarness.estimator.ResetPose(0.0f, 0.0f, 0.0f));

            const CommandVector rawCommand =
                drive.PointCommand(
                    0.0f,
                    MazeMap::CommandPD::RawCommand);
            const CommandVector stateVelocityCommand =
                drive.PointCommand(
                    0.0f,
                    MazeMap::CommandPD::StateVelocityPD);
            AssertDriveCommandsEqual(rawCommand, stateVelocityCommand);
        }

        TEST_METHOD(DriveBasePointCommandLinearOnlyStateVelocityPdZeroGainMatchesRawWhenVelocityErrorExists)
        {
            Vehicle vehicle;
            VehicleState runtimeState;
            PlantModel plant(vehicle, runtimeState);
            MazeMap::ProportionalDerivativeCluster defaultCluster = Config::kDriveBasePDCluster;
            MazeMap::ProportionalDerivativeCluster zeroVelocityCluster = defaultCluster;
            zeroVelocityCluster.VelocityStatePD.SetGains(0.0f, 0.0f);
            DriveBaseHarness driveHarness(plant, runtimeState, defaultCluster);
            DriveBase& drive = driveHarness.drive;
            Assert::IsTrue(drive.Begin());
            Assert::IsTrue(driveHarness.estimator.ResetPose(0.0f, 0.0f, 0.0f));
            PrimeDriveBaseWithEncoderDelta(drive, driveHarness.estimator, driveHarness.runtimeState, 48, 48);

            const CommandVector rawCommand =
                drive.PointCommand(
                    0.0f,
                    MazeMap::CommandPD::RawCommand);
            drive.SetProportionalDerivativeCluster(zeroVelocityCluster);
            const CommandVector zeroGainVelocityCommand =
                drive.PointCommand(
                    0.0f,
                    MazeMap::CommandPD::StateVelocityPD);

            AssertDriveCommandsEqual(rawCommand, zeroGainVelocityCommand);
        }

        TEST_METHOD(DriveBasePointCommandLinearOnlyEncoderVelocityChangesCommandWhenEncoderVelocityDiffers)
        {
            Vehicle vehicle;
            VehicleState runtimeState;
            PlantModel plant(vehicle, runtimeState);
            DriveBaseHarness driveHarness(plant, runtimeState, Config::kDriveBasePDCluster);
            DriveBase& drive = driveHarness.drive;
            Assert::IsTrue(drive.Begin());
            Assert::IsTrue(driveHarness.estimator.ResetPose(0.0f, 0.0f, 0.0f));
            PrimeDriveBaseWithEncoderDelta(drive, driveHarness.estimator, driveHarness.runtimeState, 48, 48);
            Assert::IsTrue(driveHarness.estimator.ResetPose(0.0f, 0.0f, 0.0f));

            const CommandVector rawCommand =
                drive.PointCommand(
                    0.0f,
                    MazeMap::CommandPD::RawCommand);
            const CommandVector encoderVelocityCommand =
                drive.PointCommand(
                    0.0f,
                    MazeMap::CommandPD::EncoderVelocity);

            AssertDriveCommandsDiffer(rawCommand, encoderVelocityCommand);
        }

        TEST_METHOD(DriveBaseCachesGeneratedFeedforwardAndFeedbackTelemetryAsAverageDelta)
        {
            Vehicle vehicle;
            VehicleState runtimeState;
            PlantModel plant(vehicle, runtimeState);
            DriveBaseHarness driveHarness(plant, runtimeState, Config::kDriveBasePDCluster);
            DriveBase& drive = driveHarness.drive;
            Assert::IsTrue(drive.Begin());
            Assert::IsTrue(driveHarness.estimator.ResetPose(0.0f, 0.0f, 0.0f));
            PrimeDriveBaseWithEncoderDelta(drive, driveHarness.estimator, driveHarness.runtimeState, 48, 24);
            Assert::IsTrue(driveHarness.estimator.ResetPose(0.0f, 0.0f, 0.0f));

            const CommandVector rawCommand =
                drive.PointCommand(
                    0.0f,
                    MazeMap::CommandPD::RawCommand);

            Assert::AreEqual(
                ControlVectorAverage(rawCommand),
                ControlVectorAverage(drive.GetLastFeedforward()),
                1.0e-6f);
            Assert::AreEqual(
                ControlVectorDelta(rawCommand),
                ControlVectorDelta(drive.GetLastFeedforward()),
                1.0e-6f);
            Assert::AreEqual(0.0f, ControlVectorAverage(drive.GetLastFeedback()), 1.0e-6f);
            Assert::AreEqual(0.0f, ControlVectorDelta(drive.GetLastFeedback()), 1.0e-6f);

            const CommandVector encoderVelocityCommand =
                drive.PointCommand(
                    0.0f,
                    MazeMap::CommandPD::EncoderVelocity);
            const CommandVector feedbackOnly =
                CommandVector(
                    encoderVelocityCommand.LeftMotorPwm() - rawCommand.LeftMotorPwm(),
                    encoderVelocityCommand.RightMotorPwm() - rawCommand.RightMotorPwm());

            Assert::AreEqual(
                ControlVectorAverage(rawCommand),
                ControlVectorAverage(drive.GetLastFeedforward()),
                1.0e-6f);
            Assert::AreEqual(
                ControlVectorDelta(rawCommand),
                ControlVectorDelta(drive.GetLastFeedforward()),
                1.0e-6f);
            Assert::AreEqual(
                ControlVectorAverage(feedbackOnly),
                ControlVectorAverage(drive.GetLastFeedback()),
                1.0e-6f);
            Assert::AreEqual(
                ControlVectorDelta(feedbackOnly),
                ControlVectorDelta(drive.GetLastFeedback()),
                1.0e-6f);
        }

#define DEFINE_PD_CLUSTER_OSCILLATION_TESTS(TestStem, PairingKindExpr, PdExpr)                         \
        TEST_METHOD(TestStem##MetricsFinite)                                                             \
        {                                                                                               \
            AssertOscillationPairingMetricsFinite(PdExpr, PairingKindExpr);                              \
        }                                                                                               \
        TEST_METHOD(TestStem##SignalJitterLow)                                                           \
        {                                                                                               \
            AssertOscillationPairingSignalJitterLow(PdExpr, PairingKindExpr);                            \
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
            DriveBaseOscillationPairingWheelVelocityEncoderPd,
            OscillationPairingKind::WheelVelocityEncoder,
            Config::kDriveBasePDCluster.WheelVelocityEncoderPD)

#undef DEFINE_PD_CLUSTER_OSCILLATION_TESTS

    };
}

