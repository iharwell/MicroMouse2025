
#pragma once
// Declares the vehicle process model and plant-side data types used by the micromouse UKF stack.

#include "Defines.h"
#include "EigenCompat.h"
#include "CommandVector.h"
#include "EncoderObs.h"
#include "Maze.h"
#include "SensorMount.h"
#include "Vehicle.h"
#include "VehicleState.h"

#include <algorithm>
#include <array>
#include <cstdarg>
#include <cstdint>

namespace MazeMap
{
    class SrUkfCore;
    class MotorEncoderDrive;

    // Shared vehicle plant owner for runtime dynamics, acceleration feedforward, and plant-side diagnostics.
    class EXPORT PlantModel
    {
    public:
        static constexpr float kDefaultVelocityTargetResponseTimeS = 0.025f;
        static constexpr float kTractionLimitedReserveScale = 0.90f;

        PlantModel(const Vehicle& vehicle, VehicleState& runtimeState) noexcept;

        template <typename WriteFloatFn>
        bool WriteOpenFloorUkfMetadata(WriteFloatFn&& writeFloat) const
        {
            return
                writeFloat("re_m", Vehicle::GetDriveWheelRadiusM(), 6) &&
                writeFloat("we_m", _vehicle.GetTrackWidth(), 6) &&
                writeFloat("imu_x", Vehicle::GetBackLeftImuMount().positionBodyM().x(), 6) &&
                writeFloat("imu_y", Vehicle::GetBackLeftImuMount().positionBodyM().y(), 6) &&
                writeFloat(
                    "jw_kgm2",
                    0.5f * (_leftDrive.getEquivalentWheelInertiaKgM2() + _rightDrive.getEquivalentWheelInertiaKgM2()),
                    9);
        }

        template <typename WriteConfigFn>
        bool WriteDriveModelDiagnosticConfig(WriteConfigFn&& writeConfig) const
        {
            return writeConfig(
                "motor_model:wheel_diam_m=%.6f;encoder_cpr=%lu;gear=%.6f;supply_v=%.3f",
                2.0f * Vehicle::GetDriveWheelRadiusM(),
                static_cast<unsigned long>(_leftDrive.getPulsesPerRev()),
                _leftDrive.getGearRatio(),
                _vehicle.GetBatteryVoltage());
        }

        using DebugTextSink = bool (*)(void* context, const char* type, const char* format, std::va_list args) noexcept;
        bool WriteUkfPlantDebugTextDump(void* context, DebugTextSink sink) const noexcept;

        void integrate(const App::Internal::CommandVector& control, float dt) noexcept;

        App::Internal::CommandVector ComputeFeedforward(
            float desiredAccelMps2,
            float desiredYawAccelRadps2) const noexcept;

        Eigen::Matrix<float, 2, 2> encoderPairCovarianceRadps(
            float linearSpeedSigmaMps,
            float yawRateSigmaRadps) const noexcept;
        Eigen::Matrix<float, 2, 2> encoderPairSqrtNoise(
            const EncoderObs& observation,
            float stationaryLinearSpeedSigmaMps,
            float generalLinearSpeedSigmaMps,
            float generalYawRateSigmaRadps) const noexcept;
        float stationaryEncoderOmegaSigmaRadps(float stationaryLinearSpeedSigmaMps) const noexcept;
        float measuredLinearSpeedMps(const EncoderObs& observation) const noexcept;
        float measuredYawRateRadps(const EncoderObs& observation) const noexcept;
        float measuredYawRateVarianceRadps2(
            const EncoderObs& observation,
            float stationaryLinearSpeedSigmaMps,
            float generalLinearSpeedSigmaMps,
            float generalYawRateSigmaRadps) const noexcept;
        float measuredWheelVarianceRadps2(
            const EncoderObs& observation,
            float stationaryLinearSpeedSigmaMps,
            float generalLinearSpeedSigmaMps,
            float generalYawRateSigmaRadps) const noexcept;
        float sustainedCombinedAccelerationUsage(float accelerationMps2) const noexcept;
        float nominalCombinedAccelerationUsage(float accelerationMps2) const noexcept;
        float peakCombinedAccelerationUsage(float accelerationMps2) const noexcept;
        float stopExitYawRateUsage(float yawRateRadps) const noexcept;
        float totalForwardContactForceN(const App::Internal::CommandVector& control) const noexcept;
        float totalRightContactForceN(const App::Internal::CommandVector& control) const noexcept;
        float leftBankForwardContactForceN(const App::Internal::CommandVector& control) const noexcept;
        float rightBankForwardContactForceN(const App::Internal::CommandVector& control) const noexcept;
        float contactRightForceN(const App::Internal::CommandVector& control, uint8_t contactIndex) const noexcept;
        float contactForwardForceN(const App::Internal::CommandVector& control, uint8_t contactIndex) const noexcept;
        float contactNormalLoadN(uint8_t contactIndex) const noexcept;
        float totalContactNormalLoadN() const noexcept;

        void velocityTargetTechnicalLimits(
            float& maxLongitudinalAccelMps2,
            float& maxYawAccelRadps2) const noexcept;
        void velocityTargetTechnicalLimits(
            float forwardVelocityMps,
            float yawRateRadps,
            float& maxLongitudinalAccelMps2,
            float& maxYawAccelRadps2) const noexcept;

        float driveFrictionTorque(
            float wheelBankSpeedRadps,
            float wheelTorqueRequestNm) const noexcept;
        float staticFrictionTorqueNm() const noexcept;
        float rollingFrictionTorqueNm() const noexcept;
        float staticFrictionSpeedThresholdRadps() const noexcept;
        float leftDriveEquivalentWheelInertiaKgM2() const noexcept;
        float rightDriveEquivalentWheelInertiaKgM2() const noexcept;
        float leftDriveLongitudinalTireStiffnessN() const noexcept;
        float rightDriveLongitudinalTireStiffnessN() const noexcept;
        float contactSaturation(const App::Internal::CommandVector& control, uint8_t contactIndex) const noexcept;
        float contactPreProjectionUtilization(
            const App::Internal::CommandVector& control,
            uint8_t contactIndex) const noexcept;
        float contactLateralSlipAngleRad(uint8_t contactIndex) const noexcept;
        float backLeftImuRightAccelerationMps2(
            const App::Internal::CommandVector& control) const noexcept;
        float backLeftImuForwardAccelerationMps2(
            const App::Internal::CommandVector& control) const noexcept;

    private:
        friend class SrUkfCore;

        using StateVector = VehicleState::StateVector;

        struct ContactKinematics
        {
            float rightVelocityMps = 0.0f;
            float forwardVelocityMps = 0.0f;
        };

        struct WheelKinematics
        {
            float leftBankForwardVelocityMps = 0.0f;
            float rightBankForwardVelocityMps = 0.0f;
            std::array<ContactKinematics, 4> contacts{};
        };

        struct SlipTargets
        {
            float kappaLeft = 0.0f;
            float kappaRight = 0.0f;
            std::array<float, 4> lateralRatio{};
        };

        struct ContactForce
        {
            float rightForceN = 0.0f;
            float forwardForceN = 0.0f;
            float normalForceN = 0.0f;
            float saturation = 0.0f;
            float preProjectionUtilization = 0.0f;
        };

        struct ContactForces
        {
            std::array<ContactForce, 4> contacts{};

            float SumRightForceN() const noexcept
            {
                float sum = 0.0f;
                for (const ContactForce& contact : contacts)
                {
                    sum += contact.rightForceN;
                }
                return sum;
            }

            float SumForwardForceN() const noexcept
            {
                float sum = 0.0f;
                for (const ContactForce& contact : contacts)
                {
                    sum += contact.forwardForceN;
                }
                return sum;
            }

            float LeftBankForwardForceN() const noexcept
            {
                return contacts[0].forwardForceN + contacts[2].forwardForceN;
            }

            float RightBankForwardForceN() const noexcept
            {
                return contacts[1].forwardForceN + contacts[3].forwardForceN;
            }

            float LeftBankMaxPreProjectionUtilization() const noexcept
            {
                return (std::max)(contacts[0].preProjectionUtilization, contacts[2].preProjectionUtilization);
            }

            float RightBankMaxPreProjectionUtilization() const noexcept
            {
                return (std::max)(contacts[1].preProjectionUtilization, contacts[3].preProjectionUtilization);
            }
        };

        enum class MotionRegime : uint8_t
        {
            StoppedHold = 0,
            RollingAdherent = 1,
            RollingSaturated = 2,
        };

        struct PlantDerivatives
        {
            StateVector stateDot = StateVector::Zero();
            ContactForces contactForces{};
            WheelKinematics wheelKinematics{};
            SlipTargets slipTargets{};
            Eigen::Vector2f originAccelBodyMps2 = Eigen::Vector2f::Zero();
            Eigen::Vector2f imuAccelBodyMps2 = Eigen::Vector2f::Zero();
            float longitudinalAccelMps2 = 0.0f;
            float lateralAccelMps2 = 0.0f;
            float yawAccelRadps2 = 0.0f;
            MotionRegime regime = MotionRegime::RollingAdherent;
            float maxContactUtilization = 0.0f;
        };

        StateVector BuildBoundStateVector() const noexcept;
        void ApplyStateVectorToBoundState(const StateVector& state) noexcept;
        void resolveAppliedBankTorques(
            const StateVector& currentState,
            const App::Internal::CommandVector& control,
            float& leftAppliedBankTorqueNm,
            float& rightAppliedBankTorqueNm) const noexcept;
        struct WheelOnlyMeasurementPrediction
        {
            float leftWheelSpeedRadps = 0.0f;
            float rightWheelSpeedRadps = 0.0f;
            float forwardSpeedMps = 0.0f;
            float yawRateRadps = 0.0f;
        };

        WheelOnlyMeasurementPrediction predictWheelOnlyMeasurement(
            const StateVector& state) const noexcept;

        PlantDerivatives forwardStep(
            const App::Internal::CommandVector& control) const noexcept;
        PlantDerivatives forwardStep(
            const StateVector& state,
            const App::Internal::CommandVector& control) const noexcept;
        PlantDerivatives forwardStepFromAppliedBankTorques(
            const StateVector& state,
            float leftAppliedBankTorqueNm,
            float rightAppliedBankTorqueNm) const noexcept;
        PlantDerivatives evaluateAppliedBankTorqueStep(
            const StateVector& state,
            float leftAppliedBankTorqueNm,
            float rightAppliedBankTorqueNm,
            float activityNorm) const noexcept;
        WheelKinematics wheelKinematics(const StateVector& state) const noexcept;
        SlipTargets slipTargets(const StateVector& state) const noexcept;
        SlipTargets slipTargets(
            const StateVector& state,
            const WheelKinematics& kinematics) const noexcept;
        ContactForces tireForces(const StateVector& state) const noexcept;
        ContactForces tireForces(
            const StateVector& state,
            const App::Internal::CommandVector& control) const noexcept;
        Eigen::Vector2f imuPlanarAcceleration(
            const StateVector& state,
            const App::Internal::CommandVector& control) const noexcept;
        StateVector integrateAppliedBankTorques(
            const StateVector& state,
            float leftAppliedBankTorqueNm,
            float rightAppliedBankTorqueNm,
            float dtS) const noexcept;
        static StateVector advanceStateFromDerivatives(
            const StateVector& currentState,
            const PlantDerivatives& evaluatedStep,
            float dtS) noexcept;
        Eigen::Vector2f wheelLinearVelocityFromBodyState(const StateVector& state) const noexcept;

        const Vehicle& _vehicle;
        VehicleState& _runtimeState;
        const MotorEncoderDrive& _leftDrive;
        const MotorEncoderDrive& _rightDrive;
    };
}
