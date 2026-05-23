
#pragma once
// Declares the vehicle process model and plant-side data types used by the micromouse Estimator.

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
    class Estimator;
    class MotorEncoderDrive;

    // Shared vehicle plant owner for runtime dynamics, acceleration feedforward, and plant-side diagnostics.
    class EXPORT PlantModel
    {
    public:
        static constexpr float kDefaultVelocityTargetResponseTimeS = 0.025f;

        PlantModel(const Vehicle& vehicle, VehicleState& runtimeState) noexcept;

        template <typename WriteFloatFn>
        bool WriteOpenFloorPlantMetadata(WriteFloatFn&& writeFloat) const
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

        bool WritePlantDebugTextDump(
            void* context,
            bool (*sink)(void* context, const char* type, const char* format, std::va_list args) noexcept)
            const noexcept;

        // `control` is the active command for the state interval being integrated.
        void integrate(const App::Internal::CommandVector& control, float dt) noexcept;

        App::Internal::CommandVector ComputeFeedforward(
            float desiredAccelMps2,
            float desiredYawAccelRadps2) const noexcept;
        static float forwardAccelerationResidualDecayAlpha(float dtS) noexcept;
        static float rightAccelerationResidualDecayAlpha(float dtS) noexcept;
        static float yawAccelerationResidualDecayAlpha(float dtS) noexcept;
        Eigen::Matrix<float, 2, 2> encoderPairSqrtNoise(
            const EncoderObs& observation,
            float stationaryLinearSpeedSigmaMps,
            float generalLinearSpeedSigmaMps,
            float generalYawRateSigmaRadps) const noexcept;
        float stationaryEncoderWheelSpeedSigmaRadps(float stationaryLinearSpeedSigmaMps) const noexcept;
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
        float contactForwardRelativeVelocityMps(uint8_t contactIndex) const noexcept;
        float contactRightRelativeVelocityMps(uint8_t contactIndex) const noexcept;
        float backLeftImuRightAccelerationMps2(
            const App::Internal::CommandVector& control) const noexcept;
        float backLeftImuForwardAccelerationMps2(
            const App::Internal::CommandVector& control) const noexcept;

    private:
        friend class Estimator;

        static constexpr float kForwardAccelerationResidualDecayTauS = 0.075f;
        static constexpr float kRightAccelerationResidualDecayTauS = 0.075f;
        static constexpr float kYawAccelResidualDecayTauS = 0.075f;
        static constexpr uint8_t kFrontLeft = 0U;
        static constexpr uint8_t kFrontRight = 1U;
        static constexpr uint8_t kRearLeft = 2U;
        static constexpr uint8_t kRearRight = 3U;
        static constexpr float kSignEpsilon = 1.0e-6f;
        static constexpr float kForceEpsilonN = 1.0e-4f;
        static constexpr float kRollingFrictionTorqueNm = 0.00372f;
        static constexpr float kReliableLaunchDriveCommand = 0.30f;
        static constexpr float kStaticFrictionMaxSpeedMps = 0.005f;
        static constexpr float kViscousFrictionNmPerRadps = 0.0f;
        static constexpr float kRightVelocityDampingNsPerM = 0.0f;
        static constexpr float kYawRateDampingNmsPerRad = 0.0f;
        static constexpr float kFrontLoadFraction = 0.5f;
        static constexpr float kMuFront = 1.65f;
        static constexpr float kMuRear = 1.65f;
        static constexpr float kStopExitYawRateRadps = 0.50f;

        class ContactKinematics
        {
            friend class PlantModel;

        public:
            ContactKinematics() noexcept = default;

        private:
            float _rightRelativeVelocityMps = 0.0f;
            float _forwardRelativeVelocityMps = 0.0f;
        };

        class WheelKinematics
        {
            friend class PlantModel;

        public:
            WheelKinematics() noexcept = default;

        private:
            float _leftBankForwardRelativeVelocityMps = 0.0f;
            float _rightBankForwardRelativeVelocityMps = 0.0f;
            std::array<ContactKinematics, 4> _contacts{};
        };

        class ContactForce
        {
            friend class PlantModel;

        public:
            ContactForce() noexcept = default;

        private:
            float _rightForceN = 0.0f;
            float _forwardForceN = 0.0f;
            float _normalForceN = 0.0f;
            float _saturation = 0.0f;
            float _preProjectionUtilization = 0.0f;
        };

        class ContactForces
        {
            friend class PlantModel;

        public:
            ContactForces() noexcept = default;

            float SumRightForceN() const noexcept
            {
                float sum = 0.0f;
                for (const ContactForce& contact : _contacts)
                {
                    sum += contact._rightForceN;
                }
                return sum;
            }

            float SumForwardForceN() const noexcept
            {
                float sum = 0.0f;
                for (const ContactForce& contact : _contacts)
                {
                    sum += contact._forwardForceN;
                }
                return sum;
            }

            float LeftBankForwardForceN() const noexcept
            {
                return _contacts[0]._forwardForceN + _contacts[2]._forwardForceN;
            }

            float RightBankForwardForceN() const noexcept
            {
                return _contacts[1]._forwardForceN + _contacts[3]._forwardForceN;
            }

            float LeftBankMaxPreProjectionUtilization() const noexcept
            {
                return (std::max)(_contacts[0]._preProjectionUtilization, _contacts[2]._preProjectionUtilization);
            }

            float RightBankMaxPreProjectionUtilization() const noexcept
            {
                return (std::max)(_contacts[1]._preProjectionUtilization, _contacts[3]._preProjectionUtilization);
            }

        private:
            std::array<ContactForce, 4> _contacts{};
        };

        class PlantDerivatives
        {
            friend class PlantModel;

        public:
            PlantDerivatives() noexcept = default;

        private:
            Eigen::Matrix<float, VehicleState::kDimension, 1> _stateDot = Eigen::Matrix<float, VehicleState::kDimension, 1>::Zero();
            ContactForces _contactForces{};
            WheelKinematics _wheelKinematics{};
            Eigen::Vector2f _originAccelBodyMps2 = Eigen::Vector2f::Zero();
            Eigen::Vector2f _imuAccelBodyMps2 = Eigen::Vector2f::Zero();
            float _forwardAccelMps2 = 0.0f;
            float _rightAccelMps2 = 0.0f;
            float _yawAccelRadps2 = 0.0f;
            float _maxContactUtilization = 0.0f;
        };

        Eigen::Matrix<float, VehicleState::kDimension, 1> BuildBoundStateVector() const noexcept;
        void ApplyStateVectorToBoundState(const Eigen::Matrix<float, VehicleState::kDimension, 1>& state) noexcept;
        // `control` is the active command for the state interval being predicted.
        Eigen::Matrix<float, VehicleState::kDimension, 1> predictStateFromCommandReference(
            const Eigen::Matrix<float, VehicleState::kDimension, 1>& referenceState,
            const Eigen::Matrix<float, VehicleState::kDimension, 1>& state,
            const App::Internal::CommandVector& control,
            float dtS,
            const EncoderObs* encoderInput) const noexcept;
        // `control` is the active command for the state interval whose activity is calculated.
        void plantActivityForState(
            const Eigen::Matrix<float, VehicleState::kDimension, 1>& state,
            const App::Internal::CommandVector& control,
            const EncoderObs* encoderInput,
            float& forwardAccelMps2,
            float& rightAccelMps2,
            float& yawAccelRadps2,
            float& maxContactRelativeSpeedMps,
            float& maxContactUtilization,
            float& maxContactSaturation,
            float& totalNormalLoadN) const noexcept;
        Eigen::Vector2f backLeftImuPlanarAccelerationForState(
            const Eigen::Matrix<float, VehicleState::kDimension, 1>& state,
            const App::Internal::CommandVector& control,
            const EncoderObs* encoderInput) const noexcept;
        Eigen::Matrix<float, 2, 2> encoderPairCovarianceRadps(
            float linearSpeedSigmaMps,
            float yawRateSigmaRadps) const noexcept;
        void resolveAppliedBankTorques(
            const Eigen::Matrix<float, VehicleState::kDimension, 1>& currentState,
            const App::Internal::CommandVector& control,
            float& leftAppliedBankTorqueNm,
            float& rightAppliedBankTorqueNm,
            const EncoderObs* encoderInput) const noexcept;
        static bool WriteDebugTextLine(
            void* context,
            bool (*sink)(void* context, const char* type, const char* format, std::va_list args) noexcept,
            const char* type,
            const char* format,
            ...) noexcept;
        static float SignedDirection(float preferredValue, float fallbackValue) noexcept;
        static float residualDecayAlpha(float dtS, float tauS) noexcept;
        PlantDerivatives forwardStep(
            const App::Internal::CommandVector& control) const noexcept;
        PlantDerivatives forwardStep(
            const Eigen::Matrix<float, VehicleState::kDimension, 1>& state,
            const App::Internal::CommandVector& control,
            const EncoderObs* encoderInput) const noexcept;
        PlantDerivatives forwardStepFromAppliedBankTorques(
            const Eigen::Matrix<float, VehicleState::kDimension, 1>& state,
            float leftAppliedBankTorqueNm,
            float rightAppliedBankTorqueNm) const noexcept;
        PlantDerivatives evaluateAppliedBankTorqueStep(
            const Eigen::Matrix<float, VehicleState::kDimension, 1>& state,
            float leftAppliedBankTorqueNm,
            float rightAppliedBankTorqueNm,
            const EncoderObs* encoderInput) const noexcept;
        WheelKinematics wheelKinematics(
            const Eigen::Matrix<float, VehicleState::kDimension, 1>& state,
            const EncoderObs* encoderInput) const noexcept;
        Eigen::Vector2f imuPlanarAcceleration(
            const Eigen::Matrix<float, VehicleState::kDimension, 1>& state,
            const App::Internal::CommandVector& control,
            const EncoderObs* encoderInput) const noexcept;
        Eigen::Matrix<float, VehicleState::kDimension, 1> integrateAppliedBankTorques(
            const Eigen::Matrix<float, VehicleState::kDimension, 1>& state,
            float leftAppliedBankTorqueNm,
            float rightAppliedBankTorqueNm,
            float dtS,
            const EncoderObs* encoderInput) const noexcept;
        Eigen::Matrix<float, VehicleState::kDimension, 1> predictStateWithAppliedBankTorques(
            const Eigen::Matrix<float, VehicleState::kDimension, 1>& state,
            float leftAppliedBankTorqueNm,
            float rightAppliedBankTorqueNm,
            float dtS,
            const EncoderObs* encoderInput) const noexcept;
        static Eigen::Matrix<float, VehicleState::kDimension, 1> advanceStateFromDerivatives(
            const Eigen::Matrix<float, VehicleState::kDimension, 1>& currentState,
            const PlantDerivatives& evaluatedStep,
            float dtS) noexcept;
        Eigen::Vector2f wheelLinearVelocityFromBodyState(const Eigen::Matrix<float, VehicleState::kDimension, 1>& state) const noexcept;

        const Vehicle& _vehicle;
        VehicleState& _runtimeState;
        const MotorEncoderDrive& _leftDrive;
        const MotorEncoderDrive& _rightDrive;
    };
}
