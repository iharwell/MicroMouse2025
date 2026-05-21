
#pragma once
// Declares the vehicle process model and plant-side data types used by the micromouse UKF stack.

#include "Defines.h"
#include "EigenCompat.h"
#include "CommandVector.h"
#include "EncoderObs.h"
#include "Maze.h"
#include "SensorMount.h"
#include "VehicleState.h"

#include <algorithm>
#include <array>
#include <cstdarg>
#include <cstdint>

namespace MazeMap
{
    class SrUkfCore;
    class Vehicle;
    class MotorEncoderDrive;

    // Private implementation details.
    // Contact-point velocity resolved in the project body frame (+X right, +Y forward).
    struct ContactKinematics
    {
        float rightVelocityMps = 0.0f;
        float forwardVelocityMps = 0.0f;
    };

    // Private implementation details.
    // Per-wheel-bank and per-contact kinematics derived from the current vehicle state.
    struct WheelKinematics
    {
        float leftBankForwardVelocityMps = 0.0f;
        float rightBankForwardVelocityMps = 0.0f;
        std::array<ContactKinematics, 4> contacts{};
    };

    // Private implementation details.
    // Diagnostic longitudinal slip and lateral slip ratios for each tire contact.
    struct SlipTargets
    {
        float kappaLeft = 0.0f;
        float kappaRight = 0.0f;
        std::array<float, 4> lateralRatio{};
    };

    // Private implementation details.
    // Force request and normalized traction utilization for one tire contact patch.
    struct ContactForce
    {
        float rightForceN = 0.0f;
        float forwardForceN = 0.0f;
        float normalForceN = 0.0f;
        float saturation = 0.0f;
        float preProjectionUtilization = 0.0f;
    };

    // Private implementation details.
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

    // Unsanctioned classifier.
    enum class MotionRegime : uint8_t
    {
        StoppedHold = 0,
        RollingAdherent = 1,
        RollingSaturated = 2,
    };

    // One evaluated plant derivative step plus the algebraic quantities used to produce it.
    struct PlantDerivatives
    {
        VehicleState::StateVector stateDot = VehicleState::StateVector::Zero();
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

    // Unsanctioned proxy for Vehicle and Vehicle-owned subsystem data. Slated for removal.
    struct PlantParams
    {
        float massKg;
        float effectiveLongitudinalMassKg;
        float effectiveLateralMassKg = 0.0f;
        float yawInertiaKgM2;
        float trackWidthM;
        float contactPatchLongitudinalOffsetM;
        float wheelRadiusM;
        float equivalentWheelInertiaKgM2 = 0.0f;

        float supplyVoltageV;
        float driveResistanceOhms;
        float torqueConstantNmPerA;
        float speedConstantRadpsPerVolt;
        float noLoadCurrentA;
        float motorCurrentLimitA;
        float gearRatio;
        uint16_t encoderCountsPerMotorRev;

        float drivetrainEfficiency = 1.0f;
        // April 10, 2026 latest testing-area open-floor launch fit (run_id=ofm_10696927)
        // left the rolling-region drag at about 0.00372 Nm once launch breakaway was split
        // out into the explicit static-friction term below.
        float rollingFrictionTorqueNm = 0.00372f;
        // April 11, 2026 initial breakaway estimate: assume about 0.30 normalized drive is
        // needed to overcome stiction reliably. PlantModel converts the +/-0.005 m/s window
        // into wheel-bank rad/s internally before applying this torque.
        float staticFrictionTorqueNm = 0.0f;
        float staticFrictionMaxSpeedMps = 0.005f;
        float viscousFrictionNmPerRadps = 0.0f;
        float longitudinalTireStiffnessN = 0.0f;
        float lateralVelocityDampingNsPerM = 0.0f;
        float yawRateDampingNmsPerRad = 0.0f;
        float muFront = 1.65f;
        float muRear = 1.65f;
        float muFrontPeak = 0.0f;
        float muRearPeak = 0.0f;
        float frontLoadFraction = 0.5f;
        float frontLongitudinalForceSplit = 0.5f;

        float velocityEpsilonMps = 0.05f;
        float stopEnterSpeedMps = 0.02f;
        float stopExitSpeedMps = 0.05f;
        float stopEnterYawRateRadps = 0.20f;
        float stopExitYawRateRadps = 0.50f;
        float stopEnterWheelSpeedRadps = 2.0f;
        float stopExitWheelSpeedRadps = 5.0f;
        float stopEnterCommand = 0.03f;
        float stopExitCommand = 0.06f;
        float rollingSpeedRegularizationMps = 0.05f;
        float maxIntegrationStepS = 0.0005f;
        float forceEpsilonN = 1.0e-4f;
        float fanDownforceAtFullDutyN = 0.7f;
        float combinedAccelSustainedMps2 = 1.91f * GRAVITY_MPS2;
        float combinedAccelNominalMps2 = 17.5f;
        float combinedAccelPeakMps2 = 20.1f;

        float noHitRangeM = 0.30f;
        std::array<Eigen::Vector2f, 4> contactPositionsBodyM = {
            Eigen::Vector2f::Zero(),
            Eigen::Vector2f::Zero(),
            Eigen::Vector2f::Zero(),
            Eigen::Vector2f::Zero()
        };

        SensorMount frontLeftSensor{};
        SensorMount frontRightSensor{};
        SensorMount sideLeftSensor{};
        SensorMount sideRightSensor{};
        SensorMount backLeftImuMount{};

        static EXPORT PlantParams Default() noexcept;

        Eigen::Vector2f ContactPosition(uint8_t index) const noexcept
        {
            return contactPositionsBodyM[(index < contactPositionsBodyM.size()) ? index : (contactPositionsBodyM.size() - 1U)];
        }

        float TotalNormalLoadN(float fanDutyCycle) const noexcept
        {
            return
                (massKg * GRAVITY_MPS2) +
                fanDutyCycle * fanDownforceAtFullDutyN;
        }

        float FrontWheelLoadN(float fanDutyCycle) const noexcept
        {
            return 0.5f * frontLoadFraction * TotalNormalLoadN(fanDutyCycle);
        }

        float RearWheelLoadN(float fanDutyCycle) const noexcept
        {
            return 0.5f * (1.0f - frontLoadFraction) * TotalNormalLoadN(fanDutyCycle);
        }
    };

    // Sanitized, precomputed coefficients for hot-path plant evaluation on embedded targets.
    struct PlantPreparedParams
    {
        PlantParams raw{};

        float forceEpsilonN = 1.0e-4f;

        float wheelRadiusM = 0.0f;
        float invWheelRadiusM = 0.0f;
        float trackWidthM = 0.0f;
        float halfTrackWidthM = 0.0f;
        float invTrackWidthM = 0.0f;
        float longitudinalOffsetM = 0.0f;
        float yawLeverArmM = 0.0f;

        float longitudinalMassKg = 1.0f;
        float invLongitudinalMassKg = 1.0f;
        float lateralMassKg = 1.0f;
        float invLateralMassKg = 1.0f;
        float yawInertiaKgM2 = 1.0f;
        float invYawInertiaKgM2 = 1.0f;
        float wheelInertiaKgM2 = 1.0f;
        float invWheelInertiaKgM2 = 1.0f;

        float rollingRegularizationMps = 1.0e-3f;
        float staticFrictionTorqueNm = 0.0f;
        float staticFrictionSpeedThresholdRadps = 0.0f;

        float longitudinalTireStiffnessN = 0.0f;
        float invLongitudinalTireStiffnessN = 0.0f;

        float lateralDampingNPerM = 0.0f;
        float yawDampingNmPerRadps = 0.0f;
        float lateralDampingOverMass = 0.0f;
        float yawDampingOverInertia = 0.0f;

        float baseNormalLoadN = 0.0f;
        float fanDownforceAtFullDutyN = 0.0f;
        float frontLoadFraction = 0.5f;
        float rearLoadFraction = 0.5f;
        float lambdaFront = 0.5f;
        float lambdaRear = 0.5f;

        float muFrontBase = 0.0f;
        float muRearBase = 0.0f;
        bool useEnvelopeMuFront = false;
        bool useEnvelopeMuRear = false;
        float combinedAccelPeakTimesMass = 0.0f;
        float lateralForceSustainedLimitN = 0.0f;

        float combinedAccelNominalMps2 = 0.0f;
        float combinedAccelNominalSq = 0.0f;

        float supplyVoltageV = 0.0f;
        float rollingFrictionTorqueNm = 0.0f;
        float viscousFrictionNmPerRadps = 0.0f;

        float stopEnterSpeedMps = 0.0f;
        float stopExitSpeedMps = 0.0f;
        float stopEnterYawRateRadps = 0.0f;
        float stopExitYawRateRadps = 0.0f;
        float stopEnterWheelSpeedRadps = 0.0f;
        float stopExitWheelSpeedRadps = 0.0f;
        float stopEnterCommand = 0.0f;
        float stopExitCommand = 0.0f;
    };

    // Shared vehicle plant owner for runtime dynamics, acceleration feedforward, and plant-side diagnostics.
    class EXPORT PlantModel
    {
    public:
        using StateVector = VehicleState::StateVector;
        using PreparedParams = PlantPreparedParams;

        static constexpr float kDefaultVelocityTargetResponseTimeS = 0.025f;
        static constexpr float kTractionLimitedReserveScale = 0.90f;

        PlantModel(const Vehicle& vehicle, const VehicleState& runtimeState) noexcept;
        float wallObservationNoHitRangeM() const noexcept;

        static PreparedParams Prepare(const PlantParams& params) noexcept;

        template <typename WriteFloatFn>
        bool WriteOpenFloorUkfMetadata(WriteFloatFn&& writeFloat) const
        {
            return
                writeFloat("re_m", _preparedParams.wheelRadiusM, 6) &&
                writeFloat("we_m", _preparedParams.trackWidthM, 6) &&
                writeFloat("imu_x", _preparedParams.raw.backLeftImuMount.positionBodyM().x(), 6) &&
                writeFloat("imu_y", _preparedParams.raw.backLeftImuMount.positionBodyM().y(), 6) &&
                writeFloat("jw_kgm2", _preparedParams.wheelInertiaKgM2, 9);
        }

        template <typename WriteConfigFn>
        bool WriteDriveModelDiagnosticConfig(WriteConfigFn&& writeConfig) const
        {
            return writeConfig(
                "motor_model:wheel_diam_m=%.6f;encoder_cpr=%lu;gear=%.6f;supply_v=%.3f",
                2.0f * _preparedParams.wheelRadiusM,
                static_cast<unsigned long>(_preparedParams.raw.encoderCountsPerMotorRev),
                _preparedParams.raw.gearRatio,
                _preparedParams.raw.supplyVoltageV);
        }

        using DebugTextSink = bool (*)(void* context, const char* type, const char* format, std::va_list args) noexcept;
        bool WriteUkfPlantDebugTextDump(void* context, DebugTextSink sink) const noexcept;

        PlantDerivatives forwardStep(
            const App::Internal::CommandVector& control) const noexcept;
        PlantDerivatives forwardStep(
            const StateVector& state,
            const App::Internal::CommandVector& control,
            const PlantParams& params) const noexcept;
        PlantDerivatives forwardStep(
            const StateVector& state,
            const App::Internal::CommandVector& control,
            const PreparedParams& params) const noexcept;

        WheelKinematics wheelKinematics(const StateVector& state, const PlantParams& params) const noexcept;
        WheelKinematics wheelKinematics(const StateVector& state, const PreparedParams& params) const noexcept;

        SlipTargets slipTargets(const StateVector& state, const PlantParams& params) const noexcept;
        SlipTargets slipTargets(const StateVector& state, const PreparedParams& params) const noexcept;
        SlipTargets slipTargets(
            const StateVector& state,
            const WheelKinematics& kinematics,
            const PlantParams& params) const noexcept;
        SlipTargets slipTargets(
            const StateVector& state,
            const WheelKinematics& kinematics,
            const PreparedParams& params) const noexcept;

        ContactForces tireForces(const StateVector& state, const PlantParams& params) const noexcept;
        ContactForces tireForces(const StateVector& state, const PreparedParams& params) const noexcept;
        ContactForces tireForces(
            const StateVector& state,
            const App::Internal::CommandVector& control,
            const PlantParams& params) const noexcept;
        ContactForces tireForces(
            const StateVector& state,
            const App::Internal::CommandVector& control,
            const PreparedParams& params) const noexcept;

        Eigen::Vector2f imuPlanarAcceleration(
            const StateVector& state,
            const App::Internal::CommandVector& control,
            const PlantParams& params) const noexcept;
        Eigen::Vector2f imuPlanarAcceleration(
            const StateVector& state,
            const App::Internal::CommandVector& control,
            const PreparedParams& params) const noexcept;

        StateVector integrate(
            const StateVector& state,
            const App::Internal::CommandVector& control,
            float dt,
            const PlantParams& params) const noexcept;
        StateVector integrate(
            const StateVector& state,
            const App::Internal::CommandVector& control,
            float dt,
            const PreparedParams& params) const noexcept;

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
        Eigen::Vector2f wheelLinearVelocityFromBodyState(const StateVector& state) const noexcept;
        float sustainedCombinedAccelerationUsage(float accelerationMps2) const noexcept;
        float nominalCombinedAccelerationUsage(float accelerationMps2) const noexcept;
        float peakCombinedAccelerationUsage(float accelerationMps2) const noexcept;
        float stopExitYawRateUsage(float yawRateRadps) const noexcept;

        void velocityTargetTechnicalLimits(
            float& maxLongitudinalAccelMps2,
            float& maxYawAccelRadps2) const noexcept;
        void velocityTargetTechnicalLimits(
            const StateVector& currentState,
            const PlantParams& params,
            float& maxLongitudinalAccelMps2,
            float& maxYawAccelRadps2) const noexcept;
        void velocityTargetTechnicalLimits(
            const StateVector& currentState,
            const PreparedParams& params,
            float& maxLongitudinalAccelMps2,
            float& maxYawAccelRadps2) const noexcept;
        void velocityTargetTechnicalLimits(
            float forwardVelocityMps,
            float yawRateRadps,
            const PlantParams& params,
            float& maxLongitudinalAccelMps2,
            float& maxYawAccelRadps2) const noexcept;
        void velocityTargetTechnicalLimits(
            float forwardVelocityMps,
            float yawRateRadps,
            const PreparedParams& params,
            float& maxLongitudinalAccelMps2,
            float& maxYawAccelRadps2) const noexcept;

        float driveFrictionTorque(
            float wheelBankSpeedRadps,
            float wheelTorqueRequestNm,
            const PlantParams& params) const noexcept;
        float driveFrictionTorque(
            float wheelBankSpeedRadps,
            float wheelTorqueRequestNm,
            const PreparedParams& params) const noexcept;

        PlantDerivatives forwardStepFromAppliedBankTorques(
            const StateVector& state,
            float leftAppliedBankTorqueNm,
            float rightAppliedBankTorqueNm) const noexcept;
        Eigen::Vector2f imuPlanarAcceleration(
            const StateVector& state,
            const App::Internal::CommandVector& control) const noexcept;

    private:
        friend struct PlantParams;
        friend class SrUkfCore;

        static PlantParams BuildParamsFromVehicle(const Vehicle& vehicle) noexcept;
        StateVector BuildBoundStateVector() const noexcept;
        void resolveAppliedBankTorques(
            const StateVector& currentState,
            const App::Internal::CommandVector& control,
            float& leftAppliedBankTorqueNm,
            float& rightAppliedBankTorqueNm) const noexcept;
        App::Internal::CommandVector ComputeFeedforwardFromState(
            const StateVector& currentState,
            float desiredAccelMps2,
            float desiredYawAccelRadps2) const noexcept;
        struct WheelOnlyMeasurementPrediction
        {
            float leftWheelSpeedRadps = 0.0f;
            float rightWheelSpeedRadps = 0.0f;
            float forwardSpeedMps = 0.0f;
            float yawRateRadps = 0.0f;
        };

        WheelOnlyMeasurementPrediction predictWheelOnlyMeasurement(
            const StateVector& state,
            const PreparedParams& params) const noexcept;

        PlantDerivatives forwardStepFromAppliedBankTorques(
            const StateVector& state,
            float leftAppliedBankTorqueNm,
            float rightAppliedBankTorqueNm,
            const PlantParams& params) const noexcept;
        PlantDerivatives forwardStepFromAppliedBankTorques(
            const StateVector& state,
            float leftAppliedBankTorqueNm,
            float rightAppliedBankTorqueNm,
            const PreparedParams& params) const noexcept;
        PlantDerivatives evaluateAppliedBankTorqueStep(
            const StateVector& state,
            float leftAppliedBankTorqueNm,
            float rightAppliedBankTorqueNm,
            float activityNorm,
            const PreparedParams& params) const noexcept;
        StateVector integrateAppliedBankTorques(
            const StateVector& state,
            float leftAppliedBankTorqueNm,
            float rightAppliedBankTorqueNm,
            float dtS) const noexcept;
        StateVector integrateAppliedBankTorques(
            const StateVector& state,
            float leftAppliedBankTorqueNm,
            float rightAppliedBankTorqueNm,
            const PreparedParams& params,
            float dtS) const noexcept;
        static StateVector advanceStateFromDerivatives(
            const StateVector& currentState,
            const PlantDerivatives& evaluatedStep,
            float dtS) noexcept;

        PreparedParams _preparedParams{};
        const Vehicle& _vehicle;
        const VehicleState& _runtimeState;
        const MotorEncoderDrive& _leftDrive;
        const MotorEncoderDrive& _rightDrive;
    };
}
