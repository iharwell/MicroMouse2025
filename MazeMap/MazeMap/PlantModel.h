
#pragma once
// Declares the vehicle process model and plant-side data types used by the micromouse UKF stack.

#include "Defines.h"
#include "EigenCompat.h"
#include "Maze.h"
#include "VehicleState.h"

#include <algorithm>
#include <array>
#include <cstdint>

namespace MazeMap
{
    // Contact-point velocity resolved in the project body frame (+X right, +Y forward).
    struct ContactKinematics
    {
        float rightVelocityMps = 0.0f;
        float forwardVelocityMps = 0.0f;
    };

    // Per-wheel-bank and per-contact kinematics derived from the current vehicle state.
    struct WheelKinematics
    {
        float leftBankForwardVelocityMps = 0.0f;
        float rightBankForwardVelocityMps = 0.0f;
        std::array<ContactKinematics, 4> contacts{};
    };

    // Diagnostic longitudinal slip and lateral slip ratios for each tire contact.
    struct SlipTargets
    {
        float kappaLeft = 0.0f;
        float kappaRight = 0.0f;
        std::array<float, 4> lateralRatio{};
    };

    // Force request and normalized traction utilization for one tire contact patch.
    struct ContactForce
    {
        float rightForceN = 0.0f;
        float forwardForceN = 0.0f;
        float normalForceN = 0.0f;
        float saturation = 0.0f;
    };

    // Aggregate contact-force bundle for the four tire contact patches.
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
    };

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

    // Inverse-dynamics result for a requested body motion at the current operating point.
    struct DriveCommandSolution
    {
        ControlInput control{};
        float leftSlipRatio = 0.0f;
        float rightSlipRatio = 0.0f;
        // Slip-bearing wheel speeds implied by the solved contact-force balance at the current body rates.
        float leftWheelSpeedRadps = 0.0f;
        float rightWheelSpeedRadps = 0.0f;
        // Pure rolling kinematic wheel speeds before adding the implied longitudinal slip offset.
        float leftRollingWheelSpeedRadps = 0.0f;
        float rightRollingWheelSpeedRadps = 0.0f;
        // Required wheel-side drive torque, not just tire-contact torque.
        float leftWheelTorqueNm = 0.0f;
        float rightWheelTorqueNm = 0.0f;
        // Algebraic bank angular-acceleration demand implied by the predicted achieved body
        // accelerations and used in the inverse wheel-torque balance.
        float leftWheelAccelRadps2 = 0.0f;
        float rightWheelAccelRadps2 = 0.0f;
        float leftContactForceN = 0.0f;
        float rightContactForceN = 0.0f;
        float leftContactTorqueNm = 0.0f;
        float rightContactTorqueNm = 0.0f;
        float tractionScale = 1.0f;
        bool tractionLimited = false;
        // Algebraically predicted achieved body accelerations at the returned operating point.
        // Optional host/debug validation may overwrite these when inverse validation is enabled.
        float commandedLongitudinalAccelMps2 = 0.0f;
        float commandedYawAccelRadps2 = 0.0f;
        float longitudinalAccelErrorMps2 = 0.0f;
        float yawAccelErrorRadps2 = 0.0f;
        bool converged = false;
    };

    // Tunable physical parameters and sensor extrinsics for the UKF plant model.
    struct PlantParams
    {
        float massKg;
        float effectiveLongitudinalMassKg;
        float effectiveLateralMassKg = 0.0f;
        float yawInertiaKgM2;
        float trackWidthM;
        float contactPatchLongitudinalOffsetM;
        float wheelRadiusM;
        // April 10, 2026 D:\open_floor_main.csv 0.08 launch-pulse torque balance fit. This is the wheel-bank
        // equivalent inertia used by the plant, not the bare Faulhaber rotor inertia.
        float equivalentWheelInertiaKgM2 = 1.6e-5f;

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

        float longitudinalTireStiffnessN = 6.0f;
        float corneringStiffnessFrontNPerRad = 18.0f;
        float corneringStiffnessRearNPerRad = 16.0f;
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
        float combinedAccelSustainedMps2 = 16.5f;
        float combinedAccelNominalMps2 = 17.5f;
        float combinedAccelPeakMps2 = 20.1f;

        float noHitRangeM = 0.30f;
        std::array<Eigen::Vector2f, 4> contactPositionsBodyM = {
            Eigen::Vector2f::Zero(),
            Eigen::Vector2f::Zero(),
            Eigen::Vector2f::Zero(),
            Eigen::Vector2f::Zero()
        };

        SensorExtrinsics frontLeftSensor{};
        SensorExtrinsics frontRightSensor{};
        SensorExtrinsics sideLeftSensor{};
        SensorExtrinsics sideRightSensor{};
        ImuExtrinsics imu{};

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
        float frontCorneringStiffnessAxleNPerRad = 0.0f;
        float rearCorneringStiffnessAxleNPerRad = 0.0f;

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

        float combinedAccelNominalMps2 = 0.0f;
        float combinedAccelNominalSq = 0.0f;

        float supplyVoltageV = 0.0f;
        float driveResistanceOhms = 0.0f;
        float invDriveResistanceOhms = 0.0f;
        float torqueConstantNmPerA = 0.0f;
        float speedConstantRadpsPerVolt = 0.0f;
        float noLoadCurrentA = 0.0f;
        float motorCurrentLimitA = 0.0f;
        float gearRatio = 0.0f;
        float drivetrainEfficiency = 1.0f;
        float driveGain = 0.0f;
        float invDriveGain = 0.0f;
        float wheelSpeedToBackEmfVoltPerRadps = 0.0f;
        float wheelSpeedToCurrentAPerRadps = 0.0f;
        float wheelTorquePerAmpNm = 0.0f;
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

        Eigen::Vector2f imuPositionBodyM = Eigen::Vector2f::Zero();
    };

    // Evaluates the UKF process model from controls and vehicle state.
    class EXPORT PlantModel
    {
    public:
        using StateVector = VehicleState::StateVector;
        using PreparedParams = PlantPreparedParams;

        static constexpr float kDefaultVelocityTargetResponseTimeS = 0.015f;
        static constexpr float kClosedLoopTractionReserveScale = 0.90f;

        static PreparedParams Prepare(const PlantParams& params) noexcept;

        PlantDerivatives forwardStep(
            const StateVector& state,
            const ControlInput& control,
            const PlantParams& params) const noexcept;
        PlantDerivatives forwardStep(
            const StateVector& state,
            const ControlInput& control,
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
            const ControlInput& control,
            const PlantParams& params) const noexcept;
        ContactForces tireForces(
            const StateVector& state,
            const ControlInput& control,
            const PreparedParams& params) const noexcept;

        Eigen::Vector2f imuPlanarAcceleration(
            const StateVector& state,
            const ControlInput& control,
            const PlantParams& params) const noexcept;
        Eigen::Vector2f imuPlanarAcceleration(
            const StateVector& state,
            const ControlInput& control,
            const PreparedParams& params) const noexcept;

        StateVector integrate(
            const StateVector& state,
            const ControlInput& control,
            float dt,
            const PlantParams& params) const noexcept;
        StateVector integrate(
            const StateVector& state,
            const ControlInput& control,
            float dt,
            const PreparedParams& params) const noexcept;

        // Algebraic traction-clamped inverse solve for the requested body accelerations at the
        // current operating point. The normal path performs no iterative refinement.
        DriveCommandSolution solveDriveCommands(
            const StateVector& currentState,
            float desiredLongitudinalAccelMps2,
            float desiredYawAccelRadps2,
            const PlantParams& params,
            float fanDutyCycle = 0.80f,
            float batteryVoltageV = 0.0f) const noexcept;
        DriveCommandSolution solveDriveCommands(
            const StateVector& currentState,
            float desiredLongitudinalAccelMps2,
            float desiredYawAccelRadps2,
            const PreparedParams& params,
            float fanDutyCycle = 0.80f,
            float batteryVoltageV = 0.0f) const noexcept;
        DriveCommandSolution solveDriveCommands(
            float forwardVelocityMps,
            float desiredLongitudinalAccelMps2,
            float yawRateRadps,
            float desiredYawAccelRadps2,
            const PlantParams& params,
            float fanDutyCycle = 0.80f,
            float batteryVoltageV = 0.0f) const noexcept;
        DriveCommandSolution solveDriveCommands(
            float forwardVelocityMps,
            float desiredLongitudinalAccelMps2,
            float yawRateRadps,
            float desiredYawAccelRadps2,
            const PreparedParams& params,
            float fanDutyCycle = 0.80f,
            float batteryVoltageV = 0.0f) const noexcept;

        // Returns the traction-limited control command that drives the current body rates toward the target
        // body rates over the requested response horizon using the canonical default when none is supplied.
        DriveCommandSolution solveDriveCommandsForVelocityTarget(
            const StateVector& currentState,
            float targetForwardVelocityMps,
            float targetYawRateRadps,
            const PlantParams& params,
            float fanDutyCycle = 0.80f,
            float batteryVoltageV = 0.0f,
            float responseTimeS = kDefaultVelocityTargetResponseTimeS) const noexcept;
        DriveCommandSolution solveDriveCommandsForVelocityTarget(
            const StateVector& currentState,
            float targetForwardVelocityMps,
            float targetYawRateRadps,
            const PreparedParams& params,
            float fanDutyCycle = 0.80f,
            float batteryVoltageV = 0.0f,
            float responseTimeS = kDefaultVelocityTargetResponseTimeS) const noexcept;
        DriveCommandSolution solveDriveCommandsForVelocityTarget(
            float currentForwardVelocityMps,
            float targetForwardVelocityMps,
            float currentYawRateRadps,
            float targetYawRateRadps,
            const PlantParams& params,
            float fanDutyCycle = 0.80f,
            float batteryVoltageV = 0.0f,
            float responseTimeS = kDefaultVelocityTargetResponseTimeS) const noexcept;
        DriveCommandSolution solveDriveCommandsForVelocityTarget(
            float currentForwardVelocityMps,
            float targetForwardVelocityMps,
            float currentYawRateRadps,
            float targetYawRateRadps,
            const PreparedParams& params,
            float fanDutyCycle = 0.80f,
            float batteryVoltageV = 0.0f,
            float responseTimeS = kDefaultVelocityTargetResponseTimeS) const noexcept;

        void resolveVelocityTargetAccelerations(
            float currentForwardVelocityMps,
            float targetForwardVelocityMps,
            float currentYawRateRadps,
            float targetYawRateRadps,
            float longitudinalAccelLimitMps2,
            float yawAccelLimitRadps2,
            float responseTimeS,
            float& desiredLongitudinalAccelMps2,
            float& desiredYawAccelRadps2) const noexcept;

        void resolveWheelMotionTargets(
            float targetForwardVelocityMps,
            float targetYawRateRadps,
            float targetLongitudinalAccelMps2,
            float targetYawAccelRadps2,
            const PreparedParams& params,
            float& leftTargetVelocityMps,
            float& rightTargetVelocityMps,
            float& leftTargetAccelMps2,
            float& rightTargetAccelMps2,
            float& leftTargetOmegaRadps,
            float& rightTargetOmegaRadps) const noexcept;

        void resolveBodyVelocityFromWheelSpeeds(
            float leftWheelLinearVelocityMps,
            float rightWheelLinearVelocityMps,
            const PreparedParams& params,
            float& forwardVelocityMps,
            float& yawRateRadps) const noexcept;

        void velocityTargetTechnicalLimits(
            const StateVector& currentState,
            const PlantParams& params,
            float& maxLongitudinalAccelMps2,
            float& maxYawAccelRadps2,
            float fanDutyCycle = 0.80f) const noexcept;
        void velocityTargetTechnicalLimits(
            const StateVector& currentState,
            const PreparedParams& params,
            float& maxLongitudinalAccelMps2,
            float& maxYawAccelRadps2,
            float fanDutyCycle = 0.80f) const noexcept;
        void velocityTargetTechnicalLimits(
            float forwardVelocityMps,
            float yawRateRadps,
            const PlantParams& params,
            float& maxLongitudinalAccelMps2,
            float& maxYawAccelRadps2,
            float fanDutyCycle = 0.80f) const noexcept;
        void velocityTargetTechnicalLimits(
            float forwardVelocityMps,
            float yawRateRadps,
            const PreparedParams& params,
            float& maxLongitudinalAccelMps2,
            float& maxYawAccelRadps2,
            float fanDutyCycle = 0.80f) const noexcept;

        // Returns the issued closed-loop feedforward command. If the raw request reaches the traction limit,
        // this backs the validated body accelerations off by the requested reserve scale so the outer PI loop
        // retains command headroom.
        DriveCommandSolution solveClosedLoopDriveCommands(
            const StateVector& currentState,
            float desiredLongitudinalAccelMps2,
            float desiredYawAccelRadps2,
            const PlantParams& params,
            float fanDutyCycle = 0.80f,
            float batteryVoltageV = 0.0f,
            float tractionReserveScale = 0.90f) const noexcept;
        DriveCommandSolution solveClosedLoopDriveCommands(
            const StateVector& currentState,
            float desiredLongitudinalAccelMps2,
            float desiredYawAccelRadps2,
            const PreparedParams& params,
            float fanDutyCycle = 0.80f,
            float batteryVoltageV = 0.0f,
            float tractionReserveScale = 0.90f) const noexcept;
        DriveCommandSolution solveClosedLoopDriveCommands(
            float forwardVelocityMps,
            float desiredLongitudinalAccelMps2,
            float yawRateRadps,
            float desiredYawAccelRadps2,
            const PlantParams& params,
            float fanDutyCycle = 0.80f,
            float batteryVoltageV = 0.0f,
            float tractionReserveScale = 0.90f) const noexcept;
        DriveCommandSolution solveClosedLoopDriveCommands(
            float forwardVelocityMps,
            float desiredLongitudinalAccelMps2,
            float yawRateRadps,
            float desiredYawAccelRadps2,
            const PreparedParams& params,
            float fanDutyCycle = 0.80f,
            float batteryVoltageV = 0.0f,
            float tractionReserveScale = 0.90f) const noexcept;
        DriveCommandSolution solveClosedLoopDriveCommandsForVelocityTarget(
            const StateVector& currentState,
            float targetForwardVelocityMps,
            float targetYawRateRadps,
            const PlantParams& params,
            float fanDutyCycle = 0.80f,
            float batteryVoltageV = 0.0f,
            float responseTimeS = kDefaultVelocityTargetResponseTimeS,
            float tractionReserveScale = 0.90f) const noexcept;
        DriveCommandSolution solveClosedLoopDriveCommandsForVelocityTarget(
            const StateVector& currentState,
            float targetForwardVelocityMps,
            float targetYawRateRadps,
            const PreparedParams& params,
            float fanDutyCycle = 0.80f,
            float batteryVoltageV = 0.0f,
            float responseTimeS = kDefaultVelocityTargetResponseTimeS,
            float tractionReserveScale = 0.90f) const noexcept;
        DriveCommandSolution solveClosedLoopDriveCommandsForVelocityTarget(
            float currentForwardVelocityMps,
            float targetForwardVelocityMps,
            float currentYawRateRadps,
            float targetYawRateRadps,
            const PlantParams& params,
            float fanDutyCycle = 0.80f,
            float batteryVoltageV = 0.0f,
            float responseTimeS = kDefaultVelocityTargetResponseTimeS,
            float tractionReserveScale = 0.90f) const noexcept;
        DriveCommandSolution solveClosedLoopDriveCommandsForVelocityTarget(
            float currentForwardVelocityMps,
            float targetForwardVelocityMps,
            float currentYawRateRadps,
            float targetYawRateRadps,
            const PreparedParams& params,
            float fanDutyCycle = 0.80f,
            float batteryVoltageV = 0.0f,
            float responseTimeS = kDefaultVelocityTargetResponseTimeS,
            float tractionReserveScale = 0.90f) const noexcept;

        float driveTorqueFromCommand(
            float motorCommand,
            float wheelBankSpeedRadps,
            float batteryVoltageV,
            const PlantParams& params) const noexcept;
        float driveTorqueFromCommand(
            float motorCommand,
            float wheelBankSpeedRadps,
            float batteryVoltageV,
            const PreparedParams& params) const noexcept;

        float driveCommandFromTorque(
            float wheelTorqueNm,
            float wheelBankSpeedRadps,
            float batteryVoltageV,
            const PlantParams& params) const noexcept;
        float driveCommandFromTorque(
            float wheelTorqueNm,
            float wheelBankSpeedRadps,
            float batteryVoltageV,
            const PreparedParams& params) const noexcept;

        float driveFrictionTorque(
            float wheelBankSpeedRadps,
            float wheelTorqueRequestNm,
            const PlantParams& params) const noexcept;
        float driveFrictionTorque(
            float wheelBankSpeedRadps,
            float wheelTorqueRequestNm,
            const PreparedParams& params) const noexcept;
    };
}
