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
        // Validation operating-point wheel speeds, including the implied rolling-region slip needed for Fx.
        float leftWheelSpeedRadps = 0.0f;
        float rightWheelSpeedRadps = 0.0f;
        // Pure rolling kinematic wheel speeds before adding the implied longitudinal slip offset.
        float leftRollingWheelSpeedRadps = 0.0f;
        float rightRollingWheelSpeedRadps = 0.0f;
        // Required wheel-side drive torque, not just tire-contact torque.
        float leftWheelTorqueNm = 0.0f;
        float rightWheelTorqueNm = 0.0f;
        // Requested bank angular accelerations implied by the desired body accelerations and used
        // in the inverse wheel-torque balance. These are not guaranteed to match forwardStep()
        // omega-dot at the returned slip-bearing operating point if the motor command clips.
        float leftWheelAccelRadps2 = 0.0f;
        float rightWheelAccelRadps2 = 0.0f;
        float leftContactForceN = 0.0f;
        float rightContactForceN = 0.0f;
        float leftContactTorqueNm = 0.0f;
        float rightContactTorqueNm = 0.0f;
        float tractionScale = 1.0f;
        bool tractionLimited = false;
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
        float equivalentWheelInertiaKgM2 = 4.8e-5f;

        float supplyVoltageV;
        float driveResistanceOhms;
        float torqueConstantNmPerA;
        float speedConstantRadpsPerVolt;
        float noLoadCurrentA;
        float motorCurrentLimitA;
        float gearRatio;
        uint16_t encoderCountsPerMotorRev;

        float drivetrainEfficiency = 0.5f;
        float rollingFrictionTorqueNm = 0.0035f;
        float viscousFrictionNmPerRadps = 2.5e-4f;

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
        float rollingSpeedRegularizationMps = 0.15f;
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

    // Evaluates the UKF process model from controls and vehicle state.
    class EXPORT PlantModel
    {
    public:
        using StateVector = VehicleState::StateVector;

        PlantDerivatives forwardStep(
            const StateVector& state,
            const ControlInput& control,
            const PlantParams& params) const noexcept;

        WheelKinematics wheelKinematics(const StateVector& state, const PlantParams& params) const noexcept;
        SlipTargets slipTargets(const StateVector& state, const PlantParams& params) const noexcept;
        SlipTargets slipTargets(
            const StateVector& state,
            const WheelKinematics& kinematics,
            const PlantParams& params) const noexcept;
        ContactForces tireForces(const StateVector& state, const PlantParams& params) const noexcept;
        ContactForces tireForces(
            const StateVector& state,
            const ControlInput& control,
            const PlantParams& params) const noexcept;
        Eigen::Vector2f imuPlanarAcceleration(
            const StateVector& state,
            const ControlInput& control,
            const PlantParams& params) const noexcept;
        StateVector integrate(
            const StateVector& state,
            const ControlInput& control,
            float dt,
            const PlantParams& params) const noexcept;
        DriveCommandSolution solveDriveCommands(
            float forwardVelocityMps,
            float desiredLongitudinalAccelMps2,
            float yawRateRadps,
            float desiredYawAccelRadps2,
            const PlantParams& params,
            float fanDutyCycle = 0.80f,
            float batteryVoltageV = 0.0f) const noexcept;
        float driveTorqueFromCommand(
            float motorCommand,
            float wheelBankSpeedRadps,
            float batteryVoltageV,
            const PlantParams& params) const noexcept;
        float driveCommandFromTorque(
            float wheelTorqueNm,
            float wheelBankSpeedRadps,
            float batteryVoltageV,
            const PlantParams& params) const noexcept;
        float driveFrictionTorque(float wheelBankSpeedRadps, const PlantParams& params) const noexcept;

    private:
        ContactForces tireForces(
            const WheelKinematics& kinematics,
            const SlipTargets& targets,
            const ControlInput& control,
            const PlantParams& params) const noexcept;
    };
}
