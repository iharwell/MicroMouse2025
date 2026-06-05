#pragma once

#include "..\MazeMap\VehicleState.h"

#include <array>
#include <chrono>
#include <cstdint>
#include <sstream>

namespace MazeMap
{
    namespace PlantModelDynamicsTestSupport
    {
        constexpr float kZeroLinearVelocityToleranceMps = 0.008f;
        constexpr float kSymmetricFrontLoadFraction = 0.5f;
        constexpr float kStopEnterSpeedMps = 0.02f;
        constexpr float kStopEnterYawRateRadps = 0.20f;
        constexpr float kStopEnterWheelSpeedRadps = 2.0f;
        constexpr float kPlantResidualDecayTauS = 0.075f;

        bool Near(const float expected, const float actual, const float tolerance) noexcept;

        void AppendMotionState(std::wstringstream& message, const VehicleState& state);

        float MaxPreProjectionUtilizationForBank(
            const float leftWheelSpeedRadps,
            const float rightWheelSpeedRadps,
            const uint8_t firstContactIndex,
            const uint8_t secondContactIndex);

        float MinimumSaturationForSymmetricWheelSpin();

        float MaximumSaturationForSymmetricWheelSpin();

        VehicleState IntegrateDifferentialWheelSpin();

        struct SymmetricWheelSpinStep final
        {
            VehicleState state;
            float initialLeftAbsRadps;
            float initialRightAbsRadps;
            float finalLeftAbsRadps;
            float finalRightAbsRadps;
        };

        SymmetricWheelSpinStep IntegrateSymmetricWheelSpin();

        struct FeedforwardTimingMeasurement final
        {
            std::chrono::high_resolution_clock::duration feedforwardDuration;
            std::chrono::high_resolution_clock::duration integrateDuration;
            float accumulator;
        };

        FeedforwardTimingMeasurement MeasureFeedforwardTiming();

        float PeakLateralAccelerationAcrossTicks(const float initialRightwardVelocityMps);

        struct InPlaceSlipYawDecelMeasurement final
        {
            std::array<float, 4> forwardRelativeVelocityMps;
            std::array<float, 4> rightRelativeVelocityMps;
            std::array<float, 4> rightForceN;
            float contactY;
            float yawRateRadps;
            float frontContactLimitN;
            float rearContactLimitN;
            float initialYawRateRadps;
            float actualYawRateRadps;
            float expectedYawAccelRadps2;
            float observedYawAccelRadps2;
        };

        InPlaceSlipYawDecelMeasurement MeasureInPlaceSlipYawDecel();

        float InPlaceSlipExpectedRightRelativeVelocity(
            const InPlaceSlipYawDecelMeasurement& measurement,
            const uint8_t contactIndex);

        float InPlaceSlipExpectedRightForce(
            const InPlaceSlipYawDecelMeasurement& measurement,
            const uint8_t contactIndex);

        float YawAccelerationAtForwardSpeed(const float forwardSpeedMps);

        struct LowSpeedYawAccelerationMeasurement final
        {
            float belowYawAccelRadps2;
            float centerYawAccelRadps2;
            float aboveYawAccelRadps2;
            float maxNeighborDeltaRadps2;
            float maxAllowedNeighborDeltaRadps2;
        };

        LowSpeedYawAccelerationMeasurement MeasureLowSpeedYawAcceleration();

        struct ContactForwardForceCoupleMeasurement final
        {
            float frontLeftForwardForceN;
            float frontRightForwardForceN;
            float rearLeftForwardForceN;
            float rearRightForwardForceN;
            float totalForwardForceN;
        };

        ContactForwardForceCoupleMeasurement MeasureContactForwardForceCouple();

        struct ContactContinuumFiniteSample final
        {
            float forwardSpeedMps;
            float rightSpeedMps;
            float yawRateRadps;
            std::array<float, 4> forwardRelativeVelocityMps;
            std::array<float, 4> rightRelativeVelocityMps;
            std::array<float, 4> forwardForceN;
            std::array<float, 4> rightForceN;
            std::array<float, 4> preProjectionUtilization;
            std::array<float, 4> saturation;
            float stateForwardVelocityMps;
            float stateRightwardVelocityMps;
            float stateYawRateRadps;
            float stateYawAccelRadps2;
        };

        ContactContinuumFiniteSample MeasureContactContinuumFiniteSample(
            const float forwardSpeedMps,
            const float rightSpeedMps,
            const float yawRateRadps);

        struct ContactContinuumYawAccelerationMeasurement final
        {
            std::array<float, 3> yawAccelerationsRadps2;
            float maxNeighborDeltaRadps2;
        };

        ContactContinuumYawAccelerationMeasurement
            MeasureContactContinuumYawAcceleration();

        struct InPlaceSlipSpinDownMeasurement final
        {
            float initialYawRateRadps;
            float maxYawRateAbsRadps;
            float initialMagnitudeToleranceRadps;
            float maxReboundRadps;
            float maxAllowedReboundRadps;
            int firstMonotonicFailureStep;
            float firstMonotonicFailurePreviousRadps;
            float firstMonotonicFailureActualRadps;
            int stopStep;
            float stopTimeS;
            float maxAllowedStopTimeS;
            float dtSeconds;
        };

        InPlaceSlipSpinDownMeasurement MeasureInPlaceSlipSpinDown();

        VehicleState IntegrateExactRestHold();

        VehicleState IntegrateSmallStationaryPerturbation();

        struct NearZeroLateralPerturbationMeasurement final
        {
            VehicleState initial;
            VehicleState final;
        };

        NearZeroLateralPerturbationMeasurement
            IntegrateNearZeroLateralPerturbation();

        struct ResidualDecayMeasurement final
        {
            float expectedForwardResidual;
            float actualForwardResidual;
            float expectedRightResidual;
            float actualRightResidual;
            float expectedYawResidual;
            float actualYawResidual;
        };

        ResidualDecayMeasurement MeasureResidualDecay();

        struct ImuAccelerationMeasurement final
        {
            float rightLeverContributionMps2;
            float forwardLeverContributionMps2;
            float expectedRightAccelerationMps2;
            float predictedRightAccelerationMps2;
            float expectedForwardAccelerationMps2;
            float predictedForwardAccelerationMps2;
        };

        ImuAccelerationMeasurement MeasureImuAcceleration(
            const float initialYawRateRadps,
            const float forwardVelocityMps,
            const float rightwardVelocityMps,
            const float leftWheelScale,
            const float rightWheelScale,
            const float leftCommand,
            const float rightCommand);

        struct PositiveDriveFromRestMeasurement final
        {
            VehicleState state;
            float averageAccelMps2;
        };

        PositiveDriveFromRestMeasurement IntegratePositiveDriveFromRest();

        struct StaticFrictionRestMeasurement final
        {
            VehicleState state;
            float expectedStaticFrictionTorqueNm;
            float positiveStaticFrictionTorqueNm;
            float negativeStaticFrictionTorqueNm;
            float expectedRollingFrictionTorqueNm;
            float positiveRollingFrictionTorqueNm;
            float negativeRollingFrictionTorqueNm;
        };

        StaticFrictionRestMeasurement MeasureStaticFrictionRest();

        struct LargeStepMeasurement final
        {
            VehicleState state;
            float wheelSpeedDeltaRadps;
        };

        LargeStepMeasurement IntegrateSingleLargeStep();

        struct MixedSlipMeasurement final
        {
            float leftLongitudinalSlipMps;
            float rightLongitudinalSlipMps;
            std::array<float, 4> forwardRelativeVelocityMps;
            std::array<float, 4> rightRelativeVelocityMps;
            std::array<float, 4> rightForceN;
            std::array<float, 4> forwardForceN;
            std::array<float, 4> saturation;
            std::array<float, 4> preProjectionUtilization;
            VehicleState state;
        };

        MixedSlipMeasurement MeasureMixedSlipCommand();

    }
}
