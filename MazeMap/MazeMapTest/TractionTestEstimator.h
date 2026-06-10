#pragma once

#include "..\MazeMap\CommandVector.h"
#include "..\MazeMap\EigenCompat.h"
#include "..\MazeMap\ImuAccelObs.h"
#include "..\MazeMap\PlantModel.h"
#include "..\MazeMap\SensorSnapshot.h"
#include "..\MazeMap\UKF.h"
#include "..\MazeMap\Vehicle.h"
#include "..\MazeMap\VehicleState.h"

#include <cstddef>
#include <cstdint>

namespace MazeMap
{
    class TractionTestEstimator final
    {
    public:
        using CommandVector = App::Internal::CommandVector;
        using StateVector = Eigen::Matrix<float, VehicleState::kDimension, 1>;
        using CovarianceMatrix = Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension>;

        struct PredictionInput final
        {
            const StateVector& prePredictState;
            const StateVector& sigmaPoint;
            const CommandVector& control;
            float dtSeconds;
            const SensorSnapshot::EncoderObs* encoderInput;
        };

        struct MeasurementInput final
        {
            const StateVector& sigmaPoint;
            const CommandVector& control;
            const SensorSnapshot::EncoderObs* encoderInput;
        };

        struct PlantActivity final
        {
            float forwardAccelMps2 = 0.0f;
            float rightAccelMps2 = 0.0f;
            float yawAccelRadps2 = 0.0f;
            float maxContactRelativeSpeedMps = 0.0f;
            float maxContactUtilization = 0.0f;
            float maxContactSaturation = 0.0f;
            float totalNormalLoadN = 0.0f;
        };

        struct Hooks final
        {
            void* context = nullptr;
            StateVector (*predictState)(
                void* context,
                const PlantModel& productionPlant,
                const PredictionInput& input) noexcept = nullptr;
            PlantActivity (*plantActivity)(
                void* context,
                const PlantModel& productionPlant,
                const MeasurementInput& input) noexcept = nullptr;
            Eigen::Vector2f (*backLeftImuPlanarAcceleration)(
                void* context,
                const PlantModel& productionPlant,
                const MeasurementInput& input) noexcept = nullptr;
            float (*yawRateMeasurement)(
                void* context,
                const StateVector& sigmaPoint) noexcept = nullptr;
            Eigen::Matrix2f (*encoderPairCovarianceRadps)(
                void* context,
                const PlantModel& productionPlant,
                float linearSpeedSigmaMps,
                float yawRateSigmaRadps) noexcept = nullptr;
        };

        TractionTestEstimator(
            const Vehicle& vehicle,
            const PlantModel& plantModel,
            VehicleState& runtimeState,
            const Hooks& hooks = Hooks{}) noexcept;

        static CovarianceMatrix BuildDefaultInitialCovariance() noexcept;
        static StateVector BuildRuntimeStateVector(const VehicleState& runtimeState) noexcept;

        bool Reset(const StateVector& state, const CovarianceMatrix& covariance) noexcept;
        bool ResetPose(float xMeters, float yMeters, float headingRad) noexcept;
        bool Predict(float dtSeconds, const CommandVector& control) noexcept;
        bool UpdateYawRate(float yawRateRadps) noexcept;
        bool UpdatePlanarAccel(const ImuAccelObs& observation) noexcept;

        const StateVector& WorkingState() const noexcept { return _workingFilter.state(); }
        CovarianceMatrix WorkingCovariance() const noexcept { return _workingFilter.covariance(); }
        bool LastUpdateAttempted() const noexcept { return _lastUpdateAttempted; }
        bool LastUpdateAccepted() const noexcept { return _lastUpdateAccepted; }
        float LastUpdateNis() const noexcept { return _lastUpdateNis; }
        float LastYawRateInnovationRadps() const noexcept { return _lastYawRateInnovationRadps; }
        float LastForwardAccelInnovationMps2() const noexcept { return _lastForwardAccelInnovationMps2; }
        float LastRightAccelInnovationMps2() const noexcept { return _lastRightAccelInnovationMps2; }

    private:
        enum StateIndex : int
        {
            kPx = 0,
            kPy = 1,
            kHeading = 2,
            kVf = 3,
            kVr = 4,
            kYawRate = 5,
            kDeltaAf = 6,
            kDeltaAr = 7,
            kDeltaYawAccel = 8
        };

        static constexpr float kStationaryCertificationDwellS = 0.150f;
        static constexpr float kStationaryCandidateMaxDriveCommand = 0.08f;
        static constexpr float kStationaryCandidateMaxEncoderWheelSpeedRadps = 1.0f;
        static constexpr float kStationaryCandidateMaxYawRateRadps = 0.12f;
        static constexpr float kGeneralEncoderLinearSpeedSigmaMps = 0.021187f;
        static constexpr float kGeneralEncoderYawRateSigmaRadps = 0.111268f;
        static constexpr float kImuYawRateVarianceRadps2 = 1.2e-6f;
        static constexpr float kImuYawRateSigmaRadps = 0.0010954451f;
        static constexpr float kImuGyroSensitivityToleranceFraction = 0.003f;
        static constexpr float kImuAccelSigmaMps2 = 0.569900f;
        static constexpr float kTimingUncertaintySeconds = 0.00035f;
        static constexpr float kResidualForwardBaseSigmaSsMps2 = 0.010f;
        static constexpr float kResidualRightBaseSigmaSsMps2 = 0.010f;
        static constexpr float kResidualYawAccelBaseSigmaSsRadps2 = 0.010f;
        static constexpr float kResidualForwardUtilSigmaSsMps2 = 0.018f;
        static constexpr float kResidualRightUtilSigmaSsMps2 = 0.024f;
        static constexpr float kResidualYawAccelUtilSigmaSsRadps2 = 0.032f;
        static constexpr float kResidualRightContactRelativeSpeedSigmaSsMps2 = 0.018f;
        static constexpr float kResidualYawAccelContactRelativeSpeedSigmaSsRadps2 = 0.026f;
        static constexpr float kResidualYawInducedContactSpeedSigmaSs = 0.014f;
        static constexpr float kResidualEncoderFaultSigmaSs = 0.020f;
        static constexpr float kContactSpeedScheduleReferenceMps = 0.05f;
        static constexpr float kAccelContactRelativeSpeedSigmaMps2 = 0.080f;
        static constexpr float kAccelForceLimitSigmaMps2 = 0.200f;
        static constexpr float kAccelGroundUseSigmaMps2 = 0.120f;
        static constexpr float kAccelSaturationSigmaMps2 = 0.160f;
        static constexpr std::uint8_t kImuStatusAccelDataReadyMask = 0x01U;
        static constexpr std::uint8_t kImuStatusTimestampOverflowMask = 0x80U;
        static constexpr std::uint32_t kMaxCredibleImuTimingDeltaUs = 10000U;
        static constexpr float kMinimumVelocityVariance = 1.0e-8f;
        static constexpr float kMinimumYawRateVariance = 1.0e-8f;

        static void NormalizeState(StateVector& state) noexcept;
        static CovarianceMatrix BuildProcessNoiseSquareRoot() noexcept;
        static float DriveCommandActivityIndex(const CommandVector& control) noexcept;
        static void ProjectMaskedStateAndSquareRootCovariance(
            const StateVector& priorState,
            const CovarianceMatrix& priorSqrtCovariance,
            const StateVector& updatedState,
            const CovarianceMatrix& updatedSqrtCovariance,
            const int* allowedIndices,
            std::size_t allowedCount,
            StateVector& projectedState,
            CovarianceMatrix& projectedSqrtCovariance) noexcept;

        static StateVector PredictSigmaPoint(
            void* context,
            const StateVector& sigmaPoint,
            const Eigen::Matrix<float, 3, 1>& control,
            float dtSeconds) noexcept;
        static Eigen::Matrix<float, 1, 1> YawRateMeasurementForState(
            void* context,
            const StateVector& sigmaPoint) noexcept;
        static Eigen::Matrix<float, 1, 1> ForwardAccelMeasurementForState(
            void* context,
            const StateVector& sigmaPoint) noexcept;
        static Eigen::Matrix<float, 1, 1> RightAccelMeasurementForState(
            void* context,
            const StateVector& sigmaPoint) noexcept;

        StateVector CallPredictState(const StateVector& sigmaPoint, float dtSeconds) const noexcept;
        PlantActivity CallPlantActivity(const StateVector& state) const noexcept;
        Eigen::Vector2f CallBackLeftImuPlanarAcceleration(const StateVector& state) const noexcept;
        Eigen::Matrix2f CallEncoderPairCovarianceRadps(
            float linearSpeedSigmaMps,
            float yawRateSigmaRadps) const noexcept;
        float CallYawRateMeasurement(const StateVector& sigmaPoint) const noexcept;

        bool controlCommandsAreEffectivelyZero() const noexcept;
        bool isStationaryCandidate(float yawRateRadps) const noexcept;
        void updateStationaryCertification(float yawRateRadps) noexcept;
        float YawRateMeasurementVarianceRadps2(float yawRateMeasurementRadps) const noexcept;
        void PublishFilterStateToRuntime() noexcept;

        const Vehicle& _vehicle;
        const PlantModel& _plantModel;
        VehicleState& _runtimeState;
        Hooks _hooks{};
        StateVector _state = StateVector::Zero();
        CovarianceMatrix _sqrtCovariance = CovarianceMatrix::Identity() * 1.0e-3f;
        UKF<VehicleState::kDimension, 3> _workingFilter;
        CommandVector _lastControl{};
        float _lastFanDutyCycle = 0.80f;
        float _lastBatteryVoltageV = 0.0f;
        StateVector _prePredictState = StateVector::Zero();
        CovarianceMatrix _prePredictCovariance = CovarianceMatrix::Zero();
        CovarianceMatrix _sqrtProcessNoiseDensity = CovarianceMatrix::Zero();
        Eigen::Matrix<float, 3, 3> _sqrtImuNoise = Eigen::Matrix<float, 3, 3>::Identity();
        bool _predictionUsesEncoderInput = false;
        SensorSnapshot::EncoderObs _predictionEncoderInput = SensorSnapshot{}.EncoderObservation();
        float _predictionEncoderDtSeconds = 0.0f;
        float _stationaryCandidateDwellS = 0.0f;
        bool _stationaryCertified = false;
        bool _lastUpdateAttempted = false;
        bool _lastUpdateAccepted = false;
        float _lastUpdateNis = 0.0f;
        float _lastYawRateMeasurementRadps = 0.0f;
        float _lastYawRateInnovationRadps = 0.0f;
        float _lastYawRateNis = 0.0f;
        float _lastForwardAccelInnovationMps2 = 0.0f;
        float _lastForwardAccelNis = 0.0f;
        float _lastRightAccelInnovationMps2 = 0.0f;
        float _lastRightAccelNis = 0.0f;
    };
}
