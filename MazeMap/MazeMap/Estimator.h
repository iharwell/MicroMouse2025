#pragma once
// Declares the authoritative micromouse estimator owner for state and maze-evidence updates.

#include "MazeMapRuntimeCore.h"
#include "EncoderObs.h"
#include "ImuAccelObs.h"
#include "CommandVector.h"
#include "Maze.h"
#include "MapEvidenceUpdater.h"
#include "PlantModel.h"
#include "SensorMount.h"
#include "WallObservationPipeline.h"
#include "Direction.h"
#include "SensorSnapshot.h"
#include "WallGeometryModel.h"
#include "UKF.h"

#include <array>
#include <cstddef>
#include <cstdarg>
#include <cstdint>
#include <type_traits>

namespace MazeMap::App::Internal
{
    class LoopController;
}

namespace MazeMap
{
    class Vehicle;

    // Authoritative estimator entry point used by runtime code, simulations, and tests.
    class EXPORT Estimator
    {
    public:
        explicit Estimator(
            const Vehicle& vehicle,
            const PlantModel& plantModel,
            VehicleState& runtimeState) noexcept;

        bool HasFault() const noexcept { return _faulted; }
        const char* FaultReason() const noexcept
        {
            return (_faultReason[0] != '\0') ? _faultReason : "estimator_failure";
        }
        void ClearFault() noexcept;

        bool ResetPose(float xMeters, float yMeters, float headingRad) noexcept;
        bool ResetForSessionTransition(float xMeters, float yMeters, float headingRad) noexcept;
        bool RestoreSessionStartPhysicalState(float xMeters, float yMeters, float headingRad) noexcept;
        bool SetGyroBiasZ(float gyroBiasRadps) noexcept;

        bool WriteDebugTextDump(
            void* context,
            bool (*sink)(void* context, const char* type, const char* format, std::va_list args) noexcept)
            const noexcept;

        template <typename Sink>
        bool WriteDebugTextDump(Sink&& sink) const noexcept
        {
            return WriteDebugTextDump(
                const_cast<void*>(static_cast<const void*>(&sink)),
                &InvokeTypedDebugSink<std::remove_reference_t<Sink>>);
        }

        // `control` is the active command for the state interval being calculated.
        bool predict(
            float dt,
            const App::Internal::CommandVector& control) noexcept;
        // `control` is the active command for the state interval being calculated.
        bool predict(
            float dt,
            const App::Internal::CommandVector& control,
            const EncoderObs& encoderInput,
            bool encoderInputValid) noexcept;

        bool updateEncoderPair(
            const EncoderObs& observation,
            float dt,
            bool updateHeading) noexcept;

        bool updateYawRate(float yawRateRadps) noexcept;

        bool updatePlanarAccel(const ImuAccelObs& observation) noexcept;

        bool updateFrontPair(
            const WallObs& left,
            const WallObs& right,
            const Maze& maze,
            bool freezeMapMutation = false,
            const MapEvidenceUpdater::Config& evidenceConfig = MapEvidenceUpdater::Config{}) noexcept;

        bool updateSideSensor(
            RelativeDirection which,
            const WallObs& observation,
            const Maze& maze,
            bool freezeMapMutation = false,
            const MapEvidenceUpdater::Config& evidenceConfig = MapEvidenceUpdater::Config{}) noexcept;

    private:
        friend class App::Internal::LoopController;
        friend class EstimatorBiasAndStationaryTest;
        friend class EstimatorModeAndDiagnosticsTest;
        friend class EstimatorMotionUpdateTest;
        friend class EstimatorReplayRegressionTest;
        friend Estimator RunEstimatorCycles(int numCycles, App::Internal::CommandVector control);

        const Eigen::Matrix<float, VehicleState::kDimension, 1>& workingState() const noexcept
        {
            return _workingFilter.state();
        }

        Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> workingCovariance() const noexcept
        {
            return _workingFilter.covariance();
        }

        bool reset(
            const Eigen::Matrix<float, VehicleState::kDimension, 1>& state,
            const Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension>& covariance) noexcept;
        static Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> BuildDefaultInitialCovariance() noexcept;
        static WallGeometryModel::GeometryStateFrame BuildWallGeometryFrame(
            float pX,
            float pY,
            const Eigen::Vector2f& headingUnit) noexcept;

        bool LastUpdateAttempted() const noexcept { return _lastUpdateAttempted; }
        bool LastUpdateAccepted() const noexcept { return _lastUpdateAccepted; }
        float LastUpdateNis() const noexcept { return _lastUpdateNis; }
        const GeometryPrediction& LastSideWallPrediction() const noexcept { return _lastSideWallPrediction; }
        const GeometryPrediction& LastFrontLeftWallPrediction() const noexcept { return _lastFrontLeftWallPrediction; }
        const GeometryPrediction& LastFrontRightWallPrediction() const noexcept { return _lastFrontRightWallPrediction; }

        inline static constexpr std::array<const char*, VehicleState::kDimension> kEstimatorStateFieldNames = { {
            "px_m",
            "py_m",
            "heading_rad",
            "vf_mps",
            "vr_mps",
            "yaw_rate_radps",
            "delta_af_mps2",
            "delta_ar_mps2",
            "delta_yaw_accel_radps2"
        } };

        inline static constexpr std::array<const char*, 3> kEstimatorImuNoiseFieldNames = { {
            "gyro_yaw_rate_radps",
            "accel_body_right_mps2",
            "accel_body_forward_mps2"
        } };

        inline static constexpr std::array<const char*, 2> kEstimatorFrontNoiseFieldNames = { {
            "front_left_range_m",
            "front_right_range_m"
        } };

        inline static constexpr std::array<const char*, 1> kEstimatorSideNoiseFieldNames = { {
            "side_range_m"
        } };

        static constexpr float kStationaryCertificationDwellS = 0.150f;
        static constexpr float kStationaryCandidateMaxDriveCommand = 0.08f;
        static constexpr float kStationaryCandidateMaxEncoderWheelSpeedRadps = 1.0f;
        static constexpr float kStationaryCandidateMaxCorrectedGyroRadps = 0.12f;
        static constexpr float kGeneralEncoderLinearSpeedSigmaMps = 0.021187f;
        static constexpr float kGeneralEncoderYawRateSigmaRadps = 0.111268f;
        static constexpr float kEncoderBodyNisThreshold = 13.81551f;
        static constexpr float kEncoderPseudoUtilizationInflation = 9.0f;
        static constexpr float kEncoderPseudoContactRelativeSpeedInflation = 16.0f;
        static constexpr float kEncoderPseudoYawInducedContactSpeedInflation = 25.0f;
        static constexpr float kEncoderPseudoLaunchInflation = 9.0f;
        static constexpr float kImuYawRateVarianceRadps2 = 1.2e-6f;
        static constexpr float kImuYawRateSigmaRadps = 0.0010954451f;
        // LSM6DSV16X datasheet sensitivity tolerance for the runtime DPS2000 gyro range.
        static constexpr float kImuGyroSensitivityToleranceFraction = 0.003f;
        static constexpr float kYawRateBiasProcessVarianceStationaryRadps2PerSample = 3.0e-16f;
        static constexpr float kYawRateBiasInitialVarianceUnseededRadps2 = 3.05e-4f;
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
        static constexpr std::uint32_t kMaxCredibleWallTimingDeltaUs = 20000U;
        static constexpr float kMinimumVelocityVariance = 1.0e-8f;
        static constexpr float kMinimumYawRateVariance = 1.0e-8f;
        static constexpr std::uint16_t kInitialStationaryYawRateBiasSeedStartSample = 50U;
        static constexpr std::uint16_t kInitialStationaryYawRateBiasSeedEndSample = 150U;

        template <typename Sink>
        static bool InvokeTypedDebugSink(
            void* context,
            const char* type,
            const char* format,
            std::va_list args) noexcept
        {
            return (*static_cast<Sink*>(context))(type, format, args);
        }

        float YawRateMeasurementVarianceRadps2(float yawRateMeasurementRadps) const noexcept;
        static bool EmitDebugTextLine(
            void* context,
            bool (*sink)(void* context, const char* type, const char* format, std::va_list args) noexcept,
            const char* type,
            const char* format,
            ...) noexcept;

        template <std::size_t N, typename MatrixType>
        static bool EmitNamedMatrixRowLine(
            void* context,
            bool (*sink)(void* context, const char* type, const char* format, std::va_list args) noexcept,
            const char* type,
            const std::array<const char*, N>& fieldNames,
            const MatrixType& matrix,
            const int row) noexcept
        {
            const char* const rowName = fieldNames[static_cast<std::size_t>(row)];
            for (std::size_t column = 0U; column < N; ++column)
            {
                if (!EmitDebugTextLine(
                        context,
                        sink,
                        type,
                        "row=%s;%s=%.9g",
                        rowName,
                        fieldNames[column],
                        static_cast<double>(matrix(row, static_cast<int>(column)))))
                {
                    return false;
                }
            }

            return true;
        }

        static Direction dominantDirectionForSensor(
            const SensorMount& sensor,
            const Eigen::Matrix<float, VehicleState::kDimension, 1>& state) noexcept;
        static CellCoordinates estimateSensorCell(
            const SensorMount& sensor,
            const Eigen::Matrix<float, VehicleState::kDimension, 1>& state) noexcept;
        static void NormalizeState(Eigen::Matrix<float, VehicleState::kDimension, 1>& state) noexcept;
        static Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> BuildProcessNoiseSquareRoot() noexcept;
        static float wallNoiseFromConfidence(float confidence, float minimumNoise) noexcept;
        static void ProjectMaskedStateAndSquareRootCovariance(
            const Eigen::Matrix<float, VehicleState::kDimension, 1>& priorState,
            const Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension>& priorSqrtCovariance,
            const Eigen::Matrix<float, VehicleState::kDimension, 1>& updatedState,
            const Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension>& updatedSqrtCovariance,
            const int* allowedIndices,
            std::size_t allowedCount,
            Eigen::Matrix<float, VehicleState::kDimension, 1>& projectedState,
            Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension>& projectedSqrtCovariance) noexcept;

        bool predictImpl(
            float dt,
            const App::Internal::CommandVector& control,
            const EncoderObs* encoderInput,
            bool encoderInputValid) noexcept;
        bool predictWithInterleavedSensorService(
            float dt,
            const App::Internal::CommandVector& control,
            const EncoderObs* encoderInput,
            bool encoderInputValid,
            void* loopHookContext,
            void (*loopHook)(void*) noexcept) noexcept;
        bool updateEncoderPairImpl(
            const EncoderObs& observation,
            float dt,
            bool updateYaw) noexcept;
        bool updateYawRateImpl(float yawRateRadps) noexcept;
        bool updatePlanarAccelImpl(const ImuAccelObs& observation) noexcept;

        bool controlCommandsAreEffectivelyZero() const noexcept;
        bool isStationaryCandidate(float yawRateRadps) const noexcept;
        void updateInitialStationaryYawRateBias(
            float yawRateRadps,
            bool stationaryCertified) noexcept;
        void updateStationaryCertification(float yawRateRadps) noexcept;
        float correctedYawRateRadps(float yawRateRawRadps) const noexcept;
        float runtimeFanDuty() const noexcept;
        float runtimeBatteryVoltage() const noexcept;
        void ResetRuntimeMetadata() noexcept;
        void TriggerFault(const char* reason) noexcept;

        static Eigen::Matrix<float, VehicleState::kDimension, 1> PredictSigmaPoint(
            void* context,
            const Eigen::Matrix<float, VehicleState::kDimension, 1>& sigmaPoint,
            const Eigen::Matrix<float, 3, 1>& control,
            float dtS) noexcept;
        static Eigen::Matrix<float, 2, 1> EncoderBodyMeasurementForState(
            void* context,
            const Eigen::Matrix<float, VehicleState::kDimension, 1>& sigmaPoint) noexcept;
        static Eigen::Matrix<float, 1, 1> YawRateMeasurementForState(
            void* context,
            const Eigen::Matrix<float, VehicleState::kDimension, 1>& sigmaPoint) noexcept;
        static Eigen::Matrix<float, 1, 1> ForwardAccelMeasurementForState(
            void* context,
            const Eigen::Matrix<float, VehicleState::kDimension, 1>& sigmaPoint) noexcept;
        static Eigen::Matrix<float, 1, 1> RightAccelMeasurementForState(
            void* context,
            const Eigen::Matrix<float, VehicleState::kDimension, 1>& sigmaPoint) noexcept;
        static Eigen::Matrix<float, 2, 1> FrontPairMeasurementForState(
            void* context,
            const Eigen::Matrix<float, VehicleState::kDimension, 1>& sigmaPoint) noexcept;
        static Eigen::Matrix<float, 1, 1> SideMeasurementForState(
            void* context,
            const Eigen::Matrix<float, VehicleState::kDimension, 1>& sigmaPoint) noexcept;
        Eigen::Matrix<float, 2, 1> frontPairPredictionForState(
            const Eigen::Matrix<float, VehicleState::kDimension, 1>& sigmaPoint,
            const Maze& maze) const noexcept;
        float wallPredictionForSensor(
            const Eigen::Matrix<float, VehicleState::kDimension, 1>& sigmaPoint,
            const SensorMount& sensor,
            const Maze& maze,
            float noHitRangeM) const noexcept;

        const PlantModel& _plantModel;
        const Vehicle& _vehicle;
        VehicleState& _runtimeState;
        MapEvidenceUpdater _mapEvidence;
        WallGeometryModel _geometryModel;
        UKF<VehicleState::kDimension, 3> _workingFilter;
        App::Internal::CommandVector _lastControl{};
        float _lastFanDutyCycle = 0.80f;
        float _lastBatteryVoltageV = 0.0f;
        EncoderObs _lastEncoderObs{};
        float _lastEncoderDtSeconds = 0.0f;
        Eigen::Matrix<float, VehicleState::kDimension, 1> _prePredictState = Eigen::Matrix<float, VehicleState::kDimension, 1>::Zero();
        Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> _prePredictCovariance = Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension>::Zero();
        bool _havePredictionReference = false;
        bool _acceptedEncoderUpdateSincePredict = false;
        Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> _sqrtProcessNoiseDensity = Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension>::Zero();
        Eigen::Matrix<float, 3, 3> _sqrtImuNoise = Eigen::Matrix<float, 3, 3>::Identity();
        Eigen::Matrix<float, 2, 2> _sqrtFrontNoise = Eigen::Matrix<float, 2, 2>::Identity();
        Eigen::Matrix<float, 1, 1> _sqrtSideNoise = Eigen::Matrix<float, 1, 1>::Identity();
        float _yawRateBiasAnchorRadps = 0.0f;
        float _yawRateBiasAnchorVarianceRadps2 = 3.05e-4f;
        bool _initialStationaryYawRateBiasPhaseExited = false;
        bool _initialStationaryYawRateBiasSeedApplied = false;
        bool _biasUpdateEnabled = false;
        std::uint16_t _initialStationaryYawRateBiasSampleOrdinal = 0U;
        std::uint16_t _initialStationaryYawRateBiasCollectedSeedSamples = 0U;
        double _initialStationaryYawRateBiasSeedAccumRadps = 0.0;
        float _stationaryCandidateDwellS = 0.0f;
        bool _stationaryCertified = false;
        bool _predictionUsesEncoderInput = false;
        EncoderObs _predictionEncoderInput{};
        float _measurementYawRateBiasRadps = 0.0f;
        const Maze* _measurementMaze = nullptr;
        SensorMount _measurementSensor{};
        float _measurementNoHitRangeM = 0.0f;
        float _lastYawRateMeasurementRadps = 0.0f;
        float _lastYawRateInnovationRadps = 0.0f;
        float _lastYawRateNis = 0.0f;
        float _lastForwardAccelInnovationMps2 = 0.0f;
        float _lastForwardAccelNis = 0.0f;
        float _lastRightAccelInnovationMps2 = 0.0f;
        float _lastRightAccelNis = 0.0f;
        bool _lastUpdateAttempted = false;
        bool _lastUpdateAccepted = false;
        float _lastUpdateNis = 0.0f;
        GeometryPrediction _lastSideWallPrediction{};
        GeometryPrediction _lastFrontLeftWallPrediction{};
        GeometryPrediction _lastFrontRightWallPrediction{};
        bool _faulted = false;
        char _faultReason[64] = {};
    };
}
