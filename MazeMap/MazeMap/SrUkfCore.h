#pragma once
// Declares the square-root UKF core that owns the process model and measurement updates.

#include "Maze.h"
#include "EstimatorPredictModel.h"
#include "PlantModel.h"
#include "WallGeometryModel.h"
#include "UKF.h"

#include <array>
#include <cstdint>
#include <type_traits>

namespace MazeMap
{
    // Outcome of one scalar or vector measurement-update attempt.
    struct MeasurementUpdateResult
    {
        bool attempted = false;
        bool accepted = false;
        float nis = 0.0f;
    };

    // Outcome of a single side-wall update, including the predicted geometry used to interpret it.
    struct WallUpdateResult
    {
        MeasurementUpdateResult filter{};
        GeometryPrediction prediction{};
    };

    // Outcome of a paired front-wall update, including the left/right predicted geometry.
    struct FrontPairUpdateResult
    {
        MeasurementUpdateResult filter{};
        GeometryPrediction leftPrediction{};
        GeometryPrediction rightPrediction{};
    };

    // Owns the square-root UKF state, process model, and direct measurement updates.
    class EXPORT SrUkfCore
    {
    public:
        enum class OperatingMode : std::uint8_t
        {
            StationaryCertified = 0U,
            LaunchOrReversalTransient = 1U,
            GripLinear = 2U,
            InconsistentOrSaturated = 3U
        };

        struct ModeProcessNoiseTuning
        {
            float sigmaUSqrtQ = 0.0f;
            float sigmaVSqrtQ = 0.0f;
            float sigmaRSqrtQ = 0.0f;
            float sigmaOmegaSqrtQ = 0.0f;
            float sigmaBgzSqrtQ = 0.0f;
            float stdRMin = 0.0f;
            float stdVMin = 0.0f;
        };

        struct RuntimeTuning
        {
            float generalEncoderLinearSpeedSigmaMps = 0.021187f;
            float generalEncoderYawRateSigmaRadps = 0.111268f;
            float stationaryEncoderVelocitySigmaMps = 0.002936f;
            float encoderPairNisThreshold = 13.81551f;
            float imuYawRateSigmaRadps = 0.062323f;
            float imuAccelSigmaMps2 = 0.600153f;
            float pivotScrubMaxCommandLinearMps = 0.03f;
            float pivotScrubMinCommandAngularRadps = 1.0f;
            float pivotScrubYawConsistencyThresholdRadps = 0.03f;
            float pivotScrubYawWindowMismatchThresholdRad = 0.003f;
            float pivotScrubZeroUSigmaMps = 0.06f;
            float stationaryGyroBiasTimeConstantS = 30.0f;
            float stationaryCertificationDwellS = 0.150f;
            float stationaryCandidateMaxLinearCommandMps = 0.03f;
            float stationaryCandidateMaxAngularCommandRadps = 0.15f;
            float stationaryCandidateMaxDriveCommand = 0.08f;
            float stationaryCandidateMaxEncoderOmegaRadps = 1.0f;
            float stationaryCandidateMaxCorrectedGyroRadps = 0.12f;
            float stationaryCandidateMaxAccelMps2 = 1.0f;
            float commandSignFlipWindowS = 0.025f;
            float stationaryExitLaunchWindowS = 0.060f;
            float launchHoldS = 0.030f;
            float launchLowSpeedThresholdMps = 0.25f;
            float launchDriveCommandDeltaThreshold = 0.75f;
            float inconsistentHoldS = 0.080f;
            float yawConsistencyLowPassTauS = 0.025f;
            float yawConsistencyLowPassThresholdRadps = 0.08f;
            float yawConsistencyExceedDwellS = 0.025f;
            float yawWindowDurationS = 0.080f;
            float yawWindowMismatchThresholdRad = 0.03f;
            float nhcResidualTripSigma = 3.0f;
            float nhcMinimumEnableForwardSpeedMps = 0.08f;
            float nhcDisableForwardSpeedMps = 0.05f;
            float nhcMaxDriveCommandDelta = 0.60f;
            float recoveryNhcReenableDelayS = 0.025f;
            float nhcBaseSigmaMps = 0.005f;
            float nhcSpeedSlopePerMps = 0.05f;
            float nhcMinimumSigmaMps = 0.005f;
            float nhcMaximumSigmaMps = 0.040f;
            float movingGyroBiasStdCapRadps = 0.020f;
            float recoveryYawRateStdFloorRadps = 0.030f;
            float yawValidityBiasDeltaMaxRadps = 0.02f;
            ModeProcessNoiseTuning stationaryCertifiedProcessNoise{};
            ModeProcessNoiseTuning launchOrReversalProcessNoise{};
            ModeProcessNoiseTuning gripLinearProcessNoise{};
            ModeProcessNoiseTuning inconsistentOrSaturatedProcessNoise{};
        };

        // April 20, 2026 post-fan-swap open-floor tuning from tooling/analyze_open_floor.py:
        // - later same-day cards `10:22:09` and `12:10:58` are the authoritative current-hardware baseline after
        //   the fan replacement and worse-balance parts swap,
        // - stationary gating and IMU sigma values use the later-card envelope because that is the hardware now on
        //   the robot,
        // - `SEC_20_LAUNCH / OPEN_LOOP_LAUNCH` repeatability from the later hardware sets the moving encoder noise floor,
        // - planar accel updates remain disabled, but the accel sigma still tracks the noisier post-swap stationary data.
        static constexpr float kGeneralEncoderLinearSpeedSigmaMps = 0.021187f;
        static constexpr float kGeneralEncoderYawRateSigmaRadps = 0.111268f;
        static constexpr float kStationaryEncoderVelocitySigmaMps = 0.002936f;
        static constexpr float kEncoderPairNisThreshold = 13.81551f;
        static constexpr float kImuYawRateSigmaRadps = 0.062323f;
        static constexpr float kImuAccelSigmaMps2 = 0.600153f;
        static constexpr float kPivotScrubMaxCommandLinearMps = 0.03f;
        static constexpr float kPivotScrubMinCommandAngularRadps = 1.0f;
        static constexpr float kPivotScrubYawConsistencyThresholdRadps = 0.03f;
        static constexpr float kPivotScrubYawWindowMismatchThresholdRad = 0.003f;
        static constexpr float kPivotScrubZeroUSigmaMps = 0.06f;
        static constexpr float kStationaryGyroBiasTimeConstantS = 30.0f;
        static constexpr std::uint16_t kInitialStationaryGyroBiasSeedStartSample = 50U;
        static constexpr std::uint16_t kInitialStationaryGyroBiasSeedEndSample = 150U;
        static constexpr std::uint16_t kInitialStationaryGyroBiasSeedSampleCount =
            static_cast<std::uint16_t>(
                (kInitialStationaryGyroBiasSeedEndSample - kInitialStationaryGyroBiasSeedStartSample) + 1U);

        using StateVector = VehicleState::StateVector;
        using StateMatrix = VehicleState::StateMatrix;
        using DebugTextSink = bool (*)(void* context, const char* type, const char* message) noexcept;

        SrUkfCore(
            const PlantParams& params = PlantParams::Default(),
            const PlantModel& plantModel = PlantModel()) noexcept;

        static RuntimeTuning BuildDefaultRuntimeTuning() noexcept;
        static RuntimeTuning GetRuntimeTuning() noexcept;
        static void SetRuntimeTuning(const RuntimeTuning& tuning) noexcept;
        static void ResetRuntimeTuning() noexcept;

        const StateVector& state() const noexcept
        {
            return _filter.state();
        }

        StateMatrix covariance() const noexcept
        {
            return _filter.covariance();
        }

        const PlantParams& params() const noexcept
        {
            return _params;
        }

        const PlantModel::PreparedParams& preparedParams() const noexcept
        {
            return _preparedParams;
        }

        const ModelCycleContext& modelCycleContext() const noexcept
        {
            return _frozenCycleContext;
        }

        OperatingMode operatingMode() const noexcept { return _operatingMode; }
        std::uint8_t operatingModeId() const noexcept { return static_cast<std::uint8_t>(_operatingMode); }
        float gyroBiasAnchorRadps() const noexcept { return _gyroBiasAnchorRadps; }
        float yawConsistencyLowPassRadps() const noexcept { return _yawConsistencyLowPassRadps; }
        float yawWindowMismatchRad() const noexcept { return _yawWindowMismatchRad; }
        float nhcSigmaMps() const noexcept { return _nhcSigmaMps; }
        float nhcResidualMps() const noexcept { return _nhcResidualMps; }
        float nhcResidualSigma() const noexcept { return _nhcResidualSigma; }
        bool nonholonomicConstraintEnabled() const noexcept { return _nonholonomicConstraintEnabled; }
        bool yawValidForFeedforward() const noexcept { return _yawValidForFeedforward; }
        bool biasUpdateEnabled() const noexcept { return _biasUpdateEnabled; }
        bool stationaryCertified() const noexcept { return _stationaryCertified; }
        float stationaryCandidateDwellS() const noexcept { return _stationaryCandidateDwellS; }
        float launchHoldRemainingS() const noexcept { return _launchHoldRemainingS; }
        float inconsistentHoldRemainingS() const noexcept { return _inconsistentHoldRemainingS; }
        float nhcReenableDelayRemainingS() const noexcept { return _nhcReenableDelayRemainingS; }
        bool pivotScrubMode() const noexcept { return _pivotScrubMode; }
        bool pivotScrubEncoderBodyUpdateSkipped() const noexcept { return _pivotScrubEncoderBodyUpdateSkipped; }
        bool pivotScrubZeroUSoftApplied() const noexcept { return _pivotScrubZeroUSoftApplied; }
        float pivotScrubEncoderWheelDeltaPsiRad() const noexcept { return _pivotScrubEncoderWheelDeltaPsiRad; }
        float pivotScrubEncoderWheelDeltaRRadps() const noexcept { return _pivotScrubEncoderWheelDeltaRRadps; }
        float pivotScrubEncoderWheelDeltaOmegaLRadps() const noexcept { return _pivotScrubEncoderWheelDeltaOmegaLRadps; }
        float pivotScrubEncoderWheelDeltaOmegaRRadps() const noexcept { return _pivotScrubEncoderWheelDeltaOmegaRRadps; }
        float pivotScrubEncoderWheelMaskedDeltaNorm() const noexcept { return _pivotScrubEncoderWheelMaskedDeltaNorm; }
        float pivotScrubZeroUInnovationMps() const noexcept { return _pivotScrubZeroUInnovationMps; }
        float pivotScrubZeroUDeltaMps() const noexcept { return _pivotScrubZeroUDeltaMps; }
        float pivotScrubGyroDeltaPsiRad() const noexcept { return _pivotScrubGyroDeltaPsiRad; }
        float pivotScrubGyroDeltaRRadps() const noexcept { return _pivotScrubGyroDeltaRRadps; }
        float pivotScrubGyroDeltaBgzRadps() const noexcept { return _pivotScrubGyroDeltaBgzRadps; }
        float pivotScrubGyroDeltaOmegaLRadps() const noexcept { return _pivotScrubGyroDeltaOmegaLRadps; }
        float pivotScrubGyroDeltaOmegaRRadps() const noexcept { return _pivotScrubGyroDeltaOmegaRRadps; }
        float pivotScrubGyroMaskedDeltaNorm() const noexcept { return _pivotScrubGyroMaskedDeltaNorm; }
        float closureResidualLeftMps() const noexcept { return _lastClosureResidualLeftMps; }
        float closureResidualRightMps() const noexcept { return _lastClosureResidualRightMps; }
        float gyroInnovationRadps() const noexcept { return _lastGyroInnovationRadps; }
        float gyroInnovationNis() const noexcept { return _lastGyroNis; }
        float forwardAccelInnovationMps2() const noexcept { return _lastForwardAccelInnovationMps2; }
        float forwardAccelInnovationNis() const noexcept { return _lastForwardAccelNis; }
        float lateralAccelInnovationMps2() const noexcept { return _lastLateralAccelInnovationMps2; }
        float lateralAccelInnovationNis() const noexcept { return _lastLateralAccelNis; }
        float closureLeftInnovationMps() const noexcept { return _lastClosureLeftInnovationMps; }
        float closureLeftNis() const noexcept { return _lastClosureLeftNis; }
        float closureRightInnovationMps() const noexcept { return _lastClosureRightInnovationMps; }
        float closureRightNis() const noexcept { return _lastClosureRightNis; }
        float lateralPseudoInnovationMps() const noexcept { return _lastLateralPseudoInnovationMps; }
        float lateralPseudoNis() const noexcept { return _lastLateralPseudoNis; }
        float softOdometryInnovation() const noexcept { return 0.0f; }
        float softOdometryInnovationNis() const noexcept { return 0.0f; }
        bool directWheelUpdateBodyStateInvariant() const noexcept { return _directWheelUpdateBodyStateInvariant; }
        bool releaseInflationApplied() const noexcept { return _releaseInflationApplied; }

        void setRuntimeContext(
            float commandedLinearMps,
            float commandedAngularRadps,
            std::uint16_t saturationFlags,
            float leftLaunchAssistFloor,
            float rightLaunchAssistFloor,
            bool accelBiasValid,
            float accelBodyXMps2,
            float accelBodyYMps2) noexcept;
        float resolveYawRateForFeedforward(float yawRateRawRadps) const noexcept;

        static float ComputeNonholonomicSigmaMps(float absForwardSpeedMps) noexcept;
        static bool IsStationaryCandidate(
            const ControlInput& control,
            float commandedLinearMps,
            float commandedAngularRadps,
            const EncoderObs& observation,
            float gyroRawRadps,
            float gyroBiasAnchorRadps,
            float accelBodyXMps2,
            float accelBodyYMps2,
            std::uint16_t saturationFlags) noexcept;
        static bool HasLaunchOrReversalTrigger(
            float forwardSpeedMps,
            float leftDriveCommand,
            float rightDriveCommand,
            float leftLaunchAssistFloor,
            float rightLaunchAssistFloor,
            bool recentCommandSignFlip,
            bool recentStationaryExit) noexcept;
        static bool HasInconsistentOrSaturatedTrigger(
            std::uint16_t saturationFlags,
            float yawConsistencyLowPassRadps,
            float yawWindowMismatchRad,
            bool nhcEnabled,
            float nhcResidualSigma) noexcept;
        static OperatingMode ClassifyOperatingMode(
            bool stationaryCertified,
            bool launchOrReversalActive,
            bool inconsistentOrSaturatedActive) noexcept;
        static bool IsYawValidForFeedforward(
            OperatingMode mode,
            float bgzRadps,
            float gyroBiasAnchorRadps,
            float yawConsistencyLowPassRadps,
            bool nhcEnabled,
            float lateralVelocityMps,
            float nhcSigmaMps) noexcept;
        static float ComputeStationaryGyroBiasWalkProcessVarianceRadps2(
            float dtSeconds,
            float measurementVarianceRadps2) noexcept;
        static float ComputeStationaryGyroBiasWalkPosteriorVarianceRadps2(
            float dtSeconds,
            float measurementVarianceRadps2) noexcept;

        bool WriteDebugTextDump(void* context, DebugTextSink sink) const noexcept;

        template <typename Sink>
        bool WriteDebugTextDump(Sink&& sink) const noexcept
        {
            using SinkType = std::remove_reference_t<Sink>;
            return WriteDebugTextDump(
                const_cast<void*>(static_cast<const void*>(&sink)),
                [](void* context, const char* type, const char* message) noexcept -> bool
                {
                    return (*static_cast<SinkType*>(context))(type, message);
                });
        }

        bool reset(const StateVector& state, const StateMatrix& covariance) noexcept;
        bool setState(const StateVector& state, const StateMatrix& covariance) noexcept;
        static StateMatrix BuildDefaultInitialCovariance() noexcept;

        bool predict(float dt, const ControlInput& control) noexcept;

        template <typename LoopHook>
        bool predict(float dt, const ControlInput& control, LoopHook&& loopHook) noexcept
        {
            using HookType = std::remove_reference_t<LoopHook>;
            return predictImpl(
                dt,
                control,
                const_cast<void*>(static_cast<const void*>(&loopHook)),
                [](void* context) noexcept
                {
                    (*static_cast<HookType*>(context))();
                });
        }

        MeasurementUpdateResult updateEncoderPair(
            const EncoderObs& observation,
            float dt,
            bool updateYaw = true) noexcept;

        template <typename LoopHook>
        MeasurementUpdateResult updateEncoderPair(
            const EncoderObs& observation,
            float dt,
            bool updateYaw,
            LoopHook&& loopHook) noexcept
        {
            using HookType = std::remove_reference_t<LoopHook>;
            return updateEncoderPairImpl(
                observation,
                dt,
                updateYaw,
                const_cast<void*>(static_cast<const void*>(&loopHook)),
                [](void* context) noexcept
                {
                    (*static_cast<HookType*>(context))();
                });
        }

        MeasurementUpdateResult updateYawRate(float yawRateRadps) noexcept;

        template <typename LoopHook>
        MeasurementUpdateResult updateYawRate(float yawRateRadps, LoopHook&& loopHook) noexcept
        {
            using HookType = std::remove_reference_t<LoopHook>;
            return updateYawRateImpl(
                yawRateRadps,
                const_cast<void*>(static_cast<const void*>(&loopHook)),
                [](void* context) noexcept
                {
                    (*static_cast<HookType*>(context))();
                });
        }

        MeasurementUpdateResult updatePlanarAccel(const ImuAccelObs& observation) noexcept;

        template <typename LoopHook>
        MeasurementUpdateResult updatePlanarAccel(
            const ImuAccelObs& observation,
            LoopHook&& loopHook) noexcept
        {
            using HookType = std::remove_reference_t<LoopHook>;
            return updatePlanarAccelImpl(
                observation,
                const_cast<void*>(static_cast<const void*>(&loopHook)),
                [](void* context) noexcept
                {
                    (*static_cast<HookType*>(context))();
                });
        }

        MeasurementUpdateResult updateImuMerged(const ImuMergedObs& observation) noexcept;
        FrontPairUpdateResult updateFrontPair(
            const WallObs& left,
            const WallObs& right,
            const Maze& maze) noexcept;
        WallUpdateResult updateSideSensor(
            Side which,
            const WallObs& observation,
            const Maze& maze) noexcept;

        static Eigen::Matrix<float, 2, 2> ComputeGeneralEncoderPairCovarianceRadps(
            const PlantParams& params) noexcept;
        static Eigen::Matrix<float, 2, 2> ComputeGeneralEncoderPairSqrtNoise(
            const PlantParams& params) noexcept;
        static float ComputeStationaryEncoderOmegaSigmaRadps(const PlantParams& params) noexcept;
        static Eigen::Matrix<float, 2, 2> ComputeEncoderPairSqrtNoise(
            const EncoderObs& observation,
            const PlantParams& params) noexcept;

    private:
        using LoopHookInvoker = void (*)(void*) noexcept;

        static void InvokeLoopHook(void* context, LoopHookInvoker loopHook) noexcept;
        static bool HasExactZeroWheelObservation(const EncoderObs& observation) noexcept;
        static StateMatrix BuildProcessNoiseSquareRootForMode(OperatingMode mode) noexcept;
        static float ComputeDistancePerEncoderCountM(const PlantParams& params) noexcept;
        static float ComputeMeasuredLinearSpeedMps(const EncoderObs& observation, const PlantParams& params) noexcept;
        static float ComputeMeasuredLinearSpeedVarianceMps2(const EncoderObs& observation) noexcept;
        static float ComputeMeasuredYawRateRadps(const EncoderObs& observation, const PlantParams& params) noexcept;
        static float ComputeMeasuredYawRateVarianceRadps2(const EncoderObs& observation, const PlantParams& params) noexcept;
        static float ComputeMeasuredWheelVarianceRadps2(const EncoderObs& observation, const PlantParams& params) noexcept;
        static float ComputeEncoderPairNisThreshold(const EncoderObs& observation) noexcept;
        static bool IsPivotScrubCandidate(
            const EncoderObs& observation,
            float commandedLinearMps,
            float commandedAngularRadps,
            float yawConsistencyLowPassRadps,
            float yawWindowMismatchRad,
            const PlantParams& params) noexcept;
        static float wallNoiseFromConfidence(float confidence, float minimumNoise) noexcept;
        static void ZeroGyroBiasDynamicCrossCovariances(StateMatrix& covariance) noexcept;
        static float ComputeStationaryGyroBiasBlendFactor(float dtSeconds) noexcept;
        static void ProjectMaskedStateAndCovariance(
            const StateVector& priorState,
            const StateMatrix& priorCovariance,
            const StateVector& updatedState,
            const StateMatrix& updatedCovariance,
            const int* allowedIndices,
            std::size_t allowedCount,
            StateVector& projectedState,
            StateMatrix& projectedCovariance) noexcept;
        void updateInitialStationaryGyroBias(float yawRateRadps, bool stationaryZeroMotionCandidate) noexcept;

        bool predictImpl(float dt, const ControlInput& control, void* loopHookContext, LoopHookInvoker loopHook) noexcept;
        MeasurementUpdateResult updateEncoderPairImpl(
            const EncoderObs& observation,
            float dt,
            bool updateYaw,
            void* loopHookContext,
            LoopHookInvoker loopHook) noexcept;
        MeasurementUpdateResult updateYawRateImpl(
            float yawRateRadps,
            void* loopHookContext,
            LoopHookInvoker loopHook) noexcept;
        MeasurementUpdateResult updatePlanarAccelImpl(
            const ImuAccelObs& observation,
            void* loopHookContext,
            LoopHookInvoker loopHook) noexcept;

        bool controlCommandsAreEffectivelyZero() const noexcept;
        bool applyGripLateralVelocityConstraint(
            void* loopHookContext,
            LoopHookInvoker loopHook) noexcept;
        void updateNonholonomicDiagnostics(bool constraintEnabled) noexcept;
        void anchorPoseToEncoderDelta(
            StateVector& anchoredState,
            const EncoderObs& measured,
            bool updateYaw) const noexcept;
        void applyWheelRateConstraint(
            const StateVector& priorState,
            const StateMatrix& priorCovariance,
            const EncoderObs& measured,
            float wheelVarianceRadps2,
            bool updateYaw) noexcept;
        void applyWheelSpeedConstraint(const EncoderObs& measured, float wheelVarianceRadps2) noexcept;
        void applyPivotScrubEncoderWheelConstraint(
            const StateVector& priorState,
            const StateMatrix& priorCovariance,
            const EncoderObs& measured,
            float wheelVarianceRadps2) noexcept;
        void applyPivotScrubZeroUConstraint(
            const StateVector& priorState,
            const StateMatrix& priorCovariance,
            float sigmaMps) noexcept;
        void applyPivotScrubGyroConstraint(
            const StateVector& priorState,
            const StateMatrix& priorCovariance,
            float yawRateRadps) noexcept;
        void applyStationaryZeroMotionConstraint(float yawRateRadps) noexcept;
        void updateCommandSignFlipWindow(float dtSeconds) noexcept;
        void updateStationaryCertification(float yawRateRadps) noexcept;
        void pushYawWindowContribution(float dtSeconds, float ukfYawRateRadps, float gyroYawRateRadps) noexcept;
        void updateYawConsistencyMetrics(float yawRateRadps, float ukfYawRateRadps) noexcept;
        void updateOperatingMode(float dtSeconds) noexcept;
        void updateProcessNoiseForMode() noexcept;
        void resetPivotScrubTelemetry() noexcept;
        bool shouldEnableNonholonomicConstraint() const noexcept;
        float correctedYawRateRadps(float yawRateRawRadps) const noexcept;
        void synchronizeGyroBiasStateToAnchor(
            bool zeroDynamicCrossCovariances,
            float maxGyroBiasStdRadps,
            float minimumYawRateStdRadps) noexcept;
        void enforceVarianceFloors(OperatingMode mode) noexcept;
        void sanitizeLaunchRecoveryIfNeeded(OperatingMode previousMode, OperatingMode newMode) noexcept;
        void applyReleaseInflationIfNeeded(bool wasExactStationaryLock) noexcept;
        void buildFrozenCycleContext(float dtSeconds, const ControlInput& control) noexcept;
        bool applyClosurePseudoMeasurements(void* loopHookContext, LoopHookInvoker loopHook) noexcept;
        bool applyAdaptiveLateralPseudoMeasurement(void* loopHookContext, LoopHookInvoker loopHook) noexcept;
        Eigen::Matrix<float, 2, 1> frontPairPredictionForState(
            const StateVector& sigmaPoint,
            const Maze& maze) const noexcept;
        float wallPredictionForSensor(
            const StateVector& sigmaPoint,
            const SensorExtrinsics& sensor,
            const Maze& maze) const noexcept;

        PlantModel _plantModel;
        EstimatorPredictModel _predictModel;
        WallGeometryModel _geometryModel;
        PlantParams _params;
        PlantModel::PreparedParams _preparedParams;
        UKF<VehicleState::kDimension, 3> _filter;
        ModelCycleContext _frozenCycleContext;
        TransientContactMemoryState _transientContactMemory;
        RegripRecoveryState _regripRecovery;
        ControlInput _lastControl;
        EncoderObs _lastEncoderObs;
        float _lastEncoderDtSeconds;
        StateVector _prePredictState;
        StateMatrix _prePredictCovariance;
        bool _havePredictionReference;
        bool _acceptedEncoderUpdateSincePredict;
        StateMatrix _sqrtProcessNoiseDensity;
        Eigen::Matrix<float, 3, 3> _sqrtImuNoise;
        Eigen::Matrix<float, 2, 2> _sqrtFrontNoise;
        Eigen::Matrix<float, 1, 1> _sqrtSideNoise;
        OperatingMode _operatingMode;
        float _gyroBiasAnchorRadps;
        float _gyroBiasAnchorVarianceRadps2;
        bool _initialStationaryGyroBiasPhaseExited;
        bool _initialStationaryGyroBiasSeedApplied;
        std::uint16_t _initialStationaryGyroBiasSampleOrdinal;
        std::uint16_t _initialStationaryGyroBiasCollectedSeedSamples;
        double _initialStationaryGyroBiasSeedAccumRadps;
        float _commandedLinearMps;
        float _commandedAngularRadps;
        std::uint16_t _saturationFlags;
        float _leftLaunchAssistFloor;
        float _rightLaunchAssistFloor;
        float _accelBodyXMps2;
        float _accelBodyYMps2;
        float _stationaryCandidateDwellS;
        StateVector _stationaryCandidatePoseReferenceState;
        StateMatrix _stationaryCandidatePoseReferenceCovariance;
        bool _stationaryCandidatePoseReferenceValid;
        bool _stationaryCertified;
        float _timeSinceStationaryExitS;
        float _timeSinceCommandSignFlipS;
        float _previousAverageDriveCommandSign;
        float _launchHoldRemainingS;
        float _inconsistentHoldRemainingS;
        float _nhcReenableDelayRemainingS;
        float _yawConsistencyLowPassRadps;
        float _yawWindowMismatchRad;
        float _yawConsistencyExceedDwellS;
        float _nhcSigmaMps;
        float _nhcResidualMps;
        float _nhcResidualSigma;
        bool _nonholonomicConstraintEnabled;
        bool _yawValidForFeedforward;
        bool _biasUpdateEnabled;
        bool _pivotScrubMode;
        bool _pivotScrubEncoderBodyUpdateSkipped;
        bool _pivotScrubZeroUSoftApplied;
        float _pivotScrubEncoderWheelDeltaPsiRad;
        float _pivotScrubEncoderWheelDeltaRRadps;
        float _pivotScrubEncoderWheelDeltaOmegaLRadps;
        float _pivotScrubEncoderWheelDeltaOmegaRRadps;
        float _pivotScrubEncoderWheelMaskedDeltaNorm;
        float _pivotScrubZeroUInnovationMps;
        float _pivotScrubZeroUDeltaMps;
        float _pivotScrubGyroDeltaPsiRad;
        float _pivotScrubGyroDeltaRRadps;
        float _pivotScrubGyroDeltaBgzRadps;
        float _pivotScrubGyroDeltaOmegaLRadps;
        float _pivotScrubGyroDeltaOmegaRRadps;
        float _pivotScrubGyroMaskedDeltaNorm;
        bool _directWheelUpdateBodyStateInvariant;
        bool _releaseInflationApplied;
        float _lastClosureResidualLeftMps;
        float _lastClosureResidualRightMps;
        float _lastGyroMeasurementRadps;
        float _lastGyroInnovationRadps;
        float _lastGyroNis;
        float _lastForwardAccelInnovationMps2;
        float _lastForwardAccelNis;
        float _lastLateralAccelInnovationMps2;
        float _lastLateralAccelNis;
        float _lastClosureLeftInnovationMps;
        float _lastClosureLeftNis;
        float _lastClosureRightInnovationMps;
        float _lastClosureRightNis;
        float _lastLateralPseudoInnovationMps;
        float _lastLateralPseudoNis;
        std::array<float, 128> _yawWindowDtSeconds;
        std::array<float, 128> _yawWindowUkfIntegralRad;
        std::array<float, 128> _yawWindowGyroIntegralRad;
        std::size_t _yawWindowHead;
        std::size_t _yawWindowSize;
        float _yawWindowSpanS;
    };
}
