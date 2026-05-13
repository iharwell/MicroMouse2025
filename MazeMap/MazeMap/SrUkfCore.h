#pragma once
// Declares the square-root UKF core that owns the process model and measurement updates.

#include "Maze.h"
#include "EncoderObs.h"
#include "ImuAccelObs.h"
#include "CommandVector.h"
#include "PlantModel.h"
#include "SensorMount.h"
#include "WallObservationPipeline.h"
#include "Direction.h"
#include "WallGeometryModel.h"
#include "UKF.h"

#include <array>
#include <cstdarg>
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
    private:
        struct GripUtilizationSnapshot
        {
            float longitudinalClosureSeverity = 0.0f;
            float differentialClosureSeverity = 0.0f;
            float lateralAccelerationSeverity = 0.0f;
            float yawConsistencySeverity = 0.0f;
            float leftBankAnomalySeverity = 0.0f;
            float rightBankAnomalySeverity = 0.0f;
            float leftBankPreProjectionUtilization = 0.0f;
            float rightBankPreProjectionUtilization = 0.0f;
        };

        struct TransientContactMemoryState
        {
            float leftBankMemory = 0.0f;
            float rightBankMemory = 0.0f;
        };

        struct RegripRecoveryState
        {
            bool leftBankInRecovery = false;
            bool rightBankInRecovery = false;
            float leftBankRecoveryScore = 0.0f;
            float rightBankRecoveryScore = 0.0f;
            float leftBankRecoveryTimeRemainingS = 0.0f;
            float rightBankRecoveryTimeRemainingS = 0.0f;
        };

        struct RobustUpdateSchedule
        {
            bool exactStationaryLock = false;
            bool planarAccelForwardUpdateEnabled = false;
            bool planarAccelLateralUpdateEnabled = false;
            bool softOdometryEnabled = false;
            float closureCovarianceScaleLeft = 1.0f;
            float closureCovarianceScaleRight = 1.0f;
            float lateralPseudoMeasurementCovarianceScale = 1.0f;
            float forwardSpeedProcessNoiseScale = 1.0f;
            float lateralSpeedProcessNoiseScale = 1.0f;
            float yawRateProcessNoiseScale = 1.0f;
            float leftWheelSpeedProcessNoiseScale = 1.0f;
            float rightWheelSpeedProcessNoiseScale = 1.0f;
            bool leftBankHoldoffActive = false;
            bool rightBankHoldoffActive = false;
        };

    public:
        using StateVector = VehicleState::StateVector;
        using StateMatrix = VehicleState::StateMatrix;
        using DebugTextSink = bool (*)(void* context, const char* type, const char* format, std::va_list args) noexcept;

        explicit SrUkfCore(const PlantModel& plantModel) noexcept;

        const StateVector& state() const noexcept
        {
            return _filter.state();
        }

        StateMatrix covariance() const noexcept
        {
            return _filter.covariance();
        }

        bool WriteDebugTextDump(void* context, DebugTextSink sink) const noexcept;

        template <typename Sink>
        bool WriteDebugTextDump(Sink&& sink) const noexcept
        {
            using SinkType = std::remove_reference_t<Sink>;
            return WriteDebugTextDump(
                const_cast<void*>(static_cast<const void*>(&sink)),
                [](void* context, const char* type, const char* format, std::va_list args) noexcept -> bool
                {
                    return (*static_cast<SinkType*>(context))(type, format, args);
                });
        }

        bool reset(const StateVector& state, const StateMatrix& covariance) noexcept;
        bool setState(const StateVector& state, const StateMatrix& covariance) noexcept;
        static StateMatrix BuildDefaultInitialCovariance() noexcept;

        bool predict(
            float dt,
            const App::Internal::CommandVector& control,
            float fanDutyCycle = 0.80f,
            float batteryVoltageV = 0.0f) noexcept;

        template <typename LoopHook>
        bool predict(
            float dt,
            const App::Internal::CommandVector& control,
            float fanDutyCycle,
            float batteryVoltageV,
            LoopHook&& loopHook) noexcept
        {
            using HookType = std::remove_reference_t<LoopHook>;
            return predictImpl(
                dt,
                control,
                fanDutyCycle,
                batteryVoltageV,
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

        FrontPairUpdateResult updateFrontPair(
            const WallObs& left,
            const WallObs& right,
            const Maze& maze) noexcept;
        WallUpdateResult updateSideSensor(
            RelativeDirection which,
            const WallObs& observation,
            const Maze& maze) noexcept;

    private:
        enum class OperatingMode : std::uint8_t
        {
            StationaryCertified = 0U,
            LaunchOrReversalTransient = 1U,
            GripLinear = 2U,
            InconsistentOrSaturated = 3U
        };

        using LoopHookInvoker = void (*)(void*) noexcept;

        static void InvokeLoopHook(void* context, LoopHookInvoker loopHook) noexcept;
        static bool HasExactZeroWheelObservation(const EncoderObs& observation) noexcept;
        static StateMatrix BuildProcessNoiseSquareRootForMode(OperatingMode mode) noexcept;
        void setRuntimeContext(
            float commandedLinearMps,
            float commandedAngularRadps,
            std::uint16_t saturationFlags,
            float leftLaunchAssistFloor,
            float rightLaunchAssistFloor,
            bool accelBiasValid,
            float accelBodyXMps2,
            float accelBodyYMps2) noexcept;
        static float ComputeNonholonomicSigmaMps(float absForwardSpeedMps) noexcept;
        static bool IsStationaryCandidate(
            const App::Internal::CommandVector& control,
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
        static void ComputeVelocityTargetBodyAction(
            float currentForwardVelocityMps,
            float targetForwardVelocityMps,
            float currentYawRateRadps,
            float targetYawRateRadps,
            float responseTimeS,
            float& desiredLongitudinalAccelMps2,
            float& desiredYawAccelRadps2) noexcept;
        static float ComputeEncoderPairNisThreshold(const EncoderObs& observation) noexcept;
        static StateVector IntegrateStationaryHoldState(const StateVector& currentState, float dtS) noexcept;
        bool IsPivotScrubCandidate(
            const EncoderObs& observation,
            float commandedLinearMps,
            float commandedAngularRadps,
            float yawConsistencyLowPassRadps,
            float yawWindowMismatchRad) const noexcept;
        static float wallNoiseFromConfidence(float confidence, float minimumNoise) noexcept;
        static void ProjectMaskedStateAndSquareRootCovariance(
            const StateVector& priorState,
            const StateMatrix& priorSqrtCovariance,
            const StateVector& updatedState,
            const StateMatrix& updatedSqrtCovariance,
            const int* allowedIndices,
            std::size_t allowedCount,
            StateVector& projectedState,
            StateMatrix& projectedSqrtCovariance) noexcept;
        void updateInitialStationaryGyroBias(float yawRateRadps, bool stationaryZeroMotionCandidate) noexcept;
        GripUtilizationSnapshot buildGripUtilizationSnapshot(
            const StateVector& currentState,
            float leftAppliedBankTorqueNm,
            float rightAppliedBankTorqueNm,
            float leftClosureResidualMps,
            float rightClosureResidualMps,
            float fanDutyCycle) const noexcept;
        static TransientContactMemoryState AdvanceTransientContactMemory(
            const TransientContactMemoryState& previousState,
            const GripUtilizationSnapshot& utilization,
            float dtS) noexcept;
        static RegripRecoveryState AdvanceRegripRecovery(
            const RegripRecoveryState& prior,
            const GripUtilizationSnapshot& utilization,
            const TransientContactMemoryState& memory,
            float dtS) noexcept;
        static bool IsHoldoffActive(const RegripRecoveryState& state) noexcept;
        static bool IsHoldoffActiveLeft(const RegripRecoveryState& state) noexcept;
        static bool IsHoldoffActiveRight(const RegripRecoveryState& state) noexcept;
        RobustUpdateSchedule buildFrozenSchedule(
            const GripUtilizationSnapshot& utilization,
            const TransientContactMemoryState& memory,
            const RegripRecoveryState& regrip,
            bool exactStationaryLock,
            bool lowSpeedLaunchWindowActive,
            bool inconsistencyWindowActive) const noexcept;
        float closurePseudoMeasurementScale(RelativeDirection side) const noexcept;
        float lateralPseudoMeasurementScale() const noexcept;

        bool predictImpl(
            float dt,
            const App::Internal::CommandVector& control,
            float fanDutyCycle,
            float batteryVoltageV,
            void* loopHookContext,
            LoopHookInvoker loopHook) noexcept;
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
        void applyWheelSpeedConstraint(const EncoderObs& measured, float wheelVarianceRadps2) noexcept;
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
        void refreshFrozenPolicyState(
            float dtSeconds,
            const App::Internal::CommandVector& control,
            float fanDutyCycle,
            float batteryVoltageV) noexcept;
        bool applyClosurePseudoMeasurements(void* loopHookContext, LoopHookInvoker loopHook) noexcept;
        bool applyAdaptiveLateralPseudoMeasurement(void* loopHookContext, LoopHookInvoker loopHook) noexcept;
        Eigen::Matrix<float, 2, 1> frontPairPredictionForState(
            const StateVector& sigmaPoint,
            const Maze& maze) const noexcept;
        float wallPredictionForSensor(
            const StateVector& sigmaPoint,
            const SensorMount& sensor,
            const Maze& maze) const noexcept;

        const PlantModel& _plantModel;
        WallGeometryModel _geometryModel;
        UKF<VehicleState::kDimension, 3> _filter;
        float _frozenDtS = 0.0f;
        float _frozenLeftAppliedBankTorqueNm = 0.0f;
        float _frozenRightAppliedBankTorqueNm = 0.0f;
        GripUtilizationSnapshot _frozenGripUtilization{};
        RobustUpdateSchedule _frozenSchedule{};
        TransientContactMemoryState _transientContactMemory{};
        RegripRecoveryState _regripRecovery{};
        App::Internal::CommandVector _lastControl{};
        float _lastFanDutyCycle = 0.80f;
        float _lastBatteryVoltageV = 0.0f;
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

