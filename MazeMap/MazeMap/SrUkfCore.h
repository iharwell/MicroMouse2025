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
            float stdRMin = 0.0f;
            float stdVMin = 0.0f;
        };

        struct RuntimeTuning
        {
            float generalEncoderLinearSpeedSigmaMps = 0.021187f;
            float generalEncoderYawRateSigmaRadps = 0.111268f;
            float stationaryEncoderVelocitySigmaMps = 0.002936f;
            float encoderPairNisThreshold = 13.81551f;
            float imuYawRateSigmaRadps = 0.0010954451f;
            float imuAccelSigmaMps2 = 0.569900f;
            float pivotScrubMaxCommandLinearMps = 0.03f;
            float pivotScrubMinCommandAngularRadps = 1.0f;
            float pivotScrubYawConsistencyThresholdRadps = 0.03f;
            float pivotScrubYawWindowMismatchThresholdRad = 0.003f;
            float pivotScrubZeroUSigmaMps = 0.06f;
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
            float recoveryYawRateStdFloorRadps = 0.030f;
            float yawValidityBiasDeltaMaxRadps = 0.02f;
            ModeProcessNoiseTuning stationaryCertifiedProcessNoise{};
            ModeProcessNoiseTuning launchOrReversalProcessNoise{};
            ModeProcessNoiseTuning gripLinearProcessNoise{};
            ModeProcessNoiseTuning inconsistentOrSaturatedProcessNoise{};
        };

        // April 22, 2026 Allan-backed open-floor retune from the unique April 21, 2026 current-hardware cards:
        // - stationary IMU measurement sigmas use the observed current-hardware envelope across the April 21 cards,
        // - gyro-z measurement noise uses the April 22, 2026 author 6.2 override `R_g = 1.2e-6 (rad/s)^2`,
        // - accel sigma envelope: 0.569900 m/s^2,
        // - gyro Allan envelope: 0.002002105 rad/sqrt(s) angle random walk and 8.5965e-5 rad/s bias-instability
        //   lower bound,
        // - moving and stationary gyro-bias process terms remain unchanged because these 15 s holds do not support a
        //   stronger RRW-based retune,
        // - `SEC_20_LAUNCH / OPEN_LOOP_LAUNCH` repeatability still owns the moving encoder noise floor.
        static constexpr float kGeneralEncoderLinearSpeedSigmaMps = 0.021187f;
        static constexpr float kGeneralEncoderYawRateSigmaRadps = 0.111268f;
        static constexpr float kStationaryEncoderVelocitySigmaMps = 0.002936f;
        static constexpr float kEncoderPairNisThreshold = 13.81551f;
        static constexpr float kImuYawRateVarianceRadps2 = 1.2e-6f;
        static constexpr float kImuYawRateSigmaRadps = 0.0010954451f;
        static constexpr float kGyroBiasProcessVarianceMovingRadps2PerSample = 0.0f;
        static constexpr float kGyroBiasProcessVarianceStationaryRadps2PerSample = 3.0e-16f;
        static constexpr float kGyroBiasInitialVarianceUnseededRadps2 = 3.05e-4f;
        static constexpr float kImuAccelSigmaMps2 = 0.569900f;
        static constexpr float kPivotScrubMaxCommandLinearMps = 0.03f;
        static constexpr float kPivotScrubMinCommandAngularRadps = 1.0f;
        static constexpr float kPivotScrubYawConsistencyThresholdRadps = 0.03f;
        static constexpr float kPivotScrubYawWindowMismatchThresholdRad = 0.003f;
        static constexpr float kPivotScrubZeroUSigmaMps = 0.06f;
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
        bool exactStationaryLock() const noexcept { return _frozenSchedule.exactStationaryLock; }
        bool leftBankHoldoffActive() const noexcept { return _frozenSchedule.leftBankHoldoffActive; }
        bool rightBankHoldoffActive() const noexcept { return _frozenSchedule.rightBankHoldoffActive; }
        bool leftBankInRecovery() const noexcept { return _regripRecovery.leftBankInRecovery; }
        bool rightBankInRecovery() const noexcept { return _regripRecovery.rightBankInRecovery; }
        float longitudinalClosureSeverity() const noexcept { return _frozenGripUtilization.longitudinalClosureSeverity; }
        float differentialClosureSeverity() const noexcept { return _frozenGripUtilization.differentialClosureSeverity; }
        float lateralAccelerationSeverity() const noexcept { return _frozenGripUtilization.lateralAccelerationSeverity; }
        float yawConsistencySeverity() const noexcept { return _frozenGripUtilization.yawConsistencySeverity; }
        float leftBankAnomalySeverity() const noexcept { return _frozenGripUtilization.leftBankAnomalySeverity; }
        float rightBankAnomalySeverity() const noexcept { return _frozenGripUtilization.rightBankAnomalySeverity; }
        float leftBankPreProjectionUtilization() const noexcept { return _frozenGripUtilization.leftBankPreProjectionUtilization; }
        float rightBankPreProjectionUtilization() const noexcept { return _frozenGripUtilization.rightBankPreProjectionUtilization; }
        float leftBankMemory() const noexcept { return _transientContactMemory.leftBankMemory; }
        float rightBankMemory() const noexcept { return _transientContactMemory.rightBankMemory; }
        float leftBankRecoveryScore() const noexcept { return _regripRecovery.leftBankRecoveryScore; }
        float rightBankRecoveryScore() const noexcept { return _regripRecovery.rightBankRecoveryScore; }
        float leftBankRecoveryTimeRemainingS() const noexcept { return _regripRecovery.leftBankRecoveryTimeRemainingS; }
        float rightBankRecoveryTimeRemainingS() const noexcept { return _regripRecovery.rightBankRecoveryTimeRemainingS; }
        float forwardSpeedProcessNoiseScale() const noexcept { return _frozenSchedule.forwardSpeedProcessNoiseScale; }
        float lateralSpeedProcessNoiseScale() const noexcept { return _frozenSchedule.lateralSpeedProcessNoiseScale; }
        float yawRateProcessNoiseScale() const noexcept { return _frozenSchedule.yawRateProcessNoiseScale; }
        float leftWheelSpeedProcessNoiseScale() const noexcept { return _frozenSchedule.leftWheelSpeedProcessNoiseScale; }
        float rightWheelSpeedProcessNoiseScale() const noexcept { return _frozenSchedule.rightWheelSpeedProcessNoiseScale; }
        float closureCovarianceScaleLeft() const noexcept { return _frozenSchedule.closureCovarianceScaleLeft; }
        float closureCovarianceScaleRight() const noexcept { return _frozenSchedule.closureCovarianceScaleRight; }
        float lateralPseudoMeasurementCovarianceScale() const noexcept
        {
            return _frozenSchedule.lateralPseudoMeasurementCovarianceScale;
        }
        float appliedLeftBankTorqueNm() const noexcept { return _frozenAppliedTorque.leftAppliedBankTorqueNm; }
        float appliedRightBankTorqueNm() const noexcept { return _frozenAppliedTorque.rightAppliedBankTorqueNm; }

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
        DriveCommandSolution solveAlignedDriveCommands(
            const StateVector& currentState,
            float desiredLongitudinalAccelMps2,
            float desiredYawAccelRadps2,
            float fanDutyCycle = 0.80f,
            float batteryVoltageV = 0.0f) const noexcept;
        DriveCommandSolution solveAlignedDriveCommands(
            float forwardVelocityMps,
            float desiredLongitudinalAccelMps2,
            float yawRateRadps,
            float desiredYawAccelRadps2,
            float fanDutyCycle = 0.80f,
            float batteryVoltageV = 0.0f) const noexcept;
        DriveCommandSolution solveAlignedDriveCommandsForVelocityTarget(
            const StateVector& currentState,
            float targetForwardVelocityMps,
            float targetYawRateRadps,
            float fanDutyCycle = 0.80f,
            float batteryVoltageV = 0.0f,
            float responseTimeS = PlantModel::kDefaultVelocityTargetResponseTimeS) const noexcept;
        DriveCommandSolution solveAlignedDriveCommandsForVelocityTarget(
            float currentForwardVelocityMps,
            float targetForwardVelocityMps,
            float currentYawRateRadps,
            float targetYawRateRadps,
            float fanDutyCycle = 0.80f,
            float batteryVoltageV = 0.0f,
            float responseTimeS = PlantModel::kDefaultVelocityTargetResponseTimeS) const noexcept;
        void alignedVelocityTargetTechnicalLimits(
            const StateVector& currentState,
            float& maxLongitudinalAccelMps2,
            float& maxYawAccelRadps2,
            float fanDutyCycle = 0.80f,
            float reserveUsage = 1.0f) const noexcept;
        FeedforwardAuditResult evaluateAlignedFeedforwardOffline(
            const StateVector& currentState,
            float desiredLongitudinalAccelMps2,
            float desiredYawAccelRadps2,
            float fanDutyCycle,
            float batteryVoltageV,
            float reserveUsage,
            float dtS) const noexcept;

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
        static bool IsYawValidForFeedforward(
            OperatingMode mode,
            float bgzRadps,
            float gyroBiasAnchorRadps,
            float yawConsistencyLowPassRadps,
            bool nhcEnabled,
            float lateralVelocityMps,
            float nhcSigmaMps) noexcept;

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
        static float ComputeMeasuredLinearSpeedMps(const EncoderObs& observation, const PlantParams& params) noexcept;
        static float ComputeMeasuredLinearSpeedVarianceMps2(const EncoderObs& observation) noexcept;
        static float ComputeMeasuredYawRateRadps(const EncoderObs& observation, const PlantParams& params) noexcept;
        static float ComputeMeasuredYawRateVarianceRadps2(const EncoderObs& observation, const PlantParams& params) noexcept;
        static float ComputeMeasuredWheelVarianceRadps2(const EncoderObs& observation, const PlantParams& params) noexcept;
        static float ComputeEncoderPairNisThreshold(const EncoderObs& observation) noexcept;
        static StateVector IntegrateStationaryHoldState(const StateVector& currentState, float dtS) noexcept;
        static bool IsPivotScrubCandidate(
            const EncoderObs& observation,
            float commandedLinearMps,
            float commandedAngularRadps,
            float yawConsistencyLowPassRadps,
            float yawWindowMismatchRad,
            const PlantParams& params) noexcept;
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
            const AppliedTorqueEstimate& appliedTorque,
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
        PlantModel::FeedforwardEnvelopeModifiers buildFeedforwardPolicyModifiers() const noexcept;

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

        PlantModel _plantModel;
        WallGeometryModel _geometryModel;
        PlantParams _params;
        PlantModel::PreparedParams _preparedParams;
        UKF<VehicleState::kDimension, 3> _filter;
        float _frozenDtS = 0.0f;
        AppliedTorqueEstimate _frozenAppliedTorque{};
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

