#pragma once
// Defines the drive-local command solver that proposes wheel commands and maintains controller
// state for closed-loop command generation. Hardware application and sensor capture belong to the
// runtime vehicle/sensor owners.
#include "CommandVector.h"
#include "DriveTelemetry.h"
#include "EncoderObs.h"
#include "FeedbackAxis.h"
#include "Maneuver.h"
#include "MazeMapRuntimeCore.h"
#include "PlantModel.h"
#include "PDCluster.h"
#include "SensorSnapshot.h"
#include "VehicleState.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

// Serves as the concrete drive command-proposal subsystem for the MazeMap application. It should not become
// the owner of hardware application, sensor capture, higher-level maneuver scheduling, or shared
// multi-tick motion-routine orchestration.
//
// Numeric contract:
// DriveBase command APIs expect finite scalar targets, operating points, and PlantModel feedforward
// outputs unless an overload explicitly documents stronger input recovery. Passing NaN or infinity is
// a caller/owner contract violation; DriveBase does not scrub those values at the output boundary.
namespace MazeMap
{
    class EXPORT DriveBase
    {
    private:
        static constexpr float kDefaultCommandVelocityAsapLongitudinalAccelLimitMps2 = 9.0f;
        static constexpr float kDefaultCommandVelocityAsapYawAccelLimitRadps2 = 400.0f;
    public:
        static constexpr uint16_t kModeClosedLoop = 1u << 0;
        static constexpr uint16_t kModeBraking = 1u << 3;

        explicit DriveBase(
            const MazeMap::PlantModel& plantModel,
            const MazeMap::VehicleState& runtimeState,
            const MazeMap::PDCluster& proportionalDerivativeCluster)
            : _runtimeState(runtimeState)
            , _linearFeedback(&runtimeState, false)
            , _rotationalFeedback(&runtimeState, true)
            , _plantModel(plantModel)
            , _proportionalDerivativeCluster(&proportionalDerivativeCluster)
            , _lastLinearCommandMps(0.0f)
            , _lastAngularCommandRadps(0.0f)
        {}

        void SetProportionalDerivativeCluster(const MazeMap::PDCluster& proportionalDerivativeCluster) noexcept
        {
            _proportionalDerivativeCluster = &proportionalDerivativeCluster;
        }

        const MazeMap::PDCluster& GetProportionalDerivativeCluster() noexcept { return *_proportionalDerivativeCluster; }
        const MazeMap::PDCluster& GetProportionalDerivativeCluster() const noexcept { return *_proportionalDerivativeCluster; }

        // Retained startup/session reset hook. DriveBase lifetime is runtime-owned, so this does not
        // construct hardware; it clears drive-local measurement caches, command telemetry, and braking
        // state before a mode starts issuing commands.
        bool Begin()
        {
            ResetEncoderTracking();
            ResetControllers();
            Brake();
            return true;
        }

        // Clears controller memory that must not carry across a new command session.
        void ResetControllers()
        {}

        // Returns the last generated command after PWM saturation.
        MazeMap::App::Internal::CommandVector CurrentControlVector() const noexcept
        {
            return _lastProposedCommand;
        }

        // DeltaCommand resolves a feedforward command from an explicit operating point and an explicit
        // longitudinal acceleration request. The result is symmetric by construction.
        // FeedbackSource selections here apply to the longitudinal acceleration target.
        MazeMap::App::Internal::CommandVector DeltaCommand(
            float presentLinearSpeedMps,
            float desiredLongitudinalAccelMps2,
            MazeMap::FeedbackSource linearSources = MazeMap::FeedbackSource::None) const;

        // DeltaCommand resolves the fully coupled plant feedforward from an explicit body-speed operating
        // point and explicit longitudinal/yaw acceleration requests. This is the canonical "delta from the
        // current state" entry point when both translation and rotation matter at once.
        // FeedbackSource selections here apply to the longitudinal and rotational acceleration targets.
        MazeMap::App::Internal::CommandVector DeltaCommand(
            float presentLinearSpeedMps,
            float desiredLongitudinalAccelMps2,
            float presentYawRateRadps,
            float desiredYawAccelRadps2,
            MazeMap::FeedbackSource linearSources = MazeMap::FeedbackSource::None,
            MazeMap::FeedbackSource rotationalSources = MazeMap::FeedbackSource::None) const;

        // DeltaYawRateCommand resolves a zero-mean command from the present yaw rate and a desired yaw
        // acceleration. This is the single-axis rotational variant of `DeltaCommand`.
        // FeedbackSource selections here apply to the rotational acceleration target.
        MazeMap::App::Internal::CommandVector DeltaYawRateCommand(
            float presentYawRateRadps,
            float desiredYawAccelRadps2,
            MazeMap::FeedbackSource rotationalSources = MazeMap::FeedbackSource::None) const;

        // PointCommand drives the present forward speed toward the requested forward speed while preserving
        // the current yaw rate in feedforward. Encoder feedback uses the encoder-average signal for the
        // forward-velocity target only; yaw-rate feedback is not active on this overload.
        // FeedbackSource selections here apply to the forward velocity target.
        // This overload does not set a new heading, yaw-rate, longitudinal-acceleration, or
        // lateral-acceleration target.
        MazeMap::App::Internal::CommandVector PointCommand(
            float desiredLinearSpeedMps,
            MazeMap::FeedbackSource linearSources = MazeMap::FeedbackSource::None) const;

        // PointCommand resolves the fully coupled command that drives the present forward speed and yaw rate
        // toward the requested targets over the canonical roll-off horizon while respecting the plant envelope.
        // This replaces the old ambiguous "velocity command" entry point.
        // FeedbackSource selections here apply to the forward velocity and yaw-rate targets.
        // This overload does not set a heading, longitudinal-acceleration, or lateral-acceleration target.
        MazeMap::App::Internal::CommandVector PointCommand(
            float desiredLinearSpeedMps,
            float desiredYawRateRadps,
            MazeMap::FeedbackSource linearSources = MazeMap::FeedbackSource::None,
            MazeMap::FeedbackSource rotationalSources = MazeMap::FeedbackSource::None) const;

        // PointControlVector forwards to `PointCommand`. It remains as the stable entry point for
        // loop-controller call sites that already operate in control-vector space.
        MazeMap::App::Internal::CommandVector PointControlVector(
            float desiredLinearSpeedMps,
            float desiredYawRateRadps,
            MazeMap::FeedbackSource linearSources = MazeMap::FeedbackSource::None,
            MazeMap::FeedbackSource rotationalSources = MazeMap::FeedbackSource::None) const;

        // PointCommandWithHeadingTarget resolves the same coupled velocity/yaw-rate target as
        // `PointCommand`, but it also lets the caller add DriveBase-owned heading correction in the
        // same composed command so feedforward/feedback decomposition remains authoritative here.
        MazeMap::App::Internal::CommandVector PointCommandWithHeadingTarget(
            float desiredLinearSpeedMps,
            float desiredYawRateRadps,
            float targetYawRad,
            MazeMap::FeedbackSource linearSources,
            MazeMap::FeedbackSource rotationalSources,
            MazeMap::FeedbackSource headingSources) const;

        // PointControlVectorWithHeadingTarget forwards to `PointCommandWithHeadingTarget` as the stable
        // control-vector-space entry point.
        MazeMap::App::Internal::CommandVector PointControlVectorWithHeadingTarget(
            float desiredLinearSpeedMps,
            float desiredYawRateRadps,
            float targetYawRad,
            MazeMap::FeedbackSource linearSources,
            MazeMap::FeedbackSource rotationalSources,
            MazeMap::FeedbackSource headingSources) const;

        // PointCommand consumes the drive-relevant target fields from a maneuver point. Higher-level
        // maneuver execution should target this overload instead of rebuilding scalar command bridges.
        // It exposes the same feedback selections as the scalar `(desiredLinearSpeedMps, desiredYawRateRadps)`
        // overload because it forwards directly to that entry point.
        MazeMap::App::Internal::CommandVector PointCommand(
            const MazeMap::ManeuverPoint& point,
            MazeMap::FeedbackSource linearSources = MazeMap::FeedbackSource::None,
            MazeMap::FeedbackSource rotationalSources = MazeMap::FeedbackSource::None) const;

        // PointControlVector forwards to the maneuver-point `PointCommand`. It remains as the stable entry
        // point for loop-controller call sites that already operate in control-vector space.
        MazeMap::App::Internal::CommandVector PointControlVector(
            const MazeMap::ManeuverPoint& point,
            MazeMap::FeedbackSource linearSources = MazeMap::FeedbackSource::None,
            MazeMap::FeedbackSource rotationalSources = MazeMap::FeedbackSource::None) const;

        // PointYawRateCommand drives the present yaw rate toward the requested target while preserving the
        // current linear speed in feedforward. Encoder feedback uses the encoder-delta signal for the
        // yaw-rate target only; forward-velocity feedback is not active on this overload.
        // FeedbackSource selections here apply to the yaw-rate target.
        MazeMap::App::Internal::CommandVector PointYawRateCommand(
            float desiredYawRateRadps,
            MazeMap::FeedbackSource rotationalSources = MazeMap::FeedbackSource::None) const;

        MazeMap::App::Internal::CommandVector GetFeedbackCommand(
            std::uint8_t linearDerivativeOrder,
            float linearTarget,
            MazeMap::FeedbackSource linearSources,
            std::uint8_t rotationalDerivativeOrder,
            float rotationalTarget,
            MazeMap::FeedbackSource rotationalSources) const;

        // Puts DriveBase's retained command state into an explicit brake/zero-output condition. This is
        // also a telemetry cache reset for generated commands, targets, saturation, and the last proposed
        // command; it does not directly apply hardware output.
        void Brake()
        {
            _lastLinearCommandMps = 0.0f;
            _lastAngularCommandRadps = 0.0f;
            _lastFeedforwardCommand = {};
            _lastFeedbackCommand = {};
            _lastLeftTargetVelocityMps = 0.0f;
            _lastRightTargetVelocityMps = 0.0f;
            _lastModeFlags = kModeBraking;
            _lastSaturationFlags = 0u;
            _lastProposedCommand = {};
        }

        // Returns the average wheel distance since DriveBase's current encoder reference point.
        float GetAverageDistanceMeters() const
        {
            RefreshSensorSnapshotDerivedState();
            return 0.5f * (_leftEncoderDistanceMeters + _rightEncoderDistanceMeters);
        }

        float GetLastLinearCommandMps() const
        {
            return _lastLinearCommandMps;
        }

        float GetLastAngularCommandRadps() const
        {
            return _lastAngularCommandRadps;
        }

        uint16_t GetLastSaturationFlags() const noexcept
        {
            return _lastSaturationFlags;
        }

        // The generated-command decomposition is cached at the point where DriveBase still owns both
        // the plant-model feedforward command and the PD-only correction.
        MazeMap::App::Internal::CommandVector GetLastFeedforward() const noexcept
        {
            return _lastFeedforwardCommand;
        }

        MazeMap::App::Internal::CommandVector GetLastFeedback() const noexcept
        {
            return _lastFeedbackCommand;
        }

        // Packages command-generation telemetry for a caller-supplied command without refreshing sensor
        // state. Use GetTelemetry() when encoder fields and current proposed command are needed.
        DriveTelemetry GetGeneratedTelemetry(
            const MazeMap::App::Internal::CommandVector& command) const noexcept
        {
            DriveTelemetry telemetry{};
            telemetry.leftDriveCommand = command.LeftMotorPwm();
            telemetry.rightDriveCommand = command.RightMotorPwm();
            telemetry.commandedLinearSpeedMps = _lastLinearCommandMps;
            telemetry.commandedAngularSpeedRadps = _lastAngularCommandRadps;
            telemetry.leftFeedforwardCommand = _lastFeedforwardCommand.LeftMotorPwm();
            telemetry.rightFeedforwardCommand = _lastFeedforwardCommand.RightMotorPwm();
            telemetry.leftFeedbackCommand = _lastFeedbackCommand.LeftMotorPwm();
            telemetry.rightFeedbackCommand = _lastFeedbackCommand.RightMotorPwm();
            telemetry.leftTargetVelocityMps = _lastLeftTargetVelocityMps;
            telemetry.rightTargetVelocityMps = _lastRightTargetVelocityMps;
            telemetry.modeFlags = _lastModeFlags;
            telemetry.saturationFlags = _lastSaturationFlags;
            return telemetry;
        }

        // Refreshes sensor-derived cache from runtime state and returns the full DriveBase telemetry
        // snapshot, including encoder accumulation, command decomposition, targets, and mode flags.
        DriveTelemetry GetTelemetry() const
        {
            RefreshSensorSnapshotDerivedState();
            DriveTelemetry telemetry{};
            const MazeMap::App::Internal::CommandVector appliedControl = CurrentControlVector();
            telemetry.leftDriveCommand = appliedControl.LeftMotorPwm();
            telemetry.rightDriveCommand = appliedControl.RightMotorPwm();
            telemetry.commandedLinearSpeedMps = _lastLinearCommandMps;
            telemetry.commandedAngularSpeedRadps = _lastAngularCommandRadps;
            telemetry.leftFeedforwardCommand = _lastFeedforwardCommand.LeftMotorPwm();
            telemetry.rightFeedforwardCommand = _lastFeedforwardCommand.RightMotorPwm();
            telemetry.leftFeedbackCommand = _lastFeedbackCommand.LeftMotorPwm();
            telemetry.rightFeedbackCommand = _lastFeedbackCommand.RightMotorPwm();
            telemetry.leftTargetVelocityMps = _lastLeftTargetVelocityMps;
            telemetry.rightTargetVelocityMps = _lastRightTargetVelocityMps;
            telemetry.leftEncoderCount = _leftEncoderCountTotal;
            telemetry.rightEncoderCount = _rightEncoderCountTotal;
            telemetry.leftDistanceM = _leftEncoderDistanceMeters;
            telemetry.rightDistanceM = _rightEncoderDistanceMeters;
            telemetry.leftVelocityMps = _leftEncoderVelocityMps;
            telemetry.rightVelocityMps = _rightEncoderVelocityMps;
            telemetry.leftEncoderOmegaRadps = _lastEncoderObservation.omegaLeftRadps;
            telemetry.rightEncoderOmegaRadps = _lastEncoderObservation.omegaRightRadps;
            telemetry.modeFlags = _lastModeFlags;
            telemetry.saturationFlags = _lastSaturationFlags;
            telemetry.encoderObservationValid = _encoderObservationValid;
            return telemetry;
        }

    private:
        // Clears DriveBase's cached view of current encoder telemetry and anchors future telemetry to
        // the currently published runtime sensor totals.
        void ResetEncoderTracking() noexcept
        {
            const SensorSnapshot& snapshot = _runtimeState.GetSensorSnapshot();
            _leftEncoderReferenceCounts = snapshot.leftEncoderTotalCounts;
            _rightEncoderReferenceCounts = snapshot.rightEncoderTotalCounts;
            _encoderReferenceInitialized = true;
            _leftEncoderCountTotal = 0;
            _rightEncoderCountTotal = 0;
            _leftEncoderDistanceMeters = 0.0f;
            _rightEncoderDistanceMeters = 0.0f;
            _leftEncoderVelocityMps = 0.0f;
            _rightEncoderVelocityMps = 0.0f;
            _lastEncoderObservation = MazeMap::EncoderObs{};
            _encoderObservationValid = false;
        }

        // Copies the latest runtime sensor snapshot into DriveBase's mutable measurement cache.
        void RefreshSensorSnapshotDerivedState() const noexcept;

        const MazeMap::VehicleState& _runtimeState;
        MazeMap::FeedbackAxis _linearFeedback;
        MazeMap::FeedbackAxis _rotationalFeedback;
        const MazeMap::PlantModel& _plantModel;
        const MazeMap::PDCluster* _proportionalDerivativeCluster;
        mutable float _lastLinearCommandMps;
        mutable float _lastAngularCommandRadps;
        mutable MazeMap::App::Internal::CommandVector _lastFeedforwardCommand{};
        mutable MazeMap::App::Internal::CommandVector _lastFeedbackCommand{};
        mutable MazeMap::App::Internal::CommandVector _lastProposedCommand{};
        mutable float _lastLeftTargetVelocityMps = 0.0f;
        mutable float _lastRightTargetVelocityMps = 0.0f;
        mutable std::int64_t _leftEncoderReferenceCounts = 0;
        mutable std::int64_t _rightEncoderReferenceCounts = 0;
        mutable std::int64_t _leftEncoderCountTotal = 0;
        mutable std::int64_t _rightEncoderCountTotal = 0;
        mutable float _leftEncoderDistanceMeters = 0.0f;
        mutable float _rightEncoderDistanceMeters = 0.0f;
        mutable float _leftEncoderVelocityMps = 0.0f;
        mutable float _rightEncoderVelocityMps = 0.0f;
        mutable float _lastGyroRawRadps = 0.0f;
        mutable float _lastImuYawRateRadps = 0.0f;
        mutable float _lastImuAccelBodyXMps2 = 0.0f;
        mutable float _lastImuAccelBodyYMps2 = 0.0f;
        mutable MazeMap::EncoderObs _lastEncoderObservation{};
        mutable uint16_t _lastModeFlags = kModeBraking;
        mutable uint16_t _lastSaturationFlags = 0u;
        mutable bool _encoderObservationValid = false;
        mutable bool _encoderReferenceInitialized = false;
        mutable bool _lastImuYawRateValid = false;
        mutable bool _lastImuAccelValid = false;
        // Resolves the command operating speed from runtime state first, then sensor-derived wheel/IMU
        // measurements, then zero. The battery-voltage output is retained for plant-model call shape;
        // this function currently leaves it at the caller-provided value.
        void GetVelocityCommandOperatingPoint(
            float& presentLinearSpeedMps,
            float& presentYawRateRadps,
            float& batteryVoltageV) const;

        // Central response horizon used when converting velocity/yaw-rate target error into requested
        // acceleration. Falls back to one second if the PlantModel constant is invalid.
        static float ResolveCommandResponseTimeS() noexcept;

        // Returns positive finite values unchanged and maps all other inputs to zero.
        static float ResolvePositiveOrZero(float value) noexcept;

        // Clamps value to +/-limit when limit is positive and finite; otherwise returns zero.
        static float ClampMagnitude(float value, float limit) noexcept;

        // Converts body linear/yaw targets into left/right wheel linear targets for encoder feedback and
        // command telemetry.
        void ResolveWheelTargets(
            float desiredLinearSpeedMps,
            float desiredYawRateRadps,
            float& leftWheelLinearTargetMps,
            float& rightWheelLinearTargetMps) const;

        // Asks PlantModel for feedforward PWM for the requested acceleration. A near-zero acceleration
        // request is treated as a steady-state hold at the supplied operating speed. PlantModel owns
        // finite feedforward output; DriveBase only clamps to its PWM command envelope.
        MazeMap::App::Internal::CommandVector ResolveRawAccelerationCommand(
            float presentLinearSpeedMps,
            float presentYawRateRadps,
            float desiredLongitudinalAccelMps2,
            float desiredYawAccelRadps2) const;

        // Asks PlantModel for steady-state feedforward PWM for a body velocity/yaw-rate target. PlantModel
        // owns finite feedforward output; DriveBase only clamps to its PWM command envelope.
        MazeMap::App::Internal::CommandVector ResolveRawVelocityTargetCommand(
            float desiredLinearSpeedMps,
            float desiredYawRateRadps) const;

        // Converts a requested longitudinal acceleration correction into the incremental PWM difference
        // from the steady-state command at the current operating point.
        MazeMap::App::Internal::CommandVector ResolveLongitudinalCorrectionCommand(
            float presentLinearSpeedMps,
            float presentYawRateRadps,
            float maxLongitudinalAccelMps2,
            float desiredLongitudinalAccelCorrectionMps2) const;

        // Converts a requested yaw acceleration correction into the incremental PWM difference from the
        // steady-state command at the current operating point.
        MazeMap::App::Internal::CommandVector ResolveYawCorrectionCommand(
            float presentLinearSpeedMps,
            float presentYawRateRadps,
            float maxYawAccelRadps2,
            float desiredYawAccelCorrectionRadps2) const;

        // Records the feedforward/feedback split from the last generated command for telemetry. The
        // feedback value is the pre-final-clamp difference between the composed command and base command.
        void CacheGeneratedCommandTelemetry(
            const MazeMap::App::Internal::CommandVector& feedforwardCommand,
            const MazeMap::App::Internal::CommandVector& feedbackCommand) const noexcept;

        // Resolves the default acceleration envelopes used when turning a velocity/yaw target into a
        // reachable command. The hardcoded DriveBase defaults are capped by PlantModel's technical limits.
        void ResolveDefaultVelocityTargetCommandEnvelope(
            float& maxLongitudinalAccelMps2,
            float& maxYawAccelRadps2) const
        {
            maxLongitudinalAccelMps2 = kDefaultCommandVelocityAsapLongitudinalAccelLimitMps2;
            maxYawAccelRadps2 = kDefaultCommandVelocityAsapYawAccelLimitRadps2;

            float technicalLongitudinalAccelMps2 = 0.0f;
            float technicalYawAccelRadps2 = 0.0f;
            _plantModel.velocityTargetTechnicalLimits(
                technicalLongitudinalAccelMps2,
                technicalYawAccelRadps2,
                GetMissionFanDutyCycle());

            if (std::isfinite(technicalLongitudinalAccelMps2) && (technicalLongitudinalAccelMps2 > 0.0f))
            {
                maxLongitudinalAccelMps2 =
                    (std::min)(maxLongitudinalAccelMps2, technicalLongitudinalAccelMps2);
            }
            if (std::isfinite(technicalYawAccelRadps2) && (technicalYawAccelRadps2 > 0.0f))
            {
                maxYawAccelRadps2 =
                    (std::min)(maxYawAccelRadps2, technicalYawAccelRadps2);
            }
        }

    };

}


