#pragma once
// Declares the authoritative micromouse estimator owner that combines the SR-UKF core with maze-evidence updates.

#include "MazeMapRuntimeCore.h"
#include "Maze.h"
#include "MapEvidenceUpdater.h"
#include "SensorSnapshot.h"
#include "SrUkfCore.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace MazeMap
{
    // High-level estimator entry point used by runtime code, simulations, and tests.
    class EXPORT Estimator
    {
    public:
        explicit Estimator(
            const PlantParams& params = PlantParams::Default(),
            VehicleState* runtimeState = nullptr) noexcept;

        void AttachRuntimeState(VehicleState& runtimeState) noexcept;

        SrUkfCore& ukf() noexcept { return _core; }
        const SrUkfCore& ukf() const noexcept { return _core; }
        const ModelCycleContext& modelCycleContext() const noexcept { return _core.modelCycleContext(); }
        MapEvidenceUpdater& mapEvidence() noexcept { return _mapEvidence; }
        const MapEvidenceUpdater& mapEvidence() const noexcept { return _mapEvidence; }
        VehicleState& RuntimeState() noexcept { return *_runtimeState; }
        const VehicleState& RuntimeState() const noexcept { return *_runtimeState; }
        void SyncRuntimeState() noexcept;
        bool HasFault() const noexcept { return _faulted; }
        const char* FaultReason() const noexcept
        {
            return (_faultReason[0] != '\0') ? _faultReason : "ukf_failure";
        }
        void ClearFault() noexcept;

        bool ResetPose(float xMeters, float yMeters, float yawRad) noexcept;
        bool ResetForSessionTransition(float xMeters, float yMeters, float yawRad) noexcept;
        bool SetStateCoordinate(int stateIndex, float coordinateM) noexcept;
        bool SetGyroBiasZ(float gyroBiasRadps) noexcept;
        void ProjectMeasuredKinematics(
            float dtSeconds,
            const EncoderObs& encoderObservation,
            float measuredYawRateRadps = std::numeric_limits<float>::quiet_NaN()) noexcept;

        bool predict(float dt, const ControlInput& control) noexcept;

        template <typename LoopHook>
        bool predict(float dt, const ControlInput& control, LoopHook&& loopHook) noexcept
        {
            return _core.predict(dt, control, static_cast<LoopHook&&>(loopHook));
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
            return _core.updateEncoderPair(observation, dt, updateYaw, static_cast<LoopHook&&>(loopHook));
        }

        MeasurementUpdateResult updateYawRate(float yawRateRadps) noexcept;

        template <typename LoopHook>
        MeasurementUpdateResult updateYawRate(float yawRateRadps, LoopHook&& loopHook) noexcept
        {
            return _core.updateYawRate(yawRateRadps, static_cast<LoopHook&&>(loopHook));
        }

        MeasurementUpdateResult updatePlanarAccel(const ImuAccelObs& observation) noexcept;

        template <typename LoopHook>
        MeasurementUpdateResult updatePlanarAccel(const ImuAccelObs& observation, LoopHook&& loopHook) noexcept
        {
            return _core.updatePlanarAccel(observation, static_cast<LoopHook&&>(loopHook));
        }

        MeasurementUpdateResult updateImuMerged(const ImuMergedObs& observation) noexcept;
        bool reset(
            const SrUkfCore::StateVector& state,
            const SrUkfCore::StateMatrix& covariance) noexcept;

        bool UpdateRuntimeState(
            float dtSeconds,
            const ControlInput& control,
            float commandedLinearSpeedMps,
            float commandedAngularSpeedRadps,
            std::uint16_t saturationFlags,
            float leftLaunchAssistFloor,
            float rightLaunchAssistFloor,
            const EncoderObs& encoderObservation,
            const SensorSnapshot& snapshot,
            const Maze* maze = nullptr,
            ControlCycleTiming* timing = nullptr) noexcept
        {
            return UpdateRuntimeState(
                dtSeconds,
                control,
                commandedLinearSpeedMps,
                commandedAngularSpeedRadps,
                saturationFlags,
                leftLaunchAssistFloor,
                rightLaunchAssistFloor,
                encoderObservation,
                snapshot,
                maze,
                timing,
                NoopUkfLoopHook{},
                []() noexcept {});
        }

        template <typename LoopHook, typename BeforeYawUpdate>
        bool UpdateRuntimeState(
            float dtSeconds,
            const ControlInput& control,
            float commandedLinearSpeedMps,
            float commandedAngularSpeedRadps,
            std::uint16_t saturationFlags,
            float leftLaunchAssistFloor,
            float rightLaunchAssistFloor,
            const EncoderObs& encoderObservation,
            const SensorSnapshot& snapshot,
            const Maze* maze,
            ControlCycleTiming* timing,
            LoopHook&& loopHook,
            BeforeYawUpdate&& beforeYawUpdate) noexcept
        {
            SyncRuntimeMetadata(control, snapshot, dtSeconds);
            if (_faulted)
            {
                SyncRuntimeState();
                return false;
            }

            const PlantParams& params = _core.params();
            _core.setRuntimeContext(
                commandedLinearSpeedMps,
                commandedAngularSpeedRadps,
                saturationFlags,
                leftLaunchAssistFloor,
                rightLaunchAssistFloor,
                snapshot.accelBiasValid,
                snapshot.accelBodyXMps2,
                snapshot.accelBodyYMps2);
            if (timing != nullptr)
            {
                timing->ukfPredictStartUs = micros();
            }
            if ((dtSeconds > 0.0f) && std::isfinite(dtSeconds))
            {
                if (!_core.predict(dtSeconds, control, loopHook))
                {
                    TriggerFault("predict_failed");
                    if (timing != nullptr)
                    {
                        timing->ukfPredictEndUs = micros();
                        timing->ukfPredictDurationUs = timing->ukfPredictEndUs - timing->ukfPredictStartUs;
                        timing->ukfUpdateStartUs = timing->ukfPredictEndUs;
                        timing->ukfUpdateEndUs = timing->ukfPredictEndUs;
                        timing->ukfUpdateDurationUs = 0U;
                        timing->ukfTotalDurationUs = timing->ukfPredictDurationUs;
                    }
                    SyncRuntimeState();
                    return false;
                }
            }
            if (timing != nullptr)
            {
                timing->ukfPredictEndUs = micros();
                timing->ukfPredictDurationUs = timing->ukfPredictEndUs - timing->ukfPredictStartUs;
                timing->ukfUpdateStartUs = micros();
            }

            const bool updateYawFromEncoder = !std::isfinite(snapshot.gyroRawRadps);
            (void)_core.updateEncoderPair(encoderObservation, dtSeconds, updateYawFromEncoder, loopHook);

            beforeYawUpdate();

            if (std::isfinite(snapshot.gyroRawRadps))
            {
                const MeasurementUpdateResult yawUpdate = _core.updateYawRate(snapshot.gyroRawRadps, loopHook);
                if (!yawUpdate.accepted)
                {
                    TriggerFault("yaw_update_failed");
                    if (timing != nullptr)
                    {
                        timing->ukfUpdateEndUs = micros();
                        timing->ukfUpdateDurationUs = timing->ukfUpdateEndUs - timing->ukfUpdateStartUs;
                        timing->ukfTotalDurationUs = timing->ukfPredictDurationUs + timing->ukfUpdateDurationUs;
                    }
                    SyncRuntimeState();
                    return false;
                }
            }

            ImuAccelObs accelObservation{};
            accelObservation.valid =
                snapshot.accelBiasValid &&
                std::isfinite(snapshot.accelBodyXMps2) &&
                std::isfinite(snapshot.accelBodyYMps2);
            accelObservation.accelBodyXMps2 = snapshot.accelBodyXMps2;
            accelObservation.accelBodyYMps2 = snapshot.accelBodyYMps2;
            (void)_core.updatePlanarAccel(accelObservation, loopHook);

            if (maze != nullptr)
            {
                WallObs frontLeftObs{};
                WallObs frontRightObs{};
                BuildFrontPairObservations(snapshot, params.noHitRangeM, frontLeftObs, frontRightObs);
                if (frontLeftObs.valid && frontRightObs.valid)
                {
                    (void)updateFrontPair(frontLeftObs, frontRightObs, *maze, true);
                }

                const WallObs leftSideObs = BuildSideObservation(
                    snapshot.leftDistanceValidForControl,
                    snapshot.leftTransitionDetected,
                    snapshot.leftWallObservation,
                    snapshot.sideLeftDistanceM,
                    params.noHitRangeM);
                if (leftSideObs.valid)
                {
                    (void)updateSideSensor(Side::Left, leftSideObs, *maze, true);
                }

                const WallObs rightSideObs = BuildSideObservation(
                    snapshot.rightDistanceValidForControl,
                    snapshot.rightTransitionDetected,
                    snapshot.rightWallObservation,
                    snapshot.sideRightDistanceM,
                    params.noHitRangeM);
                if (rightSideObs.valid)
                {
                    (void)updateSideSensor(Side::Right, rightSideObs, *maze, true);
                }
            }
            if (timing != nullptr)
            {
                timing->ukfUpdateEndUs = micros();
                timing->ukfUpdateDurationUs = timing->ukfUpdateEndUs - timing->ukfUpdateStartUs;
                timing->ukfTotalDurationUs = timing->ukfPredictDurationUs + timing->ukfUpdateDurationUs;
            }

            SyncRuntimeState();
            return !_faulted;
        }

        FrontPairUpdateResult updateFrontPair(
            const WallObs& left,
            const WallObs& right,
            const Maze& maze,
            bool freezeMapMutation = false,
            const MapEvidenceUpdater::Config& evidenceConfig = MapEvidenceUpdater::Config{}) noexcept;

        WallUpdateResult updateSideSensor(
            Side which,
            const WallObs& observation,
            const Maze& maze,
            bool freezeMapMutation = false,
            const MapEvidenceUpdater::Config& evidenceConfig = MapEvidenceUpdater::Config{}) noexcept;

    private:
        static Direction dominantDirectionForSensor(
            const SensorExtrinsics& sensor,
            const VehicleState::StateVector& state) noexcept;
        static CellCoordinates estimateSensorCell(
            const SensorExtrinsics& sensor,
            const VehicleState::StateVector& state) noexcept;
        static VehicleState::StateMatrix BuildInitialCovariance() noexcept
        {
            return SrUkfCore::BuildDefaultInitialCovariance();
        }
        static void BuildFrontPairObservations(
            const SensorSnapshot& snapshot,
            float maxRangeM,
            WallObs& left,
            WallObs& right) noexcept
        {
            left = WallObs{};
            right = WallObs{};
            if (!snapshot.frontWallObservationValid ||
                !snapshot.frontWall ||
                !(std::isfinite(snapshot.frontLeftDistanceM) && (snapshot.frontLeftDistanceM > 0.0f)) ||
                !(std::isfinite(snapshot.frontRightDistanceM) && (snapshot.frontRightDistanceM > 0.0f)))
            {
                return;
            }

            const float confidence =
                snapshot.frontWallUsesCharacterizationDetection ? 0.90f :
                (snapshot.frontWallUsesFallbackDetection ? 0.60f : 0.80f);
            left.valid = true;
            left.rho = (std::clamp)(snapshot.frontLeftDistanceM, 0.01f, maxRangeM);
            left.confidence = confidence;
            left.cls = ObsClass::WallLike;
            right.valid = true;
            right.rho = (std::clamp)(snapshot.frontRightDistanceM, 0.01f, maxRangeM);
            right.confidence = confidence;
            right.cls = ObsClass::WallLike;
        }
        static WallObs BuildSideObservation(
            bool distanceValidForControl,
            bool transitionDetected,
            bool wallObservation,
            float sideDistanceM,
            float maxRangeM) noexcept
        {
            WallObs observation{};
            if (!distanceValidForControl ||
                transitionDetected ||
                !wallObservation ||
                !(std::isfinite(sideDistanceM) && (sideDistanceM > 0.0f)))
            {
                return observation;
            }

            observation.valid = true;
            observation.rho = (std::clamp)(sideDistanceM, 0.01f, maxRangeM);
            observation.confidence = 0.80f;
            observation.cls = ObsClass::WallLike;
            return observation;
        }
        void SyncRuntimeMetadata(
            const ControlInput& control,
            const SensorSnapshot& snapshot,
            float dtSeconds) noexcept;
        void ResetRuntimeMetadata() noexcept;
        void TriggerFault(const char* reason) noexcept;

        SrUkfCore _core;
        MapEvidenceUpdater _mapEvidence;
        VehicleState _localRuntimeState{};
        VehicleState* _runtimeState{};
        bool _faulted = false;
        char _faultReason[64] = {};
    };
}
