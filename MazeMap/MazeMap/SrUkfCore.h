#pragma once
// Declares the square-root UKF core that owns the process model and measurement updates.

#include "PlantModel.h"
#include "WallGeometryModel.h"
#include "UKF.h"

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
        // April 8, 2026 D:\open_floor_main.csv tuning:
        // - stationary hold set the IMU noise floor,
        // - repeated open-loop launch passes set the moving encoder noise floor.
        static constexpr float kGeneralEncoderLinearSpeedSigmaMps = 0.0027f;
        static constexpr float kGeneralEncoderYawRateSigmaRadps = 0.065f;
        static constexpr float kStationaryEncoderVelocitySigmaMps = 1.76e-6f;
        static constexpr float kEncoderPairNisThreshold = 13.81551f;
        static constexpr float kImuYawRateSigmaRadps = 0.0028f;
        static constexpr float kImuAccelSigmaMps2 = 0.0052f;

        using StateVector = VehicleState::StateVector;
        using StateMatrix = VehicleState::StateMatrix;
        using DebugTextSink = bool (*)(void* context, const char* type, const char* message) noexcept;

        SrUkfCore(
            const PlantParams& params = PlantParams::Default(),
            const PlantModel& plantModel = PlantModel()) noexcept;

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

        MeasurementUpdateResult updateEncoderPair(const EncoderObs& observation, float dt) noexcept;

        template <typename LoopHook>
        MeasurementUpdateResult updateEncoderPair(
            const EncoderObs& observation,
            float dt,
            LoopHook&& loopHook) noexcept
        {
            using HookType = std::remove_reference_t<LoopHook>;
            return updateEncoderPairImpl(
                observation,
                dt,
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
            const LocalMapView& map) noexcept;
        WallUpdateResult updateSideSensor(
            Side which,
            const WallObs& observation,
            const LocalMapView& map) noexcept;

        static Eigen::Matrix<float, 2, 2> ComputeGeneralEncoderPairCovarianceRadps(
            const PlantParams& params) noexcept;
        static Eigen::Matrix<float, 2, 2> ComputeGeneralEncoderPairSqrtNoise(
            const PlantParams& params) noexcept;
        static float ComputeStationaryEncoderOmegaSigmaRadps(const PlantParams& params) noexcept;
        static Eigen::Matrix<float, 2, 2> ComputeEncoderPairSqrtNoise(
            const EncoderObs& observation,
            const PlantParams& params) noexcept;
        static float ComputeEncoderPairNisThreshold(const EncoderObs& observation) noexcept;

    private:
        using LoopHookInvoker = void (*)(void*) noexcept;

        static void InvokeLoopHook(void* context, LoopHookInvoker loopHook) noexcept;
        static bool HasExactZeroWheelObservation(const EncoderObs& observation) noexcept;
        static StateMatrix BuildDefaultProcessNoiseDensity() noexcept;
        static float ComputeDistancePerEncoderCountM(const PlantParams& params) noexcept;
        static float ComputeMeasuredLinearSpeedMps(const EncoderObs& observation, const PlantParams& params) noexcept;
        static float ComputeMeasuredLinearSpeedVarianceMps2(const EncoderObs& observation) noexcept;
        static float ComputeMeasuredYawRateRadps(const EncoderObs& observation, const PlantParams& params) noexcept;
        static float ComputeMeasuredYawRateVarianceRadps2(const EncoderObs& observation, const PlantParams& params) noexcept;
        static float ComputeMeasuredWheelVarianceRadps2(const EncoderObs& observation, const PlantParams& params) noexcept;
        static float wallNoiseFromConfidence(float confidence, float minimumNoise) noexcept;

        bool predictImpl(float dt, const ControlInput& control, void* loopHookContext, LoopHookInvoker loopHook) noexcept;
        MeasurementUpdateResult updateEncoderPairImpl(
            const EncoderObs& observation,
            float dt,
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
        void anchorPoseToEncoderDelta(StateVector& anchoredState, const EncoderObs& measured) const noexcept;
        void applyWheelRateConstraint(const EncoderObs& measured, float wheelVarianceRadps2) noexcept;
        void applyStationaryZeroMotionConstraint(float yawRateRadps) noexcept;
        Eigen::Matrix<float, 2, 1> frontPairPredictionForState(
            const StateVector& sigmaPoint,
            const LocalMapView& map) const noexcept;
        float wallPredictionForSensor(
            const StateVector& sigmaPoint,
            const SensorExtrinsics& sensor,
            const LocalMapView& map) const noexcept;

        PlantModel _plantModel;
        WallGeometryModel _geometryModel;
        PlantParams _params;
        UKF<VehicleState::kDimension, 3> _filter;
        ControlInput _lastControl;
        EncoderObs _lastEncoderObs;
        StateVector _prePredictState;
        StateMatrix _prePredictCovariance;
        bool _havePredictionReference;
        bool _acceptedEncoderUpdateSincePredict;
        StateMatrix _sqrtProcessNoiseDensity;
        Eigen::Matrix<float, 3, 3> _sqrtImuNoise;
        Eigen::Matrix<float, 2, 2> _sqrtFrontNoise;
        Eigen::Matrix<float, 1, 1> _sqrtSideNoise;
    };
}
