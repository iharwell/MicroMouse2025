#pragma once

#include "CommandVector.h"
#include "CoreConfig.h"
#include "Direction.h"
#include "MotionLimits.h"

#include <array>
#include <cstdint>

namespace MazeMap
{
    class RuntimeSensorSuite;
    class Vehicle;
    class VehicleState;
    class WallSensor;
}

namespace MazeMap::App::Internal
{
    class Drive;
    class SharedRobotRuntime;
    class WallTouch;

    // Owns the shared startup-calibration service used by boot-selected modes. Like Drive, this
    // is a shared multi-tick helper that stays subordinate to the active LoopController callback:
    // boot infrastructure performs required pre-loop `BringUp()`, then modes arm this service
    // with `Start()` and call `GetNextControls(bool& done)` on each tick.
    class EXPORT StartupCalibration final
    {
    public:
        enum class SensorCalibration : std::uint8_t
        {
            None = 0U,
            Imu = 1U << 0,
            FrontLeft = 1U << 1,
            FrontRight = 1U << 2,
            SideLeft = 1U << 3,
            SideRight = 1U << 4,
            AllSensors = (1U << 0) | (1U << 1) | (1U << 2) | (1U << 3) | (1U << 4)
        };

        StartupCalibration();

        void SetIsInMaze(bool isInMaze) noexcept;
        bool GetIsInMaze() const noexcept;

        // Infrastructure-only device bring-up. The IMU self-test and stationary bias capture are
        // intentionally nonblocking phases run by Start()/GetNextControls(...).
        bool BringUp();
        bool Active() const noexcept;
        void Cancel() noexcept;
        void Start();
        SensorCalibration GetSensorsCalibrated() const noexcept;
        CommandVector GetNextControls(bool& done);

    private:
        friend class SharedRobotRuntime;

        enum class Phase : std::uint8_t
        {
            None,
            ReportCompletion,
            ImuBaselineSettle,
            ImuBaselineSample,
            ImuStimulatedSettle,
            ImuStimulatedSample,
            ImuDisabledSettle,
            ImuBiasSample,
            SouthStartHold,
            MoveToCenter,
            CenterHold,
            RotateWest,
            WestHold,
            SampleWest,
            RotateEast,
            EastHold,
            SampleEast,
            RotateSouth,
            SouthTouch,
            RotateWestReseat,
            WestTouch,
            RotateNorth,
            NorthHold,
            SampleFrontBaseline,
            FinalHold
        };

        void AttachRuntime(SharedRobotRuntime& runtime) noexcept;

        void ResetState() noexcept;
        void ResetImuCalibrationState() noexcept;
        void UpdateDoneState(bool& done) noexcept;
        void LogIssue(const char* reason) noexcept;
        void CompleteBestEffort(const char* reason) noexcept;
        [[noreturn]] void FailCalibration(const char* reason) noexcept;
        void RefreshSensorsCalibrated() noexcept;
        void RestoreSideReferenceStateFromCalibration() noexcept;
        static float StartupCellCenterCoordinateM() noexcept;
        static MotionLimits BuildStartupTravelLimits() noexcept;
        static bool IsValidPositiveBand(float low, float high) noexcept;
        static bool IsValidNonNegativeBand(float low, float high) noexcept;
        static bool HasFrontLeftBaselineCalibration() noexcept;
        static bool HasFrontRightBaselineCalibration() noexcept;
        static bool HasFullSideCalibration(MazeMap::RelativeDirection side) noexcept;
        static bool HasAnySideCalibrationData(MazeMap::RelativeDirection side) noexcept;
        static bool HasAnyWallCalibrationData() noexcept;
        static bool TryComputeDistanceToSouthStartWall(
            const MazeMap::VehicleState& state,
            const MazeMap::WallSensor& sensor,
            float& distanceM) noexcept;
        std::uint32_t TicksForDurationUs(std::uint32_t durationUs) const noexcept;
        std::uint32_t TicksForDurationMs(std::uint32_t durationMs) const noexcept;
        std::uint32_t ImuCalibrationSampleIntervalTicks() const noexcept;
        unsigned long RequiredGyroBiasSamples() const noexcept;
        void CaptureCurrentEncoderTotalsForImuCalibration() noexcept;
        bool EncoderTotalsChangedDuringImuCalibration() const noexcept;
        void RestartImuCalibrationAfterMotion(const char* reason) noexcept;
        bool BeginImuCalibration() noexcept;
        CommandVector RunImuCalibrationPhase(bool& done);
        void BeginImuSettlePhase(Phase phase) noexcept;
        void BeginImuSamplePhase(Phase phase, unsigned long requiredSamples) noexcept;
        void AccumulateCurrentImuSelfTestSample() noexcept;
        void AccumulateCurrentImuBiasSample() noexcept;
        void StoreCurrentSelfTestAverageAsBaseline() noexcept;
        bool ValidateAndStoreStimulatedSelfTestAverage() noexcept;
        bool CompleteImuBiasMeasurement() noexcept;
        bool BeginMazeWallCalibration() noexcept;
        bool BeginDriveHoldPhase(Phase phase, std::uint16_t durationMs) noexcept;
        bool BeginDriveMovePhase(Phase phase, float targetXMeters, float targetYMeters, MazeMap::Direction headingDirection) noexcept;
        bool BeginDriveTurnPhase(Phase phase, MazeMap::Direction targetDirection) noexcept;
        bool BeginWallTouchPhase(Phase phase, MazeMap::Direction wallDirection) noexcept;

        void AdvanceAfterDrivePhase() noexcept;
        void AdvanceAfterWallTouchPhase() noexcept;
        bool SampleWestFacingSideCalibration() noexcept;
        bool SampleEastFacingSideCalibration() noexcept;
        bool SampleFrontBaseline() noexcept;

        bool SampleSideWallPair(bool& sampleComplete) noexcept;
        bool SampleFrontWallPair(bool& sampleComplete) noexcept;
        bool BeginWallSensorPairSampling(
            MazeMap::WallSensor& first,
            MazeMap::WallSensor& second,
            bool measuredValueFromRawDistance) noexcept;
        bool ServiceWallSensorPairSampling(bool& sampleComplete) noexcept;
        void AccumulateWallSensorPairSample() noexcept;
        void FinishWallSensorPairSampling() noexcept;
        void ResetWallSamplingState() noexcept;
        static void AccumulateFiniteWallValue(float sample, double& sum, std::uint16_t& count) noexcept;
        static float AverageWallCalibrationSum(double sum, std::uint16_t count) noexcept;
        bool StoreSideReference(
            MazeMap::RelativeDirection side,
            float measuredValue,
            float ambientLight,
            float differentialLight,
            bool differentialLightBandValid,
            float differentialLightBandLow,
            float differentialLightBandHigh,
            float actualDistanceM) noexcept;
        bool StoreSideBaseline(
            MazeMap::RelativeDirection side,
            float differentialLight,
            bool differentialLightBandValid,
            float differentialLightBandLow,
            float differentialLightBandHigh) noexcept;
        bool StoreFrontLeftBaseline(
            float differentialLight,
            bool differentialLightBandValid,
            float differentialLightBandLow,
            float differentialLightBandHigh) noexcept;
        bool StoreFrontRightBaseline(
            float differentialLight,
            bool differentialLightBandValid,
            float differentialLightBandLow,
            float differentialLightBandHigh) noexcept;

        static constexpr const char* kLogSource = "startup_calibration";
        static constexpr std::uint32_t kImuSelfTestSettleUs = 50000U;
        static constexpr unsigned long kImuSelfTestAverageSamples = 64UL;
        static constexpr std::uint32_t kImuCalibrationSampleIntervalUs = 2000U;
        static constexpr std::uint16_t kWallCalibrationSampleCount =
            static_cast<std::uint16_t>(Config::kWallCalibrationAverageSampleCount);
        static constexpr std::uint16_t kWallCalibrationPairSamplingTimeoutMs = 250U;

        SharedRobotRuntime* _runtime{};
        MazeMap::RuntimeSensorSuite* _sensors{};
        Drive* _driveService{};
        WallTouch* _wallTouch{};
        MazeMap::Vehicle* _vehicle{};
        MotionLimits _travelLimits{};
        bool _isInMaze{};
        bool _broughtUp{};
        bool _imuCalibrationComplete{};
        bool _useFallbackWallCalibration{};
        Phase _phase{ Phase::None };
        SensorCalibration _sensorsCalibrated{ SensorCalibration::None };
        float _leftSideReferenceDistanceM{};
        float _rightSideReferenceDistanceM{};
        bool _leftSideReferenceValid{};
        bool _rightSideReferenceValid{};
        MazeMap::WallSensor* _wallSampleFirstSensor{};
        MazeMap::WallSensor* _wallSampleSecondSensor{};
        std::array<float, Config::kWallCalibrationAverageSampleCount> _wallFirstDifferentialSamples{};
        std::array<float, Config::kWallCalibrationAverageSampleCount> _wallSecondDifferentialSamples{};
        std::uint16_t _wallSampleCount{};
        std::uint16_t _wallFirstMeasuredCount{};
        std::uint16_t _wallFirstAmbientCount{};
        std::uint16_t _wallFirstDifferentialCount{};
        std::uint16_t _wallSecondMeasuredCount{};
        std::uint16_t _wallSecondAmbientCount{};
        std::uint16_t _wallSecondDifferentialCount{};
        std::uint32_t _wallSampleTicksRemaining{};
        bool _wallSampleActive{};
        bool _wallSampleAmbientCaptured{};
        bool _wallSampleMeasuredValueFromRawDistance{};
        bool _wallSampleTimedOut{};
        double _wallFirstMeasuredSum{};
        double _wallFirstAmbientSum{};
        double _wallFirstDifferentialSum{};
        double _wallSecondMeasuredSum{};
        double _wallSecondAmbientSum{};
        double _wallSecondDifferentialSum{};
        float _wallFirstMeasuredValue{};
        float _wallFirstAmbientLight{};
        float _wallFirstDifferentialLight{};
        bool _wallFirstDifferentialLightBandValid{};
        float _wallFirstDifferentialLightBandLow{};
        float _wallFirstDifferentialLightBandHigh{};
        float _wallSecondMeasuredValue{};
        float _wallSecondAmbientLight{};
        float _wallSecondDifferentialLight{};
        bool _wallSecondDifferentialLightBandValid{};
        float _wallSecondDifferentialLightBandLow{};
        float _wallSecondDifferentialLightBandHigh{};
        unsigned long _controlPeriodUs{ Config::kControlPeriodUs };
        std::uint32_t _imuPhaseTicksRemaining{};
        std::uint32_t _imuSampleCountdownTicks{};
        unsigned long _imuRequiredSamples{};
        std::int64_t _imuStartLeftEncoderCounts{};
        std::int64_t _imuStartRightEncoderCounts{};
    };

    inline constexpr StartupCalibration::SensorCalibration operator|(
        const StartupCalibration::SensorCalibration lhs,
        const StartupCalibration::SensorCalibration rhs) noexcept
    {
        return static_cast<StartupCalibration::SensorCalibration>(
            static_cast<std::uint8_t>(lhs) | static_cast<std::uint8_t>(rhs));
    }

    inline constexpr StartupCalibration::SensorCalibration operator&(
        const StartupCalibration::SensorCalibration lhs,
        const StartupCalibration::SensorCalibration rhs) noexcept
    {
        return static_cast<StartupCalibration::SensorCalibration>(
            static_cast<std::uint8_t>(lhs) & static_cast<std::uint8_t>(rhs));
    }

    inline constexpr StartupCalibration::SensorCalibration& operator|=(
        StartupCalibration::SensorCalibration& lhs,
        const StartupCalibration::SensorCalibration rhs) noexcept
    {
        lhs = lhs | rhs;
        return lhs;
    }
}
