#pragma once
// Declares the canonical runtime sensor owner that captures mission and diagnostic sensor state into one unified snapshot.

#include "CoreConfig.h"
#include "Direction.h"
#include "EncoderObs.h"
#include "RollingAverageWindow.h"
#include "SensorSnapshot.h"

#include <cstdint>

class WallDistanceCalibration;

namespace MazeMap::App::Internal
{
    class LoopController;
    class SharedRobotRuntime;
    class StartupCalibration;
}

namespace MazeMap
{
    class Vehicle;
    class VehicleState;
    class WallSensor;

    class RuntimeSensorSuite final
    {
    public:
        static constexpr std::uint8_t kFrontWallSensorBit = 0x01U;
        static constexpr std::uint8_t kLeftWallSensorBit = 0x02U;
        static constexpr std::uint8_t kRightWallSensorBit = 0x04U;
        static constexpr std::uint8_t kWallSensorBits =
            kFrontWallSensorBit | kLeftWallSensorBit | kRightWallSensorBit;
        static constexpr std::uint8_t kEncoderSensorBit = 0x08U;
        static constexpr std::uint8_t kGyroSensorBit = 0x10U;
        static constexpr std::uint8_t kAccelSensorBit = 0x20U;
        static constexpr std::uint8_t kWallUpdateSensorBit = 0x40U;
        static constexpr std::uint8_t kDefaultSensorWorkBits =
            kWallSensorBits | kEncoderSensorBit | kGyroSensorBit | kAccelSensorBit | kWallUpdateSensorBit;

        RuntimeSensorSuite(MazeMap::Vehicle& vehicle, ::WallDistanceCalibration& wallCalibration);

        // Initializes runtime sensor devices and fixed wall-sensor capture state. IMU startup
        // self-test and bias calibration are owned by StartupCalibration's tick phases.
        bool Begin(unsigned long controlPeriodUs = Config::kControlPeriodUs);
        // Clears retained side-wall memory so a new routine starts from fresh lateral wall state.
        void ResetSideWallMemory() noexcept;
        // Reports the active IMU gyro scale used by the runtime sensor owner.
        float GetGyroSensitivityMdpsPerLsb() const noexcept;
        // Reports the active IMU accelerometer scale used by the runtime sensor owner.
        float GetAccelSensitivityMgPerLsb() const noexcept;
        // Returns the current stationary gyro-bias estimate in body yaw coordinates.
        float GetGyroBiasRadps() const noexcept;
        // Indicates whether the runtime accelerometer bias estimate is initialized.
        bool HasAccelBias() const noexcept;
        // Returns the current body-right accelerometer bias estimate in g.
        float GetAccelBiasRightG() const noexcept;
        // Returns the current body-forward accelerometer bias estimate in g.
        float GetAccelBiasForwardG() const noexcept;
        static bool SensorWorkBitsRequestWallSensors(std::uint8_t sensorWorkBits) noexcept;
        static bool SensorWorkBitsRequestCapture(std::uint8_t sensorWorkBits) noexcept;
        static bool SensorWorkBitsSupportWallUpdates(std::uint8_t sensorWorkBits) noexcept;

    private:
        friend class App::Internal::LoopController;
        friend class App::Internal::SharedRobotRuntime;
        friend class App::Internal::StartupCalibration;

        class WallTelemetryAverager final
        {
        public:
            void Clear() noexcept;
            void PushAndAverage(const MazeMap::WallSensor& sensor) noexcept;
            float AmbientLight() const noexcept;
            float LitLight() const noexcept;
            float DifferentialLight() const noexcept;
            float RawDistanceM() const noexcept;

        private:
            MazeMap::RollingAverageWindow<Config::kWallDetectionAverageWindowCycles> _ambientLight;
            MazeMap::RollingAverageWindow<Config::kWallDetectionAverageWindowCycles> _litLight;
            MazeMap::RollingAverageWindow<Config::kWallDetectionAverageWindowCycles> _differentialLight;
            MazeMap::RollingAverageWindow<Config::kWallDetectionAverageWindowCycles> _rawDistanceM;
        };

        void AttachRuntime(App::Internal::SharedRobotRuntime& runtime) noexcept;
        bool AppendTextLogLine(const char* line) noexcept;
        bool AppendTextLogFormatted(const char* format, ...) noexcept;
        void InitializeWallSensorLedOffState() noexcept;
        void BeginInterlacedCapture(
            const MazeMap::VehicleState& state,
            SensorSnapshot& snapshot,
            std::uint8_t sensorWorkBits,
            float encoderDtSeconds);
        void CaptureInterlacedInertialSnapshot() noexcept;
        void FinishInterlacedCapture();
        void ServiceFrontWallCollection() noexcept;
        void ServiceLeftWallCollection() noexcept;
        void ServiceRightWallCollection() noexcept;
        void AbortInterlacedWallCapture() noexcept;
        void FinalizeInterlacedSnapshot(SensorSnapshot& snapshot) const noexcept;
        void ClearInterlacedCaptureState() noexcept;
        MazeMap::EncoderObs CaptureEncoderObservation(float dtSeconds) noexcept;
        void CaptureEncoderCountsForCalibration(std::int32_t& leftCounts, std::int32_t& rightCounts) noexcept;
        bool HaveEncoderCountsChangedForCalibration(std::int32_t startLeftCounts, std::int32_t startRightCounts) noexcept;
        bool IsEncoderObservationUsableForPrediction(
            const MazeMap::EncoderObs& observation,
            float dtSeconds,
            const char*& degradedReason) const noexcept;
        void ReportEncoderObservationState(
            bool validForPrediction,
            const char* degradedReason,
            const MazeMap::EncoderObs& observation,
            float dtSeconds) noexcept;
        void PublishEncoderTotals(SensorSnapshot& snapshot) const noexcept;
        void CaptureEncoderSnapshot(SensorSnapshot& snapshot, float dtSeconds) noexcept;
        void CaptureInertialSnapshot(SensorSnapshot& snapshot);
        bool UpdateFrontWallState(
            float leftAmbientLight,
            float leftMeasuredDifferentialLight,
            float rightAmbientLight,
            float rightMeasuredDifferentialLight,
            float fallbackDistanceM);

        MazeMap::Vehicle& _vehicle;
        ::WallDistanceCalibration& _wallCalibration;
        App::Internal::SharedRobotRuntime* _runtime = nullptr;
        float _frontLeftWallSignalFiltered;
        float _frontRightWallSignalFiltered;
        float _sideLeftWallSignalFiltered;
        float _sideRightWallSignalFiltered;
        std::int64_t _leftEncoderTotalCounts = 0;
        std::int64_t _rightEncoderTotalCounts = 0;
        const MazeMap::VehicleState* _interlacedCaptureState = nullptr;
        SensorSnapshot* _interlacedCaptureSnapshot = nullptr;
        WallTelemetryAverager _frontLeftTelemetryAverage;
        WallTelemetryAverager _frontRightTelemetryAverage;
        WallTelemetryAverager _sideLeftTelemetryAverage;
        WallTelemetryAverager _sideRightTelemetryAverage;
        bool _frontLeftWallState;
        bool _frontRightWallState;
        bool _sideLeftWallState;
        bool _sideRightWallState;
        bool _frontWallUsesFallbackDetection;
        bool _frontLeftWallSignalInitialized;
        bool _frontRightWallSignalInitialized;
        bool _sideLeftWallSignalInitialized;
        bool _sideRightWallSignalInitialized;
        float _sideLeftPreviousSignalRise;
        float _sideRightPreviousSignalRise;
        bool _sideLeftPreviousSignalRiseValid;
        bool _sideRightPreviousSignalRiseValid;
        bool _encoderObservationDegraded;
        std::uint8_t _interlacedCaptureSensorWorkBits = 0U;
        bool _interlacedCaptureActive = false;
        bool _interlacedCaptureWalls = false;
        bool _interlacedCaptureInertial = false;
        bool _interlacedWallCaptureActive = false;
        bool _frontWallCollectionPending = false;
        bool _leftWallCollectionPending = false;
        bool _rightWallCollectionPending = false;
        bool _interlacedCaptureImuCaptured = false;
        bool _interlacedCaptureIncompleteLogged = false;
        std::uint32_t _frontWallCollectionReadyUs = 0UL;
        std::uint32_t _leftWallCollectionReadyUs = 0UL;
        std::uint32_t _rightWallCollectionReadyUs = 0UL;
    };
}
