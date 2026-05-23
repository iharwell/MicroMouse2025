#pragma once
// Declares the canonical runtime sensor owner that captures mission and diagnostic sensor state into one unified snapshot.

#include "MazeMapRuntimeCore.h"
#include "MazeMapRuntimeSignalHelpers.h"
#include "SensorSnapshot.h"
#include "WallDistanceCalibration.h"
#include "WallSensorRuntimeTypes.h"

#include <cstdint>

namespace MazeMap::App::Internal
{
    class LoopController;
}

namespace MazeMap
{
    class RuntimeSensorSuite final
    {
    public:
        RuntimeSensorSuite(MazeMap::Vehicle& vehicle, WallDistanceCalibration& wallCalibration);

        // Initializes the unified runtime sensor pipeline and calibrates stationary gyro bias for the supplied control period.
        bool Begin(unsigned long controlPeriodUs = Config::kControlPeriodUs);
        // Re-runs stationary gyro bias calibration using the supplied loop period and accel-runtime setting.
        bool CalibrateGyroBias(unsigned long controlPeriodUs, bool enableAccelRuntime);
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

    private:
        friend class App::Internal::LoopController;

        void InitializeWallSensorLedOffState() noexcept;
        void BeginInterlacedCapture(
            bool stationary,
            const MazeMap::VehicleState& state,
            SensorSnapshot& snapshot,
            bool captureWalls,
            bool captureEncoders,
            float encoderDtSeconds);
        void CaptureInterlacedInertialSnapshot() noexcept;
        void FinishInterlacedCapture();
        void ServiceFrontWallCollection() noexcept;
        void ServiceLeftWallCollection() noexcept;
        void ServiceRightWallCollection() noexcept;
        void ClearInterlacedCaptureState() noexcept;
        MazeMap::EncoderObs CaptureEncoderObservation(float dtSeconds) noexcept;
        MazeMap::EncoderCountPair CaptureEncoderCountPairForCalibration() noexcept;
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
        void CaptureInertialSnapshot(bool stationary, SensorSnapshot& snapshot);
        StationaryImuCalibrationResult WaitForBackLeftImuCalibrationSettle(
            const MazeMap::EncoderCountPair& startCounts,
            unsigned long settleMs) noexcept;
        StationaryImuCalibrationResult AverageBackLeftImuSelfTestSample(
            std::uint16_t sampleCount,
            const MazeMap::EncoderCountPair& startCounts,
            AveragedBackLeftImuSample& averagedSample) noexcept;
        StationaryImuCalibrationResult RunStationaryBackLeftImuSelfTest(unsigned long controlPeriodUs);
        bool CalibrateStationaryBackLeftGyroBias(unsigned long controlPeriodUs, bool enableAccelRuntime);
        bool TryComputeSideSignalMetrics(
            WallSensorId sensorId,
            float measuredDifferentialLight,
            float& signalRise,
            float& latchRiseThreshold,
            float& missRiseThreshold) const;
        static bool DetectTransitionFromSignalRise(
            bool windowValid,
            bool signalMetricsValid,
            float signalRise,
            float transitionThreshold,
            float& previousSignalRise,
            bool& previousValid) noexcept;
        bool ComputeSideWallObservationHit(
            WallSensorId sensorId,
            float measuredDifferentialLight,
            float fallbackDistanceM,
            float onThresholdM,
            bool detectionWindowValid) const;
        bool UpdateSideWallState(
            WallSensorId sensorId,
            float measuredDifferentialLight,
            float fallbackDistanceM,
            float onThresholdM,
            float offThresholdM,
            bool detectionWindowValid,
            float& filteredSignal,
            bool& signalInitialized,
            bool& currentState);
        bool UpdateFrontWallState(
            float leftAmbientLight,
            float leftMeasuredDifferentialLight,
            float rightAmbientLight,
            float rightMeasuredDifferentialLight,
            float fallbackDistanceM);
        WallSensorTelemetry BuildWallSensorTelemetry(
            WallSensorId sensorId,
            const WallSensorCalibrationInput& input) const;
        float ReadGyroZRadpsRaw();

        MazeMap::Vehicle& _vehicle;
        WallDistanceCalibration& _wallCalibration;
        float _gyroBiasRadps;
        float _frontLeftWallSignalFiltered;
        float _frontRightWallSignalFiltered;
        float _sideLeftWallSignalFiltered;
        float _sideRightWallSignalFiltered;
        float _accelBiasRightG;
        float _accelBiasForwardG;
        std::int64_t _leftEncoderTotalCounts = 0;
        std::int64_t _rightEncoderTotalCounts = 0;
        std::uint32_t _frontLeftLedOffCommandUs = 0UL;
        std::uint32_t _frontRightLedOffCommandUs = 0UL;
        std::uint32_t _sideLeftLedOffCommandUs = 0UL;
        std::uint32_t _sideRightLedOffCommandUs = 0UL;
        AsyncWallSensorSweepRead _interlacedWallRead{};
        const MazeMap::VehicleState* _interlacedCaptureState = nullptr;
        SensorSnapshot* _interlacedCaptureSnapshot = nullptr;
        AveragedWallSensorInputWindow<Config::kWallDetectionAverageWindowCycles> _frontLeftInputAverage;
        AveragedWallSensorInputWindow<Config::kWallDetectionAverageWindowCycles> _frontRightInputAverage;
        AveragedWallSensorInputWindow<Config::kWallDetectionAverageWindowCycles> _sideLeftInputAverage;
        AveragedWallSensorInputWindow<Config::kWallDetectionAverageWindowCycles> _sideRightInputAverage;
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
        bool _accelBiasInitialized;
        bool _encoderObservationDegraded;
        bool _interlacedCaptureActive = false;
        bool _interlacedCaptureWalls = false;
        bool _interlacedCaptureStationary = false;
        bool _interlacedCaptureImuCaptured = false;
    };
}
