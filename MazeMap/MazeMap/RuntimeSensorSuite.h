#pragma once
// Declares the canonical runtime sensor owner that captures mission and diagnostic sensor state into one unified snapshot.

#include "MazeMapRuntimeCore.h"
#include "MazeMapRuntimeSignalHelpers.h"
#include "SensorSnapshot.h"
#include "WallDistanceCalibration.h"
#include "WallSensorRuntimeTypes.h"

class RuntimeSensorSuite final
{
public:
    class CaptureServices final
    {
    public:
        // Advances the in-flight wall-sensor sweep while caller work is running.
        bool ServiceWallRead() noexcept;
        // Captures the IMU portion of the active snapshot exactly once.
        void CaptureImu() noexcept;

    private:
        friend class RuntimeSensorSuite;

        using ServiceWallReadFn = bool (*)(void* context) noexcept;
        using CaptureImuFn = void (*)(void* context) noexcept;

        void* _context{};
        ServiceWallReadFn _serviceWallRead{};
        CaptureImuFn _captureImu{};
    };

    using CaptureHandler = void (*)(void* context, SensorSnapshot& snapshot, CaptureServices& services) noexcept;

    RuntimeSensorSuite(MazeMap::Vehicle& vehicle, WallDistanceCalibration& wallCalibration);

    // Initializes the unified runtime sensor pipeline and calibrates stationary gyro bias for the supplied control period.
    bool Begin(unsigned long controlPeriodUs = Config::kControlPeriodUs);
    // Re-runs stationary gyro bias calibration using the supplied loop period and accel-runtime setting.
    bool CalibrateGyroBias(unsigned long controlPeriodUs, bool enableAccelRuntime);
    // Clears retained side-wall memory so a new routine starts from fresh lateral wall state.
    void ResetSideWallMemory() noexcept;
    // Captures the full unified sensor snapshot into caller-owned storage.
    // When provided, `callback` may interleave estimator work while the wall sweep remains in flight.
    void Capture(
        bool stationary,
        const MazeMap::VehicleState& state,
        SensorSnapshot& snapshot,
        CaptureHandler callback = nullptr,
        void* callbackContext = nullptr,
        bool captureEncoders = false,
        float encoderDtSeconds = 0.0f);

    // Reports the active IMU gyro scale used by the runtime sensor owner.
    float GetGyroSensitivityMdpsPerLsb() const noexcept;
    // Reports the active IMU accelerometer scale used by the runtime sensor owner.
    float GetAccelSensitivityMgPerLsb() const noexcept;
    // Returns the current stationary gyro-bias estimate in body yaw coordinates.
    float GetGyroBiasRadps() const noexcept;
    // Indicates whether the runtime accelerometer bias estimate is initialized.
    bool HasAccelBias() const noexcept;
    // Returns the current body-frame X accelerometer bias estimate in g.
    float GetAccelBiasXG() const noexcept;
    // Returns the current body-frame Y accelerometer bias estimate in g.
    float GetAccelBiasYG() const noexcept;

private:
    struct FilteredIrChannel final
    {
        float filteredDistanceM = 0.20f;
        bool wall = false;
        bool initialized = false;
    };

    void InitializeWallSensorLedOffState() noexcept;
    void CaptureInertialSnapshot(bool stationary, SensorSnapshot& snapshot);
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
    static float UpdateChannelFromMeasuredDistance(FilteredIrChannel& channel, float measuredDistanceM);
    static float UpdateChannelFromMeasuredDistance(
        FilteredIrChannel& channel,
        float measuredDistanceM,
        float onThresholdM,
        float offThresholdM);
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
    FilteredIrChannel _frontLeft;
    FilteredIrChannel _frontRight;
    FilteredIrChannel _sideLeft;
    FilteredIrChannel _sideRight;
    float _frontLeftWallSignalFiltered;
    float _frontRightWallSignalFiltered;
    float _sideLeftWallSignalFiltered;
    float _sideRightWallSignalFiltered;
    float _accelBiasXG;
    float _accelBiasYG;
    std::uint32_t _frontLeftLedOffCommandUs = 0UL;
    std::uint32_t _frontRightLedOffCommandUs = 0UL;
    std::uint32_t _sideLeftLedOffCommandUs = 0UL;
    std::uint32_t _sideRightLedOffCommandUs = 0UL;
    AveragedWallSensorInputWindow<Config::kWallDetectionAverageWindowCycles> _frontLeftInputAverage;
    AveragedWallSensorInputWindow<Config::kWallDetectionAverageWindowCycles> _frontRightInputAverage;
    AveragedWallSensorInputWindow<Config::kWallDetectionAverageWindowCycles> _sideLeftInputAverage;
    AveragedWallSensorInputWindow<Config::kWallDetectionAverageWindowCycles> _sideRightInputAverage;
    bool _frontLeftWallState;
    bool _frontRightWallState;
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
};
