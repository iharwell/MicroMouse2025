#pragma once

#include "BootModeDescriptor.h"
#include "LoopController.h"
#include "MazeMapApplicationMode.h"
#include "SmoothTurnYawRateController.h"

#include <cstdint>

class DiagnosticSensorSuite;
class DriveBase;

namespace MazeMap::App::Internal
{
    class SharedRobotRuntime;

    class TopSpeedMeasurementMode final : public IApplicationMode
    {
    public:
        explicit TopSpeedMeasurementMode(SharedRobotRuntime& runtime);

        bool Begin() override;
        void Run() override;

    private:
        enum class RunPhase : std::uint8_t
        {
            PrelaunchWait,
            Running,
            Braking
        };

        enum class BrakeTrigger : std::uint8_t
        {
            None,
            TimedWindowElapsed,
            ImpactDetected,
            GyroSpikeDetected,
            SelectorRemoved
        };

        enum class CompletionReason : std::uint8_t
        {
            None,
            SettledStop,
            SelectorRemoved
        };

        static void HandleRuntimeFault(void* context, const char* reason) noexcept;
        static LoopController::ControlVector ModeWorkThunk(
            void* context,
            std::uint32_t loopEndTimeUs,
            const LoopController::ModeState& state,
            LoopController::TickServices& services);

        LoopController::SessionOptions BuildLoopOptions() const noexcept;
        bool BeginLog();
        bool FailLogSetupStep(const char* step);
        bool FlushTextLogStep(const char* step);
        void CloseLog() noexcept;
        bool WriteEvent(const char* type, const char* message);
        bool WriteRunStartEvent();
        bool WriteResultSummary(std::uint32_t completedTicks);
        bool LogSample(const LoopController::ModeState& state);
        bool EnterBrakingPhase(
            BrakeTrigger trigger,
            const char* eventMessage,
            LoopController::TickServices& services);
        bool EnsureSelectorStillPresent();
        bool SelectorRemoved() const noexcept;
        void ConfigureSelectorMonitor() noexcept;
        void ReleaseSelectorMonitor() noexcept;
        bool ImpactDetected(const LoopController::ModeState& state) const noexcept;
        bool GyroSpikeDetected(const LoopController::ModeState& state) const noexcept;
        bool EncoderMotionSettled(const LoopController::ModeState& state) noexcept;
        void UpdatePeaks(const LoopController::ModeState& state) noexcept;
        LoopController::ControlVector RunTick(
            const LoopController::ModeState& state,
            LoopController::TickServices& services);
        bool Fail(const char* reason);
        void OnRuntimeFault(const char* reason) noexcept;
        void ResetRunState() noexcept;
        void SetLastCommandInputs(float linearSpeedMps, float angularRateRadps) noexcept;
        float ReadBatteryVoltage() const noexcept;

        SharedRobotRuntime& _runtime;
        LoopController& _loopController;
        DiagnosticSensorSuite& _sensors;
        DriveBase& _drive;
        bool _faulted{};
        bool _logOpen{};
        bool _pinsLatchedAtBoot{};
        RunPhase _phase{ RunPhase::PrelaunchWait };
        BrakeTrigger _brakeTrigger{ BrakeTrigger::None };
        CompletionReason _completionReason{ CompletionReason::None };
        std::uint32_t _phaseStartUs{};
        std::uint32_t _measurementStartUs{};
        std::uint32_t _controlTickSequence{};
        float _batteryVoltageStart{};
        float _fanDutyCycleStart{};
        float _measurementStartYawRad{};
        float _peakMeasuredSpeedMps{};
        float _peakPlanarAccelMps2{};
        float _peakHeadingDeviationRad{};
        float _mostNegativeForwardAccelMps2{};
        bool _impactDetected{};
        float _impactSpeedMps{};
        float _impactForwardAccelMps2{};
        std::int16_t _impactGyroXRawLsb{};
        std::int16_t _impactGyroYRawLsb{};
        float _lastCommandInputLinearSpeedMps{};
        float _lastCommandInputAngularRateRadps{};
        MazeMap::SmoothTurnYawRateControllerState _gyroZHoldControllerState{};
        std::uint8_t _settledEncoderTicks{};
        std::uint8_t _selectorDrivePin{};
        std::uint8_t _selectorSensePin{};
        bool _selectorMonitorArmed{};
        char _runId[32]{};
    };

    IApplicationMode& GetTopSpeedMeasurementMode();
    const BootModeDescriptor& GetTopSpeedMeasurementBootModeDescriptor();
}
