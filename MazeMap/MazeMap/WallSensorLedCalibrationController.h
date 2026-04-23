#pragma once

#include "Defines.h"
#include "LoopController.h"
#include "IApplicationMode.h"

#include <cstdint>

namespace MazeMap::App
{
    struct BootModeDescriptor;
}

namespace MazeMap::App::Internal
{
    class SharedRobotRuntime;

    class EXPORT WallSensorLedCalibrationController final : public IApplicationMode
    {
    public:
        explicit WallSensorLedCalibrationController(SharedRobotRuntime& runtime);

        bool Begin() override;
        void Run() override;

    private:
        enum class LedCalibrationPhase : std::uint8_t
        {
            Front,
            Side,
            Complete
        };

        static LoopController::ControlVector ModeWorkThunk(
            void* context,
            std::uint32_t loopEndTimeUs,
            const LoopController::ModeState& state,
            LoopController::TickServices& services);
        static LoopController::PauseDisposition PauseThunk(
            void* context,
            const LoopController::PauseContext& pause);
        static void TeardownOnRuntimeFault(void* context, const char* reason) noexcept;
        static void SetFrontLeds(bool enabled);
        static void SetSideLeds(bool enabled);
        static void SetAllLeds(bool enabled);

        LoopController::SessionOptions BuildLoopOptions() const noexcept;
        LoopController::ControlVector OnModeWork(LoopController::TickServices& services);
        LoopController::PauseDisposition OnPauseGranted(const LoopController::PauseContext& pause);
        void ToggleActiveLeds();
        void PrintFrequency(const char* label, std::uint32_t halfPeriodUs);
        std::uint32_t ActiveHalfPeriodUs() const noexcept;
        void RunCalibrationLoop();
        void AdvancePhase();
        void ResetState() noexcept;
        void CleanupHardware() noexcept;
        void CleanupOnRuntimeFault(const char* reason) noexcept;

        SharedRobotRuntime& _runtime;
        LoopController& _loopController;
        std::uint8_t _monitorDrivePin{};
        std::uint8_t _monitorSensePin{};
        LedCalibrationPhase _phase{ LedCalibrationPhase::Front };
        bool _ledEnabled{};
        std::uint32_t _lastToggleUs{};
        bool _monitorArmed{};
        bool _runtimeFaulted{};
        bool _pauseRequested{};
        const char* _runtimeFaultReason{};
    };

    EXPORT IApplicationMode& GetWallSensorLedCalibrationMode();
    EXPORT const BootModeDescriptor& GetWallSensorLedCalibrationBootModeDescriptor();
}
