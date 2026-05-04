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

        void SetupMode() override;
        LoopController::ControlVector RunTick(
            std::uint32_t loopEndTimeUs,
            const MazeMap::VehicleState& state,
            LoopController& loopController) override;

    private:
        enum class LedCalibrationPhase : std::uint8_t
        {
            Front,
            Side,
            Complete
        };

        static void PauseThunk(void* context, LoopController& loopController);
        static void TeardownOnRuntimeFault(void* context, const char* reason) noexcept;
        static void SetFrontLeds(bool enabled);
        static void SetSideLeds(bool enabled);
        static void SetAllLeds(bool enabled);

        LoopController::SessionOptions BuildLoopOptions() const noexcept;
        LoopController::ControlVector OnModeWork(LoopController& loopController);
        void OnPauseGranted(LoopController& loopController);
        void ToggleActiveLeds();
        void PrintFrequency(const char* label, std::uint32_t halfPeriodUs);
        std::uint32_t ActiveHalfPeriodUs() const noexcept;
        void RunCalibrationLoop();
        void AdvancePhase();
        void ResetState() noexcept;
        void FinalizeSuccessfulRun() noexcept;
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
