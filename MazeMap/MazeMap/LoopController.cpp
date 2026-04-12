#include "pch.h"
#include "LoopController.h"

namespace MazeMap::App::Internal
{
    LoopController::LoopController(
        unsigned long controlPeriodUs,
        unsigned int idleSleepUs) noexcept
        : _controlPeriodUs(controlPeriodUs)
        , _idleSleepUs(idleSleepUs)
        , _lastTickMicros(micros())
        , _lastTick{}
    {
    }

    void LoopController::Reset() noexcept
    {
        Reset(micros());
    }

    void LoopController::Reset(unsigned long nowUs) noexcept
    {
        _lastTickMicros = nowUs;
        _lastTick.startUs = nowUs;
        _lastTick.dtUs = 0U;
        _lastTick.dtSeconds = 0.0f;
    }

    LoopController::Tick LoopController::WaitForNextTick() noexcept
    {
        return WaitForNextTick([]() noexcept {});
    }

    unsigned long LoopController::ControlPeriodUs() const noexcept
    {
        return _controlPeriodUs;
    }

    unsigned int LoopController::IdleSleepUs() const noexcept
    {
        return _idleSleepUs;
    }

    unsigned long LoopController::LastTickMicros() const noexcept
    {
        return _lastTickMicros;
    }

    const LoopController::Tick& LoopController::LastTick() const noexcept
    {
        return _lastTick;
    }

    LoopController::Tick LoopController::LatchTick(unsigned long tickStartUs) noexcept
    {
        _lastTick.startUs = tickStartUs;
        _lastTick.dtUs = static_cast<std::uint32_t>(tickStartUs - _lastTickMicros);
        _lastTick.dtSeconds = static_cast<float>(_lastTick.dtUs) * 1.0e-6f;
        _lastTickMicros = tickStartUs;
        return _lastTick;
    }
}
