#pragma once

#include "Defines.h"

#include <cstdint>

namespace MazeMap::App::Internal
{
    // Mock home for the shared fixed-period control-loop timing owner.
    class LoopController final
    {
    public:
        struct Tick
        {
            unsigned long startUs = 0UL;
            std::uint32_t dtUs = 0U;
            float dtSeconds = 0.0f;
        };

        static constexpr unsigned int kDefaultIdleSleepUs = 20U;

        explicit LoopController(
            unsigned long controlPeriodUs,
            unsigned int idleSleepUs = kDefaultIdleSleepUs) noexcept;

        void Reset() noexcept;
        void Reset(unsigned long nowUs) noexcept;

        Tick WaitForNextTick() noexcept;

        template <typename IdleWorkFn>
        Tick WaitForNextTick(IdleWorkFn&& idleWork) noexcept
        {
            if ((micros() - _lastTickMicros) < _controlPeriodUs)
            {
                idleWork();
                while ((micros() - _lastTickMicros) < _controlPeriodUs)
                {
                    delayMicroseconds(_idleSleepUs);
                }
            }

            return LatchTick(micros());
        }

        unsigned long ControlPeriodUs() const noexcept;
        unsigned int IdleSleepUs() const noexcept;
        unsigned long LastTickMicros() const noexcept;
        const Tick& LastTick() const noexcept;

    private:
        Tick LatchTick(unsigned long tickStartUs) noexcept;

        unsigned long _controlPeriodUs;
        unsigned int _idleSleepUs;
        unsigned long _lastTickMicros;
        Tick _lastTick;
    };
}
