#pragma once

#include <cmath>
#include <stdint.h>

namespace MazeMap
{
    template <uint8_t WindowSize>
    class RollingAverageWindow
    {
    public:
        void Clear() noexcept
        {
            _sum = 0.0f;
            _count = 0U;
            _nextIndex = 0U;
            for (uint8_t index = 0U; index < WindowSize; ++index)
            {
                _samples[index] = 0.0f;
            }
        }

        float Push(float sample) noexcept
        {
            if (!std::isfinite(sample))
            {
                return Average();
            }

            if (_count < WindowSize)
            {
                _samples[_nextIndex] = sample;
                _sum += sample;
                ++_count;
            }
            else
            {
                _sum += sample - _samples[_nextIndex];
                _samples[_nextIndex] = sample;
            }

            _nextIndex = static_cast<uint8_t>((_nextIndex + 1U) % WindowSize);
            return Average();
        }

        float Average() const noexcept
        {
            return (_count > 0U) ? (_sum / static_cast<float>(_count)) : 0.0f;
        }

        uint8_t GetCount() const noexcept
        {
            return _count;
        }

    private:
        float _samples[WindowSize] = {};
        float _sum = 0.0f;
        uint8_t _count = 0U;
        uint8_t _nextIndex = 0U;
    };
}
