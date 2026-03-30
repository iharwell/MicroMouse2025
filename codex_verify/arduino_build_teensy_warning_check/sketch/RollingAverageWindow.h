#line 1 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\RollingAverageWindow.h"
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

            _samples[_nextIndex] = sample;
            if (_count < WindowSize)
            {
                ++_count;
            }
            _nextIndex = static_cast<uint8_t>((_nextIndex + 1U) % WindowSize);
            return Average();
        }

        float Average() const noexcept
        {
            if (_count == 0U)
            {
                return 0.0f;
            }

            float sortedSamples[WindowSize] = {};
            uint8_t validCount = 0U;
            for (uint8_t index = 0U; index < _count; ++index)
            {
                const float sample = _samples[index];
                if (!std::isfinite(sample))
                {
                    continue;
                }

                sortedSamples[validCount] = sample;
                ++validCount;
            }

            if (validCount == 0U)
            {
                return 0.0f;
            }

            InsertionSort(sortedSamples, validCount);

            uint8_t trimCount = 0U;
            if (validCount >= 5U)
            {
                trimCount = static_cast<uint8_t>(validCount / 5U);
            }

            if ((trimCount * 2U) >= validCount)
            {
                trimCount = 0U;
            }

            double sum = 0.0;
            const uint8_t startIndex = trimCount;
            const uint8_t endIndex = static_cast<uint8_t>(validCount - trimCount);
            for (uint8_t index = startIndex; index < endIndex; ++index)
            {
                sum += static_cast<double>(sortedSamples[index]);
            }

            const uint8_t keptCount = static_cast<uint8_t>(endIndex - startIndex);
            return (keptCount > 0U) ? static_cast<float>(sum / static_cast<double>(keptCount)) : 0.0f;
        }

        uint8_t GetCount() const noexcept
        {
            return _count;
        }

    private:
        static void InsertionSort(float* samples, uint8_t count) noexcept
        {
            for (uint8_t index = 1U; index < count; ++index)
            {
                const float value = samples[index];
                uint8_t insertIndex = index;
                while ((insertIndex > 0U) && (samples[insertIndex - 1U] > value))
                {
                    samples[insertIndex] = samples[insertIndex - 1U];
                    --insertIndex;
                }

                samples[insertIndex] = value;
            }
        }

        float _samples[WindowSize] = {};
        uint8_t _count = 0U;
        uint8_t _nextIndex = 0U;
    };
}
