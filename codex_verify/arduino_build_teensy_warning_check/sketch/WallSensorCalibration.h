#line 1 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\WallSensorCalibration.h"
#pragma once

#include <cmath>
#include <stdint.h>

namespace MazeMap
{
    enum class WallSensorCalibrationMode : uint8_t
    {
        DistanceOffset,
        DirectInterpolation
    };

    class WallSensorCalibrationCurve
    {
    public:
        struct Point
        {
            float measuredValue = 0.0f;
            float actualDistanceM = 0.0f;
            float ambientLight = 0.0f;
        };

        static constexpr uint8_t kMaxPoints = 32U;

        void Clear()
        {
            _count = 0U;
        }

        bool AddPoint(float measuredValue, float actualDistanceM, float ambientLight = 0.0f)
        {
            if (!std::isfinite(measuredValue) ||
                !std::isfinite(actualDistanceM) ||
                !std::isfinite(ambientLight) ||
                measuredValue <= 0.0f ||
                actualDistanceM <= 0.0f ||
                ambientLight < 0.0f)
            {
                return false;
            }

            for (uint8_t i = 0U; i < _count; ++i)
            {
                if (std::fabs(_points[i].measuredValue - measuredValue) <= 0.001f)
                {
                    _points[i].measuredValue = measuredValue;
                    _points[i].actualDistanceM = actualDistanceM;
                    _points[i].ambientLight = ambientLight;
                    return true;
                }
            }

            if (_count >= kMaxPoints)
            {
                return false;
            }

            uint8_t insertIndex = _count;
            while (insertIndex > 0U && _points[insertIndex - 1U].measuredValue > measuredValue)
            {
                _points[insertIndex] = _points[insertIndex - 1U];
                --insertIndex;
            }

            _points[insertIndex].measuredValue = measuredValue;
            _points[insertIndex].actualDistanceM = actualDistanceM;
            _points[insertIndex].ambientLight = ambientLight;
            ++_count;
            return true;
        }

        float Apply(float measuredValue, WallSensorCalibrationMode mode) const
        {
            if (_count == 0U || !std::isfinite(measuredValue) || measuredValue <= 0.0f)
            {
                return measuredValue;
            }

            if (_count == 1U)
            {
                if (mode == WallSensorCalibrationMode::DistanceOffset)
                {
                    return measuredValue + (_points[0U].actualDistanceM - _points[0U].measuredValue);
                }

                return _points[0U].actualDistanceM;
            }

            if (measuredValue <= _points[0U].measuredValue)
            {
                return Interpolate(_points[0U], _points[1U], measuredValue);
            }

            for (uint8_t i = 1U; i < _count; ++i)
            {
                if (measuredValue <= _points[i].measuredValue)
                {
                    return Interpolate(_points[i - 1U], _points[i], measuredValue);
                }
            }

            return Interpolate(_points[_count - 2U], _points[_count - 1U], measuredValue);
        }

        uint8_t GetCount() const
        {
            return _count;
        }

        const Point& GetPoint(uint8_t index) const
        {
            return _points[index];
        }

    private:
        Point _points[kMaxPoints] = {};
        uint8_t _count = 0U;

        static float Interpolate(const Point& a, const Point& b, float measuredValue)
        {
            const float measuredSpan = b.measuredValue - a.measuredValue;
            if (std::fabs(measuredSpan) <= 1.0e-6f)
            {
                return 0.5f * (a.actualDistanceM + b.actualDistanceM);
            }

            const float t = (measuredValue - a.measuredValue) / measuredSpan;
            return a.actualDistanceM + (t * (b.actualDistanceM - a.actualDistanceM));
        }
    };

}
