#pragma once




#if defined(_WIN32) || defined(_WIN64)

#else
#include <Arduino.h>
#endif
#include <array>
#include <assert.h>
#include <cmath>
#include <stddef.h>
#include <stdint.h>
#include "defines.h"
#include "vector2f.h"


namespace MazeMap
{

    class WallSensor
    {
    public:

        struct DistanceModel
        {
            float scale;          // Numerator in d = scale / (delta^exponent)
            float exponent;       // Positive exponent
            float minDeltaLight;  // Prevents divide-by-near-zero / excessive range
            float maxDistance;    // Clamp for very weak returns
        };

        WallSensor(
            uint8_t wallSensorInPin,
            uint8_t ledOutPin,
            const Vectorf<2>& position,
            const Vectorf<2>& facingDirection,
            const std::array<float, 8>& adcToLightTable,
            const DistanceModel& distanceModel
        )
            : _wallSensorInPin(wallSensorInPin),
            _ledOutPin(ledOutPin),
            _position(position),
            _facingDirection(Normalize(facingDirection)),
            _adcToLightTable(adcToLightTable),
            _distanceModel(distanceModel)
        {
#ifndef NDEBUG
            const float dirMagSq =
                facingDirection.GetX() * facingDirection.GetX() +
                facingDirection.GetY() * facingDirection.GetY();

            assert(dirMagSq > 0.0f);
            assert(distanceModel.exponent > 0.0f);
            assert(distanceModel.minDeltaLight >= 0.0f);
            assert(distanceModel.maxDistance > 0.0f);
#endif

            pinMode(_wallSensorInPin, INPUT);
            pinMode(_ledOutPin, OUTPUT);
            digitalWrite(_ledOutPin, LOW);
        }

        uint8_t GetWallSensorInPin() const { return _wallSensorInPin; }
        uint8_t GetLedOutPin() const { return _ledOutPin; }
        const Vectorf<2>& GetPosition() const { return _position; }
        const Vectorf<2>& GetFacingDirection() const { return _facingDirection; }

        void SetLedEnabled(bool enabled) const
        {
            digitalWrite(_ledOutPin, enabled ? HIGH : LOW);
        }

        float ReadLightLevel() const
        {
            const uint16_t adcReading = static_cast<uint16_t>(analogRead(_wallSensorInPin));
            return AdcToLightLevel(adcReading);
        }

        static float DifferentialLightLevel(float ambientLightLevel, float litLightLevel)
        {
            const float deltaLightLevel = litLightLevel - ambientLightLevel;
            return (deltaLightLevel > 0.0f) ? deltaLightLevel : 0.0f;
        }

        float DistanceFromDifferentialLight(float deltaLightLevel) const
        {
            if (deltaLightLevel < 0.0f)
            {
                deltaLightLevel = 0.0f;
            }

            return DeltaLightToDistance(deltaLightLevel);
        }

        float DistanceFromLightLevels(float ambientLightLevel, float litLightLevel) const
        {
            return DistanceFromDifferentialLight(DifferentialLightLevel(ambientLightLevel, litLightLevel));
        }

        float ReadDistance(float darkLightLevel) const
        {
            return DistanceFromLightLevels(darkLightLevel, ReadLightLevel());
        }

    private:
        static constexpr uint16_t kAdcMax = 4095U;
        static constexpr float kTableSpan = 7.0f;

        uint8_t _wallSensorInPin;
        uint8_t _ledOutPin;
        Vectorf<2> _position;
        Vectorf<2> _facingDirection;
        std::array<float, 8> _adcToLightTable;
        DistanceModel _distanceModel;

        static Vectorf<2> Normalize(const Vectorf<2>& v)
        {
            const float magSq = v.GetX() * v.GetX() + v.GetY() * v.GetY();

#ifndef NDEBUG
            assert(magSq > 0.0f);
#endif

            if (magSq <= 0.0f)
            {
                return { 1.0f, 0.0f };
            }

            const float invMag = 1.0f / std::sqrt(magSq);
            return { v.GetX() * invMag, v.GetY() * invMag };
        }

        static float Lerp(float a, float b, float t)
        {
            return a + (b - a) * t;
        }

        [[nodiscard]] float AdcToLightLevel(uint16_t adcReading) const
        {
            if (adcReading >= kAdcMax)
            {
                return _adcToLightTable[7];
            }

            const float scaledIndex =
                (static_cast<float>(adcReading) * kTableSpan) /
                static_cast<float>(kAdcMax);

            const size_t index = static_cast<size_t>(scaledIndex);

            if (index >= 7U)
            {
                return _adcToLightTable[7];
            }

            const float frac = scaledIndex - static_cast<float>(index);

            return Lerp(
                _adcToLightTable[index],
                _adcToLightTable[index + 1U],
                frac
            );
        }

        [[nodiscard]] float DeltaLightToDistance(float deltaLightLevel) const
        {
            if (deltaLightLevel <= _distanceModel.minDeltaLight)
            {
                return _distanceModel.maxDistance;
            }

            const float distance =
                _distanceModel.scale /
                std::pow(deltaLightLevel, _distanceModel.exponent);

            if (distance > _distanceModel.maxDistance)
            {
                return _distanceModel.maxDistance;
            }

            return distance;
        }
    };

}
