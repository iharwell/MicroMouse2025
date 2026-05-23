#pragma once




#if defined(_WIN32) || defined(_WIN64)

#else
#include <Arduino.h>
#endif
#include "EigenCompat.h"
#include <array>
#include <assert.h>
#include <cmath>
#include <stddef.h>
#include <stdint.h>
#include "defines.h"


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
            const Eigen::Vector2f& position,
            const Eigen::Vector2f& facingDirection,
            const std::array<float, 8>& adcToLightTable,
            const DistanceModel& distanceModel,
            float noHitRangeM = 0.30f
        )
            : _wallSensorInPin(wallSensorInPin),
            _ledOutPin(ledOutPin),
            _position(position),
            _facingDirection(Normalize(facingDirection)),
            _adcToLightTable(adcToLightTable),
            _distanceModel(distanceModel),
            _noHitRangeM((std::isfinite(noHitRangeM) && (noHitRangeM > 0.0f)) ? noHitRangeM : 0.30f)
        {
#ifndef NDEBUG
            const float dirMagSq =
                facingDirection.squaredNorm();

            assert(dirMagSq > 0.0f);
            assert(distanceModel.exponent > 0.0f);
            assert(distanceModel.minDeltaLight >= 0.0f);
            assert(distanceModel.maxDistance > 0.0f);
#endif

            pinMode(_wallSensorInPin, INPUT);
            pinMode(_ledOutPin, OUTPUT);
            digitalWrite(_ledOutPin, LOW);
            _hasDedicatedWallSensorAdc1Channel =
                MazeMap::Platform::ResolveWallSensorAdc1Channel(_wallSensorInPin, _wallSensorAdc1Channel);
        }

        uint8_t GetWallSensorInPin() const { return _wallSensorInPin; }
        uint8_t GetLedOutPin() const { return _ledOutPin; }
        const Eigen::Vector2f& GetPosition() const { return _position; }
        const Eigen::Vector2f& GetFacingDirection() const { return _facingDirection; }
        float GetNoHitRangeM() const { return _noHitRangeM; }

        void SetLedEnabled(bool enabled) const
        {
            digitalWrite(_ledOutPin, enabled ? HIGH : LOW);
        }

        uint16_t ReadAdcCode() const
        {
            if (_hasDedicatedWallSensorAdc1Channel)
            {
                return MazeMap::Platform::ReadWallSensorAdcCodeFromConfiguredChannel(_wallSensorAdc1Channel);
            }

            return MazeMap::Platform::ReadWallSensorAdcCode(_wallSensorInPin);
        }

        float AdcCodeToLightLevel(uint16_t adcReading) const
        {
            return AdcToLightLevel(adcReading);
        }

        float ReadLightLevel() const
        {
            return AdcCodeToLightLevel(ReadAdcCode());
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
        uint8_t _wallSensorAdc1Channel = 0U;
        bool _hasDedicatedWallSensorAdc1Channel = false;
        Eigen::Vector2f _position;
        Eigen::Vector2f _facingDirection;
        std::array<float, 8> _adcToLightTable;
        DistanceModel _distanceModel;
        float _noHitRangeM;

        static Eigen::Vector2f Normalize(const Eigen::Vector2f& v)
        {
            const float magSq = v.squaredNorm();

#ifndef NDEBUG
            assert(magSq > 0.0f);
#endif

            if (magSq <= 0.0f)
            {
                return Eigen::Vector2f(1.0f, 0.0f);
            }

            return v.normalized();
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
