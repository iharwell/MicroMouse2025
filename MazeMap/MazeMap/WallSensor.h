#pragma once

#include "Defines.h"
#include "EigenCompat.h"
#include "SensorTelemetryTypes.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace MazeMap
{
    class VehicleState;

    class EXPORT WallSensor
    {
    public:
        class DistanceModel final
        {
        public:
            constexpr DistanceModel(
                float scale,
                float exponent,
                float minDeltaLight,
                float maxDistance) noexcept
                : _scale(scale)
                , _exponent(exponent)
                , _minDeltaLight(minDeltaLight)
                , _maxDistance(maxDistance)
            {
            }

            float Scale() const noexcept;
            float Exponent() const noexcept;
            float MinDeltaLight() const noexcept;
            float MaxDistance() const noexcept;
            float DistanceFromDifferentialLight(float deltaLightLevel) const noexcept;

        private:
            float _scale;
            float _exponent;
            float _minDeltaLight;
            float _maxDistance;
        };

        WallSensor(
            std::uint8_t wallSensorInPin,
            std::uint8_t ledOutPin,
            const Eigen::Vector2f& position,
            const Eigen::Vector2f& facingDirection,
            const std::array<float, 8>& adcToLightTable,
            const DistanceModel& distanceModel,
            float noHitRangeM = 0.30f,
            std::uint32_t ambientSettleTimeUs = 0U,
            std::uint32_t litSettleTimeUs = 0U);

        std::uint8_t GetWallSensorInPin() const noexcept;
        std::uint8_t GetLedOutPin() const noexcept;
        const Eigen::Vector2f& GetPosition() const noexcept;
        const Eigen::Vector2f& GetFacingDirection() const noexcept;
        Eigen::Vector2f WorldPosition(const MazeMap::VehicleState& state) const noexcept;
        Eigen::Vector2f WorldFacing(const MazeMap::VehicleState& state) const noexcept;
        bool TryComputeWallAimCoordinateM(const MazeMap::VehicleState& state, float& alongWallCoordinateM) const noexcept;
        bool IsWallSegmentCenterWindowValid(const MazeMap::VehicleState& state, float keptFraction) const noexcept;
        bool IsSideWallSegmentCenterAligned(const MazeMap::VehicleState& state) const noexcept;
        float GetNoHitRangeM() const noexcept;
        std::uint32_t GetAmbientSettleTimeUs() const noexcept;
        std::uint32_t GetLitSettleTimeUs() const noexcept;

        void CommandLedOff(std::uint32_t commandUs) noexcept;
        void CommandLedOff() noexcept;
        void CommandLedOn(std::uint32_t commandUs) noexcept;
        void CommandLedOn() noexcept;
        bool IsLedEnabled() const noexcept;
        std::uint32_t LatestLedOffCommandUs() const noexcept;
        std::uint32_t LatestLedOnCommandUs() const noexcept;
        std::uint32_t AmbientReadyUs() const noexcept;
        std::uint32_t LitReadyUs() const noexcept;
        bool IsAmbientReadReady(std::uint32_t nowUs) const noexcept;
        bool IsLitReadReady(std::uint32_t nowUs) const noexcept;
        void AwaitAmbientReady() const noexcept;
        void AwaitLitReady() const noexcept;

        bool HasAmbientRead() const noexcept;
        bool HasLitRead() const noexcept;
        bool IsCaptureArmed() const noexcept;
        bool HasCompletedCapture() const noexcept;
        void CaptureAmbientRead() noexcept;
        void CaptureLitRead() noexcept;
        bool TryCompleteLitRead(std::uint32_t nowUs) noexcept;
        void CompleteCapture(std::uint32_t ledOffCommandUs) noexcept;
        void CaptureBlocking() noexcept;

        std::uint16_t LatestAmbientAdcCode() const noexcept;
        std::uint16_t LatestLitAdcCode() const noexcept;
        float LatestAmbientLight() const noexcept;
        float LatestLitLight() const noexcept;
        float LatestDifferentialLight() const noexcept;
        float LatestRawDistanceM() const noexcept;
        const OpticalObservationTiming& LatestTiming() const noexcept;

        std::uint16_t ReadAdcCode() const noexcept;
        float AdcCodeToLightLevel(std::uint16_t adcReading) const noexcept;
        float ReadLightLevel() const noexcept;

        static float DifferentialLightLevel(float ambientLightLevel, float litLightLevel) noexcept;

        float DistanceFromDifferentialLight(float deltaLightLevel) const noexcept;
        float DistanceFromLightLevels(float ambientLightLevel, float litLightLevel) const noexcept;
        float ReadDistance(float darkLightLevel) const noexcept;

    private:
        static constexpr std::uint16_t kAdcMax = 4095U;
        static constexpr float kTableSpan = 7.0f;

        std::uint8_t _wallSensorInPin;
        std::uint8_t _ledOutPin;
        std::uint8_t _wallSensorAdc1Channel = 0U;
        bool _hasDedicatedWallSensorAdc1Channel = false;
        Eigen::Vector2f _position;
        Eigen::Vector2f _facingDirection;
        std::array<float, 8> _adcToLightTable;
        DistanceModel _distanceModel;
        float _noHitRangeM;
        std::uint32_t _ambientSettleTimeUs;
        std::uint32_t _litSettleTimeUs;
        std::uint32_t _latestLedOffCommandUs = 0UL;
        std::uint32_t _latestLedOnCommandUs = 0UL;
        std::uint32_t _ambientReadyUs = 0UL;
        std::uint32_t _litReadyUs = 0UL;
        std::uint16_t _latestAmbientAdcCode = 0U;
        std::uint16_t _latestLitAdcCode = 0U;
        float _latestAmbientLight = 0.0f;
        float _latestLitLight = 0.0f;
        float _latestDifferentialLight = 0.0f;
        float _latestRawDistanceM = 0.20f;
        OpticalObservationTiming _latestTiming{};
        bool _ledEnabled = false;
        bool _ambientCaptured = false;
        bool _litCaptured = false;
        bool _captureArmed = false;
        bool _captureComplete = false;

        static Eigen::Vector2f Normalize(const Eigen::Vector2f& v) noexcept;
        static float Lerp(float a, float b, float t) noexcept;
        static bool TimeReached(std::uint32_t nowUs, std::uint32_t readyUs) noexcept;
        float AdcToLightLevelInternal(std::uint16_t adcReading) const noexcept;
        float DeltaLightToDistance(float deltaLightLevel) const noexcept;
    };
}
