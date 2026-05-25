#include "pch.h"
#include "WallSensor.h"

#include "CoreConfig.h"
#include "MissionStartPolicy.h"
#include "VehicleState.h"

#include <assert.h>
#include <cmath>

namespace MazeMap
{
    float WallSensor::DistanceModel::Scale() const noexcept
    {
        return _scale;
    }

    float WallSensor::DistanceModel::Exponent() const noexcept
    {
        return _exponent;
    }

    float WallSensor::DistanceModel::MinDeltaLight() const noexcept
    {
        return _minDeltaLight;
    }

    float WallSensor::DistanceModel::MaxDistance() const noexcept
    {
        return _maxDistance;
    }

    float WallSensor::DistanceModel::DistanceFromDifferentialLight(float deltaLightLevel) const noexcept
    {
        if (deltaLightLevel <= _minDeltaLight)
        {
            return _maxDistance;
        }

        const float distance = _scale / std::pow(deltaLightLevel, _exponent);
        return (distance > _maxDistance) ? _maxDistance : distance;
    }

    WallSensor::WallSensor(
        const std::uint8_t wallSensorInPin,
        const std::uint8_t ledOutPin,
        const Eigen::Vector2f& position,
        const Eigen::Vector2f& facingDirection,
        const std::array<float, 8>& adcToLightTable,
        const DistanceModel& distanceModel,
        const float noHitRangeM,
        const std::uint32_t ambientSettleTimeUs,
        const std::uint32_t litSettleTimeUs)
        : _wallSensorInPin(wallSensorInPin)
        , _ledOutPin(ledOutPin)
        , _position(position)
        , _facingDirection(Normalize(facingDirection))
        , _adcToLightTable(adcToLightTable)
        , _distanceModel(distanceModel)
        , _noHitRangeM((std::isfinite(noHitRangeM) && (noHitRangeM > 0.0f)) ? noHitRangeM : 0.30f)
        , _ambientSettleTimeUs(ambientSettleTimeUs)
        , _litSettleTimeUs(litSettleTimeUs)
    {
#ifndef NDEBUG
        const float dirMagSq = facingDirection.squaredNorm();
        assert(dirMagSq > 0.0f);
        assert(distanceModel.Exponent() > 0.0f);
        assert(distanceModel.MinDeltaLight() >= 0.0f);
        assert(distanceModel.MaxDistance() > 0.0f);
#endif

        pinMode(_wallSensorInPin, INPUT);
        pinMode(_ledOutPin, OUTPUT);
        digitalWrite(_ledOutPin, LOW);
        _hasDedicatedWallSensorAdc1Channel =
            MazeMap::Platform::ResolveWallSensorAdc1Channel(_wallSensorInPin, _wallSensorAdc1Channel);

        const std::uint32_t nowUs = micros();
        _latestLedOffCommandUs = nowUs;
        _ambientReadyUs = nowUs + _ambientSettleTimeUs;
        _latestTiming.ledOffCommandUs = nowUs;
    }

    std::uint8_t WallSensor::GetWallSensorInPin() const noexcept
    {
        return _wallSensorInPin;
    }

    std::uint8_t WallSensor::GetLedOutPin() const noexcept
    {
        return _ledOutPin;
    }

    const Eigen::Vector2f& WallSensor::GetPosition() const noexcept
    {
        return _position;
    }

    const Eigen::Vector2f& WallSensor::GetFacingDirection() const noexcept
    {
        return _facingDirection;
    }

    Eigen::Vector2f WallSensor::WorldPosition(const MazeMap::VehicleState& state) const noexcept
    {
        const Eigen::Vector2f headingUnit = state.GetHeadingUnit();
        const Eigen::Vector2f rightUnit(headingUnit.y(), -headingUnit.x());
        const Eigen::Vector2f worldOffset(
            (rightUnit.x() * _position.x()) + (headingUnit.x() * _position.y()),
            (rightUnit.y() * _position.x()) + (headingUnit.y() * _position.y()));
        return Eigen::Vector2f(
            state.GetPositionX() + worldOffset.x(),
            state.GetPositionY() + worldOffset.y());
    }

    Eigen::Vector2f WallSensor::WorldFacing(const MazeMap::VehicleState& state) const noexcept
    {
        const Eigen::Vector2f headingUnit = state.GetHeadingUnit();
        const Eigen::Vector2f rightUnit(headingUnit.y(), -headingUnit.x());
        return Eigen::Vector2f(
            (rightUnit.x() * _facingDirection.x()) + (headingUnit.x() * _facingDirection.y()),
            (rightUnit.y() * _facingDirection.x()) + (headingUnit.y() * _facingDirection.y()));
    }

    bool WallSensor::TryComputeWallAimCoordinateM(
        const MazeMap::VehicleState& state,
        float& alongWallCoordinateM) const noexcept
    {
        alongWallCoordinateM = 0.0f;
        if (!std::isfinite(state.GetPositionX()) ||
            !std::isfinite(state.GetPositionY()) ||
            !std::isfinite(state.GetHeading()))
        {
            return false;
        }

        const Eigen::Vector2f sensorPosition = WorldPosition(state);
        const Eigen::Vector2f sensorFacing = WorldFacing(state);
        const float sensorXM = sensorPosition.x();
        const float sensorYM = sensorPosition.y();
        const float facingXM = sensorFacing.x();
        const float facingYM = sensorFacing.y();
        const float innerMinCoordinateM =
            MazeMap::ComputeCellInnerMinCoordinateM(MazeMap::Config::kMazeWallThicknessM);
        const float innerMaxCoordinateM =
            MazeMap::ComputeCellInnerMaxCoordinateM(
                MazeMap::Config::kCellSizeM,
                MazeMap::Config::kMazeWallThicknessM);

        if (std::fabs(facingXM) >= std::fabs(facingYM))
        {
            if (!(std::fabs(facingXM) > 1.0e-4f))
            {
                return false;
            }

            const float cellBaseXM =
                std::floor(sensorXM / MazeMap::Config::kCellSizeM) * MazeMap::Config::kCellSizeM;
            const float wallFaceXM =
                (facingXM >= 0.0f) ?
                (cellBaseXM + innerMaxCoordinateM) :
                (cellBaseXM + innerMinCoordinateM);
            const float rayScale = (wallFaceXM - sensorXM) / facingXM;
            if (!(rayScale >= 0.0f) || !std::isfinite(rayScale))
            {
                return false;
            }

            alongWallCoordinateM = sensorYM + (rayScale * facingYM);
            return std::isfinite(alongWallCoordinateM);
        }

        if (!(std::fabs(facingYM) > 1.0e-4f))
        {
            return false;
        }

        const float cellBaseYM =
            std::floor(sensorYM / MazeMap::Config::kCellSizeM) * MazeMap::Config::kCellSizeM;
        const float wallFaceYM =
            (facingYM >= 0.0f) ?
            (cellBaseYM + innerMaxCoordinateM) :
            (cellBaseYM + innerMinCoordinateM);
        const float rayScale = (wallFaceYM - sensorYM) / facingYM;
        if (!(rayScale >= 0.0f) || !std::isfinite(rayScale))
        {
            return false;
        }

        alongWallCoordinateM = sensorXM + (rayScale * facingXM);
        return std::isfinite(alongWallCoordinateM);
    }

    bool WallSensor::IsWallSegmentCenterWindowValid(
        const MazeMap::VehicleState& state,
        const float keptFraction) const noexcept
    {
        float alongWallCoordinateM = 0.0f;
        return
            TryComputeWallAimCoordinateM(state, alongWallCoordinateM) &&
            MazeMap::IsWithinWallSegmentCenterWindowM(
                alongWallCoordinateM,
                MazeMap::Config::kCellSizeM,
                MazeMap::Config::kMazeWallThicknessM,
                keptFraction);
    }

    bool WallSensor::IsSideWallSegmentCenterAligned(const MazeMap::VehicleState& state) const noexcept
    {
        return IsWallSegmentCenterWindowValid(
            state,
            MazeMap::Config::kSideWallSegmentCenterFraction);
    }

    float WallSensor::GetNoHitRangeM() const noexcept
    {
        return _noHitRangeM;
    }

    std::uint32_t WallSensor::GetAmbientSettleTimeUs() const noexcept
    {
        return _ambientSettleTimeUs;
    }

    std::uint32_t WallSensor::GetLitSettleTimeUs() const noexcept
    {
        return _litSettleTimeUs;
    }

    void WallSensor::CommandLedOff(const std::uint32_t commandUs) noexcept
    {
        digitalWrite(_ledOutPin, LOW);
        _ledEnabled = false;
        _latestLedOffCommandUs = commandUs;
        _ambientReadyUs = commandUs + _ambientSettleTimeUs;
        _latestTiming.ledOffCommandUs = commandUs;
        _captureArmed = false;
        _ambientCaptured = false;
        _litCaptured = false;
        _captureComplete = false;
    }

    void WallSensor::CommandLedOff() noexcept
    {
        CommandLedOff(micros());
    }

    void WallSensor::CommandLedOn(const std::uint32_t commandUs) noexcept
    {
        digitalWrite(_ledOutPin, HIGH);
        _ledEnabled = true;
        _latestLedOnCommandUs = commandUs;
        _litReadyUs = commandUs + _litSettleTimeUs;
        _latestTiming.ledOnCommandUs = commandUs;
        _captureArmed = true;
        _litCaptured = false;
        _captureComplete = false;
    }

    void WallSensor::CommandLedOn() noexcept
    {
        CommandLedOn(micros());
    }

    bool WallSensor::IsLedEnabled() const noexcept
    {
        return _ledEnabled;
    }

    std::uint32_t WallSensor::LatestLedOffCommandUs() const noexcept
    {
        return _latestLedOffCommandUs;
    }

    std::uint32_t WallSensor::LatestLedOnCommandUs() const noexcept
    {
        return _latestLedOnCommandUs;
    }

    std::uint32_t WallSensor::AmbientReadyUs() const noexcept
    {
        return _ambientReadyUs;
    }

    std::uint32_t WallSensor::LitReadyUs() const noexcept
    {
        return _litReadyUs;
    }

    bool WallSensor::IsAmbientReadReady(const std::uint32_t nowUs) const noexcept
    {
        return TimeReached(nowUs, _ambientReadyUs);
    }

    bool WallSensor::IsLitReadReady(const std::uint32_t nowUs) const noexcept
    {
        return TimeReached(nowUs, _litReadyUs);
    }

    void WallSensor::AwaitAmbientReady() const noexcept
    {
        while (!IsAmbientReadReady(micros()))
        {
            const std::int32_t remainingUs = static_cast<std::int32_t>(_ambientReadyUs - micros());
            if (remainingUs > 0)
            {
                delayMicroseconds(static_cast<unsigned int>(remainingUs));
            }
        }
    }

    void WallSensor::AwaitLitReady() const noexcept
    {
        while (!IsLitReadReady(micros()))
        {
            const std::int32_t remainingUs = static_cast<std::int32_t>(_litReadyUs - micros());
            if (remainingUs > 0)
            {
                delayMicroseconds(static_cast<unsigned int>(remainingUs));
            }
        }
    }

    bool WallSensor::HasAmbientRead() const noexcept
    {
        return _ambientCaptured;
    }

    bool WallSensor::HasLitRead() const noexcept
    {
        return _litCaptured;
    }

    bool WallSensor::IsCaptureArmed() const noexcept
    {
        return _captureArmed;
    }

    bool WallSensor::HasCompletedCapture() const noexcept
    {
        return _captureComplete;
    }

    void WallSensor::CaptureAmbientRead() noexcept
    {
        _latestAmbientAdcCode = ReadAdcCode();
        _latestTiming.adcOffSampleUs = micros();
        _latestAmbientLight = AdcCodeToLightLevel(_latestAmbientAdcCode);
        _ambientCaptured = true;
        _litCaptured = false;
        _captureArmed = true;
        _captureComplete = false;
    }

    void WallSensor::CaptureLitRead() noexcept
    {
        _latestLitAdcCode = ReadAdcCode();
        _latestTiming.adcOnSampleUs = micros();
        _latestLitLight = AdcCodeToLightLevel(_latestLitAdcCode);
        _latestDifferentialLight = DifferentialLightLevel(_latestAmbientLight, _latestLitLight);
        _latestRawDistanceM = DistanceFromDifferentialLight(_latestDifferentialLight);
        _litCaptured = true;
    }

    bool WallSensor::TryCompleteLitRead(const std::uint32_t nowUs) noexcept
    {
        if (!_captureArmed)
        {
            return true;
        }

        if (!IsLitReadReady(nowUs))
        {
            return false;
        }

        CaptureLitRead();
        return true;
    }

    void WallSensor::CompleteCapture(const std::uint32_t ledOffCommandUs) noexcept
    {
        digitalWrite(_ledOutPin, LOW);
        _ledEnabled = false;
        _latestLedOffCommandUs = ledOffCommandUs;
        _ambientReadyUs = ledOffCommandUs + _ambientSettleTimeUs;
        _latestTiming.ledOffCommandUs = ledOffCommandUs;
        _latestTiming.observationReadyUs = ledOffCommandUs;
        _captureArmed = false;
        _captureComplete = _ambientCaptured && _litCaptured;
    }

    void WallSensor::CaptureBlocking() noexcept
    {
        CommandLedOff();
        AwaitAmbientReady();
        CaptureAmbientRead();
        CommandLedOn();
        AwaitLitReady();
        CaptureLitRead();
        CompleteCapture(micros());
    }

    std::uint16_t WallSensor::LatestAmbientAdcCode() const noexcept
    {
        return _latestAmbientAdcCode;
    }

    std::uint16_t WallSensor::LatestLitAdcCode() const noexcept
    {
        return _latestLitAdcCode;
    }

    float WallSensor::LatestAmbientLight() const noexcept
    {
        return _latestAmbientLight;
    }

    float WallSensor::LatestLitLight() const noexcept
    {
        return _latestLitLight;
    }

    float WallSensor::LatestDifferentialLight() const noexcept
    {
        return _latestDifferentialLight;
    }

    float WallSensor::LatestRawDistanceM() const noexcept
    {
        return _latestRawDistanceM;
    }

    const OpticalObservationTiming& WallSensor::LatestTiming() const noexcept
    {
        return _latestTiming;
    }

    std::uint16_t WallSensor::ReadAdcCode() const noexcept
    {
        if (_hasDedicatedWallSensorAdc1Channel)
        {
            return MazeMap::Platform::ReadWallSensorAdcCodeFromConfiguredChannel(_wallSensorAdc1Channel);
        }

        return MazeMap::Platform::ReadWallSensorAdcCode(_wallSensorInPin);
    }

    float WallSensor::AdcCodeToLightLevel(const std::uint16_t adcReading) const noexcept
    {
        return AdcToLightLevelInternal(adcReading);
    }

    float WallSensor::ReadLightLevel() const noexcept
    {
        return AdcCodeToLightLevel(ReadAdcCode());
    }

    float WallSensor::DifferentialLightLevel(const float ambientLightLevel, const float litLightLevel) noexcept
    {
        const float deltaLightLevel = litLightLevel - ambientLightLevel;
        return (deltaLightLevel > 0.0f) ? deltaLightLevel : 0.0f;
    }

    float WallSensor::DistanceFromDifferentialLight(float deltaLightLevel) const noexcept
    {
        if (deltaLightLevel < 0.0f)
        {
            deltaLightLevel = 0.0f;
        }

        return DeltaLightToDistance(deltaLightLevel);
    }

    float WallSensor::DistanceFromLightLevels(const float ambientLightLevel, const float litLightLevel) const noexcept
    {
        return DistanceFromDifferentialLight(DifferentialLightLevel(ambientLightLevel, litLightLevel));
    }

    float WallSensor::ReadDistance(const float darkLightLevel) const noexcept
    {
        return DistanceFromLightLevels(darkLightLevel, ReadLightLevel());
    }

    Eigen::Vector2f WallSensor::Normalize(const Eigen::Vector2f& v) noexcept
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

    float WallSensor::Lerp(const float a, const float b, const float t) noexcept
    {
        return a + ((b - a) * t);
    }

    bool WallSensor::TimeReached(const std::uint32_t nowUs, const std::uint32_t readyUs) noexcept
    {
        return static_cast<std::int32_t>(nowUs - readyUs) >= 0;
    }

    float WallSensor::AdcToLightLevelInternal(const std::uint16_t adcReading) const noexcept
    {
        if (adcReading >= kAdcMax)
        {
            return _adcToLightTable[7];
        }

        const float scaledIndex =
            (static_cast<float>(adcReading) * kTableSpan) /
            static_cast<float>(kAdcMax);
        const std::size_t index = static_cast<std::size_t>(scaledIndex);
        if (index >= 7U)
        {
            return _adcToLightTable[7];
        }

        const float frac = scaledIndex - static_cast<float>(index);
        return Lerp(_adcToLightTable[index], _adcToLightTable[index + 1U], frac);
    }

    float WallSensor::DeltaLightToDistance(const float deltaLightLevel) const noexcept
    {
        return _distanceModel.DistanceFromDifferentialLight(deltaLightLevel);
    }
}
