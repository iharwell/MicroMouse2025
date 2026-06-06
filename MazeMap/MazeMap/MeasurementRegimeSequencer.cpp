#include "pch.h"
#include "MeasurementRegimeSequencer.h"

namespace MazeMap::App::Internal
{
    MeasurementRegimeSequencer::Regime::Regime() noexcept = default;

    MeasurementRegimeSequencer::Regime::~Regime() = default;

    MeasurementRegimeSequencer::MeasurementRegimeSequencer() noexcept = default;

    MeasurementRegimeSequencer::~MeasurementRegimeSequencer() = default;

    CommandVector MeasurementRegimeSequencer::GetNextControls(bool& done)
    {
        if (_regimeIndex >= _regimeCount)
        {
            done = true;
            return CommandVector{};
        }

        Regime& regime = *_regimes[_regimeIndex];
        bool slotDone = false;
        const CommandVector command = regime.GetNextControls(
            _primitiveIndex,
            _speedBinIndex,
            _repeatIndex,
            slotDone);

        if (!slotDone)
        {
            done = false;
            return command;
        }

        ++_primitiveIndex;
        if (_primitiveIndex >= regime.PrimitiveCount())
        {
            _primitiveIndex = 0U;
            ++_repeatIndex;
            if (_repeatIndex >= regime.RepeatCount())
            {
                _repeatIndex = 0U;
                ++_speedBinIndex;
                if (_speedBinIndex >= regime.SpeedBinCount())
                {
                    _speedBinIndex = 0U;
                    ++_regimeIndex;
                }
            }
        }

        done = _regimeIndex >= _regimeCount;
        return command;
    }

    MeasurementRegimeSequencer::RegimeId
        MeasurementRegimeSequencer::CurrentRegimeId() const noexcept
    {
        return _regimes[_regimeIndex]->Id();
    }

    const char* MeasurementRegimeSequencer::CurrentRegimeName() const noexcept
    {
        return _regimes[_regimeIndex]->Name();
    }

    MazeMap::ManeuverCode MeasurementRegimeSequencer::CurrentPrimitiveCode() const noexcept
    {
        return _regimes[_regimeIndex]->PrimitiveCode(_primitiveIndex, _speedBinIndex);
    }

    float MeasurementRegimeSequencer::CurrentSpeedBinValue() const noexcept
    {
        return _regimes[_regimeIndex]->SpeedBinValue(_primitiveIndex, _speedBinIndex);
    }

    std::uint16_t MeasurementRegimeSequencer::CurrentRepeatOrdinal() const noexcept
    {
        return static_cast<std::uint16_t>(_repeatIndex + 1U);
    }
}
