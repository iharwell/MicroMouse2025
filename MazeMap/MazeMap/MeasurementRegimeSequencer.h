#pragma once

#include "CommandVector.h"
#include "Defines.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace MazeMap
{
    enum ManeuverCode : std::uint8_t;
}

namespace MazeMap::App::Internal
{
    class EXPORT MeasurementRegimeSequencer final
    {
    public:
        using RegimeId = std::uint8_t;

        class EXPORT Regime
        {
        public:
            Regime() noexcept;
            virtual ~Regime();

            Regime(const Regime&) = delete;
            Regime& operator=(const Regime&) = delete;
            Regime(Regime&&) = delete;
            Regime& operator=(Regime&&) = delete;

            virtual RegimeId Id() const noexcept = 0;
            virtual const char* Name() const noexcept = 0;
            virtual std::uint16_t PrimitiveCount() const noexcept = 0;
            virtual std::uint8_t SpeedBinCount() const noexcept = 0;
            virtual std::uint16_t RepeatCount() const noexcept = 0;
            virtual MazeMap::ManeuverCode PrimitiveCode(
                std::uint16_t primitiveIndex,
                std::uint8_t speedBinIndex) const noexcept = 0;
            virtual float SpeedBinValue(
                std::uint16_t primitiveIndex,
                std::uint8_t speedBinIndex) const noexcept = 0;
            virtual CommandVector GetNextControls(
                std::uint16_t primitiveIndex,
                std::uint8_t speedBinIndex,
                std::uint16_t repeatIndex,
                bool& done) = 0;
        };

        MeasurementRegimeSequencer() noexcept;
        ~MeasurementRegimeSequencer();

        MeasurementRegimeSequencer(const MeasurementRegimeSequencer&) = delete;
        MeasurementRegimeSequencer& operator=(const MeasurementRegimeSequencer&) = delete;
        MeasurementRegimeSequencer(MeasurementRegimeSequencer&&) = delete;
        MeasurementRegimeSequencer& operator=(MeasurementRegimeSequencer&&) = delete;

        template <std::size_t RegimeCount>
        void Start(const std::array<Regime*, RegimeCount>& regimes) noexcept;

        CommandVector GetNextControls(bool& done);

        RegimeId CurrentRegimeId() const noexcept;
        const char* CurrentRegimeName() const noexcept;
        MazeMap::ManeuverCode CurrentPrimitiveCode() const noexcept;
        float CurrentSpeedBinValue() const noexcept;
        std::uint16_t CurrentRepeatOrdinal() const noexcept;

    private:
        Regime* const* _regimes{};
        std::size_t _regimeCount{};
        std::size_t _regimeIndex{};
        std::uint16_t _primitiveIndex{};
        std::uint8_t _speedBinIndex{};
        std::uint16_t _repeatIndex{};
    };

    template <std::size_t RegimeCount>
    void MeasurementRegimeSequencer::Start(
        const std::array<Regime*, RegimeCount>& regimes) noexcept
    {
        _regimes = regimes.data();
        _regimeCount = RegimeCount;
        _regimeIndex = 0U;
        _primitiveIndex = 0U;
        _speedBinIndex = 0U;
        _repeatIndex = 0U;
    }
}
