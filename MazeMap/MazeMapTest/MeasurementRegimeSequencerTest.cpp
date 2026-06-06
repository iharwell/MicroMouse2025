#include "pch.h"
#include "CppUnitTest.h"

#include "..\MazeMap\MeasurementRegimeSequencer.h"
#include "..\MazeMap\Maneuver.h"

#include <array>
#include <cstdint>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    class TestMeasurementRegime final :
        public App::Internal::MeasurementRegimeSequencer::Regime
    {
    public:
        using RegimeId = App::Internal::MeasurementRegimeSequencer::RegimeId;

        TestMeasurementRegime(
            RegimeId id,
            const char* name,
            std::uint16_t primitiveCount,
            std::uint8_t speedBinCount,
            std::uint16_t repeatCount) noexcept
            : _id(id)
            , _name(name)
            , _primitiveCount(primitiveCount)
            , _speedBinCount(speedBinCount)
            , _repeatCount(repeatCount)
        {
        }

        RegimeId Id() const noexcept override { return _id; }
        const char* Name() const noexcept override { return _name; }
        std::uint16_t PrimitiveCount() const noexcept override { return _primitiveCount; }
        std::uint8_t SpeedBinCount() const noexcept override { return _speedBinCount; }
        std::uint16_t RepeatCount() const noexcept override { return _repeatCount; }

        ManeuverCode PrimitiveCode(
            const std::uint16_t primitiveIndex,
            const std::uint8_t speedBinIndex) const noexcept override
        {
            return static_cast<ManeuverCode>(
                static_cast<int>(S1) +
                static_cast<int>(primitiveIndex) +
                static_cast<int>(speedBinIndex));
        }

        float SpeedBinValue(
            const std::uint16_t primitiveIndex,
            const std::uint8_t speedBinIndex) const noexcept override
        {
            return static_cast<float>(primitiveIndex) + (0.1f * static_cast<float>(speedBinIndex));
        }

        App::Internal::CommandVector GetNextControls(
            const std::uint16_t primitiveIndex,
            const std::uint8_t speedBinIndex,
            const std::uint16_t repeatIndex,
            bool& done) override
        {
            done = _completeOnTick;
            ++_tickCount;
            _lastRepeatIndex = repeatIndex;
            return App::Internal::CommandVector(
                static_cast<float>((primitiveIndex * 100U) + (speedBinIndex * 10U) + repeatIndex),
                static_cast<float>(_id));
        }

        void SetCompleteOnTick(bool completeOnTick) noexcept { _completeOnTick = completeOnTick; }
        std::uint16_t TickCount() const noexcept { return _tickCount; }
        std::uint16_t LastRepeatIndex() const noexcept { return _lastRepeatIndex; }

    private:
        RegimeId _id{};
        const char* _name{};
        std::uint16_t _primitiveCount{};
        std::uint8_t _speedBinCount{};
        std::uint16_t _repeatCount{};
        std::uint16_t _tickCount{};
        std::uint16_t _lastRepeatIndex{};
        bool _completeOnTick{ true };
    };

    TEST_CLASS(MeasurementRegimeSequencerTest)
    {
    public:
        TEST_METHOD(AdvancesPrimitiveRepeatSpeedThenRegimeWhenSlotsComplete)
        {
            TestMeasurementRegime first(7U, "first", 2U, 2U, 2U);
            TestMeasurementRegime second(8U, "second", 1U, 1U, 1U);
            const std::array<App::Internal::MeasurementRegimeSequencer::Regime*, 2U> regimes = {
                &first,
                &second,
            };
            App::Internal::MeasurementRegimeSequencer sequencer;
            sequencer.Start(regimes);

            bool done = true;
            for (std::uint8_t speedBinIndex = 0U; speedBinIndex < 2U; ++speedBinIndex)
            {
                for (std::uint16_t repeatIndex = 0U; repeatIndex < 2U; ++repeatIndex)
                {
                    for (std::uint16_t primitiveIndex = 0U; primitiveIndex < 2U; ++primitiveIndex)
                    {
                        Assert::AreEqual(static_cast<std::uint8_t>(7U), sequencer.CurrentRegimeId());
                        Assert::AreEqual("first", sequencer.CurrentRegimeName());
                        Assert::AreEqual(
                            static_cast<int>(first.PrimitiveCode(primitiveIndex, speedBinIndex)),
                            static_cast<int>(sequencer.CurrentPrimitiveCode()));
                        Assert::AreEqual(
                            first.SpeedBinValue(primitiveIndex, speedBinIndex),
                            sequencer.CurrentSpeedBinValue(),
                            1.0e-6f);
                        Assert::AreEqual(
                            static_cast<std::uint16_t>(repeatIndex + 1U),
                            sequencer.CurrentRepeatOrdinal());

                        const App::Internal::CommandVector command = sequencer.GetNextControls(done);

                        Assert::AreEqual(
                            static_cast<float>((primitiveIndex * 100U) + (speedBinIndex * 10U) + repeatIndex),
                            command.LeftCommand(),
                            1.0e-6f);
                        Assert::AreEqual(7.0f, command.RightCommand(), 1.0e-6f);
                        Assert::IsFalse(done);
                    }
                }
            }

            Assert::AreEqual(static_cast<std::uint8_t>(8U), sequencer.CurrentRegimeId());
            Assert::AreEqual(static_cast<std::uint16_t>(1U), sequencer.CurrentRepeatOrdinal());

            const App::Internal::CommandVector command = sequencer.GetNextControls(done);

            Assert::AreEqual(0.0f, command.LeftCommand(), 1.0e-6f);
            Assert::AreEqual(8.0f, command.RightCommand(), 1.0e-6f);
            Assert::IsTrue(done);
            Assert::AreEqual(static_cast<std::uint16_t>(8U), first.TickCount());
            Assert::AreEqual(static_cast<std::uint16_t>(1U), second.TickCount());
        }

        TEST_METHOD(DoesNotAdvanceWhenCurrentSlotIsNotDone)
        {
            TestMeasurementRegime regime(3U, "single", 1U, 1U, 2U);
            const std::array<App::Internal::MeasurementRegimeSequencer::Regime*, 1U> regimes = { &regime };
            App::Internal::MeasurementRegimeSequencer sequencer;
            sequencer.Start(regimes);

            regime.SetCompleteOnTick(false);
            bool done = true;
            (void)sequencer.GetNextControls(done);

            Assert::IsFalse(done);
            Assert::AreEqual(static_cast<std::uint16_t>(1U), sequencer.CurrentRepeatOrdinal());
            Assert::AreEqual(static_cast<std::uint16_t>(0U), regime.LastRepeatIndex());

            regime.SetCompleteOnTick(true);
            (void)sequencer.GetNextControls(done);

            Assert::IsFalse(done);
            Assert::AreEqual(static_cast<std::uint16_t>(2U), sequencer.CurrentRepeatOrdinal());
            Assert::AreEqual(static_cast<std::uint16_t>(0U), regime.LastRepeatIndex());
        }
    };
}
