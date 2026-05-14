#pragma once

#include "Defines.h"

#include <cstdint>
#include <limits>

namespace MazeMap
{
    class PDCluster;
    class ProportionalDerivative;
    class VehicleState;

    enum class FeedbackSource : std::uint8_t
    {
        None = 0U,
        State = 1U << 0,
        Imu = 1U << 1,
        Encoder = 1U << 2
    };

    inline constexpr FeedbackSource operator|(FeedbackSource lhs, FeedbackSource rhs) noexcept
    {
        return static_cast<FeedbackSource>(
            static_cast<std::uint8_t>(lhs) |
            static_cast<std::uint8_t>(rhs));
    }

    inline constexpr FeedbackSource operator&(FeedbackSource lhs, FeedbackSource rhs) noexcept
    {
        return static_cast<FeedbackSource>(
            static_cast<std::uint8_t>(lhs) &
            static_cast<std::uint8_t>(rhs));
    }

    inline constexpr FeedbackSource& operator|=(FeedbackSource& lhs, FeedbackSource rhs) noexcept
    {
        lhs = lhs | rhs;
        return lhs;
    }

    class EXPORT FeedbackAxis final
    {
    public:
        FeedbackAxis(const VehicleState* state, bool rotational) noexcept;

        float GetFeedback(
            std::uint8_t derivativeOrder,
            float target,
            FeedbackSource sources,
            const PDCluster& proportionalDerivativeCluster) const noexcept;

    private:
        float GetSourceFeedback(
            std::uint8_t derivativeOrder,
            FeedbackSource source,
            float target,
            const PDCluster& proportionalDerivativeCluster) const noexcept;

        float Observe(
            std::uint8_t derivativeOrder,
            FeedbackSource source) const noexcept;

        float ObserveDerivative(
            std::uint8_t derivativeOrder,
            FeedbackSource source,
            float observed) const noexcept;

        float SampleObservedDerivative(
            std::uint8_t derivativeOrder,
            FeedbackSource source,
            float observed,
            float firstSampleDerivative) const noexcept;

        const ProportionalDerivative& ProportionalDerivativeFor(
            std::uint8_t derivativeOrder,
            FeedbackSource source,
            const PDCluster& proportionalDerivativeCluster) const noexcept;

        const VehicleState* _state;
        bool _rotational;
        mutable float _previousObserved[3][3];
        mutable float _previousObservedTimeS[3][3];
        mutable float _previousObservedDerivative[3][3];
    };
}
