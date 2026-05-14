#include "pch.h"
#include "PDCluster.h"

namespace
{
    constexpr MazeMap::PDCluster kConstexprCluster(
        MazeMap::ProportionalDerivative(1.0f, 0.1f),
        MazeMap::ProportionalDerivative(2.0f, 0.2f),
        MazeMap::ProportionalDerivative(3.0f, 0.3f),
        MazeMap::ProportionalDerivative(4.0f, 0.4f),
        MazeMap::ProportionalDerivative(5.0f, 0.5f),
        MazeMap::ProportionalDerivative(6.0f, 0.6f),
        MazeMap::ProportionalDerivative(7.0f, 0.7f),
        MazeMap::ProportionalDerivative(8.0f, 0.8f),
        MazeMap::ProportionalDerivative(9.0f, 0.9f));

    static_assert(
        kConstexprCluster.HeadingStatePD.GetProportionalGain() == 1.0f,
        "PDCluster must remain constexpr-constructible.");
    static_assert(
        kConstexprCluster.YawRateEncoderDeltaPD.GetDerivativeGain() == 0.6f,
        "PDCluster member access must remain constexpr-safe.");

}

void MazeMap::PDCluster::ResetDerivativeHistories() noexcept
{
    HeadingStatePD.ResetDerivativeHistory();
    VelocityStatePD.ResetDerivativeHistory();
    VelocityEncoderAveragePD.ResetDerivativeHistory();
    YawRateStatePD.ResetDerivativeHistory();
    YawRateGyroPD.ResetDerivativeHistory();
    YawRateEncoderDeltaPD.ResetDerivativeHistory();
    YawRateIMULateralAccelPD.ResetDerivativeHistory();
    LongitudinalAccelerationStatePD.ResetDerivativeHistory();
    LongitudinalAccelerationIMUForwardAccelPD.ResetDerivativeHistory();
}
