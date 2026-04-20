#include "pch.h"
#include "ProportionalDerivativeCluster.h"

namespace
{
    const MazeMap::ProportionalDerivative kZeroProportionalDerivative{};

    constexpr MazeMap::ProportionalDerivativeCluster kConstexprCluster(
        MazeMap::ProportionalDerivative(1.0f, 0.1f),
        MazeMap::ProportionalDerivative(2.0f, 0.2f),
        MazeMap::ProportionalDerivative(3.0f, 0.3f),
        MazeMap::ProportionalDerivative(4.0f, 0.4f),
        MazeMap::ProportionalDerivative(5.0f, 0.5f),
        MazeMap::ProportionalDerivative(6.0f, 0.6f),
        MazeMap::ProportionalDerivative(7.0f, 0.7f),
        MazeMap::ProportionalDerivative(8.0f, 0.8f),
        MazeMap::ProportionalDerivative(9.0f, 0.9f),
        MazeMap::ProportionalDerivative(10.0f, 1.0f),
        MazeMap::ProportionalDerivative(11.0f, 1.1f),
        MazeMap::ProportionalDerivative(12.0f, 1.2f),
        MazeMap::ProportionalDerivative(13.0f, 1.3f),
        MazeMap::ProportionalDerivative(14.0f, 1.4f),
        MazeMap::ProportionalDerivative(15.0f, 1.5f),
        MazeMap::ProportionalDerivative(16.0f, 1.6f));

    static_assert(
        kConstexprCluster.HeadingStatePD.GetProportionalGain() == 1.0f,
        "ProportionalDerivativeCluster must remain constexpr-constructible.");
    static_assert(
        kConstexprCluster.YawAccelerationEncoderDeltaPD.GetDerivativeGain() == 1.6f,
        "ProportionalDerivativeCluster member access must remain constexpr-safe.");

    unsigned CountSelectedSources(
        const bool sourceA,
        const bool sourceB = false,
        const bool sourceC = false,
        const bool sourceD = false) noexcept
    {
        return
            static_cast<unsigned>(sourceA) +
            static_cast<unsigned>(sourceB) +
            static_cast<unsigned>(sourceC) +
            static_cast<unsigned>(sourceD);
    }
}

void MazeMap::ProportionalDerivativeCluster::ResetDerivativeHistories() noexcept
{
    HeadingStatePD.ResetDerivativeHistory();
    HeadingGyroPD.ResetDerivativeHistory();
    HeadingEncoderDeltaPD.ResetDerivativeHistory();
    VelocityStatePD.ResetDerivativeHistory();
    VelocityEncoderAveragePD.ResetDerivativeHistory();
    YawRateStatePD.ResetDerivativeHistory();
    YawRateGyroPD.ResetDerivativeHistory();
    YawRateEncoderDeltaPD.ResetDerivativeHistory();
    YawRateIMULateralAccelPD.ResetDerivativeHistory();
    LongitudinalAccelerationStatePD.ResetDerivativeHistory();
    LongitudinalAccelerationIMUForwardAccelPD.ResetDerivativeHistory();
    WheelVelocityStatePD.ResetDerivativeHistory();
    WheelVelocityEncoderPD.ResetDerivativeHistory();
    YawAccelerationStatePD.ResetDerivativeHistory();
    YawAccelerationGyroPD.ResetDerivativeHistory();
    YawAccelerationEncoderDeltaPD.ResetDerivativeHistory();
}

const MazeMap::ProportionalDerivative& MazeMap::ProportionalDerivativeCluster::GetHeadingPD(
    const CommandPD pd) const noexcept
{
    const bool useState = MazeMap::HasCommandPD(pd, MazeMap::CommandPD::StateHeadingPD);
    const bool useGyro = MazeMap::HasCommandPD(pd, MazeMap::CommandPD::IMUYaw);
    const bool useEncoder = MazeMap::HasCommandPD(pd, MazeMap::CommandPD::EncoderVelocity);
    if (CountSelectedSources(useState, useGyro, useEncoder) != 1U)
    {
        return kZeroProportionalDerivative;
    }

    if (useState)
    {
        return HeadingStatePD;
    }

    if (useGyro)
    {
        return HeadingGyroPD;
    }

    return HeadingEncoderDeltaPD;
}

const MazeMap::ProportionalDerivative& MazeMap::ProportionalDerivativeCluster::GetVelocityPD(
    const CommandPD pd) const noexcept
{
    const bool useState = MazeMap::HasCommandPD(pd, MazeMap::CommandPD::StateVelocityPD);
    const bool useEncoder = MazeMap::HasCommandPD(pd, MazeMap::CommandPD::EncoderVelocity);
    if (CountSelectedSources(useState, useEncoder) != 1U)
    {
        return kZeroProportionalDerivative;
    }

    return useState ? VelocityStatePD : VelocityEncoderAveragePD;
}

const MazeMap::ProportionalDerivative& MazeMap::ProportionalDerivativeCluster::GetYawRatePD(
    const CommandPD pd) const noexcept
{
    const bool useState = MazeMap::HasCommandPD(pd, MazeMap::CommandPD::StateYawPD);
    const bool useGyro = MazeMap::HasCommandPD(pd, MazeMap::CommandPD::IMUYaw);
    const bool useEncoder = MazeMap::HasCommandPD(pd, MazeMap::CommandPD::EncoderVelocity);
    const bool useLateralAccel = MazeMap::HasCommandPD(pd, MazeMap::CommandPD::IMULateralAccel);
    if (CountSelectedSources(useState, useGyro, useEncoder, useLateralAccel) != 1U)
    {
        return kZeroProportionalDerivative;
    }

    if (useState)
    {
        return YawRateStatePD;
    }

    if (useGyro)
    {
        return YawRateGyroPD;
    }

    if (useEncoder)
    {
        return YawRateEncoderDeltaPD;
    }

    return YawRateIMULateralAccelPD;
}

const MazeMap::ProportionalDerivative& MazeMap::ProportionalDerivativeCluster::GetLongitudinalAccelerationPD(
    const CommandPD pd) const noexcept
{
    const bool useState = MazeMap::HasCommandPD(pd, MazeMap::CommandPD::StateAccelerationPD);
    const bool useImu = MazeMap::HasCommandPD(pd, MazeMap::CommandPD::IMUForwardAccel);
    if (CountSelectedSources(useState, useImu) != 1U)
    {
        return kZeroProportionalDerivative;
    }

    return useState ? LongitudinalAccelerationStatePD : LongitudinalAccelerationIMUForwardAccelPD;
}

const MazeMap::ProportionalDerivative& MazeMap::ProportionalDerivativeCluster::GetWheelVelocityPD(
    const CommandPD pd) const noexcept
{
    const bool useState = MazeMap::HasCommandPD(pd, MazeMap::CommandPD::StateWheelOmegaPD);
    const bool useEncoder = MazeMap::HasCommandPD(pd, MazeMap::CommandPD::EncoderVelocity);
    if (CountSelectedSources(useState, useEncoder) != 1U)
    {
        return kZeroProportionalDerivative;
    }

    return useState ? WheelVelocityStatePD : WheelVelocityEncoderPD;
}

const MazeMap::ProportionalDerivative& MazeMap::ProportionalDerivativeCluster::GetYawAccelerationPD(
    const CommandPD pd) const noexcept
{
    const bool useState = MazeMap::HasCommandPD(pd, MazeMap::CommandPD::StateYawPD);
    const bool useGyro = MazeMap::HasCommandPD(pd, MazeMap::CommandPD::IMUYaw);
    const bool useEncoder = MazeMap::HasCommandPD(pd, MazeMap::CommandPD::EncoderVelocity);
    if (CountSelectedSources(useState, useGyro, useEncoder) != 1U)
    {
        return kZeroProportionalDerivative;
    }

    if (useState)
    {
        return YawAccelerationStatePD;
    }

    if (useGyro)
    {
        return YawAccelerationGyroPD;
    }

    return YawAccelerationEncoderDeltaPD;
}
