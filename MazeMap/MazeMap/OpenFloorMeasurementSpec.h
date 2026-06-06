#pragma once

#include "MazeMapRuntimeCore.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace MazeMap
{
    inline constexpr const char* kOpenFloorSelectedRoutineName = "open_floor";
    inline constexpr const char* kOpenFloorFormatVersion = "ofm:f";
    inline constexpr const char* kOpenFloorRevisionBundle = "log:h;sched:h;bins:d;marks:d";
    inline constexpr const char* kOpenFloorImuSetup = "imu:bl;extr:v1";
    inline constexpr const char* kOpenFloorTimingStreamType = "timing";
    inline constexpr const char* kOpenFloorMainStreamType = "main";
    inline constexpr const char* kOpenFloorBootReason = "pins27_28";
    inline constexpr const char* kOpenFloorLogFormatSpec = "mlog:g";
    inline constexpr const char* kOpenFloorEndianness = "le";
    inline constexpr const char* kOpenFloorTimingFileName = "open_floor_timing.mmlog";
    inline constexpr const char* kOpenFloorMainFileName = "open_floor_main.mmlog";

    enum class OpenFloorSectionId : std::uint8_t
    {
        Sec00Timing = 0U,
        Sec10Static,
        Sec20Launch,
        Sec30Straight,
        Sec40Yaw,
        Sec50Smooth,
        Sec60LoopCw,
        Sec70LoopCcw,
    };

    enum class OpenFloorMarkerId : std::uint8_t
    {
        C = 0U,
        N,
        S,
        CW,
        CCW,
    };

    enum class OpenFloorPrimitiveFamily : std::uint8_t
    {
        None = 0U,
        Timing,
        StaticHold,
        Launch,
        Straight,
        InPlaceTurn,
        SmoothTurn,
        Recovery,
    };

    enum class OpenFloorFaultCode : std::uint8_t
    {
        None = 0U,
        HardwareSetupFailed,
        DriveInitFailed,
        SensorInitFailed,
        TimingLogOpenFailed,
        TimingLogWriteFailed,
        MainLogOpenFailed,
        MainLogWriteFailed,
        EstimatorFault,
        SelectorJumperRemoved,
        RecoveryTimedOut,
        LaunchBoundExceeded,
        StraightWatchdogStall,
        StraightSectionTimedOut,
        YawSectionTimedOut,
        YawProfileInvalid,
        SmoothGeometryUnavailable,
        SmoothWatchdogStall,
        SmoothSectionTimedOut,
        SmoothTargetInvalid,
        LoggerOverflow,
        LoggerWriteFailure,
    };

    inline constexpr std::uint8_t kOpenFloorSpeedBinLogIdNone = 0U;
    inline constexpr std::uint8_t kOpenFloorSpeedBinLogIdLow = 1U;
    inline constexpr std::uint8_t kOpenFloorSpeedBinLogIdMedium = 2U;
    inline constexpr std::uint8_t kOpenFloorSpeedBinLogIdHigh = 3U;

    inline constexpr std::array<float, 3U> kOpenFloorStraightSpeedBinsMps = { { 0.1f, 0.30f, 0.55f } };
    inline constexpr std::array<float, 3U> kOpenFloorYawRateBinsRadps = { { 9.0f, 18.0f, 27.0f } };
    inline constexpr std::array<ManeuverCode, 4U> kOpenFloorYawManeuverCodes = { {
        IP90,
        IP90_M,
        IP180,
        IP180_M,
    } };
    inline constexpr std::array<float, 4U> kOpenFloorYawNominalAnglesRad = { {
        HALF_PI_F,
        -HALF_PI_F,
        PI_F,
        -PI_F,
    } };
    inline constexpr std::array<float, 3U> kOpenFloorSmoothSpeedBinsMps = { { 0.4f, 0.45f, 0.45f } };
    inline constexpr std::uint8_t kOpenFloorLaunchRepeatsPerMagnitude = 3U;
    inline constexpr std::uint8_t kOpenFloorStraightRepeatsPerSpeed = 3U;
    inline constexpr unsigned long kOpenFloorLaunchPulseMs = 250UL;
    // After each launch, straight, or yaw test segment, use Drive's hold primitive to accumulate
    // this much stationary time before starting the next segment or phase.
    inline constexpr unsigned long kOpenFloorPostSegmentHoldMs = 250UL;
    // Longer brake hold between major sections so the next battery starts from a fully settled state.
    inline constexpr unsigned long kOpenFloorInterPhaseHoldMs = 500UL;
    inline constexpr float kOpenFloorLaunchDriveMagnitudeStart = 0.25f;
    inline constexpr float kOpenFloorLaunchDriveMagnitudeEnd = 0.35f;
    inline constexpr float kOpenFloorLaunchDriveMagnitudeStep = 0.05f;
    inline constexpr unsigned long kOpenFloorSelectorRemovalFaultDelayMs = 500UL;
    inline constexpr float kOpenFloorRecoveryAcceptanceRadiusM = 0.015f;
    inline constexpr float kOpenFloorRecoveryArrivalHeadingToleranceRad = 1.0f * DEG_TO_RAD_F;

    inline constexpr std::size_t OpenFloorLaunchDriveMagnitudeCount() noexcept
    {
        return
            (kOpenFloorLaunchDriveMagnitudeStep > 0.0f) &&
            (kOpenFloorLaunchDriveMagnitudeEnd >= kOpenFloorLaunchDriveMagnitudeStart) ?
            static_cast<std::size_t>(
                ((kOpenFloorLaunchDriveMagnitudeEnd - kOpenFloorLaunchDriveMagnitudeStart) /
                    kOpenFloorLaunchDriveMagnitudeStep) +
                0.5f) +
            1U :
            0U;
    }

    template <std::size_t Count>
    inline constexpr std::array<float, Count> BuildOpenFloorLaunchDriveMagnitudes() noexcept
    {
        std::array<float, Count> magnitudes{};
        for (std::size_t index = 0U; index < Count; ++index)
        {
            magnitudes[index] =
                ((index + 1U) == Count) ?
                kOpenFloorLaunchDriveMagnitudeEnd :
                (kOpenFloorLaunchDriveMagnitudeStart +
                    (static_cast<float>(index) * kOpenFloorLaunchDriveMagnitudeStep));
        }
        return magnitudes;
    }

    inline constexpr std::size_t kOpenFloorLaunchDriveMagnitudeCount = OpenFloorLaunchDriveMagnitudeCount();
    static_assert(kOpenFloorLaunchDriveMagnitudeStep > 0.0f, "Open-floor launch magnitude step must be positive.");
    static_assert(
        kOpenFloorLaunchDriveMagnitudeEnd >= kOpenFloorLaunchDriveMagnitudeStart,
        "Open-floor launch magnitude range must be ordered.");
    static_assert(kOpenFloorLaunchDriveMagnitudeCount > 0U, "Open-floor launch magnitude range must yield at least one sample.");
    inline constexpr auto kOpenFloorLaunchDriveMagnitudes =
        BuildOpenFloorLaunchDriveMagnitudes<kOpenFloorLaunchDriveMagnitudeCount>();

    inline constexpr std::uint8_t OpenFloorSpeedBinLogIdForIndex(const std::size_t speedIndex) noexcept
    {
        return (speedIndex == 0U) ? kOpenFloorSpeedBinLogIdLow :
            (speedIndex == 1U) ? kOpenFloorSpeedBinLogIdMedium :
            kOpenFloorSpeedBinLogIdHigh;
    }

    inline std::uint8_t OpenFloorSpeedBinLogIdForMagnitudeMps(const float speedMps) noexcept
    {
        const float magnitudeMps = std::fabs(speedMps);
        if (!(std::isfinite(magnitudeMps) && (magnitudeMps >= kOpenFloorStraightSpeedBinsMps[0U])))
        {
            return kOpenFloorSpeedBinLogIdNone;
        }
        if (magnitudeMps < kOpenFloorStraightSpeedBinsMps[1U])
        {
            return kOpenFloorSpeedBinLogIdLow;
        }
        if (magnitudeMps < kOpenFloorStraightSpeedBinsMps[2U])
        {
            return kOpenFloorSpeedBinLogIdMedium;
        }
        return kOpenFloorSpeedBinLogIdHigh;
    }

    inline constexpr float OpenFloorHalfStepMeters() noexcept
    {
        return Maze::GetCellDimension() * 0.5f;
    }

    inline constexpr float OpenFloorStrEquivalentDistanceMeters(std::uint8_t halfSteps) noexcept
    {
        return Maze::GetCellDimension() * 0.5f * static_cast<float>(halfSteps);
    }

    inline bool HasOpenFloorSelectorRemovalFaultDelayElapsed(
        unsigned long inactiveSinceMs,
        unsigned long nowMs) noexcept
    {
        return static_cast<unsigned long>(nowMs - inactiveSinceMs) >= kOpenFloorSelectorRemovalFaultDelayMs;
    }

    inline bool OpenFloorRecoveryWithinAcceptanceRadius(
        float dxMeters,
        float dyMeters) noexcept
    {
        if (!(std::isfinite(dxMeters) && std::isfinite(dyMeters)))
        {
            return false;
        }
        return ((dxMeters * dxMeters) + (dyMeters * dyMeters)) <=
            (kOpenFloorRecoveryAcceptanceRadiusM * kOpenFloorRecoveryAcceptanceRadiusM);
    }

    inline float OpenFloorRecoveryDistanceOutsideAcceptanceZoneM(
        float dxMeters,
        float dyMeters) noexcept
    {
        if (!(std::isfinite(dxMeters) && std::isfinite(dyMeters)))
        {
            return 0.0f;
        }
        const float centerDistanceM = MazeMap::Math::Sqrtf((dxMeters * dxMeters) + (dyMeters * dyMeters));
        return (centerDistanceM > kOpenFloorRecoveryAcceptanceRadiusM) ?
            (centerDistanceM - kOpenFloorRecoveryAcceptanceRadiusM) :
            0.0f;
    }

    inline float OpenFloorRecoverySignedLateralMissToAcceptanceZoneM(
        const Eigen::Vector2f& travelHeading,
        float dxMeters,
        float dyMeters) noexcept
    {
        if (!(std::isfinite(dxMeters) && std::isfinite(dyMeters)))
        {
            return 0.0f;
        }

        const Eigen::Vector2f leftUnit(-travelHeading.y(), travelHeading.x());
        const float lateralErrorM = (dxMeters * leftUnit.x()) + (dyMeters * leftUnit.y());
        const float missAbsM = MazeMap::Math::Absf(lateralErrorM) - kOpenFloorRecoveryAcceptanceRadiusM;
        if (!(missAbsM > 0.0f) || !std::isfinite(missAbsM))
        {
            return 0.0f;
        }

        return (lateralErrorM >= 0.0f) ? missAbsM : -missAbsM;
    }

    inline float OpenFloorRecoverySignedLongitudinalDistanceToAcceptanceZoneM(
        const Eigen::Vector2f& travelHeading,
        float dxMeters,
        float dyMeters) noexcept
    {
        if (!(std::isfinite(dxMeters) && std::isfinite(dyMeters)))
        {
            return 0.0f;
        }

        const Eigen::Vector2f leftUnit(-travelHeading.y(), travelHeading.x());
        const float longitudinalErrorM = (dxMeters * travelHeading.x()) + (dyMeters * travelHeading.y());
        const float lateralErrorM = (dxMeters * leftUnit.x()) + (dyMeters * leftUnit.y());
        const float lateralAbsM = MazeMap::Math::Absf(lateralErrorM);
        if (!(std::isfinite(longitudinalErrorM) && std::isfinite(lateralAbsM)))
        {
            return 0.0f;
        }

        if (lateralAbsM >= kOpenFloorRecoveryAcceptanceRadiusM)
        {
            return longitudinalErrorM;
        }

        const float longitudinalInsideZoneM = MazeMap::Math::Sqrtf((std::max)(
            0.0f,
            (kOpenFloorRecoveryAcceptanceRadiusM * kOpenFloorRecoveryAcceptanceRadiusM) - (lateralErrorM * lateralErrorM)));
        const float outsideAbsM = MazeMap::Math::Absf(longitudinalErrorM) - longitudinalInsideZoneM;
        if (!(outsideAbsM > 0.0f) || !std::isfinite(outsideAbsM))
        {
            return 0.0f;
        }

        return (longitudinalErrorM >= 0.0f) ? outsideAbsM : -outsideAbsM;
    }

    inline OpenFloorPrimitiveFamily OpenFloorPrimitiveFamilyForManeuverCode(ManeuverCode code) noexcept
    {
        const ManeuverCode baseCode = static_cast<ManeuverCode>(code & INVERTED_MIRRORED_MANEUVER_FLAG);
        if (baseCode == MC_NONE)
        {
            return OpenFloorPrimitiveFamily::None;
        }
        if (IsStraightCode(baseCode))
        {
            return OpenFloorPrimitiveFamily::Straight;
        }

        switch (baseCode)
        {
        case IP45:
        case IP90:
        case IP135:
        case IP180:
            return OpenFloorPrimitiveFamily::InPlaceTurn;
        case S45SS:
        case S45SD:
        case S45LS:
        case S45LD:
        case S90SS:
        case S90SD:
        case S90LS:
        case S90LD:
        case S135SS:
        case S135SD:
        case S135LS:
        case S135LD:
        case S180SS:
        case S180LS:
        case S90ELD:
        case S180ELS:
            return OpenFloorPrimitiveFamily::SmoothTurn;
        default:
            return OpenFloorPrimitiveFamily::None;
        }
    }

    inline const char* OpenFloorFaultName(OpenFloorFaultCode faultCode)
    {
        switch (faultCode)
        {
        case OpenFloorFaultCode::HardwareSetupFailed:
            return "HARDWARE_SETUP_FAILED";
        case OpenFloorFaultCode::DriveInitFailed:
            return "DRIVE_INIT_FAILED";
        case OpenFloorFaultCode::SensorInitFailed:
            return "SENSOR_INIT_FAILED";
        case OpenFloorFaultCode::TimingLogOpenFailed:
            return "TIMING_LOG_OPEN_FAILED";
        case OpenFloorFaultCode::TimingLogWriteFailed:
            return "TIMING_LOG_WRITE_FAILED";
        case OpenFloorFaultCode::MainLogOpenFailed:
            return "MAIN_LOG_OPEN_FAILED";
        case OpenFloorFaultCode::MainLogWriteFailed:
            return "MAIN_LOG_WRITE_FAILED";
        case OpenFloorFaultCode::EstimatorFault:
            return "ESTIMATOR_FAULT";
        case OpenFloorFaultCode::SelectorJumperRemoved:
            return "SELECTOR_JUMPER_REMOVED";
        case OpenFloorFaultCode::RecoveryTimedOut:
            return "RECOVERY_TIMED_OUT";
        case OpenFloorFaultCode::LaunchBoundExceeded:
            return "LAUNCH_BOUND_EXCEEDED";
        case OpenFloorFaultCode::StraightWatchdogStall:
            return "STRAIGHT_WATCHDOG_STALL";
        case OpenFloorFaultCode::StraightSectionTimedOut:
            return "STRAIGHT_SECTION_TIMED_OUT";
        case OpenFloorFaultCode::YawSectionTimedOut:
            return "YAW_SECTION_TIMED_OUT";
        case OpenFloorFaultCode::YawProfileInvalid:
            return "YAW_PROFILE_INVALID";
        case OpenFloorFaultCode::SmoothGeometryUnavailable:
            return "SMOOTH_GEOMETRY_UNAVAILABLE";
        case OpenFloorFaultCode::SmoothWatchdogStall:
            return "SMOOTH_WATCHDOG_STALL";
        case OpenFloorFaultCode::SmoothSectionTimedOut:
            return "SMOOTH_SECTION_TIMED_OUT";
        case OpenFloorFaultCode::SmoothTargetInvalid:
            return "SMOOTH_TARGET_INVALID";
        case OpenFloorFaultCode::LoggerOverflow:
            return "LOGGER_OVERFLOW";
        case OpenFloorFaultCode::LoggerWriteFailure:
            return "LOGGER_WRITE_FAILURE";
        case OpenFloorFaultCode::None:
        default:
            return "NONE";
        }
    }

    inline float OpenFloorMetersToHalfSteps(float meters)
    {
		constexpr float halfStepM = OpenFloorHalfStepMeters();
        if (!(halfStepM > 0.0f))
        {
            return 0.0f;
        }
        return meters / halfStepM;
    }

}
