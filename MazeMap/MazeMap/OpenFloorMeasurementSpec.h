#pragma once

#include "MazeMapRuntimeCore.h"

#include <array>
#include <cmath>

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

    enum class OpenFloorSectionId : uint8_t
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

    enum class OpenFloorMarkerId : uint8_t
    {
        C = 0U,
        N,
        S,
        CW,
        CCW,
    };

    enum class OpenFloorPrimitiveFamily : uint8_t
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

    enum class OpenFloorPrimitiveId : uint8_t
    {
        None = 0U,
        TimingNoMotion,
        StaticHold,
        OpenLoopLaunch,
        Str1,
        Str2,
        Str4,
        Ip90,
        Ip90M,
        Ip180,
        S45sd,
        S45sdM,
        S45ss,
        S45ssM,
        S45ls,
        S45lsM,
        S45ld,
        S45ldM,
        S90sd,
        S90sdM,
        S90ss,
        S90ssM,
        S90ls,
        S90lsM,
        S135sd,
        S135sdM,
        S135ss,
        S135ssM,
        S135ls,
        S135lsM,
        S135ld,
        S135ldM,
        S180ss,
        S180ssM,
        S180ls,
        S180lsM,
        Recovery,
        Ip180M,
    };

    enum class OpenFloorDirectionId : uint8_t
    {
        None = 0U,
        Positive,
        Negative,
        Northbound,
        Southbound,
        Clockwise,
        CounterClockwise,
        Flip,
        Left,
        Right,
    };

    enum class OpenFloorSpeedBin : uint8_t
    {
        None = 0U,
        Low,
        Medium,
        High,
    };

    enum class OpenFloorPhaseId : uint8_t
    {
        Idle = 0U,
        Hold,
        LaunchPulse,
        Recovery,
        Accel,
        Cruise,
        Brake,
        Startup,
        SteadyRotation,
        Stop,
        Entry,
        Middle,
        Exit,
    };

    enum class OpenFloorFaultCode : uint8_t
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

    struct OpenFloorMarkerPose
    {
        OpenFloorMarkerId id;
        const char* name;
        float xHalfSteps;
        float yHalfSteps;
        Direction heading;
    };

    struct OpenFloorSectionDefinition
    {
        OpenFloorSectionId id;
        const char* name;
        OpenFloorMarkerId startMarker;
    };

    struct OpenFloorPrimitiveDefinition
    {
        OpenFloorPrimitiveId id;
        const char* name;
        OpenFloorPrimitiveFamily family;
        bool mirrored;
    };

    inline constexpr std::array<OpenFloorMarkerPose, 5U> kOpenFloorMarkers = {
        OpenFloorMarkerPose{ OpenFloorMarkerId::C, "C", 2.5f, 2.5f, Up },
        OpenFloorMarkerPose{ OpenFloorMarkerId::N, "N", 2.5f, 0.5f, Up },
        OpenFloorMarkerPose{ OpenFloorMarkerId::S, "S", 2.5f, 4.5f, Down },
        OpenFloorMarkerPose{ OpenFloorMarkerId::CW, "CW", 1.5f, 1.5f, Up },
        OpenFloorMarkerPose{ OpenFloorMarkerId::CCW, "CCW", 3.5f, 1.5f, Up },
    };

    inline constexpr std::array<OpenFloorSectionDefinition, 8U> kOpenFloorSections = {
        OpenFloorSectionDefinition{ OpenFloorSectionId::Sec00Timing, "SEC_00_TIMING", OpenFloorMarkerId::C },
        OpenFloorSectionDefinition{ OpenFloorSectionId::Sec10Static, "SEC_10_STATIC", OpenFloorMarkerId::C },
        OpenFloorSectionDefinition{ OpenFloorSectionId::Sec20Launch, "SEC_20_LAUNCH", OpenFloorMarkerId::C },
        OpenFloorSectionDefinition{ OpenFloorSectionId::Sec30Straight, "SEC_30_STRAIGHT", OpenFloorMarkerId::N },
        OpenFloorSectionDefinition{ OpenFloorSectionId::Sec40Yaw, "SEC_40_YAW", OpenFloorMarkerId::C },
        OpenFloorSectionDefinition{ OpenFloorSectionId::Sec50Smooth, "SEC_50_SMOOTH", OpenFloorMarkerId::C },
        OpenFloorSectionDefinition{ OpenFloorSectionId::Sec60LoopCw, "SEC_60_LOOP_CW", OpenFloorMarkerId::CW },
        OpenFloorSectionDefinition{ OpenFloorSectionId::Sec70LoopCcw, "SEC_70_LOOP_CCW", OpenFloorMarkerId::CCW },
    };

    inline constexpr std::array<OpenFloorPrimitiveDefinition, 38U> kOpenFloorPrimitives = {
        OpenFloorPrimitiveDefinition{ OpenFloorPrimitiveId::None, "NONE", OpenFloorPrimitiveFamily::None, false },
        OpenFloorPrimitiveDefinition{ OpenFloorPrimitiveId::TimingNoMotion, "TIMING_NO_MOTION", OpenFloorPrimitiveFamily::Timing, false },
        OpenFloorPrimitiveDefinition{ OpenFloorPrimitiveId::StaticHold, "STATIC_HOLD", OpenFloorPrimitiveFamily::StaticHold, false },
        OpenFloorPrimitiveDefinition{ OpenFloorPrimitiveId::OpenLoopLaunch, "OPEN_LOOP_LAUNCH", OpenFloorPrimitiveFamily::Launch, false },
        OpenFloorPrimitiveDefinition{ OpenFloorPrimitiveId::Str1, "STR1", OpenFloorPrimitiveFamily::Straight, false },
        OpenFloorPrimitiveDefinition{ OpenFloorPrimitiveId::Str2, "STR2", OpenFloorPrimitiveFamily::Straight, false },
        OpenFloorPrimitiveDefinition{ OpenFloorPrimitiveId::Str4, "STR4", OpenFloorPrimitiveFamily::Straight, false },
        OpenFloorPrimitiveDefinition{ OpenFloorPrimitiveId::Ip90, "IP90", OpenFloorPrimitiveFamily::InPlaceTurn, false },
        OpenFloorPrimitiveDefinition{ OpenFloorPrimitiveId::Ip90M, "IP90_M", OpenFloorPrimitiveFamily::InPlaceTurn, true },
        OpenFloorPrimitiveDefinition{ OpenFloorPrimitiveId::Ip180, "IP180", OpenFloorPrimitiveFamily::InPlaceTurn, false },
        OpenFloorPrimitiveDefinition{ OpenFloorPrimitiveId::S45sd, "S45SD", OpenFloorPrimitiveFamily::SmoothTurn, false },
        OpenFloorPrimitiveDefinition{ OpenFloorPrimitiveId::S45sdM, "S45SD_M", OpenFloorPrimitiveFamily::SmoothTurn, true },
        OpenFloorPrimitiveDefinition{ OpenFloorPrimitiveId::S45ss, "S45SS", OpenFloorPrimitiveFamily::SmoothTurn, false },
        OpenFloorPrimitiveDefinition{ OpenFloorPrimitiveId::S45ssM, "S45SS_M", OpenFloorPrimitiveFamily::SmoothTurn, true },
        OpenFloorPrimitiveDefinition{ OpenFloorPrimitiveId::S45ls, "S45LS", OpenFloorPrimitiveFamily::SmoothTurn, false },
        OpenFloorPrimitiveDefinition{ OpenFloorPrimitiveId::S45lsM, "S45LS_M", OpenFloorPrimitiveFamily::SmoothTurn, true },
        OpenFloorPrimitiveDefinition{ OpenFloorPrimitiveId::S45ld, "S45LD", OpenFloorPrimitiveFamily::SmoothTurn, false },
        OpenFloorPrimitiveDefinition{ OpenFloorPrimitiveId::S45ldM, "S45LD_M", OpenFloorPrimitiveFamily::SmoothTurn, true },
        OpenFloorPrimitiveDefinition{ OpenFloorPrimitiveId::S90sd, "S90SD", OpenFloorPrimitiveFamily::SmoothTurn, false },
        OpenFloorPrimitiveDefinition{ OpenFloorPrimitiveId::S90sdM, "S90SD_M", OpenFloorPrimitiveFamily::SmoothTurn, true },
        OpenFloorPrimitiveDefinition{ OpenFloorPrimitiveId::S90ss, "S90SS", OpenFloorPrimitiveFamily::SmoothTurn, false },
        OpenFloorPrimitiveDefinition{ OpenFloorPrimitiveId::S90ssM, "S90SS_M", OpenFloorPrimitiveFamily::SmoothTurn, true },
        OpenFloorPrimitiveDefinition{ OpenFloorPrimitiveId::S90ls, "S90LS", OpenFloorPrimitiveFamily::SmoothTurn, false },
        OpenFloorPrimitiveDefinition{ OpenFloorPrimitiveId::S90lsM, "S90LS_M", OpenFloorPrimitiveFamily::SmoothTurn, true },
        OpenFloorPrimitiveDefinition{ OpenFloorPrimitiveId::S135sd, "S135SD", OpenFloorPrimitiveFamily::SmoothTurn, false },
        OpenFloorPrimitiveDefinition{ OpenFloorPrimitiveId::S135sdM, "S135SD_M", OpenFloorPrimitiveFamily::SmoothTurn, true },
        OpenFloorPrimitiveDefinition{ OpenFloorPrimitiveId::S135ss, "S135SS", OpenFloorPrimitiveFamily::SmoothTurn, false },
        OpenFloorPrimitiveDefinition{ OpenFloorPrimitiveId::S135ssM, "S135SS_M", OpenFloorPrimitiveFamily::SmoothTurn, true },
        OpenFloorPrimitiveDefinition{ OpenFloorPrimitiveId::S135ls, "S135LS", OpenFloorPrimitiveFamily::SmoothTurn, false },
        OpenFloorPrimitiveDefinition{ OpenFloorPrimitiveId::S135lsM, "S135LS_M", OpenFloorPrimitiveFamily::SmoothTurn, true },
        OpenFloorPrimitiveDefinition{ OpenFloorPrimitiveId::S135ld, "S135LD", OpenFloorPrimitiveFamily::SmoothTurn, false },
        OpenFloorPrimitiveDefinition{ OpenFloorPrimitiveId::S135ldM, "S135LD_M", OpenFloorPrimitiveFamily::SmoothTurn, true },
        OpenFloorPrimitiveDefinition{ OpenFloorPrimitiveId::S180ss, "S180SS", OpenFloorPrimitiveFamily::SmoothTurn, false },
        OpenFloorPrimitiveDefinition{ OpenFloorPrimitiveId::S180ssM, "S180SS_M", OpenFloorPrimitiveFamily::SmoothTurn, true },
        OpenFloorPrimitiveDefinition{ OpenFloorPrimitiveId::S180ls, "S180LS", OpenFloorPrimitiveFamily::SmoothTurn, false },
        OpenFloorPrimitiveDefinition{ OpenFloorPrimitiveId::S180lsM, "S180LS_M", OpenFloorPrimitiveFamily::SmoothTurn, true },
        OpenFloorPrimitiveDefinition{ OpenFloorPrimitiveId::Recovery, "RECOVERY", OpenFloorPrimitiveFamily::Recovery, false },
        OpenFloorPrimitiveDefinition{ OpenFloorPrimitiveId::Ip180M, "IP180_M", OpenFloorPrimitiveFamily::InPlaceTurn, true },
    };

    inline constexpr std::array<const char*, 58U> kOpenFloorDeferredPrimitiveIds = {
        "S3",
        "S5",
        "S6",
        "S7",
        "S8",
        "S9",
        "S10",
        "S11",
        "S12",
        "S13",
        "S14",
        "S15",
        "S16",
        "S17",
        "S18",
        "S19",
        "S20",
        "S21",
        "S22",
        "S23",
        "S24",
        "S25",
        "S26",
        "S27",
        "S28",
        "S29",
        "S30",
        "S31",
        "IP45",
        "IP45_M",
        "IP135",
        "IP135_M",
        "S45SD",
        "S45SD_M",
        "S45LS",
        "S45LS_M",
        "S45LD",
        "S45LD_M",
        "S90SD",
        "S90SD_M",
        "S90LS",
        "S90LS_M",
        "S90LD",
        "S90LD_M",
        "S135SD",
        "S135SD_M",
        "S135LS",
        "S135LS_M",
        "S135LD",
        "S135LD_M",
        "S180SS",
        "S180SS_M",
        "S180LS",
        "S180LS_M",
        "S90ELD",
        "S90ELD_M",
        "S180ELS",
        "S180ELS_M",
    };

    inline constexpr std::array<float, 3U> kOpenFloorStraightSpeedBinsMps = { 0.1f, 0.30f, 0.55f };
    inline constexpr std::array<float, 3U> kOpenFloorYawOmegaBinsRadps = { 9.0f, 18.0f, 27.0f };
    // Keep each yaw repeat nominally balanced so the section returns to its starting heading every cycle.
    inline constexpr std::array<OpenFloorPrimitiveId, 4U> kOpenFloorYawPrimitiveIds = {
        OpenFloorPrimitiveId::Ip90,
        OpenFloorPrimitiveId::Ip90M,
        OpenFloorPrimitiveId::Ip180,
        OpenFloorPrimitiveId::Ip180M,
    };
    inline constexpr std::array<OpenFloorDirectionId, 4U> kOpenFloorYawDirectionIds = {
        OpenFloorDirectionId::Clockwise,
        OpenFloorDirectionId::CounterClockwise,
        OpenFloorDirectionId::Clockwise,
        OpenFloorDirectionId::CounterClockwise,
    };
    inline constexpr std::array<float, 4U> kOpenFloorYawNominalAnglesRad = {
        HALF_PI_F,
        -HALF_PI_F,
        PI_F,
        -PI_F,
    };
    inline constexpr std::array<float, 3U> kOpenFloorSmoothSpeedBinsMps = { 0.4f, 0.45f, 0.45f };
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

    inline constexpr float OpenFloorHalfStepMeters() noexcept
    {
        return Maze::GetCellDimension() * 0.5f;
    }

    inline constexpr float OpenFloorStrEquivalentDistanceMeters(uint8_t halfSteps) noexcept
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

    inline const OpenFloorMarkerPose& GetOpenFloorMarker(OpenFloorMarkerId id)
    {
        for (const OpenFloorMarkerPose& marker : kOpenFloorMarkers)
        {
            if (marker.id == id)
            {
                return marker;
            }
        }

        return kOpenFloorMarkers[0];
    }

    inline const OpenFloorSectionDefinition& GetOpenFloorSection(OpenFloorSectionId id)
    {
        for (const OpenFloorSectionDefinition& section : kOpenFloorSections)
        {
            if (section.id == id)
            {
                return section;
            }
        }

        return kOpenFloorSections[0];
    }

    inline const char* OpenFloorSectionName(OpenFloorSectionId id)
    {
        return GetOpenFloorSection(id).name;
    }

    inline const char* OpenFloorMarkerName(OpenFloorMarkerId id)
    {
        return GetOpenFloorMarker(id).name;
    }

    inline const OpenFloorPrimitiveDefinition& GetOpenFloorPrimitive(OpenFloorPrimitiveId id)
    {
        for (const OpenFloorPrimitiveDefinition& primitive : kOpenFloorPrimitives)
        {
            if (primitive.id == id)
            {
                return primitive;
            }
        }

        return kOpenFloorPrimitives[0];
    }

    inline const char* OpenFloorPrimitiveName(OpenFloorPrimitiveId id)
    {
        return GetOpenFloorPrimitive(id).name;
    }

    inline OpenFloorPrimitiveFamily OpenFloorPrimitiveFamilyForId(OpenFloorPrimitiveId id)
    {
        return GetOpenFloorPrimitive(id).family;
    }

    inline bool OpenFloorPrimitiveIsMirrored(OpenFloorPrimitiveId id)
    {
        return GetOpenFloorPrimitive(id).mirrored;
    }

    inline const char* OpenFloorPrimitiveFamilyName(OpenFloorPrimitiveFamily family)
    {
        switch (family)
        {
        case OpenFloorPrimitiveFamily::Timing:
            return "TIMING";
        case OpenFloorPrimitiveFamily::StaticHold:
            return "STATIC_HOLD";
        case OpenFloorPrimitiveFamily::Launch:
            return "LAUNCH";
        case OpenFloorPrimitiveFamily::Straight:
            return "STRAIGHT";
        case OpenFloorPrimitiveFamily::InPlaceTurn:
            return "IN_PLACE_TURN";
        case OpenFloorPrimitiveFamily::SmoothTurn:
            return "SMOOTH_TURN";
        case OpenFloorPrimitiveFamily::Recovery:
            return "RECOVERY";
        case OpenFloorPrimitiveFamily::None:
        default:
            return "NONE";
        }
    }

    inline const char* OpenFloorDirectionName(OpenFloorDirectionId direction)
    {
        switch (direction)
        {
        case OpenFloorDirectionId::Positive:
            return "POSITIVE";
        case OpenFloorDirectionId::Negative:
            return "NEGATIVE";
        case OpenFloorDirectionId::Northbound:
            return "NORTHBOUND";
        case OpenFloorDirectionId::Southbound:
            return "SOUTHBOUND";
        case OpenFloorDirectionId::Clockwise:
            return "CLOCKWISE";
        case OpenFloorDirectionId::CounterClockwise:
            return "COUNTERCLOCKWISE";
        case OpenFloorDirectionId::Flip:
            return "FLIP";
        case OpenFloorDirectionId::Left:
            return "LEFT";
        case OpenFloorDirectionId::Right:
            return "RIGHT";
        case OpenFloorDirectionId::None:
        default:
            return "NONE";
        }
    }

    inline const char* OpenFloorSpeedBinName(OpenFloorSpeedBin bin)
    {
        switch (bin)
        {
        case OpenFloorSpeedBin::Low:
            return "LOW";
        case OpenFloorSpeedBin::Medium:
            return "MED";
        case OpenFloorSpeedBin::High:
            return "HIGH";
        case OpenFloorSpeedBin::None:
        default:
            return "NONE";
        }
    }

    inline const char* OpenFloorPhaseName(OpenFloorPhaseId phase)
    {
        switch (phase)
        {
        case OpenFloorPhaseId::Hold:
            return "hold";
        case OpenFloorPhaseId::LaunchPulse:
            return "launch_pulse";
        case OpenFloorPhaseId::Recovery:
            return "recovery";
        case OpenFloorPhaseId::Accel:
            return "accel";
        case OpenFloorPhaseId::Cruise:
            return "cruise";
        case OpenFloorPhaseId::Brake:
            return "brake";
        case OpenFloorPhaseId::Startup:
            return "startup";
        case OpenFloorPhaseId::SteadyRotation:
            return "steady_rotation";
        case OpenFloorPhaseId::Stop:
            return "stop";
        case OpenFloorPhaseId::Entry:
            return "entry";
        case OpenFloorPhaseId::Middle:
            return "middle";
        case OpenFloorPhaseId::Exit:
            return "exit";
        case OpenFloorPhaseId::Idle:
        default:
            return "idle";
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

    inline float OpenFloorMarkerXMeters(OpenFloorMarkerId id)
    {
        return GetOpenFloorMarker(id).xHalfSteps * OpenFloorHalfStepMeters();
    }

    inline float OpenFloorMarkerYMeters(OpenFloorMarkerId id)
    {
        return GetOpenFloorMarker(id).yHalfSteps * OpenFloorHalfStepMeters();
    }

    inline float OpenFloorMetersToHalfSteps(float meters)
    {
        const float halfStepM = OpenFloorHalfStepMeters();
        if (!(halfStepM > 0.0f))
        {
            return 0.0f;
        }
        return meters / halfStepM;
    }

}
