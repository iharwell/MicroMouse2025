#pragma once

#include "MazeMapRuntimeCore.h"

#include <array>
#include <cmath>

namespace MazeMap
{
    inline constexpr const char* kOpenFloorSelectedRoutineName = "open_floor_measurement";
    inline constexpr const char* kOpenFloorFormatVersion = "open_floor_measurement_rev_d";
    inline constexpr const char* kOpenFloorLoggingFormatRevision = "micromouse_logging_spec_rev_g";
    inline constexpr const char* kOpenFloorPrimitiveScheduleRevision = "open_floor_schedule_rev_d";
    inline constexpr const char* kOpenFloorPhaseBinningRevision = "open_floor_phase_bins_rev_d";
    inline constexpr const char* kOpenFloorStartMarkerDefinitionsRevision = "open_floor_markers_rev_d";
    inline constexpr const char* kOpenFloorImuExtrinsicsRevision = "back_left_imu_extrinsics_v1";
    inline constexpr const char* kOpenFloorActiveImuId = "IMU_BL";
    inline constexpr const char* kOpenFloorManifestFileName = "open_floor_run_manifest.json";
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
        Str2,
        Str4,
        Ip90,
        Ip90M,
        Ip180,
        S45ss,
        S45ssM,
        S90ss,
        S90ssM,
        S135ss,
        S135ssM,
        Recovery,
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
        ManifestWriteFailed,
        TimingLogOpenFailed,
        TimingLogWriteFailed,
        MainLogOpenFailed,
        MainLogWriteFailed,
        EstimatorFault,
        WorkspaceViolation,
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

    inline constexpr std::array<OpenFloorPrimitiveDefinition, 16U> kOpenFloorPrimitives = {
        OpenFloorPrimitiveDefinition{ OpenFloorPrimitiveId::None, "NONE", OpenFloorPrimitiveFamily::None, false },
        OpenFloorPrimitiveDefinition{ OpenFloorPrimitiveId::TimingNoMotion, "TIMING_NO_MOTION", OpenFloorPrimitiveFamily::Timing, false },
        OpenFloorPrimitiveDefinition{ OpenFloorPrimitiveId::StaticHold, "STATIC_HOLD", OpenFloorPrimitiveFamily::StaticHold, false },
        OpenFloorPrimitiveDefinition{ OpenFloorPrimitiveId::OpenLoopLaunch, "OPEN_LOOP_LAUNCH", OpenFloorPrimitiveFamily::Launch, false },
        OpenFloorPrimitiveDefinition{ OpenFloorPrimitiveId::Str2, "STR2", OpenFloorPrimitiveFamily::Straight, false },
        OpenFloorPrimitiveDefinition{ OpenFloorPrimitiveId::Str4, "STR4", OpenFloorPrimitiveFamily::Straight, false },
        OpenFloorPrimitiveDefinition{ OpenFloorPrimitiveId::Ip90, "IP90", OpenFloorPrimitiveFamily::InPlaceTurn, false },
        OpenFloorPrimitiveDefinition{ OpenFloorPrimitiveId::Ip90M, "IP90_M", OpenFloorPrimitiveFamily::InPlaceTurn, true },
        OpenFloorPrimitiveDefinition{ OpenFloorPrimitiveId::Ip180, "IP180", OpenFloorPrimitiveFamily::InPlaceTurn, false },
        OpenFloorPrimitiveDefinition{ OpenFloorPrimitiveId::S45ss, "S45SS", OpenFloorPrimitiveFamily::SmoothTurn, false },
        OpenFloorPrimitiveDefinition{ OpenFloorPrimitiveId::S45ssM, "S45SS_M", OpenFloorPrimitiveFamily::SmoothTurn, true },
        OpenFloorPrimitiveDefinition{ OpenFloorPrimitiveId::S90ss, "S90SS", OpenFloorPrimitiveFamily::SmoothTurn, false },
        OpenFloorPrimitiveDefinition{ OpenFloorPrimitiveId::S90ssM, "S90SS_M", OpenFloorPrimitiveFamily::SmoothTurn, true },
        OpenFloorPrimitiveDefinition{ OpenFloorPrimitiveId::S135ss, "S135SS", OpenFloorPrimitiveFamily::SmoothTurn, false },
        OpenFloorPrimitiveDefinition{ OpenFloorPrimitiveId::S135ssM, "S135SS_M", OpenFloorPrimitiveFamily::SmoothTurn, true },
        OpenFloorPrimitiveDefinition{ OpenFloorPrimitiveId::Recovery, "RECOVERY", OpenFloorPrimitiveFamily::Recovery, false },
    };

    inline constexpr std::array<const char*, 59U> kOpenFloorDeferredPrimitiveIds = {
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
        "IP180_M",
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

    inline constexpr std::array<float, 3U> kOpenFloorStraightSpeedBinsMps = { 0.25f, 0.40f, 0.55f };
    inline constexpr std::array<float, 3U> kOpenFloorYawOmegaBinsRadps = { 3.0f, 6.0f, 9.0f };
    inline constexpr std::array<float, 3U> kOpenFloorSmoothSpeedBinsMps = { 0.25f, 0.35f, 0.45f };
    inline constexpr std::array<float, 4U> kOpenFloorLaunchDriveMagnitudes = { 0.18f, 0.24f, 0.30f, 0.36f };

    inline constexpr float OpenFloorHalfStepMeters() noexcept
    {
        return 0.001f * DiagnosticConfig::kHalfStepMm;
    }

    inline constexpr float OpenFloorWorkspaceMaxHalfSteps() noexcept
    {
        return static_cast<float>(DiagnosticConfig::kWorkspaceSizeHalfSteps);
    }

    inline constexpr float OpenFloorWorkspaceMaxMeters() noexcept
    {
        return OpenFloorWorkspaceMaxHalfSteps() * OpenFloorHalfStepMeters();
    }

    inline constexpr float OpenFloorStrEquivalentDistanceMeters(uint8_t halfSteps) noexcept
    {
        return OpenFloorHalfStepMeters() * static_cast<float>(halfSteps);
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
        case OpenFloorFaultCode::ManifestWriteFailed:
            return "MANIFEST_WRITE_FAILED";
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
        case OpenFloorFaultCode::WorkspaceViolation:
            return "WORKSPACE_VIOLATION";
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

    inline bool IsPoseInsideOpenFloorWorkspace(const PoseEstimate& pose)
    {
        const float maxMeters = OpenFloorWorkspaceMaxMeters();
        return std::isfinite(pose.xMeters) &&
            std::isfinite(pose.yMeters) &&
            pose.xMeters >= 0.0f &&
            pose.yMeters >= 0.0f &&
            pose.xMeters <= maxMeters &&
            pose.yMeters <= maxMeters;
    }
}
