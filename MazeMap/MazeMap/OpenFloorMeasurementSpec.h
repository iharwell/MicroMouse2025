#pragma once

#include "MazeMapRuntimeCore.h"

#include <array>
#include <cmath>

namespace MazeMap
{
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

    inline const char* OpenFloorDirectionLabel(const char* value)
    {
        return (value != nullptr && value[0] != '\0') ? value : "NONE";
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
