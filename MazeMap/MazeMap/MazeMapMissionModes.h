#pragma once

#include "MazeMapApplicationMode.h"
#include "MazeMapMissionModeHost.h"

namespace MazeMap::App::Internal
{
    class MissionHostedModeBase : public IApplicationMode
    {
    public:
        explicit MissionHostedModeBase(IMissionModeHost& host);
        MissionHostedModeBase(const MissionHostedModeBase&) = delete;
        MissionHostedModeBase& operator=(const MissionHostedModeBase&) = delete;
        MissionHostedModeBase(MissionHostedModeBase&&) = delete;
        MissionHostedModeBase& operator=(MissionHostedModeBase&&) = delete;

    protected:
        IMissionModeHost& Host() const;

    private:
        IMissionModeHost& _host;
    };

    class MissionRunMode final : public MissionHostedModeBase
    {
    public:
        explicit MissionRunMode(IMissionModeHost& host);
        bool Begin() override;
        void Run() override;
    };

    class ManeuverFileTestMode final : public MissionHostedModeBase
    {
    public:
        explicit ManeuverFileTestMode(IMissionModeHost& host);
        bool Begin() override;
        void Run() override;
    };

    class CorridorRepeatabilityMode final : public MissionHostedModeBase
    {
    public:
        explicit CorridorRepeatabilityMode(IMissionModeHost& host);
        bool Begin() override;
        void Run() override;
    };

    class PositionAccuracyAuditMode final : public MissionHostedModeBase
    {
    public:
        explicit PositionAccuracyAuditMode(IMissionModeHost& host);
        bool Begin() override;
        void Run() override;
    };
}

