#pragma once

#include "BootModeDescriptor.h"
#include "MazeMapApplicationMode.h"

#include <memory>

namespace MazeMap::App::Internal
{
    class SharedRobotRuntime;

    class ManeuverFileTestMode final : public IApplicationMode
    {
    public:
        explicit ManeuverFileTestMode(SharedRobotRuntime& runtime);
        ~ManeuverFileTestMode();

        ManeuverFileTestMode(const ManeuverFileTestMode&) = delete;
        ManeuverFileTestMode& operator=(const ManeuverFileTestMode&) = delete;
        ManeuverFileTestMode(ManeuverFileTestMode&&) = delete;
        ManeuverFileTestMode& operator=(ManeuverFileTestMode&&) = delete;

        bool Begin() override;
        void Run() override;

    private:
        class Implementation;
        std::unique_ptr<Implementation> _impl;
    };

    IApplicationMode& GetManeuverFileTestMode();
    const BootModeDescriptor& GetManeuverFileTestBootModeDescriptor();
}
