#pragma once

#include "Defines.h"
#include "IApplicationMode.h"

#include <memory>

namespace MazeMap::App
{
    struct BootModeDescriptor;
}

namespace MazeMap::App::Internal
{
    class SharedRobotRuntime;

    class EXPORT OpenFloorMeasurementController final : public IApplicationMode
    {
    public:
        explicit OpenFloorMeasurementController(SharedRobotRuntime& runtime);
        ~OpenFloorMeasurementController() override;

        OpenFloorMeasurementController(const OpenFloorMeasurementController&) = delete;
        OpenFloorMeasurementController& operator=(const OpenFloorMeasurementController&) = delete;
        OpenFloorMeasurementController(OpenFloorMeasurementController&&) = delete;
        OpenFloorMeasurementController& operator=(OpenFloorMeasurementController&&) = delete;

        bool Begin() override;
        void Run() override;

    private:
        class State;
        std::unique_ptr<State> _state;
    };

    IApplicationMode& GetOpenFloorMeasurementMode();
    const MazeMap::App::BootModeDescriptor& GetOpenFloorMeasurementBootModeDescriptor();
}
