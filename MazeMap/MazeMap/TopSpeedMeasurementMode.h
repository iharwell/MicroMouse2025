#pragma once

#include "BootModeDescriptor.h"

namespace MazeMap::App::Internal
{
    class IApplicationMode;

    IApplicationMode& GetTopSpeedMeasurementMode();
    const BootModeDescriptor& GetTopSpeedMeasurementBootModeDescriptor();
}
