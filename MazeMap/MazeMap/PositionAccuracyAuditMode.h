#pragma once

#include "BootModeDescriptor.h"
#include "MazeMapApplicationMode.h"

namespace MazeMap::App::Internal
{
    IApplicationMode& GetPositionAccuracyAuditMode();
    const BootModeDescriptor& GetPositionAccuracyAuditBootModeDescriptor();
}
