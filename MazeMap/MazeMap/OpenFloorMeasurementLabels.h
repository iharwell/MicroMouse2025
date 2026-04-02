#ifndef OPENFLOORMEASUREMENTLABELS_H
#define OPENFLOORMEASUREMENTLABELS_H

// Declares the label bundle that identifies which open-floor section, primitive, direction, and phase produced a sample row.

#include "OpenFloorMeasurementSpec.h"

struct OpenFloorMeasurementLabels
{
    MazeMap::OpenFloorSectionId sectionId = MazeMap::OpenFloorSectionId::Sec00Timing;
    MazeMap::OpenFloorMarkerId startMarkerId = MazeMap::OpenFloorMarkerId::C;
    MazeMap::OpenFloorPrimitiveId primitiveId = MazeMap::OpenFloorPrimitiveId::None;
    MazeMap::OpenFloorDirectionId directionId = MazeMap::OpenFloorDirectionId::None;
    MazeMap::OpenFloorPhaseId phaseId = MazeMap::OpenFloorPhaseId::Idle;
    MazeMap::OpenFloorSpeedBin speedBin = MazeMap::OpenFloorSpeedBin::None;
    uint16_t repeatIndex = 0U;
    float progressNorm = 0.0f;
    bool abortMarker = false;
};

#endif
