#include "MazeMapApplication.h"

#include "BootModeRegistry.h"
#include "CorridorRepeatabilityMode.h"
#include "ManeuverFileTestMode.h"
#include "MazeMapControllerRegistry.h"
#include "MissionRunMode.h"
#include "PositionAccuracyAuditMode.h"

using MazeMap::App::Internal::IApplicationMode;
using MazeMap::App::Internal::IMissionModeHost;
using MazeMap::App::Internal::MissionRunMode;
using MazeMap::App::Internal::ManeuverFileTestMode;
using MazeMap::App::Internal::CorridorRepeatabilityMode;
using MazeMap::App::Internal::PositionAccuracyAuditMode;

namespace
{
    class ApplicationControllers final
    {
    public:
        ApplicationControllers()
            : ledCalibration(MazeMap::App::Internal::GetWallSensorLedCalibrationMode())
            , frontWallCharacterization(MazeMap::App::Internal::GetFrontWallCharacterizationMode())
            , auxMeasurement(MazeMap::App::Internal::GetAuxMeasurementMode())
            , diagnostic(MazeMap::App::Internal::GetDiagnosticMode())
            , mission(MazeMap::App::Internal::GetMissionModeHost())
            , missionMode(mission)
            , maneuverFileTestMode(mission)
            , corridorRepeatabilityMode(mission)
            , positionAccuracyAuditMode(mission)
        {
        }

        IApplicationMode& ledCalibration;
        IApplicationMode& frontWallCharacterization;
        IApplicationMode& auxMeasurement;
        IApplicationMode& diagnostic;
        IMissionModeHost& mission;
        MissionRunMode missionMode;
        ManeuverFileTestMode maneuverFileTestMode;
        CorridorRepeatabilityMode corridorRepeatabilityMode;
        PositionAccuracyAuditMode positionAccuracyAuditMode;
    };

    ApplicationControllers& GetApplicationControllers()
    {
        static ApplicationControllers controllers;
        return controllers;
    }

    IApplicationMode& ResolveApplicationMode(ApplicationControllers& controllers, MazeMap::App::BootModeId bootModeId)
    {
        switch (bootModeId)
        {
        case MazeMap::App::BootModeId::FrontWallCharacterization:
            return controllers.frontWallCharacterization;
        case MazeMap::App::BootModeId::WallSensorLedCalibration:
            return controllers.ledCalibration;
        case MazeMap::App::BootModeId::AuxiliaryMeasurement:
            return controllers.auxMeasurement;
        case MazeMap::App::BootModeId::CorridorRepeatability:
            return controllers.corridorRepeatabilityMode;
        case MazeMap::App::BootModeId::PositionAccuracyAudit:
            return controllers.positionAccuracyAuditMode;
        case MazeMap::App::BootModeId::ManeuverFileTest:
            return controllers.maneuverFileTestMode;
        case MazeMap::App::BootModeId::PrimaryDiagnostic:
            return controllers.diagnostic;
        case MazeMap::App::BootModeId::Mission:
        default:
            return controllers.missionMode;
        }
    }
}

namespace MazeMap::App::Internal
{
    IApplicationMode& ResolveActiveApplicationMode()
    {
        ApplicationControllers& controllers = GetApplicationControllers();
        const MazeMap::App::BootModeRegistryEntry& selectedMode = MazeMap::App::ResolveSelectedBootMode();
        return ResolveApplicationMode(controllers, selectedMode.descriptor->id);
    }
}
