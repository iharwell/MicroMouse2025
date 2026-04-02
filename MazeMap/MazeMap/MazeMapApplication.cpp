#include "MazeMapApplicationPrivate.h"

using MazeMap::App::Internal::IApplicationMode;
using MazeMap::App::Internal::IMissionModeHost;
using MazeMap::App::Internal::MissionRunMode;
using MazeMap::App::Internal::ManeuverFileTestMode;
using MazeMap::App::Internal::CorridorRepeatabilityMode;
using MazeMap::App::Internal::PositionAccuracyAuditMode;

namespace MazeMap::App::Internal
{
    IApplicationMode& GetAuxMeasurementMode();
    IApplicationMode& GetFrontWallCharacterizationMode();
    IApplicationMode& GetWallSensorLedCalibrationMode();
    IApplicationMode& GetDiagnosticMode();
    IMissionModeHost& GetMissionModeHost();
}

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

    MazeMap::App::StartupModeRequests ReadStartupModeRequests()
    {
        MazeMap::App::StartupModeRequests requests{};
        requests.frontWallCharacterization = IsFrontWallCharacterizationModeRequested();
        requests.wallSensorLedCalibration = IsWallSensorLedCalibrationModeRequested();
        requests.auxiliaryMeasurement = IsAuxiliaryMeasurementModeRequested();
        requests.maneuverFileTest = IsManeuverTestModeRequested();
        requests.primaryDiagnostic = IsPrimaryDiagnosticModeRequested();
        return requests;
    }

    IApplicationMode& ResolveAuxiliaryMeasurementMode(ApplicationControllers& controllers)
    {
        if constexpr (AuxMeasurementConfig::kRoutine == AuxMeasurementConfig::Routine::CorridorRepeatabilitySweep)
        {
            return controllers.corridorRepeatabilityMode;
        }

        if constexpr (AuxMeasurementConfig::kRoutine == AuxMeasurementConfig::Routine::PositionAccuracyAudit)
        {
            return controllers.positionAccuracyAuditMode;
        }

        return controllers.auxMeasurement;
    }

    IApplicationMode& ResolveApplicationMode(ApplicationControllers& controllers, MazeMap::App::StartupMode startupMode)
    {
        switch (startupMode)
        {
        case MazeMap::App::StartupMode::FrontWallCharacterization:
            return controllers.frontWallCharacterization;
        case MazeMap::App::StartupMode::WallSensorLedCalibration:
            return controllers.ledCalibration;
        case MazeMap::App::StartupMode::AuxiliaryMeasurement:
            return ResolveAuxiliaryMeasurementMode(controllers);
        case MazeMap::App::StartupMode::ManeuverFileTest:
            return controllers.maneuverFileTestMode;
        case MazeMap::App::StartupMode::PrimaryDiagnostic:
            return controllers.diagnostic;
        case MazeMap::App::StartupMode::Mission:
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
        return ResolveApplicationMode(controllers, ResolveStartupMode(ReadStartupModeRequests()));
    }
}

