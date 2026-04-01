#include "MazeMapApplicationPrivate.h"

using MazeMapApp::Internal::IApplicationMode;
using MazeMapApp::Internal::IMissionModeHost;
using MazeMapApp::Internal::MissionRunMode;
using MazeMapApp::Internal::ManeuverFileTestMode;
using MazeMapApp::Internal::CorridorRepeatabilityMode;
using MazeMapApp::Internal::PositionAccuracyAuditMode;

namespace MazeMapApp::Internal
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
            : ledCalibration(MazeMapApp::Internal::GetWallSensorLedCalibrationMode())
            , frontWallCharacterization(MazeMapApp::Internal::GetFrontWallCharacterizationMode())
            , auxMeasurement(MazeMapApp::Internal::GetAuxMeasurementMode())
            , diagnostic(MazeMapApp::Internal::GetDiagnosticMode())
            , mission(MazeMapApp::Internal::GetMissionModeHost())
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

    MazeMapApp::StartupModeRequests ReadStartupModeRequests()
    {
        MazeMapApp::StartupModeRequests requests{};
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

    IApplicationMode& ResolveApplicationMode(ApplicationControllers& controllers, MazeMapApp::StartupMode startupMode)
    {
        switch (startupMode)
        {
        case MazeMapApp::StartupMode::FrontWallCharacterization:
            return controllers.frontWallCharacterization;
        case MazeMapApp::StartupMode::WallSensorLedCalibration:
            return controllers.ledCalibration;
        case MazeMapApp::StartupMode::AuxiliaryMeasurement:
            return ResolveAuxiliaryMeasurementMode(controllers);
        case MazeMapApp::StartupMode::ManeuverFileTest:
            return controllers.maneuverFileTestMode;
        case MazeMapApp::StartupMode::PrimaryDiagnostic:
            return controllers.diagnostic;
        case MazeMapApp::StartupMode::Mission:
        default:
            return controllers.missionMode;
        }
    }
}

namespace MazeMapApp::Internal
{
    IApplicationMode& ResolveActiveApplicationMode()
    {
        ApplicationControllers& controllers = GetApplicationControllers();
        return ResolveApplicationMode(controllers, ResolveStartupMode(ReadStartupModeRequests()));
    }
}
