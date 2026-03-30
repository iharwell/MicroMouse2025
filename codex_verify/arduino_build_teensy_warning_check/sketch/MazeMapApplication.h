#line 1 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMapApplication.h"
#pragma once

namespace MazeMapApp
{
    enum class StartupMode
    {
        FrontWallCharacterization,
        WallSensorLedCalibration,
        AuxiliaryMeasurement,
        ManeuverFileTest,
        PrimaryDiagnostic,
        Mission
    };

    struct StartupModeRequests
    {
        bool frontWallCharacterization = false;
        bool wallSensorLedCalibration = false;
        bool auxiliaryMeasurement = false;
        bool maneuverFileTest = false;
        bool primaryDiagnostic = false;
    };

    inline StartupMode ResolveStartupMode(const StartupModeRequests& requests) noexcept
    {
        if (requests.frontWallCharacterization)
        {
            return StartupMode::FrontWallCharacterization;
        }

        if (requests.wallSensorLedCalibration)
        {
            return StartupMode::WallSensorLedCalibration;
        }

        if (requests.auxiliaryMeasurement)
        {
            return StartupMode::AuxiliaryMeasurement;
        }

        if (requests.maneuverFileTest)
        {
            return StartupMode::ManeuverFileTest;
        }

        if (requests.primaryDiagnostic)
        {
            return StartupMode::PrimaryDiagnostic;
        }

        return StartupMode::Mission;
    }

    class Application final
    {
    public:
        Application() = default;
        Application(const Application&) = delete;
        Application& operator=(const Application&) = delete;
        Application(Application&&) = delete;
        Application& operator=(Application&&) = delete;

        void Setup();
        void Loop();
    };
}
