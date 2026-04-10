#pragma once

namespace MazeMap::App
{
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

namespace MazeMapApp = MazeMap::App;

