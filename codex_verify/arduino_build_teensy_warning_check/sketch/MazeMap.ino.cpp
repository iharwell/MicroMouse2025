#include <Arduino.h>
#line 1 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
#include "MazeMapApplication.h"

namespace
{
    MazeMapApp::Application gApplication;
}

#line 8 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
void setup();
#line 13 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
void loop();
#line 8 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMap.ino"
void setup()
{
    gApplication.Setup();
}

void loop()
{
    gApplication.Loop();
}

