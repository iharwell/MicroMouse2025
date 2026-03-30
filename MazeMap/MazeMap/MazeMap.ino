#include "MazeMapApplication.h"

namespace
{
    MazeMapApp::Application gApplication;
}

void setup()
{
    gApplication.Setup();
}

void loop()
{
    gApplication.Loop();
}
