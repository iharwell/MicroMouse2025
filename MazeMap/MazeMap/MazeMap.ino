#include <Eigen.h>
#include "Application.h"

namespace
{
    MazeMap::App::Application gApplication;
}

void setup()
{
    gApplication.Setup();
}

void loop()
{
    gApplication.Loop();
}

