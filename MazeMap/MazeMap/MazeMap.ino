#include <Eigen.h>
#include "MazeMapApplication.h"

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

