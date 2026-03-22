#define CORE_TEENSY 1
#define ARDUINO 10819
#define ARDUINO_TEENSY41 1
#include "../MazeMap/MazeMap/Vector2f.h"

float project_sqrt(float x)
{
    return MazeMap::Math::Sqrtf(x);
}

float vector_mag()
{
    MazeMap::Vectorf<2> vec(3.0f, 4.0f);
    return vec.GetMagnitude();
}
