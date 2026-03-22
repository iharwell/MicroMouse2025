#include "../MazeMap/MazeMap/Vector2f.h"

int main()
{
    MazeMap::Vectorf<2> vec(3.0f, 4.0f);
    return static_cast<int>(vec.GetMagnitude());
}
