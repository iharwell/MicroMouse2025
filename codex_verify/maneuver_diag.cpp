#include <cstdio>
#include "..\MazeMap\MazeMap\ManeuverSet.h"
#include "..\MazeMap\MazeMap\DirectionalLocation.h"

using namespace MazeMap;

int main()
{
    ManeuverSet& ms = ManeuverSet::GetSet();
    std::printf("size=%u\n", static_cast<unsigned>(ms.size()));
    for (uint8_t i = 0; i < ms.size(); ++i)
    {
        __try
        {
            const Maneuver& man = ms[i];
            std::printf("index=%u code=%u steps=%u\n",
                static_cast<unsigned>(i),
                static_cast<unsigned>(man.GetManeuverID()),
                static_cast<unsigned>(man.GetStepCount()));
            DirectionalLocation loc(15, 15, Up);
            DirectionalLocation moved = man.Move(loc, false);
            std::printf("  moved=(%u,%u,%u)\n",
                static_cast<unsigned>(moved.GetLocation().GetX()),
                static_cast<unsigned>(moved.GetLocation().GetY()),
                static_cast<unsigned>(moved.GetDirection()));
        }
        __except (EXCEPTION_EXECUTE_HANDLER)
        {
            std::printf("crash at index=%u\n", static_cast<unsigned>(i));
            return 1;
        }
    }
    return 0;
}
