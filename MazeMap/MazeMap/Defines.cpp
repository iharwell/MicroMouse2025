#include "pch.h"
#include "Defines.h"

#if !defined(ARDUINO) && !defined(CORE_TEENSY) && !defined(ARDUINO_TEENSY41)
namespace MazeMap::arduino_stub_detail
{
    HostDigitalPinStates& host_pin_states() noexcept
    {
        static HostDigitalPinStates states{};
        return states;
    }

    HostDigitalPinShorts& host_pin_shorts() noexcept
    {
        static HostDigitalPinShorts shorts{};
        return shorts;
    }
}
#endif
