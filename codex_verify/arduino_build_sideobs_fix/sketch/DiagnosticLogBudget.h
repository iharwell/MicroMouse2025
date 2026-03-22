#line 1 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\DiagnosticLogBudget.h"
#pragma once

#include <cstddef>

namespace MazeMap
{
    inline constexpr std::size_t kDiagnosticEventLineCapacity = 320U;
    inline constexpr std::size_t kDiagnosticEventTimestampDigits = 10U;
    inline constexpr std::size_t kDiagnosticEventEnvelopeChars =
        (sizeof("# event,") - 1U) + kDiagnosticEventTimestampDigits + 1U + 1U + 1U;

    constexpr bool DiagnosticEventLineFits(std::size_t typeLength, std::size_t messageLength)
    {
        return (typeLength + messageLength + kDiagnosticEventEnvelopeChars + 1U) <= kDiagnosticEventLineCapacity;
    }
}
