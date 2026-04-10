#pragma once

#include "Defines.h"
#include "Maze.h"
#include "MmLog.h"

#include <cstdint>
#include <cstring>

#if defined(ARDUINO_TEENSY41)
#include <SD.h>
#else
#include <fstream>
#endif

namespace MazeMap
{
    inline bool ExportMazeSnapshot(const Maze& maze, const char* fileName)
    {
        if (fileName == nullptr || fileName[0] == '\0')
        {
            return false;
        }

#if defined(ARDUINO_TEENSY41)
        auto file = SD.sdfs.open(fileName, O_RDWR | O_CREAT | O_TRUNC);
        if (!file || !file.preAllocate(MMLOG_TEENSY_MIN_PREALLOCATE_BYTES))
#else
        std::ofstream file(fileName, std::ios::binary | std::ios::out | std::ios::trunc);
        if (!file.is_open())
#endif
        {
#if defined(ARDUINO_TEENSY41)
            if (file)
            {
                file.close();
            }
#endif
            return false;
        }

        auto failWrite = [&file]() -> bool
        {
#if defined(ARDUINO_TEENSY41)
            file.close();
#endif
            return false;
        };

        auto writeText = [&file](const char* const text) -> bool
        {
            if (text == nullptr)
            {
                return false;
            }

#if defined(ARDUINO_TEENSY41)
            const std::size_t length = std::strlen(text);
            return file.write(reinterpret_cast<const std::uint8_t*>(text), length) == length;
#else
            file << text;
            return file.good();
#endif
        };

        auto writeChar = [&file](const char value) -> bool
        {
#if defined(ARDUINO_TEENSY41)
            const std::uint8_t byte = static_cast<std::uint8_t>(value);
            return file.write(&byte, 1U) == 1U;
#else
            file.put(value);
            return file.good();
#endif
        };

        for (uint8_t y = 0U; y < maze.GetYSize(); ++y)
        {
            if (!writeText("\""))
            {
                return failWrite();
            }

            for (uint8_t x = 0U; x < maze.GetXSize(); ++x)
            {
                const CharBlock cell = maze.Index(x, y).Serialize();
                if (!writeChar(cell.chars[0]) ||
                    !writeChar(cell.chars[1]) ||
                    !writeChar(cell.chars[2]) ||
                    !writeChar(cell.chars[3]))
                {
                    return failWrite();
                }

                if (x + 1U >= maze.GetXSize())
                {
                    if (!writeText("\\n\"\n"))
                    {
                        return failWrite();
                    }
                }
                else if (!writeText(","))
                {
                    return failWrite();
                }
            }
        }

#if defined(ARDUINO_TEENSY41)
        const std::uint64_t logicalLength = file.curPosition();
        const bool ok =
            file.seekSet(logicalLength) &&
            file.truncate() &&
            file.sync();
        file.close();
        return ok;
#else
        file.flush();
        return file.good();
#endif
    }
}
