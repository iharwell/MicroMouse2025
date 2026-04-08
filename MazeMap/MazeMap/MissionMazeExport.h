#pragma once

#include "Defines.h"
#include "Maze.h"

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
        if (!file)
#else
        std::ofstream file(fileName, std::ios::binary | std::ios::out | std::ios::trunc);
        if (!file.is_open())
#endif
        {
            return false;
        }

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
                return false;
            }

            for (uint8_t x = 0U; x < maze.GetXSize(); ++x)
            {
                const CharBlock cell = maze.Index(x, y).Serialize();
                if (!writeChar(cell.chars[0]) ||
                    !writeChar(cell.chars[1]) ||
                    !writeChar(cell.chars[2]) ||
                    !writeChar(cell.chars[3]))
                {
                    return false;
                }

                if (x + 1U >= maze.GetXSize())
                {
                    if (!writeText("\\n\"\n"))
                    {
                        return false;
                    }
                }
                else if (!writeText(","))
                {
                    return false;
                }
            }
        }

#if defined(ARDUINO_TEENSY41)
        const bool ok = file.sync();
        file.close();
        return ok;
#else
        file.flush();
        return file.good();
#endif
    }
}
