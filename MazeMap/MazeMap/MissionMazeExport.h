#pragma once

#include "CoreFileExport.h"
#include "Maze.h"

namespace MazeMap
{
    inline bool ExportMazeSnapshot(const Maze& maze, const char* fileName)
    {
        if (fileName == nullptr || fileName[0] == '\0')
        {
            return false;
        }

        CoreFileExport file(fileName);
        if (!file.IsOpen())
        {
            return false;
        }

        for (uint8_t y = 0U; y < maze.GetYSize(); ++y)
        {
            if (!file.Write("\""))
            {
                return false;
            }

            for (uint8_t x = 0U; x < maze.GetXSize(); ++x)
            {
                const CharBlock cell = maze.Index(x, y).Serialize();
                if (!file.WriteChar(cell.chars[0]) ||
                    !file.WriteChar(cell.chars[1]) ||
                    !file.WriteChar(cell.chars[2]) ||
                    !file.WriteChar(cell.chars[3]))
                {
                    return false;
                }

                if (x + 1U >= maze.GetXSize())
                {
                    if (!file.Write("\\n\"\n"))
                    {
                        return false;
                    }
                }
                else if (!file.Write(","))
                {
                    return false;
                }
            }
        }

        file.Flush();
        return true;
    }
}
