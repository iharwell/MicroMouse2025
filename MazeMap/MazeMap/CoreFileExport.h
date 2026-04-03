#pragma once

#include "Defines.h"

#if defined(ARDUINO_TEENSY41)
#include <SD.h>
#else
#include <fstream>
#endif

namespace MazeMap
{
    class CoreFileExport
    {
    public:
        CoreFileExport() = default;

        explicit CoreFileExport(const char* fileName)
        {
            Open(fileName);
        }

        ~CoreFileExport()
        {
            Close();
        }

        bool Open(const char* fileName)
        {
            Close();
            if (fileName == nullptr || fileName[0] == '\0')
            {
                return false;
            }

#if defined(ARDUINO_TEENSY41)
            SD.remove(fileName);
            _file = SD.open(fileName, FILE_WRITE);
            return _file ? true : false;
#else
            _file.open(fileName, std::ios::out | std::ios::trunc);
            return _file.is_open();
#endif
        }

        bool IsOpen()
        {
#if defined(ARDUINO_TEENSY41)
            return _file ? true : false;
#else
            return _file.is_open();
#endif
        }

        bool Write(const char* text)
        {
            if (!IsOpen() || text == nullptr)
            {
                return false;
            }

#if defined(ARDUINO_TEENSY41)
            return _file.print(text) > 0U;
#else
            _file << text;
            return _file.good();
#endif
        }

        bool WriteChar(char value)
        {
            if (!IsOpen())
            {
                return false;
            }

#if defined(ARDUINO_TEENSY41)
            return _file.write(static_cast<uint8_t>(value)) == 1U;
#else
            _file.put(value);
            return _file.good();
#endif
        }

        void Flush()
        {
            if (!IsOpen())
            {
                return;
            }

#if defined(ARDUINO_TEENSY41)
            _file.flush();
#else
            _file.flush();
#endif
        }

        void Close()
        {
#if defined(ARDUINO_TEENSY41)
            if (_file)
            {
                _file.close();
            }
#else
            if (_file.is_open())
            {
                _file.close();
            }
#endif
        }

    private:
#if defined(ARDUINO_TEENSY41)
        File _file;
#else
        std::ofstream _file;
#endif
    };
}


