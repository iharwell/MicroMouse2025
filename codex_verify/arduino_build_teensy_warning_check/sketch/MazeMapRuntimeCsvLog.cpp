#line 1 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMapRuntimeCsvLog.cpp"
#include "MazeMapRuntimeCsvLog.h"

#include "DiagnosticLogBudget.h"

#include <stdio.h>

namespace MazeMapApp::Internal::Runtime
{
    bool SelectSequentialCsvFileName(
        char* buffer,
        std::size_t bufferSize,
        const char* explicitFileName,
        const char* teensyFormat,
        const char* hostFallback)
    {
        if (buffer == nullptr || bufferSize == 0U)
        {
            return false;
        }

        if (explicitFileName != nullptr && explicitFileName[0] != '\0')
        {
            snprintf(buffer, bufferSize, "%s", explicitFileName);
            return true;
        }

#if defined(ARDUINO_TEENSY41)
        if (teensyFormat == nullptr || teensyFormat[0] == '\0')
        {
            return false;
        }

        for (uint16_t index = 0U; index < 1000U; ++index)
        {
            snprintf(buffer, bufferSize, teensyFormat, static_cast<unsigned>(index));
            if (!SD.exists(buffer))
            {
                return true;
            }
        }

        snprintf(buffer, bufferSize, teensyFormat, 999U);
        return true;
#else
        (void)teensyFormat;
        snprintf(buffer, bufferSize, "%s", (hostFallback != nullptr) ? hostFallback : "measurement_log.csv");
        return true;
#endif
    }

    RuntimeCsvLogFile::RuntimeCsvLogFile() noexcept
        : _file()
        , _fileName{}
        , _lastFlushMs(0UL)
    {
    }

    bool RuntimeCsvLogFile::Begin(const char* explicitFileName, const char* teensyFormat, const char* hostFallback)
    {
        if (!SelectSequentialCsvFileName(_fileName, sizeof(_fileName), explicitFileName, teensyFormat, hostFallback))
        {
            return false;
        }
        if (!_file.Open(_fileName))
        {
            return false;
        }

        _lastFlushMs = millis();
        return true;
    }

    bool RuntimeCsvLogFile::Write(const char* line)
    {
        return _file.Write(line);
    }

    bool RuntimeCsvLogFile::WriteMetadata(const char* key, const char* value)
    {
        char line[128] = {};
        const int length = snprintf(
            line,
            sizeof(line),
            "# meta,%s,%s\n",
            (key != nullptr) ? key : "",
            (value != nullptr) ? value : "");

        if (length <= 0 || length >= static_cast<int>(sizeof(line)))
        {
            return false;
        }
        return Write(line);
    }

    bool RuntimeCsvLogFile::WriteMetadataUnsigned(const char* key, unsigned long value)
    {
        char line[128] = {};
        const int length = snprintf(
            line,
            sizeof(line),
            "# meta,%s,%lu\n",
            (key != nullptr) ? key : "",
            value);

        if (length <= 0 || length >= static_cast<int>(sizeof(line)))
        {
            return false;
        }
        return Write(line);
    }

    bool RuntimeCsvLogFile::WriteMetadataFloat(const char* key, float value, uint8_t precision)
    {
        char line[128] = {};
        const int length = snprintf(
            line,
            sizeof(line),
            "# meta,%s,%.*f\n",
            (key != nullptr) ? key : "",
            static_cast<int>(precision),
            value);
        if (length <= 0 || length >= static_cast<int>(sizeof(line)))
        {
            return false;
        }
        return Write(line);
    }

    bool RuntimeCsvLogFile::WritePhase(unsigned long phaseId, unsigned long timestampUs, const char* name)
    {
        char line[160] = {};
        const int length = snprintf(
            line,
            sizeof(line),
            "# phase,%lu,%lu,%s\n",
            phaseId,
            timestampUs,
            (name != nullptr) ? name : "");

        if (length <= 0 || length >= static_cast<int>(sizeof(line)))
        {
            return false;
        }
        return Write(line);
    }

    bool RuntimeCsvLogFile::WriteEvent(unsigned long timestampUs, const char* type, const char* message)
    {
        char line[MazeMap::kDiagnosticEventLineCapacity] = {};
        const int length = snprintf(
            line,
            sizeof(line),
            "# event,%lu,%s,%s\n",
            timestampUs,
            (type != nullptr) ? type : "",
            (message != nullptr) ? message : "");

        if (length <= 0 || length >= static_cast<int>(sizeof(line)))
        {
            return false;
        }
        return Write(line);
    }

    void RuntimeCsvLogFile::FlushIfNeeded(bool force, unsigned long flushPeriodMs)
    {
        const unsigned long nowMs = millis();
        if (force || static_cast<unsigned long>(nowMs - _lastFlushMs) >= flushPeriodMs)
        {
            _file.Flush();
            _lastFlushMs = nowMs;
        }
    }

    void RuntimeCsvLogFile::Flush()
    {
        _file.Flush();
        _lastFlushMs = millis();
    }

    void RuntimeCsvLogFile::Close()
    {
        _file.Close();
    }

    const char* RuntimeCsvLogFile::GetFileName() const noexcept
    {
        return _fileName;
    }
}
