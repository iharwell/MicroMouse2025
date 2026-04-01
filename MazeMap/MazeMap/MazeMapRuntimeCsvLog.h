#pragma once

#include "CoreFileExport.h"

#include <cstddef>
#include <cstdint>

namespace MazeMapApp::Internal::Runtime
{
    // Chooses a CSV file name for a runtime log using either an explicit name, the next Teensy slot, or a host fallback.
    EXPORT bool SelectSequentialCsvFileName(
        char* buffer,
        std::size_t bufferSize,
        const char* explicitFileName,
        const char* teensyFormat,
        const char* hostFallback);

    // Shared CSV writer used by the application loggers to keep file naming, metadata, and phase/event formatting consistent.
    class RuntimeCsvLogFile
    {
    public:
        RuntimeCsvLogFile() noexcept;

        bool Begin(const char* explicitFileName, const char* teensyFormat, const char* hostFallback);
        bool Write(const char* line);
        bool WriteMetadata(const char* key, const char* value);
        bool WriteMetadataUnsigned(const char* key, unsigned long value);
        bool WriteMetadataFloat(const char* key, float value, uint8_t precision);
        bool WritePhase(unsigned long phaseId, unsigned long timestampUs, const char* name);
        bool WriteEvent(unsigned long timestampUs, const char* type, const char* message);
        void FlushIfNeeded(bool force, unsigned long flushPeriodMs);
        void Flush();
        void Close();
        const char* GetFileName() const noexcept;

    private:
        MazeMap::CoreFileExport _file;
        char _fileName[64];
        unsigned long _lastFlushMs;
    };
}
