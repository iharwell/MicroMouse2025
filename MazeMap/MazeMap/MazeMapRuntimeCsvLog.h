#pragma once

#include "MazeMapRuntimeMmLog.h"

#include <cstddef>
#include <cstdint>

namespace MazeMap::App::Internal::Runtime
{
    // Shared structured event writer used by the application loggers for sidecar-based .mmlog event streams.
    class RuntimeEventLogFile
    {
    public:
        RuntimeEventLogFile() noexcept;

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
        static constexpr uint32_t kEventFieldCount = 6U;
        static constexpr const char* kEventSchema =
            "u32_seq,u32_t_us,u32_phase_id,s32_row_kind,s32_name,s32_message";

        bool EnsureLogOpen();
        bool AppendMetadataLine(const char* line);

        RuntimeBinaryLogFile _log;
        RuntimeTextBlockBuilder<2048U> _metadata;
        char _fileName[64];
        unsigned long _lastFlushMs;
        unsigned long _rowCount;
        bool _begun;
        bool _opened;
    };
}

