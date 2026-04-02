#include "MazeMapRuntimeCsvLog.h"

#include <limits>
#include <stdio.h>

namespace MazeMap::App::Internal::Runtime
{
    RuntimeEventLogFile::RuntimeEventLogFile() noexcept
        : _log()
        , _metadata()
        , _fileName{}
        , _lastFlushMs(0UL)
        , _rowCount(0UL)
        , _begun(false)
        , _opened(false)
    {
    }

    bool RuntimeEventLogFile::Begin(const char* explicitFileName, const char* teensyFormat, const char* hostFallback)
    {
        Close();
        if (!SelectSequentialRuntimeFileName(
            _fileName,
            sizeof(_fileName),
            explicitFileName,
            teensyFormat,
            (hostFallback != nullptr) ? hostFallback : "measurement_log.mmlog"))
        {
            return false;
        }

        _metadata.Clear();
        _lastFlushMs = millis();
        _rowCount = 0UL;
        _begun = true;
        _opened = false;
        return true;
    }

    bool RuntimeEventLogFile::Write(const char* line)
    {
        return WriteEvent(micros(), "line", line);
    }

    bool RuntimeEventLogFile::WriteMetadata(const char* key, const char* value)
    {
        char line[192] = {};
        const int length = snprintf(
            line,
            sizeof(line),
            "%s=%s",
            (key != nullptr) ? key : "",
            (value != nullptr) ? value : "");
        if (length <= 0 || length >= static_cast<int>(sizeof(line)))
        {
            return false;
        }
        return AppendMetadataLine(line);
    }

    bool RuntimeEventLogFile::WriteMetadataUnsigned(const char* key, unsigned long value)
    {
        char line[192] = {};
        const int length = snprintf(
            line,
            sizeof(line),
            "%s=%lu",
            (key != nullptr) ? key : "",
            value);
        if (length <= 0 || length >= static_cast<int>(sizeof(line)))
        {
            return false;
        }
        return AppendMetadataLine(line);
    }

    bool RuntimeEventLogFile::WriteMetadataFloat(const char* key, float value, uint8_t precision)
    {
        char line[192] = {};
        const int length = snprintf(
            line,
            sizeof(line),
            "%s=%.*f",
            (key != nullptr) ? key : "",
            static_cast<int>(precision),
            value);
        if (length <= 0 || length >= static_cast<int>(sizeof(line)))
        {
            return false;
        }
        return AppendMetadataLine(line);
    }

    bool RuntimeEventLogFile::WritePhase(unsigned long phaseId, unsigned long timestampUs, const char* name)
    {
        if (!EnsureLogOpen())
        {
            return false;
        }

        RuntimeRecordBuilder<kEventFieldCount> record;
        record.U32(static_cast<uint32_t>(_rowCount));
        record.U32(static_cast<uint32_t>(timestampUs));
        record.U32(static_cast<uint32_t>(phaseId));
        record.U32(_log.InternLabel("phase"));
        record.U32(_log.InternLabel(name));
        record.U32(std::numeric_limits<uint32_t>::max());
        if (!_log.AppendRecord(record.Data(), record.Count()))
        {
            return false;
        }

        ++_rowCount;
        return true;
    }

    bool RuntimeEventLogFile::WriteEvent(unsigned long timestampUs, const char* type, const char* message)
    {
        if (!EnsureLogOpen())
        {
            return false;
        }

        RuntimeRecordBuilder<kEventFieldCount> record;
        record.U32(static_cast<uint32_t>(_rowCount));
        record.U32(static_cast<uint32_t>(timestampUs));
        record.U32(0U);
        record.U32(_log.InternLabel((type != nullptr && type[0] != '\0') ? type : "event"));
        record.U32(std::numeric_limits<uint32_t>::max());
        record.U32(_log.InternLabel(message));
        if (!_log.AppendRecord(record.Data(), record.Count()))
        {
            return false;
        }

        ++_rowCount;
        return true;
    }

    void RuntimeEventLogFile::FlushIfNeeded(bool force, unsigned long flushPeriodMs)
    {
        const unsigned long nowMs = millis();
        if (!force && static_cast<unsigned long>(nowMs - _lastFlushMs) < flushPeriodMs)
        {
            return;
        }

        Flush();
        _lastFlushMs = nowMs;
    }

    void RuntimeEventLogFile::Flush()
    {
        if (_begun && !_opened)
        {
            (void)EnsureLogOpen();
        }
        if (_opened)
        {
            _log.Flush();
        }
        _lastFlushMs = millis();
    }

    void RuntimeEventLogFile::Close()
    {
        if (_begun && !_opened)
        {
            (void)EnsureLogOpen();
        }
        if (_opened)
        {
            _log.Close();
        }

        _metadata.Clear();
        _fileName[0] = '\0';
        _lastFlushMs = 0UL;
        _rowCount = 0UL;
        _begun = false;
        _opened = false;
    }

    const char* RuntimeEventLogFile::GetFileName() const noexcept
    {
        return _fileName;
    }

    bool RuntimeEventLogFile::EnsureLogOpen()
    {
        if (!_begun)
        {
            return false;
        }
        if (_opened)
        {
            return true;
        }

        _opened = _log.BeginSelected(_fileName, kEventSchema, kEventFieldCount, _metadata.Data(), nullptr);
        return _opened;
    }

    bool RuntimeEventLogFile::AppendMetadataLine(const char* line)
    {
        if (_opened)
        {
            return false;
        }
        return _metadata.AppendLine(line);
    }
}

