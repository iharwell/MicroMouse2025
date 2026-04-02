#pragma once
// Declares the binary mission-log file format and runtime helpers that write MMLOG-compatible telemetry streams.

#include "MmLog.h"

#if defined(ARDUINO_TEENSY41)
#include <SD.h>
#else
#include <fstream>
#endif

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <stdio.h>

namespace MazeMap::App::Internal::Runtime
{
    enum class RuntimeBinaryLogWriteMode : uint8_t
    {
        Synchronous = 0u,
        Buffered = 1u
    };

    // Keep write cost on the calling path by default; buffered mode remains available as an explicit opt-in.
    inline constexpr RuntimeBinaryLogWriteMode kDefaultRuntimeBinaryLogWriteMode =
        RuntimeBinaryLogWriteMode::Synchronous;

    class CoreBinaryFileExport
    {
    public:
        CoreBinaryFileExport() = default;

        ~CoreBinaryFileExport()
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
            return static_cast<bool>(_file);
#else
            _file.open(fileName, std::ios::out | std::ios::binary | std::ios::trunc);
            return _file.is_open();
#endif
        }

        bool IsOpen()
        {
#if defined(ARDUINO_TEENSY41)
            return static_cast<bool>(_file);
#else
            return _file.is_open();
#endif
        }

        size_t WriteBytes(const void* data, size_t size)
        {
            if (!IsOpen() || data == nullptr)
            {
                return 0u;
            }

            if (size == 0u)
            {
                return 0u;
            }

#if defined(ARDUINO_TEENSY41)
            return _file.write(reinterpret_cast<const uint8_t*>(data), size);
#else
            _file.write(reinterpret_cast<const char*>(data), static_cast<std::streamsize>(size));
            return _file.good() ? size : 0u;
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

    inline bool SelectSequentialRuntimeFileName(
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
        snprintf(buffer, bufferSize, "%s", (hostFallback != nullptr) ? hostFallback : "runtime_log.mmlog");
        return true;
#endif
    }

    inline bool BuildSiblingRuntimeFileName(
        char* buffer,
        std::size_t bufferSize,
        const char* fileName,
        const char* siblingSuffix)
    {
        if (buffer == nullptr ||
            bufferSize == 0U ||
            fileName == nullptr ||
            fileName[0] == '\0' ||
            siblingSuffix == nullptr ||
            siblingSuffix[0] == '\0')
        {
            return false;
        }

        const char* extension = std::strrchr(fileName, '.');
        const size_t baseLength = (extension != nullptr) ? static_cast<size_t>(extension - fileName) : std::strlen(fileName);
        const size_t suffixLength = std::strlen(siblingSuffix);
        if ((baseLength + suffixLength + 1U) > bufferSize)
        {
            return false;
        }

        std::memcpy(buffer, fileName, baseLength);
        std::memcpy(buffer + baseLength, siblingSuffix, suffixLength);
        buffer[baseLength + suffixLength] = '\0';
        return true;
    }

    inline const char* FileNameComponent(const char* path) noexcept
    {
        if (path == nullptr)
        {
            return "";
        }

        const char* name = path;
        for (const char* cursor = path; *cursor != '\0'; ++cursor)
        {
            if (*cursor == '/' || *cursor == '\\')
            {
                name = cursor + 1;
            }
        }
        return name;
    }

    inline uint32_t Fnv1a32(const char* text) noexcept
    {
        uint32_t hash = 2166136261u;
        if (text == nullptr)
        {
            return hash;
        }

        for (const uint8_t* cursor = reinterpret_cast<const uint8_t*>(text); *cursor != 0u; ++cursor)
        {
            hash ^= *cursor;
            hash *= 16777619u;
        }
        return hash;
    }

    inline bool IsSchemaPrefix(const char* prefix, std::size_t length) noexcept
    {
        switch (length)
        {
        case 2U:
            return
                (std::strncmp(prefix, "u8", 2U) == 0) ||
                (std::strncmp(prefix, "i8", 2U) == 0) ||
                (std::strncmp(prefix, "s8", 2U) == 0);
        case 3U:
            return
                (std::strncmp(prefix, "u16", 3U) == 0) ||
                (std::strncmp(prefix, "i16", 3U) == 0) ||
                (std::strncmp(prefix, "u32", 3U) == 0) ||
                (std::strncmp(prefix, "i32", 3U) == 0) ||
                (std::strncmp(prefix, "f32", 3U) == 0) ||
                (std::strncmp(prefix, "s16", 3U) == 0) ||
                (std::strncmp(prefix, "s32", 3U) == 0);
        default:
            return false;
        }
    }

    inline uint32_t SchemaFieldWidth(const char* prefix, std::size_t length) noexcept
    {
        if (length == 2U)
        {
            return 1U;
        }

        if (length == 3U &&
            ((std::strncmp(prefix, "u16", 3U) == 0) ||
             (std::strncmp(prefix, "i16", 3U) == 0) ||
             (std::strncmp(prefix, "s16", 3U) == 0)))
        {
            return 2U;
        }

        return 4U;
    }

    inline bool SchemaPrefixUsesStringHash(const char* prefix, std::size_t length) noexcept
    {
        return
            ((length == 2U) && (std::strncmp(prefix, "s8", 2U) == 0)) ||
            ((length == 3U) && ((std::strncmp(prefix, "s16", 3U) == 0) || (std::strncmp(prefix, "s32", 3U) == 0)));
    }

    inline bool ValidateTypedSchema(
        const char* schemaCsv,
        uint32_t expectedFieldCount,
        uint32_t expectedRowBytes,
        bool& hasStringHashField) noexcept
    {
        hasStringHashField = false;
        if (schemaCsv == nullptr || schemaCsv[0] == '\0')
        {
            return false;
        }

        uint32_t fieldCount = 0U;
        uint32_t rowBytes = 0U;
        const char* fieldStart = schemaCsv;
        for (const char* cursor = schemaCsv;; ++cursor)
        {
            if (*cursor != ',' && *cursor != '\0')
            {
                continue;
            }

            if (cursor == fieldStart)
            {
                return false;
            }

            const char* separator = fieldStart;
            while (separator < cursor && *separator != '_')
            {
                ++separator;
            }

            if (separator == fieldStart || separator == cursor)
            {
                return false;
            }

            const std::size_t prefixLength = static_cast<std::size_t>(separator - fieldStart);
            if (!IsSchemaPrefix(fieldStart, prefixLength))
            {
                return false;
            }

            rowBytes += SchemaFieldWidth(fieldStart, prefixLength);
            hasStringHashField = hasStringHashField || SchemaPrefixUsesStringHash(fieldStart, prefixLength);
            ++fieldCount;

            if (*cursor == '\0')
            {
                break;
            }
            fieldStart = cursor + 1;
        }

        return (fieldCount == expectedFieldCount) && (rowBytes == expectedRowBytes);
    }

    inline uint32_t PackTextTagOrHash(const char* text) noexcept
    {
        if (text == nullptr || text[0] == '\0')
        {
            return 0u;
        }

        char tag[4] = { '\0', '\0', '\0', '\0' };
        size_t length = 0U;
        while (text[length] != '\0')
        {
            if (length < 4U)
            {
                tag[length] = text[length];
            }
            ++length;
        }

        if (length <= 4U)
        {
            return mmlog::TAG4(tag[0], tag[1], tag[2], tag[3]);
        }

        return Fnv1a32(text);
    }

    template <std::size_t MaxBytes = 8192U>
    class RuntimeTextBlockBuilder
    {
    public:
        RuntimeTextBlockBuilder()
            : _buffer{}
            , _length(0U)
        {
            _buffer[0] = '\0';
        }

        void Clear()
        {
            _length = 0U;
            _buffer[0] = '\0';
        }

        bool AppendLine(const char* line)
        {
            if (line == nullptr)
            {
                return false;
            }

            const size_t lineLength = std::strlen(line);
            if ((_length + lineLength + 1U) >= _buffer.size())
            {
                return false;
            }

            std::memcpy(_buffer.data() + _length, line, lineLength);
            _length += lineLength;
            _buffer[_length++] = '\n';
            _buffer[_length] = '\0';
            return true;
        }

        bool AppendKeyValue(const char* key, const char* value)
        {
            char line[192] = {};
            const int length = snprintf(
                line,
                sizeof(line),
                "%s=%s",
                (key != nullptr) ? key : "",
                (value != nullptr) ? value : "");
            return AppendFormattedLine(line, length);
        }

        bool AppendUnsigned(const char* key, unsigned long value)
        {
            char line[192] = {};
            const int length = snprintf(
                line,
                sizeof(line),
                "%s=%lu",
                (key != nullptr) ? key : "",
                value);
            return AppendFormattedLine(line, length);
        }

        bool AppendFloat(const char* key, float value, uint8_t precision)
        {
            char line[192] = {};
            const int length = snprintf(
                line,
                sizeof(line),
                "%s=%.*f",
                (key != nullptr) ? key : "",
                static_cast<int>(precision),
                value);
            return AppendFormattedLine(line, length);
        }

        const char* Data() const noexcept
        {
            return (_length > 0U) ? _buffer.data() : nullptr;
        }

    private:
        bool AppendFormattedLine(const char* line, int length)
        {
            if (length <= 0 || length >= static_cast<int>(sizeof(char) * 192))
            {
                return false;
            }
            return AppendLine(line);
        }

        std::array<char, MaxBytes + 1U> _buffer;
        size_t _length;
    };

    template <std::size_t FieldCount>
    class RuntimeRecordBuilder
    {
    public:
        RuntimeRecordBuilder()
            : _words{}
            , _index(0U)
        {
            _words.fill(0u);
        }

        void Clear()
        {
            _words.fill(0u);
            _index = 0U;
        }

        void U32(uint32_t value)
        {
            if (_index < FieldCount)
            {
                _words[_index++] = mmlog::packU32(value);
            }
        }

        void I32(int32_t value)
        {
            if (_index < FieldCount)
            {
                _words[_index++] = mmlog::packI32(value);
            }
        }

        void F32(float value)
        {
            if (_index < FieldCount)
            {
                _words[_index++] = mmlog::packF32(value);
            }
        }

        const uint32_t* Data() const noexcept
        {
            return _words.data();
        }

        uint32_t Count() const noexcept
        {
            return static_cast<uint32_t>(FieldCount);
        }

        bool IsFull() const noexcept
        {
            return _index == FieldCount;
        }

    private:
        std::array<uint32_t, FieldCount> _words;
        std::size_t _index;
    };

    class RuntimeBinaryLogFile
    {
    public:
        RuntimeBinaryLogFile() noexcept
            : _file()
            , _sidecarFile()
            , _blocks(nullptr)
            , _activeIndex(-1)
            , _fieldCount(0u)
            , _recordBytes(0u)
            , _blockBytes(0u)
            , _bufferCount(0u)
            , _writeMode(kDefaultRuntimeBinaryLogWriteMode)
            , _isOpen(false)
            , _overflowed(false)
            , _writeFailed(false)
            , _queuedBlockCount(0u)
        {
            _fileName[0] = '\0';
            _sidecarName[0] = '\0';
        }

        ~RuntimeBinaryLogFile()
        {
            Close();
        }

        bool BeginSelected(
            const char* fileName,
            const char* schemaCsv,
            uint32_t fieldCount,
            const char* metadataKv,
            const char* notes,
            uint32_t flags = mmlog::FLAG_HAS_METADATA | mmlog::FLAG_HAS_SEQ | mmlog::FLAG_HAS_T_US,
            uint32_t runId = 0u,
            uint64_t startTimeUs = 0u,
            RuntimeBinaryLogWriteMode writeMode = kDefaultRuntimeBinaryLogWriteMode,
            size_t blockBytes = 4096u,
            uint8_t bufferCount = 4u)
        {
            Close();
            if (fileName == nullptr ||
                fileName[0] == '\0' ||
                schemaCsv == nullptr ||
                schemaCsv[0] == '\0' ||
                fieldCount == 0u)
            {
                return false;
            }

            snprintf(_fileName, sizeof(_fileName), "%s", fileName);
            _fieldCount = fieldCount;
            _recordBytes = fieldCount * sizeof(uint32_t);
            _blockBytes = (writeMode == RuntimeBinaryLogWriteMode::Buffered) ? blockBytes : 0u;
            _bufferCount = (writeMode == RuntimeBinaryLogWriteMode::Buffered) ? bufferCount : 0u;
            _writeMode = writeMode;
            _overflowed = false;
            _writeFailed = false;
            _queuedBlockCount = 0u;

            if ((_writeMode == RuntimeBinaryLogWriteMode::Buffered) && !AllocateBlocks())
            {
                return false;
            }

            if (!_file.Open(_fileName))
            {
                FreeBlocks();
                return false;
            }

            if (!WriteHeaderAndDescriptors(schemaCsv, metadataKv, notes, flags, runId, startTimeUs))
            {
                _file.Close();
                _sidecarFile.Close();
                FreeBlocks();
                _fileName[0] = '\0';
                _sidecarName[0] = '\0';
                return false;
            }

            _isOpen = true;
            return true;
        }

        bool AppendRecord(const uint32_t* words, uint32_t fieldCount)
        {
            if (!_isOpen || words == nullptr || fieldCount != _fieldCount)
            {
                return false;
            }

            if (_writeMode == RuntimeBinaryLogWriteMode::Synchronous)
            {
                if (_file.WriteBytes(words, _recordBytes) != _recordBytes)
                {
                    _writeFailed = true;
                    return false;
                }
                return true;
            }

            if (_blocks == nullptr || _activeIndex < 0)
            {
                return false;
            }

            Block& active = _blocks[_activeIndex];
            if ((active.used + _recordBytes) > _blockBytes)
            {
                if (!EnqueueActiveAndRotate())
                {
                    _overflowed = true;
                    return false;
                }
            }

            Block& destination = _blocks[_activeIndex];
            std::memcpy(destination.data + destination.used, words, _recordBytes);
            destination.used += _recordBytes;
            return true;
        }

        bool Service(uint32_t maxBlocks = 1u)
        {
            if (!_isOpen)
            {
                return false;
            }

            if (_writeMode == RuntimeBinaryLogWriteMode::Synchronous)
            {
                return true;
            }

            const uint32_t limit = (maxBlocks == 0u) ? 1u : maxBlocks;
            for (uint32_t index = 0u; index < limit; ++index)
            {
                const int queuedIndex = FindQueuedBlockIndex();
                if (queuedIndex < 0)
                {
                    break;
                }
                if (!WriteBlock(queuedIndex))
                {
                    return false;
                }
            }
            return true;
        }

        void Flush()
        {
            if (!_isOpen)
            {
                return;
            }

            if (_writeMode == RuntimeBinaryLogWriteMode::Buffered)
            {
                while (FindQueuedBlockIndex() >= 0)
                {
                    if (!Service(static_cast<uint32_t>(_bufferCount)))
                    {
                        break;
                    }
                }
                (void)WriteActivePartialBlock();
            }
            _file.Flush();
            _sidecarFile.Flush();
        }

        void Close()
        {
            if (_isOpen)
            {
                Flush();
                _file.Close();
                _sidecarFile.Close();
            }
            else
            {
                _file.Close();
                _sidecarFile.Close();
            }

            _isOpen = false;
            _fieldCount = 0u;
            _recordBytes = 0u;
            _blockBytes = 0u;
            _bufferCount = 0u;
            _writeMode = kDefaultRuntimeBinaryLogWriteMode;
            _overflowed = false;
            _writeFailed = false;
            _queuedBlockCount = 0u;
            _fileName[0] = '\0';
            _sidecarName[0] = '\0';
            FreeBlocks();
        }

        const char* GetFileName() const noexcept
        {
            return _fileName;
        }

        const char* GetSidecarFileName() const noexcept
        {
            return _sidecarName;
        }

        bool IsOpen() const noexcept
        {
            return _isOpen;
        }

        bool HadOverflow() const noexcept
        {
            return _overflowed;
        }

        bool HadWriteFailure() const noexcept
        {
            return _writeFailed;
        }

        uint32_t InternLabel(const char* text)
        {
            if (text == nullptr || text[0] == '\0')
            {
                return std::numeric_limits<uint32_t>::max();
            }

            const uint32_t hash = Fnv1a32(text);
            if (_sidecarFile.IsOpen())
            {
                const size_t length = std::strlen(text);
                if ((_sidecarFile.WriteBytes(text, length) != length) ||
                    (_sidecarFile.WriteBytes("\n", 1U) != 1U))
                {
                    _writeFailed = true;
                }
            }
            return hash;
        }

    private:
        struct Block
        {
            uint8_t* data = nullptr;
            uint32_t used = 0u;
            bool queued = false;
            bool active = false;
        };

        bool AllocateBlocks()
        {
            if (_bufferCount < 2u || _blockBytes < _recordBytes)
            {
                return false;
            }

            _blocks = new Block[_bufferCount];
            if (_blocks == nullptr)
            {
                return false;
            }

            for (uint8_t index = 0u; index < _bufferCount; ++index)
            {
                _blocks[index].data = static_cast<uint8_t*>(std::malloc(_blockBytes));
                if (_blocks[index].data == nullptr)
                {
                    FreeBlocks();
                    return false;
                }
                _blocks[index].used = 0u;
                _blocks[index].queued = false;
                _blocks[index].active = false;
            }

            _activeIndex = 0;
            _blocks[_activeIndex].active = true;
            return true;
        }

        void FreeBlocks()
        {
            if (_blocks != nullptr)
            {
                for (uint8_t index = 0u; index < _bufferCount; ++index)
                {
                    if (_blocks[index].data != nullptr)
                    {
                        std::free(_blocks[index].data);
                        _blocks[index].data = nullptr;
                    }
                }

                delete[] _blocks;
                _blocks = nullptr;
            }

            _activeIndex = -1;
        }

        bool WriteHeaderAndDescriptors(
            const char* schemaCsv,
            const char* metadataKv,
            const char* notes,
            uint32_t flags,
            uint32_t runId,
            uint64_t startTimeUs)
        {
            bool hasStringHashField = false;
            if (!ValidateTypedSchema(schemaCsv, _fieldCount, _recordBytes, hasStringHashField))
            {
                return false;
            }

            if (!BuildSiblingRuntimeFileName(_sidecarName, sizeof(_sidecarName), _fileName, ".sidecar"))
            {
                return false;
            }
            if (!_sidecarFile.Open(_sidecarName))
            {
                return false;
            }

            if (!WriteSidecarLine("schema_version=2"))
            {
                return false;
            }
            if (!WriteUnsignedMetadataLine("row_bytes", static_cast<unsigned long>(_recordBytes)))
            {
                return false;
            }
            if (hasStringHashField && !WriteSidecarLine("string_hash=fnv1a32"))
            {
                return false;
            }
            if ((flags != 0U) && !WriteUnsignedMetadataLine("legacy_flags", static_cast<unsigned long>(flags)))
            {
                return false;
            }
            if ((runId != 0U) && !WriteUnsignedMetadataLine("run_id", static_cast<unsigned long>(runId)))
            {
                return false;
            }
            if ((startTimeUs != 0U) && !WriteUnsigned64MetadataLine("start_time_us", startTimeUs))
            {
                return false;
            }
            if (!WriteSidecarBlock(metadataKv) || !WriteSidecarBlock(notes))
            {
                return false;
            }
            if (!WriteSidecarLine(schemaCsv) || !WriteSidecarLine("LABELS:"))
            {
                return false;
            }

            char bindingLine[96] = {};
            const int bindingLength = snprintf(
                bindingLine,
                sizeof(bindingLine),
                "sidecar_file=%s\n",
                FileNameComponent(_sidecarName));
            if (bindingLength <= 0)
            {
                return false;
            }
            return _file.WriteBytes(bindingLine, static_cast<size_t>(bindingLength)) == static_cast<size_t>(bindingLength);
        }

        bool WriteSidecarLine(const char* line)
        {
            if (line == nullptr)
            {
                return false;
            }

            const size_t length = std::strlen(line);
            return
                (_sidecarFile.WriteBytes(line, length) == length) &&
                (_sidecarFile.WriteBytes("\n", 1U) == 1U);
        }

        bool WriteSidecarBlock(const char* block)
        {
            if (block == nullptr || block[0] == '\0')
            {
                return true;
            }

            const size_t length = std::strlen(block);
            if (_sidecarFile.WriteBytes(block, length) != length)
            {
                return false;
            }

            if (block[length - 1U] != '\n' && (_sidecarFile.WriteBytes("\n", 1U) != 1U))
            {
                return false;
            }
            return true;
        }

        bool WriteUnsignedMetadataLine(const char* key, unsigned long value)
        {
            char line[96] = {};
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
            return WriteSidecarLine(line);
        }

        bool WriteUnsigned64MetadataLine(const char* key, uint64_t value)
        {
            char line[96] = {};
            const int length = snprintf(
                line,
                sizeof(line),
                "%s=%llu",
                (key != nullptr) ? key : "",
                static_cast<unsigned long long>(value));
            if (length <= 0 || length >= static_cast<int>(sizeof(line)))
            {
                return false;
            }
            return WriteSidecarLine(line);
        }

        bool EnqueueActiveAndRotate()
        {
            if (_blocks == nullptr || _activeIndex < 0)
            {
                return false;
            }

            Block& active = _blocks[_activeIndex];
            if (active.used == 0u)
            {
                return true;
            }

            const int freeIndex = FindFreeBlockIndex();
            if (freeIndex < 0)
            {
                return false;
            }

            active.active = false;
            active.queued = true;
            ++_queuedBlockCount;

            _activeIndex = freeIndex;
            _blocks[_activeIndex].active = true;
            _blocks[_activeIndex].queued = false;
            _blocks[_activeIndex].used = 0u;
            return true;
        }

        int FindFreeBlockIndex() const noexcept
        {
            if (_blocks == nullptr)
            {
                return -1;
            }

            for (uint8_t index = 0u; index < _bufferCount; ++index)
            {
                if (!_blocks[index].active && !_blocks[index].queued && (_blocks[index].used == 0u))
                {
                    return static_cast<int>(index);
                }
            }
            return -1;
        }

        int FindQueuedBlockIndex() const noexcept
        {
            if (_blocks == nullptr)
            {
                return -1;
            }

            for (uint8_t index = 0u; index < _bufferCount; ++index)
            {
                if (_blocks[index].queued && !_blocks[index].active)
                {
                    return static_cast<int>(index);
                }
            }
            return -1;
        }

        bool WriteBlock(int blockIndex)
        {
            if (_blocks == nullptr || blockIndex < 0)
            {
                return false;
            }

            Block& block = _blocks[blockIndex];
            if (block.used == 0u)
            {
                block.queued = false;
                return true;
            }

            if (_file.WriteBytes(block.data, block.used) != block.used)
            {
                _writeFailed = true;
                return false;
            }

            block.used = 0u;
            block.queued = false;
            if (_queuedBlockCount > 0u)
            {
                --_queuedBlockCount;
            }
            return true;
        }

        bool WriteActivePartialBlock()
        {
            if (_blocks == nullptr || _activeIndex < 0)
            {
                return false;
            }

            Block& active = _blocks[_activeIndex];
            if (active.used == 0u)
            {
                return true;
            }

            if (_file.WriteBytes(active.data, active.used) != active.used)
            {
                _writeFailed = true;
                return false;
            }

            active.used = 0u;
            return true;
        }

        CoreBinaryFileExport _file;
        CoreBinaryFileExport _sidecarFile;
        Block* _blocks;
        int _activeIndex;
        uint32_t _fieldCount;
        uint32_t _recordBytes;
        size_t _blockBytes;
        uint8_t _bufferCount;
        RuntimeBinaryLogWriteMode _writeMode;
        bool _isOpen;
        bool _overflowed;
        bool _writeFailed;
        uint32_t _queuedBlockCount;
        char _fileName[64];
        char _sidecarName[64];
    };

}

