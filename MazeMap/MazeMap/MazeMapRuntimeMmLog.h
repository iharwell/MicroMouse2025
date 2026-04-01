#pragma once

#include "Defines.h"

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

namespace mmlog
{
    static constexpr char kMagic[8] = { 'M', 'M', 'L', 'O', 'G', '1', '\0', '\0' };
    static constexpr uint32_t kVersion = 1u;
    static constexpr uint32_t kHeaderBytes = 64u;

    static constexpr uint32_t FLAG_HAS_CRC32 = 1u << 0;
    static constexpr uint32_t FLAG_HAS_STAGE = 1u << 1;
    static constexpr uint32_t FLAG_HAS_SEQ = 1u << 2;
    static constexpr uint32_t FLAG_HAS_T_US = 1u << 3;
    static constexpr uint32_t FLAG_HAS_DT_US = 1u << 4;
    static constexpr uint32_t FLAG_HAS_METADATA = 1u << 5;
    static constexpr uint32_t FLAG_HAS_NOTES = 1u << 6;
    static constexpr uint32_t FLAG_LITTLE_ENDIAN = 1u << 7;

    constexpr uint32_t TAG4(char a, char b, char c, char d) noexcept
    {
        return (static_cast<uint32_t>(static_cast<uint8_t>(a))) |
            (static_cast<uint32_t>(static_cast<uint8_t>(b)) << 8) |
            (static_cast<uint32_t>(static_cast<uint8_t>(c)) << 16) |
            (static_cast<uint32_t>(static_cast<uint8_t>(d)) << 24);
    }

#pragma pack(push, 1)
    struct FileHeader
    {
        char magic[8];
        uint32_t version;
        uint32_t header_bytes;
        uint32_t record_bytes;
        uint32_t field_count;
        uint32_t metadata_bytes;
        uint32_t schema_bytes;
        uint32_t notes_bytes;
        uint32_t flags;
        uint32_t run_id;
        uint32_t start_time_us_lo;
        uint32_t start_time_us_hi;
        uint32_t reserved[3];
    };
#pragma pack(pop)

    static_assert(sizeof(FileHeader) == 64u, "MMLOG1 FileHeader must be 64 bytes");

    inline uint32_t packU32(uint32_t value) noexcept
    {
        return value;
    }

    inline uint32_t packI32(int32_t value) noexcept
    {
        uint32_t packed = 0u;
        std::memcpy(&packed, &value, sizeof(packed));
        return packed;
    }

    inline uint32_t packF32(float value) noexcept
    {
        uint32_t packed = 0u;
        std::memcpy(&packed, &value, sizeof(packed));
        return packed;
    }

    inline void fillHeader(
        FileHeader& header,
        uint32_t recordBytes,
        uint32_t fieldCount,
        uint32_t metadataBytes,
        uint32_t schemaBytes,
        uint32_t notesBytes,
        uint32_t flags,
        uint32_t runId,
        uint64_t startTimeUs) noexcept
    {
        std::memcpy(header.magic, kMagic, sizeof(kMagic));
        header.version = kVersion;
        header.header_bytes = kHeaderBytes + metadataBytes + schemaBytes + notesBytes;
        header.record_bytes = recordBytes;
        header.field_count = fieldCount;
        header.metadata_bytes = metadataBytes;
        header.schema_bytes = schemaBytes;
        header.notes_bytes = notesBytes;
        header.flags = flags | FLAG_LITTLE_ENDIAN;
        header.run_id = runId;
        header.start_time_us_lo = static_cast<uint32_t>(startTimeUs & 0xFFFFFFFFull);
        header.start_time_us_hi = static_cast<uint32_t>((startTimeUs >> 32) & 0xFFFFFFFFull);
        for (uint32_t& value : header.reserved)
        {
            value = 0u;
        }
    }
}

namespace MazeMapApp::Internal::Runtime
{
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
            std::snprintf(buffer, bufferSize, "%s", explicitFileName);
            return true;
        }

#if defined(ARDUINO_TEENSY41)
        if (teensyFormat == nullptr || teensyFormat[0] == '\0')
        {
            return false;
        }

        for (uint16_t index = 0U; index < 1000U; ++index)
        {
            std::snprintf(buffer, bufferSize, teensyFormat, static_cast<unsigned>(index));
            if (!SD.exists(buffer))
            {
                return true;
            }
        }

        std::snprintf(buffer, bufferSize, teensyFormat, 999U);
        return true;
#else
        (void)teensyFormat;
        std::snprintf(buffer, bufferSize, "%s", (hostFallback != nullptr) ? hostFallback : "runtime_log.mmlog");
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

        uint32_t hash = 2166136261u;
        for (size_t index = 0U; index < length; ++index)
        {
            hash ^= static_cast<uint8_t>(text[index]);
            hash *= 16777619u;
        }
        return hash;
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
            const int length = std::snprintf(
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
            const int length = std::snprintf(
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
            const int length = std::snprintf(
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
            , _blocks(nullptr)
            , _activeIndex(-1)
            , _fieldCount(0u)
            , _recordBytes(0u)
            , _blockBytes(0u)
            , _bufferCount(0u)
            , _isOpen(false)
            , _overflowed(false)
            , _writeFailed(false)
            , _queuedBlockCount(0u)
        {
            _fileName[0] = '\0';
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

            std::snprintf(_fileName, sizeof(_fileName), "%s", fileName);
            _fieldCount = fieldCount;
            _recordBytes = fieldCount * sizeof(uint32_t);
            _blockBytes = blockBytes;
            _bufferCount = bufferCount;
            _overflowed = false;
            _writeFailed = false;
            _queuedBlockCount = 0u;

            if (!AllocateBlocks())
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
                FreeBlocks();
                _fileName[0] = '\0';
                return false;
            }

            _isOpen = true;
            return true;
        }

        bool AppendRecord(const uint32_t* words, uint32_t fieldCount)
        {
            if (!_isOpen || words == nullptr || fieldCount != _fieldCount || _blocks == nullptr || _activeIndex < 0)
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

            while (FindQueuedBlockIndex() >= 0)
            {
                if (!Service(static_cast<uint32_t>(_bufferCount)))
                {
                    break;
                }
            }
            (void)WriteActivePartialBlock();
            _file.Flush();
        }

        void Close()
        {
            if (_isOpen)
            {
                Flush();
                _file.Close();
            }

            _isOpen = false;
            _fieldCount = 0u;
            _recordBytes = 0u;
            _blockBytes = 0u;
            _bufferCount = 0u;
            _overflowed = false;
            _writeFailed = false;
            _queuedBlockCount = 0u;
            _fileName[0] = '\0';
            FreeBlocks();
        }

        const char* GetFileName() const noexcept
        {
            return _fileName;
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
            const uint32_t metadataBytes = (metadataKv != nullptr) ? static_cast<uint32_t>(std::strlen(metadataKv)) : 0u;
            const uint32_t schemaBytes = static_cast<uint32_t>(std::strlen(schemaCsv));
            const uint32_t notesBytes = (notes != nullptr) ? static_cast<uint32_t>(std::strlen(notes)) : 0u;
            if (schemaBytes == 0u)
            {
                return false;
            }

            mmlog::FileHeader header{};
            mmlog::fillHeader(
                header,
                _recordBytes,
                _fieldCount,
                metadataBytes,
                schemaBytes,
                notesBytes,
                flags,
                runId,
                startTimeUs);

            if (_file.WriteBytes(&header, sizeof(header)) != sizeof(header))
            {
                return false;
            }
            if ((metadataBytes > 0u) && (_file.WriteBytes(metadataKv, metadataBytes) != metadataBytes))
            {
                return false;
            }
            if (_file.WriteBytes(schemaCsv, schemaBytes) != schemaBytes)
            {
                return false;
            }
            if ((notesBytes > 0u) && (_file.WriteBytes(notes, notesBytes) != notesBytes))
            {
                return false;
            }
            return true;
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
        Block* _blocks;
        int _activeIndex;
        uint32_t _fieldCount;
        uint32_t _recordBytes;
        size_t _blockBytes;
        uint8_t _bufferCount;
        bool _isOpen;
        bool _overflowed;
        bool _writeFailed;
        uint32_t _queuedBlockCount;
        char _fileName[64];
    };
}
