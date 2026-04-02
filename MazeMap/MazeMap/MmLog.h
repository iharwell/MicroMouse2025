#pragma once
// Declares the MMLOG binary file-format constants, packed headers, and scalar packing helpers shared by runtime log writers.

#include "Defines.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <limits>

namespace MazeMap::MmLog
{
    static constexpr char kMagic[8] = { 'M', 'M', 'L', 'O', 'G', '1', '\0', '\0' };
    static constexpr uint32_t kVersion = 1u;
    static constexpr uint32_t kHeaderBytes = 64u;
    static constexpr uint32_t kGenericMagic = 0x474C4D4Du;
    static constexpr uint16_t kGenericVersionMajor = 2u;
    static constexpr uint16_t kGenericVersionMinor = 0u;

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

    struct GenericFileHeader
    {
        uint32_t magic;
        uint16_t fmt_ver_major;
        uint16_t fmt_ver_minor;
        uint32_t stream_type;
        uint32_t schema_id;
        uint32_t flags;
        uint32_t producer_id;
        uint32_t header_bytes;
        uint32_t metadata_bytes;
        uint32_t notes_bytes;
        uint32_t reserved0;
        uint32_t reserved1;
    };

    struct LogRecordHeader
    {
        uint32_t rec_type;
        uint16_t rec_size;
        uint16_t rec_ver;
    };
#pragma pack(pop)

    static_assert(sizeof(FileHeader) == 64u, "MMLOG1 FileHeader must be 64 bytes");
    static_assert(sizeof(GenericFileHeader) == 44u, "MMLG GenericFileHeader must be 44 bytes");
    static_assert(sizeof(LogRecordHeader) == 8u, "MMLG LogRecordHeader must be 8 bytes");

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

    inline void fillGenericHeader(
        GenericFileHeader& header,
        uint32_t streamType,
        uint32_t schemaId,
        uint32_t metadataBytes,
        uint32_t notesBytes,
        uint32_t flags,
        uint32_t producerId) noexcept
    {
        header.magic = kGenericMagic;
        header.fmt_ver_major = kGenericVersionMajor;
        header.fmt_ver_minor = kGenericVersionMinor;
        header.stream_type = streamType;
        header.schema_id = schemaId;
        header.flags = flags | FLAG_LITTLE_ENDIAN;
        header.producer_id = producerId;
        header.header_bytes = static_cast<uint32_t>(sizeof(GenericFileHeader)) + metadataBytes + notesBytes;
        header.metadata_bytes = metadataBytes;
        header.notes_bytes = notesBytes;
        header.reserved0 = 0u;
        header.reserved1 = 0u;
    }
}

namespace mmlog = MazeMap::MmLog;
