#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <type_traits>

#if defined(ARDUINO)
#  include <Arduino.h>
#  include <FS.h>
#  include <SD.h>
#endif

#if !defined(MMLOG_ENABLE_TEENSY_FIFO_SDIO)
#  if defined(CORE_TEENSY) && defined(__IMXRT1062__) && defined(BUILTIN_SDCARD)
#    define MMLOG_ENABLE_TEENSY_FIFO_SDIO 1
#  else
#    define MMLOG_ENABLE_TEENSY_FIFO_SDIO 0
#  endif
#endif

#if MMLOG_ENABLE_TEENSY_FIFO_SDIO
#  ifndef DISABLE_FS_H_WARNING
#    define DISABLE_FS_H_WARNING
#  endif
#  include <SdFat.h>
#endif
namespace MazeMap {
    /**
     * Application configuration knobs.
     *
     * Each MMLOG_* macro in this section may be overridden before including this header
     * to tailor the logger to a particular target, queue budget, or file-system policy.
     * These are compile-time constants; the class never resizes buffers or allocates memory
     * dynamically at runtime.
     */
     /** Maximum length, excluding the null terminator, accepted for a primary or sidecar path. */
#ifndef MMLOG_MAX_PATH_LENGTH
#  define MMLOG_MAX_PATH_LENGTH 128u
#endif

/** Maximum number of application metadata key/value pairs accepted before begin(). */
#ifndef MMLOG_METADATA_MAX_ENTRIES
#  define MMLOG_METADATA_MAX_ENTRIES 16u
#endif

/** Maximum metadata-key length, excluding the null terminator. */
#ifndef MMLOG_METADATA_KEY_MAX_LENGTH
#  define MMLOG_METADATA_KEY_MAX_LENGTH 63u
#endif

/** Maximum metadata-value length, excluding the null terminator. */
#ifndef MMLOG_METADATA_VALUE_MAX_LENGTH
#  define MMLOG_METADATA_VALUE_MAX_LENGTH 95u
#endif

/** Primary-file ring-buffer capacity in bytes. */
#ifndef MMLOG_PRIMARY_QUEUE_BYTES
#  define MMLOG_PRIMARY_QUEUE_BYTES 262144u
#endif

/** Sidecar-file ring-buffer capacity in bytes. */
#ifndef MMLOG_SIDECAR_QUEUE_BYTES
#  define MMLOG_SIDECAR_QUEUE_BYTES 4096u
#endif

/** Maximum number of primary-file bytes that one service() call may attempt to drain. */
#ifndef MMLOG_SERVICE_PRIMARY_BUDGET_BYTES
#  define MMLOG_SERVICE_PRIMARY_BUDGET_BYTES 512u
#endif

/** Maximum number of sidecar-file bytes that one service() call may attempt to drain. */
#ifndef MMLOG_SERVICE_SIDECAR_BUDGET_BYTES
#  define MMLOG_SERVICE_SIDECAR_BUDGET_BYTES 512u
#endif

/** Maximum label-string length, excluding the null terminator, accepted by writeLabel(). */
#ifndef MMLOG_MAX_LABEL_LENGTH
#  define MMLOG_MAX_LABEL_LENGTH 96u
#endif

/** Maximum stored length of the last error string, excluding the null terminator. */
#ifndef MMLOG_ERROR_TEXT_LENGTH
#  define MMLOG_ERROR_TEXT_LENGTH 96u
#endif

/** SdFat SDIO configuration used when Teensy FIFO SDIO support is enabled. */
#ifndef MMLOG_TEENSY_SDIO_CONFIG
#  define MMLOG_TEENSY_SDIO_CONFIG SdioConfig(FIFO_SDIO)
#endif

/** Number of bytes to preallocate for the primary file on Teensy FIFO SDIO builds. */
#ifndef MMLOG_TEENSY_PRIMARY_PREALLOCATE_BYTES
#  define MMLOG_TEENSY_PRIMARY_PREALLOCATE_BYTES (64ULL * 1024ULL * 1024ULL)
#endif

/** Number of bytes to preallocate for the sidecar file on Teensy FIFO SDIO builds. */
#ifndef MMLOG_TEENSY_SIDECAR_PREALLOCATE_BYTES
#  define MMLOG_TEENSY_SIDECAR_PREALLOCATE_BYTES 1024ULL*1024ULL
#endif

/** When nonzero, service() writes at most one 512-byte sector per file per call in Teensy FIFO SDIO mode. */
#ifndef MMLOG_TEENSY_SERVICE_AT_MOST_ONE_SECTOR_PER_CALL
#  define MMLOG_TEENSY_SERVICE_AT_MOST_ONE_SECTOR_PER_CALL 1
#endif

#if defined(_MSC_VER)
#  define MMLOG_PACKED_BEGIN __pragma(pack(push, 1))
#  define MMLOG_PACKED_END __pragma(pack(pop))
#  define MMLOG_PACKED
#else
#  define MMLOG_PACKED_BEGIN
#  define MMLOG_PACKED_END
#  ifndef MMLOG_PACKED
#    define MMLOG_PACKED __attribute__((packed))
#  endif
#endif

    namespace mmlog {

        inline constexpr std::uint32_t kSchemaVersion = 2u;
        inline constexpr const char* kPrimaryExtension = ".mmlog";
        inline constexpr const char* kSidecarExtension = ".sidecar";
        inline constexpr const char* kLabelsMarker = "LABELS:\n";
        inline constexpr std::size_t kSdSectorBytes = 512u;
        inline constexpr std::size_t kServiceBlockBytes = 512u;
        inline constexpr std::size_t kLabelsMarkerBytes = 8u;

        static_assert(MMLOG_SERVICE_PRIMARY_BUDGET_BYTES >= kServiceBlockBytes,
            "MMLOG_SERVICE_PRIMARY_BUDGET_BYTES must be at least one 512-byte service block.");
        static_assert(MMLOG_SERVICE_SIDECAR_BUDGET_BYTES >= kServiceBlockBytes,
            "MMLOG_SERVICE_SIDECAR_BUDGET_BYTES must be at least one 512-byte service block.");

#if MMLOG_ENABLE_TEENSY_FIFO_SDIO
        static_assert((MMLOG_PRIMARY_QUEUE_BYTES% kSdSectorBytes) == 0u,
            "MMLOG_PRIMARY_QUEUE_BYTES must be a multiple of 512 when MMLOG_ENABLE_TEENSY_FIFO_SDIO is enabled.");
        static_assert((MMLOG_SIDECAR_QUEUE_BYTES% kSdSectorBytes) == 0u,
            "MMLOG_SIDECAR_QUEUE_BYTES must be a multiple of 512 when MMLOG_ENABLE_TEENSY_FIFO_SDIO is enabled.");
#endif

        // -----------------------------------------------------------------------------
        // String-hash field wrappers
        // -----------------------------------------------------------------------------

        /** Strong typedef for an 8-bit string-hash field declared as s8_* in the sidecar header. */
        enum class s8_t : std::uint8_t {};
        /** Strong typedef for a 16-bit string-hash field declared as s16_* in the sidecar header. */
        enum class s16_t : std::uint16_t {};
        /** Strong typedef for a 32-bit string-hash field declared as s32_* in the sidecar header. */
        enum class s32_t : std::uint32_t {};

        static_assert(sizeof(s8_t) == 1u, "mmlog::s8_t must be exactly 1 byte.");
        static_assert(sizeof(s16_t) == 2u, "mmlog::s16_t must be exactly 2 bytes.");
        static_assert(sizeof(s32_t) == 4u, "mmlog::s32_t must be exactly 4 bytes.");
        static_assert(std::is_trivially_copyable<s8_t>::value, "mmlog::s8_t must be trivially copyable.");
        static_assert(std::is_trivially_copyable<s16_t>::value, "mmlog::s16_t must be trivially copyable.");
        static_assert(std::is_trivially_copyable<s32_t>::value, "mmlog::s32_t must be trivially copyable.");

        // -----------------------------------------------------------------------------
        // Compile-time and runtime helpers
        // -----------------------------------------------------------------------------

        namespace detail {

            enum class FieldKind : std::uint8_t {
                U8,
                I8,
                U16,
                I16,
                U32,
                I32,
                F32,
                S8,
                S16,
                S32,
            };

            struct FieldDescriptor final {
                FieldKind kind;
                const char* name;
                std::size_t width;
            };

            constexpr std::size_t cstrlen(const char* const s) noexcept {
                std::size_t n = 0u;
                while (s[n] != '\0') {
                    ++n;
                }
                return n;
            }

            constexpr std::size_t bounded_cstrlen(const char* const s, const std::size_t limitPlusOne) noexcept {
                if (s == nullptr) {
                    return 0u;
                }

                std::size_t n = 0u;
                while (n < limitPlusOne && s[n] != '\0') {
                    ++n;
                }
                return n;
            }

            constexpr const char* fieldKindToken(const FieldKind kind) noexcept {
                switch (kind) {
                case FieldKind::U8:  return "u8";
                case FieldKind::I8:  return "i8";
                case FieldKind::U16: return "u16";
                case FieldKind::I16: return "i16";
                case FieldKind::U32: return "u32";
                case FieldKind::I32: return "i32";
                case FieldKind::F32: return "f32";
                case FieldKind::S8:  return "s8";
                case FieldKind::S16: return "s16";
                case FieldKind::S32: return "s32";
                default:             return "?";
                }
            }

            template <typename T>
            struct FieldTraits;

            template <> struct FieldTraits<std::uint8_t>  final { static constexpr FieldKind kind = FieldKind::U8;  static constexpr std::size_t token_length = 2u; };
            template <> struct FieldTraits<std::int8_t>   final { static constexpr FieldKind kind = FieldKind::I8;  static constexpr std::size_t token_length = 2u; };
            template <> struct FieldTraits<std::uint16_t> final { static constexpr FieldKind kind = FieldKind::U16; static constexpr std::size_t token_length = 3u; };
            template <> struct FieldTraits<std::int16_t>  final { static constexpr FieldKind kind = FieldKind::I16; static constexpr std::size_t token_length = 3u; };
            template <> struct FieldTraits<std::uint32_t> final { static constexpr FieldKind kind = FieldKind::U32; static constexpr std::size_t token_length = 3u; };
            template <> struct FieldTraits<std::int32_t>  final { static constexpr FieldKind kind = FieldKind::I32; static constexpr std::size_t token_length = 3u; };
            template <> struct FieldTraits<float>         final { static constexpr FieldKind kind = FieldKind::F32; static constexpr std::size_t token_length = 3u; };
            template <> struct FieldTraits<s8_t>          final { static constexpr FieldKind kind = FieldKind::S8;  static constexpr std::size_t token_length = 2u; };
            template <> struct FieldTraits<s16_t>         final { static constexpr FieldKind kind = FieldKind::S16; static constexpr std::size_t token_length = 3u; };
            template <> struct FieldTraits<s32_t>         final { static constexpr FieldKind kind = FieldKind::S32; static constexpr std::size_t token_length = 3u; };

            constexpr void copyChars(char* const dst, std::size_t& pos, const char* const src) noexcept {
                for (std::size_t i = 0u; src[i] != '\0'; ++i) {
                    dst[pos++] = src[i];
                }
            }

            template <std::size_t HeaderChars, std::size_t FieldCount>
            constexpr std::array<char, HeaderChars> buildHeaderArray(const std::array<FieldDescriptor, FieldCount>& descriptors) noexcept {
                std::array<char, HeaderChars> out{};
                std::size_t pos = 0u;

                for (std::size_t i = 0u; i < FieldCount; ++i) {
                    copyChars(out.data(), pos, fieldKindToken(descriptors[i].kind));
                    out[pos++] = '_';
                    copyChars(out.data(), pos, descriptors[i].name);
                    if ((i + 1u) != FieldCount) {
                        out[pos++] = ',';
                    }
                }
                out[pos] = '\0';
                return out;
            }

            constexpr std::uint32_t fnv1a32_impl(const char* const text) noexcept {
                std::uint32_t hash = 2166136261u;
                for (std::size_t i = 0u; text[i] != '\0'; ++i) {
                    hash ^= static_cast<std::uint8_t>(text[i]);
                    hash *= 16777619u;
                }
                return hash;
            }

            template <typename T, typename = void>
            struct has_row_contract : std::false_type {};

            template <typename T>
            struct has_row_contract<
                T,
                std::void_t<
                decltype(T::field_count),
                decltype(T::row_bytes),
                decltype(T::schema_version),
                decltype(T::schema_hash),
                decltype(T::descriptors),
                decltype(T::header_storage),
                decltype(T::header_cstr())>> : std::true_type{};

            class ByteRing final {
            public:
                ByteRing() noexcept = default;

                bool attach(std::uint8_t* const buffer, const std::size_t capacity) noexcept {
                    if ((buffer == nullptr) != (capacity == 0u)) {
                        return false;
                    }
                    m_buffer = buffer;
                    m_capacity = capacity;
                    clear();
                    return true;
                }

                void clear() noexcept {
                    m_head = 0u;
                    m_tail = 0u;
                    m_size = 0u;
                }

                std::size_t capacity() const noexcept { return m_capacity; }
                std::size_t size() const noexcept { return m_size; }
                bool empty() const noexcept { return m_size == 0u; }
                std::size_t freeSpace() const noexcept { return m_capacity - m_size; }

                bool push(const std::uint8_t* const data, const std::size_t len) noexcept {
                    if ((data == nullptr && len != 0u) || len > freeSpace() || m_buffer == nullptr) {
                        return false;
                    }
                    if (len == 0u) {
                        return true;
                    }

                    const std::size_t first = ((m_tail + len) <= m_capacity) ? len : (m_capacity - m_tail);
                    std::memcpy(m_buffer + m_tail, data, first);
                    if (len > first) {
                        std::memcpy(m_buffer, data + first, len - first);
                    }
                    m_tail = (m_tail + len) % m_capacity;
                    m_size += len;
                    return true;
                }

                const std::uint8_t* readPtr() const noexcept {
                    return (m_size == 0u || m_buffer == nullptr) ? nullptr : (m_buffer + m_head);
                }

                std::size_t contiguousReadSize() const noexcept {
                    if (m_size == 0u || m_buffer == nullptr) {
                        return 0u;
                    }
                    if (m_head < m_tail) {
                        return m_tail - m_head;
                    }
                    return m_capacity - m_head;
                }

                void readCopy(std::uint8_t* const dst, const std::size_t len) const noexcept {
                    if (dst == nullptr || len == 0u) {
                        return;
                    }

                    const std::size_t first = contiguousReadSize();
                    const std::size_t firstCopy = (len < first) ? len : first;
                    std::memcpy(dst, readPtr(), firstCopy);
                    if (len > firstCopy) {
                        std::memcpy(dst + firstCopy, m_buffer, len - firstCopy);
                    }
                }

                void consume(const std::size_t len) noexcept {
                    const std::size_t amount = (len > m_size) ? m_size : len;
                    if (m_capacity != 0u) {
                        m_head = (m_head + amount) % m_capacity;
                    }
                    m_size -= amount;
                }

            private:
                std::uint8_t* m_buffer{ nullptr };
                std::size_t m_capacity{ 0u };
                std::size_t m_head{ 0u };
                std::size_t m_tail{ 0u };
                std::size_t m_size{ 0u };
            };

#if !defined(ARDUINO)
            class HostFile final {
            public:
                HostFile() noexcept = default;
                ~HostFile() noexcept { close(); }

                HostFile(const HostFile&) = delete;
                HostFile& operator=(const HostFile&) = delete;

                HostFile(HostFile&& other) noexcept : m_file(other.m_file) { other.m_file = nullptr; }
                HostFile& operator=(HostFile&& other) noexcept {
                    if (this != &other) {
                        close();
                        m_file = other.m_file;
                        other.m_file = nullptr;
                    }
                    return *this;
                }

                bool openWrite(const char* const path) noexcept {
                    close();
                    m_file = std::fopen(path, "wb");
                    return m_file != nullptr;
                }

                std::size_t write(const std::uint8_t* const data, const std::size_t len) noexcept {
                    if (m_file == nullptr || (data == nullptr && len != 0u)) {
                        return 0u;
                    }
                    return std::fwrite(data, 1u, len, m_file);
                }

                bool flush() noexcept {
                    return (m_file == nullptr) ? true : (std::fflush(m_file) == 0);
                }

                void close() noexcept {
                    if (m_file != nullptr) {
                        (void)std::fclose(m_file);
                        m_file = nullptr;
                    }
                }

                explicit operator bool() const noexcept { return m_file != nullptr; }

            private:
                std::FILE* m_file{ nullptr };
            };
#endif

        } // namespace detail

        /**
         * Computes the fixed 32-bit FNV-1a hash mandated by the file-format specification
         * for string-reference fields. The input is the exact UTF-8 byte sequence of the label
         * text without its terminating newline.
         */
        constexpr std::uint32_t fnv1a32(const char* const text) noexcept {
            return detail::fnv1a32_impl(text);
        }

        /** Returns the low 16 bits of fnv1a32(text). */
        constexpr std::uint16_t fnv1a16(const char* const text) noexcept {
            return static_cast<std::uint16_t>(detail::fnv1a32_impl(text) & 0xFFFFu);
        }

        /** Returns the low 8 bits of fnv1a32(text). */
        constexpr std::uint8_t fnv1a8(const char* const text) noexcept {
            return static_cast<std::uint8_t>(detail::fnv1a32_impl(text) & 0xFFu);
        }

        /** Convenience wrapper that returns fnv1a32(text) as mmlog::s32_t. */
        constexpr s32_t hash32(const char* const text) noexcept {
            return static_cast<s32_t>(fnv1a32(text));
        }

        /** Convenience wrapper that returns fnv1a16(text) as mmlog::s16_t. */
        constexpr s16_t hash16(const char* const text) noexcept {
            return static_cast<s16_t>(fnv1a16(text));
        }

        /** Convenience wrapper that returns fnv1a8(text) as mmlog::s8_t. */
        constexpr s8_t hash8(const char* const text) noexcept {
            return static_cast<s8_t>(fnv1a8(text));
        }

        // -----------------------------------------------------------------------------
        // Row-definition macro system
        // -----------------------------------------------------------------------------

#define MMLOG_DETAIL_DECLARE_MEMBER(Type, Name) Type Name{};
#define MMLOG_DETAIL_COUNT_FIELD(Type, Name) + 1u
#define MMLOG_DETAIL_ROW_BYTES(Type, Name) + sizeof(Type)
#define MMLOG_DETAIL_HEADER_CHARS(Type, Name) + ::mmlog::detail::FieldTraits<Type>::token_length + 1u + (sizeof(#Name) - 1u)
#define MMLOG_DETAIL_DESCRIPTOR(Type, Name) ::mmlog::detail::FieldDescriptor{::mmlog::detail::FieldTraits<Type>::kind, #Name, sizeof(Type)},

/**
 * Declares a packed row type and its compile-time schema metadata from a field-list macro.
 *
 * The field-list macro must expand as repeated invocations of X(Type, Name). The generated
 * structure is intended to act as a write-only staging object: application code assigns named
 * members, then passes the row to log(), which serializes the packed bytes in declaration order.
 *
 * Example:
 *
 *   #define MY_ROW_FIELDS(X) \
 *       X(std::uint32_t, seq) \
 *       X(float, yaw_rate) \
 *       X(mmlog::s32_t, stage)
 *
 *   MMLOG_DEFINE_ROW(MyRow, MY_ROW_FIELDS);
 *
 * The macro also emits compile-time checks that the row is packed, trivially copyable,
 * standard-layout, and no larger than one 512-byte service block.
 */
#define MMLOG_DEFINE_ROW(RowName, FieldListMacro)                                                                     \
    MMLOG_PACKED_BEGIN                                                                                                \
    struct MMLOG_PACKED RowName final {                                                                               \
        FieldListMacro(MMLOG_DETAIL_DECLARE_MEMBER)                                                                   \
                                                                                                                      \
        inline static constexpr std::size_t field_count = 0u FieldListMacro(MMLOG_DETAIL_COUNT_FIELD);               \
        inline static constexpr std::size_t row_bytes = 0u FieldListMacro(MMLOG_DETAIL_ROW_BYTES);                   \
        inline static constexpr std::size_t header_chars =                                                            \
            1u +                                                                                                      \
            (0u FieldListMacro(MMLOG_DETAIL_HEADER_CHARS)) +                                                          \
            ((field_count == 0u) ? 0u : (field_count - 1u));                                                          \
        inline static constexpr std::array<::mmlog::detail::FieldDescriptor, field_count> descriptors{{              \
            FieldListMacro(MMLOG_DETAIL_DESCRIPTOR)                                                                   \
        }};                                                                                                           \
        inline static constexpr auto header_storage =                                                                 \
            ::mmlog::detail::buildHeaderArray<header_chars>(descriptors);                                             \
        inline static constexpr std::uint32_t schema_version = ::mmlog::kSchemaVersion;                              \
        inline static constexpr std::uint32_t schema_hash = ::mmlog::fnv1a32(header_storage.data());                 \
                                                                                                                      \
        static constexpr const char* header_cstr() noexcept { return header_storage.data(); }                        \
    };                                                                                                                \
    MMLOG_PACKED_END                                                                                                  \
    static_assert(::std::is_trivially_copyable<RowName>::value, #RowName " must be trivially copyable.");          \
    static_assert(::std::is_standard_layout<RowName>::value, #RowName " must be standard layout.");                \
    static_assert(sizeof(RowName) == RowName::row_bytes, #RowName " contains padding.");                            \
    static_assert(RowName::row_bytes <= ::mmlog::kServiceBlockBytes,                                                  \
                  #RowName " exceeds the 512-byte service block size. Split the schema across multiple log files.")

#if MMLOG_ENABLE_TEENSY_FIFO_SDIO
        using StorageFileHandle = FsFile;
#elif defined(ARDUINO)
        using StorageFileHandle = File;
#else
        using StorageFileHandle = detail::HostFile;
#endif

        // -----------------------------------------------------------------------------
        // Logger
        // -----------------------------------------------------------------------------

        /**
         * Real-time oriented logger for the .mmlog/.sidecar pair defined by the micromouse logging
         * specification. The intended lifecycle is:
         *
         *   1. open(stem_or_path)
         *   2. writeMetadata(...) zero or more times
         *   3. begin(row_type_instance)
         *   4. During the control loop: log(row), optional writeLabel(label), and service()
         *   5. Outside the hot path: flush() and/or close()
         *
         * The logger owns fixed-size queues only. It performs no dynamic allocation after construction.
         */
        class MmLogLogger final {
        public:
#if MMLOG_ENABLE_TEENSY_FIFO_SDIO || !defined(ARDUINO)
            /**
             * Constructs a logger bound to the default storage backend for the current build.
             *
             * On Teensy builds, construction also claims the shared static queue storage used by
             * the runtime-owned logger instance.
             */
            MmLogLogger() noexcept;
#else
            /**
             * Constructs a logger that writes to the supplied Arduino filesystem object.
             *
             * The filesystem reference must remain valid for the lifetime of the logger.
             */
            explicit MmLogLogger(fs::FS& filesystem = SD) noexcept;
#endif
            /** Flushes outstanding data, closes any open files, and releases owned resources. */
            ~MmLogLogger() noexcept;

            /**
             * Opens a new logging session using file_name as the stem or explicit .mmlog path.
             *
             * Passing "run01" yields run01.mmlog and run01.sidecar. Passing "run01.mmlog" yields the
             * same pair after stripping the primary extension. Existing files of the same name are removed
             * first. No rows may be logged until begin() succeeds.
             */
            bool open(const char* file_name);

            /**
             * Queues one application-defined metadata entry for the sidecar.
             *
             * This may only be called after open() and before begin(). Duplicate keys are rejected.
             * The reserved keys schema_version and row_bytes are written automatically by begin().
             */
            bool writeMetadata(const char* key, const char* value);

            /**
             * Commits the schema for the current session by writing the sidecar header and opening the
             * primary file.
             *
             * The row argument is used only for type deduction; its current field values are ignored.
             * Passing a default-constructed row object is therefore sufficient. Once begin() succeeds,
             * the active session is locked to that row schema until close().
             */
            template <typename Row>
            bool begin(const Row&) {
                static_assert(detail::has_row_contract<Row>::value, "begin() requires an MMLOG_DEFINE_ROW-generated row type.");
                static_assert(std::is_trivially_copyable<Row>::value, "Logged row type must be trivially copyable.");
                static_assert(std::is_standard_layout<Row>::value, "Logged row type must be standard layout.");
                static_assert(sizeof(Row) == Row::row_bytes, "Logged row type contains padding.");
                static_assert(Row::row_bytes <= kServiceBlockBytes,
                    "Logged row type exceeds the 512-byte service block size. Split the schema across multiple log files.");

                return beginImpl(Row::header_cstr(), Row::row_bytes, Row::schema_version, Row::schema_hash);
            }

            /**
             * Appends one human-readable label string to the sidecar LABELS section.
             *
             * This does not hash the string for you and it does not deduplicate prior entries. Typical use
             * is to store mmlog::hash32(label) into an s32_t row field and then call writeLabel(label) so
             * host-side tooling can recover the text later. The call is valid only after begin().
             */
            bool writeLabel(const char* lookupString);

            /** Alias for writeLabel(), provided for call sites that prefer a shorter verb. */
            bool write(const char* lookupString) { return writeLabel(lookupString); }

            /**
             * Enqueues one packed primary row for the currently active schema.
             *
             * The row is copied byte-for-byte in member declaration order. This is correct for the intended
             * Teensy 4.x target, which is little-endian and matches the file-format endianness requirement.
             * The method rejects row types that do not match the schema previously passed to begin().
             */
            template <typename Row>
            bool log(const Row& row) {
                static_assert(detail::has_row_contract<Row>::value, "log() requires an MMLOG_DEFINE_ROW-generated row type.");
                static_assert(std::is_trivially_copyable<Row>::value, "Logged row type must be trivially copyable.");
                static_assert(std::is_standard_layout<Row>::value, "Logged row type must be standard layout.");
                static_assert(sizeof(Row) == Row::row_bytes, "Logged row type contains padding.");
                static_assert(Row::row_bytes <= kServiceBlockBytes,
                    "Logged row type exceeds the 512-byte service block size. Split the schema across multiple log files.");

                if (!m_isBegun) {
                    return fail("log() called before begin().");
                }
                if (Row::row_bytes != m_activeRowBytes || Row::schema_hash != m_activeSchemaHash) {
                    return fail("log() row type does not match active schema.");
                }
                return enqueuePrimary(reinterpret_cast<const std::uint8_t*>(&row), Row::row_bytes);
            }

            /**
             * Drains a bounded amount of queued data to storage.
             *
             * This is the hot-path maintenance call intended for use inside the control loop. On Teensy builds
             * it honors the configured byte budget and, by default, limits itself to one 512-byte sector per
             * file per call.
             */
            bool service();

            /**
             * Forces all queued data to storage and flushes both files.
             *
             * Unlike service(), this operation is intentionally unbounded and may take substantially longer.
             * Use it only at phase boundaries, shutdown, or other non-real-time points.
             */
            bool flush();

            /**
             * Flushes outstanding data, closes both files, and resets the object for reuse with a new
             * logging session. Fixed queue storage is retained so the same logger instance may be reopened
             * for a later phase without any allocation churn.
             */
            bool close();

            /** Returns true after open() succeeds and before close() resets the session. */
            bool isOpen() const noexcept { return m_isOpen; }
            /** Returns true after begin() succeeds and before close() resets the session. */
            bool isBegun() const noexcept { return m_isBegun; }
            /** Returns true when the storage backend is still completing a previously started transfer. */
            bool isTransferBusy() const noexcept;
            /** Returns the most recent error string, or an empty string when no error is latched. */
            const char* lastError() const noexcept { return m_lastError; }

        private:
            struct MetadataEntry final {
                char key[MMLOG_METADATA_KEY_MAX_LENGTH + 1u]{};
                char value[MMLOG_METADATA_VALUE_MAX_LENGTH + 1u]{};
                bool used{ false };
            };

            bool beginImpl(const char* header, std::size_t rowBytes, std::uint32_t schemaVersion, std::uint32_t schemaHash);
            bool attachStorage() noexcept;
            void releaseStorage() noexcept;
            bool enqueuePrimary(const std::uint8_t* data, std::size_t len);
            bool enqueueSidecar(const std::uint8_t* data, std::size_t len);
            bool drainQueueToFile(StorageFileHandle& file, detail::ByteRing& queue, std::size_t budget, bool sectorAligned);
            bool flushQueueToFile(StorageFileHandle& file, detail::ByteRing& queue, bool sectorAligned);
            bool writeDirect(StorageFileHandle& file, const std::uint8_t* data, std::size_t len);
            bool writeLineDirect(StorageFileHandle& file, const char* text);
            bool openPrimaryForWrite();
            bool openSidecarForWrite();
            bool removePrimaryFileIfPresent(const char* path);
            bool removeSidecarFileIfPresent(const char* path);
            bool derivePaths(const char* file_name);
            bool validateMetadataToken(const char* text) const noexcept;
            bool fail(const char* text);
            void clearError() noexcept;
            void resetSessionState() noexcept;
            void resetAllState() noexcept;
            bool metadataKeyExists(const char* key) const noexcept;
            bool isReservedMetadataKey(const char* key) const noexcept;

#if MMLOG_ENABLE_TEENSY_FIFO_SDIO
            bool m_storageAttached{ false };
#elif defined(ARDUINO)
            fs::FS& m_fs;
            bool m_storageAttached{ false };
#else
            bool m_storageAttached{ false };
#endif
            StorageFileHandle m_primaryFile;
            StorageFileHandle m_sidecarFile;

            detail::ByteRing m_primaryQueue;
            detail::ByteRing m_sidecarQueue;

#if !MMLOG_ENABLE_TEENSY_FIFO_SDIO
            alignas(32) std::uint8_t m_primaryStorage[MMLOG_PRIMARY_QUEUE_BYTES]{};
            alignas(32) std::uint8_t m_sidecarStorage[MMLOG_SIDECAR_QUEUE_BYTES]{};
#endif

            MetadataEntry m_metadata[MMLOG_METADATA_MAX_ENTRIES]{};
            std::size_t m_metadataCount{ 0u };

            char m_primaryPath[MMLOG_MAX_PATH_LENGTH + 1u]{};
            char m_sidecarPath[MMLOG_MAX_PATH_LENGTH + 1u]{};
            char m_sidecarBinding[MMLOG_MAX_PATH_LENGTH + 1u]{};
            char m_lastError[MMLOG_ERROR_TEXT_LENGTH + 1u]{};

            std::size_t m_activeRowBytes{ 0u };
            std::uint32_t m_activeSchemaHash{ 0u };
            bool m_sidecarDirty{ false };

            bool m_isOpen{ false };
            bool m_isBegun{ false };
            bool m_labelSectionStarted{ false };
        };

    } // namespace mmlog
}
