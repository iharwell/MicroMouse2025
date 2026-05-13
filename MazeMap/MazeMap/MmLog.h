#pragma once

#include "Defines.h"

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
     * mmlog system guide
     * ==================
     *
     * mmlog is the project logging format for high-rate structured robot data.
     * It is intentionally split into two files:
     *
     * - The primary `.mmlog` file starts with one text binding line,
     *   `sidecar_file=<name>.sidecar`, followed immediately by packed binary
     *   rows. Each row has the exact byte layout of the schema type passed to
     *   MmLogLogger::begin().
     * - The `.sidecar` file is text. It contains user metadata written before
     *   begin(), automatic schema metadata, the schema header line, and an
     *   optional LABELS section for string lookup values.
     *
     * The sidecar schema header is a comma-separated list of leaf fields:
     *
     *     u32_tick_us,f32_state_px_m,f32_state_py_m,s32_phase
     *
     * Each token is `<mmlog-type>_<field-name>`. Nested entries are flattened
     * by inserting the parent member name before each leaf name, so a row field
     * declared as `X(StateVectorEntry, state)` with entry leaves `px_m` and
     * `py_m` emits `f32_state_px_m,f32_state_py_m`. The binary row is not
     * transformed; it stores the packed nested entry bytes in place.
     *
     * Supported scalar field types
     * ----------------------------
     *
     *     std::uint8_t   -> u8
     *     std::int8_t    -> i8
     *     std::uint16_t  -> u16
     *     std::int16_t   -> i16
     *     std::uint32_t  -> u32
     *     std::int32_t   -> i32
     *     float          -> f32
     *     mmlog::s8_t    -> s8   low 8 bits of the label FNV-1a hash
     *     mmlog::s16_t   -> s16  low 16 bits of the label FNV-1a hash
     *     mmlog::s32_t   -> s32  full 32-bit label FNV-1a hash
     *
     * The s8/s16/s32 types are numeric string-reference fields. Store
     * hash8(), hash16(), or hash32() in the row and call writeLabel() with the
     * original label text after begin() so host tools can recover the text.
     * Labels are not deduplicated by the logger.
     *
     * Defining schemas
     * ----------------
     *
     * A schema is written as one or more X(Type, Name) field-list macros. Use
     * one of the entry macros for reusable fragments and one row macro for the
     * top-level row passed to begin() and log().
     *
     * - MMLOG_DEFINE_ENTRY(EntryName, Fields) creates a reusable public-field
     *   fragment. Entries may be nested in rows or other entries. Entries are
     *   not logged directly.
     * - MMLOG_DEFINE_ENTRY_WITH_BODY(EntryName, Fields, ...) is the same but
     *   injects public methods after the generated public fields.
     * - MMLOG_DEFINE_PRIVATE_ENTRY_WITH_BODY(EntryName, Fields, ...) makes the
     *   generated storage fields private and injects public methods. Use this
     *   when a fragment should be assigned through a domain method such as
     *   Set(const StateVector&).
     * - MMLOG_DEFINE_ROW(RowName, Fields) creates the top-level public-field
     *   row type used with begin() and log().
     * - MMLOG_DEFINE_ROW_WITH_BODY(RowName, Fields, ...) is the same but
     *   injects public methods after the generated public fields.
     * - MMLOG_DEFINE_PRIVATE_ROW_WITH_BODY(RowName, Fields, ...) makes the
     *   generated storage fields private and injects public methods. Use this
     *   for rows that should expose setters instead of raw field writes.
     *
     * The field identifier is part of the mmlog schema. If the desired output
     * is `f32_vector_px_m`, declare the private entry member as
     * `X(StateVectorEntry, vector)` and expose a public SetVector(...) method.
     * Do not name the storage field `_vector` unless the sidecar schema should
     * literally contain `_vector`.
     *
     * The generated types are packed, trivially copyable, standard-layout
     * staging objects. The macros statically reject padding and rows larger
     * than one 512-byte service block. Custom injected bodies should keep the
     * type trivial: avoid virtual functions, owning containers, references,
     * user-defined destructors, and non-trivial construction. Prefer ordinary
     * zero-initialization plus explicit setters.
     *
     * Example hierarchical row
     * ------------------------
     *
     *     struct StateVector { float pxM; float pyM; float yawRad; };
     *
     *     #define STATE_VECTOR_ENTRY_FIELDS(X) X(float, px_m) X(float, py_m) X(float, yaw_rad)
     *
     *     MMLOG_DEFINE_PRIVATE_ENTRY_WITH_BODY(
     *         StateVectorEntry,
     *         STATE_VECTOR_ENTRY_FIELDS,
     *         void Set(const StateVector& v) noexcept
     *         {
     *             px_m = v.pxM;
     *             py_m = v.pyM;
     *             yaw_rad = v.yawRad;
     *         });
     *
     *     #define OPEN_FLOOR_ROW_FIELDS(X) X(std::uint32_t, tick_us) X(StateVectorEntry, vector)
     *
     *     MMLOG_DEFINE_PRIVATE_ROW_WITH_BODY(
     *         OpenFloorRow,
     *         OPEN_FLOOR_ROW_FIELDS,
     *         void SetTickUs(std::uint32_t tickUs) noexcept { tick_us = tickUs; }
     *         void SetVectorEntry(const StateVector& v) noexcept { vector.Set(v); });
     *
     * The sidecar header for that row is:
     *
     *     u32_tick_us,f32_vector_px_m,f32_vector_py_m,f32_vector_yaw_rad
     *
     * The binary row layout is byte-for-byte equivalent to a manually flattened
     * packed row with fields `tick_us`, `vector_px_m`, `vector_py_m`, and
     * `vector_yaw_rad` in the same order.
     *
     * Logger lifecycle
     * ----------------
     *
     *     MmLogLogger log;
     *     OpenFloorRow row{};
     *
     *     log.open("open_floor_001");
     *     log.writeMetadata("mode", "open_floor");
     *     log.begin(row);
     *
     *     row.SetTickUs(tickUs);
     *     row.SetVectorEntry(stateVector);
     *     log.log(row);
     *     log.service();
     *
     *     log.flush();
     *     log.close();
     *
     * Call open() once per file pair. Call writeMetadata() only after open()
     * and before begin(). Call begin() once per session; the row argument is
     * used only for type deduction and its current values are ignored. After
     * begin(), log() accepts only the exact row schema used by begin(). Call
     * service() periodically in the control loop to drain bounded amounts of
     * queued data. Use flush() or close() only at non-real-time boundaries.
     *
     * All public logger operations report failure with `false`; inspect
     * lastError() for the latched diagnostic. The logger does not throw
     * exceptions and does not allocate dynamically after construction.
     *
     * Production ownership
     * --------------------
     *
     * Production runtime code should use the SharedRobotRuntime-owned
     * MmLogLogger instance. The logger may be closed and reopened with a
     * different schema for a later phase, but production code should not create
     * additional logger instances. Host-side tests may construct a logger
     * directly when testing mmlog behavior.
     */

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
#  define MMLOG_METADATA_MAX_ENTRIES 64u
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

/** Minimum Teensy preallocation for every SdFat-backed logger file. */
#ifndef MMLOG_TEENSY_MIN_PREALLOCATE_BYTES
#  define MMLOG_TEENSY_MIN_PREALLOCATE_BYTES (1024ULL * 1024ULL)
#endif

/** Number of bytes to preallocate for the primary file on Teensy FIFO SDIO builds. */
#ifndef MMLOG_TEENSY_PRIMARY_PREALLOCATE_BYTES
#  define MMLOG_TEENSY_PRIMARY_PREALLOCATE_BYTES (96ULL * 1024ULL * 1024ULL)
#endif

/** Number of bytes to preallocate for the sidecar file on Teensy FIFO SDIO builds. */
#ifndef MMLOG_TEENSY_SIDECAR_PREALLOCATE_BYTES
#  define MMLOG_TEENSY_SIDECAR_PREALLOCATE_BYTES (1024ULL * 1024ULL)
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
        static_assert(MMLOG_TEENSY_PRIMARY_PREALLOCATE_BYTES >= MMLOG_TEENSY_MIN_PREALLOCATE_BYTES,
            "Primary mmlog files must be preallocated with at least 1 MiB on Teensy FIFO SDIO builds.");
        static_assert(MMLOG_TEENSY_SIDECAR_PREALLOCATE_BYTES >= MMLOG_TEENSY_MIN_PREALLOCATE_BYTES,
            "Sidecar files must be preallocated with at least 1 MiB on Teensy FIFO SDIO builds.");
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

            constexpr std::uint32_t fnv1a32_impl(const char* const text) noexcept {
                std::uint32_t hash = 2166136261u;
                for (std::size_t i = 0u; text[i] != '\0'; ++i) {
                    hash ^= static_cast<std::uint8_t>(text[i]);
                    hash *= 16777619u;
                }
                return hash;
            }

            template <typename T, typename = void>
            struct has_entry_contract : std::false_type {};

            template <typename T>
            struct has_entry_contract<
                T,
                std::void_t<
                decltype(T::mmlog_entry_marker),
                decltype(T::field_count),
                decltype(T::row_bytes),
                decltype(T::header_chars),
                decltype(T::header_storage),
                decltype(T::header_cstr())>> : std::true_type {};

            template <typename T, bool IsEntry = has_entry_contract<T>::value>
            struct SchemaFieldTraits;

            constexpr std::size_t countHeaderEntries(const char* const header) noexcept {
                if (header == nullptr || header[0] == '\0') {
                    return 0u;
                }

                std::size_t count = 1u;
                for (std::size_t i = 0u; header[i] != '\0'; ++i) {
                    if (header[i] == ',') {
                        ++count;
                    }
                }
                return count;
            }

            constexpr std::size_t prefixedHeaderChars(
                const char* const header,
                const char* const prefix) noexcept {
                return cstrlen(header) + (countHeaderEntries(header) * (cstrlen(prefix) + 1u));
            }

            template <std::size_t HeaderChars>
            constexpr void appendPrefixedHeader(
                std::array<char, HeaderChars>& out,
                std::size_t& pos,
                const char* const header,
                const char* const prefix) noexcept {
                std::size_t source = 0u;

                while (header[source] != '\0') {
                    while (header[source] != '\0' && header[source] != '_' && header[source] != ',') {
                        out[pos++] = header[source++];
                    }
                    if (header[source] == '_') {
                        out[pos++] = header[source++];
                        copyChars(out.data(), pos, prefix);
                        out[pos++] = '_';
                    }
                    while (header[source] != '\0' && header[source] != ',') {
                        out[pos++] = header[source++];
                    }
                    if (header[source] == ',') {
                        out[pos++] = header[source++];
                    }
                }
            }

            template <typename T>
            struct SchemaFieldTraits<T, false> final {
                static constexpr std::size_t field_count = 1u;
                static constexpr std::size_t row_bytes = sizeof(T);

                static constexpr std::size_t headerChars(const char* const name) noexcept {
                    return FieldTraits<T>::token_length + 1u + cstrlen(name);
                }

                template <std::size_t HeaderChars>
                static constexpr void appendHeader(
                    std::array<char, HeaderChars>& out,
                    std::size_t& pos,
                    const char* const name) noexcept {
                    copyChars(out.data(), pos, fieldKindToken(FieldTraits<T>::kind));
                    out[pos++] = '_';
                    copyChars(out.data(), pos, name);
                }
            };

            template <typename T>
            struct SchemaFieldTraits<T, true> final {
                static constexpr std::size_t field_count = T::field_count;
                static constexpr std::size_t row_bytes = T::row_bytes;

                static constexpr std::size_t headerChars(const char* const name) noexcept {
                    return prefixedHeaderChars(T::header_cstr(), name);
                }

                template <std::size_t HeaderChars>
                static constexpr void appendHeader(
                    std::array<char, HeaderChars>& out,
                    std::size_t& pos,
                    const char* const name) noexcept {
                    appendPrefixedHeader(out, pos, T::header_cstr(), name);
                }
            };

            template <typename T>
            constexpr std::size_t schemaFieldHeaderChars(const char* const name) noexcept {
                return SchemaFieldTraits<T>::headerChars(name);
            }

            template <typename T, std::size_t HeaderChars>
            constexpr void appendSchemaFieldHeader(
                std::array<char, HeaderChars>& out,
                std::size_t& pos,
                bool& first,
                const char* const name) noexcept {
                if (!first) {
                    out[pos++] = ',';
                }
                first = false;
                SchemaFieldTraits<T>::appendHeader(out, pos, name);
            }

            template <std::size_t HeaderChars>
            constexpr std::array<char, HeaderChars> terminateHeader(
                std::array<char, HeaderChars> out,
                const std::size_t pos) noexcept {
                out[pos] = '\0';
                return out;
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
                decltype(T::header_chars),
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
#if defined(_MSC_VER)
                    if (path == nullptr || fopen_s(&m_file, path, "wb") != 0) {
                        m_file = nullptr;
                    }
#else
                    m_file = std::fopen(path, "wb");
#endif
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
#define MMLOG_DETAIL_COUNT_MEMBER(Type, Name) + 1u
#define MMLOG_DETAIL_COUNT_FIELD(Type, Name) + ::MazeMap::mmlog::detail::SchemaFieldTraits<Type>::field_count
#define MMLOG_DETAIL_ROW_BYTES(Type, Name) + ::MazeMap::mmlog::detail::SchemaFieldTraits<Type>::row_bytes
#define MMLOG_DETAIL_HEADER_CHARS(Type, Name) + ::MazeMap::mmlog::detail::schemaFieldHeaderChars<Type>(#Name)
#define MMLOG_DETAIL_APPEND_HEADER(Type, Name) ::MazeMap::mmlog::detail::appendSchemaFieldHeader<Type>(out, pos, first, #Name);

/**
 * Declares a packed reusable row fragment whose fields are flattened when used
 * as a member of an MMLOG_DEFINE_ROW row or another MMLOG_DEFINE_ENTRY entry.
 *
 * The entry field-list macro uses the same X(Type, Name) shape as rows. When a
 * parent schema declares X(MyEntry, prefix), the sidecar header emits each leaf
 * as type_prefix_leaf while the binary row stores the packed MyEntry bytes in
 * place. Use the private-with-body variant when the fragment should preserve a
 * domain abstraction and expose assignment through methods instead of public
 * scalar fields.
 */
#define MMLOG_DETAIL_DEFINE_ENTRY(EntryName, FieldListMacro, FieldAccess, ...)                                       \
    MMLOG_PACKED_BEGIN                                                                                                \
    struct MMLOG_PACKED EntryName final {                                                                             \
        FieldAccess:                                                                                                  \
        FieldListMacro(MMLOG_DETAIL_DECLARE_MEMBER)                                                                   \
                                                                                                                      \
    public:                                                                                                           \
        __VA_ARGS__                                                                                                   \
                                                                                                                      \
        inline static constexpr bool mmlog_entry_marker = true;                                                       \
        inline static constexpr std::size_t member_count = 0u FieldListMacro(MMLOG_DETAIL_COUNT_MEMBER);             \
        inline static constexpr std::size_t field_count = 0u FieldListMacro(MMLOG_DETAIL_COUNT_FIELD);               \
        inline static constexpr std::size_t row_bytes = 0u FieldListMacro(MMLOG_DETAIL_ROW_BYTES);                   \
        inline static constexpr std::size_t header_chars =                                                            \
            1u +                                                                                                      \
            (0u FieldListMacro(MMLOG_DETAIL_HEADER_CHARS)) +                                                          \
            ((member_count == 0u) ? 0u : (member_count - 1u));                                                        \
                                                                                                                      \
        inline static constexpr auto header_storage = []() constexpr {                                               \
            std::array<char, header_chars> out{};                                                                     \
            std::size_t pos = 0u;                                                                                     \
            bool first = true;                                                                                        \
            FieldListMacro(MMLOG_DETAIL_APPEND_HEADER)                                                                \
            return ::MazeMap::mmlog::detail::terminateHeader(out, pos);                                               \
        }();                                                                                                          \
                                                                                                                      \
        static constexpr const char* header_cstr() noexcept { return header_storage.data(); }                        \
    };                                                                                                                \
    MMLOG_PACKED_END                                                                                                  \
    static_assert(::std::is_trivially_copyable<EntryName>::value, #EntryName " must be trivially copyable.");       \
    static_assert(::std::is_standard_layout<EntryName>::value, #EntryName " must be standard layout.");             \
    static_assert(sizeof(EntryName) == EntryName::row_bytes, #EntryName " contains padding.")

#define MMLOG_DEFINE_ENTRY(EntryName, FieldListMacro)                                                                 \
    MMLOG_DETAIL_DEFINE_ENTRY(EntryName, FieldListMacro, public, )

#define MMLOG_DEFINE_ENTRY_WITH_BODY(EntryName, FieldListMacro, ...)                                                  \
    MMLOG_DETAIL_DEFINE_ENTRY(EntryName, FieldListMacro, public, __VA_ARGS__)

#define MMLOG_DEFINE_PRIVATE_ENTRY_WITH_BODY(EntryName, FieldListMacro, ...)                                          \
    MMLOG_DETAIL_DEFINE_ENTRY(EntryName, FieldListMacro, private, __VA_ARGS__)

/**
 * Declares a packed row type and its compile-time schema metadata from a field-list macro.
 *
 * The field-list macro must expand as repeated invocations of X(Type, Name). Type may be an
 * allowed scalar field type or an MMLOG_DEFINE_ENTRY-generated entry type. The
 * generated structure is a packed staging object. Public-field rows can be
 * assigned directly; private-with-body rows should expose setters that populate
 * the hidden fields and nested entries. log() serializes the packed bytes in
 * declaration order.
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
 * The macro also emits compile-time checks that the row is packed, trivially
 * copyable, standard-layout, and no larger than one 512-byte service block.
 */
#define MMLOG_DETAIL_DEFINE_ROW(RowName, FieldListMacro, FieldAccess, ...)                                           \
    MMLOG_PACKED_BEGIN                                                                                                \
    struct MMLOG_PACKED RowName final {                                                                               \
        FieldAccess:                                                                                                  \
        FieldListMacro(MMLOG_DETAIL_DECLARE_MEMBER)                                                                   \
                                                                                                                      \
    public:                                                                                                           \
        __VA_ARGS__                                                                                                   \
                                                                                                                      \
        inline static constexpr std::size_t member_count = 0u FieldListMacro(MMLOG_DETAIL_COUNT_MEMBER);             \
        inline static constexpr std::size_t field_count = 0u FieldListMacro(MMLOG_DETAIL_COUNT_FIELD);               \
        inline static constexpr std::size_t row_bytes = 0u FieldListMacro(MMLOG_DETAIL_ROW_BYTES);                   \
        inline static constexpr std::size_t header_chars =                                                            \
            1u +                                                                                                      \
            (0u FieldListMacro(MMLOG_DETAIL_HEADER_CHARS)) +                                                          \
            ((member_count == 0u) ? 0u : (member_count - 1u));                                                        \
                                                                                                                      \
        inline static constexpr auto header_storage = []() constexpr {                                               \
            std::array<char, header_chars> out{};                                                                     \
            std::size_t pos = 0u;                                                                                     \
            bool first = true;                                                                                        \
            FieldListMacro(MMLOG_DETAIL_APPEND_HEADER)                                                                \
            return ::MazeMap::mmlog::detail::terminateHeader(out, pos);                                               \
        }();                                                                                                          \
        inline static constexpr std::uint32_t schema_version = ::MazeMap::mmlog::kSchemaVersion;                     \
        inline static constexpr std::uint32_t schema_hash = ::MazeMap::mmlog::fnv1a32(header_storage.data());        \
                                                                                                                      \
        static constexpr const char* header_cstr() noexcept { return header_storage.data(); }                        \
    };                                                                                                                \
    MMLOG_PACKED_END                                                                                                  \
    static_assert(::std::is_trivially_copyable<RowName>::value, #RowName " must be trivially copyable.");          \
    static_assert(::std::is_standard_layout<RowName>::value, #RowName " must be standard layout.");                \
    static_assert(sizeof(RowName) == RowName::row_bytes, #RowName " contains padding.");                            \
    static_assert(RowName::row_bytes <= ::MazeMap::mmlog::kServiceBlockBytes,                                         \
                  #RowName " exceeds the 512-byte service block size. Split the schema across multiple log files.")

#define MMLOG_DEFINE_ROW(RowName, FieldListMacro)                                                                     \
    MMLOG_DETAIL_DEFINE_ROW(RowName, FieldListMacro, public, )

#define MMLOG_DEFINE_ROW_WITH_BODY(RowName, FieldListMacro, ...)                                                      \
    MMLOG_DETAIL_DEFINE_ROW(RowName, FieldListMacro, public, __VA_ARGS__)

#define MMLOG_DEFINE_PRIVATE_ROW_WITH_BODY(RowName, FieldListMacro, ...)                                              \
    MMLOG_DETAIL_DEFINE_ROW(RowName, FieldListMacro, private, __VA_ARGS__)

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
        class EXPORT MmLogLogger final {
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
             * Writes one application-defined metadata entry to the sidecar in call order.
             *
             * This may only be called after open() and before begin().
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

                return logImpl(&row, Row::row_bytes, Row::schema_hash);
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
            static constexpr std::size_t kLineBufferChars = 513u;

            // These non-template sinks are the intentional thin-wrapper exception in MmLogLogger.
            // Row schemas are template types by design; keeping runtime logic here avoids cloning
            // open/begin/log behavior for every schema instantiation.
            bool beginImpl(const char* header, std::size_t rowBytes, std::uint32_t schemaVersion, std::uint32_t schemaHash);
            bool logImpl(const void* row, std::size_t rowBytes, std::uint32_t schemaHash);
            bool attachStorage() noexcept;
            void releaseStorage() noexcept;
            bool queuePrimaryBytes(const std::uint8_t* data, std::size_t len);
            bool queueSidecarBytes(const std::uint8_t* data, std::size_t len);
            bool queueSidecarLine(const char* text);
            bool drainQueueToFile(StorageFileHandle& file, detail::ByteRing& queue, std::size_t budget, bool sectorAligned);
            bool flushQueueToFile(StorageFileHandle& file, detail::ByteRing& queue, bool sectorAligned);
            bool openPrimaryForWrite();
            bool openSidecarForWrite();
            bool removeFileIfPresent(const char* path);
            bool derivePaths(const char* file_name);
            bool syncFile(StorageFileHandle& file) noexcept;
            bool finalizeFileLength(StorageFileHandle& file) noexcept;
            bool fail(const char* text);
            void clearError() noexcept;
            void resetSessionState() noexcept;
            void resetAllState() noexcept;

            static void copyTruncatedText(char* destination, std::size_t destinationSize, const char* source) noexcept;

#if MMLOG_ENABLE_TEENSY_FIFO_SDIO
            alignas(32) static std::uint8_t s_primaryStorage[MMLOG_PRIMARY_QUEUE_BYTES];
            alignas(32) static std::uint8_t s_sidecarStorage[MMLOG_SIDECAR_QUEUE_BYTES];
            static bool s_teensyStorageClaimed;
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
