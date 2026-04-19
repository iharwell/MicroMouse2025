#include "pch.h"
#include "MmLog.h"

#include <cstdio>
#include <cstring>

namespace MazeMap {
    namespace mmlog {

        namespace {

            constexpr std::size_t kLineBufferChars = 192u;

            bool endsWith(const char* const text, const char* const suffix) noexcept {
                const std::size_t textLen = std::strlen(text);
                const std::size_t suffixLen = std::strlen(suffix);
                if (suffixLen > textLen) {
                    return false;
                }
                return std::strcmp(text + (textLen - suffixLen), suffix) == 0;
            }

            bool hasInvalidTextChar(const char* const text) noexcept {
                if (text == nullptr || text[0] == '\0') {
                    return true;
                }

                for (std::size_t i = 0u; text[i] != '\0'; ++i) {
                    const char c = text[i];
                    if (c == '=' || c == '\n' || c == '\r') {
                        return true;
                    }
                }
                return false;
            }

            const char* findLastSlash(const char* const text) noexcept {
                const char* last = nullptr;
                for (const char* p = text; *p != '\0'; ++p) {
                    if (*p == '/' || *p == '\\') {
                        last = p;
                    }
                }
                return last;
            }

            bool safeCopy(char* const dst, const std::size_t dstSize, const char* const src) noexcept {
                if (dst == nullptr || src == nullptr || dstSize == 0u) {
                    return false;
                }

                const std::size_t len = std::strlen(src);
                if (len >= dstSize) {
                    return false;
                }

                std::memcpy(dst, src, len + 1u);
                return true;
            }

            bool safeAppend(char* const dst, const std::size_t dstSize, const char* const src) noexcept {
                if (dst == nullptr || src == nullptr || dstSize == 0u) {
                    return false;
                }

                const std::size_t dstLen = std::strlen(dst);
                const std::size_t srcLen = std::strlen(src);
                if ((dstLen + srcLen) >= dstSize) {
                    return false;
                }

                std::memcpy(dst + dstLen, src, srcLen + 1u);
                return true;
            }

            bool makeStemPath(char* const dst, const std::size_t dstSize, const char* const input) noexcept {
                if (!safeCopy(dst, dstSize, input)) {
                    return false;
                }

                const std::size_t len = std::strlen(dst);
                if (len >= std::strlen(kPrimaryExtension) && endsWith(dst, kPrimaryExtension)) {
                    dst[len - std::strlen(kPrimaryExtension)] = '\0';
                }
                return true;
            }

#if MMLOG_ENABLE_TEENSY_FIFO_SDIO
            DMAMEM alignas(32) static std::uint8_t g_primaryStorage[MMLOG_PRIMARY_QUEUE_BYTES];
            DMAMEM alignas(32) static std::uint8_t g_sidecarStorage[MMLOG_SIDECAR_QUEUE_BYTES];
            static bool g_teensyStorageClaimed = false;
#endif

            bool syncStorageFile(StorageFileHandle& file) noexcept {
#if MMLOG_ENABLE_TEENSY_FIFO_SDIO
                return file.sync();
#elif defined(ARDUINO)
                file.flush();
                return static_cast<bool>(file);
#else
                return file.flush();
#endif
            }

            bool finalizeStorageFileLength(StorageFileHandle& file) noexcept {
#if MMLOG_ENABLE_TEENSY_FIFO_SDIO
                if (!file) {
                    return true;
                }

                const std::uint64_t logicalLength = file.curPosition();
                if (!file.seekSet(logicalLength)) {
                    return false;
                }
                if (!file.truncate()) {
                    return false;
                }
                return syncStorageFile(file);
#else
                (void)file;
                return true;
#endif
            }

        } // namespace

#if MMLOG_ENABLE_TEENSY_FIFO_SDIO || !defined(ARDUINO)
        MmLogLogger::MmLogLogger() noexcept {
            resetAllState();
            (void)attachStorage();
        }
#else
        MmLogLogger::MmLogLogger(fs::FS& filesystem) noexcept
            : m_fs(filesystem) {
            resetAllState();
            (void)attachStorage();
        }
#endif

        MmLogLogger::~MmLogLogger() noexcept {
            (void)close();
#if MMLOG_ENABLE_TEENSY_FIFO_SDIO
            releaseStorage();
#endif
        }

        bool MmLogLogger::attachStorage() noexcept {
            if (m_storageAttached) {
                return true;
            }

#if MMLOG_ENABLE_TEENSY_FIFO_SDIO
            if (g_teensyStorageClaimed) {
                return false;
            }
            g_teensyStorageClaimed = true;
            if (!m_primaryQueue.attach(g_primaryStorage, sizeof(g_primaryStorage))) {
                g_teensyStorageClaimed = false;
                return false;
            }
            if (!m_sidecarQueue.attach(g_sidecarStorage, sizeof(g_sidecarStorage))) {
                g_teensyStorageClaimed = false;
                m_primaryQueue.attach(nullptr, 0u);
                return false;
            }
#else
            if (!m_primaryQueue.attach(m_primaryStorage, sizeof(m_primaryStorage))) {
                return false;
            }
            if (!m_sidecarQueue.attach(m_sidecarStorage, sizeof(m_sidecarStorage))) {
                m_primaryQueue.attach(nullptr, 0u);
                return false;
            }
#endif

            m_storageAttached = true;
            return true;
        }

        void MmLogLogger::releaseStorage() noexcept {
#if MMLOG_ENABLE_TEENSY_FIFO_SDIO
            // Storage ownership persists across open()/close() so a single logger instance can
            // reuse the same RAM2 buffers between phases without any allocation churn.
            // The claim is only released when the owning object is destroyed.
            if (m_storageAttached) {
                m_primaryQueue.attach(nullptr, 0u);
                m_sidecarQueue.attach(nullptr, 0u);
                g_teensyStorageClaimed = false;
                m_storageAttached = false;
            }
#else
            if (m_storageAttached) {
                m_primaryQueue.clear();
                m_sidecarQueue.clear();
            }
#endif
        }

        bool MmLogLogger::open(const char* const file_name) {
            if (file_name == nullptr || file_name[0] == '\0') {
                return fail("open() requires a non-empty file name.");
            }
            if (m_isOpen) {
                return fail("open() called while another session is active.");
            }
            if (!attachStorage()) {
#if MMLOG_ENABLE_TEENSY_FIFO_SDIO
                return fail("Teensy logger storage is already claimed by another instance.");
#else
                return fail("Logger storage is not available.");
#endif
            }

            clearError();
            resetSessionState();

            if (!derivePaths(file_name)) {
                return false;
            }
            bool sidecarReady = false;
            if (!removePrimaryFileIfPresent(m_primaryPath)) {
                sidecarReady = false;
            }
            else if (!removeSidecarFileIfPresent(m_sidecarPath)) {
                sidecarReady = false;
            }
            else {
                sidecarReady = openSidecarForWrite();
            }
            if (!sidecarReady) {
                m_primaryFile.close();
                m_sidecarFile.close();
                resetSessionState();
                return false;
            }

            m_isOpen = true;
            return true;
        }

        bool MmLogLogger::writeMetadata(const char* const key, const char* const value) {
            if (!m_isOpen) {
                return fail("writeMetadata() called before open().");
            }
            if (m_isBegun) {
                return fail("writeMetadata() is only allowed before begin().");
            }
            if (!validateMetadataToken(key) || value == nullptr || value[0] == '\0') {
                return fail("Invalid metadata key or value.");
            }
            if (std::strchr(value, '\n') != nullptr || std::strchr(value, '\r') != nullptr) {
                return fail("Metadata values may not contain newlines.");
            }
            if (isReservedMetadataKey(key)) {
                return fail("Reserved metadata key.");
            }
            const std::size_t keyLength = std::strlen(key);
            if (keyLength > MMLOG_METADATA_KEY_MAX_LENGTH) {
                char message[kLineBufferChars]{};
                if (std::snprintf(
                    message,
                    sizeof(message),
                    "Metadata key too long: %s (%zu > %u).",
                    key,
                    keyLength,
                    static_cast<unsigned>(MMLOG_METADATA_KEY_MAX_LENGTH)) >= static_cast<int>(sizeof(message))) {
                    return fail("Metadata key too long.");
                }
                return fail(message);
            }

            const std::size_t valueLength = std::strlen(value);
            if (valueLength > MMLOG_METADATA_VALUE_MAX_LENGTH) {
                char message[kLineBufferChars]{};
                if (std::snprintf(
                    message,
                    sizeof(message),
                    "Metadata value too long for %s (%zu > %u).",
                    key,
                    valueLength,
                    static_cast<unsigned>(MMLOG_METADATA_VALUE_MAX_LENGTH)) >= static_cast<int>(sizeof(message))) {
                    return fail("Metadata value too long.");
                }
                return fail(message);
            }

            char line[kLineBufferChars]{};
            const int length = std::snprintf(line, sizeof(line), "%s=%s", key, value);
            if (length <= 0 || length >= static_cast<int>(sizeof(line))) {
                return fail("Metadata line too long.");
            }
            if (!writeLineDirect(m_sidecarFile, line)) {
                return fail("Failed to write metadata line.");
            }
            return true;
        }

        bool MmLogLogger::beginImpl(
            const char* const header,
            const std::size_t rowBytes,
            const std::uint32_t schemaVersion,
            const std::uint32_t schemaHash) {

            if (!m_isOpen) {
                return fail("begin() called before open().");
            }
            if (m_isBegun) {
                return fail("begin() may only be called once per session.");
            }
            if (header == nullptr || header[0] == '\0') {
                return fail("begin() received an empty header.");
            }
            if (rowBytes == 0u) {
                return fail("begin() received a zero-sized row.");
            }
            if (!m_sidecarFile) {
                return fail("Sidecar file is not open.");
            }

            {
                char line[kLineBufferChars]{};
                const char* sidecarHeaderError = nullptr;
                std::size_t requiredSidecarBytes = 0u;
                auto countSidecarLine = [&line, &requiredSidecarBytes](const int length) -> bool
                {
                    if (length <= 0 || length >= static_cast<int>(sizeof(line))) {
                        return false;
                    }

                    requiredSidecarBytes += static_cast<std::size_t>(length) + 1u;
                    return true;
                };
                auto queueSidecarLine = [this](const char* const text) -> bool
                {
                    if (text == nullptr) {
                        return false;
                    }

                    const std::size_t len = std::strlen(text);
                    static constexpr std::uint8_t newline = '\n';
                    return
                        pushSidecarQueue(reinterpret_cast<const std::uint8_t*>(text), len) &&
                        pushSidecarQueue(&newline, 1u);
                };

                if (!countSidecarLine(std::snprintf(line, sizeof(line), "schema_version=%lu", static_cast<unsigned long>(schemaVersion)))) {
                    sidecarHeaderError = "schema_version line too long.";
                }

                if (sidecarHeaderError == nullptr) {
                    if (!countSidecarLine(std::snprintf(line, sizeof(line), "row_bytes=%lu", static_cast<unsigned long>(rowBytes)))) {
                        sidecarHeaderError = "row_bytes line too long.";
                    }
                }

                if (sidecarHeaderError == nullptr) {
                    if (!countSidecarLine(std::snprintf(line, sizeof(line), "%s", header))) {
                        sidecarHeaderError = "Header line too long.";
                    } else if (requiredSidecarBytes > m_sidecarQueue.freeSpace()) {
                        sidecarHeaderError = "Sidecar header exceeds queue capacity.";
                    }
                }

                if (sidecarHeaderError == nullptr) {
                    std::snprintf(line, sizeof(line), "schema_version=%lu", static_cast<unsigned long>(schemaVersion));
                    if (!queueSidecarLine(line)) {
                        sidecarHeaderError = "Failed to queue schema_version.";
                    }
                }

                if (sidecarHeaderError == nullptr) {
                    std::snprintf(line, sizeof(line), "row_bytes=%lu", static_cast<unsigned long>(rowBytes));
                    if (!queueSidecarLine(line)) {
                        sidecarHeaderError = "Failed to queue row_bytes.";
                    }
                }

                if (sidecarHeaderError == nullptr && !queueSidecarLine(header)) {
                    sidecarHeaderError = "Failed to queue header line.";
                }

                if (sidecarHeaderError != nullptr) {
                    return fail(sidecarHeaderError);
                }
            }

            {
                const char* primaryError = nullptr;
                auto queuePrimaryLine = [this](const char* const text) -> bool
                {
                    if (text == nullptr) {
                        return false;
                    }

                    const std::size_t len = std::strlen(text);
                    static constexpr std::uint8_t newline = '\n';
                    return
                        pushPrimaryQueue(reinterpret_cast<const std::uint8_t*>(text), len) &&
                        pushPrimaryQueue(&newline, 1u);
                };

                if (!openPrimaryForWrite()) {
                    primaryError = (m_lastError[0] != '\0') ? m_lastError : "Failed to prepare primary file.";
                }
                else {
                    char bindingLine[kLineBufferChars]{};
                    if (std::snprintf(bindingLine, sizeof(bindingLine), "sidecar_file=%s", m_sidecarBinding) >= static_cast<int>(sizeof(bindingLine))) {
                        primaryError = "Sidecar binding line too long.";
                    }
                    else if (!queuePrimaryLine(bindingLine)) {
                        primaryError = "Failed to queue primary sidecar binding line.";
                    }
                }

                if (primaryError != nullptr) {
                    return fail(primaryError);
                }
            }

            m_activeRowBytes = rowBytes;
            m_activeSchemaHash = schemaHash;
            m_isBegun = true;
            return true;
        }

        bool MmLogLogger::writeLabel(const char* const lookupString) {
            if (!m_isBegun) {
                return fail("writeLabel() called before begin().");
            }
            if (lookupString == nullptr || lookupString[0] == '\0') {
                return fail("writeLabel() requires a non-empty label string.");
            }

            const std::size_t len = detail::bounded_cstrlen(lookupString, MMLOG_MAX_LABEL_LENGTH + 1u);
            if (len == 0u) {
                return fail("writeLabel() requires a non-empty label string.");
            }
            if (len > MMLOG_MAX_LABEL_LENGTH || lookupString[len] != '\0') {
                return fail("Label string exceeds MMLOG_MAX_LABEL_LENGTH.");
            }
            for (std::size_t i = 0u; i < len; ++i) {
                if (lookupString[i] == '\n' || lookupString[i] == '\r') {
                    return fail("Label strings may not contain newlines.");
                }
            }

            const std::size_t required = (m_labelSectionStarted ? 0u : kLabelsMarkerBytes) + len + 1u;
            if (required > m_sidecarQueue.freeSpace()) {
                return fail("Sidecar queue overflow.");
            }

            if (!m_labelSectionStarted) {
                if (!enqueueSidecar(reinterpret_cast<const std::uint8_t*>(kLabelsMarker), kLabelsMarkerBytes)) {
                    return false;
                }
                m_labelSectionStarted = true;
            }

            if (!enqueueSidecar(reinterpret_cast<const std::uint8_t*>(lookupString), len)) {
                return false;
            }

            static constexpr std::uint8_t newline = '\n';
            return enqueueSidecar(&newline, 1u);
        }

        bool MmLogLogger::isTransferBusy() const noexcept {
#if MMLOG_ENABLE_TEENSY_FIFO_SDIO
            auto fileBusy = [](const StorageFileHandle& file) noexcept -> bool {
                return file && const_cast<StorageFileHandle&>(file).isBusy();
            };
            return
                fileBusy(m_primaryFile) ||
                fileBusy(m_sidecarFile);
#else
            return false;
#endif
        }

        bool MmLogLogger::service() {
            // SharedRobotRuntime owns cross-file servicing arbitration. This routine only
            // arbitrates the logger-owned sidecar and primary streams when the runtime hub calls it.
            if (!m_isOpen || !m_isBegun) {
                return true;
            }
            if (isTransferBusy()) {
                return true;
            }

            if (m_sidecarFile && m_sidecarQueue.size() >= kSdSectorBytes) {
                if (!drainQueueToFile(m_sidecarFile, m_sidecarQueue, MMLOG_SERVICE_SIDECAR_BUDGET_BYTES, true)) {
                    return false;
                }
                return true;
            }

            if (m_primaryFile && m_primaryQueue.size() >= kSdSectorBytes) {
                if (!drainQueueToFile(m_primaryFile, m_primaryQueue, MMLOG_SERVICE_PRIMARY_BUDGET_BYTES, true)) {
                    return false;
                }
            }
            return true;
        }

        bool MmLogLogger::flush() {
            // flush() keeps its conventional meaning: fully drain logger-owned queues.
            // Control-loop service arbitration still belongs in SharedRobotRuntime.
            if (!m_isOpen) {
                return true;
            }

            if (m_sidecarFile && (m_sidecarDirty || !m_sidecarQueue.empty())) {
                const bool sidecarFlushed =
                    flushQueueToFile(m_sidecarFile, m_sidecarQueue, true) &&
                    syncStorageFile(m_sidecarFile);
                if (!sidecarFlushed) {
                    return fail("Failed to flush sidecar file.");
                }
                m_sidecarDirty = false;
            }
            if (m_primaryFile && !flushQueueToFile(m_primaryFile, m_primaryQueue, true)) {
                return false;
            }
            if (m_primaryFile && !syncStorageFile(m_primaryFile)) {
                return fail("Failed to flush primary file.");
            }
            return true;
        }

        bool MmLogLogger::close() {
            bool ok = true;
            char savedError[MMLOG_ERROR_TEXT_LENGTH + 1u]{};
            auto captureCloseError = [&](const char* const fallback) {
                ok = false;
                if (savedError[0] != '\0') {
                    return;
                }

                const char* const source =
                    (m_lastError[0] != '\0') ? m_lastError : ((fallback != nullptr) ? fallback : "MmLog close failed.");
                std::strncpy(savedError, source, sizeof(savedError) - 1u);
                savedError[sizeof(savedError) - 1u] = '\0';
            };

            if (m_isOpen && !flush()) {
                captureCloseError("Failed to flush log files.");
            }

            if (m_isOpen && m_primaryFile) {
                const bool primaryFinalized = finalizeStorageFileLength(m_primaryFile);
                if (!primaryFinalized) {
                    (void)fail("Failed to finalize primary file length.");
                    captureCloseError("Failed to finalize primary file length.");
                }
            }
            if (m_isOpen && m_sidecarFile) {
                const bool sidecarFinalized = finalizeStorageFileLength(m_sidecarFile);
                if (!sidecarFinalized) {
                    (void)fail("Failed to finalize sidecar file length.");
                    captureCloseError("Failed to finalize sidecar file length.");
                }
                else {
                    m_sidecarDirty = false;
                }
            }

            if (m_primaryFile) {
                m_primaryFile.close();
            }
            if (m_sidecarFile) {
                m_sidecarFile.close();
            }

            resetAllState();
            if (savedError[0] != '\0') {
                std::strncpy(m_lastError, savedError, sizeof(m_lastError) - 1u);
                m_lastError[sizeof(m_lastError) - 1u] = '\0';
            }
            return ok;
        }

        bool MmLogLogger::pushPrimaryQueue(const std::uint8_t* const data, const std::size_t len) {
            if (!m_primaryQueue.push(data, len)) {
                return fail("Primary queue overflow.");
            }
            return true;
        }

        bool MmLogLogger::pushSidecarQueue(const std::uint8_t* const data, const std::size_t len) {
            if (!m_sidecarQueue.push(data, len)) {
                return fail("Sidecar queue overflow.");
            }
            m_sidecarDirty = true;
            return true;
        }

        bool MmLogLogger::enqueuePrimary(const std::uint8_t* const data, const std::size_t len) {
            if (!m_isBegun) {
                return fail("Primary queue used before begin().");
            }
            return pushPrimaryQueue(data, len);
        }

        bool MmLogLogger::enqueueSidecar(const std::uint8_t* const data, const std::size_t len) {
            if (!m_isBegun) {
                return fail("Sidecar queue used before begin().");
            }
            return pushSidecarQueue(data, len);
        }

        bool MmLogLogger::drainQueueToFile(
            StorageFileHandle& file,
            detail::ByteRing& queue,
            std::size_t budget,
            const bool sectorAligned) {

            if (!file) {
                return fail("File handle is invalid.");
            }

#if !MMLOG_ENABLE_TEENSY_FIFO_SDIO
            (void)sectorAligned;
#endif

#if MMLOG_ENABLE_TEENSY_FIFO_SDIO
            if (sectorAligned) {
                std::uint8_t sector[kSdSectorBytes];
                std::size_t sectorsWritten = 0u;

                while (budget >= kSdSectorBytes && queue.size() >= kSdSectorBytes) {
                    if (file.isBusy()) {
                        break;
                    }

                    const std::uint8_t* const ptr = queue.readPtr();
                    const std::size_t contiguous = queue.contiguousReadSize();
                    const std::uint8_t* writePtr = ptr;
                    if (contiguous < kSdSectorBytes) {
                        queue.readCopy(sector, kSdSectorBytes);
                        writePtr = sector;
                    }

                    if (file.write(writePtr, kSdSectorBytes) != kSdSectorBytes) {
                        return fail("Sector write failed.");
                    }
                    queue.consume(kSdSectorBytes);
                    budget -= kSdSectorBytes;
                    ++sectorsWritten;

#  if MMLOG_TEENSY_SERVICE_AT_MOST_ONE_SECTOR_PER_CALL
                    if (sectorsWritten >= 1u) {
                        break;
                    }
#  endif
                }
                return true;
            }
#endif

            while (budget != 0u && !queue.empty()) {
                const std::size_t chunk = queue.contiguousReadSize();
                const std::size_t toWrite = (chunk < budget) ? chunk : budget;
                const std::uint8_t* const ptr = queue.readPtr();
                if (ptr == nullptr || toWrite == 0u) {
                    break;
                }
                if (file.write(ptr, toWrite) != toWrite) {
                    return fail("File write failed.");
                }
                queue.consume(toWrite);
                budget -= toWrite;
            }
            return true;
        }

        bool MmLogLogger::flushQueueToFile(
            StorageFileHandle& file,
            detail::ByteRing& queue,
            const bool sectorAligned) {

            if (!file) {
                return fail("File handle is invalid.");
            }

#if !MMLOG_ENABLE_TEENSY_FIFO_SDIO
            (void)sectorAligned;
#endif

#if MMLOG_ENABLE_TEENSY_FIFO_SDIO
            if (sectorAligned) {
                while (queue.size() >= kSdSectorBytes) {
                    if (!drainQueueToFile(file, queue, kSdSectorBytes, true)) {
                        return false;
                    }
                    while (file.isBusy()) {
                    }
                }

            }
#endif

            while (!queue.empty()) {
                const std::size_t chunk = queue.contiguousReadSize();
                const std::uint8_t* const ptr = queue.readPtr();
                if (ptr == nullptr || chunk == 0u) {
                    break;
                }
                if (file.write(ptr, chunk) != chunk) {
                    return fail("File flush write failed.");
                }
                queue.consume(chunk);
            }
            return true;
        }

        bool MmLogLogger::writeDirect(
            StorageFileHandle& file,
            const std::uint8_t* const data,
            const std::size_t len) {

            if (!file || (data == nullptr && len != 0u)) {
                return false;
            }
            return file.write(data, len) == len;
        }

        bool MmLogLogger::writeLineDirect(StorageFileHandle& file, const char* const text) {
            if (!file || text == nullptr) {
                return false;
            }

            const std::size_t len = std::strlen(text);
            static constexpr std::uint8_t newline = '\n';
            return writeDirect(file, reinterpret_cast<const std::uint8_t*>(text), len) && writeDirect(file, &newline, 1u);
        }

        bool MmLogLogger::openPrimaryForWrite() {
#if MMLOG_ENABLE_TEENSY_FIFO_SDIO
            m_primaryFile.close();
            if (!m_primaryFile.open(&SD.sdfs, m_primaryPath, O_RDWR | O_CREAT | O_TRUNC)) {
                return fail("Failed to open primary file.");
            }
#  if MMLOG_TEENSY_PRIMARY_PREALLOCATE_BYTES > 0
            if (!m_primaryFile.preAllocate(MMLOG_TEENSY_PRIMARY_PREALLOCATE_BYTES)) {
                return fail("Primary preAllocate() failed.");
            }
#  endif
            return true;
#elif defined(ARDUINO)
            m_primaryFile = m_fs.open(m_primaryPath, FILE_WRITE);
            if (!m_primaryFile) {
                return fail("Failed to open primary file.");
            }
            return true;
#else
            if (!m_primaryFile.openWrite(m_primaryPath)) {
                return fail("Failed to open primary file.");
            }
            return true;
#endif
        }

        bool MmLogLogger::openSidecarForWrite() {
#if MMLOG_ENABLE_TEENSY_FIFO_SDIO
            m_sidecarFile.close();
            if (!m_sidecarFile.open(&SD.sdfs, m_sidecarPath, O_RDWR | O_CREAT | O_TRUNC)) {
                return fail("Failed to open sidecar file.");
            }
#  if MMLOG_TEENSY_SIDECAR_PREALLOCATE_BYTES > 0
            if (!m_sidecarFile.preAllocate(MMLOG_TEENSY_SIDECAR_PREALLOCATE_BYTES)) {
                return fail("Sidecar preAllocate() failed.");
            }
#  endif
            return true;
#elif defined(ARDUINO)
            m_sidecarFile = m_fs.open(m_sidecarPath, FILE_WRITE);
            if (!m_sidecarFile) {
                return fail("Failed to open sidecar file.");
            }
            return true;
#else
            if (!m_sidecarFile.openWrite(m_sidecarPath)) {
                return fail("Failed to open sidecar file.");
            }
            return true;
#endif
        }

        bool MmLogLogger::removePrimaryFileIfPresent(const char* const path) {
#if MMLOG_ENABLE_TEENSY_FIFO_SDIO
            if (SD.sdfs.exists(path) && !SD.sdfs.remove(path)) {
                return fail("Failed to remove existing file.");
            }
            return true;
#elif defined(ARDUINO)
            if (m_fs.exists(path) && !m_fs.remove(path)) {
                return fail("Failed to remove existing file.");
            }
            return true;
#else
            if (std::remove(path) != 0) {
                return true;
            }
            return true;
#endif
        }

        bool MmLogLogger::removeSidecarFileIfPresent(const char* const path) {
#if MMLOG_ENABLE_TEENSY_FIFO_SDIO
            if (SD.sdfs.exists(path) && !SD.sdfs.remove(path)) {
                return fail("Failed to remove existing file.");
            }
            return true;
#elif defined(ARDUINO)
            if (m_fs.exists(path) && !m_fs.remove(path)) {
                return fail("Failed to remove existing file.");
            }
            return true;
#else
            if (std::remove(path) != 0) {
                return true;
            }
            return true;
#endif
        }

        bool MmLogLogger::derivePaths(const char* const file_name) {
            char stem[MMLOG_MAX_PATH_LENGTH + 1u]{};
            if (!makeStemPath(stem, sizeof(stem), file_name)) {
                return fail("File name is too long.");
            }

            if (!safeCopy(m_primaryPath, sizeof(m_primaryPath), stem)) {
                return fail("Primary path buffer overflow.");
            }
            if (!safeAppend(m_primaryPath, sizeof(m_primaryPath), kPrimaryExtension)) {
                return fail("Primary path buffer overflow.");
            }

            if (!safeCopy(m_sidecarPath, sizeof(m_sidecarPath), stem)) {
                return fail("Sidecar path buffer overflow.");
            }
            if (!safeAppend(m_sidecarPath, sizeof(m_sidecarPath), kSidecarExtension)) {
                return fail("Sidecar path buffer overflow.");
            }

            const char* const base = findLastSlash(m_sidecarPath);
            const char* const binding = (base == nullptr) ? m_sidecarPath : (base + 1);
            if (!safeCopy(m_sidecarBinding, sizeof(m_sidecarBinding), binding)) {
                return fail("Sidecar binding buffer overflow.");
            }

            return true;
        }

        bool MmLogLogger::validateMetadataToken(const char* const text) const noexcept {
            return !hasInvalidTextChar(text);
        }

        bool MmLogLogger::fail(const char* const text) {
            const char* const message = (text == nullptr) ? "Unknown MmLog error." : text;
            std::strncpy(m_lastError, message, sizeof(m_lastError) - 1u);
            m_lastError[sizeof(m_lastError) - 1u] = '\0';
            return false;
        }

        void MmLogLogger::clearError() noexcept {
            m_lastError[0] = '\0';
        }

        void MmLogLogger::resetSessionState() noexcept {
#if MMLOG_ENABLE_TEENSY_FIFO_SDIO || defined(ARDUINO)
            if (m_primaryFile) {
                m_primaryFile.close();
            }
            if (m_sidecarFile) {
                m_sidecarFile.close();
            }
#else
            m_primaryFile.close();
            m_sidecarFile.close();
#endif

            if (m_storageAttached) {
                m_primaryQueue.clear();
                m_sidecarQueue.clear();
            }

            m_primaryPath[0] = '\0';
            m_sidecarPath[0] = '\0';
            m_sidecarBinding[0] = '\0';
            m_activeRowBytes = 0u;
            m_activeSchemaHash = 0u;
            m_sidecarDirty = false;
            m_isBegun = false;
            m_labelSectionStarted = false;
        }

        void MmLogLogger::resetAllState() noexcept {
            resetSessionState();
            m_isOpen = false;
            clearError();
        }

        bool MmLogLogger::isReservedMetadataKey(const char* const key) const noexcept {
            return std::strcmp(key, "schema_version") == 0 || std::strcmp(key, "row_bytes") == 0;
        }

    } // namespace mmlog
}
