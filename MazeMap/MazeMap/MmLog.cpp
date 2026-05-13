#include "pch.h"
#include "MmLog.h"

#include <cstdio>
#include <cstring>

namespace MazeMap {
    namespace mmlog {

#if MMLOG_ENABLE_TEENSY_FIFO_SDIO
        DMAMEM alignas(32) std::uint8_t MmLogLogger::s_primaryStorage[MMLOG_PRIMARY_QUEUE_BYTES]{};
        DMAMEM alignas(32) std::uint8_t MmLogLogger::s_sidecarStorage[MMLOG_SIDECAR_QUEUE_BYTES]{};
        bool MmLogLogger::s_teensyStorageClaimed = false;
#endif

        void MmLogLogger::copyTruncatedText(
            char* const destination,
            const std::size_t destinationSize,
            const char* const source) noexcept {

            if (destination == nullptr || destinationSize == 0u) {
                return;
            }

            const char* const resolvedSource = (source == nullptr) ? "" : source;
#if defined(_MSC_VER)
            (void)strncpy_s(destination, destinationSize, resolvedSource, _TRUNCATE);
#else
            std::strncpy(destination, resolvedSource, destinationSize - 1u);
            destination[destinationSize - 1u] = '\0';
#endif
        }

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
            if (s_teensyStorageClaimed) {
                return false;
            }
            s_teensyStorageClaimed = true;
            if (!m_primaryQueue.attach(s_primaryStorage, sizeof(s_primaryStorage))) {
                s_teensyStorageClaimed = false;
                return false;
            }
            if (!m_sidecarQueue.attach(s_sidecarStorage, sizeof(s_sidecarStorage))) {
                s_teensyStorageClaimed = false;
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
                s_teensyStorageClaimed = false;
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
            if (!removeFileIfPresent(m_primaryPath)) {
                sidecarReady = false;
            }
            else if (!removeFileIfPresent(m_sidecarPath)) {
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
            bool invalidKey = (key == nullptr || key[0] == '\0');
            for (std::size_t i = 0u; !invalidKey && key[i] != '\0'; ++i) {
                const char c = key[i];
                invalidKey = (c == '=' || c == '\n' || c == '\r');
            }
            if (invalidKey || value == nullptr || value[0] == '\0') {
                return fail("Invalid metadata key or value.");
            }
            if (std::strchr(value, '\n') != nullptr || std::strchr(value, '\r') != nullptr) {
                return fail("Metadata values may not contain newlines.");
            }
            if (std::strcmp(key, "schema_version") == 0 || std::strcmp(key, "row_bytes") == 0) {
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
            static constexpr std::uint8_t newline = '\n';
            if (!m_sidecarFile ||
                m_sidecarFile.write(reinterpret_cast<const std::uint8_t*>(line), static_cast<std::size_t>(length)) != static_cast<std::size_t>(length) ||
                m_sidecarFile.write(&newline, 1u) != 1u) {
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

            char line[kLineBufferChars]{};
            std::snprintf(line, sizeof(line), "schema_version=%lu", static_cast<unsigned long>(schemaVersion));
            if (!queueSidecarLine(line)) {
                return fail("Failed to queue schema_version.");
            }

            std::snprintf(line, sizeof(line), "row_bytes=%lu", static_cast<unsigned long>(rowBytes));
            if (!queueSidecarLine(line)) {
                return fail("Failed to queue row_bytes.");
            }

            if (!queueSidecarLine(header)) {
                return fail("Failed to queue header line.");
            }

            {
                const char* primaryError = nullptr;
                if (!openPrimaryForWrite()) {
                    primaryError = (m_lastError[0] != '\0') ? m_lastError : "Failed to prepare primary file.";
                }
                else {
                    char bindingLine[kLineBufferChars]{};
                    if (std::snprintf(bindingLine, sizeof(bindingLine), "sidecar_file=%s", m_sidecarBinding) >= static_cast<int>(sizeof(bindingLine))) {
                        primaryError = "Sidecar binding line too long.";
                    }
                    else {
                        const std::size_t bindingLineLength = std::strlen(bindingLine);
                        static constexpr std::uint8_t newline = '\n';
                        if (!queuePrimaryBytes(reinterpret_cast<const std::uint8_t*>(bindingLine), bindingLineLength) ||
                            !queuePrimaryBytes(&newline, 1u)) {
                            primaryError = "Failed to queue primary sidecar binding line.";
                        }
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

        bool MmLogLogger::logImpl(
            const void* const row,
            const std::size_t rowBytes,
            const std::uint32_t schemaHash) {

            if (!m_isBegun) {
                return fail("log() called before begin().");
            }
            if (row == nullptr) {
                return fail("log() received a null row.");
            }
            if (rowBytes != m_activeRowBytes || schemaHash != m_activeSchemaHash) {
                return fail("log() row type does not match active schema.");
            }
            return queuePrimaryBytes(static_cast<const std::uint8_t*>(row), rowBytes);
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
                if (!queueSidecarBytes(reinterpret_cast<const std::uint8_t*>(kLabelsMarker), kLabelsMarkerBytes)) {
                    return false;
                }
                m_labelSectionStarted = true;
            }

            return queueSidecarLine(lookupString);
        }

        bool MmLogLogger::isTransferBusy() const noexcept {
#if MMLOG_ENABLE_TEENSY_FIFO_SDIO
            return
                (m_primaryFile && const_cast<StorageFileHandle&>(m_primaryFile).isBusy()) ||
                (m_sidecarFile && const_cast<StorageFileHandle&>(m_sidecarFile).isBusy());
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
                    syncFile(m_sidecarFile);
                if (!sidecarFlushed) {
                    return fail("Failed to flush sidecar file.");
                }
                m_sidecarDirty = false;
            }
            if (m_primaryFile && !flushQueueToFile(m_primaryFile, m_primaryQueue, true)) {
                return false;
            }
            if (m_primaryFile && !syncFile(m_primaryFile)) {
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
                copyTruncatedText(savedError, sizeof(savedError), source);
            };

            if (m_isOpen && !flush()) {
                captureCloseError("Failed to flush log files.");
            }

            if (m_isOpen && m_primaryFile) {
                const bool primaryFinalized = finalizeFileLength(m_primaryFile);
                if (!primaryFinalized) {
                    (void)fail("Failed to finalize primary file length.");
                    captureCloseError("Failed to finalize primary file length.");
                }
            }
            if (m_isOpen && m_sidecarFile) {
                const bool sidecarFinalized = finalizeFileLength(m_sidecarFile);
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
                copyTruncatedText(m_lastError, sizeof(m_lastError), savedError);
            }
            return ok;
        }

        bool MmLogLogger::queuePrimaryBytes(const std::uint8_t* const data, const std::size_t len) {
            if (!m_primaryQueue.push(data, len)) {
                return fail("Primary queue overflow.");
            }
            return true;
        }

        bool MmLogLogger::queueSidecarBytes(const std::uint8_t* const data, const std::size_t len) {
            if (!m_sidecarQueue.push(data, len)) {
                return fail("Sidecar queue overflow.");
            }
            m_sidecarDirty = true;
            return true;
        }

        bool MmLogLogger::queueSidecarLine(const char* const text) {
            if (text == nullptr) {
                return false;
            }

            const std::size_t len = std::strlen(text);
            static constexpr std::uint8_t newline = '\n';
            return
                queueSidecarBytes(reinterpret_cast<const std::uint8_t*>(text), len) &&
                queueSidecarBytes(&newline, 1u);
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

        bool MmLogLogger::syncFile(StorageFileHandle& file) noexcept {
#if MMLOG_ENABLE_TEENSY_FIFO_SDIO
            return file.sync();
#elif defined(ARDUINO)
            file.flush();
            return static_cast<bool>(file);
#else
            return file.flush();
#endif
        }

        bool MmLogLogger::finalizeFileLength(StorageFileHandle& file) noexcept {
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
            return syncFile(file);
#else
            (void)file;
            return true;
#endif
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

        bool MmLogLogger::removeFileIfPresent(const char* const path) {
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
            if (file_name == nullptr) {
                return fail("open() requires a non-empty file name.");
            }

            char stem[MMLOG_MAX_PATH_LENGTH + 1u]{};
            const std::size_t inputLength = std::strlen(file_name);
            const std::size_t primaryExtensionLength = std::strlen(kPrimaryExtension);
            const std::size_t sidecarExtensionLength = std::strlen(kSidecarExtension);

            std::size_t stemLength = inputLength;
            if (inputLength >= primaryExtensionLength &&
                std::strcmp(file_name + inputLength - primaryExtensionLength, kPrimaryExtension) == 0) {
                stemLength -= primaryExtensionLength;
            }
            if (stemLength >= sizeof(stem)) {
                return fail("File name is too long.");
            }
            std::memcpy(stem, file_name, stemLength);
            stem[stemLength] = '\0';

            if (stemLength + primaryExtensionLength >= sizeof(m_primaryPath)) {
                return fail("Primary path buffer overflow.");
            }
            std::memcpy(m_primaryPath, stem, stemLength);
            std::memcpy(m_primaryPath + stemLength, kPrimaryExtension, primaryExtensionLength + 1u);

            if (stemLength + sidecarExtensionLength >= sizeof(m_sidecarPath)) {
                return fail("Sidecar path buffer overflow.");
            }
            std::memcpy(m_sidecarPath, stem, stemLength);
            std::memcpy(m_sidecarPath + stemLength, kSidecarExtension, sidecarExtensionLength + 1u);

            // The primary file stores only the sidecar file name, not the full path.
            const char* binding = m_sidecarPath;
            for (const char* p = m_sidecarPath; *p != '\0'; ++p) {
                if (*p == '/' || *p == '\\') {
                    binding = p + 1;
                }
            }
            const std::size_t bindingLength = std::strlen(binding);
            if (bindingLength >= sizeof(m_sidecarBinding)) {
                return fail("Sidecar binding buffer overflow.");
            }
            std::memcpy(m_sidecarBinding, binding, bindingLength + 1u);

            return true;
        }

        bool MmLogLogger::fail(const char* const text) {
            const char* const message = (text == nullptr) ? "Unknown MmLog error." : text;
            copyTruncatedText(m_lastError, sizeof(m_lastError), message);
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

    } // namespace mmlog
}
