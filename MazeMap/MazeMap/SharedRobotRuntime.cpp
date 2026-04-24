#include "pch.h"
#include "SharedRobotRuntime.h"
#include "TeensyLayout.h"
#include "MazeMapApplicationPrivate.h"
#include "MazeMapRuntimeInfrastructure.h"
#include "RuntimeBinaryLogSupport.h"
#include "WallDistanceCalibration.h"

#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <limits>
#include <stdexcept>

using MazeMap::App::Internal::SharedRuntimeTextLogFileHandle;

static constexpr std::size_t kTextLogServiceBudgetBytes = MazeMap::mmlog::kSdSectorBytes;

#if MAZEMAP_USE_RAM2_FILE_BUFFERS
DMAMEM alignas(32) static std::uint8_t g_sharedRuntimeTextLogStorage[
    MazeMap::App::Internal::kSharedRuntimeTextLogQueueBytes];
#endif

#if MMLOG_ENABLE_TEENSY_FIFO_SDIO
static constexpr std::uint64_t kTextLogPreallocateBytes = MMLOG_TEENSY_MIN_PREALLOCATE_BYTES;
static_assert(kTextLogPreallocateBytes >= MMLOG_TEENSY_MIN_PREALLOCATE_BYTES);

static bool OpenFreshTextLogFile(SharedRuntimeTextLogFileHandle& file, const char* const path) noexcept
{
    file.close();
    if (SD.sdfs.exists(path) && !SD.sdfs.remove(path))
    {
        return false;
    }
    if (!file.open(&SD.sdfs, path, O_RDWR | O_CREAT | O_TRUNC))
    {
        return false;
    }
    if (!file.preAllocate(kTextLogPreallocateBytes))
    {
        file.close();
        return false;
    }
    return true;
}

static bool OpenAppendTextLogFile(SharedRuntimeTextLogFileHandle& file, const char* const path) noexcept
{
    file.close();
    if (!file.open(&SD.sdfs, path, O_RDWR | O_CREAT | O_AT_END))
    {
        return false;
    }
    if (file.fileSize() == 0U && !file.preAllocate(kTextLogPreallocateBytes))
    {
        file.close();
        return false;
    }
    return true;
}

static bool TextLogFileIsOpen(SharedRuntimeTextLogFileHandle& file) noexcept
{
    return static_cast<bool>(file);
}

static bool WriteTextLogBytes(
    SharedRuntimeTextLogFileHandle& file,
    const char* const text,
    const std::size_t length) noexcept
{
    return file.write(reinterpret_cast<const std::uint8_t*>(text), length) == length;
}

static bool FlushTextLogFile(SharedRuntimeTextLogFileHandle& file) noexcept
{
    return file.sync();
}

static bool TextLogTransferBusy(SharedRuntimeTextLogFileHandle& file) noexcept
{
    return file.isBusy();
}

static void CloseTextLogFile(SharedRuntimeTextLogFileHandle& file) noexcept
{
    if (file)
    {
        file.close();
    }
}

static bool FinalizeTextLogFileLength(
    SharedRuntimeTextLogFileHandle& file,
    const std::uint64_t logicalLength) noexcept
{
    if (!file.seekSet(logicalLength))
    {
        return false;
    }
    if (!file.truncate())
    {
        return false;
    }
    if (!file.seekSet(logicalLength))
    {
        return false;
    }
    return FlushTextLogFile(file);
}

static bool FlushPartialTextLogTail(
    SharedRuntimeTextLogFileHandle& file,
    MazeMap::mmlog::detail::ByteRing& queue) noexcept
{
    if (queue.empty())
    {
        return true;
    }

    std::uint8_t sector[MazeMap::mmlog::kSdSectorBytes] = {};
    const std::size_t tailBytes = queue.size();
    if (tailBytes >= MazeMap::mmlog::kSdSectorBytes)
    {
        return false;
    }

    queue.readCopy(sector, tailBytes);
    const std::uint64_t logicalLength = file.curPosition() + tailBytes;
    if (file.write(sector, MazeMap::mmlog::kSdSectorBytes) != MazeMap::mmlog::kSdSectorBytes)
    {
        return false;
    }

    queue.consume(tailBytes);
    while (file.isBusy())
    {
    }

    // Keep physical writes sector-sized, then trim the file back to the true byte count.
    return FinalizeTextLogFileLength(file, logicalLength);
}
#elif defined(ARDUINO)
static bool OpenFreshTextLogFile(SharedRuntimeTextLogFileHandle& file, const char* const path) noexcept
{
    file.close();
    SD.remove(path);
    file = SD.open(path, FILE_WRITE);
    return static_cast<bool>(file);
}

static bool OpenAppendTextLogFile(SharedRuntimeTextLogFileHandle& file, const char* const path) noexcept
{
    file.close();
    file = SD.open(path, FILE_WRITE);
    return static_cast<bool>(file);
}

static bool TextLogFileIsOpen(SharedRuntimeTextLogFileHandle& file) noexcept
{
    return static_cast<bool>(file);
}

static bool WriteTextLogBytes(
    SharedRuntimeTextLogFileHandle& file,
    const char* const text,
    const std::size_t length) noexcept
{
    return file.write(reinterpret_cast<const std::uint8_t*>(text), length) == length;
}

static bool FlushTextLogFile(SharedRuntimeTextLogFileHandle& file) noexcept
{
    file.flush();
    return static_cast<bool>(file);
}

static bool TextLogTransferBusy(SharedRuntimeTextLogFileHandle& file) noexcept
{
    (void)file;
    return false;
}

static void CloseTextLogFile(SharedRuntimeTextLogFileHandle& file) noexcept
{
    if (file)
    {
        file.close();
    }
}
#else
static bool OpenFreshTextLogFile(SharedRuntimeTextLogFileHandle& file, const char* const path) noexcept
{
    if (file != nullptr)
    {
        std::fclose(file);
        file = nullptr;
    }

    file = std::fopen(path, "wb");
    return file != nullptr;
}

static bool OpenAppendTextLogFile(SharedRuntimeTextLogFileHandle& file, const char* const path) noexcept
{
    if (file != nullptr)
    {
        std::fclose(file);
        file = nullptr;
    }

    file = std::fopen(path, "ab");
    return file != nullptr;
}

static bool TextLogFileIsOpen(SharedRuntimeTextLogFileHandle& file) noexcept
{
    return file != nullptr;
}

static bool WriteTextLogBytes(
    SharedRuntimeTextLogFileHandle& file,
    const char* const text,
    const std::size_t length) noexcept
{
    return (file != nullptr) && (std::fwrite(text, 1U, length, file) == length);
}

static bool FlushTextLogFile(SharedRuntimeTextLogFileHandle& file) noexcept
{
    return (file == nullptr) || (std::fflush(file) == 0);
}

static bool TextLogTransferBusy(SharedRuntimeTextLogFileHandle& file) noexcept
{
    (void)file;
    return false;
}

static void CloseTextLogFile(SharedRuntimeTextLogFileHandle& file) noexcept
{
    if (file != nullptr)
    {
        std::fclose(file);
        file = nullptr;
    }
}
#endif

static void ResetUtilityLogIdentity(
    char* activeFileName,
    std::size_t activeFileNameSize,
    char* activeSource,
    std::size_t activeSourceSize) noexcept
{
    if (activeFileName != nullptr && activeFileNameSize > 0U)
    {
        activeFileName[0] = '\0';
    }
    if (activeSource != nullptr && activeSourceSize > 0U)
    {
        activeSource[0] = '\0';
    }
}

static void SetRuntimeLogErrorText(char* destination, std::size_t destinationSize, const char* message) noexcept
{
    if (destination == nullptr || destinationSize == 0U)
    {
        return;
    }

    std::snprintf(destination, destinationSize, "%s", (message != nullptr) ? message : "");
}

static void AssignUtilityLogSourceFromFileName(
    char* destination,
    std::size_t destinationSize,
    const char* fileName) noexcept
{
    if (destination == nullptr || destinationSize == 0U)
    {
        return;
    }

    destination[0] = '\0';
    const char* component = MazeMap::App::Internal::Runtime::FileNameComponent(fileName);
    if (component == nullptr || component[0] == '\0')
    {
        return;
    }

    const char* extension = std::strrchr(component, '.');
    std::size_t sourceLength =
        (extension != nullptr) ? static_cast<std::size_t>(extension - component) : std::strlen(component);
    if (sourceLength >= destinationSize)
    {
        sourceLength = destinationSize - 1U;
    }

    std::memcpy(destination, component, sourceLength);
    destination[sourceLength] = '\0';
}

static bool DrainTextLogQueueToFile(
    SharedRuntimeTextLogFileHandle& file,
    MazeMap::mmlog::detail::ByteRing& queue,
    std::size_t budget) noexcept
{
    if (!TextLogFileIsOpen(file))
    {
        return false;
    }

#if MMLOG_ENABLE_TEENSY_FIFO_SDIO
    std::uint8_t sector[MazeMap::mmlog::kSdSectorBytes] = {};
    std::size_t sectorsWritten = 0U;
    while (budget >= MazeMap::mmlog::kSdSectorBytes &&
           queue.size() >= MazeMap::mmlog::kSdSectorBytes)
    {
        if (TextLogTransferBusy(file))
        {
            break;
        }

        const std::uint8_t* writePtr = queue.readPtr();
        const std::size_t contiguous = queue.contiguousReadSize();
        if (contiguous < MazeMap::mmlog::kSdSectorBytes)
        {
            queue.readCopy(sector, MazeMap::mmlog::kSdSectorBytes);
            writePtr = sector;
        }

        if (writePtr == nullptr ||
            !WriteTextLogBytes(
                file,
                reinterpret_cast<const char*>(writePtr),
                MazeMap::mmlog::kSdSectorBytes))
        {
            return false;
        }

        queue.consume(MazeMap::mmlog::kSdSectorBytes);
        budget -= MazeMap::mmlog::kSdSectorBytes;
        ++sectorsWritten;

#  if MMLOG_TEENSY_SERVICE_AT_MOST_ONE_SECTOR_PER_CALL
        if (sectorsWritten >= 1U)
        {
            break;
        }
#  endif
    }
    return true;
#else
    while (budget != 0U && !queue.empty())
    {
        const std::size_t chunk = queue.contiguousReadSize();
        const std::size_t toWrite = (chunk < budget) ? chunk : budget;
        const std::uint8_t* const ptr = queue.readPtr();
        if (ptr == nullptr || toWrite == 0U)
        {
            break;
        }

        if (!WriteTextLogBytes(file, reinterpret_cast<const char*>(ptr), toWrite))
        {
            return false;
        }

        queue.consume(toWrite);
        budget -= toWrite;
    }
    return true;
#endif
}

static bool FlushTextLogQueueToFile(
    SharedRuntimeTextLogFileHandle& file,
    MazeMap::mmlog::detail::ByteRing& queue) noexcept
{
    if (!TextLogFileIsOpen(file))
    {
        return false;
    }

#if MMLOG_ENABLE_TEENSY_FIFO_SDIO
    while (queue.size() >= MazeMap::mmlog::kSdSectorBytes)
    {
        if (!DrainTextLogQueueToFile(file, queue, MazeMap::mmlog::kSdSectorBytes))
        {
            return false;
        }
        while (TextLogTransferBusy(file))
        {
        }
    }

    return FlushPartialTextLogTail(file, queue);
#else
    while (!queue.empty())
    {
        const std::size_t chunk = queue.contiguousReadSize();
        const std::uint8_t* const ptr = queue.readPtr();
        if (ptr == nullptr || chunk == 0U)
        {
            break;
        }

        if (!WriteTextLogBytes(file, reinterpret_cast<const char*>(ptr), chunk))
        {
            return false;
        }

        queue.consume(chunk);
    }

    return true;
#endif
}

namespace MazeMap::App::Internal
{
    SharedRobotRuntime::SharedRobotRuntime()
        : speedVehicle()
        , searchVehicle()
        , maze()
        , searchPathFinder(maze, speedVehicle)
        , speedPathFinder(maze, speedVehicle)
        , wallBeliefMap()
        , plantModel()
        , runtimeState()
        , estimator(MazeMap::PlantParams::Default(), &runtimeState)
        , drive(plantModel, estimator, MazeMap::Config::kDriveBasePDCluster)
        , driveService()
        , startupCalibrationService()
        , wallTouchService()
        , maneuverExecutor()
        , controlLoop()
        , sensors(speedVehicle, gWallDistanceCalibration)
        , dataLogger()
        , textLogFile()
        , textLogQueue()
        , textLogFaulted(false)
        , textLogInitialized(false)
        , runtimeLogsClosedForFault(false)
        , modeFaultHandlerRegistered(false)
        , modeFaulted(false)
        , modeFaultCleanupCallback(nullptr)
        , modeFaultCleanupContext(nullptr)
    {
        ResetUtilityLogIdentity(
            activeDataLogFileName,
            sizeof(activeDataLogFileName),
            activeDataLogSource,
            sizeof(activeDataLogSource));
        activeModeFaultSource[0] = '\0';
        lastRuntimeLogError[0] = '\0';
#if MAZEMAP_USE_RAM2_FILE_BUFFERS
        (void)textLogQueue.attach(g_sharedRuntimeTextLogStorage, kSharedRuntimeTextLogQueueBytes);
#else
        (void)textLogQueue.attach(textLogStorage, sizeof(textLogStorage));
#endif

        // Search-mode queue timing stays intentionally conservative even though the shared
        // physical vehicle model is reused everywhere else.
        searchVehicle.SetMaxSpeed(Config::kSearchMaxSpeedMps);
        searchVehicle.SetMaxForwardAcceleration(Config::kSearchAccelMps2);
        searchVehicle.SetMaxLateralAcceleration(Config::kSearchMaxLateralAccelerationMps2);

        driveService.AttachRuntime(*this);
        startupCalibrationService.AttachRuntime(*this);
        wallTouchService.AttachRuntime(*this);
        maneuverExecutor.AttachRuntime(*this);
        controlLoop.AttachRuntime(*this);
    }

    SharedRobotRuntime::~SharedRobotRuntime()
    {
        CloseRuntimeLogsForFault();
    }

    MazeMap::Vehicle& SharedRobotRuntime::SpeedVehicle() noexcept
    {
        return speedVehicle;
    }

    const MazeMap::Vehicle& SharedRobotRuntime::SpeedVehicle() const noexcept
    {
        return speedVehicle;
    }

    MazeMap::Vehicle& SharedRobotRuntime::SearchVehicle() noexcept
    {
        return searchVehicle;
    }

    const MazeMap::Vehicle& SharedRobotRuntime::SearchVehicle() const noexcept
    {
        return searchVehicle;
    }

    MazeMap::Maze& SharedRobotRuntime::Maze() noexcept
    {
        return maze;
    }

    const MazeMap::Maze& SharedRobotRuntime::Maze() const noexcept
    {
        return maze;
    }

    MazeMap::FloodFillPathFinder& SharedRobotRuntime::SearchPathFinder() noexcept
    {
        return searchPathFinder;
    }

    MazeMap::ManeuverPathFinder& SharedRobotRuntime::SpeedPathFinder() noexcept
    {
        return speedPathFinder;
    }

    MazeMap::WallBeliefMap& SharedRobotRuntime::WallBeliefMap() noexcept
    {
        return wallBeliefMap;
    }

    bool SharedRobotRuntime::OpenUtilityDataLog(
        char* resolvedFileName,
        std::size_t bufferSize,
        const char* explicitFileName,
        const char* teensyFormat,
        const char* hostFallback)
    {
        if (!Runtime::SelectSequentialRuntimeFileName(
                resolvedFileName,
                bufferSize,
                explicitFileName,
                teensyFormat,
                hostFallback))
        {
            return false;
        }

        return OpenUtilityDataLogFile(resolvedFileName);
    }

    bool SharedRobotRuntime::OpenUtilityDataLogFile(const char* fileName)
    {
        ClearLastRuntimeLogError();
        if (fileName == nullptr || fileName[0] == '\0')
        {
            return false;
        }

        (void)CloseUtilityDataLog();
        if (!EnsureTextLogOpen())
        {
            return false;
        }

        snprintf(activeDataLogFileName, sizeof(activeDataLogFileName), "%s", fileName);
        AssignUtilityLogSourceFromFileName(activeDataLogSource, sizeof(activeDataLogSource), fileName);
        if (!UtilityDataLogger().open(fileName))
        {
            SetLastRuntimeLogErrorFromUtilityDataLogger("Failed to open utility data log file.");
            LogUtilityDataLoggerFailure("data_log_open_failed");
            ResetUtilityLogIdentity(
                activeDataLogFileName,
                sizeof(activeDataLogFileName),
                activeDataLogSource,
                sizeof(activeDataLogSource));
            return false;
        }

        if (!WriteUtilityDataLogMetadata("file", fileName) ||
            !WriteUtilityDataLogMetadata("control_log_file", TextLogFileName()))
        {
            char savedError[MMLOG_ERROR_TEXT_LENGTH + 1U] = {};
            SetRuntimeLogErrorText(savedError, sizeof(savedError), LastRuntimeLogError());
            (void)CloseUtilityDataLog();
            SetLastRuntimeLogError(savedError);
            return false;
        }

        return true;
    }

    const char* SharedRobotRuntime::ActiveUtilityDataLogFileName() const noexcept
    {
        return activeDataLogFileName;
    }

    const char* SharedRobotRuntime::TextLogFileName() const noexcept
    {
        return kSharedRuntimeTextLogFileName;
    }

    const char* SharedRobotRuntime::LastRuntimeLogError() const noexcept
    {
        return lastRuntimeLogError;
    }

    void SharedRobotRuntime::ClearLastRuntimeLogError() noexcept
    {
        SetRuntimeLogErrorText(lastRuntimeLogError, sizeof(lastRuntimeLogError), nullptr);
    }

    void SharedRobotRuntime::SetLastRuntimeLogError(const char* message) noexcept
    {
        SetRuntimeLogErrorText(lastRuntimeLogError, sizeof(lastRuntimeLogError), message);
    }

    void SharedRobotRuntime::SetLastRuntimeLogErrorFromUtilityDataLogger(const char* fallback) noexcept
    {
        const char* const error = UtilityDataLogger().lastError();
        SetLastRuntimeLogError((error != nullptr && error[0] != '\0') ? error : fallback);
    }

    bool SharedRobotRuntime::EnsureTextLogOpen()
    {
        if (textLogFaulted || runtimeLogsClosedForFault)
        {
            return false;
        }

        if (TextLogFileIsOpen(textLogFile))
        {
            return true;
        }

        const bool opened =
            textLogInitialized
            ? OpenAppendTextLogFile(textLogFile, kSharedRuntimeTextLogFileName)
            : OpenFreshTextLogFile(textLogFile, kSharedRuntimeTextLogFileName);
        if (!opened)
        {
            textLogFaulted = true;
            CloseRuntimeLogsForFault();
            return false;
        }
        textLogInitialized = true;
        return true;
    }

    bool SharedRobotRuntime::TextLogIsOpen() noexcept
    {
        return TextLogFileIsOpen(textLogFile);
    }

    bool SharedRobotRuntime::AppendTextLogLine(const char* line)
    {
        ClearLastRuntimeLogError();
        if (line == nullptr || line[0] == '\0' || !EnsureTextLogOpen())
        {
            return false;
        }

        const std::size_t length = std::strlen(line);
        static constexpr std::uint8_t newline = '\n';
        const std::size_t required = length + 1U;
        if (required > textLogQueue.freeSpace() ||
            !textLogQueue.push(reinterpret_cast<const std::uint8_t*>(line), length) ||
            !textLogQueue.push(&newline, 1U))
        {
            SetLastRuntimeLogError("logging.txt queue overflow.");
            textLogFaulted = true;
            CloseRuntimeLogsForFault();
            return false;
        }
        return true;
    }

    bool SharedRobotRuntime::AppendTextLogFormatted(const char* format, ...)
    {
        if (format == nullptr)
        {
            return false;
        }

        char line[384] = {};
        va_list args;
        va_start(args, format);
        const int length = vsnprintf(line, sizeof(line), format, args);
        va_end(args);
        if (length <= 0)
        {
            return false;
        }

        line[sizeof(line) - 1U] = '\0';
        return AppendTextLogLine(line);
    }

    bool SharedRobotRuntime::WriteTextLogEntry(
        const char* source,
        unsigned long timestampUs,
        const char* type,
        const char* message)
    {
        char line[384] = {};
        const int length = snprintf(
            line,
            sizeof(line),
            "%s [%lu] %s%s%s",
            (source != nullptr && source[0] != '\0') ? source : "runtime",
            timestampUs,
            (type != nullptr && type[0] != '\0') ? type : "event",
            (message != nullptr && message[0] != '\0') ? ": " : "",
            (message != nullptr) ? message : "");
        if (length <= 0)
        {
            return false;
        }

        line[sizeof(line) - 1U] = '\0';
        return AppendTextLogLine(line);
    }

    const char* SharedRobotRuntime::DefaultTextLogSource() const noexcept
    {
        if (activeDataLogSource[0] != '\0')
        {
            return activeDataLogSource;
        }

        if (activeModeFaultSource[0] != '\0')
        {
            return activeModeFaultSource;
        }

        return "runtime";
    }

    void SharedRobotRuntime::LogUtilityDataLoggerFailure(const char* type) noexcept
    {
        char savedError[MMLOG_ERROR_TEXT_LENGTH + 1U] = {};
        SetRuntimeLogErrorText(savedError, sizeof(savedError), LastRuntimeLogError());
        const char* const error = UtilityDataLogger().lastError();
        if (error == nullptr || error[0] == '\0')
        {
            return;
        }

        (void)WriteTextLogEntry("runtime", micros(), (type != nullptr) ? type : "data_log_failed", error);
        SetLastRuntimeLogError(savedError);
    }

    bool SharedRobotRuntime::WriteTextLogEntry(
        unsigned long timestampUs,
        const char* type,
        const char* message)
    {
        return WriteTextLogEntry(DefaultTextLogSource(), timestampUs, type, message);
    }

    bool SharedRobotRuntime::WriteTextLogMetadata(const char* key, const char* value)
    {
        return WriteTextLogMetadata(DefaultTextLogSource(), key, value);
    }

    bool SharedRobotRuntime::WriteTextLogMetadata(const char* source, const char* key, const char* value)
    {
        char message[256] = {};
        const int length = snprintf(
            message,
            sizeof(message),
            "%s=%s",
            (key != nullptr) ? key : "",
            (value != nullptr) ? value : "");
        if (length <= 0)
        {
            return false;
        }

        message[sizeof(message) - 1U] = '\0';
        return WriteTextLogEntry(source, micros(), "metadata", message);
    }

    bool SharedRobotRuntime::WriteTextLogPhase(
        unsigned long phaseId,
        unsigned long timestampUs,
        const char* name)
    {
        return WriteTextLogPhase(DefaultTextLogSource(), phaseId, timestampUs, name);
    }

    bool SharedRobotRuntime::WriteTextLogPhase(
        const char* source,
        unsigned long phaseId,
        unsigned long timestampUs,
        const char* name)
    {
        char message[256] = {};
        const int length = snprintf(
            message,
            sizeof(message),
            "phase_id=%lu;name=%s",
            phaseId,
            (name != nullptr) ? name : "");
        if (length <= 0)
        {
            return false;
        }

        message[sizeof(message) - 1U] = '\0';
        return WriteTextLogEntry(source, timestampUs, "phase", message);
    }

    void SharedRobotRuntime::FlushTextLog()
    {
        // Explicit text flushes are complete writes. They are separate from normal
        // per-cycle servicing, which is arbitrated only by ServiceUtilityDataLog.
        ClearLastRuntimeLogError();
        if (TextLogFileIsOpen(textLogFile))
        {
            if ((!textLogQueue.empty() &&
                 !FlushTextLogQueueToFile(textLogFile, textLogQueue)) ||
                !FlushTextLogFile(textLogFile))
            {
                SetLastRuntimeLogError("logging.txt flush failed.");
                textLogFaulted = true;
                CloseRuntimeLogsForFault();
            }
        }
    }

    void SharedRobotRuntime::CloseTextLog()
    {
        // Text-log close is a lifecycle/fault finalization path; normal control-loop
        // dispatch remains centralized in ServiceUtilityDataLog.
        ClearLastRuntimeLogError();
        if (TextLogFileIsOpen(textLogFile))
        {
            if ((!textLogQueue.empty() &&
                 !FlushTextLogQueueToFile(textLogFile, textLogQueue)) ||
                !FlushTextLogFile(textLogFile))
            {
                SetLastRuntimeLogError("logging.txt close failed.");
                textLogFaulted = true;
                CloseTextLogFile(textLogFile);
                textLogQueue.clear();
                return;
            }
            CloseTextLogFile(textLogFile);
        }
        textLogQueue.clear();
    }

    bool SharedRobotRuntime::RegisterModeFaultHandler(
        ModeFaultCleanupCallback callback,
        void* context,
        const char* source)
    {
        if (modeFaultHandlerRegistered)
        {
#ifdef ARDUINO_TEENSY41
            assert(false && "SharedRobotRuntime mode fault handler already registered; top-level mode switching is not supported.");
            return false;
#else
            throw std::logic_error(
                "SharedRobotRuntime mode fault handler already registered; top-level mode switching is not supported.");
#endif
        }

        modeFaultHandlerRegistered = true;
        modeFaulted = false;
        modeFaultCleanupCallback = callback;
        modeFaultCleanupContext = context;
        if (source == nullptr || source[0] == '\0')
        {
            activeModeFaultSource[0] = '\0';
        }
        else
        {
            snprintf(activeModeFaultSource, sizeof(activeModeFaultSource), "%s", source);
        }
        return true;
    }

    bool SharedRobotRuntime::FailActiveMode(const char* reason) noexcept
    {
        if (modeFaulted)
        {
            return false;
        }

        modeFaulted = true;
        SetMissionLevelFanEnabled(false);
        drive.Brake();
        drive.UseNominalWheelControlProfile();
        (void)WriteTextLogEntry(
            (activeModeFaultSource[0] != '\0') ? activeModeFaultSource : nullptr,
            micros(),
            "fault",
            (reason != nullptr) ? reason : "unknown");

        if (modeFaultCleanupCallback != nullptr)
        {
            modeFaultCleanupCallback(modeFaultCleanupContext, reason);
        }

        CloseRuntimeLogsForFault();
        StartRuntimeFaultIndicatorBlink();
        return false;
    }

    void SharedRobotRuntime::FinalizeSuccessfulModeExit() noexcept
    {
        if (modeFaulted || runtimeLogsClosedForFault)
        {
            return;
        }

        if (UtilityDataLogger().isOpen())
        {
            (void)CloseUtilityDataLog();
        }

        if (TextLogFileIsOpen(textLogFile))
        {
            CloseTextLog();
        }
    }

    bool SharedRobotRuntime::WriteUtilityDataLogMetadata(const char* key, const char* value)
    {
        ClearLastRuntimeLogError();
        const bool ok = UtilityDataLogger().writeMetadata(key, value);
        if (!ok)
        {
            SetLastRuntimeLogErrorFromUtilityDataLogger("Failed to write utility data log metadata.");
            LogUtilityDataLoggerFailure("data_log_metadata_failed");
        }
        return ok;
    }

    bool SharedRobotRuntime::WriteUtilityDataLogMetadataUnsigned(const char* key, unsigned long value)
    {
        ClearLastRuntimeLogError();
        const bool ok = Runtime::WriteMmLogMetadataUnsigned(UtilityDataLogger(), key, value);
        if (!ok)
        {
            SetLastRuntimeLogErrorFromUtilityDataLogger("Failed to write utility data log metadata.");
            LogUtilityDataLoggerFailure("data_log_metadata_failed");
        }
        return ok;
    }

    bool SharedRobotRuntime::WriteUtilityDataLogMetadataFloat(const char* key, float value, std::uint8_t precision)
    {
        ClearLastRuntimeLogError();
        const bool ok = Runtime::WriteMmLogMetadataFloat(UtilityDataLogger(), key, value, precision);
        if (!ok)
        {
            SetLastRuntimeLogErrorFromUtilityDataLogger("Failed to write utility data log metadata.");
            LogUtilityDataLoggerFailure("data_log_metadata_failed");
        }
        return ok;
    }

    bool SharedRobotRuntime::WriteUtilityDataLogAccelBiasMetadata(const RuntimeSensorSuite& sensors)
    {
        ClearLastRuntimeLogError();
        const bool ok = Runtime::WriteMmLogAccelBiasMetadata(UtilityDataLogger(), sensors);
        if (!ok)
        {
            SetLastRuntimeLogErrorFromUtilityDataLogger("Failed to write utility data log metadata.");
            LogUtilityDataLoggerFailure("data_log_metadata_failed");
        }
        return ok;
    }

    bool SharedRobotRuntime::ServiceUtilityDataLog()
    {
        // This is the single runtime-owned file-service hub. Control loops call it only
        // from per-cycle dead time; if there is no dead time, timing control wins. The hub
        // arbitrates logging.txt first, then delegates sidecar/primary arbitration to the
        // one runtime-owned MmLogLogger instance.
        ClearLastRuntimeLogError();
        if (runtimeLogsClosedForFault)
        {
            return false;
        }

        if (TextLogTransferBusy(textLogFile) || UtilityDataLogger().isTransferBusy())
        {
            return true;
        }

        if (TextLogFileIsOpen(textLogFile) &&
            textLogQueue.size() >= MazeMap::mmlog::kSdSectorBytes)
        {
            if (!DrainTextLogQueueToFile(textLogFile, textLogQueue, kTextLogServiceBudgetBytes))
            {
                SetLastRuntimeLogError("logging.txt service drain failed.");
                textLogFaulted = true;
                CloseRuntimeLogsForFault();
                return false;
            }

            return true;
        }

        const bool ok = UtilityDataLogger().service();
        if (!ok)
        {
            SetLastRuntimeLogErrorFromUtilityDataLogger("Failed to service utility data log.");
        }
        return ok;
    }

    bool SharedRobotRuntime::FlushUtilityDataLog()
    {
        // Explicit flushes are lifecycle operations. Normal control-loop servicing goes
        // through ServiceUtilityDataLog so all file arbitration stays in SharedRobotRuntime.
        ClearLastRuntimeLogError();
        const bool ok = UtilityDataLogger().flush();
        if (!ok)
        {
            SetLastRuntimeLogErrorFromUtilityDataLogger("Failed to flush utility data log.");
        }
        return ok;
    }

    bool SharedRobotRuntime::CloseUtilityDataLog()
    {
        // Close is the phase/fault finalization path for the runtime-owned mmlog files;
        // it must not be replaced by mode-local file servicing.
        ClearLastRuntimeLogError();
        const bool ok = UtilityDataLogger().close();
        if (!ok)
        {
            SetLastRuntimeLogErrorFromUtilityDataLogger("Failed to close utility data log.");
        }
        ResetUtilityLogIdentity(
            activeDataLogFileName,
            sizeof(activeDataLogFileName),
            activeDataLogSource,
            sizeof(activeDataLogSource));
        return ok;
    }

    void SharedRobotRuntime::CloseRuntimeLogsForFault() noexcept
    {
        runtimeLogsClosedForFault = true;

        if (TextLogFileIsOpen(textLogFile))
        {
            (void)CloseTextLog();
        }
        textLogQueue.clear();

        if (UtilityDataLogger().isOpen())
        {
            (void)UtilityDataLogger().close();
        }

        ResetUtilityLogIdentity(
            activeDataLogFileName,
            sizeof(activeDataLogFileName),
            activeDataLogSource,
            sizeof(activeDataLogSource));
    }

    void SharedRobotRuntime::CaptureUtilityDataLogFailure(bool& overflowed, bool& writeFailed) const noexcept
    {
        Runtime::CaptureMmLogFailure(UtilityDataLogger(), overflowed, writeFailed);
    }

    bool SharedRobotRuntime::SetMotorPWM(const float leftMotorPwm, const float rightMotorPwm) noexcept
    {
        if (!std::isfinite(leftMotorPwm) || !std::isfinite(rightMotorPwm))
        {
            drive.Brake();
        }
        else
        {
            drive.CommandOpenLoopRaw(
                LoopController::ControlVector::RawMotorPwm(leftMotorPwm, rightMotorPwm));
        }

        return true;
    }

    DriveBase& SharedRobotRuntime::Drive() noexcept
    {
        return drive;
    }

    MazeMap::Estimator& SharedRobotRuntime::Estimator() noexcept
    {
        return estimator;
    }

    MazeMap::VehicleState& SharedRobotRuntime::RuntimeState() noexcept
    {
        return runtimeState;
    }

    MazeMap::App::Internal::Drive& SharedRobotRuntime::DriveService() noexcept
    {
        return driveService;
    }

    StartupCalibration& SharedRobotRuntime::StartupCalibrationService() noexcept
    {
        return startupCalibrationService;
    }

    WallTouch& SharedRobotRuntime::WallTouchService() noexcept
    {
        return wallTouchService;
    }

    ManeuverExecutor& SharedRobotRuntime::ManeuverExecutorService() noexcept
    {
        return maneuverExecutor;
    }

    LoopController& SharedRobotRuntime::ControlLoop() noexcept
    {
        return controlLoop;
    }

    RuntimeSensorSuite& SharedRobotRuntime::Sensors() noexcept
    {
        return sensors;
    }

    MazeMap::mmlog::MmLogLogger& SharedRobotRuntime::UtilityDataLogger() noexcept
    {
        return dataLogger;
    }

    const MazeMap::mmlog::MmLogLogger& SharedRobotRuntime::UtilityDataLogger() const noexcept
    {
        return dataLogger;
    }

    void LogWallSensorAdcRegisterWrite(
        const char* phase,
        std::uint32_t expectedCfg,
        std::uint32_t readCfg,
        std::uint32_t readGc) noexcept
    {
#if defined(ARDUINO_TEENSY41)
        static bool loggedFirstNominalApply = false;
        static std::uint32_t lastLoggedReadCfg = std::numeric_limits<std::uint32_t>::max();
        static std::uint32_t lastLoggedReadGc = std::numeric_limits<std::uint32_t>::max();
        static char lastLoggedPhase[32] = {};

        SharedRobotRuntime& runtime = GetSharedRobotRuntime();
        if (!runtime.TextLogIsOpen())
        {
            return;
        }

        const char* const resolvedPhase = ((phase != nullptr) && (phase[0] != '\0')) ? phase : "unknown";
        const bool nominal = (readCfg == expectedCfg) && (readGc == 0U);
        if (nominal && (std::strcmp(resolvedPhase, "apply") == 0))
        {
            if (loggedFirstNominalApply)
            {
                return;
            }

            loggedFirstNominalApply = true;
        }
        else if ((readCfg == lastLoggedReadCfg) &&
            (readGc == lastLoggedReadGc) &&
            (std::strncmp(lastLoggedPhase, resolvedPhase, sizeof(lastLoggedPhase) - 1U) == 0))
        {
            return;
        }

        lastLoggedReadCfg = readCfg;
        lastLoggedReadGc = readGc;
        std::snprintf(lastLoggedPhase, sizeof(lastLoggedPhase), "%s", resolvedPhase);
        (void)runtime.AppendTextLogFormatted(
            "wall_adc_%s expected_cfg=%lu read_cfg=%lu read_gc=%lu",
            resolvedPhase,
            static_cast<unsigned long>(expectedCfg),
            static_cast<unsigned long>(readCfg),
            static_cast<unsigned long>(readGc));
#else
        (void)phase;
        (void)expectedCfg;
        (void)readCfg;
        (void)readGc;
#endif
    }

    SharedRobotRuntime& GetSharedRobotRuntime()
    {
        static SharedRobotRuntime runtime;
        return runtime;
    }
}
