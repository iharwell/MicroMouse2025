#include "..\..\MazeMap\MazeMap\Estimator.h"
#include "..\..\MazeMap\MazeMap\MazeMapRuntimeCore.h"
#include "..\..\MazeMap\MazeMap\OpenFloorMeasurementSpec.h"
#include "..\..\MazeMap\MazeMap\PlantModel.h"
#include "..\..\MazeMap\MazeMap\Vehicle.h"
#include "..\..\MazeMap\MazeMap\VehicleState.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

    constexpr const char* kCsvFileName = "open_floor_main.csv";
    constexpr const char* kSidecarFileName = "open_floor_main.sidecar";
    constexpr const char* kReportFileName = "report.md";
    constexpr const char* kRunSummaryFileName = "run_summary.csv";
    constexpr const char* kSectionPhaseSummaryFileName = "section_phase_summary.csv";
    constexpr const char* kAggregateMetricsFileName = "aggregate_metrics.json";
    constexpr const char* kFeedforwardPathSummaryFileName = "feedforward_path_summary.csv";
    constexpr float kRadiansToDegrees = 57.295779513082320876f;
    constexpr float kDegreesToRadians = 0.01745329251994329577f;
    constexpr float kWheelVelocitySensorBoundMps = 0.06f;
    constexpr float kYawRateSensorBoundRadps = 0.03f;

    class ReplayOptions
    {
    public:
        std::filesystem::path rootPath;
        std::filesystem::path outputPath;
        std::string runIdFilter;
        std::filesystem::path sampleCsvPath;
        std::filesystem::path feedforwardSampleCsvPath;
        std::vector<std::string> sampleMetrics;
        bool useKnownStationarySeed = false;
    };

    class SidecarInfo
    {
    public:
        std::filesystem::path path;
        std::unordered_map<std::string, std::string> metadata;
        std::vector<std::string> fieldNames;
        std::string runId;
        std::string formatVersion;
        std::string streamType;
        std::string controlLogFile;
    };

    class RunCandidate
    {
    public:
        std::filesystem::path csvPath;
        SidecarInfo sidecar;
        std::filesystem::path controlLogPath;
        float batteryVoltageV = std::numeric_limits<float>::quiet_NaN();
        std::string batterySource = "plant_default";
    };

    class DuplicateRunInfo
    {
    public:
        std::string runId;
        std::filesystem::path keptPath;
        std::filesystem::path skippedPath;
    };

    class LoggedRow
    {
    public:
        std::uint32_t masterTimeUs = 0U;
        std::uint32_t controlTickSequence = 0U;
        std::uint32_t dtUs = 0U;
        std::uint8_t sectionId = 0U;
        std::uint8_t primitiveId = 0U;
        std::uint8_t phaseId = 0U;
        std::uint16_t repeatIndex = 0U;
        std::uint16_t saturationFlags = 0U;
        MazeMap::VehicleState loggedState;
        float measuredLinearSpeedMps = 0.0f;
        float measuredYawRateRadps = 0.0f;
        float commandedLinearMps = 0.0f;
        float commandedYawRateRadps = 0.0f;
        float leftDriveCommand = 0.0f;
        float rightDriveCommand = 0.0f;
        float leftPlantCommand = 0.0f;
        float rightPlantCommand = 0.0f;
        std::int32_t leftEncoderCount = 0;
        std::int32_t rightEncoderCount = 0;
        float leftEncoderDistanceM = 0.0f;
        float rightEncoderDistanceM = 0.0f;
        float leftEncoderDistanceDeltaM = 0.0f;
        float rightEncoderDistanceDeltaM = 0.0f;
        float leftEncoderWheelSpeedRadps = 0.0f;
        float rightEncoderWheelSpeedRadps = 0.0f;
        float leftEncoderVelocityMps = 0.0f;
        float rightEncoderVelocityMps = 0.0f;
        bool encoderKinematicsValid = false;
        std::uint8_t imuStatus = 0U;
        std::int16_t imuGyroX = 0;
        std::int16_t imuGyroY = 0;
        std::int16_t imuGyroZ = 0;
        std::int16_t imuAccelX = 0;
        std::int16_t imuAccelY = 0;
        std::int16_t imuAccelZ = 0;
        std::int16_t imuTemp = 0;
        float imuGyroMdpsPerLsb = std::numeric_limits<float>::quiet_NaN();
        float imuAccelMgPerLsb = std::numeric_limits<float>::quiet_NaN();
        bool accelBiasValid = false;
        float gyroRawRadps = std::numeric_limits<float>::quiet_NaN();
        float gyroRawDebugRadps = std::numeric_limits<float>::quiet_NaN();
        float gyroBiasDebugRadps = std::numeric_limits<float>::quiet_NaN();
        float correctedYawRateRadps = std::numeric_limits<float>::quiet_NaN();
        float accelBodyRightMps2 = std::numeric_limits<float>::quiet_NaN();
        float accelBodyForwardMps2 = std::numeric_limits<float>::quiet_NaN();
        float planarAccelMps2 = std::numeric_limits<float>::quiet_NaN();
        std::uint16_t frontLeftWallAmbientAdc = 0U;
        std::uint16_t frontLeftWallLitAdc = 0U;
        std::uint16_t frontRightWallAmbientAdc = 0U;
        std::uint16_t frontRightWallLitAdc = 0U;
        std::uint16_t sideLeftWallAmbientAdc = 0U;
        std::uint16_t sideLeftWallLitAdc = 0U;
        std::uint16_t sideRightWallAmbientAdc = 0U;
        std::uint16_t sideRightWallLitAdc = 0U;
        bool wallAdcPresent = false;
        float fanDutyCycle = 0.0f;
    };

    class SampleExportRow
    {
    public:
        std::uint32_t masterTimeUs = 0U;
        std::uint32_t controlTickSequence = 0U;
        std::uint32_t dtUs = 0U;
        std::uint8_t sectionId = 0U;
        std::uint8_t primitiveId = 0U;
        std::uint8_t phaseId = 0U;
        std::uint16_t repeatIndex = 0U;
        bool accelBiasValid = false;
        std::string sectionName;
        std::string primitiveName;
        std::string phaseName;
        float predictedAccelBodyRightMps2 = std::numeric_limits<float>::quiet_NaN();
        float actualAccelBodyRightMps2 = std::numeric_limits<float>::quiet_NaN();
        float predictedAccelBodyForwardMps2 = std::numeric_limits<float>::quiet_NaN();
        float actualAccelBodyForwardMps2 = std::numeric_limits<float>::quiet_NaN();
        float predictedPlanarAccelMps2 = std::numeric_limits<float>::quiet_NaN();
        float actualPlanarAccelMps2 = std::numeric_limits<float>::quiet_NaN();
        float predictedLinearSpeedMps = std::numeric_limits<float>::quiet_NaN();
        float actualLinearSpeedMps = std::numeric_limits<float>::quiet_NaN();
        float predictedYawRateRadps = std::numeric_limits<float>::quiet_NaN();
        float actualYawRateRadps = std::numeric_limits<float>::quiet_NaN();
        float predictedRawGyroRadps = std::numeric_limits<float>::quiet_NaN();
        float actualRawGyroRadps = std::numeric_limits<float>::quiet_NaN();
    };

    class FeedforwardSampleExportRow
    {
    public:
        std::uint32_t masterTimeUs = 0U;
        std::uint32_t nextMasterTimeUs = 0U;
        std::uint32_t controlTickSequence = 0U;
        std::uint32_t nextControlTickSequence = 0U;
        std::uint32_t dtUs = 0U;
        std::uint8_t sectionId = 0U;
        std::uint8_t primitiveId = 0U;
        std::uint8_t phaseId = 0U;
        std::uint16_t repeatIndex = 0U;
        std::uint16_t saturationFlags = 0U;
        std::string pathId;
        std::string pathLabel;
        std::string pathCategory;
        std::string sectionName;
        std::string primitiveName;
        std::string phaseName;
        float currentForwardSensorMps = std::numeric_limits<float>::quiet_NaN();
        float currentLeftVelocityMps = std::numeric_limits<float>::quiet_NaN();
        float currentRightVelocityMps = std::numeric_limits<float>::quiet_NaN();
        float currentYawRateSensorRadps = std::numeric_limits<float>::quiet_NaN();
        float targetForwardSensorMps = std::numeric_limits<float>::quiet_NaN();
        float targetLeftVelocityMps = std::numeric_limits<float>::quiet_NaN();
        float targetRightVelocityMps = std::numeric_limits<float>::quiet_NaN();
        float targetYawRateSensorRadps = std::numeric_limits<float>::quiet_NaN();
        float nominalLeftCommand = std::numeric_limits<float>::quiet_NaN();
        float nominalRightCommand = std::numeric_limits<float>::quiet_NaN();
        float nominalAverageCommand = std::numeric_limits<float>::quiet_NaN();
        float nominalDeltaCommand = std::numeric_limits<float>::quiet_NaN();
        float loggedLeftDriveCommand = std::numeric_limits<float>::quiet_NaN();
        float loggedRightDriveCommand = std::numeric_limits<float>::quiet_NaN();
        float loggedAverageDriveCommand = std::numeric_limits<float>::quiet_NaN();
        float loggedDeltaDriveCommand = std::numeric_limits<float>::quiet_NaN();
        float loggedLeftPlantCommand = std::numeric_limits<float>::quiet_NaN();
        float loggedRightPlantCommand = std::numeric_limits<float>::quiet_NaN();
        float loggedAveragePlantCommand = std::numeric_limits<float>::quiet_NaN();
        float loggedDeltaPlantCommand = std::numeric_limits<float>::quiet_NaN();
        float leftDriveCommandError = std::numeric_limits<float>::quiet_NaN();
        float rightDriveCommandError = std::numeric_limits<float>::quiet_NaN();
        float averageDriveCommandError = std::numeric_limits<float>::quiet_NaN();
        float deltaDriveCommandError = std::numeric_limits<float>::quiet_NaN();
        float leftPlantCommandError = std::numeric_limits<float>::quiet_NaN();
        float rightPlantCommandError = std::numeric_limits<float>::quiet_NaN();
        float averagePlantCommandError = std::numeric_limits<float>::quiet_NaN();
        float deltaPlantCommandError = std::numeric_limits<float>::quiet_NaN();
        float envelopeLeftMin = std::numeric_limits<float>::quiet_NaN();
        float envelopeLeftMax = std::numeric_limits<float>::quiet_NaN();
        float envelopeRightMin = std::numeric_limits<float>::quiet_NaN();
        float envelopeRightMax = std::numeric_limits<float>::quiet_NaN();
        bool loggedDriveWithinEnvelope = false;
        bool loggedPlantCommandWithinEnvelope = false;
        float predictedNextForwardMps = std::numeric_limits<float>::quiet_NaN();
        float predictedNextYawRateRadps = std::numeric_limits<float>::quiet_NaN();
        float predictedForwardTargetErrorMps = std::numeric_limits<float>::quiet_NaN();
        float predictedYawTargetErrorRadps = std::numeric_limits<float>::quiet_NaN();
    };

    class ErrorStats
    {
    public:
        std::uint64_t count = 0U;
        double sum = 0.0;
        double sumSquares = 0.0;
        double sumAbs = 0.0;
        double maxAbs = 0.0;

        void add(double error) noexcept
        {
            if (!std::isfinite(error))
            {
                return;
            }

            ++count;
            sum += error;
            sumSquares += error * error;
            const double absError = std::fabs(error);
            sumAbs += absError;
            maxAbs = (std::max)(maxAbs, absError);
        }

        void merge(const ErrorStats& other) noexcept
        {
            count += other.count;
            sum += other.sum;
            sumSquares += other.sumSquares;
            sumAbs += other.sumAbs;
            maxAbs = (std::max)(maxAbs, other.maxAbs);
        }

        double rmse() const noexcept
        {
            return (count > 0U) ? std::sqrt(sumSquares / static_cast<double>(count)) : 0.0;
        }

        double mae() const noexcept
        {
            return (count > 0U) ? (sumAbs / static_cast<double>(count)) : 0.0;
        }

        double bias() const noexcept
        {
            return (count > 0U) ? (sum / static_cast<double>(count)) : 0.0;
        }
    };

    class HitRateStats
    {
    public:
        std::uint64_t total = 0U;
        std::uint64_t hits = 0U;

        void add(bool hit) noexcept
        {
            ++total;
            if (hit)
            {
                ++hits;
            }
        }

        void merge(const HitRateStats& other) noexcept
        {
            total += other.total;
            hits += other.hits;
        }

        double rate() const noexcept
        {
            return (total > 0U) ? (static_cast<double>(hits) / static_cast<double>(total)) : 0.0;
        }
    };

    class PredictionMetrics
    {
    public:
        ErrorStats leftEncoderWheelSpeedRadps;
        ErrorStats rightEncoderWheelSpeedRadps;
        ErrorStats encoderLinearSpeedMps;
        ErrorStats encoderYawRateRadps;
        ErrorStats bodyForwardSpeedMps;
        ErrorStats bodyYawRateRadps;
        ErrorStats rawGyroRadps;
        ErrorStats accelBodyRightMps2;
        ErrorStats accelBodyForwardMps2;
        ErrorStats planarAccelMps2;

        void merge(const PredictionMetrics& other) noexcept
        {
            leftEncoderWheelSpeedRadps.merge(other.leftEncoderWheelSpeedRadps);
            rightEncoderWheelSpeedRadps.merge(other.rightEncoderWheelSpeedRadps);
            encoderLinearSpeedMps.merge(other.encoderLinearSpeedMps);
            encoderYawRateRadps.merge(other.encoderYawRateRadps);
            bodyForwardSpeedMps.merge(other.bodyForwardSpeedMps);
            bodyYawRateRadps.merge(other.bodyYawRateRadps);
            rawGyroRadps.merge(other.rawGyroRadps);
            accelBodyRightMps2.merge(other.accelBodyRightMps2);
            accelBodyForwardMps2.merge(other.accelBodyForwardMps2);
            planarAccelMps2.merge(other.planarAccelMps2);
        }
    };

    class FeedforwardMetrics
    {
    public:
        ErrorStats leftDriveCommand;
        ErrorStats rightDriveCommand;
        ErrorStats averageDriveCommand;
        ErrorStats deltaDriveCommand;
        ErrorStats leftPlantCommand;
        ErrorStats rightPlantCommand;
        ErrorStats averagePlantCommand;
        ErrorStats deltaPlantCommand;
        ErrorStats predictedForwardTargetErrorMps;
        ErrorStats predictedYawTargetErrorRadps;
        HitRateStats driveEnvelopeHit;
        HitRateStats plantCommandEnvelopeHit;

        void merge(const FeedforwardMetrics& other) noexcept
        {
            leftDriveCommand.merge(other.leftDriveCommand);
            rightDriveCommand.merge(other.rightDriveCommand);
            averageDriveCommand.merge(other.averageDriveCommand);
            deltaDriveCommand.merge(other.deltaDriveCommand);
            leftPlantCommand.merge(other.leftPlantCommand);
            rightPlantCommand.merge(other.rightPlantCommand);
            averagePlantCommand.merge(other.averagePlantCommand);
            deltaPlantCommand.merge(other.deltaPlantCommand);
            predictedForwardTargetErrorMps.merge(other.predictedForwardTargetErrorMps);
            predictedYawTargetErrorRadps.merge(other.predictedYawTargetErrorRadps);
            driveEnvelopeHit.merge(other.driveEnvelopeHit);
            plantCommandEnvelopeHit.merge(other.plantCommandEnvelopeHit);
        }
    };

    class FeedforwardEnvelope
    {
    public:
        float leftMin = (std::numeric_limits<float>::infinity)();
        float leftMax = -(std::numeric_limits<float>::infinity)();
        float rightMin = (std::numeric_limits<float>::infinity)();
        float rightMax = -(std::numeric_limits<float>::infinity)();
        bool valid = false;
    };

    class FeedforwardPathSummary
    {
    public:
        std::string pathId;
        std::string label;
        std::string category;
        std::uint64_t comparableTransitions = 0U;
        std::uint64_t validSolutions = 0U;
        FeedforwardMetrics metrics{};

        void merge(const FeedforwardPathSummary& other) noexcept
        {
            comparableTransitions += other.comparableTransitions;
            validSolutions += other.validSolutions;
            metrics.merge(other.metrics);
        }
    };

    class ConsistencyMetrics
    {
    public:
        ErrorStats positionMm;
        ErrorStats headingDeg;
        ErrorStats forwardSpeedMps;
        ErrorStats rightwardSpeedMps;
        ErrorStats yawRateRadps;
        ErrorStats leftWheelSpeedRadps;
        ErrorStats rightWheelSpeedRadps;

        void merge(const ConsistencyMetrics& other) noexcept
        {
            positionMm.merge(other.positionMm);
            headingDeg.merge(other.headingDeg);
            forwardSpeedMps.merge(other.forwardSpeedMps);
            rightwardSpeedMps.merge(other.rightwardSpeedMps);
            yawRateRadps.merge(other.yawRateRadps);
            leftWheelSpeedRadps.merge(other.leftWheelSpeedRadps);
            rightWheelSpeedRadps.merge(other.rightWheelSpeedRadps);
        }
    };

    class SectionPhaseKey
    {
    public:
        std::uint8_t sectionId = 0U;
        std::uint8_t phaseId = 0U;

        bool operator==(const SectionPhaseKey& other) const noexcept
        {
            return (sectionId == other.sectionId) && (phaseId == other.phaseId);
        }
    };

    class SectionPhaseKeyHash
    {
    public:
        std::size_t operator()(const SectionPhaseKey& key) const noexcept
        {
            return (static_cast<std::size_t>(key.sectionId) << 8U) ^
                static_cast<std::size_t>(key.phaseId);
        }
    };

    class SectionPhaseReport
    {
    public:
        std::uint8_t sectionId = 0U;
        std::uint8_t phaseId = 0U;
        std::string sectionName;
        std::string phaseName;
        std::uint64_t sampleCount = 0U;
        PredictionMetrics prediction{};
        ConsistencyMetrics consistency{};

        void merge(const SectionPhaseReport& other) noexcept
        {
            sampleCount += other.sampleCount;
            prediction.merge(other.prediction);
            consistency.merge(other.consistency);
        }
    };

    class PhaseAssociationSummary
    {
    public:
        std::uint64_t totalSamples = 0U;
        std::size_t bucketCount = 0U;
        double etaSquaredAbs = 0.0;
        std::string worstBucketLabel;
        std::uint64_t worstBucketSamples = 0U;
        double worstBucketMae = 0.0;
        double globalMae = 0.0;
        double worstBucketMaeRatio = 0.0;
    };

    class RunReport
    {
    public:
        std::string runId;
        std::string formatVersion;
        std::filesystem::path csvPath;
        std::string batterySource;
        float batteryVoltageV = std::numeric_limits<float>::quiet_NaN();
        std::uint64_t totalRows = 0U;
        std::uint64_t keptRows = 0U;
        std::uint64_t scoredTransitions = 0U;
        std::uint64_t predictFailures = 0U;
        std::uint64_t yawRejects = 0U;
        std::uint64_t feedforwardTransitions = 0U;
        int ignoredSectionId = -1;
        bool completed = true;
        std::string failureReason;
        PredictionMetrics prediction{};
        ConsistencyMetrics consistency{};
        std::vector<FeedforwardPathSummary> feedforwardPaths;
        std::vector<SectionPhaseReport> sectionPhaseBuckets;
        std::vector<SampleExportRow> sampleExportRows;
        std::vector<FeedforwardSampleExportRow> feedforwardSampleExportRows;
    };

    class CorpusReport
    {
    public:
        std::vector<RunReport> runs;
        std::vector<DuplicateRunInfo> duplicates;
        std::uint64_t candidateCsvCount = 0U;
        PredictionMetrics prediction{};
        ConsistencyMetrics consistency{};
        std::vector<FeedforwardPathSummary> feedforwardPaths;
        std::vector<SectionPhaseReport> sectionPhaseBuckets;
    };

    constexpr const char* kAccelerationFeedforwardPathId = "acceleration";
    constexpr const char* kAccelerationFeedforwardPathLabel = "PlantModel acceleration feedforward";
    constexpr const char* kAccelerationFeedforwardPathCategory = "plant_model";

    static std::vector<FeedforwardPathSummary> BuildDefaultFeedforwardPathSummaries()
    {
        std::vector<FeedforwardPathSummary> summaries;
        summaries.reserve(1U);
        FeedforwardPathSummary summary{};
        summary.pathId = kAccelerationFeedforwardPathId;
        summary.label = kAccelerationFeedforwardPathLabel;
        summary.category = kAccelerationFeedforwardPathCategory;
        summaries.push_back(std::move(summary));
        return summaries;
    }

    static std::string Trim(const std::string& value)
    {
        const std::size_t first = value.find_first_not_of(" \t\r\n");
        if (first == std::string::npos)
        {
            return std::string();
        }

        const std::size_t last = value.find_last_not_of(" \t\r\n");
        return value.substr(first, last - first + 1U);
    }

    static std::string ToLower(std::string value)
    {
        for (char& c : value)
        {
            c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        }
        return value;
    }

    static std::vector<std::string> SplitCommaSeparated(const std::string& line)
    {
        std::vector<std::string> result;
        std::stringstream stream(line);
        std::string token;
        while (std::getline(stream, token, ','))
        {
            result.push_back(token);
        }
        return result;
    }

    static bool ParseUnsigned32(const std::string& text, std::uint32_t& value) noexcept
    {
        char* end = nullptr;
        const unsigned long parsed = std::strtoul(text.c_str(), &end, 10);
        if (end == text.c_str() || *end != '\0')
        {
            return false;
        }

        value = static_cast<std::uint32_t>(parsed);
        return true;
    }

    static bool ParseUnsigned16(const std::string& text, std::uint16_t& value) noexcept
    {
        std::uint32_t parsed = 0U;
        if (!ParseUnsigned32(text, parsed) || (parsed > 0xFFFFU))
        {
            return false;
        }

        value = static_cast<std::uint16_t>(parsed);
        return true;
    }

    static bool ParseUnsigned8(const std::string& text, std::uint8_t& value) noexcept
    {
        std::uint32_t parsed = 0U;
        if (!ParseUnsigned32(text, parsed) || (parsed > 0xFFU))
        {
            return false;
        }

        value = static_cast<std::uint8_t>(parsed);
        return true;
    }

    static bool ParseInt32(const std::string& text, std::int32_t& value) noexcept
    {
        char* end = nullptr;
        const long parsed = std::strtol(text.c_str(), &end, 10);
        if (end == text.c_str() || *end != '\0')
        {
            return false;
        }

        value = static_cast<std::int32_t>(parsed);
        return true;
    }

    static bool ParseInt16(const std::string& text, std::int16_t& value) noexcept
    {
        std::int32_t parsed = 0;
        if (!ParseInt32(text, parsed) ||
            (parsed < static_cast<std::int32_t>((std::numeric_limits<std::int16_t>::min)())) ||
            (parsed > static_cast<std::int32_t>((std::numeric_limits<std::int16_t>::max)())))
        {
            return false;
        }

        value = static_cast<std::int16_t>(parsed);
        return true;
    }

    static bool ParseFloat(const std::string& text, float& value) noexcept
    {
        char* end = nullptr;
        value = std::strtof(text.c_str(), &end);
        return !(end == text.c_str() || *end != '\0');
    }

    static MazeMap::VehicleState BuildKnownStationaryOpenFloorInitialState() noexcept
    {
        MazeMap::VehicleState state{};
        return state;
    }

    static bool GetToken(
        const std::vector<std::string>& tokens,
        const std::unordered_map<std::string, std::size_t>& indices,
        const char* fieldName,
        std::string& token)
    {
        const auto it = indices.find(fieldName);
        if (it == indices.end() || it->second >= tokens.size())
        {
            return false;
        }

        token = tokens[it->second];
        return true;
    }

    template <typename T, typename Parser>
    static bool ParseFieldIfPresent(
        const std::vector<std::string>& tokens,
        const std::unordered_map<std::string, std::size_t>& indices,
        const char* fieldName,
        T& value,
        Parser&& parser,
        bool& present,
        std::string& error)
    {
        std::string token;
        if (!GetToken(tokens, indices, fieldName, token))
        {
            return true;
        }

        present = true;
        if (!parser(token, value))
        {
            error = std::string("Failed to parse CSV field: ") + fieldName + " value=" + token;
            return false;
        }

        return true;
    }

    template <typename T, typename Parser>
    static bool ParseFieldIfPresent(
        const std::vector<std::string>& tokens,
        const std::unordered_map<std::string, std::size_t>& indices,
        const char* fieldName,
        T& value,
        Parser&& parser,
        std::string& error)
    {
        bool present = false;
        return ParseFieldIfPresent(tokens, indices, fieldName, value, std::forward<Parser>(parser), present, error);
    }

    static float RawImuZToBodyYawRateRadps(
        const std::int16_t rawGyroZ,
        const float gyroSensitivityMdpsPerLsb) noexcept
    {
        if (!std::isfinite(gyroSensitivityMdpsPerLsb) || (gyroSensitivityMdpsPerLsb <= 0.0f))
        {
            return std::numeric_limits<float>::quiet_NaN();
        }

        constexpr float kClockwiseYawFromSensorZSign = -1.0f;
        return
            kClockwiseYawFromSensorZSign *
            static_cast<float>(rawGyroZ) *
            gyroSensitivityMdpsPerLsb *
            0.001f *
            kDegreesToRadians;
    }

    static bool DeriveEncoderKinematicsFromCounts(std::vector<LoggedRow>& rows, std::string& error)
    {
        for (std::size_t index = 0; index < rows.size(); ++index)
        {
            LoggedRow& row = rows[index];
            row.leftEncoderDistanceM =
                MazeMap::Vehicle::DriveEncoderDistanceFromCounts(static_cast<std::int64_t>(row.leftEncoderCount));
            row.rightEncoderDistanceM =
                MazeMap::Vehicle::DriveEncoderDistanceFromCounts(static_cast<std::int64_t>(row.rightEncoderCount));

            if (!std::isfinite(row.leftEncoderDistanceM) || !std::isfinite(row.rightEncoderDistanceM))
            {
                error = "Failed to derive encoder total distance from raw counts";
                return false;
            }

            row.encoderKinematicsValid = false;
            row.leftEncoderDistanceDeltaM = 0.0f;
            row.rightEncoderDistanceDeltaM = 0.0f;
            row.leftEncoderVelocityMps = 0.0f;
            row.rightEncoderVelocityMps = 0.0f;
            row.leftEncoderWheelSpeedRadps = 0.0f;
            row.rightEncoderWheelSpeedRadps = 0.0f;

            if (index > 0U)
            {
                const LoggedRow& previous = rows[index - 1U];
                const float dtSeconds = static_cast<float>(row.dtUs) * 1.0e-6f;
                if (std::isfinite(dtSeconds) && (dtSeconds > 0.0f))
                {
                    const std::int64_t leftDeltaCounts =
                        static_cast<std::int64_t>(row.leftEncoderCount) -
                        static_cast<std::int64_t>(previous.leftEncoderCount);
                    const std::int64_t rightDeltaCounts =
                        static_cast<std::int64_t>(row.rightEncoderCount) -
                        static_cast<std::int64_t>(previous.rightEncoderCount);
                    row.leftEncoderDistanceDeltaM =
                        MazeMap::Vehicle::DriveEncoderDistanceFromCounts(leftDeltaCounts);
                    row.rightEncoderDistanceDeltaM =
                        MazeMap::Vehicle::DriveEncoderDistanceFromCounts(rightDeltaCounts);

                    row.leftEncoderVelocityMps = row.leftEncoderDistanceDeltaM / dtSeconds;
                    row.rightEncoderVelocityMps = row.rightEncoderDistanceDeltaM / dtSeconds;
                    row.leftEncoderWheelSpeedRadps =
                        MazeMap::Vehicle::WheelSpeedFromLinearVelocity(row.leftEncoderVelocityMps);
                    row.rightEncoderWheelSpeedRadps =
                        MazeMap::Vehicle::WheelSpeedFromLinearVelocity(row.rightEncoderVelocityMps);
                    row.encoderKinematicsValid =
                        std::isfinite(row.leftEncoderDistanceDeltaM) &&
                        std::isfinite(row.rightEncoderDistanceDeltaM) &&
                        std::isfinite(row.leftEncoderVelocityMps) &&
                        std::isfinite(row.rightEncoderVelocityMps) &&
                        std::isfinite(row.leftEncoderWheelSpeedRadps) &&
                        std::isfinite(row.rightEncoderWheelSpeedRadps);
                }
            }

            row.loggedState.PublishEncoderWheelSpeedsRadps(
                row.leftEncoderWheelSpeedRadps,
                row.rightEncoderWheelSpeedRadps);
        }

        return true;
    }

    template <typename T, typename Parser>
    static bool ParseField(
        const std::vector<std::string>& tokens,
        const std::unordered_map<std::string, std::size_t>& indices,
        const char* fieldName,
        T& value,
        Parser&& parser,
        std::string& error)
    {
        std::string token;
        if (!GetToken(tokens, indices, fieldName, token))
        {
            error = std::string("Missing CSV field: ") + fieldName;
            return false;
        }

        if (!parser(token, value))
        {
            error = std::string("Failed to parse CSV field: ") + fieldName + " value=" + token;
            return false;
        }

        return true;
    }

    static bool ParseSidecar(const std::filesystem::path& path, SidecarInfo& info, std::string& error)
    {
        std::ifstream stream(path);
        if (!stream)
        {
            error = "Failed to open sidecar: " + path.string();
            return false;
        }

        info = {};
        info.path = path;

        std::string line;
        bool headerSeen = false;
        while (std::getline(stream, line))
        {
            line = Trim(line);
            if (line.empty())
            {
                continue;
            }

            if (!headerSeen && (line.find('=') != std::string::npos))
            {
                const std::size_t separator = line.find('=');
                info.metadata.emplace(line.substr(0U, separator), line.substr(separator + 1U));
                continue;
            }

            if (!headerSeen)
            {
                headerSeen = true;
                const std::vector<std::string> entries = SplitCommaSeparated(line);
                for (const std::string& entry : entries)
                {
                    const std::size_t separator = entry.find('_');
                    info.fieldNames.push_back((separator == std::string::npos) ? entry : entry.substr(separator + 1U));
                }
                break;
            }
        }

        if (!headerSeen)
        {
            error = "Sidecar header missing: " + path.string();
            return false;
        }

        const auto runIt = info.metadata.find("run_id");
        const auto formatIt = info.metadata.find("format_version");
        const auto streamIt = info.metadata.find("stream_type");
        const auto controlLogIt = info.metadata.find("control_log_file");
        info.runId = (runIt != info.metadata.end()) ? runIt->second : std::string();
        info.formatVersion = (formatIt != info.metadata.end()) ? formatIt->second : std::string("unknown");
        info.streamType = (streamIt != info.metadata.end()) ? streamIt->second : std::string();
        info.controlLogFile = (controlLogIt != info.metadata.end()) ? controlLogIt->second : std::string();

        if (info.streamType == "main")
        {
            info.streamType = "open_floor_main";
        }

        if (info.streamType != "open_floor_main")
        {
            error = "Unexpected stream_type in sidecar: " + path.string();
            return false;
        }

        if (info.runId.empty())
        {
            info.runId = path.parent_path().filename().string();
        }

        if (info.runId.empty())
        {
            error = "run_id missing in sidecar: " + path.string();
            return false;
        }

        return true;
    }

    static std::filesystem::path ResolveControlLogPath(const RunCandidate& candidate)
    {
        if (!candidate.sidecar.controlLogFile.empty())
        {
            const std::filesystem::path candidatePath = candidate.sidecar.path.parent_path() / candidate.sidecar.controlLogFile;
            if (std::filesystem::exists(candidatePath))
            {
                return candidatePath;
            }
        }

        const std::filesystem::path loggingPath = candidate.csvPath.parent_path() / "logging.txt";
        if (std::filesystem::exists(loggingPath))
        {
            return loggingPath;
        }

        return std::filesystem::path();
    }

    static bool ReadBatteryVoltageStart(const std::filesystem::path& loggingPath, float& batteryVoltageV)
    {
        if (loggingPath.empty())
        {
            return false;
        }

        std::ifstream stream(loggingPath);
        if (!stream)
        {
            return false;
        }

        std::string line;
        while (std::getline(stream, line))
        {
            const std::size_t batteryPos = line.find("battery_voltage_start=");
            if (batteryPos == std::string::npos)
            {
                continue;
            }

            const std::size_t valueStart = batteryPos + std::string("battery_voltage_start=").size();
            std::size_t valueEnd = line.find(';', valueStart);
            if (valueEnd == std::string::npos)
            {
                valueEnd = line.size();
            }

            float parsed = 0.0f;
            if (ParseFloat(line.substr(valueStart, valueEnd - valueStart), parsed))
            {
                batteryVoltageV = parsed;
                return true;
            }
        }

        return false;
    }

    static bool PathLooksPrimaryDecode(const std::filesystem::path& path)
    {
        const std::string lower = ToLower(path.string());
        return lower.find("mmlog_decode_") != std::string::npos;
    }

    static bool PreferCandidate(const RunCandidate& lhs, const RunCandidate& rhs)
    {
        const bool lhsPrimary = PathLooksPrimaryDecode(lhs.csvPath);
        const bool rhsPrimary = PathLooksPrimaryDecode(rhs.csvPath);
        if (lhsPrimary != rhsPrimary)
        {
            return lhsPrimary;
        }

        std::error_code lhsEc;
        std::error_code rhsEc;
        const std::filesystem::file_time_type lhsTime = std::filesystem::last_write_time(lhs.csvPath, lhsEc);
        const std::filesystem::file_time_type rhsTime = std::filesystem::last_write_time(rhs.csvPath, rhsEc);
        if (!lhsEc && !rhsEc && lhsTime != rhsTime)
        {
            return lhsTime > rhsTime;
        }

        return lhs.csvPath.string() < rhs.csvPath.string();
    }

    static bool RunCandidateRunIdLess(const RunCandidate& lhs, const RunCandidate& rhs) noexcept
    {
        return lhs.sidecar.runId < rhs.sidecar.runId;
    }

    static bool ResolveSampleMetricSelection(
        const std::string& text,
        std::vector<std::string>& metrics,
        std::string& error);

    static bool ParseArgs(int argc, char* argv[], ReplayOptions& options, std::string& error)
    {
        options = {};
        std::string metricSelectionText;
        for (int index = 1; index < argc; ++index)
        {
            const std::string argument = argv[index];
            if (argument == "--root")
            {
                if ((index + 1) >= argc)
                {
                    error = "--root requires a path";
                    return false;
                }

                options.rootPath = std::filesystem::path(argv[++index]);
            }
            else if (argument == "--output")
            {
                if ((index + 1) >= argc)
                {
                    error = "--output requires a path";
                    return false;
                }

                options.outputPath = std::filesystem::path(argv[++index]);
            }
            else if (argument == "--run-id")
            {
                if ((index + 1) >= argc)
                {
                    error = "--run-id requires a value";
                    return false;
                }

                options.runIdFilter = argv[++index];
            }
            else if (argument == "--sample-csv")
            {
                if ((index + 1) >= argc)
                {
                    error = "--sample-csv requires a path";
                    return false;
                }

                options.sampleCsvPath = std::filesystem::path(argv[++index]);
            }
            else if (argument == "--feedforward-sample-csv")
            {
                if ((index + 1) >= argc)
                {
                    error = "--feedforward-sample-csv requires a path";
                    return false;
                }

                options.feedforwardSampleCsvPath = std::filesystem::path(argv[++index]);
            }
            else if (argument == "--metrics")
            {
                if ((index + 1) >= argc)
                {
                    error = "--metrics requires a comma-separated value list";
                    return false;
                }

                metricSelectionText = argv[++index];
            }
            else if (argument == "--tuning")
            {
                error = "--tuning is no longer supported by the canonical Estimator configuration";
                return false;
            }
            else if (argument == "--known-stationary-seed")
            {
                options.useKnownStationarySeed = true;
            }
            else if (argument == "--help" || argument == "-h")
            {
                std::cout
                    << "OpenFloorUkfReplay\n"
                    << "  --root <path>    Root directory to scan for open_floor_main.csv captures.\n"
                    << "  --output <path>  Output directory for the generated report.\n"
                    << "  --known-stationary-seed  Seed from the canonical open-floor start pose instead of the logged UKF state.\n"
                    << "  --run-id <id>    Optional single-run filter.\n"
                    << "  --sample-csv <path>  Optional per-sample CSV export for the selected run.\n"
                    << "  --feedforward-sample-csv <path>  Optional per-sample feedforward-path audit CSV export for the selected run.\n"
                    << "  --metrics <list> Optional sample metric list or aliases: default, accel_compare, speed_compare.\n";
                std::exit(0);
            }
            else
            {
                error = "Unknown argument: " + argument;
                return false;
            }
        }

        if (options.rootPath.empty())
        {
            const std::filesystem::path cwd = std::filesystem::current_path();
            options.rootPath = std::filesystem::exists(cwd / "TestResults") ? (cwd / "TestResults") : cwd;
        }

        if (!options.sampleCsvPath.empty() && options.runIdFilter.empty())
        {
            error = "--sample-csv requires --run-id so the export targets one run";
            return false;
        }

        if (!options.feedforwardSampleCsvPath.empty() && options.runIdFilter.empty())
        {
            error = "--feedforward-sample-csv requires --run-id so the export targets one run";
            return false;
        }

        if (!metricSelectionText.empty() && options.sampleCsvPath.empty())
        {
            error = "--metrics requires --sample-csv";
            return false;
        }

        if (!options.sampleCsvPath.empty() &&
            !ResolveSampleMetricSelection(metricSelectionText, options.sampleMetrics, error))
        {
            return false;
        }

        return true;
    }

    static std::string MakeTimestampString()
    {
        const std::time_t now = std::time(nullptr);
        std::tm localTime{};
#if defined(_WIN32)
        localtime_s(&localTime, &now);
#else
        localTime = *std::localtime(&now);
#endif
        std::ostringstream stream;
        stream << std::put_time(&localTime, "%Y-%m-%d_%H-%M-%S");
        return stream.str();
    }

    static std::string SectionName(std::uint8_t sectionId)
    {
        switch (sectionId)
        {
        case 0U:
            return "SEC_00_TIMING";
        case 1U:
            return "SEC_10_STATIC";
        case 2U:
            return "SEC_20_LAUNCH";
        case 3U:
            return "SEC_30_STRAIGHT";
        case 4U:
            return "SEC_40_YAW";
        case 5U:
            return "SEC_50_SMOOTH";
        case 6U:
            return "SEC_60_LOOP_CW";
        case 7U:
            return "SEC_70_LOOP_CCW";
        default:
            return "SEC_UNKNOWN_" + std::to_string(static_cast<unsigned>(sectionId));
        }
    }

    static std::string PhaseName(std::uint8_t phaseId)
    {
        switch (phaseId)
        {
        case 0U:
            return "timing";
        case 1U:
            return "static";
        case 2U:
            return "launch";
        case 3U:
            return "straight";
        case 4U:
            return "yaw";
        case 5U:
            return "smooth";
        case 6U:
            return "loop_cw";
        case 7U:
            return "loop_ccw";
        default:
            return "phase_unknown_" + std::to_string(static_cast<unsigned>(phaseId));
        }
    }

    static std::string PrimitiveName(std::uint8_t primitiveId)
    {
        if (primitiveId == static_cast<std::uint8_t>(MazeMap::MC_NONE))
        {
            return "NONE";
        }

        char codeName[24] = {};
        FormatManeuverCodeName(static_cast<MazeMap::ManeuverCode>(primitiveId), codeName, sizeof(codeName));
        return codeName;
    }

    static const std::vector<std::string>& DefaultSampleMetricSelection()
    {
        static const std::vector<std::string> kDefaultMetrics = {
            "master_time_us",
            "control_tick_sequence",
            "dt_us",
            "section_id",
            "section_name",
            "primitive_id",
            "primitive_name",
            "phase_id",
            "phase_name",
            "repeat_index",
            "accel_bias_valid",
            "predicted_accel_body_right_mps2",
            "actual_accel_body_right_mps2",
            "predicted_accel_body_forward_mps2",
            "actual_accel_body_forward_mps2",
            "predicted_planar_accel_mps2",
            "actual_planar_accel_mps2",
        };
        return kDefaultMetrics;
    }

    static bool IsRecognizedSampleMetric(const std::string& metric) noexcept
    {
        static const std::vector<std::string> kRecognizedMetrics = {
            "master_time_us",
            "control_tick_sequence",
            "dt_us",
            "section_id",
            "section_name",
            "primitive_id",
            "primitive_name",
            "phase_id",
            "phase_name",
            "repeat_index",
            "accel_bias_valid",
            "predicted_accel_body_right_mps2",
            "actual_accel_body_right_mps2",
            "predicted_accel_body_forward_mps2",
            "actual_accel_body_forward_mps2",
            "predicted_planar_accel_mps2",
            "actual_planar_accel_mps2",
            "predicted_linear_speed_mps",
            "actual_linear_speed_mps",
            "predicted_yaw_rate_radps",
            "actual_yaw_rate_radps",
            "predicted_raw_gyro_radps",
            "actual_raw_gyro_radps",
        };
        return std::find(kRecognizedMetrics.begin(), kRecognizedMetrics.end(), metric) != kRecognizedMetrics.end();
    }

    static bool AppendSampleMetricSelection(
        const std::string& token,
        std::vector<std::string>& metrics,
        std::string& error)
    {
        const std::string normalized = ToLower(Trim(token));
        if (normalized.empty())
        {
            return true;
        }

        if (normalized == "default")
        {
            for (const std::string& metric : DefaultSampleMetricSelection())
            {
                if (metric.rfind("predicted_", 0U) != 0U && metric.rfind("actual_", 0U) != 0U)
                {
                    if (std::find(metrics.begin(), metrics.end(), metric) == metrics.end())
                    {
                        metrics.push_back(metric);
                    }
                }
            }
            return true;
        }

        if (normalized == "accel_compare")
        {
            static const std::vector<std::string> kAccelCompareMetrics = {
                "predicted_accel_body_right_mps2",
                "actual_accel_body_right_mps2",
                "predicted_accel_body_forward_mps2",
                "actual_accel_body_forward_mps2",
                "predicted_planar_accel_mps2",
                "actual_planar_accel_mps2",
            };
            for (const std::string& metric : kAccelCompareMetrics)
            {
                if (std::find(metrics.begin(), metrics.end(), metric) == metrics.end())
                {
                    metrics.push_back(metric);
                }
            }
            return true;
        }

        if (normalized == "speed_compare")
        {
            static const std::vector<std::string> kSpeedCompareMetrics = {
                "predicted_linear_speed_mps",
                "actual_linear_speed_mps",
                "predicted_yaw_rate_radps",
                "actual_yaw_rate_radps",
                "predicted_raw_gyro_radps",
                "actual_raw_gyro_radps",
            };
            for (const std::string& metric : kSpeedCompareMetrics)
            {
                if (std::find(metrics.begin(), metrics.end(), metric) == metrics.end())
                {
                    metrics.push_back(metric);
                }
            }
            return true;
        }

        if (!IsRecognizedSampleMetric(normalized))
        {
            error = "Unknown sample metric: " + normalized;
            return false;
        }

        if (std::find(metrics.begin(), metrics.end(), normalized) == metrics.end())
        {
            metrics.push_back(normalized);
        }
        return true;
    }

    static bool ResolveSampleMetricSelection(
        const std::string& text,
        std::vector<std::string>& metrics,
        std::string& error)
    {
        metrics.clear();
        if (Trim(text).empty())
        {
            metrics = DefaultSampleMetricSelection();
            return true;
        }

        const std::vector<std::string> tokens = SplitCommaSeparated(text);
        for (const std::string& token : tokens)
        {
            if (!AppendSampleMetricSelection(token, metrics, error))
            {
                return false;
            }
        }

        if (metrics.empty())
        {
            metrics = DefaultSampleMetricSelection();
        }
        return true;
    }

    static std::string SectionPhaseLabel(const SectionPhaseReport& bucket)
    {
        return bucket.sectionName + " / " + bucket.phaseName;
    }

    static SectionPhaseReport& GetOrCreateSectionPhaseReport(
        std::unordered_map<SectionPhaseKey, SectionPhaseReport, SectionPhaseKeyHash>& reports,
        std::uint8_t sectionId,
        std::uint8_t phaseId)
    {
        const SectionPhaseKey key{ sectionId, phaseId };
        auto [it, inserted] = reports.try_emplace(key);
        SectionPhaseReport& report = it->second;
        if (inserted)
        {
            report.sectionId = sectionId;
            report.phaseId = phaseId;
            report.sectionName = SectionName(sectionId);
            report.phaseName = PhaseName(phaseId);
        }

        return report;
    }

    static bool SectionPhaseReportLess(const SectionPhaseReport& lhs, const SectionPhaseReport& rhs) noexcept
    {
        if (lhs.sectionId != rhs.sectionId)
        {
            return lhs.sectionId < rhs.sectionId;
        }

        return lhs.phaseId < rhs.phaseId;
    }

    static std::vector<SectionPhaseReport> ToSortedSectionPhaseReports(
        const std::unordered_map<SectionPhaseKey, SectionPhaseReport, SectionPhaseKeyHash>& reports)
    {
        std::vector<SectionPhaseReport> sorted;
        sorted.reserve(reports.size());
        for (const auto& item : reports)
        {
            sorted.push_back(item.second);
        }

        std::sort(sorted.begin(), sorted.end(), SectionPhaseReportLess);
        return sorted;
    }

    static const ErrorStats& SectionPhaseMetricStats(
        const SectionPhaseReport& bucket,
        const char* metricName) noexcept
    {
        if (std::string(metricName) == "encoder_linear")
        {
            return bucket.prediction.encoderLinearSpeedMps;
        }
        if (std::string(metricName) == "raw_gyro")
        {
            return bucket.prediction.rawGyroRadps;
        }
        if (std::string(metricName) == "accel_right")
        {
            return bucket.prediction.accelBodyRightMps2;
        }
        if (std::string(metricName) == "accel_forward")
        {
            return bucket.prediction.accelBodyForwardMps2;
        }
        if (std::string(metricName) == "post_position")
        {
            return bucket.consistency.positionMm;
        }

        return bucket.consistency.headingDeg;
    }

    static PhaseAssociationSummary ComputePhaseAssociationSummary(
        const std::vector<SectionPhaseReport>& buckets,
        const char* metricName)
    {
        PhaseAssociationSummary summary{};
        double totalSumAbs = 0.0;
        double totalSumSquares = 0.0;
        for (const SectionPhaseReport& bucket : buckets)
        {
            const ErrorStats& stats = SectionPhaseMetricStats(bucket, metricName);
            if (stats.count == 0U)
            {
                continue;
            }

            summary.totalSamples += stats.count;
            ++summary.bucketCount;
            totalSumAbs += stats.sumAbs;
            totalSumSquares += stats.sumSquares;
        }

        if (summary.totalSamples == 0U)
        {
            return summary;
        }

        summary.globalMae = totalSumAbs / static_cast<double>(summary.totalSamples);
        const double totalSs =
            totalSumSquares -
            static_cast<double>(summary.totalSamples) * summary.globalMae * summary.globalMae;

        double betweenSs = 0.0;
        for (const SectionPhaseReport& bucket : buckets)
        {
            const ErrorStats& stats = SectionPhaseMetricStats(bucket, metricName);
            if (stats.count == 0U)
            {
                continue;
            }

            const double bucketMae = stats.mae();
            betweenSs +=
                static_cast<double>(stats.count) *
                (bucketMae - summary.globalMae) *
                (bucketMae - summary.globalMae);
            if (summary.worstBucketLabel.empty() || (bucketMae > summary.worstBucketMae))
            {
                summary.worstBucketLabel = SectionPhaseLabel(bucket);
                summary.worstBucketSamples = stats.count;
                summary.worstBucketMae = bucketMae;
            }
        }

        summary.etaSquaredAbs = (totalSs > 0.0) ? (betweenSs / totalSs) : 0.0;
        summary.etaSquaredAbs = (std::clamp)(summary.etaSquaredAbs, 0.0, 1.0);
        summary.worstBucketMaeRatio =
            (summary.globalMae > 0.0) ? (summary.worstBucketMae / summary.globalMae) : 0.0;
        return summary;
    }

    static bool DiscoverRuns(
        const ReplayOptions& options,
        std::vector<RunCandidate>& runs,
        std::vector<DuplicateRunInfo>& duplicates,
        std::uint64_t& candidateCsvCount,
        std::string& error)
    {
        if (!std::filesystem::exists(options.rootPath))
        {
            error = "Replay root does not exist: " + options.rootPath.string();
            return false;
        }

        std::unordered_map<std::string, RunCandidate> selectedRuns;
        candidateCsvCount = 0U;

        const std::filesystem::directory_options iteratorOptions = std::filesystem::directory_options::skip_permission_denied;
        for (const std::filesystem::directory_entry& entry : std::filesystem::recursive_directory_iterator(options.rootPath, iteratorOptions))
        {
            if (!entry.is_regular_file() || entry.path().filename() != kCsvFileName)
            {
                continue;
            }

            ++candidateCsvCount;

            const std::filesystem::path sidecarPath = entry.path().parent_path() / kSidecarFileName;
            if (!std::filesystem::exists(sidecarPath))
            {
                continue;
            }

            RunCandidate candidate{};
            candidate.csvPath = entry.path();
            if (!ParseSidecar(sidecarPath, candidate.sidecar, error))
            {
                return false;
            }

            if (!options.runIdFilter.empty() && candidate.sidecar.runId != options.runIdFilter)
            {
                continue;
            }

            candidate.controlLogPath = ResolveControlLogPath(candidate);
            if (ReadBatteryVoltageStart(candidate.controlLogPath, candidate.batteryVoltageV))
            {
                candidate.batterySource = "logging_txt_run_start";
            }

            const auto existing = selectedRuns.find(candidate.sidecar.runId);
            if (existing == selectedRuns.end())
            {
                selectedRuns.emplace(candidate.sidecar.runId, candidate);
                continue;
            }

            if (PreferCandidate(candidate, existing->second))
            {
                duplicates.push_back({ candidate.sidecar.runId, candidate.csvPath, existing->second.csvPath });
                existing->second = candidate;
            }
            else
            {
                duplicates.push_back({ candidate.sidecar.runId, existing->second.csvPath, candidate.csvPath });
            }
        }

        if (selectedRuns.empty())
        {
            error = "No open_floor_main.csv captures found under " + options.rootPath.string();
            return false;
        }

        runs.clear();
        runs.reserve(selectedRuns.size());
        for (auto& item : selectedRuns)
        {
            runs.push_back(item.second);
        }

        std::sort(runs.begin(), runs.end(), RunCandidateRunIdLess);
        return true;
    }

    static bool ValidateCsvHeader(
        const std::vector<std::string>& headerFields,
        const SidecarInfo& sidecar,
        std::string& error)
    {
        if (headerFields.size() != sidecar.fieldNames.size())
        {
            error = "CSV header field count does not match sidecar: " + sidecar.path.string();
            return false;
        }

        for (std::size_t index = 0; index < headerFields.size(); ++index)
        {
            if (headerFields[index] != sidecar.fieldNames[index])
            {
                error = "CSV header does not match sidecar at field " + std::to_string(index);
                return false;
            }
        }

        return true;
    }

    static bool LoadRows(const RunCandidate& candidate, std::vector<LoggedRow>& rows, std::string& error)
    {
        std::ifstream stream(candidate.csvPath);
        if (!stream)
        {
            error = "Failed to open CSV: " + candidate.csvPath.string();
            return false;
        }

        std::string headerLine;
        if (!std::getline(stream, headerLine))
        {
            error = "CSV header missing: " + candidate.csvPath.string();
            return false;
        }

        const std::vector<std::string> headerFields = SplitCommaSeparated(Trim(headerLine));
        if (!ValidateCsvHeader(headerFields, candidate.sidecar, error))
        {
            return false;
        }

        std::unordered_map<std::string, std::size_t> indices;
        for (std::size_t index = 0; index < headerFields.size(); ++index)
        {
            indices.emplace(headerFields[index], index);
        }

        const auto fanIt = candidate.sidecar.metadata.find("fan_duty_cycle");
        float loggedFanDutyCycle = 0.0f;
        if (fanIt == candidate.sidecar.metadata.end() || !ParseFloat(fanIt->second, loggedFanDutyCycle))
        {
            error = "Missing or invalid fan_duty_cycle metadata: " + candidate.sidecar.path.string();
            return false;
        }
        const auto gyroScaleIt = candidate.sidecar.metadata.find("imu_gyro_mdps_per_lsb");
        float loggedGyroMdpsPerLsb = std::numeric_limits<float>::quiet_NaN();
        if (gyroScaleIt == candidate.sidecar.metadata.end() ||
            !ParseFloat(gyroScaleIt->second, loggedGyroMdpsPerLsb) ||
            !std::isfinite(loggedGyroMdpsPerLsb) ||
            (loggedGyroMdpsPerLsb <= 0.0f))
        {
            error = "Missing or invalid imu_gyro_mdps_per_lsb metadata: " + candidate.sidecar.path.string();
            return false;
        }
        const auto accelScaleIt = candidate.sidecar.metadata.find("imu_accel_mg_per_lsb");
        float loggedAccelMgPerLsb = std::numeric_limits<float>::quiet_NaN();
        if (accelScaleIt == candidate.sidecar.metadata.end() ||
            !ParseFloat(accelScaleIt->second, loggedAccelMgPerLsb) ||
            !std::isfinite(loggedAccelMgPerLsb) ||
            (loggedAccelMgPerLsb <= 0.0f))
        {
            error = "Missing or invalid imu_accel_mg_per_lsb metadata: " + candidate.sidecar.path.string();
            return false;
        }
        const bool hasWallAdcFields =
            indices.find("front_left_wall_ambient_adc") != indices.end();

        rows.clear();
        std::string line;
        while (std::getline(stream, line))
        {
            line = Trim(line);
            if (line.empty())
            {
                continue;
            }

            const std::vector<std::string> tokens = SplitCommaSeparated(line);
            if (tokens.size() != headerFields.size())
            {
                error = "CSV row width mismatch: " + candidate.csvPath.string();
                return false;
            }

            LoggedRow row{};
            float loggedPxM = 0.0f;
            float loggedPyM = 0.0f;
            float loggedHeadingRad = 0.0f;
            float loggedForwardMps = 0.0f;
            float loggedRightwardMps = 0.0f;
            float loggedYawRateRadps = 0.0f;
            float loggedForwardAccelResidualMps2 = 0.0f;
            float loggedRightwardAccelResidualMps2 = 0.0f;
            float loggedYawAccelResidualRadps2 = 0.0f;
            if (!ParseField(tokens, indices, "master_time_us", row.masterTimeUs, ParseUnsigned32, error) ||
                !ParseField(tokens, indices, "control_tick_sequence", row.controlTickSequence, ParseUnsigned32, error) ||
                !ParseField(tokens, indices, "dt_us", row.dtUs, ParseUnsigned32, error) ||
                !ParseField(tokens, indices, "section_id", row.sectionId, ParseUnsigned8, error) ||
                !ParseField(tokens, indices, "primitive_id", row.primitiveId, ParseUnsigned8, error) ||
                !ParseField(tokens, indices, "phase_id", row.phaseId, ParseUnsigned8, error) ||
                !ParseField(tokens, indices, "repeat_index", row.repeatIndex, ParseUnsigned16, error) ||
                !ParseField(tokens, indices, "saturation_flags", row.saturationFlags, ParseUnsigned16, error) ||
                !ParseField(tokens, indices, "ukf_state_px_m", loggedPxM, ParseFloat, error) ||
                !ParseField(tokens, indices, "ukf_state_py_m", loggedPyM, ParseFloat, error) ||
                !ParseField(tokens, indices, "ukf_state_heading_rad", loggedHeadingRad, ParseFloat, error) ||
                !ParseField(tokens, indices, "ukf_state_vf_mps", loggedForwardMps, ParseFloat, error) ||
                !ParseField(tokens, indices, "ukf_state_vr_mps", loggedRightwardMps, ParseFloat, error) ||
                !ParseField(tokens, indices, "ukf_state_yaw_rate_radps", loggedYawRateRadps, ParseFloat, error) ||
                !ParseField(tokens, indices, "ukf_state_delta_af_mps2", loggedForwardAccelResidualMps2, ParseFloat, error) ||
                !ParseField(tokens, indices, "ukf_state_delta_ar_mps2", loggedRightwardAccelResidualMps2, ParseFloat, error) ||
                !ParseField(tokens, indices, "ukf_state_delta_yaw_accel_radps2", loggedYawAccelResidualRadps2, ParseFloat, error) ||
                !ParseField(tokens, indices, "measured_linear_speed_mps", row.measuredLinearSpeedMps, ParseFloat, error) ||
                !ParseField(tokens, indices, "measured_yaw_rate_radps", row.measuredYawRateRadps, ParseFloat, error) ||
                !ParseField(tokens, indices, "cmd_linear_mps", row.commandedLinearMps, ParseFloat, error) ||
                !ParseField(tokens, indices, "cmd_yaw_rate_radps", row.commandedYawRateRadps, ParseFloat, error) ||
                !ParseField(tokens, indices, "left_drive_command", row.leftDriveCommand, ParseFloat, error) ||
                !ParseField(tokens, indices, "right_drive_command", row.rightDriveCommand, ParseFloat, error) ||
                !ParseField(tokens, indices, "left_plant_command", row.leftPlantCommand, ParseFloat, error) ||
                !ParseField(tokens, indices, "right_plant_command", row.rightPlantCommand, ParseFloat, error) ||
                !ParseField(tokens, indices, "left_encoder_count", row.leftEncoderCount, ParseInt32, error) ||
                !ParseField(tokens, indices, "right_encoder_count", row.rightEncoderCount, ParseInt32, error) ||
                !ParseField(tokens, indices, "imu_status", row.imuStatus, ParseUnsigned8, error) ||
                !ParseField(tokens, indices, "imu_gyro_x", row.imuGyroX, ParseInt16, error) ||
                !ParseField(tokens, indices, "imu_gyro_y", row.imuGyroY, ParseInt16, error) ||
                !ParseField(tokens, indices, "imu_gyro_z", row.imuGyroZ, ParseInt16, error) ||
                !ParseField(tokens, indices, "imu_accel_x", row.imuAccelX, ParseInt16, error) ||
                !ParseField(tokens, indices, "imu_accel_y", row.imuAccelY, ParseInt16, error) ||
                !ParseField(tokens, indices, "imu_accel_z", row.imuAccelZ, ParseInt16, error) ||
                !ParseField(tokens, indices, "imu_temp", row.imuTemp, ParseInt16, error) ||
                !ParseField(tokens, indices, "accel_body_right_mps2", row.accelBodyRightMps2, ParseFloat, error) ||
                !ParseField(tokens, indices, "accel_body_forward_mps2", row.accelBodyForwardMps2, ParseFloat, error) ||
                !ParseField(tokens, indices, "planar_accel_mps2", row.planarAccelMps2, ParseFloat, error))
            {
                return false;
            }

            std::uint8_t accelBiasValid = 0U;
            if (!ParseField(tokens, indices, "accel_bias_valid", accelBiasValid, ParseUnsigned8, error))
            {
                return false;
            }

            row.accelBiasValid = (accelBiasValid != 0U);
            row.imuGyroMdpsPerLsb = loggedGyroMdpsPerLsb;
            row.imuAccelMgPerLsb = loggedAccelMgPerLsb;
            row.gyroRawRadps = RawImuZToBodyYawRateRadps(row.imuGyroZ, row.imuGyroMdpsPerLsb);
            if (!std::isfinite(row.gyroRawRadps))
            {
                error =
                    "Failed to derive raw IMU yaw rate from imu_gyro_z and imu_gyro_mdps_per_lsb: " +
                    candidate.csvPath.string();
                return false;
            }
            bool correctedYawRatePresent = false;
            if (!ParseFieldIfPresent(tokens, indices, "gyro_raw_radps", row.gyroRawDebugRadps, ParseFloat, error) ||
                !ParseFieldIfPresent(tokens, indices, "gyro_bias_radps", row.gyroBiasDebugRadps, ParseFloat, error) ||
                !ParseFieldIfPresent(
                    tokens,
                    indices,
                    "gyro_radps",
                    row.correctedYawRateRadps,
                    ParseFloat,
                    correctedYawRatePresent,
                    error))
            {
                return false;
            }
            if (!correctedYawRatePresent)
            {
                error = "Missing corrected yaw-rate field gyro_radps: " + candidate.csvPath.string();
                return false;
            }
            if (hasWallAdcFields)
            {
                row.wallAdcPresent = true;
                if (!ParseField(tokens, indices, "front_left_wall_ambient_adc", row.frontLeftWallAmbientAdc, ParseUnsigned16, error) ||
                    !ParseField(tokens, indices, "front_left_wall_lit_adc", row.frontLeftWallLitAdc, ParseUnsigned16, error) ||
                    !ParseField(tokens, indices, "front_right_wall_ambient_adc", row.frontRightWallAmbientAdc, ParseUnsigned16, error) ||
                    !ParseField(tokens, indices, "front_right_wall_lit_adc", row.frontRightWallLitAdc, ParseUnsigned16, error) ||
                    !ParseField(tokens, indices, "side_left_wall_ambient_adc", row.sideLeftWallAmbientAdc, ParseUnsigned16, error) ||
                    !ParseField(tokens, indices, "side_left_wall_lit_adc", row.sideLeftWallLitAdc, ParseUnsigned16, error) ||
                    !ParseField(tokens, indices, "side_right_wall_ambient_adc", row.sideRightWallAmbientAdc, ParseUnsigned16, error) ||
                    !ParseField(tokens, indices, "side_right_wall_lit_adc", row.sideRightWallLitAdc, ParseUnsigned16, error))
                {
                    return false;
                }
            }
            row.fanDutyCycle = loggedFanDutyCycle;
            row.loggedState.SetPosition(Eigen::Vector2f(loggedPxM, loggedPyM));
            row.loggedState.SetHeading(loggedHeadingRad);
            row.loggedState.SetForwardVelocity(loggedForwardMps);
            row.loggedState.SetRightwardVelocity(loggedRightwardMps);
            row.loggedState.SetYawRate(loggedYawRateRadps);
            row.loggedState.SetForwardAccelerationResidual(loggedForwardAccelResidualMps2);
            row.loggedState.SetRightwardAccelerationResidual(loggedRightwardAccelResidualMps2);
            row.loggedState.SetYawAccelResidual(loggedYawAccelResidualRadps2);
            row.loggedState.SetCurrentCommand(
                MazeMap::App::Internal::CommandVector(row.leftDriveCommand, row.rightDriveCommand));
            rows.push_back(row);
        }

        if (rows.empty())
        {
            error = "CSV did not contain any rows: " + candidate.csvPath.string();
            return false;
        }

        if (!DeriveEncoderKinematicsFromCounts(rows, error))
        {
            return false;
        }

        return true;
    }

    static SensorSnapshot::EncoderObs BuildEncoderObservation(const LoggedRow& row) noexcept
    {
        SensorSnapshot::EncoderObs observation = SensorSnapshot{}.EncoderObservation();
        observation.SetCounts(row.leftEncoderCount, row.rightEncoderCount);
        observation.SetDistanceDeltasM(row.leftEncoderDistanceDeltaM, row.rightEncoderDistanceDeltaM);
        observation.SetWheelLinearVelocityMps(row.leftEncoderVelocityMps, row.rightEncoderVelocityMps);
        observation.SetWheelSpeedRadps(row.leftEncoderWheelSpeedRadps, row.rightEncoderWheelSpeedRadps);
        return observation;
    }

    static float SensorForwardVelocityMps(const LoggedRow& row) noexcept
    {
        if (!row.encoderKinematicsValid)
        {
            return std::numeric_limits<float>::quiet_NaN();
        }

        return 0.5f * (row.leftEncoderVelocityMps + row.rightEncoderVelocityMps);
    }

    static float CommandAverage(float leftCommand, float rightCommand) noexcept
    {
        return 0.5f * (leftCommand + rightCommand);
    }

    static float CommandDelta(float leftCommand, float rightCommand) noexcept
    {
        return 0.5f * (rightCommand - leftCommand);
    }

    static bool IsSensorComparableTransition(const LoggedRow& currentRow, const LoggedRow& nextRow) noexcept
    {
        return
            (currentRow.sectionId == nextRow.sectionId) &&
            (currentRow.primitiveId == nextRow.primitiveId) &&
            (currentRow.repeatIndex == nextRow.repeatIndex);
    }

    static void ScoreFeedforward(
        const FeedforwardSampleExportRow& row,
        FeedforwardMetrics& metrics) noexcept
    {
        metrics.leftDriveCommand.add(row.leftDriveCommandError);
        metrics.rightDriveCommand.add(row.rightDriveCommandError);
        metrics.averageDriveCommand.add(row.averageDriveCommandError);
        metrics.deltaDriveCommand.add(row.deltaDriveCommandError);
        metrics.leftPlantCommand.add(row.leftPlantCommandError);
        metrics.rightPlantCommand.add(row.rightPlantCommandError);
        metrics.averagePlantCommand.add(row.averagePlantCommandError);
        metrics.deltaPlantCommand.add(row.deltaPlantCommandError);
        metrics.predictedForwardTargetErrorMps.add(row.predictedForwardTargetErrorMps);
        metrics.predictedYawTargetErrorRadps.add(row.predictedYawTargetErrorRadps);
        if (std::isfinite(row.envelopeLeftMin) &&
            std::isfinite(row.envelopeLeftMax) &&
            std::isfinite(row.envelopeRightMin) &&
            std::isfinite(row.envelopeRightMax))
        {
            metrics.driveEnvelopeHit.add(row.loggedDriveWithinEnvelope);
            metrics.plantCommandEnvelopeHit.add(row.loggedPlantCommandWithinEnvelope);
        }
    }

    static bool TryComputeFeedforwardCommand(
        MazeMap::PlantModel& plantModel,
        MazeMap::Vehicle& vehicle,
        const LoggedRow& sourceRow,
        const LoggedRow& targetRow,
        MazeMap::App::Internal::CommandVector& solvedCommand) noexcept
    {
        if (!IsSensorComparableTransition(sourceRow, targetRow))
        {
            return false;
        }

        const float responseTimeS = static_cast<float>(targetRow.dtUs) * 1.0e-6f;
        const float currentForwardVelocityMps = SensorForwardVelocityMps(sourceRow);
        const float currentYawRateRadps = sourceRow.correctedYawRateRadps;
        const float targetForwardVelocityMps = SensorForwardVelocityMps(targetRow);
        const float targetYawRateRadps = targetRow.correctedYawRateRadps;
        if (!(std::isfinite(responseTimeS) &&
            (responseTimeS > 0.0f) &&
            std::isfinite(sourceRow.leftEncoderVelocityMps) &&
            std::isfinite(sourceRow.rightEncoderVelocityMps) &&
            std::isfinite(currentForwardVelocityMps) &&
            std::isfinite(currentYawRateRadps) &&
            std::isfinite(targetForwardVelocityMps) &&
            std::isfinite(targetYawRateRadps) &&
            (MazeMap::Vehicle::GetDriveWheelRadiusM() > 0.0f)))
        {
            return false;
        }

        vehicle.SetFanDuty(sourceRow.fanDutyCycle);
        float maxLongitudinalAccelMps2 = 0.0f;
        float maxYawAccelRadps2 = 0.0f;
        plantModel.velocityTargetTechnicalLimits(
            currentForwardVelocityMps,
            currentYawRateRadps,
            maxLongitudinalAccelMps2,
            maxYawAccelRadps2);

        const float longitudinalAccelLimitMps2 =
            (std::isfinite(maxLongitudinalAccelMps2) && (maxLongitudinalAccelMps2 > 0.0f)) ?
            maxLongitudinalAccelMps2 :
            0.0f;
        const float yawAccelLimitRadps2 =
            (std::isfinite(maxYawAccelRadps2) && (maxYawAccelRadps2 > 0.0f)) ?
            maxYawAccelRadps2 :
            0.0f;
        float desiredLongitudinalAccelMps2 =
            (longitudinalAccelLimitMps2 > 0.0f) ?
            ((targetForwardVelocityMps - currentForwardVelocityMps) / responseTimeS) :
            0.0f;
        float desiredYawAccelRadps2 =
            (yawAccelLimitRadps2 > 0.0f) ?
            ((targetYawRateRadps - currentYawRateRadps) / responseTimeS) :
            0.0f;
        const float longitudinalDemand =
            (longitudinalAccelLimitMps2 > 0.0f) ?
            (std::fabs(desiredLongitudinalAccelMps2) / longitudinalAccelLimitMps2) :
            0.0f;
        const float yawDemand =
            (yawAccelLimitRadps2 > 0.0f) ?
            (std::fabs(desiredYawAccelRadps2) / yawAccelLimitRadps2) :
            0.0f;
        const float balanceScale = (std::max)(1.0f, (std::max)(longitudinalDemand, yawDemand));
        desiredLongitudinalAccelMps2 =
            (std::clamp)(
                desiredLongitudinalAccelMps2 / balanceScale,
                -longitudinalAccelLimitMps2,
                longitudinalAccelLimitMps2);
        desiredYawAccelRadps2 =
            (std::clamp)(
                desiredYawAccelRadps2 / balanceScale,
                -yawAccelLimitRadps2,
                yawAccelLimitRadps2);

        if (!(std::isfinite(desiredLongitudinalAccelMps2) && std::isfinite(desiredYawAccelRadps2)))
        {
            return false;
        }

        solvedCommand = plantModel.ComputeFeedforward(
            desiredLongitudinalAccelMps2,
            desiredYawAccelRadps2);
        return solvedCommand.IsFinite();
    }

    static bool BuildFeedforwardSampleExportRow(
        MazeMap::PlantModel& plantModel,
        MazeMap::Vehicle& vehicle,
        MazeMap::VehicleState& runtimeState,
        const LoggedRow& currentRow,
        const LoggedRow& nextRow,
        FeedforwardSampleExportRow& exportRow) noexcept
    {
        MazeMap::App::Internal::CommandVector command{};
        if (!TryComputeFeedforwardCommand(plantModel, vehicle, currentRow, nextRow, command))
        {
            return false;
        }

        FeedforwardEnvelope envelope{};
        for (int currentLeftSign = -1; currentLeftSign <= 1; currentLeftSign += 2)
        {
            for (int currentRightSign = -1; currentRightSign <= 1; currentRightSign += 2)
            {
                for (int nextLeftSign = -1; nextLeftSign <= 1; nextLeftSign += 2)
                {
                    for (int nextRightSign = -1; nextRightSign <= 1; nextRightSign += 2)
                    {
                        for (int currentYawSign = -1; currentYawSign <= 1; currentYawSign += 2)
                        {
                            for (int nextYawSign = -1; nextYawSign <= 1; nextYawSign += 2)
                            {
                                LoggedRow perturbedCurrent = currentRow;
                                LoggedRow perturbedNext = nextRow;
                                perturbedCurrent.leftEncoderVelocityMps +=
                                    static_cast<float>(currentLeftSign) * kWheelVelocitySensorBoundMps;
                                perturbedCurrent.rightEncoderVelocityMps +=
                                    static_cast<float>(currentRightSign) * kWheelVelocitySensorBoundMps;
                                perturbedNext.leftEncoderVelocityMps +=
                                    static_cast<float>(nextLeftSign) * kWheelVelocitySensorBoundMps;
                                perturbedNext.rightEncoderVelocityMps +=
                                    static_cast<float>(nextRightSign) * kWheelVelocitySensorBoundMps;
                                perturbedCurrent.correctedYawRateRadps +=
                                    static_cast<float>(currentYawSign) * kYawRateSensorBoundRadps;
                                perturbedNext.correctedYawRateRadps +=
                                    static_cast<float>(nextYawSign) * kYawRateSensorBoundRadps;

                                MazeMap::App::Internal::CommandVector perturbedCommand{};
                                if (!TryComputeFeedforwardCommand(
                                    plantModel,
                                    vehicle,
                                    perturbedCurrent,
                                    perturbedNext,
                                    perturbedCommand))
                                {
                                    continue;
                                }

                                envelope.valid = true;
                                envelope.leftMin = (std::min)(envelope.leftMin, perturbedCommand.LeftCommand());
                                envelope.leftMax = (std::max)(envelope.leftMax, perturbedCommand.LeftCommand());
                                envelope.rightMin = (std::min)(envelope.rightMin, perturbedCommand.RightCommand());
                                envelope.rightMax = (std::max)(envelope.rightMax, perturbedCommand.RightCommand());
                            }
                        }
                    }
                }
            }
        }

        const float responseTimeS = static_cast<float>(nextRow.dtUs) * 1.0e-6f;
        vehicle.SetFanDuty(currentRow.fanDutyCycle);
        runtimeState.SetPosition(Eigen::Vector2f::Zero());
        runtimeState.SetHeading(0.0f);
        runtimeState.SetForwardVelocity(SensorForwardVelocityMps(currentRow));
        runtimeState.SetRightwardVelocity(0.0f);
        runtimeState.SetYawRate(currentRow.correctedYawRateRadps);
        runtimeState.PublishEncoderWheelSpeedsRadps(
            MazeMap::Vehicle::WheelSpeedFromLinearVelocity(currentRow.leftEncoderVelocityMps),
            MazeMap::Vehicle::WheelSpeedFromLinearVelocity(currentRow.rightEncoderVelocityMps));
        plantModel.integrate(command, responseTimeS);

        exportRow = {};
        exportRow.masterTimeUs = currentRow.masterTimeUs;
        exportRow.nextMasterTimeUs = nextRow.masterTimeUs;
        exportRow.controlTickSequence = currentRow.controlTickSequence;
        exportRow.nextControlTickSequence = nextRow.controlTickSequence;
        exportRow.dtUs = nextRow.dtUs;
        exportRow.sectionId = currentRow.sectionId;
        exportRow.primitiveId = currentRow.primitiveId;
        exportRow.phaseId = currentRow.phaseId;
        exportRow.repeatIndex = currentRow.repeatIndex;
        exportRow.saturationFlags = currentRow.saturationFlags;
        exportRow.pathId = kAccelerationFeedforwardPathId;
        exportRow.pathLabel = kAccelerationFeedforwardPathLabel;
        exportRow.pathCategory = kAccelerationFeedforwardPathCategory;
        exportRow.sectionName = SectionName(currentRow.sectionId);
        exportRow.primitiveName = PrimitiveName(currentRow.primitiveId);
        exportRow.phaseName = PhaseName(currentRow.phaseId);
        exportRow.currentForwardSensorMps = SensorForwardVelocityMps(currentRow);
        exportRow.currentLeftVelocityMps = currentRow.leftEncoderVelocityMps;
        exportRow.currentRightVelocityMps = currentRow.rightEncoderVelocityMps;
        exportRow.currentYawRateSensorRadps = currentRow.correctedYawRateRadps;
        exportRow.targetForwardSensorMps = SensorForwardVelocityMps(nextRow);
        exportRow.targetLeftVelocityMps = nextRow.leftEncoderVelocityMps;
        exportRow.targetRightVelocityMps = nextRow.rightEncoderVelocityMps;
        exportRow.targetYawRateSensorRadps = nextRow.correctedYawRateRadps;
        exportRow.nominalLeftCommand = command.LeftCommand();
        exportRow.nominalRightCommand = command.RightCommand();
        exportRow.nominalAverageCommand =
            CommandAverage(exportRow.nominalLeftCommand, exportRow.nominalRightCommand);
        exportRow.nominalDeltaCommand =
            CommandDelta(exportRow.nominalLeftCommand, exportRow.nominalRightCommand);
        exportRow.loggedLeftDriveCommand = currentRow.leftDriveCommand;
        exportRow.loggedRightDriveCommand = currentRow.rightDriveCommand;
        exportRow.loggedAverageDriveCommand =
            CommandAverage(currentRow.leftDriveCommand, currentRow.rightDriveCommand);
        exportRow.loggedDeltaDriveCommand =
            CommandDelta(currentRow.leftDriveCommand, currentRow.rightDriveCommand);
        exportRow.loggedLeftPlantCommand = currentRow.leftPlantCommand;
        exportRow.loggedRightPlantCommand = currentRow.rightPlantCommand;
        exportRow.loggedAveragePlantCommand =
            CommandAverage(currentRow.leftPlantCommand, currentRow.rightPlantCommand);
        exportRow.loggedDeltaPlantCommand =
            CommandDelta(currentRow.leftPlantCommand, currentRow.rightPlantCommand);
        exportRow.leftDriveCommandError = exportRow.nominalLeftCommand - exportRow.loggedLeftDriveCommand;
        exportRow.rightDriveCommandError = exportRow.nominalRightCommand - exportRow.loggedRightDriveCommand;
        exportRow.averageDriveCommandError = exportRow.nominalAverageCommand - exportRow.loggedAverageDriveCommand;
        exportRow.deltaDriveCommandError = exportRow.nominalDeltaCommand - exportRow.loggedDeltaDriveCommand;
        exportRow.leftPlantCommandError =
            exportRow.nominalLeftCommand - exportRow.loggedLeftPlantCommand;
        exportRow.rightPlantCommandError =
            exportRow.nominalRightCommand - exportRow.loggedRightPlantCommand;
        exportRow.averagePlantCommandError =
            exportRow.nominalAverageCommand - exportRow.loggedAveragePlantCommand;
        exportRow.deltaPlantCommandError =
            exportRow.nominalDeltaCommand - exportRow.loggedDeltaPlantCommand;
        exportRow.predictedNextForwardMps = runtimeState.GetForwardVelocity();
        exportRow.predictedNextYawRateRadps = runtimeState.GetYawRate();
        exportRow.predictedForwardTargetErrorMps =
            exportRow.predictedNextForwardMps - exportRow.targetForwardSensorMps;
        exportRow.predictedYawTargetErrorRadps =
            exportRow.predictedNextYawRateRadps - exportRow.targetYawRateSensorRadps;
        if (envelope.valid)
        {
            exportRow.envelopeLeftMin = envelope.leftMin;
            exportRow.envelopeLeftMax = envelope.leftMax;
            exportRow.envelopeRightMin = envelope.rightMin;
            exportRow.envelopeRightMax = envelope.rightMax;
            exportRow.loggedDriveWithinEnvelope =
                (currentRow.leftDriveCommand >= envelope.leftMin) &&
                (currentRow.leftDriveCommand <= envelope.leftMax) &&
                (currentRow.rightDriveCommand >= envelope.rightMin) &&
                (currentRow.rightDriveCommand <= envelope.rightMax);
            exportRow.loggedPlantCommandWithinEnvelope =
                (currentRow.leftPlantCommand >= envelope.leftMin) &&
                (currentRow.leftPlantCommand <= envelope.leftMax) &&
                (currentRow.rightPlantCommand >= envelope.rightMin) &&
                (currentRow.rightPlantCommand <= envelope.rightMax);
        }
        return true;
    }

    SampleExportRow BuildSampleExportRow(
        const LoggedRow& row,
        const MazeMap::VehicleState& predictedState,
        const Eigen::Vector2f& predictedAccelBodyMps2)
    {
        SampleExportRow exportRow{};
        exportRow.masterTimeUs = row.masterTimeUs;
        exportRow.controlTickSequence = row.controlTickSequence;
        exportRow.dtUs = row.dtUs;
        exportRow.sectionId = row.sectionId;
        exportRow.primitiveId = row.primitiveId;
        exportRow.phaseId = row.phaseId;
        exportRow.repeatIndex = row.repeatIndex;
        exportRow.accelBiasValid = row.accelBiasValid;
        exportRow.sectionName = SectionName(row.sectionId);
        exportRow.primitiveName = PrimitiveName(row.primitiveId);
        exportRow.phaseName = PhaseName(row.phaseId);
        exportRow.predictedAccelBodyRightMps2 = predictedAccelBodyMps2.x();
        exportRow.actualAccelBodyRightMps2 = row.accelBodyRightMps2;
        exportRow.predictedAccelBodyForwardMps2 = predictedAccelBodyMps2.y();
        exportRow.actualAccelBodyForwardMps2 = row.accelBodyForwardMps2;
        exportRow.predictedPlanarAccelMps2 = predictedAccelBodyMps2.norm();
        exportRow.actualPlanarAccelMps2 = row.planarAccelMps2;
        exportRow.predictedLinearSpeedMps = predictedState.GetForwardVelocity();
        exportRow.actualLinearSpeedMps = row.measuredLinearSpeedMps;
        exportRow.predictedYawRateRadps = predictedState.GetYawRate();
        exportRow.actualYawRateRadps = row.correctedYawRateRadps;
        exportRow.predictedRawGyroRadps = predictedState.GetYawRate();
        exportRow.actualRawGyroRadps = row.gyroRawRadps;
        return exportRow;
    }

    static void ScorePrediction(
        const LoggedRow& row,
        const MazeMap::VehicleState& predictedState,
        const Eigen::Vector2f& predictedAccelBodyMps2,
        PredictionMetrics& metrics)
    {
        metrics.leftEncoderWheelSpeedRadps.add(
            static_cast<double>(predictedState.GetWheelSpeedLeft()) -
            static_cast<double>(row.leftEncoderWheelSpeedRadps));
        metrics.rightEncoderWheelSpeedRadps.add(
            static_cast<double>(predictedState.GetWheelSpeedRight()) -
            static_cast<double>(row.rightEncoderWheelSpeedRadps));

        const double predictedEncoderLinearMps =
            static_cast<double>(MazeMap::Vehicle::BodyForwardVelocityFromWheelLinear(
                MazeMap::Vehicle::WheelLinearVelocityFromWheelSpeed(predictedState.GetWheelSpeedLeft()),
                MazeMap::Vehicle::WheelLinearVelocityFromWheelSpeed(predictedState.GetWheelSpeedRight())));
        metrics.encoderLinearSpeedMps.add(predictedEncoderLinearMps - static_cast<double>(row.measuredLinearSpeedMps));

        const double predictedEncoderYawRadps =
            static_cast<double>(MazeMap::Vehicle::BodyYawRateFromWheelLinear(
                MazeMap::Vehicle::WheelLinearVelocityFromWheelSpeed(predictedState.GetWheelSpeedLeft()),
                MazeMap::Vehicle::WheelLinearVelocityFromWheelSpeed(predictedState.GetWheelSpeedRight())));
        metrics.encoderYawRateRadps.add(predictedEncoderYawRadps - static_cast<double>(row.measuredYawRateRadps));

        metrics.bodyForwardSpeedMps.add(
            static_cast<double>(predictedState.GetForwardVelocity()) -
            static_cast<double>(row.measuredLinearSpeedMps));
        metrics.bodyYawRateRadps.add(
            static_cast<double>(predictedState.GetYawRate()) -
            static_cast<double>(row.correctedYawRateRadps));

        if (std::isfinite(row.gyroRawRadps))
        {
            const double predictedRawGyroRadps = static_cast<double>(predictedState.GetYawRate());
            metrics.rawGyroRadps.add(predictedRawGyroRadps - static_cast<double>(row.gyroRawRadps));
        }

        if (row.accelBiasValid)
        {
            metrics.accelBodyRightMps2.add(
                static_cast<double>(predictedAccelBodyMps2.x()) - static_cast<double>(row.accelBodyRightMps2));
            metrics.accelBodyForwardMps2.add(
                static_cast<double>(predictedAccelBodyMps2.y()) - static_cast<double>(row.accelBodyForwardMps2));
            metrics.planarAccelMps2.add(
                std::hypot(static_cast<double>(predictedAccelBodyMps2.x()), static_cast<double>(predictedAccelBodyMps2.y())) -
                static_cast<double>(row.planarAccelMps2));
        }
    }

    static void ScoreConsistency(
        const LoggedRow& row,
        const MazeMap::VehicleState& replayedState,
        ConsistencyMetrics& metrics)
    {
        const double dxM = static_cast<double>(replayedState.GetPositionX() - row.loggedState.GetPositionX());
        const double dyM = static_cast<double>(replayedState.GetPositionY() - row.loggedState.GetPositionY());
        metrics.positionMm.add(1000.0 * std::hypot(dxM, dyM));
        metrics.headingDeg.add(
            static_cast<double>(kRadiansToDegrees) *
            static_cast<double>(NormalizeAngle(replayedState.GetHeading() - row.loggedState.GetHeading())));
        metrics.forwardSpeedMps.add(
            static_cast<double>(replayedState.GetForwardVelocity() - row.loggedState.GetForwardVelocity()));
        metrics.rightwardSpeedMps.add(
            static_cast<double>(replayedState.GetRightwardVelocity() - row.loggedState.GetRightwardVelocity()));
        metrics.yawRateRadps.add(
            static_cast<double>(replayedState.GetYawRate() - row.loggedState.GetYawRate()));
        metrics.leftWheelSpeedRadps.add(
            static_cast<double>(replayedState.GetWheelSpeedLeft() - row.loggedState.GetWheelSpeedLeft()));
        metrics.rightWheelSpeedRadps.add(
            static_cast<double>(replayedState.GetWheelSpeedRight() - row.loggedState.GetWheelSpeedRight()));
    }

    static RunReport ReplayRun(const RunCandidate& candidate, const ReplayOptions& options)
    {
        RunReport report{};
        report.runId = candidate.sidecar.runId;
        report.formatVersion = candidate.sidecar.formatVersion;
        report.csvPath = candidate.csvPath;
        report.batterySource = candidate.batterySource;
        report.batteryVoltageV = candidate.batteryVoltageV;
        report.feedforwardPaths = BuildDefaultFeedforwardPathSummaries();

        std::string error;
        std::vector<LoggedRow> rows;
        if (!LoadRows(candidate, rows, error))
        {
            report.completed = false;
            report.failureReason = error;
            return report;
        }

        report.totalRows = rows.size();
        std::uint8_t ignoredSectionId = 0U;
        for (const LoggedRow& row : rows)
        {
            ignoredSectionId = (std::max)(ignoredSectionId, row.sectionId);
        }

        std::vector<LoggedRow> keptRows;
        keptRows.reserve(rows.size());
        for (const LoggedRow& row : rows)
        {
            if (row.sectionId != ignoredSectionId)
            {
                keptRows.push_back(row);
            }
        }

        report.ignoredSectionId = static_cast<int>(ignoredSectionId);
        report.keptRows = keptRows.size();
        if (keptRows.size() < 2U)
        {
            report.completed = false;
            report.failureReason = "Not enough rows after final-section exclusion";
            return report;
        }

        MazeMap::VehicleState initialState =
            options.useKnownStationarySeed ?
            BuildKnownStationaryOpenFloorInitialState() :
            keptRows.front().loggedState;
        MazeMap::Vehicle vehicle;
        MazeMap::VehicleState runtimeState = initialState;
        MazeMap::PlantModel plantModel(vehicle, runtimeState);

        std::unordered_map<SectionPhaseKey, SectionPhaseReport, SectionPhaseKeyHash> sectionPhaseBuckets;
        const bool exportSampleRows =
            !options.sampleCsvPath.empty() &&
            (options.runIdFilter == candidate.sidecar.runId);
        const bool exportFeedforwardRows =
            !options.feedforwardSampleCsvPath.empty() &&
            (options.runIdFilter == candidate.sidecar.runId);
        if (exportSampleRows)
        {
            report.sampleExportRows.reserve(keptRows.size() - 1U);
        }
        if (exportFeedforwardRows)
        {
            report.feedforwardSampleExportRows.reserve((keptRows.size() - 1U) * report.feedforwardPaths.size());
        }

        for (std::size_t index = 0; (index + 1U) < keptRows.size(); ++index)
        {
            const LoggedRow& currentRow = keptRows[index];
            const LoggedRow& nextRow = keptRows[index + 1U];
            if (report.feedforwardPaths.empty())
            {
                continue;
            }

            FeedforwardSampleExportRow sample{};
            if (!BuildFeedforwardSampleExportRow(
                plantModel,
                vehicle,
                runtimeState,
                currentRow,
                nextRow,
                sample))
            {
                continue;
            }

            ++report.feedforwardTransitions;
            FeedforwardPathSummary& pathSummary = report.feedforwardPaths.front();
            ++pathSummary.comparableTransitions;
            ++pathSummary.validSolutions;
            ScoreFeedforward(sample, pathSummary.metrics);
            if (exportFeedforwardRows)
            {
                report.feedforwardSampleExportRows.push_back(std::move(sample));
            }
        }

        runtimeState = initialState;
        MazeMap::Estimator estimator(vehicle, plantModel, runtimeState);

        for (std::size_t index = 1; index < keptRows.size(); ++index)
        {
            const LoggedRow& previousRow = keptRows[index - 1U];
            const LoggedRow& row = keptRows[index];
            ++report.scoredTransitions;

            const float dtSeconds = static_cast<float>(row.dtUs) * 1.0e-6f;
            const MazeMap::App::Internal::CommandVector control(
                previousRow.leftDriveCommand,
                previousRow.rightDriveCommand);
            const SensorSnapshot::EncoderObs encoderObservation = BuildEncoderObservation(row);
            const bool encoderObservationValid =
                row.encoderKinematicsValid &&
                std::isfinite(dtSeconds) &&
                (dtSeconds > 0.0f) &&
                std::isfinite(row.leftEncoderDistanceDeltaM) &&
                std::isfinite(row.rightEncoderDistanceDeltaM) &&
                std::isfinite(row.leftEncoderWheelSpeedRadps) &&
                std::isfinite(row.rightEncoderWheelSpeedRadps) &&
                std::isfinite(row.leftEncoderVelocityMps) &&
                std::isfinite(row.rightEncoderVelocityMps);
            vehicle.SetFanDuty(previousRow.fanDutyCycle);
            runtimeState.SetCurrentCommand(control);
            SensorSnapshot snapshot{};
            snapshot.PublishEncoderObservation(
                encoderObservation,
                encoderObservationValid,
                row.leftEncoderCount,
                row.rightEncoderCount,
                row.leftEncoderDistanceM,
                row.rightEncoderDistanceM);
            snapshot.SetRawYawRateRadps(std::numeric_limits<float>::quiet_NaN());
            snapshot.SetYawRateRadps(row.correctedYawRateRadps);
            snapshot.SetAccelerationBiasValid(row.accelBiasValid);
            snapshot.SetBodyRightAccelerationMps2(row.accelBodyRightMps2);
            snapshot.SetBodyForwardAccelerationMps2(row.accelBodyForwardMps2);
            snapshot.SetPlanarAccelerationMps2(row.planarAccelMps2);
            runtimeState.SetSensorSnapshot(snapshot);
            if (std::isfinite(dtSeconds) && (dtSeconds > 0.0f))
            {
                runtimeState.SetTime(runtimeState.GetTime() + dtSeconds);
            }
            if (!estimator.predict(dtSeconds, control))
            {
                ++report.predictFailures;
                report.completed = false;
                report.failureReason =
                    "predict_failed at control_tick_sequence=" + std::to_string(row.controlTickSequence);
                break;
            }

            const MazeMap::VehicleState& predictedState = runtimeState;
            const Eigen::Vector2f predictedAccelBodyMps2(
                std::numeric_limits<float>::quiet_NaN(),
                std::numeric_limits<float>::quiet_NaN());
            SectionPhaseReport& phaseBucket =
                GetOrCreateSectionPhaseReport(sectionPhaseBuckets, row.sectionId, row.phaseId);
            ++phaseBucket.sampleCount;
            ScorePrediction(
                row,
                predictedState,
                predictedAccelBodyMps2,
                report.prediction);
            ScorePrediction(
                row,
                predictedState,
                predictedAccelBodyMps2,
                phaseBucket.prediction);
            if (exportSampleRows)
            {
                report.sampleExportRows.push_back(BuildSampleExportRow(
                    row,
                    predictedState,
                    predictedAccelBodyMps2));
            }

            if (std::isfinite(snapshot.YawRateRadps()))
            {
                if (!estimator.updateYawRate(snapshot.YawRateRadps()))
                {
                    ++report.yawRejects;
                    report.completed = false;
                    report.failureReason =
                        "yaw_update_failed at control_tick_sequence=" + std::to_string(row.controlTickSequence);
                    break;
                }
            }

            if (row.accelBiasValid)
            {
                MazeMap::ImuAccelObs accelObservation{};
                accelObservation.SetValid(
                    std::isfinite(row.accelBodyRightMps2) &&
                    std::isfinite(row.accelBodyForwardMps2));
                accelObservation.SetBodyForwardRightMps2(
                    row.accelBodyForwardMps2,
                    row.accelBodyRightMps2);
                (void)estimator.updatePlanarAccel(accelObservation);
            }

            ScoreConsistency(row, runtimeState, report.consistency);
            ScoreConsistency(row, runtimeState, phaseBucket.consistency);
        }

        report.sectionPhaseBuckets = ToSortedSectionPhaseReports(sectionPhaseBuckets);
        return report;
    }

    static std::string FormatDouble(double value, int precision = 6)
    {
        std::ostringstream stream;
        stream << std::fixed << std::setprecision(precision) << value;
        return stream.str();
    }

    static void WriteMetricRow(
        std::ostream& stream,
        const char* label,
        const char* unit,
        const ErrorStats& stats)
    {
        stream
            << "| " << label
            << " | " << unit
            << " | " << stats.count
            << " | " << FormatDouble(stats.rmse())
            << " | " << FormatDouble(stats.mae())
            << " | " << FormatDouble(stats.bias())
            << " | " << FormatDouble(stats.maxAbs)
            << " |\n";
    }

    static void WriteHitRateRow(
        std::ostream& stream,
        const char* label,
        const HitRateStats& stats)
    {
        stream
            << "| " << label
            << " | " << stats.total
            << " | " << stats.hits
            << " | " << FormatDouble(100.0 * stats.rate(), 2)
            << " |\n";
    }

    static std::uint64_t TotalFeedforwardValidSolutions(const std::vector<FeedforwardPathSummary>& summaries) noexcept
    {
        std::uint64_t total = 0U;
        for (const FeedforwardPathSummary& summary : summaries)
        {
            total += summary.validSolutions;
        }
        return total;
    }

    static void WritePhaseAssociationRow(
        std::ostream& stream,
        const char* label,
        const PhaseAssociationSummary& summary)
    {
        stream
            << "| " << label
            << " | " << summary.totalSamples
            << " | " << summary.bucketCount
            << " | " << FormatDouble(summary.etaSquaredAbs, 4)
            << " | " << summary.worstBucketLabel
            << " | " << summary.worstBucketSamples
            << " | " << FormatDouble(summary.worstBucketMae)
            << " | " << FormatDouble(summary.globalMae)
            << " | " << FormatDouble(summary.worstBucketMaeRatio, 3)
            << " |\n";
    }

    static std::string SampleMetricValue(const std::string& metric, const SampleExportRow& row)
    {
        if (metric == "master_time_us")
        {
            return std::to_string(row.masterTimeUs);
        }
        if (metric == "control_tick_sequence")
        {
            return std::to_string(row.controlTickSequence);
        }
        if (metric == "dt_us")
        {
            return std::to_string(row.dtUs);
        }
        if (metric == "section_id")
        {
            return std::to_string(static_cast<unsigned>(row.sectionId));
        }
        if (metric == "section_name")
        {
            return row.sectionName;
        }
        if (metric == "primitive_id")
        {
            return std::to_string(static_cast<unsigned>(row.primitiveId));
        }
        if (metric == "primitive_name")
        {
            return row.primitiveName;
        }
        if (metric == "phase_id")
        {
            return std::to_string(static_cast<unsigned>(row.phaseId));
        }
        if (metric == "phase_name")
        {
            return row.phaseName;
        }
        if (metric == "repeat_index")
        {
            return std::to_string(row.repeatIndex);
        }
        if (metric == "accel_bias_valid")
        {
            return row.accelBiasValid ? "true" : "false";
        }
        if (metric == "predicted_accel_body_right_mps2")
        {
            return FormatDouble(row.predictedAccelBodyRightMps2);
        }
        if (metric == "actual_accel_body_right_mps2")
        {
            return FormatDouble(row.actualAccelBodyRightMps2);
        }
        if (metric == "predicted_accel_body_forward_mps2")
        {
            return FormatDouble(row.predictedAccelBodyForwardMps2);
        }
        if (metric == "actual_accel_body_forward_mps2")
        {
            return FormatDouble(row.actualAccelBodyForwardMps2);
        }
        if (metric == "predicted_planar_accel_mps2")
        {
            return FormatDouble(row.predictedPlanarAccelMps2);
        }
        if (metric == "actual_planar_accel_mps2")
        {
            return FormatDouble(row.actualPlanarAccelMps2);
        }
        if (metric == "predicted_linear_speed_mps")
        {
            return FormatDouble(row.predictedLinearSpeedMps);
        }
        if (metric == "actual_linear_speed_mps")
        {
            return FormatDouble(row.actualLinearSpeedMps);
        }
        if (metric == "predicted_yaw_rate_radps")
        {
            return FormatDouble(row.predictedYawRateRadps);
        }
        if (metric == "actual_yaw_rate_radps")
        {
            return FormatDouble(row.actualYawRateRadps);
        }
        if (metric == "predicted_raw_gyro_radps")
        {
            return FormatDouble(row.predictedRawGyroRadps);
        }
        if (metric == "actual_raw_gyro_radps")
        {
            return FormatDouble(row.actualRawGyroRadps);
        }

        return std::string();
    }

    static bool WriteSampleExportCsv(const CorpusReport& corpus, const ReplayOptions& options, std::string& error)
    {
        if (options.sampleCsvPath.empty())
        {
            return true;
        }

        auto runIt = corpus.runs.begin();
        for (; runIt != corpus.runs.end(); ++runIt)
        {
            if (runIt->runId == options.runIdFilter)
            {
                break;
            }
        }
        if (runIt == corpus.runs.end())
        {
            error = "Requested sample export run was not replayed: " + options.runIdFilter;
            return false;
        }

        if (runIt->sampleExportRows.empty())
        {
            error = "Requested sample export run did not produce any sample rows: " + options.runIdFilter;
            return false;
        }

        const std::filesystem::path parentPath = options.sampleCsvPath.parent_path();
        if (!parentPath.empty())
        {
            std::error_code createEc;
            std::filesystem::create_directories(parentPath, createEc);
            if (createEc)
            {
                error = "Failed to create sample CSV directory: " + parentPath.string();
                return false;
            }
        }

        std::ofstream csv(options.sampleCsvPath);
        if (!csv)
        {
            error = "Failed to open sample CSV for write: " + options.sampleCsvPath.string();
            return false;
        }

        for (std::size_t index = 0; index < options.sampleMetrics.size(); ++index)
        {
            if (index > 0U)
            {
                csv << ',';
            }

            csv << options.sampleMetrics[index];
        }
        csv << '\n';

        for (const SampleExportRow& row : runIt->sampleExportRows)
        {
            for (std::size_t index = 0; index < options.sampleMetrics.size(); ++index)
            {
                if (index > 0U)
                {
                    csv << ',';
                }

                csv << SampleMetricValue(options.sampleMetrics[index], row);
            }
            csv << '\n';
        }

        std::cout << "Sample export written to " << options.sampleCsvPath.string() << "\n";
        return true;
    }

    static bool WriteFeedforwardSampleExportCsv(const CorpusReport& corpus, const ReplayOptions& options, std::string& error)
    {
        if (options.feedforwardSampleCsvPath.empty())
        {
            return true;
        }

        auto runIt = corpus.runs.begin();
        for (; runIt != corpus.runs.end(); ++runIt)
        {
            if (runIt->runId == options.runIdFilter)
            {
                break;
            }
        }
        if (runIt == corpus.runs.end())
        {
            error = "Requested feedforward sample export run was not replayed: " + options.runIdFilter;
            return false;
        }

        if (runIt->feedforwardSampleExportRows.empty())
        {
            error =
                "Requested feedforward sample export run did not produce any comparable transitions: " +
                options.runIdFilter;
            return false;
        }

        const std::filesystem::path parentPath = options.feedforwardSampleCsvPath.parent_path();
        if (!parentPath.empty())
        {
            std::error_code createEc;
            std::filesystem::create_directories(parentPath, createEc);
            if (createEc)
            {
                error =
                    "Failed to create feedforward sample CSV directory: " +
                    parentPath.string();
                return false;
            }
        }

        std::ofstream csv(options.feedforwardSampleCsvPath);
        if (!csv)
        {
            error =
                "Failed to open feedforward sample CSV for write: " +
                options.feedforwardSampleCsvPath.string();
            return false;
        }

        csv
            << "master_time_us,next_master_time_us,control_tick_sequence,next_control_tick_sequence,dt_us,"
            << "section_id,section_name,primitive_id,primitive_name,phase_id,phase_name,repeat_index,"
            << "path_id,path_label,path_category,"
            << "saturation_flags,current_forward_sensor_mps,current_left_velocity_mps,current_right_velocity_mps,"
            << "current_yaw_rate_sensor_radps,target_forward_sensor_mps,target_left_velocity_mps,"
            << "target_right_velocity_mps,target_yaw_rate_sensor_radps,nominal_left_command,nominal_right_command,"
            << "nominal_average_command,nominal_delta_command,logged_left_drive_command,logged_right_drive_command,"
            << "logged_average_drive_command,logged_delta_drive_command,logged_left_plant_command,"
            << "logged_right_plant_command,logged_average_plant_command,logged_delta_plant_command,"
            << "left_drive_command_error,right_drive_command_error,average_drive_command_error,delta_drive_command_error,"
            << "left_plant_command_error,right_plant_command_error,average_plant_command_error,"
            << "delta_plant_command_error,envelope_left_min,envelope_left_max,envelope_right_min,"
            << "envelope_right_max,logged_drive_within_envelope,logged_plant_command_within_envelope,"
            << "predicted_next_forward_mps,predicted_next_yaw_rate_radps,predicted_forward_target_error_mps,"
            << "predicted_yaw_target_error_radps\n";

        for (const FeedforwardSampleExportRow& row : runIt->feedforwardSampleExportRows)
        {
            csv
                << row.masterTimeUs << ','
                << row.nextMasterTimeUs << ','
                << row.controlTickSequence << ','
                << row.nextControlTickSequence << ','
                << row.dtUs << ','
                << static_cast<unsigned>(row.sectionId) << ','
                << row.sectionName << ','
                << static_cast<unsigned>(row.primitiveId) << ','
                << row.primitiveName << ','
                << static_cast<unsigned>(row.phaseId) << ','
                << row.phaseName << ','
                << row.repeatIndex << ','
                << row.pathId << ','
                << row.pathLabel << ','
                << row.pathCategory << ','
                << row.saturationFlags << ','
                << FormatDouble(row.currentForwardSensorMps) << ','
                << FormatDouble(row.currentLeftVelocityMps) << ','
                << FormatDouble(row.currentRightVelocityMps) << ','
                << FormatDouble(row.currentYawRateSensorRadps) << ','
                << FormatDouble(row.targetForwardSensorMps) << ','
                << FormatDouble(row.targetLeftVelocityMps) << ','
                << FormatDouble(row.targetRightVelocityMps) << ','
                << FormatDouble(row.targetYawRateSensorRadps) << ','
                << FormatDouble(row.nominalLeftCommand) << ','
                << FormatDouble(row.nominalRightCommand) << ','
                << FormatDouble(row.nominalAverageCommand) << ','
                << FormatDouble(row.nominalDeltaCommand) << ','
                << FormatDouble(row.loggedLeftDriveCommand) << ','
                << FormatDouble(row.loggedRightDriveCommand) << ','
                << FormatDouble(row.loggedAverageDriveCommand) << ','
                << FormatDouble(row.loggedDeltaDriveCommand) << ','
                << FormatDouble(row.loggedLeftPlantCommand) << ','
                << FormatDouble(row.loggedRightPlantCommand) << ','
                << FormatDouble(row.loggedAveragePlantCommand) << ','
                << FormatDouble(row.loggedDeltaPlantCommand) << ','
                << FormatDouble(row.leftDriveCommandError) << ','
                << FormatDouble(row.rightDriveCommandError) << ','
                << FormatDouble(row.averageDriveCommandError) << ','
                << FormatDouble(row.deltaDriveCommandError) << ','
                << FormatDouble(row.leftPlantCommandError) << ','
                << FormatDouble(row.rightPlantCommandError) << ','
                << FormatDouble(row.averagePlantCommandError) << ','
                << FormatDouble(row.deltaPlantCommandError) << ','
                << FormatDouble(row.envelopeLeftMin) << ','
                << FormatDouble(row.envelopeLeftMax) << ','
                << FormatDouble(row.envelopeRightMin) << ','
                << FormatDouble(row.envelopeRightMax) << ','
                << (row.loggedDriveWithinEnvelope ? "true" : "false") << ','
                << (row.loggedPlantCommandWithinEnvelope ? "true" : "false") << ','
                << FormatDouble(row.predictedNextForwardMps) << ','
                << FormatDouble(row.predictedNextYawRateRadps) << ','
                << FormatDouble(row.predictedForwardTargetErrorMps) << ','
                << FormatDouble(row.predictedYawTargetErrorRadps) << '\n';
        }

        std::cout
            << "Feedforward audit sample export written to "
            << options.feedforwardSampleCsvPath.string()
            << "\n";
        std::cout << "Feedforward path summary for run " << runIt->runId << ":\n";
        for (const FeedforwardPathSummary& summary : runIt->feedforwardPaths)
        {
            if (summary.validSolutions == 0U)
            {
                continue;
            }

            std::cout
                << "  " << summary.pathId
                << ": samples=" << summary.validSolutions
                << ", drive_average_rmse=" << FormatDouble(summary.metrics.averageDriveCommand.rmse(), 6)
                << ", drive_delta_rmse=" << FormatDouble(summary.metrics.deltaDriveCommand.rmse(), 6)
                << ", plant_command_average_rmse=" << FormatDouble(summary.metrics.averagePlantCommand.rmse(), 6)
                << ", plant_command_delta_rmse=" << FormatDouble(summary.metrics.deltaPlantCommand.rmse(), 6)
                << ", drive_envelope_hit_rate_pct=" << FormatDouble(100.0 * summary.metrics.driveEnvelopeHit.rate(), 2)
                << "\n";
        }
        return true;
    }

    static bool WriteReportFiles(const CorpusReport& corpus, const ReplayOptions& options, std::string& error)
    {
        std::filesystem::path outputPath = options.outputPath;
        if (outputPath.empty())
        {
            outputPath = options.rootPath / ("open_floor_ukf_replay_" + MakeTimestampString());
        }

        std::error_code createEc;
        std::filesystem::create_directories(outputPath, createEc);
        if (createEc)
        {
            error = "Failed to create output directory: " + outputPath.string();
            return false;
        }

        const std::filesystem::path markdownPath = outputPath / kReportFileName;
        const std::filesystem::path csvPath = outputPath / kRunSummaryFileName;
        const std::filesystem::path sectionPhaseCsvPath = outputPath / kSectionPhaseSummaryFileName;
        const std::filesystem::path aggregateJsonPath = outputPath / kAggregateMetricsFileName;
        const std::filesystem::path feedforwardPathCsvPath = outputPath / kFeedforwardPathSummaryFileName;

        std::ofstream markdown(markdownPath);
        if (!markdown)
        {
            error = "Failed to open report file for write: " + markdownPath.string();
            return false;
        }

        std::size_t completedRuns = 0U;
        std::size_t noRetainedRowsRuns = 0U;
        for (const RunReport& run : corpus.runs)
        {
            if (run.completed)
            {
                ++completedRuns;
            }
            else if (run.failureReason == "Not enough rows after final-section exclusion")
            {
                ++noRetainedRowsRuns;
            }
        }
        const std::size_t replayFaultRuns = corpus.runs.size() - completedRuns - noRetainedRowsRuns;
        const PhaseAssociationSummary encoderLinearPhaseAssociation = ComputePhaseAssociationSummary(
            corpus.sectionPhaseBuckets,
            "encoder_linear");
        const PhaseAssociationSummary rawGyroPhaseAssociation = ComputePhaseAssociationSummary(
            corpus.sectionPhaseBuckets,
            "raw_gyro");
        const PhaseAssociationSummary accelRightPhaseAssociation = ComputePhaseAssociationSummary(
            corpus.sectionPhaseBuckets,
            "accel_right");
        const PhaseAssociationSummary accelForwardPhaseAssociation = ComputePhaseAssociationSummary(
            corpus.sectionPhaseBuckets,
            "accel_forward");
        const PhaseAssociationSummary postPositionPhaseAssociation = ComputePhaseAssociationSummary(
            corpus.sectionPhaseBuckets,
            "post_position");
        const PhaseAssociationSummary postHeadingPhaseAssociation = ComputePhaseAssociationSummary(
            corpus.sectionPhaseBuckets,
            "post_heading");
        const std::uint64_t totalFeedforwardEvaluations = TotalFeedforwardValidSolutions(corpus.feedforwardPaths);

        markdown
            << "# Open-Floor UKF Replay Report\n\n"
            << "- Root scanned: `" << options.rootPath.string() << "`\n"
            << "- Candidate CSVs found: " << corpus.candidateCsvCount << "\n"
            << "- Unique runs replayed: " << corpus.runs.size() << "\n"
            << "- Runs completed without replay faults: " << completedRuns << "\n"
            << "- Runs excluded after final-section removal: " << noRetainedRowsRuns << "\n"
            << "- Runs with replay faults: " << replayFaultRuns << "\n"
            << "- Duplicate run IDs skipped: " << corpus.duplicates.size() << "\n"
            << "- Final-section policy: ignore the highest `section_id` present in each run.\n"
            << "- Replay seed policy: "
            << (options.useKnownStationarySeed ?
                "canonical open-floor marker `C` stationary state using `VehicleState` semantic defaults.\n" :
                "first kept logged state using `VehicleState` semantic defaults.\n")
            << "- Battery policy: captured `battery_voltage_start` is reported; replay dynamics use `Vehicle::GetBatteryVoltage()` because no canonical logged-voltage override owner is exposed.\n"
            << "- Canonical inertial input: corrected `gyro_radps`; raw IMU fields, `gyro_raw_radps`, and `gyro_bias_radps` are retained only for debug/report output.\n"
            << "- Canonical encoder input: raw total encoder counts plus `dt_us`; decoded encoder speed/velocity columns are not required for replay.\n"
            << "- Wall ADC policy: raw ambient/lit wall ADC columns are parsed when present, but wall updates are not injected until open-floor replay has a canonical maze/world-frame owner for those observations.\n"
            << "- Encoder prediction path: replay publishes the tick snapshot into `VehicleState` before prediction; `Estimator::predict` consumes valid encoder input through `PlantModel`.\n"
            << "- Runtime-context gap: captured command targets, saturation flags, and acceleration context are parsed but not injected because no public `Estimator`/`VehicleState` owner path replaces the removed direct Estimator context hook.\n"
            << "- Plant diagnostic gap: body-acceleration prediction metrics remain non-finite until a canonical bound-state IMU-acceleration diagnostic is exposed.\n"
            << "- Prediction metrics compare the pre-update UKF prediction against observable sensor-space signals.\n"
            << "- Post-update replay deltas compare the replayed UKF state against the logged UKF state from the capture; they are consistency checks, not external ground truth.\n"
            << "- Feedforward validation uses count-derived wheel-side velocities plus corrected `gyro_radps` as the current/target sensor state, the following row's `dt_us` as the response horizon, and never uses logged UKF state for that validation.\n"
            << "- Feedforward path coverage: " << corpus.feedforwardPaths.size() << " canonical PlantModel public feedforward path evaluated.\n"
            << "- Feedforward audit sensor bounds: each wheel-side velocity is treated as a `+/- 0.06 m/s` sensor and corrected yaw rate as a `+/- 0.03 rad/s` sensor when computing per-path command envelopes.\n"
            << "- Feedforward evaluations completed: " << totalFeedforwardEvaluations << "\n"
            << "- Phase analysis uses canonical `section_id` + `phase_id` buckets from the open-floor schema.\n"
            << "- `eta_squared_abs` is the fraction of absolute-error variance explained by those section-phase buckets.\n\n";

        markdown << "## Aggregate Prediction Error\n\n";
        markdown << "| Signal | Unit | Samples | RMSE | MAE | Bias | Max Abs |\n";
        markdown << "| --- | --- | ---: | ---: | ---: | ---: | ---: |\n";
        WriteMetricRow(markdown, "left_encoder_wheel_speed", "rad/s", corpus.prediction.leftEncoderWheelSpeedRadps);
        WriteMetricRow(markdown, "right_encoder_wheel_speed", "rad/s", corpus.prediction.rightEncoderWheelSpeedRadps);
        WriteMetricRow(markdown, "encoder_linear_speed", "m/s", corpus.prediction.encoderLinearSpeedMps);
        WriteMetricRow(markdown, "encoder_yaw_rate", "rad/s", corpus.prediction.encoderYawRateRadps);
        WriteMetricRow(markdown, "body_forward_speed", "m/s", corpus.prediction.bodyForwardSpeedMps);
        WriteMetricRow(markdown, "body_yaw_rate_vs_gyro", "rad/s", corpus.prediction.bodyYawRateRadps);
        WriteMetricRow(markdown, "raw_gyro", "rad/s", corpus.prediction.rawGyroRadps);
        WriteMetricRow(markdown, "accel_body_right", "m/s^2", corpus.prediction.accelBodyRightMps2);
        WriteMetricRow(markdown, "accel_body_forward", "m/s^2", corpus.prediction.accelBodyForwardMps2);
        WriteMetricRow(markdown, "planar_accel", "m/s^2", corpus.prediction.planarAccelMps2);

        markdown << "\n## Aggregate Post-Update Replay Delta\n\n";
        markdown << "| Signal | Unit | Samples | RMSE | MAE | Bias | Max Abs |\n";
        markdown << "| --- | --- | ---: | ---: | ---: | ---: | ---: |\n";
        WriteMetricRow(markdown, "position", "mm", corpus.consistency.positionMm);
        WriteMetricRow(markdown, "heading", "deg", corpus.consistency.headingDeg);
        WriteMetricRow(markdown, "forward_speed", "m/s", corpus.consistency.forwardSpeedMps);
        WriteMetricRow(markdown, "rightward_speed", "m/s", corpus.consistency.rightwardSpeedMps);
        WriteMetricRow(markdown, "yaw_rate", "rad/s", corpus.consistency.yawRateRadps);
        WriteMetricRow(markdown, "left_wheel_speed", "rad/s", corpus.consistency.leftWheelSpeedRadps);
        WriteMetricRow(markdown, "right_wheel_speed", "rad/s", corpus.consistency.rightWheelSpeedRadps);

        if (!corpus.feedforwardPaths.empty())
        {
            markdown << "\n## Feedforward Path Audit\n\n";
            markdown << "| path_id | category | samples | drive_avg_rmse | drive_delta_rmse | ff_avg_rmse | ff_delta_rmse | drive_envelope_hit_pct | label |\n";
            markdown << "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |\n";
            for (const FeedforwardPathSummary& summary : corpus.feedforwardPaths)
            {
                markdown
                    << "| " << summary.pathId
                    << " | " << summary.category
                    << " | " << summary.validSolutions
                    << " | "
                    << (summary.validSolutions > 0U ? FormatDouble(summary.metrics.averageDriveCommand.rmse()) : "")
                    << " | "
                    << (summary.validSolutions > 0U ? FormatDouble(summary.metrics.deltaDriveCommand.rmse()) : "")
                    << " | "
                    << (summary.validSolutions > 0U ? FormatDouble(summary.metrics.averagePlantCommand.rmse()) : "")
                    << " | "
                    << (summary.validSolutions > 0U ? FormatDouble(summary.metrics.deltaPlantCommand.rmse()) : "")
                    << " | "
                    << (summary.validSolutions > 0U ? FormatDouble(100.0 * summary.metrics.driveEnvelopeHit.rate(), 2) : "")
                    << " | "
                    << summary.label
                    << " |\n";
            }
        }

        markdown << "\n## Section-Phase Error Association\n\n";
        markdown << "| Metric | Samples | Buckets | eta_squared_abs | Worst Bucket | Worst Bucket Samples | Worst Bucket MAE | Global MAE | Ratio |\n";
        markdown << "| --- | ---: | ---: | ---: | --- | ---: | ---: | ---: | ---: |\n";
        WritePhaseAssociationRow(markdown, "encoder_linear_speed", encoderLinearPhaseAssociation);
        WritePhaseAssociationRow(markdown, "raw_gyro", rawGyroPhaseAssociation);
        WritePhaseAssociationRow(markdown, "accel_body_right", accelRightPhaseAssociation);
        WritePhaseAssociationRow(markdown, "accel_body_forward", accelForwardPhaseAssociation);
        WritePhaseAssociationRow(markdown, "post_position", postPositionPhaseAssociation);
        WritePhaseAssociationRow(markdown, "post_heading", postHeadingPhaseAssociation);

        markdown << "\n## Section-Phase Breakdown\n\n";
        markdown
            << "| section | phase | samples | encoder_linear_rmse | raw_gyro_rmse | accel_x_rmse | accel_y_rmse | post_position_rmse_mm | post_heading_rmse_deg |\n"
            << "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |\n";
        for (const SectionPhaseReport& bucket : corpus.sectionPhaseBuckets)
        {
            markdown
                << "| " << bucket.sectionName
                << " | " << bucket.phaseName
                << " | " << bucket.sampleCount
                << " | " << FormatDouble(bucket.prediction.encoderLinearSpeedMps.rmse())
                << " | " << FormatDouble(bucket.prediction.rawGyroRadps.rmse())
                << " | " << FormatDouble(bucket.prediction.accelBodyRightMps2.rmse())
                << " | " << FormatDouble(bucket.prediction.accelBodyForwardMps2.rmse())
                << " | " << FormatDouble(bucket.consistency.positionMm.rmse(), 3)
                << " | " << FormatDouble(bucket.consistency.headingDeg.rmse(), 3)
                << " |\n";
        }

        markdown << "\n## Per-Run Summary\n\n";
        markdown
            << "| run_id | format | rows | kept | ignored_section | predict_failures | yaw_rejects | ff_transitions | ff_evaluations | encoder_linear_rmse | raw_gyro_rmse | accel_x_rmse | accel_y_rmse | post_position_rmse_mm | post_heading_rmse_deg | battery_source |\n"
            << "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |\n";
        for (const RunReport& run : corpus.runs)
        {
            markdown
                << "| " << run.runId
                << " | " << run.formatVersion
                << " | " << run.totalRows
                << " | " << run.keptRows
                << " | " << run.ignoredSectionId
                << " | " << run.predictFailures
                << " | " << run.yawRejects
                << " | " << run.feedforwardTransitions
                << " | " << TotalFeedforwardValidSolutions(run.feedforwardPaths)
                << " | " << FormatDouble(run.prediction.encoderLinearSpeedMps.rmse())
                << " | " << FormatDouble(run.prediction.rawGyroRadps.rmse())
                << " | " << FormatDouble(run.prediction.accelBodyRightMps2.rmse())
                << " | " << FormatDouble(run.prediction.accelBodyForwardMps2.rmse())
                << " | " << FormatDouble(run.consistency.positionMm.rmse(), 3)
                << " | " << FormatDouble(run.consistency.headingDeg.rmse(), 3)
                << " | " << run.batterySource
                << " |\n";
            if (!run.completed)
            {
                markdown << "\n  Failure: `" << run.failureReason << "`\n\n";
            }
        }

        if (!corpus.duplicates.empty())
        {
            markdown << "\n## Skipped Duplicate Run IDs\n\n";
            for (const DuplicateRunInfo& duplicate : corpus.duplicates)
            {
                markdown
                    << "- `" << duplicate.runId
                    << "` kept `" << duplicate.keptPath.string()
                    << "`, skipped `" << duplicate.skippedPath.string()
                    << "`\n";
            }
        }

        std::ofstream csv(csvPath);
        if (!csv)
        {
            error = "Failed to open CSV summary file for write: " + csvPath.string();
            return false;
        }

        csv
            << "run_id,format_version,csv_path,battery_source,battery_voltage_v,total_rows,kept_rows,scored_transitions,feedforward_transitions,feedforward_evaluations,ignored_section_id,predict_failures,yaw_rejects,completed,encoder_linear_rmse_mps,raw_gyro_rmse_radps,accel_body_right_rmse_mps2,accel_body_forward_rmse_mps2,post_position_rmse_mm,post_heading_rmse_deg,failure_reason\n";
        for (const RunReport& run : corpus.runs)
        {
            csv
                << run.runId << ','
                << run.formatVersion << ','
                << '"' << run.csvPath.string() << '"' << ','
                << run.batterySource << ','
                << FormatDouble(run.batteryVoltageV) << ','
                << run.totalRows << ','
                << run.keptRows << ','
                << run.scoredTransitions << ','
                << run.feedforwardTransitions << ','
                << TotalFeedforwardValidSolutions(run.feedforwardPaths) << ','
                << run.ignoredSectionId << ','
                << run.predictFailures << ','
                << run.yawRejects << ','
                << (run.completed ? "true" : "false") << ','
                << FormatDouble(run.prediction.encoderLinearSpeedMps.rmse()) << ','
                << FormatDouble(run.prediction.rawGyroRadps.rmse()) << ','
                << FormatDouble(run.prediction.accelBodyRightMps2.rmse()) << ','
                << FormatDouble(run.prediction.accelBodyForwardMps2.rmse()) << ','
                << FormatDouble(run.consistency.positionMm.rmse(), 3) << ','
                << FormatDouble(run.consistency.headingDeg.rmse(), 3) << ','
                << '"' << run.failureReason << '"' << '\n';
        }

        std::ofstream feedforwardPathCsv(feedforwardPathCsvPath);
        if (!feedforwardPathCsv)
        {
            error = "Failed to open feedforward path summary CSV for write: " + feedforwardPathCsvPath.string();
            return false;
        }

        feedforwardPathCsv
            << "path_id,label,category,comparable_transitions,valid_solutions,drive_average_rmse,drive_delta_rmse,plant_command_average_rmse,plant_command_delta_rmse,drive_envelope_hit_pct,plant_command_envelope_hit_pct,forward_target_rmse_mps,yaw_target_rmse_radps\n";
        for (const FeedforwardPathSummary& summary : corpus.feedforwardPaths)
        {
            feedforwardPathCsv
                << summary.pathId << ','
                << '"' << summary.label << '"' << ','
                << summary.category << ','
                << summary.comparableTransitions << ','
                << summary.validSolutions << ','
                << FormatDouble(summary.metrics.averageDriveCommand.rmse()) << ','
                << FormatDouble(summary.metrics.deltaDriveCommand.rmse()) << ','
                << FormatDouble(summary.metrics.averagePlantCommand.rmse()) << ','
                << FormatDouble(summary.metrics.deltaPlantCommand.rmse()) << ','
                << FormatDouble(100.0 * summary.metrics.driveEnvelopeHit.rate(), 2) << ','
                << FormatDouble(100.0 * summary.metrics.plantCommandEnvelopeHit.rate(), 2) << ','
                << FormatDouble(summary.metrics.predictedForwardTargetErrorMps.rmse()) << ','
                << FormatDouble(summary.metrics.predictedYawTargetErrorRadps.rmse()) << '\n';
        }

        std::ofstream sectionPhaseCsv(sectionPhaseCsvPath);
        if (!sectionPhaseCsv)
        {
            error = "Failed to open section-phase CSV summary file for write: " + sectionPhaseCsvPath.string();
            return false;
        }

        sectionPhaseCsv
            << "section_id,section_name,phase_id,phase_name,bucket_samples,encoder_linear_samples,raw_gyro_samples,accel_body_right_samples,accel_body_forward_samples,post_position_samples,post_heading_samples,encoder_linear_rmse_mps,encoder_linear_mae_mps,raw_gyro_rmse_radps,raw_gyro_mae_radps,accel_body_right_rmse_mps2,accel_body_right_mae_mps2,accel_body_forward_rmse_mps2,accel_body_forward_mae_mps2,post_position_rmse_mm,post_position_mae_mm,post_heading_rmse_deg,post_heading_mae_deg\n";
        for (const SectionPhaseReport& bucket : corpus.sectionPhaseBuckets)
        {
            sectionPhaseCsv
                << static_cast<unsigned>(bucket.sectionId) << ','
                << bucket.sectionName << ','
                << static_cast<unsigned>(bucket.phaseId) << ','
                << bucket.phaseName << ','
                << bucket.sampleCount << ','
                << bucket.prediction.encoderLinearSpeedMps.count << ','
                << bucket.prediction.rawGyroRadps.count << ','
                << bucket.prediction.accelBodyRightMps2.count << ','
                << bucket.prediction.accelBodyForwardMps2.count << ','
                << bucket.consistency.positionMm.count << ','
                << bucket.consistency.headingDeg.count << ','
                << FormatDouble(bucket.prediction.encoderLinearSpeedMps.rmse()) << ','
                << FormatDouble(bucket.prediction.encoderLinearSpeedMps.mae()) << ','
                << FormatDouble(bucket.prediction.rawGyroRadps.rmse()) << ','
                << FormatDouble(bucket.prediction.rawGyroRadps.mae()) << ','
                << FormatDouble(bucket.prediction.accelBodyRightMps2.rmse()) << ','
                << FormatDouble(bucket.prediction.accelBodyRightMps2.mae()) << ','
                << FormatDouble(bucket.prediction.accelBodyForwardMps2.rmse()) << ','
                << FormatDouble(bucket.prediction.accelBodyForwardMps2.mae()) << ','
                << FormatDouble(bucket.consistency.positionMm.rmse(), 3) << ','
                << FormatDouble(bucket.consistency.positionMm.mae(), 3) << ','
                << FormatDouble(bucket.consistency.headingDeg.rmse(), 3) << ','
                << FormatDouble(bucket.consistency.headingDeg.mae(), 3) << '\n';
        }

        std::ofstream aggregateJson(aggregateJsonPath);
        if (!aggregateJson)
        {
            error = "Failed to open aggregate metrics JSON file for write: " + aggregateJsonPath.string();
            return false;
        }

        aggregateJson
            << "{\n"
            << "  \"candidate_csv_count\": " << corpus.candidateCsvCount << ",\n"
            << "  \"unique_runs\": " << corpus.runs.size() << ",\n"
            << "  \"completed_runs\": " << completedRuns << ",\n"
            << "  \"excluded_runs\": " << noRetainedRowsRuns << ",\n"
            << "  \"replay_fault_runs\": " << replayFaultRuns << ",\n"
            << "  \"duplicate_run_ids\": " << corpus.duplicates.size() << ",\n"
            << "  \"seed_policy\": \""
            << (options.useKnownStationarySeed ? "known_stationary_marker_c" : "logged_first_state")
            << "\",\n"
            << "  \"prediction\": {\n"
            << "    \"encoder_linear_rmse_mps\": " << FormatDouble(corpus.prediction.encoderLinearSpeedMps.rmse(), 12) << ",\n"
            << "    \"raw_gyro_rmse_radps\": " << FormatDouble(corpus.prediction.rawGyroRadps.rmse(), 12) << ",\n"
            << "    \"accel_body_right_rmse_mps2\": " << FormatDouble(corpus.prediction.accelBodyRightMps2.rmse(), 12) << ",\n"
            << "    \"accel_body_forward_rmse_mps2\": " << FormatDouble(corpus.prediction.accelBodyForwardMps2.rmse(), 12) << ",\n"
            << "    \"encoder_yaw_rate_rmse_radps\": " << FormatDouble(corpus.prediction.encoderYawRateRadps.rmse(), 12) << ",\n"
            << "    \"body_forward_speed_rmse_mps\": " << FormatDouble(corpus.prediction.bodyForwardSpeedMps.rmse(), 12) << "\n"
            << "  },\n"
            << "  \"consistency\": {\n"
            << "    \"post_position_rmse_mm\": " << FormatDouble(corpus.consistency.positionMm.rmse(), 12) << ",\n"
            << "    \"post_heading_rmse_deg\": " << FormatDouble(corpus.consistency.headingDeg.rmse(), 12) << "\n"
            << "  },\n"
            << "  \"feedforward_paths\": {\n";
        for (std::size_t pathIndex = 0; pathIndex < corpus.feedforwardPaths.size(); ++pathIndex)
        {
            const FeedforwardPathSummary& summary = corpus.feedforwardPaths[pathIndex];
            aggregateJson
                << "    \"" << summary.pathId << "\": {\n"
                << "      \"label\": \"" << summary.label << "\",\n"
                << "      \"category\": \"" << summary.category << "\",\n"
                << "      \"comparable_transitions\": " << summary.comparableTransitions << ",\n"
                << "      \"valid_solutions\": " << summary.validSolutions << ",\n"
                << "      \"drive_average_rmse\": " << FormatDouble(summary.metrics.averageDriveCommand.rmse(), 12) << ",\n"
                << "      \"drive_delta_rmse\": " << FormatDouble(summary.metrics.deltaDriveCommand.rmse(), 12) << ",\n"
                << "      \"plant_command_average_rmse\": " << FormatDouble(summary.metrics.averagePlantCommand.rmse(), 12) << ",\n"
                << "      \"plant_command_delta_rmse\": " << FormatDouble(summary.metrics.deltaPlantCommand.rmse(), 12) << ",\n"
                << "      \"drive_envelope_hit_pct\": " << FormatDouble(100.0 * summary.metrics.driveEnvelopeHit.rate(), 12) << ",\n"
                << "      \"plant_command_envelope_hit_pct\": " << FormatDouble(100.0 * summary.metrics.plantCommandEnvelopeHit.rate(), 12) << ",\n"
                << "      \"forward_target_rmse_mps\": " << FormatDouble(summary.metrics.predictedForwardTargetErrorMps.rmse(), 12) << ",\n"
                << "      \"yaw_target_rmse_radps\": " << FormatDouble(summary.metrics.predictedYawTargetErrorRadps.rmse(), 12) << "\n"
                << "    }" << ((pathIndex + 1U) < corpus.feedforwardPaths.size() ? "," : "") << "\n";
        }
        aggregateJson
            << "  }\n"
            << "}\n";

        std::cout
            << "Report written to " << markdownPath.string() << "\n"
            << "Run summary written to " << csvPath.string() << "\n"
            << "Feedforward path summary written to " << feedforwardPathCsvPath.string() << "\n"
            << "Section-phase summary written to " << sectionPhaseCsvPath.string() << "\n"
            << "Aggregate metrics written to " << aggregateJsonPath.string() << "\n";
        return true;
    }

int main(int argc, char* argv[])
{
    ReplayOptions options{};
    std::string error;
    if (!ParseArgs(argc, argv, options, error))
    {
        std::cerr << error << "\n";
        return 1;
    }

    std::vector<RunCandidate> runs;
    CorpusReport corpus{};
    corpus.feedforwardPaths = BuildDefaultFeedforwardPathSummaries();
    if (!DiscoverRuns(options, runs, corpus.duplicates, corpus.candidateCsvCount, error))
    {
        std::cerr << error << "\n";
        return 1;
    }

    std::unordered_map<SectionPhaseKey, SectionPhaseReport, SectionPhaseKeyHash> corpusSectionPhaseBuckets;
    for (const RunCandidate& run : runs)
    {
        RunReport report = ReplayRun(run, options);
        corpus.prediction.merge(report.prediction);
        corpus.consistency.merge(report.consistency);
        for (std::size_t pathIndex = 0; pathIndex < corpus.feedforwardPaths.size(); ++pathIndex)
        {
            corpus.feedforwardPaths[pathIndex].merge(report.feedforwardPaths[pathIndex]);
        }
        for (const SectionPhaseReport& bucket : report.sectionPhaseBuckets)
        {
            GetOrCreateSectionPhaseReport(corpusSectionPhaseBuckets, bucket.sectionId, bucket.phaseId).merge(bucket);
        }
        corpus.runs.push_back(std::move(report));
    }

    corpus.sectionPhaseBuckets = ToSortedSectionPhaseReports(corpusSectionPhaseBuckets);

    if (!WriteReportFiles(corpus, options, error))
    {
        std::cerr << error << "\n";
        return 1;
    }

    if (!WriteSampleExportCsv(corpus, options, error))
    {
        std::cerr << error << "\n";
        return 1;
    }

    if (!WriteFeedforwardSampleExportCsv(corpus, options, error))
    {
        std::cerr << error << "\n";
        return 1;
    }

    std::size_t completedRuns = 0U;
    for (const RunReport& report : corpus.runs)
    {
        if (report.completed)
        {
            ++completedRuns;
        }
    }
    std::cout
        << "Replayed " << corpus.runs.size() << " unique runs; "
        << completedRuns << " completed without replay faults.\n";
    return 0;
}
