#include "..\..\MazeMap\MazeMap\Estimator.h"
#include "..\..\MazeMap\MazeMap\MazeMapRuntimeCore.h"
#include "..\..\MazeMap\MazeMap\OpenFloorMeasurementSpec.h"
#include "..\..\MazeMap\MazeMap\PlantModel.h"

#include <array>
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

namespace fs = std::filesystem;

namespace
{
    constexpr const char* kCsvFileName = "open_floor_main.csv";
    constexpr const char* kSidecarFileName = "open_floor_main.sidecar";
    constexpr const char* kReportFileName = "report.md";
    constexpr const char* kRunSummaryFileName = "run_summary.csv";
    constexpr const char* kSectionPhaseSummaryFileName = "section_phase_summary.csv";
    constexpr const char* kAggregateMetricsFileName = "aggregate_metrics.json";
    constexpr const char* kFeedforwardPathSummaryFileName = "feedforward_path_summary.csv";
    constexpr float kRadiansToDegrees = 57.295779513082320876f;
    constexpr float kWheelVelocitySensorBoundMps = 0.06f;
    constexpr float kYawRateSensorBoundRadps = 0.03f;
    constexpr float kDefaultFeedforwardReserveUsage = 0.90f;

    struct ReplayOptions
    {
        fs::path rootPath;
        fs::path outputPath;
        fs::path tuningPath;
        std::string runIdFilter;
        fs::path sampleCsvPath;
        fs::path feedforwardSampleCsvPath;
        std::vector<std::string> sampleMetrics;
        bool useKnownStationarySeed = false;
    };

    struct SidecarInfo
    {
        fs::path path;
        std::unordered_map<std::string, std::string> metadata;
        std::vector<std::string> fieldNames;
        std::string runId;
        std::string formatVersion;
        std::string streamType;
        std::string controlLogFile;
    };

    struct RunCandidate
    {
        fs::path csvPath;
        SidecarInfo sidecar;
        fs::path controlLogPath;
        float batteryVoltageV = std::numeric_limits<float>::quiet_NaN();
        std::string batterySource = "plant_default";
    };

    struct DuplicateRunInfo
    {
        std::string runId;
        fs::path keptPath;
        fs::path skippedPath;
    };

    struct LoggedRow
    {
        std::uint32_t masterTimeUs = 0U;
        std::uint32_t controlTickSequence = 0U;
        std::uint32_t dtUs = 0U;
        std::uint8_t sectionId = 0U;
        std::uint8_t primitiveId = 0U;
        std::uint8_t phaseId = 0U;
        std::uint16_t repeatIndex = 0U;
        std::uint16_t saturationFlags = 0U;
        MazeMap::VehicleState::StateVector loggedState = MazeMap::VehicleState::StateVector::Zero();
        float measuredLinearSpeedMps = 0.0f;
        float measuredAngularSpeedRadps = 0.0f;
        float commandedLinearMps = 0.0f;
        float commandedAngularRadps = 0.0f;
        float leftDriveCommand = 0.0f;
        float rightDriveCommand = 0.0f;
        float leftFeedforwardCommand = 0.0f;
        float rightFeedforwardCommand = 0.0f;
        float leftLaunchAssistFloor = 0.0f;
        float rightLaunchAssistFloor = 0.0f;
        std::int32_t leftEncoderCount = 0;
        std::int32_t rightEncoderCount = 0;
        float leftEncoderOmegaRadps = 0.0f;
        float rightEncoderOmegaRadps = 0.0f;
        float leftEncoderVelocityMps = 0.0f;
        float rightEncoderVelocityMps = 0.0f;
        bool accelBiasValid = false;
        float gyroRawRadps = std::numeric_limits<float>::quiet_NaN();
        float gyroCorrectedRadps = std::numeric_limits<float>::quiet_NaN();
        float accelBodyXMps2 = std::numeric_limits<float>::quiet_NaN();
        float accelBodyYMps2 = std::numeric_limits<float>::quiet_NaN();
        float planarAccelMps2 = std::numeric_limits<float>::quiet_NaN();
        float fanDutyCycle = 0.0f;
    };

    struct SampleExportRow
    {
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
        float predictedAccelBodyXMps2 = std::numeric_limits<float>::quiet_NaN();
        float actualAccelBodyXMps2 = std::numeric_limits<float>::quiet_NaN();
        float predictedAccelBodyYMps2 = std::numeric_limits<float>::quiet_NaN();
        float actualAccelBodyYMps2 = std::numeric_limits<float>::quiet_NaN();
        float predictedPlanarAccelMps2 = std::numeric_limits<float>::quiet_NaN();
        float actualPlanarAccelMps2 = std::numeric_limits<float>::quiet_NaN();
        float predictedLinearSpeedMps = std::numeric_limits<float>::quiet_NaN();
        float actualLinearSpeedMps = std::numeric_limits<float>::quiet_NaN();
        float predictedYawRateRadps = std::numeric_limits<float>::quiet_NaN();
        float actualYawRateRadps = std::numeric_limits<float>::quiet_NaN();
        float predictedRawGyroRadps = std::numeric_limits<float>::quiet_NaN();
        float actualRawGyroRadps = std::numeric_limits<float>::quiet_NaN();
    };

    struct FeedforwardSampleExportRow
    {
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
        float loggedLeftFeedforwardCommand = std::numeric_limits<float>::quiet_NaN();
        float loggedRightFeedforwardCommand = std::numeric_limits<float>::quiet_NaN();
        float loggedAverageFeedforwardCommand = std::numeric_limits<float>::quiet_NaN();
        float loggedDeltaFeedforwardCommand = std::numeric_limits<float>::quiet_NaN();
        float leftDriveCommandError = std::numeric_limits<float>::quiet_NaN();
        float rightDriveCommandError = std::numeric_limits<float>::quiet_NaN();
        float averageDriveCommandError = std::numeric_limits<float>::quiet_NaN();
        float deltaDriveCommandError = std::numeric_limits<float>::quiet_NaN();
        float leftFeedforwardCommandError = std::numeric_limits<float>::quiet_NaN();
        float rightFeedforwardCommandError = std::numeric_limits<float>::quiet_NaN();
        float averageFeedforwardCommandError = std::numeric_limits<float>::quiet_NaN();
        float deltaFeedforwardCommandError = std::numeric_limits<float>::quiet_NaN();
        float envelopeLeftMin = std::numeric_limits<float>::quiet_NaN();
        float envelopeLeftMax = std::numeric_limits<float>::quiet_NaN();
        float envelopeRightMin = std::numeric_limits<float>::quiet_NaN();
        float envelopeRightMax = std::numeric_limits<float>::quiet_NaN();
        bool loggedDriveWithinEnvelope = false;
        bool loggedFeedforwardWithinEnvelope = false;
        float predictedNextForwardMps = std::numeric_limits<float>::quiet_NaN();
        float predictedNextYawRateRadps = std::numeric_limits<float>::quiet_NaN();
        float predictedForwardTargetErrorMps = std::numeric_limits<float>::quiet_NaN();
        float predictedYawTargetErrorRadps = std::numeric_limits<float>::quiet_NaN();
    };

    enum class FeedforwardPathId : std::size_t
    {
        Acceleration = 0U,
        SteadyState,
        Count
    };

    struct FeedforwardPathDefinition
    {
        FeedforwardPathId id = FeedforwardPathId::Acceleration;
        const char* pathId = "";
        const char* label = "";
        const char* category = "";
    };

    struct ErrorStats
    {
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

    struct HitRateStats
    {
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

    struct PredictionMetrics
    {
        ErrorStats encoderLeftOmegaRadps;
        ErrorStats encoderRightOmegaRadps;
        ErrorStats encoderLinearSpeedMps;
        ErrorStats encoderYawRateRadps;
        ErrorStats bodyForwardSpeedMps;
        ErrorStats bodyYawRateRadps;
        ErrorStats rawGyroRadps;
        ErrorStats accelBodyXMps2;
        ErrorStats accelBodyYMps2;
        ErrorStats planarAccelMps2;

        void merge(const PredictionMetrics& other) noexcept
        {
            encoderLeftOmegaRadps.merge(other.encoderLeftOmegaRadps);
            encoderRightOmegaRadps.merge(other.encoderRightOmegaRadps);
            encoderLinearSpeedMps.merge(other.encoderLinearSpeedMps);
            encoderYawRateRadps.merge(other.encoderYawRateRadps);
            bodyForwardSpeedMps.merge(other.bodyForwardSpeedMps);
            bodyYawRateRadps.merge(other.bodyYawRateRadps);
            rawGyroRadps.merge(other.rawGyroRadps);
            accelBodyXMps2.merge(other.accelBodyXMps2);
            accelBodyYMps2.merge(other.accelBodyYMps2);
            planarAccelMps2.merge(other.planarAccelMps2);
        }
    };

    struct FeedforwardMetrics
    {
        ErrorStats leftDriveCommand;
        ErrorStats rightDriveCommand;
        ErrorStats averageDriveCommand;
        ErrorStats deltaDriveCommand;
        ErrorStats leftFeedforwardCommand;
        ErrorStats rightFeedforwardCommand;
        ErrorStats averageFeedforwardCommand;
        ErrorStats deltaFeedforwardCommand;
        ErrorStats predictedForwardTargetErrorMps;
        ErrorStats predictedYawTargetErrorRadps;
        HitRateStats driveEnvelopeHit;
        HitRateStats feedforwardEnvelopeHit;

        void merge(const FeedforwardMetrics& other) noexcept
        {
            leftDriveCommand.merge(other.leftDriveCommand);
            rightDriveCommand.merge(other.rightDriveCommand);
            averageDriveCommand.merge(other.averageDriveCommand);
            deltaDriveCommand.merge(other.deltaDriveCommand);
            leftFeedforwardCommand.merge(other.leftFeedforwardCommand);
            rightFeedforwardCommand.merge(other.rightFeedforwardCommand);
            averageFeedforwardCommand.merge(other.averageFeedforwardCommand);
            deltaFeedforwardCommand.merge(other.deltaFeedforwardCommand);
            predictedForwardTargetErrorMps.merge(other.predictedForwardTargetErrorMps);
            predictedYawTargetErrorRadps.merge(other.predictedYawTargetErrorRadps);
            driveEnvelopeHit.merge(other.driveEnvelopeHit);
            feedforwardEnvelopeHit.merge(other.feedforwardEnvelopeHit);
        }
    };

    struct FeedforwardEnvelope
    {
        float leftMin = (std::numeric_limits<float>::infinity)();
        float leftMax = -(std::numeric_limits<float>::infinity)();
        float rightMin = (std::numeric_limits<float>::infinity)();
        float rightMax = -(std::numeric_limits<float>::infinity)();
        bool valid = false;
    };

    struct FeedforwardPathSummary
    {
        FeedforwardPathId id = FeedforwardPathId::Acceleration;
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

    struct FeedforwardTransitionInputs
    {
        MazeMap::VehicleState::StateVector currentState = MazeMap::VehicleState::StateVector::Zero();
        float currentForwardSensorMps = std::numeric_limits<float>::quiet_NaN();
        float currentYawRateSensorRadps = std::numeric_limits<float>::quiet_NaN();
        float targetForwardSensorMps = std::numeric_limits<float>::quiet_NaN();
        float targetYawRateSensorRadps = std::numeric_limits<float>::quiet_NaN();
        float desiredStateLongitudinalAccelMps2 = std::numeric_limits<float>::quiet_NaN();
        float desiredStateYawAccelRadps2 = std::numeric_limits<float>::quiet_NaN();
        float desiredScalarLongitudinalAccelMps2 = std::numeric_limits<float>::quiet_NaN();
        float desiredScalarYawAccelRadps2 = std::numeric_limits<float>::quiet_NaN();
        float responseTimeS = std::numeric_limits<float>::quiet_NaN();
        float fanDutyCycle = 0.0f;
        float batteryVoltageV = 0.0f;
        float reserveUsage = kDefaultFeedforwardReserveUsage;
    };

    struct ConsistencyMetrics
    {
        ErrorStats positionMm;
        ErrorStats headingDeg;
        ErrorStats forwardSpeedMps;
        ErrorStats lateralSpeedMps;
        ErrorStats yawRateRadps;
        ErrorStats leftWheelOmegaRadps;
        ErrorStats rightWheelOmegaRadps;
        ErrorStats gyroBiasRadps;

        void merge(const ConsistencyMetrics& other) noexcept
        {
            positionMm.merge(other.positionMm);
            headingDeg.merge(other.headingDeg);
            forwardSpeedMps.merge(other.forwardSpeedMps);
            lateralSpeedMps.merge(other.lateralSpeedMps);
            yawRateRadps.merge(other.yawRateRadps);
            leftWheelOmegaRadps.merge(other.leftWheelOmegaRadps);
            rightWheelOmegaRadps.merge(other.rightWheelOmegaRadps);
            gyroBiasRadps.merge(other.gyroBiasRadps);
        }
    };

    struct SectionPhaseKey
    {
        std::uint8_t sectionId = 0U;
        std::uint8_t phaseId = 0U;

        bool operator==(const SectionPhaseKey& other) const noexcept
        {
            return (sectionId == other.sectionId) && (phaseId == other.phaseId);
        }
    };

    struct SectionPhaseKeyHash
    {
        std::size_t operator()(const SectionPhaseKey& key) const noexcept
        {
            return (static_cast<std::size_t>(key.sectionId) << 8U) ^
                static_cast<std::size_t>(key.phaseId);
        }
    };

    struct SectionPhaseReport
    {
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

    struct PhaseAssociationSummary
    {
        std::uint64_t totalSamples = 0U;
        std::size_t bucketCount = 0U;
        double etaSquaredAbs = 0.0;
        std::string worstBucketLabel;
        std::uint64_t worstBucketSamples = 0U;
        double worstBucketMae = 0.0;
        double globalMae = 0.0;
        double worstBucketMaeRatio = 0.0;
    };

    struct RunReport
    {
        std::string runId;
        std::string formatVersion;
        fs::path csvPath;
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

    struct CorpusReport
    {
        std::vector<RunReport> runs;
        std::vector<DuplicateRunInfo> duplicates;
        std::uint64_t candidateCsvCount = 0U;
        PredictionMetrics prediction{};
        ConsistencyMetrics consistency{};
        std::vector<FeedforwardPathSummary> feedforwardPaths;
        std::vector<SectionPhaseReport> sectionPhaseBuckets;
    };

    constexpr std::size_t kFeedforwardPathCount = static_cast<std::size_t>(FeedforwardPathId::Count);

    const std::array<FeedforwardPathDefinition, kFeedforwardPathCount>& GetFeedforwardPathDefinitions() noexcept
    {
        static const std::array<FeedforwardPathDefinition, kFeedforwardPathCount> definitions =
        {{
            { FeedforwardPathId::Acceleration, "acceleration", "PlantModel acceleration feedforward", "plant_model" },
            { FeedforwardPathId::SteadyState, "steady_state", "PlantModel steady-state feedforward", "plant_model" }
        }};
        return definitions;
    }

    std::vector<FeedforwardPathSummary> BuildDefaultFeedforwardPathSummaries()
    {
        std::vector<FeedforwardPathSummary> summaries;
        summaries.reserve(kFeedforwardPathCount);
        for (const FeedforwardPathDefinition& definition : GetFeedforwardPathDefinitions())
        {
            FeedforwardPathSummary summary{};
            summary.id = definition.id;
            summary.pathId = definition.pathId;
            summary.label = definition.label;
            summary.category = definition.category;
            summaries.push_back(std::move(summary));
        }
        return summaries;
    }

    std::string Trim(const std::string& value)
    {
        const std::size_t first = value.find_first_not_of(" \t\r\n");
        if (first == std::string::npos)
        {
            return std::string();
        }

        const std::size_t last = value.find_last_not_of(" \t\r\n");
        return value.substr(first, last - first + 1U);
    }

    std::string ToLower(std::string value)
    {
        std::transform(
            value.begin(),
            value.end(),
            value.begin(),
            [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        return value;
    }

    std::vector<std::string> SplitCommaSeparated(const std::string& line)
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

    bool ParseUnsigned32(const std::string& text, std::uint32_t& value) noexcept
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

    bool ParseUnsigned16(const std::string& text, std::uint16_t& value) noexcept
    {
        std::uint32_t parsed = 0U;
        if (!ParseUnsigned32(text, parsed) || (parsed > 0xFFFFU))
        {
            return false;
        }

        value = static_cast<std::uint16_t>(parsed);
        return true;
    }

    bool ParseUnsigned8(const std::string& text, std::uint8_t& value) noexcept
    {
        std::uint32_t parsed = 0U;
        if (!ParseUnsigned32(text, parsed) || (parsed > 0xFFU))
        {
            return false;
        }

        value = static_cast<std::uint8_t>(parsed);
        return true;
    }

    bool ParseInt32(const std::string& text, std::int32_t& value) noexcept
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

    bool ParseFloat(const std::string& text, float& value) noexcept
    {
        char* end = nullptr;
        value = std::strtof(text.c_str(), &end);
        return !(end == text.c_str() || *end != '\0');
    }

    bool ApplyTuningSetting(
        MazeMap::SrUkfCore::RuntimeTuning& tuning,
        const std::string& rawKey,
        float value,
        std::string& error)
    {
        const std::string key = ToLower(Trim(rawKey));
        if (key == "general_encoder_linear_speed_sigma_mps") tuning.generalEncoderLinearSpeedSigmaMps = value;
        else if (key == "general_encoder_yaw_rate_sigma_radps") tuning.generalEncoderYawRateSigmaRadps = value;
        else if (key == "stationary_encoder_velocity_sigma_mps") tuning.stationaryEncoderVelocitySigmaMps = value;
        else if (key == "encoder_pair_nis_threshold") tuning.encoderPairNisThreshold = value;
        else if (key == "imu_yaw_rate_sigma_radps") tuning.imuYawRateSigmaRadps = value;
        else if (key == "imu_accel_sigma_mps2") tuning.imuAccelSigmaMps2 = value;
        else if (key == "pivot_scrub_max_command_linear_mps") tuning.pivotScrubMaxCommandLinearMps = value;
        else if (key == "pivot_scrub_min_command_angular_radps") tuning.pivotScrubMinCommandAngularRadps = value;
        else if (key == "pivot_scrub_yaw_consistency_threshold_radps") tuning.pivotScrubYawConsistencyThresholdRadps = value;
        else if (key == "pivot_scrub_yaw_window_mismatch_threshold_rad") tuning.pivotScrubYawWindowMismatchThresholdRad = value;
        else if (key == "pivot_scrub_zero_u_sigma_mps") tuning.pivotScrubZeroUSigmaMps = value;
        else if (key == "stationary_gyro_bias_time_constant_s") { (void)value; }
        else if (key == "stationary_certification_dwell_s") tuning.stationaryCertificationDwellS = value;
        else if (key == "stationary_candidate_max_linear_command_mps") tuning.stationaryCandidateMaxLinearCommandMps = value;
        else if (key == "stationary_candidate_max_angular_command_radps") tuning.stationaryCandidateMaxAngularCommandRadps = value;
        else if (key == "stationary_candidate_max_drive_command") tuning.stationaryCandidateMaxDriveCommand = value;
        else if (key == "stationary_candidate_max_encoder_omega_radps") tuning.stationaryCandidateMaxEncoderOmegaRadps = value;
        else if (key == "stationary_candidate_max_corrected_gyro_radps") tuning.stationaryCandidateMaxCorrectedGyroRadps = value;
        else if (key == "stationary_candidate_max_accel_mps2") tuning.stationaryCandidateMaxAccelMps2 = value;
        else if (key == "command_sign_flip_window_s") tuning.commandSignFlipWindowS = value;
        else if (key == "stationary_exit_launch_window_s") tuning.stationaryExitLaunchWindowS = value;
        else if (key == "launch_hold_s") tuning.launchHoldS = value;
        else if (key == "launch_low_speed_threshold_mps") tuning.launchLowSpeedThresholdMps = value;
        else if (key == "launch_drive_command_delta_threshold") tuning.launchDriveCommandDeltaThreshold = value;
        else if (key == "inconsistent_hold_s") tuning.inconsistentHoldS = value;
        else if (key == "yaw_consistency_low_pass_tau_s") tuning.yawConsistencyLowPassTauS = value;
        else if (key == "yaw_consistency_low_pass_threshold_radps") tuning.yawConsistencyLowPassThresholdRadps = value;
        else if (key == "yaw_consistency_exceed_dwell_s") tuning.yawConsistencyExceedDwellS = value;
        else if (key == "yaw_window_duration_s") tuning.yawWindowDurationS = value;
        else if (key == "yaw_window_mismatch_threshold_rad") tuning.yawWindowMismatchThresholdRad = value;
        else if (key == "nhc_residual_trip_sigma") tuning.nhcResidualTripSigma = value;
        else if (key == "nhc_minimum_enable_forward_speed_mps") tuning.nhcMinimumEnableForwardSpeedMps = value;
        else if (key == "nhc_disable_forward_speed_mps") tuning.nhcDisableForwardSpeedMps = value;
        else if (key == "nhc_max_drive_command_delta") tuning.nhcMaxDriveCommandDelta = value;
        else if (key == "recovery_nhc_reenable_delay_s") tuning.recoveryNhcReenableDelayS = value;
        else if (key == "nhc_base_sigma_mps") tuning.nhcBaseSigmaMps = value;
        else if (key == "nhc_speed_slope_per_mps") tuning.nhcSpeedSlopePerMps = value;
        else if (key == "nhc_minimum_sigma_mps") tuning.nhcMinimumSigmaMps = value;
        else if (key == "nhc_maximum_sigma_mps") tuning.nhcMaximumSigmaMps = value;
        else if (key == "moving_gyro_bias_std_cap_radps") { (void)value; }
        else if (key == "recovery_yaw_rate_std_floor_radps") tuning.recoveryYawRateStdFloorRadps = value;
        else if (key == "yaw_validity_bias_delta_max_radps") tuning.yawValidityBiasDeltaMaxRadps = value;
        else if (key == "stationary_sigma_u_sqrt_q") tuning.stationaryCertifiedProcessNoise.sigmaUSqrtQ = value;
        else if (key == "stationary_sigma_v_sqrt_q") tuning.stationaryCertifiedProcessNoise.sigmaVSqrtQ = value;
        else if (key == "stationary_sigma_r_sqrt_q") tuning.stationaryCertifiedProcessNoise.sigmaRSqrtQ = value;
        else if (key == "stationary_sigma_omega_sqrt_q") tuning.stationaryCertifiedProcessNoise.sigmaOmegaSqrtQ = value;
        else if (key == "stationary_sigma_bgz_sqrt_q") { (void)value; }
        else if (key == "stationary_std_r_min") tuning.stationaryCertifiedProcessNoise.stdRMin = value;
        else if (key == "stationary_std_v_min") tuning.stationaryCertifiedProcessNoise.stdVMin = value;
        else if (key == "launch_sigma_u_sqrt_q") tuning.launchOrReversalProcessNoise.sigmaUSqrtQ = value;
        else if (key == "launch_sigma_v_sqrt_q") tuning.launchOrReversalProcessNoise.sigmaVSqrtQ = value;
        else if (key == "launch_sigma_r_sqrt_q") tuning.launchOrReversalProcessNoise.sigmaRSqrtQ = value;
        else if (key == "launch_sigma_omega_sqrt_q") tuning.launchOrReversalProcessNoise.sigmaOmegaSqrtQ = value;
        else if (key == "launch_sigma_bgz_sqrt_q") { (void)value; }
        else if (key == "launch_std_r_min") tuning.launchOrReversalProcessNoise.stdRMin = value;
        else if (key == "launch_std_v_min") tuning.launchOrReversalProcessNoise.stdVMin = value;
        else if (key == "grip_sigma_u_sqrt_q") tuning.gripLinearProcessNoise.sigmaUSqrtQ = value;
        else if (key == "grip_sigma_v_sqrt_q") tuning.gripLinearProcessNoise.sigmaVSqrtQ = value;
        else if (key == "grip_sigma_r_sqrt_q") tuning.gripLinearProcessNoise.sigmaRSqrtQ = value;
        else if (key == "grip_sigma_omega_sqrt_q") tuning.gripLinearProcessNoise.sigmaOmegaSqrtQ = value;
        else if (key == "grip_sigma_bgz_sqrt_q") { (void)value; }
        else if (key == "grip_std_r_min") tuning.gripLinearProcessNoise.stdRMin = value;
        else if (key == "grip_std_v_min") tuning.gripLinearProcessNoise.stdVMin = value;
        else if (key == "inconsistent_sigma_u_sqrt_q") tuning.inconsistentOrSaturatedProcessNoise.sigmaUSqrtQ = value;
        else if (key == "inconsistent_sigma_v_sqrt_q") tuning.inconsistentOrSaturatedProcessNoise.sigmaVSqrtQ = value;
        else if (key == "inconsistent_sigma_r_sqrt_q") tuning.inconsistentOrSaturatedProcessNoise.sigmaRSqrtQ = value;
        else if (key == "inconsistent_sigma_omega_sqrt_q") tuning.inconsistentOrSaturatedProcessNoise.sigmaOmegaSqrtQ = value;
        else if (key == "inconsistent_sigma_bgz_sqrt_q") { (void)value; }
        else if (key == "inconsistent_std_r_min") tuning.inconsistentOrSaturatedProcessNoise.stdRMin = value;
        else if (key == "inconsistent_std_v_min") tuning.inconsistentOrSaturatedProcessNoise.stdVMin = value;
        else
        {
            error = "Unknown tuning key: " + rawKey;
            return false;
        }

        return true;
    }

    bool LoadTuningFile(const fs::path& path, std::string& error)
    {
        if (path.empty())
        {
            MazeMap::SrUkfCore::ResetRuntimeTuning();
            return true;
        }

        std::ifstream input(path);
        if (!input)
        {
            error = "Failed to open tuning file: " + path.string();
            return false;
        }

        MazeMap::SrUkfCore::RuntimeTuning tuning = MazeMap::SrUkfCore::BuildDefaultRuntimeTuning();
        std::string line;
        std::size_t lineNumber = 0U;
        while (std::getline(input, line))
        {
            ++lineNumber;
            const std::size_t commentStart = line.find('#');
            if (commentStart != std::string::npos)
            {
                line = line.substr(0, commentStart);
            }

            line = Trim(line);
            if (line.empty())
            {
                continue;
            }

            const std::size_t equals = line.find('=');
            if (equals == std::string::npos)
            {
                error = "Malformed tuning line " + std::to_string(lineNumber) + ": " + line;
                return false;
            }

            const std::string key = Trim(line.substr(0, equals));
            const std::string valueText = Trim(line.substr(equals + 1U));
            float value = 0.0f;
            if (!ParseFloat(valueText, value))
            {
                error = "Invalid float at tuning line " + std::to_string(lineNumber) + ": " + valueText;
                return false;
            }

            if (!ApplyTuningSetting(tuning, key, value, error))
            {
                error += " at line " + std::to_string(lineNumber);
                return false;
            }
        }

        MazeMap::SrUkfCore::SetRuntimeTuning(tuning);
        return true;
    }

    MazeMap::SrUkfCore::StateVector BuildKnownStationaryOpenFloorInitialState() noexcept
    {
        MazeMap::SrUkfCore::StateVector state = MazeMap::SrUkfCore::StateVector::Zero();
        MazeMap::VehicleState::NormalizeStateVector(state);
        return state;
    }

    bool GetToken(
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
    bool ParseField(
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

    bool ParseSidecar(const fs::path& path, SidecarInfo& info, std::string& error)
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

    fs::path ResolveControlLogPath(const RunCandidate& candidate)
    {
        if (!candidate.sidecar.controlLogFile.empty())
        {
            const fs::path candidatePath = candidate.sidecar.path.parent_path() / candidate.sidecar.controlLogFile;
            if (fs::exists(candidatePath))
            {
                return candidatePath;
            }
        }

        const fs::path loggingPath = candidate.csvPath.parent_path() / "logging.txt";
        if (fs::exists(loggingPath))
        {
            return loggingPath;
        }

        return fs::path();
    }

    bool ReadBatteryVoltageStart(const fs::path& loggingPath, float& batteryVoltageV)
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

    bool PathLooksPrimaryDecode(const fs::path& path)
    {
        const std::string lower = ToLower(path.string());
        return lower.find("mmlog_decode_") != std::string::npos;
    }

    bool PreferCandidate(const RunCandidate& lhs, const RunCandidate& rhs)
    {
        const bool lhsPrimary = PathLooksPrimaryDecode(lhs.csvPath);
        const bool rhsPrimary = PathLooksPrimaryDecode(rhs.csvPath);
        if (lhsPrimary != rhsPrimary)
        {
            return lhsPrimary;
        }

        std::error_code lhsEc;
        std::error_code rhsEc;
        const fs::file_time_type lhsTime = fs::last_write_time(lhs.csvPath, lhsEc);
        const fs::file_time_type rhsTime = fs::last_write_time(rhs.csvPath, rhsEc);
        if (!lhsEc && !rhsEc && lhsTime != rhsTime)
        {
            return lhsTime > rhsTime;
        }

        return lhs.csvPath.string() < rhs.csvPath.string();
    }

    bool ResolveSampleMetricSelection(
        const std::string& text,
        std::vector<std::string>& metrics,
        std::string& error);

    bool ParseArgs(int argc, char* argv[], ReplayOptions& options, std::string& error)
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

                options.rootPath = fs::path(argv[++index]);
            }
            else if (argument == "--output")
            {
                if ((index + 1) >= argc)
                {
                    error = "--output requires a path";
                    return false;
                }

                options.outputPath = fs::path(argv[++index]);
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

                options.sampleCsvPath = fs::path(argv[++index]);
            }
            else if (argument == "--feedforward-sample-csv")
            {
                if ((index + 1) >= argc)
                {
                    error = "--feedforward-sample-csv requires a path";
                    return false;
                }

                options.feedforwardSampleCsvPath = fs::path(argv[++index]);
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
                if ((index + 1) >= argc)
                {
                    error = "--tuning requires a path";
                    return false;
                }

                options.tuningPath = fs::path(argv[++index]);
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
                    << "  --tuning <path>  Optional key=value tuning override file.\n"
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
            const fs::path cwd = fs::current_path();
            options.rootPath = fs::exists(cwd / "TestResults") ? (cwd / "TestResults") : cwd;
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

    std::string MakeTimestampString()
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

    std::string SectionName(std::uint8_t sectionId)
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

    std::string PhaseName(std::uint8_t phaseId)
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

    std::string PrimitiveName(std::uint8_t primitiveId)
    {
        if (primitiveId == static_cast<std::uint8_t>(MazeMap::MC_NONE))
        {
            return "NONE";
        }

        char codeName[24] = {};
        FormatManeuverCodeName(static_cast<MazeMap::ManeuverCode>(primitiveId), codeName, sizeof(codeName));
        return codeName;
    }

    const std::vector<std::string>& DefaultSampleMetricSelection()
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
            "predicted_accel_body_x_mps2",
            "actual_accel_body_x_mps2",
            "predicted_accel_body_y_mps2",
            "actual_accel_body_y_mps2",
            "predicted_planar_accel_mps2",
            "actual_planar_accel_mps2",
        };
        return kDefaultMetrics;
    }

    bool IsRecognizedSampleMetric(const std::string& metric) noexcept
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
            "predicted_accel_body_x_mps2",
            "actual_accel_body_x_mps2",
            "predicted_accel_body_y_mps2",
            "actual_accel_body_y_mps2",
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

    bool AppendSampleMetricSelection(
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
                "predicted_accel_body_x_mps2",
                "actual_accel_body_x_mps2",
                "predicted_accel_body_y_mps2",
                "actual_accel_body_y_mps2",
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

    bool ResolveSampleMetricSelection(
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

    std::string SectionPhaseLabel(const SectionPhaseReport& bucket)
    {
        return bucket.sectionName + " / " + bucket.phaseName;
    }

    using SectionPhaseReportMap = std::unordered_map<SectionPhaseKey, SectionPhaseReport, SectionPhaseKeyHash>;

    SectionPhaseReport& GetOrCreateSectionPhaseReport(
        SectionPhaseReportMap& reports,
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

    std::vector<SectionPhaseReport> ToSortedSectionPhaseReports(const SectionPhaseReportMap& reports)
    {
        std::vector<SectionPhaseReport> sorted;
        sorted.reserve(reports.size());
        for (const auto& item : reports)
        {
            sorted.push_back(item.second);
        }

        std::sort(
            sorted.begin(),
            sorted.end(),
            [](const SectionPhaseReport& lhs, const SectionPhaseReport& rhs)
            {
                if (lhs.sectionId != rhs.sectionId)
                {
                    return lhs.sectionId < rhs.sectionId;
                }

                return lhs.phaseId < rhs.phaseId;
            });
        return sorted;
    }

    template <typename Accessor>
    PhaseAssociationSummary ComputePhaseAssociationSummary(
        const std::vector<SectionPhaseReport>& buckets,
        Accessor&& accessor)
    {
        PhaseAssociationSummary summary{};
        double totalSumAbs = 0.0;
        double totalSumSquares = 0.0;
        for (const SectionPhaseReport& bucket : buckets)
        {
            const ErrorStats& stats = accessor(bucket);
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
            const ErrorStats& stats = accessor(bucket);
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

    bool DiscoverRuns(
        const ReplayOptions& options,
        std::vector<RunCandidate>& runs,
        std::vector<DuplicateRunInfo>& duplicates,
        std::uint64_t& candidateCsvCount,
        std::string& error)
    {
        if (!fs::exists(options.rootPath))
        {
            error = "Replay root does not exist: " + options.rootPath.string();
            return false;
        }

        std::unordered_map<std::string, RunCandidate> selectedRuns;
        candidateCsvCount = 0U;

        const fs::directory_options iteratorOptions = fs::directory_options::skip_permission_denied;
        for (const fs::directory_entry& entry : fs::recursive_directory_iterator(options.rootPath, iteratorOptions))
        {
            if (!entry.is_regular_file() || entry.path().filename() != kCsvFileName)
            {
                continue;
            }

            ++candidateCsvCount;

            const fs::path sidecarPath = entry.path().parent_path() / kSidecarFileName;
            if (!fs::exists(sidecarPath))
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

        std::sort(
            runs.begin(),
            runs.end(),
            [](const RunCandidate& lhs, const RunCandidate& rhs)
            {
                return lhs.sidecar.runId < rhs.sidecar.runId;
            });
        return true;
    }

    bool ValidateCsvHeader(
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

    bool LoadRows(const RunCandidate& candidate, std::vector<LoggedRow>& rows, std::string& error)
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
            if (!ParseField(tokens, indices, "master_time_us", row.masterTimeUs, ParseUnsigned32, error) ||
                !ParseField(tokens, indices, "control_tick_sequence", row.controlTickSequence, ParseUnsigned32, error) ||
                !ParseField(tokens, indices, "dt_us", row.dtUs, ParseUnsigned32, error) ||
                !ParseField(tokens, indices, "section_id", row.sectionId, ParseUnsigned8, error) ||
                !ParseField(tokens, indices, "primitive_id", row.primitiveId, ParseUnsigned8, error) ||
                !ParseField(tokens, indices, "phase_id", row.phaseId, ParseUnsigned8, error) ||
                !ParseField(tokens, indices, "repeat_index", row.repeatIndex, ParseUnsigned16, error) ||
                !ParseField(tokens, indices, "saturation_flags", row.saturationFlags, ParseUnsigned16, error) ||
                !ParseField(tokens, indices, "ukf_state_px_m", row.loggedState(MazeMap::VehicleState::kPx), ParseFloat, error) ||
                !ParseField(tokens, indices, "ukf_state_py_m", row.loggedState(MazeMap::VehicleState::kPy), ParseFloat, error) ||
                !ParseField(tokens, indices, "ukf_state_psi_rad", row.loggedState(MazeMap::VehicleState::kPsi), ParseFloat, error) ||
                !ParseField(tokens, indices, "ukf_state_u_mps", row.loggedState(MazeMap::VehicleState::kU), ParseFloat, error) ||
                !ParseField(tokens, indices, "ukf_state_v_mps", row.loggedState(MazeMap::VehicleState::kV), ParseFloat, error) ||
                !ParseField(tokens, indices, "ukf_state_r_radps", row.loggedState(MazeMap::VehicleState::kR), ParseFloat, error) ||
                !ParseField(tokens, indices, "ukf_state_omega_l_radps", row.loggedState(MazeMap::VehicleState::kOmegaL), ParseFloat, error) ||
                !ParseField(tokens, indices, "ukf_state_omega_r_radps", row.loggedState(MazeMap::VehicleState::kOmegaR), ParseFloat, error) ||
                !ParseField(tokens, indices, "ukf_state_bgz_radps", row.loggedState(MazeMap::VehicleState::kBgz), ParseFloat, error) ||
                !ParseField(tokens, indices, "measured_linear_speed_mps", row.measuredLinearSpeedMps, ParseFloat, error) ||
                !ParseField(tokens, indices, "measured_angular_speed_radps", row.measuredAngularSpeedRadps, ParseFloat, error) ||
                !ParseField(tokens, indices, "cmd_linear_mps", row.commandedLinearMps, ParseFloat, error) ||
                !ParseField(tokens, indices, "cmd_angular_radps", row.commandedAngularRadps, ParseFloat, error) ||
                !ParseField(tokens, indices, "left_drive_command", row.leftDriveCommand, ParseFloat, error) ||
                !ParseField(tokens, indices, "right_drive_command", row.rightDriveCommand, ParseFloat, error) ||
                !ParseField(tokens, indices, "left_feedforward_command", row.leftFeedforwardCommand, ParseFloat, error) ||
                !ParseField(tokens, indices, "right_feedforward_command", row.rightFeedforwardCommand, ParseFloat, error) ||
                !ParseField(tokens, indices, "left_launch_assist_floor", row.leftLaunchAssistFloor, ParseFloat, error) ||
                !ParseField(tokens, indices, "right_launch_assist_floor", row.rightLaunchAssistFloor, ParseFloat, error) ||
                !ParseField(tokens, indices, "left_encoder_count", row.leftEncoderCount, ParseInt32, error) ||
                !ParseField(tokens, indices, "right_encoder_count", row.rightEncoderCount, ParseInt32, error) ||
                !ParseField(tokens, indices, "left_encoder_omega_radps", row.leftEncoderOmegaRadps, ParseFloat, error) ||
                !ParseField(tokens, indices, "right_encoder_omega_radps", row.rightEncoderOmegaRadps, ParseFloat, error) ||
                !ParseField(tokens, indices, "left_encoder_velocity_mps", row.leftEncoderVelocityMps, ParseFloat, error) ||
                !ParseField(tokens, indices, "right_encoder_velocity_mps", row.rightEncoderVelocityMps, ParseFloat, error) ||
                !ParseField(tokens, indices, "gyro_raw_radps", row.gyroRawRadps, ParseFloat, error) ||
                !ParseField(tokens, indices, "gyro_radps", row.gyroCorrectedRadps, ParseFloat, error) ||
                !ParseField(tokens, indices, "accel_body_x_mps2", row.accelBodyXMps2, ParseFloat, error) ||
                !ParseField(tokens, indices, "accel_body_y_mps2", row.accelBodyYMps2, ParseFloat, error) ||
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
            row.fanDutyCycle = loggedFanDutyCycle;
            MazeMap::VehicleState::NormalizeStateVector(row.loggedState);
            rows.push_back(row);
        }

        if (rows.empty())
        {
            error = "CSV did not contain any rows: " + candidate.csvPath.string();
            return false;
        }

        return true;
    }

    MazeMap::ControlInput BuildControlInput(const LoggedRow& row, float batteryVoltageV, float fallbackBatteryVoltageV) noexcept
    {
        MazeMap::ControlInput control{};
        control.leftMotorCommand = row.leftDriveCommand;
        control.rightMotorCommand = row.rightDriveCommand;
        control.fanDutyCycle = row.fanDutyCycle;
        control.batteryVoltageV = std::isfinite(batteryVoltageV) ? batteryVoltageV : fallbackBatteryVoltageV;
        return control;
    }

    MazeMap::EncoderObs BuildEncoderObservation(const LoggedRow& row) noexcept
    {
        MazeMap::EncoderObs observation{};
        observation.totalLeftCounts = row.leftEncoderCount;
        observation.totalRightCounts = row.rightEncoderCount;
        observation.omegaLeftRadps = row.leftEncoderOmegaRadps;
        observation.omegaRightRadps = row.rightEncoderOmegaRadps;
        return observation;
    }

    float SensorForwardVelocityMps(const LoggedRow& row) noexcept
    {
        return 0.5f * (row.leftEncoderVelocityMps + row.rightEncoderVelocityMps);
    }

    float CommandAverage(float leftCommand, float rightCommand) noexcept
    {
        return 0.5f * (leftCommand + rightCommand);
    }

    float CommandDelta(float leftCommand, float rightCommand) noexcept
    {
        return 0.5f * (rightCommand - leftCommand);
    }

    bool IsSensorComparableTransition(const LoggedRow& currentRow, const LoggedRow& nextRow) noexcept
    {
        return
            (currentRow.sectionId == nextRow.sectionId) &&
            (currentRow.primitiveId == nextRow.primitiveId) &&
            (currentRow.repeatIndex == nextRow.repeatIndex);
    }

    const FeedforwardPathDefinition& GetFeedforwardPathDefinition(const FeedforwardPathId id) noexcept
    {
        return GetFeedforwardPathDefinitions()[static_cast<std::size_t>(id)];
    }

    bool BuildSensorOnlyFeedforwardState(
        const LoggedRow& row,
        const MazeMap::PlantModel::PreparedParams& prepared,
        MazeMap::VehicleState::StateVector& state) noexcept
    {
        if (!(std::isfinite(row.leftEncoderVelocityMps) &&
            std::isfinite(row.rightEncoderVelocityMps) &&
            std::isfinite(row.gyroCorrectedRadps) &&
            (prepared.raw.wheelRadiusM > 0.0f)))
        {
            return false;
        }

        state = MazeMap::VehicleState::StateVector::Zero();
        state(MazeMap::VehicleState::kU) = SensorForwardVelocityMps(row);
        state(MazeMap::VehicleState::kV) = 0.0f;
        state(MazeMap::VehicleState::kR) = row.gyroCorrectedRadps;
        state(MazeMap::VehicleState::kOmegaL) = row.leftEncoderVelocityMps / prepared.raw.wheelRadiusM;
        state(MazeMap::VehicleState::kOmegaR) = row.rightEncoderVelocityMps / prepared.raw.wheelRadiusM;
        MazeMap::VehicleState::NormalizeStateVector(state);
        return true;
    }

    bool BuildFeedforwardTransitionInputs(
        const MazeMap::PlantModel& plantModel,
        const LoggedRow& currentRow,
        const LoggedRow& nextRow,
        const MazeMap::PlantModel::PreparedParams& prepared,
        float batteryVoltageV,
        FeedforwardTransitionInputs& inputs) noexcept
    {
        if (!IsSensorComparableTransition(currentRow, nextRow))
        {
            return false;
        }

        const float responseTimeS = static_cast<float>(nextRow.dtUs) * 1.0e-6f;
        if (!(std::isfinite(responseTimeS) && (responseTimeS > 0.0f)))
        {
            return false;
        }

        MazeMap::VehicleState::StateVector currentState = MazeMap::VehicleState::StateVector::Zero();
        if (!BuildSensorOnlyFeedforwardState(currentRow, prepared, currentState))
        {
            return false;
        }

        const float currentForwardVelocityMps = SensorForwardVelocityMps(currentRow);
        const float currentYawRateRadps = currentRow.gyroCorrectedRadps;
        const float targetForwardVelocityMps = SensorForwardVelocityMps(nextRow);
        const float targetYawRateRadps = nextRow.gyroCorrectedRadps;
        if (!(std::isfinite(currentForwardVelocityMps) &&
            std::isfinite(currentYawRateRadps) &&
            std::isfinite(targetForwardVelocityMps) &&
            std::isfinite(targetYawRateRadps)))
        {
            return false;
        }

        float maxStateLongitudinalAccelMps2 = 0.0f;
        float maxStateYawAccelRadps2 = 0.0f;
        plantModel.velocityTargetTechnicalLimits(
            currentState,
            prepared,
            maxStateLongitudinalAccelMps2,
            maxStateYawAccelRadps2,
            currentRow.fanDutyCycle);

        float desiredStateLongitudinalAccelMps2 = 0.0f;
        float desiredStateYawAccelRadps2 = 0.0f;
        plantModel.ComputeBodyAction(
            currentForwardVelocityMps,
            targetForwardVelocityMps,
            currentYawRateRadps,
            targetYawRateRadps,
            maxStateLongitudinalAccelMps2,
            maxStateYawAccelRadps2,
            responseTimeS,
            desiredStateLongitudinalAccelMps2,
            desiredStateYawAccelRadps2);

        float maxScalarLongitudinalAccelMps2 = 0.0f;
        float maxScalarYawAccelRadps2 = 0.0f;
        plantModel.velocityTargetTechnicalLimits(
            currentForwardVelocityMps,
            currentYawRateRadps,
            prepared,
            maxScalarLongitudinalAccelMps2,
            maxScalarYawAccelRadps2,
            currentRow.fanDutyCycle);

        float desiredScalarLongitudinalAccelMps2 = 0.0f;
        float desiredScalarYawAccelRadps2 = 0.0f;
        plantModel.ComputeBodyAction(
            currentForwardVelocityMps,
            targetForwardVelocityMps,
            currentYawRateRadps,
            targetYawRateRadps,
            maxScalarLongitudinalAccelMps2,
            maxScalarYawAccelRadps2,
            responseTimeS,
            desiredScalarLongitudinalAccelMps2,
            desiredScalarYawAccelRadps2);

        if (!(std::isfinite(desiredStateLongitudinalAccelMps2) &&
            std::isfinite(desiredStateYawAccelRadps2) &&
            std::isfinite(desiredScalarLongitudinalAccelMps2) &&
            std::isfinite(desiredScalarYawAccelRadps2)))
        {
            return false;
        }

        inputs = {};
        inputs.currentState = currentState;
        inputs.currentForwardSensorMps = currentForwardVelocityMps;
        inputs.currentYawRateSensorRadps = currentYawRateRadps;
        inputs.targetForwardSensorMps = targetForwardVelocityMps;
        inputs.targetYawRateSensorRadps = targetYawRateRadps;
        inputs.desiredStateLongitudinalAccelMps2 = desiredStateLongitudinalAccelMps2;
        inputs.desiredStateYawAccelRadps2 = desiredStateYawAccelRadps2;
        inputs.desiredScalarLongitudinalAccelMps2 = desiredScalarLongitudinalAccelMps2;
        inputs.desiredScalarYawAccelRadps2 = desiredScalarYawAccelRadps2;
        inputs.responseTimeS = responseTimeS;
        inputs.fanDutyCycle = currentRow.fanDutyCycle;
        inputs.batteryVoltageV = batteryVoltageV;
        inputs.reserveUsage = kDefaultFeedforwardReserveUsage;
        return true;
    }

    bool TrySolveFeedforwardPath(
        const MazeMap::PlantModel& plantModel,
        const FeedforwardPathId pathId,
        const FeedforwardTransitionInputs& inputs,
        const MazeMap::PlantModel::PreparedParams& prepared,
        MazeMap::App::Internal::CommandVector& command) noexcept
    {
        (void)prepared;
        switch (pathId)
        {
        case FeedforwardPathId::Acceleration:
            command =
                plantModel.solveAccelerationFeedforward(
                    inputs.desiredScalarLongitudinalAccelMps2,
                    inputs.desiredScalarYawAccelRadps2);
            return true;
        case FeedforwardPathId::SteadyState:
            command =
                plantModel.solveSteadyStateFeedforward(
                    inputs.targetForwardSensorMps,
                    inputs.targetYawRateSensorRadps);
            return true;
        default:
            return false;
        }
    }

    FeedforwardEnvelope ComputeFeedforwardCommandEnvelope(
        const MazeMap::PlantModel& plantModel,
        const FeedforwardPathId pathId,
        const LoggedRow& currentRow,
        const LoggedRow& nextRow,
        const MazeMap::PlantModel::PreparedParams& prepared,
        float batteryVoltageV) noexcept
    {
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
                                perturbedCurrent.gyroCorrectedRadps +=
                                    static_cast<float>(currentYawSign) * kYawRateSensorBoundRadps;
                                perturbedNext.gyroCorrectedRadps +=
                                    static_cast<float>(nextYawSign) * kYawRateSensorBoundRadps;

                                FeedforwardTransitionInputs perturbedInputs{};
                                if (!BuildFeedforwardTransitionInputs(
                                    plantModel,
                                    perturbedCurrent,
                                    perturbedNext,
                                    prepared,
                                    batteryVoltageV,
                                    perturbedInputs))
                                {
                                    continue;
                                }

                                MazeMap::App::Internal::CommandVector command{};
                                if (!TrySolveFeedforwardPath(
                                    plantModel,
                                    pathId,
                                    perturbedInputs,
                                    prepared,
                                    command))
                                {
                                    continue;
                                }
                                if (!command.IsFinite())
                                {
                                    continue;
                                }

                                envelope.valid = true;
                                envelope.leftMin = (std::min)(envelope.leftMin, command.LeftMotorPwm());
                                envelope.leftMax = (std::max)(envelope.leftMax, command.LeftMotorPwm());
                                envelope.rightMin = (std::min)(envelope.rightMin, command.RightMotorPwm());
                                envelope.rightMax = (std::max)(envelope.rightMax, command.RightMotorPwm());
                            }
                        }
                    }
                }
            }
        }

        return envelope;
    }

    void ScoreFeedforward(
        const FeedforwardSampleExportRow& row,
        FeedforwardMetrics& metrics) noexcept
    {
        metrics.leftDriveCommand.add(row.leftDriveCommandError);
        metrics.rightDriveCommand.add(row.rightDriveCommandError);
        metrics.averageDriveCommand.add(row.averageDriveCommandError);
        metrics.deltaDriveCommand.add(row.deltaDriveCommandError);
        metrics.leftFeedforwardCommand.add(row.leftFeedforwardCommandError);
        metrics.rightFeedforwardCommand.add(row.rightFeedforwardCommandError);
        metrics.averageFeedforwardCommand.add(row.averageFeedforwardCommandError);
        metrics.deltaFeedforwardCommand.add(row.deltaFeedforwardCommandError);
        metrics.predictedForwardTargetErrorMps.add(row.predictedForwardTargetErrorMps);
        metrics.predictedYawTargetErrorRadps.add(row.predictedYawTargetErrorRadps);
        if (std::isfinite(row.envelopeLeftMin) &&
            std::isfinite(row.envelopeLeftMax) &&
            std::isfinite(row.envelopeRightMin) &&
            std::isfinite(row.envelopeRightMax))
        {
            metrics.driveEnvelopeHit.add(row.loggedDriveWithinEnvelope);
            metrics.feedforwardEnvelopeHit.add(row.loggedFeedforwardWithinEnvelope);
        }
    }

    bool BuildFeedforwardSampleExportRow(
        const MazeMap::PlantModel& plantModel,
        const FeedforwardPathId pathId,
        const MazeMap::PlantModel::PreparedParams& prepared,
        const LoggedRow& currentRow,
        const LoggedRow& nextRow,
        float batteryVoltageV,
        FeedforwardSampleExportRow& exportRow) noexcept
    {
        const FeedforwardPathDefinition& definition = GetFeedforwardPathDefinition(pathId);
        FeedforwardTransitionInputs inputs{};
        if (!BuildFeedforwardTransitionInputs(
            plantModel,
            currentRow,
            nextRow,
            prepared,
            batteryVoltageV,
            inputs))
        {
            return false;
        }

        MazeMap::App::Internal::CommandVector command{};
        if (!TrySolveFeedforwardPath(
            plantModel,
            pathId,
            inputs,
            prepared,
            command))
        {
            return false;
        }
        if (!command.IsFinite())
        {
            return false;
        }

        const FeedforwardEnvelope envelope =
            ComputeFeedforwardCommandEnvelope(
                plantModel,
                pathId,
                currentRow,
                nextRow,
                prepared,
                batteryVoltageV);

        const MazeMap::VehicleState::StateVector predictedNextState =
            plantModel.integrate(
                inputs.currentState,
                command,
                inputs.fanDutyCycle,
                inputs.batteryVoltageV,
                inputs.responseTimeS,
                prepared);

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
        exportRow.pathId = definition.pathId;
        exportRow.pathLabel = definition.label;
        exportRow.pathCategory = definition.category;
        exportRow.sectionName = SectionName(currentRow.sectionId);
        exportRow.primitiveName = PrimitiveName(currentRow.primitiveId);
        exportRow.phaseName = PhaseName(currentRow.phaseId);
        exportRow.currentForwardSensorMps = inputs.currentForwardSensorMps;
        exportRow.currentLeftVelocityMps = currentRow.leftEncoderVelocityMps;
        exportRow.currentRightVelocityMps = currentRow.rightEncoderVelocityMps;
        exportRow.currentYawRateSensorRadps = inputs.currentYawRateSensorRadps;
        exportRow.targetForwardSensorMps = inputs.targetForwardSensorMps;
        exportRow.targetLeftVelocityMps = nextRow.leftEncoderVelocityMps;
        exportRow.targetRightVelocityMps = nextRow.rightEncoderVelocityMps;
        exportRow.targetYawRateSensorRadps = inputs.targetYawRateSensorRadps;
        exportRow.nominalLeftCommand = command.LeftMotorPwm();
        exportRow.nominalRightCommand = command.RightMotorPwm();
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
        exportRow.loggedLeftFeedforwardCommand = currentRow.leftFeedforwardCommand;
        exportRow.loggedRightFeedforwardCommand = currentRow.rightFeedforwardCommand;
        exportRow.loggedAverageFeedforwardCommand =
            CommandAverage(currentRow.leftFeedforwardCommand, currentRow.rightFeedforwardCommand);
        exportRow.loggedDeltaFeedforwardCommand =
            CommandDelta(currentRow.leftFeedforwardCommand, currentRow.rightFeedforwardCommand);
        exportRow.leftDriveCommandError = exportRow.nominalLeftCommand - exportRow.loggedLeftDriveCommand;
        exportRow.rightDriveCommandError = exportRow.nominalRightCommand - exportRow.loggedRightDriveCommand;
        exportRow.averageDriveCommandError = exportRow.nominalAverageCommand - exportRow.loggedAverageDriveCommand;
        exportRow.deltaDriveCommandError = exportRow.nominalDeltaCommand - exportRow.loggedDeltaDriveCommand;
        exportRow.leftFeedforwardCommandError =
            exportRow.nominalLeftCommand - exportRow.loggedLeftFeedforwardCommand;
        exportRow.rightFeedforwardCommandError =
            exportRow.nominalRightCommand - exportRow.loggedRightFeedforwardCommand;
        exportRow.averageFeedforwardCommandError =
            exportRow.nominalAverageCommand - exportRow.loggedAverageFeedforwardCommand;
        exportRow.deltaFeedforwardCommandError =
            exportRow.nominalDeltaCommand - exportRow.loggedDeltaFeedforwardCommand;
        exportRow.predictedNextForwardMps = predictedNextState(MazeMap::VehicleState::kU);
        exportRow.predictedNextYawRateRadps = predictedNextState(MazeMap::VehicleState::kR);
        exportRow.predictedForwardTargetErrorMps = exportRow.predictedNextForwardMps - inputs.targetForwardSensorMps;
        exportRow.predictedYawTargetErrorRadps = exportRow.predictedNextYawRateRadps - inputs.targetYawRateSensorRadps;
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
            exportRow.loggedFeedforwardWithinEnvelope =
                (currentRow.leftFeedforwardCommand >= envelope.leftMin) &&
                (currentRow.leftFeedforwardCommand <= envelope.leftMax) &&
                (currentRow.rightFeedforwardCommand >= envelope.rightMin) &&
                (currentRow.rightFeedforwardCommand <= envelope.rightMax);
        }
        return true;
    }

    SampleExportRow BuildSampleExportRow(
        const LoggedRow& row,
        const MazeMap::VehicleState::StateVector& predictedState,
        float predictedGyroBiasAnchorRadps,
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
        exportRow.predictedAccelBodyXMps2 = predictedAccelBodyMps2.x();
        exportRow.actualAccelBodyXMps2 = row.accelBodyXMps2;
        exportRow.predictedAccelBodyYMps2 = predictedAccelBodyMps2.y();
        exportRow.actualAccelBodyYMps2 = row.accelBodyYMps2;
        exportRow.predictedPlanarAccelMps2 = predictedAccelBodyMps2.norm();
        exportRow.actualPlanarAccelMps2 = row.planarAccelMps2;
        exportRow.predictedLinearSpeedMps = predictedState(MazeMap::VehicleState::kU);
        exportRow.actualLinearSpeedMps = row.measuredLinearSpeedMps;
        exportRow.predictedYawRateRadps = predictedState(MazeMap::VehicleState::kR);
        exportRow.actualYawRateRadps = row.gyroCorrectedRadps;
        exportRow.predictedRawGyroRadps =
            predictedState(MazeMap::VehicleState::kR) + predictedGyroBiasAnchorRadps;
        exportRow.actualRawGyroRadps = row.gyroRawRadps;
        return exportRow;
    }

    void ScorePrediction(
        const LoggedRow& row,
        const MazeMap::VehicleState::StateVector& predictedState,
        float predictedGyroBiasAnchorRadps,
        const MazeMap::PlantParams& params,
        const Eigen::Vector2f& predictedAccelBodyMps2,
        PredictionMetrics& metrics)
    {
        metrics.encoderLeftOmegaRadps.add(
            static_cast<double>(predictedState(MazeMap::VehicleState::kOmegaL)) -
            static_cast<double>(row.leftEncoderOmegaRadps));
        metrics.encoderRightOmegaRadps.add(
            static_cast<double>(predictedState(MazeMap::VehicleState::kOmegaR)) -
            static_cast<double>(row.rightEncoderOmegaRadps));

        const double predictedEncoderLinearMps =
            0.5 * static_cast<double>(params.wheelRadiusM) *
            (static_cast<double>(predictedState(MazeMap::VehicleState::kOmegaL)) +
             static_cast<double>(predictedState(MazeMap::VehicleState::kOmegaR)));
        metrics.encoderLinearSpeedMps.add(predictedEncoderLinearMps - static_cast<double>(row.measuredLinearSpeedMps));

        const double predictedEncoderYawRadps =
            static_cast<double>(params.wheelRadiusM) *
            (static_cast<double>(predictedState(MazeMap::VehicleState::kOmegaL)) -
             static_cast<double>(predictedState(MazeMap::VehicleState::kOmegaR))) /
            static_cast<double>(params.trackWidthM);
        metrics.encoderYawRateRadps.add(predictedEncoderYawRadps - static_cast<double>(row.measuredAngularSpeedRadps));

        metrics.bodyForwardSpeedMps.add(
            static_cast<double>(predictedState(MazeMap::VehicleState::kU)) -
            static_cast<double>(row.measuredLinearSpeedMps));
        metrics.bodyYawRateRadps.add(
            static_cast<double>(predictedState(MazeMap::VehicleState::kR)) -
            static_cast<double>(row.gyroCorrectedRadps));

        if (std::isfinite(row.gyroRawRadps))
        {
            const double predictedRawGyroRadps =
                static_cast<double>(predictedState(MazeMap::VehicleState::kR)) +
                static_cast<double>(predictedGyroBiasAnchorRadps);
            metrics.rawGyroRadps.add(predictedRawGyroRadps - static_cast<double>(row.gyroRawRadps));
        }

        if (row.accelBiasValid)
        {
            metrics.accelBodyXMps2.add(
                static_cast<double>(predictedAccelBodyMps2.x()) - static_cast<double>(row.accelBodyXMps2));
            metrics.accelBodyYMps2.add(
                static_cast<double>(predictedAccelBodyMps2.y()) - static_cast<double>(row.accelBodyYMps2));
            metrics.planarAccelMps2.add(
                std::hypot(static_cast<double>(predictedAccelBodyMps2.x()), static_cast<double>(predictedAccelBodyMps2.y())) -
                static_cast<double>(row.planarAccelMps2));
        }
    }

    void ScoreConsistency(
        const LoggedRow& row,
        const MazeMap::VehicleState::StateVector& replayedState,
        ConsistencyMetrics& metrics)
    {
        const double dxM = static_cast<double>(replayedState(MazeMap::VehicleState::kPx) - row.loggedState(MazeMap::VehicleState::kPx));
        const double dyM = static_cast<double>(replayedState(MazeMap::VehicleState::kPy) - row.loggedState(MazeMap::VehicleState::kPy));
        metrics.positionMm.add(1000.0 * std::hypot(dxM, dyM));
        metrics.headingDeg.add(
            static_cast<double>(kRadiansToDegrees) *
            static_cast<double>(MazeMap::VehicleState::NormalizeAngle(
                replayedState(MazeMap::VehicleState::kPsi) - row.loggedState(MazeMap::VehicleState::kPsi))));
        metrics.forwardSpeedMps.add(
            static_cast<double>(replayedState(MazeMap::VehicleState::kU) - row.loggedState(MazeMap::VehicleState::kU)));
        metrics.lateralSpeedMps.add(
            static_cast<double>(replayedState(MazeMap::VehicleState::kV) - row.loggedState(MazeMap::VehicleState::kV)));
        metrics.yawRateRadps.add(
            static_cast<double>(replayedState(MazeMap::VehicleState::kR) - row.loggedState(MazeMap::VehicleState::kR)));
        metrics.leftWheelOmegaRadps.add(
            static_cast<double>(replayedState(MazeMap::VehicleState::kOmegaL) - row.loggedState(MazeMap::VehicleState::kOmegaL)));
        metrics.rightWheelOmegaRadps.add(
            static_cast<double>(replayedState(MazeMap::VehicleState::kOmegaR) - row.loggedState(MazeMap::VehicleState::kOmegaR)));
        metrics.gyroBiasRadps.add(
            static_cast<double>(replayedState(MazeMap::VehicleState::kBgz) - row.loggedState(MazeMap::VehicleState::kBgz)));
    }

    RunReport ReplayRun(const RunCandidate& candidate, const ReplayOptions& options)
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

        MazeMap::Estimator ukf;
        MazeMap::PlantModel plantModel;
        const MazeMap::SrUkfCore::StateVector initialState =
            options.useKnownStationarySeed ?
            BuildKnownStationaryOpenFloorInitialState() :
            keptRows.front().loggedState;
        if (!ukf.reset(initialState, MazeMap::SrUkfCore::BuildDefaultInitialCovariance()))
        {
            report.completed = false;
            report.failureReason = "Failed to seed replay UKF";
            return report;
        }

        SectionPhaseReportMap sectionPhaseBuckets;
        const MazeMap::PlantParams& params = ukf.ukf().params();
        const MazeMap::PlantModel::PreparedParams prepared = ukf.ukf().preparedParams();
        const float fallbackBatteryVoltageV = params.supplyVoltageV;
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
            FeedforwardTransitionInputs transitionInputs{};
            if (!BuildFeedforwardTransitionInputs(
                plantModel,
                currentRow,
                nextRow,
                prepared,
                std::isfinite(candidate.batteryVoltageV) ? candidate.batteryVoltageV : fallbackBatteryVoltageV,
                transitionInputs))
            {
                continue;
            }

            ++report.feedforwardTransitions;
            for (FeedforwardPathSummary& pathSummary : report.feedforwardPaths)
            {
                ++pathSummary.comparableTransitions;
                FeedforwardSampleExportRow sample{};
                if (!BuildFeedforwardSampleExportRow(
                    plantModel,
                    pathSummary.id,
                    prepared,
                    currentRow,
                    nextRow,
                    transitionInputs.batteryVoltageV,
                    sample))
                {
                    continue;
                }

                ++pathSummary.validSolutions;
                ScoreFeedforward(sample, pathSummary.metrics);
                if (exportFeedforwardRows)
                {
                    report.feedforwardSampleExportRows.push_back(std::move(sample));
                }
            }
        }

        for (std::size_t index = 1; index < keptRows.size(); ++index)
        {
            const LoggedRow& row = keptRows[index];
            ++report.scoredTransitions;

            ukf.ukf().setRuntimeContext(
                row.commandedLinearMps,
                row.commandedAngularRadps,
                row.saturationFlags,
                row.leftLaunchAssistFloor,
                row.rightLaunchAssistFloor,
                row.accelBiasValid,
                row.accelBodyXMps2,
                row.accelBodyYMps2);

            const float dtSeconds = static_cast<float>(row.dtUs) * 1.0e-6f;
            const MazeMap::ControlInput control =
                BuildControlInput(row, candidate.batteryVoltageV, fallbackBatteryVoltageV);
            if (!ukf.predict(dtSeconds, control))
            {
                ++report.predictFailures;
                report.completed = false;
                report.failureReason =
                    "predict_failed at control_tick_sequence=" + std::to_string(row.controlTickSequence);
                break;
            }

            const MazeMap::VehicleState::StateVector predictedState = ukf.ukf().state();
            const Eigen::Vector2f predictedAccelBodyMps2 =
                plantModel.imuPlanarAcceleration(predictedState, control, ukf.ukf().preparedParams());
            SectionPhaseReport& phaseBucket =
                GetOrCreateSectionPhaseReport(sectionPhaseBuckets, row.sectionId, row.phaseId);
            ++phaseBucket.sampleCount;
            ScorePrediction(
                row,
                predictedState,
                ukf.ukf().gyroBiasAnchorRadps(),
                params,
                predictedAccelBodyMps2,
                report.prediction);
            ScorePrediction(
                row,
                predictedState,
                ukf.ukf().gyroBiasAnchorRadps(),
                params,
                predictedAccelBodyMps2,
                phaseBucket.prediction);
            if (exportSampleRows)
            {
                report.sampleExportRows.push_back(BuildSampleExportRow(
                    row,
                    predictedState,
                    ukf.ukf().gyroBiasAnchorRadps(),
                    predictedAccelBodyMps2));
            }

            (void)ukf.updateEncoderPair(BuildEncoderObservation(row), dtSeconds);
            if (std::isfinite(row.gyroRawRadps))
            {
                const MazeMap::MeasurementUpdateResult yawUpdate = ukf.updateYawRate(row.gyroRawRadps);
                if (!yawUpdate.accepted)
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
                accelObservation.valid =
                    std::isfinite(row.accelBodyXMps2) &&
                    std::isfinite(row.accelBodyYMps2);
                accelObservation.accelBodyXMps2 = row.accelBodyXMps2;
                accelObservation.accelBodyYMps2 = row.accelBodyYMps2;
                (void)ukf.updatePlanarAccel(accelObservation);
            }

            ScoreConsistency(row, ukf.ukf().state(), report.consistency);
            ScoreConsistency(row, ukf.ukf().state(), phaseBucket.consistency);
        }

        report.sectionPhaseBuckets = ToSortedSectionPhaseReports(sectionPhaseBuckets);
        return report;
    }

    std::string FormatDouble(double value, int precision = 6)
    {
        std::ostringstream stream;
        stream << std::fixed << std::setprecision(precision) << value;
        return stream.str();
    }

    void WriteMetricRow(
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

    void WriteHitRateRow(
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

    std::uint64_t TotalFeedforwardValidSolutions(const std::vector<FeedforwardPathSummary>& summaries) noexcept
    {
        std::uint64_t total = 0U;
        for (const FeedforwardPathSummary& summary : summaries)
        {
            total += summary.validSolutions;
        }
        return total;
    }

    void WritePhaseAssociationRow(
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

    std::string SampleMetricValue(const std::string& metric, const SampleExportRow& row)
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
        if (metric == "predicted_accel_body_x_mps2")
        {
            return FormatDouble(row.predictedAccelBodyXMps2);
        }
        if (metric == "actual_accel_body_x_mps2")
        {
            return FormatDouble(row.actualAccelBodyXMps2);
        }
        if (metric == "predicted_accel_body_y_mps2")
        {
            return FormatDouble(row.predictedAccelBodyYMps2);
        }
        if (metric == "actual_accel_body_y_mps2")
        {
            return FormatDouble(row.actualAccelBodyYMps2);
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

    bool WriteSampleExportCsv(const CorpusReport& corpus, const ReplayOptions& options, std::string& error)
    {
        if (options.sampleCsvPath.empty())
        {
            return true;
        }

        const auto runIt = std::find_if(
            corpus.runs.begin(),
            corpus.runs.end(),
            [&options](const RunReport& run) { return run.runId == options.runIdFilter; });
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

        const fs::path parentPath = options.sampleCsvPath.parent_path();
        if (!parentPath.empty())
        {
            std::error_code createEc;
            fs::create_directories(parentPath, createEc);
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

    bool WriteFeedforwardSampleExportCsv(const CorpusReport& corpus, const ReplayOptions& options, std::string& error)
    {
        if (options.feedforwardSampleCsvPath.empty())
        {
            return true;
        }

        const auto runIt = std::find_if(
            corpus.runs.begin(),
            corpus.runs.end(),
            [&options](const RunReport& run) { return run.runId == options.runIdFilter; });
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

        const fs::path parentPath = options.feedforwardSampleCsvPath.parent_path();
        if (!parentPath.empty())
        {
            std::error_code createEc;
            fs::create_directories(parentPath, createEc);
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
            << "logged_average_drive_command,logged_delta_drive_command,logged_left_feedforward_command,"
            << "logged_right_feedforward_command,logged_average_feedforward_command,logged_delta_feedforward_command,"
            << "left_drive_command_error,right_drive_command_error,average_drive_command_error,delta_drive_command_error,"
            << "left_feedforward_command_error,right_feedforward_command_error,average_feedforward_command_error,"
            << "delta_feedforward_command_error,envelope_left_min,envelope_left_max,envelope_right_min,"
            << "envelope_right_max,logged_drive_within_envelope,logged_feedforward_within_envelope,"
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
                << FormatDouble(row.loggedLeftFeedforwardCommand) << ','
                << FormatDouble(row.loggedRightFeedforwardCommand) << ','
                << FormatDouble(row.loggedAverageFeedforwardCommand) << ','
                << FormatDouble(row.loggedDeltaFeedforwardCommand) << ','
                << FormatDouble(row.leftDriveCommandError) << ','
                << FormatDouble(row.rightDriveCommandError) << ','
                << FormatDouble(row.averageDriveCommandError) << ','
                << FormatDouble(row.deltaDriveCommandError) << ','
                << FormatDouble(row.leftFeedforwardCommandError) << ','
                << FormatDouble(row.rightFeedforwardCommandError) << ','
                << FormatDouble(row.averageFeedforwardCommandError) << ','
                << FormatDouble(row.deltaFeedforwardCommandError) << ','
                << FormatDouble(row.envelopeLeftMin) << ','
                << FormatDouble(row.envelopeLeftMax) << ','
                << FormatDouble(row.envelopeRightMin) << ','
                << FormatDouble(row.envelopeRightMax) << ','
                << (row.loggedDriveWithinEnvelope ? "true" : "false") << ','
                << (row.loggedFeedforwardWithinEnvelope ? "true" : "false") << ','
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
                << ", feedforward_average_rmse=" << FormatDouble(summary.metrics.averageFeedforwardCommand.rmse(), 6)
                << ", feedforward_delta_rmse=" << FormatDouble(summary.metrics.deltaFeedforwardCommand.rmse(), 6)
                << ", drive_envelope_hit_rate_pct=" << FormatDouble(100.0 * summary.metrics.driveEnvelopeHit.rate(), 2)
                << "\n";
        }
        return true;
    }

    bool WriteReportFiles(const CorpusReport& corpus, const ReplayOptions& options, std::string& error)
    {
        fs::path outputPath = options.outputPath;
        if (outputPath.empty())
        {
            outputPath = options.rootPath / ("open_floor_ukf_replay_" + MakeTimestampString());
        }

        std::error_code createEc;
        fs::create_directories(outputPath, createEc);
        if (createEc)
        {
            error = "Failed to create output directory: " + outputPath.string();
            return false;
        }

        const fs::path markdownPath = outputPath / kReportFileName;
        const fs::path csvPath = outputPath / kRunSummaryFileName;
        const fs::path sectionPhaseCsvPath = outputPath / kSectionPhaseSummaryFileName;
        const fs::path aggregateJsonPath = outputPath / kAggregateMetricsFileName;
        const fs::path feedforwardPathCsvPath = outputPath / kFeedforwardPathSummaryFileName;

        std::ofstream markdown(markdownPath);
        if (!markdown)
        {
            error = "Failed to open report file for write: " + markdownPath.string();
            return false;
        }

        const std::size_t completedRuns = static_cast<std::size_t>(std::count_if(
            corpus.runs.begin(),
            corpus.runs.end(),
            [](const RunReport& run) { return run.completed; }));
        const std::size_t noRetainedRowsRuns = static_cast<std::size_t>(std::count_if(
            corpus.runs.begin(),
            corpus.runs.end(),
            [](const RunReport& run)
            {
                return !run.completed &&
                    run.failureReason == "Not enough rows after final-section exclusion";
            }));
        const std::size_t replayFaultRuns = corpus.runs.size() - completedRuns - noRetainedRowsRuns;
        const PhaseAssociationSummary encoderLinearPhaseAssociation = ComputePhaseAssociationSummary(
            corpus.sectionPhaseBuckets,
            [](const SectionPhaseReport& bucket) -> const ErrorStats&
            {
                return bucket.prediction.encoderLinearSpeedMps;
            });
        const PhaseAssociationSummary rawGyroPhaseAssociation = ComputePhaseAssociationSummary(
            corpus.sectionPhaseBuckets,
            [](const SectionPhaseReport& bucket) -> const ErrorStats&
            {
                return bucket.prediction.rawGyroRadps;
            });
        const PhaseAssociationSummary accelXPhaseAssociation = ComputePhaseAssociationSummary(
            corpus.sectionPhaseBuckets,
            [](const SectionPhaseReport& bucket) -> const ErrorStats&
            {
                return bucket.prediction.accelBodyXMps2;
            });
        const PhaseAssociationSummary accelYPhaseAssociation = ComputePhaseAssociationSummary(
            corpus.sectionPhaseBuckets,
            [](const SectionPhaseReport& bucket) -> const ErrorStats&
            {
                return bucket.prediction.accelBodyYMps2;
            });
        const PhaseAssociationSummary postPositionPhaseAssociation = ComputePhaseAssociationSummary(
            corpus.sectionPhaseBuckets,
            [](const SectionPhaseReport& bucket) -> const ErrorStats&
            {
                return bucket.consistency.positionMm;
            });
        const PhaseAssociationSummary postHeadingPhaseAssociation = ComputePhaseAssociationSummary(
            corpus.sectionPhaseBuckets,
            [](const SectionPhaseReport& bucket) -> const ErrorStats&
            {
                return bucket.consistency.headingDeg;
            });
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
                "canonical open-floor marker `C` stationary state plus `SrUkfCore::BuildDefaultInitialCovariance()`.\n" :
                "first kept logged state plus `SrUkfCore::BuildDefaultInitialCovariance()`.\n")
            << "- Tuning override file: "
            << (options.tuningPath.empty() ? "`<none>`\n" : (std::string("`") + options.tuningPath.string() + "`\n"))
            << "- Battery policy: use `battery_voltage_start` from the bound `logging.txt` when available; otherwise fall back to the current plant default.\n"
            << "- Prediction metrics compare the pre-update UKF prediction against observable sensor-space signals.\n"
            << "- Post-update replay deltas compare the replayed UKF state against the logged UKF state from the capture; they are consistency checks, not external ground truth.\n"
            << "- Feedforward validation uses the present row's wheel-side velocities plus debiased gyro as the current state, the following row's wheel-side velocities plus debiased gyro as the target state, the following row's `dt_us` as the response horizon, and never uses logged UKF state for that validation.\n"
            << "- Feedforward path coverage: " << corpus.feedforwardPaths.size() << " canonical PlantModel public paths evaluated.\n"
            << "- Feedforward audit sensor bounds: each wheel-side velocity is treated as a `+/- 0.06 m/s` sensor and debiased yaw rate as a `+/- 0.03 rad/s` sensor when computing per-path command envelopes.\n"
            << "- Feedforward evaluations completed: " << totalFeedforwardEvaluations << "\n"
            << "- Phase analysis uses canonical `section_id` + `phase_id` buckets from the open-floor schema.\n"
            << "- `eta_squared_abs` is the fraction of absolute-error variance explained by those section-phase buckets.\n\n";

        markdown << "## Aggregate Prediction Error\n\n";
        markdown << "| Signal | Unit | Samples | RMSE | MAE | Bias | Max Abs |\n";
        markdown << "| --- | --- | ---: | ---: | ---: | ---: | ---: |\n";
        WriteMetricRow(markdown, "encoder_left_omega", "rad/s", corpus.prediction.encoderLeftOmegaRadps);
        WriteMetricRow(markdown, "encoder_right_omega", "rad/s", corpus.prediction.encoderRightOmegaRadps);
        WriteMetricRow(markdown, "encoder_linear_speed", "m/s", corpus.prediction.encoderLinearSpeedMps);
        WriteMetricRow(markdown, "encoder_yaw_rate", "rad/s", corpus.prediction.encoderYawRateRadps);
        WriteMetricRow(markdown, "body_forward_speed", "m/s", corpus.prediction.bodyForwardSpeedMps);
        WriteMetricRow(markdown, "body_yaw_rate_vs_gyro", "rad/s", corpus.prediction.bodyYawRateRadps);
        WriteMetricRow(markdown, "raw_gyro", "rad/s", corpus.prediction.rawGyroRadps);
        WriteMetricRow(markdown, "accel_body_x", "m/s^2", corpus.prediction.accelBodyXMps2);
        WriteMetricRow(markdown, "accel_body_y", "m/s^2", corpus.prediction.accelBodyYMps2);
        WriteMetricRow(markdown, "planar_accel", "m/s^2", corpus.prediction.planarAccelMps2);

        markdown << "\n## Aggregate Post-Update Replay Delta\n\n";
        markdown << "| Signal | Unit | Samples | RMSE | MAE | Bias | Max Abs |\n";
        markdown << "| --- | --- | ---: | ---: | ---: | ---: | ---: |\n";
        WriteMetricRow(markdown, "position", "mm", corpus.consistency.positionMm);
        WriteMetricRow(markdown, "heading", "deg", corpus.consistency.headingDeg);
        WriteMetricRow(markdown, "forward_speed", "m/s", corpus.consistency.forwardSpeedMps);
        WriteMetricRow(markdown, "lateral_speed", "m/s", corpus.consistency.lateralSpeedMps);
        WriteMetricRow(markdown, "yaw_rate", "rad/s", corpus.consistency.yawRateRadps);
        WriteMetricRow(markdown, "left_wheel_omega", "rad/s", corpus.consistency.leftWheelOmegaRadps);
        WriteMetricRow(markdown, "right_wheel_omega", "rad/s", corpus.consistency.rightWheelOmegaRadps);
        WriteMetricRow(markdown, "gyro_bias", "rad/s", corpus.consistency.gyroBiasRadps);

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
                    << (summary.validSolutions > 0U ? FormatDouble(summary.metrics.averageFeedforwardCommand.rmse()) : "")
                    << " | "
                    << (summary.validSolutions > 0U ? FormatDouble(summary.metrics.deltaFeedforwardCommand.rmse()) : "")
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
        WritePhaseAssociationRow(markdown, "accel_body_x", accelXPhaseAssociation);
        WritePhaseAssociationRow(markdown, "accel_body_y", accelYPhaseAssociation);
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
                << " | " << FormatDouble(bucket.prediction.accelBodyXMps2.rmse())
                << " | " << FormatDouble(bucket.prediction.accelBodyYMps2.rmse())
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
                << " | " << FormatDouble(run.prediction.accelBodyXMps2.rmse())
                << " | " << FormatDouble(run.prediction.accelBodyYMps2.rmse())
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
            << "run_id,format_version,csv_path,battery_source,battery_voltage_v,total_rows,kept_rows,scored_transitions,feedforward_transitions,feedforward_evaluations,ignored_section_id,predict_failures,yaw_rejects,completed,encoder_linear_rmse_mps,raw_gyro_rmse_radps,accel_body_x_rmse_mps2,accel_body_y_rmse_mps2,post_position_rmse_mm,post_heading_rmse_deg,failure_reason\n";
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
                << FormatDouble(run.prediction.accelBodyXMps2.rmse()) << ','
                << FormatDouble(run.prediction.accelBodyYMps2.rmse()) << ','
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
            << "path_id,label,category,comparable_transitions,valid_solutions,drive_average_rmse,drive_delta_rmse,feedforward_average_rmse,feedforward_delta_rmse,drive_envelope_hit_pct,feedforward_envelope_hit_pct,forward_target_rmse_mps,yaw_target_rmse_radps\n";
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
                << FormatDouble(summary.metrics.averageFeedforwardCommand.rmse()) << ','
                << FormatDouble(summary.metrics.deltaFeedforwardCommand.rmse()) << ','
                << FormatDouble(100.0 * summary.metrics.driveEnvelopeHit.rate(), 2) << ','
                << FormatDouble(100.0 * summary.metrics.feedforwardEnvelopeHit.rate(), 2) << ','
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
            << "section_id,section_name,phase_id,phase_name,bucket_samples,encoder_linear_samples,raw_gyro_samples,accel_body_x_samples,accel_body_y_samples,post_position_samples,post_heading_samples,encoder_linear_rmse_mps,encoder_linear_mae_mps,raw_gyro_rmse_radps,raw_gyro_mae_radps,accel_body_x_rmse_mps2,accel_body_x_mae_mps2,accel_body_y_rmse_mps2,accel_body_y_mae_mps2,post_position_rmse_mm,post_position_mae_mm,post_heading_rmse_deg,post_heading_mae_deg\n";
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
                << bucket.prediction.accelBodyXMps2.count << ','
                << bucket.prediction.accelBodyYMps2.count << ','
                << bucket.consistency.positionMm.count << ','
                << bucket.consistency.headingDeg.count << ','
                << FormatDouble(bucket.prediction.encoderLinearSpeedMps.rmse()) << ','
                << FormatDouble(bucket.prediction.encoderLinearSpeedMps.mae()) << ','
                << FormatDouble(bucket.prediction.rawGyroRadps.rmse()) << ','
                << FormatDouble(bucket.prediction.rawGyroRadps.mae()) << ','
                << FormatDouble(bucket.prediction.accelBodyXMps2.rmse()) << ','
                << FormatDouble(bucket.prediction.accelBodyXMps2.mae()) << ','
                << FormatDouble(bucket.prediction.accelBodyYMps2.rmse()) << ','
                << FormatDouble(bucket.prediction.accelBodyYMps2.mae()) << ','
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
            << "  \"tuning_path\": "
            << (options.tuningPath.empty() ? "null" : (std::string("\"") + options.tuningPath.generic_string() + "\"")) << ",\n"
            << "  \"prediction\": {\n"
            << "    \"encoder_linear_rmse_mps\": " << FormatDouble(corpus.prediction.encoderLinearSpeedMps.rmse(), 12) << ",\n"
            << "    \"raw_gyro_rmse_radps\": " << FormatDouble(corpus.prediction.rawGyroRadps.rmse(), 12) << ",\n"
            << "    \"accel_body_x_rmse_mps2\": " << FormatDouble(corpus.prediction.accelBodyXMps2.rmse(), 12) << ",\n"
            << "    \"accel_body_y_rmse_mps2\": " << FormatDouble(corpus.prediction.accelBodyYMps2.rmse(), 12) << ",\n"
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
                << "      \"feedforward_average_rmse\": " << FormatDouble(summary.metrics.averageFeedforwardCommand.rmse(), 12) << ",\n"
                << "      \"feedforward_delta_rmse\": " << FormatDouble(summary.metrics.deltaFeedforwardCommand.rmse(), 12) << ",\n"
                << "      \"drive_envelope_hit_pct\": " << FormatDouble(100.0 * summary.metrics.driveEnvelopeHit.rate(), 12) << ",\n"
                << "      \"feedforward_envelope_hit_pct\": " << FormatDouble(100.0 * summary.metrics.feedforwardEnvelopeHit.rate(), 12) << ",\n"
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

    if (!LoadTuningFile(options.tuningPath, error))
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

    SectionPhaseReportMap corpusSectionPhaseBuckets;
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

    const std::size_t completedRuns = static_cast<std::size_t>(std::count_if(
        corpus.runs.begin(),
        corpus.runs.end(),
        [](const RunReport& report) { return report.completed; }));
    std::cout
        << "Replayed " << corpus.runs.size() << " unique runs; "
        << completedRuns << " completed without replay faults.\n";
    return 0;
}
