#include "..\..\MazeMap\MazeMap\CommandVector.h"
#include "..\..\MazeMap\MazeMap\CoreConfig.h"
#include "..\..\MazeMap\MazeMap\DirectionalLocation.h"
#include "..\..\MazeMap\MazeMap\Drive.h"
#include "..\..\MazeMap\MazeMap\DriveBase.h"
#include "..\..\MazeMap\MazeMap\DriveBaseTrackingTuning.h"
#include "..\..\MazeMap\MazeMap\DriveTelemetry.h"
#include "..\..\MazeMap\MazeMap\Estimator.h"
#include "..\..\MazeMap\MazeMap\ManeuverInstance.h"
#include "..\..\MazeMap\MazeMap\ManeuverSet.h"
#include "..\..\MazeMap\MazeMap\MazeLocation.h"
#include "..\..\MazeMap\MazeMap\MazeMapRuntimeCore.h"
#include "..\..\MazeMap\MazeMap\MotionLimits.h"
#include "..\..\MazeMap\MazeMap\PlantModel.h"
#include "..\..\MazeMap\MazeMap\SensorSnapshot.h"
#include "..\..\MazeMap\MazeMap\SharedRobotRuntime.h"
#include "..\..\MazeMap\MazeMap\Vehicle.h"
#include "..\..\MazeMap\MazeMap\VehicleState.h"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <new>
#include <numeric>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

    constexpr double kTickSecondsExact = 0.001;
    constexpr float kTickSeconds = static_cast<float>(kTickSecondsExact);
    constexpr float kFanDuty = 0.80f;
    constexpr float kSaturationThreshold = 0.999f;
    constexpr std::size_t kDefaultSearchPasses = 4U;
    constexpr std::size_t kDefaultSearchGridPoints = 9U;
    constexpr std::size_t kDefaultTopCandidateCount = 8U;
    constexpr float kLogSearchMinimumRelativeToMaximum = 1.0e-6f;
    constexpr float kLogSearchAbsolutePositiveMinimum = 1.0e-6f;
    constexpr float kLateWindowFraction = 0.25f;
    constexpr float kLateWindowMinimumS = 0.25f;
    constexpr int kOscillationSignChangeThreshold = 2;
    constexpr int kVelocityStepRmsWindowTicks = 500;
    constexpr int kYawRateStepRmsWindowTicks = 500;
    constexpr int kAccelerationStepRmsWindowTicks = 100;
    constexpr double kAccelerationStepRmsWindowSeconds =
        static_cast<double>(kAccelerationStepRmsWindowTicks) * kTickSecondsExact;
    constexpr float kStepTransitionEpsilon = 1.0e-6f;
    constexpr double kVelocityStepRmsScoreWeight = 60.0;
    constexpr double kYawRateStepRmsScoreWeight = 60.0;
    constexpr double kForwardAccelStepRmsScoreWeight = 40.0;
    constexpr double kYawAccelStepRmsScoreWeight = 40.0;
    constexpr int kStartStraightMaxTicks = 6000;
    constexpr int kDriveManeuverMaxTicks = 20000;
    constexpr int kPlantIntegrationSubstepsPerTick = 10;
    constexpr float kStartStraightDistanceM = 0.30f;
    constexpr float kStartStraightCruiseMps = 0.30f;
    constexpr float kSmoothManeuverEntrySpeedMps = 0.50f;
    constexpr float kInPlaceManeuverPositionToleranceM = 0.020f;
    constexpr float kManeuverHeadingToleranceRad = 3.0f * DEG_TO_RAD_F;
    constexpr float kSmoothManeuverPositionToleranceM = 0.030f;
    constexpr float kManeuverTimeToleranceFraction = 0.40f;
    constexpr float kSmoothManeuverVelocityVariationLimit = 0.05f;
    constexpr float kSmoothManeuverYawAccelerationVariationLimit = 0.20f;
    constexpr float kSmoothManeuverYawRateVariationLimit = 0.08f;
    constexpr float kRippleOscillationDeadband = 1.0e-3f;
    constexpr float kManeuverYawRateMagnitudeEpsilonRadps = 1.0e-4f;
    constexpr float kManeuverTurnPlateauFraction = 0.95f;
    constexpr std::size_t kManeuverRampDeltaTrimSamples = 5U;
    constexpr float kPlantIntegrationSubstepSeconds =
        kTickSeconds / static_cast<float>(kPlantIntegrationSubstepsPerTick);
    constexpr float kTrackingDistanceScaleFloorM = 0.5f * MazeMap::Config::kCellSizeM;
    constexpr float kTrackingHeadingScaleFloorRad = 0.10f;
    constexpr float kTrackingForwardVelocityScaleFloorMps = kSmoothManeuverEntrySpeedMps;
    constexpr float kTrackingYawRateScaleFloorRadps = 1.0f;
    constexpr float kTrackingForwardAccelScaleFloorMps2 = 1.0f;
    constexpr float kTrackingYawAccelScaleFloorRadps2 = 1.0f;

    enum class SignalKind
    {
        ForwardVelocity,
        YawRate,
        Heading
    };

    enum class GainIndex : std::size_t
    {
        ForwardPositionToAccel = 0U,
        ForwardVelocityToAccel,
        ForwardAccelerationError,
        YawPositionToAccel,
        YawVelocityToAccel,
        YawAccelerationError,
        Count
    };

    class GainSet
    {
    public:
        float forwardPositionToAccelGain = 0.0f;
        float forwardVelocityToAccelGain = 0.0f;
        float forwardAccelerationErrorGain = 0.0f;
        float yawPositionToAccelGain = 0.0f;
        float yawVelocityToAccelGain = 0.0f;
        float yawAccelerationErrorGain = 0.0f;
    };

    class ScenarioSpec
    {
    public:
        const char* name = "";
        const char* description = "";
        SignalKind signal = SignalKind::ForwardVelocity;
        float durationS = 0.0f;
        float tolerance = 0.0f;
        float initialForwardMps = 0.0f;
        float initialYawRateRadps = 0.0f;
        float initialYawRad = 0.0f;
        float targetForwardMps = std::numeric_limits<float>::quiet_NaN();
        float targetYawRateRadps = std::numeric_limits<float>::quiet_NaN();
        float targetForwardAccelMps2 = std::numeric_limits<float>::quiet_NaN();
        float targetYawAccelRadps2 = std::numeric_limits<float>::quiet_NaN();
        float targetYawRad = std::numeric_limits<float>::quiet_NaN();
    };

    class WheelObservationState
    {
    public:
        float leftDistanceM = 0.0f;
        float rightDistanceM = 0.0f;
    };

    class ScopedFanDuty final
    {
    public:
        ScopedFanDuty(MazeMap::Vehicle& vehicle, float duty) noexcept
            : target(vehicle)
            , previous(vehicle.GetFanDuty())
        {
            target.SetFanDuty(duty);
        }

        ~ScopedFanDuty() noexcept
        {
            target.SetFanDuty(previous);
        }

        MazeMap::Vehicle& target;
        float previous = 0.0f;
    };

    class ScenarioMetrics
    {
    public:
        std::string name;
        std::string description;
        std::string signal;
        float target = 0.0f;
        float initial = 0.0f;
        float tolerance = 0.0f;
        float durationS = 0.0f;
        float initialForwardMps = 0.0f;
        float initialYawRateRadps = 0.0f;
        float initialYawRad = 0.0f;
        float targetForwardMps = 0.0f;
        float targetYawRateRadps = 0.0f;
        float targetForwardAccelMps2 = 0.0f;
        float targetYawAccelRadps2 = 0.0f;
        float targetYawRad = 0.0f;
        float targetKinematicLateralAccelMps2 = 0.0f;
        int samples = 0;
        float finalValue = 0.0f;
        float finalError = 0.0f;
        double integratedAbsoluteError = 0.0;
        double integratedSquaredError = 0.0;
        bool forwardVelocityStepActive = false;
        bool yawRateStepActive = false;
        bool headingStepActive = false;
        bool forwardAccelStepMetricActive = false;
        bool yawAccelStepMetricActive = false;
        int first500VelocityErrorSamples = 0;
        double first500VelocityErrorRms = (std::numeric_limits<double>::quiet_NaN)();
        int first500YawRateErrorSamples = 0;
        double first500YawRateErrorRms = (std::numeric_limits<double>::quiet_NaN)();
        int first100ForwardAccelErrorSamples = 0;
        int first100ForwardAccelObjectiveSamples = 0;
        int first100ForwardAccelFallbackSamples = 0;
        double first100ForwardAccelObjectiveRms = (std::numeric_limits<double>::quiet_NaN)();
        double first100ForwardAccelErrorRms = (std::numeric_limits<double>::quiet_NaN)();
        std::string first100ForwardAccelDefinition = "inactive";
        int first100YawAccelErrorSamples = 0;
        int first100YawAccelObjectiveSamples = 0;
        int first100YawAccelFallbackSamples = 0;
        double first100YawAccelObjectiveRms = (std::numeric_limits<double>::quiet_NaN)();
        double first100YawAccelErrorRms = (std::numeric_limits<double>::quiet_NaN)();
        std::string first100YawAccelDefinition = "inactive";
        float minimumAbsError = 0.0f;
        float responseReductionFraction = 0.0f;
        double responseFailurePenalty = 0.0;
        float settlingTimeS = 0.0f;
        bool settled = false;
        float overshoot = 0.0f;
        float commandSaturationFraction = 0.0f;
        float plantClipRequestFraction = 0.0f;
        float maxAbsForwardVelocityMps = 0.0f;
        float maxAbsYawRateRadps = 0.0f;
        float maxAbsForwardAccelMps2 = 0.0f;
        float maxAbsYawAccelRadps2 = 0.0f;
        float maxAbsKinematicLateralAccelMps2 = 0.0f;
        bool crossedTarget = false;
        float firstTargetCrossingTimeS = std::numeric_limits<float>::quiet_NaN();
        int signChangesAfterFirstCrossing = 0;
        float lateWindowStartS = 0.0f;
        int lateWindowSamples = 0;
        float lateWindowPeakToPeakError = 0.0f;
        double lateWindowRmsError = 0.0;
        bool oscillatory = false;
        double oscillationPenalty = 0.0;
        int solverFailureCount = 0;
        int nonFiniteCount = 0;
        bool failed = false;
        double score = 0.0;
    };

    class AcceptanceMetrics
    {
    public:
        std::string name;
        std::string description;
        std::string path;
        std::string code;
        std::string metric;
        bool started = false;
        bool completed = false;
        bool passed = false;
        bool blocker = false;
        bool rippleOscillatory = false;
        bool allControlsFinite = true;
        bool commandEvidenceValid = true;
        bool requestedObjectivesFinite = true;
        bool truthFinite = true;
        bool solverClean = true;
        int maxTicks = 0;
        int appliedTicks = 0;
        float elapsedSeconds = 0.0f;
        float targetDistanceM = (std::numeric_limits<float>::quiet_NaN)();
        float encoderAverageDistanceM = (std::numeric_limits<float>::quiet_NaN)();
        float finalXM = (std::numeric_limits<float>::quiet_NaN)();
        float finalYM = (std::numeric_limits<float>::quiet_NaN)();
        float finalYawRad = (std::numeric_limits<float>::quiet_NaN)();
        float targetYawRad = (std::numeric_limits<float>::quiet_NaN)();
        float finalHeadingErrorRad = (std::numeric_limits<float>::quiet_NaN)();
        float finalHeadingErrorDeg = (std::numeric_limits<float>::quiet_NaN)();
        float headingToleranceRad = (std::numeric_limits<float>::quiet_NaN)();
        float headingToleranceDeg = (std::numeric_limits<float>::quiet_NaN)();
        float finalPositionErrorM = (std::numeric_limits<float>::quiet_NaN)();
        float positionToleranceM = (std::numeric_limits<float>::quiet_NaN)();
        float shiftDistanceM = (std::numeric_limits<float>::quiet_NaN)();
        float shiftToleranceM = (std::numeric_limits<float>::quiet_NaN)();
        float expectedElapsedSeconds = (std::numeric_limits<float>::quiet_NaN)();
        float elapsedRelativeError = (std::numeric_limits<float>::quiet_NaN)();
        float elapsedRelativeTolerance = (std::numeric_limits<float>::quiet_NaN)();
        float velocityVariation = (std::numeric_limits<float>::quiet_NaN)();
        float velocityVariationLimit = (std::numeric_limits<float>::quiet_NaN)();
        float yawAccelerationVariation = (std::numeric_limits<float>::quiet_NaN)();
        float yawAccelerationVariationLimit = (std::numeric_limits<float>::quiet_NaN)();
        float yawRateVariation = (std::numeric_limits<float>::quiet_NaN)();
        float yawRateVariationLimit = (std::numeric_limits<float>::quiet_NaN)();
        int trackingSampleCount = 0;
        float distanceRmsErrorM = (std::numeric_limits<float>::quiet_NaN)();
        float distanceRmsReferenceM = (std::numeric_limits<float>::quiet_NaN)();
        float headingRmsErrorRad = (std::numeric_limits<float>::quiet_NaN)();
        float headingRmsErrorDeg = (std::numeric_limits<float>::quiet_NaN)();
        float headingRmsReferenceRad = (std::numeric_limits<float>::quiet_NaN)();
        float forwardVelocityRmsErrorMps = (std::numeric_limits<float>::quiet_NaN)();
        float forwardVelocityRmsReferenceMps = (std::numeric_limits<float>::quiet_NaN)();
        float yawRateRmsErrorRadps = (std::numeric_limits<float>::quiet_NaN)();
        float yawRateRmsReferenceRadps = (std::numeric_limits<float>::quiet_NaN)();
        float forwardAccelRmsErrorMps2 = (std::numeric_limits<float>::quiet_NaN)();
        float forwardAccelRmsReferenceMps2 = (std::numeric_limits<float>::quiet_NaN)();
        float yawAccelRmsErrorRadps2 = (std::numeric_limits<float>::quiet_NaN)();
        float yawAccelRmsReferenceRadps2 = (std::numeric_limits<float>::quiet_NaN)();
        int solverFailureCount = 0;
        int nonFiniteCount = 0;
        double scorePenalty = 0.0;
    };

    class AcceptanceCommandSample
    {
    public:
        float timeSeconds = 0.0f;
        float linearCommandMps = 0.0f;
        float angularCommandRadps = 0.0f;
        float expectedDistanceM = 0.0f;
        float distanceErrorM = 0.0f;
        float expectedHeadingRad = 0.0f;
        float headingErrorRad = 0.0f;
        float forwardVelocityErrorMps = 0.0f;
        float yawRateErrorRadps = 0.0f;
        float expectedForwardAccelMps2 = 0.0f;
        float forwardAccelErrorMps2 = 0.0f;
        float expectedYawAccelRadps2 = 0.0f;
        float yawAccelErrorRadps2 = 0.0f;
    };

    class ManeuverAcceptanceTrace
    {
    public:
        bool started = false;
        bool completed = false;
        bool allControlsFinite = true;
        bool commandEvidenceValid = true;
        bool requestedObjectivesFinite = true;
        bool truthFinite = true;
        bool solverClean = true;
        int solverFailureCount = 0;
        int nonFiniteCount = 0;
        int appliedTicks = 0;
        float elapsedSeconds = 0.0f;
        float expectedDistanceM = 0.0f;
        float expectedHeadingRad = 0.0f;
        WheelObservationState wheels{};
        MazeMap::VehicleState truth{};
        std::vector<AcceptanceCommandSample> samples;
    };

    class EvaluationResult
    {
    public:
        GainSet gains{};
        std::vector<ScenarioMetrics> scenarios;
        std::vector<AcceptanceMetrics> acceptanceScenarios;
        double score = 0.0;
        bool failed = false;
        bool oscillationFlagged = false;
        bool acceptanceBlocked = false;
    };

    class SearchRange
    {
    public:
        float minimum = 0.0f;
        float maximum = 0.0f;
    };

    class Options
    {
    public:
        GainSet candidate{};
        bool hasExplicitCandidate = false;
        bool runSearch = false;
        std::size_t searchGridPoints = kDefaultSearchGridPoints;
        std::size_t searchPasses = kDefaultSearchPasses;
        std::size_t topCandidateCount = kDefaultTopCandidateCount;
        std::array<SearchRange, static_cast<std::size_t>(GainIndex::Count)> searchRanges{};
        std::array<bool, static_cast<std::size_t>(GainIndex::Count)> searchMinimumSpecified{};
        std::array<bool, static_cast<std::size_t>(GainIndex::Count)> searchMaximumSpecified{};
    };

    static const char* GainName(GainIndex index) noexcept
    {
        switch (index)
        {
        case GainIndex::ForwardPositionToAccel:
            return "forward_position_to_accel_gain";
        case GainIndex::ForwardVelocityToAccel:
            return "forward_velocity_to_accel_gain";
        case GainIndex::ForwardAccelerationError:
            return "forward_acceleration_error_gain";
        case GainIndex::YawPositionToAccel:
            return "yaw_position_to_accel_gain";
        case GainIndex::YawVelocityToAccel:
            return "yaw_velocity_to_accel_gain";
        case GainIndex::YawAccelerationError:
            return "yaw_acceleration_error_gain";
        case GainIndex::Count:
        default:
            return "unknown";
        }
    }

    static const char* SignalName(SignalKind signal) noexcept
    {
        switch (signal)
        {
        case SignalKind::ForwardVelocity:
            return "forward_velocity_mps";
        case SignalKind::YawRate:
            return "yaw_rate_radps";
        case SignalKind::Heading:
            return "heading_rad";
        default:
            return "unknown";
        }
    }

    static float QuietNaN() noexcept
    {
        return std::numeric_limits<float>::quiet_NaN();
    }

    static bool ParseFloatText(const std::string& text, float& value) noexcept
    {
        char* end = nullptr;
        errno = 0;
        const double parsed = std::strtod(text.c_str(), &end);
        if ((end == text.c_str()) || (end == nullptr) || (*end != '\0') || (errno == ERANGE) || !std::isfinite(parsed))
        {
            return false;
        }

        value = static_cast<float>(parsed);
        return std::isfinite(value);
    }

    static bool ParseSizeText(const std::string& text, std::size_t& value) noexcept
    {
        char* end = nullptr;
        errno = 0;
        const unsigned long parsed = std::strtoul(text.c_str(), &end, 10);
        if ((end == text.c_str()) || (end == nullptr) || (*end != '\0') || (errno == ERANGE))
        {
            return false;
        }

        value = static_cast<std::size_t>(parsed);
        return true;
    }

    static bool ReadOptionValue(int argc, char* argv[], int& index, const std::string& option, std::string& value, std::string& error)
    {
        const std::string arg = argv[index];
        const std::string prefix = option + "=";
        if (arg.rfind(prefix, 0U) == 0U)
        {
            value = arg.substr(prefix.size());
            return true;
        }

        if (arg == option)
        {
            if ((index + 1) >= argc)
            {
                error = "Missing value for " + option;
                return false;
            }
            ++index;
            value = argv[index];
            return true;
        }

        return false;
    }

    static bool ResolveGainOption(const std::string& name, GainIndex& index) noexcept
    {
        if (name == "--forward-position-to-accel-gain")
        {
            index = GainIndex::ForwardPositionToAccel;
            return true;
        }
        if (name == "--forward-velocity-to-accel-gain")
        {
            index = GainIndex::ForwardVelocityToAccel;
            return true;
        }
        if (name == "--forward-acceleration-error-gain")
        {
            index = GainIndex::ForwardAccelerationError;
            return true;
        }
        if (name == "--yaw-position-to-accel-gain")
        {
            index = GainIndex::YawPositionToAccel;
            return true;
        }
        if (name == "--yaw-velocity-to-accel-gain")
        {
            index = GainIndex::YawVelocityToAccel;
            return true;
        }
        if (name == "--yaw-acceleration-error-gain")
        {
            index = GainIndex::YawAccelerationError;
            return true;
        }
        return false;
    }

    static bool ResolveSearchBoundOption(const std::string& name, GainIndex& index, bool& minimum) noexcept
    {
        if (name == "--forward-position-to-accel-gain-min")
        {
            index = GainIndex::ForwardPositionToAccel;
            minimum = true;
            return true;
        }
        if (name == "--forward-position-to-accel-gain-max")
        {
            index = GainIndex::ForwardPositionToAccel;
            minimum = false;
            return true;
        }
        if (name == "--forward-velocity-to-accel-gain-min")
        {
            index = GainIndex::ForwardVelocityToAccel;
            minimum = true;
            return true;
        }
        if (name == "--forward-velocity-to-accel-gain-max")
        {
            index = GainIndex::ForwardVelocityToAccel;
            minimum = false;
            return true;
        }
        if (name == "--forward-acceleration-error-gain-min")
        {
            index = GainIndex::ForwardAccelerationError;
            minimum = true;
            return true;
        }
        if (name == "--forward-acceleration-error-gain-max")
        {
            index = GainIndex::ForwardAccelerationError;
            minimum = false;
            return true;
        }
        if (name == "--yaw-position-to-accel-gain-min")
        {
            index = GainIndex::YawPositionToAccel;
            minimum = true;
            return true;
        }
        if (name == "--yaw-position-to-accel-gain-max")
        {
            index = GainIndex::YawPositionToAccel;
            minimum = false;
            return true;
        }
        if (name == "--yaw-velocity-to-accel-gain-min")
        {
            index = GainIndex::YawVelocityToAccel;
            minimum = true;
            return true;
        }
        if (name == "--yaw-velocity-to-accel-gain-max")
        {
            index = GainIndex::YawVelocityToAccel;
            minimum = false;
            return true;
        }
        if (name == "--yaw-acceleration-error-gain-min")
        {
            index = GainIndex::YawAccelerationError;
            minimum = true;
            return true;
        }
        if (name == "--yaw-acceleration-error-gain-max")
        {
            index = GainIndex::YawAccelerationError;
            minimum = false;
            return true;
        }
        return false;
    }

    static bool ApplyGainOption(const std::string& name, float value, GainSet& gains) noexcept
    {
        GainIndex index = GainIndex::Count;
        if (!ResolveGainOption(name, index))
        {
            return false;
        }

        value = std::isfinite(value) ? (std::max)(0.0f, value) : 0.0f;
        switch (index)
        {
        case GainIndex::ForwardPositionToAccel:
            gains.forwardPositionToAccelGain = value;
            break;
        case GainIndex::ForwardVelocityToAccel:
            gains.forwardVelocityToAccelGain = value;
            break;
        case GainIndex::ForwardAccelerationError:
            gains.forwardAccelerationErrorGain = value;
            break;
        case GainIndex::YawPositionToAccel:
            gains.yawPositionToAccelGain = value;
            break;
        case GainIndex::YawVelocityToAccel:
            gains.yawVelocityToAccelGain = value;
            break;
        case GainIndex::YawAccelerationError:
            gains.yawAccelerationErrorGain = value;
            break;
        case GainIndex::Count:
        default:
            return false;
        }
        return true;
    }

    static bool ApplySearchBoundOption(const std::string& name, float value, Options& options) noexcept
    {
        GainIndex index = GainIndex::Count;
        bool minimum = false;
        if (!ResolveSearchBoundOption(name, index, minimum))
        {
            return false;
        }

        const std::size_t rawIndex = static_cast<std::size_t>(index);
        value = std::isfinite(value) ? (std::max)(0.0f, value) : 0.0f;
        if (minimum)
        {
            options.searchRanges[rawIndex].minimum = value;
            options.searchMinimumSpecified[rawIndex] = true;
        }
        else
        {
            options.searchRanges[rawIndex].maximum = value;
            options.searchMaximumSpecified[rawIndex] = true;
        }
        return true;
    }

    static void PrintUsage()
    {
        std::cout
            << "PdTuning evaluates DriveBaseTrackingTuning candidates against the current C++ PlantModel.\n\n"
            << "Usage:\n"
            << "  PdTuning.exe [gain options]\n"
            << "  PdTuning.exe --search [--search-passes N] [--search-points N] [search bounds]\n\n"
            << "Gain options default to Config::kDriveBaseTrackingTuning named gain values:\n"
            << "  --forward-position-to-accel-gain V      --forward-velocity-to-accel-gain V\n"
            << "  --forward-acceleration-error-gain V\n"
            << "  --yaw-position-to-accel-gain V          --yaw-velocity-to-accel-gain V\n"
            << "  --yaw-acceleration-error-gain V\n\n"
            << "Search bounds use matching min/max option pairs:\n"
            << "  --forward-position-to-accel-gain-min V  --forward-position-to-accel-gain-max V\n"
            << "  --forward-velocity-to-accel-gain-min V  --forward-velocity-to-accel-gain-max V\n"
            << "  --forward-acceleration-error-gain-min V --forward-acceleration-error-gain-max V\n"
            << "  --yaw-position-to-accel-gain-min V      --yaw-position-to-accel-gain-max V\n"
            << "  --yaw-velocity-to-accel-gain-min V      --yaw-velocity-to-accel-gain-max V\n"
            << "  --yaw-acceleration-error-gain-min V     --yaw-acceleration-error-gain-max V\n\n"
            << "Search uses log-spaced positive gain values with zero retained as an explicit candidate.\n\n"
            << "Stdout is JSON. Errors and this help text are not part of optimizer output.\n";
    }

    static GainSet ExtractProductionTunedGains() noexcept
    {
        GainSet gains{};
        const MazeMap::DriveAxisTrackingTuning& forward =
            MazeMap::Config::kDriveBaseTrackingTuning.ForwardAxis();
        const MazeMap::DriveAxisTrackingTuning& yaw =
            MazeMap::Config::kDriveBaseTrackingTuning.YawAxis();
        gains.forwardPositionToAccelGain = forward.PositionErrorToAccelerationGain();
        gains.forwardVelocityToAccelGain = forward.VelocityErrorToAccelerationGain();
        gains.forwardAccelerationErrorGain = forward.AccelerationErrorToAccelerationGain();
        gains.yawPositionToAccelGain = yaw.PositionErrorToAccelerationGain();
        gains.yawVelocityToAccelGain = yaw.VelocityErrorToAccelerationGain();
        gains.yawAccelerationErrorGain = yaw.AccelerationErrorToAccelerationGain();
        return gains;
    }

    static MazeMap::DriveBaseTrackingTuning BuildCandidateTrackingTuning(const GainSet& gains) noexcept
    {
        return MazeMap::DriveBaseTrackingTuning(
            MazeMap::DriveAxisTrackingTuning(
                gains.forwardPositionToAccelGain,
                gains.forwardVelocityToAccelGain,
                gains.forwardAccelerationErrorGain),
            MazeMap::DriveAxisTrackingTuning(
                gains.yawPositionToAccelGain,
                gains.yawVelocityToAccelGain,
                gains.yawAccelerationErrorGain));
    }

    static void RebuildRuntimeDriveBaseForCandidate(
        MazeMap::App::Internal::SharedRobotRuntime& runtime,
        const MazeMap::DriveBaseTrackingTuning& trackingTuning) noexcept
    {
        // Tool-only candidate injection: keep Drive attached to the runtime-owned DriveBase address.
        MazeMap::DriveBase& driveBase = runtime.DriveBase();
        driveBase.~DriveBase();
        (void)::new (static_cast<void*>(&driveBase))
            MazeMap::DriveBase(runtime.Plant(), runtime.RuntimeState(), trackingTuning);
        runtime.DriveBase().ClearCommandEvidence();
    }

    static MotionLimits MakePrimitiveLimits() noexcept
    {
        MotionLimits limits{};
        limits.SetMaxSpeedMps(0.35f);
        limits.SetAccelMps2(3.0f);
        limits.SetDecelMps2(3.0f);
        limits.SetMaxAngularSpeedRadps(6.0f);
        limits.SetAngularAccelRadps2(30.0f);
        limits.SetAngleToleranceRad(MazeMap::Config::kAngleToleranceRad);
        return limits;
    }

    static float GetGain(const GainSet& gains, GainIndex index) noexcept
    {
        switch (index)
        {
        case GainIndex::ForwardPositionToAccel:
            return gains.forwardPositionToAccelGain;
        case GainIndex::ForwardVelocityToAccel:
            return gains.forwardVelocityToAccelGain;
        case GainIndex::ForwardAccelerationError:
            return gains.forwardAccelerationErrorGain;
        case GainIndex::YawPositionToAccel:
            return gains.yawPositionToAccelGain;
        case GainIndex::YawVelocityToAccel:
            return gains.yawVelocityToAccelGain;
        case GainIndex::YawAccelerationError:
            return gains.yawAccelerationErrorGain;
        case GainIndex::Count:
        default:
            return 0.0f;
        }
    }

    static void SetGain(GainSet& gains, GainIndex index, float value) noexcept
    {
        value = std::isfinite(value) ? (std::max)(0.0f, value) : 0.0f;
        switch (index)
        {
        case GainIndex::ForwardPositionToAccel:
            gains.forwardPositionToAccelGain = value;
            break;
        case GainIndex::ForwardVelocityToAccel:
            gains.forwardVelocityToAccelGain = value;
            break;
        case GainIndex::ForwardAccelerationError:
            gains.forwardAccelerationErrorGain = value;
            break;
        case GainIndex::YawPositionToAccel:
            gains.yawPositionToAccelGain = value;
            break;
        case GainIndex::YawVelocityToAccel:
            gains.yawVelocityToAccelGain = value;
            break;
        case GainIndex::YawAccelerationError:
            gains.yawAccelerationErrorGain = value;
            break;
        case GainIndex::Count:
        default:
            break;
        }
    }

    ScenarioSpec MakeScenarioSpec(
        const char* name,
        const char* description,
        SignalKind signal,
        float durationS,
        float tolerance,
        float initialForwardMps,
        float initialYawRateRadps,
        float initialYawRad,
        float targetForwardMps,
        float targetYawRateRadps,
        float targetForwardAccelMps2,
        float targetYawAccelRadps2,
        float targetYawRad) noexcept
    {
        ScenarioSpec spec{};
        spec.name = name;
        spec.description = description;
        spec.signal = signal;
        spec.durationS = durationS;
        spec.tolerance = tolerance;
        spec.initialForwardMps = initialForwardMps;
        spec.initialYawRateRadps = initialYawRateRadps;
        spec.initialYawRad = initialYawRad;
        spec.targetForwardMps = targetForwardMps;
        spec.targetYawRateRadps = targetYawRateRadps;
        spec.targetForwardAccelMps2 = targetForwardAccelMps2;
        spec.targetYawAccelRadps2 = targetYawAccelRadps2;
        spec.targetYawRad = targetYawRad;
        return spec;
    }

    static std::array<ScenarioSpec, 7> BuildScenarioSpecs() noexcept
    {
        return {{
            MakeScenarioSpec(
                "forward_launch_0_to_4_mps",
                "Zero-to-maximum forward-speed capture; exercises the 4 m/s operating jump and should drive 12+ m/s^2 plant response without ringing.",
                SignalKind::ForwardVelocity,
                1.25f,
                0.050f,
                0.0f,
                0.0f,
                0.0f,
                4.0f,
                0.0f,
                QuietNaN(),
                QuietNaN(),
                0.0f),
            MakeScenarioSpec(
                "forward_brake_4_to_0_mps",
                "Maximum-speed forward capture back to zero; exposes braking saturation, overshoot, and low-speed settling oscillation after a 4 m/s step.",
                SignalKind::ForwardVelocity,
                1.25f,
                0.050f,
                4.0f,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                QuietNaN(),
                QuietNaN(),
                0.0f),
            MakeScenarioSpec(
                "forward_high_speed_disturbance_4_to_2_mps",
                "Nonzero-speed target disturbance at the race envelope; checks whether velocity feedback damps a high-speed correction instead of hunting.",
                SignalKind::ForwardVelocity,
                1.00f,
                0.050f,
                4.0f,
                0.0f,
                0.0f,
                2.0f,
                0.0f,
                QuietNaN(),
                QuietNaN(),
                0.0f),
            MakeScenarioSpec(
                "yaw_rate_max_effort_0_to_9_radps",
                "In-place maximum-rate turn capture using the vehicle's 9 rad/s turn-rate envelope; high yaw-rate error should hit aggressive turn authority.",
                SignalKind::YawRate,
                1.10f,
                0.120f,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                9.0f,
                QuietNaN(),
                QuietNaN(),
                QuietNaN()),
            MakeScenarioSpec(
                "yaw_rate_reversal_9_to_minus9_radps",
                "Maximum-rate in-place reversal; stresses saturation recovery and repeated zero-crossings in the yaw-rate loop.",
                SignalKind::YawRate,
                1.40f,
                0.150f,
                0.0f,
                9.0f,
                0.0f,
                0.0f,
                -9.0f,
                QuietNaN(),
                QuietNaN(),
                QuietNaN()),
            MakeScenarioSpec(
                "combined_3mps_16p5mps2_turn",
                "High-speed arc command at 3 m/s and 5.5 rad/s, representing 16.5 m/s^2 kinematic lateral acceleration with velocity hold active.",
                SignalKind::YawRate,
                1.50f,
                0.120f,
                3.0f,
                0.0f,
                0.0f,
                3.0f,
                5.5f,
                QuietNaN(),
                QuietNaN(),
                QuietNaN()),
            MakeScenarioSpec(
                "combined_3mps_heading_correction_15deg",
                "High-speed heading capture while holding 3 m/s forward speed; exercises heading/yaw-rate damping without changing the production controller path.",
                SignalKind::Heading,
                1.50f,
                0.012f,
                3.0f,
                0.0f,
                15.0f * DEG_TO_RAD_F,
                3.0f,
                0.0f,
                QuietNaN(),
                QuietNaN(),
                0.0f)
        }};
    }

    static float ScenarioTarget(const ScenarioSpec& spec) noexcept
    {
        switch (spec.signal)
        {
        case SignalKind::ForwardVelocity:
            return spec.targetForwardMps;
        case SignalKind::YawRate:
            return spec.targetYawRateRadps;
        case SignalKind::Heading:
            return spec.targetYawRad;
        default:
            return 0.0f;
        }
    }

    static float ScenarioInitial(const ScenarioSpec& spec) noexcept
    {
        switch (spec.signal)
        {
        case SignalKind::ForwardVelocity:
            return spec.initialForwardMps;
        case SignalKind::YawRate:
            return spec.initialYawRateRadps;
        case SignalKind::Heading:
            return spec.initialYawRad;
        default:
            return 0.0f;
        }
    }

    static float ScenarioScale(const ScenarioSpec& spec) noexcept
    {
        const float target = ScenarioTarget(spec);
        const float initial = ScenarioInitial(spec);
        const float excursion = (std::max)(
            std::fabs(target - initial),
            (std::max)(std::fabs(target), spec.tolerance * 10.0f));
        return (std::max)(excursion, 1.0e-6f);
    }

    static float TargetKinematicLateralAccelMps2(const ScenarioSpec& spec) noexcept
    {
        return
            (std::isfinite(spec.targetForwardMps) && std::isfinite(spec.targetYawRateRadps)) ?
            (spec.targetForwardMps * spec.targetYawRateRadps) :
            QuietNaN();
    }

    static bool HasLinearStepTransition(float initial, float target) noexcept
    {
        return
            std::isfinite(initial) &&
            std::isfinite(target) &&
            (std::fabs(target - initial) > kStepTransitionEpsilon);
    }

    static bool HasHeadingStepTransition(float initialYawRad, float targetYawRad) noexcept
    {
        return
            std::isfinite(initialYawRad) &&
            std::isfinite(targetYawRad) &&
            (std::fabs(NormalizeAngle(targetYawRad - initialYawRad)) > kStepTransitionEpsilon);
    }

    static bool HasFiniteForwardAccelerationObjective(const DriveTelemetry& telemetry) noexcept
    {
        return std::isfinite(telemetry.composedForwardAccelMps2);
    }

    static bool HasFiniteYawAccelerationObjective(const DriveTelemetry& telemetry) noexcept
    {
        return std::isfinite(telemetry.composedYawAccelRadps2);
    }

    const char* AccelerationMetricDefinition(
        int samples,
        int objectiveSamples,
        int fallbackSamples) noexcept
    {
        if (samples <= 0)
        {
            return "inactive";
        }
        if ((objectiveSamples > 0) && (fallbackSamples > 0))
        {
            return "composed_objective_tracking_with_inactive_fallback";
        }
        if (objectiveSamples > 0)
        {
            return "composed_objective_tracking";
        }
        return "inactive_or_nonfinite_objective_fallback";
    }

    static double AxisStepScale(float initial, float target, float tolerance) noexcept
    {
        const double excursion =
            (std::isfinite(initial) && std::isfinite(target)) ?
            static_cast<double>(std::fabs(target - initial)) :
            0.0;
        const double targetMagnitude =
            std::isfinite(target) ? static_cast<double>(std::fabs(target)) : 0.0;
        const double toleranceScale =
            std::isfinite(tolerance) ? static_cast<double>(std::fabs(tolerance) * 10.0f) : 0.0;
        return (std::max)((std::max)(excursion, targetMagnitude), (std::max)(toleranceScale, 1.0e-6));
    }

    static double RmsOrZero(double rms, double scale) noexcept
    {
        if (!std::isfinite(rms) || !std::isfinite(scale) || (scale <= 0.0))
        {
            return 0.0;
        }
        return rms / scale;
    }

    static double AccelerationRmsScale(double objectiveRms, double fallbackScale) noexcept
    {
        const double objectiveScale = std::isfinite(objectiveRms) ? objectiveRms : 0.0;
        return (std::max)((std::max)(objectiveScale, fallbackScale), 1.0e-6);
    }

    static int ErrorSign(float error, float deadband) noexcept
    {
        if (!std::isfinite(error))
        {
            return 0;
        }
        if (error > deadband)
        {
            return 1;
        }
        if (error < -deadband)
        {
            return -1;
        }
        return 0;
    }

    static bool VehicleStateIsFinite(const MazeMap::VehicleState& state) noexcept
    {
        return
            std::isfinite(state.GetPositionX()) &&
            std::isfinite(state.GetPositionY()) &&
            std::isfinite(state.GetHeading()) &&
            std::isfinite(state.GetForwardVelocity()) &&
            std::isfinite(state.GetRightwardVelocity()) &&
            std::isfinite(state.GetYawRate()) &&
            std::isfinite(state.GetWheelSpeedLeft()) &&
            std::isfinite(state.GetWheelSpeedRight());
    }

    static float SignalValue(const MazeMap::VehicleState& state, SignalKind signal) noexcept
    {
        switch (signal)
        {
        case SignalKind::ForwardVelocity:
            return state.GetForwardVelocity();
        case SignalKind::YawRate:
            return state.GetYawRate();
        case SignalKind::Heading:
            return state.GetHeading();
        default:
            return 0.0f;
        }
    }

    static float SignalError(const ScenarioSpec& spec, const MazeMap::VehicleState& state) noexcept
    {
        const float target = ScenarioTarget(spec);
        const float value = SignalValue(state, spec.signal);
        if (spec.signal == SignalKind::Heading)
        {
            return NormalizeAngle(target - value);
        }
        return target - value;
    }

    static MazeMap::VehicleState BuildInitialState(const ScenarioSpec& spec) noexcept
    {
        MazeMap::VehicleState state{};
        state.SetHeading(spec.initialYawRad);
        state.SetForwardVelocity(spec.initialForwardMps);
        state.SetYawRate(spec.initialYawRateRadps);
        float leftWheelSpeedRadps = 0.0f;
        float rightWheelSpeedRadps = 0.0f;
        MazeMap::Vehicle::WheelSpeedsFromBodyVelocity(
            spec.initialForwardMps,
            spec.initialYawRateRadps,
            leftWheelSpeedRadps,
            rightWheelSpeedRadps);
        state.PublishEncoderWheelSpeedsRadps(leftWheelSpeedRadps, rightWheelSpeedRadps);
        return state;
    }

    static void PublishTruthToRuntime(
        MazeMap::VehicleState& runtimeState,
        const MazeMap::VehicleState& truth,
        const WheelObservationState& wheels,
        float leftDistanceDeltaM,
        float rightDistanceDeltaM,
        float dtSeconds,
        bool advanceTime) noexcept
    {
        SensorSnapshot snapshot{};
        snapshot.SetRawYawRateRadps(truth.GetYawRate());
        snapshot.SetYawRateRadps(truth.GetYawRate());
        SensorSnapshot::EncoderObs encoderObservation = SensorSnapshot{}.EncoderObservation();
        encoderObservation.SetLeftDistanceDeltaM(leftDistanceDeltaM);
        encoderObservation.SetRightDistanceDeltaM(rightDistanceDeltaM);
        encoderObservation.SetLeftVelocityMps(MazeMap::Vehicle::WheelLinearVelocityFromWheelSpeed(truth.GetWheelSpeedLeft()));
        encoderObservation.SetRightVelocityMps(MazeMap::Vehicle::WheelLinearVelocityFromWheelSpeed(truth.GetWheelSpeedRight()));
        encoderObservation.SetLeftWheelSpeedRadps(truth.GetWheelSpeedLeft());
        encoderObservation.SetRightWheelSpeedRadps(truth.GetWheelSpeedRight());
        snapshot.PublishEncoderObservation(
            encoderObservation,
            true,
            snapshot.LeftEncoderTotalCounts(),
            snapshot.RightEncoderTotalCounts(),
            wheels.leftDistanceM,
            wheels.rightDistanceM);

        runtimeState.SetPosition(truth.GetPosition());
        runtimeState.SetHeading(truth.GetHeading());
        runtimeState.SetForwardVelocity(truth.GetForwardVelocity());
        runtimeState.SetRightwardVelocity(truth.GetRightwardVelocity());
        runtimeState.SetYawRate(truth.GetYawRate());
        runtimeState.PublishEncoderWheelSpeedsRadps(truth.GetWheelSpeedLeft(), truth.GetWheelSpeedRight());
        runtimeState.SetForwardAcceleration(0.0f);
        runtimeState.SetRightAcceleration(0.0f);
        runtimeState.SetYawAccel(0.0f);
        if (advanceTime)
        {
            runtimeState.SetTime(runtimeState.GetTime() + dtSeconds);
        }
        runtimeState.SetTimestampUs(static_cast<std::uint32_t>(runtimeState.GetTime() * 1000000.0f));
        runtimeState.SetSensorSnapshot(snapshot);
    }

    static WheelObservationState IntegratePlantCommandForTick(
        MazeMap::PlantModel& plant,
        MazeMap::VehicleState& runtimeState,
        const MazeMap::App::Internal::CommandVector& control) noexcept
    {
        WheelObservationState tickDelta{};
        for (int substep = 0; substep < kPlantIntegrationSubstepsPerTick; ++substep)
        {
            const float previousLeftWheelSpeedRadps = runtimeState.GetWheelSpeedLeft();
            const float previousRightWheelSpeedRadps = runtimeState.GetWheelSpeedRight();
            plant.integrate(control, kPlantIntegrationSubstepSeconds);

            tickDelta.leftDistanceM +=
                0.5f *
                (previousLeftWheelSpeedRadps + runtimeState.GetWheelSpeedLeft()) *
                MazeMap::Vehicle::GetDriveWheelRadiusM() *
                kPlantIntegrationSubstepSeconds;
            tickDelta.rightDistanceM +=
                0.5f *
                (previousRightWheelSpeedRadps + runtimeState.GetWheelSpeedRight()) *
                MazeMap::Vehicle::GetDriveWheelRadiusM() *
                kPlantIntegrationSubstepSeconds;
        }
        return tickDelta;
    }

    static bool AdvanceTruth(
        MazeMap::PlantModel& plant,
        MazeMap::VehicleState& runtimeState,
        WheelObservationState& wheels,
        const MazeMap::App::Internal::CommandVector& control) noexcept
    {
        const WheelObservationState tickDelta =
            IntegratePlantCommandForTick(plant, runtimeState, control);
        wheels.leftDistanceM += tickDelta.leftDistanceM;
        wheels.rightDistanceM += tickDelta.rightDistanceM;
        PublishTruthToRuntime(
            runtimeState,
            runtimeState,
            wheels,
            tickDelta.leftDistanceM,
            tickDelta.rightDistanceM,
            kTickSeconds,
            false);
        return VehicleStateIsFinite(runtimeState);
    }

    static MazeMap::VehicleState BuildAcceptanceTruthState(
        const float forwardMps,
        const float yawRad) noexcept
    {
        MazeMap::VehicleState state{};
        state.SetHeading(yawRad);
        state.SetForwardVelocity(forwardMps);
        state.PublishEncoderWheelSpeedsRadps(
            MazeMap::Vehicle::WheelSpeedFromLinearVelocity(forwardMps),
            MazeMap::Vehicle::WheelSpeedFromLinearVelocity(forwardMps));
        return state;
    }

    static float AverageEncoderDistanceM(const WheelObservationState& wheels) noexcept
    {
        return 0.5f * (wheels.leftDistanceM + wheels.rightDistanceM);
    }

    static SensorSnapshot BuildDriveManeuverSensorSnapshot(const float yawRateRadps = 0.0f) noexcept
    {
        SensorSnapshot snapshot{};
        snapshot.SetRawYawRateRadps(yawRateRadps);
        snapshot.SetYawRateRadps(yawRateRadps);
        return snapshot;
    }

    static int32_t ConsumeWholeEncoderCounts(const float deltaCounts, float& remainderCounts) noexcept
    {
        remainderCounts += deltaCounts;
        const int32_t wholeCounts =
            (remainderCounts >= 0.0f) ?
            static_cast<int32_t>(std::floor(remainderCounts)) :
            static_cast<int32_t>(std::ceil(remainderCounts));
        remainderCounts -= static_cast<float>(wholeCounts);
        return wholeCounts;
    }

    static bool ApplyEncoderObservation(
        MazeMap::App::Internal::SharedRobotRuntime& runtime,
        const float leftDistanceDeltaM,
        const float rightDistanceDeltaM,
        const float yawRateRadps,
        float& leftEncoderRemainderCounts,
        float& rightEncoderRemainderCounts,
        const float dtSeconds,
        const MazeMap::App::Internal::CommandVector& appliedControl)
    {
        const float distancePerCountM = MazeMap::Vehicle::DriveEncoderDistanceFromCounts(1);
        const int32_t leftCounts =
            ConsumeWholeEncoderCounts(leftDistanceDeltaM / distancePerCountM, leftEncoderRemainderCounts);
        const int32_t rightCounts =
            ConsumeWholeEncoderCounts(rightDistanceDeltaM / distancePerCountM, rightEncoderRemainderCounts);

        MazeMap::VehicleState& runtimeState = runtime.RuntimeState();
        SensorSnapshot snapshot = BuildDriveManeuverSensorSnapshot(yawRateRadps);
        SensorSnapshot::EncoderObs encoderObservation = snapshot.EncoderObservation();
        encoderObservation.SetTotalLeftCounts(leftCounts);
        encoderObservation.SetTotalRightCounts(rightCounts);
        encoderObservation.SetLeftDistanceDeltaM(static_cast<float>(leftCounts) * distancePerCountM);
        encoderObservation.SetRightDistanceDeltaM(static_cast<float>(rightCounts) * distancePerCountM);
        if ((dtSeconds > 0.0f) && std::isfinite(dtSeconds))
        {
            const float invDtSeconds = 1.0f / dtSeconds;
            encoderObservation.SetLeftVelocityMps(encoderObservation.LeftDistanceDeltaM() * invDtSeconds);
            encoderObservation.SetRightVelocityMps(encoderObservation.RightDistanceDeltaM() * invDtSeconds);
            encoderObservation.SetLeftWheelSpeedRadps(MazeMap::Vehicle::WheelSpeedFromLinearVelocity(encoderObservation.LeftVelocityMps()));
            encoderObservation.SetRightWheelSpeedRadps(MazeMap::Vehicle::WheelSpeedFromLinearVelocity(encoderObservation.RightVelocityMps()));
        }
        const std::int64_t leftEncoderTotalCounts =
            runtimeState.GetSensorSnapshot().LeftEncoderTotalCounts() + static_cast<std::int64_t>(leftCounts);
        const std::int64_t rightEncoderTotalCounts =
            runtimeState.GetSensorSnapshot().RightEncoderTotalCounts() + static_cast<std::int64_t>(rightCounts);
        snapshot.PublishEncoderObservation(
            encoderObservation,
            true,
            leftEncoderTotalCounts,
            rightEncoderTotalCounts,
            MazeMap::Vehicle::DriveEncoderDistanceFromCounts(leftEncoderTotalCounts),
            MazeMap::Vehicle::DriveEncoderDistanceFromCounts(rightEncoderTotalCounts));

        runtimeState.SetSensorSnapshot(snapshot);
        if (std::isfinite(dtSeconds) && (dtSeconds > 0.0f))
        {
            runtimeState.SetTime(runtimeState.GetTime() + dtSeconds);
            runtimeState.SetTimestampUs(static_cast<std::uint32_t>(runtimeState.GetTime() * 1000000.0f));
        }

        MazeMap::Estimator& estimator = runtime.Estimator();
        if (estimator.HasFault())
        {
            return false;
        }

        if (std::isfinite(dtSeconds) &&
            (dtSeconds > 0.0f) &&
            !estimator.predict(dtSeconds, appliedControl))
        {
            return false;
        }

        if (std::isfinite(snapshot.YawRateRadps()))
        {
            if (!estimator.updateYawRate(snapshot.YawRateRadps()))
            {
                return false;
            }
        }

        MazeMap::ImuAccelObs accelObservation{};
        (void)estimator.updatePlanarAccel(accelObservation);
        return !estimator.HasFault();
    }

    static void PrimeDriveForSmoothEntry(
        MazeMap::App::Internal::SharedRobotRuntime& runtime,
        MazeMap::VehicleState& truth,
        float& leftEncoderRemainderCounts,
        float& rightEncoderRemainderCounts)
    {
        truth = BuildAcceptanceTruthState(kSmoothManeuverEntrySpeedMps, 0.0f);

        const float distancePerCountM = MazeMap::Vehicle::DriveEncoderDistanceFromCounts(1);
        float projectedLeftEncoderRemainderCounts = leftEncoderRemainderCounts;
        float projectedRightEncoderRemainderCounts = rightEncoderRemainderCounts;
        const int32_t projectedLeftCounts = ConsumeWholeEncoderCounts(
            (kSmoothManeuverEntrySpeedMps * kTickSeconds) / distancePerCountM,
            projectedLeftEncoderRemainderCounts);
        const int32_t projectedRightCounts = ConsumeWholeEncoderCounts(
            (kSmoothManeuverEntrySpeedMps * kTickSeconds) / distancePerCountM,
            projectedRightEncoderRemainderCounts);
        const float projectedForwardDistanceM =
            0.5f * static_cast<float>(projectedLeftCounts + projectedRightCounts) * distancePerCountM;

        (void)runtime.Estimator().ResetPose(0.0f, -projectedForwardDistanceM, 0.0f);
        MazeMap::VehicleState& runtimeState = runtime.RuntimeState();
        runtimeState.SetForwardVelocity(kSmoothManeuverEntrySpeedMps);
        runtimeState.PublishEncoderWheelSpeedsRadps(
            MazeMap::Vehicle::WheelSpeedFromLinearVelocity(kSmoothManeuverEntrySpeedMps),
            MazeMap::Vehicle::WheelSpeedFromLinearVelocity(kSmoothManeuverEntrySpeedMps));
        (void)ApplyEncoderObservation(
            runtime,
            kSmoothManeuverEntrySpeedMps * kTickSeconds,
            kSmoothManeuverEntrySpeedMps * kTickSeconds,
            0.0f,
            leftEncoderRemainderCounts,
            rightEncoderRemainderCounts,
            kTickSeconds,
            MazeMap::App::Internal::CommandVector::Brake());
    }

    static bool AdvanceRuntimeDriveCycle(
        MazeMap::App::Internal::SharedRobotRuntime& runtime,
        MazeMap::PlantModel& truthPlant,
        MazeMap::VehicleState& truth,
        WheelObservationState& wheels,
        float& leftEncoderRemainderCounts,
        float& rightEncoderRemainderCounts,
        const MazeMap::App::Internal::CommandVector& control)
    {
        const WheelObservationState tickDelta =
            IntegratePlantCommandForTick(truthPlant, truth, control);
        wheels.leftDistanceM += tickDelta.leftDistanceM;
        wheels.rightDistanceM += tickDelta.rightDistanceM;

        return ApplyEncoderObservation(
            runtime,
            tickDelta.leftDistanceM,
            tickDelta.rightDistanceM,
            truth.GetYawRate(),
            leftEncoderRemainderCounts,
            rightEncoderRemainderCounts,
            kTickSeconds,
            control);
    }

    static void RecordAcceptanceTelemetry(
        AcceptanceMetrics& metrics,
        const MazeMap::App::Internal::CommandVector& control,
        const DriveTelemetry& telemetry) noexcept
    {
        metrics.allControlsFinite = metrics.allControlsFinite && control.IsFinite();
        metrics.commandEvidenceValid =
            metrics.commandEvidenceValid &&
            ((telemetry.commandKindFlags & DriveTelemetry::kCommandKindBodyProposal) != 0U) &&
            ((telemetry.telemetryValidFlags & DriveTelemetry::kTelemetryCommandEvidenceValid) != 0U) &&
            ((telemetry.telemetryValidFlags & DriveTelemetry::kTelemetryPlantCommandValid) != 0U) &&
            (std::fabs(control.LeftCommand() - telemetry.leftDriveCommand) <= 1.0e-5f) &&
            (std::fabs(control.RightCommand() - telemetry.rightDriveCommand) <= 1.0e-5f);
        metrics.requestedObjectivesFinite =
            metrics.requestedObjectivesFinite &&
            std::isfinite(telemetry.requestedForwardMps) &&
            std::isfinite(telemetry.requestedYawRateRadps) &&
            std::isfinite(telemetry.requestedForwardAccelMps2) &&
            std::isfinite(telemetry.requestedYawAccelRadps2) &&
            std::isfinite(telemetry.requestedYawRad);
        metrics.solverClean =
            metrics.solverClean &&
            ((telemetry.commandKindFlags & DriveTelemetry::kCommandKindSolverFailureEvidence) == 0U);
        if ((telemetry.commandKindFlags & DriveTelemetry::kCommandKindSolverFailureEvidence) != 0U)
        {
            ++metrics.solverFailureCount;
        }
        if (!control.IsFinite())
        {
            ++metrics.nonFiniteCount;
        }
    }

    static void CaptureAcceptanceFinalState(
        AcceptanceMetrics& metrics,
        const MazeMap::VehicleState& truth,
        const WheelObservationState& wheels) noexcept
    {
        const bool truthFinite = VehicleStateIsFinite(truth);
        metrics.truthFinite = metrics.truthFinite && truthFinite;
        if (!truthFinite)
        {
            ++metrics.nonFiniteCount;
        }
        metrics.encoderAverageDistanceM = AverageEncoderDistanceM(wheels);
        metrics.finalXM = truth.GetPositionX();
        metrics.finalYM = truth.GetPositionY();
        metrics.finalYawRad = truth.GetHeading();
        metrics.elapsedSeconds = static_cast<float>(metrics.appliedTicks) * kTickSeconds;
    }

    static double ComputeRmsRatioScore(float rmsError, float rmsReference) noexcept
    {
        if (!std::isfinite(rmsError) || !std::isfinite(rmsReference) || !(rmsReference > 0.0f))
        {
            return 0.0;
        }

        const double ratio = static_cast<double>(rmsError) / static_cast<double>(rmsReference);
        return ratio * ratio;
    }

    static double ComputeManeuverTrackingRmsScore(const AcceptanceMetrics& metrics) noexcept
    {
        return
            ComputeRmsRatioScore(metrics.distanceRmsErrorM, metrics.distanceRmsReferenceM) +
            ComputeRmsRatioScore(metrics.headingRmsErrorRad, metrics.headingRmsReferenceRad) +
            ComputeRmsRatioScore(metrics.forwardVelocityRmsErrorMps, metrics.forwardVelocityRmsReferenceMps) +
            ComputeRmsRatioScore(metrics.yawRateRmsErrorRadps, metrics.yawRateRmsReferenceRadps) +
            ComputeRmsRatioScore(metrics.forwardAccelRmsErrorMps2, metrics.forwardAccelRmsReferenceMps2) +
            ComputeRmsRatioScore(metrics.yawAccelRmsErrorRadps2, metrics.yawAccelRmsReferenceRadps2);
    }

    static double ComputeAcceptancePenalty(const AcceptanceMetrics& metrics) noexcept
    {
        if (metrics.metric == "maneuver_tracking_rms")
        {
            return ComputeManeuverTrackingRmsScore(metrics);
        }

        return 0.0;
    }

    static bool IsPdTuningInformationalAcceptanceMetric(const AcceptanceMetrics& metrics) noexcept
    {
        return
            (metrics.metric == "in_place_time") ||
            (metrics.metric == "in_place_shift") ||
            (metrics.metric == "in_place_heading") ||
            (metrics.metric == "smooth_velocity_variation") ||
            (metrics.metric == "smooth_yaw_acceleration_variation") ||
            (metrics.metric == "smooth_yaw_rate_variation") ||
            (metrics.metric == "smooth_final_position") ||
            (metrics.metric == "smooth_final_heading");
    }

    static bool IsNormalizedRippleOscillatory(float normalizedVariation) noexcept
    {
        return std::isfinite(normalizedVariation) && (normalizedVariation > kRippleOscillationDeadband);
    }

    static bool IsAcceptanceRippleOscillatory(const AcceptanceMetrics& metrics) noexcept
    {
        if (metrics.metric == "smooth_velocity_variation")
        {
            return IsNormalizedRippleOscillatory(metrics.velocityVariation);
        }
        if (metrics.metric == "smooth_yaw_acceleration_variation")
        {
            return IsNormalizedRippleOscillatory(metrics.yawAccelerationVariation);
        }
        if (metrics.metric == "smooth_yaw_rate_variation")
        {
            return IsNormalizedRippleOscillatory(metrics.yawRateVariation);
        }
        return false;
    }

    static void FinalizeAcceptance(AcceptanceMetrics& metrics, const bool acceptanceCondition) noexcept
    {
        metrics.passed =
            metrics.started &&
            acceptanceCondition &&
            metrics.allControlsFinite &&
            metrics.commandEvidenceValid &&
            metrics.requestedObjectivesFinite &&
            metrics.truthFinite &&
            metrics.solverClean &&
            (metrics.nonFiniteCount == 0);
        metrics.blocker = !metrics.passed && !IsPdTuningInformationalAcceptanceMetric(metrics);
        metrics.rippleOscillatory = IsAcceptanceRippleOscillatory(metrics);
        metrics.scorePenalty =
            IsPdTuningInformationalAcceptanceMetric(metrics) ? 0.0 : ComputeAcceptancePenalty(metrics);
    }

    static AcceptanceMetrics RunStartStraightAcceptance(const GainSet& gains)
    {
        MazeMap::DriveBaseTrackingTuning trackingTuning = BuildCandidateTrackingTuning(gains);
        MazeMap::App::Internal::SharedRobotRuntime runtime(kTickSeconds);
        RebuildRuntimeDriveBaseForCandidate(runtime, trackingTuning);
        ScopedFanDuty fanDuty(runtime.Vehicle(), kFanDuty);

        WheelObservationState wheels{};
        PublishTruthToRuntime(
            runtime.RuntimeState(),
            BuildAcceptanceTruthState(0.0f, 0.0f),
            wheels,
            0.0f,
            0.0f,
            0.0f,
            true);

        AcceptanceMetrics metrics{};
        metrics.name = "drive_primitive_start_straight_completes";
        metrics.description =
            "Drive::StartStraight closed-loop acceptance through SharedRobotRuntime, Drive, DriveBase, PlantModel::integrate, and sensor snapshot publication.";
        metrics.path = "SharedRobotRuntime.DriveService.StartStraight";
        metrics.code = "StartStraight";
        metrics.metric = "drive_primitive_completion";
        metrics.started = true;
        metrics.maxTicks = kStartStraightMaxTicks;
        metrics.targetDistanceM = kStartStraightDistanceM;

        MazeMap::App::Internal::Drive& drive = runtime.DriveService();
        drive.SetOperationMode(MazeMap::App::Internal::Drive::OperationMode::OpenFloor);
        drive.SetLimits(MakePrimitiveLimits());
        drive.StartStraight(kStartStraightDistanceM, kStartStraightCruiseMps, 0.0f);

        for (int tick = 0; tick < kStartStraightMaxTicks; ++tick)
        {
            bool done = false;
            const MazeMap::App::Internal::CommandVector control = drive.GetNextControls(done);
            if (done)
            {
                metrics.completed = true;
                break;
            }

            RecordAcceptanceTelemetry(metrics, control, runtime.DriveBase().LastTelemetry());
            const bool advanced = AdvanceTruth(runtime.Plant(), runtime.RuntimeState(), wheels, control);
            ++metrics.appliedTicks;
            metrics.truthFinite = metrics.truthFinite && advanced && VehicleStateIsFinite(runtime.RuntimeState());
            if (!advanced || !VehicleStateIsFinite(runtime.RuntimeState()))
            {
                ++metrics.nonFiniteCount;
                break;
            }
        }

        CaptureAcceptanceFinalState(metrics, runtime.RuntimeState(), wheels);
        FinalizeAcceptance(metrics, metrics.completed);
        return metrics;
    }

    static MazeMap::DirectionalLocation BuildManeuverStart() noexcept
    {
        return MazeMap::DirectionalLocation(MazeMap::MazeLocation(0U, 0U), MazeMap::Up);
    }

    static MazeMap::DirectionalLocation BuildNominalEndLocation(const MazeMap::ManeuverCode code)
    {
        return MazeMap::ManeuverSet::GetSet().Move(code, BuildManeuverStart());
    }

    static float BuildNominalEndXMeters(const MazeMap::ManeuverCode code)
    {
        const MazeMap::DirectionalLocation nominalEnd = BuildNominalEndLocation(code);
        return 0.5f * MazeMap::Config::kCellSizeM * static_cast<float>(nominalEnd.GetLocation().GetX());
    }

    static float BuildNominalEndYMeters(const MazeMap::ManeuverCode code)
    {
        const MazeMap::DirectionalLocation nominalEnd = BuildNominalEndLocation(code);
        return 0.5f * MazeMap::Config::kCellSizeM * static_cast<float>(nominalEnd.GetLocation().GetY());
    }

    static float BuildNominalEndYawRad(const MazeMap::ManeuverCode code)
    {
        return DirectionToYawRad(BuildNominalEndLocation(code).GetDirection());
    }

    static const char* ManeuverCodeLabel(const MazeMap::ManeuverCode code) noexcept
    {
        switch (code)
        {
        case MazeMap::IP45:
            return "IP45";
        case MazeMap::IP90:
            return "IP90";
        case MazeMap::IP135:
            return "IP135";
        case MazeMap::IP180:
            return "IP180";
        case MazeMap::S45LS:
            return "S45LS";
        case MazeMap::S45LD:
            return "S45LD";
        case MazeMap::S45SS:
            return "S45SS";
        case MazeMap::S45SD:
            return "S45SD";
        case MazeMap::S90LS:
            return "S90LS";
        case MazeMap::S90SS:
            return "S90SS";
        case MazeMap::S90SD:
            return "S90SD";
        case MazeMap::S135SS:
            return "S135SS";
        case MazeMap::S135SD:
            return "S135SD";
        case MazeMap::S135LS:
            return "S135LS";
        case MazeMap::S135LD:
            return "S135LD";
        case MazeMap::S180LS:
            return "S180LS";
        case MazeMap::S180SS:
            return "S180SS";
        default:
            return "UNKNOWN";
        }
    }

    static std::string LowercaseCopy(std::string text)
    {
        for (char& ch : text)
        {
            ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
        }
        return text;
    }

    static float ComputeInPlaceTurnKinematicTimeSeconds(
        const float angleRad,
        const MotionLimits& limits) noexcept
    {
        return limits.ComputeMinimumTurnDurationSeconds(angleRad);
    }

    static void RecordManeuverTraceTelemetry(
        ManeuverAcceptanceTrace& trace,
        const MazeMap::App::Internal::CommandVector& control,
        const DriveTelemetry& telemetry) noexcept
    {
        trace.allControlsFinite = trace.allControlsFinite && control.IsFinite();
        trace.commandEvidenceValid =
            trace.commandEvidenceValid &&
            ((telemetry.commandKindFlags & DriveTelemetry::kCommandKindBodyProposal) != 0U) &&
            ((telemetry.telemetryValidFlags & DriveTelemetry::kTelemetryCommandEvidenceValid) != 0U) &&
            ((telemetry.telemetryValidFlags & DriveTelemetry::kTelemetryPlantCommandValid) != 0U) &&
            (std::fabs(control.LeftCommand() - telemetry.leftDriveCommand) <= 1.0e-5f) &&
            (std::fabs(control.RightCommand() - telemetry.rightDriveCommand) <= 1.0e-5f);
        trace.requestedObjectivesFinite =
            trace.requestedObjectivesFinite &&
            std::isfinite(telemetry.requestedForwardMps) &&
            std::isfinite(telemetry.requestedYawRateRadps) &&
            std::isfinite(telemetry.requestedForwardAccelMps2) &&
            std::isfinite(telemetry.requestedYawAccelRadps2) &&
            std::isfinite(telemetry.requestedYawRad);
        trace.solverClean =
            trace.solverClean &&
            ((telemetry.commandKindFlags & DriveTelemetry::kCommandKindSolverFailureEvidence) == 0U);
        if ((telemetry.commandKindFlags & DriveTelemetry::kCommandKindSolverFailureEvidence) != 0U)
        {
            ++trace.solverFailureCount;
        }
        if (!control.IsFinite())
        {
            ++trace.nonFiniteCount;
        }
    }

    static float UseFiniteTelemetryValueOrZero(float value) noexcept
    {
        return std::isfinite(value) ? value : 0.0f;
    }

    static void RecordManeuverTrackingSample(
        ManeuverAcceptanceTrace& trace,
        const DriveTelemetry& telemetry,
        const float previousForwardMps,
        const float previousYawRateRadps) noexcept
    {
        const float requestedForwardMps = UseFiniteTelemetryValueOrZero(telemetry.requestedForwardMps);
        const float requestedYawRateRadps = UseFiniteTelemetryValueOrZero(telemetry.requestedYawRateRadps);
        const float expectedForwardAccelMps2 = UseFiniteTelemetryValueOrZero(telemetry.composedForwardAccelMps2);
        const float expectedYawAccelRadps2 = UseFiniteTelemetryValueOrZero(telemetry.composedYawAccelRadps2);

        trace.expectedDistanceM += requestedForwardMps * kTickSeconds;
        trace.expectedHeadingRad =
            std::isfinite(telemetry.requestedYawRad) ?
            telemetry.requestedYawRad :
            NormalizeAngle(trace.expectedHeadingRad + (requestedYawRateRadps * kTickSeconds));

        const float actualDistanceM = AverageEncoderDistanceM(trace.wheels);
        const float actualHeadingRad = trace.truth.GetHeading();
        const float actualForwardMps = trace.truth.GetForwardVelocity();
        const float actualYawRateRadps = trace.truth.GetYawRate();
        const float actualForwardAccelMps2 = (actualForwardMps - previousForwardMps) / kTickSeconds;
        const float actualYawAccelRadps2 = (actualYawRateRadps - previousYawRateRadps) / kTickSeconds;

        trace.samples.push_back(
            AcceptanceCommandSample{
                trace.elapsedSeconds,
                requestedForwardMps,
                requestedYawRateRadps,
                trace.expectedDistanceM,
                trace.expectedDistanceM - actualDistanceM,
                trace.expectedHeadingRad,
                AngleErrorRad(trace.expectedHeadingRad, actualHeadingRad),
                requestedForwardMps - actualForwardMps,
                requestedYawRateRadps - actualYawRateRadps,
                expectedForwardAccelMps2,
                expectedForwardAccelMps2 - actualForwardAccelMps2,
                expectedYawAccelRadps2,
                expectedYawAccelRadps2 - actualYawAccelRadps2
            });
    }

    ManeuverAcceptanceTrace SimulateDriveManeuverAcceptance(
        const GainSet& gains,
        const MazeMap::ManeuverCode code,
        const bool smoothTurn)
    {
        MazeMap::DriveBaseTrackingTuning trackingTuning = BuildCandidateTrackingTuning(gains);
        MazeMap::App::Internal::SharedRobotRuntime runtime(kTickSeconds);
        RebuildRuntimeDriveBaseForCandidate(runtime, trackingTuning);
        ScopedFanDuty fanDuty(runtime.Vehicle(), kFanDuty);

        float leftEncoderRemainderCounts = 0.0f;
        float rightEncoderRemainderCounts = 0.0f;
        ManeuverAcceptanceTrace trace{};

        (void)runtime.Estimator().ResetPose(0.0f, 0.0f, 0.0f);
        if (smoothTurn)
        {
            PrimeDriveForSmoothEntry(
                runtime,
                trace.truth,
                leftEncoderRemainderCounts,
                rightEncoderRemainderCounts);
        }
        else
        {
            trace.truth = BuildAcceptanceTruthState(0.0f, 0.0f);
            PublishTruthToRuntime(
                runtime.RuntimeState(),
                trace.truth,
                trace.wheels,
                0.0f,
                0.0f,
                0.0f,
                true);
        }
        MazeMap::PlantModel truthPlant(runtime.Vehicle(), trace.truth);

        MazeMap::ManeuverInstance maneuver(
            code,
            BuildManeuverStart(),
            smoothTurn ? kSmoothManeuverEntrySpeedMps : 0.0f,
            smoothTurn ? kSmoothManeuverEntrySpeedMps : 0.0f);

        MazeMap::App::Internal::Drive& drive = runtime.DriveService();
        drive.StartManeuver(maneuver);
        trace.started = true;

        for (int tick = 0; tick < kDriveManeuverMaxTicks; ++tick)
        {
            if (runtime.Estimator().HasFault())
            {
                trace.solverClean = false;
                break;
            }

            bool done = false;
            const MazeMap::App::Internal::CommandVector control = drive.GetNextControls(done);
            if (done)
            {
                trace.completed = true;
                break;
            }

            const DriveTelemetry telemetry = runtime.DriveBase().LastTelemetry();
            const float previousForwardMps = trace.truth.GetForwardVelocity();
            const float previousYawRateRadps = trace.truth.GetYawRate();
            RecordManeuverTraceTelemetry(trace, control, telemetry);
            const bool advanced = AdvanceRuntimeDriveCycle(
                runtime,
                truthPlant,
                trace.truth,
                trace.wheels,
                leftEncoderRemainderCounts,
                rightEncoderRemainderCounts,
                control);
            ++trace.appliedTicks;
            trace.elapsedSeconds = static_cast<float>(trace.appliedTicks) * kTickSeconds;
            trace.truthFinite = trace.truthFinite && advanced && VehicleStateIsFinite(trace.truth);
            if (!advanced || !VehicleStateIsFinite(trace.truth))
            {
                ++trace.nonFiniteCount;
                break;
            }
            RecordManeuverTrackingSample(trace, telemetry, previousForwardMps, previousYawRateRadps);
        }

        const SensorSnapshot& finalSnapshot = runtime.RuntimeState().GetSensorSnapshot();
        trace.wheels.leftDistanceM = finalSnapshot.LeftEncoderDistanceM();
        trace.wheels.rightDistanceM = finalSnapshot.RightEncoderDistanceM();
        return trace;
    }

    static double ComputeNormalizedSpan(const std::vector<float>& values) noexcept
    {
        if (values.size() < 2U)
        {
            return (std::numeric_limits<double>::quiet_NaN)();
        }

        const auto minmax = std::minmax_element(values.begin(), values.end());
        const double average =
            static_cast<double>(std::accumulate(values.begin(), values.end(), 0.0f)) /
            static_cast<double>(values.size());
        if (!(average > 0.0) || !std::isfinite(average))
        {
            return (std::numeric_limits<double>::quiet_NaN)();
        }

        return static_cast<double>(*minmax.second - *minmax.first) / average;
    }

    static std::vector<float> CollectLinearCommandMagnitudes(const ManeuverAcceptanceTrace& trace)
    {
        std::vector<float> magnitudes;
        magnitudes.reserve(trace.samples.size());
        for (const AcceptanceCommandSample& sample : trace.samples)
        {
            magnitudes.push_back(std::fabs(sample.linearCommandMps));
        }
        return magnitudes;
    }

    static std::vector<float> CollectTurnYawRateMagnitudes(const ManeuverAcceptanceTrace& trace)
    {
        std::vector<float> magnitudes;
        if (trace.samples.empty())
        {
            return magnitudes;
        }

        float maxYawRateMagnitudeRadps = 0.0f;
        for (const AcceptanceCommandSample& sample : trace.samples)
        {
            maxYawRateMagnitudeRadps =
                (std::max)(maxYawRateMagnitudeRadps, std::fabs(sample.angularCommandRadps));
        }

        const float plateauThresholdRadps = kManeuverTurnPlateauFraction * maxYawRateMagnitudeRadps;
        for (const AcceptanceCommandSample& sample : trace.samples)
        {
            const float magnitudeRadps = std::fabs(sample.angularCommandRadps);
            if (magnitudeRadps >= plateauThresholdRadps)
            {
                magnitudes.push_back(magnitudeRadps);
            }
        }

        return magnitudes;
    }

    static void AppendTrimmedRampYawAccelMagnitudes(
        const ManeuverAcceptanceTrace& trace,
        std::vector<float>& magnitudes,
        const std::size_t deltaBeginIndex,
        const std::size_t deltaEndIndexExclusive)
    {
        if (deltaBeginIndex >= deltaEndIndexExclusive)
        {
            return;
        }

        const std::size_t regionLength = deltaEndIndexExclusive - deltaBeginIndex;
        if (regionLength <= (2U * kManeuverRampDeltaTrimSamples))
        {
            return;
        }

        const std::size_t trimmedBeginIndex = deltaBeginIndex + kManeuverRampDeltaTrimSamples;
        const std::size_t trimmedEndIndexExclusive = deltaEndIndexExclusive - kManeuverRampDeltaTrimSamples;
        for (std::size_t index = trimmedBeginIndex; index < trimmedEndIndexExclusive; ++index)
        {
            const float previousYawRateMagnitudeRadps =
                std::fabs(trace.samples[index - 1U].angularCommandRadps);
            const float currentYawRateMagnitudeRadps =
                std::fabs(trace.samples[index].angularCommandRadps);
            if ((previousYawRateMagnitudeRadps <= kManeuverYawRateMagnitudeEpsilonRadps) ||
                (currentYawRateMagnitudeRadps <= kManeuverYawRateMagnitudeEpsilonRadps))
            {
                continue;
            }

            const float dtSeconds = trace.samples[index].timeSeconds - trace.samples[index - 1U].timeSeconds;
            if (!(dtSeconds > 0.0f))
            {
                continue;
            }

            magnitudes.push_back(
                std::fabs(
                    (trace.samples[index].angularCommandRadps - trace.samples[index - 1U].angularCommandRadps) /
                    dtSeconds));
        }
    }

    static std::vector<float> CollectRampYawAccelMagnitudes(const ManeuverAcceptanceTrace& trace)
    {
        std::vector<float> magnitudes;
        if (trace.samples.size() < 3U)
        {
            return magnitudes;
        }

        float maxYawRateMagnitudeRadps = 0.0f;
        std::size_t plateauBeginIndex = trace.samples.size();
        std::size_t plateauEndIndex = 0U;
        for (std::size_t index = 0U; index < trace.samples.size(); ++index)
        {
            maxYawRateMagnitudeRadps =
                (std::max)(
                    maxYawRateMagnitudeRadps,
                    std::fabs(trace.samples[index].angularCommandRadps));
        }

        const float plateauThresholdRadps = kManeuverTurnPlateauFraction * maxYawRateMagnitudeRadps;
        for (std::size_t index = 0U; index < trace.samples.size(); ++index)
        {
            if (std::fabs(trace.samples[index].angularCommandRadps) >= plateauThresholdRadps)
            {
                plateauBeginIndex = (std::min)(plateauBeginIndex, index);
                plateauEndIndex = index;
            }
        }

        if (plateauBeginIndex == trace.samples.size())
        {
            return magnitudes;
        }

        AppendTrimmedRampYawAccelMagnitudes(trace, magnitudes, 1U, plateauBeginIndex + 1U);
        AppendTrimmedRampYawAccelMagnitudes(trace, magnitudes, plateauEndIndex + 1U, trace.samples.size());
        return magnitudes;
    }

    static float ComputeSampleRms(
        const std::vector<AcceptanceCommandSample>& samples,
        float AcceptanceCommandSample::* member) noexcept
    {
        double squaredSum = 0.0;
        int count = 0;
        for (const AcceptanceCommandSample& sample : samples)
        {
            const float value = sample.*member;
            if (!std::isfinite(value))
            {
                continue;
            }
            squaredSum += static_cast<double>(value) * static_cast<double>(value);
            ++count;
        }

        return (count > 0) ?
            static_cast<float>(std::sqrt(squaredSum / static_cast<double>(count))) :
            (std::numeric_limits<float>::quiet_NaN)();
    }

    static float ResolveRmsReference(float requestedRms, float floorValue) noexcept
    {
        return (std::max)(
            std::isfinite(requestedRms) ? std::fabs(requestedRms) : 0.0f,
            floorValue);
    }

    static AcceptanceMetrics BuildManeuverAcceptanceBase(
        const ManeuverAcceptanceTrace& trace,
        const MazeMap::ManeuverCode code,
        const bool smoothTurn,
        const char* metricSuffix,
        const char* metricName)
    {
        AcceptanceMetrics metrics{};
        metrics.name =
            LowercaseCopy(
                std::string("drive_maneuver_") +
                ManeuverCodeLabel(code) +
                "_" +
                metricSuffix);
        metrics.description =
            std::string(smoothTurn ? "Smooth" : "In-place") +
            " Drive::StartManeuver release-test metric through SharedRobotRuntime, Drive, DriveBase, PlantModel::integrate, and estimator sensor updates.";
        metrics.path = "SharedRobotRuntime.DriveService.StartManeuver";
        metrics.code = ManeuverCodeLabel(code);
        metrics.metric = metricName;
        metrics.started = trace.started;
        metrics.completed = trace.completed;
        metrics.maxTicks = kDriveManeuverMaxTicks;
        metrics.appliedTicks = trace.appliedTicks;
        metrics.elapsedSeconds = trace.elapsedSeconds;
        metrics.targetDistanceM =
            smoothTurn ?
            MazeMap::ManeuverSet::GetSet().GetTravelDistanceMeters(code, MazeMap::Config::kCellSizeM) :
            0.0f;
        metrics.targetYawRad = BuildNominalEndYawRad(code);
        metrics.headingToleranceRad = kManeuverHeadingToleranceRad;
        metrics.headingToleranceDeg = kManeuverHeadingToleranceRad * RAD_TO_DEG_F;
        metrics.positionToleranceM =
            smoothTurn ? kSmoothManeuverPositionToleranceM : kInPlaceManeuverPositionToleranceM;
        metrics.allControlsFinite = trace.allControlsFinite;
        metrics.commandEvidenceValid = trace.commandEvidenceValid;
        metrics.requestedObjectivesFinite = trace.requestedObjectivesFinite;
        metrics.truthFinite = trace.truthFinite;
        metrics.solverClean = trace.solverClean;
        metrics.solverFailureCount = trace.solverFailureCount;
        metrics.nonFiniteCount = trace.nonFiniteCount;
        CaptureAcceptanceFinalState(metrics, trace.truth, trace.wheels);
        metrics.finalHeadingErrorRad =
            std::fabs(AngleErrorRad(metrics.targetYawRad, metrics.finalYawRad));
        metrics.finalHeadingErrorDeg = metrics.finalHeadingErrorRad * RAD_TO_DEG_F;
        metrics.finalPositionErrorM =
            std::hypot(
                metrics.finalXM - BuildNominalEndXMeters(code),
                metrics.finalYM - BuildNominalEndYMeters(code));
        metrics.shiftDistanceM = std::hypot(metrics.finalXM, metrics.finalYM);
        metrics.shiftToleranceM = kInPlaceManeuverPositionToleranceM;
        metrics.trackingSampleCount = static_cast<int>(trace.samples.size());
        metrics.distanceRmsErrorM =
            ComputeSampleRms(trace.samples, &AcceptanceCommandSample::distanceErrorM);
        metrics.distanceRmsReferenceM =
            ResolveRmsReference(
                ComputeSampleRms(trace.samples, &AcceptanceCommandSample::expectedDistanceM),
                kTrackingDistanceScaleFloorM);
        metrics.headingRmsErrorRad =
            ComputeSampleRms(trace.samples, &AcceptanceCommandSample::headingErrorRad);
        metrics.headingRmsErrorDeg = metrics.headingRmsErrorRad * RAD_TO_DEG_F;
        metrics.headingRmsReferenceRad =
            ResolveRmsReference(
                ComputeSampleRms(trace.samples, &AcceptanceCommandSample::expectedHeadingRad),
                kTrackingHeadingScaleFloorRad);
        metrics.forwardVelocityRmsErrorMps =
            ComputeSampleRms(trace.samples, &AcceptanceCommandSample::forwardVelocityErrorMps);
        metrics.forwardVelocityRmsReferenceMps =
            ResolveRmsReference(
                ComputeSampleRms(trace.samples, &AcceptanceCommandSample::linearCommandMps),
                kTrackingForwardVelocityScaleFloorMps);
        metrics.yawRateRmsErrorRadps =
            ComputeSampleRms(trace.samples, &AcceptanceCommandSample::yawRateErrorRadps);
        metrics.yawRateRmsReferenceRadps =
            ResolveRmsReference(
                ComputeSampleRms(trace.samples, &AcceptanceCommandSample::angularCommandRadps),
                kTrackingYawRateScaleFloorRadps);
        metrics.forwardAccelRmsErrorMps2 =
            ComputeSampleRms(trace.samples, &AcceptanceCommandSample::forwardAccelErrorMps2);
        metrics.forwardAccelRmsReferenceMps2 =
            ResolveRmsReference(
                ComputeSampleRms(trace.samples, &AcceptanceCommandSample::expectedForwardAccelMps2),
                kTrackingForwardAccelScaleFloorMps2);
        metrics.yawAccelRmsErrorRadps2 =
            ComputeSampleRms(trace.samples, &AcceptanceCommandSample::yawAccelErrorRadps2);
        metrics.yawAccelRmsReferenceRadps2 =
            ResolveRmsReference(
                ComputeSampleRms(trace.samples, &AcceptanceCommandSample::expectedYawAccelRadps2),
                kTrackingYawAccelScaleFloorRadps2);
        return metrics;
    }

    static AcceptanceMetrics MakeCompletionAcceptance(
        const ManeuverAcceptanceTrace& trace,
        const MazeMap::ManeuverCode code,
        const bool smoothTurn)
    {
        AcceptanceMetrics metrics =
            BuildManeuverAcceptanceBase(trace, code, smoothTurn, "completes", "completion");
        FinalizeAcceptance(metrics, metrics.completed && !trace.samples.empty());
        return metrics;
    }

    static AcceptanceMetrics MakeCommandEvidenceAcceptance(
        const ManeuverAcceptanceTrace& trace,
        const MazeMap::ManeuverCode code,
        const bool smoothTurn)
    {
        AcceptanceMetrics metrics =
            BuildManeuverAcceptanceBase(trace, code, smoothTurn, "command_evidence_matches_returned_command", "command_evidence");
        FinalizeAcceptance(
            metrics,
            metrics.completed &&
            !trace.samples.empty() &&
            metrics.allControlsFinite &&
            metrics.commandEvidenceValid &&
            metrics.requestedObjectivesFinite &&
            metrics.truthFinite);
        return metrics;
    }

    static AcceptanceMetrics MakeManeuverTrackingRmsAcceptance(
        const ManeuverAcceptanceTrace& trace,
        const MazeMap::ManeuverCode code,
        const bool smoothTurn)
    {
        AcceptanceMetrics metrics =
            BuildManeuverAcceptanceBase(trace, code, smoothTurn, "tracking_rms", "maneuver_tracking_rms");
        FinalizeAcceptance(
            metrics,
            metrics.completed &&
            !trace.samples.empty() &&
            std::isfinite(metrics.distanceRmsErrorM) &&
            std::isfinite(metrics.headingRmsErrorRad) &&
            std::isfinite(metrics.forwardVelocityRmsErrorMps) &&
            std::isfinite(metrics.yawRateRmsErrorRadps) &&
            std::isfinite(metrics.forwardAccelRmsErrorMps2) &&
            std::isfinite(metrics.yawAccelRmsErrorRadps2));
        return metrics;
    }

    static AcceptanceMetrics MakeInPlaceShiftAcceptance(
        const ManeuverAcceptanceTrace& trace,
        const MazeMap::ManeuverCode code)
    {
        AcceptanceMetrics metrics =
            BuildManeuverAcceptanceBase(trace, code, false, "shift_acceptance", "in_place_shift");
        FinalizeAcceptance(
            metrics,
            metrics.completed &&
            std::isfinite(metrics.shiftDistanceM) &&
            (metrics.shiftDistanceM < kInPlaceManeuverPositionToleranceM));
        return metrics;
    }

    static AcceptanceMetrics MakeInPlaceHeadingAcceptance(
        const ManeuverAcceptanceTrace& trace,
        const MazeMap::ManeuverCode code)
    {
        AcceptanceMetrics metrics =
            BuildManeuverAcceptanceBase(trace, code, false, "heading_acceptance", "in_place_heading");
        FinalizeAcceptance(
            metrics,
            metrics.completed &&
            std::isfinite(metrics.finalHeadingErrorRad) &&
            (metrics.finalHeadingErrorRad <= kManeuverHeadingToleranceRad));
        return metrics;
    }

    static AcceptanceMetrics MakeInPlaceTimeAcceptance(
        const ManeuverAcceptanceTrace& trace,
        const MazeMap::ManeuverCode code)
    {
        AcceptanceMetrics metrics =
            BuildManeuverAcceptanceBase(trace, code, false, "time_acceptance", "in_place_time");
        MazeMap::App::Internal::SharedRobotRuntime runtime(kTickSeconds);
        metrics.expectedElapsedSeconds =
            ComputeInPlaceTurnKinematicTimeSeconds(
                std::fabs(BuildNominalEndYawRad(code)),
                runtime.DriveService().GetLimits());
        metrics.elapsedRelativeTolerance = kManeuverTimeToleranceFraction;
        metrics.elapsedRelativeError =
            (metrics.expectedElapsedSeconds > 0.0f) ?
            (std::fabs(metrics.elapsedSeconds - metrics.expectedElapsedSeconds) / metrics.expectedElapsedSeconds) :
            (std::numeric_limits<float>::quiet_NaN)();
        FinalizeAcceptance(
            metrics,
            metrics.completed &&
            std::isfinite(metrics.elapsedRelativeError) &&
            (metrics.elapsedRelativeError <= kManeuverTimeToleranceFraction));
        return metrics;
    }

    static AcceptanceMetrics MakeSmoothVelocityVariationAcceptance(
        const ManeuverAcceptanceTrace& trace,
        const MazeMap::ManeuverCode code)
    {
        AcceptanceMetrics metrics =
            BuildManeuverAcceptanceBase(trace, code, true, "velocity_variation_acceptance", "smooth_velocity_variation");
        metrics.velocityVariation = static_cast<float>(ComputeNormalizedSpan(CollectLinearCommandMagnitudes(trace)));
        metrics.velocityVariationLimit = kSmoothManeuverVelocityVariationLimit;
        FinalizeAcceptance(
            metrics,
            metrics.completed &&
            std::isfinite(metrics.velocityVariation) &&
            (metrics.velocityVariation < kSmoothManeuverVelocityVariationLimit));
        return metrics;
    }

    static AcceptanceMetrics MakeSmoothYawAccelerationVariationAcceptance(
        const ManeuverAcceptanceTrace& trace,
        const MazeMap::ManeuverCode code)
    {
        AcceptanceMetrics metrics =
            BuildManeuverAcceptanceBase(trace, code, true, "yaw_acceleration_variation_acceptance", "smooth_yaw_acceleration_variation");
        metrics.yawAccelerationVariation =
            static_cast<float>(ComputeNormalizedSpan(CollectRampYawAccelMagnitudes(trace)));
        metrics.yawAccelerationVariationLimit = kSmoothManeuverYawAccelerationVariationLimit;
        FinalizeAcceptance(
            metrics,
            metrics.completed &&
            std::isfinite(metrics.yawAccelerationVariation) &&
            (metrics.yawAccelerationVariation < kSmoothManeuverYawAccelerationVariationLimit));
        return metrics;
    }

    static AcceptanceMetrics MakeSmoothYawRateVariationAcceptance(
        const ManeuverAcceptanceTrace& trace,
        const MazeMap::ManeuverCode code)
    {
        AcceptanceMetrics metrics =
            BuildManeuverAcceptanceBase(trace, code, true, "yaw_rate_variation_acceptance", "smooth_yaw_rate_variation");
        metrics.yawRateVariation = static_cast<float>(ComputeNormalizedSpan(CollectTurnYawRateMagnitudes(trace)));
        metrics.yawRateVariationLimit = kSmoothManeuverYawRateVariationLimit;
        FinalizeAcceptance(
            metrics,
            metrics.completed &&
            std::isfinite(metrics.yawRateVariation) &&
            (metrics.yawRateVariation < kSmoothManeuverYawRateVariationLimit));
        return metrics;
    }

    static AcceptanceMetrics MakeSmoothFinalPositionAcceptance(
        const ManeuverAcceptanceTrace& trace,
        const MazeMap::ManeuverCode code)
    {
        AcceptanceMetrics metrics =
            BuildManeuverAcceptanceBase(trace, code, true, "final_position_acceptance", "smooth_final_position");
        FinalizeAcceptance(
            metrics,
            metrics.completed &&
            std::isfinite(metrics.finalPositionErrorM) &&
            (metrics.finalPositionErrorM <= kSmoothManeuverPositionToleranceM));
        return metrics;
    }

    static AcceptanceMetrics MakeSmoothFinalHeadingAcceptance(
        const ManeuverAcceptanceTrace& trace,
        const MazeMap::ManeuverCode code)
    {
        AcceptanceMetrics metrics =
            BuildManeuverAcceptanceBase(trace, code, true, "final_heading_acceptance", "smooth_final_heading");
        FinalizeAcceptance(
            metrics,
            metrics.completed &&
            std::isfinite(metrics.finalHeadingErrorRad) &&
            (metrics.finalHeadingErrorRad <= kManeuverHeadingToleranceRad));
        return metrics;
    }

    static double ComputeScenarioScore(const ScenarioMetrics& metrics) noexcept
    {
        const double excursion = (std::max)(
            static_cast<double>(std::fabs(metrics.target - metrics.initial)),
            (std::max)(static_cast<double>(std::fabs(metrics.target)), static_cast<double>(metrics.tolerance * 10.0f)));
        const double scale = (std::max)(excursion, 1.0e-6);
        const double duration = (std::max)(static_cast<double>(metrics.durationS), 1.0e-6);
        const double finalTerm = std::fabs(metrics.finalError) / scale;
        const double iaeTerm = metrics.integratedAbsoluteError / (scale * duration);
        const double iseTerm = metrics.integratedSquaredError / (scale * scale * duration);
        const double overshootTerm = static_cast<double>(metrics.overshoot) / scale;
        const double latePeakToPeakTerm = static_cast<double>(metrics.lateWindowPeakToPeakError) / scale;
        const double lateRmsTerm = metrics.lateWindowRmsError / scale;
        const double minimumErrorTerm = static_cast<double>(metrics.minimumAbsError) / scale;
        const double settlingTerm = metrics.settled ? (metrics.settlingTimeS / metrics.durationS) : 2.0;
        const double velocityStepScale =
            AxisStepScale(metrics.initialForwardMps, metrics.targetForwardMps, metrics.tolerance);
        const double yawRateStepScale =
            AxisStepScale(metrics.initialYawRateRadps, metrics.targetYawRateRadps, metrics.tolerance);
        const double velocityStepRmsTerm =
            RmsOrZero(metrics.first500VelocityErrorRms, velocityStepScale);
        const double yawRateStepRmsTerm =
            RmsOrZero(metrics.first500YawRateErrorRms, yawRateStepScale);
        const double forwardAccelFallbackScale = velocityStepScale / kAccelerationStepRmsWindowSeconds;
        const double yawAccelFallbackScale = yawRateStepScale / kAccelerationStepRmsWindowSeconds;
        const double forwardAccelStepRmsTerm =
            RmsOrZero(
                metrics.first100ForwardAccelErrorRms,
                AccelerationRmsScale(metrics.first100ForwardAccelObjectiveRms, forwardAccelFallbackScale));
        const double yawAccelStepRmsTerm =
            RmsOrZero(
                metrics.first100YawAccelErrorRms,
                AccelerationRmsScale(metrics.first100YawAccelObjectiveRms, yawAccelFallbackScale));
        const double saturationTerm =
            static_cast<double>(metrics.commandSaturationFraction) +
            (0.5 * static_cast<double>(metrics.plantClipRequestFraction));
        const double failureTerm =
            metrics.failed ? 100000.0 :
            (1000.0 * static_cast<double>(metrics.nonFiniteCount + metrics.solverFailureCount));

        return
            (3.0 * finalTerm) +
            (8.0 * iaeTerm) +
            (2.0 * iseTerm) +
            (2.0 * overshootTerm) +
            (4.0 * latePeakToPeakTerm) +
            (6.0 * lateRmsTerm) +
            (12.0 * minimumErrorTerm) +
            (kVelocityStepRmsScoreWeight * velocityStepRmsTerm) +
            (kYawRateStepRmsScoreWeight * yawRateStepRmsTerm) +
            (kForwardAccelStepRmsScoreWeight * forwardAccelStepRmsTerm) +
            (kYawAccelStepRmsScoreWeight * yawAccelStepRmsTerm) +
            settlingTerm +
            (2.0 * saturationTerm) +
            metrics.responseFailurePenalty +
            metrics.oscillationPenalty +
            failureTerm;
    }

    static double ComputeResponseFailurePenalty(const ScenarioMetrics& metrics) noexcept
    {
        const double excursion = (std::max)(
            static_cast<double>(std::fabs(metrics.target - metrics.initial)),
            (std::max)(static_cast<double>(std::fabs(metrics.target)), static_cast<double>(metrics.tolerance * 10.0f)));
        const double scale = (std::max)(excursion, 1.0e-6);
        const double minimumErrorTerm = static_cast<double>(metrics.minimumAbsError) / scale;
        const double poorResponseExcess = (std::max)(0.0, minimumErrorTerm - 0.60);

        return 500000.0 * poorResponseExcess * poorResponseExcess;
    }

    static bool IsScenarioOscillatory(const ScenarioMetrics& metrics) noexcept
    {
        const double excursion = (std::max)(
            static_cast<double>(std::fabs(metrics.target - metrics.initial)),
            (std::max)(static_cast<double>(std::fabs(metrics.target)), static_cast<double>(metrics.tolerance * 10.0f)));
        const double scale = (std::max)(excursion, 1.0e-6);
        const double latePeakToPeakLimit =
            (std::max)(static_cast<double>(metrics.tolerance) * 4.0, scale * 0.04);
        const double lateRmsLimit =
            (std::max)(static_cast<double>(metrics.tolerance) * 1.5, scale * 0.015);
        const bool repeatedCrossing =
            metrics.signChangesAfterFirstCrossing >= kOscillationSignChangeThreshold;
        const bool lateRinging =
            (static_cast<double>(metrics.lateWindowPeakToPeakError) > latePeakToPeakLimit) &&
            (metrics.lateWindowRmsError > lateRmsLimit);

        return
            (metrics.signChangesAfterFirstCrossing >= (2 * kOscillationSignChangeThreshold)) ||
            (repeatedCrossing && lateRinging);
    }

    static double ComputeOscillationPenalty(const ScenarioMetrics& metrics) noexcept
    {
        const double excursion = (std::max)(
            static_cast<double>(std::fabs(metrics.target - metrics.initial)),
            (std::max)(static_cast<double>(std::fabs(metrics.target)), static_cast<double>(metrics.tolerance * 10.0f)));
        const double scale = (std::max)(excursion, 1.0e-6);
        const double latePeakToPeakTerm = static_cast<double>(metrics.lateWindowPeakToPeakError) / scale;
        const double lateRmsTerm = metrics.lateWindowRmsError / scale;
        const double signChangeTerm = static_cast<double>(metrics.signChangesAfterFirstCrossing);
        const double flagPenalty = metrics.oscillatory ? 2500.0 : 0.0;

        return
            flagPenalty +
            (75.0 * signChangeTerm) +
            (80.0 * latePeakToPeakTerm) +
            (120.0 * lateRmsTerm);
    }

    static ScenarioMetrics RunScenario(const ScenarioSpec& spec, const GainSet& gains)
    {
        MazeMap::Vehicle vehicle{};
        vehicle.SetFanDuty(kFanDuty);
        MazeMap::VehicleState runtimeState = BuildInitialState(spec);
        MazeMap::PlantModel plant(vehicle, runtimeState);
        MazeMap::DriveBaseTrackingTuning trackingTuning = BuildCandidateTrackingTuning(gains);
        MazeMap::DriveBase driveBase(plant, runtimeState, trackingTuning);

        WheelObservationState wheels{};
        PublishTruthToRuntime(runtimeState, runtimeState, wheels, 0.0f, 0.0f, 0.0f, true);
        driveBase.ClearCommandEvidence();

        ScenarioMetrics metrics{};
        metrics.name = spec.name;
        metrics.description = spec.description;
        metrics.signal = SignalName(spec.signal);
        metrics.target = ScenarioTarget(spec);
        metrics.initial = ScenarioInitial(spec);
        metrics.tolerance = spec.tolerance;
        metrics.durationS = spec.durationS;
        metrics.initialForwardMps = spec.initialForwardMps;
        metrics.initialYawRateRadps = spec.initialYawRateRadps;
        metrics.initialYawRad = spec.initialYawRad;
        metrics.targetForwardMps = spec.targetForwardMps;
        metrics.targetYawRateRadps = spec.targetYawRateRadps;
        metrics.targetForwardAccelMps2 = spec.targetForwardAccelMps2;
        metrics.targetYawAccelRadps2 = spec.targetYawAccelRadps2;
        metrics.targetYawRad = spec.targetYawRad;
        metrics.targetKinematicLateralAccelMps2 = TargetKinematicLateralAccelMps2(spec);
        metrics.forwardVelocityStepActive = HasLinearStepTransition(spec.initialForwardMps, spec.targetForwardMps);
        metrics.yawRateStepActive = HasLinearStepTransition(spec.initialYawRateRadps, spec.targetYawRateRadps);
        metrics.headingStepActive = HasHeadingStepTransition(spec.initialYawRad, spec.targetYawRad);
        metrics.forwardAccelStepMetricActive = metrics.forwardVelocityStepActive;
        metrics.yawAccelStepMetricActive = metrics.yawRateStepActive || metrics.headingStepActive;
        metrics.settlingTimeS = spec.durationS;

        const float initialError = SignalError(spec, runtimeState);
        const float initialAbsError = std::fabs(initialError);
        metrics.minimumAbsError = initialAbsError;
        metrics.maxAbsForwardVelocityMps = std::fabs(runtimeState.GetForwardVelocity());
        metrics.maxAbsYawRateRadps = std::fabs(runtimeState.GetYawRate());
        metrics.maxAbsKinematicLateralAccelMps2 =
            std::fabs(runtimeState.GetForwardVelocity() * runtimeState.GetYawRate());
        const float scenarioScale = ScenarioScale(spec);
        const float crossingDeadband =
            (std::max)((std::max)(spec.tolerance * 0.10f, scenarioScale * 0.001f), 1.0e-5f);
        const float lateWindowDurationS =
            (std::min)(spec.durationS, (std::max)(kLateWindowMinimumS, spec.durationS * kLateWindowFraction));
        metrics.lateWindowStartS = (std::max)(0.0f, spec.durationS - lateWindowDurationS);
        const int lateWindowStartSample =
            static_cast<int>(std::floor(metrics.lateWindowStartS / kTickSeconds));
        int previousNonZeroErrorSign = ErrorSign(initialError, crossingDeadband);
        float lateWindowMinimumError = std::numeric_limits<float>::infinity();
        float lateWindowMaximumError = -std::numeric_limits<float>::infinity();
        double lateWindowSquaredError = 0.0;
        float overshoot = 0.0f;
        int saturatedSamples = 0;
        int plantClipSamples = 0;
        int lastUnsettledSample = -1;
        const int maxSamples = static_cast<int>(std::ceil(spec.durationS / kTickSeconds));
        double first500VelocitySquaredError = 0.0;
        double first500YawRateSquaredError = 0.0;
        double first100ForwardAccelSquaredError = 0.0;
        double first100ForwardAccelObjectiveSquared = 0.0;
        double first100YawAccelSquaredError = 0.0;
        double first100YawAccelObjectiveSquared = 0.0;

        for (int sample = 0; sample < maxSamples; ++sample)
        {
            const MazeMap::App::Internal::CommandVector control =
                driveBase.ProposeBodyTick(
                    spec.targetForwardMps,
                    spec.targetYawRateRadps,
                    spec.targetForwardAccelMps2,
                    spec.targetYawAccelRadps2,
                    spec.targetYawRad);
            const DriveTelemetry& telemetry = driveBase.LastTelemetry();

            if (!control.IsFinite())
            {
                ++metrics.nonFiniteCount;
            }
            if ((telemetry.commandKindFlags & DriveTelemetry::kCommandKindSolverFailureEvidence) != 0U)
            {
                ++metrics.solverFailureCount;
            }

            if ((std::fabs(control.LeftCommand()) >= kSaturationThreshold) ||
                (std::fabs(control.RightCommand()) >= kSaturationThreshold))
            {
                ++saturatedSamples;
            }
            if ((std::fabs(telemetry.leftPlantCommand) > 1.0f) ||
                (std::fabs(telemetry.rightPlantCommand) > 1.0f))
            {
                ++plantClipSamples;
            }

            const float previousForwardMps = runtimeState.GetForwardVelocity();
            const float previousYawRateRadps = runtimeState.GetYawRate();
            const bool advanced = AdvanceTruth(plant, runtimeState, wheels, control);
            ++metrics.samples;

            const float forwardAccelMps2 =
                (runtimeState.GetForwardVelocity() - previousForwardMps) / kTickSeconds;
            const float yawAccelRadps2 =
                (runtimeState.GetYawRate() - previousYawRateRadps) / kTickSeconds;
            metrics.maxAbsForwardVelocityMps =
                (std::max)(metrics.maxAbsForwardVelocityMps, std::fabs(runtimeState.GetForwardVelocity()));
            metrics.maxAbsYawRateRadps =
                (std::max)(metrics.maxAbsYawRateRadps, std::fabs(runtimeState.GetYawRate()));
            metrics.maxAbsForwardAccelMps2 =
                (std::max)(metrics.maxAbsForwardAccelMps2, std::fabs(forwardAccelMps2));
            metrics.maxAbsYawAccelRadps2 =
                (std::max)(metrics.maxAbsYawAccelRadps2, std::fabs(yawAccelRadps2));
            metrics.maxAbsKinematicLateralAccelMps2 =
                (std::max)(
                    metrics.maxAbsKinematicLateralAccelMps2,
                    std::fabs(runtimeState.GetForwardVelocity() * runtimeState.GetYawRate()));

            if (metrics.forwardVelocityStepActive && (sample < kVelocityStepRmsWindowTicks))
            {
                const float velocityErrorMps = spec.targetForwardMps - runtimeState.GetForwardVelocity();
                if (std::isfinite(velocityErrorMps))
                {
                    first500VelocitySquaredError +=
                        static_cast<double>(velocityErrorMps) * static_cast<double>(velocityErrorMps);
                    ++metrics.first500VelocityErrorSamples;
                }
            }

            if (metrics.yawRateStepActive && (sample < kYawRateStepRmsWindowTicks))
            {
                const float yawRateErrorRadps = spec.targetYawRateRadps - runtimeState.GetYawRate();
                if (std::isfinite(yawRateErrorRadps))
                {
                    first500YawRateSquaredError +=
                        static_cast<double>(yawRateErrorRadps) * static_cast<double>(yawRateErrorRadps);
                    ++metrics.first500YawRateErrorSamples;
                }
            }

            if (metrics.forwardAccelStepMetricActive && (sample < kAccelerationStepRmsWindowTicks) &&
                std::isfinite(forwardAccelMps2))
            {
                float forwardAccelErrorMps2 = forwardAccelMps2;
                if (HasFiniteForwardAccelerationObjective(telemetry))
                {
                    forwardAccelErrorMps2 = forwardAccelMps2 - telemetry.composedForwardAccelMps2;
                    first100ForwardAccelObjectiveSquared +=
                        static_cast<double>(telemetry.composedForwardAccelMps2) *
                        static_cast<double>(telemetry.composedForwardAccelMps2);
                    ++metrics.first100ForwardAccelObjectiveSamples;
                }
                else
                {
                    ++metrics.first100ForwardAccelFallbackSamples;
                }

                first100ForwardAccelSquaredError +=
                    static_cast<double>(forwardAccelErrorMps2) * static_cast<double>(forwardAccelErrorMps2);
                ++metrics.first100ForwardAccelErrorSamples;
            }

            if (metrics.yawAccelStepMetricActive && (sample < kAccelerationStepRmsWindowTicks) &&
                std::isfinite(yawAccelRadps2))
            {
                float yawAccelErrorRadps2 = yawAccelRadps2;
                if (HasFiniteYawAccelerationObjective(telemetry))
                {
                    yawAccelErrorRadps2 = yawAccelRadps2 - telemetry.composedYawAccelRadps2;
                    first100YawAccelObjectiveSquared +=
                        static_cast<double>(telemetry.composedYawAccelRadps2) *
                        static_cast<double>(telemetry.composedYawAccelRadps2);
                    ++metrics.first100YawAccelObjectiveSamples;
                }
                else
                {
                    ++metrics.first100YawAccelFallbackSamples;
                }

                first100YawAccelSquaredError +=
                    static_cast<double>(yawAccelErrorRadps2) * static_cast<double>(yawAccelErrorRadps2);
                ++metrics.first100YawAccelErrorSamples;
            }

            const float error = SignalError(spec, runtimeState);
            const float absError = std::fabs(error);
            metrics.minimumAbsError = (std::min)(metrics.minimumAbsError, absError);
            metrics.integratedAbsoluteError += static_cast<double>(absError) * kTickSeconds;
            metrics.integratedSquaredError += static_cast<double>(error) * static_cast<double>(error) * kTickSeconds;
            if (absError > spec.tolerance)
            {
                lastUnsettledSample = sample;
            }

            if (initialError > 0.0f)
            {
                overshoot = (std::max)(overshoot, -error);
            }
            else if (initialError < 0.0f)
            {
                overshoot = (std::max)(overshoot, error);
            }

            const int errorSign = ErrorSign(error, crossingDeadband);
            if (errorSign != 0)
            {
                if ((previousNonZeroErrorSign != 0) && (errorSign != previousNonZeroErrorSign))
                {
                    if (!metrics.crossedTarget)
                    {
                        metrics.crossedTarget = true;
                        metrics.firstTargetCrossingTimeS = static_cast<float>(sample + 1) * kTickSeconds;
                    }
                    else
                    {
                        ++metrics.signChangesAfterFirstCrossing;
                    }
                }
                previousNonZeroErrorSign = errorSign;
            }

            if (sample >= lateWindowStartSample)
            {
                lateWindowMinimumError = (std::min)(lateWindowMinimumError, error);
                lateWindowMaximumError = (std::max)(lateWindowMaximumError, error);
                lateWindowSquaredError += static_cast<double>(error) * static_cast<double>(error);
                ++metrics.lateWindowSamples;
            }

            if (!advanced || !VehicleStateIsFinite(runtimeState))
            {
                ++metrics.nonFiniteCount;
                metrics.failed = true;
                break;
            }
        }

        metrics.finalValue = SignalValue(runtimeState, spec.signal);
        metrics.finalError = SignalError(spec, runtimeState);
        metrics.overshoot = (std::max)(0.0f, overshoot);
        if (metrics.samples > 0)
        {
            metrics.commandSaturationFraction = static_cast<float>(saturatedSamples) / static_cast<float>(metrics.samples);
            metrics.plantClipRequestFraction = static_cast<float>(plantClipSamples) / static_cast<float>(metrics.samples);
        }
        metrics.settled = (metrics.samples > 0) && (lastUnsettledSample < (metrics.samples - 1));
        metrics.settlingTimeS =
            metrics.settled ?
            (static_cast<float>(lastUnsettledSample + 1) * kTickSeconds) :
            spec.durationS;
        if (metrics.lateWindowSamples > 0)
        {
            metrics.lateWindowPeakToPeakError = lateWindowMaximumError - lateWindowMinimumError;
            metrics.lateWindowRmsError =
                std::sqrt(lateWindowSquaredError / static_cast<double>(metrics.lateWindowSamples));
        }
        if (metrics.first500VelocityErrorSamples > 0)
        {
            metrics.first500VelocityErrorRms =
                std::sqrt(first500VelocitySquaredError / static_cast<double>(metrics.first500VelocityErrorSamples));
        }
        if (metrics.first500YawRateErrorSamples > 0)
        {
            metrics.first500YawRateErrorRms =
                std::sqrt(first500YawRateSquaredError / static_cast<double>(metrics.first500YawRateErrorSamples));
        }
        if (metrics.first100ForwardAccelErrorSamples > 0)
        {
            metrics.first100ForwardAccelErrorRms =
                std::sqrt(
                    first100ForwardAccelSquaredError /
                    static_cast<double>(metrics.first100ForwardAccelErrorSamples));
        }
        if (metrics.first100ForwardAccelObjectiveSamples > 0)
        {
            metrics.first100ForwardAccelObjectiveRms =
                std::sqrt(
                    first100ForwardAccelObjectiveSquared /
                    static_cast<double>(metrics.first100ForwardAccelObjectiveSamples));
        }
        metrics.first100ForwardAccelDefinition =
            AccelerationMetricDefinition(
                metrics.first100ForwardAccelErrorSamples,
                metrics.first100ForwardAccelObjectiveSamples,
                metrics.first100ForwardAccelFallbackSamples);
        if (metrics.first100YawAccelErrorSamples > 0)
        {
            metrics.first100YawAccelErrorRms =
                std::sqrt(
                    first100YawAccelSquaredError /
                    static_cast<double>(metrics.first100YawAccelErrorSamples));
        }
        if (metrics.first100YawAccelObjectiveSamples > 0)
        {
            metrics.first100YawAccelObjectiveRms =
                std::sqrt(
                    first100YawAccelObjectiveSquared /
                    static_cast<double>(metrics.first100YawAccelObjectiveSamples));
        }
        metrics.first100YawAccelDefinition =
            AccelerationMetricDefinition(
                metrics.first100YawAccelErrorSamples,
                metrics.first100YawAccelObjectiveSamples,
                metrics.first100YawAccelFallbackSamples);
        metrics.responseReductionFraction =
            (initialAbsError > 1.0e-6f) ?
            (1.0f - ((std::min)(metrics.minimumAbsError, initialAbsError) / initialAbsError)) :
            1.0f;
        metrics.responseFailurePenalty = ComputeResponseFailurePenalty(metrics);
        metrics.oscillatory = IsScenarioOscillatory(metrics);
        metrics.oscillationPenalty = ComputeOscillationPenalty(metrics);
        metrics.failed = metrics.failed || (metrics.nonFiniteCount > 0);
        metrics.score = ComputeScenarioScore(metrics);
        return metrics;
    }

    static std::vector<AcceptanceMetrics> RunAcceptanceScenarios(const GainSet& gains)
    {
        std::vector<AcceptanceMetrics> acceptances;
        constexpr std::array<MazeMap::ManeuverCode, 4U> inPlaceManeuvers = {{
            MazeMap::IP45,
            MazeMap::IP90,
            MazeMap::IP135,
            MazeMap::IP180
        }};
        constexpr std::array<MazeMap::ManeuverCode, 13U> smoothManeuvers = {{
            MazeMap::S45LS,
            MazeMap::S45LD,
            MazeMap::S45SS,
            MazeMap::S45SD,
            MazeMap::S90LS,
            MazeMap::S90SS,
            MazeMap::S90SD,
            MazeMap::S135LS,
            MazeMap::S135LD,
            MazeMap::S135SS,
            MazeMap::S135SD,
            MazeMap::S180LS,
            MazeMap::S180SS
        }};

        acceptances.reserve(1U + (inPlaceManeuvers.size() * 6U) + (smoothManeuvers.size() * 8U));
        acceptances.push_back(RunStartStraightAcceptance(gains));

        for (const MazeMap::ManeuverCode code : inPlaceManeuvers)
        {
            const ManeuverAcceptanceTrace trace = SimulateDriveManeuverAcceptance(gains, code, false);
            acceptances.push_back(MakeCompletionAcceptance(trace, code, false));
            acceptances.push_back(MakeCommandEvidenceAcceptance(trace, code, false));
            acceptances.push_back(MakeManeuverTrackingRmsAcceptance(trace, code, false));
            acceptances.push_back(MakeInPlaceShiftAcceptance(trace, code));
            acceptances.push_back(MakeInPlaceHeadingAcceptance(trace, code));
            acceptances.push_back(MakeInPlaceTimeAcceptance(trace, code));
        }

        for (const MazeMap::ManeuverCode code : smoothManeuvers)
        {
            const ManeuverAcceptanceTrace trace = SimulateDriveManeuverAcceptance(gains, code, true);
            acceptances.push_back(MakeCompletionAcceptance(trace, code, true));
            acceptances.push_back(MakeCommandEvidenceAcceptance(trace, code, true));
            acceptances.push_back(MakeManeuverTrackingRmsAcceptance(trace, code, true));
            acceptances.push_back(MakeSmoothVelocityVariationAcceptance(trace, code));
            acceptances.push_back(MakeSmoothYawAccelerationVariationAcceptance(trace, code));
            acceptances.push_back(MakeSmoothYawRateVariationAcceptance(trace, code));
            acceptances.push_back(MakeSmoothFinalPositionAcceptance(trace, code));
            acceptances.push_back(MakeSmoothFinalHeadingAcceptance(trace, code));
        }

        return acceptances;
    }

    static EvaluationResult Evaluate(const GainSet& gains)
    {
        EvaluationResult result{};
        result.gains = gains;
        const std::array<ScenarioSpec, 7> specs = BuildScenarioSpecs();
        result.scenarios.reserve(specs.size());
        for (const ScenarioSpec& spec : specs)
        {
            ScenarioMetrics metrics = RunScenario(spec, gains);
            result.failed = result.failed || metrics.failed;
            result.oscillationFlagged = result.oscillationFlagged || metrics.oscillatory;
            result.score += metrics.score;
            result.scenarios.push_back(std::move(metrics));
        }
        result.acceptanceScenarios = RunAcceptanceScenarios(gains);
        for (const AcceptanceMetrics& acceptance : result.acceptanceScenarios)
        {
            result.acceptanceBlocked = result.acceptanceBlocked || acceptance.blocker;
            result.oscillationFlagged = result.oscillationFlagged || acceptance.rippleOscillatory;
            result.failed = result.failed || acceptance.blocker;
            result.score += acceptance.scorePenalty;
        }
        return result;
    }

    static SearchRange BuildRange(float seed, float fallbackMaximum, float multiplier) noexcept
    {
        const float maximum = (std::max)(fallbackMaximum, std::isfinite(seed) ? (seed * multiplier) : fallbackMaximum);
        return { 0.0f, maximum };
    }

    static bool SearchRangeIsExplicit(const Options& options, GainIndex index) noexcept
    {
        const std::size_t rawIndex = static_cast<std::size_t>(index);
        return options.searchMinimumSpecified[rawIndex] && options.searchMaximumSpecified[rawIndex];
    }

    static std::array<SearchRange, static_cast<std::size_t>(GainIndex::Count)> BuildInitialSearchRanges(
        const GainSet& seed,
        const Options& options) noexcept
    {
        std::array<SearchRange, static_cast<std::size_t>(GainIndex::Count)> ranges = {{
            BuildRange(seed.forwardPositionToAccelGain, 120.0f, 5.0f),
            BuildRange(seed.forwardVelocityToAccelGain, 120.0f, 5.0f),
            BuildRange(seed.forwardAccelerationErrorGain, 8.0f, 500.0f),
            BuildRange(seed.yawPositionToAccelGain, 120.0f, 5.0f),
            BuildRange(seed.yawVelocityToAccelGain, 220.0f, 5.0f),
            BuildRange(seed.yawAccelerationErrorGain, 60.0f, 5.0f)
        }};
        for (std::size_t rawIndex = 0U; rawIndex < static_cast<std::size_t>(GainIndex::Count); ++rawIndex)
        {
            if (SearchRangeIsExplicit(options, static_cast<GainIndex>(rawIndex)))
            {
                ranges[rawIndex] = options.searchRanges[rawIndex];
            }
        }
        return ranges;
    }

    static GainSet ClampGainsToSearchRanges(
        GainSet gains,
        const std::array<SearchRange, static_cast<std::size_t>(GainIndex::Count)>& ranges) noexcept
    {
        for (std::size_t rawIndex = 0U; rawIndex < static_cast<std::size_t>(GainIndex::Count); ++rawIndex)
        {
            const SearchRange& range = ranges[rawIndex];
            if (!std::isfinite(range.minimum) || !std::isfinite(range.maximum))
            {
                continue;
            }
            const float value = GetGain(gains, static_cast<GainIndex>(rawIndex));
            SetGain(
                gains,
                static_cast<GainIndex>(rawIndex),
                (std::min)((std::max)(value, range.minimum), range.maximum));
        }
        return gains;
    }

    static bool NearlySameGridValue(float left, float right) noexcept
    {
        const float scale = (std::max)(1.0f, (std::max)(std::fabs(left), std::fabs(right)));
        return std::fabs(left - right) <= (scale * 1.0e-6f);
    }

    static void NormalizeGridValues(std::vector<float>& values)
    {
        std::sort(values.begin(), values.end());
        auto uniqueEnd =
            std::unique(
                values.begin(),
                values.end(),
                [](float left, float right) noexcept
                {
                    return NearlySameGridValue(left, right);
                });
        values.erase(uniqueEnd, values.end());
    }

    static float BuildPositiveLogSearchMinimum(const SearchRange& range) noexcept
    {
        if (!std::isfinite(range.maximum) || !(range.maximum > 0.0f))
        {
            return 0.0f;
        }

        const float broadMinimum =
            (std::max)(
                kLogSearchAbsolutePositiveMinimum,
                range.maximum * kLogSearchMinimumRelativeToMaximum);
        const float requestedMinimum =
            (range.minimum > 0.0f) ? range.minimum : broadMinimum;
        return (std::min)((std::max)(requestedMinimum, broadMinimum), range.maximum);
    }

    static std::vector<float> BuildLogGridValues(const SearchRange& range, std::size_t gridPoints)
    {
        gridPoints = (std::max)(static_cast<std::size_t>(2U), gridPoints);
        std::vector<float> values;
        values.reserve(gridPoints);
        if (!(std::isfinite(range.minimum) && std::isfinite(range.maximum)) || (range.maximum <= range.minimum))
        {
            values.push_back((std::max)(0.0f, range.minimum));
            return values;
        }

        if (range.minimum <= 0.0f)
        {
            values.push_back(0.0f);
        }

        const float positiveMaximum = (std::max)(0.0f, range.maximum);
        const std::size_t positiveCount = gridPoints - values.size();
        if ((positiveCount == 0U) || !(positiveMaximum > 0.0f))
        {
            NormalizeGridValues(values);
            return values;
        }

        const float positiveMinimum = BuildPositiveLogSearchMinimum(range);
        if ((positiveCount == 1U) || !(positiveMaximum > positiveMinimum))
        {
            values.push_back(positiveMaximum);
            NormalizeGridValues(values);
            return values;
        }

        const double logMinimum = std::log(static_cast<double>(positiveMinimum));
        const double logMaximum = std::log(static_cast<double>(positiveMaximum));
        for (std::size_t index = 0U; index < positiveCount; ++index)
        {
            const double fraction =
                static_cast<double>(index) / static_cast<double>(positiveCount - 1U);
            values.push_back(static_cast<float>(std::exp(logMinimum + (fraction * (logMaximum - logMinimum)))));
        }
        NormalizeGridValues(values);
        return values;
    }

    static SearchRange BuildNextLogSearchRange(
        const SearchRange& absoluteRange,
        const std::vector<float>& evaluatedValues,
        float center) noexcept
    {
        if (!std::isfinite(center) || evaluatedValues.empty())
        {
            return absoluteRange;
        }

        center = (std::min)((std::max)(center, absoluteRange.minimum), absoluteRange.maximum);
        float nextMinimum = absoluteRange.minimum;
        float nextMaximum = absoluteRange.maximum;
        for (float value : evaluatedValues)
        {
            if (!std::isfinite(value))
            {
                continue;
            }
            if ((value < center) && (value > nextMinimum))
            {
                nextMinimum = value;
            }
            if ((value > center) && (value < nextMaximum))
            {
                nextMaximum = value;
            }
        }

        if (!(nextMaximum > nextMinimum))
        {
            return absoluteRange;
        }

        return { nextMinimum, nextMaximum };
    }

    static int CandidateFeasibilityRank(const EvaluationResult& result) noexcept
    {
        return (result.failed || result.acceptanceBlocked) ? 1 : 0;
    }

    static bool CandidateIsBetter(const EvaluationResult& candidate, const EvaluationResult& incumbent) noexcept
    {
        const int candidateRank = CandidateFeasibilityRank(candidate);
        const int incumbentRank = CandidateFeasibilityRank(incumbent);
        if (candidateRank != incumbentRank)
        {
            return candidateRank < incumbentRank;
        }
        return candidate.score < incumbent.score;
    }

    static void AddTopCandidate(std::vector<EvaluationResult>& topCandidates, EvaluationResult candidate, std::size_t maxCount)
    {
        if (maxCount == 0U)
        {
            return;
        }

        auto insertAt = topCandidates.begin();
        for (; insertAt != topCandidates.end(); ++insertAt)
        {
            if (CandidateIsBetter(candidate, *insertAt))
            {
                break;
            }
        }
        topCandidates.insert(insertAt, std::move(candidate));
        if (topCandidates.size() > maxCount)
        {
            topCandidates.pop_back();
        }
    }

    class SearchResult
    {
    public:
        EvaluationResult best{};
        std::vector<EvaluationResult> topCandidates;
        std::size_t evaluatedCandidates = 0U;
    };

    static SearchResult RunSearch(const GainSet& seed, const Options& options)
    {
        SearchResult result{};
        std::array<SearchRange, static_cast<std::size_t>(GainIndex::Count)> ranges =
            BuildInitialSearchRanges(seed, options);
        const std::array<SearchRange, static_cast<std::size_t>(GainIndex::Count)> absoluteRanges = ranges;
        const GainSet searchSeed = ClampGainsToSearchRanges(seed, ranges);

        result.best = Evaluate(searchSeed);
        result.evaluatedCandidates = 1U;
        AddTopCandidate(result.topCandidates, result.best, options.topCandidateCount);

        for (std::size_t pass = 0U; pass < options.searchPasses; ++pass)
        {
            for (std::size_t rawIndex = 0U; rawIndex < static_cast<std::size_t>(GainIndex::Count); ++rawIndex)
            {
                const GainIndex gainIndex = static_cast<GainIndex>(rawIndex);
                const std::vector<float> values = BuildLogGridValues(ranges[rawIndex], options.searchGridPoints);
                for (float value : values)
                {
                    GainSet candidate = result.best.gains;
                    SetGain(candidate, gainIndex, value);
                    EvaluationResult evaluation = Evaluate(candidate);
                    ++result.evaluatedCandidates;
                    if (CandidateIsBetter(evaluation, result.best))
                    {
                        result.best = evaluation;
                    }
                    AddTopCandidate(result.topCandidates, std::move(evaluation), options.topCandidateCount);
                }

                const float center = GetGain(result.best.gains, gainIndex);
                ranges[rawIndex] = BuildNextLogSearchRange(absoluteRanges[rawIndex], values, center);
            }
        }

        return result;
    }

    static bool ParseArgs(int argc, char* argv[], Options& options, std::string& error)
    {
        options.candidate = ExtractProductionTunedGains();
        for (int index = 1; index < argc; ++index)
        {
            const std::string arg = argv[index];
            if ((arg == "--help") || (arg == "-h") || (arg == "/?"))
            {
                PrintUsage();
                std::exit(0);
            }
            if (arg == "--search")
            {
                options.runSearch = true;
                continue;
            }

            std::string value;
            if (ReadOptionValue(argc, argv, index, "--search-passes", value, error))
            {
                if (!ParseSizeText(value, options.searchPasses) || (options.searchPasses == 0U))
                {
                    error = "Invalid --search-passes value: " + value;
                    return false;
                }
                continue;
            }
            if (ReadOptionValue(argc, argv, index, "--search-grid", value, error) ||
                ReadOptionValue(argc, argv, index, "--search-points", value, error) ||
                ReadOptionValue(argc, argv, index, "--points", value, error))
            {
                if (!ParseSizeText(value, options.searchGridPoints) || (options.searchGridPoints < 2U))
                {
                    error = "Invalid search point count: " + value;
                    return false;
                }
                continue;
            }
            if (ReadOptionValue(argc, argv, index, "--top", value, error))
            {
                if (!ParseSizeText(value, options.topCandidateCount))
                {
                    error = "Invalid --top value: " + value;
                    return false;
                }
                continue;
            }

            bool handledGain = false;
            const std::array<const char*, 8> gainOptions = {{
                "--velocity-kp", "--vel-kp",
                "--velocity-kd", "--vel-kd",
                "--heading-kp", "--heading-kd",
                "--yawrate-kp", "--yaw-rate-kp"
            }};
            for (const char* gainOption : gainOptions)
            {
                value.clear();
                if (ReadOptionValue(argc, argv, index, gainOption, value, error))
                {
                    float parsed = 0.0f;
                    if (!ParseFloatText(value, parsed) || (parsed < 0.0f))
                    {
                        error = "Invalid " + std::string(gainOption) + " value: " + value;
                        return false;
                    }
                    if (!ApplyGainOption(gainOption, parsed, options.candidate))
                    {
                        error = "Internal parser error for " + std::string(gainOption);
                        return false;
                    }
                    options.hasExplicitCandidate = true;
                    handledGain = true;
                    break;
                }
            }
            if (handledGain)
            {
                continue;
            }

            const std::array<const char*, 20> boundOptions = {{
                "--velocity-kp-min", "--vel-kp-min",
                "--velocity-kp-max", "--vel-kp-max",
                "--velocity-kd-min", "--vel-kd-min",
                "--velocity-kd-max", "--vel-kd-max",
                "--heading-kp-min", "--heading-kp-max",
                "--heading-kd-min", "--heading-kd-max",
                "--yawrate-kp-min", "--yaw-rate-kp-min",
                "--yawrate-kp-max", "--yaw-rate-kp-max",
                "--yawrate-kd-min", "--yaw-rate-kd-min",
                "--yawrate-kd-max", "--yaw-rate-kd-max"
            }};
            for (const char* boundOption : boundOptions)
            {
                value.clear();
                if (ReadOptionValue(argc, argv, index, boundOption, value, error))
                {
                    float parsed = 0.0f;
                    if (!ParseFloatText(value, parsed) || (parsed < 0.0f))
                    {
                        error = "Invalid " + std::string(boundOption) + " value: " + value;
                        return false;
                    }
                    if (!ApplySearchBoundOption(boundOption, parsed, options))
                    {
                        error = "Internal parser error for " + std::string(boundOption);
                        return false;
                    }
                    handledGain = true;
                    break;
                }
            }
            if (handledGain)
            {
                continue;
            }

            const std::array<const char*, 2> yawKdOptions = {{ "--yawrate-kd", "--yaw-rate-kd" }};
            for (const char* gainOption : yawKdOptions)
            {
                value.clear();
                if (ReadOptionValue(argc, argv, index, gainOption, value, error))
                {
                    float parsed = 0.0f;
                    if (!ParseFloatText(value, parsed) || (parsed < 0.0f))
                    {
                        error = "Invalid " + std::string(gainOption) + " value: " + value;
                        return false;
                    }
                    if (!ApplyGainOption(gainOption, parsed, options.candidate))
                    {
                        error = "Internal parser error for " + std::string(gainOption);
                        return false;
                    }
                    options.hasExplicitCandidate = true;
                    handledGain = true;
                    break;
                }
            }
            if (handledGain)
            {
                continue;
            }

            error = "Unknown argument: " + arg;
            return false;
        }

        for (std::size_t rawIndex = 0U; rawIndex < static_cast<std::size_t>(GainIndex::Count); ++rawIndex)
        {
            const bool hasMinimum = options.searchMinimumSpecified[rawIndex];
            const bool hasMaximum = options.searchMaximumSpecified[rawIndex];
            if (hasMinimum != hasMaximum)
            {
                error =
                    "Search bounds for " +
                    std::string(GainName(static_cast<GainIndex>(rawIndex))) +
                    " require both min and max values.";
                return false;
            }
            if (hasMinimum && (options.searchRanges[rawIndex].maximum < options.searchRanges[rawIndex].minimum))
            {
                error =
                    "Search bounds for " +
                    std::string(GainName(static_cast<GainIndex>(rawIndex))) +
                    " must have max >= min.";
                return false;
            }
        }

        return true;
    }

    static std::string JsonString(const std::string& value)
    {
        std::ostringstream output;
        output << '"';
        for (char ch : value)
        {
            switch (ch)
            {
            case '\\':
                output << "\\\\";
                break;
            case '"':
                output << "\\\"";
                break;
            case '\n':
                output << "\\n";
                break;
            case '\r':
                output << "\\r";
                break;
            case '\t':
                output << "\\t";
                break;
            default:
                output << ch;
                break;
            }
        }
        output << '"';
        return output.str();
    }

    static void WriteJsonNumber(std::ostream& output, double value)
    {
        if (!std::isfinite(value))
        {
            output << "null";
            return;
        }
        output << std::setprecision(10) << value;
    }

    static void WriteGainsJson(std::ostream& output, const GainSet& gains, int indent)
    {
        const std::string pad(static_cast<std::size_t>(indent), ' ');
        output << "{\n"
            << pad << "  \"forward_axis\": { \"position_to_accel_gain\": ";
        WriteJsonNumber(output, gains.forwardPositionToAccelGain);
        output << ", \"velocity_to_accel_gain\": ";
        WriteJsonNumber(output, gains.forwardVelocityToAccelGain);
        output << ", \"acceleration_error_gain\": ";
        WriteJsonNumber(output, gains.forwardAccelerationErrorGain);
        output << " },\n"
            << pad << "  \"yaw_axis\": { \"position_to_accel_gain\": ";
        WriteJsonNumber(output, gains.yawPositionToAccelGain);
        output << ", \"velocity_to_accel_gain\": ";
        WriteJsonNumber(output, gains.yawVelocityToAccelGain);
        output << ", \"acceleration_error_gain\": ";
        WriteJsonNumber(output, gains.yawAccelerationErrorGain);
        output << " }\n"
            << pad << "}";
    }

    static void WriteScenarioJson(std::ostream& output, const ScenarioMetrics& metrics, int indent)
    {
        const std::string pad(static_cast<std::size_t>(indent), ' ');
        output << "{\n"
            << pad << "  \"name\": " << JsonString(metrics.name) << ",\n"
            << pad << "  \"description\": " << JsonString(metrics.description) << ",\n"
            << pad << "  \"signal\": " << JsonString(metrics.signal) << ",\n"
            << pad << "  \"definition\": {\n"
            << pad << "    \"initial_forward_mps\": ";
        WriteJsonNumber(output, metrics.initialForwardMps);
        output << ",\n" << pad << "    \"initial_yaw_rate_radps\": ";
        WriteJsonNumber(output, metrics.initialYawRateRadps);
        output << ",\n" << pad << "    \"initial_heading_rad\": ";
        WriteJsonNumber(output, metrics.initialYawRad);
        output << ",\n" << pad << "    \"target_forward_mps\": ";
        WriteJsonNumber(output, metrics.targetForwardMps);
        output << ",\n" << pad << "    \"target_yaw_rate_radps\": ";
        WriteJsonNumber(output, metrics.targetYawRateRadps);
        output << ",\n" << pad << "    \"target_forward_accel_mps2\": ";
        WriteJsonNumber(output, metrics.targetForwardAccelMps2);
        output << ",\n" << pad << "    \"target_yaw_accel_radps2\": ";
        WriteJsonNumber(output, metrics.targetYawAccelRadps2);
        output << ",\n" << pad << "    \"target_heading_rad\": ";
        WriteJsonNumber(output, metrics.targetYawRad);
        output << ",\n" << pad << "    \"target_kinematic_lateral_accel_mps2\": ";
        WriteJsonNumber(output, metrics.targetKinematicLateralAccelMps2);
        output << "\n" << pad << "  },\n"
            << pad << "  \"target\": ";
        WriteJsonNumber(output, metrics.target);
        output << ",\n" << pad << "  \"initial\": ";
        WriteJsonNumber(output, metrics.initial);
        output << ",\n" << pad << "  \"tolerance\": ";
        WriteJsonNumber(output, metrics.tolerance);
        output << ",\n" << pad << "  \"duration_s\": ";
        WriteJsonNumber(output, metrics.durationS);
        output << ",\n" << pad << "  \"samples\": " << metrics.samples << ",\n"
            << pad << "  \"final_value\": ";
        WriteJsonNumber(output, metrics.finalValue);
        output << ",\n" << pad << "  \"final_error\": ";
        WriteJsonNumber(output, metrics.finalError);
        output << ",\n" << pad << "  \"integrated_absolute_error\": ";
        WriteJsonNumber(output, metrics.integratedAbsoluteError);
        output << ",\n" << pad << "  \"integrated_squared_error\": ";
        WriteJsonNumber(output, metrics.integratedSquaredError);
        output << ",\n" << pad << "  \"step_response\": {\n"
            << pad << "    \"tick_seconds\": ";
        WriteJsonNumber(output, kTickSecondsExact);
        output << ",\n" << pad << "    \"velocity_error_first_500_ticks\": {\n"
            << pad << "      \"active\": " << (metrics.forwardVelocityStepActive ? "true" : "false") << ",\n"
            << pad << "      \"target\": \"target_forward_mps\",\n"
            << pad << "      \"samples\": " << metrics.first500VelocityErrorSamples << ",\n"
            << pad << "      \"rms_mps\": ";
        WriteJsonNumber(output, metrics.first500VelocityErrorRms);
        output << "\n" << pad << "    },\n"
            << pad << "    \"yaw_rate_error_first_500_ticks\": {\n"
            << pad << "      \"active\": " << (metrics.yawRateStepActive ? "true" : "false") << ",\n"
            << pad << "      \"target\": \"target_yaw_rate_radps\",\n"
            << pad << "      \"samples\": " << metrics.first500YawRateErrorSamples << ",\n"
            << pad << "      \"rms_radps\": ";
        WriteJsonNumber(output, metrics.first500YawRateErrorRms);
        output << "\n" << pad << "    },\n"
            << pad << "    \"forward_accel_error_first_100_ticks\": {\n"
            << pad << "      \"active\": " << (metrics.forwardAccelStepMetricActive ? "true" : "false") << ",\n"
            << pad << "      \"definition\": " << JsonString(metrics.first100ForwardAccelDefinition) << ",\n"
            << pad << "      \"samples\": " << metrics.first100ForwardAccelErrorSamples << ",\n"
            << pad << "      \"composed_objective_samples\": " << metrics.first100ForwardAccelObjectiveSamples << ",\n"
            << pad << "      \"fallback_samples\": " << metrics.first100ForwardAccelFallbackSamples << ",\n"
            << pad << "      \"composed_objective_rms_mps2\": ";
        WriteJsonNumber(output, metrics.first100ForwardAccelObjectiveRms);
        output << ",\n" << pad << "      \"rms_mps2\": ";
        WriteJsonNumber(output, metrics.first100ForwardAccelErrorRms);
        output << "\n" << pad << "    },\n"
            << pad << "    \"yaw_accel_error_first_100_ticks\": {\n"
            << pad << "      \"active\": " << (metrics.yawAccelStepMetricActive ? "true" : "false") << ",\n"
            << pad << "      \"definition\": " << JsonString(metrics.first100YawAccelDefinition) << ",\n"
            << pad << "      \"samples\": " << metrics.first100YawAccelErrorSamples << ",\n"
            << pad << "      \"composed_objective_samples\": " << metrics.first100YawAccelObjectiveSamples << ",\n"
            << pad << "      \"fallback_samples\": " << metrics.first100YawAccelFallbackSamples << ",\n"
            << pad << "      \"composed_objective_rms_radps2\": ";
        WriteJsonNumber(output, metrics.first100YawAccelObjectiveRms);
        output << ",\n" << pad << "      \"rms_radps2\": ";
        WriteJsonNumber(output, metrics.first100YawAccelErrorRms);
        output << "\n" << pad << "    }\n"
            << pad << "  }";
        output << ",\n" << pad << "  \"minimum_abs_error\": ";
        WriteJsonNumber(output, metrics.minimumAbsError);
        output << ",\n" << pad << "  \"response_reduction_fraction\": ";
        WriteJsonNumber(output, metrics.responseReductionFraction);
        output << ",\n" << pad << "  \"response_failure_penalty\": ";
        WriteJsonNumber(output, metrics.responseFailurePenalty);
        output << ",\n" << pad << "  \"settling_time_s\": ";
        WriteJsonNumber(output, metrics.settlingTimeS);
        output << ",\n" << pad << "  \"settled\": " << (metrics.settled ? "true" : "false") << ",\n"
            << pad << "  \"overshoot\": ";
        WriteJsonNumber(output, metrics.overshoot);
        output << ",\n" << pad << "  \"command_saturation_fraction\": ";
        WriteJsonNumber(output, metrics.commandSaturationFraction);
        output << ",\n" << pad << "  \"plant_clip_request_fraction\": ";
        WriteJsonNumber(output, metrics.plantClipRequestFraction);
        output << ",\n" << pad << "  \"max_abs_forward_velocity_mps\": ";
        WriteJsonNumber(output, metrics.maxAbsForwardVelocityMps);
        output << ",\n" << pad << "  \"max_abs_yaw_rate_radps\": ";
        WriteJsonNumber(output, metrics.maxAbsYawRateRadps);
        output << ",\n" << pad << "  \"max_abs_forward_accel_mps2\": ";
        WriteJsonNumber(output, metrics.maxAbsForwardAccelMps2);
        output << ",\n" << pad << "  \"max_abs_yaw_accel_radps2\": ";
        WriteJsonNumber(output, metrics.maxAbsYawAccelRadps2);
        output << ",\n" << pad << "  \"max_abs_kinematic_lateral_accel_mps2\": ";
        WriteJsonNumber(output, metrics.maxAbsKinematicLateralAccelMps2);
        output << ",\n" << pad << "  \"oscillation\": {\n"
            << pad << "    \"crossed_target\": " << (metrics.crossedTarget ? "true" : "false") << ",\n"
            << pad << "    \"first_target_crossing_time_s\": ";
        WriteJsonNumber(output, metrics.firstTargetCrossingTimeS);
        output << ",\n" << pad << "    \"sign_changes_after_first_crossing\": " << metrics.signChangesAfterFirstCrossing << ",\n"
            << pad << "    \"late_window_start_s\": ";
        WriteJsonNumber(output, metrics.lateWindowStartS);
        output << ",\n" << pad << "    \"late_window_samples\": " << metrics.lateWindowSamples << ",\n"
            << pad << "    \"late_window_peak_to_peak_error\": ";
        WriteJsonNumber(output, metrics.lateWindowPeakToPeakError);
        output << ",\n" << pad << "    \"late_window_rms_error\": ";
        WriteJsonNumber(output, metrics.lateWindowRmsError);
        output << ",\n" << pad << "    \"oscillatory\": " << (metrics.oscillatory ? "true" : "false") << ",\n"
            << pad << "    \"penalty\": ";
        WriteJsonNumber(output, metrics.oscillationPenalty);
        output << "\n" << pad << "  }";
        output << ",\n" << pad << "  \"solver_failure_count\": " << metrics.solverFailureCount << ",\n"
            << pad << "  \"non_finite_count\": " << metrics.nonFiniteCount << ",\n"
            << pad << "  \"failed\": " << (metrics.failed ? "true" : "false") << ",\n"
            << pad << "  \"score\": ";
        WriteJsonNumber(output, metrics.score);
        output << "\n" << pad << "}";
    }

    static void WriteAcceptanceJson(std::ostream& output, const AcceptanceMetrics& metrics, int indent)
    {
        const std::string pad(static_cast<std::size_t>(indent), ' ');
        output << "{\n"
            << pad << "  \"name\": " << JsonString(metrics.name) << ",\n"
            << pad << "  \"description\": " << JsonString(metrics.description) << ",\n"
            << pad << "  \"path\": " << JsonString(metrics.path) << ",\n"
            << pad << "  \"code\": " << JsonString(metrics.code) << ",\n"
            << pad << "  \"metric\": " << JsonString(metrics.metric) << ",\n"
            << pad << "  \"tick_seconds\": ";
        WriteJsonNumber(output, kTickSecondsExact);
        output << ",\n"
            << pad << "  \"max_ticks\": " << metrics.maxTicks << ",\n"
            << pad << "  \"applied_ticks\": " << metrics.appliedTicks << ",\n"
            << pad << "  \"elapsed_seconds\": ";
        WriteJsonNumber(output, metrics.elapsedSeconds);
        output << ",\n"
            << pad << "  \"started\": " << (metrics.started ? "true" : "false") << ",\n"
            << pad << "  \"completed\": " << (metrics.completed ? "true" : "false") << ",\n"
            << pad << "  \"passed\": " << (metrics.passed ? "true" : "false") << ",\n"
            << pad << "  \"blocker\": " << (metrics.blocker ? "true" : "false") << ",\n"
            << pad << "  \"ripple_oscillatory\": " << (metrics.rippleOscillatory ? "true" : "false") << ",\n"
            << pad << "  \"definition\": {\n"
            << pad << "    \"target_distance_m\": ";
        WriteJsonNumber(output, metrics.targetDistanceM);
        output << ",\n"
            << pad << "    \"target_heading_rad\": ";
        WriteJsonNumber(output, metrics.targetYawRad);
        output << ",\n"
            << pad << "    \"heading_tolerance_rad\": ";
        WriteJsonNumber(output, metrics.headingToleranceRad);
        output << ",\n"
            << pad << "    \"heading_tolerance_deg\": ";
        WriteJsonNumber(output, metrics.headingToleranceDeg);
        output << ",\n"
            << pad << "    \"position_tolerance_m\": ";
        WriteJsonNumber(output, metrics.positionToleranceM);
        output << ",\n"
            << pad << "    \"shift_tolerance_m\": ";
        WriteJsonNumber(output, metrics.shiftToleranceM);
        output << ",\n"
            << pad << "    \"expected_elapsed_seconds\": ";
        WriteJsonNumber(output, metrics.expectedElapsedSeconds);
        output << ",\n"
            << pad << "    \"elapsed_relative_tolerance\": ";
        WriteJsonNumber(output, metrics.elapsedRelativeTolerance);
        output << ",\n"
            << pad << "    \"velocity_variation_limit\": ";
        WriteJsonNumber(output, metrics.velocityVariationLimit);
        output << ",\n"
            << pad << "    \"yaw_acceleration_variation_limit\": ";
        WriteJsonNumber(output, metrics.yawAccelerationVariationLimit);
        output << ",\n"
            << pad << "    \"yaw_rate_variation_limit\": ";
        WriteJsonNumber(output, metrics.yawRateVariationLimit);
        output << "\n" << pad << "  },\n"
            << pad << "  \"final\": {\n"
            << pad << "    \"x_m\": ";
        WriteJsonNumber(output, metrics.finalXM);
        output << ",\n"
            << pad << "    \"y_m\": ";
        WriteJsonNumber(output, metrics.finalYM);
        output << ",\n"
            << pad << "    \"heading_rad\": ";
        WriteJsonNumber(output, metrics.finalYawRad);
        output << ",\n"
            << pad << "    \"encoder_average_distance_m\": ";
        WriteJsonNumber(output, metrics.encoderAverageDistanceM);
        output << ",\n"
            << pad << "    \"heading_error_rad\": ";
        WriteJsonNumber(output, metrics.finalHeadingErrorRad);
        output << ",\n"
            << pad << "    \"heading_error_deg\": ";
        WriteJsonNumber(output, metrics.finalHeadingErrorDeg);
        output << ",\n"
            << pad << "    \"position_error_m\": ";
        WriteJsonNumber(output, metrics.finalPositionErrorM);
        output << ",\n"
            << pad << "    \"shift_distance_m\": ";
        WriteJsonNumber(output, metrics.shiftDistanceM);
        output << ",\n"
            << pad << "    \"elapsed_relative_error\": ";
        WriteJsonNumber(output, metrics.elapsedRelativeError);
        output << ",\n"
            << pad << "    \"velocity_variation\": ";
        WriteJsonNumber(output, metrics.velocityVariation);
        output << ",\n"
            << pad << "    \"yaw_acceleration_variation\": ";
        WriteJsonNumber(output, metrics.yawAccelerationVariation);
        output << ",\n"
            << pad << "    \"yaw_rate_variation\": ";
        WriteJsonNumber(output, metrics.yawRateVariation);
        output << "\n" << pad << "  },\n"
            << pad << "  \"tracking_rms\": {\n"
            << pad << "    \"samples\": " << metrics.trackingSampleCount << ",\n"
            << pad << "    \"distance_error_m\": ";
        WriteJsonNumber(output, metrics.distanceRmsErrorM);
        output << ",\n"
            << pad << "    \"distance_reference_m\": ";
        WriteJsonNumber(output, metrics.distanceRmsReferenceM);
        output << ",\n"
            << pad << "    \"heading_error_rad\": ";
        WriteJsonNumber(output, metrics.headingRmsErrorRad);
        output << ",\n"
            << pad << "    \"heading_error_deg\": ";
        WriteJsonNumber(output, metrics.headingRmsErrorDeg);
        output << ",\n"
            << pad << "    \"heading_reference_rad\": ";
        WriteJsonNumber(output, metrics.headingRmsReferenceRad);
        output << ",\n"
            << pad << "    \"forward_velocity_error_mps\": ";
        WriteJsonNumber(output, metrics.forwardVelocityRmsErrorMps);
        output << ",\n"
            << pad << "    \"forward_velocity_reference_mps\": ";
        WriteJsonNumber(output, metrics.forwardVelocityRmsReferenceMps);
        output << ",\n"
            << pad << "    \"yaw_rate_error_radps\": ";
        WriteJsonNumber(output, metrics.yawRateRmsErrorRadps);
        output << ",\n"
            << pad << "    \"yaw_rate_reference_radps\": ";
        WriteJsonNumber(output, metrics.yawRateRmsReferenceRadps);
        output << ",\n"
            << pad << "    \"forward_accel_error_mps2\": ";
        WriteJsonNumber(output, metrics.forwardAccelRmsErrorMps2);
        output << ",\n"
            << pad << "    \"forward_accel_reference_mps2\": ";
        WriteJsonNumber(output, metrics.forwardAccelRmsReferenceMps2);
        output << ",\n"
            << pad << "    \"yaw_accel_error_radps2\": ";
        WriteJsonNumber(output, metrics.yawAccelRmsErrorRadps2);
        output << ",\n"
            << pad << "    \"yaw_accel_reference_radps2\": ";
        WriteJsonNumber(output, metrics.yawAccelRmsReferenceRadps2);
        output << "\n" << pad << "  },\n"
            << pad << "  \"health\": {\n"
            << pad << "    \"all_controls_finite\": " << (metrics.allControlsFinite ? "true" : "false") << ",\n"
            << pad << "    \"command_evidence_valid\": " << (metrics.commandEvidenceValid ? "true" : "false") << ",\n"
            << pad << "    \"requested_objectives_finite\": " << (metrics.requestedObjectivesFinite ? "true" : "false") << ",\n"
            << pad << "    \"truth_finite\": " << (metrics.truthFinite ? "true" : "false") << ",\n"
            << pad << "    \"solver_clean\": " << (metrics.solverClean ? "true" : "false") << ",\n"
            << pad << "    \"solver_failure_count\": " << metrics.solverFailureCount << ",\n"
            << pad << "    \"non_finite_count\": " << metrics.nonFiniteCount << "\n"
            << pad << "  },\n"
            << pad << "  \"score_contribution\": ";
        WriteJsonNumber(output, metrics.scorePenalty);
        output << ",\n"
            << pad << "  \"score_penalty\": ";
        WriteJsonNumber(output, metrics.scorePenalty);
        output << "\n" << pad << "}";
    }

    static void WriteEvaluationJson(std::ostream& output, const char* label, const EvaluationResult& result, int indent)
    {
        const std::string pad(static_cast<std::size_t>(indent), ' ');
        output << pad << JsonString(label) << ": {\n"
            << pad << "  \"score\": ";
        WriteJsonNumber(output, result.score);
        output << ",\n" << pad << "  \"failed\": " << (result.failed ? "true" : "false") << ",\n"
            << pad << "  \"oscillation_flagged\": " << (result.oscillationFlagged ? "true" : "false") << ",\n"
            << pad << "  \"acceptance_blocked\": " << (result.acceptanceBlocked ? "true" : "false") << ",\n"
            << pad << "  \"gains\": ";
        WriteGainsJson(output, result.gains, indent + 2);
        output << ",\n" << pad << "  \"scenarios\": [\n";
        for (std::size_t index = 0U; index < result.scenarios.size(); ++index)
        {
            output << pad << "    ";
            WriteScenarioJson(output, result.scenarios[index], indent + 4);
            output << ((index + 1U) < result.scenarios.size() ? "," : "") << "\n";
        }
        output << pad << "  ],\n"
            << pad << "  \"acceptance_scenarios\": [\n";
        for (std::size_t index = 0U; index < result.acceptanceScenarios.size(); ++index)
        {
            output << pad << "    ";
            WriteAcceptanceJson(output, result.acceptanceScenarios[index], indent + 4);
            output << ((index + 1U) < result.acceptanceScenarios.size() ? "," : "") << "\n";
        }
        output << pad << "  ]\n" << pad << "}";
    }

    static void WriteTopCandidatesJson(std::ostream& output, const std::vector<EvaluationResult>& topCandidates, int indent)
    {
        const std::string pad(static_cast<std::size_t>(indent), ' ');
        output << "[\n";
        for (std::size_t index = 0U; index < topCandidates.size(); ++index)
        {
            const EvaluationResult& candidate = topCandidates[index];
            output << pad << "  {\n"
                << pad << "    \"rank\": " << (index + 1U) << ",\n"
                << pad << "    \"score\": ";
            WriteJsonNumber(output, candidate.score);
            output << ",\n" << pad << "    \"oscillation_flagged\": " << (candidate.oscillationFlagged ? "true" : "false") << ",\n"
                << pad << "    \"acceptance_blocked\": " << (candidate.acceptanceBlocked ? "true" : "false") << ",\n"
                << pad << "    \"gains\": ";
            WriteGainsJson(output, candidate.gains, indent + 4);
            output << "\n" << pad << "  }" << ((index + 1U) < topCandidates.size() ? "," : "") << "\n";
        }
        output << pad << "]";
    }

    static void WriteSearchBoundsJson(
        std::ostream& output,
        const GainSet& seed,
        const Options& options,
        int indent)
    {
        const std::string pad(static_cast<std::size_t>(indent), ' ');
        const std::array<SearchRange, static_cast<std::size_t>(GainIndex::Count)> ranges =
            BuildInitialSearchRanges(seed, options);
        output << "{\n";
        for (std::size_t rawIndex = 0U; rawIndex < static_cast<std::size_t>(GainIndex::Count); ++rawIndex)
        {
            const GainIndex index = static_cast<GainIndex>(rawIndex);
            output << pad << "  " << JsonString(GainName(index)) << ": {\n"
                << pad << "    \"minimum\": ";
            WriteJsonNumber(output, ranges[rawIndex].minimum);
            output << ",\n"
                << pad << "    \"maximum\": ";
            WriteJsonNumber(output, ranges[rawIndex].maximum);
            output << ",\n"
                << pad << "    \"source\": " << JsonString(SearchRangeIsExplicit(options, index) ? "explicit" : "derived")
                << "\n" << pad << "  }" << ((rawIndex + 1U) < static_cast<std::size_t>(GainIndex::Count) ? "," : "") << "\n";
        }
        output << pad << "}";
    }

    static void WriteOutputJson(
        const EvaluationResult& baseline,
        const EvaluationResult& candidate,
        const Options& options,
        const SearchResult* search)
    {
        std::cout << "{\n"
            << "  \"tool\": \"PdTuning\",\n"
            << "  \"tick_seconds\": ";
        WriteJsonNumber(std::cout, kTickSecondsExact);
        std::cout << ",\n"
            << "  \"fan_duty\": ";
        WriteJsonNumber(std::cout, kFanDuty);
        std::cout << ",\n"
            << "  \"physical_parameters_fixed\": true,\n"
            << "  \"acceptance_metric_definitions\": {\n"
            << "    \"scoring\": \"Completion, command-evidence, release-limit, timing, and final-position rows are reported with pass/blocker flags but do not contribute weighted score. DriveManeuver ranking uses the maneuver_tracking_rms row: the sum of squared per-tick RMS tracking-error ratios for distance, heading, forward velocity, yaw rate, forward acceleration, and yaw acceleration.\",\n"
            << "    \"drive_primitive_start_straight_completes\": \"Runs Drive::StartStraight(0.30 m, 0.30 m/s, 0.0 m/s exit) for up to 6000 exact 0.001s ticks through SharedRobotRuntime, Drive, DriveBase, PlantModel::integrate, and sensor snapshot publication; completion failure is reported as a blocker but contributes no weighted score.\",\n"
            << "    \"drive_maneuver_in_place_codes\": \"Covers IP45, IP90, IP135, and IP180 through the same SharedRobotRuntime/Drive/DriveBase/PlantModel path as DriveManeuverTests.\",\n"
            << "    \"drive_maneuver_in_place_completion\": \"Each in-place maneuver must start, complete within 20000 exact 0.001s ticks, and emit command samples.\",\n"
            << "    \"drive_maneuver_in_place_command_evidence\": \"Each in-place maneuver must return finite wheel commands matching DriveTelemetry command evidence, with finite requested body objectives and finite truth state.\",\n"
            << "    \"drive_maneuver_tracking_rms\": \"Each maneuver records one tracking sample per 0.001s command tick after applying ten 0.0001s plant substeps with that same command. The scored value is the sum of squared RMS error ratios for cumulative distance, instantaneous heading, forward velocity, yaw rate, forward acceleration, and yaw acceleration.\",\n"
            << "    \"drive_maneuver_in_place_shift\": \"Each in-place maneuver final translation is reported against 0.020 m, but this position check is informational for the current PD ripple tuning pass and does not block acceptance or contribute score.\",\n"
            << "    \"drive_maneuver_in_place_heading\": \"Each in-place maneuver final heading error is reported against 3 degrees, but the threshold row is informational for the current PD ripple tuning pass and does not block acceptance or contribute score.\",\n"
            << "    \"drive_maneuver_in_place_time\": \"Each in-place maneuver elapsed time is reported against MotionLimits::ComputeMinimumTurnDurationSeconds for the nominal turn angle, but this timing check is informational for PD tuning and does not block acceptance or contribute score.\",\n"
            << "    \"drive_maneuver_smooth_codes\": \"Covers S45LS, S45LD, S45SS, S45SD, S90LS, S90SS, S90SD, S135LS, S135LD, S135SS, S135SD, S180LS, and S180SS through the same SharedRobotRuntime/Drive/DriveBase/PlantModel path as DriveManeuverTests.\",\n"
            << "    \"drive_maneuver_smooth_completion\": \"Each smooth maneuver at 0.50 m/s entry/exit must start, complete within 20000 exact 0.001s ticks, and emit command samples.\",\n"
            << "    \"drive_maneuver_smooth_command_evidence\": \"Each smooth maneuver must return finite wheel commands matching DriveTelemetry command evidence, with finite requested body objectives and finite truth state.\",\n"
            << "    \"drive_maneuver_smooth_velocity_variation\": \"Each smooth maneuver normalized span of requested forward-speed magnitudes is reported against 0.05 but is informational for the current PD ripple tuning pass and does not block acceptance or contribute score.\",\n"
            << "    \"drive_maneuver_smooth_yaw_acceleration_variation\": \"Each smooth maneuver normalized span of trimmed ramp yaw-acceleration magnitudes is reported against 0.20 but is informational for the current PD ripple tuning pass and does not block acceptance or contribute score.\",\n"
            << "    \"drive_maneuver_smooth_yaw_rate_variation\": \"Each smooth maneuver normalized span of plateau yaw-rate magnitudes is reported against 0.08 but is informational for the current PD ripple tuning pass and does not block acceptance or contribute score.\",\n"
            << "    \"drive_maneuver_ripple_oscillation\": \"Any smooth velocity, yaw-acceleration, or yaw-rate variation above the normalized ripple deadband sets ripple_oscillatory=true on that row and contributes to the aggregate oscillation_flagged value.\",\n"
            << "    \"drive_maneuver_smooth_final_position\": \"Each smooth maneuver final position error is reported against 0.030 m, but this position check is informational for the current PD ripple tuning pass and does not block acceptance or contribute score.\",\n"
            << "    \"drive_maneuver_smooth_final_heading\": \"Each smooth maneuver final heading error is reported against 3 degrees, but the threshold row is informational for the current PD ripple tuning pass and does not block acceptance or contribute score.\"\n"
            << "  },\n"
            << "  \"step_response_metric_definitions\": {\n"
            << "    \"tick_seconds\": ";
        WriteJsonNumber(std::cout, kTickSecondsExact);
        std::cout << ",\n"
            << "    \"velocity_error_first_500_ticks\": \"RMS(targetForwardMps - state.vf) over the first 500 0.001s ticks when targetForwardMps steps from its initial value.\",\n"
            << "    \"yaw_rate_error_first_500_ticks\": \"RMS(targetYawRateRadps - state.yaw_rate) over the first 500 0.001s ticks when targetYawRateRadps steps from its initial value.\",\n"
            << "    \"forward_accel_error_first_100_ticks\": \"RMS((nextVf - prevVf) / 0.001 - DriveTelemetry.composedForwardAccelMps2) over the first 100 ticks for forward velocity steps when DriveTelemetry publishes a finite composed forward-acceleration objective; fallback samples use RMS((nextVf - prevVf) / 0.001) as undesired forward acceleration when the composed objective is inactive or non-finite.\",\n"
            << "    \"yaw_accel_error_first_100_ticks\": \"RMS((nextYawRate - prevYawRate) / 0.001 - DriveTelemetry.composedYawAccelRadps2) over the first 100 ticks for yaw-rate or heading steps when DriveTelemetry publishes a finite composed yaw-acceleration objective; fallback samples use RMS((nextYawRate - prevYawRate) / 0.001) as undesired yaw acceleration when the composed objective is inactive or non-finite.\"\n"
            << "  },\n"
            << "  \"score_weights\": {\n"
            << "    \"velocity_error_first_500_ticks\": ";
        WriteJsonNumber(std::cout, kVelocityStepRmsScoreWeight);
        std::cout << ",\n"
            << "    \"yaw_rate_error_first_500_ticks\": ";
        WriteJsonNumber(std::cout, kYawRateStepRmsScoreWeight);
        std::cout << ",\n"
            << "    \"forward_accel_error_first_100_ticks\": ";
        WriteJsonNumber(std::cout, kForwardAccelStepRmsScoreWeight);
        std::cout << ",\n"
            << "    \"yaw_accel_error_first_100_ticks\": ";
        WriteJsonNumber(std::cout, kYawAccelStepRmsScoreWeight);
        std::cout << ",\n"
            << "    \"acceptance_completion\": 0,\n"
            << "    \"acceptance_command_evidence\": 0,\n"
            << "    \"in_place_shift\": 0,\n"
            << "    \"in_place_heading\": 0,\n"
            << "    \"in_place_time\": 0,\n"
            << "    \"maneuver_tracking_rms\": 1,\n"
            << "    \"smooth_velocity_variation\": 0,\n"
            << "    \"smooth_yaw_acceleration_variation\": 0,\n"
            << "    \"smooth_yaw_rate_variation\": 0,\n"
            << "    \"smooth_final_position\": 0,\n"
            << "    \"smooth_final_heading\": 0,\n"
            << "    \"ripple_oscillation_deadband\": ";
        WriteJsonNumber(std::cout, kRippleOscillationDeadband);
        std::cout << "\n"
            << "  },\n"
            << "  \"baseline_source\": \"Config::kDriveBaseTrackingTuning\",\n"
            << "  \"current_drivebase_feedback_path\": {\n"
            << "    \"forward_axis_position_error_supplied\": false,\n"
            << "    \"forward_axis_acceleration_error_supplied_when_target_is_finite\": true,\n"
            << "    \"yaw_axis_acceleration_error_supplied_when_target_is_finite\": true\n"
            << "  },\n"
            << "  \"optimized_dimensions\": [\n";
        for (std::size_t index = 0U; index < static_cast<std::size_t>(GainIndex::Count); ++index)
        {
            std::cout << "    " << JsonString(GainName(static_cast<GainIndex>(index)))
                << ((index + 1U) < static_cast<std::size_t>(GainIndex::Count) ? "," : "") << "\n";
        }
        std::cout << "  ],\n";
        WriteEvaluationJson(std::cout, "baseline", baseline, 2);
        std::cout << ",\n";
        WriteEvaluationJson(std::cout, options.hasExplicitCandidate ? "candidate" : "default_candidate", candidate, 2);
        if (search != nullptr)
        {
            std::cout << ",\n"
                << "  \"search\": {\n"
                << "    \"enabled\": true,\n"
                << "    \"passes\": " << options.searchPasses << ",\n"
                << "    \"grid_points_per_dimension\": " << options.searchGridPoints << ",\n"
                << "    \"coordinate_grid\": \"log_spaced_positive_with_zero_endpoint\",\n"
                << "    \"positive_minimum_relative_to_range_max\": ";
            WriteJsonNumber(std::cout, kLogSearchMinimumRelativeToMaximum);
            std::cout << ",\n"
                << "    \"absolute_positive_minimum\": ";
            WriteJsonNumber(std::cout, kLogSearchAbsolutePositiveMinimum);
            std::cout << ",\n"
                << "    \"bounds\": ";
            WriteSearchBoundsJson(std::cout, options.candidate, options, 4);
            std::cout << ",\n"
                << "    \"evaluated_candidates\": " << search->evaluatedCandidates << ",\n";
            WriteEvaluationJson(std::cout, "best", search->best, 4);
            std::cout << ",\n"
                << "    \"top_candidates\": ";
            WriteTopCandidatesJson(std::cout, search->topCandidates, 4);
            std::cout << "\n  }\n";
        }
        else
        {
            std::cout << ",\n"
                << "  \"search\": { \"enabled\": false }\n";
        }
        std::cout << "}\n";
    }

int main(int argc, char* argv[])
{
    Options options{};
    std::string error;
    if (!ParseArgs(argc, argv, options, error))
    {
        std::cerr << error << "\n";
        return 1;
    }

    const GainSet baselineGains = ExtractProductionTunedGains();
    const EvaluationResult baseline = Evaluate(baselineGains);
    const EvaluationResult candidate = Evaluate(options.candidate);

    SearchResult searchResult{};
    const SearchResult* searchPtr = nullptr;
    if (options.runSearch)
    {
        searchResult = RunSearch(options.candidate, options);
        searchPtr = &searchResult;
    }

    WriteOutputJson(baseline, candidate, options, searchPtr);
    return (baseline.failed || candidate.failed || (searchPtr != nullptr && searchPtr->best.failed)) ? 2 : 0;
}
