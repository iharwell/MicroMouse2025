#include "..\..\MazeMap\MazeMap\CommandVector.h"
#include "..\..\MazeMap\MazeMap\CoreConfig.h"
#include "..\..\MazeMap\MazeMap\DirectionalLocation.h"
#include "..\..\MazeMap\MazeMap\Drive.h"
#include "..\..\MazeMap\MazeMap\DriveBase.h"
#include "..\..\MazeMap\MazeMap\DriveTelemetry.h"
#include "..\..\MazeMap\MazeMap\Estimator.h"
#include "..\..\MazeMap\MazeMap\ManeuverInstance.h"
#include "..\..\MazeMap\MazeMap\ManeuverSet.h"
#include "..\..\MazeMap\MazeMap\MazeLocation.h"
#include "..\..\MazeMap\MazeMap\MazeMapRuntimeCore.h"
#include "..\..\MazeMap\MazeMap\MotionLimits.h"
#include "..\..\MazeMap\MazeMap\PDCluster.h"
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
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace
{
    using CommandVector = MazeMap::App::Internal::CommandVector;

    constexpr double kTickSecondsExact = 0.001;
    constexpr float kTickSeconds = static_cast<float>(kTickSecondsExact);
    constexpr float kFanDuty = 0.80f;
    constexpr float kSaturationThreshold = 0.999f;
    constexpr std::size_t kDefaultSearchPasses = 4U;
    constexpr std::size_t kDefaultSearchGridPoints = 9U;
    constexpr std::size_t kDefaultTopCandidateCount = 8U;
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
    constexpr double kAcceptanceBlockerPenalty = 1000000.0;
    constexpr int kStartStraightMaxTicks = 6000;
    constexpr int kDriveManeuverMaxTicks = 20000;
    constexpr float kStartStraightDistanceM = 0.30f;
    constexpr float kStartStraightCruiseMps = 0.30f;
    constexpr float kSmoothManeuverEntrySpeedMps = 0.50f;
    constexpr float kManeuverHeadingToleranceRad = 3.0f * DEG_TO_RAD_F;
    constexpr float kManeuverPositionToleranceM = 0.030f;

    enum class SignalKind
    {
        ForwardVelocity,
        YawRate,
        Heading
    };

    enum class GainIndex : std::size_t
    {
        VelocityKp = 0U,
        VelocityKd,
        HeadingKp,
        HeadingKd,
        YawRateKp,
        YawRateKd,
        Count
    };

    struct GainSet
    {
        float velocityKp = 0.0f;
        float velocityKd = 0.0f;
        float headingKp = 0.0f;
        float headingKd = 0.0f;
        float yawRateKp = 0.0f;
        float yawRateKd = 0.0f;
    };

    struct ScenarioSpec
    {
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

    struct WheelObservationState
    {
        float leftDistanceM = 0.0f;
        float rightDistanceM = 0.0f;
    };

    struct ScopedFanDuty final
    {
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

    struct ScenarioMetrics
    {
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

    struct AcceptanceMetrics
    {
        std::string name;
        std::string description;
        std::string path;
        std::string code;
        bool started = false;
        bool completed = false;
        bool passed = false;
        bool blocker = false;
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
        int solverFailureCount = 0;
        int nonFiniteCount = 0;
        double scorePenalty = 0.0;
    };

    struct EvaluationResult
    {
        GainSet gains{};
        std::vector<ScenarioMetrics> scenarios;
        std::vector<AcceptanceMetrics> acceptanceScenarios;
        double score = 0.0;
        bool failed = false;
        bool oscillationFlagged = false;
        bool acceptanceBlocked = false;
    };

    struct Options
    {
        GainSet candidate{};
        bool hasExplicitCandidate = false;
        bool runSearch = false;
        std::size_t searchPasses = kDefaultSearchPasses;
        std::size_t searchGridPoints = kDefaultSearchGridPoints;
        std::size_t topCandidateCount = kDefaultTopCandidateCount;
    };

    struct SearchRange
    {
        float minimum = 0.0f;
        float maximum = 0.0f;
    };

    const char* GainName(GainIndex index) noexcept
    {
        switch (index)
        {
        case GainIndex::VelocityKp:
            return "velocity_kp";
        case GainIndex::VelocityKd:
            return "velocity_kd";
        case GainIndex::HeadingKp:
            return "heading_kp";
        case GainIndex::HeadingKd:
            return "heading_kd";
        case GainIndex::YawRateKp:
            return "yawrate_kp";
        case GainIndex::YawRateKd:
            return "yawrate_kd";
        case GainIndex::Count:
        default:
            return "unknown";
        }
    }

    const char* SignalName(SignalKind signal) noexcept
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

    float QuietNaN() noexcept
    {
        return std::numeric_limits<float>::quiet_NaN();
    }

    bool ParseFloatText(const std::string& text, float& value) noexcept
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

    bool ParseSizeText(const std::string& text, std::size_t& value) noexcept
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

    bool ReadOptionValue(int argc, char* argv[], int& index, const std::string& option, std::string& value, std::string& error)
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

    bool ApplyGainOption(const std::string& name, float value, GainSet& gains) noexcept
    {
        if ((name == "--velocity-kp") || (name == "--vel-kp"))
        {
            gains.velocityKp = value;
            return true;
        }
        if ((name == "--velocity-kd") || (name == "--vel-kd"))
        {
            gains.velocityKd = value;
            return true;
        }
        if (name == "--heading-kp")
        {
            gains.headingKp = value;
            return true;
        }
        if (name == "--heading-kd")
        {
            gains.headingKd = value;
            return true;
        }
        if ((name == "--yawrate-kp") || (name == "--yaw-rate-kp"))
        {
            gains.yawRateKp = value;
            return true;
        }
        if ((name == "--yawrate-kd") || (name == "--yaw-rate-kd"))
        {
            gains.yawRateKd = value;
            return true;
        }
        return false;
    }

    void PrintUsage()
    {
        std::cout
            << "PdTuning evaluates DriveBase PDCluster candidates against the current C++ PlantModel.\n\n"
            << "Usage:\n"
            << "  PdTuning.exe [gain options]\n"
            << "  PdTuning.exe --search [--search-passes N] [--search-grid N]\n\n"
            << "Gain options default to Config::kDriveBasePDCluster values:\n"
            << "  --velocity-kp V  --velocity-kd V\n"
            << "  --heading-kp V   --heading-kd V\n"
            << "  --yawrate-kp V   --yawrate-kd V\n\n"
            << "Stdout is JSON. Errors and this help text are not part of optimizer output.\n";
    }

    GainSet ExtractTunedGains(const MazeMap::PDCluster& cluster) noexcept
    {
        GainSet gains{};
        gains.velocityKp = cluster.VelocityStatePD.GetProportionalGain();
        gains.velocityKd = cluster.VelocityStatePD.GetDerivativeGain();
        gains.headingKp = cluster.HeadingStatePD.GetProportionalGain();
        gains.headingKd = cluster.HeadingStatePD.GetDerivativeGain();
        gains.yawRateKp = cluster.YawRateStatePD.GetProportionalGain();
        gains.yawRateKd = cluster.YawRateStatePD.GetDerivativeGain();
        return gains;
    }

    MazeMap::PDCluster BuildCandidateCluster(const GainSet& gains) noexcept
    {
        MazeMap::PDCluster cluster = MazeMap::Config::kDriveBasePDCluster;
        cluster.VelocityStatePD = MazeMap::ProportionalDerivative(gains.velocityKp, gains.velocityKd);
        cluster.HeadingStatePD = MazeMap::ProportionalDerivative(gains.headingKp, gains.headingKd);
        cluster.YawRateStatePD = MazeMap::ProportionalDerivative(gains.yawRateKp, gains.yawRateKd);
        return cluster;
    }

    void RebuildRuntimeDriveBaseForCandidate(
        MazeMap::App::Internal::SharedRobotRuntime& runtime,
        const MazeMap::PDCluster& cluster) noexcept
    {
        // Tool-only candidate injection: keep Drive attached to the runtime-owned DriveBase address.
        MazeMap::DriveBase& driveBase = runtime.DriveBase();
        driveBase.~DriveBase();
        (void)::new (static_cast<void*>(&driveBase))
            MazeMap::DriveBase(runtime.Plant(), runtime.RuntimeState(), cluster);
        runtime.DriveBase().ClearCommandEvidence();
    }

    MotionLimits MakePrimitiveLimits() noexcept
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

    float GetGain(const GainSet& gains, GainIndex index) noexcept
    {
        switch (index)
        {
        case GainIndex::VelocityKp:
            return gains.velocityKp;
        case GainIndex::VelocityKd:
            return gains.velocityKd;
        case GainIndex::HeadingKp:
            return gains.headingKp;
        case GainIndex::HeadingKd:
            return gains.headingKd;
        case GainIndex::YawRateKp:
            return gains.yawRateKp;
        case GainIndex::YawRateKd:
            return gains.yawRateKd;
        case GainIndex::Count:
        default:
            return 0.0f;
        }
    }

    void SetGain(GainSet& gains, GainIndex index, float value) noexcept
    {
        value = std::isfinite(value) ? (std::max)(0.0f, value) : 0.0f;
        switch (index)
        {
        case GainIndex::VelocityKp:
            gains.velocityKp = value;
            break;
        case GainIndex::VelocityKd:
            gains.velocityKd = value;
            break;
        case GainIndex::HeadingKp:
            gains.headingKp = value;
            break;
        case GainIndex::HeadingKd:
            gains.headingKd = value;
            break;
        case GainIndex::YawRateKp:
            gains.yawRateKp = value;
            break;
        case GainIndex::YawRateKd:
            gains.yawRateKd = value;
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

    std::array<ScenarioSpec, 7> BuildScenarioSpecs() noexcept
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

    float ScenarioTarget(const ScenarioSpec& spec) noexcept
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

    float ScenarioInitial(const ScenarioSpec& spec) noexcept
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

    float ScenarioScale(const ScenarioSpec& spec) noexcept
    {
        const float target = ScenarioTarget(spec);
        const float initial = ScenarioInitial(spec);
        const float excursion = (std::max)(
            std::fabs(target - initial),
            (std::max)(std::fabs(target), spec.tolerance * 10.0f));
        return (std::max)(excursion, 1.0e-6f);
    }

    float TargetKinematicLateralAccelMps2(const ScenarioSpec& spec) noexcept
    {
        return
            (std::isfinite(spec.targetForwardMps) && std::isfinite(spec.targetYawRateRadps)) ?
            (spec.targetForwardMps * spec.targetYawRateRadps) :
            QuietNaN();
    }

    bool HasLinearStepTransition(float initial, float target) noexcept
    {
        return
            std::isfinite(initial) &&
            std::isfinite(target) &&
            (std::fabs(target - initial) > kStepTransitionEpsilon);
    }

    bool HasHeadingStepTransition(float initialYawRad, float targetYawRad) noexcept
    {
        return
            std::isfinite(initialYawRad) &&
            std::isfinite(targetYawRad) &&
            (std::fabs(MazeMap::VehicleState::NormalizeAngle(targetYawRad - initialYawRad)) > kStepTransitionEpsilon);
    }

    bool HasActiveForwardAccelerationObjective(const DriveTelemetry& telemetry) noexcept
    {
        const bool hasVelocityFeedback =
            (telemetry.feedbackBranchFlags & DriveTelemetry::kFeedbackForwardVelocityInactive) == 0U;
        const bool hasRequestedAccel =
            (telemetry.scalarIntentFlags & DriveTelemetry::kScalarForwardAccelFinite) != 0U;
        return
            std::isfinite(telemetry.composedForwardAccelMps2) &&
            (hasVelocityFeedback || hasRequestedAccel);
    }

    bool HasActiveYawAccelerationObjective(const DriveTelemetry& telemetry) noexcept
    {
        const bool hasYawRateFeedback =
            (telemetry.feedbackBranchFlags & DriveTelemetry::kFeedbackYawRateInactive) == 0U;
        const bool hasHeadingFeedback =
            (telemetry.feedbackBranchFlags & DriveTelemetry::kFeedbackHeadingInactive) == 0U;
        const bool hasRequestedAccel =
            (telemetry.scalarIntentFlags & DriveTelemetry::kScalarYawAccelFinite) != 0U;
        return
            std::isfinite(telemetry.composedYawAccelRadps2) &&
            (hasYawRateFeedback || hasHeadingFeedback || hasRequestedAccel);
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

    double AxisStepScale(float initial, float target, float tolerance) noexcept
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

    double RmsOrZero(double rms, double scale) noexcept
    {
        if (!std::isfinite(rms) || !std::isfinite(scale) || (scale <= 0.0))
        {
            return 0.0;
        }
        return rms / scale;
    }

    double AccelerationRmsScale(double objectiveRms, double fallbackScale) noexcept
    {
        const double objectiveScale = std::isfinite(objectiveRms) ? objectiveRms : 0.0;
        return (std::max)((std::max)(objectiveScale, fallbackScale), 1.0e-6);
    }

    int ErrorSign(float error, float deadband) noexcept
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

    float SignalValue(const MazeMap::VehicleState::StateVector& state, SignalKind signal) noexcept
    {
        switch (signal)
        {
        case SignalKind::ForwardVelocity:
            return state(MazeMap::VehicleState::kU);
        case SignalKind::YawRate:
            return state(MazeMap::VehicleState::kR);
        case SignalKind::Heading:
            return state(MazeMap::VehicleState::kPsi);
        default:
            return 0.0f;
        }
    }

    float SignalError(const ScenarioSpec& spec, const MazeMap::VehicleState::StateVector& state) noexcept
    {
        const float target = ScenarioTarget(spec);
        const float value = SignalValue(state, spec.signal);
        if (spec.signal == SignalKind::Heading)
        {
            return MazeMap::VehicleState::NormalizeAngle(target - value);
        }
        return target - value;
    }

    MazeMap::VehicleState::StateVector BuildInitialState(
        const ScenarioSpec& spec,
        const MazeMap::PlantModel::PreparedParams& params) noexcept
    {
        MazeMap::VehicleState::StateVector state = MazeMap::VehicleState::StateVector::Zero();
        state(MazeMap::VehicleState::kPsi) = spec.initialYawRad;
        state(MazeMap::VehicleState::kU) = spec.initialForwardMps;
        state(MazeMap::VehicleState::kR) = spec.initialYawRateRadps;
        state(MazeMap::VehicleState::kOmegaL) =
            (spec.initialForwardMps + (params.halfTrackWidthM * spec.initialYawRateRadps)) *
            params.invWheelRadiusM;
        state(MazeMap::VehicleState::kOmegaR) =
            (spec.initialForwardMps - (params.halfTrackWidthM * spec.initialYawRateRadps)) *
            params.invWheelRadiusM;
        MazeMap::VehicleState::NormalizeStateVector(state);
        return state;
    }

    void PublishTruthToRuntime(
        MazeMap::VehicleState& runtimeState,
        const MazeMap::VehicleState::StateVector& truth,
        const WheelObservationState& wheels,
        float leftDistanceDeltaM,
        float rightDistanceDeltaM,
        float dtSeconds,
        const MazeMap::PlantModel::PreparedParams& params) noexcept
    {
        SensorSnapshot snapshot{};
        snapshot.gyroRawRadps = truth(MazeMap::VehicleState::kR);
        snapshot.gyroRadps = truth(MazeMap::VehicleState::kR);
        snapshot.encoderObservationValid = true;
        snapshot.leftEncoderDistanceM = wheels.leftDistanceM;
        snapshot.rightEncoderDistanceM = wheels.rightDistanceM;
        snapshot.encoderObservation.leftDistanceDeltaM = leftDistanceDeltaM;
        snapshot.encoderObservation.rightDistanceDeltaM = rightDistanceDeltaM;
        snapshot.encoderObservation.leftVelocityMps = truth(MazeMap::VehicleState::kOmegaL) * params.wheelRadiusM;
        snapshot.encoderObservation.rightVelocityMps = truth(MazeMap::VehicleState::kOmegaR) * params.wheelRadiusM;
        snapshot.encoderObservation.omegaLeftRadps = truth(MazeMap::VehicleState::kOmegaL);
        snapshot.encoderObservation.omegaRightRadps = truth(MazeMap::VehicleState::kOmegaR);

        runtimeState.SetPosition(Eigen::Vector2f(truth(MazeMap::VehicleState::kPx), truth(MazeMap::VehicleState::kPy)));
        runtimeState.SetOrientation(truth(MazeMap::VehicleState::kPsi));
        runtimeState.SetVelocity(truth(MazeMap::VehicleState::kU));
        runtimeState.SetLateralVelocity(truth(MazeMap::VehicleState::kV));
        runtimeState.SetRotationalVelocity(truth(MazeMap::VehicleState::kR));
        runtimeState.SetWheelSpeedLeft(truth(MazeMap::VehicleState::kOmegaL));
        runtimeState.SetWheelSpeedRight(truth(MazeMap::VehicleState::kOmegaR));
        runtimeState.SetGyroBiasZ(truth(MazeMap::VehicleState::kBgz));
        runtimeState.SetLongitudinalAcceleration(0.0f);
        runtimeState.SetLateralAcceleration(0.0f);
        runtimeState.SetYawAcceleration(0.0f);
        runtimeState.SetTime(runtimeState.GetTime() + dtSeconds);
        runtimeState.SetTimestampUs(static_cast<std::uint32_t>(runtimeState.GetTime() * 1000000.0f));
        runtimeState.SetSensorSnapshot(snapshot);
    }

    void AdvanceTruth(
        const MazeMap::PlantModel& plant,
        MazeMap::VehicleState& runtimeState,
        MazeMap::VehicleState::StateVector& truth,
        WheelObservationState& wheels,
        const CommandVector& control,
        const MazeMap::PlantModel::PreparedParams& params) noexcept
    {
        const MazeMap::VehicleState::StateVector previous = truth;
        truth = plant.integrate(previous, control, kTickSeconds, params);

        const float leftDeltaM =
            0.5f *
            (previous(MazeMap::VehicleState::kOmegaL) + truth(MazeMap::VehicleState::kOmegaL)) *
            params.wheelRadiusM *
            kTickSeconds;
        const float rightDeltaM =
            0.5f *
            (previous(MazeMap::VehicleState::kOmegaR) + truth(MazeMap::VehicleState::kOmegaR)) *
            params.wheelRadiusM *
            kTickSeconds;
        wheels.leftDistanceM += leftDeltaM;
        wheels.rightDistanceM += rightDeltaM;
        PublishTruthToRuntime(runtimeState, truth, wheels, leftDeltaM, rightDeltaM, kTickSeconds, params);
    }

    MazeMap::VehicleState::StateVector BuildAcceptanceTruthState(
        const float forwardMps,
        const float yawRad,
        const MazeMap::PlantModel::PreparedParams& params) noexcept
    {
        MazeMap::VehicleState::StateVector state = MazeMap::VehicleState::StateVector::Zero();
        state(MazeMap::VehicleState::kPsi) = yawRad;
        state(MazeMap::VehicleState::kU) = forwardMps;
        state(MazeMap::VehicleState::kOmegaL) = forwardMps * params.invWheelRadiusM;
        state(MazeMap::VehicleState::kOmegaR) = forwardMps * params.invWheelRadiusM;
        MazeMap::VehicleState::NormalizeStateVector(state);
        return state;
    }

    float AverageEncoderDistanceM(const WheelObservationState& wheels) noexcept
    {
        return 0.5f * (wheels.leftDistanceM + wheels.rightDistanceM);
    }

    SensorSnapshot BuildDriveManeuverSensorSnapshot(const float yawRateRadps = 0.0f) noexcept
    {
        SensorSnapshot snapshot{};
        snapshot.gyroRawRadps = yawRateRadps;
        snapshot.gyroRadps = yawRateRadps;
        return snapshot;
    }

    int32_t ConsumeWholeEncoderCounts(const float deltaCounts, float& remainderCounts) noexcept
    {
        remainderCounts += deltaCounts;
        const int32_t wholeCounts =
            (remainderCounts >= 0.0f) ?
            static_cast<int32_t>(std::floor(remainderCounts)) :
            static_cast<int32_t>(std::ceil(remainderCounts));
        remainderCounts -= static_cast<float>(wholeCounts);
        return wholeCounts;
    }

    void ApplyEncoderObservation(
        MazeMap::App::Internal::SharedRobotRuntime& runtime,
        const float leftDistanceDeltaM,
        const float rightDistanceDeltaM,
        const float yawRateRadps,
        float& leftEncoderRemainderCounts,
        float& rightEncoderRemainderCounts,
        const float dtSeconds,
        const CommandVector& appliedControl)
    {
        const float distancePerCountM = MazeMap::Vehicle::DriveEncoderDistanceFromCounts(1);
        const int32_t leftCounts =
            ConsumeWholeEncoderCounts(leftDistanceDeltaM / distancePerCountM, leftEncoderRemainderCounts);
        const int32_t rightCounts =
            ConsumeWholeEncoderCounts(rightDistanceDeltaM / distancePerCountM, rightEncoderRemainderCounts);

        MazeMap::VehicleState& runtimeState = runtime.RuntimeState();
        SensorSnapshot snapshot = BuildDriveManeuverSensorSnapshot(yawRateRadps);
        snapshot.encoderObservation.totalLeftCounts = leftCounts;
        snapshot.encoderObservation.totalRightCounts = rightCounts;
        snapshot.encoderObservation.leftDistanceDeltaM = static_cast<float>(leftCounts) * distancePerCountM;
        snapshot.encoderObservation.rightDistanceDeltaM = static_cast<float>(rightCounts) * distancePerCountM;
        if ((dtSeconds > 0.0f) && std::isfinite(dtSeconds))
        {
            const float invDtSeconds = 1.0f / dtSeconds;
            snapshot.encoderObservation.leftVelocityMps =
                snapshot.encoderObservation.leftDistanceDeltaM * invDtSeconds;
            snapshot.encoderObservation.rightVelocityMps =
                snapshot.encoderObservation.rightDistanceDeltaM * invDtSeconds;
            snapshot.encoderObservation.omegaLeftRadps =
                MazeMap::Vehicle::WheelOmegaFromLinearVelocity(snapshot.encoderObservation.leftVelocityMps);
            snapshot.encoderObservation.omegaRightRadps =
                MazeMap::Vehicle::WheelOmegaFromLinearVelocity(snapshot.encoderObservation.rightVelocityMps);
        }
        snapshot.encoderObservationValid = true;
        snapshot.leftEncoderTotalCounts =
            runtimeState.GetSensorSnapshot().leftEncoderTotalCounts + static_cast<std::int64_t>(leftCounts);
        snapshot.rightEncoderTotalCounts =
            runtimeState.GetSensorSnapshot().rightEncoderTotalCounts + static_cast<std::int64_t>(rightCounts);
        snapshot.leftEncoderDistanceM =
            MazeMap::Vehicle::DriveEncoderDistanceFromCounts(snapshot.leftEncoderTotalCounts);
        snapshot.rightEncoderDistanceM =
            MazeMap::Vehicle::DriveEncoderDistanceFromCounts(snapshot.rightEncoderTotalCounts);

        runtimeState.SetSensorSnapshot(snapshot);
        if (std::isfinite(dtSeconds) && (dtSeconds > 0.0f))
        {
            runtimeState.SetTime(runtimeState.GetTime() + dtSeconds);
            runtimeState.SetTimestampUs(static_cast<std::uint32_t>(runtimeState.GetTime() * 1000000.0f));
        }

        MazeMap::Estimator& estimator = runtime.Estimator();
        if (estimator.HasFault())
        {
            return;
        }

        if (std::isfinite(dtSeconds) && (dtSeconds > 0.0f) && !estimator.predict(dtSeconds, appliedControl))
        {
            return;
        }

        if (snapshot.encoderObservationValid)
        {
            (void)estimator.updateEncoderPair(snapshot.encoderObservation, dtSeconds, false);
        }

        if (std::isfinite(snapshot.gyroRawRadps))
        {
            const MazeMap::MeasurementUpdateResult yawUpdate = estimator.updateYawRate(snapshot.gyroRawRadps);
            if (!yawUpdate.accepted)
            {
                return;
            }
        }

        MazeMap::ImuAccelObs accelObservation{};
        (void)estimator.updatePlanarAccel(accelObservation);
    }

    void PrimeDriveForSmoothEntry(
        MazeMap::App::Internal::SharedRobotRuntime& runtime,
        MazeMap::VehicleState::StateVector& truth,
        float& leftEncoderRemainderCounts,
        float& rightEncoderRemainderCounts,
        const MazeMap::PlantModel::PreparedParams& params)
    {
        truth = BuildAcceptanceTruthState(kSmoothManeuverEntrySpeedMps, 0.0f, params);

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
        ApplyEncoderObservation(
            runtime,
            kSmoothManeuverEntrySpeedMps * kTickSeconds,
            kSmoothManeuverEntrySpeedMps * kTickSeconds,
            0.0f,
            leftEncoderRemainderCounts,
            rightEncoderRemainderCounts,
            kTickSeconds,
            CommandVector::Brake());
    }

    void AdvanceRuntimeDriveCycle(
        MazeMap::App::Internal::SharedRobotRuntime& runtime,
        MazeMap::VehicleState::StateVector& truth,
        float& leftEncoderRemainderCounts,
        float& rightEncoderRemainderCounts,
        const CommandVector& control,
        const MazeMap::PlantModel::PreparedParams& params)
    {
        const MazeMap::VehicleState::StateVector previousTruth = truth;
        truth = runtime.Plant().integrate(truth, control, kTickSeconds, params);

        const float leftDistanceDeltaM =
            0.5f *
            (previousTruth(MazeMap::VehicleState::kOmegaL) + truth(MazeMap::VehicleState::kOmegaL)) *
            params.wheelRadiusM *
            kTickSeconds;
        const float rightDistanceDeltaM =
            0.5f *
            (previousTruth(MazeMap::VehicleState::kOmegaR) + truth(MazeMap::VehicleState::kOmegaR)) *
            params.wheelRadiusM *
            kTickSeconds;

        ApplyEncoderObservation(
            runtime,
            leftDistanceDeltaM,
            rightDistanceDeltaM,
            truth(MazeMap::VehicleState::kR),
            leftEncoderRemainderCounts,
            rightEncoderRemainderCounts,
            kTickSeconds,
            control);
    }

    void RecordAcceptanceTelemetry(
        AcceptanceMetrics& metrics,
        const CommandVector& control,
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
        metrics.solverClean = metrics.solverClean && (telemetry.solverFailureFlags == 0U);
        if (telemetry.solverFailureFlags != 0U)
        {
            ++metrics.solverFailureCount;
        }
        if (!control.IsFinite())
        {
            ++metrics.nonFiniteCount;
        }
    }

    void CaptureAcceptanceFinalState(
        AcceptanceMetrics& metrics,
        const MazeMap::VehicleState::StateVector& truth,
        const WheelObservationState& wheels) noexcept
    {
        metrics.truthFinite = metrics.truthFinite && truth.allFinite();
        if (!truth.allFinite())
        {
            ++metrics.nonFiniteCount;
        }
        metrics.encoderAverageDistanceM = AverageEncoderDistanceM(wheels);
        metrics.finalXM = truth(MazeMap::VehicleState::kPx);
        metrics.finalYM = truth(MazeMap::VehicleState::kPy);
        metrics.finalYawRad = truth(MazeMap::VehicleState::kPsi);
        metrics.elapsedSeconds = static_cast<float>(metrics.appliedTicks) * kTickSeconds;
    }

    double ComputeAcceptancePenalty(const AcceptanceMetrics& metrics) noexcept
    {
        if (metrics.passed)
        {
            return 0.0;
        }

        double penalty = kAcceptanceBlockerPenalty;
        if (!metrics.completed)
        {
            penalty += 0.5 * kAcceptanceBlockerPenalty;
        }
        if (std::isfinite(metrics.finalHeadingErrorRad) && std::isfinite(metrics.headingToleranceRad))
        {
            const double excessRad =
                (std::max)(0.0, static_cast<double>(metrics.finalHeadingErrorRad - metrics.headingToleranceRad));
            penalty += 500000.0 * excessRad;
        }
        if (std::isfinite(metrics.finalPositionErrorM) && std::isfinite(metrics.positionToleranceM))
        {
            const double excessM =
                (std::max)(0.0, static_cast<double>(metrics.finalPositionErrorM - metrics.positionToleranceM));
            penalty += 10000.0 * excessM;
        }
        penalty += 100000.0 * static_cast<double>(metrics.nonFiniteCount + metrics.solverFailureCount);
        return penalty;
    }

    void FinalizeAcceptance(AcceptanceMetrics& metrics, const bool acceptanceCondition) noexcept
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
        metrics.blocker = !metrics.passed;
        metrics.scorePenalty = ComputeAcceptancePenalty(metrics);
    }

    AcceptanceMetrics RunStartStraightAcceptance(const GainSet& gains)
    {
        MazeMap::PDCluster cluster = BuildCandidateCluster(gains);
        MazeMap::App::Internal::SharedRobotRuntime runtime(kTickSeconds);
        RebuildRuntimeDriveBaseForCandidate(runtime, cluster);
        ScopedFanDuty fanDuty(runtime.Vehicle(), kFanDuty);

        const MazeMap::PlantParams rawParams = MazeMap::PlantParams::Default();
        const MazeMap::PlantModel::PreparedParams params = MazeMap::PlantModel::Prepare(rawParams);
        WheelObservationState wheels{};
        MazeMap::VehicleState::StateVector truth = BuildAcceptanceTruthState(0.0f, 0.0f, params);
        PublishTruthToRuntime(runtime.RuntimeState(), truth, wheels, 0.0f, 0.0f, 0.0f, params);

        AcceptanceMetrics metrics{};
        metrics.name = "drive_primitive_start_straight_completes";
        metrics.description =
            "Drive::StartStraight closed-loop acceptance through SharedRobotRuntime, Drive, DriveBase, PlantModel::integrate, and sensor snapshot publication.";
        metrics.path = "SharedRobotRuntime.DriveService.StartStraight";
        metrics.code = "StartStraight";
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
            const CommandVector control = drive.GetNextControls(done);
            if (done)
            {
                metrics.completed = true;
                break;
            }

            RecordAcceptanceTelemetry(metrics, control, runtime.DriveBase().LastTelemetry());
            AdvanceTruth(runtime.Plant(), runtime.RuntimeState(), truth, wheels, control, params);
            ++metrics.appliedTicks;
            metrics.truthFinite = metrics.truthFinite && truth.allFinite();
            if (!truth.allFinite())
            {
                ++metrics.nonFiniteCount;
                break;
            }
        }

        CaptureAcceptanceFinalState(metrics, truth, wheels);
        FinalizeAcceptance(metrics, metrics.completed);
        return metrics;
    }

    MazeMap::DirectionalLocation BuildManeuverStart() noexcept
    {
        return MazeMap::DirectionalLocation(MazeMap::MazeLocation(0U, 0U), MazeMap::Up);
    }

    MazeMap::DirectionalLocation BuildNominalEndLocation(const MazeMap::ManeuverCode code)
    {
        return MazeMap::ManeuverSet::GetSet().Move(code, BuildManeuverStart());
    }

    float BuildNominalEndXMeters(const MazeMap::ManeuverCode code)
    {
        const MazeMap::DirectionalLocation nominalEnd = BuildNominalEndLocation(code);
        return 0.5f * MazeMap::Config::kCellSizeM * static_cast<float>(nominalEnd.GetLocation().GetX());
    }

    float BuildNominalEndYMeters(const MazeMap::ManeuverCode code)
    {
        const MazeMap::DirectionalLocation nominalEnd = BuildNominalEndLocation(code);
        return 0.5f * MazeMap::Config::kCellSizeM * static_cast<float>(nominalEnd.GetLocation().GetY());
    }

    float BuildNominalEndYawRad(const MazeMap::ManeuverCode code)
    {
        return DirectionToYawRad(BuildNominalEndLocation(code).GetDirection());
    }

    const char* ManeuverCodeLabel(const MazeMap::ManeuverCode code) noexcept
    {
        switch (code)
        {
        case MazeMap::S135SD:
            return "S135SD";
        case MazeMap::S135LS:
            return "S135LS";
        case MazeMap::S135LD:
            return "S135LD";
        case MazeMap::S180SS:
            return "S180SS";
        default:
            return "UNKNOWN";
        }
    }

    AcceptanceMetrics RunSmoothManeuverHeadingAcceptance(
        const GainSet& gains,
        const MazeMap::ManeuverCode code)
    {
        MazeMap::PDCluster cluster = BuildCandidateCluster(gains);
        MazeMap::App::Internal::SharedRobotRuntime runtime(kTickSeconds);
        RebuildRuntimeDriveBaseForCandidate(runtime, cluster);
        ScopedFanDuty fanDuty(runtime.Vehicle(), kFanDuty);

        const MazeMap::PlantParams rawParams = MazeMap::PlantParams::Default();
        const MazeMap::PlantModel::PreparedParams params = MazeMap::PlantModel::Prepare(rawParams);
        float leftEncoderRemainderCounts = 0.0f;
        float rightEncoderRemainderCounts = 0.0f;
        MazeMap::VehicleState::StateVector truth = MazeMap::VehicleState::StateVector::Zero();
        PrimeDriveForSmoothEntry(
            runtime,
            truth,
            leftEncoderRemainderCounts,
            rightEncoderRemainderCounts,
            params);

        AcceptanceMetrics metrics{};
        metrics.name = std::string("drive_maneuver_") + ManeuverCodeLabel(code) + "_final_heading_acceptance";
        std::transform(metrics.name.begin(), metrics.name.end(), metrics.name.begin(),
            [](const unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
        metrics.description =
            "Smooth Drive::StartManeuver final-heading acceptance through SharedRobotRuntime, Drive, DriveBase, PlantModel::integrate, and direct truth-to-sensor snapshot publication.";
        metrics.path = "SharedRobotRuntime.DriveService.StartManeuver";
        metrics.code = ManeuverCodeLabel(code);
        metrics.started = true;
        metrics.maxTicks = kDriveManeuverMaxTicks;
        metrics.targetDistanceM = MazeMap::ManeuverSet::GetSet().GetTravelDistanceMeters(code, MazeMap::Config::kCellSizeM);
        metrics.targetYawRad = BuildNominalEndYawRad(code);
        metrics.headingToleranceRad = kManeuverHeadingToleranceRad;
        metrics.headingToleranceDeg = kManeuverHeadingToleranceRad * RAD_TO_DEG_F;
        metrics.positionToleranceM = kManeuverPositionToleranceM;

        MazeMap::ManeuverInstance maneuver(
            code,
            BuildManeuverStart(),
            kSmoothManeuverEntrySpeedMps,
            kSmoothManeuverEntrySpeedMps);

        MazeMap::App::Internal::Drive& drive = runtime.DriveService();
        drive.StartManeuver(maneuver);

        for (int tick = 0; tick < kDriveManeuverMaxTicks; ++tick)
        {
            bool done = false;
            const CommandVector control = drive.GetNextControls(done);
            if (done)
            {
                metrics.completed = true;
                break;
            }

            RecordAcceptanceTelemetry(metrics, control, runtime.DriveBase().LastTelemetry());
            AdvanceRuntimeDriveCycle(
                runtime,
                truth,
                leftEncoderRemainderCounts,
                rightEncoderRemainderCounts,
                control,
                params);
            ++metrics.appliedTicks;
            metrics.truthFinite = metrics.truthFinite && truth.allFinite();
            if (!truth.allFinite())
            {
                ++metrics.nonFiniteCount;
                break;
            }
        }

        WheelObservationState wheels{};
        const SensorSnapshot& finalSnapshot = runtime.RuntimeState().GetSensorSnapshot();
        wheels.leftDistanceM = finalSnapshot.leftEncoderDistanceM;
        wheels.rightDistanceM = finalSnapshot.rightEncoderDistanceM;
        CaptureAcceptanceFinalState(metrics, truth, wheels);
        metrics.finalHeadingErrorRad =
            std::fabs(AngleErrorRad(metrics.targetYawRad, metrics.finalYawRad));
        metrics.finalHeadingErrorDeg = metrics.finalHeadingErrorRad * RAD_TO_DEG_F;
        metrics.finalPositionErrorM =
            std::hypot(
                metrics.finalXM - BuildNominalEndXMeters(code),
                metrics.finalYM - BuildNominalEndYMeters(code));
        FinalizeAcceptance(
            metrics,
            metrics.completed &&
            std::isfinite(metrics.finalHeadingErrorRad) &&
            (metrics.finalHeadingErrorRad <= kManeuverHeadingToleranceRad));
        return metrics;
    }

    double ComputeScenarioScore(const ScenarioMetrics& metrics) noexcept
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

    double ComputeResponseFailurePenalty(const ScenarioMetrics& metrics) noexcept
    {
        const double excursion = (std::max)(
            static_cast<double>(std::fabs(metrics.target - metrics.initial)),
            (std::max)(static_cast<double>(std::fabs(metrics.target)), static_cast<double>(metrics.tolerance * 10.0f)));
        const double scale = (std::max)(excursion, 1.0e-6);
        const double minimumErrorTerm = static_cast<double>(metrics.minimumAbsError) / scale;
        const double poorResponseExcess = (std::max)(0.0, minimumErrorTerm - 0.60);

        return 500000.0 * poorResponseExcess * poorResponseExcess;
    }

    bool IsScenarioOscillatory(const ScenarioMetrics& metrics) noexcept
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

    double ComputeOscillationPenalty(const ScenarioMetrics& metrics) noexcept
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

    ScenarioMetrics RunScenario(const ScenarioSpec& spec, const GainSet& gains)
    {
        MazeMap::Vehicle vehicle{};
        vehicle.SetFanDuty(kFanDuty);
        MazeMap::VehicleState runtimeState{};
        MazeMap::PlantModel plant(vehicle, runtimeState);
        const MazeMap::PlantParams rawParams = MazeMap::PlantParams::Default();
        const MazeMap::PlantModel::PreparedParams params = MazeMap::PlantModel::Prepare(rawParams);
        MazeMap::PDCluster cluster = BuildCandidateCluster(gains);
        MazeMap::DriveBase driveBase(plant, runtimeState, cluster);

        WheelObservationState wheels{};
        MazeMap::VehicleState::StateVector truth = BuildInitialState(spec, params);
        PublishTruthToRuntime(runtimeState, truth, wheels, 0.0f, 0.0f, 0.0f, params);
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

        const float initialError = SignalError(spec, truth);
        const float initialAbsError = std::fabs(initialError);
        metrics.minimumAbsError = initialAbsError;
        metrics.maxAbsForwardVelocityMps = std::fabs(truth(MazeMap::VehicleState::kU));
        metrics.maxAbsYawRateRadps = std::fabs(truth(MazeMap::VehicleState::kR));
        metrics.maxAbsKinematicLateralAccelMps2 =
            std::fabs(truth(MazeMap::VehicleState::kU) * truth(MazeMap::VehicleState::kR));
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
            const CommandVector control =
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
            if (telemetry.solverFailureFlags != 0U)
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

            const MazeMap::VehicleState::StateVector previousTruth = truth;
            AdvanceTruth(plant, runtimeState, truth, wheels, control, params);
            ++metrics.samples;

            const float forwardAccelMps2 =
                (truth(MazeMap::VehicleState::kU) - previousTruth(MazeMap::VehicleState::kU)) / kTickSeconds;
            const float yawAccelRadps2 =
                (truth(MazeMap::VehicleState::kR) - previousTruth(MazeMap::VehicleState::kR)) / kTickSeconds;
            metrics.maxAbsForwardVelocityMps =
                (std::max)(metrics.maxAbsForwardVelocityMps, std::fabs(truth(MazeMap::VehicleState::kU)));
            metrics.maxAbsYawRateRadps =
                (std::max)(metrics.maxAbsYawRateRadps, std::fabs(truth(MazeMap::VehicleState::kR)));
            metrics.maxAbsForwardAccelMps2 =
                (std::max)(metrics.maxAbsForwardAccelMps2, std::fabs(forwardAccelMps2));
            metrics.maxAbsYawAccelRadps2 =
                (std::max)(metrics.maxAbsYawAccelRadps2, std::fabs(yawAccelRadps2));
            metrics.maxAbsKinematicLateralAccelMps2 =
                (std::max)(
                    metrics.maxAbsKinematicLateralAccelMps2,
                    std::fabs(truth(MazeMap::VehicleState::kU) * truth(MazeMap::VehicleState::kR)));

            if (metrics.forwardVelocityStepActive && (sample < kVelocityStepRmsWindowTicks))
            {
                const float velocityErrorMps = spec.targetForwardMps - truth(MazeMap::VehicleState::kU);
                if (std::isfinite(velocityErrorMps))
                {
                    first500VelocitySquaredError +=
                        static_cast<double>(velocityErrorMps) * static_cast<double>(velocityErrorMps);
                    ++metrics.first500VelocityErrorSamples;
                }
            }

            if (metrics.yawRateStepActive && (sample < kYawRateStepRmsWindowTicks))
            {
                const float yawRateErrorRadps = spec.targetYawRateRadps - truth(MazeMap::VehicleState::kR);
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
                if (HasActiveForwardAccelerationObjective(telemetry))
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
                if (HasActiveYawAccelerationObjective(telemetry))
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

            const float error = SignalError(spec, truth);
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

            if (!truth.allFinite())
            {
                ++metrics.nonFiniteCount;
                metrics.failed = true;
                break;
            }
        }

        metrics.finalValue = SignalValue(truth, spec.signal);
        metrics.finalError = SignalError(spec, truth);
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

    std::vector<AcceptanceMetrics> RunAcceptanceScenarios(const GainSet& gains)
    {
        std::vector<AcceptanceMetrics> acceptances;
        acceptances.reserve(5U);
        acceptances.push_back(RunStartStraightAcceptance(gains));
        acceptances.push_back(RunSmoothManeuverHeadingAcceptance(gains, MazeMap::S180SS));
        acceptances.push_back(RunSmoothManeuverHeadingAcceptance(gains, MazeMap::S135SD));
        acceptances.push_back(RunSmoothManeuverHeadingAcceptance(gains, MazeMap::S135LD));
        acceptances.push_back(RunSmoothManeuverHeadingAcceptance(gains, MazeMap::S135LS));
        return acceptances;
    }

    EvaluationResult Evaluate(const GainSet& gains)
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
            result.failed = result.failed || acceptance.blocker;
            result.score += acceptance.scorePenalty;
        }
        return result;
    }

    SearchRange BuildRange(float seed, float fallbackMaximum, float multiplier) noexcept
    {
        const float maximum = (std::max)(fallbackMaximum, std::isfinite(seed) ? (seed * multiplier) : fallbackMaximum);
        return { 0.0f, maximum };
    }

    std::array<SearchRange, static_cast<std::size_t>(GainIndex::Count)> BuildInitialSearchRanges(const GainSet& seed) noexcept
    {
        return {{
            BuildRange(seed.velocityKp, 120.0f, 5.0f),
            BuildRange(seed.velocityKd, 8.0f, 500.0f),
            BuildRange(seed.headingKp, 120.0f, 5.0f),
            BuildRange(seed.headingKd, 40.0f, 5.0f),
            BuildRange(seed.yawRateKp, 220.0f, 5.0f),
            BuildRange(seed.yawRateKd, 60.0f, 5.0f)
        }};
    }

    std::vector<float> BuildGridValues(const SearchRange& range, std::size_t gridPoints)
    {
        gridPoints = (std::max)(static_cast<std::size_t>(2U), gridPoints);
        std::vector<float> values;
        values.reserve(gridPoints);
        if (!(std::isfinite(range.minimum) && std::isfinite(range.maximum)) || (range.maximum <= range.minimum))
        {
            values.push_back((std::max)(0.0f, range.minimum));
            return values;
        }

        for (std::size_t index = 0U; index < gridPoints; ++index)
        {
            const float fraction =
                static_cast<float>(index) / static_cast<float>(gridPoints - 1U);
            values.push_back(range.minimum + (fraction * (range.maximum - range.minimum)));
        }
        return values;
    }

    void AddTopCandidate(std::vector<EvaluationResult>& topCandidates, EvaluationResult candidate, std::size_t maxCount)
    {
        if (maxCount == 0U)
        {
            return;
        }

        auto insertAt = std::find_if(
            topCandidates.begin(),
            topCandidates.end(),
            [&candidate](const EvaluationResult& existing)
            {
                return candidate.score < existing.score;
            });
        topCandidates.insert(insertAt, std::move(candidate));
        if (topCandidates.size() > maxCount)
        {
            topCandidates.pop_back();
        }
    }

    struct SearchResult
    {
        EvaluationResult best{};
        std::vector<EvaluationResult> topCandidates;
        std::size_t evaluatedCandidates = 0U;
    };

    SearchResult RunSearch(const GainSet& seed, const Options& options)
    {
        SearchResult result{};
        result.best = Evaluate(seed);
        result.evaluatedCandidates = 1U;
        AddTopCandidate(result.topCandidates, result.best, options.topCandidateCount);

        std::array<SearchRange, static_cast<std::size_t>(GainIndex::Count)> ranges = BuildInitialSearchRanges(seed);
        const std::array<SearchRange, static_cast<std::size_t>(GainIndex::Count)> absoluteRanges = ranges;

        for (std::size_t pass = 0U; pass < options.searchPasses; ++pass)
        {
            for (std::size_t rawIndex = 0U; rawIndex < static_cast<std::size_t>(GainIndex::Count); ++rawIndex)
            {
                const GainIndex gainIndex = static_cast<GainIndex>(rawIndex);
                const std::vector<float> values = BuildGridValues(ranges[rawIndex], options.searchGridPoints);
                for (float value : values)
                {
                    GainSet candidate = result.best.gains;
                    SetGain(candidate, gainIndex, value);
                    EvaluationResult evaluation = Evaluate(candidate);
                    ++result.evaluatedCandidates;
                    if (evaluation.score < result.best.score)
                    {
                        result.best = evaluation;
                    }
                    AddTopCandidate(result.topCandidates, std::move(evaluation), options.topCandidateCount);
                }

                const float span = ranges[rawIndex].maximum - ranges[rawIndex].minimum;
                const float step =
                    (options.searchGridPoints > 1U) ?
                    (span / static_cast<float>(options.searchGridPoints - 1U)) :
                    span;
                const float center = GetGain(result.best.gains, gainIndex);
                ranges[rawIndex].minimum =
                    (std::max)(absoluteRanges[rawIndex].minimum, center - step);
                ranges[rawIndex].maximum =
                    (std::min)(absoluteRanges[rawIndex].maximum, center + step);
            }
        }

        return result;
    }

    bool ParseArgs(int argc, char* argv[], Options& options, std::string& error)
    {
        options.candidate = ExtractTunedGains(MazeMap::Config::kDriveBasePDCluster);
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
            if (ReadOptionValue(argc, argv, index, "--search-grid", value, error))
            {
                if (!ParseSizeText(value, options.searchGridPoints) || (options.searchGridPoints < 2U))
                {
                    error = "Invalid --search-grid value: " + value;
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

        return true;
    }

    std::string JsonString(const std::string& value)
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

    void WriteJsonNumber(std::ostream& output, double value)
    {
        if (!std::isfinite(value))
        {
            output << "null";
            return;
        }
        output << std::setprecision(10) << value;
    }

    void WriteGainsJson(std::ostream& output, const GainSet& gains, int indent)
    {
        const std::string pad(static_cast<std::size_t>(indent), ' ');
        output << "{\n"
            << pad << "  \"velocity_state_pd\": { \"kp\": ";
        WriteJsonNumber(output, gains.velocityKp);
        output << ", \"kd\": ";
        WriteJsonNumber(output, gains.velocityKd);
        output << " },\n"
            << pad << "  \"heading_state_pd\": { \"kp\": ";
        WriteJsonNumber(output, gains.headingKp);
        output << ", \"kd\": ";
        WriteJsonNumber(output, gains.headingKd);
        output << " },\n"
            << pad << "  \"yaw_rate_state_pd\": { \"kp\": ";
        WriteJsonNumber(output, gains.yawRateKp);
        output << ", \"kd\": ";
        WriteJsonNumber(output, gains.yawRateKd);
        output << " }\n"
            << pad << "}";
    }

    void WriteScenarioJson(std::ostream& output, const ScenarioMetrics& metrics, int indent)
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
        output << ",\n" << pad << "    \"initial_yaw_rad\": ";
        WriteJsonNumber(output, metrics.initialYawRad);
        output << ",\n" << pad << "    \"target_forward_mps\": ";
        WriteJsonNumber(output, metrics.targetForwardMps);
        output << ",\n" << pad << "    \"target_yaw_rate_radps\": ";
        WriteJsonNumber(output, metrics.targetYawRateRadps);
        output << ",\n" << pad << "    \"target_forward_accel_mps2\": ";
        WriteJsonNumber(output, metrics.targetForwardAccelMps2);
        output << ",\n" << pad << "    \"target_yaw_accel_radps2\": ";
        WriteJsonNumber(output, metrics.targetYawAccelRadps2);
        output << ",\n" << pad << "    \"target_yaw_rad\": ";
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

    void WriteAcceptanceJson(std::ostream& output, const AcceptanceMetrics& metrics, int indent)
    {
        const std::string pad(static_cast<std::size_t>(indent), ' ');
        output << "{\n"
            << pad << "  \"name\": " << JsonString(metrics.name) << ",\n"
            << pad << "  \"description\": " << JsonString(metrics.description) << ",\n"
            << pad << "  \"path\": " << JsonString(metrics.path) << ",\n"
            << pad << "  \"code\": " << JsonString(metrics.code) << ",\n"
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
            << pad << "  \"definition\": {\n"
            << pad << "    \"target_distance_m\": ";
        WriteJsonNumber(output, metrics.targetDistanceM);
        output << ",\n"
            << pad << "    \"target_yaw_rad\": ";
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
        output << "\n" << pad << "  },\n"
            << pad << "  \"final\": {\n"
            << pad << "    \"x_m\": ";
        WriteJsonNumber(output, metrics.finalXM);
        output << ",\n"
            << pad << "    \"y_m\": ";
        WriteJsonNumber(output, metrics.finalYM);
        output << ",\n"
            << pad << "    \"yaw_rad\": ";
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
            << pad << "  \"score_penalty\": ";
        WriteJsonNumber(output, metrics.scorePenalty);
        output << "\n" << pad << "}";
    }

    void WriteEvaluationJson(std::ostream& output, const char* label, const EvaluationResult& result, int indent)
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

    void WriteTopCandidatesJson(std::ostream& output, const std::vector<EvaluationResult>& topCandidates, int indent)
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

    void WriteOutputJson(
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
            << "    \"drive_primitive_start_straight_completes\": \"Runs Drive::StartStraight(0.30 m, 0.30 m/s, 0.0 m/s exit) for up to 6000 exact 0.001s ticks through SharedRobotRuntime, Drive, DriveBase, PlantModel::integrate, and sensor snapshot publication; completion failure is a blocker.\",\n"
            << "    \"drive_maneuver_final_heading_acceptance\": \"Runs smooth Drive::StartManeuver at 0.50 m/s entry/exit for S180SS, S135SD, S135LD, and S135LS for up to 20000 exact 0.001s ticks; completion or final heading error above 3 degrees is a blocker. Position error is emitted for context but does not define these heading-focused blockers.\"\n"
            << "  },\n"
            << "  \"step_response_metric_definitions\": {\n"
            << "    \"tick_seconds\": ";
        WriteJsonNumber(std::cout, kTickSecondsExact);
        std::cout << ",\n"
            << "    \"velocity_error_first_500_ticks\": \"RMS(targetForwardMps - state.u) over the first 500 0.001s ticks when targetForwardMps steps from its initial value.\",\n"
            << "    \"yaw_rate_error_first_500_ticks\": \"RMS(targetYawRateRadps - state.r) over the first 500 0.001s ticks when targetYawRateRadps steps from its initial value.\",\n"
            << "    \"forward_accel_error_first_100_ticks\": \"RMS((nextU - prevU) / 0.001 - DriveTelemetry.composedForwardAccelMps2) over the first 100 ticks for forward velocity steps when the composed objective is finite and active; fallback samples use RMS((nextU - prevU) / 0.001) as undesired forward acceleration when the objective is inactive or non-finite.\",\n"
            << "    \"yaw_accel_error_first_100_ticks\": \"RMS((nextR - prevR) / 0.001 - DriveTelemetry.composedYawAccelRadps2) over the first 100 ticks for yaw-rate or heading steps when the composed objective is finite and active; fallback samples use RMS((nextR - prevR) / 0.001) as undesired yaw acceleration when the objective is inactive or non-finite.\"\n"
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
        std::cout << "\n"
            << "  },\n"
            << "  \"baseline_source\": \"Config::kDriveBasePDCluster\",\n"
            << "  \"current_drivebase_feedback_path\": {\n"
            << "    \"velocity_state_pd_kd_sampled\": false,\n"
            << "    \"heading_state_pd_kd_uses_yaw_rate_error\": true,\n"
            << "    \"yaw_rate_state_pd_kd_sampled\": false\n"
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

    const GainSet baselineGains = ExtractTunedGains(MazeMap::Config::kDriveBasePDCluster);
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
