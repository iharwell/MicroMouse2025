#pragma once

#include "CppUnitTest.h"

#include "..\MazeMap\PlantModel.h"
#include "..\MazeMap\Estimator.h"
#include "..\MazeMap\Vehicle.h"
#include "..\MazeMap\VehicleState.h"

#include <algorithm>
#include <cstdarg>
#include <cstdint>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace MazeMap
{
    constexpr float kEstimatorTestImuYawRateVarianceRadps2 = 1.2e-6f;
    constexpr float kEstimatorTestImuYawRateSigmaRadps = 0.0010954451f;
    constexpr float kEstimatorTestImuGyroSensitivityToleranceFraction = 0.003f;
    constexpr float kEstimatorTestImuAccelSigmaMps2 = 0.569900f;
    constexpr float kEstimatorTestGeneralEncoderLinearSpeedSigmaMps = 0.021187f;
    constexpr float kEstimatorTestGeneralEncoderYawRateSigmaRadps = 0.111268f;
    constexpr float kEstimatorTestStationaryEncoderVelocitySigmaMps = 0.002936f;
    constexpr float kEstimatorTestEncoderPairNisThreshold = 13.81551f;
    constexpr float kEstimatorTestPivotScrubMinCommandAngularRadps = 1.0f;
    constexpr float kEstimatorTestStationaryCertificationDwellS = 0.150f;
    constexpr float kEstimatorTestGyroBiasProcessVarianceStationaryRadps2PerSample = 3.0e-16f;
    constexpr float kEstimatorTestGyroBiasProcessVarianceMovingRadps2PerSample = 0.0f;
    constexpr float kEstimatorTestGyroBiasInitialVarianceUnseededRadps2 = 3.05e-4f;
    constexpr std::uint16_t kEstimatorTestInitialStationaryGyroBiasSeedStartSample = 3U;
    constexpr std::uint16_t kEstimatorTestInitialStationaryGyroBiasSeedEndSample = 8U;

    struct EstimatorTestRuntime final
    {
        Vehicle vehicle{};
        VehicleState runtimeState{};
        PlantModel plantModel;

        EstimatorTestRuntime() noexcept
            : plantModel(vehicle, runtimeState)
        {
            vehicle.SetFanDuty(0.80f);
        }
    };

    inline Estimator MakeDefaultEstimator() noexcept
    {
        EstimatorTestRuntime* const runtime = new EstimatorTestRuntime();
        return Estimator(runtime->vehicle, runtime->plantModel, runtime->runtimeState);
    }

    inline float StationaryGyroBiasMeasurementVarianceRadps2() noexcept
    {
        return kEstimatorTestImuYawRateVarianceRadps2;
    }

    struct InitialStationaryGyroBiasExpectation final
    {
        bool phaseExited = false;
        bool seedApplied = false;
        std::uint16_t sampleOrdinal = 0U;
        std::uint16_t collectedSeedSamples = 0U;
        double seedAccumRadps = 0.0;
        float biasRadps = 0.0f;
        float varianceRadps2 = 0.0f;
    };

    inline void AdvanceInitialStationaryGyroBiasExpectation(
        InitialStationaryGyroBiasExpectation& expectation,
        const float yawRateRadps,
        const float dtSeconds,
        const bool startupStationaryTick = true) noexcept
    {
        (void)dtSeconds;
        if (expectation.phaseExited)
        {
            return;
        }

        if (!startupStationaryTick)
        {
            expectation.phaseExited = true;
            return;
        }

        if (!std::isfinite(yawRateRadps))
        {
            return;
        }

        if (expectation.sampleOrdinal < (std::numeric_limits<std::uint16_t>::max)())
        {
            ++expectation.sampleOrdinal;
        }

        if ((expectation.sampleOrdinal >= kEstimatorTestInitialStationaryGyroBiasSeedStartSample) &&
            (expectation.sampleOrdinal <= kEstimatorTestInitialStationaryGyroBiasSeedEndSample))
        {
            expectation.seedAccumRadps += static_cast<double>(yawRateRadps);
            if (expectation.collectedSeedSamples < (std::numeric_limits<std::uint16_t>::max)())
            {
                ++expectation.collectedSeedSamples;
            }
        }

        const float measurementVarianceRadps2 = StationaryGyroBiasMeasurementVarianceRadps2();
        if (!expectation.seedApplied)
        {
            if ((expectation.sampleOrdinal >= kEstimatorTestInitialStationaryGyroBiasSeedEndSample) &&
                (expectation.collectedSeedSamples > 0U))
            {
                expectation.biasRadps = static_cast<float>(
                    expectation.seedAccumRadps /
                    static_cast<double>(expectation.collectedSeedSamples));
                expectation.varianceRadps2 = kEstimatorTestGyroBiasInitialVarianceUnseededRadps2;
                if (!(std::isfinite(expectation.varianceRadps2) && (expectation.varianceRadps2 > 0.0f)))
                {
                    expectation.varianceRadps2 = kEstimatorTestGyroBiasInitialVarianceUnseededRadps2;
                }
                expectation.seedApplied = true;
            }
            return;
        }

        const float priorVarianceRadps2 =
            (std::isfinite(expectation.varianceRadps2) && (expectation.varianceRadps2 > 0.0f)) ?
            expectation.varianceRadps2 :
            kEstimatorTestGyroBiasInitialVarianceUnseededRadps2;
        const float predictedVarianceRadps2 =
            priorVarianceRadps2 +
            kEstimatorTestGyroBiasProcessVarianceStationaryRadps2PerSample;
        const float innovationVarianceRadps2 = predictedVarianceRadps2 + measurementVarianceRadps2;
        if (!(std::isfinite(predictedVarianceRadps2) && std::isfinite(innovationVarianceRadps2)) ||
            !(innovationVarianceRadps2 > 0.0f))
        {
            return;
        }

        const float kalmanGain =
            (std::clamp)(predictedVarianceRadps2 / innovationVarianceRadps2, 0.0f, 1.0f);
        expectation.biasRadps += kalmanGain * (yawRateRadps - expectation.biasRadps);
        expectation.varianceRadps2 = (1.0f - kalmanGain) * predictedVarianceRadps2;
        if (!(std::isfinite(expectation.varianceRadps2) && (expectation.varianceRadps2 > 0.0f)))
        {
            expectation.varianceRadps2 = kEstimatorTestGyroBiasInitialVarianceUnseededRadps2;
        }
    }

    inline std::vector<std::pair<std::string, std::string>> CollectDebugDumpLines(const Estimator& core)
    {
        std::vector<std::pair<std::string, std::string>> dumpLines;
        const bool dumpOk = core.WriteDebugTextDump(
            [&dumpLines](const char* type, const char* format, std::va_list args) noexcept
            {
                char message[1024] = {};
                std::va_list argsCopy;
                va_copy(argsCopy, args);
                const int length = std::vsnprintf(message, sizeof(message), format, argsCopy);
                va_end(argsCopy);
                if (length <= 0 || length >= static_cast<int>(sizeof(message)))
                {
                    return false;
                }
                dumpLines.emplace_back(
                    (type != nullptr) ? type : "",
                    message);
                return true;
            });
        (void)dumpOk;
        return dumpLines;
    }

    inline std::string FindProcessNoiseRowMessage(
        const std::vector<std::pair<std::string, std::string>>& dumpLines,
        const char* rowName)
    {
        const std::string rowToken = std::string("row=") + rowName;
        for (const auto& line : dumpLines)
        {
            if (line.first == "estimator_dump_process_noise_sqrt_row" &&
                line.second.find(rowToken) != std::string::npos)
            {
                return line.second;
            }
        }
        return std::string();
    }

    inline std::string FindDebugDumpMessage(
        const std::vector<std::pair<std::string, std::string>>& dumpLines,
        const char* type)
    {
        for (const auto& line : dumpLines)
        {
            if (line.first == type)
            {
                return line.second;
            }
        }
        return std::string();
    }

    inline float ExtractNamedFloat(const std::string& message, const char* fieldName)
    {
        const std::string token = std::string(fieldName) + "=";
        const std::size_t start = message.find(token);
        if (start == std::string::npos)
        {
            return std::numeric_limits<float>::quiet_NaN();
        }

        const char* valueStart = message.c_str() + start + token.size();
        char* valueEnd = nullptr;
        return std::strtof(valueStart, &valueEnd);
    }

    inline float FindProcessNoiseDiagonal(const Estimator& core, const char* rowName)
    {
        const auto dumpLines = CollectDebugDumpLines(core);
        return ExtractNamedFloat(FindProcessNoiseRowMessage(dumpLines, rowName), rowName);
    }

    inline float FindDebugDumpFloat(const Estimator& core, const char* type, const char* fieldName)
    {
        const auto dumpLines = CollectDebugDumpLines(core);
        const std::string token = std::string(fieldName) + "=";
        for (const auto& line : dumpLines)
        {
            if ((line.first == type) && (line.second.find(token) != std::string::npos))
            {
                return ExtractNamedFloat(line.second, fieldName);
            }
        }

        return std::numeric_limits<float>::quiet_NaN();
    }

    inline bool ExtractNamedBool(const std::string& message, const char* fieldName, bool fallback = false)
    {
        const std::string token = std::string(fieldName) + "=";
        const std::size_t start = message.find(token);
        if (start == std::string::npos)
        {
            return fallback;
        }

        const char* valueStart = message.c_str() + start + token.size();
        if (std::strncmp(valueStart, "true", 4) == 0)
        {
            return true;
        }
        if (std::strncmp(valueStart, "false", 5) == 0)
        {
            return false;
        }
        return fallback;
    }

    inline bool FindDebugDumpBool(
        const Estimator& core,
        const char* type,
        const char* fieldName,
        bool fallback = false)
    {
        return ExtractNamedBool(FindDebugDumpMessage(CollectDebugDumpLines(core), type), fieldName, fallback);
    }

    inline int FindDebugDumpModeId(const Estimator& core)
    {
        return static_cast<int>(FindDebugDumpFloat(core, "estimator_dump_mode", "mode_id"));
    }

    inline float EstimatorTestNonholonomicSigmaMps(float absForwardSpeedMps) noexcept
    {
        const float resolvedForwardSpeedMps =
            (std::isfinite(absForwardSpeedMps) && (absForwardSpeedMps > 0.0f)) ?
            absForwardSpeedMps :
            0.0f;
        const float sigmaMps = std::sqrt(
            (0.005f * 0.005f) +
            ((0.035f * resolvedForwardSpeedMps) * (0.035f * resolvedForwardSpeedMps)));
        return (std::clamp)(sigmaMps, 0.005f, 0.060f);
    }

    struct SyntheticEncoderRemainderState final
    {
        float leftRemainderCounts = 0.0f;
        float rightRemainderCounts = 0.0f;
    };

    inline int32_t ConsumeWholeEncoderCounts(float deltaCounts, float& remainderCounts) noexcept
    {
        remainderCounts += deltaCounts;
        const int32_t wholeCounts =
            (remainderCounts >= 0.0f) ?
            static_cast<int32_t>(std::floor(remainderCounts)) :
            static_cast<int32_t>(std::ceil(remainderCounts));
        remainderCounts -= static_cast<float>(wholeCounts);
        return wholeCounts;
    }

    inline EncoderObs BuildPredictionMatchingEncoderObservation(
        const Eigen::Matrix<float, VehicleState::kDimension, 1>& previousState,
        const Eigen::Matrix<float, VehicleState::kDimension, 1>& predictedState,
        float dtSeconds,
        SyntheticEncoderRemainderState& remainderState) noexcept
    {
        EncoderObs encoder{};
        encoder.SetLeftWheelSpeedRadps(Vehicle::WheelSpeedFromLinearVelocity(
                Vehicle::LeftWheelLinearVelocityFromBody(predictedState(3), predictedState(5))));
        encoder.SetRightWheelSpeedRadps(Vehicle::WheelSpeedFromLinearVelocity(
                Vehicle::RightWheelLinearVelocityFromBody(predictedState(3), predictedState(5))));

        if (!(std::isfinite(dtSeconds) && (dtSeconds > 0.0f)))
        {
            return encoder;
        }

        const float distancePerCountM = Vehicle::DriveEncoderDistanceFromCounts(1);
        if (!(std::isfinite(distancePerCountM) && (distancePerCountM > 0.0f)))
        {
            return encoder;
        }

        const float wheelRadiusM = Vehicle::GetDriveWheelRadiusM();
        const float leftDistanceDeltaM =
            0.5f *
            (Vehicle::WheelSpeedFromLinearVelocity(
                Vehicle::LeftWheelLinearVelocityFromBody(previousState(3), previousState(5))) +
             encoder.LeftWheelSpeedRadps()) *
            wheelRadiusM *
            dtSeconds;
        const float rightDistanceDeltaM =
            0.5f *
            (Vehicle::WheelSpeedFromLinearVelocity(
                Vehicle::RightWheelLinearVelocityFromBody(previousState(3), previousState(5))) +
             encoder.RightWheelSpeedRadps()) *
            wheelRadiusM *
            dtSeconds;

        encoder.SetTotalLeftCounts(ConsumeWholeEncoderCounts(
                leftDistanceDeltaM / distancePerCountM,
                remainderState.leftRemainderCounts));
        encoder.SetTotalRightCounts(ConsumeWholeEncoderCounts(
                rightDistanceDeltaM / distancePerCountM,
                remainderState.rightRemainderCounts));
        return encoder;
    }

    inline void ApplyPredictionMatchingEncoderAndYawUpdates(
        Estimator& core,
        const Eigen::Matrix<float, VehicleState::kDimension, 1>& previousState,
        const Eigen::Matrix<float, VehicleState::kDimension, 1>& predictedState,
        float dtSeconds,
        SyntheticEncoderRemainderState& remainderState)
    {
        const EncoderObs encoder =
            BuildPredictionMatchingEncoderObservation(
                previousState,
                predictedState,
                dtSeconds,
                remainderState);
        const bool encoderAccepted = core.updateEncoderPair(encoder, dtSeconds, true);
        Microsoft::VisualStudio::CppUnitTestFramework::Assert::IsTrue(
            FindDebugDumpBool(core, "estimator_dump_update_metrics", "last_update_attempted"));
        Microsoft::VisualStudio::CppUnitTestFramework::Assert::IsTrue(encoderAccepted);
        Microsoft::VisualStudio::CppUnitTestFramework::Assert::IsTrue(
            FindDebugDumpBool(core, "estimator_dump_update_metrics", "last_update_accepted"));

        const bool yawAccepted =
            core.updateYawRate(predictedState(5));
        Microsoft::VisualStudio::CppUnitTestFramework::Assert::IsTrue(
            FindDebugDumpBool(core, "estimator_dump_update_metrics", "last_update_attempted"));
        Microsoft::VisualStudio::CppUnitTestFramework::Assert::IsTrue(yawAccepted);
        Microsoft::VisualStudio::CppUnitTestFramework::Assert::IsTrue(
            FindDebugDumpBool(core, "estimator_dump_update_metrics", "last_update_accepted"));
    }

    inline void RunPredictionMatchingCycle(
        Estimator& core,
        const App::Internal::CommandVector& control,
        float dtSeconds,
        SyntheticEncoderRemainderState& remainderState,
        float commandedLinearMps,
        float commandedAngularRadps,
        std::uint16_t saturationFlags = 0U)
    {
        (void)commandedLinearMps;
        (void)commandedAngularRadps;
        (void)saturationFlags;
        Eigen::Matrix<float, VehicleState::kDimension, 1> stateBeforePredict;
        stateBeforePredict <<
            FindDebugDumpFloat(core, "estimator_dump_state", "px_m"),
            FindDebugDumpFloat(core, "estimator_dump_state", "py_m"),
            FindDebugDumpFloat(core, "estimator_dump_state", "heading_rad"),
            FindDebugDumpFloat(core, "estimator_dump_state", "vf_mps"),
            FindDebugDumpFloat(core, "estimator_dump_state", "vr_mps"),
            FindDebugDumpFloat(core, "estimator_dump_state", "yaw_rate_radps"),
            FindDebugDumpFloat(core, "estimator_dump_state", "delta_af_mps2"),
            FindDebugDumpFloat(core, "estimator_dump_state", "delta_ar_mps2"),
            FindDebugDumpFloat(core, "estimator_dump_state", "delta_yaw_accel_radps2");
        Microsoft::VisualStudio::CppUnitTestFramework::Assert::IsTrue(
            core.predict(dtSeconds, control));
        Eigen::Matrix<float, VehicleState::kDimension, 1> stateAfterPredict;
        stateAfterPredict <<
            FindDebugDumpFloat(core, "estimator_dump_state", "px_m"),
            FindDebugDumpFloat(core, "estimator_dump_state", "py_m"),
            FindDebugDumpFloat(core, "estimator_dump_state", "heading_rad"),
            FindDebugDumpFloat(core, "estimator_dump_state", "vf_mps"),
            FindDebugDumpFloat(core, "estimator_dump_state", "vr_mps"),
            FindDebugDumpFloat(core, "estimator_dump_state", "yaw_rate_radps"),
            FindDebugDumpFloat(core, "estimator_dump_state", "delta_af_mps2"),
            FindDebugDumpFloat(core, "estimator_dump_state", "delta_ar_mps2"),
            FindDebugDumpFloat(core, "estimator_dump_state", "delta_yaw_accel_radps2");
        ApplyPredictionMatchingEncoderAndYawUpdates(
            core,
            stateBeforePredict,
            stateAfterPredict,
            dtSeconds,
            remainderState);
    }

    inline std::size_t FindFirstDumpLineIndexContaining(
        const std::vector<std::pair<std::string, std::string>>& dumpLines,
        const char* token)
    {
        if (token == nullptr)
        {
            return dumpLines.size();
        }

        for (std::size_t index = 0; index < dumpLines.size(); ++index)
        {
            if ((dumpLines[index].first.find(token) != std::string::npos) ||
                (dumpLines[index].second.find(token) != std::string::npos))
            {
                return index;
            }
        }

        return dumpLines.size();
    }

    inline Estimator RunEstimatorCycles(
        int numCycles,
        App::Internal::CommandVector control =
            App::Internal::CommandVector(0.0f, 0.0f))
    {
        Eigen::Matrix<float, VehicleState::kDimension, 1> initialState = Eigen::Matrix<float, VehicleState::kDimension, 1>::Zero();
        initialState(0) = 0.0f;
        initialState(1) = 0.0f;
        initialState(2) = (0.0f);
        initialState(3) = 0.0f;
        initialState(4) = 0.0f;
        initialState(5) = 0.0f;
        initialState(6) = 0.0f;
        initialState(7) = 0.0f;
        initialState(8) = 0.0f;
        Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> initialCovariance = Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension>::Zero();
        initialCovariance(0, 0) = 0.001f * 0.001f;
        initialCovariance(1, 1) = 0.001f * 0.001f;
        initialCovariance(2, 2) = 0.01f * 0.01f;
        initialCovariance(3, 3) = 0.005f * 0.005f;
        initialCovariance(4, 4) = 0.005f * 0.005f;
        initialCovariance(5, 5) = 0.05f * 0.05f;
        initialCovariance(6, 6) = 0.05f * 0.05f;
        initialCovariance(7, 7) = 0.05f * 0.05f;
        initialCovariance(8, 8) = 0.02f * 0.02f;

        Estimator core = MakeDefaultEstimator();
        Microsoft::VisualStudio::CppUnitTestFramework::Assert::IsTrue(core.reset(initialState, initialCovariance));
        EncoderObs encoder{};
        constexpr float dt = 0.001f;

        for (int step = 0; step < numCycles; ++step)
        {
            Microsoft::VisualStudio::CppUnitTestFramework::Assert::IsTrue(
                core.predict(dt, control));

            const bool encoderAccepted = core.updateEncoderPair(encoder, dt, true);
            Microsoft::VisualStudio::CppUnitTestFramework::Assert::IsTrue(
                FindDebugDumpBool(core, "estimator_dump_update_metrics", "last_update_attempted"));
            Microsoft::VisualStudio::CppUnitTestFramework::Assert::IsTrue(encoderAccepted);
            Microsoft::VisualStudio::CppUnitTestFramework::Assert::IsTrue(
                FindDebugDumpBool(core, "estimator_dump_update_metrics", "last_update_accepted"));

            const bool yawAccepted = core.updateYawRate(0.0f);
            Microsoft::VisualStudio::CppUnitTestFramework::Assert::IsTrue(
                FindDebugDumpBool(core, "estimator_dump_update_metrics", "last_update_attempted"));
            Microsoft::VisualStudio::CppUnitTestFramework::Assert::IsTrue(yawAccepted);
            Microsoft::VisualStudio::CppUnitTestFramework::Assert::IsTrue(
                FindDebugDumpBool(core, "estimator_dump_update_metrics", "last_update_accepted"));

            const ImuAccelObs accel(true, 0.0f, 0.0f);
            const bool accelAccepted = core.updatePlanarAccel(accel);
            Microsoft::VisualStudio::CppUnitTestFramework::Assert::IsTrue(
                FindDebugDumpBool(core, "estimator_dump_update_metrics", "last_update_attempted"));
            Microsoft::VisualStudio::CppUnitTestFramework::Assert::IsTrue(accelAccepted);
            Microsoft::VisualStudio::CppUnitTestFramework::Assert::IsTrue(
                FindDebugDumpBool(core, "estimator_dump_update_metrics", "last_update_accepted"));
        }

        return core;
    }
}


