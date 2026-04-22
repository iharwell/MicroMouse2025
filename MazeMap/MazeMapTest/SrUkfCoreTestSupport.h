#pragma once

#include "CppUnitTest.h"

#include "EstimatorTestSupport.h"

#include "..\MazeMap\SrUkfCore.h"

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace MazeMap
{
    inline float StationaryGyroBiasBlendFactor(float dtSeconds) noexcept
    {
        if (!(std::isfinite(dtSeconds) && (dtSeconds > 0.0f)) ||
            !(std::isfinite(SrUkfCore::kStationaryGyroBiasTimeConstantS) &&
                (SrUkfCore::kStationaryGyroBiasTimeConstantS > 0.0f)))
        {
            return 0.0f;
        }

        return (std::clamp)(
            1.0f - std::exp(-dtSeconds / SrUkfCore::kStationaryGyroBiasTimeConstantS),
            0.0f,
            1.0f);
    }

    inline float StationaryGyroBiasMeasurementVarianceRadps2() noexcept
    {
        return SrUkfCore::kImuYawRateSigmaRadps * SrUkfCore::kImuYawRateSigmaRadps;
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

        if ((expectation.sampleOrdinal >= SrUkfCore::kInitialStationaryGyroBiasSeedStartSample) &&
            (expectation.sampleOrdinal <= SrUkfCore::kInitialStationaryGyroBiasSeedEndSample))
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
            if ((expectation.sampleOrdinal >= SrUkfCore::kInitialStationaryGyroBiasSeedEndSample) &&
                (expectation.collectedSeedSamples > 0U))
            {
                expectation.biasRadps = static_cast<float>(
                    expectation.seedAccumRadps /
                    static_cast<double>(expectation.collectedSeedSamples));
                expectation.varianceRadps2 =
                    SrUkfCore::ComputeStationaryGyroBiasWalkPosteriorVarianceRadps2(
                        dtSeconds,
                        measurementVarianceRadps2);
                if (!(std::isfinite(expectation.varianceRadps2) && (expectation.varianceRadps2 > 0.0f)))
                {
                    expectation.varianceRadps2 = 1.0e-8f;
                }
                expectation.seedApplied = true;
            }
            return;
        }

        const float priorVarianceRadps2 =
            (std::isfinite(expectation.varianceRadps2) && (expectation.varianceRadps2 > 0.0f)) ?
            expectation.varianceRadps2 :
            1.0e-8f;
        const float predictedVarianceRadps2 =
            priorVarianceRadps2 +
            SrUkfCore::ComputeStationaryGyroBiasWalkProcessVarianceRadps2(
                dtSeconds,
                measurementVarianceRadps2);
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
            expectation.varianceRadps2 = 1.0e-8f;
        }
    }

    inline std::vector<std::pair<std::string, std::string>> CollectDebugDumpLines(const SrUkfCore& core)
    {
        std::vector<std::pair<std::string, std::string>> dumpLines;
        const bool dumpOk = core.WriteDebugTextDump(
            [&dumpLines](const char* type, const char* message) noexcept
            {
                dumpLines.emplace_back(
                    (type != nullptr) ? type : "",
                    (message != nullptr) ? message : "");
                return true;
            });
        if (!dumpOk)
        {
            dumpLines.clear();
        }
        return dumpLines;
    }

    inline std::string FindProcessNoiseRowMessage(
        const std::vector<std::pair<std::string, std::string>>& dumpLines,
        const char* rowName)
    {
        const std::string rowToken = std::string("row=") + rowName;
        for (const auto& line : dumpLines)
        {
            if (line.first == "ukf_dump_process_noise_sqrt_row" &&
                line.second.find(rowToken) != std::string::npos)
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

    inline float FindProcessNoiseDiagonal(const SrUkfCore& core, const char* rowName)
    {
        const auto dumpLines = CollectDebugDumpLines(core);
        return ExtractNamedFloat(FindProcessNoiseRowMessage(dumpLines, rowName), rowName);
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
        const VehicleState::StateVector& previousState,
        const VehicleState::StateVector& predictedState,
        const PlantParams& params,
        float dtSeconds,
        SyntheticEncoderRemainderState& remainderState) noexcept
    {
        EncoderObs encoder{};
        encoder.omegaLeftRadps = predictedState(VehicleState::kOmegaL);
        encoder.omegaRightRadps = predictedState(VehicleState::kOmegaR);

        if (!(std::isfinite(dtSeconds) && (dtSeconds > 0.0f)))
        {
            return encoder;
        }

        const float distancePerCountM = DistancePerEncoderCountMeters(params);
        if (!(std::isfinite(distancePerCountM) && (distancePerCountM > 0.0f)))
        {
            return encoder;
        }

        const float leftDistanceDeltaM =
            0.5f *
            (previousState(VehicleState::kOmegaL) + predictedState(VehicleState::kOmegaL)) *
            params.wheelRadiusM *
            dtSeconds;
        const float rightDistanceDeltaM =
            0.5f *
            (previousState(VehicleState::kOmegaR) + predictedState(VehicleState::kOmegaR)) *
            params.wheelRadiusM *
            dtSeconds;

        encoder.totalLeftCounts =
            ConsumeWholeEncoderCounts(
                leftDistanceDeltaM / distancePerCountM,
                remainderState.leftRemainderCounts);
        encoder.totalRightCounts =
            ConsumeWholeEncoderCounts(
                rightDistanceDeltaM / distancePerCountM,
                remainderState.rightRemainderCounts);
        return encoder;
    }

    inline void ApplyPredictionMatchingEncoderAndYawUpdates(
        SrUkfCore& core,
        const VehicleState::StateVector& previousState,
        const VehicleState::StateVector& predictedState,
        const PlantParams& params,
        float dtSeconds,
        SyntheticEncoderRemainderState& remainderState)
    {
        const EncoderObs encoder =
            BuildPredictionMatchingEncoderObservation(
                previousState,
                predictedState,
                params,
                dtSeconds,
                remainderState);
        const MeasurementUpdateResult encoderResult = core.updateEncoderPair(encoder, dtSeconds);
        Microsoft::VisualStudio::CppUnitTestFramework::Assert::IsTrue(encoderResult.attempted);
        Microsoft::VisualStudio::CppUnitTestFramework::Assert::IsTrue(encoderResult.accepted);

        const MeasurementUpdateResult yawResult =
            core.updateYawRate(predictedState(VehicleState::kR));
        Microsoft::VisualStudio::CppUnitTestFramework::Assert::IsTrue(yawResult.attempted);
        Microsoft::VisualStudio::CppUnitTestFramework::Assert::IsTrue(yawResult.accepted);
    }

    inline void RunPredictionMatchingCycle(
        SrUkfCore& core,
        const ControlInput& control,
        const PlantParams& params,
        float dtSeconds,
        SyntheticEncoderRemainderState& remainderState,
        float commandedLinearMps,
        float commandedAngularRadps,
        std::uint16_t saturationFlags = 0U,
        float leftLaunchAssistFloor = 0.0f,
        float rightLaunchAssistFloor = 0.0f)
    {
        core.setRuntimeContext(
            commandedLinearMps,
            commandedAngularRadps,
            saturationFlags,
            leftLaunchAssistFloor,
            rightLaunchAssistFloor,
            true,
            0.0f,
            0.0f);
        const VehicleState::StateVector stateBeforePredict = core.state();
        Microsoft::VisualStudio::CppUnitTestFramework::Assert::IsTrue(core.predict(dtSeconds, control));
        ApplyPredictionMatchingEncoderAndYawUpdates(
            core,
            stateBeforePredict,
            core.state(),
            params,
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

    inline SrUkfCore RunUKFCycles(int numCycles, ControlInput control = ControlInput{})
    {
        const VehicleState::StateVector initialState =
            BuildUkfState(
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                0.0f);
        const VehicleState::StateMatrix initialCovariance =
            BuildUkfCovariance(0.001f, 0.01f, 0.005f, 0.005f, 0.05f, 0.05f, 0.02f);

        SrUkfCore core;
        Microsoft::VisualStudio::CppUnitTestFramework::Assert::IsTrue(core.reset(initialState, initialCovariance));
        EncoderObs encoder{};
        constexpr float dt = 0.001f;

        for (int step = 0; step < numCycles; ++step)
        {
            core.setRuntimeContext(0.0f, 0.0f, 0U, 0.0f, 0.0f, true, 0.0f, 0.0f);
            Microsoft::VisualStudio::CppUnitTestFramework::Assert::IsTrue(core.predict(dt, control));

            const MeasurementUpdateResult encoderResult = core.updateEncoderPair(encoder, dt);
            Microsoft::VisualStudio::CppUnitTestFramework::Assert::IsTrue(encoderResult.attempted);
            Microsoft::VisualStudio::CppUnitTestFramework::Assert::IsTrue(encoderResult.accepted);

            const MeasurementUpdateResult yawResult = core.updateYawRate(0.0f);
            Microsoft::VisualStudio::CppUnitTestFramework::Assert::IsTrue(yawResult.attempted);
            Microsoft::VisualStudio::CppUnitTestFramework::Assert::IsTrue(yawResult.accepted);

            ImuAccelObs accel{};
            accel.valid = true;
            accel.accelBodyXMps2 = 0.0f;
            accel.accelBodyYMps2 = 0.0f;
            const MeasurementUpdateResult accelResult = core.updatePlanarAccel(accel);
            Microsoft::VisualStudio::CppUnitTestFramework::Assert::IsTrue(accelResult.attempted);
            Microsoft::VisualStudio::CppUnitTestFramework::Assert::IsTrue(accelResult.accepted);
        }

        return core;
    }
}
