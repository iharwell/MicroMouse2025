#pragma once

#include <array>
#include <cstdint>

#include "WallSensorCalibration.h"

#include <cmath>
#include <limits>

namespace MazeMap
{
    template <size_t MaxSamples>
    inline bool TryComputeRobustSignalBandFromSamples(
        const std::array<float, MaxSamples>& samples,
        uint16_t count,
        float scaledMadMultiplier,
        float& median,
        float& low,
        float& high)
    {
        median = 0.0f;
        low = 0.0f;
        high = 0.0f;
        if (count == 0U ||
            count > static_cast<uint16_t>(MaxSamples) ||
            !std::isfinite(scaledMadMultiplier) ||
            scaledMadMultiplier < 0.0f)
        {
            return false;
        }

        auto insertionSort = [](float* values, uint16_t valueCount) noexcept
        {
            for (uint16_t index = 1U; index < valueCount; ++index)
            {
                const float value = values[index];
                uint16_t insertIndex = index;
                while ((insertIndex > 0U) && (values[insertIndex - 1U] > value))
                {
                    values[insertIndex] = values[insertIndex - 1U];
                    --insertIndex;
                }

                values[insertIndex] = value;
            }
        };

        auto computeMedian = [](const float* values, uint16_t valueCount) noexcept
        {
            if ((valueCount & 1U) != 0U)
            {
                return values[valueCount / 2U];
            }

            const uint16_t upperIndex = valueCount / 2U;
            return 0.5f * (values[upperIndex - 1U] + values[upperIndex]);
        };

        float sortedSamples[MaxSamples] = {};
        uint16_t validCount = 0U;
        for (uint16_t index = 0U; index < count; ++index)
        {
            const float sample = samples[index];
            if (!std::isfinite(sample) || sample < 0.0f)
            {
                continue;
            }

            sortedSamples[validCount] = sample;
            ++validCount;
        }

        if (validCount == 0U)
        {
            return false;
        }

        insertionSort(sortedSamples, validCount);
        median = computeMedian(sortedSamples, validCount);

        float absoluteDeviations[MaxSamples] = {};
        for (uint16_t index = 0U; index < validCount; ++index)
        {
            absoluteDeviations[index] = std::fabs(sortedSamples[index] - median);
        }

        insertionSort(absoluteDeviations, validCount);
        const float mad = computeMedian(absoluteDeviations, validCount);
        constexpr float kScaledMadNormalization = 0.67448975f;
        const float scaledMad =
            (mad > 0.0f) ? (mad / kScaledMadNormalization) : 0.0f;
        const float halfWidth = scaledMadMultiplier * scaledMad;

        low = median - halfWidth;
        if (low < 0.0f)
        {
            low = 0.0f;
        }
        high = median + halfWidth;
        return
            std::isfinite(median) &&
            std::isfinite(low) &&
            std::isfinite(high) &&
            low <= median &&
            median <= high;
    }

    template <size_t MaxSamples>
    inline bool TryComputeRobustDistanceMatchedSignalBandFromSamples(
        const std::array<float, MaxSamples>& signalSamples,
        const std::array<float, MaxSamples>& distanceSamples,
        uint16_t count,
        float targetDistanceM,
        uint16_t selectedSampleCount,
        uint16_t minimumSelectedCount,
        float maximumDistanceErrorM,
        float scaledMadMultiplier,
        float& median,
        float& low,
        float& high)
    {
        median = 0.0f;
        low = 0.0f;
        high = 0.0f;
        if (count == 0U ||
            count > static_cast<uint16_t>(MaxSamples) ||
            !std::isfinite(targetDistanceM) ||
            targetDistanceM <= 0.0f ||
            selectedSampleCount == 0U ||
            minimumSelectedCount == 0U ||
            minimumSelectedCount > selectedSampleCount ||
            !std::isfinite(maximumDistanceErrorM) ||
            maximumDistanceErrorM < 0.0f ||
            !std::isfinite(scaledMadMultiplier) ||
            scaledMadMultiplier < 0.0f)
        {
            return false;
        }

        float selectedSignals[MaxSamples] = {};
        float selectedErrors[MaxSamples] = {};
        uint16_t validCount = 0U;
        for (uint16_t index = 0U; index < count; ++index)
        {
            const float signal = signalSamples[index];
            const float distance = distanceSamples[index];
            if (!std::isfinite(signal) ||
                signal < 0.0f ||
                !std::isfinite(distance) ||
                distance <= 0.0f)
            {
                continue;
            }

            selectedSignals[validCount] = signal;
            selectedErrors[validCount] = std::fabs(distance - targetDistanceM);
            ++validCount;
        }

        if (validCount < minimumSelectedCount)
        {
            return false;
        }

        for (uint16_t index = 1U; index < validCount; ++index)
        {
            const float signal = selectedSignals[index];
            const float error = selectedErrors[index];
            uint16_t insertIndex = index;
            while ((insertIndex > 0U) && (selectedErrors[insertIndex - 1U] > error))
            {
                selectedSignals[insertIndex] = selectedSignals[insertIndex - 1U];
                selectedErrors[insertIndex] = selectedErrors[insertIndex - 1U];
                --insertIndex;
            }

            selectedSignals[insertIndex] = signal;
            selectedErrors[insertIndex] = error;
        }

        uint16_t retainedCount = (selectedSampleCount < validCount) ? selectedSampleCount : validCount;
        if (retainedCount < minimumSelectedCount ||
            selectedErrors[retainedCount - 1U] > maximumDistanceErrorM)
        {
            return false;
        }

        std::array<float, MaxSamples> retainedSignals{};
        for (uint16_t index = 0U; index < retainedCount; ++index)
        {
            retainedSignals[index] = selectedSignals[index];
        }

        return TryComputeRobustSignalBandFromSamples(
            retainedSignals,
            retainedCount,
            scaledMadMultiplier,
            median,
            low,
            high);
    }

    inline bool TryComputeSignalHighThresholds(
        float calibrationMeasuredValue,
        float latchSignalFraction,
        float releaseSignalFraction,
        float& onMeasuredThreshold,
        float& offMeasuredThreshold)
    {
        onMeasuredThreshold = 0.0f;
        offMeasuredThreshold = 0.0f;
        if (!std::isfinite(calibrationMeasuredValue) ||
            !std::isfinite(latchSignalFraction) ||
            !std::isfinite(releaseSignalFraction) ||
            calibrationMeasuredValue <= 0.0f ||
            latchSignalFraction <= 0.0f ||
            releaseSignalFraction <= 0.0f ||
            releaseSignalFraction >= latchSignalFraction)
        {
            return false;
        }

        onMeasuredThreshold = calibrationMeasuredValue * latchSignalFraction;
        offMeasuredThreshold = calibrationMeasuredValue * releaseSignalFraction;
        return
            std::isfinite(onMeasuredThreshold) &&
            std::isfinite(offMeasuredThreshold) &&
            onMeasuredThreshold > 0.0f &&
            offMeasuredThreshold > 0.0f &&
            offMeasuredThreshold < onMeasuredThreshold;
    }

    inline bool TryComputeSignalRiseThresholds(
        float baselineMeasuredValue,
        float highMeasuredValue,
        float latchSpanFraction,
        float releaseSpanFraction,
        float& onRiseThreshold,
        float& offRiseThreshold)
    {
        onRiseThreshold = 0.0f;
        offRiseThreshold = 0.0f;
        if (!std::isfinite(baselineMeasuredValue) ||
            !std::isfinite(highMeasuredValue) ||
            !std::isfinite(latchSpanFraction) ||
            !std::isfinite(releaseSpanFraction) ||
            baselineMeasuredValue < 0.0f ||
            highMeasuredValue <= baselineMeasuredValue ||
            latchSpanFraction <= 0.0f ||
            releaseSpanFraction <= 0.0f ||
            releaseSpanFraction >= latchSpanFraction)
        {
            return false;
        }

        const float span = highMeasuredValue - baselineMeasuredValue;
        onRiseThreshold = span * latchSpanFraction;
        offRiseThreshold = span * releaseSpanFraction;
        return
            std::isfinite(onRiseThreshold) &&
            std::isfinite(offRiseThreshold) &&
            onRiseThreshold > 0.0f &&
            offRiseThreshold > 0.0f &&
            offRiseThreshold < onRiseThreshold;
    }

    inline bool TryComputeConservativeSignalRiseThresholdsFromBands(
        float baselineMeasuredValueLow,
        float baselineMeasuredValueHigh,
        float highMeasuredValueLow,
        float highMeasuredValueHigh,
        float latchSpanFraction,
        float releaseSpanFraction,
        float& onRiseThreshold,
        float& offRiseThreshold,
        float& signalBaseline)
    {
        onRiseThreshold = 0.0f;
        offRiseThreshold = 0.0f;
        signalBaseline = 0.0f;
        if (!std::isfinite(baselineMeasuredValueLow) ||
            !std::isfinite(baselineMeasuredValueHigh) ||
            !std::isfinite(highMeasuredValueLow) ||
            !std::isfinite(highMeasuredValueHigh) ||
            baselineMeasuredValueLow < 0.0f ||
            baselineMeasuredValueHigh < baselineMeasuredValueLow ||
            highMeasuredValueLow <= 0.0f ||
            highMeasuredValueHigh < highMeasuredValueLow)
        {
            return false;
        }

        if (!TryComputeSignalRiseThresholds(
                baselineMeasuredValueHigh,
                highMeasuredValueLow,
                latchSpanFraction,
                releaseSpanFraction,
                onRiseThreshold,
                offRiseThreshold))
        {
            return false;
        }

        signalBaseline = baselineMeasuredValueHigh;
        return
            std::isfinite(signalBaseline) &&
            signalBaseline >= baselineMeasuredValueLow &&
            signalBaseline <= baselineMeasuredValueHigh;
    }

    inline bool TryComputeSignalBandThresholds(
        float baselineMeasuredValue,
        float highMeasuredValue,
        float latchSpanFraction,
        float releaseSpanFraction,
        float& onMeasuredThreshold,
        float& offMeasuredThreshold)
    {
        onMeasuredThreshold = 0.0f;
        offMeasuredThreshold = 0.0f;

        float onRiseThreshold = 0.0f;
        float offRiseThreshold = 0.0f;
        if (!TryComputeSignalRiseThresholds(
                baselineMeasuredValue,
                highMeasuredValue,
                latchSpanFraction,
                releaseSpanFraction,
                onRiseThreshold,
                offRiseThreshold))
        {
            return false;
        }

        onMeasuredThreshold = baselineMeasuredValue + onRiseThreshold;
        offMeasuredThreshold = baselineMeasuredValue + offRiseThreshold;
        return
            std::isfinite(onMeasuredThreshold) &&
            std::isfinite(offMeasuredThreshold) &&
            onMeasuredThreshold > baselineMeasuredValue &&
            offMeasuredThreshold > baselineMeasuredValue &&
            offMeasuredThreshold < onMeasuredThreshold;
    }

    inline bool TryScaleSignalHighThresholds(
        float scale,
        float& onMeasuredThreshold,
        float& offMeasuredThreshold)
    {
        if (!std::isfinite(scale) ||
            !std::isfinite(onMeasuredThreshold) ||
            !std::isfinite(offMeasuredThreshold) ||
            scale <= 0.0f ||
            onMeasuredThreshold <= 0.0f ||
            offMeasuredThreshold <= 0.0f ||
            offMeasuredThreshold >= onMeasuredThreshold)
        {
            return false;
        }

        onMeasuredThreshold *= scale;
        offMeasuredThreshold *= scale;
        return
            std::isfinite(onMeasuredThreshold) &&
            std::isfinite(offMeasuredThreshold) &&
            onMeasuredThreshold > 0.0f &&
            offMeasuredThreshold > 0.0f &&
            offMeasuredThreshold < onMeasuredThreshold;
    }

    inline bool HysteresisSignalHigh(
        bool currentState,
        float measuredValue,
        float onMeasuredThreshold,
        float offMeasuredThreshold)
    {
        if (!std::isfinite(measuredValue) ||
            !std::isfinite(onMeasuredThreshold) ||
            !std::isfinite(offMeasuredThreshold) ||
            onMeasuredThreshold <= 0.0f ||
            offMeasuredThreshold <= 0.0f ||
            offMeasuredThreshold >= onMeasuredThreshold)
        {
            return false;
        }

        return currentState ?
            (measuredValue > offMeasuredThreshold) :
            (measuredValue > onMeasuredThreshold);
    }

    inline bool TryComputeInverseSquareSignalAtDistanceFromReference(
        float referenceSignal,
        float referenceDistanceM,
        float targetDistanceM,
        float& signal)
    {
        signal = 0.0f;
        if (!std::isfinite(referenceSignal) ||
            !std::isfinite(referenceDistanceM) ||
            !std::isfinite(targetDistanceM) ||
            referenceSignal <= 0.0f ||
            referenceDistanceM <= 0.0f ||
            targetDistanceM <= 0.0f)
        {
            return false;
        }

        const float distanceRatio = referenceDistanceM / targetDistanceM;
        if (!(distanceRatio > 0.0f) || !std::isfinite(distanceRatio))
        {
            return false;
        }

        signal = referenceSignal * distanceRatio * distanceRatio;
        return std::isfinite(signal) && signal > 0.0f;
    }

    inline bool TryComputeInverseSquareDistanceFromReferenceSignal(
        float measuredSignal,
        float referenceSignal,
        float referenceDistanceM,
        float& distanceM)
    {
        distanceM = 0.0f;
        if (!std::isfinite(measuredSignal) ||
            !std::isfinite(referenceSignal) ||
            !std::isfinite(referenceDistanceM) ||
            measuredSignal <= 0.0f ||
            referenceSignal <= 0.0f ||
            referenceDistanceM <= 0.0f)
        {
            return false;
        }

        const float signalRatio = measuredSignal / referenceSignal;
        if (!(signalRatio > 0.0f) || !std::isfinite(signalRatio))
        {
            return false;
        }

        const float distanceScale = std::sqrt(1.0f / signalRatio);
        if (!(distanceScale > 0.0f) || !std::isfinite(distanceScale))
        {
            return false;
        }

        distanceM = referenceDistanceM * distanceScale;
        return std::isfinite(distanceM) && distanceM > 0.0f;
    }

    inline bool TryComputeFrontWallHalfwayIntoAdjacentDistanceM(
        float cellSizeM,
        float wallThicknessM,
        float frontSensorForwardOffsetM,
        float& distanceThresholdM)
    {
        distanceThresholdM = 0.0f;
        if (!std::isfinite(cellSizeM) ||
            !std::isfinite(wallThicknessM) ||
            !std::isfinite(frontSensorForwardOffsetM) ||
            cellSizeM <= 0.0f ||
            wallThicknessM < 0.0f)
        {
            return false;
        }

        distanceThresholdM = cellSizeM - (0.5f * wallThicknessM) - frontSensorForwardOffsetM;
        return std::isfinite(distanceThresholdM) && distanceThresholdM > 0.0f;
    }

    inline bool TryExpandWallThresholdDistanceM(float onThresholdM, float hysteresisDistanceM, float& offThresholdM)
    {
        offThresholdM = 0.0f;
        if (!std::isfinite(onThresholdM) ||
            !std::isfinite(hysteresisDistanceM) ||
            onThresholdM <= 0.0f ||
            hysteresisDistanceM < 0.0f)
        {
            return false;
        }

        offThresholdM = onThresholdM + hysteresisDistanceM;
        return std::isfinite(offThresholdM) && offThresholdM >= onThresholdM;
    }

    inline bool TryClampWallThresholdDistanceRangeM(
        float preferredOnThresholdM,
        float preferredOffThresholdM,
        float maxOnThresholdM,
        float maxOffThresholdM,
        float& onThresholdM,
        float& offThresholdM)
    {
        onThresholdM = 0.0f;
        offThresholdM = 0.0f;
        if (!std::isfinite(preferredOnThresholdM) ||
            !std::isfinite(preferredOffThresholdM) ||
            !std::isfinite(maxOnThresholdM) ||
            !std::isfinite(maxOffThresholdM) ||
            preferredOnThresholdM <= 0.0f ||
            preferredOffThresholdM < preferredOnThresholdM ||
            maxOnThresholdM <= 0.0f ||
            maxOffThresholdM < maxOnThresholdM)
        {
            return false;
        }

        onThresholdM = (preferredOnThresholdM < maxOnThresholdM) ? preferredOnThresholdM : maxOnThresholdM;
        offThresholdM = (preferredOffThresholdM < maxOffThresholdM) ? preferredOffThresholdM : maxOffThresholdM;
        if (offThresholdM < onThresholdM)
        {
            offThresholdM = onThresholdM;
        }

        return std::isfinite(onThresholdM) &&
            std::isfinite(offThresholdM) &&
            onThresholdM > 0.0f &&
            offThresholdM >= onThresholdM;
    }

    inline double StableLogExpm1(double x)
    {
        if (!std::isfinite(x) || x <= 0.0)
        {
            return std::numeric_limits<double>::quiet_NaN();
        }

        if (x > 50.0)
        {
            return x;
        }

        return std::log(std::expm1(x));
    }

    inline double StableSoftplus(double x)
    {
        if (!std::isfinite(x))
        {
            return std::numeric_limits<double>::quiet_NaN();
        }

        if (x > 50.0)
        {
            return x;
        }

        if (x < -50.0)
        {
            return std::exp(x);
        }

        return std::log1p(std::exp(x));
    }

    inline bool TryFitAmbientAwareLogDifferentialSignalModel(
        const WallSensorCalibrationCurve& curve,
        float& gain,
        float& lightScale)
    {
        gain = 0.0f;
        lightScale = 0.0f;
        if (curve.GetCount() < 2U)
        {
            return false;
        }

        double maxMeasuredValue = 0.0;
        uint8_t validPointCount = 0U;
        for (uint8_t index = 0U; index < curve.GetCount(); ++index)
        {
            const WallSensorCalibrationCurve::Point& point = curve.GetPoint(index);
            if (!std::isfinite(point.measuredValue) ||
                !std::isfinite(point.actualDistanceM) ||
                !std::isfinite(point.ambientLight) ||
                point.measuredValue <= 0.0f ||
                point.actualDistanceM <= 0.0f ||
                point.ambientLight < 0.0f)
            {
                return false;
            }

            maxMeasuredValue = (std::max)(maxMeasuredValue, static_cast<double>(point.measuredValue));
            ++validPointCount;
        }

        if (validPointCount < 2U || !(maxMeasuredValue > 0.0))
        {
            return false;
        }

        const double minGain = (std::max)(maxMeasuredValue / 40.0, 0.005);
        const double maxGain = (std::max)(maxMeasuredValue * 40.0, 5.0);
        double logMinGain = std::log(minGain);
        double logMaxGain = std::log(maxGain);
        double bestLogGain = 0.0;
        double bestMeanLogScale = 0.0;
        double bestError = std::numeric_limits<double>::infinity();

        for (uint8_t pass = 0U; pass < 4U; ++pass)
        {
            const double sweepWidth = logMaxGain - logMinGain;
            const double stepSize = sweepWidth / 96.0;
            for (uint8_t step = 0U; step <= 96U; ++step)
            {
                const double t = static_cast<double>(step) / 96.0;
                const double candidateLogGain = logMinGain + (t * sweepWidth);
                const double candidateGain = std::exp(candidateLogGain);

                double sumLogScale = 0.0;
                double sumSquaredLogScale = 0.0;
                bool fitOk = true;
                for (uint8_t index = 0U; index < curve.GetCount(); ++index)
                {
                    const WallSensorCalibrationCurve::Point& point = curve.GetPoint(index);
                    const double normalizedMeasuredValue = static_cast<double>(point.measuredValue) / candidateGain;
                    const double logScaleSample =
                        (2.0 * std::log(static_cast<double>(point.actualDistanceM))) +
                        (static_cast<double>(point.ambientLight) / candidateGain) +
                        StableLogExpm1(normalizedMeasuredValue);
                    if (!std::isfinite(logScaleSample))
                    {
                        fitOk = false;
                        break;
                    }

                    sumLogScale += logScaleSample;
                    sumSquaredLogScale += logScaleSample * logScaleSample;
                }

                if (!fitOk)
                {
                    continue;
                }

                const double meanLogScale = sumLogScale / static_cast<double>(curve.GetCount());
                const double error =
                    sumSquaredLogScale -
                    (static_cast<double>(curve.GetCount()) * meanLogScale * meanLogScale);
                if (error < bestError)
                {
                    bestError = error;
                    bestLogGain = candidateLogGain;
                    bestMeanLogScale = meanLogScale;
                }
            }

            if (!std::isfinite(bestError))
            {
                return false;
            }

            logMinGain = (std::max)(std::log(minGain), bestLogGain - stepSize);
            logMaxGain = (std::min)(std::log(maxGain), bestLogGain + stepSize);
        }

        gain = static_cast<float>(std::exp(bestLogGain));
        lightScale = static_cast<float>(std::exp(bestMeanLogScale));
        return
            std::isfinite(gain) &&
            std::isfinite(lightScale) &&
            gain > 0.0f &&
            lightScale > 0.0f;
    }

    inline bool TryComputeMeasuredValueForActualDistanceUsingAmbientAwareLogSignalModel(
        float gain,
        float lightScale,
        float ambientLight,
        float actualDistanceM,
        float& measuredValue)
    {
        measuredValue = 0.0f;
        if (!std::isfinite(gain) ||
            !std::isfinite(lightScale) ||
            !std::isfinite(ambientLight) ||
            !std::isfinite(actualDistanceM) ||
            gain <= 0.0f ||
            lightScale <= 0.0f ||
            ambientLight < 0.0f ||
            actualDistanceM <= 0.0f)
        {
            return false;
        }

        const double logRatio =
            std::log(static_cast<double>(lightScale)) -
            (static_cast<double>(ambientLight) / static_cast<double>(gain)) -
            (2.0 * std::log(static_cast<double>(actualDistanceM)));
        const double predictedMeasuredValue = static_cast<double>(gain) * StableSoftplus(logRatio);
        measuredValue = static_cast<float>(predictedMeasuredValue);
        return std::isfinite(measuredValue) && measuredValue > 0.0f;
    }
}
