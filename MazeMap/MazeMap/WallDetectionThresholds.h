#pragma once

#include "WallSensorCalibration.h"

#include <cmath>
#include <limits>

namespace MazeMap
{
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

    inline bool TryComputeLinearWallSignalDistanceThresholdM(float calibrationDistanceM, float signalFraction, float& distanceThresholdM)
    {
        distanceThresholdM = 0.0f;
        if (!std::isfinite(calibrationDistanceM) ||
            !std::isfinite(signalFraction) ||
            calibrationDistanceM <= 0.0f ||
            signalFraction <= 0.0f)
        {
            return false;
        }

        const float distanceScale = std::sqrt(1.0f / signalFraction);
        if (!std::isfinite(distanceScale) || distanceScale <= 0.0f)
        {
            return false;
        }

        distanceThresholdM = calibrationDistanceM * distanceScale;
        return std::isfinite(distanceThresholdM) && distanceThresholdM > 0.0f;
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
