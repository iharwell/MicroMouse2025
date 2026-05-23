#include "pch.h"
#include "SensorSnapshot.h"

#include "CoreConfig.h"
#include "WallObservationPipeline.h"

#include <algorithm>
#include <cmath>

static bool ObservationVoteWinsMajority(uint8_t votes, uint8_t sampleCount)
{
    return sampleCount > 0U && votes >= static_cast<uint8_t>((sampleCount / 2U) + 1U);
}

static float AverageFiniteObservationValue(float sum, uint8_t count, float fallbackValue)
{
    return (count > 0U) ? (sum / static_cast<float>(count)) : fallbackValue;
}

static void UpdateWallEvidenceScore(
    const MazeMap::WallSampleClassification classification,
    const float hitWeight,
    const float missWeight,
    const float unknownDecay,
    float& score) noexcept
{
    switch (classification)
    {
    case MazeMap::WallSampleClassification::WallHit:
        if (std::isfinite(hitWeight) && hitWeight > 0.0f)
        {
            score += hitWeight;
        }
        break;
    case MazeMap::WallSampleClassification::WallMiss:
        if (std::isfinite(missWeight) && missWeight > 0.0f)
        {
            score -= missWeight;
        }
        break;
    case MazeMap::WallSampleClassification::Unknown:
    default:
        if (std::isfinite(unknownDecay) && unknownDecay > 0.0f)
        {
            if (score > 0.0f)
            {
                score = (std::max)(0.0f, score - unknownDecay);
            }
            else if (score < 0.0f)
            {
                score = (std::min)(0.0f, score + unknownDecay);
            }
        }
        break;
    }
}

static void InjectWallEvidenceMissImpulse(const float missWeight, float& score) noexcept
{
    if (std::isfinite(missWeight) && missWeight > 0.0f)
    {
        score -= missWeight;
    }
}

static MazeMap::WallSampleClassification FinalWallEvidenceClassification(
    const float score,
    const float commitThreshold) noexcept
{
    if (!(std::isfinite(commitThreshold) && commitThreshold > 0.0f))
    {
        return MazeMap::WallSampleClassification::Unknown;
    }

    if (score >= commitThreshold)
    {
        return MazeMap::WallSampleClassification::WallHit;
    }
    if (score <= -commitThreshold)
    {
        return MazeMap::WallSampleClassification::WallMiss;
    }

    return MazeMap::WallSampleClassification::Unknown;
}

bool SensorSnapshot::BuildEvidenceObservationSnapshot(
    const SensorSnapshot* samples,
    uint8_t sampleCount) noexcept
{
    if (samples == nullptr || sampleCount == 0U)
    {
        return false;
    }

    SensorSnapshot& combinedSnapshot = *this;
    combinedSnapshot = SensorSnapshot{};

    uint8_t frontLeftWallVotes = 0U;
    uint8_t frontRightWallVotes = 0U;
    uint8_t frontObservationCount = 0U;
    float frontLeftDistanceSum = 0.0f;
    float frontRightDistanceSum = 0.0f;
    float frontLeftDifferentialLightSum = 0.0f;
    float frontRightDifferentialLightSum = 0.0f;
    float sideLeftDistanceSum = 0.0f;
    float sideRightDistanceSum = 0.0f;
    float sideLeftDifferentialLightSum = 0.0f;
    float sideRightDifferentialLightSum = 0.0f;
    float corridorErrorSum = 0.0f;
    float frontSkewSum = 0.0f;
    float planarAccelSum = 0.0f;
    float gyroSum = 0.0f;
    uint8_t frontLeftDistanceCount = 0U;
    uint8_t frontRightDistanceCount = 0U;
    uint8_t frontLeftDifferentialLightCount = 0U;
    uint8_t frontRightDifferentialLightCount = 0U;
    uint8_t sideLeftDistanceCount = 0U;
    uint8_t sideRightDistanceCount = 0U;
    uint8_t sideLeftDifferentialLightCount = 0U;
    uint8_t sideRightDifferentialLightCount = 0U;
    uint8_t corridorErrorCount = 0U;
    uint8_t frontSkewCount = 0U;
    uint8_t planarAccelCount = 0U;
    uint8_t gyroCount = 0U;
    float frontEvidenceScore = 0.0f;
    float leftEvidenceScore = 0.0f;
    float rightEvidenceScore = 0.0f;
    bool leftTransitionDetected = false;
    bool rightTransitionDetected = false;
    bool encoderObservationValid = false;
    MazeMap::EncoderObs encoderObservation{};
    MazeMap::WallObs frontLeftWallSensorObservation{};
    MazeMap::WallObs frontRightWallSensorObservation{};
    MazeMap::WallObs sideLeftWallSensorObservation{};
    MazeMap::WallObs sideRightWallSensorObservation{};

    for (uint8_t sampleIndex = 0U; sampleIndex < sampleCount; ++sampleIndex)
    {
        const SensorSnapshot& sample = samples[sampleIndex];
        if (sample.EncoderObservationValid())
        {
            encoderObservation = sample.EncoderObservation();
            encoderObservationValid = true;
        }

        const bool frontObservationValid = sample.FrontWallObservationValid();
        const MazeMap::WallSampleClassification frontClassification =
            frontObservationValid ?
            (sample.HasFrontWall() ?
                MazeMap::WallSampleClassification::WallHit :
                MazeMap::WallSampleClassification::WallMiss) :
            MazeMap::WallSampleClassification::Unknown;
        if (frontObservationValid)
        {
            ++frontObservationCount;
            if (sample.HasFrontLeftWall())
            {
                ++frontLeftWallVotes;
            }
            if (sample.HasFrontRightWall())
            {
                ++frontRightWallVotes;
            }
        }
        UpdateWallEvidenceScore(
            frontClassification,
            MazeMap::Config::kWallMapEvidenceHitWeight,
            MazeMap::Config::kWallMapEvidenceMissWeight,
            MazeMap::Config::kWallMapEvidenceUnknownDecay,
            frontEvidenceScore);

        UpdateWallEvidenceScore(
            sample.LeftWallObservationWindowValid() ?
                (sample.HasLeftWallObservation() ?
                    MazeMap::WallSampleClassification::WallHit :
                    MazeMap::WallSampleClassification::WallMiss) :
                MazeMap::WallSampleClassification::Unknown,
            MazeMap::Config::kWallMapEvidenceHitWeight,
            MazeMap::Config::kWallMapEvidenceMissWeight,
            MazeMap::Config::kWallMapEvidenceUnknownDecay,
            leftEvidenceScore);
        if (sample.LeftTransitionDetected())
        {
            leftTransitionDetected = true;
            InjectWallEvidenceMissImpulse(
                MazeMap::Config::kWallMapEvidenceTransitionMissWeight,
                leftEvidenceScore);
        }

        UpdateWallEvidenceScore(
            sample.RightWallObservationWindowValid() ?
                (sample.HasRightWallObservation() ?
                    MazeMap::WallSampleClassification::WallHit :
                    MazeMap::WallSampleClassification::WallMiss) :
                MazeMap::WallSampleClassification::Unknown,
            MazeMap::Config::kWallMapEvidenceHitWeight,
            MazeMap::Config::kWallMapEvidenceMissWeight,
            MazeMap::Config::kWallMapEvidenceUnknownDecay,
            rightEvidenceScore);
        if (sample.RightTransitionDetected())
        {
            rightTransitionDetected = true;
            InjectWallEvidenceMissImpulse(
                MazeMap::Config::kWallMapEvidenceTransitionMissWeight,
                rightEvidenceScore);
        }

        if (std::isfinite(sample.FrontLeftDistanceM()))
        {
            frontLeftDistanceSum += sample.FrontLeftDistanceM();
            ++frontLeftDistanceCount;
        }
        if (std::isfinite(sample.FrontRightDistanceM()))
        {
            frontRightDistanceSum += sample.FrontRightDistanceM();
            ++frontRightDistanceCount;
        }
        if (std::isfinite(sample.FrontLeftDifferentialLight()))
        {
            frontLeftDifferentialLightSum += sample.FrontLeftDifferentialLight();
            ++frontLeftDifferentialLightCount;
        }
        if (std::isfinite(sample.FrontRightDifferentialLight()))
        {
            frontRightDifferentialLightSum += sample.FrontRightDifferentialLight();
            ++frontRightDifferentialLightCount;
        }
        if (std::isfinite(sample.SideLeftDistanceM()))
        {
            sideLeftDistanceSum += sample.SideLeftDistanceM();
            ++sideLeftDistanceCount;
        }
        if (std::isfinite(sample.SideRightDistanceM()))
        {
            sideRightDistanceSum += sample.SideRightDistanceM();
            ++sideRightDistanceCount;
        }
        if (std::isfinite(sample.SideLeftDifferentialLight()))
        {
            sideLeftDifferentialLightSum += sample.SideLeftDifferentialLight();
            ++sideLeftDifferentialLightCount;
        }
        if (std::isfinite(sample.SideRightDifferentialLight()))
        {
            sideRightDifferentialLightSum += sample.SideRightDifferentialLight();
            ++sideRightDifferentialLightCount;
        }
        if (std::isfinite(sample.CorridorErrorM()))
        {
            corridorErrorSum += sample.CorridorErrorM();
            ++corridorErrorCount;
        }
        if (std::isfinite(sample.FrontSkewM()))
        {
            frontSkewSum += sample.FrontSkewM();
            ++frontSkewCount;
        }
        if (std::isfinite(sample.PlanarAccelerationMps2()))
        {
            planarAccelSum += sample.PlanarAccelerationMps2();
            ++planarAccelCount;
        }
        if (std::isfinite(sample.YawRateRadps()))
        {
            gyroSum += sample.YawRateRadps();
            ++gyroCount;
        }
        if (frontObservationValid && sample.FrontLeftWallSensorObservation().IsValid())
        {
            frontLeftWallSensorObservation = sample.FrontLeftWallSensorObservation();
        }
        if (frontObservationValid && sample.FrontRightWallSensorObservation().IsValid())
        {
            frontRightWallSensorObservation = sample.FrontRightWallSensorObservation();
        }
        if (sample.SideLeftWallSensorObservation().IsValid())
        {
            sideLeftWallSensorObservation = sample.SideLeftWallSensorObservation();
        }
        if (sample.SideRightWallSensorObservation().IsValid())
        {
            sideRightWallSensorObservation = sample.SideRightWallSensorObservation();
        }
    }

    const SensorSnapshot& lastSample = samples[sampleCount - 1U];
    combinedSnapshot.SetEncoderTotals(
        lastSample.LeftEncoderTotalCounts(),
        lastSample.RightEncoderTotalCounts());
    combinedSnapshot.SetEncoderDistancesM(
        lastSample.LeftEncoderDistanceM(),
        lastSample.RightEncoderDistanceM());
    combinedSnapshot.SetFrontLeftDistanceM(
        AverageFiniteObservationValue(frontLeftDistanceSum, frontLeftDistanceCount, lastSample.FrontLeftDistanceM()));
    combinedSnapshot.SetFrontRightDistanceM(
        AverageFiniteObservationValue(frontRightDistanceSum, frontRightDistanceCount, lastSample.FrontRightDistanceM()));
    combinedSnapshot.SetFrontLeftDifferentialLight(
        AverageFiniteObservationValue(frontLeftDifferentialLightSum, frontLeftDifferentialLightCount, lastSample.FrontLeftDifferentialLight()));
    combinedSnapshot.SetFrontRightDifferentialLight(
        AverageFiniteObservationValue(frontRightDifferentialLightSum, frontRightDifferentialLightCount, lastSample.FrontRightDifferentialLight()));
    combinedSnapshot.SetSideLeftDistanceM(
        AverageFiniteObservationValue(sideLeftDistanceSum, sideLeftDistanceCount, lastSample.SideLeftDistanceM()));
    combinedSnapshot.SetSideRightDistanceM(
        AverageFiniteObservationValue(sideRightDistanceSum, sideRightDistanceCount, lastSample.SideRightDistanceM()));
    combinedSnapshot.SetSideLeftDifferentialLight(
        AverageFiniteObservationValue(sideLeftDifferentialLightSum, sideLeftDifferentialLightCount, lastSample.SideLeftDifferentialLight()));
    combinedSnapshot.SetSideRightDifferentialLight(
        AverageFiniteObservationValue(sideRightDifferentialLightSum, sideRightDifferentialLightCount, lastSample.SideRightDifferentialLight()));
    combinedSnapshot.SetCorridorErrorM(
        AverageFiniteObservationValue(corridorErrorSum, corridorErrorCount, lastSample.CorridorErrorM()));
    combinedSnapshot.SetFrontSkewM(
        AverageFiniteObservationValue(frontSkewSum, frontSkewCount, lastSample.FrontSkewM()));
    combinedSnapshot.SetPlanarAccelerationMps2(
        AverageFiniteObservationValue(planarAccelSum, planarAccelCount, lastSample.PlanarAccelerationMps2()));
    combinedSnapshot.SetYawRateRadps(
        AverageFiniteObservationValue(gyroSum, gyroCount, lastSample.YawRateRadps()));
    const MazeMap::WallSampleClassification frontDecision =
        FinalWallEvidenceClassification(
            frontEvidenceScore,
            MazeMap::Config::kWallMapEvidenceCommitThreshold);
    const MazeMap::WallSampleClassification leftDecision =
        FinalWallEvidenceClassification(
            leftEvidenceScore,
            MazeMap::Config::kWallMapEvidenceCommitThreshold);
    const MazeMap::WallSampleClassification rightDecision =
        FinalWallEvidenceClassification(
            rightEvidenceScore,
            MazeMap::Config::kWallMapEvidenceCommitThreshold);
    combinedSnapshot.SetFrontWall(frontDecision == MazeMap::WallSampleClassification::WallHit);
    combinedSnapshot.SetFrontLeftWall(ObservationVoteWinsMajority(frontLeftWallVotes, frontObservationCount));
    combinedSnapshot.SetFrontRightWall(ObservationVoteWinsMajority(frontRightWallVotes, frontObservationCount));
    combinedSnapshot.SetFrontWallObservationValid(frontDecision != MazeMap::WallSampleClassification::Unknown);
    combinedSnapshot.SetFrontWallUsesFallbackDetection(combinedSnapshot.FrontWallObservationValid());
    combinedSnapshot.SetFrontWallUsesCharacterizationDetection(false);
    combinedSnapshot.SetLeftWallObservation(leftDecision == MazeMap::WallSampleClassification::WallHit);
    combinedSnapshot.SetRightWallObservation(rightDecision == MazeMap::WallSampleClassification::WallHit);
    combinedSnapshot.SetLeftWall(combinedSnapshot.HasLeftWallObservation());
    combinedSnapshot.SetRightWall(combinedSnapshot.HasRightWallObservation());
    combinedSnapshot.SetLeftWallObservationWindowValid(leftDecision != MazeMap::WallSampleClassification::Unknown);
    combinedSnapshot.SetRightWallObservationWindowValid(rightDecision != MazeMap::WallSampleClassification::Unknown);
    combinedSnapshot.SetLeftTransitionDetected(leftTransitionDetected);
    combinedSnapshot.SetRightTransitionDetected(rightTransitionDetected);
    combinedSnapshot.SetEncoderObservation(encoderObservation, encoderObservationValid);
    combinedSnapshot.SetFrontLeftWallSensorObservation(frontLeftWallSensorObservation);
    combinedSnapshot.SetFrontRightWallSensorObservation(frontRightWallSensorObservation);
    combinedSnapshot.SetSideLeftWallSensorObservation(sideLeftWallSensorObservation);
    combinedSnapshot.SetSideRightWallSensorObservation(sideRightWallSensorObservation);
    return true;
}
