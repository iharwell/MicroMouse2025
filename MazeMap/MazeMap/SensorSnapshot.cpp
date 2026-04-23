#include "pch.h"
#include "SensorSnapshot.h"

#include "CoreConfig.h"
#include "WallObservationPipeline.h"

#include <cmath>

void ClearFrontWallObservationDecision(SensorSnapshot& snapshot)
{
    snapshot.frontWall = false;
    snapshot.frontLeftWall = false;
    snapshot.frontRightWall = false;
    snapshot.frontWallObservationValid = false;
    snapshot.frontWallUsesFallbackDetection = false;
    snapshot.frontWallUsesCharacterizationDetection = false;
}


namespace
{
    bool ObservationVoteWinsMajority(uint8_t votes, uint8_t sampleCount)
{
    return sampleCount > 0U && votes >= static_cast<uint8_t>((sampleCount / 2U) + 1U);
}

    float AverageFiniteObservationValue(float sum, uint8_t count, float fallbackValue)
{
    return (count > 0U) ? (sum / static_cast<float>(count)) : fallbackValue;
}

}

bool BuildEvidenceObservationSnapshot(
    const SensorSnapshot* samples,
    uint8_t sampleCount,
    SensorSnapshot& combinedSnapshot,
    RollingObservationVoteSummary& voteSummary)
{
    if (samples == nullptr || sampleCount == 0U)
    {
        return false;
    }

    voteSummary = RollingObservationVoteSummary{};
    voteSummary.sampleCount = sampleCount;
    combinedSnapshot = SensorSnapshot{};

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
    MazeMap::WallDecisionAccumulator frontEvidence{};
    MazeMap::WallDecisionAccumulator leftEvidence{};
    MazeMap::WallDecisionAccumulator rightEvidence{};
    bool leftTransitionDetected = false;
    bool rightTransitionDetected = false;

    for (uint8_t sampleIndex = 0U; sampleIndex < sampleCount; ++sampleIndex)
    {
        const SensorSnapshot& sample = samples[sampleIndex];
        if (sample.frontWall)
        {
            ++voteSummary.frontWallVotes;
        }
        if (sample.frontLeftWall)
        {
            ++voteSummary.frontLeftWallVotes;
        }
        if (sample.frontRightWall)
        {
            ++voteSummary.frontRightWallVotes;
        }
        if (sample.frontWallUsesFallbackDetection)
        {
            ++voteSummary.frontFallbackVotes;
        }
        if (sample.leftWallObservation)
        {
            ++voteSummary.leftWallVotes;
        }
        if (sample.rightWallObservation)
        {
            ++voteSummary.rightWallVotes;
        }
        if (sample.leftWallObservationWindowValid)
        {
            ++voteSummary.leftWindowValidVotes;
        }
        if (sample.rightWallObservationWindowValid)
        {
            ++voteSummary.rightWindowValidVotes;
        }

        frontEvidence.Update(
            sample.frontWall ?
                MazeMap::WallSampleClassification::WallHit :
                MazeMap::WallSampleClassification::WallMiss,
            MazeMap::Config::kWallMapEvidenceHitWeight,
            MazeMap::Config::kWallMapEvidenceMissWeight,
            MazeMap::Config::kWallMapEvidenceUnknownDecay);

        leftEvidence.Update(
            sample.leftWallObservationWindowValid ?
                (sample.leftWallObservation ?
                    MazeMap::WallSampleClassification::WallHit :
                    MazeMap::WallSampleClassification::WallMiss) :
                MazeMap::WallSampleClassification::Unknown,
            MazeMap::Config::kWallMapEvidenceHitWeight,
            MazeMap::Config::kWallMapEvidenceMissWeight,
            MazeMap::Config::kWallMapEvidenceUnknownDecay);
        if (sample.leftTransitionDetected)
        {
            leftTransitionDetected = true;
            leftEvidence.InjectMissImpulse(MazeMap::Config::kWallMapEvidenceTransitionMissWeight);
        }

        rightEvidence.Update(
            sample.rightWallObservationWindowValid ?
                (sample.rightWallObservation ?
                    MazeMap::WallSampleClassification::WallHit :
                    MazeMap::WallSampleClassification::WallMiss) :
                MazeMap::WallSampleClassification::Unknown,
            MazeMap::Config::kWallMapEvidenceHitWeight,
            MazeMap::Config::kWallMapEvidenceMissWeight,
            MazeMap::Config::kWallMapEvidenceUnknownDecay);
        if (sample.rightTransitionDetected)
        {
            rightTransitionDetected = true;
            rightEvidence.InjectMissImpulse(MazeMap::Config::kWallMapEvidenceTransitionMissWeight);
        }

        if (std::isfinite(sample.frontLeftDistanceM))
        {
            frontLeftDistanceSum += sample.frontLeftDistanceM;
            ++frontLeftDistanceCount;
        }
        if (std::isfinite(sample.frontRightDistanceM))
        {
            frontRightDistanceSum += sample.frontRightDistanceM;
            ++frontRightDistanceCount;
        }
        if (std::isfinite(sample.frontLeftDifferentialLight))
        {
            frontLeftDifferentialLightSum += sample.frontLeftDifferentialLight;
            ++frontLeftDifferentialLightCount;
        }
        if (std::isfinite(sample.frontRightDifferentialLight))
        {
            frontRightDifferentialLightSum += sample.frontRightDifferentialLight;
            ++frontRightDifferentialLightCount;
        }
        if (std::isfinite(sample.sideLeftDistanceM))
        {
            sideLeftDistanceSum += sample.sideLeftDistanceM;
            ++sideLeftDistanceCount;
        }
        if (std::isfinite(sample.sideRightDistanceM))
        {
            sideRightDistanceSum += sample.sideRightDistanceM;
            ++sideRightDistanceCount;
        }
        if (std::isfinite(sample.sideLeftDifferentialLight))
        {
            sideLeftDifferentialLightSum += sample.sideLeftDifferentialLight;
            ++sideLeftDifferentialLightCount;
        }
        if (std::isfinite(sample.sideRightDifferentialLight))
        {
            sideRightDifferentialLightSum += sample.sideRightDifferentialLight;
            ++sideRightDifferentialLightCount;
        }
        if (std::isfinite(sample.corridorErrorM))
        {
            corridorErrorSum += sample.corridorErrorM;
            ++corridorErrorCount;
        }
        if (std::isfinite(sample.frontSkewM))
        {
            frontSkewSum += sample.frontSkewM;
            ++frontSkewCount;
        }
        if (std::isfinite(sample.planarAccelMps2))
        {
            planarAccelSum += sample.planarAccelMps2;
            ++planarAccelCount;
        }
        if (std::isfinite(sample.gyroRadps))
        {
            gyroSum += sample.gyroRadps;
            ++gyroCount;
        }
    }

    const SensorSnapshot& lastSample = samples[sampleCount - 1U];
    combinedSnapshot.frontLeftDistanceM =
        AverageFiniteObservationValue(frontLeftDistanceSum, frontLeftDistanceCount, lastSample.frontLeftDistanceM);
    combinedSnapshot.frontRightDistanceM =
        AverageFiniteObservationValue(frontRightDistanceSum, frontRightDistanceCount, lastSample.frontRightDistanceM);
    combinedSnapshot.frontLeftDifferentialLight =
        AverageFiniteObservationValue(frontLeftDifferentialLightSum, frontLeftDifferentialLightCount, lastSample.frontLeftDifferentialLight);
    combinedSnapshot.frontRightDifferentialLight =
        AverageFiniteObservationValue(frontRightDifferentialLightSum, frontRightDifferentialLightCount, lastSample.frontRightDifferentialLight);
    combinedSnapshot.sideLeftDistanceM =
        AverageFiniteObservationValue(sideLeftDistanceSum, sideLeftDistanceCount, lastSample.sideLeftDistanceM);
    combinedSnapshot.sideRightDistanceM =
        AverageFiniteObservationValue(sideRightDistanceSum, sideRightDistanceCount, lastSample.sideRightDistanceM);
    combinedSnapshot.sideLeftDifferentialLight =
        AverageFiniteObservationValue(sideLeftDifferentialLightSum, sideLeftDifferentialLightCount, lastSample.sideLeftDifferentialLight);
    combinedSnapshot.sideRightDifferentialLight =
        AverageFiniteObservationValue(sideRightDifferentialLightSum, sideRightDifferentialLightCount, lastSample.sideRightDifferentialLight);
    combinedSnapshot.corridorErrorM =
        AverageFiniteObservationValue(corridorErrorSum, corridorErrorCount, lastSample.corridorErrorM);
    combinedSnapshot.frontSkewM =
        AverageFiniteObservationValue(frontSkewSum, frontSkewCount, lastSample.frontSkewM);
    combinedSnapshot.planarAccelMps2 =
        AverageFiniteObservationValue(planarAccelSum, planarAccelCount, lastSample.planarAccelMps2);
    combinedSnapshot.gyroRadps =
        AverageFiniteObservationValue(gyroSum, gyroCount, lastSample.gyroRadps);
    const MazeMap::WallSampleClassification frontDecision =
        frontEvidence.FinalClassification(MazeMap::Config::kWallMapEvidenceCommitThreshold);
    const MazeMap::WallSampleClassification leftDecision =
        leftEvidence.FinalClassification(MazeMap::Config::kWallMapEvidenceCommitThreshold);
    const MazeMap::WallSampleClassification rightDecision =
        rightEvidence.FinalClassification(MazeMap::Config::kWallMapEvidenceCommitThreshold);
    combinedSnapshot.frontWall = frontDecision == MazeMap::WallSampleClassification::WallHit;
    combinedSnapshot.frontLeftWall = ObservationVoteWinsMajority(voteSummary.frontLeftWallVotes, sampleCount);
    combinedSnapshot.frontRightWall = ObservationVoteWinsMajority(voteSummary.frontRightWallVotes, sampleCount);
    combinedSnapshot.frontWallObservationValid = frontDecision != MazeMap::WallSampleClassification::Unknown;
    combinedSnapshot.frontWallUsesFallbackDetection = combinedSnapshot.frontWallObservationValid;
    combinedSnapshot.frontWallUsesCharacterizationDetection = false;
    combinedSnapshot.leftWallObservation = leftDecision == MazeMap::WallSampleClassification::WallHit;
    combinedSnapshot.rightWallObservation = rightDecision == MazeMap::WallSampleClassification::WallHit;
    combinedSnapshot.leftWall = combinedSnapshot.leftWallObservation;
    combinedSnapshot.rightWall = combinedSnapshot.rightWallObservation;
    combinedSnapshot.leftWallObservationWindowValid = leftDecision != MazeMap::WallSampleClassification::Unknown;
    combinedSnapshot.rightWallObservationWindowValid = rightDecision != MazeMap::WallSampleClassification::Unknown;
    combinedSnapshot.leftTransitionDetected = leftTransitionDetected;
    combinedSnapshot.rightTransitionDetected = rightTransitionDetected;
    return true;
}
