#pragma once
// Declares the wall-evidence accumulator that converts accepted UKF wall updates into maze edge state.

#include "WallGeometryModel.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace MazeMap
{
    // Accumulated evidence for one maze edge.
    struct EdgeEvidence
    {
        int8_t score = 0;
        WallState state = WallState::Unknown;
    };

    // Score thresholds and weighting policy for wall-evidence accumulation.
    struct MapEvidenceUpdaterConfig
    {
        int8_t maxScore = 8;
        int8_t wallThreshold = 3;
        int8_t openThreshold = 3;
        int8_t wallHitWeight = 1;
        int8_t openHitWeight = 1;
        float minimumConfidence = 0.25f;
    };

    // Tracks map-edge evidence derived from wall observations accepted by the UKF.
    class EXPORT MapEvidenceUpdater
    {
    public:
        static constexpr uint8_t kMazeSize = 16U;
        static constexpr uint8_t kDirectionCount = 4U;

        void Reset() noexcept;

        const EdgeEvidence& Get(const CellCoordinates& cell, Direction direction) const noexcept;

        bool Apply(
            const CellCoordinates& cell,
            Direction direction,
            const WallObs& observation,
            const GeometryPrediction& bestFit,
            const MapEvidenceUpdaterConfig& config = MapEvidenceUpdaterConfig{},
            bool freezeMutation = false) noexcept;

    private:
        using DirectionRow = std::array<EdgeEvidence, kDirectionCount>;
        using MazeRow = std::array<DirectionRow, kMazeSize>;
        using MazePlane = std::array<MazeRow, kMazeSize>;

        MazePlane _edges{};

        static bool isOrdinal(Direction direction) noexcept;
        static size_t directionIndex(Direction direction) noexcept;
        static int8_t saturatingAdd(int8_t current, int8_t delta, int8_t limit) noexcept;
        static WallState stateFromScore(int8_t score, const MapEvidenceUpdaterConfig& config) noexcept;
        void setMirrored(const CellCoordinates& cell, Direction direction, const EdgeEvidence& evidence) noexcept;
    };
}
