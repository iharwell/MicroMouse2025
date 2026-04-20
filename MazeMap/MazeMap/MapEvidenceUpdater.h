#pragma once
// Declares the wall-evidence accumulator that converts accepted UKF wall updates into maze edge state.

#include "WallGeometryModel.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace MazeMap
{
    // Tracks map-edge evidence derived from wall observations accepted by the UKF.
    class EXPORT MapEvidenceUpdater
    {
    public:
        static constexpr uint8_t kMazeSize = 16U;
        static constexpr uint8_t kDirectionCount = 4U;

        // Accumulated evidence for one maze edge.
        struct EdgeEvidence
        {
            int8_t score = 0;
            WallState state = WallState::Unknown;
        };

        // Score thresholds and weighting policy for wall-evidence accumulation.
        struct Config
        {
            int8_t maxScore = 8;
            int8_t wallThreshold = 3;
            int8_t openThreshold = 3;
            int8_t wallHitWeight = 1;
            int8_t openHitWeight = 1;
            float minimumConfidence = 0.25f;
        };

        void Reset() noexcept;

        const EdgeEvidence& Get(const CellCoordinates& cell, Direction direction) const noexcept;

        bool Apply(
            const CellCoordinates& cell,
            Direction direction,
            const WallObs& observation,
            const GeometryPrediction& bestFit,
            bool freezeMutation = false) noexcept;

        bool Apply(
            const CellCoordinates& cell,
            Direction direction,
            const WallObs& observation,
            const GeometryPrediction& bestFit,
            const Config& config,
            bool freezeMutation = false) noexcept;

    private:
        static constexpr size_t kBoundaryCount = static_cast<size_t>(kMazeSize) + 1U;

        using HorizontalBoundaryRow = std::array<EdgeEvidence, kBoundaryCount>;
        using HorizontalPlane = std::array<HorizontalBoundaryRow, kMazeSize>;
        using VerticalCellRow = std::array<EdgeEvidence, kMazeSize>;
        using VerticalPlane = std::array<VerticalCellRow, kBoundaryCount>;

        HorizontalPlane _horizontalEdges{};
        VerticalPlane _verticalEdges{};

        static bool isOrdinal(Direction direction) noexcept;
        static int8_t saturatingAdd(int8_t current, int8_t delta, int8_t limit) noexcept;
        static WallState stateFromScore(int8_t score, const Config& config) noexcept;
        EdgeEvidence& edgeFor(const CellCoordinates& cell, Direction direction) noexcept;
        const EdgeEvidence& edgeFor(const CellCoordinates& cell, Direction direction) const noexcept;
    };
}
