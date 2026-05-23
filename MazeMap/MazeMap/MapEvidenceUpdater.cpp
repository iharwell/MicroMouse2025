#include "pch.h"
#include "MapEvidenceUpdater.h"

namespace MazeMap
{
    void MapEvidenceUpdater::Reset() noexcept
    {
        _horizontalEdges = {};
        _verticalEdges = {};
    }

    const MapEvidenceUpdater::EdgeEvidence& MapEvidenceUpdater::Get(
        const CellCoordinates& cell,
        Direction direction) const noexcept
    {
        return edgeFor(cell, direction);
    }

    bool MapEvidenceUpdater::isOrdinal(Direction direction) noexcept
    {
        return
            direction == Direction::Up ||
            direction == Direction::Down ||
            direction == Direction::Left ||
            direction == Direction::Right;
    }

    int8_t MapEvidenceUpdater::saturatingAdd(int8_t current, int8_t delta, int8_t limit) noexcept
    {
        const int candidate = static_cast<int>(current) + static_cast<int>(delta);
        if (candidate > limit)
        {
            return limit;
        }
        if (candidate < -limit)
        {
            return static_cast<int8_t>(-limit);
        }
        return static_cast<int8_t>(candidate);
    }

    WallState MapEvidenceUpdater::stateFromScore(int8_t score, const Config& config) noexcept
    {
        if (score >= config.wallThreshold)
        {
            return WallState::Wall;
        }
        if (score <= -config.openThreshold)
        {
            return WallState::NoWall;
        }
        return WallState::Unknown;
    }

    MapEvidenceUpdater::EdgeEvidence& MapEvidenceUpdater::edgeFor(
        const CellCoordinates& cell,
        Direction direction) noexcept
    {
        return const_cast<EdgeEvidence&>(
            const_cast<const MapEvidenceUpdater*>(this)->edgeFor(cell, direction));
    }

    const MapEvidenceUpdater::EdgeEvidence& MapEvidenceUpdater::edgeFor(
        const CellCoordinates& cell,
        Direction direction) const noexcept
    {
        const size_t x = static_cast<size_t>(cell.GetX());
        const size_t y = static_cast<size_t>(cell.GetY());
        switch (direction)
        {
        case Direction::Up:
            return _horizontalEdges[x][y + 1U];
        case Direction::Down:
            return _horizontalEdges[x][y];
        case Direction::Left:
            return _verticalEdges[x][y];
        case Direction::Right:
        default:
            return _verticalEdges[x + 1U][y];
        }
    }

    bool MapEvidenceUpdater::Apply(
        const CellCoordinates& cell,
        Direction direction,
        const WallObs& observation,
        const GeometryPrediction& bestFit,
        bool freezeMutation) noexcept
    {
        return Apply(cell, direction, observation, bestFit, Config{}, freezeMutation);
    }

    bool MapEvidenceUpdater::Apply(
        const CellCoordinates& cell,
        Direction direction,
        const WallObs& observation,
        const GeometryPrediction& bestFit,
        const Config& config,
        bool freezeMutation) noexcept
    {
        if (freezeMutation ||
            !observation.IsValid() ||
            (observation.Confidence() < config.minimumConfidence) ||
            !isOrdinal(direction))
        {
            return false;
        }

        if ((observation.Class() == ObsClass::Ambiguous) || (observation.Class() == ObsClass::PostLike))
        {
            return false;
        }

        CellCoordinates targetCell = cell;
        Direction targetDirection = direction;
        if ((bestFit.type == GeometryHitType::WallFace) && isOrdinal(bestFit.edge))
        {
            targetCell = bestFit.cell;
            targetDirection = bestFit.edge;
        }

        EdgeEvidence& updated = edgeFor(targetCell, targetDirection);
        if (observation.Class() == ObsClass::WallLike)
        {
            updated.score = saturatingAdd(updated.score, config.wallHitWeight, config.maxScore);
        }
        else if (observation.Class() == ObsClass::OpenLike && bestFit.type != GeometryHitType::Post)
        {
            updated.score = saturatingAdd(updated.score, static_cast<int8_t>(-config.openHitWeight), config.maxScore);
        }
        else
        {
            return false;
        }

        updated.state = stateFromScore(updated.score, config);
        return true;
    }
}
