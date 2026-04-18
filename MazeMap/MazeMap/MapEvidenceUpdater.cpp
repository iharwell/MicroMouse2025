#include "pch.h"
#include "MapEvidenceUpdater.h"

namespace MazeMap
{
    void MapEvidenceUpdater::Reset() noexcept
    {
        for (auto& plane : _edges)
        {
            for (auto& row : plane)
            {
                for (auto& edge : row)
                {
                    edge = EdgeEvidence{};
                }
            }
        }
    }

    const MapEvidenceUpdater::EdgeEvidence& MapEvidenceUpdater::Get(
        const CellCoordinates& cell,
        Direction direction) const noexcept
    {
        return _edges[cell.GetX()][cell.GetY()][directionIndex(direction)];
    }

    bool MapEvidenceUpdater::isOrdinal(Direction direction) noexcept
    {
        return
            direction == Direction::Up ||
            direction == Direction::Down ||
            direction == Direction::Left ||
            direction == Direction::Right;
    }

    size_t MapEvidenceUpdater::directionIndex(Direction direction) noexcept
    {
        switch (direction)
        {
        case Direction::Up:
            return 0U;
        case Direction::Down:
            return 1U;
        case Direction::Left:
            return 2U;
        case Direction::Right:
        default:
            return 3U;
        }
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

    void MapEvidenceUpdater::setMirrored(
        const CellCoordinates& cell,
        Direction direction,
        const EdgeEvidence& evidence) noexcept
    {
        _edges[cell.GetX()][cell.GetY()][directionIndex(direction)] = evidence;
        if (cell.IsValidMove(direction))
        {
            const CellCoordinates neighbor = cell >> direction;
            _edges[neighbor.GetX()][neighbor.GetY()][directionIndex(-direction)] = evidence;
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
            !observation.valid ||
            (observation.confidence < config.minimumConfidence) ||
            !isOrdinal(direction))
        {
            return false;
        }

        if ((observation.cls == ObsClass::Ambiguous) || (observation.cls == ObsClass::PostLike))
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

        EdgeEvidence updated = Get(targetCell, targetDirection);
        if (observation.cls == ObsClass::WallLike)
        {
            updated.score = saturatingAdd(updated.score, config.wallHitWeight, config.maxScore);
        }
        else if (observation.cls == ObsClass::OpenLike && bestFit.type != GeometryHitType::Post)
        {
            updated.score = saturatingAdd(updated.score, static_cast<int8_t>(-config.openHitWeight), config.maxScore);
        }
        else
        {
            return false;
        }

        updated.state = stateFromScore(updated.score, config);
        setMirrored(targetCell, targetDirection, updated);
        return true;
    }
}
