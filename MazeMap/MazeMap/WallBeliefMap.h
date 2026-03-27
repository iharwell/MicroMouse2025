#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

#include "Cell.h"
#include "CellCoordinates.h"
#include "Direction.h"
#include "WallObservationPipeline.h"

namespace MazeMap
{
    struct WallBeliefConfig
    {
        float hitLogOdds = 1.20f;
        float missLogOdds = 1.20f;
        float contradictoryMissLogOdds = 0.55f;
        float setThreshold = 1.00f;
        float clearThreshold = -1.00f;
        float saturationMagnitude = 3.50f;
    };

    struct WallBeliefState
    {
        float logOdds = 0.0f;
        WallState hardState = WallState::Unknown;
        uint16_t visitCount = 0U;
        uint16_t contradictionCount = 0U;
        uint32_t lastUpdateTick = 0UL;
    };

    struct WallBeliefUpdate
    {
        bool valid = false;
        bool changed = false;
        float logOdds = 0.0f;
        WallState hardState = WallState::Unknown;
        uint16_t visitCount = 0U;
        uint16_t contradictionCount = 0U;
    };

    class WallBeliefMap
    {
    public:
        static constexpr size_t kMazeSize = 16U;
        static constexpr size_t kDirectionCount = 4U;

        void Reset() noexcept
        {
            for (auto& plane : _states)
            {
                for (auto& row : plane)
                {
                    for (auto& state : row)
                    {
                        state = WallBeliefState{};
                    }
                }
            }
        }

        const WallBeliefState& Get(const CellCoordinates& cell, Direction direction) const noexcept
        {
            return _states[cell.GetX()][cell.GetY()][DirectionIndex(direction)];
        }

        void SeedKnownState(
            const CellCoordinates& cell,
            Direction direction,
            WallState hardState,
            const WallBeliefConfig& config,
            uint32_t tick = 0UL) noexcept
        {
            if (!IsOrdinalDirection(direction) || hardState == WallState::Unknown)
            {
                return;
            }

            WallBeliefState state{};
            state.hardState = hardState;
            state.logOdds = (hardState == WallState::Wall) ?
                (std::max)(config.setThreshold, 0.0f) :
                (std::min)(config.clearThreshold, 0.0f);
            state.visitCount = 1U;
            state.lastUpdateTick = tick;
            SetMirroredState(cell, direction, state);
        }

        WallBeliefUpdate ApplyObservation(
            const CellCoordinates& cell,
            Direction direction,
            WallSampleClassification classification,
            const WallBeliefConfig& config,
            uint32_t tick = 0UL) noexcept
        {
            WallBeliefUpdate update{};
            if (!IsOrdinalDirection(direction) || classification == WallSampleClassification::Unknown)
            {
                return update;
            }

            WallBeliefState state = Get(cell, direction);
            const WallState previousHardState = state.hardState;
            const float previousLogOdds = state.logOdds;
            const float deltaLogOdds =
                (classification == WallSampleClassification::WallHit) ?
                config.hitLogOdds :
                ((state.hardState == WallState::Wall) ? -config.contradictoryMissLogOdds : -config.missLogOdds);

            if (!std::isfinite(deltaLogOdds))
            {
                return update;
            }

            const float saturation = std::fabs(config.saturationMagnitude);
            state.logOdds = (std::clamp)(
                state.logOdds + deltaLogOdds,
                -saturation,
                saturation);
            ++state.visitCount;
            state.lastUpdateTick = tick;

            if ((classification == WallSampleClassification::WallHit && previousHardState == WallState::NoWall) ||
                (classification == WallSampleClassification::WallMiss && previousHardState == WallState::Wall))
            {
                ++state.contradictionCount;
            }

            if (state.logOdds >= config.setThreshold)
            {
                state.hardState = WallState::Wall;
            }
            else if (state.logOdds <= config.clearThreshold)
            {
                state.hardState = WallState::NoWall;
            }
            else if (previousHardState == WallState::Unknown)
            {
                state.hardState = WallState::Unknown;
            }
            else
            {
                state.hardState = previousHardState;
            }

            SetMirroredState(cell, direction, state);

            update.valid = true;
            update.changed =
                (state.hardState != previousHardState) ||
                (std::fabs(state.logOdds - previousLogOdds) > 1.0e-6f);
            update.logOdds = state.logOdds;
            update.hardState = state.hardState;
            update.visitCount = state.visitCount;
            update.contradictionCount = state.contradictionCount;
            return update;
        }

    private:
        using DirectionStateRow = std::array<WallBeliefState, kDirectionCount>;
        using MazeRow = std::array<DirectionStateRow, kMazeSize>;
        using MazePlane = std::array<MazeRow, kMazeSize>;

        MazePlane _states{};

        static bool IsOrdinalDirection(Direction direction) noexcept
        {
            return
                direction == Direction::Up ||
                direction == Direction::Down ||
                direction == Direction::Left ||
                direction == Direction::Right;
        }

        static size_t DirectionIndex(Direction direction) noexcept
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
                return 3U;
            default:
                return 0U;
            }
        }

        void SetMirroredState(
            const CellCoordinates& cell,
            Direction direction,
            const WallBeliefState& state) noexcept
        {
            _states[cell.GetX()][cell.GetY()][DirectionIndex(direction)] = state;
            if (cell.IsValidMove(direction))
            {
                const CellCoordinates neighbor = cell >> direction;
                _states[neighbor.GetX()][neighbor.GetY()][DirectionIndex(-direction)] = state;
            }
        }
    };
}
