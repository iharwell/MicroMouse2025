#include "pch.h"
#include "CppUnitTest.h"
#include "..\MazeMap\SearchRunPlanner.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    TEST_CLASS(SearchRunPlannerTest)
    {
    private:
        static void SetFullyKnownCell(Maze& maze, uint8_t x, uint8_t y)
        {
            maze[CellCoordinates(x, y)] = Cell(x, y, WallState::Wall, WallState::Wall, WallState::Wall, WallState::Wall);
        }

        static void SetPartiallyKnownCell(Maze& maze, uint8_t x, uint8_t y)
        {
            maze[CellCoordinates(x, y)] = Cell(x, y, WallState::Wall, WallState::Wall, WallState::Wall, WallState::Unknown);
        }

    public:
        TEST_METHOD(PlanSearchStraightSegment_SplitsFinalUnmappedCell)
        {
            Maze maze;
            SetFullyKnownCell(maze, 5, 6);
            SetFullyKnownCell(maze, 5, 7);
            SetPartiallyKnownCell(maze, 5, 8);

            Path<PATH_SIZE> path;
            path.push_back(CellCoordinates(5, 5));
            path.push_back(CellCoordinates(5, 6));
            path.push_back(CellCoordinates(5, 7));
            path.push_back(CellCoordinates(5, 8));

            const SearchStraightPlan plan = PlanSearchStraightSegment(maze, path, 1U);

            Assert::IsTrue(plan.direction == Direction::Up);
            Assert::AreEqual(static_cast<uint16_t>(3U), plan.TotalCellCount());
            Assert::AreEqual(static_cast<uint16_t>(2U), plan.fullSpeedCellCount);
            Assert::AreEqual(static_cast<uint16_t>(1U), plan.cautiousCellCount);
            Assert::AreEqual(static_cast<uint16_t>(3U), plan.segmentEndIndex);
        }

        TEST_METHOD(PlanSearchStraightSegment_KeepsMappedStraightAtFullSpeed)
        {
            Maze maze;
            SetFullyKnownCell(maze, 6, 5);
            SetFullyKnownCell(maze, 7, 5);

            Path<PATH_SIZE> path;
            path.push_back(CellCoordinates(5, 5));
            path.push_back(CellCoordinates(6, 5));
            path.push_back(CellCoordinates(7, 5));

            const SearchStraightPlan plan = PlanSearchStraightSegment(maze, path, 1U);

            Assert::IsTrue(plan.direction == Direction::Right);
            Assert::AreEqual(static_cast<uint16_t>(2U), plan.TotalCellCount());
            Assert::AreEqual(static_cast<uint16_t>(2U), plan.fullSpeedCellCount);
            Assert::AreEqual(static_cast<uint16_t>(0U), plan.cautiousCellCount);
            Assert::AreEqual(static_cast<uint16_t>(2U), plan.segmentEndIndex);
        }

        TEST_METHOD(PlanSearchStraightSegment_ImmediateUnmappedCellUsesCautiousApproach)
        {
            Maze maze;
            SetPartiallyKnownCell(maze, 4, 5);

            Path<PATH_SIZE> path;
            path.push_back(CellCoordinates(4, 4));
            path.push_back(CellCoordinates(4, 5));

            const SearchStraightPlan plan = PlanSearchStraightSegment(maze, path, 1U);

            Assert::IsTrue(plan.direction == Direction::Up);
            Assert::AreEqual(static_cast<uint16_t>(1U), plan.TotalCellCount());
            Assert::AreEqual(static_cast<uint16_t>(0U), plan.fullSpeedCellCount);
            Assert::AreEqual(static_cast<uint16_t>(1U), plan.cautiousCellCount);
        }

        TEST_METHOD(ComputeSafeUnmappedCruiseSpeed_UsesDetectionAndStopMargins)
        {
            const float safeSpeedMps = ComputeSafeUnmappedCruiseSpeed(0.30f, 0.085f, 0.03f, 0.034f, 0.003f);
            Assert::AreEqual(0.21633308f, safeSpeedMps, 0.0001f);
        }

        TEST_METHOD(ComputeSafeUnmappedCruiseSpeed_ClampsInvalidInputsToZero)
        {
            Assert::AreEqual(0.0f, ComputeSafeUnmappedCruiseSpeed(0.0f, 0.085f, 0.03f, 0.034f, 0.003f), 0.0f);
            Assert::AreEqual(0.0f, ComputeSafeUnmappedCruiseSpeed(0.30f, 0.010f, 0.0f, 0.020f, 0.0f), 0.0f);
        }
    };
}
