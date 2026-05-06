#include "pch.h"
#include "CppUnitTest.h"
#include "Templates.h"
#include "..\MazeMap\DirectionalPathFinder.h"
#include <cmath>
#include <sstream>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
	TEST_CLASS(DirectionalPathFinderTest)
	{
	private:
		using HalfPath = HalfStepPath<PATH_SIZE * 2>;

		Vehicle _vehicle = Vehicle();
		static constexpr float kTimeTolerance = 0.001f;

		static Maze CreateOpenMaze()
		{
			Maze maze;
			for (uint8_t x = 0; x < 16; ++x)
			{
				for (uint8_t y = 0; y < 16; ++y)
				{
					Cell& cell = maze(x, y);
					cell.SetUp(y == 15 ? WallState::Wall : WallState::NoWall);
					cell.SetDown(y == 0 ? WallState::Wall : WallState::NoWall);
					cell.SetLeft(x == 0 ? WallState::Wall : WallState::NoWall);
					cell.SetRight(x == 15 ? WallState::Wall : WallState::NoWall);
				}
			}
			maze.PreCalculate();
			return maze;
		}

		static Maze CreateSingleUnknownCorridorMaze()
		{
			Maze maze;
			maze.SetWall(CellCoordinates(0, 0), Direction::Up, WallState::NoWall);
			maze.SetWall(CellCoordinates(0, 0), Direction::Right, WallState::Wall);
			return maze;
		}

		static Maze CreateSymmetricUnknownChoiceMaze()
		{
			Maze maze;
			maze.SetWall(CellCoordinates(0, 0), Direction::Up, WallState::NoWall);
			maze.SetWall(CellCoordinates(0, 0), Direction::Right, WallState::NoWall);
			return maze;
		}

		static std::wstring LocationMessage(const MazeLocation& previous, const MazeLocation& current)
		{
			std::wstringstream stream;
			stream << L"Half-step move "
				<< Microsoft::VisualStudio::CppUnitTestFramework::ToString(previous)
				<< L" -> "
				<< Microsoft::VisualStudio::CppUnitTestFramework::ToString(current);
			return stream.str();
		}

		static void AssertHalfStepPathAccessible(const Maze& maze, const HalfPath& path)
		{
			Assert::IsTrue(path.GetSize() > 0, L"Expected a non-empty half-step path.");
			for (uint16_t i = 0; i < path.GetSize(); ++i)
			{
				Assert::IsTrue(maze.IsAccessibleLocation(path[i]), ToString(path[i]).c_str());
				if (i == 0)
				{
					continue;
				}

				const int dx = std::abs(static_cast<int>(path[i].GetX()) - static_cast<int>(path[i - 1].GetX()));
				const int dy = std::abs(static_cast<int>(path[i].GetY()) - static_cast<int>(path[i - 1].GetY()));
				const std::wstring message = LocationMessage(path[i - 1], path[i]);
				Assert::IsTrue(dx <= 1 && dy <= 1 && (dx + dy) > 0, message.c_str());
			}
		}

		static bool ContainsDiagonalStep(const HalfPath& path)
		{
			for (uint16_t i = 1; i < path.GetSize(); ++i)
			{
				const int dx = std::abs(static_cast<int>(path[i].GetX()) - static_cast<int>(path[i - 1].GetX()));
				const int dy = std::abs(static_cast<int>(path[i].GetY()) - static_cast<int>(path[i - 1].GetY()));
				if (dx == 1 && dy == 1)
				{
					return true;
				}
			}

			return false;
		}

		static void AssertPathEndsInCell(const HalfPath& path, CellCoordinates expectedCell)
		{
			Assert::IsTrue(path.GetSize() > 0, L"Expected a non-empty half-step path.");
			Assert::AreEqual(MazeLocation::CellCenter(expectedCell), path[path.GetSize() - 1]);
		}

	public:
		TEST_METHOD(HalfStepPathFromTo_OpenMazeProducesAccessibleContiguousPath)
		{
			Maze maze = CreateOpenMaze();
			DirectionalPathFinder finder(maze, _vehicle);
			HalfPath path;

			const CellCoordinates start(0, 0);
			const CellCoordinates end(10, 10);
			finder.HalfStepPathFromTo(start, Direction::Up, end, path);

			Assert::IsTrue(path.GetSize() > 0, L"Expected a non-empty point-to-point path.");
			Assert::AreEqual(MazeLocation::CellCenter(start), path.first());
			Assert::AreEqual(MazeLocation::CellCenter(end), path.last());
			AssertHalfStepPathAccessible(maze, path);
			Assert::IsTrue(ContainsDiagonalStep(path), L"Expected the open-maze route to use diagonal half-steps.");
			Assert::IsTrue(std::isfinite(finder.GetLastEstimatedTime()), L"Expected the estimated travel time to be finite.");
			Assert::IsTrue(finder.GetLastEstimatedTime() > 0.0f, L"Expected a positive travel-time estimate for a non-zero route.");
		}

		TEST_METHOD(HalfStepPathFromTo_SameStartAndEndClearsEstimatedTime)
		{
			Maze maze = CreateOpenMaze();
			DirectionalPathFinder finder(maze, _vehicle);
			HalfPath path;

			finder.HalfStepPathFromTo(CellCoordinates(0, 0), Direction::Up, CellCoordinates(6, 0), path);
			Assert::IsTrue(finder.GetLastEstimatedTime() > 0.0f, L"Expected a positive estimate after the warm-up route.");

			finder.HalfStepPathFromTo(CellCoordinates(0, 0), Direction::Up, CellCoordinates(0, 0), path);

			Assert::AreEqual(1, static_cast<int>(path.GetSize()));
			Assert::AreEqual(MazeLocation::CellCenter(CellCoordinates(0, 0)), path.first());
			Assert::AreEqual(0.0f, finder.GetLastEstimatedTime(), kTimeTolerance);
		}

		TEST_METHOD(HalfStepPathToNearestUnknown_ReturnsReachableUnknownFrontierCell)
		{
			Maze maze = CreateSingleUnknownCorridorMaze();
			DirectionalPathFinder finder(maze, _vehicle);
			HalfPath path;

			const CellCoordinates start(0, 0);
			const CellCoordinates expectedFrontier(0, 1);
			Assert::IsFalse(maze[expectedFrontier].IsFullyKnown(), L"Test setup must leave the frontier cell partially unknown.");

			finder.HalfStepPathToNearestUnknown(start, Direction::Up, path);

			Assert::IsTrue(path.GetSize() > 0, L"Expected a nearest-unknown path.");
			Assert::AreEqual(MazeLocation::CellCenter(start), path.first());
			AssertHalfStepPathAccessible(maze, path);
			AssertPathEndsInCell(path, expectedFrontier);
		}

		TEST_METHOD(HalfStepPathToNearestUnknown_PrefersCurrentHeadingForSymmetricFrontiers)
		{
			Maze maze = CreateSymmetricUnknownChoiceMaze();
			DirectionalPathFinder upwardFinder(maze, _vehicle);
			DirectionalPathFinder rightwardFinder(maze, _vehicle);
			HalfPath upwardPath;
			HalfPath rightwardPath;

			upwardFinder.HalfStepPathToNearestUnknown(CellCoordinates(0, 0), Direction::Up, upwardPath);
			rightwardFinder.HalfStepPathToNearestUnknown(CellCoordinates(0, 0), Direction::Right, rightwardPath);

			AssertPathEndsInCell(upwardPath, CellCoordinates(0, 1));
			AssertPathEndsInCell(rightwardPath, CellCoordinates(1, 0));
		}
	};
}

