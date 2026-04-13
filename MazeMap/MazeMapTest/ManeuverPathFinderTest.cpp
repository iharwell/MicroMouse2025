#include "pch.h"
#include "CppUnitTest.h"
#include "MazeRef.h"
#include "Templates.h"
#include "..\MazeMap\ManeuverPathFinder.h"
#include "..\MazeMap\ManeuverPath.h"
#include "..\MazeMap\ManeuverSet.h"
#include "..\MazeMap\PathFinder.h"
#include <cmath>
#include <sstream>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
	TEST_CLASS(ManeuverPathFinderTest)
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

		static Maze CreateGoalReturnCorridorMaze()
		{
			Maze maze;
			for (uint8_t x = 0; x < 16; ++x)
			{
				for (uint8_t y = 0; y < 16; ++y)
				{
					Cell& cell = maze(x, y);
					cell.SetUp(WallState::Wall);
					cell.SetDown(WallState::Wall);
					cell.SetLeft(WallState::Wall);
					cell.SetRight(WallState::Wall);
				}
			}

			const struct Passage
			{
				uint8_t x;
				uint8_t y;
				Direction direction;
			} passages[] = {
				{ 0, 0, Direction::Up },
				{ 0, 1, Direction::Up },
				{ 0, 2, Direction::Up },
				{ 0, 3, Direction::Up },
				{ 0, 4, Direction::Right },
				{ 1, 4, Direction::Down },
				{ 1, 3, Direction::Down },
				{ 1, 2, Direction::Down },
				{ 1, 1, Direction::Down },
				{ 1, 0, Direction::Right },
				{ 1, 1, Direction::Right },
				{ 2, 0, Direction::Up },
			};

			for (const Passage& passage : passages)
			{
				maze.SetWall(maze(passage.x, passage.y), passage.direction, WallState::NoWall);
			}

			maze.PreCalculate();
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

		static bool IsGoalCell(const Maze& maze, const MazeLocation& location)
		{
			CellCoordinates goal = maze.GetGoalLowerLeft();
			CellCoordinates actual = static_cast<CellCoordinates>(location);
			int dx = static_cast<int>(actual.GetX()) - static_cast<int>(goal.GetX());
			int dy = static_cast<int>(actual.GetY()) - static_cast<int>(goal.GetY());
			return dx >= 0 && dx <= 1 && dy >= 0 && dy <= 1;
		}

		static bool ContainsDiagonalStep(const HalfPath& path)
		{
			for (uint16_t i = 1; i < path.GetSize(); ++i)
			{
				int dx = static_cast<int>(path[i].GetX()) - static_cast<int>(path[i - 1].GetX());
				int dy = static_cast<int>(path[i].GetY()) - static_cast<int>(path[i - 1].GetY());
				if (std::abs(dx) == 1 && std::abs(dy) == 1)
				{
					return true;
				}
			}
			return false;
		}

		static bool ContainsSmoothTurn(const ManeuverPath& path)
		{
			for (uint16_t i = 0; i < path.GetSize(); ++i)
			{
				const ManeuverCode baseCode = static_cast<ManeuverCode>(path[i] & INVERTED_MIRRORED_MANEUVER_FLAG);
				if (baseCode > S31 && baseCode != IP45 && baseCode != IP90 && baseCode != IP135 && baseCode != IP180)
				{
					return true;
				}
			}
			return false;
		}

		static bool ContainsInPlaceTurn(const ManeuverPath& path)
		{
			for (uint16_t i = 0; i < path.GetSize(); ++i)
			{
				const ManeuverCode baseCode = static_cast<ManeuverCode>(path[i] & INVERTED_MIRRORED_MANEUVER_FLAG);
				if (baseCode == IP45 || baseCode == IP90 || baseCode == IP135 || baseCode == IP180)
				{
					return true;
				}
			}
			return false;
		}

		static void AssertHalfStepPathAccessible(const Maze& maze, const HalfPath& path)
		{
			Assert::IsTrue(path.GetSize() > 0, L"Expected a non-empty half-step path.");
			for (uint16_t i = 0; i < path.GetSize(); ++i)
			{
				Assert::IsTrue(maze.IsAccessibleLocation(path[i]), Microsoft::VisualStudio::CppUnitTestFramework::ToString(path[i]).c_str());
				if (i == 0)
				{
					continue;
				}

				int dx = std::abs(static_cast<int>(path[i].GetX()) - static_cast<int>(path[i - 1].GetX()));
				int dy = std::abs(static_cast<int>(path[i].GetY()) - static_cast<int>(path[i - 1].GetY()));
				std::wstring message = LocationMessage(path[i - 1], path[i]);
				Assert::IsTrue(dx <= 1 && dy <= 1 && (dx + dy) > 0, message.c_str());
			}
		}

		static void AssertHalfStepPathsEqual(const HalfPath& expected, const HalfPath& actual)
		{
			Assert::AreEqual(static_cast<int>(expected.GetSize()), static_cast<int>(actual.GetSize()), L"Half-step path sizes differ.");
			for (uint16_t i = 0; i < expected.GetSize(); ++i)
			{
				Assert::AreEqual(expected[i], actual[i]);
			}
		}

		static void AssertManeuverPathExecutable(const Maze& maze, DirectionalLocation start, const ManeuverPath& path)
		{
			const ManeuverSet& maneuvers = ManeuverSet::GetSet();
			DirectionalLocation current = start;

			for (uint16_t i = 0; i < path.GetSize(); ++i)
			{
				ManeuverCode code = path[i];
				std::wstring message = CodeString(code);

				if (code <= ManeuverCode::S31)
				{
					for (uint8_t step = 0; step < static_cast<uint8_t>(code); ++step)
					{
						current = current.MoveForward(1);
						Assert::IsTrue(maze.IsAccessibleLocation(current.GetLocation()), message.c_str());
					}
					continue;
				}

				bool mirrored = static_cast<uint8_t>(code & ManeuverCode::MIRRORED_MANEUVER_FLAG) != 0;
				Assert::IsTrue(maneuvers[code].IsValidMove(current, maze, mirrored), message.c_str());
				current = maneuvers[code].Move(current, mirrored);
				Assert::IsTrue(maze.IsAccessibleLocation(current.GetLocation()), message.c_str());
			}

			Assert::AreEqual(path.ExecutePath(start), current);
		}

		static HalfPath ExpandPath(DirectionalLocation start, ManeuverPath& path)
		{
			HalfPath expanded;
			path.ToHalfStepPath(start, expanded);
			return expanded;
		}

		static void AssertSingleTurnReferencePath(const HalfPath& path, const Maze& maze, uint16_t maxExtraSteps)
		{
			HalfPath expected;
			expected.push_back(MazeLocation(1, 1));
			expected.push_back(MazeLocation(1, 2));
			expected.push_back(MazeLocation(1, 3));
			expected.push_back(MazeLocation(1, 4));
			expected.push_back(MazeLocation(1, 5));
			expected.push_back(MazeLocation(1, 6));
			expected.push_back(MazeLocation(1, 7));
			expected.push_back(MazeLocation(1, 8));
			expected.push_back(MazeLocation(1, 9));
			expected.push_back(MazeLocation(1, 10));
			expected.push_back(MazeLocation(1, 11));
			expected.push_back(MazeLocation(1, 12));
			expected.push_back(MazeLocation(1, 13));
			expected.push_back(MazeLocation(1, 14));
			expected.push_back(MazeLocation(1, 15));
			expected.push_back(MazeLocation(1, 16));
			expected.push_back(MazeLocation(1, 17));
			expected.push_back(MazeLocation(1, 18));
			expected.push_back(MazeLocation(1, 19));
			expected.push_back(MazeLocation(1, 20));
			expected.push_back(MazeLocation(1, 21));
			expected.push_back(MazeLocation(1, 22));
			expected.push_back(MazeLocation(1, 23));
			expected.push_back(MazeLocation(1, 24));
			expected.push_back(MazeLocation(1, 25));
			expected.push_back(MazeLocation(1, 26));
			expected.push_back(MazeLocation(1, 27));
			expected.push_back(MazeLocation(1, 28));
			expected.push_back(MazeLocation(1, 29));
			expected.push_back(MazeLocation(1, 30));
			expected.push_back(MazeLocation(2, 31));
			Assert::IsTrue(path.GetSize() >= expected.GetSize(), L"Expected the single-turn path to at least reach the reference exit point.");
			for (uint16_t i = 0; i < expected.GetSize(); ++i)
			{
				Assert::AreEqual(expected[i], path[i]);
			}
			for (uint16_t i = expected.GetSize(); i < path.GetSize(); ++i)
			{
				Assert::IsTrue(IsGoalCell(maze, path[i]), L"Expected any extra single-turn steps to stay inside the goal cell.");
			}
		}

	public:
		TEST_METHOD(SingleTurnGoalPathMatchesReferenceAndExpansion)
		{
			Mazes::SetupMazes();
			Maze& maze = Mazes::GetSingleTurnMaze();
			ManeuverPathFinder finder(maze, _vehicle);
			DirectionalLocation start(MazeLocation::CellCenter(CellCoordinates(0, 0)), Direction::Up);

			ManeuverPath maneuverPath;
			finder.ManeuverPathToGoal(CellCoordinates(0, 0), Direction::Up, maneuverPath);

			HalfPath fromManeuverPath = ExpandPath(start, maneuverPath);
			HalfPath directHalfPath;
			finder.HalfStepPathToGoal(CellCoordinates(0, 0), Direction::Up, directHalfPath);

			AssertSingleTurnReferencePath(fromManeuverPath, maze, 2);
			AssertSingleTurnReferencePath(directHalfPath, maze, 0);
			AssertHalfStepPathAccessible(maze, directHalfPath);
			AssertManeuverPathExecutable(maze, start, maneuverPath);
			Assert::AreEqual(
				maneuverPath.Cost(_vehicle, maze.GetCellDimension()),
				finder.GetLastEstimatedTime(),
				kTimeTolerance);
			Assert::IsTrue(IsGoalCell(maze, maneuverPath.ExecutePath(start).GetLocation()));
		}

		TEST_METHOD(APEC2016GoalPathIsExecutableReversibleAndDiagonal)
		{
			Mazes::SetupMazes();
			Maze& maze = Mazes::GetMazeAPEC2016();
			ManeuverPathFinder finder(maze, _vehicle);
			DirectionalLocation start(MazeLocation::CellCenter(CellCoordinates(0, 0)), Direction::Up);

			ManeuverPath path;
			finder.ManeuverPathToGoal(CellCoordinates(0, 0), Direction::Up, path);
			Assert::IsTrue(path.GetSize() > 0, L"Expected a non-empty maneuver path.");

			HalfPath halfPath = ExpandPath(start, path);
			AssertHalfStepPathAccessible(maze, halfPath);
			AssertManeuverPathExecutable(maze, start, path);
			Assert::IsTrue(ContainsDiagonalStep(halfPath), L"Expected diagonal traversal on the APEC 2016 maze.");

			DirectionalLocation end = path.ExecutePath(start);
			Assert::IsTrue(IsGoalCell(maze, end.GetLocation()));
			Assert::AreEqual(start, path.ExecuteReverse(end));
		}

		TEST_METHOD(OpenMazeFromToUsesDiagonalTraversalWhenBeneficial)
		{
			Maze maze = CreateOpenMaze();
			ManeuverPathFinder finder(maze, _vehicle);
			DirectionalLocation start(MazeLocation::CellCenter(CellCoordinates(0, 0)), Direction::Up);
			CellCoordinates target(10, 10);

			ManeuverPath path;
			finder.ManeuverPathFromTo(CellCoordinates(0, 0), Direction::Up, target, path);
			Assert::IsTrue(path.GetSize() > 0, L"Expected a non-empty maneuver path.");

			HalfPath halfPath = ExpandPath(start, path);
			AssertHalfStepPathAccessible(maze, halfPath);
			AssertManeuverPathExecutable(maze, start, path);
			Assert::IsTrue(ContainsDiagonalStep(halfPath), L"Expected the open-maze route to use diagonal travel.");

			DirectionalLocation end = path.ExecutePath(start);
			Assert::AreEqual(MazeLocation::CellCenter(target), end.GetLocation());
			Assert::AreEqual(start, path.ExecuteReverse(end));
		}

		TEST_METHOD(ReturnCorridorFromGoalConvertsFloodFillHalfStepPathToSmoothManeuvers)
		{
			Maze maze = CreateGoalReturnCorridorMaze();
			FloodFillPathFinder finder(maze, _vehicle);
			DirectionalLocation start(MazeLocation::CellCenter(CellCoordinates(2, 1)), Direction::Left);
			Path<PATH_SIZE> cellPath;
			finder.PathFromTo(CellCoordinates(2, 1), Direction::Left, CellCoordinates(0, 0), cellPath);
			Assert::IsTrue(cellPath.GetSize() > 1, L"Expected a non-empty flood-fill return path.");

			HalfPath halfPath;
			HalfPath::HalfStepPathFromPath(cellPath, halfPath);
			Assert::IsTrue(halfPath.GetSize() > 1, L"Expected a non-empty half-step return path.");

			ManeuverPath path;
			Assert::IsTrue(ManeuverPath::FromHalfStep(halfPath, start, path), L"Expected a maneuver path built from the directional return path.");

			HalfPath expandedPath = ExpandPath(start, path);
			AssertHalfStepPathAccessible(maze, halfPath);
			AssertHalfStepPathsEqual(halfPath, expandedPath);
			AssertManeuverPathExecutable(maze, start, path);
			Assert::IsTrue(ContainsSmoothTurn(path), L"Expected the return path to use at least one smooth turn.");
			Assert::IsFalse(ContainsInPlaceTurn(path), L"Expected the return path to avoid in-place turns.");

			const DirectionalLocation end = path.ExecutePath(start);
			Assert::AreEqual(MazeLocation::CellCenter(CellCoordinates(0, 0)), end.GetLocation());
			Assert::AreEqual(start, path.ExecuteReverse(end));
		}

		TEST_METHOD(GetCodeAndCostTrackTheGradientAlongGoalPath)
		{
			Mazes::SetupMazes();
			Maze& maze = Mazes::GetMaze2();
			ManeuverPathFinder finder(maze, _vehicle);
			DirectionalLocation current(MazeLocation::CellCenter(CellCoordinates(0, 0)), Direction::Up);

			ManeuverPath path;
			finder.ManeuverPathToGoal(CellCoordinates(0, 0), Direction::Up, path);
			Assert::IsTrue(path.GetSize() > 0, L"Expected a non-empty maneuver path.");

			float priorCost = finder.GetLastEstimatedTime();
			float startScore = finder.GetCost(DirectionalLocation(current.GetLocation(), -current.GetDirection()));
			Assert::IsTrue(std::isfinite(startScore), L"Expected the start-state score to be finite.");
			Assert::IsTrue(startScore >= 0.0f, L"Expected the start-state score to be non-negative.");
			priorCost = startScore;

			const ManeuverSet& maneuvers = ManeuverSet::GetSet();
			for (uint16_t i = 0; i < path.GetSize(); ++i)
			{
				DirectionalLocation scoreState(current.GetLocation(), -current.GetDirection());
                Assert::IsTrue(ManeuverCode::MC_NONE != finder.GetCode(scoreState), L"Expected an intermediate score state to retain a maneuver code.");
				float stateCost = finder.GetCost(scoreState);
				Assert::IsTrue(std::isfinite(stateCost), L"Expected the score to remain finite along the chosen path.");
				Assert::IsTrue(stateCost <= priorCost + kTimeTolerance, L"Expected score values to descend along the path.");

				current = maneuvers.Move(path[i], current);
				priorCost = stateCost;
			}

			DirectionalLocation terminalScoreState(current.GetLocation(), -current.GetDirection());
			Assert::IsTrue(ManeuverCode::MC_NONE == finder.GetCode(terminalScoreState));
			Assert::AreEqual(0.0f, finder.GetCost(terminalScoreState), kTimeTolerance);
			Assert::IsTrue(IsGoalCell(maze, current.GetLocation()));
		}
	};
}


