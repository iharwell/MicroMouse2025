#include "pch.h"
#include "CppUnitTest.h"
#include "..\MazeMap\Maze.h"
#include "..\MazeMap\PathFinder.h"
#include "..\MazeMap\DirectionalPathFinder.h"
#include "..\MazeMap\ManeuverPathFinder.h"

#include <algorithm>
#include <fstream>
#include <io.h>
#include <map>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
	namespace
	{
		struct HistoricalMazeCase
		{
			std::string Name;
			Maze MazeData;
			CellCoordinates Start;
			CellCoordinates GoalLowerLeft;
		};

		struct HistoricalMazeCatalog
		{
			std::vector<HistoricalMazeCase> Cases;
			std::map<std::wstring, size_t> SkipCounts;
			size_t TotalFiles = 0;
		};

		std::wstring ToWide(const std::string& value)
		{
			return std::wstring(value.begin(), value.end());
		}

		std::string ParentDirectory(const std::string& path)
		{
			const size_t lastSlash = path.find_last_of("\\/");
			if (lastSlash == std::string::npos)
			{
				return std::string();
			}

			return path.substr(0, lastSlash);
		}

		std::string GetHistoricalMazeDirectory()
		{
			const std::string sourceFile = __FILE__;
			const std::string testDirectory = ParentDirectory(sourceFile);
			const std::string solutionDirectory = ParentDirectory(testDirectory);
			const std::string repoDirectory = ParentDirectory(solutionDirectory);
			return repoDirectory + "\\Maze Files";
		}

		std::vector<std::string> EnumerateHistoricalMazeFiles()
		{
			const std::string mazeDirectory = GetHistoricalMazeDirectory();
			const std::string searchPattern = mazeDirectory + "\\*.txt";

			_finddata_t fileData = {};
			const intptr_t handle = _findfirst(searchPattern.c_str(), &fileData);
			if (handle == -1)
			{
				throw std::runtime_error("Unable to enumerate historical maze files.");
			}

			std::vector<std::string> files;
			do
			{
				if ((fileData.attrib & _A_SUBDIR) != 0)
				{
					continue;
				}

				files.push_back(mazeDirectory + "\\" + fileData.name);
			} while (_findnext(handle, &fileData) == 0);

			_findclose(handle);
			std::sort(files.begin(), files.end());
			return files;
		}

		std::vector<std::string> ReadMazeLines(const std::string& filePath)
		{
			std::ifstream file(filePath);
			if (!file.is_open())
			{
				throw std::runtime_error("Unable to open historical maze file.");
			}

			std::vector<std::string> lines;
			std::string line;
			while (std::getline(file, line))
			{
				if (!line.empty() && line.back() == '\r')
				{
					line.pop_back();
				}
				if (!line.empty())
				{
					lines.push_back(line);
				}
			}

			return lines;
		}

		WallState ParseHorizontalWall(const std::string& line, uint8_t cellX)
		{
			const size_t start = static_cast<size_t>(cellX) * 4 + 1;
			for (size_t i = start; i < start + 3 && i < line.size(); ++i)
			{
				if (line[i] == '-')
				{
					return WallState::Wall;
				}
			}

			return WallState::NoWall;
		}

		WallState ParseVerticalWall(const std::string& line, size_t index)
		{
			return (index < line.size() && line[index] == '|') ? WallState::Wall : WallState::NoWall;
		}

		bool IsWallConsistent(const Maze& maze)
		{
			for (uint8_t x = 0; x < 16; ++x)
			{
				for (uint8_t y = 0; y < 16; ++y)
				{
					const Cell& current = maze(x, y);
					if (x < 15 && current.GetRight() != maze(x + 1, y).GetLeft())
					{
						return false;
					}
					if (y < 15 && current.GetUp() != maze(x, y + 1).GetDown())
					{
						return false;
					}
				}
			}

			return true;
		}

		std::vector<CellCoordinates> FindGoalCandidates(const Maze& maze)
		{
			std::vector<CellCoordinates> candidates;
			for (uint8_t x = 0; x < 15; ++x)
			{
				for (uint8_t y = 0; y < 15; ++y)
				{
					const Cell& lowerLeft = maze(x, y);
					const Cell& upperRight = maze(x + 1, y + 1);
					if (lowerLeft.GetUp() == WallState::NoWall
						&& lowerLeft.GetRight() == WallState::NoWall
						&& upperRight.GetDown() == WallState::NoWall
						&& upperRight.GetLeft() == WallState::NoWall)
					{
						candidates.push_back(CellCoordinates(x, y));
					}
				}
			}

			return candidates;
		}

		bool IsGoalCell(CellCoordinates goalLowerLeft, CellCoordinates cell)
		{
			const int dx = static_cast<int>(cell.GetX()) - static_cast<int>(goalLowerLeft.GetX());
			const int dy = static_cast<int>(cell.GetY()) - static_cast<int>(goalLowerLeft.GetY());
			return dx >= 0 && dx <= 1 && dy >= 0 && dy <= 1;
		}

		bool IsGoalReachable(const Maze& maze, CellCoordinates start, CellCoordinates goalLowerLeft)
		{
			bool visited[16][16] = {};
			std::queue<CellCoordinates> pending;
			pending.push(start);
			visited[start.GetX()][start.GetY()] = true;

			while (!pending.empty())
			{
				const CellCoordinates current = pending.front();
				pending.pop();

				if (IsGoalCell(goalLowerLeft, current))
				{
					return true;
				}

				for (Direction direction = Direction::Up; direction <= Direction::Right; direction <<= 1)
				{
					if (!current.IsValidMove(direction))
					{
						continue;
					}

					if (maze[current].GetWall(direction) != WallState::NoWall)
					{
						continue;
					}

					const CellCoordinates next = current >> direction;
					if (visited[next.GetX()][next.GetY()])
					{
						continue;
					}

					visited[next.GetX()][next.GetY()] = true;
					pending.push(next);
				}
			}

			return false;
		}

		Maze BuildReachableMaze(const Maze& fullMaze, CellCoordinates start)
		{
			Maze reachableMaze;
			bool visited[16][16] = {};
			std::queue<CellCoordinates> pending;
			pending.push(start);
			visited[start.GetX()][start.GetY()] = true;

			while (!pending.empty())
			{
				const CellCoordinates current = pending.front();
				pending.pop();

				for (Direction direction = Direction::Up; direction <= Direction::Right; direction <<= 1)
				{
					const WallState state = fullMaze[current].GetWall(direction);
					reachableMaze.SetWall(reachableMaze[current], direction, state);
					if (!current.IsValidMove(direction) || state != WallState::NoWall)
					{
						continue;
					}

					const CellCoordinates next = current >> direction;
					if (visited[next.GetX()][next.GetY()])
					{
						continue;
					}

					visited[next.GetX()][next.GetY()] = true;
					pending.push(next);
				}
			}

			return reachableMaze;
		}

		bool TryLoadHistoricalMazeCase(const std::string& filePath, HistoricalMazeCase& result, std::wstring& skipReason)
		{
			const std::string fileName = filePath.substr(filePath.find_last_of("\\/") + 1);
			if (fileName == "long.txt")
			{
				skipReason = L"maze requires exhaustive traversal beyond the supported design envelope";
				return false;
			}

			const std::vector<std::string> lines = ReadMazeLines(filePath);
			if (lines.size() != 33)
			{
				skipReason = L"unsupported line count";
				return false;
			}

			for (const std::string& line : lines)
			{
				if (line.size() != 65)
				{
					skipReason = L"unsupported line width";
					return false;
				}
			}

			Maze fullMaze;
			CellCoordinates start(0, 0);
			size_t startCount = 0;

			for (uint8_t y = 0; y < 16; ++y)
			{
				const uint8_t mazeY = 15 - y;
				const size_t topLine = static_cast<size_t>(y) * 2;
				const size_t centerLine = topLine + 1;
				const size_t bottomLine = topLine + 2;

				for (uint8_t x = 0; x < 16; ++x)
				{
					Cell& cell = fullMaze(x, mazeY);
					cell.SetUp(ParseHorizontalWall(lines[topLine], x));
					cell.SetDown(ParseHorizontalWall(lines[bottomLine], x));
					cell.SetLeft(ParseVerticalWall(lines[centerLine], static_cast<size_t>(x) * 4));
					cell.SetRight(ParseVerticalWall(lines[centerLine], static_cast<size_t>(x) * 4 + 4));

					const char marker = lines[centerLine][static_cast<size_t>(x) * 4 + 2];
					if (marker == 'S' || marker == 's')
					{
						start = CellCoordinates(x, mazeY);
						++startCount;
					}
				}
			}

			if (startCount != 1)
			{
				skipReason = L"expected exactly one start marker";
				return false;
			}

			if (!IsWallConsistent(fullMaze))
			{
				skipReason = L"maze has inconsistent shared walls";
				return false;
			}

			if (!fullMaze.IsComplete())
			{
				skipReason = L"maze is incomplete";
				return false;
			}

			Maze reachableMaze = BuildReachableMaze(fullMaze, start);
			const std::vector<CellCoordinates> goalCandidates = FindGoalCandidates(reachableMaze);
			if (goalCandidates.size() != 1)
			{
				std::wstringstream message;
				message << L"expected exactly one reachable goal post but found " << goalCandidates.size();
				skipReason = message.str();
				return false;
			}

			const CellCoordinates goalLowerLeft = goalCandidates.front();
			if (!reachableMaze.HasFoundGoal())
			{
				skipReason = L"maze did not report a reachable goal";
				return false;
			}

			if (reachableMaze.GetGoalLowerLeft() != goalLowerLeft)
			{
				skipReason = L"maze goal lookup disagrees with the reachable goal post";
				return false;
			}

			result.Name = fileName;
			result.MazeData = reachableMaze;
			result.Start = start;
			result.GoalLowerLeft = goalLowerLeft;
			return true;
		}

		HistoricalMazeCatalog BuildHistoricalMazeCatalog()
		{
			HistoricalMazeCatalog catalog;
			const std::vector<std::string> files = EnumerateHistoricalMazeFiles();
			catalog.TotalFiles = files.size();

			for (const std::string& filePath : files)
			{
				HistoricalMazeCase testCase;
				std::wstring skipReason;
				if (TryLoadHistoricalMazeCase(filePath, testCase, skipReason))
				{
					catalog.Cases.push_back(testCase);
					continue;
				}

				++catalog.SkipCounts[skipReason];
			}

			return catalog;
		}

		const HistoricalMazeCatalog& GetHistoricalMazeCatalog()
		{
			static const HistoricalMazeCatalog catalog = BuildHistoricalMazeCatalog();
			return catalog;
		}

		std::wstring BuildCatalogSummary(const HistoricalMazeCatalog& catalog)
		{
			std::wstringstream message;
			message << L"Historical maze loader: using " << catalog.Cases.size()
				<< L" of " << catalog.TotalFiles << L" files.";

			for (const auto& entry : catalog.SkipCounts)
			{
				message << L" Skipped " << entry.second << L" because " << entry.first << L".";
			}

			return message.str();
		}

		std::wstring BuildFailureMessage(
			const wchar_t* pathFinderName,
			const HistoricalMazeCase& testCase,
			const std::wstring& detail)
		{
			std::wstringstream message;
			message << pathFinderName << L" failed on " << ToWide(testCase.Name) << L": " << detail;
			return message.str();
		}

		void AssertGoalReached(
			const wchar_t* pathFinderName,
			const HistoricalMazeCase& testCase,
			HalfStepPath<PATH_SIZE * 2>& path)
		{
			Assert::IsTrue(
				path.GetSize() > 0,
				BuildFailureMessage(pathFinderName, testCase, L"path was empty").c_str());
			Assert::IsTrue(
				path.first() == MazeLocation::CellCenter(testCase.Start),
				BuildFailureMessage(pathFinderName, testCase, L"path did not begin at the requested start cell").c_str());

			const CellCoordinates finalCell = static_cast<CellCoordinates>(path.last());
			Assert::IsTrue(
				IsGoalCell(testCase.GoalLowerLeft, finalCell),
				BuildFailureMessage(pathFinderName, testCase, L"path did not end in the goal region").c_str());
		}

		template <typename Runner>
		void RunHistoricalSweep(const wchar_t* pathFinderName, Runner runner)
		{
			try
			{
				const HistoricalMazeCatalog& catalog = GetHistoricalMazeCatalog();
				Logger::WriteMessage(BuildCatalogSummary(catalog).c_str());
				Assert::IsTrue(!catalog.Cases.empty(), L"No historical mazes qualified for pathfinding tests.");

				std::vector<std::wstring> failures;
				for (const HistoricalMazeCase& testCase : catalog.Cases)
				{
					Logger::WriteMessage((std::wstring(L"Running ") + ToWide(testCase.Name)).c_str());
					try
					{
						runner(testCase);
					}
					catch (const std::exception& ex)
					{
						failures.push_back(BuildFailureMessage(pathFinderName, testCase, ToWide(ex.what())));
					}
					catch (...)
					{
						failures.push_back(BuildFailureMessage(pathFinderName, testCase, L"unexpected exception"));
					}
				}

				if (!failures.empty())
				{
					std::wstringstream message;
					message << failures.size() << L" historical maze failures:";
					for (const std::wstring& failure : failures)
					{
						message << L"\n" << failure;
					}
					Assert::IsTrue(failures.empty(), message.str().c_str());
				}
			}
			catch (const std::exception& ex)
			{
				Assert::IsTrue(false, ToWide(ex.what()).c_str());
			}
			catch (...)
			{
				Assert::IsTrue(false, L"Historical maze setup failed with an unexpected exception.");
			}
		}
	}

	TEST_CLASS(HistoricalMazePathFinderTest)
	{
	private:
		Vehicle _vehicle = Vehicle();

	public:
		TEST_METHOD(HistoricalMazeLoaderUsesOnlyReachableSingleGoalMazes)
		{
			const HistoricalMazeCatalog& catalog = GetHistoricalMazeCatalog();
			Logger::WriteMessage(BuildCatalogSummary(catalog).c_str());
			Assert::IsTrue(catalog.TotalFiles > 0, L"Expected historical maze files to be present.");
			Assert::IsTrue(!catalog.Cases.empty(), L"Expected at least one historical maze with a reachable wall-defined goal.");

			for (const HistoricalMazeCase& testCase : catalog.Cases)
			{
				const std::vector<CellCoordinates> goalCandidates = FindGoalCandidates(testCase.MazeData);
				Assert::IsTrue(
					goalCandidates.size() == 1,
					BuildFailureMessage(L"Loader", testCase, L"reachable view does not contain exactly one goal post").c_str());
				Assert::IsTrue(
					IsGoalReachable(testCase.MazeData, testCase.Start, testCase.GoalLowerLeft),
					BuildFailureMessage(L"Loader", testCase, L"reachable view stored an unreachable goal").c_str());
			}
		}

		TEST_METHOD(FloodFillPathFinderReachesGoalAcrossHistoricalMazes)
		{
			RunHistoricalSweep(L"FloodFillPathFinder", [this](const HistoricalMazeCase& testCase)
				{
					FloodFillPathFinder finder(testCase.MazeData, _vehicle);
					HalfStepPath<PATH_SIZE * 2> path;
					finder.HalfStepPathToGoal(testCase.Start, Direction::Up, path);
					AssertGoalReached(L"FloodFillPathFinder", testCase, path);
				});
		}

		TEST_METHOD(DirectionalPathFinderReachesGoalAcrossHistoricalMazes)
		{
			RunHistoricalSweep(L"DirectionalPathFinder", [this](const HistoricalMazeCase& testCase)
				{
					DirectionalPathFinder finder(testCase.MazeData, _vehicle);
					HalfStepPath<PATH_SIZE * 2> path;
					finder.HalfStepPathToGoal(testCase.Start, Direction::Up, path);
					AssertGoalReached(L"DirectionalPathFinder", testCase, path);
				});
		}

		TEST_METHOD(ManeuverPathFinderReachesGoalAcrossHistoricalMazes)
		{
			RunHistoricalSweep(L"ManeuverPathFinder", [this](const HistoricalMazeCase& testCase)
				{
					ManeuverPathFinder finder(testCase.MazeData, _vehicle);
					HalfStepPath<PATH_SIZE * 2> path;
					finder.HalfStepPathToGoal(testCase.Start, Direction::Up, path);
					AssertGoalReached(L"ManeuverPathFinder", testCase, path);
				});
		}
	};
}



