#include "pch.h"
#include "CppUnitTest.h"
#include "MazeRef.h"
#include "Templates.h"
#include "..\MazeMap\PathFinder.h"
#include "..\MazeMap\DirectionalPathFinder.h"
#include "../MazeMap/ManeuverPathFinder.h"
#include <map>
#include <chrono>
using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
	TEST_CLASS(PathFinderTest)
	{
	public:
		Vehicle v = Vehicle(15.0f, 18.0f, 35.0f, 4.5f, 5000.0f);
		void TestPathContinuity(const Path<PATH_SIZE>& path)
		{
			for (size_t i = 1; i < path.GetSize(); i++)
			{
				Direction prevDirection = path[i].DirectionTo(path[i - 1]);
				Assert::AreEqual(path[i - 1], path[i] >> prevDirection);
			}
		}

		void TestPathFinder(PathFinder& pathFinder, Maze& m)
		{
			Path<PATH_SIZE> P;
			pathFinder.PathToGoal(CellCoordinates(0, 0), Direction::Up, P);

			Assert::AreEqual(CellCoordinates(0, 0), P.first());

			int dx = P.last().GetX() - m.GetGoalLowerLeft().GetX();
			int dy = P.last().GetY() - m.GetGoalLowerLeft().GetY();

			Assert::IsTrue(dx >= 0);
			Assert::IsTrue(dx <= 1);
			Assert::IsTrue(dy >= 0);
			Assert::IsTrue(dy <= 1);
			return;
		}

		DirectionalLocation ExecutePartial(DirectionalLocation start, ManeuverPath& p, int steps)
		{
			ManeuverPath tmp = ManeuverPath();
			for (size_t i = 0; i < steps; i++)
			{
				tmp.push_back(p[i]);
			}

			return tmp.ExecutePath(start);
		}
		DirectionalLocation ExecutePartialRoundTrip(DirectionalLocation start, ManeuverPath& p, int steps)
		{
			ManeuverPath tmp = ManeuverPath();
			for (size_t i = 0; i < steps; i++)
			{
				tmp.push_back(p[i]);
			}

			DirectionalLocation end = tmp.ExecutePath(start);
			return tmp.ExecuteReverse(end);
		}

		void TestPathReverse(ManeuverPath& p)
		{
			DirectionalLocation start(1, 1, Up);
			for (int i = 1; i < p.GetSize(); ++i)
			{
				DirectionalLocation roundTrip = ExecutePartialRoundTrip(start, p, i);
				Assert::AreEqual(start, roundTrip);
			}

			return;
		}
		TEST_METHOD(TestComplexPathFinding)
		{
			//Vehicle V = Vehicle(5.0f,22.0f,10.0f,4.0f);
			Mazes::SetupMazes();
			DirectionalPathFinder* dpf= new DirectionalPathFinder(Mazes::GetMaze1(), v);
			/*TestPathFinder(*dpf);*/
			Mazes::GetMaze1().IsComplete();
			Mazes::GetMaze1().HasFoundGoal();
			Mazes::GetMaze1().PreCalculate();
			Path<PATH_SIZE> *p = new Path<PATH_SIZE>();
			dpf->PathToGoal(CellCoordinates(0, 0), Direction::Up, *p);

			TestPathContinuity(*p);

			delete p;
			delete dpf;
			return;
		}
		TEST_METHOD(TestComplexPathFinding2)
		{
			//Vehicle V = Vehicle(5.0f,22.0f,10.0f,4.0f);
			Mazes::SetupMazes();
			Maze& m = Mazes::GetMaze3();
			m.IsComplete();
			m.HasFoundGoal();
			m.PreCalculate();
			DirectionalPathFinder* dpf = new DirectionalPathFinder(m, v);
			TestPathFinder(*dpf, m);

			delete dpf;
			return;
		}
		TEST_METHOD(TestManeuverPathFinding)
		{
			//Vehicle V = Vehicle(5.0f,22.0f,10.0f,4.0f);
			Mazes::SetupMazes();
			ManeuverPathFinder* mpf = new ManeuverPathFinder(Mazes::GetMaze1(), v);
			/*TestPathFinder(*dpf);*/
			Mazes::GetMaze1().IsComplete();
			Mazes::GetMaze1().HasFoundGoal();
			Mazes::GetMaze1().PreCalculate();
			Path<PATH_SIZE>* p = new Path<PATH_SIZE>();
			mpf->PathToGoal(CellCoordinates(0, 0), Direction::Up, *p);

			TestPathContinuity(*p);

			delete p;
			delete mpf;
			return;
		}
		TEST_METHOD(TestManeuverPathFindingSingleTurn)
		{
			//Vehicle V = Vehicle(5.0f,22.0f,10.0f,4.0f);
			Mazes::SetupMazes();
			Maze& m = Mazes::GetSingleTurnMaze();
			m.IsComplete();
			m.HasFoundGoal();
			m.PreCalculate();
			ManeuverPathFinder* mpf = new ManeuverPathFinder(m, v);
			HalfStepPath<PATH_SIZE * 2> P;
			mpf->HalfStepPathToGoal(CellCoordinates(0, 0), Direction::Up, P);

			int dx = P.last().GetX() - m.GetGoalLowerLeft().GetX();
			int dy = P.last().GetY() - m.GetGoalLowerLeft().GetY();

			HalfStepPath<PATH_SIZE * 2> expected = HalfStepPath<PATH_SIZE * 2>();
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

			for (int i = 0; i < expected.GetSize(); ++i)
			{
				Assert::AreEqual(expected[i], P[i]);
			}

			delete mpf;
			return;
		}
		TEST_METHOD(TestManeuverPathFinding2)
		{
			//Vehicle V = Vehicle(5.0f,22.0f,10.0f,4.0f);
			Mazes::SetupMazes();
			Maze& m = Mazes::GetMaze2();
			m.IsComplete();
			m.HasFoundGoal();
			m.PreCalculate();
			ManeuverPathFinder* mpf = new ManeuverPathFinder(m, v);
			HalfStepPath<PATH_SIZE*2> P;
			mpf->HalfStepPathToGoal(CellCoordinates(0, 0), Direction::Up, P);

			Assert::AreEqual(MazeLocation(1,1), P.first());

			MazeLocation goalLL = MazeLocation::CellCenter(m.GetGoalLowerLeft());
			MazeLocation last = P.last();


			int dx = P.last().GetX() - goalLL.GetX();
			int dy = P.last().GetY() - goalLL.GetY();

			Assert::IsTrue(dx >= 0);
			Assert::IsTrue(dx <= 3);
			Assert::IsTrue(dy >= 0);
			Assert::IsTrue(dy <= 3);
			return;

			delete mpf;
			return;
		}
		TEST_METHOD(TestManeuverPathFinding3)
		{
			//Vehicle V = Vehicle(5.0f,22.0f,10.0f,4.0f);
			ManeuverSet& ms = ManeuverSet::GetSet();
			Mazes::SetupMazes();
			Maze& m = Mazes::GetMaze2();
			m.IsComplete();
			m.HasFoundGoal();
			m.PreCalculate();
			ManeuverPathFinder* mpf = new ManeuverPathFinder(m, v);
			ManeuverPath mp = ManeuverPath();
			HalfStepPath<PATH_SIZE * 2> P;
			mpf->ManeuverPathToGoal(CellCoordinates(0, 0), Direction::Up, mp);



			mp.ToHalfStepPath(DirectionalLocation(1, 1, Up), P);

			int len = 1;
			for (size_t i = 0; i < mp.GetSize(); i++)
			{
				for (size_t j = 0; j < ms.GetStepCount(mp[i]); j++)
				{
					len += ms.GetStep(mp[i], j).GetDistance();
				}
			}



			Assert::AreEqual(len, (int)P.GetSize());

			delete mpf;
			return;
		}
		TEST_METHOD(TestPathFinding)
		{
			Maze m = Mazes::GetMaze3();
			//Vehicle v = Vehicle();
			Assert::IsTrue(m.IsComplete());
			Assert::IsTrue(m.HasFoundGoal());
			FloodFillPathFinder ffpf(m, v);

			Path<PATH_SIZE> P;
			ffpf.PathToGoal(CellCoordinates(0, 0), Direction::Up, P);

			Assert::AreEqual(CellCoordinates(0, 0), P.first());

			int dx = P.last().GetX() - m.GetGoalLowerLeft().GetX();
			int dy = P.last().GetY() - m.GetGoalLowerLeft().GetY();

			Assert::IsTrue(dx >= 0);
			Assert::IsTrue(dx <= 1);
			Assert::IsTrue(dy >= 0);
			Assert::IsTrue(dy <= 1);
			return;
		}
		TEST_METHOD(TestPathFindingContinuity)
		{
			Maze m = Mazes::GetMaze1();
			//Vehicle v = Vehicle();
			Assert::IsTrue(m.IsComplete());
			Assert::IsTrue(m.HasFoundGoal());
			FloodFillPathFinder ffpf(m, v);

			Path<PATH_SIZE> P;
			ffpf.PathToGoal(CellCoordinates(0, 0), Direction::Up, P);

			Assert::AreEqual(CellCoordinates(0, 0), P.first());

			for (size_t i = 1; i < P.GetSize(); i++)
			{
				Direction prevDirection = P[i].DirectionTo(P[i - 1]);
				Assert::AreEqual(P[i - 1], P[i] >> prevDirection);
			}

			return;
		}
		TEST_METHOD(APEC2016ManeuverPathFinderTest)
		{
			Maze m = Mazes::GetMazeAPEC2016();
			//Vehicle v = Vehicle();
			Assert::IsTrue(m.IsComplete());
			Assert::IsTrue(m.HasFoundGoal());
			ManeuverPathFinder* mpf = new ManeuverPathFinder(m, v);

			ManeuverPath mp;
			mpf->ManeuverPathToGoal(CellCoordinates(0, 0), Direction::Up, mp);

			std::map<ManeuverCode, int> maneuverCounts = std::map<ManeuverCode, int>();
			for (uint16_t i = 0; i < mp.GetSize(); i++)
			{
				if (mp[i] < S31)
				{
					continue;
				}
				ManeuverCode base = static_cast<ManeuverCode>(mp[i] & (~MIRRORED_MANEUVER_FLAG));
				++maneuverCounts[base];
			}
			TestPathReverse(mp);
			return;
		}
		TEST_METHOD(APEC2016ManeuverPathFinderTest2)
		{
			Maze m = Mazes::GetMazeAPEC2016();
			//Vehicle v = Vehicle();
			Assert::IsTrue(m.IsComplete());
			Assert::IsTrue(m.HasFoundGoal());
			ManeuverPathFinder* mpf = new ManeuverPathFinder(m, v);
			ManeuverSet& ms = ManeuverSet::GetSet();

			ManeuverPath mp;
			mpf->ManeuverPathFromTo(CellCoordinates(8, 7), Direction::Right, CellCoordinates(0,0), mp);

			std::map<ManeuverCode, int> maneuverCounts = std::map<ManeuverCode, int>();

			DirectionalLocation current(MazeLocation::CellCenter(CellCoordinates(8, 7)), Right);

			for (size_t i = 0; i < mp.GetSize(); i++)
			{
				ManeuverCode code = mp[i];
				for (size_t j = 0; j < ms.GetStepCount(code); j++)
				{
					auto step = ms.GetStep(code, j);
					current = current.Turn(step.GetDirection());
					for (size_t k = 0; k < step.GetDistance(); k++)
					{
						current = current.MoveForward(1);
						Assert::IsTrue(m.IsAccessibleLocation(current.GetLocation()));
					}
				}
			}

			TestPathReverse(mp);
			return;
		}
		TEST_METHOD(APEC2016ManeuverPathFinderSpeedTest)
		{
			Maze m = Mazes::GetMazeAPEC2016();
			//Vehicle v = Vehicle();
			m.PreCalculate();
			Assert::IsTrue(m.IsComplete());
			Assert::IsTrue(m.HasFoundGoal());
			ManeuverPathFinder* mpf = new ManeuverPathFinder(m, v);

			ManeuverPath mp;
			mpf->ManeuverPathToGoal(CellCoordinates(0, 0), Direction::Up, mp);

			auto start = std::chrono::high_resolution_clock::now();
			for (size_t i = 0; i < 100; i++)
			{
				mpf->ManeuverPathToGoal(CellCoordinates(0, 0), Direction::Up, mp);

			}
			auto end = std::chrono::high_resolution_clock::now();
			std::chrono::nanoseconds duration = end - start;
			std::wstringstream ss = std::wstringstream();
			ss << ( duration.count() * 0.000000001);
			std::wstring str = ss.str();
			Microsoft::VisualStudio::CppUnitTestFramework::Logger::WriteMessage(str.c_str());
			return;
		}
	};
}