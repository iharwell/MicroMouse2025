#include "pch.h"
#include "CppUnitTest.h"
#include "MazeRef.h"
#include "Templates.h"
#include "..\MazeMap\PathFinder.h"
#include "..\MazeMap\DirectionalPathFinder.h"
using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
	TEST_CLASS(PathFinderTest)
	{
	public:
		Vehicle v = Vehicle(5.0f, 2.0f, 10.0f, 4.0f);

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
	};
}