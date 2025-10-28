#include "pch.h"
#include "CppUnitTest.h"
#include "..\MazeMap\Maze.h"
#include "MazeRef.h"
#include <vector>

#include <sstream>
#include <chrono>
#include "Templates.h"
#include "..\MazeMap\PathFinder.h"
using namespace Microsoft::VisualStudio::CppUnitTestFramework;


namespace MazeMap
{

	bool Mazes::_setup = false;
	Maze Mazes::Maze1 = Maze();
	Maze Mazes::Maze2 = Maze();
	Maze Mazes::Maze3 = Maze();
	std::string Mazes::maze1Data = std::string(
		"OWWW,OWWO,WWOO,WWOO,WWOO,WWOO,WWOO,OWOW,WWWO,OWOO,OWOO,OWOW,OWWO,OWOO,OWOO,OWOW\n"
		"OOWW,WOWO,OWOO,OWOO,OWOO,OWOW,OWWW,OOWO,WWOO,OOOW,OOWW,OOWW,WOWW,OOWW,OOWW,OOWW\n"
		"OOWW,OWWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWO,WWOO,WOOW,WOWW,OOWO,WWOO,WOOW,WOWW,OOWW\n"
		"OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWO,WWOO,WWOO,WWOO,OOOO,WWOO,WWOO,OWOW,OOWW\n"
		"OOWW,OOWO,OOOW,OOWW,OOWW,OOWW,OOWW,OOWO,WWOO,WWOO,WWOO,OOOO,WWOO,WWOO,OOOW,OOWW\n"
		"OOWW,OOWW,OOWW,OOWO,OOOO,OOOO,OOOW,OOWO,WWOO,WWOO,WWOO,OOOO,WWOO,WWOO,OOOW,OOWW\n"
		"OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,WOWO,WWOO,WWOO,WWOO,WOOO,WWOO,WWOO,WOOW,OOWW\n"
		"OOWW,WOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OWWO,OWOW,WWWO,WWOO,WWOO,WWOO,WWOO,OWOO,OOOW\n"
		"OOWO,WWOO,OOOO,OOOW,OOWW,OOWW,OOWW,WOWO,WOOO,WWOO,WWOO,WWOO,WWOO,WWOO,WOOW,OOWW\n"
		"OOWW,OWWW,OOWW,OOWW,OOWW,OOWW,OOWW,OWWO,WWOO,WWOO,WWOO,OWOO,WWOO,WWOO,OWOW,OOWW\n"
		"OOWW,OOWW,OOWW,OOWO,OOOO,OOOO,OOOW,OOWO,WWOO,WWOO,WWOO,OOOO,WWOO,WWOO,OOOW,OOWW\n"
		"OOWW,OOWO,OOOW,OOWW,OOWW,OOWW,OOWW,OOWO,WWOO,WWOO,WWOO,OOOO,WWOO,WWOO,OOOW,OOWW\n"
		"OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWO,WWOO,WWOO,WWOO,OOOO,WWOO,WWOO,WOOW,OOWW\n"
		"OOWW,WOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWO,WWOO,OWOW,OWWW,OOWO,WWOO,OWOW,OWWW,OOWW\n"
		"OOWW,OWWO,WOOO,WOOO,WOOO,WOOW,WOWW,OOWO,WWOO,OOOW,OOWW,OOWW,OWWW,OOWW,OOWW,OOWW\n"
		"WOWW,WOWO,WWOO,WWOO,WWOO,WWOO,WWOO,WOOW,WWWO,WOOO,WOOO,WOOW,WOWO,WOOO,WOOO,WOOW");
	std::string Mazes::maze2Data = std::string(
		"OWWW,OWWO,WWOO,WWOO,WWOO,WWOO,WWOO,OWOW,WWWO,OWOO,OWOO,OWOW,OWWO,OWOO,OWOO,OWOW\n"
		"OOWW,WOWO,OWOO,OWOO,OWOO,OWOW,OWWW,OOWO,WWOO,OOOW,OOWW,OOWW,WOWW,OOWW,OOWW,OOWW\n"
		"OOWW,OWWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWO,WWOO,WOOW,WOWW,OOWO,WWOO,WOOW,WOWW,OOWW\n"
		"OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWO,WWOO,WWOO,WWOO,OOOO,WWOO,WWOO,OWOW,OOWW\n"
		"OOWW,OOWO,OOOW,OOWW,OOWW,OOWW,OOWW,OOWO,WWOO,WWOO,WWOO,OOOO,WWOO,WWOO,WOOW,OOWW\n"
		"OOWW,OOWW,OOWW,OOWO,OOOO,OOOO,OOOW,OOWO,WWOO,WWOO,WWOO,OOOO,WWOW,UWWU,UWUW,OOWW\n"
		"OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,WOWO,WWOO,WWOO,WWOO,WOOO,WWOW,WUWU,WUUW,OOWW\n"
		"OOWW,WOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OWWO,OWOW,WWWO,WWOO,WWOO,WWOO,WWOO,OWOO,OOOW\n"
		"OOWO,WWOO,OOOO,OOOW,OOWW,OOWW,OOWW,WOWO,WOOO,WWOO,WWOO,WWOO,WWOO,WWOO,WOOW,OOWW\n"
		"OOWW,OWWW,OOWW,OOWW,OOWW,OOWW,OOWW,OWWO,WWOO,WWOO,WWOO,OWOO,WWOO,WWOO,OWOW,OOWW\n"
		"OOWW,OOWW,OOWW,OOWO,OOOO,OOOO,OOOW,OOWO,WWOO,WWOO,WWOO,OOOO,WWOO,WWOO,OOOW,OOWW\n"
		"OOWW,OOWO,OOOW,OOWW,OOWW,OOWW,OOWW,OOWO,WWOO,WWOO,WWOO,OOOO,WWOO,WWOO,OOOW,OOWW\n"
		"OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWO,WWOO,WWOO,WWOO,OOOO,WWOO,WWOO,WOOW,OOWW\n"
		"OOWW,WOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWO,WWOO,OWOW,OWWW,OOWO,WWOO,OWOW,OWWW,OOWW\n"
		"OOWW,OWWO,WOOO,WOOO,WOOO,WOOW,WOWW,OOWO,WWOO,OOOW,OOWW,OOWW,OWWW,OOWW,OOWW,OOWW\n"
		"WOWW,WOWO,WWOO,WWOO,WWOO,WWOO,WWOO,WOOW,WWWO,WOOO,WOOO,WOOW,WOWO,WOOO,WOOO,WOOW");
	std::string Mazes::maze3Data = std::string(
		"OWWO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,OWOW\n"
		"OOWW,OWWO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WOOW\n"
		"OOWW,WOWO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,WWOO,OWOW\n"
		"WOWO,OWOW,OWWO,OWOW,OWWO,OWOW,OWWO,OWOW,OWWO,OWOW,OWWO,OWOW,OWWO,OWOW,OWWO,OOOW\n"
		"OWWW,WOWO,WOOW,WOWO,WOOW,WOWO,WOOW,WOWO,WOOW,WOWO,WOOW,WOWO,WOOW,WOWO,WOOW,OOWW\n"
		"OOWW,OWWW,OWOO,OWOO,OWOO,OWOO,OWOO,OWOO,OWOO,OWOO,OWOW,OWWO,OWOW,WWWO,WWOW,OOWW\n"
		"OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,WOWO,WOOO,WWOO,WWOO,WOOW\n"
		"OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OWOO,OWOO,OWOO,OWOO,OWOW\n"
		"OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW\n"
		"OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW\n"
		"OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW\n"
		"OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW\n"
		"OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW\n"
		"OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW\n"
		"OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW,OOWW\n"
		"WOWW,WOWW,WOWW,WOWW,WOWW,WOWW,WOWW,WOWW,WOWW,WOWW,WOWW,WOWW,WOWW,WOWW,WOWW,WOWW\n"
	);
	TEST_CLASS(MazeTest)
	{
	public:



		static void ValidateMaze(const Maze& maze)
		{
			bool error = false;
			for (int i = 0; i < maze.GetXSize(); i++)
			{
				for (int j = 0; j < maze.GetYSize(); j++)
				{
					Cell current = maze(i, j);
					CharBlock cbCur = current.Serialize();
					if (j < 15)
					{
						Cell up = maze(i, j + 1);
						CharBlock cbUp = up.Serialize();
						WallState cu = current.GetUp();
						WallState ud = up.GetDown();
						if (cu != ud)
						{
							error = true;
						}
						if (current.GetUp() != up.GetDown())
						{
							std::wstringstream ss = std::wstringstream();
							/*ss << "Error: Expected " << Microsoft::VisualStudio::CppUnitTestFramework::ToString(cu);
							ss << ", Found " << Microsoft::VisualStudio::CppUnitTestFramework::ToString(ud);*/
							ss << "\nAt " << Microsoft::VisualStudio::CppUnitTestFramework::ToString(current);
							ss << " and " << Microsoft::VisualStudio::CppUnitTestFramework::ToString(up) << ".";
							Assert::AreEqual(current.GetUp(), up.GetDown(), (&ss.str()[0]));
						}
						
					}
					if (i < 15)
					{
						Cell right = maze(i+1, j);
						CharBlock cbRight = right.Serialize();

						WallState cr = current.GetRight();
						WallState rl = right.GetLeft();
						if (current.GetRight() != right.GetLeft())
						{
							error = true;
						}
						if (current.GetRight() != right.GetLeft())
						{
							std::wstringstream ss = std::wstringstream();
							/*ss << "Error: Expected " << Microsoft::VisualStudio::CppUnitTestFramework::ToString(cr);
							ss << ", Found " << Microsoft::VisualStudio::CppUnitTestFramework::ToString(rl);*/
							ss << "\nAt " << Microsoft::VisualStudio::CppUnitTestFramework::ToString(current);
							ss << " and " << Microsoft::VisualStudio::CppUnitTestFramework::ToString(right) << ".";
							Assert::AreEqual(current.GetRight(), right.GetLeft(), (&ss.str()[0]));
						}
					}

				}
			}
		}

		TEST_METHOD(TestUnitTestInfrastructure)
		{
			Maze m1 = Mazes::GetMaze1();
			Maze m2 = Mazes::GetMaze2();
			Maze m3 = Mazes::GetMaze2();

			ValidateMaze(m1);
			ValidateMaze(m2);
			ValidateMaze(m3);
		}
		TEST_METHOD(TestIsAccessibleLocation)
		{
			Maze m1 = Mazes::GetMaze1();

			for (size_t i = 0; i < 16; i++)
			{
				for (size_t j = 0; j < 16; j++)
				{
					for (Direction d = Direction::Up; d <= Direction::Right; d= d<<1)
					{
						MazeLocation l((i << 1) + 1, (j << 1) + 1);
						WallState s = m1(i, j).GetWall(d);
						l = l >> d;
						bool b2 = (s == WallState::NoWall) == m1.IsAccessibleLocation(l);
						bool b = m1.IsAccessibleLocation(l);
						Assert::AreEqual(s == WallState::NoWall, b);
						Assert::IsTrue(b2);
					}
				}
			}
		}
		TEST_METHOD(TestConstructor)
		{
			Maze m = Maze();
			Assert::AreEqual(m.GetCellDimension(), 18.0F);
			for (int i = 0; i < m.GetXSize(); i++)
			{
				for (int j = 0; j < m.GetYSize(); j++)
				{
					Cell& c = m(i, j);

					Assert::AreEqual((int)c.GetX(), i);
					Assert::AreEqual((int)c.GetY(), j);
					if (j == 15)
					{
						Assert::AreEqual(Wall, c.GetUp());
					}
					else
					{
						Assert::AreEqual(Unknown, c.GetUp());
					}
					if (j == 0)
					{
						Assert::AreEqual(Wall, c.GetDown());
					}
					else
					{
						Assert::AreEqual(Unknown, c.GetDown());
					}
					if (i == 0)
					{
						Assert::AreEqual(Wall, c.GetLeft());
					}
					else
					{
						Assert::AreEqual(Unknown, c.GetLeft());
					}
					if (i == 15)
					{
						Assert::AreEqual(Wall, c.GetRight());
					}
					else
					{
						Assert::AreEqual(Unknown, c.GetRight());
					}
				}
			}
		}
		TEST_METHOD(TestSize)
		{
			Maze m = Maze();
			int xSize = m.GetXSize();
			int ySize = m.GetYSize();
			Assert::AreEqual(m.GetXSize(), (uint8_t)16);
			Assert::AreEqual(m.GetYSize(), (uint8_t)16);
		}
		TEST_METHOD(TestIndex)
		{
			Maze m = Maze();
			Cell& c = m(1, 1);

			c.SetUp(Wall);
			Assert::AreEqual(c.GetUp(), m(1, 1).GetUp());
		}
		TEST_METHOD(TestIndex2)
		{
			Maze m = Maze();
			Cell& c = m(1, 1);

			m(1, 1).SetUp(Wall);
			Assert::AreEqual(c.GetUp(), m(1, 1).GetUp());
		}
		TEST_METHOD(TestIndex3)
		{
			Maze m = Maze();
			Cell& c = m(2, 2);

			m(2, 2).SetDown(NoWall);
			Assert::AreEqual(NoWall, m(2, 2).GetDown());
		}
		TEST_METHOD(TestIndex4)
		{
			Maze m = Maze();
			Cell& c = m(2, 2);

			c.SetDown(NoWall);
			Assert::AreEqual(NoWall, m(2, 2).GetDown());
		}
		TEST_METHOD(TestSetWallA)
		{
			Maze m = Maze();
			Cell& c = m(1, 1);
			m.SetWall(c, Up, Wall);
			Assert::AreEqual(c.GetUp(), Wall);
			Assert::AreEqual(m(1, 1).GetUp(), Wall);
			Assert::AreEqual(m(1, 1).GetDown(), Unknown);
			Assert::AreEqual(m(1, 1).GetLeft(), Unknown);
			Assert::AreEqual(m(1, 1).GetRight(), Unknown);

			ValidateMaze(m);
		}
		TEST_METHOD(TestSetWallA2)
		{
			Maze m = Maze();
			Cell& c = m(1, 15);
			m.SetWall(c, Up, Wall);
			Assert::AreEqual(m(1, 15).GetUp(), Wall);
			Assert::AreEqual(m(1, 15).GetDown(), Unknown);
			Assert::AreEqual(m(1, 15).GetLeft(), Unknown);
			Assert::AreEqual(m(1, 15).GetRight(), Unknown);

			ValidateMaze(m);
		}
		TEST_METHOD(TestSetWallB)
		{
			Maze m = Maze();
			Cell& c = m(1, 1);
			m.SetWall(c, Down, Wall);
			Assert::AreEqual(m(1, 1).GetUp(), Unknown);
			Assert::AreEqual(m(1, 1).GetDown(), Wall);
			Assert::AreEqual(m(1, 1).GetLeft(), Unknown);
			Assert::AreEqual(m(1, 1).GetRight(), Unknown);

			ValidateMaze(m);
		}
		TEST_METHOD(TestSetWallB2)
		{
			Maze m = Maze();
			Cell& c = m(1, 0);
			m.SetWall(c, Down, Wall);
			Assert::AreEqual(m(1, 0).GetUp(), Unknown);
			Assert::AreEqual(m(1, 0).GetDown(), Wall);
			Assert::AreEqual(m(1, 0).GetLeft(), Unknown);
			Assert::AreEqual(m(1, 0).GetRight(), Unknown);

			ValidateMaze(m);
		}
		TEST_METHOD(TestSetWallC)
		{
			Maze m = Maze();
			Cell& c = m(1, 1);
			m.SetWall(c, Left, Wall);
			Assert::AreEqual(m(1, 1).GetUp(), Unknown);
			Assert::AreEqual(m(1, 1).GetDown(), Unknown);
			Assert::AreEqual(m(1, 1).GetLeft(), Wall);
			Assert::AreEqual(m(1, 1).GetRight(), Unknown);

			ValidateMaze(m);
		}
		TEST_METHOD(TestSetWallC2)
		{
			Maze m = Maze();
			Cell& c = m(0, 1);
			m.SetWall(c, Left, Wall);
			Assert::AreEqual(m(0, 1).GetUp(), Unknown);
			Assert::AreEqual(m(0, 1).GetDown(), Unknown);
			Assert::AreEqual(m(0, 1).GetLeft(), Wall);
			Assert::AreEqual(m(0, 1).GetRight(), Unknown);

			ValidateMaze(m);
		}
		TEST_METHOD(TestSetWallD)
		{
			Maze m = Maze();
			Cell& c = m(1, 1);
			m.SetWall(c, Right, Wall);
			Assert::AreEqual(m(1, 1).GetUp(), Unknown);
			Assert::AreEqual(m(1, 1).GetDown(), Unknown);
			Assert::AreEqual(m(1, 1).GetLeft(), Unknown);
			Assert::AreEqual(m(1, 1).GetRight(), Wall);

			ValidateMaze(m);
		}
		TEST_METHOD(TestSetWallD2)
		{
			Maze m = Maze();
			Cell& c = m(15, 1);
			m.SetWall(c, Right, Wall);
			Assert::AreEqual(m(15, 1).GetUp(), Unknown);
			Assert::AreEqual(m(15, 1).GetDown(), Unknown);
			Assert::AreEqual(m(15, 1).GetLeft(), Unknown);
			Assert::AreEqual(m(15, 1).GetRight(), Wall);

			ValidateMaze(m);
		}
		TEST_METHOD(TestGoalFound)
		{
			Maze& m = Mazes::GetMaze1();

			Assert::IsTrue(m.HasFoundGoal());
		}
		TEST_METHOD(TestRefMazes)
		{
			Maze& m1 = Mazes::GetMaze1();

			Assert::IsTrue(m1(1, 2).GetUp() == WallState::NoWall);
			Assert::IsTrue(m1(1, 2).GetDown() == WallState::Wall);
			Assert::IsTrue(m1(1, 2).GetLeft() == WallState::Wall);
			Assert::IsTrue(m1(1, 2).GetRight() == WallState::Wall);

			Assert::IsTrue(m1(2, 1).GetUp() == WallState::NoWall);
			Assert::IsTrue(m1(2, 1).GetDown() == WallState::Wall);
			Assert::IsTrue(m1(2, 1).GetLeft() == WallState::NoWall);
			Assert::IsTrue(m1(2, 1).GetRight() == WallState::NoWall);


		}
		TEST_METHOD(TestRefMazes2)
		{
			Maze& m1 = Mazes::GetMaze1();
			ValidateMaze(m1);
		}

		TEST_METHOD(TestRefMazes3)
		{
			Maze& m2 = Mazes::GetMaze2();
			ValidateMaze(m2);
		}
		TEST_METHOD(TestIsComplete1)
		{
			Maze& m = Mazes::GetMaze1();

			Assert::IsTrue(m.IsComplete());
		}
		TEST_METHOD(TestIsCompleteSpeed1000)
		{
			Maze& m = Mazes::GetMaze1();
			m.IsComplete();
			WallState ws = m(0, 0).GetUp();
			auto start = std::chrono::high_resolution_clock::now();
			for (int i = 0; i < 100; ++i)
			{
				m.SetWall(m(0, 0), Up, Unknown);
				m.SetWall(m(0, 0), Up, ws);
				m.IsComplete();
				m.SetWall(m(0, 0), Up, Unknown);
				m.SetWall(m(0, 0), Up, ws);
				m.IsComplete();
				m.SetWall(m(0, 0), Up, Unknown);
				m.SetWall(m(0, 0), Up, ws);
				m.IsComplete();
				m.SetWall(m(0, 0), Up, Unknown);
				m.SetWall(m(0, 0), Up, ws);
				m.IsComplete();
				m.SetWall(m(0, 0), Up, Unknown);
				m.SetWall(m(0, 0), Up, ws);
				m.IsComplete();

				m.SetWall(m(0, 0), Up, Unknown);
				m.SetWall(m(0, 0), Up, ws);
				m.IsComplete();
				m.SetWall(m(0, 0), Up, Unknown);
				m.SetWall(m(0, 0), Up, ws);
				m.IsComplete();
				m.SetWall(m(0, 0), Up, Unknown);
				m.SetWall(m(0, 0), Up, ws);
				m.IsComplete();
				m.SetWall(m(0, 0), Up, Unknown);
				m.SetWall(m(0, 0), Up, ws);
				m.IsComplete();
				m.SetWall(m(0, 0), Up, Unknown);
				m.SetWall(m(0, 0), Up, ws);
				m.IsComplete();
			}
			auto end = std::chrono::high_resolution_clock::now();
			auto duration = end - start;
			Assert::IsTrue(m.IsComplete());
		}
		TEST_METHOD(TestIsComplete1b)
		{
			Maze m = Mazes::GetMaze1();
			m.SetWall(m(3, 4), Direction::Up, WallState::Unknown);

			Assert::AreEqual(WallState::Unknown, m(3, 4).GetUp());
			Assert::IsFalse(m(3, 4).IsFullyKnown());

			Assert::IsFalse(m.IsComplete());
		}
		TEST_METHOD(TestIsComplete1c)
		{
			Maze m = Mazes::GetMaze1();
			Vehicle v = Vehicle();
			Assert::IsTrue(m.IsComplete());
			Assert::IsTrue(m.HasFoundGoal());
			FloodFillPathFinder ffpf(m, v);

			Path<PATH_SIZE> P;
			ffpf.PathToGoal(CellCoordinates(0, 0), Direction::Up, P);

			Assert::IsTrue(P.GetSize() > 40);
			return;
		}
		TEST_METHOD(TestPathFinding)
		{
			Maze m = Mazes::GetMaze1();
			Vehicle v = Vehicle();
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
			Vehicle v = Vehicle();
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
		TEST_METHOD(TestIsComplete2)
		{
			Maze& m = Mazes::GetMaze2();
			bool result = m.IsComplete();
			Assert::IsTrue(result);
		}
	};
}