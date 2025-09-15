#include "pch.h"
#include "CppUnitTest.h"
#include "..\MazeMap\Maze.h"

#include <vector>

#include <sstream>

#include "Templates.h"
using namespace Microsoft::VisualStudio::CppUnitTestFramework;


namespace MazeMap
{
	TEST_CLASS(MazeTest)
	{
	public:

		static void ValidateMaze(const Maze& maze)
		{
			for (int i = 0; i < maze.GetXSize(); i++)
			{
				for (int j = 0; j < maze.GetYSize(); j++)
				{
					Cell current = maze(i, j);
					if (j < 15)
					{
						Cell up = maze(i, j + 1);
						Assert::AreEqual(current.GetUp(), up.GetDown());
					}
					if (i < 15)
					{
						Cell right = maze(i+1, j);
						Assert::AreEqual(current.GetRight(), right.GetLeft());
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

					Assert::AreEqual(c.GetX(), i);
					Assert::AreEqual(c.GetY(), j);
					Assert::AreEqual(c.GetUp(),    Unknown);
					Assert::AreEqual(c.GetDown(),  Unknown);
					Assert::AreEqual(c.GetLeft(),  Unknown);
					Assert::AreEqual(c.GetRight(), Unknown);
				}
			}
		}
		TEST_METHOD(TestSize)
		{
			Maze m = Maze();
			int xSize = m.GetXSize();
			int ySize = m.GetYSize();
			Assert::AreEqual(m.GetXSize(), 16);
			Assert::AreEqual(m.GetYSize(), 16);
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
	};
}