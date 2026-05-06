#include "pch.h"
#include "Templates.h"
#include "CppUnitTest.h"
#include "..\MazeMap\Direction.h"
#include "..\MazeMap\MazeLocation.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
	TEST_CLASS(DirectionTest)
	{
	public:

		TEST_METHOD(OpTest1)
		{
			Direction d = Direction::Up;
			RelativeDirection rd = RelativeDirection::Left45;
			Assert::AreEqual(Direction::UpLeft, d + rd);
		}
		TEST_METHOD(OpTest2)
		{
			RelativeDirection d = RelativeDirection::Forward;
			RelativeDirection rd = RelativeDirection::Left45;
			Assert::AreEqual(RelativeDirection::Left45, d + rd);
		}
		TEST_METHOD(DirectionFromCellCenterTest)
		{
			MazeLocation loc = MazeLocation(15, 15);
			Direction d = loc.DirectionFromCellCenter();
			Assert::IsTrue(d == Direction::None);
		}
		TEST_METHOD(MazeLocationStepTest)
		{
			MazeLocation loc = MazeLocation(15, 15);
			MazeLocation expected = MazeLocation(16, 15);
			Direction d = Direction::Right;
			Assert::AreEqual(expected.GetX(), (loc >> d).GetX());
			Assert::AreEqual(expected.GetY(), (loc >> d).GetY());
		}
		TEST_METHOD(DirectionalLocationStepTest1)
		{
			DirectionalLocation loc = DirectionalLocation(MazeLocation(15, 15), Direction::Up);
			DirectionalLocation expected = DirectionalLocation(MazeLocation(15, 17), Direction::Up);
			Assert::IsTrue(expected == (loc.MoveForward(2)));
		}
		TEST_METHOD(DirectionalLocationStepTest2)
		{
			DirectionalLocation loc = DirectionalLocation(MazeLocation(15, 15), Direction::DownLeft);
			DirectionalLocation expected = DirectionalLocation(MazeLocation(13, 13), Direction::DownLeft);
			Assert::IsTrue(expected == (loc.MoveForward(2)));
		}
		TEST_METHOD(DirectionalLocationStepTest3)
		{
			DirectionalLocation loc = DirectionalLocation(MazeLocation(15, 15), Direction::Up);
			RelativeDirectionalDistance rdd(Right45, 2);
			DirectionalLocation expected = DirectionalLocation(MazeLocation(17, 17), Direction::UpRight);
			Assert::AreEqual(expected, (loc >> rdd));
		}
		TEST_METHOD(DirectionalLocationStepTest4)
		{
			DirectionalLocation loc = DirectionalLocation(MazeLocation(15, 15), Direction::Up);
			RelativeDirectionalDistance rdd(Left45, 2);
			DirectionalLocation expected = DirectionalLocation(MazeLocation(13, 17), Direction::UpLeft);
			Assert::AreEqual(expected, (loc >> rdd));
		}
		TEST_METHOD(MazeLocationStepTest2)
		{
			MazeLocation loc = MazeLocation(15, 15);
			MazeLocation expected = MazeLocation(15, 16);
			Direction d = Direction::Up;
			Assert::AreEqual(expected.GetX(), (loc >> d).GetX());
			Assert::AreEqual(expected.GetY(), (loc >> d).GetY());
		}
		TEST_METHOD(MazeLocationStepTest3)
		{
			MazeLocation loc = MazeLocation(15, 15);
			MazeLocation expected = MazeLocation(16, 16);
			Direction d = Direction::UpRight;
			Assert::AreEqual(expected.GetX(), (loc >> d).GetX());
			Assert::AreEqual(expected.GetY(), (loc >> d).GetY());
		}
		TEST_METHOD(MazeLocationStepTest4)
		{
			MazeLocation loc = MazeLocation(15, 15);
			MazeLocation expected = MazeLocation(14, 15);
			Direction d = Direction::Left;
			Assert::AreEqual(expected.GetX(), (loc >> d).GetX());
			Assert::AreEqual(expected.GetY(), (loc >> d).GetY());
		}
		TEST_METHOD(MazeLocationStepTest5)
		{
			MazeLocation loc = MazeLocation(15, 15);
			MazeLocation expected = MazeLocation(16, 15);
			Direction d = Direction::Right;
			Assert::AreEqual(expected.GetX(), (loc >> d).GetX());
			Assert::AreEqual(expected.GetY(), (loc >> d).GetY());
		}
		TEST_METHOD(MazeLocationFirstCellTest1)
		{
			MazeLocation loc = MazeLocation(15, 15);
			CellCoordinates expected = CellCoordinates(7, 7);
			Assert::AreEqual(expected.GetX(), loc.GetFirstConnectedCell().GetX());
			Assert::AreEqual(expected.GetY(), loc.GetFirstConnectedCell().GetY());
		}
		TEST_METHOD(MazeLocationFirstCellTest2)
		{
			MazeLocation loc = MazeLocation(14, 15);
			CellCoordinates expected = CellCoordinates(6, 7);
			Assert::AreEqual(expected.GetX(), loc.GetFirstConnectedCell().GetX());
			Assert::AreEqual(expected.GetY(), loc.GetFirstConnectedCell().GetY());
		}
		TEST_METHOD(MazeLocationFirstCellTest3)
		{
			MazeLocation loc = MazeLocation(15, 14);
			CellCoordinates expected = CellCoordinates(7, 6);
			Assert::AreEqual(expected.GetX(), loc.GetFirstConnectedCell().GetX());
			Assert::AreEqual(expected.GetY(), loc.GetFirstConnectedCell().GetY());
		}
		TEST_METHOD(MazeLocationSecondCellTest1)
		{
			MazeLocation loc = MazeLocation(15, 15);
			CellCoordinates expected = CellCoordinates(7, 7);
			Assert::AreEqual(expected.GetX(), loc.GetSecondConnectedCell().GetX());
			Assert::AreEqual(expected.GetY(), loc.GetSecondConnectedCell().GetY());
		}
		TEST_METHOD(MazeLocationSecondCellTest2)
		{
			MazeLocation loc = MazeLocation(14, 15);
			CellCoordinates expected = CellCoordinates(7, 7);
			Assert::AreEqual(expected.GetX(), loc.GetSecondConnectedCell().GetX());
			Assert::AreEqual(expected.GetY(), loc.GetSecondConnectedCell().GetY());
		}
		TEST_METHOD(MazeLocationSecondCellTest3)
		{
			MazeLocation loc = MazeLocation(15, 14);
			CellCoordinates expected = CellCoordinates(7, 7);
			Assert::AreEqual(expected.GetX(), loc.GetSecondConnectedCell().GetX());
			Assert::AreEqual(expected.GetY(), loc.GetSecondConnectedCell().GetY());
		}

		TEST_METHOD(RelativeDirectionInvertTest1)
		{
			Assert::AreEqual(Forward, -Forward);
		}

		TEST_METHOD(RelativeDirectionInvertTest2)
		{
			Assert::AreEqual(Left45, -Right45);
		}

		TEST_METHOD(RelativeDirectionInvertTest3)
		{
			Assert::AreEqual(Left90, -Right90);
		}

		TEST_METHOD(RelativeDirectionInvertTest4)
		{
			Assert::AreEqual(Left135, -Right135);
		}

		TEST_METHOD(RelativeDirectionInvertTest5)
		{
			Assert::AreEqual(Reverse, -Reverse);
		}

		TEST_METHOD(RelativeDirectionInvertTest6)
		{
			Assert::AreEqual(Right45, -Left45);
		}

		TEST_METHOD(RelativeDirectionInvertTest7)
		{
			Assert::AreEqual(Right90, -Left90);
		}

		TEST_METHOD(RelativeDirectionInvertTest8)
		{
			Assert::AreEqual(Right135, -Left135);
		}

	};
}
