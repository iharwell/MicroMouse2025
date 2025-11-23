#include "pch.h"
#include "CppUnitTest.h"
#include "Templates.h"
#include "..\MazeMap\Direction.h"
#include "../MazeMap/MazeLocation.h"
#include "../MazeMap/DirectionalLocation.h"
using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
	TEST_CLASS(RelativeDirectionalDistanceTest)
	{
	public:

		void CheckMove(DirectionalLocation loc, RelativeDirectionalDistance rdd)
		{
			auto result1 = loc >> rdd;
			auto result2 = loc.Turn(rdd.GetDirection()).MoveForward(rdd.GetDistance());
			auto result3 = loc.Turn(rdd.GetDirection());
			for (uint8_t i = 0; i < rdd.GetDistance(); i++)
			{
				result3 = result3.MoveForward(1);
			}

			Assert::AreEqual(result1, result2, L"Operator does not match turn plus move.");
			Assert::AreEqual(result1, result3, L"Operator does not match turn plus iterated move.");
		}

		TEST_METHOD(Move0Test0)
		{
			DirectionalLocation loc(15, 15, Up);
			RelativeDirectionalDistance rdd(Forward, 0);
			CheckMove(loc, rdd);
		}

		TEST_METHOD(Move0Test1)
		{
			DirectionalLocation loc(15, 15, Up);
			RelativeDirectionalDistance rdd(Right45, 0);
			CheckMove(loc, rdd);
		}

		TEST_METHOD(Move0Test2)
		{
			DirectionalLocation loc(15, 15, Up);
			RelativeDirectionalDistance rdd(Left45, 0);
			CheckMove(loc, rdd);
		}

		TEST_METHOD(Move0Test3)
		{
			DirectionalLocation loc(15, 15, Up);
			RelativeDirectionalDistance rdd(Right90, 0);
			CheckMove(loc, rdd);
		}

		TEST_METHOD(Move0Test4)
		{
			DirectionalLocation loc(15, 15, Up);
			RelativeDirectionalDistance rdd(Left90, 0);
			CheckMove(loc, rdd);
		}

		TEST_METHOD(Move0Test5)
		{
			DirectionalLocation loc(15, 15, Up);
			RelativeDirectionalDistance rdd(Right135, 0);
			CheckMove(loc, rdd);
		}

		TEST_METHOD(Move0Test6)
		{
			DirectionalLocation loc(15, 15, Up);
			RelativeDirectionalDistance rdd(Left135, 0);
			CheckMove(loc, rdd);
		}

		TEST_METHOD(Move0Test7)
		{
			DirectionalLocation loc(15, 15, Up);
			RelativeDirectionalDistance rdd(Reverse, 0);
			CheckMove(loc, rdd);
		}

		TEST_METHOD(Move1Test0)
		{
			DirectionalLocation loc(15, 15, Up);
			RelativeDirectionalDistance rdd(Forward, 1);
			CheckMove(loc, rdd);
		}

		TEST_METHOD(Move1Test1)
		{
			DirectionalLocation loc(15, 15, Up);
			RelativeDirectionalDistance rdd(Right45, 1);
			CheckMove(loc, rdd);
		}

		TEST_METHOD(Move1Test2)
		{
			DirectionalLocation loc(15, 15, Up);
			RelativeDirectionalDistance rdd(Left45, 1);
			CheckMove(loc, rdd);
		}

		TEST_METHOD(Move1Test3)
		{
			DirectionalLocation loc(15, 15, Up);
			RelativeDirectionalDistance rdd(Right90, 1);
			CheckMove(loc, rdd);
		}

		TEST_METHOD(Move1Test4)
		{
			DirectionalLocation loc(15, 15, Up);
			RelativeDirectionalDistance rdd(Left90, 1);
			CheckMove(loc, rdd);
		}

		TEST_METHOD(Move1Test5)
		{
			DirectionalLocation loc(15, 15, Up);
			RelativeDirectionalDistance rdd(Right135, 1);
			CheckMove(loc, rdd);
		}

		TEST_METHOD(Move1Test6)
		{
			DirectionalLocation loc(15, 15, Up);
			RelativeDirectionalDistance rdd(Left135, 1);
			CheckMove(loc, rdd);
		}

		TEST_METHOD(Move1Test7)
		{
			DirectionalLocation loc(15, 15, Up);
			RelativeDirectionalDistance rdd(Reverse, 1);
			CheckMove(loc, rdd);
		}

		TEST_METHOD(Move2Test0)
		{
			DirectionalLocation loc(15, 15, Up);
			RelativeDirectionalDistance rdd(Forward, 2);
			CheckMove(loc, rdd);
		}

		TEST_METHOD(Move2Test1)
		{
			DirectionalLocation loc(15, 15, Up);
			RelativeDirectionalDistance rdd(Right45, 2);
			CheckMove(loc, rdd);
		}

		TEST_METHOD(Move2Test2)
		{
			DirectionalLocation loc(15, 15, Up);
			RelativeDirectionalDistance rdd(Left45, 2);
			CheckMove(loc, rdd);
		}

		TEST_METHOD(Move2Test3)
		{
			DirectionalLocation loc(15, 15, Up);
			RelativeDirectionalDistance rdd(Right90, 2);
			CheckMove(loc, rdd);
		}

		TEST_METHOD(Move2Test4)
		{
			DirectionalLocation loc(15, 15, Up);
			RelativeDirectionalDistance rdd(Left90, 2);
			CheckMove(loc, rdd);
		}

		TEST_METHOD(Move2Test5)
		{
			DirectionalLocation loc(15, 15, Up);
			RelativeDirectionalDistance rdd(Right135, 2);
			CheckMove(loc, rdd);
		}

		TEST_METHOD(Move2Test6)
		{
			DirectionalLocation loc(15, 15, Up);
			RelativeDirectionalDistance rdd(Left135, 2);
			CheckMove(loc, rdd);
		}

		TEST_METHOD(Move2Test7)
		{
			DirectionalLocation loc(15, 15, Up);
			RelativeDirectionalDistance rdd(Reverse, 2);
			CheckMove(loc, rdd);
		}

		TEST_METHOD(WalkingTest1)
		{
			DirectionalLocation loc(15, 15, Up);
			RelativeDirectionalDistance rdd(Reverse, 2);

			for (uint8_t i = 0; i < 8; i++)
			{
				rdd.SetDistance(i);
				RelativeDirection rd = Forward;
				for (uint8_t k = 0; k < 8; ++k)
				{
					rd = rd + Right45;
					rdd.SetDirection(rd);

					int steps = 0;

					DirectionalLocation expected = loc >> rdd;

					DirectionalLocation actual = loc;
					actual = actual.Turn(rdd.GetDirection());

					for (uint8_t j = 0; j < rdd.GetDistance(); ++j)
					{
						actual = actual.MoveForward(1);
						++steps;
					}

					Assert::AreEqual(expected, actual);
					Assert::AreEqual((int)rdd.GetDistance(), steps);
				}
			}

		}
	};
}