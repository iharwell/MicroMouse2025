#include "pch.h"
#include "..\MazeMap\Cell.h"
#include "CppUnitTest.h"

#include <sstream>
#include "Templates.h"
using namespace Microsoft::VisualStudio::CppUnitTestFramework;


namespace MazeMap
{
	TEST_CLASS(CellTest)
	{
	public:
		TEST_METHOD(TestX)
		{
			Cell c(3, 4);
			Assert::AreEqual((int)c.GetX(), 3, L"Construct and retrieve on X failed.");
		}
		TEST_METHOD(TestY)
		{
			Cell c(3, 4);
			Assert::AreEqual((int)c.GetY(), 4, L"Construct and retrieve on Y failed.");
		}
		TEST_METHOD(TestUp)
		{
			Cell c(3, 4, NoWall, NoWall, NoWall, NoWall);
			Assert::AreEqual(c.GetUp(), NoWall, L"Construct and retrieve Up failed.");

			c.SetUp(Unknown);
			Assert::AreEqual(c.GetUp(), Unknown, L"Set and retrieve Up failed.");
			
		}
		TEST_METHOD(TestDown)
		{
			Cell c(3, 4, NoWall, NoWall, NoWall, NoWall);
			Assert::AreEqual(NoWall, c.GetDown(), L"Construct and retrieve Down failed.");

			c.SetDown(Unknown);
			Assert::AreEqual( Unknown, c.GetDown(), L"Set and retrieve Down failed.");
		}
		TEST_METHOD(TestLeft)
		{
			Cell c(3, 4, NoWall, NoWall, NoWall, NoWall);
			Assert::AreEqual(c.GetLeft(), NoWall, L"Construct and retrieve Left failed.");

			c.SetLeft(Unknown);
			Assert::AreEqual(c.GetLeft(), Unknown, L"Set and retrieve Left failed.");
		}
		TEST_METHOD(TestRight)
		{
			Cell c(3, 4, NoWall, NoWall, NoWall, NoWall);
			Assert::AreEqual(c.GetRight(), NoWall, L"Construct and retrieve Right failed.");

			c.SetRight(Unknown);
			Assert::AreEqual(c.GetRight(), Unknown, L"Set and retrieve Right failed.");
		}
		TEST_METHOD(TestVisited)
		{
			Cell c = Cell(3, 4, Unknown, Unknown, Unknown, Unknown);
			Assert::IsFalse(c.IsFullyKnown());

			c = Cell(3, 4, NoWall, NoWall, NoWall, Unknown);
			Assert::IsFalse(c.IsFullyKnown());

			c = Cell(3, 4, NoWall, NoWall, Unknown, NoWall);
			Assert::IsFalse(c.IsFullyKnown());

			c = Cell(3, 4, NoWall, Unknown, NoWall, NoWall);
			Assert::IsFalse(c.IsFullyKnown());

			c = Cell(3, 4, Unknown, NoWall, NoWall, NoWall);
			Assert::IsFalse(c.IsFullyKnown());

			c = Cell(3, 4, NoWall, NoWall, NoWall, NoWall);
			Assert::IsTrue(c.IsFullyKnown());

			c = Cell(3, 4, Wall, Wall, Wall, Unknown);
			Assert::IsFalse(c.IsFullyKnown());

			c = Cell(3, 4, Wall, Wall, Unknown, Wall);
			Assert::IsFalse(c.IsFullyKnown());

			c = Cell(3, 4, Wall, Unknown, Wall, Wall);
			Assert::IsFalse(c.IsFullyKnown());

			c = Cell(3, 4, Unknown, Wall, Wall, Wall);
			Assert::IsFalse(c.IsFullyKnown());

			c = Cell(3, 4, Wall, Wall, Wall, Wall);
			Assert::IsTrue(c.IsFullyKnown());

			c = Cell(3, 4, Wall, Wall, Wall, NoWall);
			Assert::IsTrue(c.IsFullyKnown());

			c = Cell(3, 4, Wall, Wall, NoWall, Wall);
			Assert::IsTrue(c.IsFullyKnown());

			c = Cell(3, 4, Wall, NoWall, Wall, Wall);
			Assert::IsTrue(c.IsFullyKnown());

			c = Cell(3, 4, NoWall, Wall, Wall, Wall);
			Assert::IsTrue(c.IsFullyKnown());
		}
	};
}
