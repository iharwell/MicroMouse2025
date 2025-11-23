#include "pch.h"
#include "CppUnitTest.h"
#include "..\MazeMap\Vector2f.h"
using namespace Microsoft::VisualStudio::CppUnitTestFramework;
#include <cmath>
const float PI = 3.14159265358979323846f;
namespace MazeMap
{
	TEST_CLASS(Vector2fTest)
	{
	public:

		TEST_METHOD(ConstructorTest1)
		{
			Vectorf<2> v = Vectorf<2>();
			Assert::AreEqual(0.0f, v.GetX());
			Assert::AreEqual(0.0f, v.GetY());
		}
		TEST_METHOD(ConstructorTest2)
		{
			Vectorf<2> v = Vectorf<2>(1.0f,2.0f);
			Assert::AreEqual(1.0f, v.GetX());
			Assert::AreEqual(2.0f, v.GetY());
		}

		TEST_METHOD(MagnitudeSquaredTest)
		{
			Vectorf<2> v1(-1.0f, 2.0f);
			float magsq = 5.0f;
			Assert::AreEqual(magsq, v1.GetMagnitudeSquared());
		}
		TEST_METHOD(MagnitudeTest)
		{
			Vectorf<2> v(3.0f, 4.0f);
			Assert::AreEqual(5.0f, v.GetMagnitude());
		}
		TEST_METHOD(AngleTest)
		{
			Vectorf<2> v(2.0f, 2.0f);
			Assert::AreEqual(PI/4, v.GetAngle());
		}
		TEST_METHOD(AngleToTest)
		{
			Vectorf<2> v1(2.0f, 2.0f);
			Vectorf<2> v2(0.0f, 2.0f);
			Assert::AreEqual(PI / 4, v1.AngleTo(v2));
			Assert::AreEqual(-PI / 4, v2.AngleTo(v1));
		}
		TEST_METHOD(RotateByTest)
		{
			Vectorf<2> v1(3.0f, 2.0f);
			Vectorf<2> v2(2.0f, 3.0f);
			Vectorf<2> result = v1.RotateBy(v2);

			Assert::AreEqual(PI / 2, result.GetAngle());
			Assert::AreEqual(v1.GetMagnitude(), result.GetMagnitude());
		}
		TEST_METHOD(IncrementTest)
		{
			Vectorf<2> v1(0.0f, 2.0f);
			Vectorf<2> v2(1.0f, 0.0f);
			v1 += v2;

			Assert::AreEqual(v2.GetX(), v1.GetX());
			Assert::AreEqual(2.0f, v1.GetY());
		}
		TEST_METHOD(DecrementTest)
		{
			Vectorf<2> v1(1.0f, 2.0f);
			Vectorf<2> v2(1.0f, 0.0f);
			v1 -= v2;

			Assert::AreEqual(0.0f, v1.GetX());
			Assert::AreEqual(2.0f, v1.GetY());
		}

		TEST_METHOD(MulSetTest)
		{
			Vectorf<2> v1(1.0f, 2.0f);
			float scale = 2.0f;
			v1 *= scale;

			Assert::AreEqual(2.0f, v1.GetX());
			Assert::AreEqual(4.0f, v1.GetY());
		}

		TEST_METHOD(DivSetTest)
		{
			Vectorf<2> v1(1.0f, 2.0f);
			float scale = 2.0f;
			v1 /= scale;

			Assert::AreEqual(0.5f, v1.GetX());
			Assert::AreEqual(1.0f, v1.GetY());
		}

		TEST_METHOD(DotProdTest)
		{
			Vectorf<2> v1(1.0f, 2.0f);
			Vectorf<2> v2(3.0f, 4.0f);
			float result = v1*v2;
			float expected = 3.0f + 8.0f;

			Assert::AreEqual(expected, result);
		}


	};
}