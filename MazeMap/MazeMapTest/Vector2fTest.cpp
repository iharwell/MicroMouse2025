#include "pch.h"
#include "CppUnitTest.h"

#include <Eigen/Core>
#include <cmath>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
	static constexpr float kPi = 3.14159265358979323846f;

	TEST_CLASS(Vector2fTest)
	{
	public:
		TEST_METHOD(MatrixVectorMultiplyUsesEigen)
		{
			Eigen::Matrix2f matrix;
			matrix << 0.0f, 1.0f,
				1.0f, 0.0f;
			const Eigen::Vector2f vector(3.0f, 4.0f);
			const Eigen::Vector2f result = matrix * vector;

			Assert::AreEqual(vector.x(), result.y());
			Assert::AreEqual(vector.y(), result.x());
		}

		TEST_METHOD(VectorDefaultConstructsAndAssigns)
		{
			Eigen::Vector2f vector = Eigen::Vector2f::Zero();
			Assert::AreEqual(0.0f, vector.x());
			Assert::AreEqual(0.0f, vector.y());

			vector = Eigen::Vector2f(1.0f, 2.0f);
			Assert::AreEqual(1.0f, vector.x());
			Assert::AreEqual(2.0f, vector.y());
		}

		TEST_METHOD(VectorNormAndSquaredNormUseEigen)
		{
			const Eigen::Vector2f vector(-1.0f, 2.0f);
			Assert::AreEqual(5.0f, vector.squaredNorm());

			const Eigen::Vector2f other(3.0f, 4.0f);
			Assert::AreEqual(5.0f, other.norm());
		}

		TEST_METHOD(VectorAngleCalculationsUseDirectEigenComponents)
		{
			const Eigen::Vector2f v1(2.0f, 2.0f);
			const Eigen::Vector2f v2(0.0f, 2.0f);

			const float angle1 = std::atan2(v1.y(), v1.x());
			const float angle12 = std::atan2((v1.x() * v2.y()) - (v1.y() * v2.x()), v1.dot(v2));
			const float angle21 = std::atan2((v2.x() * v1.y()) - (v2.y() * v1.x()), v2.dot(v1));

			Assert::AreEqual(kPi / 4.0f, angle1, 0.0001f);
			Assert::AreEqual(kPi / 4.0f, angle12, 0.0001f);
			Assert::AreEqual(-kPi / 4.0f, angle21, 0.0001f);
		}

		TEST_METHOD(RotationViaNormalizedEigenBasisPreservesMagnitude)
		{
			const Eigen::Vector2f vector(3.0f, 2.0f);
			const Eigen::Vector2f rotation(2.0f, 3.0f);
			const float rotationNorm = rotation.norm();

			Eigen::Matrix2f basis;
			basis << rotation.x() / rotationNorm, -rotation.y() / rotationNorm,
				rotation.y() / rotationNorm, rotation.x() / rotationNorm;

			const Eigen::Vector2f result = basis * vector;
			const float resultAngle = std::atan2(result.y(), result.x());

			Assert::AreEqual(kPi / 2.0f, resultAngle, 0.0001f);
			Assert::AreEqual(vector.norm(), result.norm(), 0.0001f);
		}

		TEST_METHOD(VectorArithmeticAndDotProductUseEigenOperators)
		{
			Eigen::Vector2f v1(1.0f, 2.0f);
			const Eigen::Vector2f v2(3.0f, 4.0f);

			v1 += Eigen::Vector2f(1.0f, 0.0f);
			Assert::AreEqual(2.0f, v1.x());
			Assert::AreEqual(2.0f, v1.y());

			v1 -= Eigen::Vector2f(1.0f, 0.0f);
			Assert::AreEqual(1.0f, v1.x());
			Assert::AreEqual(2.0f, v1.y());

			v1 *= 2.0f;
			Assert::AreEqual(2.0f, v1.x());
			Assert::AreEqual(4.0f, v1.y());

			v1 /= 2.0f;
			Assert::AreEqual(1.0f, v1.x());
			Assert::AreEqual(2.0f, v1.y());

			Assert::AreEqual(11.0f, v1.dot(v2));
		}
	};
}

