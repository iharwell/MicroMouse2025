#include "pch.h"
#include "CppUnitTest.h"

#include "..\MazeMap\CommandVector.h"

#include <cmath>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    TEST_CLASS(CommandVectorTest)
    {
    public:
        TEST_METHOD(ConstructorStoresWheelCommands)
        {
            const App::Internal::CommandVector command(0.25f, -0.50f);

            Assert::AreEqual(0.25f, command.LeftMotorPwm(), 1.0e-6f);
            Assert::AreEqual(-0.50f, command.RightMotorPwm(), 1.0e-6f);
            Assert::IsTrue(command.IsFinite());
        }

        TEST_METHOD(AverageAndDifferentialAccessorsMapToWheelCommands)
        {
            App::Internal::CommandVector command(0.40f, 0.20f);

            Assert::AreEqual(0.30f, command.Average(), 1.0e-6f);
            Assert::AreEqual(0.10f, command.Differential(), 1.0e-6f);

            command.SetAverage(0.60f);
            Assert::AreEqual(0.70f, command.LeftMotorPwm(), 1.0e-6f);
            Assert::AreEqual(0.50f, command.RightMotorPwm(), 1.0e-6f);
            Assert::AreEqual(0.10f, command.Differential(), 1.0e-6f);

            command.SetDifferential(-0.20f);
            Assert::AreEqual(0.40f, command.LeftMotorPwm(), 1.0e-6f);
            Assert::AreEqual(0.80f, command.RightMotorPwm(), 1.0e-6f);
            Assert::AreEqual(0.60f, command.Average(), 1.0e-6f);

            const App::Internal::CommandVector rebuilt =
                App::Internal::CommandVector::FromAverageAndDifferential(0.50f, 0.25f);
            Assert::AreEqual(0.75f, rebuilt.LeftMotorPwm(), 1.0e-6f);
            Assert::AreEqual(0.25f, rebuilt.RightMotorPwm(), 1.0e-6f);
        }

        TEST_METHOD(OperatorsComposeWheelCommands)
        {
            const App::Internal::CommandVector lhs(0.20f, 0.10f);
            const App::Internal::CommandVector rhs(0.05f, -0.20f);

            App::Internal::CommandVector sum = lhs + rhs;
            Assert::AreEqual(0.25f, sum.LeftMotorPwm(), 1.0e-6f);
            Assert::AreEqual(-0.10f, sum.RightMotorPwm(), 1.0e-6f);

            sum -= rhs;
            Assert::AreEqual(lhs.LeftMotorPwm(), sum.LeftMotorPwm(), 1.0e-6f);
            Assert::AreEqual(lhs.RightMotorPwm(), sum.RightMotorPwm(), 1.0e-6f);

            const App::Internal::CommandVector scaled = 2.0f * lhs;
            Assert::AreEqual(0.40f, scaled.LeftMotorPwm(), 1.0e-6f);
            Assert::AreEqual(0.20f, scaled.RightMotorPwm(), 1.0e-6f);

            const App::Internal::CommandVector divided = scaled / 2.0f;
            Assert::IsTrue(divided == lhs);
        }

        TEST_METHOD(BrakeCommandUsesNonFinitePwmVocabulary)
        {
            Assert::IsFalse(App::Internal::CommandVector::Brake().IsFinite());
            Assert::IsFalse(std::isfinite(App::Internal::CommandVector::Brake().LeftMotorPwm()));
            Assert::IsFalse(std::isfinite(App::Internal::CommandVector::Brake().RightMotorPwm()));
        }
    };
}
