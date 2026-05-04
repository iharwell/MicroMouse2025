#include "pch.h"
#include "CppUnitTest.h"

#include "..\MazeMap\LoopController.h"

#include <cmath>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    TEST_CLASS(LoopControllerTest)
    {
    public:
        TEST_METHOD(LoopControllerStartsModeCallbackOnFirstTick)
        {
            Assert::AreEqual(
                1U,
                MazeMap::App::Internal::LoopController::kInitialModeCallbackTick);
        }

        TEST_METHOD(SessionOptionsRequireExplicitSessionStartPoint)
        {
            MazeMap::App::Internal::LoopController::SessionOptions options{};
            Assert::IsFalse(std::isfinite(options.SessionStartPointX));
            Assert::IsFalse(std::isfinite(options.SessionStartPointY));
        }
    };
}
