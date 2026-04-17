#include "pch.h"
#include "CppUnitTest.h"

#include "..\MazeMap\LoopController.h"

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
    };
}
