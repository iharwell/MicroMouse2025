#include "pch.h"
#include "CppUnitTest.h"

#include "..\MazeMap\LoopController.h"

#include <cmath>
#include <type_traits>

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

        TEST_METHOD(StageNextSessionStateRequiresExplicitSessionStartPoint)
        {
            Assert::IsFalse((
                std::is_invocable_v<
                    decltype(&MazeMap::App::Internal::LoopController::StageNextSessionState),
                    MazeMap::App::Internal::LoopController*,
                    std::uint32_t>));
            Assert::IsTrue((
                std::is_invocable_v<
                    decltype(&MazeMap::App::Internal::LoopController::StageNextSessionState),
                    MazeMap::App::Internal::LoopController*,
                    std::uint32_t,
                    float,
                    float,
                    MazeMap::App::Internal::LoopController::WallMask,
                    bool,
                    bool,
                    bool,
                    bool>));
        }
    };
}

