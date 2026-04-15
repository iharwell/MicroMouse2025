#include "pch.h"
#include "CppUnitTest.h"

#include "..\MazeMap\DriveBase.h"
#include "..\MazeMap\OpenLoopDriveCommand.h"

#include <cstdint>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    namespace
    {
        constexpr std::uint8_t kDriveBaseRightEncoderChannel = 1U;
        constexpr std::uint8_t kDriveBaseLeftEncoderChannel = 2U;

        DiagnosticSensorSnapshot BuildDriveBaseDiagnosticSnapshot() noexcept
        {
            DiagnosticSensorSnapshot snapshot{};
            snapshot.gyroRawRadps = 0.0f;
            snapshot.gyroRadps = 0.0f;
            return snapshot;
        }

        void PrimeDriveBaseWithEncoderDelta(
            DriveBase& drive,
            const int32_t leftCounts,
            const int32_t rightCounts,
            const float dtSeconds = 0.001f)
        {
            MazeMap::Platform::WriteEncoderCount(kDriveBaseLeftEncoderChannel, leftCounts);
            MazeMap::Platform::WriteEncoderCount(kDriveBaseRightEncoderChannel, rightCounts);
            const DiagnosticSensorSnapshot snapshot = BuildDriveBaseDiagnosticSnapshot();
            drive.UpdateOdometry(dtSeconds, snapshot, nullptr, nullptr);
        }
    }

    TEST_CLASS(DriveBaseTest)
    {
    public:
        TEST_METHOD(DriveBaseDeltaCommandStaysSymmetricAcrossWheelSpeedMismatch)
        {
            DriveBase drive;
            Assert::IsTrue(drive.Begin());

            PrimeDriveBaseWithEncoderDelta(drive, 6, 42);

            const OpenLoopDriveCommand command =
                drive.DeltaCommand(
                    0.20f,
                    8.5f);

            Assert::IsTrue(IsFiniteOpenLoopDriveCommand(command));
            Assert::IsTrue(command.leftDriveCommand > 0.0f);
            Assert::AreEqual(command.leftDriveCommand, command.rightDriveCommand, 1.0e-6f);
        }

        TEST_METHOD(DriveBaseDeltaCommandHeadingHoldStaysSymmetricWhenAlreadyAligned)
        {
            DriveBase drive;
            Assert::IsTrue(drive.Begin());

            PrimeDriveBaseWithEncoderDelta(drive, 6, 42);

            const OpenLoopDriveCommand command =
                drive.DeltaCommand(
                    0.20f,
                    8.5f,
                    MazeMap::CommandPD::StateHeadingPD);

            Assert::IsTrue(IsFiniteOpenLoopDriveCommand(command));
            Assert::AreEqual(command.leftDriveCommand, command.rightDriveCommand, 1.0e-6f);
        }
    };
}
