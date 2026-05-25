#include "pch.h"
#include "CppUnitTest.h"

#include "..\MazeMap\MotorEncoderDrive.h"
#include "..\MazeMap\Vehicle.h"

#include <sstream>
#include <string>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    namespace
    {
        constexpr float kWheelBankSpeedMagnitudeRadps = 24.0f;
        constexpr float kCommandTolerance = 1.0e-5f;
        constexpr float kLowWindowPosition = 0.25f;
        constexpr float kMidWindowPosition = 0.50f;
        constexpr float kHighWindowPosition = 0.75f;

        struct CommandWindow final
        {
            float deadbandCommand = 0.0f;
            float saturationCommand = 0.0f;
        };

        struct RoundTripSample final
        {
            const wchar_t* name = L"";
            float command = 0.0f;
            float roundTripCommand = 0.0f;
            float torqueNm = 0.0f;
            float deadbandCommand = 0.0f;
            float saturationCommand = 0.0f;
            float wheelBankSpeedRadps = 0.0f;
        };

        class MotorDriveFixture final
        {
        public:
            RoundTripSample BuildSample(
                const wchar_t* name,
                const MotorEncoderDrive& drive,
                float commandSign,
                float windowPosition) const noexcept
            {
                const CommandWindow window = BuildCommandWindow(drive, commandSign);
                const float command =
                    window.deadbandCommand +
                    (windowPosition * (window.saturationCommand - window.deadbandCommand));
                const float wheelBankSpeedRadps = commandSign * kWheelBankSpeedMagnitudeRadps;
                const float torqueNm =
                    drive.getTorqueFromCommand(command, wheelBankSpeedRadps, _vehicle.GetBatteryVoltage());
                const float roundTripCommand =
                    drive.getCommandFromTorque(torqueNm, wheelBankSpeedRadps, _vehicle.GetBatteryVoltage());

                return RoundTripSample{
                    name,
                    command,
                    roundTripCommand,
                    torqueNm,
                    window.deadbandCommand,
                    window.saturationCommand,
                    wheelBankSpeedRadps };
            }

            const MotorEncoderDrive& LeftDrive() const noexcept
            {
                return _vehicle.GetLeftMotorEncoderDrive();
            }

            const MotorEncoderDrive& RightDrive() const noexcept
            {
                return _vehicle.GetRightMotorEncoderDrive();
            }

        private:
            CommandWindow BuildCommandWindow(const MotorEncoderDrive& drive, float commandSign) const noexcept
            {
                const float wheelBankSpeedRadps = commandSign * kWheelBankSpeedMagnitudeRadps;
                const float groundSpeedMps =
                    Vehicle::WheelLinearVelocityFromWheelSpeed(wheelBankSpeedRadps);
                const float deadbandCommand =
                    drive.getDriveCommandForGroundForce(
                        0.0f,
                        groundSpeedMps);
                const float saturationForceN =
                    drive.getForwardForceFromCommand(
                        commandSign,
                        wheelBankSpeedRadps,
                        _vehicle.GetBatteryVoltage());
                const float saturationCommand =
                    drive.getDriveCommandForGroundForce(
                        saturationForceN,
                        groundSpeedMps);

                return CommandWindow{ deadbandCommand, saturationCommand };
            }

            Vehicle _vehicle{};
        };

        std::wstring BuildRoundTripMessage(const RoundTripSample& sample)
        {
            std::wstringstream message;
            message
                << sample.name
                << L": command round trip mismatch"
                << L"; command=" << sample.command
                << L"; round_trip=" << sample.roundTripCommand
                << L"; torque_nm=" << sample.torqueNm
                << L"; deadband=" << sample.deadbandCommand
                << L"; saturation=" << sample.saturationCommand
                << L"; wheel_speed_radps=" << sample.wheelBankSpeedRadps;
            return message.str();
        }
    }

    TEST_CLASS(MotorEncoderDriveTest)
    {
    public:
        TEST_METHOD(LeftPosMid)
        {
            const MotorDriveFixture fixture;
            const RoundTripSample sample =
                fixture.BuildSample(L"LeftPosMid", fixture.LeftDrive(), 1.0f, kMidWindowPosition);
            const std::wstring message = BuildRoundTripMessage(sample);

            Assert::AreEqual(
                sample.command,
                sample.roundTripCommand,
                kCommandTolerance,
                message.c_str());
        }

        TEST_METHOD(LeftPosLow)
        {
            const MotorDriveFixture fixture;
            const RoundTripSample sample =
                fixture.BuildSample(L"LeftPosLow", fixture.LeftDrive(), 1.0f, kLowWindowPosition);
            const std::wstring message = BuildRoundTripMessage(sample);

            Assert::AreEqual(
                sample.command,
                sample.roundTripCommand,
                kCommandTolerance,
                message.c_str());
        }

        TEST_METHOD(LeftPosHigh)
        {
            const MotorDriveFixture fixture;
            const RoundTripSample sample =
                fixture.BuildSample(L"LeftPosHigh", fixture.LeftDrive(), 1.0f, kHighWindowPosition);
            const std::wstring message = BuildRoundTripMessage(sample);

            Assert::AreEqual(
                sample.command,
                sample.roundTripCommand,
                kCommandTolerance,
                message.c_str());
        }

        TEST_METHOD(LeftNegMid)
        {
            const MotorDriveFixture fixture;
            const RoundTripSample sample =
                fixture.BuildSample(L"LeftNegMid", fixture.LeftDrive(), -1.0f, kMidWindowPosition);
            const std::wstring message = BuildRoundTripMessage(sample);

            Assert::AreEqual(
                sample.command,
                sample.roundTripCommand,
                kCommandTolerance,
                message.c_str());
        }

        TEST_METHOD(LeftNegLow)
        {
            const MotorDriveFixture fixture;
            const RoundTripSample sample =
                fixture.BuildSample(L"LeftNegLow", fixture.LeftDrive(), -1.0f, kLowWindowPosition);
            const std::wstring message = BuildRoundTripMessage(sample);

            Assert::AreEqual(
                sample.command,
                sample.roundTripCommand,
                kCommandTolerance,
                message.c_str());
        }

        TEST_METHOD(LeftNegHigh)
        {
            const MotorDriveFixture fixture;
            const RoundTripSample sample =
                fixture.BuildSample(L"LeftNegHigh", fixture.LeftDrive(), -1.0f, kHighWindowPosition);
            const std::wstring message = BuildRoundTripMessage(sample);

            Assert::AreEqual(
                sample.command,
                sample.roundTripCommand,
                kCommandTolerance,
                message.c_str());
        }

        TEST_METHOD(RightPosMid)
        {
            const MotorDriveFixture fixture;
            const RoundTripSample sample =
                fixture.BuildSample(L"RightPosMid", fixture.RightDrive(), 1.0f, kMidWindowPosition);
            const std::wstring message = BuildRoundTripMessage(sample);

            Assert::AreEqual(
                sample.command,
                sample.roundTripCommand,
                kCommandTolerance,
                message.c_str());
        }

        TEST_METHOD(RightPosLow)
        {
            const MotorDriveFixture fixture;
            const RoundTripSample sample =
                fixture.BuildSample(L"RightPosLow", fixture.RightDrive(), 1.0f, kLowWindowPosition);
            const std::wstring message = BuildRoundTripMessage(sample);

            Assert::AreEqual(
                sample.command,
                sample.roundTripCommand,
                kCommandTolerance,
                message.c_str());
        }

        TEST_METHOD(RightPosHigh)
        {
            const MotorDriveFixture fixture;
            const RoundTripSample sample =
                fixture.BuildSample(L"RightPosHigh", fixture.RightDrive(), 1.0f, kHighWindowPosition);
            const std::wstring message = BuildRoundTripMessage(sample);

            Assert::AreEqual(
                sample.command,
                sample.roundTripCommand,
                kCommandTolerance,
                message.c_str());
        }

        TEST_METHOD(RightNegMid)
        {
            const MotorDriveFixture fixture;
            const RoundTripSample sample =
                fixture.BuildSample(L"RightNegMid", fixture.RightDrive(), -1.0f, kMidWindowPosition);
            const std::wstring message = BuildRoundTripMessage(sample);

            Assert::AreEqual(
                sample.command,
                sample.roundTripCommand,
                kCommandTolerance,
                message.c_str());
        }

        TEST_METHOD(RightNegLow)
        {
            const MotorDriveFixture fixture;
            const RoundTripSample sample =
                fixture.BuildSample(L"RightNegLow", fixture.RightDrive(), -1.0f, kLowWindowPosition);
            const std::wstring message = BuildRoundTripMessage(sample);

            Assert::AreEqual(
                sample.command,
                sample.roundTripCommand,
                kCommandTolerance,
                message.c_str());
        }

        TEST_METHOD(RightNegHigh)
        {
            const MotorDriveFixture fixture;
            const RoundTripSample sample =
                fixture.BuildSample(L"RightNegHigh", fixture.RightDrive(), -1.0f, kHighWindowPosition);
            const std::wstring message = BuildRoundTripMessage(sample);

            Assert::AreEqual(
                sample.command,
                sample.roundTripCommand,
                kCommandTolerance,
                message.c_str());
        }
    };
}
