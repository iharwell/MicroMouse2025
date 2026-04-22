#pragma once

#include "CppUnitTest.h"
#include "TimeStepPropagationTestSupport.h"

#include "..\MazeMap\PlantModel.h"

#include <cstddef>

namespace MazeMap
{
    inline void AssertStateVectorNear(
        const VehicleState::StateVector& expected,
        const VehicleState::StateVector& actual,
        float tolerance)
    {
        for (int index = 0; index < expected.size(); ++index)
        {
            Microsoft::VisualStudio::CppUnitTestFramework::Assert::AreEqual(expected(index), actual(index), tolerance);
        }
    }

    inline void AssertWheelKinematicsNear(
        const WheelKinematics& expected,
        const WheelKinematics& actual,
        float tolerance)
    {
        Microsoft::VisualStudio::CppUnitTestFramework::Assert::AreEqual(
            expected.leftBankForwardVelocityMps,
            actual.leftBankForwardVelocityMps,
            tolerance);
        Microsoft::VisualStudio::CppUnitTestFramework::Assert::AreEqual(
            expected.rightBankForwardVelocityMps,
            actual.rightBankForwardVelocityMps,
            tolerance);
        for (std::size_t index = 0; index < expected.contacts.size(); ++index)
        {
            Microsoft::VisualStudio::CppUnitTestFramework::Assert::AreEqual(
                expected.contacts[index].rightVelocityMps,
                actual.contacts[index].rightVelocityMps,
                tolerance);
            Microsoft::VisualStudio::CppUnitTestFramework::Assert::AreEqual(
                expected.contacts[index].forwardVelocityMps,
                actual.contacts[index].forwardVelocityMps,
                tolerance);
        }
    }

    inline void AssertSlipTargetsNear(
        const SlipTargets& expected,
        const SlipTargets& actual,
        float tolerance)
    {
        Microsoft::VisualStudio::CppUnitTestFramework::Assert::AreEqual(expected.kappaLeft, actual.kappaLeft, tolerance);
        Microsoft::VisualStudio::CppUnitTestFramework::Assert::AreEqual(expected.kappaRight, actual.kappaRight, tolerance);
        for (std::size_t index = 0; index < expected.lateralRatio.size(); ++index)
        {
            Microsoft::VisualStudio::CppUnitTestFramework::Assert::AreEqual(
                expected.lateralRatio[index],
                actual.lateralRatio[index],
                tolerance);
        }
    }

    inline void AssertContactForcesNear(
        const ContactForces& expected,
        const ContactForces& actual,
        float tolerance)
    {
        for (std::size_t index = 0; index < expected.contacts.size(); ++index)
        {
            Microsoft::VisualStudio::CppUnitTestFramework::Assert::AreEqual(
                expected.contacts[index].rightForceN,
                actual.contacts[index].rightForceN,
                tolerance);
            Microsoft::VisualStudio::CppUnitTestFramework::Assert::AreEqual(
                expected.contacts[index].forwardForceN,
                actual.contacts[index].forwardForceN,
                tolerance);
            Microsoft::VisualStudio::CppUnitTestFramework::Assert::AreEqual(
                expected.contacts[index].normalForceN,
                actual.contacts[index].normalForceN,
                tolerance);
            Microsoft::VisualStudio::CppUnitTestFramework::Assert::AreEqual(
                expected.contacts[index].saturation,
                actual.contacts[index].saturation,
                tolerance);
            Microsoft::VisualStudio::CppUnitTestFramework::Assert::AreEqual(
                expected.contacts[index].preProjectionUtilization,
                actual.contacts[index].preProjectionUtilization,
                tolerance);
        }
    }

    inline void AssertPlantDerivativesNear(
        const PlantDerivatives& expected,
        const PlantDerivatives& actual,
        float tolerance)
    {
        AssertStateVectorNear(expected.stateDot, actual.stateDot, tolerance);
        AssertContactForcesNear(expected.contactForces, actual.contactForces, tolerance);
        AssertWheelKinematicsNear(expected.wheelKinematics, actual.wheelKinematics, tolerance);
        AssertSlipTargetsNear(expected.slipTargets, actual.slipTargets, tolerance);
        Microsoft::VisualStudio::CppUnitTestFramework::Assert::AreEqual(
            expected.originAccelBodyMps2.x(),
            actual.originAccelBodyMps2.x(),
            tolerance);
        Microsoft::VisualStudio::CppUnitTestFramework::Assert::AreEqual(
            expected.originAccelBodyMps2.y(),
            actual.originAccelBodyMps2.y(),
            tolerance);
        Microsoft::VisualStudio::CppUnitTestFramework::Assert::AreEqual(
            expected.imuAccelBodyMps2.x(),
            actual.imuAccelBodyMps2.x(),
            tolerance);
        Microsoft::VisualStudio::CppUnitTestFramework::Assert::AreEqual(
            expected.imuAccelBodyMps2.y(),
            actual.imuAccelBodyMps2.y(),
            tolerance);
        Microsoft::VisualStudio::CppUnitTestFramework::Assert::AreEqual(
            expected.longitudinalAccelMps2,
            actual.longitudinalAccelMps2,
            tolerance);
        Microsoft::VisualStudio::CppUnitTestFramework::Assert::AreEqual(
            expected.lateralAccelMps2,
            actual.lateralAccelMps2,
            tolerance);
        Microsoft::VisualStudio::CppUnitTestFramework::Assert::AreEqual(
            expected.yawAccelRadps2,
            actual.yawAccelRadps2,
            tolerance);
        Microsoft::VisualStudio::CppUnitTestFramework::Assert::AreEqual(
            static_cast<int>(expected.regime),
            static_cast<int>(actual.regime));
        Microsoft::VisualStudio::CppUnitTestFramework::Assert::AreEqual(
            expected.maxContactUtilization,
            actual.maxContactUtilization,
            tolerance);
    }

    inline void AssertDriveCommandSolutionNear(
        const DriveCommandSolution& expected,
        const DriveCommandSolution& actual,
        float tolerance)
    {
        Microsoft::VisualStudio::CppUnitTestFramework::Assert::AreEqual(
            expected.control.leftMotorCommand,
            actual.control.leftMotorCommand,
            tolerance);
        Microsoft::VisualStudio::CppUnitTestFramework::Assert::AreEqual(
            expected.control.rightMotorCommand,
            actual.control.rightMotorCommand,
            tolerance);
        Microsoft::VisualStudio::CppUnitTestFramework::Assert::AreEqual(
            expected.control.fanDutyCycle,
            actual.control.fanDutyCycle,
            tolerance);
        Microsoft::VisualStudio::CppUnitTestFramework::Assert::AreEqual(
            expected.control.batteryVoltageV,
            actual.control.batteryVoltageV,
            tolerance);
        Microsoft::VisualStudio::CppUnitTestFramework::Assert::AreEqual(expected.leftSlipRatio, actual.leftSlipRatio, tolerance);
        Microsoft::VisualStudio::CppUnitTestFramework::Assert::AreEqual(expected.rightSlipRatio, actual.rightSlipRatio, tolerance);
        Microsoft::VisualStudio::CppUnitTestFramework::Assert::AreEqual(expected.leftWheelSpeedRadps, actual.leftWheelSpeedRadps, tolerance);
        Microsoft::VisualStudio::CppUnitTestFramework::Assert::AreEqual(expected.rightWheelSpeedRadps, actual.rightWheelSpeedRadps, tolerance);
        Microsoft::VisualStudio::CppUnitTestFramework::Assert::AreEqual(
            expected.leftRollingWheelSpeedRadps,
            actual.leftRollingWheelSpeedRadps,
            tolerance);
        Microsoft::VisualStudio::CppUnitTestFramework::Assert::AreEqual(
            expected.rightRollingWheelSpeedRadps,
            actual.rightRollingWheelSpeedRadps,
            tolerance);
        Microsoft::VisualStudio::CppUnitTestFramework::Assert::AreEqual(expected.leftWheelTorqueNm, actual.leftWheelTorqueNm, tolerance);
        Microsoft::VisualStudio::CppUnitTestFramework::Assert::AreEqual(expected.rightWheelTorqueNm, actual.rightWheelTorqueNm, tolerance);
        Microsoft::VisualStudio::CppUnitTestFramework::Assert::AreEqual(
            expected.leftWheelAccelRadps2,
            actual.leftWheelAccelRadps2,
            tolerance);
        Microsoft::VisualStudio::CppUnitTestFramework::Assert::AreEqual(
            expected.rightWheelAccelRadps2,
            actual.rightWheelAccelRadps2,
            tolerance);
        Microsoft::VisualStudio::CppUnitTestFramework::Assert::AreEqual(expected.leftContactForceN, actual.leftContactForceN, tolerance);
        Microsoft::VisualStudio::CppUnitTestFramework::Assert::AreEqual(expected.rightContactForceN, actual.rightContactForceN, tolerance);
        Microsoft::VisualStudio::CppUnitTestFramework::Assert::AreEqual(expected.leftContactTorqueNm, actual.leftContactTorqueNm, tolerance);
        Microsoft::VisualStudio::CppUnitTestFramework::Assert::AreEqual(expected.rightContactTorqueNm, actual.rightContactTorqueNm, tolerance);
        Microsoft::VisualStudio::CppUnitTestFramework::Assert::AreEqual(expected.tractionScale, actual.tractionScale, tolerance);
        Microsoft::VisualStudio::CppUnitTestFramework::Assert::AreEqual(
            static_cast<int>(expected.tractionLimited),
            static_cast<int>(actual.tractionLimited));
        Microsoft::VisualStudio::CppUnitTestFramework::Assert::AreEqual(
            expected.commandedLongitudinalAccelMps2,
            actual.commandedLongitudinalAccelMps2,
            tolerance);
        Microsoft::VisualStudio::CppUnitTestFramework::Assert::AreEqual(
            expected.commandedYawAccelRadps2,
            actual.commandedYawAccelRadps2,
            tolerance);
        Microsoft::VisualStudio::CppUnitTestFramework::Assert::AreEqual(
            expected.longitudinalAccelErrorMps2,
            actual.longitudinalAccelErrorMps2,
            tolerance);
        Microsoft::VisualStudio::CppUnitTestFramework::Assert::AreEqual(
            expected.yawAccelErrorRadps2,
            actual.yawAccelErrorRadps2,
            tolerance);
        Microsoft::VisualStudio::CppUnitTestFramework::Assert::AreEqual(
            static_cast<int>(expected.converged),
            static_cast<int>(actual.converged));
    }

}
