#pragma once

#include "CppUnitTest.h"
#include "TimeStepPropagationTestSupport.h"

#include "..\MazeMap\PlantModel.h"
#include "..\MazeMap\Vehicle.h"

#include <cstddef>

namespace MazeMap
{
    struct PlantModelTestRuntime final
    {
        Vehicle vehicle{};
        VehicleState runtimeState{};
        PlantModel plant;

        PlantModelTestRuntime() noexcept
            : plant(vehicle, runtimeState)
        {
            vehicle.SetFanDuty(0.80f);
        }
    };

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

}


