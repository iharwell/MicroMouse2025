#include "pch.h"
#include "CppUnitTest.h"

#include "EstimatorFilterTestSupport.h"
#include "EstimatorMotionUpdateAccess.h"
#include "TractionTestEstimator.h"

#include <algorithm>
#include <cmath>
#include <string>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    namespace
    {
        constexpr int kVf = 3;

        struct HookProbe final
        {
            int predictCalls = 0;
            int yawMeasurementCalls = 0;
            int imuMeasurementCalls = 0;
        };

        float MaxAbsStateDelta(
            const TractionTestEstimator::StateVector& lhs,
            const TractionTestEstimator::StateVector& rhs) noexcept
        {
            float maxAbs = 0.0f;
            for (int row = 0; row < VehicleState::kDimension; ++row)
            {
                maxAbs = (std::max)(maxAbs, std::fabs(lhs(row) - rhs(row)));
            }
            return maxAbs;
        }

        float MaxAbsCovarianceDelta(
            const TractionTestEstimator::CovarianceMatrix& lhs,
            const TractionTestEstimator::CovarianceMatrix& rhs) noexcept
        {
            float maxAbs = 0.0f;
            for (int row = 0; row < VehicleState::kDimension; ++row)
            {
                for (int col = 0; col < VehicleState::kDimension; ++col)
                {
                    maxAbs = (std::max)(maxAbs, std::fabs(lhs(row, col) - rhs(row, col)));
                }
            }
            return maxAbs;
        }

        TractionTestEstimator::StateVector PredictWithInjectedForwardVelocity(
            void* context,
            const PlantModel& productionPlant,
            const TractionTestEstimator::PredictionInput& input) noexcept
        {
            (void)productionPlant;
            HookProbe* const probe = static_cast<HookProbe*>(context);
            if (probe != nullptr)
            {
                ++probe->predictCalls;
            }

            TractionTestEstimator::StateVector state = input.sigmaPoint;
            state(kVf) += 2.0f * input.dtSeconds;
            return state;
        }

        Eigen::Vector2f FixedPlanarAccelerationMeasurement(
            void* context,
            const PlantModel& productionPlant,
            const TractionTestEstimator::MeasurementInput& input) noexcept
        {
            (void)productionPlant;
            (void)input;
            HookProbe* const probe = static_cast<HookProbe*>(context);
            if (probe != nullptr)
            {
                ++probe->imuMeasurementCalls;
            }
            return Eigen::Vector2f(-0.20f, 0.40f);
        }

        float YawRateMeasurementWithOffset(
            void* context,
            const TractionTestEstimator::StateVector& sigmaPoint) noexcept
        {
            HookProbe* const probe = static_cast<HookProbe*>(context);
            if (probe != nullptr)
            {
                ++probe->yawMeasurementCalls;
            }
            return sigmaPoint(5) + 0.02f;
        }

        std::wstring FloatMessage(const wchar_t* name, const float value)
        {
            return std::wstring(name) + L"=" + std::to_wstring(value);
        }
    }

    TEST_CLASS(TractionTestEstimatorTest)
    {
    public:
        TEST_METHOD(DefaultHooksMatchProductionEstimatorMotionAndImuUpdates)
        {
            EstimatorTestRuntime productionRuntime;
            EstimatorTestRuntime testRuntime;
            Estimator productionEstimator(
                productionRuntime.vehicle,
                productionRuntime.plantModel,
                productionRuntime.runtimeState);
            TractionTestEstimator testEstimator(
                testRuntime.vehicle,
                testRuntime.plantModel,
                testRuntime.runtimeState);

            TractionTestEstimator::StateVector initialState =
                TractionTestEstimator::StateVector::Zero();
            initialState(0) = 0.020f;
            initialState(1) = 0.030f;
            initialState(2) = 0.010f;
            initialState(3) = 0.120f;
            initialState(4) = -0.015f;
            initialState(5) = 0.040f;
            const TractionTestEstimator::CovarianceMatrix initialCovariance =
                TractionTestEstimator::BuildDefaultInitialCovariance();

            Assert::IsTrue(
                EstimatorMotionUpdateTest::Reset(
                    productionEstimator,
                    initialState,
                    initialCovariance),
                L"production reset");
            Assert::IsTrue(
                testEstimator.Reset(initialState, initialCovariance),
                L"test estimator reset");

            const TractionTestEstimator::CommandVector control(0.34f, 0.29f);
            constexpr float dtSeconds = 0.001f;
            Assert::AreEqual(
                productionEstimator.predict(dtSeconds, control),
                testEstimator.Predict(dtSeconds, control),
                L"predict acceptance");
            Assert::AreEqual(
                productionEstimator.updateYawRate(0.044f),
                testEstimator.UpdateYawRate(0.044f),
                L"yaw update acceptance");

            const float expectedForwardAccel =
                productionRuntime.plantModel.backLeftImuForwardAccelerationMps2(control);
            const float expectedRightAccel =
                productionRuntime.plantModel.backLeftImuRightAccelerationMps2(control);
            const ImuAccelObs observation(true, expectedForwardAccel, expectedRightAccel);
            Assert::AreEqual(
                productionEstimator.updatePlanarAccel(observation),
                testEstimator.UpdatePlanarAccel(observation),
                L"planar accel update acceptance");

            const float stateDelta = MaxAbsStateDelta(
                EstimatorMotionUpdateTest::WorkingState(productionEstimator),
                testEstimator.WorkingState());
            Assert::IsTrue(
                stateDelta <= 2.0e-5f,
                FloatMessage(L"max state delta", stateDelta).c_str());

            const float covarianceDelta = MaxAbsCovarianceDelta(
                EstimatorMotionUpdateTest::WorkingCovariance(productionEstimator),
                testEstimator.WorkingCovariance());
            Assert::IsTrue(
                covarianceDelta <= 2.0e-5f,
                FloatMessage(L"max covariance delta", covarianceDelta).c_str());
        }

        TEST_METHOD(InjectedHooksDrivePredictionAndMeasurementCallbacks)
        {
            EstimatorTestRuntime runtime;
            HookProbe probe;
            TractionTestEstimator::Hooks hooks{};
            hooks.context = &probe;
            hooks.predictState = &PredictWithInjectedForwardVelocity;
            hooks.backLeftImuPlanarAcceleration = &FixedPlanarAccelerationMeasurement;
            hooks.yawRateMeasurement = &YawRateMeasurementWithOffset;

            TractionTestEstimator estimator(
                runtime.vehicle,
                runtime.plantModel,
                runtime.runtimeState,
                hooks);
            Assert::IsTrue(estimator.ResetPose(0.0f, 0.0f, 0.0f), L"reset pose");

            Assert::IsTrue(
                estimator.Predict(0.010f, TractionTestEstimator::CommandVector(0.0f, 0.0f)),
                L"predict");
            Assert::IsTrue(probe.predictCalls > 0, L"predict hook was not called");
            Assert::IsTrue(
                estimator.WorkingState()(kVf) > 0.015f,
                FloatMessage(L"forward velocity", estimator.WorkingState()(kVf)).c_str());

            Assert::IsTrue(estimator.UpdateYawRate(0.02f), L"yaw update");
            Assert::IsTrue(probe.yawMeasurementCalls > 0, L"yaw measurement hook was not called");

            const ImuAccelObs observation(true, 0.40f, -0.20f);
            Assert::IsTrue(estimator.UpdatePlanarAccel(observation), L"planar accel update");
            Assert::IsTrue(probe.imuMeasurementCalls > 0, L"IMU measurement hook was not called");
        }
    };
}
