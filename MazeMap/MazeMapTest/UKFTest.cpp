#include "pch.h"
#include "CppUnitTest.h"
#include "Templates.h"

#include "..\MazeMap\OpenFloorMeasurementSpec.h"
#include "..\MazeMap\PlantModel.h"
#include "..\MazeMap\SrUkfCore.h"
#include "..\MazeMap\UKF.h"

#include <array>
#include <cstdlib>
#include <limits>
#include <string>
#include <utility>
#include <vector>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    namespace
    {
        using ScalarState = Eigen::Matrix<float, 2, 1>;
        using ScalarCovariance = Eigen::Matrix<float, 2, 2>;
        using ScalarControl = Eigen::Matrix<float, 1, 1>;

        VehicleState::StateVector BuildUkfState(
            float xM,
            float yM,
            float yawRad,
            float forwardVelocityMps,
            float lateralVelocityMps,
            float yawRateRadps,
            float leftWheelSpeedRadps,
            float rightWheelSpeedRadps,
            float gyroBiasRadps = 0.0f)
        {
            VehicleState::StateVector state = VehicleState::StateVector::Zero();
            state(VehicleState::kPx) = xM;
            state(VehicleState::kPy) = yM;
            state(VehicleState::kPsi) = yawRad;
            state(VehicleState::kU) = forwardVelocityMps;
            state(VehicleState::kV) = lateralVelocityMps;
            state(VehicleState::kR) = yawRateRadps;
            state(VehicleState::kOmegaL) = leftWheelSpeedRadps;
            state(VehicleState::kOmegaR) = rightWheelSpeedRadps;
            state(VehicleState::kBgz) = gyroBiasRadps;
            VehicleState::NormalizeStateVector(state);
            return state;
        }

        VehicleState::StateMatrix BuildUkfCovariance(
            float positionSigmaM = 0.01f,
            float headingSigmaRad = 0.03f,
            float forwardVelocitySigmaMps = 0.05f,
            float lateralVelocitySigmaMps = 0.05f,
            float yawRateSigmaRadps = 0.10f,
            float wheelSigmaRadps = 0.30f,
            float gyroBiasSigmaRadps = 0.03f)
        {
            VehicleState::StateMatrix covariance = VehicleState::StateMatrix::Zero();
            covariance(VehicleState::kPx, VehicleState::kPx) = positionSigmaM * positionSigmaM;
            covariance(VehicleState::kPy, VehicleState::kPy) = positionSigmaM * positionSigmaM;
            covariance(VehicleState::kPsi, VehicleState::kPsi) = headingSigmaRad * headingSigmaRad;
            covariance(VehicleState::kU, VehicleState::kU) = forwardVelocitySigmaMps * forwardVelocitySigmaMps;
            covariance(VehicleState::kV, VehicleState::kV) = lateralVelocitySigmaMps * lateralVelocitySigmaMps;
            covariance(VehicleState::kR, VehicleState::kR) = yawRateSigmaRadps * yawRateSigmaRadps;
            covariance(VehicleState::kOmegaL, VehicleState::kOmegaL) = wheelSigmaRadps * wheelSigmaRadps;
            covariance(VehicleState::kOmegaR, VehicleState::kOmegaR) = wheelSigmaRadps * wheelSigmaRadps;
            covariance(VehicleState::kBgz, VehicleState::kBgz) = gyroBiasSigmaRadps * gyroBiasSigmaRadps;
            return covariance;
        }

        LocalMapView BuildLocalMapView(const Maze& maze) noexcept
        {
            LocalMapView map{};
            map.maze = &maze;
            map.radiusCells = 1U;
            return map;
        }

        struct LoggedOpenFloorPoseJumpSample
        {
            float dtSeconds = 0.0f;
            float poseXM = 0.0f;
            float poseYM = 0.0f;
            float poseYawRad = 0.0f;
            float measuredLinearSpeedMps = 0.0f;
            float measuredAngularSpeedRadps = 0.0f;
            float leftDriveCommand = 0.0f;
            float rightDriveCommand = 0.0f;
            int32_t leftEncoderCount = 0;
            int32_t rightEncoderCount = 0;
            float leftEncoderOmegaRadps = 0.0f;
            float rightEncoderOmegaRadps = 0.0f;
            float gyroRawRadps = 0.0f;
            float gyroBiasRadps = 0.0f;
            float accelBodyXMps2 = 0.0f;
            float accelBodyYMps2 = 0.0f;
        };

        // D:\open_floor_main.csv rows 10714..10721 from the April 8, 2026 run_id=ofm_13260055 launch fault.
        constexpr LoggedOpenFloorPoseJumpSample kLatestLoggedOpenFloorPoseJumpWindow[] = {
            {
                0.001846f, 0.2263727039f, 0.2257606983f, 0.0033714205f,
                0.0031810037f, -0.0251083225f,
                0.39405635f, -0.4572624862f,
                144, 115,
                0.9068602324f, 0.9068602324f,
                -0.0280998014f, -0.0029914798f,
                0.0326640718f, 0.1455999613f
            },
            {
                0.002804f, 0.2484023273f, 0.2257129699f, 0.0042725629f,
                -0.0062825959f, -0.0507646613f,
                0.0f, 0.0f,
                156, 97,
                1.0090416670f, -0.5045208335f,
                -0.0537561402f, -0.0029914798f,
                0.1570908427f, 0.0690296590f
            },
            {
                0.001036f, 0.2408000827f, 0.2257105708f, 0.0040618582f,
                -0.0028340411f, -0.0580950454f,
                -0.18f, -0.18f,
                155, 98,
                1.8268189430f, -2.8232655525f,
                -0.0610865243f, -0.0029914798f,
                -0.0486919172f, 0.3585612178f
            },
            {
                0.002741f, 0.2401545793f, 0.2257211059f, 0.0032611242f,
                0.0032134983f, -0.1558334827f,
                -0.18f, -0.18f,
                150, 104,
                -0.8989821672f, 0.4494910836f,
                -0.1588249654f, -0.0029914798f,
                -0.5057210326f, -0.3951779306f
            },
            {
                0.001017f, 0.2526942492f, 0.2256730497f, 0.0032170448f,
                0.0115479520f, -0.1265119463f,
                -0.18f, -0.18f,
                151, 107,
                -0.6795662045f, 1.1892408133f,
                -0.1295034289f, -0.0029914798f,
                -0.4315435290f, -0.8928850293f
            },
            {
                0.001739f, 0.2828577459f, 0.2256010771f, 0.0024208522f,
                0.0084418245f, -0.1094077229f,
                -0.18f, -0.18f,
                156, 108,
                0.9157773852f, 0.9157773852f,
                -0.1123992056f, -0.0029914798f,
                0.0661635697f, -1.0173118114f
            },
            {
                0.001016f, 0.3010919690f, 0.2255954593f, -0.0043987799f,
                0.0057796580f, -0.0751992688f,
                -0.18f, -0.18f,
                158, 108,
                1.0711276531f, 0.2677819133f,
                -0.0781907514f, -0.0029914798f,
                0.1259841472f, -1.0819180012f
            },
            {
                0.001796f, 0.5462470055f, 0.2249572873f, 0.0019957498f,
                0.0f, -0.0568733141f,
                -0.18f, -0.18f,
                159, 107,
                0.9166785479f, 0.0f,
                -0.0598647930f, -0.0029914798f,
                0.3317668736f, -1.1082390547f
            }
        };

        float DistancePerEncoderCountMeters(const PlantParams& params)
        {
            return
                (2.0f * PI_F * params.wheelRadiusM) /
                (params.gearRatio * static_cast<float>(params.encoderCountsPerMotorRev));
        }

        float AbsoluteCovarianceCorrelation(
            const VehicleState::StateMatrix& covariance,
            int lhsIndex,
            int rhsIndex)
        {
            const float lhsVariance = covariance(lhsIndex, lhsIndex);
            const float rhsVariance = covariance(rhsIndex, rhsIndex);
            if (!(lhsVariance > 0.0f) || !(rhsVariance > 0.0f))
            {
                return 0.0f;
            }

            return
                std::fabs(covariance(lhsIndex, rhsIndex)) /
                MazeMap::Math::Sqrtf(lhsVariance * rhsVariance);
        }

        std::vector<std::pair<std::string, std::string>> CollectDebugDumpLines(const SrUkfCore& core)
        {
            std::vector<std::pair<std::string, std::string>> dumpLines;
            const bool dumpOk = core.WriteDebugTextDump(
                [&dumpLines](const char* type, const char* message) noexcept
                {
                    dumpLines.emplace_back(
                        (type != nullptr) ? type : "",
                        (message != nullptr) ? message : "");
                    return true;
                });
            if (!dumpOk)
            {
                dumpLines.clear();
            }
            return dumpLines;
        }

        std::string FindProcessNoiseRowMessage(
            const std::vector<std::pair<std::string, std::string>>& dumpLines,
            const char* rowName)
        {
            const std::string rowToken = std::string("row=") + rowName;
            for (const auto& line : dumpLines)
            {
                if (line.first == "ukf_dump_process_noise_sqrt_row" &&
                    line.second.find(rowToken) != std::string::npos)
                {
                    return line.second;
                }
            }
            return std::string();
        }

        float ExtractNamedFloat(const std::string& message, const char* fieldName)
        {
            const std::string token = std::string(fieldName) + "=";
            const std::size_t start = message.find(token);
            if (start == std::string::npos)
            {
                return std::numeric_limits<float>::quiet_NaN();
            }

            const char* valueStart = message.c_str() + start + token.size();
            char* valueEnd = nullptr;
            return std::strtof(valueStart, &valueEnd);
        }

        float FindProcessNoiseDiagonal(const SrUkfCore& core, const char* rowName)
        {
            const auto dumpLines = CollectDebugDumpLines(core);
            return ExtractNamedFloat(FindProcessNoiseRowMessage(dumpLines, rowName), rowName);
        }
    }

    TEST_CLASS(UKFTest)
    {
    public:
        using FilterState = Eigen::Matrix<float, 4, 1>;
        using FilterCovariance = Eigen::Matrix<float, 4, 4>;
        using FilterControl = Eigen::Matrix<float, 2, 1>;
        using AngleState = Eigen::Matrix<float, 1, 1>;
        using AngleCovariance = Eigen::Matrix<float, 1, 1>;
        using AngleControl = Eigen::Matrix<float, 1, 1>;

        TEST_METHOD(SquareRootUkfMaintainsPositiveDefiniteCovariance)
        {
            UKF<4, 2> filter;

            FilterState initialState;
            initialState << 0.0f, 0.0f, 0.2f, -0.1f;
            FilterCovariance initialCovariance = FilterCovariance::Identity() * 0.15f;
            filter.setState(initialState, initialCovariance);
            filter.setProcessNoise(FilterCovariance::Identity() * 1.0e-4f);

            const Eigen::Matrix<float, 2, 2> sqrtMeasurementNoise =
                Eigen::Matrix<float, 2, 2>::Identity() * 0.05f;

            FilterState truth = initialState;
            FilterControl control;
            control << 0.15f, -0.05f;

            constexpr float dt = 0.001f;
            for (int step = 0; step < 120; ++step)
            {
                truth(0) += dt * truth(2);
                truth(1) += dt * truth(3);
                truth(2) += dt * control(0);
                truth(3) += dt * control(1);

                const bool predictOk = filter.Predict(
                    dt,
                    control,
                    [](const FilterState& sigmaPoint, const FilterControl& sigmaControl, float sigmaDt) noexcept
                    {
                        FilterState predicted = sigmaPoint;
                        predicted(0) += sigmaDt * sigmaPoint(2);
                        predicted(1) += sigmaDt * sigmaPoint(3);
                        predicted(2) += sigmaDt * sigmaControl(0);
                        predicted(3) += sigmaDt * sigmaControl(1);
                        return predicted;
                    });
                Assert::IsTrue(predictOk);

                Eigen::Matrix<float, 2, 1> measurement;
                measurement << truth(0), truth(1);
                const bool updateOk = filter.Update<2>(
                    measurement,
                    sqrtMeasurementNoise,
                    15.0f,
                    [](const FilterState& sigmaPoint) noexcept
                    {
                        Eigen::Matrix<float, 2, 1> projection;
                        projection << sigmaPoint(0), sigmaPoint(1);
                        return projection;
                    });
                Assert::IsTrue(updateOk);

                const FilterCovariance covariance = filter.covariance();
                const Eigen::LLT<FilterCovariance> llt(covariance);
                Assert::IsTrue(llt.info() == Eigen::Success);
                Assert::IsTrue(std::isfinite(covariance.trace()));
                Assert::IsTrue(std::isfinite(filter.state().sum()));
            }
        }

        TEST_METHOD(SquareRootUkfUsesReferenceRelativeWrappedMeanForAngles)
        {
            UKF<1, 1> filter;
            filter.setStateNormalizer([](AngleState& state) noexcept
                {
                    state(0) = VehicleState::NormalizeAngle(state(0));
                });

            AngleState initialState;
            initialState << (PI_F - 0.01f);
            AngleCovariance covariance = AngleCovariance::Zero();
            covariance(0, 0) = 0.04f;
            filter.setState(initialState, covariance);
            filter.setProcessNoise(AngleCovariance::Zero());

            AngleControl control = AngleControl::Zero();
            const bool predictOk = filter.Predict(
                0.001f,
                control,
                [](const AngleState& sigmaPoint, const AngleControl&, float) noexcept
                {
                    return sigmaPoint;
                });

            Assert::IsTrue(predictOk);
            const float wrappedError =
                VehicleState::NormalizeAngle(filter.state()(0) - initialState(0));
            Assert::IsTrue(std::fabs(wrappedError) < 0.05f);
        }

        TEST_METHOD(SquareRootUkfVelocitySigmaChangesPositionVarianceButNotPositionMean)
        {
            UKF<2, 1> lowVarianceFilter;
            UKF<2, 1> highVarianceFilter;

            ScalarState initialState{};
            initialState << 1.25f, 2.0f;
            ScalarCovariance lowCovariance = ScalarCovariance::Zero();
            lowCovariance(0, 0) = 1.0e-6f;
            lowCovariance(1, 1) = 0.01f * 0.01f;
            ScalarCovariance highCovariance = ScalarCovariance::Zero();
            highCovariance(0, 0) = 1.0e-6f;
            highCovariance(1, 1) = 0.10f * 0.10f;

            lowVarianceFilter.setState(initialState, lowCovariance);
            highVarianceFilter.setState(initialState, highCovariance);
            lowVarianceFilter.setProcessNoise(ScalarCovariance::Zero());
            highVarianceFilter.setProcessNoise(ScalarCovariance::Zero());

            const ScalarControl control = ScalarControl::Zero();
            constexpr float dt = 0.40f;
            const auto constantVelocityModel =
                [](const ScalarState& sigmaPoint, const ScalarControl&, float sigmaDt) noexcept
                {
                    ScalarState predicted = sigmaPoint;
                    predicted(0) += sigmaPoint(1) * sigmaDt;
                    return predicted;
                };

            Assert::IsTrue(lowVarianceFilter.Predict(dt, control, constantVelocityModel));
            Assert::IsTrue(highVarianceFilter.Predict(dt, control, constantVelocityModel));

            const float expectedPositionM = initialState(0) + (initialState(1) * dt);
            Assert::AreEqual(expectedPositionM, lowVarianceFilter.state()(0), 1.0e-4f);
            Assert::AreEqual(expectedPositionM, highVarianceFilter.state()(0), 1.0e-4f);
            Assert::AreEqual(lowVarianceFilter.state()(0), highVarianceFilter.state()(0), 1.0e-4f);
            Assert::AreEqual(initialState(1), lowVarianceFilter.state()(1), 1.0e-6f);
            Assert::AreEqual(initialState(1), highVarianceFilter.state()(1), 1.0e-6f);

            const ScalarCovariance lowPosterior = lowVarianceFilter.covariance();
            const ScalarCovariance highPosterior = highVarianceFilter.covariance();
            Assert::IsTrue(highPosterior(0, 0) > (lowPosterior(0, 0) * 10.0f));
            Assert::IsTrue(highPosterior(1, 1) > (lowPosterior(1, 1) * 10.0f));
        }

        TEST_METHOD(PlantModelSymmetricDriveDoesNotCreateYawBias)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();

            VehicleState::StateVector state = VehicleState::StateVector::Zero();
            state(VehicleState::kU) = 2.5f;
            state(VehicleState::kOmegaL) = state(VehicleState::kU) / params.wheelRadiusM;
            state(VehicleState::kOmegaR) = state(VehicleState::kU) / params.wheelRadiusM;

            ControlInput control;
            control.leftMotorCommand = 0.55f;
            control.rightMotorCommand = 0.55f;
            control.fanDutyCycle = 0.80f;
            control.batteryVoltageV = params.supplyVoltageV;

            const PlantDerivatives derivatives = plant.forwardStep(state, control, params);
            Assert::IsTrue(std::isfinite(derivatives.stateDot(VehicleState::kR)));
            Assert::AreEqual(0.0f, derivatives.stateDot(VehicleState::kR), 1.0e-4f);
        }

        TEST_METHOD(PlantModelExactRestHoldKeepsMotionStateAtZero)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();
            VehicleState::StateVector state = BuildUkfState(
                0.03f,
                0.09f,
                0.21f,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                0.12f);

            ControlInput control{};
            control.batteryVoltageV = params.supplyVoltageV;
            constexpr float dt = 0.001f;
            for (int step = 0; step < 1000; ++step)
            {
                state = plant.integrate(state, control, dt, params);
            }

            Assert::AreEqual(0.03f, state(VehicleState::kPx), 1.0e-7f);
            Assert::AreEqual(0.09f, state(VehicleState::kPy), 1.0e-7f);
            Assert::AreEqual(0.21f, state(VehicleState::kPsi), 1.0e-7f);
            Assert::AreEqual(0.12f, state(VehicleState::kBgz), 1.0e-7f);
            Assert::AreEqual(0.0f, state(VehicleState::kU), 1.0e-7f);
            Assert::AreEqual(0.0f, state(VehicleState::kV), 1.0e-7f);
            Assert::AreEqual(0.0f, state(VehicleState::kR), 1.0e-7f);
            Assert::AreEqual(0.0f, state(VehicleState::kOmegaL), 1.0e-7f);
            Assert::AreEqual(0.0f, state(VehicleState::kOmegaR), 1.0e-7f);
        }

        TEST_METHOD(PlantModelSmallStationaryPerturbationsSnapBackToRest)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();
            VehicleState::StateVector state = BuildUkfState(
                0.0f,
                0.09f,
                0.0f,
                0.0f,
                0.0f,
                0.05f,
                0.8f,
                -0.7f);

            ControlInput control{};
            control.batteryVoltageV = params.supplyVoltageV;
            constexpr float dt = 0.001f;
            for (int step = 0; step < 25; ++step)
            {
                state = plant.integrate(state, control, dt, params);
            }

            Assert::AreEqual(0.0f, state(VehicleState::kU), 1.0e-7f);
            Assert::AreEqual(0.0f, state(VehicleState::kV), 1.0e-7f);
            Assert::AreEqual(0.0f, state(VehicleState::kR), 1.0e-7f);
            Assert::AreEqual(0.0f, state(VehicleState::kOmegaL), 1.0e-7f);
            Assert::AreEqual(0.0f, state(VehicleState::kOmegaR), 1.0e-7f);
        }

        TEST_METHOD(VehicleStateIsStationaryUsesCurrentUkfThresholds)
        {
            const PlantParams params = PlantParams::Default();
            const float wheelSpeedThresholdRadps =
                SrUkfCore::kStationaryEncoderVelocitySigmaMps / params.wheelRadiusM;

            VehicleState stationaryState;
            stationaryState.SetStateVector(BuildUkfState(
                0.40f,
                -0.18f,
                0.35f,
                0.0f,
                0.0f,
                0.5f * (3.0f * SrUkfCore::kImuYawRateSigmaRadps),
                0.5f * wheelSpeedThresholdRadps,
                -0.5f * wheelSpeedThresholdRadps,
                0.12f));
            Assert::IsTrue(stationaryState.IsStationary());

            VehicleState movingState = stationaryState;
            VehicleState::StateVector movingVector = movingState.GetStateVector();
            movingVector(VehicleState::kU) = 2.0f * SrUkfCore::kStationaryEncoderVelocitySigmaMps;
            movingState.SetStateVector(movingVector);
            Assert::IsFalse(movingState.IsStationary());

            movingVector = stationaryState.GetStateVector();
            movingVector(VehicleState::kR) = 3.1f * SrUkfCore::kImuYawRateSigmaRadps;
            movingState.SetStateVector(movingVector);
            Assert::IsFalse(movingState.IsStationary());

            movingVector = stationaryState.GetStateVector();
            movingVector(VehicleState::kOmegaL) = 1.1f * wheelSpeedThresholdRadps;
            movingState.SetStateVector(movingVector);
            Assert::IsFalse(movingState.IsStationary());
        }

        TEST_METHOD(VehicleStateStationaryConstraintAnchorsPoseAndCollapsesStationaryStates)
        {
            const PlantParams params = PlantParams::Default();
            const float distancePerEncoderCountM = DistancePerEncoderCountMeters(params);

            VehicleState state;
            state.SetStateVector(BuildUkfState(
                0.40f,
                -0.18f,
                0.35f,
                0.8f,
                -0.12f,
                0.4f,
                9.0f,
                7.5f,
                -0.02f));
            state.SetCovariance(BuildUkfCovariance(0.05f, 0.08f, 0.30f, 0.20f, 0.25f, 0.45f, 0.06f));
            Assert::IsFalse(state.IsStationary());

            const VehicleState::StateVector poseReferenceState = BuildUkfState(
                1.20f,
                0.70f,
                -0.20f,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                0.0f);
            VehicleState::StateMatrix poseReferenceCovariance =
                BuildUkfCovariance(0.012f, 0.02f, 0.15f, 0.11f, 0.09f, 0.40f, 0.03f);
            poseReferenceCovariance(VehicleState::kPx, VehicleState::kPy) = 2.5e-5f;
            poseReferenceCovariance(VehicleState::kPy, VehicleState::kPx) = 2.5e-5f;
            poseReferenceCovariance(VehicleState::kPx, VehicleState::kPsi) = -1.5e-5f;
            poseReferenceCovariance(VehicleState::kPsi, VehicleState::kPx) = -1.5e-5f;
            poseReferenceCovariance(VehicleState::kPy, VehicleState::kPsi) = 1.2e-5f;
            poseReferenceCovariance(VehicleState::kPsi, VehicleState::kPy) = 1.2e-5f;

            EncoderObs encoder{};
            encoder.totalLeftCounts = 12;
            encoder.totalRightCounts = 8;

            constexpr float measuredYawRateRadps = 0.013f;
            state.ApplyStationaryZeroMotionConstraint(
                encoder,
                measuredYawRateRadps,
                true,
                true,
                poseReferenceState,
                poseReferenceCovariance,
                distancePerEncoderCountM,
                params.trackWidthM);

            const float leftDistanceM =
                static_cast<float>(encoder.totalLeftCounts) * distancePerEncoderCountM;
            const float rightDistanceM =
                static_cast<float>(encoder.totalRightCounts) * distancePerEncoderCountM;
            const float forwardDistanceM = 0.5f * (leftDistanceM + rightDistanceM);
            const float deltaYawRad = (leftDistanceM - rightDistanceM) / params.trackWidthM;
            const float expectedYawRad =
                VehicleState::NormalizeAngle(poseReferenceState(VehicleState::kPsi) + deltaYawRad);
            const float translationYawRad =
                VehicleState::NormalizeAngle(poseReferenceState(VehicleState::kPsi) + (0.5f * deltaYawRad));
            const float expectedPxM =
                poseReferenceState(VehicleState::kPx) + (forwardDistanceM * std::sin(translationYawRad));
            const float expectedPyM =
                poseReferenceState(VehicleState::kPy) + (forwardDistanceM * std::cos(translationYawRad));

            const VehicleState::StateVector& constrainedState = state.GetStateVector();
            const VehicleState::StateMatrix constrainedCovariance = state.GetCovariance();

            Assert::AreEqual(expectedPxM, constrainedState(VehicleState::kPx), 1.0e-6f);
            Assert::AreEqual(expectedPyM, constrainedState(VehicleState::kPy), 1.0e-6f);
            Assert::AreEqual(expectedYawRad, constrainedState(VehicleState::kPsi), 1.0e-6f);
            Assert::AreEqual(0.0f, constrainedState(VehicleState::kU), 1.0e-7f);
            Assert::AreEqual(0.0f, constrainedState(VehicleState::kV), 1.0e-7f);
            Assert::AreEqual(0.0f, constrainedState(VehicleState::kR), 1.0e-7f);
            Assert::AreEqual(0.0f, constrainedState(VehicleState::kOmegaL), 1.0e-7f);
            Assert::AreEqual(0.0f, constrainedState(VehicleState::kOmegaR), 1.0e-7f);
            Assert::AreEqual(measuredYawRateRadps, constrainedState(VehicleState::kBgz), 1.0e-7f);
            Assert::IsTrue(state.IsStationary());

            constexpr std::array<int, 3> kPoseIndices = {
                VehicleState::kPx,
                VehicleState::kPy,
                VehicleState::kPsi
            };
            for (const int row : kPoseIndices)
            {
                for (const int col : kPoseIndices)
                {
                    Assert::AreEqual(
                        poseReferenceCovariance(row, col),
                        constrainedCovariance(row, col),
                        1.0e-7f);
                }
            }

            Assert::IsTrue(constrainedCovariance(VehicleState::kU, VehicleState::kU) <= 1.0e-12f);
            Assert::IsTrue(constrainedCovariance(VehicleState::kV, VehicleState::kV) <= 1.0e-12f);
            Assert::IsTrue(constrainedCovariance(VehicleState::kR, VehicleState::kR) <= 1.0e-12f);
            Assert::IsTrue(constrainedCovariance(VehicleState::kOmegaL, VehicleState::kOmegaL) <= 1.0e-12f);
            Assert::IsTrue(constrainedCovariance(VehicleState::kOmegaR, VehicleState::kOmegaR) <= 1.0e-12f);
            Assert::IsTrue(constrainedCovariance(VehicleState::kBgz, VehicleState::kBgz) <= 1.0e-12f);
        }

        TEST_METHOD(PlantModelComputesFiniteAlgebraicSlipAndForces)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();

            VehicleState::StateVector state = VehicleState::StateVector::Zero();
            state(VehicleState::kU) = 2.0f;
            state(VehicleState::kV) = 0.15f;
            state(VehicleState::kR) = 1.8f;
            state(VehicleState::kOmegaL) = 1.05f * (state(VehicleState::kU) / params.wheelRadiusM);
            state(VehicleState::kOmegaR) = 0.95f * (state(VehicleState::kU) / params.wheelRadiusM);

            ControlInput control;
            control.leftMotorCommand = 0.65f;
            control.rightMotorCommand = 0.60f;
            control.fanDutyCycle = 0.80f;
            control.batteryVoltageV = params.supplyVoltageV;

            const WheelKinematics kinematics = plant.wheelKinematics(state, params);
            const SlipTargets slip = plant.slipTargets(state, kinematics, params);
            const ContactForces forces = plant.tireForces(state, control, params);

            Assert::IsTrue(std::isfinite(slip.kappaLeft));
            Assert::IsTrue(std::isfinite(slip.kappaRight));
            for (float ratio : slip.lateralRatio)
            {
                Assert::IsTrue(std::isfinite(ratio));
            }

            for (const ContactForce& force : forces.contacts)
            {
                Assert::IsTrue(std::isfinite(force.rightForceN));
                Assert::IsTrue(std::isfinite(force.forwardForceN));
                Assert::IsTrue(force.saturation >= 0.0f);
                Assert::IsTrue(force.saturation <= 1.0f);
            }
        }

        TEST_METHOD(PlantModelImuAccelerationIncludesLeverArmTerms)
        {
            PlantModel plant;
            PlantParams zeroLeverParams = PlantParams::Default();
            zeroLeverParams.imu.positionBodyM = Eigen::Vector2f::Zero();
            PlantParams leverParams = zeroLeverParams;
            leverParams.imu.positionBodyM = Eigen::Vector2f(0.020f, -0.010f);

            VehicleState::StateVector state = VehicleState::StateVector::Zero();
            state(VehicleState::kU) = 1.4f;
            state(VehicleState::kV) = 0.2f;
            state(VehicleState::kR) = 5.0f;
            state(VehicleState::kOmegaL) = 0.9f * (state(VehicleState::kU) / zeroLeverParams.wheelRadiusM);
            state(VehicleState::kOmegaR) = 1.1f * (state(VehicleState::kU) / zeroLeverParams.wheelRadiusM);

            ControlInput control;
            control.leftMotorCommand = 0.30f;
            control.rightMotorCommand = 0.55f;
            control.fanDutyCycle = 0.80f;
            control.batteryVoltageV = zeroLeverParams.supplyVoltageV;

            const PlantDerivatives zeroLever = plant.forwardStep(state, control, zeroLeverParams);
            const PlantDerivatives withLever = plant.forwardStep(state, control, leverParams);

            const float expectedDeltaX =
                (withLever.stateDot(VehicleState::kR) * leverParams.imu.positionBodyM.y()) -
                ((state(VehicleState::kR) * state(VehicleState::kR)) * leverParams.imu.positionBodyM.x());
            const float expectedDeltaY =
                (-withLever.stateDot(VehicleState::kR) * leverParams.imu.positionBodyM.x()) -
                ((state(VehicleState::kR) * state(VehicleState::kR)) * leverParams.imu.positionBodyM.y());

            Assert::AreEqual(
                zeroLever.imuAccelBodyMps2.x() + expectedDeltaX,
                withLever.imuAccelBodyMps2.x(),
                1.0e-5f);
            Assert::AreEqual(
                zeroLever.imuAccelBodyMps2.y() + expectedDeltaY,
                withLever.imuAccelBodyMps2.y(),
                1.0e-5f);
        }

        TEST_METHOD(PlantModelPredictsImuAccelerationInProjectBodyAxes)
        {
            PlantModel plant;
            PlantParams params = PlantParams::Default();

            VehicleState::StateVector state = VehicleState::StateVector::Zero();
            state(VehicleState::kU) = 1.1f;
            state(VehicleState::kV) = -0.3f;
            state(VehicleState::kR) = 4.0f;
            state(VehicleState::kOmegaL) = 0.95f * (state(VehicleState::kU) / params.wheelRadiusM);
            state(VehicleState::kOmegaR) = 1.05f * (state(VehicleState::kU) / params.wheelRadiusM);

            ControlInput control;
            control.leftMotorCommand = 0.25f;
            control.rightMotorCommand = 0.45f;
            control.fanDutyCycle = 0.80f;
            control.batteryVoltageV = params.supplyVoltageV;

            const PlantDerivatives derivatives = plant.forwardStep(state, control, params);
            const Eigen::Vector2f predictedMeasurement = plant.imuPlanarAcceleration(state, control, params);

            Assert::AreEqual(derivatives.imuAccelBodyMps2.x(), predictedMeasurement.x(), 1.0e-5f);
            Assert::AreEqual(derivatives.imuAccelBodyMps2.y(), predictedMeasurement.y(), 1.0e-5f);
        }

        TEST_METHOD(PlantModelSymmetricPositiveDriveFromRestMovesForward)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();
            VehicleState::StateVector state = BuildUkfState(
                0.0f,
                0.09f,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                0.0f);

            ControlInput control{};
            control.leftMotorCommand = 0.50f;
            control.rightMotorCommand = 0.50f;
            control.fanDutyCycle = 0.80f;
            control.batteryVoltageV = params.supplyVoltageV;

            constexpr float dt = 0.002f;
            constexpr int kSteps = 25;
            for (int step = 0; step < kSteps; ++step)
            {
                state = plant.integrate(state, control, dt, params);
            }

            const float totalTimeS = dt * static_cast<float>(kSteps);
            const float averageAccelMps2 = state(VehicleState::kU) / totalTimeS;

            Assert::IsTrue(std::isfinite(state.sum()));
            Assert::IsTrue(state(VehicleState::kU) > 0.0f);
            Assert::IsTrue(state(VehicleState::kPy) > 0.09f);
            Assert::IsTrue(averageAccelMps2 > 0.0f);
            Assert::IsTrue(averageAccelMps2 < 60.0f);
            Assert::IsTrue(std::fabs(state(VehicleState::kPx)) < 0.002f);
            Assert::IsTrue(std::fabs(state(VehicleState::kV)) < 0.02f);
            Assert::IsTrue(std::fabs(state(VehicleState::kR)) < 0.10f);
            Assert::IsTrue(std::fabs(state(VehicleState::kPsi)) < 0.01f);
        }

        TEST_METHOD(PlantModelIntegrateSingleLargeStepRemainsFiniteAndSymmetric)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();

            VehicleState::StateVector state = BuildUkfState(
                0.0f,
                0.09f,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                0.0f);
            ControlInput control{};
            control.leftMotorCommand = 0.45f;
            control.rightMotorCommand = 0.45f;
            control.fanDutyCycle = 0.80f;
            control.batteryVoltageV = params.supplyVoltageV;

            constexpr float dt = 0.004f;
            const VehicleState::StateVector integrated = plant.integrate(state, control, dt, params);

            Assert::IsTrue(std::isfinite(integrated.sum()));
            Assert::IsTrue(std::isfinite(integrated(VehicleState::kU)));
            Assert::IsTrue(std::isfinite(integrated(VehicleState::kOmegaL)));
            Assert::IsTrue(std::isfinite(integrated(VehicleState::kOmegaR)));
            Assert::IsTrue(std::fabs(integrated(VehicleState::kOmegaL) - integrated(VehicleState::kOmegaR)) < 1.0f);
            Assert::IsTrue(std::fabs(integrated(VehicleState::kPx)) < 0.005f);
            Assert::IsTrue(std::fabs(integrated(VehicleState::kR)) < 0.10f);
        }

        TEST_METHOD(PlantModelIntegratePreservesHeadingNormalization)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();
            VehicleState::StateVector state = BuildUkfState(
                0.0f,
                0.09f,
                PI_F - 0.01f,
                0.5f,
                0.0f,
                6.0f,
                0.5f / params.wheelRadiusM,
                0.5f / params.wheelRadiusM);

            ControlInput control{};
            control.batteryVoltageV = params.supplyVoltageV;
            const VehicleState::StateVector integrated = plant.integrate(state, control, 0.01f, params);

            Assert::IsTrue(integrated(VehicleState::kPsi) <= PI_F);
            Assert::IsTrue(integrated(VehicleState::kPsi) >= -PI_F);
        }

        TEST_METHOD(PlantModelSolveDriveCommandsZeroRequestReturnsZeroCommand)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();
            const DriveCommandSolution solution =
                plant.solveDriveCommands(0.0f, 0.0f, 0.0f, 0.0f, params, 0.80f, params.supplyVoltageV);

            Assert::IsFalse(solution.tractionLimited);
            Assert::IsTrue(solution.converged);
            Assert::AreEqual(0.0f, solution.control.leftMotorCommand, 1.0e-6f);
            Assert::AreEqual(0.0f, solution.control.rightMotorCommand, 1.0e-6f);
            Assert::AreEqual(0.0f, solution.leftWheelTorqueNm, 1.0e-6f);
            Assert::AreEqual(0.0f, solution.rightWheelTorqueNm, 1.0e-6f);
        }

        TEST_METHOD(PlantModelSolveDriveCommandsIncludesWheelInertiaAndFriction)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();
            const DriveCommandSolution solution =
                plant.solveDriveCommands(1.8f, 3.5f, 1.5f, 14.0f, params, 0.80f, params.supplyVoltageV);

            const float expectedLeftTorqueNm =
                solution.leftContactTorqueNm +
                (params.equivalentWheelInertiaKgM2 * solution.leftWheelAccelRadps2) +
                plant.driveFrictionTorque(solution.leftWheelSpeedRadps, params);
            const float expectedRightTorqueNm =
                solution.rightContactTorqueNm +
                (params.equivalentWheelInertiaKgM2 * solution.rightWheelAccelRadps2) +
                plant.driveFrictionTorque(solution.rightWheelSpeedRadps, params);

            Assert::AreEqual(expectedLeftTorqueNm, solution.leftWheelTorqueNm, 1.0e-6f);
            Assert::AreEqual(expectedRightTorqueNm, solution.rightWheelTorqueNm, 1.0e-6f);
            Assert::IsTrue(std::fabs(solution.leftWheelTorqueNm - solution.leftContactTorqueNm) > 1.0e-4f);
            Assert::IsTrue(std::fabs(solution.rightWheelTorqueNm - solution.rightContactTorqueNm) > 1.0e-4f);
        }

        TEST_METHOD(PlantModelSolveDriveCommandsReturnsBodyConsistentOperatingPointAtModerateCombinedTarget)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();
            constexpr float forwardVelocityMps = 2.1f;
            constexpr float desiredLongitudinalAccelMps2 = 1.2f;
            constexpr float yawRateRadps = 4.0f;
            constexpr float desiredYawAccelRadps2 = 4.5f;

            const DriveCommandSolution solution =
                plant.solveDriveCommands(
                    forwardVelocityMps,
                    desiredLongitudinalAccelMps2,
                    yawRateRadps,
                    desiredYawAccelRadps2,
                    params,
                    0.80f,
                    params.supplyVoltageV);

            VehicleState::StateVector state = VehicleState::StateVector::Zero();
            state(VehicleState::kU) = forwardVelocityMps;
            state(VehicleState::kR) = yawRateRadps;
            state(VehicleState::kOmegaL) = solution.leftWheelSpeedRadps;
            state(VehicleState::kOmegaR) = solution.rightWheelSpeedRadps;

            const PlantDerivatives achieved = plant.forwardStep(state, solution.control, params);
            const float expectedLeftWheelAccelRadps2 =
                (desiredLongitudinalAccelMps2 + (0.5f * params.trackWidthM * desiredYawAccelRadps2)) / params.wheelRadiusM;
            const float expectedRightWheelAccelRadps2 =
                (desiredLongitudinalAccelMps2 - (0.5f * params.trackWidthM * desiredYawAccelRadps2)) / params.wheelRadiusM;

            Assert::IsTrue(std::isfinite(solution.control.leftMotorCommand));
            Assert::IsTrue(std::isfinite(solution.control.rightMotorCommand));
            Assert::IsTrue(std::isfinite(achieved.longitudinalAccelMps2));
            Assert::IsTrue(std::isfinite(achieved.yawAccelRadps2));
            Assert::AreEqual(desiredLongitudinalAccelMps2, achieved.longitudinalAccelMps2, 0.05f);
            Assert::AreEqual(desiredYawAccelRadps2, achieved.yawAccelRadps2, 0.20f);
            Assert::AreEqual(expectedLeftWheelAccelRadps2, solution.leftWheelAccelRadps2, 1.0e-5f);
            Assert::AreEqual(expectedRightWheelAccelRadps2, solution.rightWheelAccelRadps2, 1.0e-5f);
            Assert::IsTrue(std::isfinite(achieved.stateDot(VehicleState::kOmegaL)));
            Assert::IsTrue(std::isfinite(achieved.stateDot(VehicleState::kOmegaR)));
        }

        TEST_METHOD(PlantModelSolveDriveCommandsDoesNotTractionLimitWellInsideNominalEnvelope)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();
            const float forwardVelocityMps = 2.0f;
            const float yawRateRadps = 4.0f;
            const float desiredLongitudinalAccelMps2 = 1.0f;

            const DriveCommandSolution solution =
                plant.solveDriveCommands(
                    forwardVelocityMps,
                    desiredLongitudinalAccelMps2,
                    yawRateRadps,
                    0.0f,
                    params,
                    0.80f,
                    params.supplyVoltageV);

            Assert::IsFalse(solution.tractionLimited);
            Assert::IsTrue(std::isfinite(solution.control.leftMotorCommand));
            Assert::IsTrue(std::isfinite(solution.control.rightMotorCommand));
        }

        TEST_METHOD(PlantModelSolveDriveCommandsSupportsHistoricalThreeMeterPerSecondEnvelope)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();
            // Earlier pre-UKF testing reached about 3 m/s in roughly 50 mm of real travel. Keep the plant
            // drag/friction model from making that known envelope unreachable even though the current D: CSV
            // launch capture only reaches the lower post-UKF retrofit speed.
            const DriveCommandSolution solution =
                plant.solveDriveCommands(
                    3.0f,
                    1.0f,
                    0.0f,
                    0.0f,
                    params,
                    0.80f,
                    params.supplyVoltageV);

            Assert::IsFalse(solution.tractionLimited);
            Assert::IsTrue(solution.converged);
            Assert::IsTrue(std::fabs(solution.control.leftMotorCommand) < 1.0f);
            Assert::IsTrue(std::fabs(solution.control.rightMotorCommand) < 1.0f);
        }

        TEST_METHOD(PlantModelSolveDriveCommandsBeyondNominalReturnsClippedFiniteSolution)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();
            const DriveCommandSolution solution =
                plant.solveDriveCommands(2.0f, params.combinedAccelNominalMps2 + 2.0f, 0.0f, 0.0f, params, 0.80f, params.supplyVoltageV);

            VehicleState::StateVector state = VehicleState::StateVector::Zero();
            state(VehicleState::kU) = 2.0f;
            state(VehicleState::kOmegaL) = solution.leftWheelSpeedRadps;
            state(VehicleState::kOmegaR) = solution.rightWheelSpeedRadps;
            const PlantDerivatives achieved = plant.forwardStep(state, solution.control, params);

            Assert::IsTrue(solution.tractionLimited);
            Assert::IsTrue(std::isfinite(solution.control.leftMotorCommand));
            Assert::IsTrue(std::isfinite(solution.control.rightMotorCommand));
            Assert::IsTrue(std::isfinite(achieved.longitudinalAccelMps2));
            Assert::IsTrue(std::fabs(achieved.longitudinalAccelMps2) <= (params.combinedAccelPeakMps2 + 1.0f));
        }

        TEST_METHOD(PlantModelSolveDriveCommandsBeyondPeakRemainsStable)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();
            const DriveCommandSolution solution =
                plant.solveDriveCommands(2.0f, params.combinedAccelPeakMps2 + 5.0f, 0.0f, 0.0f, params, 0.80f, params.supplyVoltageV);

            VehicleState::StateVector state = VehicleState::StateVector::Zero();
            state(VehicleState::kU) = 2.0f;
            state(VehicleState::kOmegaL) = solution.leftWheelSpeedRadps;
            state(VehicleState::kOmegaR) = solution.rightWheelSpeedRadps;
            const PlantDerivatives achieved = plant.forwardStep(state, solution.control, params);

            Assert::IsTrue(solution.tractionLimited);
            Assert::IsTrue(std::isfinite(achieved.longitudinalAccelMps2));
            Assert::IsTrue(std::isfinite(achieved.yawAccelRadps2));
            Assert::IsTrue(std::fabs(solution.control.leftMotorCommand) <= 1.0f);
            Assert::IsTrue(std::fabs(solution.control.rightMotorCommand) <= 1.0f);
        }

        TEST_METHOD(PlantModelSolveDriveCommandsNearZeroSpeedTurnRemainsFinite)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();
            const DriveCommandSolution solution =
                plant.solveDriveCommands(0.005f, 0.0f, 0.0f, 20.0f, params, 0.80f, params.supplyVoltageV);

            Assert::IsTrue(std::isfinite(solution.control.leftMotorCommand));
            Assert::IsTrue(std::isfinite(solution.control.rightMotorCommand));
            Assert::IsTrue(std::isfinite(solution.leftWheelSpeedRadps));
            Assert::IsTrue(std::isfinite(solution.rightWheelSpeedRadps));
            Assert::IsTrue(std::isfinite(solution.leftWheelTorqueNm));
            Assert::IsTrue(std::isfinite(solution.rightWheelTorqueNm));
        }

        TEST_METHOD(PlantModelDriveCommandInverseMatchesForwardMotorModel)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();
            const float wheelSpeedRadps = 145.0f;
            const float desiredWheelTorqueNm = 0.0035f;

            const float command =
                plant.driveCommandFromTorque(
                    desiredWheelTorqueNm,
                    wheelSpeedRadps,
                    params.supplyVoltageV,
                    params);
            const float reconstructedTorqueNm =
                plant.driveTorqueFromCommand(
                    command,
                    wheelSpeedRadps,
                    params.supplyVoltageV,
                    params);

            Assert::IsTrue(std::isfinite(command));
            Assert::IsTrue(std::fabs(command) <= 1.0f);
            Assert::AreEqual(desiredWheelTorqueNm, reconstructedTorqueNm, 1.0e-4f);
        }

        TEST_METHOD(ComputeEncoderPairSqrtNoise_UsesGeneralSigmaMappingForNonZeroReadings)
        {
            const PlantParams params = PlantParams::Default();
            EncoderObs observation{};
            observation.omegaLeftRadps = 1.0f;
            observation.omegaRightRadps = 1.0f;

            const Eigen::Matrix<float, 2, 2> sqrtNoise = SrUkfCore::ComputeEncoderPairSqrtNoise(observation, params);
            const Eigen::Matrix<float, 2, 2> covariance = sqrtNoise * sqrtNoise.transpose();
            const float halfTrackWidthM = 0.5f * params.trackWidthM;
            const float varianceUMps2 =
                SrUkfCore::kGeneralEncoderLinearSpeedSigmaMps * SrUkfCore::kGeneralEncoderLinearSpeedSigmaMps;
            const float varianceYawRateRadps2 =
                SrUkfCore::kGeneralEncoderYawRateSigmaRadps * SrUkfCore::kGeneralEncoderYawRateSigmaRadps;
            const float invWheelRadius2 = 1.0f / (params.wheelRadiusM * params.wheelRadiusM);
            const float expectedVarianceRadps2 =
                (varianceUMps2 + ((halfTrackWidthM * halfTrackWidthM) * varianceYawRateRadps2)) * invWheelRadius2;
            const float expectedCovarianceRadps2 =
                (varianceUMps2 - ((halfTrackWidthM * halfTrackWidthM) * varianceYawRateRadps2)) * invWheelRadius2;

            Assert::AreEqual(expectedVarianceRadps2, covariance(0, 0), 1.0e-5f);
            Assert::AreEqual(expectedVarianceRadps2, covariance(1, 1), 1.0e-5f);
            Assert::AreEqual(expectedCovarianceRadps2, covariance(0, 1), 1.0e-5f);
            Assert::AreEqual(expectedCovarianceRadps2, covariance(1, 0), 1.0e-5f);
        }

        TEST_METHOD(ComputeStationaryEncoderOmegaSigmaRadps_UsesRequestedZeroSpeedSigma)
        {
            const PlantParams params = PlantParams::Default();
            const float expectedSigmaRadps = 1.76e-6f / params.wheelRadiusM;
            Assert::AreEqual(expectedSigmaRadps, SrUkfCore::ComputeStationaryEncoderOmegaSigmaRadps(params), 1.0e-9f);
        }

        TEST_METHOD(ConfiguredMeasurementSigmasMatchLatestOpenFloorLogTuning)
        {
            Assert::AreEqual(0.0066f, SrUkfCore::kGeneralEncoderLinearSpeedSigmaMps, 1.0e-9f);
            Assert::AreEqual(0.0484f, SrUkfCore::kGeneralEncoderYawRateSigmaRadps, 1.0e-9f);
            Assert::AreEqual(0.0131f, SrUkfCore::kImuYawRateSigmaRadps, 1.0e-9f);
            Assert::AreEqual(0.1305f, SrUkfCore::kImuAccelSigmaMps2, 1.0e-9f);
        }

        TEST_METHOD(BuildDefaultInitialCovariance_ReturnsCanonicalResetCovariance)
        {
            const VehicleState::StateMatrix covariance = SrUkfCore::BuildDefaultInitialCovariance();

            Assert::AreEqual(1.0e-5f, covariance(VehicleState::kPx, VehicleState::kPx), 1.0e-9f);
            Assert::AreEqual(1.0e-5f, covariance(VehicleState::kPy, VehicleState::kPy), 1.0e-9f);
            Assert::AreEqual(1.0e-3f, covariance(VehicleState::kPsi, VehicleState::kPsi), 1.0e-9f);
            Assert::AreEqual(1.0e-3f, covariance(VehicleState::kU, VehicleState::kU), 1.0e-9f);
            Assert::AreEqual(1.0e-3f, covariance(VehicleState::kV, VehicleState::kV), 1.0e-9f);
            Assert::AreEqual(1.0e-3f, covariance(VehicleState::kR, VehicleState::kR), 1.0e-9f);
            Assert::AreEqual(0.25f, covariance(VehicleState::kOmegaL, VehicleState::kOmegaL), 1.0e-9f);
            Assert::AreEqual(0.25f, covariance(VehicleState::kOmegaR, VehicleState::kOmegaR), 1.0e-9f);
            Assert::AreEqual(0.01f, covariance(VehicleState::kBgz, VehicleState::kBgz), 1.0e-9f);
        }

        TEST_METHOD(SrUkfCorePredictSchedulesLateralProcessNoiseFromTurnSeverity)
        {
            const PlantParams params = PlantParams::Default();
            SrUkfCore core(params);
            ControlInput control{};
            control.batteryVoltageV = params.supplyVoltageV;

            Assert::IsTrue(core.setState(
                BuildUkfState(
                    0.0f,
                    0.0f,
                    0.0f,
                    0.01f,
                    0.0f,
                    12.0f,
                    0.0f,
                    0.0f),
                BuildUkfCovariance()));
            Assert::IsTrue(core.predict(0.01f, control));

            const float guardedNoise = FindProcessNoiseDiagonal(core, "v_mps");
            Assert::AreEqual(0.12f, guardedNoise, 1.0e-6f);

            Assert::IsTrue(core.setState(
                BuildUkfState(
                    0.0f,
                    0.0f,
                    0.0f,
                    2.0f,
                    0.0f,
                    10.0f,
                    0.0f,
                    0.0f),
                BuildUkfCovariance()));
            Assert::IsTrue(core.predict(0.01f, control));

            const float rhoSustain = std::fabs(2.0f * 10.0f) / Vehicle::GetSustainedLateralAccelerationReferenceMps2();
            const float expectedHighNoise = 2.50f + (4.00f * (rhoSustain - 1.00f));
            const float scheduledHighNoise = FindProcessNoiseDiagonal(core, "v_mps");
            Assert::AreEqual(expectedHighNoise, scheduledHighNoise, 1.0e-5f);
            Assert::AreEqual(0.0f, FindProcessNoiseDiagonal(core, "px_m"), 1.0e-9f);
            Assert::AreEqual(0.0f, FindProcessNoiseDiagonal(core, "py_m"), 1.0e-9f);
        }

        TEST_METHOD(SrUkfCoreZeroVelocityEncoderUpdateKeepsYawRateVarianceBoundedAtRest)
        {
            const PlantParams params = PlantParams::Default();
            PlantModel plant;
            SrUkfCore core(params);

            const VehicleState::StateVector initialState =
                BuildUkfState(
                    0.0f,
                    0.09f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f);
            const VehicleState::StateMatrix initialCovariance =
                BuildUkfCovariance(0.001f, 0.01f, 0.005f, 0.005f, 1.0f, 0.05f, 0.02f);
            Assert::IsTrue(core.reset(initialState, initialCovariance));

            ControlInput control{};
            control.fanDutyCycle = 0.80f;
            control.batteryVoltageV = params.supplyVoltageV;

            const ContactForces lowForceContacts = plant.tireForces(initialState, control, params);
            Assert::IsTrue(std::fabs(lowForceContacts.SumForwardForceN()) < 1.0e-4f);
            Assert::IsTrue(std::fabs(lowForceContacts.SumRightForceN()) < 1.0e-4f);

            const VehicleState::StateMatrix beforeCovariance = core.covariance();
            const float initialYawRateVarianceRadps2 =
                beforeCovariance(VehicleState::kR, VehicleState::kR);
            const float initialLeftWheelVarianceRadps2 =
                beforeCovariance(VehicleState::kOmegaL, VehicleState::kOmegaL);
            const float initialRightWheelVarianceRadps2 =
                beforeCovariance(VehicleState::kOmegaR, VehicleState::kOmegaR);

            Assert::IsTrue(initialLeftWheelVarianceRadps2 < (0.01f * initialYawRateVarianceRadps2));
            Assert::IsTrue(initialRightWheelVarianceRadps2 < (0.01f * initialYawRateVarianceRadps2));

            constexpr float dt = 0.001f;
            Assert::IsTrue(core.predict(dt, control));

            const VehicleState::StateMatrix predictedCovariance = core.covariance();
            Assert::IsTrue(std::isfinite(predictedCovariance(VehicleState::kR, VehicleState::kR)));
            Assert::IsTrue(std::isfinite(predictedCovariance(VehicleState::kOmegaL, VehicleState::kOmegaL)));
            Assert::IsTrue(std::isfinite(predictedCovariance(VehicleState::kOmegaR, VehicleState::kOmegaR)));

            EncoderObs encoder{};
            const MeasurementUpdateResult encoderResult = core.updateEncoderPair(encoder, dt);
            Assert::IsTrue(encoderResult.attempted);
            Assert::IsTrue(encoderResult.accepted);

            const VehicleState::StateMatrix afterCovariance = core.covariance();
            Assert::IsTrue(
                afterCovariance(VehicleState::kR, VehicleState::kR) <
                (0.1f * initialYawRateVarianceRadps2));
            Assert::IsTrue(
                afterCovariance(VehicleState::kPsi, VehicleState::kPsi) <
                (0.1f * initialYawRateVarianceRadps2));
        }

        TEST_METHOD(SrUkfCoreMovingEncoderUpdateKeepsYawRateVarianceLowAfterPredictAndUpdate)
        {
            const PlantParams params = PlantParams::Default();
            PlantModel plant;
            SrUkfCore core(params);
            const float distancePerCountM = DistancePerEncoderCountMeters(params);
            const float measuredWheelOmegaRadps = distancePerCountM / (params.wheelRadiusM * 0.001f);
            const float measuredLinearSpeedMps = params.wheelRadiusM * measuredWheelOmegaRadps;

            const VehicleState::StateVector initialState =
                BuildUkfState(
                    0.0f,
                    0.09f,
                    0.0f,
                    measuredLinearSpeedMps,
                    0.0f,
                    0.0f,
                    measuredWheelOmegaRadps,
                    measuredWheelOmegaRadps,
                    0.0f);
            const VehicleState::StateMatrix initialCovariance =
                BuildUkfCovariance(0.001f, 0.01f, 0.005f, 0.005f, 1.0f, 0.05f, 0.02f);
            Assert::IsTrue(core.reset(initialState, initialCovariance));

            ControlInput control{};
            control.fanDutyCycle = 0.80f;
            control.batteryVoltageV = params.supplyVoltageV;

            const ContactForces lowForceContacts = plant.tireForces(initialState, control, params);
            Assert::IsTrue(std::fabs(lowForceContacts.SumForwardForceN()) < 1.0e-4f);
            Assert::IsTrue(std::fabs(lowForceContacts.SumRightForceN()) < 1.0e-4f);

            const VehicleState::StateMatrix beforeCovariance = core.covariance();
            const float initialYawRateVarianceRadps2 =
                beforeCovariance(VehicleState::kR, VehicleState::kR);
            Assert::IsTrue(
                beforeCovariance(VehicleState::kOmegaL, VehicleState::kOmegaL) <
                (0.01f * initialYawRateVarianceRadps2));
            Assert::IsTrue(
                beforeCovariance(VehicleState::kOmegaR, VehicleState::kOmegaR) <
                (0.01f * initialYawRateVarianceRadps2));

            constexpr float dt = 0.001f;
            Assert::IsTrue(core.predict(dt, control));

            const VehicleState::StateMatrix predictedCovariance = core.covariance();
            Assert::IsTrue(std::isfinite(predictedCovariance(VehicleState::kR, VehicleState::kR)));
            Assert::IsTrue(std::isfinite(predictedCovariance(VehicleState::kOmegaL, VehicleState::kOmegaL)));
            Assert::IsTrue(std::isfinite(predictedCovariance(VehicleState::kOmegaR, VehicleState::kOmegaR)));

            EncoderObs encoder{};
            encoder.totalLeftCounts = 1;
            encoder.totalRightCounts = 1;
            encoder.omegaLeftRadps = core.state()(VehicleState::kOmegaL);
            encoder.omegaRightRadps = core.state()(VehicleState::kOmegaR);
            const MeasurementUpdateResult encoderResult = core.updateEncoderPair(encoder, dt);
            Assert::IsTrue(encoderResult.attempted);
            Assert::IsTrue(encoderResult.accepted);

            const VehicleState::StateMatrix afterCovariance = core.covariance();
            Assert::IsTrue(
                afterCovariance(VehicleState::kR, VehicleState::kR) <=
                (predictedCovariance(VehicleState::kR, VehicleState::kR) + 1.0e-9f));
            Assert::IsTrue(
                afterCovariance(VehicleState::kR, VehicleState::kR) <
                (0.1f * initialYawRateVarianceRadps2));
            Assert::IsTrue(std::isfinite(afterCovariance(VehicleState::kR, VehicleState::kOmegaL)));
            Assert::IsTrue(std::isfinite(afterCovariance(VehicleState::kR, VehicleState::kOmegaR)));
        }

        TEST_METHOD(SrUkfCoreStationaryYawConstraintRapidlyCollapsesStationaryMotionAndBiasCovariance)
        {
            SrUkfCore core;
            const VehicleState::StateVector initialState = BuildUkfState(
                0.0f,
                0.09f,
                0.0f,
                0.0f,
                0.35f,
                0.0f,
                0.0f,
                0.0f);
            Assert::IsTrue(core.reset(initialState, BuildUkfCovariance()));

            ControlInput control{};
            constexpr float dt = 0.001f;
            Assert::IsTrue(core.predict(dt, control));

            EncoderObs encoder{};
            const MeasurementUpdateResult encoderResult = core.updateEncoderPair(encoder, dt);
            Assert::IsTrue(encoderResult.attempted);
            Assert::IsTrue(encoderResult.accepted);

            const MeasurementUpdateResult yawResult = core.updateYawRate(0.0f);
            Assert::IsTrue(yawResult.attempted);
            Assert::IsTrue(yawResult.accepted);

            const VehicleState::StateVector& state = core.state();
            const VehicleState::StateMatrix covariance = core.covariance();
            Assert::AreEqual(0.0f, state(VehicleState::kU), 1.0e-6f);
            Assert::AreEqual(0.0f, state(VehicleState::kV), 1.0e-6f);
            Assert::AreEqual(0.0f, state(VehicleState::kR), 1.0e-6f);
            Assert::AreEqual(0.0f, state(VehicleState::kOmegaL), 1.0e-6f);
            Assert::AreEqual(0.0f, state(VehicleState::kOmegaR), 1.0e-6f);
            Assert::AreEqual(0.0f, state(VehicleState::kBgz), 1.0e-6f);
            Assert::IsTrue(covariance(VehicleState::kU, VehicleState::kU) <= 1.0e-12f);
            Assert::IsTrue(covariance(VehicleState::kV, VehicleState::kV) <= 1.0e-12f);
            Assert::IsTrue(covariance(VehicleState::kR, VehicleState::kR) <= 1.0e-12f);
            Assert::IsTrue(covariance(VehicleState::kOmegaL, VehicleState::kOmegaL) <= 1.0e-12f);
            Assert::IsTrue(covariance(VehicleState::kOmegaR, VehicleState::kOmegaR) <= 1.0e-12f);
            Assert::IsTrue(covariance(VehicleState::kBgz, VehicleState::kBgz) <= 1.0e-12f);
        }

        TEST_METHOD(SrUkfCoreHighUtilizationYawConstraintDoesNotZeroLateralVelocity)
        {
            SrUkfCore core;
            const VehicleState::StateVector initialState = BuildUkfState(
                0.0f,
                0.09f,
                0.0f,
                2.0f,
                0.40f,
                9.0f,
                0.0f,
                0.0f);
            Assert::IsTrue(core.reset(
                initialState,
                BuildUkfCovariance(0.01f, 0.03f, 0.05f, 0.30f, 0.05f, 0.30f, 0.03f)));

            EncoderObs encoder{};
            const MeasurementUpdateResult encoderResult = core.updateEncoderPair(encoder, 0.0f);
            Assert::IsTrue(encoderResult.attempted);
            Assert::IsTrue(encoderResult.accepted);

            const MeasurementUpdateResult yawResult = core.updateYawRate(9.0f);
            Assert::IsTrue(yawResult.attempted);
            Assert::IsTrue(yawResult.accepted);

            const VehicleState::StateVector& state = core.state();
            const VehicleState::StateMatrix covariance = core.covariance();
            Assert::AreEqual(0.0f, state(VehicleState::kU), 1.0e-6f);
            Assert::AreEqual(0.0f, state(VehicleState::kR), 1.0e-6f);
            Assert::IsTrue(std::fabs(state(VehicleState::kV)) > 0.10f);
            Assert::IsTrue(covariance(VehicleState::kV, VehicleState::kV) > 1.0e-2f);
        }

        TEST_METHOD(SrUkfCoreDebugTextDumpIncludesStateCovarianceNoiseAndPlantConfiguration)
        {
            SrUkfCore core;
            const VehicleState::StateVector state = BuildUkfState(
                0.05f,
                0.11f,
                0.02f,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                0.003f);
            const VehicleState::StateMatrix covariance = BuildUkfCovariance();
            Assert::IsTrue(core.reset(state, covariance));

            ControlInput control{};
            control.leftMotorCommand = 0.25f;
            control.rightMotorCommand = 0.35f;
            control.fanDutyCycle = 0.80f;
            control.batteryVoltageV = 7.95f;
            Assert::IsTrue(core.predict(0.01f, control));

            EncoderObs encoderObservation{};
            encoderObservation.totalLeftCounts = 8;
            encoderObservation.totalRightCounts = 9;
            encoderObservation.omegaLeftRadps = 1.2f;
            encoderObservation.omegaRightRadps = 1.3f;
            const MeasurementUpdateResult encoderResult = core.updateEncoderPair(encoderObservation, 0.01f);
            Assert::IsTrue(encoderResult.attempted);

            std::vector<std::pair<std::string, std::string>> dumpLines;
            Assert::IsTrue(core.WriteDebugTextDump(
                [&dumpLines](const char* type, const char* message) noexcept
                {
                    dumpLines.emplace_back(
                        (type != nullptr) ? type : "",
                        (message != nullptr) ? message : "");
                    return true;
                }));

            auto countType =
                [&dumpLines](const char* type)
                {
                    std::size_t count = 0U;
                    for (const auto& line : dumpLines)
                    {
                        if (line.first == type)
                        {
                            ++count;
                        }
                    }
                    return count;
                };
            auto findMessage =
                [&dumpLines](const char* type)
                {
                    for (const auto& line : dumpLines)
                    {
                        if (line.first == type)
                        {
                            return line.second;
                        }
                    }
                    return std::string();
                };

            Assert::IsTrue(dumpLines.size() >= 42U);
            Assert::AreEqual(static_cast<std::size_t>(9U), countType("ukf_dump_covariance_row"));
            Assert::AreEqual(static_cast<std::size_t>(9U), countType("ukf_dump_process_noise_sqrt_row"));
            Assert::AreEqual(static_cast<std::size_t>(4U), countType("ukf_dump_contact_position"));
            Assert::AreEqual(static_cast<std::size_t>(3U), countType("ukf_dump_imu_noise_sqrt_row"));
            Assert::AreEqual(static_cast<std::size_t>(2U), countType("ukf_dump_front_noise_sqrt_row"));
            Assert::AreEqual(static_cast<std::size_t>(1U), countType("ukf_dump_side_noise_sqrt_row"));

            const std::string stateLine = findMessage("ukf_dump_state");
            Assert::IsTrue(stateLine.find("px_m=") != std::string::npos);
            Assert::IsTrue(stateLine.find("bgz_radps=") != std::string::npos);

            const std::string predictionReferenceLine = findMessage("ukf_dump_prediction_reference");
            Assert::IsTrue(predictionReferenceLine.find("have_prediction_reference=true") != std::string::npos);

            const std::string lastControlLine = findMessage("ukf_dump_last_control");
            Assert::IsTrue(lastControlLine.find("left_motor_command=0.25") != std::string::npos);
            Assert::IsTrue(lastControlLine.find("right_motor_command=0.349999994") != std::string::npos ||
                           lastControlLine.find("right_motor_command=0.35") != std::string::npos);
            Assert::IsTrue(lastControlLine.find("battery_voltage_v=7.94999981") != std::string::npos ||
                           lastControlLine.find("battery_voltage_v=7.95") != std::string::npos);

            const std::string lastEncoderLine = findMessage("ukf_dump_last_encoder_obs");
            Assert::IsTrue(lastEncoderLine.find("total_left_counts=8") != std::string::npos);
            Assert::IsTrue(lastEncoderLine.find("total_right_counts=9") != std::string::npos);
            Assert::IsTrue(lastEncoderLine.find("omega_left_radps=1.20000005") != std::string::npos ||
                           lastEncoderLine.find("omega_left_radps=1.2") != std::string::npos);

            Assert::IsFalse(findMessage("ukf_dump_params_mass_geometry").empty());
            Assert::IsFalse(findMessage("ukf_dump_imu_extrinsics").empty());
        }

        TEST_METHOD(SrUkfCoreRejectsInvalidMergedImuUpdate)
        {
            SrUkfCore core;
            ImuMergedObs observation{};
            observation.valid = false;
            observation.gyroZRadps = 0.5f;
            observation.accelBodyXMps2 = 1.0f;
            observation.accelBodyYMps2 = 0.1f;

            const MeasurementUpdateResult result = core.updateImuMerged(observation);
            Assert::IsFalse(result.attempted);
            Assert::IsFalse(result.accepted);
        }

        TEST_METHOD(SrUkfCoreMergedImuUpdateIgnoresPlanarAccelValues)
        {
            SrUkfCore baselineCore;
            SrUkfCore perturbedCore;
            ControlInput control{};
            EncoderObs encoder{};
            constexpr float dt = 0.0005f;
            constexpr float rawStationaryGyroRadps = 0.12f;

            Assert::IsTrue(baselineCore.predict(dt, control));
            Assert::IsTrue(perturbedCore.predict(dt, control));

            const MeasurementUpdateResult baselineEncoderResult = baselineCore.updateEncoderPair(encoder, dt);
            const MeasurementUpdateResult perturbedEncoderResult = perturbedCore.updateEncoderPair(encoder, dt);
            Assert::IsTrue(baselineEncoderResult.accepted);
            Assert::IsTrue(perturbedEncoderResult.accepted);

            ImuMergedObs baselineObservation{};
            baselineObservation.valid = true;
            baselineObservation.gyroZRadps = rawStationaryGyroRadps;
            baselineObservation.accelBodyXMps2 = 0.0f;
            baselineObservation.accelBodyYMps2 = 0.0f;

            ImuMergedObs perturbedObservation = baselineObservation;
            perturbedObservation.accelBodyXMps2 = 250.0f;
            perturbedObservation.accelBodyYMps2 = -175.0f;

            const MeasurementUpdateResult baselineResult = baselineCore.updateImuMerged(baselineObservation);
            const MeasurementUpdateResult perturbedResult = perturbedCore.updateImuMerged(perturbedObservation);
            Assert::IsTrue(baselineResult.attempted);
            Assert::IsTrue(baselineResult.accepted);
            Assert::IsTrue(perturbedResult.attempted);
            Assert::IsTrue(perturbedResult.accepted);

            const VehicleState::StateVector baselineState = baselineCore.state();
            const VehicleState::StateVector perturbedState = perturbedCore.state();
            const VehicleState::StateMatrix baselineCovariance = baselineCore.covariance();
            const VehicleState::StateMatrix perturbedCovariance = perturbedCore.covariance();

            Assert::IsTrue((baselineState - perturbedState).cwiseAbs().maxCoeff() <= 1.0e-7f);
            Assert::IsTrue((baselineCovariance - perturbedCovariance).cwiseAbs().maxCoeff() <= 1.0e-7f);
            Assert::AreEqual(0.0f, baselineState(VehicleState::kR), 1.0e-6f);
            Assert::AreEqual(rawStationaryGyroRadps, baselineState(VehicleState::kBgz), 1.0e-6f);
        }

        TEST_METHOD(SrUkfCorePlanarAccelUpdateIsDisabledAndLeavesFilterStateUntouched)
        {
            SrUkfCore core;
            const VehicleState::StateVector initialState =
                BuildUkfState(
                    0.03f,
                    0.11f,
                    0.08f,
                    1.2f,
                    0.02f,
                    0.15f,
                    12.0f,
                    12.0f,
                    0.01f);
            const VehicleState::StateMatrix initialCovariance =
                BuildUkfCovariance(0.02f, 0.05f, 0.20f, 0.15f, 0.25f, 0.50f, 0.05f);
            Assert::IsTrue(core.reset(initialState, initialCovariance));

            const VehicleState::StateVector beforeState = core.state();
            const VehicleState::StateMatrix beforeCovariance = core.covariance();

            ImuAccelObs observation{};
            observation.valid = true;
            observation.accelBodyXMps2 = 123.0f;
            observation.accelBodyYMps2 = -87.0f;
            const MeasurementUpdateResult result = core.updatePlanarAccel(observation);

            const VehicleState::StateVector afterState = core.state();
            const VehicleState::StateMatrix afterCovariance = core.covariance();
            Assert::IsFalse(result.attempted);
            Assert::IsTrue(result.accepted);
            Assert::IsTrue((afterState - beforeState).cwiseAbs().maxCoeff() <= 1.0e-7f);
            Assert::IsTrue((afterCovariance - beforeCovariance).cwiseAbs().maxCoeff() <= 1.0e-7f);
        }

        TEST_METHOD(SquareRootUkfSymmetrizesAsymmetricCovarianceInputs)
        {
            UKF<4, 2> filter;

            FilterState initialState;
            initialState << 0.2f, -0.1f, 0.4f, -0.3f;
            FilterCovariance initialCovariance = FilterCovariance::Identity() * 0.1f;
            initialCovariance(0, 1) = 0.03f;
            initialCovariance(1, 0) = -0.01f;
            initialCovariance(2, 3) = 0.02f;
            initialCovariance(3, 2) = -0.015f;
            filter.setState(initialState, initialCovariance);

            FilterCovariance processNoise = FilterCovariance::Zero();
            processNoise(0, 0) = 1.0e-4f;
            processNoise(1, 1) = 2.0e-4f;
            processNoise(2, 2) = 3.0e-4f;
            processNoise(3, 3) = 4.0e-4f;
            processNoise(0, 2) = 5.0e-5f;
            processNoise(2, 0) = -2.0e-5f;
            filter.setProcessNoise(processNoise);

            FilterControl control;
            control << 0.1f, -0.08f;

            const bool predictOk = filter.Predict(
                0.002f,
                control,
                [](const FilterState& sigmaPoint, const FilterControl& sigmaControl, float sigmaDt) noexcept
                {
                    FilterState predicted = sigmaPoint;
                    predicted(0) += sigmaDt * sigmaPoint(2);
                    predicted(1) += sigmaDt * sigmaPoint(3);
                    predicted(2) += sigmaDt * sigmaControl(0);
                    predicted(3) += sigmaDt * sigmaControl(1);
                    return predicted;
                });

            Assert::IsTrue(predictOk);
            const FilterCovariance covariance = filter.covariance();
            const FilterCovariance asymmetry = covariance - covariance.transpose();
            Assert::IsTrue(asymmetry.cwiseAbs().maxCoeff() <= 1.0e-6f);

            const Eigen::LLT<FilterCovariance> llt(covariance);
            Assert::IsTrue(llt.info() == Eigen::Success);
        }

        TEST_METHOD(SrUkfCoreUsesStationaryGyroMeasurementToConstrainBiasState)
        {
            SrUkfCore core;
            ControlInput control{};
            EncoderObs encoder{};
            constexpr float dt = 0.0005f;

            Assert::IsTrue(core.predict(dt, control));
            const MeasurementUpdateResult encoderResult = core.updateEncoderPair(encoder, dt);
            Assert::IsTrue(encoderResult.accepted);

            const float rawStationaryGyroRadps = 0.12f;
            const MeasurementUpdateResult yawResult = core.updateYawRate(rawStationaryGyroRadps);
            Assert::IsTrue(yawResult.attempted);
            Assert::IsTrue(yawResult.accepted);

            const VehicleState::StateVector& state = core.state();
            Assert::AreEqual(0.0f, state(VehicleState::kR), 1.0e-6f);
            Assert::AreEqual(rawStationaryGyroRadps, state(VehicleState::kBgz), 1.0e-6f);
        }

        TEST_METHOD(SrUkfCoreStationaryGyroMeasurementDropsBiasVarianceNearZero)
        {
            SrUkfCore core;
            ControlInput control{};
            EncoderObs encoder{};
            constexpr float dt = 0.0005f;
            constexpr float rawStationaryGyroRadps = 0.12f;

            const VehicleState::StateMatrix beforeCovariance = core.covariance();
            Assert::IsTrue(
                beforeCovariance(VehicleState::kBgz, VehicleState::kBgz) >
                0.0f);

            Assert::IsTrue(core.predict(dt, control));
            const MeasurementUpdateResult encoderResult = core.updateEncoderPair(encoder, dt);
            Assert::IsTrue(encoderResult.attempted);
            Assert::IsTrue(encoderResult.accepted);

            const MeasurementUpdateResult yawResult = core.updateYawRate(rawStationaryGyroRadps);
            Assert::IsTrue(yawResult.attempted);
            Assert::IsTrue(yawResult.accepted);

            const VehicleState::StateMatrix covariance = core.covariance();
            Assert::IsTrue(covariance(VehicleState::kBgz, VehicleState::kBgz) <= 1.0e-12f);
            Assert::IsTrue(
                covariance(VehicleState::kBgz, VehicleState::kBgz) <
                beforeCovariance(VehicleState::kBgz, VehicleState::kBgz));
        }

        TEST_METHOD(SrUkfCoreLatestEncoderObservationPullsWheelRatesTowardLatestMeasurement)
        {
            const PlantParams params = PlantParams::Default();
            const float distancePerCountM = DistancePerEncoderCountMeters(params);
            const auto omegaFromCounts = [distancePerCountM, &params](int32_t counts, float dtSeconds) noexcept
            {
                return (static_cast<float>(counts) * distancePerCountM) / (params.wheelRadiusM * dtSeconds);
            };
            SrUkfCore core(params);
            ControlInput control{};
            constexpr float dt = 0.010f;

            Assert::IsTrue(core.predict(dt, control));
            EncoderObs first{};
            first.totalLeftCounts = 2;
            first.totalRightCounts = -1;
            first.omegaLeftRadps = omegaFromCounts(first.totalLeftCounts, dt);
            first.omegaRightRadps = omegaFromCounts(first.totalRightCounts, dt);
            const MeasurementUpdateResult firstResult = core.updateEncoderPair(first, dt);
            Assert::IsTrue(firstResult.attempted);
            Assert::IsTrue(firstResult.accepted);

            Assert::IsTrue(core.predict(dt, control));
            EncoderObs second{};
            second.totalLeftCounts = 5;
            second.totalRightCounts = -3;
            second.omegaLeftRadps = omegaFromCounts(second.totalLeftCounts, dt);
            second.omegaRightRadps = omegaFromCounts(second.totalRightCounts, dt);
            const MeasurementUpdateResult secondResult = core.updateEncoderPair(second, dt);
            Assert::IsTrue(secondResult.attempted);
            Assert::IsTrue(secondResult.accepted);

            const VehicleState::StateVector& state = core.state();
            Assert::IsTrue(std::isfinite(state(VehicleState::kOmegaL)));
            Assert::IsTrue(std::isfinite(state(VehicleState::kOmegaR)));
            Assert::IsTrue(state(VehicleState::kOmegaL) > 0.0f);
            Assert::IsTrue(state(VehicleState::kOmegaR) < 0.0f);
            Assert::IsTrue(
                std::fabs(state(VehicleState::kOmegaL) - second.omegaLeftRadps) <
                std::fabs(first.omegaLeftRadps - second.omegaLeftRadps));
            Assert::IsTrue(
                std::fabs(state(VehicleState::kOmegaR) - second.omegaRightRadps) <
                std::fabs(first.omegaRightRadps - second.omegaRightRadps));
        }

        TEST_METHOD(SrUkfCoreDoesNotLetControlInputCreateUnboundedForwardMotionWithEncoderOpposition)
        {
            SrUkfCore core;
            const PlantParams params = PlantParams::Default();
            ControlInput control{};
            control.leftMotorCommand = 0.18f;
            control.rightMotorCommand = 0.18f;
            control.fanDutyCycle = 0.80f;
            control.batteryVoltageV = params.supplyVoltageV;
            EncoderObs encoder{};
            constexpr float dt = 0.001f;

            for (int step = 0; step < 200; ++step)
            {
                Assert::IsTrue(core.predict(dt, control));

                const MeasurementUpdateResult encoderResult = core.updateEncoderPair(encoder, dt);
                Assert::IsTrue(encoderResult.attempted);
                Assert::IsTrue(encoderResult.accepted);

                const MeasurementUpdateResult yawResult = core.updateYawRate(0.0f);
                Assert::IsTrue(yawResult.attempted);
                Assert::IsTrue(yawResult.accepted);
            }

            const VehicleState::StateVector& state = core.state();
            Assert::IsTrue(std::fabs(state(VehicleState::kPx)) < 1.0e-3f);
            Assert::IsTrue(std::fabs(state(VehicleState::kPy)) < 1.0e-3f);
            Assert::IsTrue(std::fabs(state(VehicleState::kU)) < 1.0e-4f);
            Assert::IsTrue(std::fabs(state(VehicleState::kOmegaL)) < 1.0e-4f);
            Assert::IsTrue(std::fabs(state(VehicleState::kOmegaR)) < 1.0e-4f);
        }

        TEST_METHOD(SrUkfCoreMustLetControlInputCreateForwardMotionWithNoEncoder)
        {
            SrUkfCore core;
            const PlantParams params = PlantParams::Default();
            ControlInput control{};
            control.leftMotorCommand = 0.5f;
            control.rightMotorCommand = 0.5f;
            control.fanDutyCycle = 0.80f;
            control.batteryVoltageV = params.supplyVoltageV;
            EncoderObs encoder{};
            constexpr float dt = 0.001f;

            for (int step = 0; step < 200; ++step)
            {
                Assert::IsTrue(core.predict(dt, control));

                const MeasurementUpdateResult yawResult = core.updateYawRate(0.0f);
                Assert::IsTrue(yawResult.attempted);
                Assert::IsTrue(yawResult.accepted);
            }

            const VehicleState::StateVector& state = core.state();
            Assert::IsTrue(std::fabs(state(VehicleState::kPy)) > 1.0e-2f);
            Assert::IsTrue(std::fabs(state(VehicleState::kU)) > 1.0e-2f);
        }

        TEST_METHOD(SrUkfCoreAcceptsLaunchEncoderDeltasWhenOpenLoopPredictionDisagrees)
        {
            struct LaunchEncoderSample
            {
                float dtSeconds;
                float leftOmegaRadps;
                float rightOmegaRadps;
                float gyroRawRadps;
            };

            // D:\open_floor_main.csv, April 10, 2026, repeat 11 launch pulse.
            // The raw launch command makes the open-loop plant prediction disagree
            // with measured wheel rates; encoder deltas must still remain authoritative.
            constexpr LaunchEncoderSample samples[] = {
                { 0.001011f, 3.99f, 3.99f, -0.019f },
                { 0.001015f, 9.67f, 15.20f, 0.002f },
                { 0.001005f, 8.72f, 13.76f, 0.092f },
                { 0.001020f, 6.49f, 12.05f, 0.203f },
                { 0.001004f, 5.02f, 9.59f, 0.152f },
                { 0.001019f, 4.64f, 8.35f, 0.149f },
                { 0.001005f, 5.48f, 6.40f, 0.116f },
                { 0.001000f, 6.49f, 6.49f, -0.006f },
                { 0.001005f, 8.38f, 5.59f, -0.108f },
                { 0.001000f, 10.19f, 6.95f, -0.156f },
            };

            const PlantParams params = PlantParams::Default();
            const float distancePerCountM = DistancePerEncoderCountMeters(params);
            const auto countsFromOmega = [distancePerCountM, &params](float omegaRadps, float dtSeconds) noexcept
            {
                const float counts =
                    (omegaRadps * params.wheelRadiusM * dtSeconds) / distancePerCountM;
                return static_cast<int32_t>((counts >= 0.0f) ? (counts + 0.5f) : (counts - 0.5f));
            };

            SrUkfCore core(params);
            const VehicleState::StateVector initialState =
                BuildUkfState(
                    0.225f,
                    0.225f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f);
            Assert::IsTrue(core.reset(initialState, BuildUkfCovariance()));

            ControlInput control{};
            control.leftMotorCommand = 0.08f;
            control.rightMotorCommand = 0.08f;
            control.fanDutyCycle = 0.80f;
            control.batteryVoltageV = params.supplyVoltageV;

            for (int index = 0; index < static_cast<int>(sizeof(samples) / sizeof(samples[0])); ++index)
            {
                const LaunchEncoderSample& sample = samples[index];
                Assert::IsTrue(core.predict(sample.dtSeconds, control));

                EncoderObs encoder{};
                encoder.totalLeftCounts = countsFromOmega(sample.leftOmegaRadps, sample.dtSeconds);
                encoder.totalRightCounts = countsFromOmega(sample.rightOmegaRadps, sample.dtSeconds);
                encoder.omegaLeftRadps = sample.leftOmegaRadps;
                encoder.omegaRightRadps = sample.rightOmegaRadps;
                const MeasurementUpdateResult encoderResult = core.updateEncoderPair(encoder, sample.dtSeconds);

                Assert::IsTrue(encoderResult.attempted);
                Assert::IsTrue(
                    encoderResult.accepted,
                    (std::wstring(L"Launch encoder update rejected at sample ") +
                        std::to_wstring(index)).c_str());

                const VehicleState::StateVector& encoderConstrainedState = core.state();
                Assert::IsTrue(std::isfinite(encoderConstrainedState(VehicleState::kU)));
                Assert::AreEqual(sample.leftOmegaRadps, encoderConstrainedState(VehicleState::kOmegaL), 1.0e-5f);
                Assert::AreEqual(sample.rightOmegaRadps, encoderConstrainedState(VehicleState::kOmegaR), 1.0e-5f);

                const MeasurementUpdateResult yawResult = core.updateYawRate(sample.gyroRawRadps);
                Assert::IsTrue(yawResult.attempted);
                Assert::IsTrue(yawResult.accepted);
            }
        }

        TEST_METHOD(SrUkfCoreEncoderArcUpdateUsesProjectTurnSignConventions)
        {
            const PlantParams params = PlantParams::Default();
            SrUkfCore core(params);
            const VehicleState::StateVector initialState =
                BuildUkfState(
                    0.0f,
                    0.09f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f);
            Assert::IsTrue(core.reset(
                initialState,
                BuildUkfCovariance(0.005f, 0.01f, 0.01f, 0.01f, 0.02f, 0.05f, 0.01f)));

            ControlInput control{};
            control.fanDutyCycle = 0.80f;
            control.batteryVoltageV = params.supplyVoltageV;

            constexpr float dt = 0.010f;
            Assert::IsTrue(core.predict(dt, control));

            const float distancePerCountM =
                (2.0f * PI_F * params.wheelRadiusM) /
                (params.gearRatio * static_cast<float>(params.encoderCountsPerMotorRev));
            EncoderObs encoder{};
            encoder.totalLeftCounts = 4;
            encoder.totalRightCounts = 2;
            encoder.omegaLeftRadps =
                (static_cast<float>(encoder.totalLeftCounts) * distancePerCountM) /
                (params.wheelRadiusM * dt);
            encoder.omegaRightRadps =
                (static_cast<float>(encoder.totalRightCounts) * distancePerCountM) /
                (params.wheelRadiusM * dt);

            const MeasurementUpdateResult result = core.updateEncoderPair(encoder, dt);
            Assert::IsTrue(result.attempted);
            Assert::IsTrue(result.accepted);

            const float expectedForwardDistanceM =
                0.5f * static_cast<float>(encoder.totalLeftCounts + encoder.totalRightCounts) * distancePerCountM;
            const float expectedYawRad =
                static_cast<float>(encoder.totalLeftCounts - encoder.totalRightCounts) * distancePerCountM / params.trackWidthM;
            const VehicleState::StateVector& state = core.state();

            Assert::IsTrue(std::isfinite(state.sum()));
            Assert::IsTrue(state(VehicleState::kPy) > initialState(VehicleState::kPy));
            Assert::IsTrue(state(VehicleState::kPsi) > 0.0f);
            Assert::IsTrue(std::fabs(state(VehicleState::kPx)) < expectedForwardDistanceM);
            Assert::IsTrue(std::fabs(state(VehicleState::kPsi)) <= (2.0f * expectedYawRad));
            Assert::IsTrue(state(VehicleState::kU) > 0.0f);
        }

        TEST_METHOD(SrUkfCoreSplitDrivePredictBuildsTurnRateWhileKeepingForwardProgress)
        {
            const PlantParams params = PlantParams::Default();
            SrUkfCore core(params);
            constexpr float initialForwardVelocityMps = 1.0f;
            const float initialWheelSpeedRadps = initialForwardVelocityMps / params.wheelRadiusM;
            const VehicleState::StateVector initialState =
                BuildUkfState(
                    0.0f,
                    0.09f,
                    0.0f,
                    initialForwardVelocityMps,
                    0.0f,
                    0.0f,
                    initialWheelSpeedRadps,
                    initialWheelSpeedRadps);
            Assert::IsTrue(core.reset(initialState, BuildUkfCovariance()));

            ControlInput control{};
            control.leftMotorCommand = 0.30f;
            control.rightMotorCommand = 0.60f;
            control.fanDutyCycle = 0.80f;
            control.batteryVoltageV = params.supplyVoltageV;

            constexpr float dt = 0.002f;
            constexpr int kSteps = 75;
            for (int step = 0; step < kSteps; ++step)
            {
                Assert::IsTrue(core.predict(dt, control));
            }

            const VehicleState::StateVector& state = core.state();
            Assert::IsTrue(state(VehicleState::kPy) > initialState(VehicleState::kPy));
            Assert::IsTrue(state(VehicleState::kU) > 0.0f);
            Assert::IsTrue(state(VehicleState::kPsi) < 0.0f);
            Assert::IsTrue(state(VehicleState::kPx) < 0.0f);
        }

        TEST_METHOD(SrUkfCoreRepeatedForwardEncoderUpdatesStayMostlyStraightAndBoundAcceleration)
        {
            const PlantParams params = PlantParams::Default();
            SrUkfCore core(params);
            const VehicleState::StateVector initialState =
                BuildUkfState(
                    0.0f,
                    0.12f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f);
            Assert::IsTrue(core.reset(initialState, BuildUkfCovariance(0.01f, 0.03f, 0.05f, 0.05f, 0.08f, 0.20f, 0.03f)));

            const float distancePerCountM =
                (2.0f * PI_F * params.wheelRadiusM) /
                (params.gearRatio * static_cast<float>(params.encoderCountsPerMotorRev));
            constexpr float dt = 0.005f;
            const float expectedSpeedMps = distancePerCountM / dt;
            const float measuredOmegaRadps = expectedSpeedMps / params.wheelRadiusM;
            ControlInput control{};
            control.fanDutyCycle = 0.80f;
            control.batteryVoltageV = params.supplyVoltageV;

            float previousPositionM = initialState(VehicleState::kPy);
            float previousSpeedMps = expectedSpeedMps;
            constexpr int kSteps = 5;
            for (int step = 0; step < kSteps; ++step)
            {
                Assert::IsTrue(core.predict(dt, control));

                EncoderObs encoder{};
                encoder.totalLeftCounts = 1;
                encoder.totalRightCounts = 1;
                encoder.omegaLeftRadps = measuredOmegaRadps;
                encoder.omegaRightRadps = measuredOmegaRadps;
                const MeasurementUpdateResult result = core.updateEncoderPair(encoder, dt);
                Assert::IsTrue(result.attempted);
                Assert::IsTrue(result.accepted);

                const VehicleState::StateVector& state = core.state();
                const float intervalDistanceM = state(VehicleState::kPy) - previousPositionM;
                const float intervalSpeedMps = intervalDistanceM / dt;
                const float intervalAccelMps2 = (intervalSpeedMps - previousSpeedMps) / dt;
                const float expectedPositionM =
                    initialState(VehicleState::kPy) + ((step + 1.0f) * distancePerCountM);

                Assert::IsTrue(intervalDistanceM > 0.0f);
                Assert::IsTrue(
                    std::fabs(state(VehicleState::kPx)) <
                    (0.25f * (state(VehicleState::kPy) - initialState(VehicleState::kPy))));
                Assert::AreEqual(expectedPositionM, state(VehicleState::kPy), distancePerCountM * 0.25f);
                Assert::AreEqual(expectedSpeedMps, intervalSpeedMps, 0.15f);
                Assert::IsTrue(std::fabs(intervalAccelMps2) < 60.0f);
                Assert::IsTrue(state(VehicleState::kU) > 0.0f);
                Assert::IsTrue(state(VehicleState::kU) < 3.0f);

                previousPositionM = state(VehicleState::kPy);
                previousSpeedMps = intervalSpeedMps;
            }
        }

        TEST_METHOD(SrUkfCoreSingleSymmetricEncoderCountAdvancesForwardByAboutOneCount)
        {
            SrUkfCore core;
            const PlantParams params = PlantParams::Default();
            ControlInput control{};
            control.leftMotorCommand = 0.18f;
            control.rightMotorCommand = 0.18f;
            control.fanDutyCycle = 0.80f;
            control.batteryVoltageV = params.supplyVoltageV;
            constexpr float dt = 0.001f;

            const float distancePerCountM =
                (2.0f * PI_F * params.wheelRadiusM) /
                (params.gearRatio * static_cast<float>(params.encoderCountsPerMotorRev));
            const float measuredOmegaRadps = distancePerCountM / (params.wheelRadiusM * dt);

            Assert::IsTrue(core.predict(dt, control));

            EncoderObs encoder{};
            encoder.totalLeftCounts = 1;
            encoder.totalRightCounts = 1;
            encoder.omegaLeftRadps = measuredOmegaRadps;
            encoder.omegaRightRadps = measuredOmegaRadps;
            const MeasurementUpdateResult encoderResult = core.updateEncoderPair(encoder, dt);
            Assert::IsTrue(encoderResult.attempted);
            Assert::IsTrue(encoderResult.accepted);

            const VehicleState::StateVector& state = core.state();
            Assert::IsTrue(std::fabs(state(VehicleState::kPx)) < distancePerCountM);
            Assert::IsTrue(std::fabs(state(VehicleState::kPy) - distancePerCountM) < (0.5f * distancePerCountM));
            Assert::IsTrue(state(VehicleState::kU) > 0.0f);
            Assert::IsTrue(state(VehicleState::kU) < (2.0f * (distancePerCountM / dt)));
        }

        TEST_METHOD(SrUkfCoreZeroEncoderObservationCollapsesMotionStateWithoutTeleportingPose)
        {
            const PlantParams params = PlantParams::Default();
            SrUkfCore core(params);
            constexpr float initialForwardVelocityMps = 1.2f;
            const float wheelSpeedRadps = initialForwardVelocityMps / params.wheelRadiusM;
            const VehicleState::StateVector initialState =
                BuildUkfState(
                    0.03f,
                    0.11f,
                    0.08f,
                    initialForwardVelocityMps,
                    0.02f,
                    0.15f,
                    wheelSpeedRadps,
                    wheelSpeedRadps);
            Assert::IsTrue(core.reset(initialState, BuildUkfCovariance(0.02f, 0.05f, 0.20f, 0.15f, 0.25f, 0.50f, 0.05f)));

            ControlInput control{};
            control.leftMotorCommand = 0.40f;
            control.rightMotorCommand = 0.40f;
            control.fanDutyCycle = 0.80f;
            control.batteryVoltageV = params.supplyVoltageV;
            constexpr float dt = 0.01f;
            Assert::IsTrue(core.predict(dt, control));
            const VehicleState::StateMatrix predictedCovariance = core.covariance();

            EncoderObs encoder{};
            const MeasurementUpdateResult result = core.updateEncoderPair(encoder, dt);
            Assert::IsTrue(result.attempted);
            Assert::IsTrue(result.accepted);

            const VehicleState::StateVector& state = core.state();
            const VehicleState::StateMatrix covariance = core.covariance();
            const float maxOpenLoopTravelM = initialForwardVelocityMps * dt;
            Assert::IsTrue(std::fabs(state(VehicleState::kPx) - initialState(VehicleState::kPx)) < maxOpenLoopTravelM);
            Assert::IsTrue(std::fabs(state(VehicleState::kPy) - initialState(VehicleState::kPy)) < maxOpenLoopTravelM);
            Assert::IsTrue(std::fabs(state(VehicleState::kU)) < 1.0e-6f);
            Assert::IsTrue(std::fabs(state(VehicleState::kOmegaL)) < 1.0e-6f);
            Assert::IsTrue(std::fabs(state(VehicleState::kOmegaR)) < 1.0e-6f);
            Assert::IsTrue(
                covariance(VehicleState::kU, VehicleState::kU) <
                (0.1f * predictedCovariance(VehicleState::kU, VehicleState::kU)));
            Assert::IsTrue(
                covariance(VehicleState::kOmegaL, VehicleState::kOmegaL) <
                (0.1f * predictedCovariance(VehicleState::kOmegaL, VehicleState::kOmegaL)));
            Assert::IsTrue(
                covariance(VehicleState::kOmegaR, VehicleState::kOmegaR) <
                (0.1f * predictedCovariance(VehicleState::kOmegaR, VehicleState::kOmegaR)));
            Assert::IsTrue(std::isfinite(covariance(VehicleState::kPy, VehicleState::kPy)));
            Assert::IsTrue(covariance(VehicleState::kPy, VehicleState::kPy) > 1.0e-12f);
        }

        TEST_METHOD(SrUkfCoreYawRateUpdateImprovesYawMeasurementFitWithoutMovingPosition)
        {
            SrUkfCore core;
            const VehicleState::StateVector initialState =
                BuildUkfState(
                    0.02f,
                    0.14f,
                    0.10f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f);
            Assert::IsTrue(core.reset(initialState, BuildUkfCovariance(0.01f, 0.04f, 0.02f, 0.02f, 0.30f, 0.10f, 0.30f)));

            const VehicleState::StateVector before = core.state();
            const VehicleState::StateMatrix beforeCovariance = core.covariance();
            constexpr float observedYawRateRadps = 0.35f;
            const float beforeError =
                std::fabs((before(VehicleState::kR) + before(VehicleState::kBgz)) - observedYawRateRadps);

            const MeasurementUpdateResult result = core.updateYawRate(observedYawRateRadps);
            Assert::IsTrue(result.attempted);
            Assert::IsTrue(result.accepted);

            const VehicleState::StateVector& after = core.state();
            const VehicleState::StateMatrix afterCovariance = core.covariance();
            const float afterError =
                std::fabs((after(VehicleState::kR) + after(VehicleState::kBgz)) - observedYawRateRadps);

            Assert::AreEqual(before(VehicleState::kPx), after(VehicleState::kPx), 1.0e-6f);
            Assert::AreEqual(before(VehicleState::kPy), after(VehicleState::kPy), 1.0e-6f);
            Assert::IsTrue(afterError < beforeError);
            Assert::IsTrue(afterCovariance(VehicleState::kBgz, VehicleState::kBgz) <
                beforeCovariance(VehicleState::kBgz, VehicleState::kBgz));
        }

        TEST_METHOD(SrUkfCoreFrontWallUpdateMovesForwardForCloserSymmetricObservation)
        {
            Maze maze;
            maze.SetWall(maze(0, 0), Direction::Up, WallState::Wall);
            const LocalMapView map = BuildLocalMapView(maze);
            const PlantParams params = PlantParams::Default();
            const VehicleState::StateVector initialState =
                BuildUkfState(
                    0.09f,
                    0.09f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f);

            WallGeometryModel geometry;
            const GeometryPrediction leftPrediction = geometry.predictRay(initialState, params.frontLeftSensor, map);
            const GeometryPrediction rightPrediction = geometry.predictRay(initialState, params.frontRightSensor, map);
            Assert::IsTrue(leftPrediction.hit);
            Assert::IsTrue(rightPrediction.hit);

            SrUkfCore core(params);
            Assert::IsTrue(core.reset(initialState, BuildUkfCovariance(0.02f, 0.04f, 0.02f, 0.02f, 0.05f, 0.05f, 0.02f)));
            const VehicleState::StateVector before = core.state();
            const VehicleState::StateMatrix beforeCovariance = core.covariance();

            WallObs leftObservation{};
            leftObservation.valid = true;
            leftObservation.confidence = 1.0f;
            leftObservation.cls = ObsClass::WallLike;
            leftObservation.rho = leftPrediction.rangeM - 0.012f;

            WallObs rightObservation = leftObservation;
            rightObservation.rho = rightPrediction.rangeM - 0.012f;

            const FrontPairUpdateResult result = core.updateFrontPair(leftObservation, rightObservation, map);
            Assert::IsTrue(result.filter.attempted);
            Assert::IsTrue(result.filter.accepted);

            const VehicleState::StateVector& after = core.state();
            const VehicleState::StateMatrix afterCovariance = core.covariance();
            Assert::IsTrue(after(VehicleState::kPy) > (before(VehicleState::kPy) + 0.002f));
            Assert::IsTrue(std::fabs(after(VehicleState::kPx) - before(VehicleState::kPx)) < 0.004f);
            Assert::IsTrue(afterCovariance(VehicleState::kPy, VehicleState::kPy) <
                beforeCovariance(VehicleState::kPy, VehicleState::kPy));
        }

        TEST_METHOD(SrUkfCoreLeftWallUpdateMovesLeftForCloserObservation)
        {
            Maze maze;
            maze.SetWall(maze(0, 0), Direction::Left, WallState::Wall);
            const LocalMapView map = BuildLocalMapView(maze);
            const PlantParams params = PlantParams::Default();
            const VehicleState::StateVector initialState =
                BuildUkfState(
                    0.09f,
                    0.09f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f);

            WallGeometryModel geometry;
            const GeometryPrediction baseline = geometry.predictRay(initialState, params.sideLeftSensor, map);
            Assert::IsTrue(baseline.hit);

            SrUkfCore core(params);
            Assert::IsTrue(core.reset(initialState, BuildUkfCovariance(0.02f, 0.04f, 0.02f, 0.02f, 0.05f, 0.05f, 0.02f)));
            const VehicleState::StateVector before = core.state();
            const VehicleState::StateMatrix beforeCovariance = core.covariance();

            WallObs observation{};
            observation.valid = true;
            observation.confidence = 1.0f;
            observation.cls = ObsClass::WallLike;
            observation.rho = baseline.rangeM - 0.012f;

            const WallUpdateResult result = core.updateSideSensor(Side::Left, observation, map);
            Assert::IsTrue(result.filter.attempted);
            Assert::IsTrue(result.filter.accepted);

            const VehicleState::StateVector& after = core.state();
            const VehicleState::StateMatrix afterCovariance = core.covariance();
            Assert::IsTrue(after(VehicleState::kPx) < (before(VehicleState::kPx) - 0.002f));
            Assert::IsTrue(std::fabs(after(VehicleState::kPy) - before(VehicleState::kPy)) < 0.004f);
            Assert::IsTrue(afterCovariance(VehicleState::kPx, VehicleState::kPx) <
                beforeCovariance(VehicleState::kPx, VehicleState::kPx));
        }

        TEST_METHOD(SrUkfCoreReplayOfLatestOpenFloorLaunchLogDoesNotProduceXPoseBoundaryJump)
        {
            const PlantParams params = PlantParams::Default();
            const float distancePerCountM = DistancePerEncoderCountMeters(params);
            constexpr int sampleCount =
                static_cast<int>(sizeof(kLatestLoggedOpenFloorPoseJumpWindow) / sizeof(kLatestLoggedOpenFloorPoseJumpWindow[0]));
            const LoggedOpenFloorPoseJumpSample& first = kLatestLoggedOpenFloorPoseJumpWindow[0];

            SrUkfCore core(params);
            const VehicleState::StateVector initialState = BuildUkfState(
                first.poseXM,
                first.poseYM,
                first.poseYawRad,
                first.measuredLinearSpeedMps,
                0.0f,
                first.gyroRawRadps - first.gyroBiasRadps,
                first.leftEncoderOmegaRadps,
                first.rightEncoderOmegaRadps,
                first.gyroBiasRadps);
            Assert::IsTrue(core.reset(initialState, SrUkfCore::BuildDefaultInitialCovariance()));

            float maxStepDxM = 0.0f;
            float maxAbsDxFromStartM = 0.0f;
            float encoderForwardTravelM = 0.0f;
            float previousXM = first.poseXM;

            for (int index = 1; index < sampleCount; ++index)
            {
                const LoggedOpenFloorPoseJumpSample& sample = kLatestLoggedOpenFloorPoseJumpWindow[index];

                ControlInput control{};
                control.leftMotorCommand = sample.leftDriveCommand;
                control.rightMotorCommand = sample.rightDriveCommand;
                control.fanDutyCycle = 0.80f;
                control.batteryVoltageV = params.supplyVoltageV;
                Assert::IsTrue(core.predict(sample.dtSeconds, control));

                EncoderObs encoderObservation{};
                encoderObservation.totalLeftCounts = sample.leftEncoderCount;
                encoderObservation.totalRightCounts = sample.rightEncoderCount;
                encoderObservation.omegaLeftRadps = sample.leftEncoderOmegaRadps;
                encoderObservation.omegaRightRadps = sample.rightEncoderOmegaRadps;
                const MeasurementUpdateResult encoderResult = core.updateEncoderPair(encoderObservation, sample.dtSeconds);
                Assert::IsTrue(encoderResult.attempted);

                const MeasurementUpdateResult yawResult = core.updateYawRate(sample.gyroRawRadps);
                Assert::IsTrue(yawResult.attempted);

                ImuAccelObs accelObservation{};
                accelObservation.valid =
                    std::isfinite(sample.accelBodyXMps2) &&
                    std::isfinite(sample.accelBodyYMps2);
                accelObservation.accelBodyXMps2 = sample.accelBodyXMps2;
                accelObservation.accelBodyYMps2 = sample.accelBodyYMps2;
                (void)core.updatePlanarAccel(accelObservation);

                const float currentXM = core.state()(VehicleState::kPx);
                maxStepDxM = (std::max)(maxStepDxM, std::fabs(currentXM - previousXM));
                maxAbsDxFromStartM = (std::max)(maxAbsDxFromStartM, std::fabs(currentXM - first.poseXM));
                previousXM = currentXM;

                encoderForwardTravelM += std::fabs(
                    0.5f * static_cast<float>(sample.leftEncoderCount + sample.rightEncoderCount) * distancePerCountM);
            }

            const float finalXM = core.state()(VehicleState::kPx);
            const float finalYM = core.state()(VehicleState::kPy);

            Assert::IsTrue(maxStepDxM < 0.01f);
            Assert::IsTrue(maxAbsDxFromStartM < 0.01f);
            Assert::IsTrue(finalXM < OpenFloorWorkspaceMaxMeters());
            Assert::IsTrue(std::fabs(finalXM - first.poseXM) < 0.01f);
            Assert::IsTrue(std::fabs(finalXM - first.poseXM) < (encoderForwardTravelM + 0.005f));
            Assert::IsTrue(finalYM > (first.poseYM - 0.005f));
        }

        SrUkfCore RunUKFCycles(int numCycles, ControlInput& control)
        {
            const VehicleState::StateVector initialState =
                BuildUkfState(
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f);
            const VehicleState::StateMatrix initialCovariance =
                BuildUkfCovariance(0.001f, 0.01f, 0.005f, 0.005f, 0.05f, 0.05f, 0.02f);

            SrUkfCore core;
            core.reset(initialState, initialCovariance);
            EncoderObs encoder{};
            constexpr float dt = 0.001f;

            for (int step = 0; step < numCycles; ++step)
            {
                Assert::IsTrue(core.predict(dt, control));

                const MeasurementUpdateResult yawResult = core.updateYawRate(0.0f);
                Assert::IsTrue(yawResult.attempted);
                Assert::IsTrue(yawResult.accepted);
            }
            return core;
        }

        SrUkfCore RunUKFCycles(int numCycles)
        {
			return RunUKFCycles(numCycles, ControlInput{});
        }
        TEST_METHOD(SrUkfCoreControlDirectionsCorrect)
        {
			PlantModel model = PlantModel();
            const PlantParams& params = PlantParams::Default();
            float accelTarget = 1.0f;
            const VehicleState::StateVector initialState =
                BuildUkfState(
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f);
            const VehicleState::StateMatrix initialCovariance =
                BuildUkfCovariance(0.001f, 0.01f, 0.005f, 0.005f, 0.05f, 0.05f, 0.02f);

            SrUkfCore core;
            core.reset(initialState, initialCovariance);
            EncoderObs encoder{};
            constexpr float dt = 0.001f;

            for (int step = 0; step < 3000; ++step)
            {
                auto control = model.solveDriveCommands(core.state()(VehicleState::kU), accelTarget, core.state()(VehicleState::kR), 0.0f, params);

                Assert::IsTrue(core.predict(dt, control.control));

                const MeasurementUpdateResult yawResult = core.updateYawRate(0.0f);
                Assert::IsTrue(yawResult.attempted);
                Assert::IsTrue(yawResult.accepted);
            }

			auto state = core.state();
            Assert::IsTrue(state(VehicleState::kU) > 1.0f,
                (std::wstring(L"Forward velocity was too low: ") +
                    std::to_wstring(state(VehicleState::kU))).c_str());

            Assert::IsTrue(fabs(state(VehicleState::kV)) < 0.01f,
                (std::wstring(L"Lateral velocity was too high: ") +
                    std::to_wstring(state(VehicleState::kV))).c_str());

            Assert::IsTrue(fabs(state(VehicleState::kR)) < 0.5f,
                (std::wstring(L"Angular velocity was too high: ") +
                    std::to_wstring(state(VehicleState::kR))).c_str());
        }
        TEST_METHOD(SrUkfCoreControlDirectionsCorrectAfterStationary)
        {
            PlantModel model = PlantModel();
            const PlantParams& params = PlantParams::Default();
            float accelTarget = 1.0f;
            EncoderObs encoder{};
            constexpr float dt = 0.001f;
            auto core = RunUKFCycles(2000, ControlInput{});

            for (int step = 0; step < 3000; ++step)
            {
                auto control = model.solveDriveCommands(core.state()(VehicleState::kU), accelTarget, core.state()(VehicleState::kR), 0.0f, params);

                Assert::IsTrue(core.predict(dt, control.control));

                const MeasurementUpdateResult yawResult = core.updateYawRate(0.0f);
                Assert::IsTrue(yawResult.attempted);
                Assert::IsTrue(yawResult.accepted);
            }

            auto state = core.state();
            Assert::IsTrue(state(VehicleState::kU) > 1.0f,
                (std::wstring(L"Forward velocity was too low: ") +
                    std::to_wstring(state(VehicleState::kU))).c_str());

            Assert::IsTrue(fabs(state(VehicleState::kV)) < 0.01f,
                (std::wstring(L"Lateral velocity was too high: ") +
                    std::to_wstring(state(VehicleState::kV))).c_str());

            Assert::IsTrue(fabs(state(VehicleState::kR)) < 0.5f,
                (std::wstring(L"Angular velocity was too high: ") +
                    std::to_wstring(state(VehicleState::kR))).c_str());
        }
        TEST_METHOD(SrUkfCoreDoesNotDriftOrLoseCertaintyUnderRepeatedZeroMotionMeasurementsPoseX)
        {
			SrUkfCore core = RunUKFCycles(2000);

            const VehicleState::StateVector& state = core.state();
            Assert::IsTrue(std::fabs(state(VehicleState::kPx)) < 1.0e-4f);


			// If the robot is stationary, we grow increasingly sure that the velocity, yaw rate, and wheel speeds are all near zero.
            // We should still have some uncertainty about the exact position and heading, but it shouldn't grow without bound.
            // The gyro bias should be allowed to absorb the stationary measurements, as this is when it's most appropriate to update that value.
			const auto covariance = core.covariance();
            Assert::IsTrue(covariance(VehicleState::kPx, VehicleState::kPx) < 10.0f, L"Final x position variance was too high");
        }
        TEST_METHOD(SrUkfCoreDoesNotDriftOrLoseCertaintyUnderRepeatedZeroMotionMeasurementsPoseY)
        {
            SrUkfCore core = RunUKFCycles(2000);

            const VehicleState::StateVector& state = core.state();
            Assert::IsTrue(std::fabs(state(VehicleState::kPy)) < 1.0e-4f);


            // If the robot is stationary, we grow increasingly sure that the velocity, yaw rate, and wheel speeds are all near zero.
            // We should still have some uncertainty about the exact position and heading, but it shouldn't grow without bound.
            // The gyro bias should be allowed to absorb the stationary measurements, as this is when it's most appropriate to update that value.
            const auto covariance = core.covariance();
            Assert::IsTrue(covariance(VehicleState::kPy, VehicleState::kPy) < 10.0f, L"Final y position variance was too high");
        }
        TEST_METHOD(SrUkfCoreDoesNotDriftOrLoseCertaintyUnderRepeatedZeroMotionMeasurementsForwardVelocity)
        {
            SrUkfCore core = RunUKFCycles(2000);

            const VehicleState::StateVector& state = core.state();
            Assert::IsTrue(std::fabs(state(VehicleState::kU)) < 1.0e-4f);


            // If the robot is stationary, we grow increasingly sure that the velocity, yaw rate, and wheel speeds are all near zero.
            // We should still have some uncertainty about the exact position and heading, but it shouldn't grow without bound.
            // The gyro bias should be allowed to absorb the stationary measurements, as this is when it's most appropriate to update that value.
            const auto covariance = core.covariance();
            Assert::IsTrue(covariance(VehicleState::kU, VehicleState::kU) < 0.0001f, L"Final forward velocity variance was too high");
        }

        TEST_METHOD(SrUkfCoreDoesNotDriftOrLoseCertaintyUnderRepeatedZeroMotionMeasurementsLateralVelocity)
        {
            SrUkfCore core = RunUKFCycles(2000);

            const VehicleState::StateVector& state = core.state();
            Assert::IsTrue(std::fabs(state(VehicleState::kV)) < 1.0e-5f);


            // If the robot is stationary, we grow increasingly sure that the velocity, yaw rate, and wheel speeds are all near zero.
            // We should still have some uncertainty about the exact position and heading, but it shouldn't grow without bound.
            // The gyro bias should be allowed to absorb the stationary measurements, as this is when it's most appropriate to update that value.
            const auto covariance = core.covariance();
            Assert::IsTrue(covariance(VehicleState::kV, VehicleState::kV) < 0.0001f, (L"Final lateral velocity variance was too high:\n" +
                std::to_wstring((covariance(VehicleState::kV,VehicleState::kV)))).c_str());
        }
        TEST_METHOD(SrUkfCoreDoesNotDriftOrLoseCertaintyUnderRepeatedZeroMotionMeasurementsYawRate)
        {
            SrUkfCore core = RunUKFCycles(2000);

            const VehicleState::StateVector& state = core.state();
            Assert::IsTrue(std::fabs(state(VehicleState::kR)) < 1.0e-4f);


            // If the robot is stationary, we grow increasingly sure that the velocity, yaw rate, and wheel speeds are all near zero.
            // We should still have some uncertainty about the exact position and heading, but it shouldn't grow without bound.
            // The gyro bias should be allowed to absorb the stationary measurements, as this is when it's most appropriate to update that value.
            const auto covariance = core.covariance();
            Assert::IsTrue(covariance(VehicleState::kR, VehicleState::kR) < 0.0001f, L"Final yaw rate variance was too high");
        }
        TEST_METHOD(SrUkfCoreDoesNotDriftOrLoseCertaintyUnderRepeatedZeroMotionMeasurementsOmegaL)
        {
            SrUkfCore core = RunUKFCycles(2000);

            const VehicleState::StateVector& state = core.state();
            Assert::IsTrue(std::fabs(state(VehicleState::kOmegaL)) < 1.0e-4f);


            // If the robot is stationary, we grow increasingly sure that the velocity, yaw rate, and wheel speeds are all near zero.
            // We should still have some uncertainty about the exact position and heading, but it shouldn't grow without bound.
            // The gyro bias should be allowed to absorb the stationary measurements, as this is when it's most appropriate to update that value.
            const auto covariance = core.covariance();
            Assert::IsTrue(covariance(VehicleState::kOmegaL, VehicleState::kOmegaL) < 0.0001f, L"Final left wheel speed variance was too high");
        }
        TEST_METHOD(SrUkfCoreDoesNotDriftOrLoseCertaintyUnderRepeatedZeroMotionMeasurementsOmegaR)
        {
            SrUkfCore core = RunUKFCycles(2000);

            const VehicleState::StateVector& state = core.state();
            Assert::IsTrue(std::fabs(state(VehicleState::kOmegaR)) < 1.0e-4f);


            // If the robot is stationary, we grow increasingly sure that the velocity, yaw rate, and wheel speeds are all near zero.
            // We should still have some uncertainty about the exact position and heading, but it shouldn't grow without bound.
            // The gyro bias should be allowed to absorb the stationary measurements, as this is when it's most appropriate to update that value.
            const auto covariance = core.covariance();
            Assert::IsTrue(covariance(VehicleState::kOmegaR, VehicleState::kOmegaR) < 0.0001f, L"Final left wheel speed variance was too high");
        }
        TEST_METHOD(SrUkfCoreRepeatedZeroEncoderUpdatesDriveYawRateVarianceExtremelyLow)
        {
            const VehicleState::StateVector initialState =
                BuildUkfState(
                    0.0f,
                    0.09f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f);
            const VehicleState::StateMatrix initialCovariance =
                BuildUkfCovariance(0.001f, 0.01f, 0.005f, 0.005f, 1.0f, 0.05f, 0.02f);

            SrUkfCore core;
            Assert::IsTrue(core.reset(initialState, initialCovariance));

            ControlInput control{};
            EncoderObs encoder{};
            constexpr float dt = 0.001f;
            constexpr int kSteps = 1000;

            for (int step = 0; step < kSteps; ++step)
            {
                Assert::IsTrue(core.predict(dt, control));

                const MeasurementUpdateResult encoderResult = core.updateEncoderPair(encoder, dt);
                Assert::IsTrue(encoderResult.attempted);
                Assert::IsTrue(encoderResult.accepted);
            }

            const VehicleState::StateMatrix covariance = core.covariance();
            const float finalYawRateVarianceRadps2 = covariance(VehicleState::kR, VehicleState::kR);
            Assert::IsTrue(std::isfinite(covariance(VehicleState::kR, VehicleState::kR)));
            Assert::IsTrue(
                finalYawRateVarianceRadps2 < 1.0e-4f,
                (std::wstring(L"Final yaw-rate variance was ") + std::to_wstring(finalYawRateVarianceRadps2)).c_str());
        }

        TEST_METHOD(SrUkfCoreRepeatedZeroEncoderUpdatesDriveLateralVelocityVarianceExtremelyLow)
        {
            const VehicleState::StateVector initialState =
                BuildUkfState(
                    0.0f,
                    0.09f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f);
            const VehicleState::StateMatrix initialCovariance =
                BuildUkfCovariance(0.001f, 0.01f, 0.005f, 1.0f, 1.0f, 0.05f, 0.02f);

            SrUkfCore core;
            Assert::IsTrue(core.reset(initialState, initialCovariance));

            const float initialLateralVelocityVarianceMps2 = core.covariance()(VehicleState::kV, VehicleState::kV);
            Assert::AreEqual(1.0f, initialLateralVelocityVarianceMps2, 1.0e-6f);

            ControlInput control{};
            EncoderObs encoder{};
            constexpr float dt = 0.001f;
            constexpr int kSteps = 1000;

            for (int step = 0; step < kSteps; ++step)
            {
                Assert::IsTrue(core.predict(dt, control));

                const MeasurementUpdateResult encoderResult = core.updateEncoderPair(encoder, dt);
                Assert::IsTrue(encoderResult.attempted);
                Assert::IsTrue(encoderResult.accepted);
            }

            const VehicleState::StateMatrix covariance = core.covariance();
            const float finalLateralVelocityVarianceMps2 = covariance(VehicleState::kV, VehicleState::kV);
            Assert::IsTrue(std::isfinite(finalLateralVelocityVarianceMps2));
            Assert::IsTrue(
                finalLateralVelocityVarianceMps2 < 1.0e-4f,
                (std::wstring(L"Final lateral-velocity variance was ") +
                    std::to_wstring(finalLateralVelocityVarianceMps2)).c_str());
        }
        TEST_METHOD(SrUkfCoreRepeatedStationaryCyclesKeepMotionAndBiasCovarianceNearZeroWhilePoseRemainsBounded)
        {
            const VehicleState::StateVector initialState =
                BuildUkfState(
                    0.18f,
                    0.27f,
                    0.11f,
                    0.35f,
                    -0.22f,
                    0.18f,
                    8.0f,
                    -7.5f,
                    0.04f);
            const VehicleState::StateMatrix initialCovariance =
                BuildUkfCovariance(0.02f, 0.04f, 0.20f, 0.15f, 0.25f, 0.50f, 0.05f);

            SrUkfCore core;
            Assert::IsTrue(core.reset(initialState, initialCovariance));

            ControlInput control{};
            EncoderObs encoder{};
            constexpr float dt = 0.001f;
            constexpr int kSteps = 1000;

            for (int step = 0; step < kSteps; ++step)
            {
                Assert::IsTrue(core.predict(dt, control));

                const MeasurementUpdateResult encoderResult = core.updateEncoderPair(encoder, dt);
                Assert::IsTrue(encoderResult.attempted);
                Assert::IsTrue(encoderResult.accepted);

                const MeasurementUpdateResult yawResult = core.updateYawRate(0.0f);
                Assert::IsTrue(yawResult.attempted);
                Assert::IsTrue(yawResult.accepted);
            }

            const VehicleState::StateVector& state = core.state();
            const VehicleState::StateMatrix covariance = core.covariance();

            Assert::AreEqual(initialState(VehicleState::kPx), state(VehicleState::kPx), 1.0e-5f);
            Assert::AreEqual(initialState(VehicleState::kPy), state(VehicleState::kPy), 1.0e-5f);
            Assert::AreEqual(initialState(VehicleState::kPsi), state(VehicleState::kPsi), 1.0e-5f);
            Assert::AreEqual(0.0f, state(VehicleState::kU), 1.0e-6f);
            Assert::AreEqual(0.0f, state(VehicleState::kV), 1.0e-6f);
            Assert::AreEqual(0.0f, state(VehicleState::kR), 1.0e-6f);
            Assert::AreEqual(0.0f, state(VehicleState::kOmegaL), 1.0e-6f);
            Assert::AreEqual(0.0f, state(VehicleState::kOmegaR), 1.0e-6f);
            Assert::AreEqual(0.0f, state(VehicleState::kBgz), 1.0e-6f);

            Assert::IsTrue(covariance(VehicleState::kU, VehicleState::kU) <= 1.0e-12f);
            Assert::IsTrue(covariance(VehicleState::kV, VehicleState::kV) <= 1.0e-12f);
            Assert::IsTrue(covariance(VehicleState::kR, VehicleState::kR) <= 1.0e-12f);
            Assert::IsTrue(covariance(VehicleState::kOmegaL, VehicleState::kOmegaL) <= 1.0e-12f);
            Assert::IsTrue(covariance(VehicleState::kOmegaR, VehicleState::kOmegaR) <= 1.0e-12f);
            Assert::IsTrue(covariance(VehicleState::kBgz, VehicleState::kBgz) <= 1.0e-12f);

            Assert::IsTrue(std::isfinite(covariance(VehicleState::kPx, VehicleState::kPx)));
            Assert::IsTrue(std::isfinite(covariance(VehicleState::kPy, VehicleState::kPy)));
            Assert::IsTrue(std::isfinite(covariance(VehicleState::kPsi, VehicleState::kPsi)));
            Assert::IsTrue(
                covariance(VehicleState::kPx, VehicleState::kPx) <=
                (initialCovariance(VehicleState::kPx, VehicleState::kPx) + 1.0e-9f));
            Assert::IsTrue(
                covariance(VehicleState::kPy, VehicleState::kPy) <=
                (initialCovariance(VehicleState::kPy, VehicleState::kPy) + 1.0e-9f));
            Assert::IsTrue(
                covariance(VehicleState::kPsi, VehicleState::kPsi) <=
                (initialCovariance(VehicleState::kPsi, VehicleState::kPsi) + 1.0e-9f));
        }

    };
}
