#include "pch.h"
#include "CppUnitTest.h"

#include "..\MazeMap\PlantModel.h"
#include "..\MazeMap\SensorSnapshot.h"
#include "..\MazeMap\Vehicle.h"
#include "..\MazeMap\VehicleState.h"

#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <limits>
#include <string>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    namespace
    {
        constexpr float kStationaryEncoderVelocitySigmaMps = 0.002936f;
        constexpr float kImuYawRateSigmaRadps = 0.0010954451f;

        float DtSecondsForCountWheelSpeed(
            const std::int32_t counts,
            const float wheelSpeedRadps) noexcept
        {
            const float wheelLinearMps = Vehicle::WheelLinearVelocityFromWheelSpeed(wheelSpeedRadps);
            if ((counts == 0) ||
                !std::isfinite(wheelLinearMps) ||
                !(std::fabs(wheelLinearMps) > 0.0f))
            {
                return 0.001f;
            }

            return std::fabs(
                Vehicle::DriveEncoderDistanceFromCounts(counts) /
                wheelLinearMps);
        }

        void PublishEncoderObservationFromCountDeltas(
            VehicleState& state,
            const std::int32_t leftDeltaCounts,
            const std::int32_t rightDeltaCounts,
            const float dtSeconds) noexcept
        {
            const float leftDistanceDeltaM =
                Vehicle::DriveEncoderDistanceFromCounts(leftDeltaCounts);
            const float rightDistanceDeltaM =
                Vehicle::DriveEncoderDistanceFromCounts(rightDeltaCounts);
            const float invDtSeconds =
                (std::isfinite(dtSeconds) && (dtSeconds > 0.0f)) ?
                (1.0f / dtSeconds) :
                0.0f;

            SensorSnapshot::EncoderObs encoderObservation = SensorSnapshot{}.EncoderObservation();
            encoderObservation.SetTotalLeftCounts(leftDeltaCounts);
            encoderObservation.SetTotalRightCounts(rightDeltaCounts);
            encoderObservation.SetLeftDistanceDeltaM(leftDistanceDeltaM);
            encoderObservation.SetRightDistanceDeltaM(rightDistanceDeltaM);
            encoderObservation.SetLeftVelocityMps(leftDistanceDeltaM * invDtSeconds);
            encoderObservation.SetRightVelocityMps(rightDistanceDeltaM * invDtSeconds);
            encoderObservation.SetLeftWheelSpeedRadps(
                Vehicle::WheelSpeedFromLinearVelocity(encoderObservation.LeftVelocityMps()));
            encoderObservation.SetRightWheelSpeedRadps(
                Vehicle::WheelSpeedFromLinearVelocity(encoderObservation.RightVelocityMps()));

            const SensorSnapshot& previousSnapshot = state.GetSensorSnapshot();
            const std::int64_t leftTotalCounts =
                previousSnapshot.LeftEncoderTotalCounts() +
                static_cast<std::int64_t>(leftDeltaCounts);
            const std::int64_t rightTotalCounts =
                previousSnapshot.RightEncoderTotalCounts() +
                static_cast<std::int64_t>(rightDeltaCounts);
            SensorSnapshot snapshot = previousSnapshot;
            snapshot.PublishEncoderObservation(
                encoderObservation,
                true,
                leftTotalCounts,
                rightTotalCounts,
                Vehicle::DriveEncoderDistanceFromCounts(leftTotalCounts),
                Vehicle::DriveEncoderDistanceFromCounts(rightTotalCounts));
            state.SetSensorSnapshot(snapshot);
        }

#define VEHICLE_STATE_LOG_TEST_ROW_FIELDS(X) \
    X(VehicleStateLogEntry, ukf_state)

        MMLOG_DEFINE_PRIVATE_ROW_WITH_BODY(
            VehicleStateLogTestRow,
            VEHICLE_STATE_LOG_TEST_ROW_FIELDS,
            void Set(const VehicleState& state) noexcept
            {
                ukf_state.Set(state);
            });

#define VEHICLE_STATE_LOG_TEST_FLATTENED_ROW_FIELDS(X) \
    X(float, ukf_state_px_m) \
    X(float, ukf_state_py_m) \
    X(float, ukf_state_heading_rad) \
    X(float, ukf_state_vf_mps) \
    X(float, ukf_state_vr_mps) \
    X(float, ukf_state_yaw_rate_radps) \
    X(float, ukf_state_delta_af_mps2) \
    X(float, ukf_state_delta_ar_mps2) \
    X(float, ukf_state_delta_yaw_accel_radps2)

        MMLOG_DEFINE_ROW(
            VehicleStateLogTestFlattenedRow,
            VEHICLE_STATE_LOG_TEST_FLATTENED_ROW_FIELDS);

#undef VEHICLE_STATE_LOG_TEST_FLATTENED_ROW_FIELDS
#undef VEHICLE_STATE_LOG_TEST_ROW_FIELDS
    }

    TEST_CLASS(VehicleStateTest)
    {
    public:
        TEST_METHOD(VehicleWheelSpeedsFromBodyVelocityMatchesBodyKinematics)
        {
            constexpr float forwardMps = 0.72f;
            constexpr float yawRateRadps = 1.35f;

            float leftWheelSpeedRadps = 0.0f;
            float rightWheelSpeedRadps = 0.0f;
            Vehicle::WheelSpeedsFromBodyVelocity(
                forwardMps,
                yawRateRadps,
                leftWheelSpeedRadps,
                rightWheelSpeedRadps);

            const float leftWheelMps = Vehicle::WheelLinearVelocityFromWheelSpeed(leftWheelSpeedRadps);
            const float rightWheelMps = Vehicle::WheelLinearVelocityFromWheelSpeed(rightWheelSpeedRadps);

            Assert::AreEqual(
                Vehicle::LeftWheelLinearVelocityFromBody(forwardMps, yawRateRadps),
                leftWheelMps,
                1.0e-6f);
            Assert::AreEqual(
                Vehicle::RightWheelLinearVelocityFromBody(forwardMps, yawRateRadps),
                rightWheelMps,
                1.0e-6f);
            Assert::AreEqual(
                forwardMps,
                Vehicle::BodyForwardVelocityFromWheelLinear(leftWheelMps, rightWheelMps),
                1.0e-6f);
            Assert::AreEqual(
                yawRateRadps,
                Vehicle::BodyYawRateFromWheelLinear(leftWheelMps, rightWheelMps),
                1.0e-6f);
        }

        TEST_METHOD(VehicleStateIsStationaryUsesCurrentUkfThresholds)
        {
            const float wheelSpeedThresholdRadps =
                Vehicle::WheelSpeedFromLinearVelocity(kStationaryEncoderVelocitySigmaMps);

            VehicleState stationaryState;
            stationaryState.SetPosition(Eigen::Vector2f(0.40f, -0.18f));
            stationaryState.SetHeading(0.35f);
            stationaryState.SetForwardVelocity(0.0f);
            stationaryState.SetRightwardVelocity(0.0f);
            stationaryState.SetYawRate(0.5f * (3.0f * kImuYawRateSigmaRadps));
            PublishEncoderObservationFromCountDeltas(
                stationaryState,
                1,
                -1,
                DtSecondsForCountWheelSpeed(1, 0.5f * wheelSpeedThresholdRadps));
            Assert::IsTrue(stationaryState.IsStationary());

            VehicleState movingState = stationaryState;
            movingState.SetForwardVelocity(2.0f * kStationaryEncoderVelocitySigmaMps);
            Assert::IsFalse(movingState.IsStationary());

            movingState = stationaryState;
            movingState.SetYawRate(3.1f * kImuYawRateSigmaRadps);
            Assert::IsFalse(movingState.IsStationary());

            movingState = stationaryState;
            PublishEncoderObservationFromCountDeltas(
                movingState,
                11,
                -5,
                DtSecondsForCountWheelSpeed(11, 1.1f * wheelSpeedThresholdRadps));
            Assert::IsFalse(movingState.IsStationary());
        }

        TEST_METHOD(VehicleStateCurrentCommandRoundTripsAppliedCommand)
        {
            VehicleState state;
            const App::Internal::CommandVector command(0.25f, -0.125f);

            state.SetCurrentCommand(command);

            Assert::AreEqual(command.LeftCommand(), state.GetCurrentCommand().LeftCommand(), 0.0f);
            Assert::AreEqual(command.RightCommand(), state.GetCurrentCommand().RightCommand(), 0.0f);
        }

        TEST_METHOD(VehicleBatteryVoltageMatchesPlantModelSupplyVoltage)
        {
            const Vehicle vehicle;
            VehicleState runtimeState;
            const PlantModel plantModel(vehicle, runtimeState);
            float plantModelSupplyVoltageV = std::numeric_limits<float>::quiet_NaN();

            const bool wrotePlantDump = plantModel.WritePlantDebugTextDump(
                &plantModelSupplyVoltageV,
                [](
                    void* context,
                    const char* type,
                    const char* format,
                    std::va_list args) noexcept
                {
                    if (std::strcmp(type, "plant_dump_params_drive_electrical") != 0)
                    {
                        return true;
                    }

                    char message[512] = {};
                    std::va_list argsCopy;
                    va_copy(argsCopy, args);
                    const int length = std::vsnprintf(message, sizeof(message), format, argsCopy);
                    va_end(argsCopy);
                    if ((length <= 0) || (length >= static_cast<int>(sizeof(message))))
                    {
                        return false;
                    }

                    constexpr char kSupplyVoltageToken[] = "supply_voltage_v=";
                    const char* const valueStart = std::strstr(message, kSupplyVoltageToken);
                    if (valueStart == nullptr)
                    {
                        return false;
                    }
                    *static_cast<float*>(context) =
                        std::strtof(valueStart + (sizeof(kSupplyVoltageToken) - 1U), nullptr);
                    return true;
                });

            Assert::IsTrue(wrotePlantDump);
            Assert::IsTrue(std::isfinite(plantModelSupplyVoltageV));
            Assert::AreEqual(vehicle.GetBatteryVoltage(), plantModelSupplyVoltageV, 0.0f);
        }

        TEST_METHOD(VehicleStateLogEntryProjectsThroughDomainGetters)
        {
            VehicleState state;
            state.SetPosition(Eigen::Vector2f(1.25f, -2.5f));
            state.SetHeading(3.75f);
            state.SetForwardVelocity(-4.125f);
            state.SetRightwardVelocity(5.5f);
            state.SetYawRate(-6.625f);
            state.SetForwardAccelerationResidual(7.875f);
            state.SetRightwardAccelerationResidual(-8.25f);
            state.SetYawAccelResidual(9.5f);

            VehicleStateLogTestRow projected{};
            projected.Set(state);

            VehicleStateLogTestFlattenedRow flattened{};
            flattened.ukf_state_px_m = state.GetPositionX();
            flattened.ukf_state_py_m = state.GetPositionY();
            flattened.ukf_state_heading_rad = state.GetHeading();
            flattened.ukf_state_vf_mps = state.GetForwardVelocity();
            flattened.ukf_state_vr_mps = state.GetRightwardVelocity();
            flattened.ukf_state_yaw_rate_radps = state.GetYawRate();
            flattened.ukf_state_delta_af_mps2 = state.GetForwardAccelerationResidual();
            flattened.ukf_state_delta_ar_mps2 = state.GetRightwardAccelerationResidual();
            flattened.ukf_state_delta_yaw_accel_radps2 = state.GetYawAccelResidual();

            const std::string expectedHeader =
                "f32_ukf_state_px_m,f32_ukf_state_py_m,f32_ukf_state_heading_rad,"
                "f32_ukf_state_vf_mps,f32_ukf_state_vr_mps,f32_ukf_state_yaw_rate_radps,"
                "f32_ukf_state_delta_af_mps2,f32_ukf_state_delta_ar_mps2,"
                "f32_ukf_state_delta_yaw_accel_radps2";

            Assert::AreEqual(expectedHeader, std::string(VehicleStateLogTestRow::header_cstr()));
            Assert::IsTrue(
                std::string(VehicleStateLogTestRow::header_cstr()).find("ukf_state_gyro_bias_z_radps") ==
                std::string::npos);
            Assert::AreEqual(sizeof(flattened), sizeof(projected));
            Assert::AreEqual(
                0,
                std::memcmp(
                    &flattened,
                    &projected,
                    sizeof(VehicleStateLogTestRow)));
        }
    };
}

