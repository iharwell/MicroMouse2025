#include "pch.h"
#include "CppUnitTest.h"

#include "..\MazeMap\PlantModel.h"
#include "..\MazeMap\Vehicle.h"
#include "..\MazeMap\VehicleState.h"

#include <cstdarg>
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
    X(float, ukf_state_psi_rad) \
    X(float, ukf_state_u_mps) \
    X(float, ukf_state_v_mps) \
    X(float, ukf_state_r_radps) \
    X(float, ukf_state_omega_l_radps) \
    X(float, ukf_state_omega_r_radps) \
    X(float, ukf_state_bgz_radps)

        MMLOG_DEFINE_ROW(
            VehicleStateLogTestFlattenedRow,
            VEHICLE_STATE_LOG_TEST_FLATTENED_ROW_FIELDS);

#undef VEHICLE_STATE_LOG_TEST_FLATTENED_ROW_FIELDS
#undef VEHICLE_STATE_LOG_TEST_ROW_FIELDS
    }

    TEST_CLASS(VehicleStateTest)
    {
    public:
        TEST_METHOD(VehicleWheelOmegasFromBodyVelocityMatchesBodyKinematics)
        {
            constexpr float forwardMps = 0.72f;
            constexpr float yawRateRadps = 1.35f;

            float leftOmegaRadps = 0.0f;
            float rightOmegaRadps = 0.0f;
            Vehicle::WheelOmegasFromBodyVelocity(
                forwardMps,
                yawRateRadps,
                leftOmegaRadps,
                rightOmegaRadps);

            const float leftWheelMps = Vehicle::WheelLinearVelocityFromOmega(leftOmegaRadps);
            const float rightWheelMps = Vehicle::WheelLinearVelocityFromOmega(rightOmegaRadps);

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
                Vehicle::WheelOmegaFromLinearVelocity(kStationaryEncoderVelocitySigmaMps);

            VehicleState stationaryState;
            stationaryState.SetPosition(Eigen::Vector2f(0.40f, -0.18f));
            stationaryState.SetOrientation(0.35f);
            stationaryState.SetVelocity(0.0f);
            stationaryState.SetLateralVelocity(0.0f);
            stationaryState.SetRotationalVelocity(0.5f * (3.0f * kImuYawRateSigmaRadps));
            stationaryState.SetWheelSpeedLeft(0.5f * wheelSpeedThresholdRadps);
            stationaryState.SetWheelSpeedRight(-0.5f * wheelSpeedThresholdRadps);
            stationaryState.SetGyroBiasZ(0.12f);
            Assert::IsTrue(stationaryState.IsStationary());

            VehicleState movingState = stationaryState;
            movingState.SetVelocity(2.0f * kStationaryEncoderVelocitySigmaMps);
            Assert::IsFalse(movingState.IsStationary());

            movingState = stationaryState;
            movingState.SetRotationalVelocity(3.1f * kImuYawRateSigmaRadps);
            Assert::IsFalse(movingState.IsStationary());

            movingState = stationaryState;
            movingState.SetWheelSpeedLeft(1.1f * wheelSpeedThresholdRadps);
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

            const bool wrotePlantDump = plantModel.WriteUkfPlantDebugTextDump(
                &plantModelSupplyVoltageV,
                [](
                    void* context,
                    const char* type,
                    const char* format,
                    std::va_list args) noexcept
                {
                    if (std::strcmp(type, "ukf_dump_params_drive_electrical") != 0)
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
            state.SetOrientation(3.75f);
            state.SetVelocity(-4.125f);
            state.SetLateralVelocity(5.5f);
            state.SetRotationalVelocity(-6.625f);
            state.SetWheelSpeedLeft(7.875f);
            state.SetWheelSpeedRight(-8.25f);
            state.SetGyroBiasZ(9.5f);

            VehicleStateLogTestRow projected{};
            projected.Set(state);

            VehicleStateLogTestFlattenedRow flattened{};
            flattened.ukf_state_px_m = state.GetPositionX();
            flattened.ukf_state_py_m = state.GetPositionY();
            flattened.ukf_state_psi_rad = state.GetOrientation();
            flattened.ukf_state_u_mps = state.GetVelocity();
            flattened.ukf_state_v_mps = state.GetLateralVelocity();
            flattened.ukf_state_r_radps = state.GetRotationalVelocity();
            flattened.ukf_state_omega_l_radps = state.GetWheelSpeedLeft();
            flattened.ukf_state_omega_r_radps = state.GetWheelSpeedRight();
            flattened.ukf_state_bgz_radps = state.GetGyroBiasZ();

            const std::string expectedHeader =
                "f32_ukf_state_px_m,f32_ukf_state_py_m,f32_ukf_state_psi_rad,"
                "f32_ukf_state_u_mps,f32_ukf_state_v_mps,f32_ukf_state_r_radps,"
                "f32_ukf_state_omega_l_radps,f32_ukf_state_omega_r_radps,"
                "f32_ukf_state_bgz_radps";

            Assert::AreEqual(expectedHeader, std::string(VehicleStateLogTestRow::header_cstr()));
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

