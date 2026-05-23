#include "pch.h"
#include "CppUnitTest.h"
#include "..\MazeMap\MazeMapRuntimeMmLog.h"

#include <cstdint>
#include <cstring>
#include <string>
#include <type_traits>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace
{
    struct MmLogTestVectorSource final
    {
        float pxM{};
        float pyM{};
        float headingRad{};
        float forwardSpeedMps{};
        float rightwardSpeedMps{};
        float yawRateRadps{};
        float leftWheelSpeedRadps{};
        float rightWheelSpeedRadps{};
        float gyroBiasZRadps{};
    };

#define MMLOG_TEST_VECTOR_ENTRY_FIELDS(X) \
    X(float, px_m) \
    X(float, py_m) \
    X(float, heading_rad) \
    X(float, vf_mps) \
    X(float, vr_mps) \
    X(float, yaw_rate_radps) \
    X(float, left_wheel_speed_radps) \
    X(float, right_wheel_speed_radps) \
    X(float, gyro_bias_z_radps)

    MMLOG_DEFINE_PRIVATE_ENTRY_WITH_BODY(
        MmLogTestVectorEntry,
        MMLOG_TEST_VECTOR_ENTRY_FIELDS,
        void Set(const MmLogTestVectorSource& source) noexcept
        {
            px_m = source.pxM;
            py_m = source.pyM;
            heading_rad = source.headingRad;
            vf_mps = source.forwardSpeedMps;
            vr_mps = source.rightwardSpeedMps;
            yaw_rate_radps = source.yawRateRadps;
            left_wheel_speed_radps = source.leftWheelSpeedRadps;
            right_wheel_speed_radps = source.rightWheelSpeedRadps;
            gyro_bias_z_radps = source.gyroBiasZRadps;
        });

#define MMLOG_TEST_HIERARCHICAL_ROW_FIELDS(X) \
    X(std::uint32_t, tick_us) \
    X(std::uint8_t, mode_id) \
    X(MmLogTestVectorEntry, state) \
    X(float, command_mps)

    MMLOG_DEFINE_PRIVATE_ROW_WITH_BODY(
        MmLogTestHierarchicalRow,
        MMLOG_TEST_HIERARCHICAL_ROW_FIELDS,
        void SetTickUs(const std::uint32_t tickUs) noexcept { tick_us = tickUs; }
        void SetModeId(const std::uint8_t modeId) noexcept { mode_id = modeId; }
        void SetVectorEntry(const MmLogTestVectorSource& source) noexcept { state.Set(source); }
        void SetCommandMps(const float commandMps) noexcept { command_mps = commandMps; });

#define MMLOG_TEST_FLATTENED_ROW_FIELDS(X) \
    X(std::uint32_t, tick_us) \
    X(std::uint8_t, mode_id) \
    X(float, state_px_m) \
    X(float, state_py_m) \
    X(float, state_heading_rad) \
    X(float, state_vf_mps) \
    X(float, state_vr_mps) \
    X(float, state_yaw_rate_radps) \
    X(float, state_left_wheel_speed_radps) \
    X(float, state_right_wheel_speed_radps) \
    X(float, state_gyro_bias_z_radps) \
    X(float, command_mps)

    MMLOG_DEFINE_ROW(MmLogTestFlattenedRow, MMLOG_TEST_FLATTENED_ROW_FIELDS);

#undef MMLOG_TEST_FLATTENED_ROW_FIELDS
#undef MMLOG_TEST_HIERARCHICAL_ROW_FIELDS
#undef MMLOG_TEST_VECTOR_ENTRY_FIELDS
}

namespace MazeMap
{
    TEST_CLASS(MmLogSchemaCompositionTest)
    {
    public:
        TEST_METHOD(HierarchicalRowHeaderStringFlattensEntryWithMemberPrefix)
        {
            const std::string expected =
                "u32_tick_us,u8_mode_id,"
                "f32_state_px_m,f32_state_py_m,f32_state_heading_rad,"
                "f32_state_vf_mps,f32_state_vr_mps,f32_state_yaw_rate_radps,"
                "f32_state_left_wheel_speed_radps,f32_state_right_wheel_speed_radps,"
                "f32_state_gyro_bias_z_radps,"
                "f32_command_mps";

            Assert::AreEqual(expected, std::string(MmLogTestHierarchicalRow::header_cstr()));
        }

        TEST_METHOD(HierarchicalRowSizeMatchesFlattenedScalarWidth)
        {
            static_assert(std::is_trivially_copyable<MmLogTestVectorEntry>::value);
            static_assert(std::is_standard_layout<MmLogTestVectorEntry>::value);
            static_assert(std::is_trivially_copyable<MmLogTestHierarchicalRow>::value);
            static_assert(std::is_standard_layout<MmLogTestHierarchicalRow>::value);

            constexpr std::size_t expectedBytes =
                sizeof(std::uint32_t) +
                sizeof(std::uint8_t) +
                (9U * sizeof(float)) +
                sizeof(float);

            Assert::AreEqual(expectedBytes, sizeof(MmLogTestHierarchicalRow));
            Assert::AreEqual(expectedBytes, MmLogTestHierarchicalRow::row_bytes);
        }

        TEST_METHOD(HierarchicalRowMemoryLayoutMatchesManuallyFlattenedSchema)
        {
            const MmLogTestVectorSource source{
                1.25f,
                -2.5f,
                3.75f,
                -4.125f,
                5.5f,
                -6.625f,
                7.875f,
                -8.25f,
                9.5f
            };

            MmLogTestHierarchicalRow hierarchical{};
            hierarchical.SetTickUs(0x01020304U);
            hierarchical.SetModeId(0x7BU);
            hierarchical.SetVectorEntry(source);
            hierarchical.SetCommandMps(-10.75f);

            MmLogTestFlattenedRow flattened{};
            flattened.tick_us = 0x01020304U;
            flattened.mode_id = 0x7BU;
            flattened.state_px_m = source.pxM;
            flattened.state_py_m = source.pyM;
            flattened.state_heading_rad = source.headingRad;
            flattened.state_vf_mps = source.forwardSpeedMps;
            flattened.state_vr_mps = source.rightwardSpeedMps;
            flattened.state_yaw_rate_radps = source.yawRateRadps;
            flattened.state_left_wheel_speed_radps = source.leftWheelSpeedRadps;
            flattened.state_right_wheel_speed_radps = source.rightWheelSpeedRadps;
            flattened.state_gyro_bias_z_radps = source.gyroBiasZRadps;
            flattened.command_mps = -10.75f;

            Assert::AreEqual(sizeof(flattened), sizeof(hierarchical));
            Assert::AreEqual(
                0,
                std::memcmp(
                    &flattened,
                    &hierarchical,
                    sizeof(MmLogTestHierarchicalRow)));
        }
    };
}
