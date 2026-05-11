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
#define MMLOG_TEST_VECTOR_ENTRY_FIELDS(X) \
    X(float, px_m) \
    X(float, py_m) \
    X(float, psi_rad) \
    X(float, u_mps) \
    X(float, v_mps) \
    X(float, r_radps) \
    X(float, omega_l_radps) \
    X(float, omega_r_radps) \
    X(float, bgz_radps)

    MMLOG_DEFINE_ENTRY(MmLogTestVectorEntry, MMLOG_TEST_VECTOR_ENTRY_FIELDS);

#define MMLOG_TEST_HIERARCHICAL_ROW_FIELDS(X) \
    X(std::uint32_t, tick_us) \
    X(std::uint8_t, mode_id) \
    X(MmLogTestVectorEntry, state) \
    X(float, command_mps)

    MMLOG_DEFINE_ROW(MmLogTestHierarchicalRow, MMLOG_TEST_HIERARCHICAL_ROW_FIELDS);

#define MMLOG_TEST_FLATTENED_ROW_FIELDS(X) \
    X(std::uint32_t, tick_us) \
    X(std::uint8_t, mode_id) \
    X(float, state_px_m) \
    X(float, state_py_m) \
    X(float, state_psi_rad) \
    X(float, state_u_mps) \
    X(float, state_v_mps) \
    X(float, state_r_radps) \
    X(float, state_omega_l_radps) \
    X(float, state_omega_r_radps) \
    X(float, state_bgz_radps) \
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
                "f32_state_px_m,f32_state_py_m,f32_state_psi_rad,"
                "f32_state_u_mps,f32_state_v_mps,f32_state_r_radps,"
                "f32_state_omega_l_radps,f32_state_omega_r_radps,f32_state_bgz_radps,"
                "f32_command_mps";

            Assert::AreEqual(expected, std::string(MmLogTestHierarchicalRow::header_cstr()));
        }

        TEST_METHOD(HierarchicalRowSizeMatchesFlattenedScalarWidth)
        {
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
            MmLogTestHierarchicalRow hierarchical{};
            hierarchical.tick_us = 0x01020304U;
            hierarchical.mode_id = 0x7BU;
            hierarchical.state.px_m = 1.25f;
            hierarchical.state.py_m = -2.5f;
            hierarchical.state.psi_rad = 3.75f;
            hierarchical.state.u_mps = -4.125f;
            hierarchical.state.v_mps = 5.5f;
            hierarchical.state.r_radps = -6.625f;
            hierarchical.state.omega_l_radps = 7.875f;
            hierarchical.state.omega_r_radps = -8.25f;
            hierarchical.state.bgz_radps = 9.5f;
            hierarchical.command_mps = -10.75f;

            MmLogTestFlattenedRow flattened{};
            flattened.tick_us = 0x01020304U;
            flattened.mode_id = 0x7BU;
            flattened.state_px_m = 1.25f;
            flattened.state_py_m = -2.5f;
            flattened.state_psi_rad = 3.75f;
            flattened.state_u_mps = -4.125f;
            flattened.state_v_mps = 5.5f;
            flattened.state_r_radps = -6.625f;
            flattened.state_omega_l_radps = 7.875f;
            flattened.state_omega_r_radps = -8.25f;
            flattened.state_bgz_radps = 9.5f;
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
