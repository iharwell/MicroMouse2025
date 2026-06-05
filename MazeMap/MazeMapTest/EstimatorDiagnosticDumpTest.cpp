#include "pch.h"
#include "CppUnitTest.h"

#include "EstimatorModeAndDiagnosticsTestSupport.h"

#include <cmath>
#include <limits>
#include <string>
#include <vector>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    using namespace EstimatorModeAndDiagnosticsTestSupport;

    TEST_CLASS(EstimatorDiagnosticDumpTest)
    {
    public:
        TEST_METHOD(ReportsPivotScrubInactive)
        {
            const DiagnosticResult result = RunDiagnosticScenario();
            const std::wstring message = BoolMessage(L"pivot_scrub_mode", result.pivotScrubMode, L"false", ScenarioMessage(result.status).c_str());
            Assert::IsFalse(result.pivotScrubMode, message.c_str());
        }

        TEST_METHOD(ReportsEncoderBodyUpdateNotSkipped)
        {
            const DiagnosticResult result = RunDiagnosticScenario();
            const std::wstring message = BoolMessage(L"encoder_body_update_skipped", result.encoderBodyUpdateSkipped, L"false", ScenarioMessage(result.status).c_str());
            Assert::IsFalse(result.encoderBodyUpdateSkipped, message.c_str());
        }

        TEST_METHOD(ReportsZeroUSoftNotApplied)
        {
            const DiagnosticResult result = RunDiagnosticScenario();
            const std::wstring message = BoolMessage(L"zero_u_soft_applied", result.zeroUSoftApplied, L"false", ScenarioMessage(result.status).c_str());
            Assert::IsFalse(result.zeroUSoftApplied, message.c_str());
        }

        TEST_METHOD(EncoderMaskedDeltaIsZero)
        {
            const DiagnosticResult result = RunDiagnosticScenario();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(0.0f, result.encoderMaskedDeltaNorm, 1.0e-6f, message.c_str());
        }

        TEST_METHOD(ZeroUInnovationIsZero)
        {
            const DiagnosticResult result = RunDiagnosticScenario();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(0.0f, result.zeroUInnovationMps, 1.0e-6f, message.c_str());
        }

        TEST_METHOD(GyroMaskedDeltaIsZero)
        {
            const DiagnosticResult result = RunDiagnosticScenario();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(0.0f, result.gyroMaskedDeltaNorm, 1.0e-6f, message.c_str());
        }

        TEST_METHOD(ContainsPivotScrubLine)
        {
            const DiagnosticResult result = RunDiagnosticScenario();
            const std::wstring message = LimitMessage(L"pivot_mode_index", static_cast<float>(result.pivotModeIndex), L"< dump_line_count", static_cast<float>(result.dumpLineCount), ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.pivotModeIndex < result.dumpLineCount, message.c_str());
        }

        TEST_METHOD(PivotScrubLineReportsInactive)
        {
            const DiagnosticResult result = RunDiagnosticScenario();
            const std::wstring message = BoolMessage(L"pivot_line_reports_inactive", result.pivotLineReportsInactive, L"true", ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.pivotLineReportsInactive, message.c_str());
        }

    };
}
