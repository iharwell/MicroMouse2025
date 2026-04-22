#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "pch.h"
#include "CppUnitTest.h"
#include "..\MazeMap\MazeMapRuntimeCore.h"
#include "..\MazeMap\MazeMapRuntimeMmLog.h"
#include "..\MazeMap\MazeMapRuntimeSignalHelpers.h"
#include "..\MazeMap\RuntimeBinaryLogSupport.h"
#include "..\MazeMap\SigmaPointSetSimplex.h"
#include "..\MazeMap\UKF.h"

#include <cstdio>
#include <array>
#include <fstream>
#include <iterator>
#include <string>
#include <cstring>
#include <limits>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

#define RUNTIME_HELPER_TEST_FIELDS(X) \
    X(std::uint32_t, seq) \
    X(float,         value) \
    X(mmlog::s32_t,  kind)

MMLOG_DEFINE_ROW(RuntimeHelperTestRow, RUNTIME_HELPER_TEST_FIELDS);

namespace MazeMap::App
{
    TEST_CLASS(MazeMapRuntimeHelperTest)
    {
    public:
        static MazeMap::WallSensor MakeTestWallSensor(
            uint8_t wallSensorPin,
            uint8_t ledPin,
            const Eigen::Vector2f& position,
            const Eigen::Vector2f& facingDirection)
        {
            const std::array<float, 8> adcToLightTable = { 0.0f, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f };
            const MazeMap::WallSensor::DistanceModel distanceModel = { 1.0f, 1.0f, 0.05f, 10.0f };
            return MazeMap::WallSensor(
                wallSensorPin,
                ledPin,
                position,
                facingDirection,
                adcToLightTable,
                distanceModel);
        }

        static std::string ReadAllBytes(const std::string& path)
        {
            std::ifstream file(path, std::ios::binary);
            return std::string((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
        }

        static std::string ReplaceExtension(const std::string& path, const char* extension)
        {
            const std::size_t dot = path.find_last_of('.');
            return (dot == std::string::npos) ? (path + extension) : (path.substr(0, dot) + extension);
        }

        static std::string BaseName(const std::string& path)
        {
            const std::size_t slash = path.find_last_of("\\/");
            return (slash == std::string::npos) ? path : path.substr(slash + 1U);
        }

        static std::string CreateTempPath(const char* extension)
        {
            static unsigned long counter = 0UL;
            char path[96] = {};
            const int length = snprintf(
                path,
                sizeof(path),
                "codex_runtime_binary_log_%lu%s",
                ++counter,
                (extension != nullptr) ? extension : "");
            Assert::IsTrue(length > 0 && length < static_cast<int>(sizeof(path)));
            std::remove(path);
            return std::string(path);
        }

        TEST_METHOD(ComputeSignalRiseAboveBaseline_ClampsInvalidAndBelowBaselineValues)
        {
            using MazeMap::App::Internal::Runtime::ComputeSignalRiseAboveBaseline;

            Assert::AreEqual(0.0f, ComputeSignalRiseAboveBaseline(std::numeric_limits<float>::quiet_NaN(), 10.0f));
            Assert::AreEqual(0.0f, ComputeSignalRiseAboveBaseline(8.0f, 10.0f));
            Assert::AreEqual(2.5f, ComputeSignalRiseAboveBaseline(12.5f, 10.0f));
        }

        TEST_METHOD(UpdateFilteredSignalState_UsesSharedHysteresisThresholds)
        {
            using MazeMap::App::Internal::Runtime::UpdateFilteredSignalState;

            float filteredSignal = 0.0f;
            bool currentState = false;
            bool initialized = false;

            Assert::IsFalse(UpdateFilteredSignalState(8.0f, 10.0f, 5.0f, filteredSignal, currentState, initialized));
            Assert::IsTrue(initialized);
            Assert::AreEqual(8.0f, filteredSignal);

            Assert::IsTrue(UpdateFilteredSignalState(12.0f, 10.0f, 5.0f, filteredSignal, currentState, initialized));
            Assert::IsTrue(currentState);

            Assert::IsTrue(UpdateFilteredSignalState(6.0f, 10.0f, 5.0f, filteredSignal, currentState, initialized));
            Assert::IsFalse(UpdateFilteredSignalState(4.0f, 10.0f, 5.0f, filteredSignal, currentState, initialized));
        }

        TEST_METHOD(ComputeCorridorError_UsesAvailableWallObservationsConsistently)
        {
            using MazeMap::App::Internal::Runtime::ComputeCorridorError;

            Assert::AreEqual(0.01f, ComputeCorridorError(0.11f, 0.09f, true, true, 0.10f), 1.0e-6f);
            Assert::AreEqual(0.02f, ComputeCorridorError(0.12f, 0.09f, true, false, 0.10f), 1.0e-6f);
            Assert::AreEqual(0.03f, ComputeCorridorError(0.12f, 0.07f, false, true, 0.10f), 1.0e-6f);
            Assert::AreEqual(0.0f, ComputeCorridorError(0.12f, 0.07f, false, false, 0.10f), 1.0e-6f);
        }

        TEST_METHOD(AsyncWallSensorSweepAwaitCompletesOutstandingStages)
        {
            HostResetDigitalPins();

            MazeMap::WallSensor frontLeft = MakeTestWallSensor(20U, 40U, Eigen::Vector2f(-0.01f, 0.02f), Eigen::Vector2f(0.0f, 1.0f));
            MazeMap::WallSensor frontRight = MakeTestWallSensor(21U, 41U, Eigen::Vector2f(0.01f, 0.02f), Eigen::Vector2f(0.0f, 1.0f));
            MazeMap::WallSensor sideLeft = MakeTestWallSensor(22U, 42U, Eigen::Vector2f(-0.02f, 0.0f), Eigen::Vector2f(-1.0f, 0.0f));
            MazeMap::WallSensor sideRight = MakeTestWallSensor(23U, 43U, Eigen::Vector2f(0.02f, 0.0f), Eigen::Vector2f(1.0f, 0.0f));

            ::AsyncWallSensorSweepRead read{};
            const uint32_t initialLedOffUs = micros();
            ::StartAsyncWallSensorSweepRead(
                frontLeft,
                initialLedOffUs,
                frontRight,
                initialLedOffUs,
                sideLeft,
                initialLedOffUs,
                sideRight,
                initialLedOffUs,
                read);

            Assert::IsTrue(read.active);
            Assert::AreEqual(HIGH, digitalRead(frontLeft.GetLedOutPin()));
            Assert::AreEqual(HIGH, digitalRead(frontRight.GetLedOutPin()));

            ::AwaitAsyncWallSensorSweepRead(read);

            Assert::IsFalse(read.active);
            Assert::AreEqual(static_cast<int>(::AsyncWallSensorSweepStage::Complete), static_cast<int>(read.stage));
            Assert::IsTrue(read.frontLeftSample.timing.observationReadyUs != 0UL);
            Assert::IsTrue(read.frontRightSample.timing.observationReadyUs != 0UL);
            Assert::IsTrue(read.sideLeftSample.timing.observationReadyUs != 0UL);
            Assert::IsTrue(read.sideRightSample.timing.observationReadyUs != 0UL);
            Assert::AreEqual(LOW, digitalRead(frontLeft.GetLedOutPin()));
            Assert::AreEqual(LOW, digitalRead(frontRight.GetLedOutPin()));
            Assert::AreEqual(LOW, digitalRead(sideLeft.GetLedOutPin()));
            Assert::AreEqual(LOW, digitalRead(sideRight.GetLedOutPin()));
            Assert::IsTrue(read.nextFrontLeftLedOffCommandUs >= initialLedOffUs);
            Assert::IsTrue(read.nextFrontRightLedOffCommandUs >= initialLedOffUs);
            Assert::IsTrue(read.nextSideLeftLedOffCommandUs >= initialLedOffUs);
            Assert::IsTrue(read.nextSideRightLedOffCommandUs >= initialLedOffUs);
        }

        TEST_METHOD(SelectSequentialRuntimeFileName_UsesExplicitNameWhenProvided)
        {
            using MazeMap::App::Internal::Runtime::SelectSequentialRuntimeFileName;

            char buffer[32] = {};
            Assert::IsTrue(SelectSequentialRuntimeFileName(buffer, sizeof(buffer), "custom.mmlog", "diag%03u.mmlog", "fallback.mmlog"));
            Assert::IsTrue(std::strcmp(buffer, "custom.mmlog") == 0);
        }

        TEST_METHOD(SelectSequentialRuntimeFileName_UsesHostFallbackWhenExplicitNameMissing)
        {
            using MazeMap::App::Internal::Runtime::SelectSequentialRuntimeFileName;

            char buffer[32] = {};
            Assert::IsTrue(SelectSequentialRuntimeFileName(buffer, sizeof(buffer), nullptr, "diag%03u.mmlog", "fallback.mmlog"));
            Assert::IsTrue(std::strcmp(buffer, "fallback.mmlog") == 0);
        }

        TEST_METHOD(BuildSiblingRuntimeFileName_ReplacesExtension)
        {
            using MazeMap::App::Internal::Runtime::BuildSiblingRuntimeFileName;

            char buffer[64] = {};
            Assert::IsTrue(BuildSiblingRuntimeFileName(buffer, sizeof(buffer), "open_floor_main.mmlog", ".events.mmlog"));
            Assert::IsTrue(std::strcmp(buffer, "open_floor_main.events.mmlog") == 0);
        }

        TEST_METHOD(DirectionToYawRad_UsesProjectUpAsZeroAndClockwisePositive)
        {
            Assert::AreEqual(0.0f, DirectionToYawRad(MazeMap::Up), 1.0e-6f);
            Assert::AreEqual(0.25f * PI_F, DirectionToYawRad(MazeMap::UpRight), 1.0e-6f);
            Assert::AreEqual(HALF_PI_F, DirectionToYawRad(MazeMap::Right), 1.0e-6f);
            Assert::AreEqual(PI_F, DirectionToYawRad(MazeMap::Down), 1.0e-6f);
            Assert::AreEqual(-HALF_PI_F, DirectionToYawRad(MazeMap::Left), 1.0e-6f);
            Assert::AreEqual(-0.25f * PI_F, DirectionToYawRad(MazeMap::UpLeft), 1.0e-6f);
        }

        TEST_METHOD(HeadingUnitFromYawRad_MapsYawZeroToPositiveY)
        {
            const Eigen::Vector2f up = HeadingUnitFromYawRad(0.0f);
            Assert::AreEqual(0.0f, up.x(), 1.0e-6f);
            Assert::AreEqual(1.0f, up.y(), 1.0e-6f);

            const Eigen::Vector2f right = HeadingUnitFromYawRad(HALF_PI_F);
            Assert::AreEqual(1.0f, right.x(), 1.0e-6f);
            Assert::AreEqual(0.0f, right.y(), 1.0e-6f);

            const Eigen::Vector2f left = HeadingUnitFromYawRad(-HALF_PI_F);
            Assert::AreEqual(-1.0f, left.x(), 1.0e-6f);
            Assert::AreEqual(0.0f, left.y(), 1.0e-6f);
        }

        TEST_METHOD(HeadingErrorRad_MatchesClockwiseYawErrorConvention)
        {
            const float upToRight =
                HeadingErrorRad(DirectionToUnitVector(MazeMap::Right), DirectionToUnitVector(MazeMap::Up));
            Assert::AreEqual(HALF_PI_F, upToRight, 1.0e-6f);

            const float rightToUp =
                HeadingErrorRad(DirectionToUnitVector(MazeMap::Up), DirectionToUnitVector(MazeMap::Right));
            Assert::AreEqual(-HALF_PI_F, rightToUp, 1.0e-6f);

            const float diagonalError = HeadingErrorRad(
                DirectionToUnitVector(MazeMap::UpRight),
                DirectionToUnitVector(MazeMap::UpLeft));
            Assert::AreEqual(
                AngleErrorRad(DirectionToYawRad(MazeMap::UpRight), DirectionToYawRad(MazeMap::UpLeft)),
                diagonalError,
                1.0e-6f);
        }

        TEST_METHOD(SigmaPointSetSimplex_UsesRecursiveWeightsAndUnitCovariance)
        {
            using MazeMap::SigmaPointSetSimplex;
            using SimplexFilter = MazeMap::UKF<3, 1>;

            Assert::AreEqual(5, SigmaPointSetSimplex::ActiveSigmaCountForDimension(3));

            Eigen::Matrix<float, 5, 1> meanWeights;
            Eigen::Matrix<float, 5, 1> covarianceWeights;
            SigmaPointSetSimplex::ComputeWeights<3>(meanWeights, covarianceWeights);

            Assert::AreEqual(0.0f, meanWeights(0), 1.0e-6f);
            Assert::AreEqual(2.0f, covarianceWeights(0), 1.0e-6f);
            for (int index = 1; index < 5; ++index)
            {
                Assert::AreEqual(0.25f, meanWeights(index), 1.0e-6f);
                Assert::AreEqual(0.25f, covarianceWeights(index), 1.0e-6f);
            }

            Eigen::Matrix<float, 3, 1> mean = Eigen::Matrix<float, 3, 1>::Zero();
            Eigen::Matrix<float, 3, 3> sqrtCovariance = Eigen::Matrix<float, 3, 3>::Identity();
            Eigen::Matrix<float, 3, 5> sigmaPoints;
            Assert::IsTrue(SigmaPointSetSimplex::GenerateSigmaPoints<3>(mean, sqrtCovariance, sigmaPoints));

            Eigen::Matrix<float, 3, 1> weightedMean = Eigen::Matrix<float, 3, 1>::Zero();
            for (int column = 0; column < 5; ++column)
            {
                weightedMean += meanWeights(column) * sigmaPoints.col(column);
            }
            Assert::AreEqual(0.0f, weightedMean(0), 1.0e-6f);
            Assert::AreEqual(0.0f, weightedMean(1), 1.0e-6f);
            Assert::AreEqual(0.0f, weightedMean(2), 1.0e-6f);

            Eigen::Matrix<float, 3, 3> weightedCovariance = Eigen::Matrix<float, 3, 3>::Zero();
            for (int column = 0; column < 5; ++column)
            {
                const Eigen::Matrix<float, 3, 1> residual = sigmaPoints.col(column) - weightedMean;
                weightedCovariance += covarianceWeights(column) * (residual * residual.transpose());
            }
            Assert::AreEqual(1.0f, weightedCovariance(0, 0), 1.0e-6f);
            Assert::AreEqual(1.0f, weightedCovariance(1, 1), 1.0e-6f);
            Assert::AreEqual(1.0f, weightedCovariance(2, 2), 1.0e-6f);
            Assert::AreEqual(0.0f, weightedCovariance(0, 1), 1.0e-6f);
            Assert::AreEqual(0.0f, weightedCovariance(0, 2), 1.0e-6f);
            Assert::AreEqual(0.0f, weightedCovariance(1, 2), 1.0e-6f);

            SimplexFilter filter;
            Assert::IsTrue(filter.sigmaPointStrategy() == SimplexFilter::SigmaPointStrategy::Simplex);
        }

        TEST_METHOD(Lsm6Dsv16xGyroProjectYawConversionFlipsSensorCounterclockwiseSign)
        {
            MazeMap::Vehicle::ImuBackLeft imu;
            constexpr int16_t rawSample = 3200;

            Assert::AreEqual(-1.0f, MazeMap::Vehicle::ImuBackLeft::ClockwiseYawFromSensorZSign(), 1.0e-6f);
            Assert::AreEqual(
                -imu.GyroRawToDps(rawSample),
                imu.GyroRawToClockwiseYawDps(rawSample),
                1.0e-6f);
        }

        TEST_METHOD(MmLogLogger_WritesRevGBindingAndSidecar)
        {
            using MazeMap::mmlog::MmLogLogger;

            const std::string primaryPath = CreateTempPath(".mmlog");
            const std::string sidecarPath = ReplaceExtension(primaryPath, ".sidecar");

            MmLogLogger log;
            Assert::IsTrue(log.open(primaryPath.c_str()));
            Assert::IsTrue(log.writeMetadata("mode", "test"));
            Assert::IsTrue(log.writeMetadata("unit_case", "runtime_helper"));
            Assert::IsTrue(log.begin(RuntimeHelperTestRow{}));

            RuntimeHelperTestRow row{};
            row.seq = 1U;
            row.value = 1.25f;
            row.kind = mmlog::hash32("TEST_KIND");
            Assert::IsTrue(log.log(row));
            Assert::IsTrue(log.writeLabel("TEST_KIND"));
            Assert::IsTrue(log.close());

            const std::string primaryBytes = ReadAllBytes(primaryPath);
            const std::size_t newline = primaryBytes.find('\n');
            Assert::IsTrue(newline != std::string::npos);
            Assert::IsTrue((std::string("sidecar_file=") + BaseName(sidecarPath)) == primaryBytes.substr(0U, newline));
            Assert::IsTrue(static_cast<size_t>(newline + 1U + (3U * sizeof(uint32_t))) == primaryBytes.size());

            const std::string sidecarText = ReadAllBytes(sidecarPath);
            Assert::IsTrue(sidecarText.find("schema_version=" + std::to_string(mmlog::kSchemaVersion) + "\n") != std::string::npos);
            Assert::IsTrue(sidecarText.find("row_bytes=12\n") != std::string::npos);
            Assert::IsTrue(sidecarText.find("mode=test\n") != std::string::npos);
            Assert::IsTrue(sidecarText.find("unit_case=runtime_helper\n") != std::string::npos);
            Assert::IsTrue(sidecarText.find("u32_seq,f32_value,s32_kind\n") != std::string::npos);
            Assert::IsTrue(sidecarText.find("LABELS:\nTEST_KIND\n") != std::string::npos);

            std::remove(primaryPath.c_str());
            std::remove(sidecarPath.c_str());
        }

        TEST_METHOD(MmLogLogger_PrimaryFileContainsOnlyBindingLineAndBinaryRows)
        {
            using MazeMap::mmlog::MmLogLogger;

            static_assert(sizeof(RuntimeHelperTestRow) == (3U * sizeof(uint32_t)));

            const std::string primaryPath = CreateTempPath(".mmlog");
            const std::string sidecarPath = ReplaceExtension(primaryPath, ".sidecar");

            RuntimeHelperTestRow firstRow{};
            firstRow.seq = 0x01020304U;
            firstRow.value = 0.5f;
            firstRow.kind = static_cast<mmlog::s32_t>(0x11223344U);

            RuntimeHelperTestRow secondRow{};
            secondRow.seq = 0x55667788U;
            secondRow.value = 2.0f;
            secondRow.kind = static_cast<mmlog::s32_t>(0x12345678U);

            MmLogLogger log;
            Assert::IsTrue(log.open(primaryPath.c_str()));
            Assert::IsTrue(log.writeMetadata("mode", "test"));
            Assert::IsTrue(log.writeMetadata("phase", "primary_bytes_only"));
            Assert::IsTrue(log.begin(RuntimeHelperTestRow{}));
            Assert::IsTrue(log.log(firstRow));
            Assert::IsTrue(log.log(secondRow));
            Assert::IsTrue(log.writeLabel("PRIMARY_BYTES_ONLY"));
            Assert::IsTrue(log.close());

            std::string expectedPrimaryBytes = std::string("sidecar_file=") + BaseName(sidecarPath) + "\n";
            expectedPrimaryBytes.append(reinterpret_cast<const char*>(&firstRow), sizeof(firstRow));
            expectedPrimaryBytes.append(reinterpret_cast<const char*>(&secondRow), sizeof(secondRow));

            const std::string primaryBytes = ReadAllBytes(primaryPath);
            Assert::AreEqual(expectedPrimaryBytes.size(), primaryBytes.size());
            Assert::IsTrue(expectedPrimaryBytes == primaryBytes);

            std::remove(primaryPath.c_str());
            std::remove(sidecarPath.c_str());
        }

        TEST_METHOD(MmLogLogger_AcceptsLongMetadataKeys)
        {
            using MazeMap::mmlog::MmLogLogger;

            const std::string primaryPath = CreateTempPath(".mmlog");
            const std::string sidecarPath = ReplaceExtension(primaryPath, ".sidecar");

            MmLogLogger log;
            Assert::IsTrue(log.open(primaryPath.c_str()));
            Assert::IsTrue(log.writeMetadata("start_marker_definitions_revision", "rev_g"));
            Assert::IsTrue(log.writeMetadata("imu_gyro_lpf1_cut213_datasheet_ref_hz", "223.000"));
            Assert::IsTrue(log.begin(RuntimeHelperTestRow{}));
            Assert::IsTrue(log.close());

            const std::string sidecarText = ReadAllBytes(sidecarPath);
            Assert::IsTrue(sidecarText.find("start_marker_definitions_revision=rev_g\n") != std::string::npos);
            Assert::IsTrue(sidecarText.find("imu_gyro_lpf1_cut213_datasheet_ref_hz=223.000\n") != std::string::npos);

            std::remove(primaryPath.c_str());
            std::remove(sidecarPath.c_str());
        }

        TEST_METHOD(MmLogLogger_WritesManyMetadataLinesWithoutUsingSidecarQueueStorage)
        {
            using MazeMap::mmlog::MmLogLogger;

            const std::string primaryPath = CreateTempPath(".mmlog");
            const std::string sidecarPath = ReplaceExtension(primaryPath, ".sidecar");

            MmLogLogger log;
            Assert::IsTrue(log.open(primaryPath.c_str()));

            const std::string value(static_cast<std::size_t>(MMLOG_METADATA_VALUE_MAX_LENGTH), 'v');
            for (std::size_t i = 0u; i < 40u; ++i)
            {
                char key[32] = {};
                const int length = snprintf(key, sizeof(key), "overflow_key_%02zu", i);
                Assert::IsTrue(length > 0 && length < static_cast<int>(sizeof(key)));
                Assert::IsTrue(log.writeMetadata(key, value.c_str()));
            }

            Assert::IsTrue(log.begin(RuntimeHelperTestRow{}));
            Assert::IsTrue(log.close());

            const std::string sidecarText = ReadAllBytes(sidecarPath);
            const std::size_t firstMetadata = sidecarText.find("overflow_key_00=");
            const std::size_t lastMetadata = sidecarText.find("overflow_key_39=");
            const std::size_t schemaVersion = sidecarText.find("schema_version=");
            const std::size_t header = sidecarText.find("u32_seq,f32_value,s32_kind\n");
            Assert::IsTrue(firstMetadata != std::string::npos);
            Assert::IsTrue(lastMetadata != std::string::npos);
            Assert::IsTrue(schemaVersion != std::string::npos);
            Assert::IsTrue(header != std::string::npos);
            Assert::IsTrue(firstMetadata < lastMetadata);
            Assert::IsTrue(lastMetadata < schemaVersion);
            Assert::IsTrue(schemaVersion < header);

            std::remove(primaryPath.c_str());
            std::remove(sidecarPath.c_str());
        }

    };
}
