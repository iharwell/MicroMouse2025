#pragma once
// Declares shared runtime state, calibration, and sensor-processing utilities used across the MazeMap application runtime.

#include "Defines.h"
#include "DiagonalWallCentering.h"
#include "EncoderStallPolicy.h"
#include "Maze.h"
#include "MissionStartPolicy.h"
#include "MotionLimits.h"
#include "PDCluster.h"
#include "Vehicle.h"
#include "VehicleState.h"
#include "WallDetectionThresholds.h"
#include "WallObservationPipeline.h"
#include "WallSensorCalibration.h"
#include "Pins.h"

#if !defined(ARDUINO_TEENSY41) && !defined(MAZEMAP_PINS_NAMESPACE_AVAILABLE)
namespace MazeMap::Pins
{
    constexpr uint8_t R_MotorA = 5;
    constexpr uint8_t R_MotorB = 6;
    constexpr uint8_t R_EncA = 7;
    constexpr uint8_t R_EncB = 8;
    constexpr uint8_t L_MotorA = 24;
    constexpr uint8_t L_MotorB = 25;
    constexpr uint8_t L_EncA = 2;
    constexpr uint8_t L_EncB = 3;
    constexpr uint8_t Fan_CTRL = 4;
    constexpr uint8_t IMU_INT_1A = 32;
    constexpr uint8_t IMU_INT_1B = 33;
    constexpr uint8_t LED_Ctrl_Forward_Right = 19;
    constexpr uint8_t LED_Ctrl_Forward_Left = 18;
    constexpr uint8_t LED_Ctrl_Side_Right = 17;
    constexpr uint8_t LED_Ctrl_Side_Left = 16;
}
#endif

#if !defined(MAZEMAP_PINS_NAMESPACE_AVAILABLE)
namespace Pins = MazeMap::Pins;
#endif

#if !defined(ARDUINO_TEENSY41)
inline bool SetupHardware()
{
    return true;
}
#endif

#include "CoreConfig.h"
#include "SensorTelemetryTypes.h"
#include "WallSensorRuntimeTypes.h"


#include "DiagnosticConfig.h"
#include "AuxMeasurementConfig.h"
#include "FrontWallCharacterizationConfig.h"
#include "LedCalibrationConfig.h"
#include "EigenCompat.h"

inline uint8_t ReadDrivenLowPinWithPullup(uint8_t pin)
{
    pinMode(pin, INPUT_PULLUP);
    delayMicroseconds(20);
    const uint8_t level = static_cast<uint8_t>(digitalRead(pin));
    pinMode(pin, INPUT);
    return level;
}

inline void FormatHexByte(uint8_t value, char (&buffer)[3])
{
    snprintf(buffer, sizeof(buffer), "%02X", static_cast<unsigned>(value));
}

inline const char* CalibrationWallName(CalibrationWall wall)
{
    switch (wall)
    {
    case CalibrationWall::West:
        return "west";
    case CalibrationWall::East:
        return "east";
    case CalibrationWall::South:
        return "south";
    case CalibrationWall::North:
        return "north";
    default:
        return "unknown";
    }
}

inline const char* WallTouchOutcomeName(WallTouchOutcome outcome)
{
    switch (outcome)
    {
    case WallTouchOutcome::SeatedContact:
        return "seated_contact";
    case WallTouchOutcome::PassedThroughNoWall:
        return "passed_through";
    default:
        return "unknown";
    }
}

inline const char* DirectionName(MazeMap::Direction direction)
{
    switch (direction)
    {
    case MazeMap::Up:
        return "up";
    case MazeMap::UpRight:
        return "up_right";
    case MazeMap::Right:
        return "right";
    case MazeMap::DownRight:
        return "down_right";
    case MazeMap::Down:
        return "down";
    case MazeMap::DownLeft:
        return "down_left";
    case MazeMap::Left:
        return "left";
    case MazeMap::UpLeft:
        return "up_left";
    default:
        return "unknown";
    }
}

inline const char* WallStateName(MazeMap::WallState state)
{
    switch (state)
    {
    case MazeMap::NoWall:
        return "no_wall";
    case MazeMap::Wall:
        return "wall";
    case MazeMap::Unknown:
    default:
        return "unknown";
    }
}

inline float SignF(float value)
{
    return static_cast<float>((value > 0.0f) - (value < 0.0f));
}

inline float WrapAngleRad(float angle)
{
    return std::remainder(angle, TWO_PI_F);
}

inline float AngleErrorRad(float target, float measured)
{
    return WrapAngleRad(target - measured);
}

inline Eigen::Vector2f DirectionToUnitVector(MazeMap::Direction direction)
{
    float dx = 0.0f;
    float dy = 0.0f;
    MazeMap::GetHeading(direction, dx, dy);
    return Eigen::Vector2f(dx, dy);
}

inline float DirectionToYawRad(MazeMap::Direction direction)
{
    switch (direction)
    {
    case MazeMap::Up:
        return 0.0f;
    case MazeMap::UpRight:
        return 0.25f * PI_F;
    case MazeMap::Right:
        return HALF_PI_F;
    case MazeMap::DownRight:
        return 0.75f * PI_F;
    case MazeMap::Down:
        return PI_F;
    case MazeMap::DownLeft:
        return -0.75f * PI_F;
    case MazeMap::Left:
        return -HALF_PI_F;
    case MazeMap::UpLeft:
        return -0.25f * PI_F;
    default:
        return 0.0f;
    }
}

inline Eigen::Vector2f HeadingUnitFromYawRad(float yawRad)
{
    return Eigen::Vector2f(sinf(yawRad), cosf(yawRad));
}

inline float HeadingErrorRad(const Eigen::Vector2f& targetHeading, const Eigen::Vector2f& measuredHeading)
{
    const float dot = (std::clamp)(targetHeading.dot(measuredHeading), -1.0f, 1.0f);
    const float cross = (targetHeading.x() * measuredHeading.y()) - (targetHeading.y() * measuredHeading.x());
    return atan2f(cross, dot);
}

inline float ReachableSpeedWithBoundary(float boundarySpeed, float distance, float accel)
{
    if (accel <= 0.0f)
    {
        return (std::max)(boundarySpeed, 0.0f);
    }

    return MazeMap::LinearKinematics::V1IgnoringT((std::max)(distance, 0.0f), boundarySpeed, accel);
}

inline Eigen::Vector2f RightUnitFromHeading(const Eigen::Vector2f& headingUnit)
{
    return Eigen::Vector2f(headingUnit.y(), -headingUnit.x());
}

inline Eigen::Vector2f RotateBodyVectorToWorld(
    const MazeMap::VehicleState& state,
    const Eigen::Vector2f& bodyVector)
{
    const Eigen::Vector2f headingUnit = state.GetHeadingUnit();
    const Eigen::Vector2f rightUnit = RightUnitFromHeading(headingUnit);
    return Eigen::Vector2f(
        (rightUnit.x() * bodyVector.x()) + (headingUnit.x() * bodyVector.y()),
        (rightUnit.y() * bodyVector.x()) + (headingUnit.y() * bodyVector.y()));
}

inline bool TryComputeEffectiveTurnRadiusM(
    float leftDistanceM,
    float rightDistanceM,
    float yawChangeRad,
    float& turnRadiusM)
{
    if (!std::isfinite(leftDistanceM) ||
        !std::isfinite(rightDistanceM) ||
        !std::isfinite(yawChangeRad) ||
        std::fabs(yawChangeRad) < 1.0e-4f)
    {
        turnRadiusM = 0.0f;
        return false;
    }

    turnRadiusM = std::fabs(0.5f * (leftDistanceM + rightDistanceM) / yawChangeRad);
    return std::isfinite(turnRadiusM) && (turnRadiusM > 0.0f);
}

inline bool TryGetCellCenterMeters(const MazeMap::CellCoordinates& cell, float& xMeters, float& yMeters)
{
    MazeMap::MazeLocation::CellCenter(cell).GetPhysicalLocation(MazeMap::Config::kCellSizeM, xMeters, yMeters);
    return std::isfinite(xMeters) && std::isfinite(yMeters);
}

inline bool TryGetCellWallFaceCoordinateM(
    const MazeMap::CellCoordinates& cell,
    MazeMap::Direction wallDirection,
    float& coordinateM)
{
    coordinateM = 0.0f;
    const float cellBaseXM = static_cast<float>(cell.GetX()) * MazeMap::Config::kCellSizeM;
    const float cellBaseYM = static_cast<float>(cell.GetY()) * MazeMap::Config::kCellSizeM;
    switch (wallDirection)
    {
    case MazeMap::Left:
        coordinateM = cellBaseXM + MazeMap::ComputeCellInnerMinCoordinateM(MazeMap::Config::kMazeWallThicknessM);
        return true;
    case MazeMap::Right:
        coordinateM = cellBaseXM + MazeMap::ComputeCellInnerMaxCoordinateM(MazeMap::Config::kCellSizeM, MazeMap::Config::kMazeWallThicknessM);
        return true;
    case MazeMap::Down:
        coordinateM = cellBaseYM + MazeMap::ComputeCellInnerMinCoordinateM(MazeMap::Config::kMazeWallThicknessM);
        return true;
    case MazeMap::Up:
        coordinateM = cellBaseYM + MazeMap::ComputeCellInnerMaxCoordinateM(MazeMap::Config::kCellSizeM, MazeMap::Config::kMazeWallThicknessM);
        return true;
    default:
        return false;
    }
}

inline bool TryComputeDistanceToCellWallM(
    const MazeMap::VehicleState& state,
    const MazeMap::WallSensor& sensor,
    const MazeMap::CellCoordinates& cell,
    MazeMap::Direction wallDirection,
    float& distanceM)
{
    distanceM = 0.0f;

    float wallCoordinateM = 0.0f;
    if (!TryGetCellWallFaceCoordinateM(cell, wallDirection, wallCoordinateM))
    {
        return false;
    }

    const Eigen::Vector2f sensorPosition = sensor.WorldPosition(state);
    const Eigen::Vector2f sensorFacing = sensor.WorldFacing(state);
    const float cellBaseXM = static_cast<float>(cell.GetX()) * MazeMap::Config::kCellSizeM;
    const float cellBaseYM = static_cast<float>(cell.GetY()) * MazeMap::Config::kCellSizeM;
    const float cellInnerMinXM = cellBaseXM + MazeMap::ComputeCellInnerMinCoordinateM(MazeMap::Config::kMazeWallThicknessM);
    const float cellInnerMaxXM = cellBaseXM + MazeMap::ComputeCellInnerMaxCoordinateM(MazeMap::Config::kCellSizeM, MazeMap::Config::kMazeWallThicknessM);
    const float cellInnerMinYM = cellBaseYM + MazeMap::ComputeCellInnerMinCoordinateM(MazeMap::Config::kMazeWallThicknessM);
    const float cellInnerMaxYM = cellBaseYM + MazeMap::ComputeCellInnerMaxCoordinateM(MazeMap::Config::kCellSizeM, MazeMap::Config::kMazeWallThicknessM);

    switch (wallDirection)
    {
    case MazeMap::Left:
    case MazeMap::Right:
    {
        if (((wallDirection == MazeMap::Left) && sensorFacing.x() >= -0.1f) ||
            ((wallDirection == MazeMap::Right) && sensorFacing.x() <= 0.1f))
        {
            return false;
        }

        const float candidateDistanceM = (wallCoordinateM - sensorPosition.x()) / sensorFacing.x();
        const float intersectionYM = sensorPosition.y() + (candidateDistanceM * sensorFacing.y());
        if (candidateDistanceM <= 0.0f ||
            intersectionYM < (cellInnerMinYM - 0.005f) ||
            intersectionYM > (cellInnerMaxYM + 0.005f))
        {
            return false;
        }

        distanceM = candidateDistanceM;
        return std::isfinite(distanceM) && distanceM > 0.0f;
    }
    case MazeMap::Down:
    case MazeMap::Up:
    {
        if (((wallDirection == MazeMap::Down) && sensorFacing.y() >= -0.1f) ||
            ((wallDirection == MazeMap::Up) && sensorFacing.y() <= 0.1f))
        {
            return false;
        }

        const float candidateDistanceM = (wallCoordinateM - sensorPosition.y()) / sensorFacing.y();
        const float intersectionXM = sensorPosition.x() + (candidateDistanceM * sensorFacing.x());
        if (candidateDistanceM <= 0.0f ||
            intersectionXM < (cellInnerMinXM - 0.005f) ||
            intersectionXM > (cellInnerMaxXM + 0.005f))
        {
            return false;
        }

        distanceM = candidateDistanceM;
        return std::isfinite(distanceM) && distanceM > 0.0f;
    }
    default:
        return false;
    }
}

inline bool TryComputeFrontWallCandidateDistancesForPose(
    const MazeMap::VehicleState& state,
    const MazeMap::Vehicle& vehicle,
    const MazeMap::CellCoordinates& observedCell,
    MazeMap::Direction observedDirection,
    float& frontLeftDistanceM,
    float& frontRightDistanceM)
{
    frontLeftDistanceM = NAN;
    frontRightDistanceM = NAN;
    const MazeMap::Direction forwardDirection = observedDirection + MazeMap::Forward;
    const bool haveFrontLeftDistance =
        TryComputeDistanceToCellWallM(
            state,
            vehicle.FrontLeftWallSensor(),
            observedCell,
            forwardDirection,
            frontLeftDistanceM);
    const bool haveFrontRightDistance =
        TryComputeDistanceToCellWallM(
            state,
            vehicle.FrontRightWallSensor(),
            observedCell,
            forwardDirection,
            frontRightDistanceM);
    return haveFrontLeftDistance || haveFrontRightDistance;
}

inline bool TryComputeFrontWallObservationSampleDistanceM(
    const MazeMap::Vehicle& vehicle,
    const MazeMap::WallSensor& sensor,
    uint8_t sampleIndex,
    float& distanceM)
{
    distanceM = 0.0f;

    float poseXM = 0.0f;
    float poseYM = 0.0f;
    const float sideSensorForwardOffsetM =
        (std::max)(vehicle.SideLeftWallSensor().GetPosition().y(), vehicle.SideRightWallSensor().GetPosition().y());
    const MazeMap::CellCoordinates observedCell(0, 0);
    if (!MazeMap::TryComputeSideWallObservationSamplePoseM(
            observedCell,
            MazeMap::Up,
            MazeMap::Config::kCellSizeM,
            MazeMap::Config::kMazeWallThicknessM,
            sideSensorForwardOffsetM,
            MazeMap::Config::kSideWallSegmentCenterFraction,
            sampleIndex,
            MazeMap::Config::kSearchRollingObservationSampleCount,
            poseXM,
            poseYM))
    {
        return false;
    }

    MazeMap::VehicleState state{};
    state.SetPosition(Eigen::Vector2f(poseXM, poseYM));
    state.SetHeading(DirectionToYawRad(MazeMap::Up));
    state.SetForwardVelocity(0.0f);
    state.SetYawRate(0.0f);
    return TryComputeDistanceToCellWallM(state, sensor, observedCell, MazeMap::Up, distanceM);
}

inline bool TryComputeWallTouchTargetCoordinateForCellWall(
    const MazeMap::CellCoordinates& cell,
    MazeMap::Direction wallDirection,
    float& targetCoordinateM,
    CalibrationWall& calibrationWall)
{
    targetCoordinateM = 0.0f;
    float wallFaceCoordinateM = 0.0f;
    if (!TryGetCellWallFaceCoordinateM(cell, wallDirection, wallFaceCoordinateM))
    {
        return false;
    }

    switch (wallDirection)
    {
    case MazeMap::Left:
        calibrationWall = CalibrationWall::West;
        targetCoordinateM = wallFaceCoordinateM + MazeMap::Config::kWallTouchContactStandoffM;
        return true;
    case MazeMap::Right:
        calibrationWall = CalibrationWall::East;
        targetCoordinateM = wallFaceCoordinateM - MazeMap::Config::kWallTouchContactStandoffM;
        return true;
    case MazeMap::Down:
        calibrationWall = CalibrationWall::South;
        targetCoordinateM = wallFaceCoordinateM + MazeMap::Config::kWallTouchContactStandoffM;
        return true;
    case MazeMap::Up:
        calibrationWall = CalibrationWall::North;
        targetCoordinateM = wallFaceCoordinateM - MazeMap::Config::kWallTouchContactStandoffM;
        return true;
    default:
        return false;
    }
}

inline bool TryComputePoseAxisFromObservedWall(
    const MazeMap::VehicleState& state,
    const MazeMap::WallSensor& sensor,
    float measuredDistanceM,
    const MazeMap::CellCoordinates& cell,
    MazeMap::Direction wallDirection,
    float& coordinateM)
{
    coordinateM = 0.0f;
    if (!std::isfinite(measuredDistanceM) || measuredDistanceM <= 0.0f)
    {
        return false;
    }

    float wallCoordinateM = 0.0f;
    if (!TryGetCellWallFaceCoordinateM(cell, wallDirection, wallCoordinateM))
    {
        return false;
    }

    const Eigen::Vector2f worldOffset = RotateBodyVectorToWorld(state, sensor.GetPosition());
    const Eigen::Vector2f sensorFacing = sensor.WorldFacing(state);
    if (wallDirection == MazeMap::Left || wallDirection == MazeMap::Right)
    {
        if (!std::isfinite(worldOffset.x()) ||
            !std::isfinite(sensorFacing.x()) ||
            ((wallDirection == MazeMap::Left) && sensorFacing.x() >= -0.1f) ||
            ((wallDirection == MazeMap::Right) && sensorFacing.x() <= 0.1f))
        {
            return false;
        }

        coordinateM = wallCoordinateM - worldOffset.x() - (measuredDistanceM * sensorFacing.x());
        return std::isfinite(coordinateM);
    }

    if (!std::isfinite(worldOffset.y()) ||
        !std::isfinite(sensorFacing.y()) ||
        ((wallDirection == MazeMap::Down) && sensorFacing.y() >= -0.1f) ||
        ((wallDirection == MazeMap::Up) && sensorFacing.y() <= 0.1f))
    {
        return false;
    }

    coordinateM = wallCoordinateM - worldOffset.y() - (measuredDistanceM * sensorFacing.y());
    return std::isfinite(coordinateM);
}

inline bool HysteresisWall(bool currentState, float distanceM, float onThresholdM, float offThresholdM)
{
    if (currentState)
    {
        return distanceM < offThresholdM;
    }
    return distanceM < onThresholdM;
}

inline bool IsApproximatelyDiagonalHeadingUnit(const Eigen::Vector2f& headingUnit)
{
    const float absX = std::fabs(headingUnit.x());
    const float absY = std::fabs(headingUnit.y());
    return absX > 0.5f && absY > 0.5f && std::fabs(absX - absY) <= 0.15f;
}

inline MazeMap::ManeuverCode RelativeToInPlaceCode(MazeMap::RelativeDirection rel)
{
    switch (rel)
    {
    case MazeMap::Left45:
        return MazeMap::IP45_M;
    case MazeMap::Right45:
        return MazeMap::IP45;
    case MazeMap::Left90:
        return MazeMap::IP90_M;
    case MazeMap::Right90:
        return MazeMap::IP90;
    case MazeMap::Left135:
        return MazeMap::IP135_M;
    case MazeMap::Right135:
        return MazeMap::IP135;
    case MazeMap::Reverse:
        return MazeMap::IP180;
    default:
        return MazeMap::MC_NONE;
    }
}

inline bool IsStraightCode(MazeMap::ManeuverCode code)
{
    return code != MazeMap::MC_NONE && code <= MazeMap::S31;
}

inline void TrimAsciiWhitespace(char* text)
{
    if (text == nullptr)
    {
        return;
    }

    char* start = text;
    while (*start != '\0' && isspace(static_cast<unsigned char>(*start)) != 0)
    {
        ++start;
    }

    if (start != text)
    {
        memmove(text, start, strlen(start) + 1U);
    }

    size_t length = strlen(text);
    while (length > 0U && isspace(static_cast<unsigned char>(text[length - 1U])) != 0)
    {
        text[length - 1U] = '\0';
        --length;
    }
}

inline void NormalizeToken(char* text)
{
    TrimAsciiWhitespace(text);
    if (text == nullptr)
    {
        return;
    }

    for (char* cursor = text; *cursor != '\0'; ++cursor)
    {
        *cursor = static_cast<char>(toupper(static_cast<unsigned char>(*cursor)));
    }
}

inline bool TryParseBaseManeuverCodeName(const char* token, MazeMap::ManeuverCode& code)
{
    if (token == nullptr || token[0] == '\0')
    {
        return false;
    }

    if (token[0] == 'S' && isdigit(static_cast<unsigned char>(token[1])) != 0)
    {
        char* end = nullptr;
        const long value = strtol(token + 1, &end, 10);
        if (end != nullptr && *end == '\0' && value >= 1L && value <= 31L)
        {
            code = static_cast<MazeMap::ManeuverCode>(value);
            return true;
        }
    }

    if (strcmp(token, "IP45") == 0) { code = MazeMap::IP45; return true; }
    if (strcmp(token, "IP90") == 0) { code = MazeMap::IP90; return true; }
    if (strcmp(token, "IP135") == 0) { code = MazeMap::IP135; return true; }
    if (strcmp(token, "IP180") == 0) { code = MazeMap::IP180; return true; }
    if (strcmp(token, "S45SS") == 0) { code = MazeMap::S45SS; return true; }
    if (strcmp(token, "S45SD") == 0) { code = MazeMap::S45SD; return true; }
    if (strcmp(token, "S45LS") == 0) { code = MazeMap::S45LS; return true; }
    if (strcmp(token, "S45LD") == 0) { code = MazeMap::S45LD; return true; }
    if (strcmp(token, "S90SS") == 0) { code = MazeMap::S90SS; return true; }
    if (strcmp(token, "S90SD") == 0) { code = MazeMap::S90SD; return true; }
    if (strcmp(token, "S90LS") == 0) { code = MazeMap::S90LS; return true; }
    if (strcmp(token, "S90LD") == 0) { code = MazeMap::S90LD; return true; }
    if (strcmp(token, "S135SS") == 0) { code = MazeMap::S135SS; return true; }
    if (strcmp(token, "S135SD") == 0) { code = MazeMap::S135SD; return true; }
    if (strcmp(token, "S135LS") == 0) { code = MazeMap::S135LS; return true; }
    if (strcmp(token, "S135LD") == 0) { code = MazeMap::S135LD; return true; }
    if (strcmp(token, "S180SS") == 0) { code = MazeMap::S180SS; return true; }
    if (strcmp(token, "S180LS") == 0) { code = MazeMap::S180LS; return true; }
    if (strcmp(token, "S90ELD") == 0) { code = MazeMap::S90ELD; return true; }
    if (strcmp(token, "S180ELS") == 0) { code = MazeMap::S180ELS; return true; }
    return false;
}

inline bool TryParseManeuverCodeToken(const char* token, MazeMap::ManeuverCode& code)
{
    if (token == nullptr)
    {
        return false;
    }

    char normalized[24] = {};
    snprintf(normalized, sizeof(normalized), "%s", token);
    NormalizeToken(normalized);
    if (normalized[0] == '\0')
    {
        return false;
    }

    char* numericEnd = nullptr;
    const long numericValue = strtol(normalized, &numericEnd, 0);
    if (numericEnd != nullptr && *numericEnd == '\0' && numericValue >= 0L && numericValue <= 255L)
    {
        code = static_cast<MazeMap::ManeuverCode>(numericValue);
        return true;
    }

    const size_t length = strlen(normalized);
    if (length > 2U && strcmp(normalized + length - 2U, "_M") == 0)
    {
        normalized[length - 2U] = '\0';
        MazeMap::ManeuverCode baseCode = MazeMap::MC_NONE;
        if (!TryParseBaseManeuverCodeName(normalized, baseCode))
        {
            return false;
        }

        code = static_cast<MazeMap::ManeuverCode>(
            static_cast<uint8_t>(baseCode) |
            static_cast<uint8_t>(MazeMap::MIRRORED_MANEUVER_FLAG));
        return true;
    }

    return TryParseBaseManeuverCodeName(normalized, code);
}

inline void FormatManeuverCodeName(MazeMap::ManeuverCode code, char* buffer, size_t bufferSize)
{
    if (buffer == nullptr || bufferSize == 0U)
    {
        return;
    }

    const bool mirrored = (code & MazeMap::MIRRORED_MANEUVER_FLAG) == MazeMap::MIRRORED_MANEUVER_FLAG;
    const MazeMap::ManeuverCode baseCode = code & MazeMap::INVERTED_MIRRORED_MANEUVER_FLAG;

    if (baseCode >= MazeMap::S1 && baseCode <= MazeMap::S31)
    {
        snprintf(
            buffer,
            bufferSize,
            "S%u%s",
            static_cast<unsigned>(static_cast<uint8_t>(baseCode)),
            mirrored ? "_M" : "");
        return;
    }

    const char* name = "UNKNOWN";
    switch (baseCode)
    {
    case MazeMap::IP45: name = "IP45"; break;
    case MazeMap::IP90: name = "IP90"; break;
    case MazeMap::IP135: name = "IP135"; break;
    case MazeMap::IP180: name = "IP180"; break;
    case MazeMap::S45SS: name = "S45SS"; break;
    case MazeMap::S45SD: name = "S45SD"; break;
    case MazeMap::S45LS: name = "S45LS"; break;
    case MazeMap::S45LD: name = "S45LD"; break;
    case MazeMap::S90SS: name = "S90SS"; break;
    case MazeMap::S90SD: name = "S90SD"; break;
    case MazeMap::S90LS: name = "S90LS"; break;
    case MazeMap::S90LD: name = "S90LD"; break;
    case MazeMap::S135SS: name = "S135SS"; break;
    case MazeMap::S135SD: name = "S135SD"; break;
    case MazeMap::S135LS: name = "S135LS"; break;
    case MazeMap::S135LD: name = "S135LD"; break;
    case MazeMap::S180SS: name = "S180SS"; break;
    case MazeMap::S180LS: name = "S180LS"; break;
    case MazeMap::S90ELD: name = "S90ELD"; break;
    case MazeMap::S180ELS: name = "S180ELS"; break;
    default: break;
    }

    snprintf(buffer, bufferSize, "%s%s", name, mirrored ? "_M" : "");
}
