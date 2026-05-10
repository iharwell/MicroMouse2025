#pragma once

#include "CommandVector.h"
#include "CellCoordinates.h"
#include "CommandPD.h"
#include "Direction.h"
#include "MazeMapRuntimeCore.h"
#include "SensorSnapshot.h"
#include "VehicleState.h"

#include <cstdint>

class DriveBase;

namespace MazeMap::App::Internal
{
    class Drive;
    class SharedRobotRuntime;

    class EXPORT WallTouch final
    {
    public:
        WallTouch();

        void SetLimits(const MotionLimits& limits) noexcept;
        void SetAllowPassThroughNoWall(bool allowPassThroughNoWall) noexcept;

        bool Active() const noexcept;
        void Cancel() noexcept;

        void Start(const MazeMap::CellCoordinates& wallCell, MazeMap::Direction wallDirection);
        CommandVector GetNextControls(bool& done);

    private:
        friend class SharedRobotRuntime;

        enum class ActivePhase : std::uint8_t
        {
            None,
            Seek,
            Seat,
            Square,
            HoldBeforeReturn,
            ReturnToPreferred
        };

        struct State final
        {
            MazeMap::Direction wallDirection{ MazeMap::None };
            float preferredXMeters{};
            float preferredYMeters{};
            float contactXMeters{};
            float contactYMeters{};
            float targetYawRad{};
            float startDistanceM{};
            float lastProgressDistanceM{};
            float minLatchTravelM{};
            float maxApproachTravelM{};
            unsigned long touchStartMs{};
            unsigned long phaseStartMs{};
            unsigned long contactCandidateStartMs{};
            unsigned long frontSignalMissingStartMs{};
            unsigned long squareStableStartMs{};
            unsigned long lastMotionMs{};
            bool resetPoseDuringHold{};
            bool poseResetApplied{};
        };

        void AttachRuntime(SharedRobotRuntime& runtime) noexcept;

        bool CanStart() const noexcept;
        void ResetActivePhase() noexcept;
        void SetFault(const char* reason) noexcept;
        bool BeginHoldBeforeReturn(bool resetPoseDuringHold) noexcept;
        bool BeginReturnToPreferred() noexcept;
        CommandVector ForwardControl(
            const MazeMap::VehicleState& state,
            float desiredSpeedMps,
            float yawRateBiasRadps = 0.0f) const;

        CommandVector SeekControls(
            const MazeMap::VehicleState& state,
            const SensorSnapshot& sensors,
            const DriveTelemetry& driveTelemetry,
            bool& done);
        CommandVector SeatControls(const MazeMap::VehicleState& state, bool& done);
        CommandVector SquareControls(
            const MazeMap::VehicleState& state,
            const SensorSnapshot& sensors,
            bool& done);
        CommandVector HoldBeforeReturnControls(
            const MazeMap::VehicleState& state,
            bool& done);
        CommandVector ReturnToPreferredControls(bool& done);

        SharedRobotRuntime* _runtime{};
        DriveBase* _drive{};
        Drive* _driveService{};
        MotionLimits _limits{};
        MazeMap::CommandPD _trackingCommandPd{};
        bool _allowPassThroughNoWall{};
        bool _faulted{};
        ActivePhase _activePhase{ ActivePhase::None };
        State _state{};
    };
}
