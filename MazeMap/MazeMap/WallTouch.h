#pragma once

#include "CellCoordinates.h"
#include "CommandPD.h"
#include "Direction.h"
#include "LoopController.h"
#include "MazeMapRuntimeCore.h"

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
        void SetTrackingCommandPD(MazeMap::CommandPD trackingCommandPd) noexcept;
        void SetAllowPassThroughNoWall(bool allowPassThroughNoWall) noexcept;

        bool Active() const noexcept;
        void Cancel() noexcept;

        void Start(const MazeMap::CellCoordinates& wallCell, MazeMap::Direction wallDirection);
        LoopController::ControlVector GetNextControls(bool& done);

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
        const LoopController::ModeState* TryGetLoopState() const noexcept;
        bool BeginHoldBeforeReturn(bool resetPoseDuringHold) noexcept;
        bool BeginReturnToPreferred() noexcept;
        LoopController::ControlVector ForwardControl(
            const LoopController::ModeState& state,
            float desiredSpeedMps,
            float yawRateBiasRadps = 0.0f) const;

        LoopController::ControlVector SeekControls(const LoopController::ModeState& state, bool& done);
        LoopController::ControlVector SeatControls(const LoopController::ModeState& state, bool& done);
        LoopController::ControlVector SquareControls(const LoopController::ModeState& state, bool& done);
        LoopController::ControlVector HoldBeforeReturnControls(const LoopController::ModeState& state, bool& done);
        LoopController::ControlVector ReturnToPreferredControls(bool& done);

        SharedRobotRuntime* _runtime{};
        LoopController* _loopController{};
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
