#pragma once

#include "LoopController.h"
#include "MazeMapRuntimeCore.h"

#include <array>
#include <cstdint>

class DriveBase;
class RuntimeSensorSuite;

namespace MazeMap
{
    class Vehicle;
}

namespace MazeMap::App::Internal
{
    class Drive;
    class SharedRobotRuntime;
    class WallTouch;

    // Owns the shared startup-calibration service used by boot-selected modes. Like Drive, this
    // is a shared multi-tick helper that stays subordinate to the active LoopController callback:
    // the mode performs any required pre-loop `BringUp()`, then arms the service with `Start()`
    // and may call `GetNextControls(bool& done)` on each tick when it wants StartupCalibration's
    // proposed controls for the current phase.
    class EXPORT StartupCalibration final
    {
    public:
        StartupCalibration();

        // `SetIsInMaze(isInMaze)`:
        // Selects whether startup should include the maze-specific wall-calibration sequence after
        // shared bring-up.
        void SetIsInMaze(bool isInMaze) noexcept;

        // Returns whether the maze-specific startup sequence is enabled.
        bool GetIsInMaze() const noexcept;

        // Infrastructure-only hardware bring-up. This is not suitable for mode tick code and must
        // be called before LoopController execution begins.
        bool BringUp();

        // Reports whether StartupCalibration currently owns an active startup sequence.
        bool Active() const noexcept;

        // Cancels the active startup sequence immediately and clears private execution state.
        void Cancel() noexcept;

        // Arms the startup-calibration sequence using the currently configured startup mode.
        //
        // `BringUp()` must have completed successfully first.
        void Start();

        // Generic per-tick query used by the active mode callback.
        //
        // Parameters:
        // `done`:
        // Set to `true` when the active startup sequence completes normally on this tick.
        //
        // Return value:
        // Proposed control vector for the present tick. The mode may return it directly to
        // LoopController, replace it, or ignore it.
        LoopController::ControlVector GetNextControls(bool& done);

    private:
        friend class SharedRobotRuntime;

        enum class Phase : std::uint8_t
        {
            None,
            SouthStartHold,
            MoveToCenter,
            CenterHold,
            RotateWest,
            WestHold,
            SampleWest,
            RotateEast,
            EastHold,
            SampleEast,
            RotateSouth,
            SouthTouch,
            RotateWestReseat,
            WestTouch,
            RotateNorth,
            NorthHold,
            SampleFrontBaseline,
            FinalHold
        };

        void AttachRuntime(SharedRobotRuntime& runtime) noexcept;

        bool CanStart() const noexcept;
        void ResetState() noexcept;
        void Fail(const char* reason) noexcept;
        bool BeginDriveHoldPhase(Phase phase, std::uint16_t durationMs) noexcept;
        bool BeginDriveMovePhase(Phase phase, float targetXMeters, float targetYMeters, MazeMap::Direction headingDirection) noexcept;
        bool BeginDriveTurnPhase(Phase phase, MazeMap::Direction targetDirection) noexcept;
        bool BeginWallTouchPhase(Phase phase, MazeMap::Direction wallDirection) noexcept;

        void AdvanceAfterDrivePhase() noexcept;
        void AdvanceAfterWallTouchPhase() noexcept;
        bool SampleWestFacingSideCalibration() noexcept;
        bool SampleEastFacingSideCalibration() noexcept;
        bool SampleFrontBaseline() noexcept;
        bool StoreSideReference(
            WallSensorId sensorId,
            const WallSensorCalibrationCapture& capture,
            float actualDistanceM) noexcept;
        bool StoreSideBaseline(
            WallSensorId sensorId,
            const WallSensorCalibrationCapture& capture) noexcept;
        bool StoreFrontBaseline(
            WallSensorId sensorId,
            const WallSensorCalibrationCapture& capture) noexcept;

        SharedRobotRuntime* _runtime{};
        LoopController* _loopController{};
        RuntimeSensorSuite* _sensors{};
        DriveBase* _drive{};
        Drive* _driveService{};
        WallTouch* _wallTouch{};
        MazeMap::Vehicle* _speedVehicle{};
        MotionLimits _travelLimits{};
        bool _isInMaze{};
        bool _broughtUp{};
        bool _faulted{};
        Phase _phase{ Phase::None };
        std::array<float, 2U> _sideReferenceDistancesM{};
        std::array<bool, 2U> _sideReferenceValid{};
    };
}
