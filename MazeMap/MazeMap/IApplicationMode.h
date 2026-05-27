#pragma once

#include "CommandVector.h"
#include "LoopController.h"

#include <cstdint>

namespace MazeMap::App::Internal
{
    class BootFramework;

    // Small, authoritative top-level mode contract for boot-selected application execution.
    //
    // Layering contract:
    // IApplicationMode owns only top-level mode preparation and per-tick decision logic. It does
    // not own startup callback hookup, the loop cadence, or the terminal Run() boundary.
    //
    // Startup contract:
    // - Infrastructure resolves the active IApplicationMode object.
    // - Infrastructure brings up descriptor-required startup services before SetupMode(...).
    // - SetupMode(BootFramework& framework) performs one-time pre-loop preparation only for the
    //   selected boot mode.
    // - SetupMode(BootFramework& framework) is an infrastructure callback; a mode must never call
    //   it directly.
    // - SetupMode(BootFramework& framework) is not a reusable reset hook for another pass through
    //   the same mode object.
    // - SetupMode(BootFramework& framework) may configure the initial LoopController session
    //   state through StageNextSessionState(...).
    // - Infrastructure then binds the mode object itself as the initial callback context and
    //   enters LoopController::Run().
    //
    // Tick-ownership contract:
    // - RunTick(...) is the authoritative initial control-loop callback for every application
    //   mode and for every successor session restart.
    // - RunTick(...) is an infrastructure callback; a mode must never call it directly.
    // - During RunTick(...), the mode may keep callback ownership, transfer it explicitly,
    //   request pause, request an end-session boundary, or request terminal whole-program halt.
    // - Ordinary top-level mode completion is terminal whole-program end, not end-session.
    class IApplicationMode
    {
    public:
        // `~IApplicationMode()`:
        // Virtual interface destructor for polymorphic top-level mode ownership.
        //
        // Behavior:
        // Allows infrastructure to destroy a concrete boot-selected mode through this narrow
        // interface after startup selection has resolved the authoritative mode object.
        virtual ~IApplicationMode() = default;

        // `SetupMode(BootFramework& framework)`:
        // Performs one-time top-level mode preparation before LoopController::Run() is entered.
        // This is an infrastructure callback and must not be called directly by the mode.
        //
        // Behavior:
        // - May configure runtime services, logs, or mode-local retained state.
        // - May configure the initial session state through StageNextSessionState(...).
        // - Does not choose the initial callback/context pair and does not enter Run().
        // - Is called at most once for the selected boot mode during a program run.
        // - Failures are terminal and should go through SharedRobotRuntime::FailActiveMode(...),
        //   not boolean return values.
        virtual void SetupMode(BootFramework& framework) = 0;

        // `OnModeFault(reason)`:
        // Runs selected-mode cleanup that must happen on the terminal runtime fault path.
        //
        // Behavior:
        // Infrastructure calls this through BootFramework's single registered fault bridge.
        // Modes must not call it directly; ordinary recovery and phase transitions belong in
        // private mode methods.
        virtual void OnModeFault(const char* reason) noexcept
        {
            (void)reason;
        }

        // `RunTick(loopEndTimeUs, state, loopController)`:
        // Produces the mode's command proposal for the current strict-cadence tick.
        // This is an infrastructure callback and must not be called directly by the mode.
        //
        // Parameters:
        // `loopEndTimeUs`:
        // Absolute synchronized end time for the current tick.
        //
        // `state`:
        // Authoritative runtime-state snapshot for this tick.
        //
        // `loopController`:
        // Direct access to the authoritative lifecycle/control surface for pause, end-session,
        // successor-session staging, explicit callback transfer, timing reads, and terminal halt.
        //
        // Return value:
        // Control proposal for the next command application point. The callback may instead
        // request a different lifecycle boundary, but that boundary is still explicit rather than
        // implied by return.
        virtual CommandVector RunTick(
            std::uint32_t loopEndTimeUs,
            const MazeMap::VehicleState& state,
            LoopController& loopController) = 0;
    };
}

