#pragma once

#include "CommandVector.h"
#include "SensorSnapshot.h"
#include "VehicleState.h"

#include <cstdint>
#include <limits>

namespace MazeMap::App
{
    class Application;
}

namespace MazeMap::App::Internal
{
    class IApplicationMode;
    class SharedRobotRuntime;

    // Strict-cadence, total owner of one active loop-backed application session.
    //
    // Layering contract:
    // LoopController owns the fixed-period schedule, tick timing publication, command-application
    // timing, sensing cadence, active callback dispatch, and the explicit lifecycle boundaries that
    // may leave strict cadence temporarily. It does not select the top-level mode, own boot-mode
    // discovery, or decide what the active callback means semantically.
    //
    // Startup and session-start contract:
    // - Infrastructure resolves the active IApplicationMode object.
    // - SetupMode() is one-time pre-Run() preparation for the selected boot mode, not a reusable
    //   reset hook for another pass through the same mode object.
    // - SetupMode() must stage the initial SessionOptions through StageNextSessionState(...).
    // - Infrastructure privately binds the mode object and enters Run().
    // - Every session start, including successor-session restarts, installs
    //   IApplicationMode::RunTick(...) with the bound mode object as the callback context.
    // - Modes do not choose their own initial callback/context pair.
    //
    // Callback ownership model:
    // - Exactly one active callback owns each in-session tick.
    // - SetNextModeWorkCallback(...) explicitly transfers ownership of the next in-session tick.
    // - RequestPause(...) and RequestEndSession(...) do not continue inside strict cadence.
    //   LoopController brakes, settles, and then invokes the corresponding callback boundary.
    // - HaltExecutionEndProgram() is the only non-fault path that causes Run() to return.
    //
    // Public state model:
    // - StageNextSessionState(...) stages only the SessionOptions that the next session start will
    //   consume. A later call replaces the previously staged options.
    // - RequestPause(...) preserves continuity. Its callback runs after brake settlement and may
    //   then resume the current session, request end-session, request terminal halt, or transfer
    //   in-session callback ownership for the next resumed tick.
    // - RequestEndSession(...) intentionally breaks continuity. Its callback runs after brake
    //   settlement, must stage the successor session or request HaltExecutionEndProgram(), and may
    //   not request nested boundary work that belongs only to an active session tick.
    //
    // No public single-tick stepping API is allowed here. Exposing a caller-owned tick boundary
    // would split cadence ownership and invalidate LoopController's reason for existing.
    class EXPORT LoopController final
    {
    public:
        // Selects which wall-sensor groups participate in one session's sensor work.
        //
        // SensorWorkPlan uses this as the authoritative wall-sensor opt-out surface. Modes stage
        // the exact wall groups they want for the session, and LoopController filters wall-sensor
        // capture/output behavior from this mask instead of from separate special-case flags.
        enum class WallMask : std::uint8_t
        {
            None = 0x00,  // No wall-sensor groups selected.
            Front = 0x01, // Front wall-sensor group selected.
            Left = 0x02,  // Left wall-sensor group selected.
            Right = 0x04, // Right wall-sensor group selected.
            All = 0x07    // All wall-sensor groups selected.
        };

        // Per-session sensing/update plan consumed by StageNextSessionState(...).
        //
        // This is the authoritative session-local sensor opt-out contract. Modes describe one
        // homogeneous plan here, and LoopController derives capture participation, estimator input
        // participation, and wall-update participation from this one plan rather than from
        // separate ad hoc sensor-usage flags.
        class SensorWorkPlan final
        {
            enum : std::uint8_t { kWalls = 0x07U, kEncoder = 0x08U, kGyro = 0x10U, kAccel = 0x20U, kWallUpdate = 0x40U };

        public:
            bool UsesFrontWallSensors() const noexcept { return (_bits & static_cast<std::uint8_t>(WallMask::Front)) != 0U; }
            bool UsesLeftWallSensors() const noexcept { return (_bits & static_cast<std::uint8_t>(WallMask::Left)) != 0U; }
            bool UsesRightWallSensors() const noexcept { return (_bits & static_cast<std::uint8_t>(WallMask::Right)) != 0U; }
            bool ReadEncoders() const noexcept { return UseEncoderUpdate(); }
            bool ReadImuBundle() const noexcept { return UseGyroUpdate() || UseAccelUpdate(); }
            bool UseEncoderUpdate() const noexcept { return (_bits & kEncoder) != 0U; }
            bool UseGyroUpdate() const noexcept { return (_bits & kGyro) != 0U; }
            bool UseAccelUpdate() const noexcept { return (_bits & kAccel) != 0U; }
            bool UseWallUpdates() const noexcept { return (_bits & kWallUpdate) != 0U; }
            void SetWallMask(const WallMask wallMask) noexcept { _bits = static_cast<std::uint8_t>((_bits & static_cast<std::uint8_t>(~kWalls)) | (static_cast<std::uint8_t>(wallMask) & kWalls)); }
            void SetUseEncoderUpdate(const bool enabled) noexcept { Set(kEncoder, enabled); }
            void SetUseGyroUpdate(const bool enabled) noexcept { Set(kGyro, enabled); }
            void SetUseAccelUpdate(const bool enabled) noexcept { Set(kAccel, enabled); }
            void SetUseWallUpdates(const bool enabled) noexcept { Set(kWallUpdate, enabled); }

        private:
            void Set(const std::uint8_t bit, const bool enabled) noexcept { _bits = enabled ? static_cast<std::uint8_t>(_bits | bit) : static_cast<std::uint8_t>(_bits & static_cast<std::uint8_t>(~bit)); }
            std::uint8_t _bits{ kWalls | kEncoder | kGyro | kAccel | kWallUpdate };
        };

        static_assert(sizeof(SensorWorkPlan) == sizeof(std::uint8_t));

        // Fixed startup state for one session.
        //
        // SessionOptions does not choose the initial callback or callback context; infrastructure
        // always supplies IApplicationMode::RunTick(...) plus the bound mode object for that.
        struct SessionOptions final
        {
            std::uint32_t controlPeriodUs{}; // Strict period between synchronized tick boundaries.
            SensorWorkPlan workPlan{};       // Fixed sensing/update plan for that session.
            float SessionStartPointX{ std::numeric_limits<float>::quiet_NaN() }; // Required X position for the reset physical start state of that session.
            float SessionStartPointY{ std::numeric_limits<float>::quiet_NaN() }; // Required Y position for the reset physical start state of that session.
        };

        // Sequence number of the first RunTick-eligible tick in every session.
        //
        // Session-local tick numbering restarts at this value whenever a new session begins.
        static constexpr std::uint32_t kInitialModeCallbackTick = 1U;

        // Published timing snapshot for one completed tick.
        //
        // These timestamps and durations are observation output only. They do not give callers a
        // public cadence-control plane back into LoopController.
        struct TimingDiagnostics final
        {
            std::uint32_t sequence{};             // One-based session-local tick sequence.
            std::uint32_t tickStartUs{};          // Absolute tick-start timestamp.
            std::uint32_t dtUs{};                 // Elapsed time since the previous tick start.
            std::uint32_t commandAppliedUs{};     // Absolute timestamp when the tick-start command application completed.
            std::uint32_t encoderLatchUs{};       // Absolute timestamp just before sensor/encoder capture begins.
            std::uint32_t encoderReadDoneUs{};    // Absolute timestamp after encoder observations have been consumed.
            std::uint32_t ukfPredictStartUs{};    // Absolute timestamp just before estimator predict begins.
            std::uint32_t ukfPredictEndUs{};      // Absolute timestamp when estimator predict completes.
            std::uint32_t ukfPredictDurationUs{}; // Observed estimator predict duration.
            std::uint32_t ukfUpdateStartUs{};     // Absolute timestamp just before estimator updates begin.
            std::uint32_t ukfUpdateEndUs{};       // Absolute timestamp when estimator updates complete.
            std::uint32_t ukfUpdateDurationUs{};  // Observed estimator update duration.
            std::uint32_t ukfTotalDurationUs{};   // Combined estimator predict+update duration.
            std::uint32_t callbackReturnUs{};     // Absolute timestamp when the active mode callback returned.
            std::uint32_t postServiceDoneUs{};    // Absolute timestamp when post-callback services completed.
            std::uint32_t tickFinalizeUs{};       // Absolute timestamp when end-of-tick finalize work completed.
            std::uint32_t cycleCounterStart{};    // Cycle counter sampled at tick start.
            std::uint32_t cycleCounterEnd{};      // Cycle counter sampled at finalize.
            std::uint16_t overrunUs{};            // Positive overrun beyond the scheduled deadline.
        };

        // Explicit in-session callback transfer target.
        //
        // The callback receives:
        // - the caller-owned context pointer explicitly supplied with SetNextModeWorkCallback(...),
        // - the absolute end time for the current tick,
        // - the authoritative runtime-state snapshot for the current tick, and
        // - direct access to LoopController's lifecycle/control methods.
        //
        // Returning from this callback completes only the current tick. It does not imply pause,
        // end-session, or terminal halt unless one of those boundaries was requested explicitly.
        using ModeWorkCallback = CommandVector (*)(
            void* context,
            std::uint32_t loopEndTimeUs,
            const MazeMap::VehicleState& state,
            LoopController& loopController);

        // Continuity-preserving pause boundary callback.
        //
        // This callback runs only after LoopController has braked and observed brake settlement.
        // The session remains the same session after the callback returns unless the callback
        // explicitly requests end-session or terminal halt.
        using PauseCallback = void (*)(void* context, LoopController& loopController);

        // Continuity-breaking end-session boundary callback.
        //
        // This callback runs only after LoopController has braked and observed brake settlement.
        // It is the only supported boundary for staging the successor SessionOptions. The callback
        // must either:
        // - call StageNextSessionState(...), or
        // - call HaltExecutionEndProgram().
        //
        // Nested pause requests, nested end-session requests, and in-session callback-transfer
        // requests from this boundary are contract violations and fault the active mode.
        using EndSessionCallback = void (*)(void* context, LoopController& loopController);

        // Constructs an unattached LoopController in its inert pre-runtime, pre-mode-bound state.
        //
        // SharedRobotRuntime later attaches the runtime, and application infrastructure later
        // binds the top-level mode before entering Run().
        LoopController() = default;

        // `StageNextSessionState(options)`:
        // Installs the SessionOptions that the next session start will consume.
        //
        // Parameters:
        // `options`:
        // Strict-cadence period and sensing/update plan for the next session start.
        //
        // Behavior:
        // - SetupMode() must call this before infrastructure-owned Run() is entered.
        // - A later call replaces any previously staged successor-session state.
        // - After RequestEndSession(...), the end-session callback must call this before it
        //   returns unless it instead requests HaltExecutionEndProgram().
        // - Invalid SessionOptions are a terminal contract violation and fault the active mode.
        void StageNextSessionState(const SessionOptions& options) noexcept;

        // `RequestPause(callback, context)`:
        // Requests the continuity-preserving pause boundary.
        //
        // Parameters:
        // `callback`:
        // Callback invoked after brake settlement and outside the strict fixed-period tick.
        //
        // `context`:
        // Caller-owned context pointer passed back to `callback`.
        //
        // Behavior:
        // - The current tick still completes first.
        // - LoopController then brakes, waits for brake settlement, and invokes `callback`.
        // - On return, the same session resumes unless `callback` explicitly requested
        //   end-session or terminal halt.
        // - A null callback is a terminal contract violation.
        void RequestPause(PauseCallback callback, void* context) noexcept;

        // `RequestEndSession(callback, context)`:
        // Requests the continuity-breaking end-session boundary.
        //
        // Parameters:
        // `callback`:
        // Callback invoked after brake settlement and before the successor session begins.
        //
        // `context`:
        // Caller-owned context pointer passed back to `callback`.
        //
        // Behavior:
        // - The current tick still completes first.
        // - LoopController then brakes, waits for brake settlement, clears any previously staged
        //   successor-session state, and invokes `callback`.
        // - `callback` must stage the next SessionOptions or request HaltExecutionEndProgram().
        // - Once `callback` returns, LoopController either starts the next session immediately or
        //   returns from Run() if terminal halt was requested.
        // - Every successor session restarts with IApplicationMode::RunTick(...) and the bound
        //   mode object as context.
        // - A null callback is a terminal contract violation.
        void RequestEndSession(EndSessionCallback callback, void* context) noexcept;

        // `HaltExecutionEndProgram()`:
        // Requests terminal whole-program execution end.
        //
        // Behavior:
        // - When called from an in-session tick callback, the current tick still completes first.
        // - When called from pause or end-session boundary work, Run() returns after that boundary
        //   finishes and braking is confirmed.
        // - Run() returns only for this outcome; ordinary top-level completion is terminal at the
        //   infrastructure boundary, not an end-session path.
        // - Once Run() returns for this outcome, program execution is ending. Mode code must not
        //   treat this as a same-process opportunity to rerun or reenter the boot-selected mode.
        void HaltExecutionEndProgram() noexcept;

        // `SetNextModeWorkCallback(callback, context)`:
        // Explicitly transfers ownership of the next in-session tick.
        //
        // Parameters:
        // `callback`:
        // Callback that should own the next strict-cadence session tick.
        //
        // `context`:
        // Caller-owned context pointer passed back to `callback`.
        //
        // Behavior:
        // - The callback/context pair is always explicit; LoopController never carries callback
        //   context forward implicitly.
        // - The transfer is ignored if a higher-priority lifecycle boundary for the current tick
        //   wins first, such as pause, end-session, terminal halt, or fault.
        // - A null callback is a terminal contract violation.
        void SetNextModeWorkCallback(ModeWorkCallback callback, void* context) noexcept;

        // `SessionActive()`:
        // Returns whether Run() currently owns an active session lifecycle.
        //
        // Return value:
        // `true` while LoopController is inside an active session. `false` before Run(),
        // between terminal return and destruction, or after a terminal fault path has taken over.
        bool SessionActive() const noexcept;

        // `LastDiagnostics()`:
        // Returns the most recent loop timing snapshot.
        //
        // This is read-only observation output and does not expose cadence-control authority.
        //
        // Return value:
        // The most recently published completed-tick snapshot. Before the first completed tick,
        // the returned object still exists but its fields remain at their zero-initialized
        // defaults.
        const TimingDiagnostics& LastDiagnostics() const noexcept;

        // `LastAppliedCommand()`:
        // Returns the command most recently applied at tick start.
        //
        // Return value:
        // The command LoopController most recently handed to the runtime actuation hook at a
        // tick boundary.
        const CommandVector& LastAppliedCommand() const noexcept;

    private:
        friend class ::MazeMap::App::Application;
        friend class SharedRobotRuntime;

        static CommandVector RunApplicationModeTick(
            void* context,
            std::uint32_t loopEndTimeUs,
            const MazeMap::VehicleState& state,
            LoopController& loopController);
        static bool IsBrakeCommand(const CommandVector& command) noexcept;
        static bool IsZeroCommand(const CommandVector& command) noexcept;
        static std::uint32_t ReadCycleCounter() noexcept;

        void RunSessionStartWallSensorAdcProbe() noexcept;
        void AttachRuntime(SharedRobotRuntime& runtime) noexcept;
        void BindApplicationMode(IApplicationMode& mode) noexcept;
        void Run();
        void StartSessionFromStagedState() noexcept;
        void RestoreSessionStartPhysicalState() noexcept;
        bool ValidateSessionOptions(const SessionOptions& options) const noexcept;
        bool SupportsSensorWorkPlan(const SensorWorkPlan& workPlan) const noexcept;
        void ClearPendingRequests() noexcept;
        bool ApplyControlAtTickStart(const CommandVector& control) noexcept;
        bool CaptureTickState(float dtSeconds, std::uint32_t tickStartUs);
        void ResetWorkingTiming(
            std::uint32_t sequence,
            std::uint32_t tickStartUs,
            std::uint32_t dtUs) noexcept;
        TimingDiagnostics& WorkingTiming() noexcept;
        const TimingDiagnostics& PublishedTiming() const noexcept;
        void PublishWorkingTiming() noexcept;
        void RecordModeReturnTiming() noexcept;
        void RecordPostServiceTiming() noexcept;
        void FinalizeTiming() noexcept;
        bool ServiceRuntimeLogsNormal() noexcept;
        void ServiceRuntimeLogsForFaultPath() noexcept;
        std::uint32_t ComputeRemainingSlackUs(std::uint32_t absoluteDeadlineUs) const noexcept;
        bool ShouldTreatAppliedControlAsStationary() const noexcept;
        void ResolvePauseRequest();
        void ResolveEndSessionRequest();
        void WaitForBrakeSettlement();
        void ResetExecutionState() noexcept;

        SharedRobotRuntime* _runtime{};    // Runtime owner for actuation, sensing, logs, and state.
        IApplicationMode* _boundMode{};    // Bound top-level mode whose RunTick(...) starts each session.
        float _modeStartYawRad{ std::numeric_limits<float>::quiet_NaN() }; // Captured start-of-mode yaw reused for successor session resets.
        SessionOptions _options{};         // Active session configuration.
        SessionOptions _stagedNextSessionOptions{}; // Successor-session configuration awaiting start.
        bool _stagedNextSessionValid{};    // Whether _stagedNextSessionOptions is presently valid.
        ModeWorkCallback _activeModeWorkCallback{}; // Current strict-cadence tick owner.
        void* _activeModeWorkContext{};    // Explicit context paired with _activeModeWorkCallback.
        bool _sessionActive{};             // Whether Run() currently owns an active session lifecycle.
        bool _publishedTimingValid{};      // Whether a completed-tick timing snapshot is published.
        std::uint32_t _tickCount{};        // One-based active-session tick sequence.
        std::uint32_t _lastTickStartUs{};  // Previous tick-start timestamp.
        std::uint32_t _nextSyncTargetUs{}; // Absolute deadline for the current/next synchronized tick.
        CommandVector _queuedControl{};    // Command to apply at the start of the next tick.
        CommandVector _appliedControl{};   // Command applied at the start of the current tick.
        bool _sessionStartWallSensorAdcProbePending{}; // Deferred session-start ADC probe request.
        TimingDiagnostics _timingBuffers[2]{}; // Double-buffered published/working timing storage.
        std::uint8_t _publishedTimingIndex{ 0U }; // Index of the published completed-tick timing buffer.
        std::uint8_t _workingTimingIndex{ 1U };   // Index of the mutable working timing buffer.
        PauseCallback _pendingPauseCallback{};     // Pending continuity-preserving boundary callback.
        void* _pendingPauseContext{};              // Context paired with _pendingPauseCallback.
        EndSessionCallback _pendingEndSessionCallback{}; // Pending continuity-breaking boundary callback.
        void* _pendingEndSessionContext{};               // Context paired with _pendingEndSessionCallback.
        bool _programHaltRequested{};              // Pending terminal return-from-Run request.
        ModeWorkCallback _stagedModeWorkCallback{}; // Explicit transfer target for the next in-session tick.
        void* _stagedModeWorkContext{};             // Context paired with _stagedModeWorkCallback.
    };
}
