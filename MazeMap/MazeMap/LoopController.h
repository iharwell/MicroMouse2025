#pragma once

#include "CommandVector.h"
#include "RuntimeSensorSuite.h"
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
    // - SetupMode() must stage the initial session state through StageNextSessionState(...).
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
    // - StageNextSessionState(...) stages only the fixed session state that the next session start
    //   will consume. A later call replaces the previously staged state.
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
        // StageNextSessionState(...) uses this as the authoritative wall-sensor opt-out surface.
        // Modes stage
        // the exact wall groups they want for the session, and LoopController filters wall-sensor
        // capture/output behavior from this mask instead of from separate special-case flags.
        enum class WallMask : std::uint8_t
        {
            None = 0x00, // No wall-sensor groups selected.
            Front = RuntimeSensorSuite::kFrontWallSensorBit, // Front wall-sensor group selected.
            Left = RuntimeSensorSuite::kLeftWallSensorBit,   // Left wall-sensor group selected.
            Right = RuntimeSensorSuite::kRightWallSensorBit, // Right wall-sensor group selected.
            All = RuntimeSensorSuite::kWallSensorBits        // All wall-sensor groups selected.
        };

        // Sequence number of the first RunTick-eligible tick in every session.
        //
        // Session-local tick numbering restarts at this value whenever a new session begins.
        static constexpr std::uint32_t kInitialModeCallbackTick = 1U;

        // Constructs an unattached LoopController in its inert pre-runtime, pre-mode-bound state.
        //
        // SharedRobotRuntime later attaches the runtime, and application infrastructure later
        // binds the top-level mode before entering Run().
        LoopController() = default;

        // `StageNextSessionState(...)`:
        // Installs the fixed state that the next session start will consume.
        //
        // Parameters:
        // `controlPeriodUs`:
        // Strict period between synchronized tick boundaries.
        //
        // `sessionStartPointX`, `sessionStartPointY`:
        // Required reset physical start position for that session.
        //
        // `wallMask`, `useEncoderUpdate`, `useGyroUpdate`, `useAccelUpdate`, `useWallUpdates`:
        // Session-local sensor participation used for capture, estimator input, and wall updates.
        //
        // Behavior:
        // - SetupMode() must call this before infrastructure-owned Run() is entered.
        // - A later call replaces any previously staged successor-session state.
        // - After RequestEndSession(...), the end-session callback must call this before it
        //   returns unless it instead requests HaltExecutionEndProgram().
        // - Invalid session state is a terminal contract violation and faults the active mode.
        void StageNextSessionState(
            std::uint32_t controlPeriodUs,
            float sessionStartPointX,
            float sessionStartPointY,
            WallMask wallMask = WallMask::All,
            bool useEncoderUpdate = true,
            bool useGyroUpdate = true,
            bool useAccelUpdate = true,
            bool useWallUpdates = true) noexcept;

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
        void RequestPause(
            void (*callback)(void* context, LoopController& loopController),
            void* context) noexcept;

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
        // - `callback` must stage the next session state or request HaltExecutionEndProgram().
        // - Once `callback` returns, LoopController either starts the next session immediately or
        //   returns from Run() if terminal halt was requested.
        // - Every successor session restarts with IApplicationMode::RunTick(...) and the bound
        //   mode object as context.
        // - A null callback is a terminal contract violation.
        void RequestEndSession(
            void (*callback)(void* context, LoopController& loopController),
            void* context) noexcept;

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
        void SetNextModeWorkCallback(
            CommandVector (*callback)(
                void* context,
                std::uint32_t loopEndTimeUs,
                const MazeMap::VehicleState& state,
                LoopController& loopController),
            void* context) noexcept;

        // `SessionActive()`:
        // Returns whether Run() currently owns an active session lifecycle.
        //
        // Return value:
        // `true` while LoopController is inside an active session. `false` before Run(),
        // between terminal return and destruction, or after a terminal fault path has taken over.
        bool SessionActive() const noexcept;

        std::uint32_t LastTimingSequence() const noexcept;
        std::uint32_t LastTimingTickStartUs() const noexcept;
        std::uint32_t LastTimingDtUs() const noexcept;
        std::uint32_t LastTimingCommandAppliedUs() const noexcept;
        std::uint32_t LastTimingEncoderLatchUs() const noexcept;
        std::uint32_t LastTimingEncoderReadDoneUs() const noexcept;
        std::uint32_t LastTimingEstimatorPredictStartUs() const noexcept;
        std::uint32_t LastTimingEstimatorPredictEndUs() const noexcept;
        std::uint32_t LastTimingEstimatorPredictDurationUs() const noexcept;
        std::uint32_t LastTimingEstimatorUpdateStartUs() const noexcept;
        std::uint32_t LastTimingEstimatorUpdateEndUs() const noexcept;
        std::uint32_t LastTimingEstimatorUpdateDurationUs() const noexcept;
        std::uint32_t LastTimingEstimatorTotalDurationUs() const noexcept;
        std::uint32_t LastTimingCallbackReturnUs() const noexcept;
        std::uint32_t LastTimingPostServiceDoneUs() const noexcept;
        std::uint32_t LastTimingTickFinalizeUs() const noexcept;
        std::uint32_t LastTimingCycleCounterStart() const noexcept;
        std::uint32_t LastTimingCycleCounterEnd() const noexcept;
        std::uint16_t LastTimingOverrunUs() const noexcept;

        // `LastAppliedCommand()`:
        // Returns the command most recently applied at a command application point.
        //
        // Return value:
        // The command LoopController most recently handed to the runtime actuation hook.
        const CommandVector& LastAppliedCommand() const noexcept;

    private:
        friend class ::MazeMap::App::Application;
        friend class SharedRobotRuntime;

        class TimingBuffer final
        {
        public:
            std::uint32_t Sequence() const noexcept { return _sequence; }
            std::uint32_t TickStartUs() const noexcept { return _tickStartUs; }
            std::uint32_t DtUs() const noexcept { return _dtUs; }
            std::uint32_t CommandAppliedUs() const noexcept { return _commandAppliedUs; }
            std::uint32_t EncoderLatchUs() const noexcept { return _encoderLatchUs; }
            std::uint32_t EncoderReadDoneUs() const noexcept { return _encoderReadDoneUs; }
            std::uint32_t EstimatorPredictStartUs() const noexcept { return _estimatorPredictStartUs; }
            std::uint32_t EstimatorPredictEndUs() const noexcept { return _estimatorPredictEndUs; }
            std::uint32_t EstimatorPredictDurationUs() const noexcept { return _estimatorPredictDurationUs; }
            std::uint32_t EstimatorUpdateStartUs() const noexcept { return _estimatorUpdateStartUs; }
            std::uint32_t EstimatorUpdateEndUs() const noexcept { return _estimatorUpdateEndUs; }
            std::uint32_t EstimatorUpdateDurationUs() const noexcept { return _estimatorUpdateDurationUs; }
            std::uint32_t EstimatorTotalDurationUs() const noexcept { return _estimatorTotalDurationUs; }
            std::uint32_t CallbackReturnUs() const noexcept { return _callbackReturnUs; }
            std::uint32_t PostServiceDoneUs() const noexcept { return _postServiceDoneUs; }
            std::uint32_t TickFinalizeUs() const noexcept { return _tickFinalizeUs; }
            std::uint32_t CycleCounterStart() const noexcept { return _cycleCounterStart; }
            std::uint32_t CycleCounterEnd() const noexcept { return _cycleCounterEnd; }
            std::uint16_t OverrunUs() const noexcept { return _overrunUs; }

        private:
            friend class LoopController;

            std::uint32_t _sequence{};
            std::uint32_t _tickStartUs{};
            std::uint32_t _dtUs{};
            std::uint32_t _commandAppliedUs{};
            std::uint32_t _encoderLatchUs{};
            std::uint32_t _encoderReadDoneUs{};
            std::uint32_t _estimatorPredictStartUs{};
            std::uint32_t _estimatorPredictEndUs{};
            std::uint32_t _estimatorPredictDurationUs{};
            std::uint32_t _estimatorUpdateStartUs{};
            std::uint32_t _estimatorUpdateEndUs{};
            std::uint32_t _estimatorUpdateDurationUs{};
            std::uint32_t _estimatorTotalDurationUs{};
            std::uint32_t _callbackReturnUs{};
            std::uint32_t _postServiceDoneUs{};
            std::uint32_t _tickFinalizeUs{};
            std::uint32_t _cycleCounterStart{};
            std::uint32_t _cycleCounterEnd{};
            std::uint16_t _overrunUs{};
        };

        static constexpr std::uint8_t kDefaultSensorWorkBits = RuntimeSensorSuite::kDefaultSensorWorkBits;

        static CommandVector RunApplicationModeTick(
            void* context,
            std::uint32_t loopEndTimeUs,
            const MazeMap::VehicleState& state,
            LoopController& loopController);
        static bool IsBrakeCommand(const CommandVector& command) noexcept;
        static bool IsZeroCommand(const CommandVector& command) noexcept;
        static std::uint32_t ReadCycleCounter() noexcept;
        static void WaitUntilUs(std::uint32_t absoluteDeadlineUs) noexcept;
        static void ServiceInterlacedSensorCapture(void* context) noexcept;

        void RunSessionStartWallSensorAdcProbe() noexcept;
        void AttachRuntime(SharedRobotRuntime& runtime) noexcept;
        void BindApplicationMode(IApplicationMode& mode) noexcept;
        void Run();
        void StartSessionFromStagedState() noexcept;
        void RestoreSessionStartPhysicalState() noexcept;
        bool ValidateSessionState(
            std::uint32_t controlPeriodUs,
            float sessionStartPointX,
            float sessionStartPointY,
            std::uint8_t sensorWorkBits) const noexcept;
        bool SupportsSensorWorkBits(std::uint8_t sensorWorkBits) const noexcept;
        void ClearPendingRequests() noexcept;
        bool ApplyControlAtApplicationPoint(const CommandVector& control) noexcept;
        bool CaptureTickState(float dtSeconds, std::uint32_t tickStartUs);
        void ResetWorkingTiming(
            std::uint32_t sequence,
            std::uint32_t tickStartUs,
            std::uint32_t dtUs) noexcept;
        TimingBuffer& WorkingTiming() noexcept;
        const TimingBuffer& PublishedTiming() const noexcept;
        void PublishWorkingTiming() noexcept;
        void RecordModeReturnTiming() noexcept;
        void RecordPostServiceTiming() noexcept;
        void FinalizeTiming() noexcept;
        bool ServiceRuntimeLogsNormal() noexcept;
        void ServiceRuntimeLogsForFaultPath() noexcept;
        std::uint32_t ComputeRemainingSlackUs(std::uint32_t absoluteDeadlineUs) const noexcept;
        bool ShouldTreatCurrentControlAsStationary() const noexcept;
        void ResolvePauseRequest();
        void ResolveEndSessionRequest();
        void WaitForBrakeSettlement();
        void ResetExecutionState() noexcept;

        SharedRobotRuntime* _runtime{};    // Runtime owner for actuation, sensing, logs, and state.
        IApplicationMode* _boundMode{};    // Bound top-level mode whose RunTick(...) starts each session.
        float _modeStartHeadingRad{ std::numeric_limits<float>::quiet_NaN() }; // Captured start-of-mode heading reused for successor session resets.
        std::uint32_t _controlPeriodUs{};
        std::uint32_t _stagedControlPeriodUs{};
        float _sessionStartPointX{ std::numeric_limits<float>::quiet_NaN() };
        float _sessionStartPointY{ std::numeric_limits<float>::quiet_NaN() };
        float _stagedSessionStartPointX{ std::numeric_limits<float>::quiet_NaN() };
        float _stagedSessionStartPointY{ std::numeric_limits<float>::quiet_NaN() };
        std::uint8_t _sensorWorkBits{ kDefaultSensorWorkBits };
        std::uint8_t _stagedSensorWorkBits{ kDefaultSensorWorkBits };
        bool _stagedNextSessionValid{};    // Whether successor-session state is presently valid.
        CommandVector (*_activeModeWorkCallback)( // Current strict-cadence tick owner.
            void* context,
            std::uint32_t loopEndTimeUs,
            const MazeMap::VehicleState& state,
            LoopController& loopController){};
        void* _activeModeWorkContext{};    // Explicit context paired with _activeModeWorkCallback.
        bool _sessionActive{};             // Whether Run() currently owns an active session lifecycle.
        bool _publishedTimingValid{};      // Whether a completed-tick timing snapshot is published.
        std::uint32_t _tickCount{};        // One-based active-session tick sequence.
        std::uint32_t _lastTickStartUs{};  // Previous tick-start timestamp.
        std::uint32_t _nextSyncTargetUs{}; // Absolute deadline for the current/next synchronized tick.
        CommandVector _nextControl{};      // Command staged for the next command application point.
        CommandVector _currentControl{};   // Command currently published as active and used by the next prediction.
        bool _sessionStartWallSensorAdcProbePending{}; // Deferred session-start ADC probe request.
        TimingBuffer _timingBuffers[2]{}; // Double-buffered published/working timing storage.
        std::uint8_t _publishedTimingIndex{ 0U }; // Index of the published completed-tick timing buffer.
        std::uint8_t _workingTimingIndex{ 1U };   // Index of the mutable working timing buffer.
        void (*_pendingPauseCallback)(void* context, LoopController& loopController){}; // Pending continuity-preserving boundary callback.
        void* _pendingPauseContext{};              // Context paired with _pendingPauseCallback.
        void (*_pendingEndSessionCallback)(void* context, LoopController& loopController){}; // Pending continuity-breaking boundary callback.
        void* _pendingEndSessionContext{};               // Context paired with _pendingEndSessionCallback.
        bool _programHaltRequested{};              // Pending terminal return-from-Run request.
        CommandVector (*_stagedModeWorkCallback)( // Explicit transfer target for the next in-session tick.
            void* context,
            std::uint32_t loopEndTimeUs,
            const MazeMap::VehicleState& state,
            LoopController& loopController){};
        void* _stagedModeWorkContext{};             // Context paired with _stagedModeWorkCallback.
    };
}
