#pragma once

#include "DriveTelemetry.h"
#include "MazeMapRuntimeCore.h"
#include "SensorSnapshot.h"

#include <cstdint>

namespace MazeMap::App::Internal
{
    class Drive;
    class SharedRobotRuntime;
    class StartupCalibration;
    class WallTouch;

    // LoopController owns one uninterrupted fixed-period control session. Its sole job is to lock
    // cadence, command-application timing, sensing/update timing, and the final sync wait into one
    // authoritative owner. Do not add public per-tick stepping APIs here. A public RunOneTick /
    // Step / Advance surface would hand cadence ownership back to callers and reduce this class to
    // a bad timer wrapper that can only imitate a real control-loop authority poorly.
    class EXPORT LoopController final
    {
    public:
        enum class WallMask : std::uint8_t
        {
            None = 0x00,
            Front = 0x01,
            Left = 0x02,
            Right = 0x04,
            All = 0x07
        };

        struct SensorWorkPlan final
        {
            WallMask wallMask{ WallMask::All };
            bool readEncoders{ true };
            bool readImuBundle{ true };
            bool useEncoderUpdate{ true };
            bool useGyroUpdate{ true };
            bool useAccelUpdate{ true };
            bool useWallUpdates{ true };
        };

        struct SessionOptions final
        {
            std::uint32_t controlPeriodUs{};
            SensorWorkPlan workPlan{};
        };

		// P0: If the loop method returns, the loop is complete. Period. The fault system can and should eat all execution post-fault inside of SharedRobotRuntime.
        struct SessionResult final
        {
            enum class Status : std::uint8_t
            {
                Completed,
                StoppedByRuntime
            } status{ Status::Completed };

            std::uint32_t tickCount{};
        };

        // The mode callback is eligible on the first control tick; any longer startup wait
        // belongs in the mode callback rather than in LoopController dispatch.
        static constexpr std::uint32_t kInitialModeCallbackTick = 1U;

        struct EXPORT ControlVector final
        {
            float leftMotorPwm{};
            float rightMotorPwm{};

            static const ControlVector Brake;
            static ControlVector RawMotorPwm(
                float leftMotorPwm,
                float rightMotorPwm) noexcept;
        };

        struct TimingDiagnostics final
        {
            std::uint32_t sequence{};
            std::uint32_t tickStartUs{};
            std::uint32_t dtUs{};
            ControlCycleTiming controlTiming{};
            OpticalObservationTiming frontTiming{};
            OpticalObservationTiming leftTiming{};
            OpticalObservationTiming rightTiming{};
            ImuObservationTiming imuTiming{};
            std::uint16_t tActuationAppliedUs{};
            std::uint16_t tModeReturnUs{};
            std::uint16_t tPostServiceDoneUs{};
            std::uint16_t overrunUs{};
            std::uint8_t flags{};
        };

		// P0: This is a very thin struct, and the callback is guaranteed to be non-null when the reason is provided. Also, LoopController shouldn't be logging this shit internally.
        struct PauseContext final
        {
            const char* reason{};
        };

        // P0: This is a cluster of things that encourage abuse of the system. Users of it should be restructured to match the contract of the class.
        struct PauseDisposition final
        {
            // P0: The only item in this set that is valid is "Resume." There's no point to the type.
            enum class Action : std::uint8_t
            {
                Resume,
                Complete,
                StopByRuntime
            } action{ Action::Resume };

            // P0: The only valid value for this is false, so the entire field is pointless and encourages breaking the system.
            bool resetClockOnResume{ true };
            const char* stopReason{};

            // P0: This is the only valid return value, so the whole struct is pointless.
            static PauseDisposition Resume() noexcept;
            // P0: If you want to stop the loop, you call EndSession.
            static PauseDisposition Complete() noexcept;
            // P0: This is only valid coming from SharedRobotRuntime. All other calls should die, and the mechanism needs to move to a private/friend barrier to prevent abuse.
            static PauseDisposition StopByRuntime(const char* reason) noexcept;
        };

        class TickServices;

        using ModeWorkCallback = ControlVector (*)(
            void* context,
            std::uint32_t loopEndTimeUs,
            const MazeMap::VehicleState& state,
            TickServices& services);

        using PauseCallback = PauseDisposition (*)(
            void* context,
            const PauseContext& pause);

		// P0: The only valid item in this struct is the callback, and the only valid callback is one that returns "Resume" with no clock reset. This struct encourages abuse of the system by making it look like you are allowed to do more than that.
        struct PauseRequest final
        {
            PauseCallback onPauseGranted{};
            // P1: This encourages callers to store strings specifically for this method, and adds a required logging layer for pauses. This is counterproductive bloat.
            const char* reason{};
            // P0: LoopController only pauses once fully settled, and the callback is not allowed to issue motion commands. This implies otherwise.
            float maxAbsLinearSpeed{ -1.0f };
            // P0: LoopController only pauses once fully settled, and the callback is not allowed to issue motion commands. This implies otherwise.
            float maxAbsAngularSpeed{ -1.0f };
            // P1: This is pointless, as the stop policy is owned here.
            std::uint8_t consecutiveSettledTicks{};
            // P0: Flushing is not controlled by callers.
            bool flushLogsBeforeGrant{ true };
            // P0: LoopController is not allowed to reset the clock ever.
            bool resetClockOnResume{ true };
        };

        struct ModeCallbacks final
        {
            ModeWorkCallback onModeWork{};
            void* context{};
        };

        // P0: This class has no reason to exist, as all contents would be better as LoopController methods.
        class TickServices final
        {
        public:
            // P1: This encourages division of fault ownership.
            void Fault(const char* reason) noexcept;
            // Pause is the only sanctioned way to leave strict periodic cadence for non-periodic
            // work. Do not replace it with caller-driven "tick once, then return to me" control.
			// P0: Should be a method of LoopController, not a nested class.
            void RequestPause(const PauseRequest& request) noexcept;
            // P0: Should be a method of LoopController, not a nested class.
            void RequestEndLoop() noexcept;
            // P1: This should be a method of LoopController, not a nested class.
            void SetNextModeWorkCallbacks(const ModeCallbacks& callbacks) noexcept;
            // P0: Copying things by value?
            void SetNextModeWorkCallback(ModeWorkCallback callback) noexcept;

        private:
            friend class LoopController;
            explicit TickServices(LoopController& owner) noexcept;

            LoopController* _owner{};
        };

        LoopController() = default;

        // The public contract is intentionally session-scoped. Callers configure one loop session,
        // hand LoopController the mode callback, and let Run() own the cadence until the session
        // ends or pauses explicitly. Do not add public single-tick entry points here: any API that
        // returns control after an individual tick fundamentally breaks LoopController's sole
        // responsibility by making cadence caller-owned again.
        bool BeginSession(const SessionOptions& options, const ModeCallbacks& callbacks);
        SessionResult Run();
        void EndSession();

        bool SessionActive() const noexcept;
        const TimingDiagnostics& LastDiagnostics() const noexcept;
        const ControlVector& LastAppliedCommand() const noexcept;
        std::uint32_t CurrentTickSequence() const noexcept;
        std::uint32_t CurrentTickStartUs() const noexcept;
        std::uint32_t CurrentTickDtUs() const noexcept;
        float CurrentTickDtSeconds() const noexcept;

    private:
        friend class Drive;
        friend class SharedRobotRuntime;
        friend class StartupCalibration;
        friend class WallTouch;

        enum class DeferredTerminalOutcome : std::uint8_t
        {
            None,
            Complete,
            RuntimeStop
        };

        struct LatchedRequests final
        {
            const char* runtimeStopReason{};
            bool endRequested{};
            bool pauseRequested{};
            PauseRequest pauseRequest{};
            bool nextModeWorkRequested{};
            ModeCallbacks nextModeWork{};
        };

        struct MotorPwmSink final
        {
            using SetMotorPwmFn = bool (*)(void* context, float leftMotorPwm, float rightMotorPwm) noexcept;

            void* context{};
            SetMotorPwmFn setMotorPwm{};

            explicit operator bool() const noexcept
            {
                return setMotorPwm != nullptr;
            }

            bool Apply(const ControlVector& control) const noexcept
            {
                return
                    (setMotorPwm != nullptr) &&
                    setMotorPwm(context, control.leftMotorPwm, control.rightMotorPwm);
            }
        };

        static constexpr std::uint8_t kTimingFlagResumedFromPause = 1U << 0;
        static constexpr std::uint8_t kTimingFlagPausePending = 1U << 1;
        static constexpr std::uint8_t kTimingFlagRuntimeStopPending = 1U << 2;

        static std::uint16_t RelativeTickUs(std::uint32_t tickStartUs, std::uint32_t timestampUs) noexcept;
        static bool IsBrakeMotorPwmCommand(const ControlVector& command) noexcept;
        static bool IsZeroMotorPwmCommand(const ControlVector& command) noexcept;
        static bool IsFullSensorWorkPlan(const SensorWorkPlan& workPlan) noexcept;
        static std::uint32_t ReadCycleCounter() noexcept;
        void RunSessionStartWallSensorAdcProbe() noexcept;
        void AttachRuntime(SharedRobotRuntime& runtime) noexcept;
        bool ValidateSessionOptions(const SessionOptions& options) const noexcept;
        bool SupportsSensorWorkPlan(const SensorWorkPlan& workPlan) const noexcept;
        const TimingDiagnostics* CurrentTimingForReaders() const noexcept;
        void ResetLatchedRequests() noexcept;
        bool ApplyControlAtTickStart(const ControlVector& control) noexcept;
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
        bool ResolvePauseRequest(SessionResult& result);
        bool WaitForPauseSettlement(const PauseRequest& request);
        void ResetSessionState() noexcept;

        SharedRobotRuntime* _runtime{};
        SessionOptions _options{};
        ModeCallbacks _callbacks{};
        ModeWorkCallback _activeModeWorkCallback{};
        void* _activeModeWorkContext{};
        bool _sessionBegun{};
        bool _sessionActive{};
        bool _resumePending{};
        bool _publishedTimingValid{};
        std::uint32_t _tickCount{};
        std::uint32_t _lastTickStartUs{};
        std::uint32_t _nextSyncTargetUs{};
        ControlVector _queuedControl{};
        ControlVector _appliedControl{};
        MotorPwmSink _motorPwmSink{};
        bool _sessionStartWallSensorAdcProbePending{};
        TimingDiagnostics _timingBuffers[2]{};
        std::uint8_t _publishedTimingIndex{ 0U };
        std::uint8_t _workingTimingIndex{ 1U };
        LatchedRequests _requests{};
        DeferredTerminalOutcome _deferredTerminalOutcome{ DeferredTerminalOutcome::None };
        const char* _deferredTerminalReason{};
        PauseContext _pauseContextScratch{};
    };
}
